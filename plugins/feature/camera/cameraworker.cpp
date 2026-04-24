///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>

#include <QDebug>
#include <QFont>
#include <QFontMetrics>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPainter>
#include <QRandomGenerator>
#include <QFile>
#include <QTextStream>
#include <QSharedPointer>
#include <QtEndian>
#include <QUrl>
#include <QUrlQuery>
#include <QColor>
#include <QDateTime>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCamera>
#include <QCameraDevice>
#include <QCameraFormat>
#include <QMediaCaptureSession>
#include <QMediaDevices>
#include <QSet>
#include <QVideoFrame>
#include <QVideoSink>
#else
#include <QAbstractVideoBuffer>
#include <QAbstractVideoSurface>
#include <QCamera>
#include <QCameraExposure>
#include <QCameraFocus>
#include <QCameraImageProcessing>
#include <QCameraInfo>
#include <QCameraViewfinderSettings>
#include <QSet>
#include <QVideoFrame>
#endif

#include "maincore.h"
#include "dsp/dspengine.h"
#include "audio/audiodevicemanager.h"
#include "cameraworker.h"

MESSAGE_CLASS_DEFINITION(CameraWorker::MsgConfigureCameraWorker, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgRefreshCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportResolutions, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaCameraInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaStatus, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAvailableDevices, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportQtCameraCapabilities, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportQtFocusModes, Message)

CameraWorker::CameraWorker() :
    m_msgQueueToGUI(nullptr),
    m_availableDeviceHandler({}, QStringList{"spectrumview"}),
    m_capturing(false),
    m_imageSaved(false),
    m_captureTimer(this),
    m_networkManager(nullptr),
    m_alpacaFrameRequestPending(false),
    m_alpacaClientId(QRandomGenerator::global()->bounded(quint32(1), quint32(std::numeric_limits<quint32>::max()))),
    m_alpacaClientTransactionId(1),
    m_alpacaSensorType(0),
    m_alpacaImageBytesSupported(true),
    m_statusTimer(this),
    m_spectrumPipeSource(nullptr)
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    , m_qtCamera(nullptr)
    , m_videoSink(nullptr)
    , m_captureSession(nullptr)
#else
    , m_qtCamera(nullptr)
    , m_videoSurface(nullptr)
#endif
{
    QObject::connect(
        &m_availableDeviceHandler,
        &AvailableDeviceHandler::messageEnqueued,
        this,
        &CameraWorker::handleDeviceMessageQueue);
    QObject::connect(
        &m_availableDeviceHandler,
        &AvailableDeviceHandler::devicesChanged,
        this,
        &CameraWorker::onAvailableDevicesChanged);
    m_availableDeviceHandler.scanAvailableDevices();

    // Audio FIFO: stereo 16-bit PCM at 48 kHz; 4800 sample frames × 4 bytes each
    static constexpr int kAudioFifoFrames = 4800;
    static constexpr int kBytesPerSampleFrame = 4; // 2 channels × 2 bytes (int16)
    m_captureAudioFifo.setSize(kAudioFifoFrames);
    m_outputAudioFifo.setSize(kAudioFifoFrames);
    m_audioTransferBuffer.resize(kAudioFifoFrames * kBytesPerSampleFrame);

    QObject::connect(&m_captureAudioFifo, &AudioFifo::dataReady, this, &CameraWorker::onCaptureAudioDataReady);

    DSPEngine::instance()->getAudioDeviceManager()->addAudioSink(&m_outputAudioFifo, getInputMessageQueue());
    DSPEngine::instance()->getAudioDeviceManager()->addAudioSource(&m_captureAudioFifo, getInputMessageQueue());
}

CameraWorker::~CameraWorker()
{
    DSPEngine::instance()->getAudioDeviceManager()->removeAudioSource(&m_captureAudioFifo);
    DSPEngine::instance()->getAudioDeviceManager()->removeAudioSink(&m_outputAudioFifo);
    stopWork();
    QObject::disconnect(
        &m_availableDeviceHandler,
        &AvailableDeviceHandler::messageEnqueued,
        this,
        &CameraWorker::handleDeviceMessageQueue);
    QObject::disconnect(
        &m_availableDeviceHandler,
        &AvailableDeviceHandler::devicesChanged,
        this,
        &CameraWorker::onAvailableDevicesChanged);
}

void CameraWorker::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraWorker::handleInputMessages);
    QObject::connect(&m_captureTimer, &QTimer::timeout, this, &CameraWorker::captureTick);
    QObject::connect(&m_statusTimer, &QTimer::timeout, this, &CameraWorker::statusTick);

    if (!m_networkManager) {
        m_networkManager = new QNetworkAccessManager(this);
    }

    reportCameraList();

    // Notify GUI of already-known spectrum-view devices
    if (m_msgQueueToGUI)
    {
        const AvailableDeviceList& devices = m_availableDeviceHandler.getAvailableDeviceList();
        QStringList longIds;
        longIds.reserve(devices.size());
        for (const auto& device : devices) {
            longIds.append(device.getLongId());
        }
        m_msgQueueToGUI->push(MsgReportAvailableDevices::create(longIds));
    }

    if (m_settings.isAlpacaCamera())
    {
        alpacaQueryCameraCapabilities();
        m_statusTimer.start(500);
    }
    if (m_settings.m_captureActive) {
        startCapture();
    }
}

void CameraWorker::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraWorker::handleInputMessages);
    QObject::disconnect(&m_captureTimer, &QTimer::timeout, this, &CameraWorker::captureTick);
    QObject::disconnect(&m_statusTimer, &QTimer::timeout, this, &CameraWorker::statusTick);
    stopCapture();
    m_statusTimer.stop();
    m_inputMessageQueue.clear();
}

void CameraWorker::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraWorker::handleDeviceMessageQueue(MessageQueue* messageQueue)
{
    Message* message;

    while ((message = messageQueue->pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraWorker::onAvailableDevicesChanged(const QStringList& renameFrom, const QStringList& renameTo,
                                              const QStringList& removed, const QStringList& added)
{
    (void) renameFrom;
    (void) renameTo;
    (void) removed;
    (void) added;

    // Re-resolve the selected device pointer in case the device list changed
    m_spectrumPipeSource = nullptr;
    if (!m_settings.m_spectrumDevice.isEmpty())
    {
        const AvailableDeviceList& devices = m_availableDeviceHandler.getAvailableDeviceList();
        for (const auto& device : devices)
        {
            if (device.getLongId() == m_settings.m_spectrumDevice)
            {
                m_spectrumPipeSource = device.m_object;
                break;
            }
        }
    }

    if (!m_msgQueueToGUI) {
        return;
    }

    const AvailableDeviceList& devices = m_availableDeviceHandler.getAvailableDeviceList();
    QStringList longIds;
    longIds.reserve(devices.size());
    for (const auto& device : devices) {
        longIds.append(device.getLongId());
    }
    m_msgQueueToGUI->push(MsgReportAvailableDevices::create(longIds));
}

bool CameraWorker::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraWorker::match(cmd))
    {
        QMutexLocker locker(&m_mutex);
        MsgConfigureCameraWorker& cfg = (MsgConfigureCameraWorker&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgStartStop::match(cmd))
    {
        MsgStartStop& startStop = (MsgStartStop&) cmd;
        if (startStop.getStartStop()) {
            startCapture();
        } else {
            stopCapture();
        }
        return true;
    }
    else if (MsgRefreshCameraList::match(cmd))
    {
        reportCameraList();
        return true;
    }
    else if (MainCore::MsgImage::match(cmd))
    {
        MainCore::MsgImage& imgMsg = (MainCore::MsgImage&) cmd;
        // Only accept images from the selected device; if none is selected, accept all
        if (!m_spectrumPipeSource || imgMsg.getPipeSource() == m_spectrumPipeSource) {
            m_spectrumViewImage = imgMsg.getImage();
        }
        return true;
    }

    return false;
}

void CameraWorker::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraWorker::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    const bool recapture = force
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("framesPerSecond")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("isoSensitivity")
        || settingsKeys.contains("alpacaHost")
        || settingsKeys.contains("alpacaPort")
        || settingsKeys.contains("alpacaBinX")
        || settingsKeys.contains("alpacaBinY")
        || settingsKeys.contains("alpacaGain")
        || settingsKeys.contains("alpacaOffset")
        || settingsKeys.contains("alpacaReadoutMode");

    // Detect whether any post-processing parameter changed
    static const QStringList kPostProcessingKeys = {
        "brightness", "contrast", "invertColors", "overlayDateTime", "dateTimeColor",
        "dateTimeFormat", "dateTimePosX", "dateTimePosY",
        "diffMask", "dilationSize", "overlayFontFamily", "overlayFontScale",
        "motionDetect", "motionBoxColor", "minContourArea",
        "overlaySpectrum", "spectrumDevice", "spectrumOffsetX", "spectrumOffsetY", "spectrumScale",
        "yoloEnabled", "yoloModelPath", "yoloLabelsPath", "yoloConfThreshold", "yoloNmsThreshold", "yoloBoxColor"
    };
    const bool postProcessChanged = force || std::any_of(kPostProcessingKeys.cbegin(), kPostProcessingKeys.cend(),
        [&settingsKeys](const QString& k) { return settingsKeys.contains(k); });

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    // Reset diff-mask history when the feature is toggled off
    if ((force && !m_settings.m_diffMask) || (settingsKeys.contains("diffMask") && !m_settings.m_diffMask)) {
        m_previousRawFrame = QImage();
    }

    // Reset MOG2 state when motion detection is toggled off
    if ((force && !m_settings.m_motionDetect) || (settingsKeys.contains("motionDetect") && !m_settings.m_motionDetect)) {
        m_bgSubtractor = cv::Ptr<cv::BackgroundSubtractorMOG2>();
    }

    // Drop cached YOLO net if the model path changed so it will be reloaded lazily
    if (settingsKeys.contains("yoloModelPath") || (force && m_yoloLoadedModelPath != m_settings.m_yoloModelPath)) {
        m_yoloNet = cv::dnn::Net();
        m_yoloLoadedModelPath.clear();
    }

    // Drop cached YOLO labels if the labels path changed
    if (settingsKeys.contains("yoloLabelsPath") || (force && m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath)) {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();
    }

    if (settingsKeys.contains("captureActive") || force)
    {
        if (m_settings.m_captureActive) {
            startCapture();
        } else {
            stopCapture();
        }
    }
    else if (recapture && m_capturing)
    {
        stopCapture();
        startCapture();
    }

    // Manage VideoWriter independently of camera restart
    if (settingsKeys.contains("saveVideo") || settingsKeys.contains("videoFileName"))
    {
        if (m_videoWriter.isOpened()) {
            m_videoWriter.release();
        }
        // VideoWriter will be (re-)opened on the next frame if saveVideo is enabled
    }

    if (force || settingsKeys.contains("alpacaHost") || settingsKeys.contains("alpacaPort")) {
        reportCameraList();
    }

    if (force
        || settingsKeys.contains("cameraId"))
    {
        reportResolutions();
    }

    if (m_settings.isAlpacaCamera()
        && m_networkManager
        && (force
            || settingsKeys.contains("cameraId")
            || settingsKeys.contains("alpacaHost")
            || settingsKeys.contains("alpacaPort")
            || settingsKeys.contains("cameraId")))
    {
        alpacaQueryCameraCapabilities();
    }

    if (force || settingsKeys.contains("cameraId"))
    {
        if (m_settings.isAlpacaCamera()) {
            if (!m_statusTimer.isActive()) {
                m_statusTimer.start(500);
            }
        } else {
            m_statusTimer.stop();
        }
    }

    // Resolve the device object pointer when spectrumDevice setting changes
    if (force || settingsKeys.contains("spectrumDevice"))
    {
        m_spectrumPipeSource = nullptr;
        if (!m_settings.m_spectrumDevice.isEmpty())
        {
            const AvailableDeviceList& devices = m_availableDeviceHandler.getAvailableDeviceList();
            for (const auto& device : devices)
            {
                if (device.getLongId() == m_settings.m_spectrumDevice)
                {
                    m_spectrumPipeSource = device.m_object;
                    break;
                }
            }
        }
        // When the device changes, clear the cached image to avoid showing a stale overlay
        m_spectrumViewImage = QImage();
    }

    // If a post-processing parameter changed and we have a stored raw frame, reprocess and push to GUI
    if (postProcessChanged && !m_lastRawFrame.isNull()) {
        const QImage processed = applyPostProcessing(m_lastRawFrame);
        reportFrameToGUI(processed);
    }

    // Update audio output device when the name changes
    if (force || settingsKeys.contains("audioDeviceName"))
    {
        AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
        const int outputDeviceIndex = audioDeviceManager->getOutputDeviceIndex(m_settings.m_audioDeviceName);
        audioDeviceManager->removeAudioSink(&m_outputAudioFifo);
        audioDeviceManager->addAudioSink(&m_outputAudioFifo, getInputMessageQueue(), outputDeviceIndex);
    }

    // Apply Qt-camera image-control settings inline (no recapture needed)
    if (m_settings.isQtCamera() && m_capturing && m_qtCamera)
    {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        if (force || settingsKeys.contains("whiteBalanceMode")) {
            m_qtCamera->setWhiteBalanceMode(static_cast<QCamera::WhiteBalanceMode>(m_settings.m_whiteBalanceMode));
        }
        if (force || settingsKeys.contains("exposureCompensation")) {
            m_qtCamera->setExposureCompensation(static_cast<float>(m_settings.m_exposureCompensation));
        }
        if (force || settingsKeys.contains("focusMode")) {
            m_qtCamera->setFocusMode(static_cast<QCamera::FocusMode>(m_settings.m_focusMode));
        }
        if (force || settingsKeys.contains("focusDistance")) {
            m_qtCamera->setFocusDistance(static_cast<float>(m_settings.m_focusDistance));
        }
        if (force || settingsKeys.contains("zoomFactor")) {
            const float clampedZoom = static_cast<float>(
                qBound(m_qtCamera->minimumZoomFactor(), static_cast<float>(m_settings.m_zoomFactor), m_qtCamera->maximumZoomFactor()));
            m_qtCamera->setZoomFactor(clampedZoom);
        }
#else
        if (force || settingsKeys.contains("whiteBalanceMode"))
        {
            QCameraImageProcessing *imageProcessing = m_qtCamera->imageProcessing();
            if (imageProcessing) {
                imageProcessing->setWhiteBalanceMode(
                    static_cast<QCameraImageProcessing::WhiteBalanceMode>(m_settings.m_whiteBalanceMode));
            }
        }
        if (force || settingsKeys.contains("zoomFactor"))
        {
            QCameraFocus *cameraFocus = m_qtCamera->focus();
            if (cameraFocus) {
                const qreal minZoom = cameraFocus->minimumOpticalZoom();
                const qreal maxZoom = cameraFocus->maximumOpticalZoom();
                const qreal clampedZoom = qBound(minZoom, static_cast<qreal>(m_settings.m_zoomFactor), maxZoom);
                cameraFocus->setOpticalZoom(clampedZoom);
            }
        }
#endif
    }
}

void CameraWorker::reportCameraList()
{
    QStringList qtCameraIds;

    // List Qt cameras
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();

    for (const QCameraDevice& camera : cameras)
    {
        const QString id = QString("qt:%1:%2").arg(QString::fromUtf8(camera.id())).arg(camera.description());
        qtCameraIds.append(id);
    }
#else
    const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();

    for (const QCameraInfo& info : cameras)
    {
        const QString id = QString("qt:%1:%2").arg(info.deviceName()).arg(info.description());
        qtCameraIds.append(id);
    }
#endif

    // List Alpaca cameras
    if (m_networkManager)
    {
        QNetworkRequest request(QUrl(buildAlpacaBaseUrl() + "/management/v1/configureddevices"));
        QNetworkReply *reply = m_networkManager->get(request);

        QObject::connect(reply, &QNetworkReply::finished, this, [this, reply, qtCameraIds]() {
            QStringList cameraIds = qtCameraIds;

            if (reply->error() == QNetworkReply::NoError) {
                cameraIds.append(parseAlpacaCameraList(reply->readAll()));
            }

            if (m_msgQueueToGUI) {
                m_msgQueueToGUI->push(MsgReportCameraList::create(cameraIds));
            }

            reply->deleteLater();
        });
    }
    else
    {
        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportCameraList::create(qtCameraIds));
        }
    }
}

void CameraWorker::reportResolutions()
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_settings.isQtCamera())
    {
        const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
        QList<QSize> resolutions;
        const QString targetId = m_settings.cameraIdString();
        const QString targetDescription = m_settings.cameraDescription();

        for (const QCameraDevice& device : cameras)
        {
            const QString id = QString::fromUtf8(device.id());
            if ((id == targetId) || (device.description() == targetDescription))
            {
                QSet<QString> seen;
                for (const QCameraFormat& format : device.videoFormats())
                {
                    const QString key = QString("%1x%2")
                        .arg(format.resolution().width())
                        .arg(format.resolution().height());
                    if (!seen.contains(key))
                    {
                        seen.insert(key);
                        resolutions.append(format.resolution());
                    }
                }
                break;
            }
        }

        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportResolutions::create(resolutions));
        }
    }
#else
    // Qt5: enumerate supported viewfinder resolutions from the selected camera.
    // QCamera::supportedViewfinderResolutions() is available once the camera is loaded;
    // try it optimistically — it will return an empty list on platforms that don't support
    // it before loading, in which case the GUI shows no pre-defined resolutions and the
    // camera uses its default.
    if (m_settings.isQtCamera())
    {
        QList<QSize> resolutions;
        const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
        const QString targetId = m_settings.cameraIdString();
        const QString targetDescription = m_settings.cameraDescription();

        for (const QCameraInfo& info : cameras)
        {
            const QString id = info.deviceName();
            if ((id == targetId) || (info.description() == targetDescription))
            {
                QCamera tempCam(info);
                QSet<QString> seen;

                for (const QSize& res : tempCam.supportedViewfinderResolutions())
                {
                    const QString key = QString("%1x%2").arg(res.width()).arg(res.height());
                    if (!seen.contains(key))
                    {
                        seen.insert(key);
                        resolutions.append(res);
                    }
                }

                break;
            }
        }

        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportResolutions::create(resolutions));
        }
    }
#endif
}

void CameraWorker::startCapture()
{
    if (m_capturing) {
        return;
    }

    m_imageSaved = false;
    m_capturing = true;

    if (m_settings.isAlpacaCamera())
    {
        m_alpacaFrameRequestPending = false;
        const int intervalMs = std::max(10, static_cast<int>(std::lround(1000.0 / std::max(1, m_settings.m_framesPerSecond))));
        m_captureTimer.start(intervalMs);
        captureTick();
    }
    else if (m_settings.isQtCamera())
    {
        setupQtCapture();
    }
}

void CameraWorker::stopCapture()
{
    m_capturing = false;
    m_captureTimer.stop();

    if (m_videoWriter.isOpened()) {
        m_videoWriter.release();
    }

    cleanupQtCapture();
}

void CameraWorker::captureTick()
{
    if (!m_capturing) {
        return;
    }

    if (!m_networkManager || m_alpacaFrameRequestPending) {
        return;
    }

    m_alpacaFrameRequestPending = true;
    alpacaSetCameraParams();
}

// Helper: PUT a simple integer property on the Alpaca camera synchronously (via async reply),
// then invoke the continuation lambda.
static void alpacaPutIntProperty(
    QNetworkAccessManager *nam,
    const QString& baseUrl,
    int cameraId,
    const QString& property,
    const QString& bodyKey,
    int value,
    quint32 clientId,
    quint32& transactionId,
    std::function<void()> continuation)
{
    QUrl url(baseUrl + QString("/api/v1/camera/%1/%2").arg(cameraId).arg(property));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem(bodyKey, QString::number(value));
    body.addQueryItem("ClientID", QString::number(clientId));
    body.addQueryItem("ClientTransactionID", QString::number(transactionId++));

    QNetworkReply *reply = nam->put(request, body.toString(QUrl::FullyEncoded).toUtf8());

    QObject::connect(reply, &QNetworkReply::finished, reply, [reply, property, continuation]() {
        if (reply->error() != QNetworkReply::NoError) {
            qDebug() << "CameraWorker: PUT" << property << "error:" << reply->errorString();
        }
        reply->deleteLater();
        continuation();
    });
}

void CameraWorker::alpacaSetCameraParams()
{
    // Chain: binX -> binY -> gain -> offset -> readoutMode -> startExposure
    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.cameraIdInt();

    auto doStartExposure = [this]() {
        if (m_capturing) {
            alpacaStartExposure();
        } else {
            m_alpacaFrameRequestPending = false;
        }
    };

    auto doReadoutMode = [this, baseUrl, camId, doStartExposure]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        alpacaPutIntProperty(m_networkManager, baseUrl, camId, "readoutmode", "ReadoutMode",
            m_settings.m_alpacaReadoutMode, m_alpacaClientId, m_alpacaClientTransactionId, doStartExposure);
    };

    auto doOffset = [this, baseUrl, camId, doReadoutMode]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (m_settings.m_alpacaOffset >= 0) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "offset", "Offset",
                m_settings.m_alpacaOffset, m_alpacaClientId, m_alpacaClientTransactionId, doReadoutMode);
        } else {
            doReadoutMode();
        }
    };

    auto doGain = [this, baseUrl, camId, doOffset]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (m_settings.m_alpacaGain >= 0) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "gain", "Gain",
                m_settings.m_alpacaGain, m_alpacaClientId, m_alpacaClientTransactionId, doOffset);
        } else {
            doOffset();
        }
    };

    auto doBinY = [this, baseUrl, camId, doGain]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        alpacaPutIntProperty(m_networkManager, baseUrl, camId, "biny", "BinY",
            m_settings.m_alpacaBinY, m_alpacaClientId, m_alpacaClientTransactionId, doGain);
    };

    alpacaPutIntProperty(m_networkManager, baseUrl, camId, "binx", "BinX",
        m_settings.m_alpacaBinX, m_alpacaClientId, m_alpacaClientTransactionId, doBinY);
}

void CameraWorker::alpacaStartExposure()
{
    QUrl url(buildAlpacaBaseUrl() + QString("/api/v1/camera/%1/startexposure").arg(m_settings.cameraIdInt()));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    const double durationSecs = m_settings.m_exposureTimeMs / 1000.0;
    QUrlQuery body;
    body.addQueryItem("Duration", QString::number(durationSecs, 'f', 3));
    body.addQueryItem("Light", "True");
    body.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));

    QNetworkReply *reply = m_networkManager->put(request, body.toString(QUrl::FullyEncoded).toUtf8());

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        reply->deleteLater();

        if (!m_capturing) {
            m_alpacaFrameRequestPending = false;
            return;
        }

        if (reply->error() != QNetworkReply::NoError) {
            qDebug() << "CameraWorker::alpacaStartExposure: error:" << reply->errorString();
            m_alpacaFrameRequestPending = false;
            return;
        }

        // Wait for the exposure duration before polling imageready
        QTimer::singleShot(m_settings.m_exposureTimeMs, this, [this]() {
            if (m_capturing) {
                alpacaCheckImageReady();
            } else {
                m_alpacaFrameRequestPending = false;
            }
        });
    });
}

void CameraWorker::alpacaCheckImageReady()
{
    QUrl url(buildAlpacaBaseUrl() + QString("/api/v1/camera/%1/imageready").arg(m_settings.cameraIdInt()));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
    url.setQuery(query);

    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        reply->deleteLater();

        if (!m_capturing) {
            m_alpacaFrameRequestPending = false;
            return;
        }

        bool ready = false;
        if (reply->error() == QNetworkReply::NoError) {
            const QJsonDocument doc = QJsonDocument::fromJson(reply->readAll());
            if (doc.isObject()) {
                ready = doc.object().value("Value").toBool(false);
            }
        }

        if (ready) {
            alpacaFetchImageArray();
        } else {
            // Image not yet ready — poll again after a short delay
            QTimer::singleShot(100, this, [this]() {
                if (m_capturing) {
                    alpacaCheckImageReady();
                } else {
                    m_alpacaFrameRequestPending = false;
                }
            });
        }
    });
}

void CameraWorker::alpacaFetchImageArray()
{
    QUrl url(buildAlpacaBaseUrl() + QString("/api/v1/camera/%1/imagearray").arg(m_settings.cameraIdInt()));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
    url.setQuery(query);

    QNetworkRequest request(url);
    // Signal support for the faster binary ImageBytes protocol; server falls back to JSON if unsupported
    if (m_alpacaImageBytesSupported) {
        request.setRawHeader("Accept", "application/imagebytes");
    }

    QNetworkReply *reply = m_networkManager->get(request);

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        m_alpacaFrameRequestPending = false;

        if (!m_capturing) {
            reply->deleteLater();
            return;
        }

        QImage image = createPlaceholderFrame();

        if (reply->error() == QNetworkReply::NoError) {
            const QByteArray data = reply->readAll();
            const QString contentType = reply->header(QNetworkRequest::ContentTypeHeader).toString();
            if (contentType.contains(QLatin1String("application/imagebytes"), Qt::CaseInsensitive)) {
                m_alpacaImageBytesSupported = true;
                image = parseAlpacaImageBytes(data);
            } else {
                // Server returned JSON — either it doesn't support ImageBytes or we didn't request it
                if (m_alpacaImageBytesSupported) {
                    qDebug() << "CameraWorker::alpacaFetchImageArray: server returned JSON; disabling ImageBytes for this camera";
                    m_alpacaImageBytesSupported = false;
                }
                image = parseAlpacaImageArray(data);
            }
        }

        processNewFrame(image);
        reply->deleteLater();
    });
}

void CameraWorker::processNewFrame(const QImage& image)
{
    m_captureDateTime = QDateTime::currentDateTime();

    // Apply all post-processing effects; result is what the GUI will display
    const QImage processed = applyPostProcessing(image);

    // Advance the raw-frame history (used by diff mask on the next frame)
    m_previousRawFrame = m_lastRawFrame;
    m_lastRawFrame = image;

    reportFrameToGUI(processed);

    // Save a single JPEG of the raw frame per capture session
    if (m_settings.m_saveImage && !m_imageSaved && !m_settings.m_imageFileName.isEmpty())
    {
        image.save(m_settings.m_imageFileName, "JPEG");
        m_imageSaved = true;
    }

    // Write video frame (raw or post-processed, depending on setting)
    if (m_settings.m_saveVideo && !m_settings.m_videoFileName.isEmpty())
    {
        // Lazily open the VideoWriter on the first frame so we know the frame size
        if (!m_videoWriter.isOpened())
        {
            // 'mp4v' (MPEG-4 Part 2) is widely supported by OpenCV across all platforms.
            // The output file extension (.mp4) determines the container format.
            const int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            m_videoWriter.open(
                m_settings.m_videoFileName.toStdString(),
                fourcc,
                std::max(1, m_settings.m_framesPerSecond),
                cv::Size(image.width(), image.height()),
                true);
        }

        if (m_videoWriter.isOpened())
        {
            const QImage& frameToWrite = m_settings.m_videoPostProcess ? processed : image;
            const QImage rgb = frameToWrite.convertToFormat(QImage::Format_RGB888);
            cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                        const_cast<uchar*>(rgb.bits()),
                        static_cast<size_t>(rgb.bytesPerLine()));
            cv::Mat bgrMat;
            cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
            m_videoWriter.write(bgrMat);
        }
    }
}

void CameraWorker::reportFrameToGUI(const QImage& image)
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportFrame::create(image));
    }
}

/**
 * Applies all enabled post-processing effects to @p input in order:
 *   1. Brightness/contrast adjustment
 *   2. Colour inversion
 *   3. Diff mask against the previous raw frame
 *   4. MOG2 motion detection with bounding boxes
 *   5. Spectrum view image overlay
 *   6. Date/time text overlay (rendered with QPainter onto the final QImage)
 *
 * Returns the processed image, or a copy of @p input when no effects are active.
 * Must be called from the worker thread only (modifies m_bgSubtractor).
 */
QImage CameraWorker::applyPostProcessing(const QImage& input)
{
    // Pixel grayscale difference (0–255) below which a pixel is considered unchanged between frames.
    static constexpr int kDiffThreshold = 30;

    const bool needsSpectrumOverlay = m_settings.m_overlaySpectrum && !m_spectrumViewImage.isNull();
    const bool needsBrightContrast = (m_settings.m_brightness != 0.0 || m_settings.m_contrast != 1.0);
    const bool needsAny = needsBrightContrast
        || m_settings.m_invertColors
        || m_settings.m_overlayDateTime
        || (m_settings.m_diffMask && !m_previousRawFrame.isNull())
        || m_settings.m_motionDetect
        || (m_settings.m_yoloEnabled && !m_settings.m_yoloModelPath.isEmpty())
        || needsSpectrumOverlay;

    if (!needsAny) {
        return input;
    }

    const QImage rgb = input.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uchar*>(rgb.bits()),
                static_cast<size_t>(rgb.bytesPerLine()));
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);

    if (needsBrightContrast)
    {
        cv::Mat adjusted;
        cv::convertScaleAbs(bgrMat, adjusted, m_settings.m_contrast, m_settings.m_brightness);
        bgrMat = adjusted;
    }

    if (m_settings.m_invertColors) {
        cv::bitwise_not(bgrMat, bgrMat);
    }

    if (m_settings.m_diffMask && !m_previousRawFrame.isNull()
        && m_previousRawFrame.width() == input.width()
        && m_previousRawFrame.height() == input.height())
    {
        const QImage prevRgb = m_previousRawFrame.convertToFormat(QImage::Format_RGB888);
        cv::Mat prevMat(prevRgb.height(), prevRgb.width(), CV_8UC3,
                        const_cast<uchar*>(prevRgb.bits()),
                        static_cast<size_t>(prevRgb.bytesPerLine()));
        cv::Mat prevBgr;
        cv::cvtColor(prevMat, prevBgr, cv::COLOR_RGB2BGR);

        cv::Mat gray, prevGray, diff, mask;
        cv::cvtColor(bgrMat, gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(prevBgr, prevGray, cv::COLOR_BGR2GRAY);
        cv::absdiff(gray, prevGray, diff);
        cv::threshold(diff, mask, kDiffThreshold, 255, cv::THRESH_BINARY);

        if (m_settings.m_dilationSize > 0)
        {
            const int ksize = 2 * m_settings.m_dilationSize + 1;
            const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
            cv::dilate(mask, mask, kernel);
        }

        cv::Mat result = cv::Mat::zeros(bgrMat.size(), bgrMat.type());
        cv::bitwise_and(bgrMat, bgrMat, result, mask);
        bgrMat = result;
    }

    if (m_settings.m_motionDetect)
    {
        if (!m_bgSubtractor) {
            m_bgSubtractor = cv::createBackgroundSubtractorMOG2();
        }

        cv::Mat fgMask;
        m_bgSubtractor->apply(bgrMat, fgMask);
        cv::threshold(fgMask, fgMask, 200, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        const QColor& bc = m_settings.m_motionBoxColor;
        const cv::Scalar boxColor(bc.blue(), bc.green(), bc.red());
        for (const auto& contour : contours)
        {
            if (cv::contourArea(contour) >= static_cast<double>(m_settings.m_minContourArea)) {
                cv::rectangle(bgrMat, cv::boundingRect(contour), boxColor, 2);
            }
        }
    }

    if (m_settings.m_yoloEnabled && !m_settings.m_yoloModelPath.isEmpty())
    {
        runYoloDetections(bgrMat);
    }

    if (m_settings.m_overlayDateTime)
    {
        // Date/time text is rendered after the cv::Mat pipeline using QPainter,
        // which gives access to the full system font library.
    }

    if (needsSpectrumOverlay)
    {
        // Scale the spectrum image if a non-unity scale is requested
        QImage specSrc = m_spectrumViewImage;
        if (qAbs(m_settings.m_spectrumScale - 1.0) > 1e-4)
        {
            const int sw = static_cast<int>(specSrc.width()  * m_settings.m_spectrumScale);
            const int sh = static_cast<int>(specSrc.height() * m_settings.m_spectrumScale);
            if (sw > 0 && sh > 0) {
                specSrc = specSrc.scaled(sw, sh, Qt::KeepAspectRatio, Qt::SmoothTransformation);
            }
        }

        const QImage specRgb = specSrc.convertToFormat(QImage::Format_RGBA8888);
        const int dstW = bgrMat.cols;
        const int dstH = bgrMat.rows;
        const int ox = m_settings.m_spectrumOffsetX;
        const int oy = m_settings.m_spectrumOffsetY;
        const int sw = specRgb.width();
        const int sh = specRgb.height();

        // Determine the intersection of the spectrum image with the camera frame
        const int srcX0 = std::max(0, -ox);
        const int srcY0 = std::max(0, -oy);
        const int srcX1 = std::min(sw, dstW - ox);
        const int srcY1 = std::min(sh, dstH - oy);

        for (int sy = srcY0; sy < srcY1; ++sy)
        {
            const uchar* srcRow = specRgb.constScanLine(sy);
            const int dy = oy + sy;
            uchar* dstRow = bgrMat.ptr<uchar>(dy);

            for (int sx = srcX0; sx < srcX1; ++sx)
            {
                const int srcPx = sx * 4;
                const uchar alpha = srcRow[srcPx + 3];
                if (alpha == 0) {
                    continue;
                }
                const int dx = (ox + sx) * 3;
                if (alpha == 255)
                {
                    // Fully opaque: direct copy (note: bgrMat is BGR, specRgb is RGBA)
                    dstRow[dx]     = srcRow[srcPx + 2]; // B
                    dstRow[dx + 1] = srcRow[srcPx + 1]; // G
                    dstRow[dx + 2] = srcRow[srcPx];     // R
                }
                else
                {
                    // Alpha-blend
                    const int a = alpha;
                    const int invA = 255 - a;
                    dstRow[dx]     = static_cast<uchar>((srcRow[srcPx + 2] * a + dstRow[dx]     * invA) / 255);
                    dstRow[dx + 1] = static_cast<uchar>((srcRow[srcPx + 1] * a + dstRow[dx + 1] * invA) / 255);
                    dstRow[dx + 2] = static_cast<uchar>((srcRow[srcPx]     * a + dstRow[dx + 2] * invA) / 255);
                }
            }
        }
    }

    cv::cvtColor(bgrMat, bgrMat, cv::COLOR_BGR2RGB);
    const QImage rawResult(bgrMat.data, bgrMat.cols, bgrMat.rows,
                           static_cast<qsizetype>(bgrMat.step[0]),
                           QImage::Format_RGB888);
    QImage result = rawResult.copy(); // detach from cv::Mat memory

    if (m_settings.m_overlayDateTime)
    {
        const QString fmt = m_settings.m_dateTimeFormat.isEmpty()
                            ? QStringLiteral("yyyy-MM-dd hh:mm:ss")
                            : m_settings.m_dateTimeFormat;
        const QString text = m_captureDateTime.toString(fmt);
        QFont font;
        if (!m_settings.m_overlayFontFamily.isEmpty()) {
            font.setFamily(m_settings.m_overlayFontFamily);
        }
        font.setPointSizeF(m_settings.m_overlayFontScale);
        const QFontMetrics fm(font);
        QPainter painter(&result);
        painter.setRenderHint(QPainter::TextAntialiasing);
        painter.setFont(font);
        painter.setPen(m_settings.m_dateTimeColor);
        const int x = m_settings.m_dateTimePosX;
        const int y = (m_settings.m_dateTimePosY > 0)
                      ? m_settings.m_dateTimePosY
                      : result.height() - fm.descent() - 2;
        painter.drawText(x, y, text);
    }

    return result;
}

void CameraWorker::runYoloDetections(cv::Mat& bgrMat)
{
    // Lazily load class labels
    if (m_yoloLoadedLabelsPath != m_settings.m_yoloLabelsPath)
    {
        m_yoloLabels.clear();
        m_yoloLoadedLabelsPath.clear();

        if (!m_settings.m_yoloLabelsPath.isEmpty())
        {
            QFile f(m_settings.m_yoloLabelsPath);
            if (f.open(QIODevice::ReadOnly | QIODevice::Text))
            {
                QTextStream ts(&f);
                while (!ts.atEnd())
                {
                    const QString line = ts.readLine().trimmed();
                    if (!line.isEmpty()) {
                        m_yoloLabels.append(line);
                    }
                }
                m_yoloLoadedLabelsPath = m_settings.m_yoloLabelsPath;
            }
            else
            {
                qWarning() << "CameraWorker::runYoloDetections: cannot open labels file:" << m_settings.m_yoloLabelsPath;
            }
        }
    }

    // Lazily load the ONNX model
    if (m_yoloLoadedModelPath != m_settings.m_yoloModelPath)
    {
        m_yoloNet = cv::dnn::Net();
        m_yoloLoadedModelPath.clear();

        if (!m_settings.m_yoloModelPath.isEmpty())
        {
            try
            {
                m_yoloNet = cv::dnn::readNetFromONNX(m_settings.m_yoloModelPath.toStdString());
                m_yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
                m_yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                m_yoloLoadedModelPath = m_settings.m_yoloModelPath;
                qDebug() << "CameraWorker::runYoloDetections: loaded model" << m_settings.m_yoloModelPath;
            }
            catch (const cv::Exception& e)
            {
                qWarning() << "CameraWorker::runYoloDetections: failed to load model:" << e.what();
                return;
            }
        }
        else
        {
            return;
        }
    }

    if (m_yoloNet.empty()) {
        return;
    }

    // Build a 640×640 blob from the frame (letterboxing handled by the model)
    const int inputSize = 640;
    const float scaleX = static_cast<float>(bgrMat.cols) / inputSize;
    const float scaleY = static_cast<float>(bgrMat.rows) / inputSize;

    cv::Mat blob;
    cv::dnn::blobFromImage(bgrMat, blob, 1.0 / 255.0,
                           cv::Size(inputSize, inputSize),
                           cv::Scalar(), /*swapRB=*/true, /*crop=*/false);

    m_yoloNet.setInput(blob);

    std::vector<cv::Mat> outputs;
    try
    {
        m_yoloNet.forward(outputs, m_yoloNet.getUnconnectedOutLayersNames());
    }
    catch (const cv::Exception& e)
    {
        qWarning() << "CameraWorker::runYoloDetections: inference failed:" << e.what();
        return;
    }

    if (outputs.empty()) {
        return;
    }

    // Parse detections. Supports two common YOLO ONNX output layouts:
    //
    // YOLOv8 / YOLOv9 / YOLOv10:  [1, (4 + numClasses), numAnchors]
    //   rows = 4 + numClasses, cols = numAnchors, no explicit objectness score.
    //
    // YOLOv5 / YOLOv7:             [1, numAnchors, (5 + numClasses)]
    //   rows = numAnchors, cols = 5 + numClasses, col[4] is objectness.
    //
    // We detect which layout is in use by checking which dimension is larger.

    cv::Mat det = outputs[0];
    if (det.dims == 3) {
        // Squeeze the batch dimension: [1, rows, cols] → [rows, cols]
        det = det.reshape(1, det.size[1]);
    }
    // det is now a 2D matrix [rows × cols]

    const float confThresh = static_cast<float>(m_settings.m_yoloConfThreshold);
    const float nmsThresh  = static_cast<float>(m_settings.m_yoloNmsThreshold);

    std::vector<cv::Rect> boxes;
    std::vector<float>    scores;
    std::vector<int>      classIds;

    // Determine which layout we have
    // YOLOv8 style: rows <= ~200 (4 + nClasses), cols >> rows
    const bool isV8Style = (det.rows < det.cols);

    if (isV8Style)
    {
        // det: [4 + numClasses, numAnchors]
        // Each column is one anchor: [cx, cy, w, h, cls0, cls1, ...]
        const int numAnchors = det.cols;
        const int numClasses = det.rows - 4;
        if (numClasses <= 0) {
            return;
        }

        for (int a = 0; a < numAnchors; ++a)
        {
            // Find the class with the highest score
            float bestScore = 0.0f;
            int   bestClass = 0;
            for (int c = 0; c < numClasses; ++c)
            {
                const float s = det.at<float>(4 + c, a);
                if (s > bestScore) {
                    bestScore = s;
                    bestClass = c;
                }
            }
            if (bestScore < confThresh) {
                continue;
            }

            const float cx = det.at<float>(0, a) * scaleX;
            const float cy = det.at<float>(1, a) * scaleY;
            const float  w = det.at<float>(2, a) * scaleX;
            const float  h = det.at<float>(3, a) * scaleY;

            const int x1 = static_cast<int>(cx - w / 2.0f);
            const int y1 = static_cast<int>(cy - h / 2.0f);

            boxes.push_back(cv::Rect(x1, y1, static_cast<int>(w), static_cast<int>(h)));
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
        }
    }
    else
    {
        // YOLOv5 style: [numAnchors, 5 + numClasses]
        const int numAnchors = det.rows;
        const int numClasses = det.cols - 5;
        if (numClasses < 0) {
            return;
        }

        for (int a = 0; a < numAnchors; ++a)
        {
            const float objectness = det.at<float>(a, 4);
            if (objectness < confThresh) {
                continue;
            }

            float bestScore = 0.0f;
            int   bestClass = 0;
            for (int c = 0; c < numClasses; ++c)
            {
                const float s = objectness * det.at<float>(a, 5 + c);
                if (s > bestScore) {
                    bestScore = s;
                    bestClass = c;
                }
            }
            if (bestScore < confThresh) {
                continue;
            }

            const float cx = det.at<float>(a, 0) * scaleX;
            const float cy = det.at<float>(a, 1) * scaleY;
            const float  w = det.at<float>(a, 2) * scaleX;
            const float  h = det.at<float>(a, 3) * scaleY;

            const int x1 = static_cast<int>(cx - w / 2.0f);
            const int y1 = static_cast<int>(cy - h / 2.0f);

            boxes.push_back(cv::Rect(x1, y1, static_cast<int>(w), static_cast<int>(h)));
            scores.push_back(bestScore);
            classIds.push_back(bestClass);
        }
    }

    // Apply NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, confThresh, nmsThresh, indices);

    // Draw bounding boxes and labels
    const QColor& bc = m_settings.m_yoloBoxColor;
    const cv::Scalar boxColor(bc.blue(), bc.green(), bc.red());
    const cv::Scalar textBg(0, 0, 0);

    for (int idx : indices)
    {
        const cv::Rect& box = boxes[idx];
        cv::rectangle(bgrMat, box, boxColor, 2);

        // Build label text
        QString label;
        if (!m_yoloLabels.isEmpty() && classIds[idx] < m_yoloLabels.size()) {
            label = m_yoloLabels[classIds[idx]];
        } else {
            label = QStringLiteral("cls%1").arg(classIds[idx]);
        }
        label += QStringLiteral(" %1%").arg(static_cast<int>(scores[idx] * 100.0f + 0.5f));

        const std::string labelStd = label.toStdString();
        int baseLine = 0;
        const cv::Size textSize = cv::getTextSize(labelStd, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        const int labelY = std::max(box.y, textSize.height + 2);
        cv::rectangle(bgrMat,
                      cv::Point(box.x, labelY - textSize.height - 2),
                      cv::Point(box.x + textSize.width, labelY + baseLine),
                      textBg, cv::FILLED);
        cv::putText(bgrMat, labelStd,
                    cv::Point(box.x, labelY),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, boxColor, 1, cv::LINE_AA);
    }
}

void CameraWorker::alpacaQueryCameraCapabilities()
{
    if (!m_networkManager) {
        return;
    }

    // Reset ImageBytes support flag so we re-probe support for the new camera;
    // cameras on the same Alpaca server may have different capabilities.
    m_alpacaImageBytesSupported = true;

    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.cameraIdInt();

    // Struct to accumulate results from parallel requests
    struct CapInfo {
        int maxBinX = 1;
        int maxBinY = 1;
        QStringList gains;
        int gainMin = 0;
        int gainMax = 0;
        QStringList offsets;
        int offsetMin = 0;
        int offsetMax = 0;
        QStringList readoutModes;
        QString sensorName;
        int sensorType = 0;
        double pixelSizeX = 0.0;
        double pixelSizeY = 0.0;
        int cameraSizeX = 0;
        int cameraSizeY = 0;
        double ccdTemperature = 0.0;
        bool ccdTemperatureValid = false;
        int pending = 0;
    };

    auto info = QSharedPointer<CapInfo>::create();

    // Properties to query: name, JSON Value key, handler
    static const QStringList properties = {
        "maxbinx", "maxbiny", "gains", "gainmin", "gainmax",
        "offsets", "offsetmin", "offsetmax",
        "readoutmodes", "sensorname", "sensortype",
        "pixelsizex", "pixelsizey", "cameraxsize", "cameraysize",
        "ccdtemperature"
    };

    info->pending = properties.size();

    auto checkDone = [this, info]() {
        info->pending--;
        if (info->pending > 0) {
            return;
        }

        m_alpacaSensorType = info->sensorType;

        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportAlpacaCameraInfo::create(
                info->maxBinX, info->maxBinY,
                info->gains, info->gainMin, info->gainMax,
                info->offsets, info->offsetMin, info->offsetMax,
                info->readoutModes,
                info->sensorName, info->sensorType,
                info->pixelSizeX, info->pixelSizeY,
                info->cameraSizeX, info->cameraSizeY,
                info->ccdTemperature, info->ccdTemperatureValid));
        }
    };

    auto query = [this, baseUrl, camId, info, checkDone](const QString& prop) {
        QUrl url(baseUrl + QString("/api/v1/camera/%1/%2").arg(camId).arg(prop));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_alpacaClientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
        url.setQuery(q);

        QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

        QObject::connect(reply, &QNetworkReply::finished, reply, [reply, prop, info, checkDone]() {
            if (reply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(reply->readAll());
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    const int errNum = root.value("ErrorNumber").toInt(0);
                    if (errNum == 0) {
                        const QJsonValue val = root.value("Value");
                        if (prop == "maxbinx") {
                            info->maxBinX = std::max(1, val.toInt(1));
                        } else if (prop == "maxbiny") {
                            info->maxBinY = std::max(1, val.toInt(1));
                        } else if (prop == "gains") {
                            if (val.isArray()) {
                                for (const QJsonValue& g : val.toArray()) {
                                    info->gains.append(g.toString());
                                }
                            }
                        } else if (prop == "gainmin") {
                            info->gainMin = val.toInt(0);
                        } else if (prop == "gainmax") {
                            info->gainMax = val.toInt(0);
                        } else if (prop == "offsets") {
                            if (val.isArray()) {
                                for (const QJsonValue& o : val.toArray()) {
                                    info->offsets.append(o.toString());
                                }
                            }
                        } else if (prop == "offsetmin") {
                            info->offsetMin = val.toInt(0);
                        } else if (prop == "offsetmax") {
                            info->offsetMax = val.toInt(0);
                        } else if (prop == "readoutmodes") {
                            if (val.isArray()) {
                                for (const QJsonValue& m : val.toArray()) {
                                    info->readoutModes.append(m.toString());
                                }
                            }
                        } else if (prop == "sensorname") {
                            info->sensorName = val.toString();
                        } else if (prop == "sensortype") {
                            info->sensorType = val.toInt(0);
                        } else if (prop == "pixelsizex") {
                            info->pixelSizeX = val.toDouble(0.0);
                        } else if (prop == "pixelsizey") {
                            info->pixelSizeY = val.toDouble(0.0);
                        } else if (prop == "cameraxsize") {
                            info->cameraSizeX = val.toInt(0);
                        } else if (prop == "cameraysize") {
                            info->cameraSizeY = val.toInt(0);
                        } else if (prop == "ccdtemperature") {
                            info->ccdTemperature = val.toDouble(0.0);
                            info->ccdTemperatureValid = true;
                        }
                    }
                }
            }
            reply->deleteLater();
            checkDone();
        });
    };

    for (const QString& prop : properties) {
        query(prop);
    }
}

void CameraWorker::statusTick()
{
    if (m_networkManager && m_settings.isAlpacaCamera()) {
        alpacaPollStatus();
    }
}

void CameraWorker::alpacaPollStatus()
{
    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.cameraIdInt();

    // Accumulate results from two parallel GETs
    struct StatusInfo {
        int cameraState = -1;
        double ccdTemperature = 0.0;
        bool ccdTemperatureValid = false;
        int pending = 0;
    };

    auto status = QSharedPointer<StatusInfo>::create();
    status->pending = 2;

    auto checkDone = [this, status]() {
        status->pending--;
        if (status->pending > 0) {
            return;
        }
        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportAlpacaStatus::create(
                status->cameraState,
                status->ccdTemperature,
                status->ccdTemperatureValid));
        }
    };

    auto makeGet = [this, baseUrl, camId](const QString& prop) {
        QUrl url(baseUrl + QString("/api/v1/camera/%1/%2").arg(camId).arg(prop));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_alpacaClientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
        url.setQuery(q);
        return m_networkManager->get(QNetworkRequest(url));
    };

    QNetworkReply *stateReply = makeGet("camerastate");
    QObject::connect(stateReply, &QNetworkReply::finished, stateReply, [stateReply, status, checkDone]() {
        if (stateReply->error() == QNetworkReply::NoError) {
            const QJsonDocument doc = QJsonDocument::fromJson(stateReply->readAll());
            if (doc.isObject()) {
                const QJsonObject root = doc.object();
                if (root.value("ErrorNumber").toInt(0) == 0) {
                    status->cameraState = root.value("Value").toInt(-1);
                }
            }
        }
        stateReply->deleteLater();
        checkDone();
    });

    QNetworkReply *tempReply = makeGet("ccdtemperature");
    QObject::connect(tempReply, &QNetworkReply::finished, tempReply, [tempReply, status, checkDone]() {
        if (tempReply->error() == QNetworkReply::NoError) {
            const QJsonDocument doc = QJsonDocument::fromJson(tempReply->readAll());
            if (doc.isObject()) {
                const QJsonObject root = doc.object();
                if (root.value("ErrorNumber").toInt(0) == 0) {
                    status->ccdTemperature = root.value("Value").toDouble(0.0);
                    status->ccdTemperatureValid = true;
                }
            }
        }
        tempReply->deleteLater();
        checkDone();
    });
}

QString CameraWorker::buildAlpacaBaseUrl() const
{
    return QString("http://%1:%2")
        .arg(m_settings.m_alpacaHost)
        .arg(m_settings.m_alpacaPort);
}

QStringList CameraWorker::parseAlpacaCameraList(const QByteArray& payload) const
{
    QStringList result;
    const QJsonDocument doc = QJsonDocument::fromJson(payload);

    if (!doc.isObject()) {
        return result;
    }

    QJsonArray devices;
    const QJsonObject root = doc.object();

    if (root.contains("Value") && root.value("Value").isArray()) {
        devices = root.value("Value").toArray();
    } else if (root.contains("value") && root.value("value").isArray()) {
        devices = root.value("value").toArray();
    }

    for (const QJsonValue& value : devices)
    {
        if (!value.isObject()) {
            continue;
        }

        const QJsonObject obj = value.toObject();
        const QString type = obj.value("DeviceType").toString().toLower();

        if (type != "camera") {
            continue;
        }

        const int number = obj.value("DeviceNumber").toInt(-1);
        const QString name = obj.value("DeviceName").toString();

        if (number >= 0) {
            result.append(QString("alpaca:%1:%2").arg(number).arg(name));
        }
    }

    return result;
}

QImage CameraWorker::parseAlpacaImageArray(const QByteArray& payload) const
{
    const QJsonDocument doc = QJsonDocument::fromJson(payload);
    if (!doc.isObject()) {
        return createPlaceholderFrame();
    }

    const QJsonObject root = doc.object();

    const int errorNumber = root.value("ErrorNumber").toInt(0);
    if (errorNumber != 0) {
        qDebug() << "CameraWorker::parseAlpacaImageArray: Alpaca error" << errorNumber
                 << root.value("ErrorMessage").toString();
        return createPlaceholderFrame();
    }

    const int rank = root.value("Rank").toInt(0);
    const QJsonArray value = root.value("Value").toArray();

    if (value.isEmpty()) {
        return createPlaceholderFrame();
    }

    // Alpaca imagearray layout:
    //   Rank 2 (monochrome or Bayer): Value[column][row]
    //   Rank 3 (colour):              Value[plane][column][row], plane 0=R, 1=G, 2=B
    if (rank == 2)
    {
        const int width = value.size();
        if (width == 0) {
            return createPlaceholderFrame();
        }
        const QJsonArray firstCol = value[0].toArray();
        const int height = firstCol.size();
        if (height == 0) {
            return createPlaceholderFrame();
        }

        // First pass: find minimum and maximum pixel values for black-level correction and linear scaling to 8-bit
        int minVal = std::numeric_limits<int>::max();
        int maxVal = std::numeric_limits<int>::min();
        for (const QJsonValue& col : value) {
            for (const QJsonValue& pix : col.toArray()) {
                const int v = pix.toInt(0);
                if (v < minVal) { minVal = v; }
                if (v > maxVal) { maxVal = v; }
            }
        }
        const int range = maxVal - minVal;
        // Uniform image: map to mid-gray to avoid division by zero
        const double scale = (range > 0) ? (255.0 / range) : 0.0;
        const int uniformGray = (range == 0) ? (minVal > 0 ? 128 : 0) : 0;

        // Build scaled raw array (column-major), with black-level subtracted
        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            const QJsonArray col = value[x].toArray();
            for (int y = 0; y < height; ++y) {
                raw[x][y] = (range > 0)
                    ? qBound(0, static_cast<int>((col[y].toInt(0) - minVal) * scale), 255)
                    : uniformGray;
            }
        }

        // Bayer demosaicing for sensorType 2 (RGGB), 3 (CMYG), 4 (CMYG2), 5 (LRGB)
        // sensorType 0 = Monochrome, 1 = Colour (handled by rank 3 normally)
        return renderRawPixelArray(raw, width, height);
    }
    else if (rank == 3)
    {
        if (value.size() < 3) {
            return createPlaceholderFrame();
        }
        const QJsonArray planeR = value[0].toArray();
        const QJsonArray planeG = value[1].toArray();
        const QJsonArray planeB = value[2].toArray();

        const int width = planeR.size();
        if (width == 0) {
            return createPlaceholderFrame();
        }
        const int height = planeR[0].toArray().size();
        if (height == 0) {
            return createPlaceholderFrame();
        }

        // First pass: find minimum and maximum pixel values across all planes for black-level correction and linear scaling
        int minVal = std::numeric_limits<int>::max();
        int maxVal = std::numeric_limits<int>::min();
        for (const QJsonArray* plane : {&planeR, &planeG, &planeB}) {
            for (const QJsonValue& col : *plane) {
                for (const QJsonValue& pix : col.toArray()) {
                    const int v = pix.toInt(0);
                    if (v < minVal) { minVal = v; }
                    if (v > maxVal) { maxVal = v; }
                }
            }
        }
        const int range3 = maxVal - minVal;
        // Uniform image: map to mid-gray to avoid division by zero
        const double scale = (range3 > 0) ? (255.0 / range3) : 0.0;
        const int uniformGray3 = (range3 == 0) ? (minVal > 0 ? 128 : 0) : 0;

        QImage image(width, height, QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            const QJsonArray colR = planeR[x].toArray();
            const QJsonArray colG = planeG[x].toArray();
            const QJsonArray colB = planeB[x].toArray();
            for (int y = 0; y < height; ++y) {
                const int r = (range3 > 0) ? qBound(0, static_cast<int>((colR[y].toInt(0) - minVal) * scale), 255) : uniformGray3;
                const int g = (range3 > 0) ? qBound(0, static_cast<int>((colG[y].toInt(0) - minVal) * scale), 255) : uniformGray3;
                const int b = (range3 > 0) ? qBound(0, static_cast<int>((colB[y].toInt(0) - minVal) * scale), 255) : uniformGray3;
                reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
            }
        }
        return image;
    }

    return createPlaceholderFrame();
}

QImage CameraWorker::renderRawPixelArray(const QVector<QVector<int>>& raw, int width, int height) const
{
    // Bayer demosaicing for sensorType 2 (RGGB).
    // RGGB pattern (bayerOffsetX/Y assumed 0):
    //   (even x, even y) = R
    //   (odd  x, even y) = G1
    //   (even x, odd  y) = G2
    //   (odd  x, odd  y) = B
    // For CMYG/CMYG2/LRGB (sensorType 3-5) we fall back to greyscale
    // as those require colour-matrix transforms beyond the scope here.
    if (m_alpacaSensorType == 2)
    {
        QImage image(width, height, QImage::Format_RGB32);
        auto clamp = [](int v) { return qBound(0, v, 255); };
        auto safe  = [&raw, width, height](int x, int y) -> int {
            return raw[qBound(0, x, width-1)][qBound(0, y, height-1)];
        };

        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                int r, g, b;
                if ((x % 2 == 0) && (y % 2 == 0)) {
                    // R site
                    r = raw[x][y];
                    g = clamp((safe(x-1,y) + safe(x+1,y) + safe(x,y-1) + safe(x,y+1)) / 4);
                    b = clamp((safe(x-1,y-1) + safe(x+1,y-1) + safe(x-1,y+1) + safe(x+1,y+1)) / 4);
                } else if ((x % 2 == 1) && (y % 2 == 0)) {
                    // G1 site (R row)
                    r = clamp((safe(x-1,y) + safe(x+1,y)) / 2);
                    g = raw[x][y];
                    b = clamp((safe(x,y-1) + safe(x,y+1)) / 2);
                } else if ((x % 2 == 0) && (y % 2 == 1)) {
                    // G2 site (B row)
                    r = clamp((safe(x,y-1) + safe(x,y+1)) / 2);
                    g = raw[x][y];
                    b = clamp((safe(x-1,y) + safe(x+1,y)) / 2);
                } else {
                    // B site
                    b = raw[x][y];
                    g = clamp((safe(x-1,y) + safe(x+1,y) + safe(x,y-1) + safe(x,y+1)) / 4);
                    r = clamp((safe(x-1,y-1) + safe(x+1,y-1) + safe(x-1,y+1) + safe(x+1,y+1)) / 4);
                }
                reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
            }
        }
        return image;
    }

    // Monochrome (sensorType 0 or 1 returning rank 2, or unsupported Bayer types 3-5)
    QImage image(width, height, QImage::Format_Grayscale8);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            image.scanLine(y)[x] = static_cast<uchar>(raw[x][y]);
        }
    }
    return image;
}

// Alpaca ImageBytes binary format (ASCOM Alpaca spec):
// https://ascom-standards.org/api/?urls.primaryName=ASCOM%20Alpaca%20Device%20API#/Camera/get__device_type___device_number__imagearray
//
//   Byte  0- 3: MetaDataVersion  (int32 LE) — must be 1
//   Byte  4- 7: ErrorNumber      (int32 LE) — 0 = success
//   Byte  8-11: ClientTransactionID (int32 LE)
//   Byte 12-15: ServerTransactionID (int32 LE)
//   Byte 16-19: DataStart        (int32 LE) — byte offset to pixel data, typically 44
//   Byte 20-23: ImageElementType (int32 LE) — original ADU element type (ASCOM ImageArrayElementTypes enum)
//   Byte 24-27: TransmissionElementType (int32 LE) — wire type (same enum)
//   Byte 28-31: Rank             (int32 LE) — 2 or 3
//   Byte 32-35: Dimension1       (int32 LE) — rank2: width; rank3: number of planes
//   Byte 36-39: Dimension2       (int32 LE) — rank2: height; rank3: width
//   Byte 40-43: Dimension3       (int32 LE) — rank2: unused (0); rank3: height
//
// ASCOM ImageArrayElementTypes enum:
//   1=Int16, 2=Int32, 3=Double, 4=Single, 5=UInt64, 6=Byte, 7=Int64, 8=UInt16, 9=UInt32
//
// Pixel data is column-major within each rank-2 plane: pixel[x][y] at index (x*height + y)
// For rank 3: pixel[plane][x][y] at index (plane*width*height + x*height + y)
QImage CameraWorker::parseAlpacaImageBytes(const QByteArray& payload) const
{
    static constexpr int    kHeaderSize         = 44;
    // ASCOM ImageArrayElementTypes enum values
    static constexpr qint32 kElementTypeInt16   = 1;
    static constexpr qint32 kElementTypeInt32   = 2;
    static constexpr qint32 kElementTypeDouble  = 3;
    static constexpr qint32 kElementTypeSingle  = 4;
    static constexpr qint32 kElementTypeByte    = 6;
    static constexpr qint32 kElementTypeUInt16  = 8;

    if (payload.size() < kHeaderSize) {
        qDebug() << "CameraWorker::parseAlpacaImageBytes: payload too small" << payload.size();
        return createPlaceholderFrame();
    }

    const char* hdr = payload.constData();

    const qint32 metadataVersion  = qFromLittleEndian<qint32>(hdr + 0);
    const qint32 errorNumber      = qFromLittleEndian<qint32>(hdr + 4);
    // clientTransactionID        = qFromLittleEndian<qint32>(hdr + 8);  // informational
    // serverTransactionID        = qFromLittleEndian<qint32>(hdr + 12); // informational
    const qint32 dataStart        = qFromLittleEndian<qint32>(hdr + 16);
    // imageElementType           = qFromLittleEndian<qint32>(hdr + 20); // original ADU type
    const qint32 transmissionType = qFromLittleEndian<qint32>(hdr + 24);
    const qint32 rank             = qFromLittleEndian<qint32>(hdr + 28);
    const qint32 dim1             = qFromLittleEndian<qint32>(hdr + 32);
    const qint32 dim2             = qFromLittleEndian<qint32>(hdr + 36);
    const qint32 dim3             = qFromLittleEndian<qint32>(hdr + 40);

    if (metadataVersion != 1) {
        qDebug() << "CameraWorker::parseAlpacaImageBytes: unsupported MetaDataVersion" << metadataVersion;
        return createPlaceholderFrame();
    }

    if (errorNumber != 0) {
        qDebug() << "CameraWorker::parseAlpacaImageBytes: Alpaca error" << errorNumber;
        return createPlaceholderFrame();
    }

    if (dataStart < kHeaderSize || dataStart > payload.size()) {
        qDebug() << "CameraWorker::parseAlpacaImageBytes: invalid DataStart" << dataStart;
        return createPlaceholderFrame();
    }

    int elementSize = 0;
    switch (transmissionType) {
        case kElementTypeInt16:  elementSize = 2; break;
        case kElementTypeInt32:  elementSize = 4; break;
        case kElementTypeDouble: elementSize = 8; break;
        case kElementTypeSingle: elementSize = 4; break;
        case kElementTypeByte:   elementSize = 1; break;
        case kElementTypeUInt16: elementSize = 2; break;
        default:
            qDebug() << "CameraWorker::parseAlpacaImageBytes: unknown TransmissionElementType" << transmissionType;
            return createPlaceholderFrame();
    }

    const char*   pixels         = payload.constData() + dataStart;
    const qsizetype pixelDataLen = payload.size() - dataStart;

    // Read one pixel value as double (any supported element type) at a given byte offset.
    // Using double avoids overflow during the min/max scan and subsequent scaling.
    auto readPixelAsDouble = [&](qsizetype byteOffset) -> double {
        if (byteOffset + elementSize > pixelDataLen) {
            return 0.0;
        }
        const char* p = pixels + byteOffset;
        switch (transmissionType) {
            case kElementTypeInt16:
                return static_cast<double>(qFromLittleEndian<qint16>(p));
            case kElementTypeInt32:
                return static_cast<double>(qFromLittleEndian<qint32>(p));
            case kElementTypeDouble: {
                // Use memcpy + assume little-endian host (x86/x64/ARM — all Qt-supported platforms).
                // Qt does not provide qFromLittleEndian<double> for all versions.
                double v;
                std::memcpy(&v, p, sizeof(v));
                return v;
            }
            case kElementTypeSingle: {
                float v;
                std::memcpy(&v, p, sizeof(v));
                return static_cast<double>(v);
            }
            case kElementTypeByte:
                return static_cast<double>(static_cast<quint8>(*p));
            case kElementTypeUInt16:
                return static_cast<double>(qFromLittleEndian<quint16>(p));
            default:
                return 0.0;
        }
    };

    if (rank == 2)
    {
        const int width  = static_cast<int>(dim1);
        const int height = static_cast<int>(dim2);
        if (width <= 0 || height <= 0) {
            return createPlaceholderFrame();
        }
        const qsizetype required = static_cast<qsizetype>(width) * height * elementSize;
        if (pixelDataLen < required) {
            qDebug() << "CameraWorker::parseAlpacaImageBytes: insufficient pixel data for rank 2:"
                     << pixelDataLen << "<" << required;
            return createPlaceholderFrame();
        }

        // First pass: min/max for black-level correction and linear scaling to 8-bit
        double minVal = std::numeric_limits<double>::max();
        double maxVal = std::numeric_limits<double>::lowest();
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const double v = readPixelAsDouble(static_cast<qsizetype>(x * height + y) * elementSize);
                if (v < minVal) { minVal = v; }
                if (v > maxVal) { maxVal = v; }
            }
        }
        const double range = maxVal - minVal;
        const double scale = (range > 0.0) ? (255.0 / range) : 0.0;
        const int uniformGray = (range == 0.0) ? (minVal > 0.0 ? 128 : 0) : 0;

        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const double v = readPixelAsDouble(static_cast<qsizetype>(x * height + y) * elementSize);
                raw[x][y] = (range > 0.0)
                    ? qBound(0, static_cast<int>((v - minVal) * scale), 255)
                    : uniformGray;
            }
        }

        return renderRawPixelArray(raw, width, height);
    }
    else if (rank == 3)
    {
        const int planes = static_cast<int>(dim1);
        const int width  = static_cast<int>(dim2);
        const int height = static_cast<int>(dim3);
        if (planes < 3 || width <= 0 || height <= 0) {
            return createPlaceholderFrame();
        }
        const qsizetype required = static_cast<qsizetype>(planes) * width * height * elementSize;
        if (pixelDataLen < required) {
            qDebug() << "CameraWorker::parseAlpacaImageBytes: insufficient pixel data for rank 3:"
                     << pixelDataLen << "<" << required;
            return createPlaceholderFrame();
        }

        auto pixelAt = [&](int plane, int x, int y) -> double {
            return readPixelAsDouble(static_cast<qsizetype>(plane * width * height + x * height + y) * elementSize);
        };

        // First pass: min/max across all planes for black-level correction
        double minVal = std::numeric_limits<double>::max();
        double maxVal = std::numeric_limits<double>::lowest();
        for (int p = 0; p < 3; ++p) {
            for (int x = 0; x < width; ++x) {
                for (int y = 0; y < height; ++y) {
                    const double v = pixelAt(p, x, y);
                    if (v < minVal) { minVal = v; }
                    if (v > maxVal) { maxVal = v; }
                }
            }
        }
        const double range3 = maxVal - minVal;
        const double scale = (range3 > 0.0) ? (255.0 / range3) : 0.0;
        const int uniformGray3 = (range3 == 0.0) ? (minVal > 0.0 ? 128 : 0) : 0;

        QImage image(width, height, QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const int r = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(0, x, y) - minVal) * scale), 255)
                    : uniformGray3;
                const int g = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(1, x, y) - minVal) * scale), 255)
                    : uniformGray3;
                const int b = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(2, x, y) - minVal) * scale), 255)
                    : uniformGray3;
                reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
            }
        }
        return image;
    }

    return createPlaceholderFrame();
}

QImage CameraWorker::createPlaceholderFrame() const
{
    QImage image(
        std::max(16, m_settings.m_resolutionWidth),
        std::max(16, m_settings.m_resolutionHeight),
        QImage::Format_RGB32
    );

    image.fill(QColor(32, 32, 32));
    return image;
}

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
void CameraWorker::setupQtCapture()
{
    cleanupQtCapture();

    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    if (cameras.isEmpty()) {
        reportFrameToGUI(createPlaceholderFrame());
        return;
    }

    QCameraDevice selectedDevice = cameras.front();

    for (const QCameraDevice& device : cameras)
    {
        const QString id = QString::fromUtf8(device.id());

        if ((id == m_settings.m_cameraId) || (device.description() == m_settings.m_cameraId)) {
            selectedDevice = device;
            break;
        }
    }

    m_captureSession = new QMediaCaptureSession(this);
    m_qtCamera = new QCamera(selectedDevice, this);
    m_videoSink = new QVideoSink(this);

    QCameraFormat chosenFormat;

    for (const QCameraFormat& format : selectedDevice.videoFormats())
    {
        if ((format.resolution().width() == m_settings.m_resolutionWidth)
                && (format.resolution().height() == m_settings.m_resolutionHeight)
                && (format.maxFrameRate() >= m_settings.m_framesPerSecond))
        {
            chosenFormat = format;
            break;
        }
    }

    if (!chosenFormat.isNull()) {
        m_qtCamera->setCameraFormat(chosenFormat);
    }

    m_qtCamera->setExposureMode(QCamera::ExposureManual);
    m_qtCamera->setManualExposureTime(static_cast<float>(m_settings.m_exposureTimeMs) / 1000.0f);
    m_qtCamera->setManualIsoSensitivity(m_settings.m_isoSensitivity);
    m_qtCamera->setWhiteBalanceMode(static_cast<QCamera::WhiteBalanceMode>(m_settings.m_whiteBalanceMode));
    m_qtCamera->setExposureCompensation(static_cast<float>(m_settings.m_exposureCompensation));
    m_qtCamera->setFocusMode(static_cast<QCamera::FocusMode>(m_settings.m_focusMode));
    m_qtCamera->setFocusDistance(static_cast<float>(m_settings.m_focusDistance));

    m_captureSession->setCamera(m_qtCamera);
    m_captureSession->setVideoOutput(m_videoSink);

    connect(m_videoSink, &QVideoSink::videoFrameChanged, this, &CameraWorker::processQtVideoFrame);

    m_qtCamera->start();

    // Report zoom capabilities after starting so the GUI can configure the zoom control
    const float minZoom = m_qtCamera->minimumZoomFactor();
    const float maxZoom = m_qtCamera->maximumZoomFactor();
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportQtCameraCapabilities::create(minZoom, maxZoom));
    }

    // Apply zoom (clamped to what the hardware supports)
    const float clampedZoom = qBound(minZoom, static_cast<float>(m_settings.m_zoomFactor), maxZoom);
    m_qtCamera->setZoomFactor(clampedZoom);

    // Report available focus modes (Qt 6 only)
    {
        static const QList<int> kFocusModes = {
            static_cast<int>(QCamera::FocusModeAuto),
            static_cast<int>(QCamera::FocusModeAutoNear),
            static_cast<int>(QCamera::FocusModeAutoFar),
            static_cast<int>(QCamera::FocusModeHyperfocal),
            static_cast<int>(QCamera::FocusModeInfinity),
            static_cast<int>(QCamera::FocusModeManual),
        };
        QList<int> supportedModes;
        for (int mode : kFocusModes) {
            if (m_qtCamera->isFocusModeSupported(static_cast<QCamera::FocusMode>(mode))) {
                supportedModes.append(mode);
            }
        }
        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportQtFocusModes::create(supportedModes));
        }
    }
}

void CameraWorker::cleanupQtCapture()
{
    if (m_qtCamera)
    {
        m_qtCamera->stop();
        m_qtCamera->deleteLater();
        m_qtCamera = nullptr;
    }

    if (m_videoSink)
    {
        m_videoSink->deleteLater();
        m_videoSink = nullptr;
    }

    if (m_captureSession)
    {
        m_captureSession->deleteLater();
        m_captureSession = nullptr;
    }
}

void CameraWorker::processQtVideoFrame(const QVideoFrame& frame)
{
    if (!m_capturing) {
        return;
    }

    const QImage image = frame.toImage();

    if (!image.isNull()) {
        processNewFrame(image);
    }
}

#else // Qt 5 implementations

CameraVideoSurface::CameraVideoSurface(CameraWorker *worker, QObject *parent)
    : QAbstractVideoSurface(parent)
    , m_worker(worker)
{
}

QList<QVideoFrame::PixelFormat> CameraVideoSurface::supportedPixelFormats(
    QAbstractVideoBuffer::HandleType handleType) const
{
    Q_UNUSED(handleType);
    return {
        QVideoFrame::Format_ARGB32,
        QVideoFrame::Format_ARGB32_Premultiplied,
        QVideoFrame::Format_RGB32,
        QVideoFrame::Format_RGB24,
        QVideoFrame::Format_RGB565,
        QVideoFrame::Format_RGB555,
        QVideoFrame::Format_BGRA32,
        QVideoFrame::Format_BGR32,
        QVideoFrame::Format_BGR24,
        QVideoFrame::Format_YUV444,
        QVideoFrame::Format_YUV420P,
        QVideoFrame::Format_YV12,
        QVideoFrame::Format_UYVY,
        QVideoFrame::Format_YUYV,
        QVideoFrame::Format_NV12,
        QVideoFrame::Format_NV21,
    };
}

bool CameraVideoSurface::present(const QVideoFrame& frame)
{
    if (!frame.isValid()) {
        return false;
    }

    QVideoFrame mutableFrame(frame);
    mutableFrame.map(QAbstractVideoBuffer::ReadOnly);

    const QImage::Format imageFormat = QVideoFrame::imageFormatFromPixelFormat(mutableFrame.pixelFormat());
    QImage image;

    if (imageFormat != QImage::Format_Invalid)
    {
        // Direct mapping: wrap the frame's data, then deep-copy before unmapping
        image = QImage(
            mutableFrame.bits(),
            mutableFrame.width(),
            mutableFrame.height(),
            mutableFrame.bytesPerLine(),
            imageFormat
        ).copy();
    }

    mutableFrame.unmap();

    if (!image.isNull()) {
        emit frameAvailable(image);
    }

    return true;
}

void CameraWorker::setupQtCapture()
{
    cleanupQtCapture();

    const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();

    if (cameras.isEmpty())
    {
        reportFrameToGUI(createPlaceholderFrame());
        return;
    }

    QCameraInfo selectedInfo = cameras.front();

    for (const QCameraInfo& info : cameras)
    {
        const QString id = info.deviceName();
        if ((id == m_settings.m_cameraId) || (info.description() == m_settings.m_cameraId))
        {
            selectedInfo = info;
            break;
        }
    }

    m_qtCamera = new QCamera(selectedInfo, this);
    m_videoSurface = new CameraVideoSurface(this, this);

    m_qtCamera->setViewfinder(m_videoSurface);

    if (m_settings.m_resolutionWidth > 0 && m_settings.m_resolutionHeight > 0)
    {
        QCameraViewfinderSettings vfSettings;
        vfSettings.setResolution(m_settings.m_resolutionWidth, m_settings.m_resolutionHeight);
        vfSettings.setMaximumFrameRate(m_settings.m_framesPerSecond);
        m_qtCamera->setViewfinderSettings(vfSettings);
    }

    QCameraExposure *exposure = m_qtCamera->exposure();

    if (exposure)
    {
        exposure->setExposureMode(QCameraExposure::ExposureManual);
        exposure->setManualShutterSpeed(static_cast<qreal>(m_settings.m_exposureTimeMs) / 1000.0);
        exposure->setManualIsoSensitivity(m_settings.m_isoSensitivity);
    }

    QCameraImageProcessing *imageProcessing = m_qtCamera->imageProcessing();
    if (imageProcessing) {
        imageProcessing->setWhiteBalanceMode(
            static_cast<QCameraImageProcessing::WhiteBalanceMode>(m_settings.m_whiteBalanceMode));
    }

    // Use a queued connection so that present() (called from the camera thread) safely
    // marshals the QImage across to the CameraWorker thread for processing.
    connect(m_videoSurface, &CameraVideoSurface::frameAvailable,
            this, &CameraWorker::processQt5VideoFrame, Qt::QueuedConnection);

    m_qtCamera->start();

    // Report zoom capabilities so the GUI can configure the zoom control
    {
        QCameraFocus *cameraFocus = m_qtCamera->focus();
        const qreal minZoom = cameraFocus ? cameraFocus->minimumOpticalZoom() : 1.0;
        const qreal maxZoom = cameraFocus ? cameraFocus->maximumOpticalZoom() : 1.0;
        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportQtCameraCapabilities::create(minZoom, maxZoom));
        }
        if (cameraFocus && maxZoom > minZoom) {
            const qreal clampedZoom = qBound(minZoom, static_cast<qreal>(m_settings.m_zoomFactor), maxZoom);
            cameraFocus->setOpticalZoom(clampedZoom);
        }
    }
}

void CameraWorker::cleanupQtCapture()
{
    if (m_qtCamera)
    {
        m_qtCamera->stop();
        m_qtCamera->deleteLater();
        m_qtCamera = nullptr;
    }

    if (m_videoSurface)
    {
        m_videoSurface->deleteLater();
        m_videoSurface = nullptr;
    }
}

void CameraWorker::processQt5VideoFrame(const QImage& image)
{
    if (!m_capturing) {
        return;
    }

    if (!image.isNull()) {
        processNewFrame(image);
    }
}
#endif

void CameraWorker::onCaptureAudioDataReady()
{
    if (m_settings.m_audioMute) {
        m_captureAudioFifo.clear();
        return;
    }

    // Each audio sample frame is 4 bytes: stereo 16-bit PCM (2 channels × 2 bytes)
    static constexpr int kBytesPerSampleFrame = 4;
    unsigned int nbRead;

    while ((nbRead = m_captureAudioFifo.read(m_audioTransferBuffer.data(), m_audioTransferBuffer.size() / kBytesPerSampleFrame)) != 0)
    {
        m_outputAudioFifo.write(m_audioTransferBuffer.data(), nbRead);
    }
}
