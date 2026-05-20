///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
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
#include <functional>
#include <limits>

#include <QDebug>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QSharedPointer>
#include <QUrl>
#include <QUrlQuery>

#include "maincore.h"
#include "util/profiler.h"
#include "camera.h"
#include "cameraalpacacontroller.h"
#include "cameraasicontroller.h"
#include "camerafinder.h"
#include "cameraframepreprocessor.h"
#include "camerapostprocessor.h"
#include "cameraworker.h"

MESSAGE_CLASS_DEFINITION(CameraWorker::MsgConfigureCameraWorker, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgRefreshCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaDeviceList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaCameraInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAsiCameraInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaFilterWheelInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaStatus, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAvailableDevices, Message)

CameraWorker::CameraWorker() :
    m_msgQueueToGUI(nullptr),
    m_msgQueueToFeature(nullptr),
    m_framePreprocessor(nullptr),
    m_postProcessorInputMessageQueue(nullptr),
    m_availableDeviceHandler({}, QStringList{"spectrumview"}),
    m_capturing(false),
    m_captureTimer(this),
    m_networkManager(nullptr),
    m_cameraFinder(new CameraFinder(this)),
    m_stackFrameIndex(0),
    m_hdrExposureIndex(0),
    m_alpaca(),
    m_qtAudio(this),
    m_statusTimer(this),
    m_spectrumPipeSource(nullptr)
#ifdef ASICAMERA_FOUND
    ,
    m_asi()
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

}

CameraWorker::~CameraWorker()
{
    stopWork();
    m_inputMessageQueue.clear();
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
    delete m_networkManager;
    m_networkManager = nullptr;
}

void CameraWorker::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraWorker::handleInputMessages);
    QObject::connect(&m_captureTimer, &QTimer::timeout, this, &CameraWorker::captureTick);
    QObject::connect(&m_statusTimer, &QTimer::timeout, this, &CameraWorker::statusTick);

    if (!m_networkManager) {
        m_networkManager = new QNetworkAccessManager(this);
    }

    // Notify GUI of already-known spectrum-view devices
    reportAvailableDevicesToGUI();

    // Handle any messages already on the queue
    handleInputMessages();
}

void CameraWorker::setMessageQueueToGUI(MessageQueue *messageQueue)
{
    m_msgQueueToGUI = messageQueue;

    if (m_cameraFinder) {
        m_cameraFinder->setMessageQueueToGUI(messageQueue);
    }

    reportAvailableDevicesToGUI();
}

void CameraWorker::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraWorker::handleInputMessages);
    QObject::disconnect(&m_captureTimer, &QTimer::timeout, this, &CameraWorker::captureTick);
    QObject::disconnect(&m_statusTimer, &QTimer::timeout, this, &CameraWorker::statusTick);
    stopCapture();

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpaca.m_connected)
    {
        alpacaSetConnected(false);
    }

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpaca.m_focuserConnected)
    {
        alpacaSetFocuserConnected(false);
    }

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpaca.m_filterWheelConnected)
    {
        alpacaSetFilterWheelConnected(false);
    }

#ifdef ASICAMERA_FOUND
    asiCloseCamera();
#endif

    m_statusTimer.stop();
}

void CameraWorker::resetAlpacaConnectionState()
{
    m_alpaca.resetConnectionState();
}

void CameraWorker::resetAlpacaFilterWheelConnectionState()
{
    m_alpaca.resetFilterWheelConnectionState();
}

void CameraWorker::resetAlpacaFocuserConnectionState()
{
    m_alpaca.resetFocuserConnectionState();
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

    reportAvailableDevicesToGUI();
}

void CameraWorker::reportAvailableDevicesToGUI() const
{
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

void CameraWorker::reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage)
{
    if (!m_msgQueueToFeature || m_reportedFeatureErrorKeys.contains(errorKey)) {
        return;
    }

    m_reportedFeatureErrorKeys.insert(errorKey);
    m_msgQueueToFeature->push(Camera::MsgReportError::create(title, errorMessage));
}

bool CameraWorker::isHdrBracketingActive() const
{
    if (!m_settings.isHdrStackingEnabled()) {
        return false;
    }

    if (m_settings.isAlpacaCamera()) {
        return true;
    }

#ifdef ASICAMERA_FOUND
    if (m_settings.isAsiCamera()) {
        return m_settings.isIntervalCaptureMode();
    }
#endif

    return false;
}

void CameraWorker::resetHdrBracketState()
{
    m_stackFrameIndex = 0;
    m_hdrExposureIndex = 0;

#ifdef ASICAMERA_FOUND
    if (m_settings.isAsiCamera()) {
        m_asi.invalidateSettings();
    }
#endif
}

int CameraWorker::currentStackBurstFrameCount() const
{
    if (isHdrBracketingActive()) {
        return currentHdrExposureCount();
    }

    if (m_settings.m_stackEnabled && (m_settings.m_stackFrameCount > 1)) {
        return qBound(1, m_settings.m_stackFrameCount, 256);
    }

    return 1;
}

int CameraWorker::currentStackBurstIndex() const
{
    if (isHdrBracketingActive()) {
        return currentHdrExposureIndex();
    }

    return qBound(0, m_stackFrameIndex, currentStackBurstFrameCount() - 1);
}

int CameraWorker::currentHdrExposureCount() const
{
    return isHdrBracketingActive() ? m_settings.getHdrExposureCount() : 0;
}

int CameraWorker::currentHdrExposureIndex() const
{
    return isHdrBracketingActive() ? qBound(0, m_hdrExposureIndex, currentHdrExposureCount() - 1) : -1;
}

double CameraWorker::currentCaptureExposureTimeMs() const
{
    const double exposureTimeMs = isHdrBracketingActive()
        ? m_settings.getHdrExposureTimeMs(currentHdrExposureIndex())
        : std::max(CameraSettings::m_minExposureTimeMs, m_settings.m_exposureTimeMs);

    if (m_settings.isAlpacaCamera()) {
        return qBound(m_alpaca.m_exposureMinMs, exposureTimeMs, m_alpaca.m_exposureMaxMs);
    }

    return exposureTimeMs;
}

void CameraWorker::advanceStackBurstState()
{
    if (isHdrBracketingActive())
    {
        advanceHdrBracketState();
        return;
    }

    const int stackFrameCount = currentStackBurstFrameCount();
    m_stackFrameIndex = (m_stackFrameIndex + 1) % std::max(1, stackFrameCount);
}

void CameraWorker::advanceHdrBracketState()
{
    if (!isHdrBracketingActive()) {
        return;
    }

    const int hdrExposureCount = currentHdrExposureCount();
    m_hdrExposureIndex = (m_hdrExposureIndex + 1) % std::max(1, hdrExposureCount);

#ifdef ASICAMERA_FOUND
    if (m_settings.isAsiCamera()) {
        m_asi.invalidateSettings();
    }
#endif
}

bool CameraWorker::useStackIntervalCadence() const
{
    return m_settings.isIntervalCaptureMode() && (currentStackBurstFrameCount() > 1);
}

int CameraWorker::captureTimerIntervalMs() const
{
    return m_settings.isIntervalCaptureMode()
        ? m_settings.getCaptureIntervalMs()
        : std::max(10, static_cast<int>(std::lround(1000.0 / std::max(1, m_settings.m_framesPerSecond))));
}

void CameraWorker::scheduleNextCaptureAfterFrame()
{
    if (!m_capturing || !useStackIntervalCadence()) {
        return;
    }

    if (currentStackBurstIndex() == 0)
    {
        m_captureTimer.start(m_settings.getCaptureIntervalMs());
    }
    else
    {
        QTimer::singleShot(0, this, [this]() {
            if (m_capturing && useStackIntervalCadence()) {
                captureTick();
            }
        });
    }
}

void CameraWorker::scheduleNextCaptureAfterFailure()
{
    if (m_capturing && useStackIntervalCadence()) {
        m_captureTimer.start(m_settings.getCaptureIntervalMs());
    }
}

void CameraWorker::populateFrameExposureMetadata(CameraPipelineFrame& frame) const
{
    frame.m_captureDateTime = QDateTime::currentDateTime();
    frame.m_exposureTimeMs = currentCaptureExposureTimeMs();
    frame.m_hdrExposureIndex = currentHdrExposureIndex();
    frame.m_hdrExposureCount = currentHdrExposureCount();
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
        MsgStartStop& cfg = (MsgStartStop&) cmd;
        m_reportedFeatureErrorKeys.clear();

        if (cfg.getStartStop()) {
            startCapture();
        } else {
            stopCapture();
        }

        return true;
    }
    else if (MsgRefreshCameraList::match(cmd))
    {
        if (m_cameraFinder) {
            m_cameraFinder->reportCameraList(m_settings);
        }

        return true;
    }
    else if (MainCore::MsgImage::match(cmd))
    {
        MainCore::MsgImage& imgMsg = (MainCore::MsgImage&) cmd;
        // Only accept images from the selected device; if none is selected, accept all
        if ((!m_spectrumPipeSource || imgMsg.getPipeSource() == m_spectrumPipeSource) && m_postProcessorInputMessageQueue) {
            m_postProcessorInputMessageQueue->push(CameraPostProcessor::MsgSpectrumFrame::create(imgMsg.getImage()));
        }
        return true;
    }

    return false;
}

void CameraWorker::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraWorker::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    const bool cameraSourceChanged = force
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("cameraId");
    const bool captureModeChanged = force
        || settingsKeys.contains("captureMode");
    const bool hdrSettingsChanged = force
        || settingsKeys.contains("stackEnabled")
        || settingsKeys.contains("stackMethod")
        || settingsKeys.contains("stackHdrAlgorithm")
        || settingsKeys.contains("stackHdrExposureCount")
        || settingsKeys.contains("stackHdrExposure1Ms")
        || settingsKeys.contains("stackHdrExposure2Ms")
        || settingsKeys.contains("stackHdrExposure3Ms")
        || settingsKeys.contains("stackHdrExposure4Ms");
    const bool stackCadenceChanged = hdrSettingsChanged
        || settingsKeys.contains("stackFrameCount");
    const bool captureCadenceChanged = force
        || settingsKeys.contains("captureInterval")
        || settingsKeys.contains("captureIntervalUnits")
        || settingsKeys.contains("framesPerSecond")
        || stackCadenceChanged;
    const bool recapture = m_capturing && (
        cameraSourceChanged
        || (m_settings.isQtCamera() && (force || settingsKeys.contains("audioDeviceName")))
        || (m_settings.isAsiCamera() && captureModeChanged));
    const bool alpacaEndpointChanged = force
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("alpacaHost")
        || settingsKeys.contains("alpacaPort")
        || settingsKeys.contains("cameraId");
    const bool disconnectPreviousAlpaca = m_settings.isAlpacaCamera()
        && m_networkManager
        && m_alpaca.m_connected
        && alpacaEndpointChanged;

    if (recapture)
    {
        stopCapture();
#ifdef ASICAMERA_FOUND
        if (cameraSourceChanged && m_settings.isAsiCamera()) {
            asiCloseCamera();
        }
#endif
    }

    if (disconnectPreviousAlpaca) {
        alpacaSetConnected(false);
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (force || settingsKeys.contains("audioMute")) {
        m_qtAudio.setMuted(m_settings.m_audioMute);
    }

    if (hdrSettingsChanged) {
        resetHdrBracketState();
    }
    if (stackCadenceChanged) {
        m_stackFrameIndex = 0;
    }

    if (force
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("cameraDescription")
        || settingsKeys.contains("yoloLabelsPath"))
    {
        m_reportedFeatureErrorKeys.clear();
    }

    if ((force
            || settingsKeys.contains("alpacaDiscoveryEnabled")
            || settingsKeys.contains("alpacaHost")
            || settingsKeys.contains("alpacaPort"))
        && m_cameraFinder)
    {
        m_cameraFinder->reportCameraList(m_settings);
    }

    if (!recapture && m_capturing && (m_settings.isAlpacaCamera() || m_settings.isAsiCamera()) && captureCadenceChanged)
    {
        if (m_settings.isAsiCamera() && !m_settings.isIntervalCaptureMode())
        {
            m_captureTimer.stop();
#ifdef ASICAMERA_FOUND
            scheduleNextAsiVideoCapture();
#endif
        }
        else
        {
            m_captureTimer.start(captureTimerIntervalMs());
        }
    }

    const bool alpacaFocuserEndpointChanged = force
        || settingsKeys.contains("alpacaFocuserEnabled")
        || settingsKeys.contains("alpacaFocuserHost")
        || settingsKeys.contains("alpacaFocuserPort")
        || settingsKeys.contains("alpacaFocuserDeviceNumber");
    const bool alpacaFocuserPositionChanged = force
        || settingsKeys.contains("alpacaFocusPosition");
    const bool alpacaFilterWheelEndpointChanged = force
        || settingsKeys.contains("alpacaFilterWheelEnabled")
        || settingsKeys.contains("alpacaFilterWheelHost")
        || settingsKeys.contains("alpacaFilterWheelPort")
        || settingsKeys.contains("alpacaFilterWheelDeviceNumber");
    const bool alpacaFilterWheelPositionChanged = force
        || settingsKeys.contains("alpacaFilterWheelPosition");

    if (alpacaEndpointChanged) {
        resetAlpacaConnectionState();
    }

    if (alpacaFilterWheelEndpointChanged) {
        resetAlpacaFilterWheelConnectionState();
    }

    if (alpacaFocuserEndpointChanged) {
        resetAlpacaFocuserConnectionState();
    }

    if (m_settings.isAlpacaCamera()
        && m_networkManager
        && alpacaEndpointChanged)
    {
        alpacaBootstrap();
    }

    if (m_settings.isAlpacaCamera()
        && m_networkManager
        && m_settings.m_alpacaFocuserEnabled
        && (alpacaFocuserEndpointChanged || alpacaFocuserPositionChanged))
    {
        if (settingsKeys.contains("alpacaFocusPosition")) {
            alpacaSetFocuserPosition();
        }
    }

    if (m_settings.isAlpacaCamera()
        && m_networkManager
        && m_settings.m_alpacaFilterWheelEnabled
        && (alpacaFilterWheelEndpointChanged || alpacaFilterWheelPositionChanged))
    {
        if (alpacaFilterWheelEndpointChanged) {
            alpacaQueryFilterWheelInfo();
        } else if (settingsKeys.contains("alpacaFilterWheelPosition")) {
            alpacaSetFilterWheelPosition();
        }
    }

    if (force || settingsKeys.contains("cameraProtocol") || settingsKeys.contains("cameraId"))
    {
        if (m_settings.isAlpacaCamera() || m_settings.isAsiCamera()) {
            if (!m_statusTimer.isActive()) {
                m_statusTimer.start(m_alpacaStatusPollIntervalMs);
            }
        } else {
            m_statusTimer.stop();
        }
    }

#ifdef ASICAMERA_FOUND
    if (m_settings.isAsiCamera()
        && (force
            || settingsKeys.contains("cameraProtocol")
            || settingsKeys.contains("cameraId")
            || settingsKeys.contains("cameraBinX")
            || settingsKeys.contains("cameraBinY")
            || settingsKeys.contains("cameraNumX")
            || settingsKeys.contains("cameraNumY")
            || settingsKeys.contains("cameraStartX")
            || settingsKeys.contains("cameraStartY")
            || settingsKeys.contains("cameraGain")
            || settingsKeys.contains("cameraOffset")
            || settingsKeys.contains("asiCoolerOn")
            || settingsKeys.contains("asiTargetTemp")
            || settingsKeys.contains("asiUsbBandwidth")
            || settingsKeys.contains("asiHighSpeedMode")
            || settingsKeys.contains("asiAutoExposureGain")
            || settingsKeys.contains("asiColorImageType")
            || settingsKeys.contains("exposureTimeMs")))
    {
        invalidateAsiSettings();
        asiQueryCameraCapabilities();
    }
    else if (force || settingsKeys.contains("cameraProtocol") || settingsKeys.contains("cameraId"))
    {
        invalidateAsiSettings();
        asiCloseCamera();
    }
#endif

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
        if (m_postProcessorInputMessageQueue) {
            m_postProcessorInputMessageQueue->push(CameraPostProcessor::MsgSpectrumFrame::create(QImage()));
        }
    }

    if (recapture) {
        startCapture();
    }
}

void CameraWorker::startCapture()
{
    if (m_capturing) {
        return;
    }

    resetHdrBracketState();
    m_capturing = true;
    m_alpaca.resetCaptureState();
    m_alpaca.m_lastBinX = m_settings.m_cameraBinX;
    m_alpaca.m_lastBinY = m_settings.m_cameraBinY;
    m_alpaca.m_lastNumX = m_settings.m_cameraNumX;
    m_alpaca.m_lastNumY = m_settings.m_cameraNumY;
    m_alpaca.m_lastEffectiveNumX = -1;
    m_alpaca.m_lastEffectiveNumY = -1;
    m_alpaca.m_lastStartX = m_settings.m_cameraStartX;
    m_alpaca.m_lastStartY = m_settings.m_cameraStartY;
    m_alpaca.m_lastGain = m_settings.m_cameraGain;
    m_alpaca.m_lastOffset = m_settings.m_cameraOffset;
    m_alpaca.m_lastReadoutMode = m_settings.m_cameraReadoutMode;

    if (m_settings.isAlpacaCamera())
    {
        m_alpaca.m_captureTimer.start();
        m_alpaca.m_frameRequestPending = false;
        m_captureTimer.start(captureTimerIntervalMs());

        if (m_alpaca.m_connected && !m_alpaca.m_bootstrapPending && (m_alpaca.m_cameraSizeX > 0) && (m_alpaca.m_cameraSizeY > 0)) {
            captureTick();
        } else {
            alpacaBootstrap();
        }
    }
#ifdef ASICAMERA_FOUND
    else if (m_settings.isAsiCamera())
    {
        invalidateAsiSettings();
        if (m_settings.isIntervalCaptureMode())
        {
            m_captureTimer.start(captureTimerIntervalMs());
            captureTick();
        }
        else
        {
            m_captureTimer.stop();
            scheduleNextAsiVideoCapture();
        }
    }
#endif
    else if (m_settings.isQtCamera())
    {
        // Qt camera capture is mainly managed by CameraGUI on the main thread. The worker only bridges audio.
        m_qtAudio.start(m_settings, getInputMessageQueue());
    }
}

void CameraWorker::stopCapture()
{
    m_capturing = false;
    m_captureTimer.stop();
    m_alpaca.m_captureTimer.invalidate();
    resetHdrBracketState();

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpaca.m_frameRequestPending) {
        alpacaAbortExposure();
    }

#ifdef ASICAMERA_FOUND
    m_asi.cancelContinuousCapture();
    m_asi.stopVideoCapture(m_settings.cameraIdInt());
    if (m_settings.isAsiCamera() && m_settings.isIntervalCaptureMode()) {
        m_asi.stopExposure(m_settings.cameraIdInt());
    }
    m_asi.invalidateSettings();
#endif

    m_qtAudio.stop();
}

void CameraWorker::captureTick()
{
    if (!m_capturing) {
        return;
    }

#ifdef ASICAMERA_FOUND
    if (m_settings.isAsiCamera())
    {
        if (useStackIntervalCadence()) {
            m_captureTimer.stop();
        }
        asiCaptureTick();
        return;
    }
#endif

    if (!m_networkManager || m_alpaca.m_frameRequestPending) {
        return;
    }

    if (!m_alpaca.m_connected || m_alpaca.m_connectionPending || m_alpaca.m_bootstrapPending)
    {
        alpacaBootstrap();
        return;
    }

    if (!m_alpaca.m_captureTimer.isValid()) {
        m_alpaca.m_captureTimer.start();
    }
    if (useStackIntervalCadence()) {
        m_captureTimer.stop();
    }
    m_alpaca.m_frameRequestPending = true;
    alpacaSetCameraParams();
}

// Helper: PUT a simple integer property on the Alpaca camera asynchronously,
// then invoke the continuation lambda only when the write succeeds.
static void alpacaPutIntProperty(
    QNetworkAccessManager *nam,
    const QString& baseUrl,
    int cameraId,
    const QString& property,
    const QString& bodyKey,
    int value,
    quint32 clientId,
    quint32& transactionId,
    bool logApi,
    std::function<void()> continuation,
    std::function<void()> onSuccess = {},
    std::function<void(int, const QString&)> onFailure = {})
{
    QUrl url(baseUrl + QString("/api/v1/camera/%1/%2").arg(cameraId).arg(property));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem(bodyKey, QString::number(value));
    body.addQueryItem("ClientID", QString::number(clientId));
    body.addQueryItem("ClientTransactionID", QString::number(transactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    if (logApi) {
        qDebug() << "CameraWorker::AlpacaAPI request" << "PUT" << url.toString() << payload;
    }
    QNetworkReply *reply = nam->put(request, payload);

    QObject::connect(reply, &QNetworkReply::finished, reply, [reply, property, continuation, onSuccess, onFailure, logApi, url]() {
        const QByteArray responseBody = reply->readAll();
        if (logApi) {
            qDebug() << "CameraWorker::AlpacaAPI response" << "PUT" << url.toString()
                     << "error" << reply->error() << reply->errorString() << responseBody;
        }
        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        const bool alpacaPayloadParsed = CameraAlpacaController::parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage);
        bool success = (reply->error() == QNetworkReply::NoError);

        if (success && alpacaPayloadParsed) {
            success = (alpacaErrorNumber == 0);
        }

        if (success)
        {
            if (onSuccess) {
                onSuccess();
            }
            reply->deleteLater();
            continuation();
            return;
        }

        if (reply->error() != QNetworkReply::NoError)
        {
            qDebug() << "CameraWorker: PUT" << property << "error:" << reply->errorString();
            if (onFailure) {
                onFailure(static_cast<int>(reply->error()), reply->errorString());
            }
        }
        else
        {
            qDebug() << "CameraWorker: PUT" << property << "Alpaca error:"
                     << alpacaErrorNumber << alpacaErrorMessage;
            if (onFailure) {
                onFailure(alpacaErrorNumber, alpacaErrorMessage);
            }
        }
        reply->deleteLater();
    });
}

static bool isAlpacaDriverError(int errorNumber)
{
    return errorNumber >= 1024;
}

void CameraWorker::alpacaSetConnected(bool connected, std::function<void()> continuation)
{
    if (!m_networkManager) {
        return;
    }

    const QString baseUrl = CameraAlpacaController::baseUrl(m_settings);
    const int camId = m_settings.cameraIdInt();
    QUrl url(baseUrl + QString("/api/v1/camera/%1/connected").arg(camId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? QStringLiteral("true") : QStringLiteral("false"));
    body.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logAlpacaRequest("PUT", url, payload);

    QNetworkReply *reply = m_networkManager->put(request, payload);

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply, connected, continuation]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);

        bool success = false;

        if (reply->error() == QNetworkReply::NoError)
        {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);

            if (doc.isObject()) {
                success = (doc.object().value("ErrorNumber").toInt(-1) == 0);
            }
        }

        if (success)
        {
            m_alpaca.m_connected = connected;
        }
        else if (connected)
        {
            if (reply->error() != QNetworkReply::NoError)
            {
                m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
                m_alpaca.m_lastErrorMessage = reply->errorString();
            }
            m_alpaca.m_connected = false;
            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
        }
        else
        {
            if (reply->error() != QNetworkReply::NoError)
            {
                m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
                m_alpaca.m_lastErrorMessage = reply->errorString();
            }
            reportAlpacaStatusToGUI();
        }

        if (!connected) {
            m_alpaca.m_connectionPending = false;
            m_alpaca.m_connected = false;
            m_alpaca.m_pendingConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_alpaca.m_connectionPending = false;
            m_alpaca.m_pendingConnectedContinuations.clear();
        }

        reply->deleteLater();
    });
}

void CameraWorker::alpacaRunWhenConnected(std::function<void()> continuation)
{
    if (m_alpaca.m_connected)
    {
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_alpaca.m_pendingConnectedContinuations.append(continuation);
    }

    if (m_alpaca.m_connectionPending) {
        return;
    }

    m_alpaca.m_connectionPending = true;
    alpacaSetConnected(true, [this]() {
        m_alpaca.m_connectionPending = false;
        const auto continuations = std::move(m_alpaca.m_pendingConnectedContinuations);
        m_alpaca.m_pendingConnectedContinuations.clear();

        for (const auto& continuation : continuations)
        {
            if (continuation) {
                continuation();
            }
        }
    });
}

void CameraWorker::alpacaSetFocuserConnected(bool connected, std::function<void()> continuation)
{
    if (!m_networkManager || !m_settings.isAlpacaCamera() || !m_settings.m_alpacaFocuserEnabled) {
        return;
    }

    const QString baseUrl = CameraAlpacaController::focuserBaseUrl(m_settings);
    const int deviceNumber = std::max(0, m_settings.m_alpacaFocuserDeviceNumber);
    QUrl url(baseUrl + QString("/api/v1/focuser/%1/connected").arg(deviceNumber));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? "true" : "false");
    body.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logAlpacaRequest("PUT", url, payload);

    QNetworkReply *reply = m_networkManager->put(request, payload);

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply, connected, continuation]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);

        bool success = false;

        if (reply->error() == QNetworkReply::NoError)
        {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);

            if (doc.isObject()) {
                success = (doc.object().value("ErrorNumber").toInt(-1) == 0);
            }
        }

        if (success) {
            m_alpaca.m_focuserConnected = connected;
        } else if (connected) {
            if (reply->error() != QNetworkReply::NoError)
            {
                m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
                m_alpaca.m_lastErrorMessage = reply->errorString();
            }
            m_alpaca.m_focuserConnected = false;
            reportAlpacaStatusToGUI();
        } else {
            if (reply->error() != QNetworkReply::NoError)
            {
                m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
                m_alpaca.m_lastErrorMessage = reply->errorString();
            }
            reportAlpacaStatusToGUI();
        }

        if (!connected) {
            m_alpaca.m_focuserConnectionPending = false;
            m_alpaca.m_focuserConnected = false;
            m_alpaca.m_pendingFocuserConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_alpaca.m_focuserConnectionPending = false;
            m_alpaca.m_pendingFocuserConnectedContinuations.clear();
        }

        reply->deleteLater();
    });
}

void CameraWorker::alpacaRunFocuserWhenConnected(std::function<void()> continuation)
{
    if (m_alpaca.m_focuserConnected)
    {
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_alpaca.m_pendingFocuserConnectedContinuations.append(continuation);
    }

    if (m_alpaca.m_focuserConnectionPending) {
        return;
    }

    m_alpaca.m_focuserConnectionPending = true;
    alpacaSetFocuserConnected(true, [this]() {
        m_alpaca.m_focuserConnectionPending = false;
        const auto continuations = std::move(m_alpaca.m_pendingFocuserConnectedContinuations);
        m_alpaca.m_pendingFocuserConnectedContinuations.clear();

        for (const auto& continuation : continuations)
        {
            if (continuation) {
                continuation();
            }
        }
    });
}

void CameraWorker::alpacaSetFocuserPosition()
{
    if (!m_networkManager || !m_settings.isAlpacaCamera() || !m_settings.m_alpacaFocuserEnabled) {
        return;
    }

    alpacaRunFocuserWhenConnected([this]() {
        const QString baseUrl = CameraAlpacaController::focuserBaseUrl(m_settings);
        const int deviceNumber = std::max(0, m_settings.m_alpacaFocuserDeviceNumber);
        QUrl url(baseUrl + QString("/api/v1/focuser/%1/move").arg(deviceNumber));
        QNetworkRequest request(url);
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

        QUrlQuery body;
        body.addQueryItem("Position", QString::number(std::max(0, m_settings.m_alpacaFocusPosition)));
        body.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
        body.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));

        const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
        logAlpacaRequest("PUT", url, payload);

        QNetworkReply *reply = m_networkManager->put(request, payload);
        QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply]() {
            const QByteArray responseBody = reply->readAll();
            logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);
            int alpacaErrorNumber = 0;
            QString alpacaErrorMessage;
            const bool alpacaPayloadParsed = CameraAlpacaController::parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage);
            const bool success = (reply->error() == QNetworkReply::NoError)
                && (!alpacaPayloadParsed || (alpacaErrorNumber == 0));

            if (!success)
            {
                if (reply->error() != QNetworkReply::NoError)
                {
                    m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
                    m_alpaca.m_lastErrorMessage = reply->errorString();
                }
                else
                {
                    m_alpaca.m_lastErrorNumber = alpacaErrorNumber;
                    m_alpaca.m_lastErrorMessage = alpacaErrorMessage;
                }

                reportAlpacaStatusToGUI();
            }
            reply->deleteLater();
        });
    });
}

void CameraWorker::alpacaSetFilterWheelConnected(bool connected, std::function<void()> continuation)
{
    if (!m_networkManager) {
        return;
    }

    const QString baseUrl = CameraAlpacaController::filterWheelBaseUrl(m_settings);
    const int deviceNumber = std::max(0, m_settings.m_alpacaFilterWheelDeviceNumber);
    QUrl url(baseUrl + QString("/api/v1/filterwheel/%1/connected").arg(deviceNumber));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? QStringLiteral("true") : QStringLiteral("false"));
    body.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logAlpacaRequest("PUT", url, payload);

    QNetworkReply *reply = m_networkManager->put(request, payload);

    QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply, connected, continuation]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);

        bool success = false;

        if (reply->error() == QNetworkReply::NoError)
        {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);

            if (doc.isObject()) {
                success = (doc.object().value("ErrorNumber").toInt(-1) == 0);
            }
        }

        if (success) {
            m_alpaca.m_filterWheelConnected = connected;
        } else if (connected) {
            if (reply->error() != QNetworkReply::NoError)
            {
                m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
                m_alpaca.m_lastErrorMessage = reply->errorString();
            }
            m_alpaca.m_filterWheelConnected = false;
            reportAlpacaStatusToGUI();
        } else {
            if (reply->error() != QNetworkReply::NoError)
            {
                m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
                m_alpaca.m_lastErrorMessage = reply->errorString();
            }
            reportAlpacaStatusToGUI();
        }

        if (!connected) {
            m_alpaca.m_filterWheelConnectionPending = false;
            m_alpaca.m_filterWheelConnected = false;
            m_alpaca.m_pendingFilterWheelConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_alpaca.m_filterWheelConnectionPending = false;
            m_alpaca.m_pendingFilterWheelConnectedContinuations.clear();
        }

        reply->deleteLater();
    });
}

void CameraWorker::alpacaRunFilterWheelWhenConnected(std::function<void()> continuation)
{
    if (m_alpaca.m_filterWheelConnected)
    {
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_alpaca.m_pendingFilterWheelConnectedContinuations.append(continuation);
    }

    if (m_alpaca.m_filterWheelConnectionPending) {
        return;
    }

    m_alpaca.m_filterWheelConnectionPending = true;
    alpacaSetFilterWheelConnected(true, [this]() {
        m_alpaca.m_filterWheelConnectionPending = false;
        const auto continuations = std::move(m_alpaca.m_pendingFilterWheelConnectedContinuations);
        m_alpaca.m_pendingFilterWheelConnectedContinuations.clear();

        for (const auto& continuation : continuations)
        {
            if (continuation) {
                continuation();
            }
        }
    });
}

void CameraWorker::alpacaQueryFilterWheelInfo()
{
    if (!m_networkManager || !m_settings.isAlpacaCamera() || !m_settings.m_alpacaFilterWheelEnabled) {
        return;
    }

    alpacaRunFilterWheelWhenConnected([this]() {
        const QString baseUrl = CameraAlpacaController::filterWheelBaseUrl(m_settings);
        const int deviceNumber = std::max(0, m_settings.m_alpacaFilterWheelDeviceNumber);

        struct FilterWheelInfo {
            QStringList names;
            int position = -1;
            int pending = 0;
        };

        auto info = QSharedPointer<FilterWheelInfo>::create();
        info->pending = 2;

        auto checkDone = [this, info]() {
            info->pending--;
            if (info->pending > 0) {
                return;
            }

            if (m_msgQueueToGUI) {
                m_msgQueueToGUI->push(MsgReportAlpacaFilterWheelInfo::create(info->names, info->position));
            }
        };

        auto makeGet = [this, baseUrl, deviceNumber](const QString& prop) {
            QUrl url(baseUrl + QString("/api/v1/filterwheel/%1/%2").arg(deviceNumber).arg(prop));
            QUrlQuery q;
            q.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
            q.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));
            url.setQuery(q);
            logAlpacaRequest("GET", url);
            return m_networkManager->get(QNetworkRequest(url));
        };

        QNetworkReply *namesReply = makeGet("names");
        QObject::connect(namesReply, &QNetworkReply::finished, namesReply, [this, namesReply, info, checkDone]() {
            const QByteArray responseBody = namesReply->readAll();
            logAlpacaResponse("GET", namesReply->request().url(), namesReply, responseBody);
            if (namesReply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    if (root.value("ErrorNumber").toInt(0) == 0) {
                        const QJsonValue namesValue = root.contains(QStringLiteral("Value"))
                            ? root.value(QStringLiteral("Value"))
                            : root.value(QStringLiteral("value"));

                        if (namesValue.isArray())
                        {
                            const QJsonArray values = namesValue.toArray();

                            for (const QJsonValue& value : values)
                            {
                                if (value.isString()) {
                                    info->names.append(value.toString());
                                } else if (value.isObject()) {
                                    const QJsonObject obj = value.toObject();
                                    info->names.append(obj.value(QStringLiteral("Name")).toString(
                                        obj.value(QStringLiteral("name")).toString()));
                                }
                            }
                        }
                        else if (namesValue.isString())
                        {
                            info->names.append(namesValue.toString());
                        }
                    }
                }
            }
            namesReply->deleteLater();
            checkDone();
        });

        QNetworkReply *positionReply = makeGet("position");
        QObject::connect(positionReply, &QNetworkReply::finished, positionReply, [this, positionReply, info, checkDone]() {
            const QByteArray responseBody = positionReply->readAll();
            logAlpacaResponse("GET", positionReply->request().url(), positionReply, responseBody);
            if (positionReply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    if (root.value("ErrorNumber").toInt(0) == 0) {
                        const QJsonValue positionValue = root.contains(QStringLiteral("Value"))
                            ? root.value(QStringLiteral("Value"))
                            : root.value(QStringLiteral("value"));
                        info->position = std::max(0, positionValue.toInt(0));
                    }
                }
            }
            positionReply->deleteLater();
            checkDone();
        });
    });
}

void CameraWorker::alpacaQueryFilterWheelPosition(std::function<void(int)> continuation)
{
    if (!m_networkManager || !m_settings.isAlpacaCamera() || !m_settings.m_alpacaFilterWheelEnabled) {
        if (continuation) {
            continuation(-1);
        }
        return;
    }

    alpacaRunFilterWheelWhenConnected([this, continuation]() {
        const QString baseUrl = CameraAlpacaController::filterWheelBaseUrl(m_settings);
        const int deviceNumber = std::max(0, m_settings.m_alpacaFilterWheelDeviceNumber);
        QUrl url(baseUrl + QString("/api/v1/filterwheel/%1/position").arg(deviceNumber));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));
        url.setQuery(q);
        logAlpacaRequest("GET", url);

        QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));
        QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply, continuation]() {
            int position = -1;
            const QByteArray responseBody = reply->readAll();
            logAlpacaResponse("GET", reply->request().url(), reply, responseBody);
            if (reply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    if (root.value("ErrorNumber").toInt(0) == 0) {
                        const QJsonValue positionValue = root.contains(QStringLiteral("Value"))
                            ? root.value(QStringLiteral("Value"))
                            : root.value(QStringLiteral("value"));
                        position = positionValue.toInt(-1);
                    }
                }
            }
            reply->deleteLater();
            if (continuation) {
                continuation(position);
            }
        });
    });
}

void CameraWorker::alpacaWaitForFilterWheelPosition(int retriesRemaining)
{
    if (retriesRemaining <= 0 || !m_networkManager || !m_settings.isAlpacaCamera() || !m_settings.m_alpacaFilterWheelEnabled) {
        return;
    }

    alpacaQueryFilterWheelPosition([this, retriesRemaining](int position) {
        if (position >= 0)
        {
            if (m_msgQueueToGUI) {
                m_msgQueueToGUI->push(MsgReportAlpacaFilterWheelInfo::create(QStringList(), position));
            }
            return;
        }

        QTimer::singleShot(250, this, [this, retriesRemaining]() {
            alpacaWaitForFilterWheelPosition(retriesRemaining - 1);
        });
    });
}

void CameraWorker::alpacaSetFilterWheelPosition()
{
    if (!m_networkManager || !m_settings.isAlpacaCamera() || !m_settings.m_alpacaFilterWheelEnabled) {
        return;
    }

    alpacaRunFilterWheelWhenConnected([this]() {
        const QString baseUrl = CameraAlpacaController::filterWheelBaseUrl(m_settings);
        const int deviceNumber = std::max(0, m_settings.m_alpacaFilterWheelDeviceNumber);
        QUrl url(baseUrl + QString("/api/v1/filterwheel/%1/position").arg(deviceNumber));
        QNetworkRequest request(url);
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

        QUrlQuery body;
        body.addQueryItem("Position", QString::number(std::max(0, m_settings.m_alpacaFilterWheelPosition)));
        body.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
        body.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));

        const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
        logAlpacaRequest("PUT", url, payload);

        QNetworkReply *reply = m_networkManager->put(request, payload);
        QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply]() {
            const QByteArray responseBody = reply->readAll();
            logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);
            int alpacaErrorNumber = 0;
            QString alpacaErrorMessage;
            const bool alpacaPayloadParsed = CameraAlpacaController::parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage);
            const bool success = (reply->error() == QNetworkReply::NoError)
                && (!alpacaPayloadParsed || (alpacaErrorNumber == 0));

            if (success)
            {
                alpacaWaitForFilterWheelPosition(20);
            }
            else
            {
                if (reply->error() != QNetworkReply::NoError)
                {
                    m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
                    m_alpaca.m_lastErrorMessage = reply->errorString();
                }
                else
                {
                    m_alpaca.m_lastErrorNumber = alpacaErrorNumber;
                    m_alpaca.m_lastErrorMessage = alpacaErrorMessage;
                }

                reportAlpacaStatusToGUI();
            }
            reply->deleteLater();
        });
    });
}

void CameraWorker::alpacaBootstrap(std::function<void()> continuation)
{
    if (!m_networkManager) {
        return;
    }

    if (continuation) {
        m_alpaca.m_pendingBootstrapContinuations.append(continuation);
    }

    if (m_alpaca.m_bootstrapPending) {
        return;
    }

    m_alpaca.m_bootstrapPending = true;

    alpacaRunWhenConnected([this]() {
        if (!m_statusTimer.isActive()) {
            m_statusTimer.start(m_alpacaStatusPollIntervalMs);
        }

        alpacaQueryCameraCapabilities([this]() {
            m_alpaca.m_bootstrapPending = false;
            alpacaPollStatus();

            const auto continuations = std::move(m_alpaca.m_pendingBootstrapContinuations);
            m_alpaca.m_pendingBootstrapContinuations.clear();

            for (const auto& continuation : continuations)
            {
                if (continuation) {
                    continuation();
                }
            }

            if (m_capturing && !m_alpaca.m_frameRequestPending) {
                captureTick();
            }
        });
    });
}

void CameraWorker::alpacaSetCameraParams()
{
    // Chain: binX -> binY -> subframe ROI -> gain -> offset -> readoutMode -> startExposure
    const QString baseUrl = CameraAlpacaController::baseUrl(m_settings);
    const int camId = m_settings.cameraIdInt();
    const bool forceAllParams = !m_alpaca.m_paramsInitialized;
    const int maxSubframeX = std::max(1, m_alpaca.m_cameraSizeX / std::max(1, m_settings.m_cameraBinX));
    const int maxSubframeY = std::max(1, m_alpaca.m_cameraSizeY / std::max(1, m_settings.m_cameraBinY));
    const bool fullFrameNumXRequested = (m_settings.m_cameraNumX == 0);
    const bool fullFrameNumYRequested = (m_settings.m_cameraNumY == 0);
    const bool canResolveNumX = !fullFrameNumXRequested || (m_alpaca.m_cameraSizeX > 0);
    const bool canResolveNumY = !fullFrameNumYRequested || (m_alpaca.m_cameraSizeY > 0);
    const int effectiveNumX = fullFrameNumXRequested
        ? std::max(1, maxSubframeX - std::max(0, m_settings.m_cameraStartX))
        : m_settings.m_cameraNumX;
    const int effectiveNumY = fullFrameNumYRequested
        ? std::max(1, maxSubframeY - std::max(0, m_settings.m_cameraStartY))
        : m_settings.m_cameraNumY;
    const bool setBinX = forceAllParams || (m_alpaca.m_lastBinX != m_settings.m_cameraBinX);
    const bool setBinY = forceAllParams || (m_alpaca.m_lastBinY != m_settings.m_cameraBinY);
    const bool setNumX = canResolveNumX
        && (forceAllParams || (m_alpaca.m_lastEffectiveNumX != effectiveNumX));
    const bool setNumY = canResolveNumY
        && (forceAllParams || (m_alpaca.m_lastEffectiveNumY != effectiveNumY));
    const bool setStartX = forceAllParams || (m_alpaca.m_lastStartX != m_settings.m_cameraStartX);
    const bool setStartY = forceAllParams || (m_alpaca.m_lastStartY != m_settings.m_cameraStartY);
    const bool setGain = (m_settings.m_cameraGain >= 0)
        && (forceAllParams || (m_alpaca.m_lastGain != m_settings.m_cameraGain));
    const bool setOffset = (m_settings.m_cameraOffset >= 0)
        && (forceAllParams || (m_alpaca.m_lastOffset != m_settings.m_cameraOffset));
    const bool setReadoutMode = forceAllParams || (m_alpaca.m_lastReadoutMode != m_settings.m_cameraReadoutMode);

    auto doStartExposure = [this]() {
        if (m_capturing) {
            m_alpaca.m_paramsInitialized = true;
            alpacaStartExposure();
        } else {
            m_alpaca.m_frameRequestPending = false;
        }
    };

    auto handleParamFailure = [this](int errorNumber, const QString& errorMessage) {
        m_alpaca.m_lastErrorNumber = errorNumber;
        m_alpaca.m_lastErrorMessage = errorMessage;
        m_alpaca.m_frameRequestPending = false;
        reportAlpacaStatusToGUI();
        scheduleNextCaptureAfterFailure();
    };

    auto handleOptionalParamFailure = [this, handleParamFailure](const QString& property, int errorNumber, const QString& errorMessage,
            const std::function<void()>& markAttempted, const std::function<void()>& continuation) {
        if (isAlpacaDriverError(errorNumber))
        {
            qDebug() << "CameraWorker:" << property << "Alpaca error is non-fatal for exposure:"
                     << errorNumber << errorMessage;
            m_alpaca.m_lastErrorNumber = errorNumber;
            m_alpaca.m_lastErrorMessage = errorMessage;
            if (markAttempted) {
                markAttempted();
            }
            reportAlpacaStatusToGUI();
            if (continuation) {
                continuation();
            }
            return;
        }

        handleParamFailure(errorNumber, errorMessage);
    };

    auto doReadoutMode = [this, baseUrl, camId, doStartExposure, setReadoutMode, handleOptionalParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
        if (setReadoutMode) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "readoutmode", "ReadoutMode",
                m_settings.m_cameraReadoutMode, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doStartExposure,
                [this]() { m_alpaca.m_lastReadoutMode = m_settings.m_cameraReadoutMode; },
                [this, handleOptionalParamFailure, doStartExposure](int errorNumber, const QString& errorMessage) {
                    handleOptionalParamFailure(
                        QStringLiteral("readoutmode"),
                        errorNumber,
                        errorMessage,
                        [this]() { m_alpaca.m_lastReadoutMode = m_settings.m_cameraReadoutMode; },
                        doStartExposure);
                });
        } else {
            doStartExposure();
        }
    };

    auto doOffset = [this, baseUrl, camId, doReadoutMode, setOffset, handleOptionalParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
        if (setOffset) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "offset", "Offset",
                m_settings.m_cameraOffset, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doReadoutMode,
                [this]() { m_alpaca.m_lastOffset = m_settings.m_cameraOffset; },
                [this, handleOptionalParamFailure, doReadoutMode](int errorNumber, const QString& errorMessage) {
                    handleOptionalParamFailure(
                        QStringLiteral("offset"),
                        errorNumber,
                        errorMessage,
                        [this]() { m_alpaca.m_lastOffset = m_settings.m_cameraOffset; },
                        doReadoutMode);
                });
        } else {
            doReadoutMode();
        }
    };

    auto doGain = [this, baseUrl, camId, doOffset, setGain, handleOptionalParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
        if (setGain) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "gain", "Gain",
                m_settings.m_cameraGain, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doOffset,
                [this]() { m_alpaca.m_lastGain = m_settings.m_cameraGain; },
                [this, handleOptionalParamFailure, doOffset](int errorNumber, const QString& errorMessage) {
                    handleOptionalParamFailure(
                        QStringLiteral("gain"),
                        errorNumber,
                        errorMessage,
                        [this]() { m_alpaca.m_lastGain = m_settings.m_cameraGain; },
                        doOffset);
                });
        } else {
            doOffset();
        }
    };

    auto doAxisY = [this, baseUrl, camId, doGain, setNumY, setStartY, effectiveNumY, handleParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }

        std::function<void()> maybeSetStartYAfterNum = [this, baseUrl, camId, doGain, setStartY, handleParamFailure]() {
            if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
            if (setStartY) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "starty", "StartY",
                    m_settings.m_cameraStartY, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doGain,
                    [this]() { m_alpaca.m_lastStartY = m_settings.m_cameraStartY; },
                    handleParamFailure);
            } else {
                doGain();
            }
        };

        std::function<void()> maybeSetNumYAfterStart = [this, baseUrl, camId, doGain, setNumY, effectiveNumY, handleParamFailure]() {
            if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
            if (setNumY) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numy", "NumY",
                    effectiveNumY, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doGain,
                    [this, effectiveNumY]() {
                        m_alpaca.m_lastNumY = m_settings.m_cameraNumY;
                        m_alpaca.m_lastEffectiveNumY = effectiveNumY;
                    },
                    handleParamFailure);
            } else {
                doGain();
            }
        };

        if (setStartY && (m_settings.m_cameraStartY < m_alpaca.m_lastStartY))
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "starty", "StartY",
                m_settings.m_cameraStartY, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, maybeSetNumYAfterStart,
                [this]() { m_alpaca.m_lastStartY = m_settings.m_cameraStartY; },
                handleParamFailure);
        }
        else if (setNumY)
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numy", "NumY",
                effectiveNumY, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, maybeSetStartYAfterNum,
                [this, effectiveNumY]() {
                    m_alpaca.m_lastNumY = m_settings.m_cameraNumY;
                    m_alpaca.m_lastEffectiveNumY = effectiveNumY;
                },
                handleParamFailure);
        }
        else
        {
            maybeSetStartYAfterNum();
        }
    };

    auto doAxisX = [this, baseUrl, camId, doAxisY, setNumX, setStartX, effectiveNumX, handleParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }

        std::function<void()> maybeSetStartXAfterNum = [this, baseUrl, camId, doAxisY, setStartX, handleParamFailure]() {
            if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
            if (setStartX) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "startx", "StartX",
                    m_settings.m_cameraStartX, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doAxisY,
                    [this]() { m_alpaca.m_lastStartX = m_settings.m_cameraStartX; },
                    handleParamFailure);
            } else {
                doAxisY();
            }
        };

        std::function<void()> maybeSetNumXAfterStart = [this, baseUrl, camId, doAxisY, setNumX, effectiveNumX, handleParamFailure]() {
            if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
            if (setNumX) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numx", "NumX",
                    effectiveNumX, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doAxisY,
                    [this, effectiveNumX]() {
                        m_alpaca.m_lastNumX = m_settings.m_cameraNumX;
                        m_alpaca.m_lastEffectiveNumX = effectiveNumX;
                    },
                    handleParamFailure);
            } else {
                doAxisY();
            }
        };

        if (setStartX && (m_settings.m_cameraStartX < m_alpaca.m_lastStartX))
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "startx", "StartX",
                m_settings.m_cameraStartX, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, maybeSetNumXAfterStart,
                [this]() { m_alpaca.m_lastStartX = m_settings.m_cameraStartX; },
                handleParamFailure);
        }
        else if (setNumX)
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numx", "NumX",
                effectiveNumX, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, maybeSetStartXAfterNum,
                [this, effectiveNumX]() {
                    m_alpaca.m_lastNumX = m_settings.m_cameraNumX;
                    m_alpaca.m_lastEffectiveNumX = effectiveNumX;
                },
                handleParamFailure);
        }
        else
        {
            maybeSetStartXAfterNum();
        }
    };

    auto doBinY = [this, baseUrl, camId, doAxisX, setBinY, handleParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
        if (setBinY) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "biny", "BinY",
                m_settings.m_cameraBinY, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doAxisX,
                [this]() { m_alpaca.m_lastBinY = m_settings.m_cameraBinY; },
                handleParamFailure);
        } else {
            doAxisX();
        }
    };

    if (setBinX) {
        alpacaPutIntProperty(m_networkManager, baseUrl, camId, "binx", "BinX",
            m_settings.m_cameraBinX, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doBinY,
            [this]() { m_alpaca.m_lastBinX = m_settings.m_cameraBinX; },
            handleParamFailure);
    } else {
        doBinY();
    }
}

void CameraWorker::alpacaStartExposure()
{
    QUrl url(CameraAlpacaController::baseUrl(m_settings) + QString("/api/v1/camera/%1/startexposure").arg(m_settings.cameraIdInt()));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    const double exposureTimeMs = currentCaptureExposureTimeMs();
    const double durationSecs = exposureTimeMs / 1000.0;
    QUrlQuery body;
    body.addQueryItem("Duration", QString::number(durationSecs, 'f', 6)); // 6 needed for microsecond precision
    body.addQueryItem("Light", "True");
    body.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logAlpacaRequest("PUT", url, payload);
    QNetworkReply *reply = m_networkManager->put(request, payload);

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply, exposureTimeMs]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);
        reply->deleteLater();

        if (!m_capturing) {
            m_alpaca.m_frameRequestPending = false;
            return;
        }

        if (reply->error() != QNetworkReply::NoError) {
            qDebug() << "CameraWorker::alpacaStartExposure: error:" << reply->errorString();
            m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
            m_alpaca.m_lastErrorMessage = reply->errorString();
            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
            return;
        }

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        if (CameraAlpacaController::parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage)
            && (alpacaErrorNumber != 0))
        {
            m_alpaca.m_lastErrorNumber = alpacaErrorNumber;
            m_alpaca.m_lastErrorMessage = alpacaErrorMessage;
            qDebug() << "CameraWorker::alpacaStartExposure: Alpaca error"
                     << m_alpaca.m_lastErrorNumber << m_alpaca.m_lastErrorMessage;
            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
            return;
        }

        m_alpaca.m_exposureSeenActive = false;

        // Wait for the exposure duration before polling imageready
        QTimer::singleShot(static_cast<int>(std::ceil(exposureTimeMs)), this, [this]() {
            if (m_capturing) {
                alpacaCheckImageReady();
            } else {
                m_alpaca.m_frameRequestPending = false;
            }
        });
    });
}

void CameraWorker::alpacaAbortExposure()
{
    QUrl url(CameraAlpacaController::baseUrl(m_settings) + QString("/api/v1/camera/%1/abortexposure").arg(m_settings.cameraIdInt()));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logAlpacaRequest("PUT", url, payload);
    QNetworkReply *reply = m_networkManager->put(request, payload);

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);
        reply->deleteLater();

        if (reply->error() != QNetworkReply::NoError)
        {
            qDebug() << "CameraWorker::alpacaAbortExposure: error:" << reply->errorString();
            return;
        }

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        if (CameraAlpacaController::parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage)
            && (alpacaErrorNumber != 0))
        {
            qDebug() << "CameraWorker::alpacaAbortExposure: Alpaca error"
                     << alpacaErrorNumber << alpacaErrorMessage;
        }
    });
}

void CameraWorker::alpacaCheckImageReady()
{
    QUrl url(CameraAlpacaController::baseUrl(m_settings) + QString("/api/v1/camera/%1/imageready").arg(m_settings.cameraIdInt()));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));
    url.setQuery(query);

    logAlpacaRequest("GET", url);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("GET", reply->request().url(), reply, responseBody);
        reply->deleteLater();

        if (!m_capturing) {
            m_alpaca.m_frameRequestPending = false;
            return;
        }

        if (reply->error() != QNetworkReply::NoError)
        {
            m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
            m_alpaca.m_lastErrorMessage = reply->errorString();
            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
            return;
        }

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        if (CameraAlpacaController::parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage)
            && (alpacaErrorNumber != 0))
        {
            m_alpaca.m_lastErrorNumber = alpacaErrorNumber;
            m_alpaca.m_lastErrorMessage = alpacaErrorMessage;
            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
            return;
        }

        bool ready = false;
        const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
        if (doc.isObject()) {
            ready = doc.object().value("Value").toBool(false);
        }

        if (ready) {
            alpacaFetchImageArray();
        } else {
            // Some Alpaca devices keep ImageReady false after exposure; CameraState gives us a fallback.
            alpacaCheckCameraStateForImageReady();
        }
    });
}

void CameraWorker::alpacaCheckCameraStateForImageReady()
{
    QUrl url(CameraAlpacaController::baseUrl(m_settings) + QString("/api/v1/camera/%1/camerastate").arg(m_settings.cameraIdInt()));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));
    url.setQuery(query);

    logAlpacaRequest("GET", url);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("GET", reply->request().url(), reply, responseBody);
        reply->deleteLater();

        if (!m_capturing) {
            m_alpaca.m_frameRequestPending = false;
            return;
        }

        if (reply->error() != QNetworkReply::NoError)
        {
            m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
            m_alpaca.m_lastErrorMessage = reply->errorString();
            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
            return;
        }

        int alpacaErrorNumber = 0;
        QString alpacaErrorMessage;
        if (CameraAlpacaController::parseErrorPayload(responseBody, alpacaErrorNumber, alpacaErrorMessage)
            && (alpacaErrorNumber != 0))
        {
            m_alpaca.m_lastErrorNumber = alpacaErrorNumber;
            m_alpaca.m_lastErrorMessage = alpacaErrorMessage;
            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
            return;
        }

        int cameraState = -1;
        const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
        if (doc.isObject()) {
            cameraState = doc.object().value("Value").toInt(-1);
        }

        if (cameraState > 0) {
            m_alpaca.m_exposureSeenActive = true;
        }

        if ((cameraState == 0) && m_alpaca.m_exposureSeenActive) {
            alpacaFetchImageArray();
        } else {
            QTimer::singleShot(m_alpacaImageReadyPollIntervalMs, this, [this]() {
                if (m_capturing) {
                    alpacaCheckImageReady();
                } else {
                    m_alpaca.m_frameRequestPending = false;
                }
            });
        }
    });
}

void CameraWorker::alpacaFetchImageArray()
{
    QUrl url(CameraAlpacaController::baseUrl(m_settings) + QString("/api/v1/camera/%1/imagearray").arg(m_settings.cameraIdInt()));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));
    url.setQuery(query);

    QNetworkRequest request(url);
    // Signal support for the faster binary ImageBytes protocol; server falls back to JSON if unsupported
    if (m_alpaca.m_imageBytesSupported) {
        request.setRawHeader("Accept", "application/imagebytes");
    }

    logAlpacaRequest("GET", url);
    QNetworkReply *reply = m_networkManager->get(request);

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        m_alpaca.m_frameRequestPending = false;
        const QByteArray data = reply->readAll();
        logAlpacaResponse("GET", reply->request().url(), reply, data);

        if (!m_capturing) {
            reply->deleteLater();
            return;
        }

        if (reply->error() != QNetworkReply::NoError)
        {
            m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
            m_alpaca.m_lastErrorMessage = reply->errorString();
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
            reply->deleteLater();
            return;
        }

        QImage image = createPlaceholderFrame();
        CameraPipelineFrame::BayerPattern bayerPattern = CameraPipelineFrame::BayerNone;
        QString receiveImageFormat;

        const QString contentType = reply->header(QNetworkRequest::ContentTypeHeader).toString();
        if (contentType.contains(QLatin1String("application/imagebytes"), Qt::CaseInsensitive))
        {
            int imageBytesErrorNumber = 0;
            if (CameraAlpacaController::parseImageBytesError(data, imageBytesErrorNumber) && (imageBytesErrorNumber != 0))
            {
                m_alpaca.m_lastErrorNumber = imageBytesErrorNumber;
                m_alpaca.m_lastErrorMessage = QStringLiteral("ImageBytes Alpaca error %1").arg(imageBytesErrorNumber);
                reportAlpacaStatusToGUI();
                scheduleNextCaptureAfterFailure();
                reply->deleteLater();
                return;
            }

            m_alpaca.m_imageBytesSupported = true;
            image = m_alpaca.parseImageBytes(data, createPlaceholderFrame(), &receiveImageFormat, &bayerPattern);
        }
        else
        {
            int alpacaErrorNumber = 0;
            QString alpacaErrorMessage;
            if (CameraAlpacaController::parseErrorPayload(data, alpacaErrorNumber, alpacaErrorMessage)
                && (alpacaErrorNumber != 0))
            {
                m_alpaca.m_lastErrorNumber = alpacaErrorNumber;
                m_alpaca.m_lastErrorMessage = alpacaErrorMessage;
                reportAlpacaStatusToGUI();
                scheduleNextCaptureAfterFailure();
                reply->deleteLater();
                return;
            }

            // Server returned JSON - either it doesn't support ImageBytes or we didn't request it
            if (m_alpaca.m_imageBytesSupported) {
                qDebug() << "CameraWorker::alpacaFetchImageArray: server returned JSON; disabling ImageBytes for this camera";
                m_alpaca.m_imageBytesSupported = false;
            }
            image = m_alpaca.parseImageArray(data, createPlaceholderFrame(), &receiveImageFormat, &bayerPattern);
        }

        m_alpaca.m_lastReceiveImageFormat = receiveImageFormat;

        if (m_alpaca.m_captureTimer.isValid())
        {
            m_alpaca.m_lastCaptureTimeMs = m_alpaca.m_captureTimer.elapsed();
            m_alpaca.m_captureTimer.invalidate();
        }

        if (m_framePreprocessor) {
            CameraPipelineFramePtr frame(new CameraPipelineFrame);
            frame->m_image = image;
            populateFrameExposureMetadata(*frame);
            frame->m_bayerPattern = bayerPattern;
            m_framePreprocessor->submitFrame(frame);
        }
        advanceStackBurstState();
        scheduleNextCaptureAfterFrame();
        reply->deleteLater();
    });
}

void CameraWorker::alpacaQueryCameraCapabilities(std::function<void()> continuation)
{
    if (!m_networkManager) {
        return;
    }

    // Reset ImageBytes support flag so we re-probe support for the new camera;
    // cameras on the same Alpaca server may have different capabilities.
    m_alpaca.m_imageBytesSupported = true;

    const QString baseUrl = CameraAlpacaController::baseUrl(m_settings);
    const int camId = m_settings.cameraIdInt();

    // Struct to accumulate results from parallel requests
    struct CapInfo {
        QString name;
        QString description;
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
        int bayerOffsetX = 0;
        int bayerOffsetY = 0;
        double pixelSizeX = 0.0;
        double pixelSizeY = 0.0;
        int cameraSizeX = 0;
        int cameraSizeY = 0;
        double ccdTemperature = 0.0;
        bool ccdTemperatureValid = false;
        double exposureMinMs = 1.0;
        double exposureMaxMs = 60000.0;
        double exposureResolutionMs = 1.0;
        int pending = 0;
    };

    auto info = QSharedPointer<CapInfo>::create();

    // Properties to query: name, JSON Value key, handler
    static const QStringList properties = {
        "name", "description",
        "maxbinx", "maxbiny", "gains", "gainmin", "gainmax",
        "offsets", "offsetmin", "offsetmax",
        "readoutmodes", "sensorname", "sensortype", "bayeroffsetx", "bayeroffsety",
        "pixelsizex", "pixelsizey", "cameraxsize", "cameraysize",
        "ccdtemperature", "exposuremin", "exposuremax", "exposureresolution"
    };

    info->pending = properties.size();

    auto checkDone = [this, info, continuation]() {
        info->pending--;
        if (info->pending > 0) {
            return;
        }

        m_alpaca.m_sensorType = info->sensorType;
        m_alpaca.m_cameraSizeX = std::max(0, info->cameraSizeX);
        m_alpaca.m_cameraSizeY = std::max(0, info->cameraSizeY);
        m_alpaca.m_bayerOffsetX = info->bayerOffsetX;
        m_alpaca.m_bayerOffsetY = info->bayerOffsetY;
        info->exposureMinMs = std::max(0.001, info->exposureMinMs);
        info->exposureResolutionMs = std::max(0.001, info->exposureResolutionMs);
        info->exposureMaxMs = std::max(info->exposureMinMs, info->exposureMaxMs);
        m_alpaca.m_exposureMinMs = info->exposureMinMs;
        m_alpaca.m_exposureMaxMs = info->exposureMaxMs;

        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportAlpacaCameraInfo::create(
                info->name, info->description,
                info->maxBinX, info->maxBinY,
                info->gains, info->gainMin, info->gainMax,
                info->offsets, info->offsetMin, info->offsetMax,
                info->readoutModes,
                info->sensorName, info->sensorType,
                info->pixelSizeX, info->pixelSizeY,
                info->cameraSizeX, info->cameraSizeY,
                info->ccdTemperature, info->ccdTemperatureValid,
                info->exposureMinMs, info->exposureMaxMs, info->exposureResolutionMs));
        }

        if (continuation) {
            continuation();
        }
    };

    auto query = [this, baseUrl, camId, info, checkDone](const QString& prop) {
        QUrl url(baseUrl + QString("/api/v1/camera/%1/%2").arg(camId).arg(prop));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));
        url.setQuery(q);
        logAlpacaRequest("GET", url);

        QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

        QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply, prop, info, checkDone]() {
            const QByteArray responseBody = reply->readAll();
            logAlpacaResponse("GET", reply->request().url(), reply, responseBody);
            if (reply->error() == QNetworkReply::NoError) {
                const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
                if (doc.isObject()) {
                    const QJsonObject root = doc.object();
                    const int errNum = root.value("ErrorNumber").toInt(0);
                    if (errNum == 0) {
                        const QJsonValue val = root.value("Value");
                        if (prop == "name") {
                            info->name = val.toString();
                        } else if (prop == "description") {
                            info->description = val.toString();
                        } else if (prop == "maxbinx") {
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
                        } else if (prop == "bayeroffsetx") {
                            info->bayerOffsetX = val.toInt(0);
                        } else if (prop == "bayeroffsety") {
                            info->bayerOffsetY = val.toInt(0);
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
                        } else if (prop == "exposuremin") {
                            info->exposureMinMs = std::max(0.001, val.toDouble(0.0) * 1000.0);
                        } else if (prop == "exposuremax") {
                            info->exposureMaxMs = std::max(info->exposureMinMs, val.toDouble(0.0) * 1000.0);
                        } else if (prop == "exposureresolution") {
                            info->exposureResolutionMs = std::max(0.001, val.toDouble(0.0) * 1000.0);
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
#ifdef ASICAMERA_FOUND
    else if (m_settings.isAsiCamera()) {
        asiPollStatus();
    }
#endif
}

void CameraWorker::alpacaPollStatus()
{
    const QString baseUrl = CameraAlpacaController::baseUrl(m_settings);
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
        reportAlpacaStatusToGUI(status->cameraState, status->ccdTemperature, status->ccdTemperatureValid);
    };

    auto makeGet = [this, baseUrl, camId](const QString& prop) {
        QUrl url(baseUrl + QString("/api/v1/camera/%1/%2").arg(camId).arg(prop));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_alpaca.m_clientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_alpaca.m_clientTransactionId++));
        url.setQuery(q);
        logAlpacaRequest("GET", url);
        return m_networkManager->get(QNetworkRequest(url));
    };

    QNetworkReply *stateReply = makeGet("camerastate");
    QObject::connect(stateReply, &QNetworkReply::finished, stateReply, [this, stateReply, status, checkDone]() {
        const QByteArray responseBody = stateReply->readAll();
        logAlpacaResponse("GET", stateReply->request().url(), stateReply, responseBody);
        if (stateReply->error() == QNetworkReply::NoError) {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
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
    QObject::connect(tempReply, &QNetworkReply::finished, tempReply, [this, tempReply, status, checkDone]() {
        const QByteArray responseBody = tempReply->readAll();
        logAlpacaResponse("GET", tempReply->request().url(), tempReply, responseBody);
        if (tempReply->error() == QNetworkReply::NoError) {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
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

void CameraWorker::logAlpacaRequest(const QString& method, const QUrl& url, const QByteArray& payload) const
{
    if (!m_settings.m_alpacaApiLogEnabled) {
        return;
    }

    if (payload.isEmpty()) {
        qDebug() << "CameraWorker::AlpacaAPI request" << method << url.toString();
    } else {
        qDebug() << "CameraWorker::AlpacaAPI request" << method << url.toString() << payload;
    }
}

void CameraWorker::logAlpacaResponse(const QString& method, const QUrl& url, QNetworkReply *reply, const QByteArray& payload)
{
    const QString path = url.path();

    int alpacaErrorNumber = 0;
    QString alpacaErrorMessage;
    bool alpacaPayloadParsed = false;

    alpacaPayloadParsed = CameraAlpacaController::parseErrorPayload(payload, alpacaErrorNumber, alpacaErrorMessage);
    const bool optionalCapabilityUnavailable = alpacaPayloadParsed
        && (alpacaErrorNumber != 0)
        && CameraAlpacaController::isOptionalCapabilityPath(path);

    if (reply->error() != QNetworkReply::NoError)
    {
        m_alpaca.m_lastErrorNumber = static_cast<int>(reply->error());
        m_alpaca.m_lastErrorMessage = reply->errorString();
    }
    else if (alpacaPayloadParsed && !optionalCapabilityUnavailable)
    {
        m_alpaca.m_lastErrorNumber = alpacaErrorNumber;
        m_alpaca.m_lastErrorMessage = alpacaErrorMessage;
    }

    if (!m_settings.m_alpacaApiLogEnabled) {
        return;
    }

    if (path.endsWith(QStringLiteral("/imagearray"), Qt::CaseInsensitive))
    {
        qDebug() << "CameraWorker::AlpacaAPI response" << method << url.toString()
                 << CameraAlpacaController::transportError(reply)
                 << "alpacaError" << alpacaErrorNumber << alpacaErrorMessage
                 << QString("<imagearray payload of %1 bytes omitted>").arg(payload.size());
        return;
    }

    if (alpacaPayloadParsed)
    {
        if (optionalCapabilityUnavailable)
        {
            qDebug() << "CameraWorker::AlpacaAPI optional capability unavailable" << method << url.toString()
                     << "alpacaError" << alpacaErrorNumber << alpacaErrorMessage;
        }
        else
        {
            qDebug() << "CameraWorker::AlpacaAPI response" << method << url.toString()
                     << CameraAlpacaController::transportError(reply)
                     << "alpacaError" << alpacaErrorNumber << alpacaErrorMessage
                     << payload;
        }
    }
    else
    {
        qDebug() << "CameraWorker::AlpacaAPI response" << method << url.toString()
                 << CameraAlpacaController::transportError(reply)
                 << payload;
    }
}

void CameraWorker::reportAlpacaStatusToGUI(int cameraState, double ccdTemperature, bool ccdTemperatureValid)
{
    if (!m_msgQueueToGUI) {
        return;
    }

    m_msgQueueToGUI->push(MsgReportAlpacaStatus::create(
        cameraState,
        ccdTemperature,
        ccdTemperatureValid,
        m_alpaca.m_lastCaptureTimeMs,
        m_alpaca.m_lastReceiveImageFormat,
        m_alpaca.m_lastErrorNumber,
        m_alpaca.m_lastErrorMessage));
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

#ifdef ASICAMERA_FOUND

bool CameraWorker::asiOpenCamera()
{
    const int cameraId = m_settings.cameraIdInt();

    if (m_asi.openCamera(cameraId)) {
        return true;
    }

    switch (m_asi.lastOpenFailureStage())
    {
    case CameraAsiController::OpenFailureOpen:
        reportErrorToFeature(
            QStringLiteral("asiOpen:%1").arg(cameraId),
            tr("ASI camera open failed"),
            tr("Failed to open ASI camera %1:\n%2").arg(cameraId).arg(m_asi.lastErrorMessage()));
        break;
    case CameraAsiController::OpenFailureInit:
        reportErrorToFeature(
            QStringLiteral("asiInit:%1").arg(cameraId),
            tr("ASI camera initialization failed"),
            tr("Failed to initialize ASI camera %1:\n%2").arg(cameraId).arg(m_asi.lastErrorMessage()));
        break;
    case CameraAsiController::OpenFailureMode:
        reportErrorToFeature(
            QStringLiteral("asiMode:%1").arg(cameraId),
            tr("ASI camera mode setup failed"),
            tr("Failed to set ASI camera %1 to normal mode:\n%2").arg(cameraId).arg(m_asi.lastErrorMessage()));
        break;
    case CameraAsiController::OpenFailureNone:
        break;
    }

    return false;
}

void CameraWorker::asiCloseCamera()
{
    m_asi.closeCamera(m_settings.cameraIdInt());
}

void CameraWorker::asiQueryCameraCapabilities()
{
    if (!m_settings.isAsiCamera()) {
        return;
    }

    asiCloseCamera();

    const int cameraId = m_settings.cameraIdInt();
    if ((cameraId < 0) || !asiOpenCamera()) {
        return;
    }

    CameraAsiController::CapabilitiesReport report;
    if (!m_asi.queryCameraCapabilities(cameraId, m_settings, report)) {
        return;
    }

    if (m_msgQueueToGUI)
    {
        m_msgQueueToGUI->push(MsgReportAsiCameraInfo::create(
            report.m_name,
            report.m_maxBinX,
            report.m_maxBinY,
            report.m_gainMin,
            report.m_gainMax,
            report.m_offsetMin,
            report.m_offsetMax,
            report.m_cameraSizeX,
            report.m_cameraSizeY,
            report.m_pixelSizeUm,
            report.m_bitDepth,
            report.m_colorCamera,
            report.m_exposureMinMs,
            report.m_exposureMaxMs,
            report.m_coolerSupported,
            report.m_coolerOn,
            report.m_targetTempSupported,
            report.m_targetTempMin,
            report.m_targetTempMax,
            report.m_targetTemp,
            report.m_usbBandwidthSupported,
            report.m_usbBandwidthMin,
            report.m_usbBandwidthMax,
            report.m_usbBandwidth,
            report.m_highSpeedModeSupported,
            report.m_highSpeedMode,
            report.m_rgb24Supported,
            report.m_raw16Supported,
            report.m_raw8Supported));
    }

    asiPollStatus();
}

bool CameraWorker::asiApplyCameraSettings()
{
    if (!m_asi.hasCameraSize()) {
        asiQueryCameraCapabilities();
    }

    if (!asiOpenCamera()) {
        return false;
    }

    return m_asi.applyCameraSettings(m_settings.cameraIdInt(), m_settings, currentCaptureExposureTimeMs());
}
bool CameraWorker::asiCaptureExposureFrame()
{
    const int cameraId = m_settings.cameraIdInt();
    const CameraAsiController::CaptureResult result = m_asi.captureExposureFrame(cameraId, currentCaptureExposureTimeMs());

    if (result == CameraAsiController::CaptureStartFailed)
    {
        reportErrorToFeature(
            QStringLiteral("asiStartExposure:%1").arg(cameraId),
            tr("ASI exposure start failed"),
            tr("Failed to start ASI exposure on camera %1:\n%2").arg(cameraId).arg(m_asi.lastErrorMessage()));
        return false;
    }

    if (result != CameraAsiController::CaptureSuccess) {
        return false;
    }

    if (m_framePreprocessor) {
        CameraPipelineFrame::BayerPattern bayerPattern = CameraPipelineFrame::BayerNone;
        CameraPipelineFramePtr frame(new CameraPipelineFrame);
        frame->m_image = m_asi.frameToImage(createPlaceholderFrame(), &bayerPattern);
        populateFrameExposureMetadata(*frame);
        frame->m_bayerPattern = bayerPattern;
        m_framePreprocessor->submitFrame(frame);
    }
    advanceStackBurstState();
    scheduleNextCaptureAfterFrame();
    return true;
}

bool CameraWorker::useAsiContinuousVideoCadence() const
{
    return m_capturing && m_settings.isAsiCamera() && !m_settings.isIntervalCaptureMode();
}

void CameraWorker::scheduleNextAsiVideoCapture(int delayMs)
{
    if (!useAsiContinuousVideoCadence() || m_asi.continuousCaptureScheduled()) {
        return;
    }

    m_asi.markContinuousCaptureScheduled();
    const quint64 generation = m_asi.continuousCaptureGeneration();
    QTimer::singleShot(std::max(0, delayMs), this, [this, generation]() {
        if (!m_asi.clearContinuousCaptureScheduled(generation)) {
            return;
        }
        if (useAsiContinuousVideoCadence()) {
            captureTick();
        }
    });
}

void CameraWorker::asiCaptureVideoFrame()
{
    PROFILER_START();

    const int cameraId = m_settings.cameraIdInt();
    const int waitMs = std::max(1000, static_cast<int>(std::ceil(currentCaptureExposureTimeMs())) + 500);
    const CameraAsiController::CaptureResult result = m_asi.captureVideoFrame(cameraId, waitMs);

    if (result == CameraAsiController::CaptureStartFailed)
    {
        reportErrorToFeature(
            QStringLiteral("asiStartVideo:%1").arg(cameraId),
            tr("ASI video capture start failed"),
            tr("Failed to start ASI video capture on camera %1:\n%2").arg(cameraId).arg(m_asi.lastErrorMessage()));
        scheduleNextAsiVideoCapture(100);
        PROFILER_STOP(__FUNCTION__);
        return;
    }

    if (result == CameraAsiController::CaptureSuccess)
    {
        if (m_framePreprocessor) {
            CameraPipelineFrame::BayerPattern bayerPattern = CameraPipelineFrame::BayerNone;
            CameraPipelineFramePtr frame(new CameraPipelineFrame);
            frame->m_image = m_asi.frameToImage(createPlaceholderFrame(), &bayerPattern);
            populateFrameExposureMetadata(*frame);
            frame->m_bayerPattern = bayerPattern;
            m_framePreprocessor->submitFrame(frame);
        }
    }

    scheduleNextAsiVideoCapture(result == CameraAsiController::CaptureSuccess ? 0 : 10);

    PROFILER_STOP(__FUNCTION__);
}

void CameraWorker::asiCaptureTick()
{
    if (!m_capturing) {
        return;
    }

    if (!m_asi.settingsApplied() && !asiApplyCameraSettings())
    {
        scheduleNextAsiVideoCapture(100);
        return;
    }

    if (m_settings.isIntervalCaptureMode()) {
        if (!asiCaptureExposureFrame()) {
            scheduleNextCaptureAfterFailure();
        }
    } else {
        asiCaptureVideoFrame();
    }
}

void CameraWorker::invalidateAsiSettings()
{
    m_asi.invalidateSettings();
}

void CameraWorker::asiPollStatus()
{
    if (!m_settings.isAsiCamera()) {
        return;
    }

    const int cameraId = m_settings.cameraIdInt();
    CameraAsiController::StatusReport report = m_asi.statusReport();

    if ((cameraId >= 0) && asiOpenCamera()) {
        report = m_asi.pollStatus(cameraId);
    }

    if (m_msgQueueToGUI)
    {
        m_msgQueueToGUI->push(MsgReportAlpacaStatus::create(
            m_capturing ? 1 : 0,
            report.m_ccdTemperature,
            report.m_ccdTemperatureValid,
            report.m_lastCaptureTimeMs,
            report.m_imageTypeName,
            report.m_lastErrorNumber,
            report.m_lastErrorMessage));
    }
}

#endif
