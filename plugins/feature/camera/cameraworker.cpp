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
#include <limits>

#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRandomGenerator>
#include <QUrl>
#include <QUrlQuery>
#include <QColor>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCamera>
#include <QCameraDevice>
#include <QCameraFormat>
#include <QMediaCaptureSession>
#include <QMediaDevices>
#include <QMediaRecorder>
#include <QMediaFormat>
#include <QSet>
#include <QVideoFrame>
#include <QVideoSink>
#endif

#include "cameraworker.h"

MESSAGE_CLASS_DEFINITION(CameraWorker::MsgConfigureCameraWorker, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgRefreshCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportResolutions, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportFrame, Message)

CameraWorker::CameraWorker() :
    m_msgQueueToGUI(nullptr),
    m_capturing(false),
    m_imageSaved(false),
    m_captureTimer(this),
    m_networkManager(nullptr),
    m_alpacaFrameRequestPending(false),
    m_alpacaClientId(QRandomGenerator::global()->generate() % std::numeric_limits<quint32>::max() + 1u),
    m_alpacaClientTransactionId(1)
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    , m_qtCamera(nullptr)
    , m_videoSink(nullptr)
    , m_captureSession(nullptr)
    , m_mediaRecorder(nullptr)
#endif
{
}

CameraWorker::~CameraWorker()
{
    stopWork();
}

void CameraWorker::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraWorker::handleInputMessages);
    QObject::connect(&m_captureTimer, &QTimer::timeout, this, &CameraWorker::captureTick);

    if (!m_networkManager) {
        m_networkManager = new QNetworkAccessManager(this);
    }

    reportCameraList();
    if (m_settings.m_captureActive) {
        startCapture();
    }
}

void CameraWorker::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraWorker::handleInputMessages);
    QObject::disconnect(&m_captureTimer, &QTimer::timeout, this, &CameraWorker::captureTick);
    stopCapture();
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

    return false;
}

void CameraWorker::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraWorker::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    const bool recapture = force
        || settingsKeys.contains("cameraAPI")
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("framesPerSecond")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("isoSensitivity")
        || settingsKeys.contains("alpacaHost")
        || settingsKeys.contains("alpacaPort")
        || settingsKeys.contains("alpacaCameraId")
        || settingsKeys.contains("saveVideo")
        || settingsKeys.contains("videoFileName");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
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

    if (force || settingsKeys.contains("cameraAPI") || settingsKeys.contains("alpacaHost") || settingsKeys.contains("alpacaPort")) {
        reportCameraList();
    }

    if (force
        || settingsKeys.contains("cameraAPI")
        || settingsKeys.contains("cameraId"))
    {
        reportResolutions();
    }
}

void CameraWorker::reportCameraList()
{
    if (m_settings.m_cameraAPI == CameraSettings::CameraAPIAlpaca)
    {
        if (!m_networkManager)
        {
            QStringList cameraIds;
            cameraIds.append(QString("alpaca:%1").arg(m_settings.m_alpacaCameraId));

            if (m_msgQueueToGUI) {
                m_msgQueueToGUI->push(MsgReportCameraList::create(cameraIds));
            }

            return;
        }

        QNetworkRequest request(QUrl(buildAlpacaBaseUrl() + "/management/v1/configureddevices"));
        QNetworkReply *reply = m_networkManager->get(request);

        QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
            QStringList cameraIds;

            if (reply->error() == QNetworkReply::NoError) {
                cameraIds = parseAlpacaCameraList(reply->readAll());
            }

            if (cameraIds.isEmpty()) {
                cameraIds.append(QString("alpaca:%1").arg(m_settings.m_alpacaCameraId));
            }

            if (m_msgQueueToGUI) {
                m_msgQueueToGUI->push(MsgReportCameraList::create(cameraIds));
            }

            reply->deleteLater();
        });

        return;
    }

    QStringList cameraIds;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();

    for (const QCameraDevice& camera : cameras)
    {
        const QString id = QString::fromUtf8(camera.id());
        cameraIds.append(id.isEmpty() ? camera.description() : id);
    }
#else
    cameraIds.append("Qt camera API unavailable (Qt6 required)");
#endif

    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportCameraList::create(cameraIds));
    }
}

void CameraWorker::reportResolutions()
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (m_settings.m_cameraAPI == CameraSettings::CameraAPIQtCamera)
    {
        const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
        QList<QSize> resolutions;

        for (const QCameraDevice& device : cameras)
        {
            const QString id = QString::fromUtf8(device.id());
            if ((id == m_settings.m_cameraId) || (device.description() == m_settings.m_cameraId))
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
#endif
}


void CameraWorker::startCapture()
{
    if (m_capturing) {
        return;
    }

    m_imageSaved = false;
    m_capturing = true;

    if (m_settings.m_cameraAPI == CameraSettings::CameraAPIAlpaca)
    {
        m_alpacaFrameRequestPending = false;
        const int intervalMs = std::max(10, static_cast<int>(std::lround(1000.0 / std::max(1, m_settings.m_framesPerSecond))));
        m_captureTimer.start(intervalMs);
        captureTick();
    }
    else
    {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        setupQtCapture();
#else
        reportFrameToGUI(createPlaceholderFrame());
#endif
    }
}

void CameraWorker::stopCapture()
{
    m_capturing = false;
    m_captureTimer.stop();

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    cleanupQtCapture();
#endif
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

    QUrl url(buildAlpacaBaseUrl() + QString("/api/v1/camera/%1/image").arg(m_settings.m_alpacaCameraId));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
    url.setQuery(query);

    QNetworkRequest request(url);
    QNetworkReply *reply = m_networkManager->get(request);

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        m_alpacaFrameRequestPending = false;

        if (!m_capturing) {
            reply->deleteLater();
            return;
        }

        QImage image = createPlaceholderFrame();

        if (reply->error() == QNetworkReply::NoError) {
            image = parseAlpacaImage(reply->readAll());
        }

        processNewFrame(image);
        reply->deleteLater();
    });
}

void CameraWorker::processNewFrame(const QImage& image)
{
    reportFrameToGUI(image);

    if (m_settings.m_saveImage && !m_imageSaved && !m_settings.m_imageFileName.isEmpty())
    {
        image.save(m_settings.m_imageFileName, "JPEG");
        m_imageSaved = true;
    }
}

void CameraWorker::reportFrameToGUI(const QImage& image)
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportFrame::create(image));
    }
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

QImage CameraWorker::parseAlpacaImage(const QByteArray& payload) const
{
    QImage image;
    image.loadFromData(payload);

    if (image.isNull()) {
        return createPlaceholderFrame();
    }

    return image;
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

    m_captureSession->setCamera(m_qtCamera);
    m_captureSession->setVideoOutput(m_videoSink);

    connect(m_videoSink, &QVideoSink::videoFrameChanged, this, &CameraWorker::processQtVideoFrame);

    if (m_settings.m_saveVideo && !m_settings.m_videoFileName.isEmpty())
    {
        m_mediaRecorder = new QMediaRecorder(this);
        m_mediaRecorder->setMediaFormat(QMediaFormat(QMediaFormat::MPEG4));
        m_mediaRecorder->setOutputLocation(QUrl::fromLocalFile(m_settings.m_videoFileName));
        m_captureSession->setRecorder(m_mediaRecorder);
        m_mediaRecorder->record();
    }

    m_qtCamera->start();
}

void CameraWorker::cleanupQtCapture()
{
    if (m_mediaRecorder)
    {
        m_mediaRecorder->stop();
        m_mediaRecorder->deleteLater();
        m_mediaRecorder = nullptr;
    }

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
#endif
