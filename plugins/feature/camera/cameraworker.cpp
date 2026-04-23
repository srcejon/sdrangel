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
#include <functional>
#include <limits>

#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRandomGenerator>
#include <QSharedPointer>
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
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaCameraInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaStatus, Message)

CameraWorker::CameraWorker() :
    m_msgQueueToGUI(nullptr),
    m_capturing(false),
    m_imageSaved(false),
    m_captureTimer(this),
    m_networkManager(nullptr),
    m_alpacaFrameRequestPending(false),
    m_alpacaClientId(QRandomGenerator::global()->bounded(quint64(1), quint64(std::numeric_limits<quint32>::max()) + 1)),
    m_alpacaClientTransactionId(1),
    m_alpacaSensorType(0),
    m_statusTimer(this)
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
    QObject::connect(&m_statusTimer, &QTimer::timeout, this, &CameraWorker::statusTick);

    if (!m_networkManager) {
        m_networkManager = new QNetworkAccessManager(this);
    }

    reportCameraList();
    if (m_settings.m_cameraAPI == CameraSettings::CameraAPIAlpaca) {
        alpacaQueryCameraCapabilities();
        m_statusTimer.start(2000);
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
        || settingsKeys.contains("alpacaBinX")
        || settingsKeys.contains("alpacaBinY")
        || settingsKeys.contains("alpacaGain")
        || settingsKeys.contains("alpacaReadoutMode")
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

    if (m_settings.m_cameraAPI == CameraSettings::CameraAPIAlpaca
        && m_networkManager
        && (force
            || settingsKeys.contains("cameraAPI")
            || settingsKeys.contains("alpacaHost")
            || settingsKeys.contains("alpacaPort")
            || settingsKeys.contains("alpacaCameraId")
            || settingsKeys.contains("cameraId")))
    {
        alpacaQueryCameraCapabilities();
    }

    if (force || settingsKeys.contains("cameraAPI"))
    {
        if (m_settings.m_cameraAPI == CameraSettings::CameraAPIAlpaca) {
            if (!m_statusTimer.isActive()) {
                m_statusTimer.start(2000);
            }
        } else {
            m_statusTimer.stop();
        }
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
    // Chain: binX -> binY -> gain -> readoutMode -> startExposure
    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.m_alpacaCameraId;

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

    auto doGain = [this, baseUrl, camId, doReadoutMode]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (m_settings.m_alpacaGain >= 0) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "gain", "Gain",
                m_settings.m_alpacaGain, m_alpacaClientId, m_alpacaClientTransactionId, doReadoutMode);
        } else {
            doReadoutMode();
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
    QUrl url(buildAlpacaBaseUrl() + QString("/api/v1/camera/%1/startexposure").arg(m_settings.m_alpacaCameraId));
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
    QUrl url(buildAlpacaBaseUrl() + QString("/api/v1/camera/%1/imageready").arg(m_settings.m_alpacaCameraId));
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
    QUrl url(buildAlpacaBaseUrl() + QString("/api/v1/camera/%1/imagearray").arg(m_settings.m_alpacaCameraId));
    QUrlQuery query;
    query.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    query.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
    url.setQuery(query);

    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        m_alpacaFrameRequestPending = false;

        if (!m_capturing) {
            reply->deleteLater();
            return;
        }

        QImage image = createPlaceholderFrame();

        if (reply->error() == QNetworkReply::NoError) {
            image = parseAlpacaImageArray(reply->readAll());
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

void CameraWorker::alpacaQueryCameraCapabilities()
{
    if (!m_networkManager) {
        return;
    }

    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.m_alpacaCameraId;

    // Struct to accumulate results from parallel requests
    struct CapInfo {
        int maxBinX = 1;
        int maxBinY = 1;
        QStringList gains;
        int gainMin = 0;
        int gainMax = 0;
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
    if (m_networkManager && m_settings.m_cameraAPI == CameraSettings::CameraAPIAlpaca) {
        alpacaPollStatus();
    }
}

void CameraWorker::alpacaPollStatus()
{
    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.m_alpacaCameraId;

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

        // First pass: find maximum pixel value for linear scaling to 8-bit
        int maxVal = 1;
        for (const QJsonValue& col : value) {
            for (const QJsonValue& pix : col.toArray()) {
                maxVal = std::max(maxVal, pix.toInt(0));
            }
        }
        const double scale = 255.0 / maxVal;

        // Build scaled raw array (column-major)
        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            const QJsonArray col = value[x].toArray();
            for (int y = 0; y < height; ++y) {
                raw[x][y] = qBound(0, static_cast<int>(col[y].toInt(0) * scale), 255);
            }
        }

        // Bayer demosaicing for sensorType 2 (RGGB), 3 (CMYG), 4 (CMYG2), 5 (LRGB)
        // sensorType 0 = Monochrome, 1 = Colour (handled by rank 3 normally)
        const bool isBayer = (m_alpacaSensorType >= 2 && m_alpacaSensorType <= 5);
        if (isBayer)
        {
            // Simple bilinear Bayer demosaicing for RGGB (sensorType 2).
            // RGGB pattern (bayerOffsetX/Y assumed 0):
            //   (even x, even y) = R
            //   (odd  x, even y) = G1
            //   (even x, odd  y) = G2
            //   (odd  x, odd  y) = B
            //
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
        }

        // Monochrome (sensorType 0 or 1 returning rank 2, or unsupported Bayer types)
        QImage image(width, height, QImage::Format_Grayscale8);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                image.scanLine(y)[x] = static_cast<uchar>(raw[x][y]);
            }
        }
        return image;
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

        // First pass: find maximum pixel value across all planes for linear scaling
        int maxVal = 1;
        for (const QJsonArray* plane : {&planeR, &planeG, &planeB}) {
            for (const QJsonValue& col : *plane) {
                for (const QJsonValue& pix : col.toArray()) {
                    maxVal = std::max(maxVal, pix.toInt(0));
                }
            }
        }
        const double scale = 255.0 / maxVal;

        QImage image(width, height, QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            const QJsonArray colR = planeR[x].toArray();
            const QJsonArray colG = planeG[x].toArray();
            const QJsonArray colB = planeB[x].toArray();
            for (int y = 0; y < height; ++y) {
                const int r = qBound(0, static_cast<int>(colR[y].toInt(0) * scale), 255);
                const int g = qBound(0, static_cast<int>(colG[y].toInt(0) * scale), 255);
                const int b = qBound(0, static_cast<int>(colB[y].toInt(0) * scale), 255);
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
