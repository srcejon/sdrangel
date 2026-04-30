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
#include <cstring>
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
#include <QtEndian>
#include <QUrl>
#include <QUrlQuery>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCamera>
#include <QCameraDevice>
#include <QCameraFormat>
#include <QMediaDevices>
#include <QSet>
#else
#include <QCamera>
#include <QCameraInfo>
#include <QSet>
#endif

#include "maincore.h"
#include "dsp/dspengine.h"
#include "audio/audiodevicemanager.h"
#include "camerapostprocessor.h"
#include "cameraworker.h"

namespace {

QImage renderGrayscaleRaw(const QVector<QVector<int>>& raw, int width, int height)
{
    int minValue = std::numeric_limits<int>::max();
    int maxValue = std::numeric_limits<int>::min();

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            minValue = std::min(minValue, raw[x][y]);
            maxValue = std::max(maxValue, raw[x][y]);
        }
    }

    const int range = maxValue - minValue;
    const int uniformGray = (range == 0) ? (minValue > 0 ? 128 : 0) : 0;

    QImage image(width, height, QImage::Format_Grayscale8);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            const int value = (range > 0)
                ? qBound(0, static_cast<int>(((raw[x][y] - minValue) * 255.0) / range), 255)
                : uniformGray;
            image.scanLine(y)[x] = static_cast<uchar>(value);
        }
    }

    return image;
}

QString normalizeAudioMatchName(QString text)
{
    text = text.toLower();

    for (int i = 0; i < text.size(); ++i)
    {
        if (!text[i].isLetterOrNumber()) {
            text[i] = QLatin1Char(' ');
        }
    }

    const QStringList skipTokens = {
        QStringLiteral("audio"),
        QStringLiteral("camera"),
        QStringLiteral("device"),
        QStringLiteral("input"),
        QStringLiteral("microphone"),
        QStringLiteral("mic"),
        QStringLiteral("video"),
        QStringLiteral("webcam")
    };

    QStringList tokens = text.simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
    tokens.erase(
        std::remove_if(
            tokens.begin(),
            tokens.end(),
            [&skipTokens](const QString& token) { return skipTokens.contains(token); }),
        tokens.end());
    return tokens.join(QLatin1Char(' '));
}

int scoreAudioDeviceMatch(const QString& cameraName, const QString& audioName)
{
    if (cameraName.isEmpty() || audioName.isEmpty()) {
        return -1;
    }

    const QString cameraLower = cameraName.toLower();
    const QString audioLower = audioName.toLower();

    if (cameraLower == audioLower) {
        return 1000;
    }

    int score = 0;

    if (audioLower.contains(cameraLower) || cameraLower.contains(audioLower)) {
        score += 400;
    }

    const QString normalizedCamera = normalizeAudioMatchName(cameraName);
    const QString normalizedAudio = normalizeAudioMatchName(audioName);

    if (!normalizedCamera.isEmpty() && normalizedCamera == normalizedAudio) {
        score += 300;
    } else if (!normalizedCamera.isEmpty() && !normalizedAudio.isEmpty()
            && (normalizedAudio.contains(normalizedCamera) || normalizedCamera.contains(normalizedAudio))) {
        score += 150;
    }

    const QStringList cameraTokens = normalizedCamera.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    const QStringList audioTokens = normalizedAudio.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    int tokenMatches = 0;

    for (const QString& token : cameraTokens)
    {
        if (audioTokens.contains(token)) {
            ++tokenMatches;
        }
    }

    score += tokenMatches * 25;
    return score;
}

void alignQtCameraAudioInputRate(AudioDeviceManager *audioDeviceManager, int inputDeviceIndex, int outputDeviceIndex)
{
    if ((audioDeviceManager == nullptr) || (inputDeviceIndex < 0)) {
        return;
    }

    const int outputSampleRate = audioDeviceManager->getOutputSampleRate(outputDeviceIndex);

    if (outputSampleRate <= 0) {
        return;
    }

    const QList<AudioDeviceInfo>& inputDevices = AudioDeviceInfo::availableInputDevices();

    if (inputDeviceIndex >= inputDevices.size()) {
        return;
    }

    const AudioDeviceInfo& inputDeviceInfo = inputDevices.at(inputDeviceIndex);
    const QList<int> supportedSampleRates = inputDeviceInfo.supportedSampleRates();

    if (!supportedSampleRates.contains(outputSampleRate))
    {
        qWarning() << "CameraWorker: input audio device" << inputDeviceInfo.deviceName()
                   << "does not support output sample rate" << outputSampleRate
                   << "supported sample rates:" << supportedSampleRates;
        return;
    }

    AudioDeviceManager::InputDeviceInfo configuredInputInfo;
    audioDeviceManager->getInputDeviceInfo(inputDeviceInfo.deviceName(), configuredInputInfo);

    if (configuredInputInfo.sampleRate == outputSampleRate) {
        return;
    }

    configuredInputInfo.sampleRate = outputSampleRate;
    audioDeviceManager->setInputDeviceInfo(inputDeviceIndex, configuredInputInfo);

    qDebug() << "CameraWorker: aligned input audio device" << inputDeviceInfo.deviceName()
             << "sample rate to output rate" << outputSampleRate;
}

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
int findQtCameraAudioInputIndex(const CameraSettings& settings)
{
    if (!settings.isQtCamera()) {
        return -1;
    }

    const QString targetId = settings.cameraIdString();
    const QString targetDescription = settings.cameraDescription();
    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();

    QString cameraDescription = targetDescription;

    for (const QCameraDevice& device : cameras)
    {
        const QString id = QString::fromUtf8(device.id());

        if ((id == targetId) || (device.description() == targetDescription))
        {
            cameraDescription = device.description();
            break;
        }
    }

    const QList<AudioDeviceInfo>& audioInputs = AudioDeviceInfo::availableInputDevices();
    int bestIndex = -1;
    int bestScore = -1;

    for (int i = 0; i < audioInputs.size(); ++i)
    {
        const QString audioName = audioInputs[i].deviceName();
        const int descriptionScore = scoreAudioDeviceMatch(cameraDescription, audioName);
        const int idScore = scoreAudioDeviceMatch(targetId, audioName);
        const int score = std::max(descriptionScore, idScore);

        if (score > bestScore)
        {
            bestScore = score;
            bestIndex = i;
        }
    }

    return bestScore >= 150 ? bestIndex : -1;
}
#endif

} // namespace

MESSAGE_CLASS_DEFINITION(CameraWorker::MsgConfigureCameraWorker, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgRefreshCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaCameraInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaStatus, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAvailableDevices, Message)

CameraWorker::CameraWorker() :
    m_msgQueueToGUI(nullptr),
    m_postProcessorInputMessageQueue(nullptr),
    m_availableDeviceHandler({}, QStringList{"spectrumview"}),
    m_capturing(false),
    m_capturingAudio(false),
    m_captureTimer(this),
    m_networkManager(nullptr),
    m_alpacaFrameRequestPending(false),
    m_alpacaClientId(QRandomGenerator::global()->bounded(quint32(1), quint32(std::numeric_limits<quint32>::max()))),
    m_alpacaClientTransactionId(1),
    m_alpacaSensorType(0),
    m_alpacaCameraSizeX(0),
    m_alpacaCameraSizeY(0),
    m_alpacaImageBytesSupported(true),
    m_alpacaParamsInitialized(false),
    m_lastAlpacaBinX(0),
    m_lastAlpacaBinY(0),
    m_lastAlpacaNumX(0),
    m_lastAlpacaNumY(0),
    m_lastAlpacaStartX(0),
    m_lastAlpacaStartY(0),
    m_lastAlpacaGain(-1),
    m_lastAlpacaOffset(-1),
    m_lastAlpacaReadoutMode(0),
    m_statusTimer(this),
    m_lastAlpacaCaptureTimeMs(-1),
    m_spectrumPipeSource(nullptr)
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
    static constexpr int audioFifoFrames = 4800*4;
    static constexpr int bytesPerSampleFrame = 4; // 2 channels × 2 bytes (int16)
    m_captureAudioFifo.setSize(audioFifoFrames);
    m_outputAudioFifo.setSize(audioFifoFrames);
    m_audioTransferBuffer.resize(audioFifoFrames * bytesPerSampleFrame);


}

CameraWorker::~CameraWorker()
{
    delete m_networkManager;
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
        m_statusTimer.start(m_alpacaStatusPollIntervalMs);
    }

    startCapture();

    // Handle any messages already on the queue
    handleInputMessages();
}

void CameraWorker::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraWorker::handleInputMessages);
    QObject::disconnect(&m_captureTimer, &QTimer::timeout, this, &CameraWorker::captureTick);
    QObject::disconnect(&m_statusTimer, &QTimer::timeout, this, &CameraWorker::statusTick);
    stopCapture();
    m_statusTimer.stop();
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

    const bool recapture = force
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("audioDeviceName")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("captureMode")
        || settingsKeys.contains("captureInterval")
        || settingsKeys.contains("captureIntervalUnits")
        || settingsKeys.contains("framesPerSecond")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("isoSensitivity")
        || settingsKeys.contains("alpacaHost")
        || settingsKeys.contains("alpacaPort");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (recapture && m_capturing)
    {
        stopCapture();
        startCapture();
    }

    if (m_settings.isAlpacaCamera()
        && m_networkManager
        && (force
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
                m_statusTimer.start(m_alpacaStatusPollIntervalMs);
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
        if (m_postProcessorInputMessageQueue) {
            m_postProcessorInputMessageQueue->push(CameraPostProcessor::MsgSpectrumFrame::create(QImage()));
        }
    }
}

void CameraWorker::startCapture()
{
    if (m_capturing) {
        return;
    }

    m_capturing = true;
    m_lastAlpacaCaptureTimeMs = -1;
    m_alpacaCaptureTimer.invalidate();
    m_alpacaParamsInitialized = false;
    m_lastAlpacaBinX = m_settings.m_alpacaBinX;
    m_lastAlpacaBinY = m_settings.m_alpacaBinY;
    m_lastAlpacaNumX = m_settings.m_alpacaNumX;
    m_lastAlpacaNumY = m_settings.m_alpacaNumY;
    m_lastAlpacaStartX = m_settings.m_alpacaStartX;
    m_lastAlpacaStartY = m_settings.m_alpacaStartY;
    m_lastAlpacaGain = m_settings.m_alpacaGain;
    m_lastAlpacaOffset = m_settings.m_alpacaOffset;
    m_lastAlpacaReadoutMode = m_settings.m_alpacaReadoutMode;

    if (m_settings.isAlpacaCamera())
    {
        m_alpacaCaptureTimer.start();
        m_alpacaFrameRequestPending = false;
        m_captureTimer.start(m_settings.isIntervalCaptureMode()
            ? m_settings.getCaptureIntervalMs()
            : std::max(10, static_cast<int>(std::lround(1000.0 / std::max(1, m_settings.m_framesPerSecond)))));
        captureTick();
    }
    else if (m_settings.isQtCamera())
    {
        // Qt camera capture is mainly managed by CameraGUI on the main thread. We just do audio
        AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
        const int outputDeviceIndex = audioDeviceManager->getOutputDeviceIndex(m_settings.m_audioDeviceName);
        int inputDeviceIndex = -1;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        inputDeviceIndex = findQtCameraAudioInputIndex(m_settings);
        alignQtCameraAudioInputRate(audioDeviceManager, inputDeviceIndex, outputDeviceIndex);
#endif
        qDebug() << "CameraWorker: starting audio capture: outputDeviceIndex" << outputDeviceIndex
                 << "inputDeviceIndex" << inputDeviceIndex;
        audioDeviceManager->addAudioSink(&m_outputAudioFifo, getInputMessageQueue(), outputDeviceIndex);
        audioDeviceManager->addAudioSource(&m_captureAudioFifo, getInputMessageQueue(), inputDeviceIndex);
        QObject::connect(&m_captureAudioFifo, &AudioFifo::dataReady, this, &CameraWorker::onCaptureAudioDataReady);
        m_capturingAudio = true;
    }
}

void CameraWorker::stopCapture()
{
    m_capturing = false;
    m_captureTimer.stop();
    m_alpacaCaptureTimer.invalidate();

    if (m_capturingAudio)
    {
        qDebug() << "CameraWorker: stopping audio capture";
        QObject::disconnect(&m_captureAudioFifo, &AudioFifo::dataReady, this, &CameraWorker::onCaptureAudioDataReady);
        AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
        audioDeviceManager->removeAudioSource(&m_captureAudioFifo);
        audioDeviceManager->removeAudioSink(&m_outputAudioFifo);
        m_capturingAudio = false;
    }
}

void CameraWorker::captureTick()
{
    if (!m_capturing) {
        return;
    }

    if (!m_networkManager || m_alpacaFrameRequestPending) {
        return;
    }

    if (!m_alpacaCaptureTimer.isValid()) {
        m_alpacaCaptureTimer.start();
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
    std::function<void()> continuation,
    std::function<void()> onSuccess = {})
{
    QUrl url(baseUrl + QString("/api/v1/camera/%1/%2").arg(cameraId).arg(property));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem(bodyKey, QString::number(value));
    body.addQueryItem("ClientID", QString::number(clientId));
    body.addQueryItem("ClientTransactionID", QString::number(transactionId++));

    QNetworkReply *reply = nam->put(request, body.toString(QUrl::FullyEncoded).toUtf8());

    QObject::connect(reply, &QNetworkReply::finished, reply, [reply, property, continuation, onSuccess]() {
        if (reply->error() != QNetworkReply::NoError) {
            qDebug() << "CameraWorker: PUT" << property << "error:" << reply->errorString();
        } else if (onSuccess) {
            onSuccess();
        }
        reply->deleteLater();
        continuation();
    });
}

void CameraWorker::alpacaSetCameraParams()
{
    // Chain: binX -> binY -> subframe ROI -> gain -> offset -> readoutMode -> startExposure
    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.cameraIdInt();
    const bool forceAllParams = !m_alpacaParamsInitialized;
    const int maxSubframeX = std::max(1, m_alpacaCameraSizeX / std::max(1, m_settings.m_alpacaBinX));
    const int maxSubframeY = std::max(1, m_alpacaCameraSizeY / std::max(1, m_settings.m_alpacaBinY));
    const int effectiveNumX = (m_settings.m_alpacaNumX > 0) ? m_settings.m_alpacaNumX
        : std::max(1, maxSubframeX - std::max(0, m_settings.m_alpacaStartX));
    const int effectiveNumY = (m_settings.m_alpacaNumY > 0) ? m_settings.m_alpacaNumY
        : std::max(1, maxSubframeY - std::max(0, m_settings.m_alpacaStartY));
    const int lastMaxSubframeX = std::max(1, m_alpacaCameraSizeX / std::max(1, m_lastAlpacaBinX));
    const int lastMaxSubframeY = std::max(1, m_alpacaCameraSizeY / std::max(1, m_lastAlpacaBinY));
    const int lastEffectiveNumX = (m_lastAlpacaNumX > 0) ? m_lastAlpacaNumX
        : std::max(1, lastMaxSubframeX - std::max(0, m_lastAlpacaStartX));
    const int lastEffectiveNumY = (m_lastAlpacaNumY > 0) ? m_lastAlpacaNumY
        : std::max(1, lastMaxSubframeY - std::max(0, m_lastAlpacaStartY));
    const bool setBinX = forceAllParams || (m_lastAlpacaBinX != m_settings.m_alpacaBinX);
    const bool setBinY = forceAllParams || (m_lastAlpacaBinY != m_settings.m_alpacaBinY);
    const bool setNumX = forceAllParams || (lastEffectiveNumX != effectiveNumX);
    const bool setNumY = forceAllParams || (lastEffectiveNumY != effectiveNumY);
    const bool setStartX = forceAllParams || (m_lastAlpacaStartX != m_settings.m_alpacaStartX);
    const bool setStartY = forceAllParams || (m_lastAlpacaStartY != m_settings.m_alpacaStartY);
    const bool setGain = (m_settings.m_alpacaGain >= 0)
        && (forceAllParams || (m_lastAlpacaGain != m_settings.m_alpacaGain));
    const bool setOffset = (m_settings.m_alpacaOffset >= 0)
        && (forceAllParams || (m_lastAlpacaOffset != m_settings.m_alpacaOffset));
    const bool setReadoutMode = forceAllParams || (m_lastAlpacaReadoutMode != m_settings.m_alpacaReadoutMode);

    auto doStartExposure = [this]() {
        if (m_capturing) {
            m_alpacaParamsInitialized = true;
            alpacaStartExposure();
        } else {
            m_alpacaFrameRequestPending = false;
        }
    };

    auto doReadoutMode = [this, baseUrl, camId, doStartExposure, setReadoutMode]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (setReadoutMode) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "readoutmode", "ReadoutMode",
                m_settings.m_alpacaReadoutMode, m_alpacaClientId, m_alpacaClientTransactionId, doStartExposure,
                [this]() { m_lastAlpacaReadoutMode = m_settings.m_alpacaReadoutMode; });
        } else {
            doStartExposure();
        }
    };

    auto doOffset = [this, baseUrl, camId, doReadoutMode, setOffset]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (setOffset) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "offset", "Offset",
                m_settings.m_alpacaOffset, m_alpacaClientId, m_alpacaClientTransactionId, doReadoutMode,
                [this]() { m_lastAlpacaOffset = m_settings.m_alpacaOffset; });
        } else {
            doReadoutMode();
        }
    };

    auto doGain = [this, baseUrl, camId, doOffset, setGain]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (setGain) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "gain", "Gain",
                m_settings.m_alpacaGain, m_alpacaClientId, m_alpacaClientTransactionId, doOffset,
                [this]() { m_lastAlpacaGain = m_settings.m_alpacaGain; });
        } else {
            doOffset();
        }
    };

    auto doAxisY = [this, baseUrl, camId, doGain, setNumY, setStartY, effectiveNumY]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }

        std::function<void()> maybeSetStartYAfterNum = [this, baseUrl, camId, doGain, setStartY]() {
            if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
            if (setStartY) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "starty", "StartY",
                    m_settings.m_alpacaStartY, m_alpacaClientId, m_alpacaClientTransactionId, doGain,
                    [this]() { m_lastAlpacaStartY = m_settings.m_alpacaStartY; });
            } else {
                doGain();
            }
        };

        std::function<void()> maybeSetNumYAfterStart = [this, baseUrl, camId, doGain, setNumY, effectiveNumY]() {
            if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
            if (setNumY) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numy", "NumY",
                    effectiveNumY, m_alpacaClientId, m_alpacaClientTransactionId, doGain,
                    [this]() { m_lastAlpacaNumY = m_settings.m_alpacaNumY; });
            } else {
                doGain();
            }
        };

        if (setStartY && (m_settings.m_alpacaStartY < m_lastAlpacaStartY))
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "starty", "StartY",
                m_settings.m_alpacaStartY, m_alpacaClientId, m_alpacaClientTransactionId, maybeSetNumYAfterStart,
                [this]() { m_lastAlpacaStartY = m_settings.m_alpacaStartY; });
        }
        else if (setNumY)
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numy", "NumY",
                effectiveNumY, m_alpacaClientId, m_alpacaClientTransactionId, maybeSetStartYAfterNum,
                [this]() { m_lastAlpacaNumY = m_settings.m_alpacaNumY; });
        }
        else
        {
            maybeSetStartYAfterNum();
        }
    };

    auto doAxisX = [this, baseUrl, camId, doAxisY, setNumX, setStartX, effectiveNumX]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }

        std::function<void()> maybeSetStartXAfterNum = [this, baseUrl, camId, doAxisY, setStartX]() {
            if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
            if (setStartX) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "startx", "StartX",
                    m_settings.m_alpacaStartX, m_alpacaClientId, m_alpacaClientTransactionId, doAxisY,
                    [this]() { m_lastAlpacaStartX = m_settings.m_alpacaStartX; });
            } else {
                doAxisY();
            }
        };

        std::function<void()> maybeSetNumXAfterStart = [this, baseUrl, camId, doAxisY, setNumX, effectiveNumX]() {
            if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
            if (setNumX) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numx", "NumX",
                    effectiveNumX, m_alpacaClientId, m_alpacaClientTransactionId, doAxisY,
                    [this]() { m_lastAlpacaNumX = m_settings.m_alpacaNumX; });
            } else {
                doAxisY();
            }
        };

        if (setStartX && (m_settings.m_alpacaStartX < m_lastAlpacaStartX))
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "startx", "StartX",
                m_settings.m_alpacaStartX, m_alpacaClientId, m_alpacaClientTransactionId, maybeSetNumXAfterStart,
                [this]() { m_lastAlpacaStartX = m_settings.m_alpacaStartX; });
        }
        else if (setNumX)
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numx", "NumX",
                effectiveNumX, m_alpacaClientId, m_alpacaClientTransactionId, maybeSetStartXAfterNum,
                [this]() { m_lastAlpacaNumX = m_settings.m_alpacaNumX; });
        }
        else
        {
            maybeSetStartXAfterNum();
        }
    };

    auto doBinY = [this, baseUrl, camId, doAxisX, setBinY]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (setBinY) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "biny", "BinY",
                m_settings.m_alpacaBinY, m_alpacaClientId, m_alpacaClientTransactionId, doAxisX,
                [this]() { m_lastAlpacaBinY = m_settings.m_alpacaBinY; });
        } else {
            doAxisX();
        }
    };

    if (setBinX) {
        alpacaPutIntProperty(m_networkManager, baseUrl, camId, "binx", "BinX",
            m_settings.m_alpacaBinX, m_alpacaClientId, m_alpacaClientTransactionId, doBinY,
            [this]() { m_lastAlpacaBinX = m_settings.m_alpacaBinX; });
    } else {
        doBinY();
    }
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
        QTimer::singleShot(static_cast<int>(std::ceil(m_settings.m_exposureTimeMs)), this, [this]() {
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
            QTimer::singleShot(m_alpacaImageReadyPollIntervalMs, this, [this]() {
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

        if (m_alpacaCaptureTimer.isValid())
        {
            m_lastAlpacaCaptureTimeMs = m_alpacaCaptureTimer.elapsed();
            m_alpacaCaptureTimer.invalidate();
        }

        if (m_postProcessorInputMessageQueue) {
            m_postProcessorInputMessageQueue->push(CameraPostProcessor::MsgProcessFrame::create(image));
        }
        reply->deleteLater();
    });
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
        double exposureMinMs = 1.0;
        double exposureMaxMs = 60000.0;
        double exposureResolutionMs = 1.0;
        int pending = 0;
    };

    auto info = QSharedPointer<CapInfo>::create();

    // Properties to query: name, JSON Value key, handler
    static const QStringList properties = {
        "maxbinx", "maxbiny", "gains", "gainmin", "gainmax",
        "offsets", "offsetmin", "offsetmax",
        "readoutmodes", "sensorname", "sensortype",
        "pixelsizex", "pixelsizey", "cameraxsize", "cameraysize",
        "ccdtemperature", "exposuremin", "exposuremax", "exposureresolution"
    };

    info->pending = properties.size();

    auto checkDone = [this, info]() {
        info->pending--;
        if (info->pending > 0) {
            return;
        }

        m_alpacaSensorType = info->sensorType;
        m_alpacaCameraSizeX = std::max(0, info->cameraSizeX);
        m_alpacaCameraSizeY = std::max(0, info->cameraSizeY);
        info->exposureMinMs = std::max(0.001, info->exposureMinMs);
        info->exposureResolutionMs = std::max(0.001, info->exposureResolutionMs);
        info->exposureMaxMs = std::max(info->exposureMinMs, info->exposureMaxMs);

        if (m_msgQueueToGUI) {
            m_msgQueueToGUI->push(MsgReportAlpacaCameraInfo::create(
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
                status->ccdTemperatureValid,
                m_lastAlpacaCaptureTimeMs));
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
        const double scale = (range > 0) ? (255.0 / range) : 0.0;
        const int uniformGray = (range == 0) ? (minVal > 0 ? 128 : 0) : 0;

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
    return renderGrayscaleRaw(raw, width, height);
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


void CameraWorker::onCaptureAudioDataReady()
{
    if (m_settings.m_audioMute)
    {
        m_captureAudioFifo.clear();
        return;
    }

    // Each audio sample frame is 4 bytes: stereo 16-bit PCM (2 channels × 2 bytes)
    static constexpr int bytesPerSampleFrame = 4;
    unsigned int nbRead;

    while ((nbRead = m_captureAudioFifo.read(m_audioTransferBuffer.data(), m_audioTransferBuffer.size() / bytesPerSampleFrame)) != 0) {
        m_outputAudioFifo.write(m_audioTransferBuffer.data(), nbRead);
    }
}

