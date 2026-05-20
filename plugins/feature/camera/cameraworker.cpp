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
#include <QThread>
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

#ifdef ASICAMERA_FOUND
#include <ASICamera2.h>
#endif

#include "maincore.h"
#include "dsp/dspengine.h"
#include "audio/audiodevicemanager.h"
#include "util/profiler.h"
#include "camera.h"
#include "cameraalpacacontroller.h"
#include "cameraasicontroller.h"
#include "camerafinder.h"
#include "cameraframepreprocessor.h"
#include "camerapostprocessor.h"
#include "cameraworker.h"

QString CameraWorker::normalizeAudioMatchName(QString text)
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

int CameraWorker::scoreAudioDeviceMatch(const QString& cameraName, const QString& audioName)
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

void CameraWorker::alignQtCameraAudioInputRate(AudioDeviceManager *audioDeviceManager, int inputDeviceIndex, int outputDeviceIndex)
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
int CameraWorker::findQtCameraAudioInputIndex(const CameraSettings& settings)
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
    m_capturingAudio(false),
    m_captureTimer(this),
    m_networkManager(nullptr),
    m_cameraFinder(new CameraFinder(this)),
    m_stackFrameIndex(0),
    m_hdrExposureIndex(0),
    m_alpaca(),
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

    // Audio FIFO: stereo 16-bit PCM at 48 kHz; 4800 sample frames × 4 bytes each
    static constexpr int audioFifoFrames = 4800*4;
    static constexpr int bytesPerSampleFrame = 4; // 2 channels × 2 bytes (int16)
    m_captureAudioFifo.setSize(audioFifoFrames);
    m_outputAudioFifo.setSize(audioFifoFrames);
    m_audioTransferBuffer.resize(audioFifoFrames * bytesPerSampleFrame);
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
        m_asi.m_settingsApplied = false;
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
        m_asi.m_settingsApplied = false;
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
    m_alpaca.m_captureTimer.invalidate();
    resetHdrBracketState();

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpaca.m_frameRequestPending) {
        alpacaAbortExposure();
    }

#ifdef ASICAMERA_FOUND
    ++m_asi.m_continuousCaptureGeneration;
    m_asi.m_continuousCaptureScheduled = false;
    m_asi.stopVideoCapture(m_settings.cameraIdInt());
    if (m_settings.isAsiCamera() && m_settings.isIntervalCaptureMode()) {
        m_asi.stopExposure(m_settings.cameraIdInt());
    }
    m_asi.m_settingsApplied = false;
#endif

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

    auto doReadoutMode = [this, baseUrl, camId, doStartExposure, setReadoutMode, handleParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
        if (setReadoutMode) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "readoutmode", "ReadoutMode",
                m_settings.m_cameraReadoutMode, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doStartExposure,
                [this]() { m_alpaca.m_lastReadoutMode = m_settings.m_cameraReadoutMode; },
                handleParamFailure);
        } else {
            doStartExposure();
        }
    };

    auto doOffset = [this, baseUrl, camId, doReadoutMode, setOffset, handleParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
        if (setOffset) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "offset", "Offset",
                m_settings.m_cameraOffset, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doReadoutMode,
                [this]() { m_alpaca.m_lastOffset = m_settings.m_cameraOffset; },
                handleParamFailure);
        } else {
            doReadoutMode();
        }
    };

    auto doGain = [this, baseUrl, camId, doOffset, setGain, handleParamFailure]() {
        if (!m_capturing) { m_alpaca.m_frameRequestPending = false; return; }
        if (setGain) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "gain", "Gain",
                m_settings.m_cameraGain, m_alpaca.m_clientId, m_alpaca.m_clientTransactionId, m_settings.m_alpacaApiLogEnabled, doOffset,
                [this]() { m_alpaca.m_lastGain = m_settings.m_cameraGain; },
                handleParamFailure);
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

    switch (m_asi.m_lastOpenFailureStage)
    {
    case CameraAsiController::OpenFailureOpen:
        reportErrorToFeature(
            QStringLiteral("asiOpen:%1").arg(cameraId),
            tr("ASI camera open failed"),
            tr("Failed to open ASI camera %1:\n%2").arg(cameraId).arg(m_asi.m_lastErrorMessage));
        break;
    case CameraAsiController::OpenFailureInit:
        reportErrorToFeature(
            QStringLiteral("asiInit:%1").arg(cameraId),
            tr("ASI camera initialization failed"),
            tr("Failed to initialize ASI camera %1:\n%2").arg(cameraId).arg(m_asi.m_lastErrorMessage));
        break;
    case CameraAsiController::OpenFailureMode:
        reportErrorToFeature(
            QStringLiteral("asiMode:%1").arg(cameraId),
            tr("ASI camera mode setup failed"),
            tr("Failed to set ASI camera %1 to normal mode:\n%2").arg(cameraId).arg(m_asi.m_lastErrorMessage));
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

    ASI_CAMERA_INFO cameraInfo {};

    if (!CameraAsiController::getCameraInfoById(cameraId, cameraInfo)) {
        return;
    }

    ASI_CONTROL_CAPS gainRange {};
    ASI_CONTROL_CAPS offsetRange {};
    ASI_CONTROL_CAPS exposureRange {};
    ASI_CONTROL_CAPS coolerOnCaps {};
    ASI_CONTROL_CAPS targetTempCaps {};
    ASI_CONTROL_CAPS usbBandwidthCaps {};
    ASI_CONTROL_CAPS highSpeedModeCaps {};
    const bool hasGainRange = CameraAsiController::getControlCapsByType(cameraId, ASI_GAIN, gainRange);
    const bool hasOffsetRange = CameraAsiController::getControlCapsByType(cameraId, ASI_OFFSET, offsetRange);
    const bool hasExposureRange = CameraAsiController::getControlCapsByType(cameraId, ASI_EXPOSURE, exposureRange);
    const bool hasCoolerOn = CameraAsiController::getControlCapsByType(cameraId, ASI_COOLER_ON, coolerOnCaps);
    const bool hasTargetTemp = CameraAsiController::getControlCapsByType(cameraId, ASI_TARGET_TEMP, targetTempCaps);
    const bool hasUsbBandwidth = CameraAsiController::getControlCapsByType(cameraId, ASI_BANDWIDTHOVERLOAD, usbBandwidthCaps);
    const bool hasHighSpeedMode = CameraAsiController::getControlCapsByType(cameraId, ASI_HIGH_SPEED_MODE, highSpeedModeCaps);
    long coolerOnValue = 0;
    long targetTempValue = 0;
    long usbBandwidthValue = 0;
    long highSpeedModeValue = 0;
    ASI_BOOL isAuto = ASI_FALSE;
    const bool hasCoolerOnValue = hasCoolerOn && CameraAsiController::getControlValueByType(cameraId, ASI_COOLER_ON, coolerOnValue, isAuto);
    const bool hasTargetTempValue = hasTargetTemp && CameraAsiController::getControlValueByType(cameraId, ASI_TARGET_TEMP, targetTempValue, isAuto);
    const bool hasUsbBandwidthValue = hasUsbBandwidth && CameraAsiController::getControlValueByType(cameraId, ASI_BANDWIDTHOVERLOAD, usbBandwidthValue, isAuto);
    const bool hasHighSpeedModeValue = hasHighSpeedMode && CameraAsiController::getControlValueByType(cameraId, ASI_HIGH_SPEED_MODE, highSpeedModeValue, isAuto);

    m_asi.m_cameraSizeX = static_cast<int>(cameraInfo.MaxWidth);
    m_asi.m_cameraSizeY = static_cast<int>(cameraInfo.MaxHeight);
    m_asi.m_maxBinX = 1;
    m_asi.m_maxBinY = 1;
    for (int bin : cameraInfo.SupportedBins)
    {
        if (bin <= 0) {
            break;
        }

        m_asi.m_maxBinX = std::max(m_asi.m_maxBinX, bin);
        m_asi.m_maxBinY = std::max(m_asi.m_maxBinY, bin);
    }
    m_asi.m_bayerPattern = cameraInfo.BayerPattern;
    m_asi.m_colorCamera = cameraInfo.IsColorCam == ASI_TRUE;
    m_asi.m_triggerCamera = cameraInfo.IsTriggerCam == ASI_TRUE;
    m_asi.m_bitDepth = cameraInfo.BitDepth;
    m_asi.m_pixelSizeUm = cameraInfo.PixelSize;
    m_asi.m_exposureMinMs = hasExposureRange ? std::max(0.001, exposureRange.MinValue / 1000.0) : 0.001;
    m_asi.m_exposureMaxMs = hasExposureRange ? std::max(m_asi.m_exposureMinMs, exposureRange.MaxValue / 1000.0) : 60000.0;
    m_asi.m_rgb24Supported = CameraAsiController::supportsImageType(cameraInfo, ASI_IMG_RGB24);
    m_asi.m_raw16Supported = CameraAsiController::supportsImageType(cameraInfo, ASI_IMG_RAW16);
    m_asi.m_raw8Supported = CameraAsiController::supportsImageType(cameraInfo, ASI_IMG_RAW8);
    m_asi.m_imageType = CameraAsiController::selectImageType(cameraInfo, m_settings);

    if (m_msgQueueToGUI)
    {
        m_msgQueueToGUI->push(MsgReportAsiCameraInfo::create(
            QString::fromUtf8(cameraInfo.Name),
            m_asi.m_maxBinX,
            m_asi.m_maxBinY,
            hasGainRange ? static_cast<int>(gainRange.MinValue) : 0,
            hasGainRange ? static_cast<int>(gainRange.MaxValue) : 100,
            hasOffsetRange ? static_cast<int>(offsetRange.MinValue) : 0,
            hasOffsetRange ? static_cast<int>(offsetRange.MaxValue) : 100,
            m_asi.m_cameraSizeX,
            m_asi.m_cameraSizeY,
            m_asi.m_pixelSizeUm,
            m_asi.m_bitDepth,
            m_asi.m_colorCamera,
            m_asi.m_exposureMinMs,
            m_asi.m_exposureMaxMs,
            hasCoolerOn && coolerOnCaps.IsWritable == ASI_TRUE,
            hasCoolerOnValue ? (coolerOnValue != 0) : false,
            hasTargetTemp && targetTempCaps.IsWritable == ASI_TRUE,
            hasTargetTemp ? static_cast<int>(targetTempCaps.MinValue) : 0,
            hasTargetTemp ? static_cast<int>(targetTempCaps.MaxValue) : 0,
            hasTargetTempValue ? static_cast<int>(targetTempValue) : 0,
            hasUsbBandwidth && usbBandwidthCaps.IsWritable == ASI_TRUE,
            hasUsbBandwidth ? static_cast<int>(usbBandwidthCaps.MinValue) : 0,
            hasUsbBandwidth ? static_cast<int>(usbBandwidthCaps.MaxValue) : 0,
            hasUsbBandwidthValue ? static_cast<int>(usbBandwidthValue) : 0,
            hasHighSpeedMode && highSpeedModeCaps.IsWritable == ASI_TRUE,
            hasHighSpeedModeValue ? (highSpeedModeValue != 0) : false,
            m_asi.m_rgb24Supported,
            m_asi.m_raw16Supported,
            m_asi.m_raw8Supported));
    }

    asiPollStatus();
}

bool CameraWorker::asiApplyCameraSettings()
{
    if ((m_asi.m_cameraSizeX <= 0) || (m_asi.m_cameraSizeY <= 0)) {
        asiQueryCameraCapabilities();
    }

    if (!asiOpenCamera()) {
        return false;
    }

    const int cameraId = m_settings.cameraIdInt();
    ASI_CAMERA_INFO cameraInfo {};
    if (CameraAsiController::getCameraInfoById(cameraId, cameraInfo)) {
        m_asi.m_imageType = CameraAsiController::selectImageType(cameraInfo, m_settings);
    }
    const int bin = std::max(1, std::min(m_settings.m_cameraBinX, m_settings.m_cameraBinY));
    const int roiWidthStep = 8;
    const int roiHeightStep = 2;
    const int minWidth = 16;
    const int minHeight = 16;
    const int maxWidth = std::max(minWidth, m_asi.m_cameraSizeX / std::max(1, bin));
    const int maxHeight = std::max(minHeight, m_asi.m_cameraSizeY / std::max(1, bin));

    auto alignDown = [](int value, int step, int minimum) {
        const int aligned = (value / step) * step;
        return std::max(minimum, aligned);
    };

    const int requestedWidth = (m_settings.m_cameraNumX == 0)
        ? maxWidth
        : qBound(minWidth, m_settings.m_cameraNumX, maxWidth);
    const int requestedHeight = (m_settings.m_cameraNumY == 0)
        ? maxHeight
        : qBound(minHeight, m_settings.m_cameraNumY, maxHeight);

    const int width = alignDown(requestedWidth, roiWidthStep, minWidth);
    const int height = alignDown(requestedHeight, roiHeightStep, minHeight);
    const int startX = qBound(0, m_settings.m_cameraStartX, std::max(0, maxWidth - width));
    const int startY = qBound(0, m_settings.m_cameraStartY, std::max(0, maxHeight - height));

    const ASI_ERROR_CODE roiError = ASISetROIFormat(cameraId, width, height, bin, static_cast<ASI_IMG_TYPE>(m_asi.m_imageType));
    if (roiError != ASI_SUCCESS) {
        setLastAsiError(roiError, CameraAsiController::errorCodeToString(roiError));
        qDebug() << "CameraWorker: ASISetROIFormat failed:" << roiError << CameraAsiController::errorCodeToString(roiError)
                   << "width" << width << "height" << height << "bin" << bin << "imageType" << m_asi.m_imageType;
        return false;
    }

    const ASI_ERROR_CODE startPosError = ASISetStartPos(cameraId, startX, startY);
    if (startPosError != ASI_SUCCESS) {
        setLastAsiError(startPosError, CameraAsiController::errorCodeToString(startPosError));
        qDebug() << "CameraWorker: ASISetStartPos failed:" << startPosError << CameraAsiController::errorCodeToString(startPosError)
                 << "startX" << startX << "startY" << startY << "width" << width << "height" << height << "bin" << bin;
        return false;
    }

    const ASI_BOOL autoExposureGain = (m_settings.m_asiAutoExposureGain
            && (m_settings.m_captureMode == CameraSettings::CaptureModeFrameRate))
        ? ASI_TRUE
        : ASI_FALSE;
    auto writableControl = [cameraId](ASI_CONTROL_TYPE controlType, ASI_CONTROL_CAPS *controlCaps = nullptr) -> bool {
        ASI_CONTROL_CAPS caps {};
        if (!CameraAsiController::getControlCapsByType(cameraId, controlType, caps) || (caps.IsWritable != ASI_TRUE)) {
            return false;
        }

        if (controlCaps) {
            *controlCaps = caps;
        }
        return true;
    };

    ASI_CONTROL_CAPS coolerOnCaps {};
    ASI_CONTROL_CAPS targetTempCaps {};
    ASI_CONTROL_CAPS usbBandwidthCaps {};
    ASI_CONTROL_CAPS highSpeedModeCaps {};
    const bool canSetCoolerOn = writableControl(ASI_COOLER_ON, &coolerOnCaps);
    const bool canSetTargetTemp = writableControl(ASI_TARGET_TEMP, &targetTempCaps);
    const bool canSetUsbBandwidth = writableControl(ASI_BANDWIDTHOVERLOAD, &usbBandwidthCaps);
    const bool canSetHighSpeedMode = writableControl(ASI_HIGH_SPEED_MODE, &highSpeedModeCaps);

    const ASI_ERROR_CODE exposureError = ASISetControlValue(cameraId, ASI_EXPOSURE,
        std::max(1L, static_cast<long>(std::llround(currentCaptureExposureTimeMs() * 1000.0))), autoExposureGain);
    const ASI_ERROR_CODE gainError = ASISetControlValue(cameraId, ASI_GAIN,
        std::max(0L, static_cast<long>(m_settings.m_cameraGain)), autoExposureGain);
    const ASI_ERROR_CODE offsetError = ASISetControlValue(cameraId, ASI_OFFSET,
        std::max(0L, static_cast<long>(m_settings.m_cameraOffset)), ASI_FALSE);
    const ASI_ERROR_CODE coolerOnError = ((m_settings.m_asiCoolerOn >= 0) && canSetCoolerOn)
        ? ASISetControlValue(cameraId, ASI_COOLER_ON, m_settings.m_asiCoolerOn != 0 ? 1L : 0L, ASI_FALSE)
        : ASI_SUCCESS;
    const ASI_ERROR_CODE targetTempError = ((m_settings.m_asiTargetTemp != std::numeric_limits<int>::min()) && canSetTargetTemp)
        ? ASISetControlValue(cameraId, ASI_TARGET_TEMP,
            qBound(targetTempCaps.MinValue, static_cast<long>(m_settings.m_asiTargetTemp), targetTempCaps.MaxValue),
            ASI_FALSE)
        : ASI_SUCCESS;
    const ASI_ERROR_CODE usbBandwidthError = ((m_settings.m_asiUsbBandwidth >= 0) && canSetUsbBandwidth)
        ? ASISetControlValue(cameraId, ASI_BANDWIDTHOVERLOAD,
            qBound(usbBandwidthCaps.MinValue, static_cast<long>(m_settings.m_asiUsbBandwidth), usbBandwidthCaps.MaxValue),
            ASI_FALSE)
        : ASI_SUCCESS;
    const ASI_ERROR_CODE highSpeedModeError = ((m_settings.m_asiHighSpeedMode >= 0) && canSetHighSpeedMode)
        ? ASISetControlValue(cameraId, ASI_HIGH_SPEED_MODE, m_settings.m_asiHighSpeedMode != 0 ? 1L : 0L, ASI_FALSE)
        : ASI_SUCCESS;

    if (exposureError != ASI_SUCCESS) {
        setLastAsiError(exposureError, CameraAsiController::errorCodeToString(exposureError));
        qDebug() << "CameraWorker: ASISetControlValue(EXPOSURE) failed:" << exposureError << CameraAsiController::errorCodeToString(exposureError);
        return false;
    }

    if (gainError != ASI_SUCCESS) {
        setLastAsiError(gainError, CameraAsiController::errorCodeToString(gainError));
        qDebug() << "CameraWorker: ASISetControlValue(GAIN) failed:" << gainError << CameraAsiController::errorCodeToString(gainError);
        return false;
    }

    if (offsetError != ASI_SUCCESS) {
        setLastAsiError(offsetError, CameraAsiController::errorCodeToString(offsetError));
        qDebug() << "CameraWorker: ASISetControlValue(OFFSET) failed:" << offsetError << CameraAsiController::errorCodeToString(offsetError);
        return false;
    }
    if (coolerOnError != ASI_SUCCESS) {
        setLastAsiError(coolerOnError, CameraAsiController::errorCodeToString(coolerOnError));
        qDebug() << "CameraWorker: ASISetControlValue(COOLER_ON) failed:" << coolerOnError << CameraAsiController::errorCodeToString(coolerOnError);
        return false;
    }
    if (targetTempError != ASI_SUCCESS) {
        setLastAsiError(targetTempError, CameraAsiController::errorCodeToString(targetTempError));
        qDebug() << "CameraWorker: ASISetControlValue(TARGET_TEMP) failed:" << targetTempError << CameraAsiController::errorCodeToString(targetTempError);
        return false;
    }
    if (usbBandwidthError != ASI_SUCCESS) {
        setLastAsiError(usbBandwidthError, CameraAsiController::errorCodeToString(usbBandwidthError));
        qDebug() << "CameraWorker: ASISetControlValue(BANDWIDTHOVERLOAD) failed:" << usbBandwidthError << CameraAsiController::errorCodeToString(usbBandwidthError);
        return false;
    }
    if (highSpeedModeError != ASI_SUCCESS) {
        setLastAsiError(highSpeedModeError, CameraAsiController::errorCodeToString(highSpeedModeError));
        qDebug() << "CameraWorker: ASISetControlValue(HIGH_SPEED_MODE) failed:" << highSpeedModeError << CameraAsiController::errorCodeToString(highSpeedModeError);
        return false;
    }

    m_asi.m_frameWidth = width;
    m_asi.m_frameHeight = height;

    int bytesPerPixel = 1;
    switch (m_asi.m_imageType)
    {
    case ASI_IMG_RGB24:
        bytesPerPixel = 3;
        break;
    case ASI_IMG_RAW16:
        bytesPerPixel = 2;
        break;
    default:
        bytesPerPixel = 1;
        break;
    }

    m_asi.m_frameBuffer.resize(width * height * bytesPerPixel);
    m_asi.m_settingsApplied = true;
    setLastAsiError(ASI_SUCCESS, QString());
    return true;
}

bool CameraWorker::asiCaptureExposureFrame()
{
    const int cameraId = m_settings.cameraIdInt();

    if (m_asi.m_videoCaptureStarted)
    {
        if (!m_asi.stopVideoCapture(cameraId)) {
            return false;
        }
    }

    const ASI_ERROR_CODE startExposureError = ASIStartExposure(cameraId, ASI_FALSE);
    if (startExposureError != ASI_SUCCESS)
    {
        setLastAsiError(startExposureError, CameraAsiController::errorCodeToString(startExposureError));
        qDebug() << "CameraWorker: ASIStartExposure failed:" << startExposureError << CameraAsiController::errorCodeToString(startExposureError);
        reportErrorToFeature(
            QStringLiteral("asiStartExposure:%1").arg(cameraId),
            tr("ASI exposure start failed"),
            tr("Failed to start ASI exposure on camera %1:\n%2").arg(cameraId).arg(CameraAsiController::errorCodeToString(startExposureError)));
        return false;
    }

    setLastAsiError(ASI_SUCCESS, QString());
    QElapsedTimer captureTimer;
    captureTimer.start();

    const double exposureTimeMs = currentCaptureExposureTimeMs();
    const qint64 timeoutMs = std::max<qint64>(1000, static_cast<qint64>(std::ceil(exposureTimeMs)) + 5000);
    const unsigned long pollSleepMs = static_cast<unsigned long>(
        std::min<qint64>(50, std::max<qint64>(2, static_cast<qint64>(std::ceil(exposureTimeMs / 4.0)))));
    ASI_EXPOSURE_STATUS exposureStatus = ASI_EXP_IDLE;

    while (captureTimer.elapsed() <= timeoutMs)
    {
        const ASI_ERROR_CODE statusError = ASIGetExpStatus(cameraId, &exposureStatus);
        if (statusError != ASI_SUCCESS)
        {
            setLastAsiError(statusError, CameraAsiController::errorCodeToString(statusError));
            qDebug() << "CameraWorker: ASIGetExpStatus failed:" << statusError << CameraAsiController::errorCodeToString(statusError);
            return false;
        }

        if (exposureStatus == ASI_EXP_SUCCESS) {
            break;
        }

        if (exposureStatus == ASI_EXP_FAILED)
        {
            setLastAsiError(ASI_ERROR_GENERAL_ERROR, QStringLiteral("Exposure failed"));
            qDebug() << "CameraWorker: ASI exposure failed";
            return false;
        }

        QThread::msleep(pollSleepMs);
    }

    if (exposureStatus != ASI_EXP_SUCCESS)
    {
        setLastAsiError(ASI_ERROR_TIMEOUT, CameraAsiController::errorCodeToString(ASI_ERROR_TIMEOUT));
        qDebug() << "CameraWorker: ASI exposure timed out after" << timeoutMs << "ms";
        return false;
    }

    const ASI_ERROR_CODE dataError = ASIGetDataAfterExp(cameraId, m_asi.m_frameBuffer.data(), m_asi.m_frameBuffer.size());
    if (dataError != ASI_SUCCESS)
    {
        setLastAsiError(dataError, CameraAsiController::errorCodeToString(dataError));
        qDebug() << "CameraWorker: ASIGetDataAfterExp failed:" << dataError << CameraAsiController::errorCodeToString(dataError)
                 << "bufferSize" << m_asi.m_frameBuffer.size()
                 << "width" << m_asi.m_frameWidth << "height" << m_asi.m_frameHeight;
        return false;
    }

    setLastAsiError(ASI_SUCCESS, QString());
    m_asi.m_lastCaptureTimeMs = captureTimer.elapsed();
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
    if (!useAsiContinuousVideoCadence() || m_asi.m_continuousCaptureScheduled) {
        return;
    }

    m_asi.m_continuousCaptureScheduled = true;
    const quint64 generation = m_asi.m_continuousCaptureGeneration;
    QTimer::singleShot(std::max(0, delayMs), this, [this, generation]() {
        if (generation != m_asi.m_continuousCaptureGeneration) {
            return;
        }
        m_asi.m_continuousCaptureScheduled = false;
        if (useAsiContinuousVideoCadence()) {
            captureTick();
        }
    });
}

void CameraWorker::asiCaptureVideoFrame()
{
    PROFILER_START();

    const int cameraId = m_settings.cameraIdInt();

    if (!m_asi.m_videoCaptureStarted)
    {
        const ASI_ERROR_CODE startCaptureError = ASIStartVideoCapture(cameraId);
        if (startCaptureError != ASI_SUCCESS) {
            setLastAsiError(startCaptureError, CameraAsiController::errorCodeToString(startCaptureError));
            qDebug() << "CameraWorker: ASIStartVideoCapture failed:" << startCaptureError << CameraAsiController::errorCodeToString(startCaptureError);
            reportErrorToFeature(
                QStringLiteral("asiStartVideo:%1").arg(cameraId),
                tr("ASI video capture start failed"),
                tr("Failed to start ASI video capture on camera %1:\n%2").arg(cameraId).arg(CameraAsiController::errorCodeToString(startCaptureError)));
            scheduleNextAsiVideoCapture(100);
            return;
        }
        setLastAsiError(ASI_SUCCESS, QString());
        m_asi.m_videoCaptureStarted = true;
    }

    const int waitMs = std::max(1000, static_cast<int>(std::ceil(currentCaptureExposureTimeMs())) + 500);
    QElapsedTimer captureTimer;
    captureTimer.start();
    const ASI_ERROR_CODE getVideoError = ASIGetVideoData(cameraId, m_asi.m_frameBuffer.data(), m_asi.m_frameBuffer.size(), waitMs);
    if (getVideoError == ASI_SUCCESS)
    {
        setLastAsiError(ASI_SUCCESS, QString());
        m_asi.m_lastCaptureTimeMs = captureTimer.elapsed();
        if (m_framePreprocessor) {
            CameraPipelineFrame::BayerPattern bayerPattern = CameraPipelineFrame::BayerNone;
            CameraPipelineFramePtr frame(new CameraPipelineFrame);
            frame->m_image = m_asi.frameToImage(createPlaceholderFrame(), &bayerPattern);
            populateFrameExposureMetadata(*frame);
            frame->m_bayerPattern = bayerPattern;
            m_framePreprocessor->submitFrame(frame);
        }
    }
    else
    {
        setLastAsiError(getVideoError, CameraAsiController::errorCodeToString(getVideoError));
        qDebug() << "CameraWorker: ASIGetVideoData failed:" << getVideoError << CameraAsiController::errorCodeToString(getVideoError)
                 << "waitMs" << waitMs << "bufferSize" << m_asi.m_frameBuffer.size()
                 << "width" << m_asi.m_frameWidth << "height" << m_asi.m_frameHeight;
    }

    scheduleNextAsiVideoCapture(getVideoError == ASI_SUCCESS ? 0 : 10);

    PROFILER_STOP(__FUNCTION__);
}

void CameraWorker::asiCaptureTick()
{
    if (!m_capturing) {
        return;
    }

    if (!m_asi.m_settingsApplied && !asiApplyCameraSettings())
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
    m_asi.m_settingsApplied = false;
}

void CameraWorker::asiPollStatus()
{
    if (!m_settings.isAsiCamera()) {
        return;
    }

    const int cameraId = m_settings.cameraIdInt();
    long temperatureTenthsC = 0;
    ASI_BOOL isAuto = ASI_FALSE;
    bool temperatureValid = false;

    if ((cameraId >= 0) && asiOpenCamera())
    {
        const ASI_ERROR_CODE temperatureError = ASIGetControlValue(cameraId, ASI_TEMPERATURE, &temperatureTenthsC, &isAuto);
        if (temperatureError == ASI_SUCCESS)
        {
            setLastAsiError(ASI_SUCCESS, QString());
            m_asi.m_lastCcdTemperature = temperatureTenthsC / 10.0;
            m_asi.m_lastCcdTemperatureValid = true;
            temperatureValid = true;
        }
        else
        {
            setLastAsiError(temperatureError, CameraAsiController::errorCodeToString(temperatureError));
            m_asi.m_lastCcdTemperatureValid = false;
        }
    }
    else
    {
        m_asi.m_lastCcdTemperatureValid = false;
    }

    if (m_msgQueueToGUI)
    {
        m_msgQueueToGUI->push(MsgReportAlpacaStatus::create(
            m_capturing ? 1 : 0,
            m_asi.m_lastCcdTemperature,
            temperatureValid,
            m_asi.m_lastCaptureTimeMs,
            m_asi.m_imageType == ASI_IMG_RGB24 ? QStringLiteral("RGB24")
                : m_asi.m_imageType == ASI_IMG_RAW16 ? QStringLiteral("RAW16")
                : m_asi.m_imageType == ASI_IMG_RAW8 ? QStringLiteral("RAW8")
                : QStringLiteral("Y8"),
            m_asi.m_lastErrorNumber,
            m_asi.m_lastErrorMessage));
    }
}

void CameraWorker::setLastAsiError(int errorCode, const QString& errorMessage)
{
    m_asi.setLastError(errorCode, errorMessage);
}

#endif

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
