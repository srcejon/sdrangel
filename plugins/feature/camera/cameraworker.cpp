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
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QRandomGenerator>
#include <QSharedPointer>
#include <QThread>
#include <QtEndian>
#include <QUrl>
#include <QUrlQuery>
#include <QMetaEnum>

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
#include "camerafinder.h"
#include "cameraframealigner.h"
#include "camerapostprocessor.h"
#include "cameraworker.h"

#ifdef ASICAMERA_FOUND

QString CameraWorker::asiErrorCodeToString(ASI_ERROR_CODE errorCode)
{
    switch (errorCode)
    {
    case ASI_SUCCESS: return QStringLiteral("ASI_SUCCESS");
    case ASI_ERROR_INVALID_INDEX: return QStringLiteral("ASI_ERROR_INVALID_INDEX");
    case ASI_ERROR_INVALID_ID: return QStringLiteral("ASI_ERROR_INVALID_ID");
    case ASI_ERROR_INVALID_CONTROL_TYPE: return QStringLiteral("ASI_ERROR_INVALID_CONTROL_TYPE");
    case ASI_ERROR_CAMERA_CLOSED: return QStringLiteral("ASI_ERROR_CAMERA_CLOSED");
    case ASI_ERROR_CAMERA_REMOVED: return QStringLiteral("ASI_ERROR_CAMERA_REMOVED");
    case ASI_ERROR_INVALID_PATH: return QStringLiteral("ASI_ERROR_INVALID_PATH");
    case ASI_ERROR_INVALID_FILEFORMAT: return QStringLiteral("ASI_ERROR_INVALID_FILEFORMAT");
    case ASI_ERROR_INVALID_SIZE: return QStringLiteral("ASI_ERROR_INVALID_SIZE");
    case ASI_ERROR_INVALID_IMGTYPE: return QStringLiteral("ASI_ERROR_INVALID_IMGTYPE");
    case ASI_ERROR_OUTOF_BOUNDARY: return QStringLiteral("ASI_ERROR_OUTOF_BOUNDARY");
    case ASI_ERROR_TIMEOUT: return QStringLiteral("ASI_ERROR_TIMEOUT");
    case ASI_ERROR_INVALID_SEQUENCE: return QStringLiteral("ASI_ERROR_INVALID_SEQUENCE");
    case ASI_ERROR_BUFFER_TOO_SMALL: return QStringLiteral("ASI_ERROR_BUFFER_TOO_SMALL");
    case ASI_ERROR_VIDEO_MODE_ACTIVE: return QStringLiteral("ASI_ERROR_VIDEO_MODE_ACTIVE");
    case ASI_ERROR_EXPOSURE_IN_PROGRESS: return QStringLiteral("ASI_ERROR_EXPOSURE_IN_PROGRESS");
    case ASI_ERROR_GENERAL_ERROR: return QStringLiteral("ASI_ERROR_GENERAL_ERROR");
    case ASI_ERROR_INVALID_MODE: return QStringLiteral("ASI_ERROR_INVALID_MODE");
    case ASI_ERROR_GPS_NOT_SUPPORTED: return QStringLiteral("ASI_ERROR_GPS_NOT_SUPPORTED");
    case ASI_ERROR_GPS_VER_ERR: return QStringLiteral("ASI_ERROR_GPS_VER_ERR");
    case ASI_ERROR_GPS_FPGA_ERR: return QStringLiteral("ASI_ERROR_GPS_FPGA_ERR");
    case ASI_ERROR_GPS_PARAM_OUT_OF_RANGE: return QStringLiteral("ASI_ERROR_GPS_PARAM_OUT_OF_RANGE");
    case ASI_ERROR_GPS_DATA_INVALID: return QStringLiteral("ASI_ERROR_GPS_DATA_INVALID");
    default: return QStringLiteral("ASI_ERROR_UNKNOWN");
    }
}

bool CameraWorker::asiGetCameraInfoById(int cameraId, ASI_CAMERA_INFO& cameraInfo)
{
    const ASI_ERROR_CODE error = ASIGetCameraPropertyByID(cameraId, &cameraInfo);

    if (error != ASI_SUCCESS) {
        qDebug() << "CameraWorker: ASIGetCameraPropertyByID failed:" << error << asiErrorCodeToString(error);
    }

    return error == ASI_SUCCESS;
}

bool CameraWorker::asiGetControlCapsByType(int cameraId, ASI_CONTROL_TYPE controlType, ASI_CONTROL_CAPS& controlCaps)
{
    int numControls = 0;
    const ASI_ERROR_CODE numControlsError = ASIGetNumOfControls(cameraId, &numControls);

    if (numControlsError != ASI_SUCCESS) {
        qDebug() << "CameraWorker: ASIGetNumOfControls failed:" << numControlsError << asiErrorCodeToString(numControlsError);
        return false;
    }

    for (int controlIndex = 0; controlIndex < numControls; ++controlIndex)
    {
        ASI_CONTROL_CAPS candidate {};
        const ASI_ERROR_CODE controlCapsError = ASIGetControlCaps(cameraId, controlIndex, &candidate);

        if (controlCapsError != ASI_SUCCESS)
        {
            qDebug() << "CameraWorker: ASIGetControlCaps failed:" << controlCapsError << asiErrorCodeToString(controlCapsError)
                     << "controlIndex" << controlIndex;
            continue;
        }

        if (candidate.ControlType == controlType)
        {
            controlCaps = candidate;
            return true;
        }
    }

    return false;
}

bool CameraWorker::asiGetControlValueByType(int cameraId, ASI_CONTROL_TYPE controlType, long& value, ASI_BOOL& isAuto)
{
    const ASI_ERROR_CODE error = ASIGetControlValue(cameraId, controlType, &value, &isAuto);

    if (error != ASI_SUCCESS) {
        qDebug() << "CameraWorker: ASIGetControlValue failed:" << error << asiErrorCodeToString(error)
                 << "controlType" << static_cast<int>(controlType);
    }

    return error == ASI_SUCCESS;
}

bool CameraWorker::asiSupportsImageType(const ASI_CAMERA_INFO& cameraInfo, ASI_IMG_TYPE imageType)
{
    for (ASI_IMG_TYPE candidate : cameraInfo.SupportedVideoFormat)
    {
        if (candidate == ASI_IMG_END) {
            break;
        }

        if (candidate == imageType) {
            return true;
        }
    }

    return false;
}

int CameraWorker::asiBayerToOpenCvCode(int bayerPattern)
{
    switch (bayerPattern)
    {
    case ASI_BAYER_RG:
        return cv::COLOR_BayerBG2BGR;
    case ASI_BAYER_BG:
        return cv::COLOR_BayerRG2BGR;
    case ASI_BAYER_GR:
        return cv::COLOR_BayerGB2BGR;
    case ASI_BAYER_GB:
        return cv::COLOR_BayerGR2BGR;
    default:
        return cv::COLOR_BayerBG2BGR;
    }
}

CameraPipelineFrame::BayerPattern CameraWorker::asiBayerToPipelinePattern(int bayerPattern)
{
    switch (bayerPattern)
    {
    case ASI_BAYER_RG:
        return CameraPipelineFrame::BayerRGGB;
    case ASI_BAYER_BG:
        return CameraPipelineFrame::BayerBGGR;
    case ASI_BAYER_GR:
        return CameraPipelineFrame::BayerGRBG;
    case ASI_BAYER_GB:
        return CameraPipelineFrame::BayerGBRG;
    default:
        return CameraPipelineFrame::BayerRGGB;
    }
}

ASI_IMG_TYPE CameraWorker::asiSelectImageType(const ASI_CAMERA_INFO& cameraInfo) const
{
    if (cameraInfo.IsColorCam == ASI_TRUE)
    {
        const bool wantsRaw16 = m_settings.m_asiColorImageType == CameraSettings::AsiColorImageTypeRaw16;

        if (wantsRaw16 && asiSupportsImageType(cameraInfo, ASI_IMG_RAW16)) {
            return ASI_IMG_RAW16;
        }
        if (!wantsRaw16 && asiSupportsImageType(cameraInfo, ASI_IMG_RGB24)) {
            return ASI_IMG_RGB24;
        }
        if (asiSupportsImageType(cameraInfo, ASI_IMG_RGB24)) {
            return ASI_IMG_RGB24;
        }
        if (asiSupportsImageType(cameraInfo, ASI_IMG_RAW16)) {
            return ASI_IMG_RAW16;
        }
        if (asiSupportsImageType(cameraInfo, ASI_IMG_RAW8)) {
            return ASI_IMG_RAW8;
        }
        if (asiSupportsImageType(cameraInfo, ASI_IMG_Y8)) {
            return ASI_IMG_Y8;
        }
    }
    else
    {
        if (asiSupportsImageType(cameraInfo, ASI_IMG_RAW16)) {
            return ASI_IMG_RAW16;
        }
        if (asiSupportsImageType(cameraInfo, ASI_IMG_RAW8)) {
            return ASI_IMG_RAW8;
        }
        if (asiSupportsImageType(cameraInfo, ASI_IMG_Y8)) {
            return ASI_IMG_Y8;
        }
    }

    return ASI_IMG_Y8;
}

#endif

QImage CameraWorker::renderGrayscaleRaw(const QVector<QVector<int>>& raw, int width, int height, bool use16Bit)
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
    if (use16Bit)
    {
        const int uniformGray = (range == 0) ? (minValue > 0 ? 32768 : 0) : 0;
        QImage image(width, height, QImage::Format_Grayscale16);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const int value = (range > 0)
                    ? qBound(0, static_cast<int>(((raw[x][y] - minValue) * 65535.0) / range), 65535)
                    : uniformGray;
                reinterpret_cast<quint16*>(image.scanLine(y))[x] = static_cast<quint16>(value);
            }
        }
        return image;
    }

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
    m_frameAligner(nullptr),
    m_postProcessorInputMessageQueue(nullptr),
    m_availableDeviceHandler({}, QStringList{"spectrumview"}),
    m_capturing(false),
    m_capturingAudio(false),
    m_captureTimer(this),
    m_networkManager(nullptr),
    m_cameraFinder(new CameraFinder(this)),
    m_stackFrameIndex(0),
    m_hdrExposureIndex(0),
    m_alpacaFrameRequestPending(false),
    m_alpacaClientId(QRandomGenerator::global()->bounded(quint32(1), quint32(std::numeric_limits<quint32>::max()))),
    m_alpacaClientTransactionId(1),
    m_alpacaSensorType(0),
    m_alpacaCameraSizeX(0),
    m_alpacaCameraSizeY(0),
    m_alpacaBayerOffsetX(0),
    m_alpacaBayerOffsetY(0),
    m_alpacaImageBytesSupported(true),
    m_lastAlpacaErrorNumber(0),
    m_lastAlpacaErrorMessage(),
    m_lastAlpacaReceiveImageFormat(),
    m_alpacaConnected(false),
    m_alpacaConnectionPending(false),
    m_alpacaFocuserConnected(false),
    m_alpacaFocuserConnectionPending(false),
    m_alpacaFilterWheelConnected(false),
    m_alpacaFilterWheelConnectionPending(false),
    m_alpacaBootstrapPending(false),
    m_alpacaParamsInitialized(false),
    m_lastAlpacaBinX(0),
    m_lastAlpacaBinY(0),
    m_lastAlpacaNumX(0),
    m_lastAlpacaNumY(0),
    m_lastAlpacaEffectiveNumX(-1),
    m_lastAlpacaEffectiveNumY(-1),
    m_lastAlpacaStartX(0),
    m_lastAlpacaStartY(0),
    m_lastAlpacaGain(-1),
    m_lastAlpacaOffset(-1),
    m_lastAlpacaReadoutMode(0),
    m_statusTimer(this),
    m_lastAlpacaCaptureTimeMs(-1),
    m_spectrumPipeSource(nullptr)
#ifdef ASICAMERA_FOUND
    ,
    m_asiCameraOpen(false),
    m_asiVideoCaptureStarted(false),
    m_asiSettingsApplied(false),
    m_asiTriggerCamera(false),
    m_asiCameraSizeX(0),
    m_asiCameraSizeY(0),
    m_asiMaxBinX(1),
    m_asiMaxBinY(1),
    m_asiBayerPattern(ASI_BAYER_RG),
    m_asiColorCamera(false),
    m_asiBitDepth(8),
    m_asiImageType(ASI_IMG_Y8),
    m_asiRgb24Supported(false),
    m_asiRaw16Supported(false),
    m_asiPixelSizeUm(0.0),
    m_asiExposureMinMs(0.001),
    m_asiExposureMaxMs(60000.0),
    m_asiFrameWidth(0),
    m_asiFrameHeight(0),
    m_asiFrameBuffer(),
    m_lastAsiCcdTemperature(0.0),
    m_lastAsiCcdTemperatureValid(false),
    m_lastAsiCaptureTimeMs(-1),
    m_lastAsiErrorNumber(0),
    m_lastAsiErrorMessage()
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

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpacaConnected)
    {
        alpacaSetConnected(false);
    }

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpacaFocuserConnected)
    {
        alpacaSetFocuserConnected(false);
    }

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpacaFilterWheelConnected)
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
    m_alpacaConnected = false;
    m_alpacaConnectionPending = false;
    m_alpacaPendingConnectedContinuations.clear();
    m_alpacaBootstrapPending = false;
    m_alpacaPendingBootstrapContinuations.clear();
    m_lastAlpacaErrorNumber = 0;
    m_lastAlpacaErrorMessage.clear();
}

void CameraWorker::resetAlpacaFilterWheelConnectionState()
{
    m_alpacaFilterWheelConnected = false;
    m_alpacaFilterWheelConnectionPending = false;
    m_alpacaPendingFilterWheelConnectedContinuations.clear();
}

void CameraWorker::resetAlpacaFocuserConnectionState()
{
    m_alpacaFocuserConnected = false;
    m_alpacaFocuserConnectionPending = false;
    m_alpacaPendingFocuserConnectedContinuations.clear();
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
        m_asiSettingsApplied = false;
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
    return isHdrBracketingActive()
        ? m_settings.getHdrExposureTimeMs(currentHdrExposureIndex())
        : std::max(CameraSettings::m_minExposureTimeMs, m_settings.m_exposureTimeMs);
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
        m_asiSettingsApplied = false;
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

    const bool recapture = m_capturing && (
        cameraSourceChanged
        || (m_settings.isQtCamera() && (force || settingsKeys.contains("audioDeviceName")))
        || (m_settings.isAsiCamera() && captureModeChanged));

    if (recapture)
    {
        stopCapture();
        startCapture();
    }
    else if (m_capturing && (m_settings.isAlpacaCamera() || m_settings.isAsiCamera()) && captureCadenceChanged)
    {
        m_captureTimer.start(captureTimerIntervalMs());
    }

    const bool alpacaEndpointChanged = force
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("alpacaHost")
        || settingsKeys.contains("alpacaPort")
        || settingsKeys.contains("cameraId");
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
}

void CameraWorker::startCapture()
{
    if (m_capturing) {
        return;
    }

    resetHdrBracketState();
    m_capturing = true;
    m_lastAlpacaCaptureTimeMs = -1;
    m_alpacaCaptureTimer.invalidate();
    m_alpacaParamsInitialized = false;
    m_lastAlpacaBinX = m_settings.m_cameraBinX;
    m_lastAlpacaBinY = m_settings.m_cameraBinY;
    m_lastAlpacaNumX = m_settings.m_cameraNumX;
    m_lastAlpacaNumY = m_settings.m_cameraNumY;
    m_lastAlpacaEffectiveNumX = -1;
    m_lastAlpacaEffectiveNumY = -1;
    m_lastAlpacaStartX = m_settings.m_cameraStartX;
    m_lastAlpacaStartY = m_settings.m_cameraStartY;
    m_lastAlpacaGain = m_settings.m_cameraGain;
    m_lastAlpacaOffset = m_settings.m_cameraOffset;
    m_lastAlpacaReadoutMode = m_settings.m_cameraReadoutMode;

    if (m_settings.isAlpacaCamera())
    {
        m_alpacaCaptureTimer.start();
        m_alpacaFrameRequestPending = false;
        m_captureTimer.start(captureTimerIntervalMs());

        if (m_alpacaConnected && !m_alpacaBootstrapPending && (m_alpacaCameraSizeX > 0) && (m_alpacaCameraSizeY > 0)) {
            captureTick();
        } else {
            alpacaBootstrap();
        }
    }
#ifdef ASICAMERA_FOUND
    else if (m_settings.isAsiCamera())
    {
        invalidateAsiSettings();
        m_captureTimer.start(captureTimerIntervalMs());
        captureTick();
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
    m_alpacaCaptureTimer.invalidate();
    resetHdrBracketState();

#ifdef ASICAMERA_FOUND
    if (m_asiVideoCaptureStarted)
    {
        const ASI_ERROR_CODE stopVideoError = ASIStopVideoCapture(m_settings.cameraIdInt());
        if (stopVideoError != ASI_SUCCESS) {
            qDebug() << "CameraWorker: ASIStopVideoCapture failed:" << stopVideoError << asiErrorCodeToString(stopVideoError);
        }
        m_asiVideoCaptureStarted = false;
    }

    if (m_settings.isAsiCamera() && m_settings.isIntervalCaptureMode() && m_asiCameraOpen)
    {
        const ASI_ERROR_CODE stopExposureError = ASIStopExposure(m_settings.cameraIdInt());
        if ((stopExposureError != ASI_SUCCESS)
            && (stopExposureError != ASI_ERROR_GENERAL_ERROR)
            && (stopExposureError != ASI_ERROR_INVALID_MODE))
        {
            qDebug() << "CameraWorker: ASIStopExposure failed:" << stopExposureError << asiErrorCodeToString(stopExposureError);
        }
    }
    m_asiSettingsApplied = false;
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

    if (!m_networkManager || m_alpacaFrameRequestPending) {
        return;
    }

    if (!m_alpacaConnected || m_alpacaConnectionPending || m_alpacaBootstrapPending)
    {
        alpacaBootstrap();
        return;
    }

    if (!m_alpacaCaptureTimer.isValid()) {
        m_alpacaCaptureTimer.start();
    }
    if (useStackIntervalCadence()) {
        m_captureTimer.stop();
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
    bool logApi,
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

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    if (logApi) {
        qDebug() << "CameraWorker::AlpacaAPI request" << "PUT" << url.toString() << payload;
    }
    QNetworkReply *reply = nam->put(request, payload);

    QObject::connect(reply, &QNetworkReply::finished, reply, [reply, property, continuation, onSuccess, logApi, url]() {
        const QByteArray responseBody = reply->readAll();
        if (logApi) {
            qDebug() << "CameraWorker::AlpacaAPI response" << "PUT" << url.toString()
                     << "error" << reply->error() << reply->errorString() << responseBody;
        }
        if (reply->error() != QNetworkReply::NoError) {
            qDebug() << "CameraWorker: PUT" << property << "error:" << reply->errorString();
        } else if (onSuccess) {
            onSuccess();
        }
        reply->deleteLater();
        continuation();
    });
}

void CameraWorker::alpacaSetConnected(bool connected, std::function<void()> continuation)
{
    if (!m_networkManager) {
        return;
    }

    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.cameraIdInt();
    QUrl url(baseUrl + QString("/api/v1/camera/%1/connected").arg(camId));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? QStringLiteral("true") : QStringLiteral("false"));
    body.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));

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
            m_alpacaConnected = connected;
        }
        else if (connected)
        {
            m_alpacaConnected = false;
            m_alpacaFrameRequestPending = false;
        }

        if (!connected) {
            m_alpacaConnectionPending = false;
            m_alpacaConnected = false;
            m_alpacaPendingConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_alpacaConnectionPending = false;
            m_alpacaPendingConnectedContinuations.clear();
        }

        reply->deleteLater();
    });
}

void CameraWorker::alpacaRunWhenConnected(std::function<void()> continuation)
{
    if (m_alpacaConnected)
    {
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_alpacaPendingConnectedContinuations.append(continuation);
    }

    if (m_alpacaConnectionPending) {
        return;
    }

    m_alpacaConnectionPending = true;
    alpacaSetConnected(true, [this]() {
        m_alpacaConnectionPending = false;
        const auto continuations = std::move(m_alpacaPendingConnectedContinuations);
        m_alpacaPendingConnectedContinuations.clear();

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

    const QString baseUrl = buildAlpacaFocuserBaseUrl();
    const int deviceNumber = std::max(0, m_settings.m_alpacaFocuserDeviceNumber);
    QUrl url(baseUrl + QString("/api/v1/focuser/%1/connected").arg(deviceNumber));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? "true" : "false");
    body.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));

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
            m_alpacaFocuserConnected = connected;
        } else if (connected) {
            m_alpacaFocuserConnected = false;
        }

        if (!connected) {
            m_alpacaFocuserConnectionPending = false;
            m_alpacaFocuserConnected = false;
            m_alpacaPendingFocuserConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_alpacaFocuserConnectionPending = false;
            m_alpacaPendingFocuserConnectedContinuations.clear();
        }

        reply->deleteLater();
    });
}

void CameraWorker::alpacaRunFocuserWhenConnected(std::function<void()> continuation)
{
    if (m_alpacaFocuserConnected)
    {
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_alpacaPendingFocuserConnectedContinuations.append(continuation);
    }

    if (m_alpacaFocuserConnectionPending) {
        return;
    }

    m_alpacaFocuserConnectionPending = true;
    alpacaSetFocuserConnected(true, [this]() {
        m_alpacaFocuserConnectionPending = false;
        const auto continuations = std::move(m_alpacaPendingFocuserConnectedContinuations);
        m_alpacaPendingFocuserConnectedContinuations.clear();

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
        const QString baseUrl = buildAlpacaFocuserBaseUrl();
        const int deviceNumber = std::max(0, m_settings.m_alpacaFocuserDeviceNumber);
        QUrl url(baseUrl + QString("/api/v1/focuser/%1/move").arg(deviceNumber));
        QNetworkRequest request(url);
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

        QUrlQuery body;
        body.addQueryItem("Position", QString::number(std::max(0, m_settings.m_alpacaFocusPosition)));
        body.addQueryItem("ClientID", QString::number(m_alpacaClientId));
        body.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));

        const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
        logAlpacaRequest("PUT", url, payload);

        QNetworkReply *reply = m_networkManager->put(request, payload);
        QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply]() {
            const QByteArray responseBody = reply->readAll();
            logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);
            reply->deleteLater();
        });
    });
}

void CameraWorker::alpacaSetFilterWheelConnected(bool connected, std::function<void()> continuation)
{
    if (!m_networkManager) {
        return;
    }

    const QString baseUrl = buildAlpacaFilterWheelBaseUrl();
    const int deviceNumber = std::max(0, m_settings.m_alpacaFilterWheelDeviceNumber);
    QUrl url(baseUrl + QString("/api/v1/filterwheel/%1/connected").arg(deviceNumber));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    QUrlQuery body;
    body.addQueryItem("Connected", connected ? QStringLiteral("true") : QStringLiteral("false"));
    body.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));

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
            m_alpacaFilterWheelConnected = connected;
        } else if (connected) {
            m_alpacaFilterWheelConnected = false;
        }

        if (!connected) {
            m_alpacaFilterWheelConnectionPending = false;
            m_alpacaFilterWheelConnected = false;
            m_alpacaPendingFilterWheelConnectedContinuations.clear();
        }

        if (success)
        {
            if (continuation) {
                continuation();
            }
        }
        else if (connected)
        {
            m_alpacaFilterWheelConnectionPending = false;
            m_alpacaPendingFilterWheelConnectedContinuations.clear();
        }

        reply->deleteLater();
    });
}

void CameraWorker::alpacaRunFilterWheelWhenConnected(std::function<void()> continuation)
{
    if (m_alpacaFilterWheelConnected)
    {
        if (continuation) {
            continuation();
        }
        return;
    }

    if (continuation) {
        m_alpacaPendingFilterWheelConnectedContinuations.append(continuation);
    }

    if (m_alpacaFilterWheelConnectionPending) {
        return;
    }

    m_alpacaFilterWheelConnectionPending = true;
    alpacaSetFilterWheelConnected(true, [this]() {
        m_alpacaFilterWheelConnectionPending = false;
        const auto continuations = std::move(m_alpacaPendingFilterWheelConnectedContinuations);
        m_alpacaPendingFilterWheelConnectedContinuations.clear();

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
        const QString baseUrl = buildAlpacaFilterWheelBaseUrl();
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
            q.addQueryItem("ClientID", QString::number(m_alpacaClientId));
            q.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
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
        const QString baseUrl = buildAlpacaFilterWheelBaseUrl();
        const int deviceNumber = std::max(0, m_settings.m_alpacaFilterWheelDeviceNumber);
        QUrl url(baseUrl + QString("/api/v1/filterwheel/%1/position").arg(deviceNumber));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_alpacaClientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
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
        const QString baseUrl = buildAlpacaFilterWheelBaseUrl();
        const int deviceNumber = std::max(0, m_settings.m_alpacaFilterWheelDeviceNumber);
        QUrl url(baseUrl + QString("/api/v1/filterwheel/%1/position").arg(deviceNumber));
        QNetworkRequest request(url);
        request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

        QUrlQuery body;
        body.addQueryItem("Position", QString::number(std::max(0, m_settings.m_alpacaFilterWheelPosition)));
        body.addQueryItem("ClientID", QString::number(m_alpacaClientId));
        body.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));

        const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
        logAlpacaRequest("PUT", url, payload);

        QNetworkReply *reply = m_networkManager->put(request, payload);
        QObject::connect(reply, &QNetworkReply::finished, reply, [this, reply]() {
            const QByteArray responseBody = reply->readAll();
            logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);
            reply->deleteLater();
            alpacaWaitForFilterWheelPosition(20);
        });
    });
}

void CameraWorker::alpacaBootstrap(std::function<void()> continuation)
{
    if (!m_networkManager) {
        return;
    }

    if (continuation) {
        m_alpacaPendingBootstrapContinuations.append(continuation);
    }

    if (m_alpacaBootstrapPending) {
        return;
    }

    m_alpacaBootstrapPending = true;

    alpacaRunWhenConnected([this]() {
        if (!m_statusTimer.isActive()) {
            m_statusTimer.start(m_alpacaStatusPollIntervalMs);
        }

        alpacaQueryCameraCapabilities([this]() {
            m_alpacaBootstrapPending = false;
            alpacaPollStatus();

            const auto continuations = std::move(m_alpacaPendingBootstrapContinuations);
            m_alpacaPendingBootstrapContinuations.clear();

            for (const auto& continuation : continuations)
            {
                if (continuation) {
                    continuation();
                }
            }

            if (m_capturing && !m_alpacaFrameRequestPending) {
                captureTick();
            }
        });
    });
}

void CameraWorker::alpacaSetCameraParams()
{
    // Chain: binX -> binY -> subframe ROI -> gain -> offset -> readoutMode -> startExposure
    const QString baseUrl = buildAlpacaBaseUrl();
    const int camId = m_settings.cameraIdInt();
    const bool forceAllParams = !m_alpacaParamsInitialized;
    const int maxSubframeX = std::max(1, m_alpacaCameraSizeX / std::max(1, m_settings.m_cameraBinX));
    const int maxSubframeY = std::max(1, m_alpacaCameraSizeY / std::max(1, m_settings.m_cameraBinY));
    const bool fullFrameNumXRequested = (m_settings.m_cameraNumX == 0);
    const bool fullFrameNumYRequested = (m_settings.m_cameraNumY == 0);
    const bool canResolveNumX = !fullFrameNumXRequested || (m_alpacaCameraSizeX > 0);
    const bool canResolveNumY = !fullFrameNumYRequested || (m_alpacaCameraSizeY > 0);
    const int effectiveNumX = fullFrameNumXRequested
        ? std::max(1, maxSubframeX - std::max(0, m_settings.m_cameraStartX))
        : m_settings.m_cameraNumX;
    const int effectiveNumY = fullFrameNumYRequested
        ? std::max(1, maxSubframeY - std::max(0, m_settings.m_cameraStartY))
        : m_settings.m_cameraNumY;
    const bool setBinX = forceAllParams || (m_lastAlpacaBinX != m_settings.m_cameraBinX);
    const bool setBinY = forceAllParams || (m_lastAlpacaBinY != m_settings.m_cameraBinY);
    const bool setNumX = canResolveNumX
        && (forceAllParams || (m_lastAlpacaEffectiveNumX != effectiveNumX));
    const bool setNumY = canResolveNumY
        && (forceAllParams || (m_lastAlpacaEffectiveNumY != effectiveNumY));
    const bool setStartX = forceAllParams || (m_lastAlpacaStartX != m_settings.m_cameraStartX);
    const bool setStartY = forceAllParams || (m_lastAlpacaStartY != m_settings.m_cameraStartY);
    const bool setGain = (m_settings.m_cameraGain >= 0)
        && (forceAllParams || (m_lastAlpacaGain != m_settings.m_cameraGain));
    const bool setOffset = (m_settings.m_cameraOffset >= 0)
        && (forceAllParams || (m_lastAlpacaOffset != m_settings.m_cameraOffset));
    const bool setReadoutMode = forceAllParams || (m_lastAlpacaReadoutMode != m_settings.m_cameraReadoutMode);

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
                m_settings.m_cameraReadoutMode, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doStartExposure,
                [this]() { m_lastAlpacaReadoutMode = m_settings.m_cameraReadoutMode; });
        } else {
            doStartExposure();
        }
    };

    auto doOffset = [this, baseUrl, camId, doReadoutMode, setOffset]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (setOffset) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "offset", "Offset",
                m_settings.m_cameraOffset, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doReadoutMode,
                [this]() { m_lastAlpacaOffset = m_settings.m_cameraOffset; });
        } else {
            doReadoutMode();
        }
    };

    auto doGain = [this, baseUrl, camId, doOffset, setGain]() {
        if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
        if (setGain) {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "gain", "Gain",
                m_settings.m_cameraGain, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doOffset,
                [this]() { m_lastAlpacaGain = m_settings.m_cameraGain; });
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
                    m_settings.m_cameraStartY, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doGain,
                    [this]() { m_lastAlpacaStartY = m_settings.m_cameraStartY; });
            } else {
                doGain();
            }
        };

        std::function<void()> maybeSetNumYAfterStart = [this, baseUrl, camId, doGain, setNumY, effectiveNumY]() {
            if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
            if (setNumY) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numy", "NumY",
                    effectiveNumY, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doGain,
                    [this, effectiveNumY]() {
                        m_lastAlpacaNumY = m_settings.m_cameraNumY;
                        m_lastAlpacaEffectiveNumY = effectiveNumY;
                    });
            } else {
                doGain();
            }
        };

        if (setStartY && (m_settings.m_cameraStartY < m_lastAlpacaStartY))
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "starty", "StartY",
                m_settings.m_cameraStartY, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, maybeSetNumYAfterStart,
                [this]() { m_lastAlpacaStartY = m_settings.m_cameraStartY; });
        }
        else if (setNumY)
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numy", "NumY",
                effectiveNumY, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, maybeSetStartYAfterNum,
                [this, effectiveNumY]() {
                    m_lastAlpacaNumY = m_settings.m_cameraNumY;
                    m_lastAlpacaEffectiveNumY = effectiveNumY;
                });
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
                    m_settings.m_cameraStartX, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doAxisY,
                    [this]() { m_lastAlpacaStartX = m_settings.m_cameraStartX; });
            } else {
                doAxisY();
            }
        };

        std::function<void()> maybeSetNumXAfterStart = [this, baseUrl, camId, doAxisY, setNumX, effectiveNumX]() {
            if (!m_capturing) { m_alpacaFrameRequestPending = false; return; }
            if (setNumX) {
                alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numx", "NumX",
                    effectiveNumX, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doAxisY,
                    [this, effectiveNumX]() {
                        m_lastAlpacaNumX = m_settings.m_cameraNumX;
                        m_lastAlpacaEffectiveNumX = effectiveNumX;
                    });
            } else {
                doAxisY();
            }
        };

        if (setStartX && (m_settings.m_cameraStartX < m_lastAlpacaStartX))
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "startx", "StartX",
                m_settings.m_cameraStartX, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, maybeSetNumXAfterStart,
                [this]() { m_lastAlpacaStartX = m_settings.m_cameraStartX; });
        }
        else if (setNumX)
        {
            alpacaPutIntProperty(m_networkManager, baseUrl, camId, "numx", "NumX",
                effectiveNumX, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, maybeSetStartXAfterNum,
                [this, effectiveNumX]() {
                    m_lastAlpacaNumX = m_settings.m_cameraNumX;
                    m_lastAlpacaEffectiveNumX = effectiveNumX;
                });
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
                m_settings.m_cameraBinY, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doAxisX,
                [this]() { m_lastAlpacaBinY = m_settings.m_cameraBinY; });
        } else {
            doAxisX();
        }
    };

    if (setBinX) {
        alpacaPutIntProperty(m_networkManager, baseUrl, camId, "binx", "BinX",
            m_settings.m_cameraBinX, m_alpacaClientId, m_alpacaClientTransactionId, m_settings.m_alpacaApiLogEnabled, doBinY,
            [this]() { m_lastAlpacaBinX = m_settings.m_cameraBinX; });
    } else {
        doBinY();
    }
}

void CameraWorker::alpacaStartExposure()
{
    QUrl url(buildAlpacaBaseUrl() + QString("/api/v1/camera/%1/startexposure").arg(m_settings.cameraIdInt()));
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");

    const double exposureTimeMs = currentCaptureExposureTimeMs();
    const double durationSecs = exposureTimeMs / 1000.0;
    QUrlQuery body;
    body.addQueryItem("Duration", QString::number(durationSecs, 'f', 6)); // 6 needed for microsecond precision
    body.addQueryItem("Light", "True");
    body.addQueryItem("ClientID", QString::number(m_alpacaClientId));
    body.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));

    const QByteArray payload = body.toString(QUrl::FullyEncoded).toUtf8();
    logAlpacaRequest("PUT", url, payload);
    QNetworkReply *reply = m_networkManager->put(request, payload);

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply, exposureTimeMs]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("PUT", reply->request().url(), reply, responseBody);
        reply->deleteLater();

        if (!m_capturing) {
            m_alpacaFrameRequestPending = false;
            return;
        }

        if (reply->error() != QNetworkReply::NoError) {
            qDebug() << "CameraWorker::alpacaStartExposure: error:" << reply->errorString();
            m_alpacaFrameRequestPending = false;
            scheduleNextCaptureAfterFailure();
            return;
        }

        // Wait for the exposure duration before polling imageready
        QTimer::singleShot(static_cast<int>(std::ceil(exposureTimeMs)), this, [this]() {
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

    logAlpacaRequest("GET", url);
    QNetworkReply *reply = m_networkManager->get(QNetworkRequest(url));

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        const QByteArray responseBody = reply->readAll();
        logAlpacaResponse("GET", reply->request().url(), reply, responseBody);
        reply->deleteLater();

        if (!m_capturing) {
            m_alpacaFrameRequestPending = false;
            return;
        }

        bool ready = false;
        if (reply->error() == QNetworkReply::NoError) {
            const QJsonDocument doc = QJsonDocument::fromJson(responseBody);
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

    logAlpacaRequest("GET", url);
    QNetworkReply *reply = m_networkManager->get(request);

    QObject::connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        m_alpacaFrameRequestPending = false;
        const QByteArray data = reply->readAll();
        logAlpacaResponse("GET", reply->request().url(), reply, data);

        if (!m_capturing) {
            reply->deleteLater();
            return;
        }

        QImage image = createPlaceholderFrame();
        CameraPipelineFrame::BayerPattern bayerPattern = CameraPipelineFrame::BayerNone;
        QString receiveImageFormat;

        if (reply->error() == QNetworkReply::NoError) {
            const QString contentType = reply->header(QNetworkRequest::ContentTypeHeader).toString();
            if (contentType.contains(QLatin1String("application/imagebytes"), Qt::CaseInsensitive)) {
                m_alpacaImageBytesSupported = true;
                image = parseAlpacaImageBytes(data, &receiveImageFormat, &bayerPattern);
            } else {
                // Server returned JSON — either it doesn't support ImageBytes or we didn't request it
                if (m_alpacaImageBytesSupported) {
                    qDebug() << "CameraWorker::alpacaFetchImageArray: server returned JSON; disabling ImageBytes for this camera";
                    m_alpacaImageBytesSupported = false;
                }
                image = parseAlpacaImageArray(data, &receiveImageFormat, &bayerPattern);
            }
        }

        m_lastAlpacaReceiveImageFormat = receiveImageFormat;

        if (m_alpacaCaptureTimer.isValid())
        {
            m_lastAlpacaCaptureTimeMs = m_alpacaCaptureTimer.elapsed();
            m_alpacaCaptureTimer.invalidate();
        }

        if (m_frameAligner) {
            CameraPipelineFramePtr frame(new CameraPipelineFrame);
            frame->m_image = image;
            populateFrameExposureMetadata(*frame);
            frame->m_bayerPattern = bayerPattern;
            m_frameAligner->submitFrame(frame);
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
    m_alpacaImageBytesSupported = true;

    const QString baseUrl = buildAlpacaBaseUrl();
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

        m_alpacaSensorType = info->sensorType;
        m_alpacaCameraSizeX = std::max(0, info->cameraSizeX);
        m_alpacaCameraSizeY = std::max(0, info->cameraSizeY);
        m_alpacaBayerOffsetX = info->bayerOffsetX;
        m_alpacaBayerOffsetY = info->bayerOffsetY;
        info->exposureMinMs = std::max(0.001, info->exposureMinMs);
        info->exposureResolutionMs = std::max(0.001, info->exposureResolutionMs);
        info->exposureMaxMs = std::max(info->exposureMinMs, info->exposureMaxMs);

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
        q.addQueryItem("ClientID", QString::number(m_alpacaClientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
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
                m_lastAlpacaCaptureTimeMs,
                m_lastAlpacaReceiveImageFormat,
                m_lastAlpacaErrorNumber,
                m_lastAlpacaErrorMessage));
        }
    };

    auto makeGet = [this, baseUrl, camId](const QString& prop) {
        QUrl url(baseUrl + QString("/api/v1/camera/%1/%2").arg(camId).arg(prop));
        QUrlQuery q;
        q.addQueryItem("ClientID", QString::number(m_alpacaClientId));
        q.addQueryItem("ClientTransactionID", QString::number(m_alpacaClientTransactionId++));
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

QString CameraWorker::buildAlpacaBaseUrl() const
{
    return QString("http://%1:%2")
        .arg(m_settings.m_alpacaHost)
        .arg(m_settings.m_alpacaPort);
}

QString CameraWorker::buildAlpacaFocuserBaseUrl() const
{
    return QString("http://%1:%2")
        .arg(m_settings.m_alpacaFocuserHost)
        .arg(m_settings.m_alpacaFocuserPort);
}

QString CameraWorker::buildAlpacaFilterWheelBaseUrl() const
{
    return QString("http://%1:%2")
        .arg(m_settings.m_alpacaFilterWheelHost)
        .arg(m_settings.m_alpacaFilterWheelPort);
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

    const QJsonDocument doc = QJsonDocument::fromJson(payload);

    if (doc.isObject())
    {
        const QJsonObject root = doc.object();

        if (root.contains(QStringLiteral("ErrorNumber")) || root.contains(QStringLiteral("ErrorMessage")))
        {
            alpacaPayloadParsed = true;
            alpacaErrorNumber = root.value(QStringLiteral("ErrorNumber")).toInt(0);
            alpacaErrorMessage = root.value(QStringLiteral("ErrorMessage")).toString();
        }
    }

    m_lastAlpacaErrorNumber = alpacaErrorNumber;
    m_lastAlpacaErrorMessage = alpacaErrorMessage;

    if (!m_settings.m_alpacaApiLogEnabled) {
        return;
    }

    if (path.endsWith(QStringLiteral("/imagearray"), Qt::CaseInsensitive))
    {
        qDebug() << "CameraWorker::AlpacaAPI response" << method << url.toString()
                 << transportError(reply)
                 << "alpacaError" << alpacaErrorNumber << alpacaErrorMessage
                 << QString("<imagearray payload of %1 bytes omitted>").arg(payload.size());
        return;
    }

    if (alpacaPayloadParsed)
    {
        qDebug() << "CameraWorker::AlpacaAPI response" << method << url.toString()
                 << transportError(reply)
                 << "alpacaError" << alpacaErrorNumber << alpacaErrorMessage
                 << payload;
    }
    else
    {
        qDebug() << "CameraWorker::AlpacaAPI response" << method << url.toString()
                 << transportError(reply)
                 << payload;
    }
}

QString CameraWorker::transportError(QNetworkReply *reply) const
{
    if (reply->error() == QNetworkReply::NoError) {
        return "";
    } else {
        return QString("%1 %2").arg(QMetaEnum::fromType<QNetworkReply::NetworkError>().valueToKey(reply->error())).arg(reply->errorString());
    }
}

QImage CameraWorker::parseAlpacaImageArray(const QByteArray& payload, QString *receiveImageFormat,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    if (bayerPattern) {
        *bayerPattern = CameraPipelineFrame::BayerNone;
    }

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

        // First pass: find minimum and maximum pixel values for black-level correction and scaling.
        int minVal = std::numeric_limits<int>::max();
        int maxVal = std::numeric_limits<int>::min();
        for (const QJsonValue& col : value) {
            for (const QJsonValue& pix : col.toArray()) {
                const int v = pix.toInt(0);
                if (v < minVal) { minVal = v; }
                if (v > maxVal) { maxVal = v; }
            }
        }
        const bool use16Bit = (minVal < 0) || (maxVal > 255);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageArray rank2 %1")
                .arg(use16Bit ? QStringLiteral("16-bit") : QStringLiteral("8-bit"));
        }

        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            const QJsonArray col = value[x].toArray();
            for (int y = 0; y < height; ++y) {
                raw[x][y] = col[y].toInt(0);
            }
        }

        // Bayer demosaicing for sensorType 2 (RGGB), 3 (CMYG), 4 (CMYG2), 5 (LRGB)
        // sensorType 0 = Monochrome, 1 = Colour (handled by rank 3 normally)
        return renderRawPixelArray(raw, width, height, use16Bit, bayerPattern);
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

        // First pass: find minimum and maximum pixel values for black-level correction and scaling.
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
        const bool use16Bit = (minVal < 0) || (maxVal > 255);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageArray rank3 %1")
                .arg(use16Bit ? QStringLiteral("16-bit") : QStringLiteral("8-bit"));
        }
        const double scale = (range3 > 0) ? ((use16Bit ? 65535.0 : 255.0) / range3) : 0.0;
        const int uniformGray3 = (range3 == 0) ? (minVal > 0 ? (use16Bit ? 32768 : 128) : 0) : 0;

        QImage image(width, height, use16Bit ? QImage::Format_RGBA64 : QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            const QJsonArray colR = planeR[x].toArray();
            const QJsonArray colG = planeG[x].toArray();
            const QJsonArray colB = planeB[x].toArray();
            for (int y = 0; y < height; ++y) {
                const int maxComponent = use16Bit ? 65535 : 255;
                const int r = (range3 > 0) ? qBound(0, static_cast<int>((colR[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;
                const int g = (range3 > 0) ? qBound(0, static_cast<int>((colG[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;
                const int b = (range3 > 0) ? qBound(0, static_cast<int>((colB[y].toInt(0) - minVal) * scale), maxComponent) : uniformGray3;

                if (use16Bit) {
                    reinterpret_cast<QRgba64*>(image.scanLine(y))[x] = qRgba64(r, g, b, 65535);
                } else {
                    reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
                }
            }
        }
        return image;
    }

    return createPlaceholderFrame();
}

QImage CameraWorker::renderRawPixelArray(const QVector<QVector<int>>& raw, int width, int height, bool use16Bit,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    int minValue = std::numeric_limits<int>::max();
    int maxValue = std::numeric_limits<int>::min();

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            minValue = std::min(minValue, raw[x][y]);
            maxValue = std::max(maxValue, raw[x][y]);
        }
    }

    if (m_alpacaSensorType == 2)
    {
        const int phaseX = ((m_alpacaBayerOffsetX % 2) + 2) % 2;
        const int phaseY = ((m_alpacaBayerOffsetY % 2) + 2) % 2;

        if (bayerPattern)
        {
            if ((phaseX == 0) && (phaseY == 0)) {
                *bayerPattern = CameraPipelineFrame::BayerRGGB;
            } else if ((phaseX == 1) && (phaseY == 0)) {
                *bayerPattern = CameraPipelineFrame::BayerGRBG;
            } else if ((phaseX == 0) && (phaseY == 1)) {
                *bayerPattern = CameraPipelineFrame::BayerGBRG;
            } else {
                *bayerPattern = CameraPipelineFrame::BayerBGGR;
            }
        }

        return renderGrayscaleRaw(raw, width, height, use16Bit);
    }

    // Monochrome (sensorType 0 or 1 returning rank 2, or unsupported Bayer types 3-5)
    return renderGrayscaleRaw(raw, width, height, use16Bit);
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
QImage CameraWorker::parseAlpacaImageBytes(const QByteArray& payload, QString *receiveImageFormat,
    CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    if (bayerPattern) {
        *bayerPattern = CameraPipelineFrame::BayerNone;
    }

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
    const qint32 imageElementType = qFromLittleEndian<qint32>(hdr + 20);
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

    auto elementTypeName = [](qint32 elementType) -> QString {
        switch (elementType) {
        case kElementTypeInt16: return QStringLiteral("Int16");
        case kElementTypeInt32: return QStringLiteral("Int32");
        case kElementTypeDouble: return QStringLiteral("Double");
        case kElementTypeSingle: return QStringLiteral("Single");
        case kElementTypeByte: return QStringLiteral("Byte");
        case kElementTypeUInt16: return QStringLiteral("UInt16");
        default: return QStringLiteral("Unknown");
        }
    };

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
        const bool use16Bit = (imageElementType == kElementTypeInt16)
                           || (imageElementType == kElementTypeUInt16)
                           || (transmissionType == kElementTypeInt16)
                           || (transmissionType == kElementTypeUInt16)
                           || (minVal < 0.0)
                           || (maxVal > 255.0);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageBytes rank2 %1/%2%3")
                .arg(elementTypeName(imageElementType),
                     elementTypeName(transmissionType),
                     use16Bit ? QStringLiteral(" 16-bit") : QString());
        }

        QVector<QVector<int>> raw(width, QVector<int>(height, 0));
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const double v = readPixelAsDouble(static_cast<qsizetype>(x * height + y) * elementSize);
                raw[x][y] = qRound(v);
            }
        }

        return renderRawPixelArray(raw, width, height, use16Bit, bayerPattern);
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
        const bool use16Bit = (imageElementType == kElementTypeInt16)
                           || (imageElementType == kElementTypeUInt16)
                           || (transmissionType == kElementTypeInt16)
                           || (transmissionType == kElementTypeUInt16)
                           || (minVal < 0.0)
                           || (maxVal > 255.0);
        if (receiveImageFormat) {
            *receiveImageFormat = QStringLiteral("ImageBytes rank3 %1/%2%3")
                .arg(elementTypeName(imageElementType),
                     elementTypeName(transmissionType),
                     use16Bit ? QStringLiteral(" 16-bit") : QString());
        }
        const double scale = (range3 > 0.0) ? ((use16Bit ? 65535.0 : 255.0) / range3) : 0.0;
        const int uniformGray3 = (range3 == 0.0) ? (minVal > 0.0 ? (use16Bit ? 32768 : 128) : 0) : 0;

        QImage image(width, height, use16Bit ? QImage::Format_RGBA64 : QImage::Format_RGB32);
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                const int maxComponent = use16Bit ? 65535 : 255;
                const int r = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(0, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;
                const int g = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(1, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;
                const int b = (range3 > 0.0)
                    ? qBound(0, static_cast<int>((pixelAt(2, x, y) - minVal) * scale), maxComponent)
                    : uniformGray3;

                if (use16Bit) {
                    reinterpret_cast<QRgba64*>(image.scanLine(y))[x] = qRgba64(r, g, b, 65535);
                } else {
                    reinterpret_cast<QRgb*>(image.scanLine(y))[x] = qRgb(r, g, b);
                }
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

#ifdef ASICAMERA_FOUND

bool CameraWorker::asiOpenCamera()
{
    if (m_asiCameraOpen) {
        return true;
    }

    const int cameraId = m_settings.cameraIdInt();

    if (cameraId < 0) {
        return false;
    }

    const ASI_ERROR_CODE openError = ASIOpenCamera(cameraId);
    if (openError != ASI_SUCCESS) {
        setLastAsiError(openError, asiErrorCodeToString(openError));
        qDebug() << "CameraWorker: ASIOpenCamera failed:" << openError << asiErrorCodeToString(openError);
        reportErrorToFeature(
            QStringLiteral("asiOpen:%1").arg(cameraId),
            tr("ASI camera open failed"),
            tr("Failed to open ASI camera %1:\n%2").arg(cameraId).arg(asiErrorCodeToString(openError)));
        return false;
    }

    const ASI_ERROR_CODE initError = ASIInitCamera(cameraId);
    if (initError != ASI_SUCCESS)
    {
        setLastAsiError(initError, asiErrorCodeToString(initError));
        qDebug() << "CameraWorker: ASIInitCamera failed:" << initError << asiErrorCodeToString(initError);
        reportErrorToFeature(
            QStringLiteral("asiInit:%1").arg(cameraId),
            tr("ASI camera initialization failed"),
            tr("Failed to initialize ASI camera %1:\n%2").arg(cameraId).arg(asiErrorCodeToString(initError)));
        const ASI_ERROR_CODE closeError = ASICloseCamera(cameraId);
        if (closeError != ASI_SUCCESS) {
            qDebug() << "CameraWorker: ASICloseCamera failed after init error:" << closeError << asiErrorCodeToString(closeError);
        }
        return false;
    }

    ASI_CAMERA_INFO cameraInfo {};
    const bool hasCameraInfo = asiGetCameraInfoById(cameraId, cameraInfo);
    if (hasCameraInfo) {
        m_asiTriggerCamera = cameraInfo.IsTriggerCam == ASI_TRUE;
    }

    if (m_asiTriggerCamera)
    {
        const ASI_ERROR_CODE modeError = ASISetCameraMode(cameraId, ASI_MODE_NORMAL);

        if (modeError != ASI_SUCCESS)
        {
            setLastAsiError(modeError, asiErrorCodeToString(modeError));
            qDebug() << "CameraWorker: ASISetCameraMode failed:" << modeError << asiErrorCodeToString(modeError);
            reportErrorToFeature(
                QStringLiteral("asiMode:%1").arg(cameraId),
                tr("ASI camera mode setup failed"),
                tr("Failed to set ASI camera %1 to normal mode:\n%2").arg(cameraId).arg(asiErrorCodeToString(modeError)));
            const ASI_ERROR_CODE closeError = ASICloseCamera(cameraId);
            if (closeError != ASI_SUCCESS) {
                qDebug() << "CameraWorker: ASICloseCamera failed after mode error:" << closeError << asiErrorCodeToString(closeError);
            }
            return false;
        }
    }

    setLastAsiError(ASI_SUCCESS, QString());
    m_asiCameraOpen = true;
    return true;
}

void CameraWorker::asiCloseCamera()
{
    const int cameraId = m_settings.cameraIdInt();

    if (m_asiVideoCaptureStarted && (cameraId >= 0))
    {
        const ASI_ERROR_CODE stopError = ASIStopVideoCapture(cameraId);
        if (stopError != ASI_SUCCESS) {
            qDebug() << "CameraWorker: ASIStopVideoCapture failed:" << stopError << asiErrorCodeToString(stopError);
        }
        m_asiVideoCaptureStarted = false;
    }

    if (m_asiCameraOpen && (cameraId >= 0))
    {
        const ASI_ERROR_CODE closeError = ASICloseCamera(cameraId);
        if (closeError != ASI_SUCCESS) {
            qDebug() << "CameraWorker: ASICloseCamera failed:" << closeError << asiErrorCodeToString(closeError);
        }
        m_asiCameraOpen = false;
    }

    m_asiSettingsApplied = false;
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

    if (!asiGetCameraInfoById(cameraId, cameraInfo)) {
        return;
    }

    ASI_CONTROL_CAPS gainRange {};
    ASI_CONTROL_CAPS offsetRange {};
    ASI_CONTROL_CAPS exposureRange {};
    ASI_CONTROL_CAPS coolerOnCaps {};
    ASI_CONTROL_CAPS targetTempCaps {};
    ASI_CONTROL_CAPS usbBandwidthCaps {};
    ASI_CONTROL_CAPS highSpeedModeCaps {};
    const bool hasGainRange = asiGetControlCapsByType(cameraId, ASI_GAIN, gainRange);
    const bool hasOffsetRange = asiGetControlCapsByType(cameraId, ASI_OFFSET, offsetRange);
    const bool hasExposureRange = asiGetControlCapsByType(cameraId, ASI_EXPOSURE, exposureRange);
    const bool hasCoolerOn = asiGetControlCapsByType(cameraId, ASI_COOLER_ON, coolerOnCaps);
    const bool hasTargetTemp = asiGetControlCapsByType(cameraId, ASI_TARGET_TEMP, targetTempCaps);
    const bool hasUsbBandwidth = asiGetControlCapsByType(cameraId, ASI_BANDWIDTHOVERLOAD, usbBandwidthCaps);
    const bool hasHighSpeedMode = asiGetControlCapsByType(cameraId, ASI_HIGH_SPEED_MODE, highSpeedModeCaps);
    long coolerOnValue = 0;
    long targetTempValue = 0;
    long usbBandwidthValue = 0;
    long highSpeedModeValue = 0;
    ASI_BOOL isAuto = ASI_FALSE;
    const bool hasCoolerOnValue = hasCoolerOn && asiGetControlValueByType(cameraId, ASI_COOLER_ON, coolerOnValue, isAuto);
    const bool hasTargetTempValue = hasTargetTemp && asiGetControlValueByType(cameraId, ASI_TARGET_TEMP, targetTempValue, isAuto);
    const bool hasUsbBandwidthValue = hasUsbBandwidth && asiGetControlValueByType(cameraId, ASI_BANDWIDTHOVERLOAD, usbBandwidthValue, isAuto);
    const bool hasHighSpeedModeValue = hasHighSpeedMode && asiGetControlValueByType(cameraId, ASI_HIGH_SPEED_MODE, highSpeedModeValue, isAuto);

    m_asiCameraSizeX = static_cast<int>(cameraInfo.MaxWidth);
    m_asiCameraSizeY = static_cast<int>(cameraInfo.MaxHeight);
    m_asiMaxBinX = 1;
    m_asiMaxBinY = 1;
    for (int bin : cameraInfo.SupportedBins)
    {
        if (bin <= 0) {
            break;
        }

        m_asiMaxBinX = std::max(m_asiMaxBinX, bin);
        m_asiMaxBinY = std::max(m_asiMaxBinY, bin);
    }
    m_asiBayerPattern = cameraInfo.BayerPattern;
    m_asiColorCamera = cameraInfo.IsColorCam == ASI_TRUE;
    m_asiTriggerCamera = cameraInfo.IsTriggerCam == ASI_TRUE;
    m_asiBitDepth = cameraInfo.BitDepth;
    m_asiPixelSizeUm = cameraInfo.PixelSize;
    m_asiExposureMinMs = hasExposureRange ? std::max(0.001, exposureRange.MinValue / 1000.0) : 0.001;
    m_asiExposureMaxMs = hasExposureRange ? std::max(m_asiExposureMinMs, exposureRange.MaxValue / 1000.0) : 60000.0;
    m_asiRgb24Supported = asiSupportsImageType(cameraInfo, ASI_IMG_RGB24);
    m_asiRaw16Supported = asiSupportsImageType(cameraInfo, ASI_IMG_RAW16);
    m_asiImageType = asiSelectImageType(cameraInfo);

    if (m_msgQueueToGUI)
    {
        m_msgQueueToGUI->push(MsgReportAsiCameraInfo::create(
            QString::fromUtf8(cameraInfo.Name),
            m_asiMaxBinX,
            m_asiMaxBinY,
            hasGainRange ? static_cast<int>(gainRange.MinValue) : 0,
            hasGainRange ? static_cast<int>(gainRange.MaxValue) : 100,
            hasOffsetRange ? static_cast<int>(offsetRange.MinValue) : 0,
            hasOffsetRange ? static_cast<int>(offsetRange.MaxValue) : 100,
            m_asiCameraSizeX,
            m_asiCameraSizeY,
            m_asiPixelSizeUm,
            m_asiBitDepth,
            m_asiColorCamera,
            m_asiExposureMinMs,
            m_asiExposureMaxMs,
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
            m_asiRgb24Supported,
            m_asiRaw16Supported));
    }

    asiPollStatus();
}

bool CameraWorker::asiApplyCameraSettings()
{
    if ((m_asiCameraSizeX <= 0) || (m_asiCameraSizeY <= 0)) {
        asiQueryCameraCapabilities();
    }

    if (!asiOpenCamera()) {
        return false;
    }

    const int cameraId = m_settings.cameraIdInt();
    ASI_CAMERA_INFO cameraInfo {};
    if (asiGetCameraInfoById(cameraId, cameraInfo)) {
        m_asiImageType = asiSelectImageType(cameraInfo);
    }
    const int bin = std::max(1, std::min(m_settings.m_cameraBinX, m_settings.m_cameraBinY));
    const int roiWidthStep = 8;
    const int roiHeightStep = 2;
    const int minWidth = 16;
    const int minHeight = 16;
    const int maxWidth = std::max(minWidth, m_asiCameraSizeX / std::max(1, bin));
    const int maxHeight = std::max(minHeight, m_asiCameraSizeY / std::max(1, bin));

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

    const ASI_ERROR_CODE roiError = ASISetROIFormat(cameraId, width, height, bin, static_cast<ASI_IMG_TYPE>(m_asiImageType));
    if (roiError != ASI_SUCCESS) {
        setLastAsiError(roiError, asiErrorCodeToString(roiError));
        qDebug() << "CameraWorker: ASISetROIFormat failed:" << roiError << asiErrorCodeToString(roiError)
                   << "width" << width << "height" << height << "bin" << bin << "imageType" << m_asiImageType;
        return false;
    }

    const ASI_ERROR_CODE startPosError = ASISetStartPos(cameraId, startX, startY);
    if (startPosError != ASI_SUCCESS) {
        setLastAsiError(startPosError, asiErrorCodeToString(startPosError));
        qDebug() << "CameraWorker: ASISetStartPos failed:" << startPosError << asiErrorCodeToString(startPosError)
                 << "startX" << startX << "startY" << startY << "width" << width << "height" << height << "bin" << bin;
        return false;
    }

    const ASI_BOOL autoExposureGain = (m_settings.m_asiAutoExposureGain
            && (m_settings.m_captureMode == CameraSettings::CaptureModeFrameRate))
        ? ASI_TRUE
        : ASI_FALSE;
    auto writableControl = [cameraId](ASI_CONTROL_TYPE controlType, ASI_CONTROL_CAPS *controlCaps = nullptr) -> bool {
        ASI_CONTROL_CAPS caps {};
        if (!asiGetControlCapsByType(cameraId, controlType, caps) || (caps.IsWritable != ASI_TRUE)) {
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
        setLastAsiError(exposureError, asiErrorCodeToString(exposureError));
        qDebug() << "CameraWorker: ASISetControlValue(EXPOSURE) failed:" << exposureError << asiErrorCodeToString(exposureError);
        return false;
    }

    if (gainError != ASI_SUCCESS) {
        setLastAsiError(gainError, asiErrorCodeToString(gainError));
        qDebug() << "CameraWorker: ASISetControlValue(GAIN) failed:" << gainError << asiErrorCodeToString(gainError);
        return false;
    }

    if (offsetError != ASI_SUCCESS) {
        setLastAsiError(offsetError, asiErrorCodeToString(offsetError));
        qDebug() << "CameraWorker: ASISetControlValue(OFFSET) failed:" << offsetError << asiErrorCodeToString(offsetError);
        return false;
    }
    if (coolerOnError != ASI_SUCCESS) {
        setLastAsiError(coolerOnError, asiErrorCodeToString(coolerOnError));
        qDebug() << "CameraWorker: ASISetControlValue(COOLER_ON) failed:" << coolerOnError << asiErrorCodeToString(coolerOnError);
        return false;
    }
    if (targetTempError != ASI_SUCCESS) {
        setLastAsiError(targetTempError, asiErrorCodeToString(targetTempError));
        qDebug() << "CameraWorker: ASISetControlValue(TARGET_TEMP) failed:" << targetTempError << asiErrorCodeToString(targetTempError);
        return false;
    }
    if (usbBandwidthError != ASI_SUCCESS) {
        setLastAsiError(usbBandwidthError, asiErrorCodeToString(usbBandwidthError));
        qDebug() << "CameraWorker: ASISetControlValue(BANDWIDTHOVERLOAD) failed:" << usbBandwidthError << asiErrorCodeToString(usbBandwidthError);
        return false;
    }
    if (highSpeedModeError != ASI_SUCCESS) {
        setLastAsiError(highSpeedModeError, asiErrorCodeToString(highSpeedModeError));
        qDebug() << "CameraWorker: ASISetControlValue(HIGH_SPEED_MODE) failed:" << highSpeedModeError << asiErrorCodeToString(highSpeedModeError);
        return false;
    }

    m_asiFrameWidth = width;
    m_asiFrameHeight = height;

    int bytesPerPixel = 1;
    switch (m_asiImageType)
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

    m_asiFrameBuffer.resize(width * height * bytesPerPixel);
    m_asiSettingsApplied = true;
    setLastAsiError(ASI_SUCCESS, QString());
    return true;
}

QImage CameraWorker::asiFrameToImage(CameraPipelineFrame::BayerPattern *bayerPattern) const
{
    if (bayerPattern) {
        *bayerPattern = CameraPipelineFrame::BayerNone;
    }

    if (m_asiFrameWidth <= 0 || m_asiFrameHeight <= 0 || m_asiFrameBuffer.isEmpty()) {
        return createPlaceholderFrame();
    }

    if (m_asiImageType == ASI_IMG_RGB24)
    {
        QImage image(m_asiFrameBuffer.constData(), m_asiFrameWidth, m_asiFrameHeight, m_asiFrameWidth * 3, QImage::Format_RGB888);
        return image.rgbSwapped();
    }

    if (m_asiImageType == ASI_IMG_Y8 || (!m_asiColorCamera && m_asiImageType == ASI_IMG_RAW8))
    {
        QImage image(m_asiFrameWidth, m_asiFrameHeight, QImage::Format_Grayscale8);
        for (int y = 0; y < m_asiFrameHeight; ++y) {
            std::memcpy(image.scanLine(y), m_asiFrameBuffer.constData() + (y * m_asiFrameWidth), static_cast<size_t>(m_asiFrameWidth));
        }
        return image;
    }

    cv::Mat rawMat;
    if (m_asiImageType == ASI_IMG_RAW16)
    {
        cv::Mat raw16(m_asiFrameHeight, m_asiFrameWidth, CV_16UC1, const_cast<uchar*>(m_asiFrameBuffer.constData()));

        if (!m_asiColorCamera)
        {
            QImage image(m_asiFrameWidth, m_asiFrameHeight, QImage::Format_Grayscale16);
            for (int y = 0; y < m_asiFrameHeight; ++y) {
                std::memcpy(image.scanLine(y), raw16.ptr(y), static_cast<size_t>(m_asiFrameWidth * sizeof(quint16)));
            }
            return image;
        }

        if (bayerPattern) {
            *bayerPattern = asiBayerToPipelinePattern(m_asiBayerPattern);
        }

        QImage image(m_asiFrameWidth, m_asiFrameHeight, QImage::Format_Grayscale16);
        for (int y = 0; y < m_asiFrameHeight; ++y) {
            std::memcpy(image.scanLine(y), raw16.ptr(y), static_cast<size_t>(m_asiFrameWidth * sizeof(quint16)));
        }
        return image;
    }
    else
    {
        rawMat = cv::Mat(m_asiFrameHeight, m_asiFrameWidth, CV_8UC1, const_cast<uchar*>(m_asiFrameBuffer.constData())).clone();
    }

    if (!m_asiColorCamera)
    {
        QImage image(m_asiFrameWidth, m_asiFrameHeight, QImage::Format_Grayscale8);
        for (int y = 0; y < m_asiFrameHeight; ++y) {
            std::memcpy(image.scanLine(y), rawMat.ptr(y), static_cast<size_t>(m_asiFrameWidth));
        }
        return image;
    }

    if (bayerPattern) {
        *bayerPattern = asiBayerToPipelinePattern(m_asiBayerPattern);
    }

    QImage image(m_asiFrameWidth, m_asiFrameHeight, QImage::Format_Grayscale8);
    for (int y = 0; y < m_asiFrameHeight; ++y) {
        std::memcpy(image.scanLine(y), rawMat.ptr(y), static_cast<size_t>(m_asiFrameWidth));
    }
    return image;
}

bool CameraWorker::asiCaptureExposureFrame()
{
    const int cameraId = m_settings.cameraIdInt();

    if (m_asiVideoCaptureStarted)
    {
        const ASI_ERROR_CODE stopVideoError = ASIStopVideoCapture(cameraId);
        if (stopVideoError != ASI_SUCCESS)
        {
            setLastAsiError(stopVideoError, asiErrorCodeToString(stopVideoError));
            qDebug() << "CameraWorker: ASIStopVideoCapture failed before exposure:" << stopVideoError << asiErrorCodeToString(stopVideoError);
            return false;
        }

        m_asiVideoCaptureStarted = false;
    }

    const ASI_ERROR_CODE startExposureError = ASIStartExposure(cameraId, ASI_FALSE);
    if (startExposureError != ASI_SUCCESS)
    {
        setLastAsiError(startExposureError, asiErrorCodeToString(startExposureError));
        qDebug() << "CameraWorker: ASIStartExposure failed:" << startExposureError << asiErrorCodeToString(startExposureError);
        reportErrorToFeature(
            QStringLiteral("asiStartExposure:%1").arg(cameraId),
            tr("ASI exposure start failed"),
            tr("Failed to start ASI exposure on camera %1:\n%2").arg(cameraId).arg(asiErrorCodeToString(startExposureError)));
        return false;
    }

    setLastAsiError(ASI_SUCCESS, QString());
    QElapsedTimer captureTimer;
    captureTimer.start();

    const qint64 timeoutMs = std::max<qint64>(1000, static_cast<qint64>(std::ceil(currentCaptureExposureTimeMs())) + 5000);
    ASI_EXPOSURE_STATUS exposureStatus = ASI_EXP_IDLE;

    while (captureTimer.elapsed() <= timeoutMs)
    {
        const ASI_ERROR_CODE statusError = ASIGetExpStatus(cameraId, &exposureStatus);
        if (statusError != ASI_SUCCESS)
        {
            setLastAsiError(statusError, asiErrorCodeToString(statusError));
            qDebug() << "CameraWorker: ASIGetExpStatus failed:" << statusError << asiErrorCodeToString(statusError);
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

        QThread::msleep(50);
    }

    if (exposureStatus != ASI_EXP_SUCCESS)
    {
        setLastAsiError(ASI_ERROR_TIMEOUT, asiErrorCodeToString(ASI_ERROR_TIMEOUT));
        qDebug() << "CameraWorker: ASI exposure timed out after" << timeoutMs << "ms";
        return false;
    }

    const ASI_ERROR_CODE dataError = ASIGetDataAfterExp(cameraId, m_asiFrameBuffer.data(), m_asiFrameBuffer.size());
    if (dataError != ASI_SUCCESS)
    {
        setLastAsiError(dataError, asiErrorCodeToString(dataError));
        qDebug() << "CameraWorker: ASIGetDataAfterExp failed:" << dataError << asiErrorCodeToString(dataError)
                 << "bufferSize" << m_asiFrameBuffer.size()
                 << "width" << m_asiFrameWidth << "height" << m_asiFrameHeight;
        return false;
    }

    setLastAsiError(ASI_SUCCESS, QString());
    m_lastAsiCaptureTimeMs = captureTimer.elapsed();
    if (m_frameAligner) {
        CameraPipelineFrame::BayerPattern bayerPattern = CameraPipelineFrame::BayerNone;
        CameraPipelineFramePtr frame(new CameraPipelineFrame);
        frame->m_image = asiFrameToImage(&bayerPattern);
        populateFrameExposureMetadata(*frame);
        frame->m_bayerPattern = bayerPattern;
        m_frameAligner->submitFrame(frame);
    }
    advanceStackBurstState();
    scheduleNextCaptureAfterFrame();
    return true;
}

void CameraWorker::asiCaptureVideoFrame()
{
    const int cameraId = m_settings.cameraIdInt();

    if (!m_asiVideoCaptureStarted)
    {
        const ASI_ERROR_CODE startCaptureError = ASIStartVideoCapture(cameraId);
        if (startCaptureError != ASI_SUCCESS) {
            setLastAsiError(startCaptureError, asiErrorCodeToString(startCaptureError));
            qDebug() << "CameraWorker: ASIStartVideoCapture failed:" << startCaptureError << asiErrorCodeToString(startCaptureError);
            reportErrorToFeature(
                QStringLiteral("asiStartVideo:%1").arg(cameraId),
                tr("ASI video capture start failed"),
                tr("Failed to start ASI video capture on camera %1:\n%2").arg(cameraId).arg(asiErrorCodeToString(startCaptureError)));
            return;
        }
        setLastAsiError(ASI_SUCCESS, QString());
        m_asiVideoCaptureStarted = true;
    }

    const int waitMs = std::max(1000, static_cast<int>(std::ceil(currentCaptureExposureTimeMs())) + 500);
    QElapsedTimer captureTimer;
    captureTimer.start();
    const ASI_ERROR_CODE getVideoError = ASIGetVideoData(cameraId, m_asiFrameBuffer.data(), m_asiFrameBuffer.size(), waitMs);
    if (getVideoError == ASI_SUCCESS)
    {
        setLastAsiError(ASI_SUCCESS, QString());
        m_lastAsiCaptureTimeMs = captureTimer.elapsed();
        if (m_frameAligner) {
            CameraPipelineFrame::BayerPattern bayerPattern = CameraPipelineFrame::BayerNone;
            CameraPipelineFramePtr frame(new CameraPipelineFrame);
            frame->m_image = asiFrameToImage(&bayerPattern);
            populateFrameExposureMetadata(*frame);
            frame->m_bayerPattern = bayerPattern;
            m_frameAligner->submitFrame(frame);
        }
    }
    else
    {
        setLastAsiError(getVideoError, asiErrorCodeToString(getVideoError));
        qDebug() << "CameraWorker: ASIGetVideoData failed:" << getVideoError << asiErrorCodeToString(getVideoError)
                 << "waitMs" << waitMs << "bufferSize" << m_asiFrameBuffer.size()
                 << "width" << m_asiFrameWidth << "height" << m_asiFrameHeight;
    }
}

void CameraWorker::asiCaptureTick()
{
    if (!m_capturing) {
        return;
    }

    if (!m_asiSettingsApplied && !asiApplyCameraSettings()) {
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
    m_asiSettingsApplied = false;
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
            m_lastAsiCcdTemperature = temperatureTenthsC / 10.0;
            m_lastAsiCcdTemperatureValid = true;
            temperatureValid = true;
        }
        else
        {
            setLastAsiError(temperatureError, asiErrorCodeToString(temperatureError));
            m_lastAsiCcdTemperatureValid = false;
        }
    }
    else
    {
        m_lastAsiCcdTemperatureValid = false;
    }

    if (m_msgQueueToGUI)
    {
        m_msgQueueToGUI->push(MsgReportAlpacaStatus::create(
            m_capturing ? 1 : 0,
            m_lastAsiCcdTemperature,
            temperatureValid,
            m_lastAsiCaptureTimeMs,
            m_asiImageType == ASI_IMG_RGB24 ? QStringLiteral("RGB24")
                : m_asiImageType == ASI_IMG_RAW16 ? QStringLiteral("RAW16")
                : m_asiImageType == ASI_IMG_RAW8 ? QStringLiteral("RAW8")
                : QStringLiteral("Y8"),
            m_lastAsiErrorNumber,
            m_lastAsiErrorMessage));
    }
}

void CameraWorker::setLastAsiError(int errorCode, const QString& errorMessage)
{
    m_lastAsiErrorNumber = errorCode;
    m_lastAsiErrorMessage = errorMessage;
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
