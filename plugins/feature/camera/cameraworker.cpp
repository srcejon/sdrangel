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
#include <array>
#include <cmath>
#include <functional>
#include <limits>

#include <QColor>
#include <QDebug>
#include <QDateTime>
#include <QNetworkAccessManager>

#include "maincore.h"
#include "util/profiler.h"
#include "camera.h"
#include "cameraalpacacontroller.h"
#include "cameraasicontroller.h"
#include "camerafinder.h"
#include "cameraframepreprocessor.h"
#include "camerapostprocessor.h"
#include "cameravideofiledecoder.h"
#include "cameraworker.h"

MESSAGE_CLASS_DEFINITION(CameraWorker::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgRefreshCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgStartAutoFocus, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgVideoFileControl, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportCameraList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportVideoFilePlayback, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaDeviceList, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaCameraInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAsiCameraInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaFilterWheelInfo, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAlpacaStatus, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAvailableDevices, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAutoExposureGain, Message)
MESSAGE_CLASS_DEFINITION(CameraWorker::MsgReportAutoFocus, Message)

CameraWorker::CameraWorker() :
    m_msgQueueToGUI(nullptr),
    m_msgQueueToFeature(nullptr),
    m_framePreprocessor(nullptr),
    m_postProcessorInputMessageQueue(nullptr),
    m_availableDeviceHandler({}, QStringList{"spectrumview"}),
    m_capturing(false),
    m_captureEpoch(0),
    m_captureTimer(this),
    m_videoFilePositionMs(0),
    m_videoFileDurationMs(0),
    m_videoFilePlaying(false),
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
    m_captureTimer.setTimerType(Qt::PreciseTimer);
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
        setControllerCameraConnected(false);
    }

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpaca.m_focuserConnected)
    {
        setControllerFocuserConnected(false);
    }

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpaca.m_filterWheelConnected)
    {
        setControllerFilterWheelConnected(false);
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
    frame.m_captureEpoch = m_captureEpoch;
    frame.m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
    frame.m_manualPreviewFrame = false;
    frame.m_exposureTimeMs = currentCaptureExposureTimeMs();
    frame.m_hdrExposureIndex = currentHdrExposureIndex();
    frame.m_hdrExposureCount = currentHdrExposureCount();
}

bool CameraWorker::measureAutoExposureGain(const QImage& image, double& measuredBrightness, double& saturatedFraction) const
{
    if (image.isNull() || image.width() <= 0 || image.height() <= 0) {
        return false;
    }

    std::array<int, 256> histogram = {};
    const int pixelCount = image.width() * image.height();
    const int stride = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(pixelCount) / 200000.0)));
    int samples = 0;
    int saturated = 0;

    for (int y = 0; y < image.height(); y += stride)
    {
        for (int x = 0; x < image.width(); x += stride)
        {
            const QColor color = image.pixelColor(x, y);
            const int luminance = qBound(0,
                static_cast<int>(std::lround(0.2126 * color.red() + 0.7152 * color.green() + 0.0722 * color.blue())),
                255);
            ++histogram[static_cast<size_t>(luminance)];
            ++samples;

            if (luminance >= 253) {
                ++saturated;
            }
        }
    }

    if (samples <= 0) {
        return false;
    }

    const int percentileSample = qBound(
        1,
        static_cast<int>(std::ceil(samples * qBound(50.0, m_settings.m_autoExposureTargetPercentile, 99.9) / 100.0)),
        samples);
    int cumulative = 0;
    int percentileValue = 0;
    for (size_t value = 0; value < histogram.size(); ++value)
    {
        cumulative += histogram[value];
        if (cumulative >= percentileSample)
        {
            percentileValue = static_cast<int>(value);
            break;
        }
    }

    measuredBrightness = static_cast<double>(percentileValue) / 255.0;
    saturatedFraction = static_cast<double>(saturated) / static_cast<double>(samples);
    return true;
}

void CameraWorker::reportAutoExposureGainToGUI(double measuredBrightness, double saturatedFraction) const
{
    if (!m_msgQueueToGUI) {
        return;
    }

    m_msgQueueToGUI->push(MsgReportAutoExposureGain::create(
        m_settings.m_exposureTimeMs,
        m_settings.m_cameraGain,
        measuredBrightness,
        saturatedFraction));
}

void CameraWorker::maybeAdjustAutoExposureGain(const CameraPipelineFrame& frame)
{
    if (!m_settings.m_autoExposureGainEnabled
        || isHdrBracketingActive()
        || (!m_settings.isAlpacaCamera() && !m_settings.isAsiCamera())
        || (currentStackBurstIndex() != (currentStackBurstFrameCount() - 1)))
    {
        m_autoExposure = AutoExposureState();
        return;
    }

    double measuredBrightness = 0.0;
    double saturatedFraction = 0.0;
    if (!measureAutoExposureGain(frame.m_image, measuredBrightness, saturatedFraction)) {
        return;
    }

    ++m_autoExposure.m_debugFrameCounter;
    const bool debugThisFrame = (m_autoExposure.m_debugFrameCounter <= 5)
        || ((m_autoExposure.m_debugFrameCounter % 10) == 0);

    if (m_autoExposure.m_settleFramesRemaining > 0)
    {
        --m_autoExposure.m_settleFramesRemaining;
        m_autoExposure.m_valid = false;
        if (debugThisFrame)
        {
            qDebug() << "CameraWorker::autoExposureGain"
                     << "decision" << "settle"
                     << "rawBrightness" << measuredBrightness
                     << "rawSaturated" << saturatedFraction
                     << "remaining" << m_autoExposure.m_settleFramesRemaining
                     << "exposureMs" << m_settings.m_exposureTimeMs
                     << "gain" << m_settings.m_cameraGain;
        }
        reportAutoExposureGainToGUI(measuredBrightness, saturatedFraction);
        return;
    }

    if (!m_autoExposure.m_valid)
    {
        m_autoExposure.m_brightness = measuredBrightness;
        m_autoExposure.m_saturatedFraction = saturatedFraction;
        m_autoExposure.m_valid = true;
    }
    else
    {
        static constexpr double smoothing = 0.10;
        m_autoExposure.m_brightness += smoothing * (measuredBrightness - m_autoExposure.m_brightness);
        m_autoExposure.m_saturatedFraction += smoothing * (saturatedFraction - m_autoExposure.m_saturatedFraction);
    }

    const double requestedTarget = qBound(0.01, m_settings.m_autoExposureTargetBrightness / 100.0, 0.995);
    const double target = requestedTarget;
    const double measured = qBound(0.001, m_autoExposure.m_brightness, 1.0);
    const double maxChange = qBound(0.01, m_settings.m_autoExposureMaxChangePercent / 100.0, 1.0);
    const double maxLogChange = std::log(1.0 + maxChange);
    const double error = std::log(target / measured);
    const double saturationHighLimit = target >= 0.95 ? 0.12 : 0.04;
    const double saturationLowLimit = target >= 0.95 ? 0.08 : 0.02;
    if (m_autoExposure.m_saturatedFraction > saturationHighLimit) {
        ++m_autoExposure.m_saturatedFrames;
    } else if ((m_autoExposure.m_saturatedFraction < saturationLowLimit) || (error > 0.0)) {
        m_autoExposure.m_saturatedFrames = 0;
    } else if (m_autoExposure.m_saturatedFrames > 0) {
        --m_autoExposure.m_saturatedFrames;
    }
    const bool saturated = (m_autoExposure.m_saturatedFraction > saturationHighLimit) && (m_autoExposure.m_saturatedFrames >= 3);
    const double deadband = error > 0.0 ? 0.03 : 0.08;
    const double correctionError = saturated
        ? error
        : std::copysign(std::max(0.0, std::abs(error) - deadband), error);

    auto modeName = [&]() -> const char*
    {
        switch (m_settings.m_autoExposureGainMode)
        {
        case CameraSettings::AutoExposureGainGainFirst:
            return "gain-first";
        case CameraSettings::AutoExposureGainExposureOnly:
            return "exposure-only";
        case CameraSettings::AutoExposureGainGainOnly:
            return "gain-only";
        case CameraSettings::AutoExposureGainExposureFirst:
        default:
            return "exposure-first";
        }
    };

    auto logAutoExposure = [&](const char *decision, double factor, double newExposureMs, int newGain)
    {
        qDebug() << "CameraWorker::autoExposureGain"
                 << "decision" << decision
                 << "mode" << modeName()
                 << "rawBrightness" << measuredBrightness
                 << "smoothBrightness" << m_autoExposure.m_brightness
                 << "requestedTarget" << requestedTarget
                 << "target" << target
                 << "rawSaturated" << saturatedFraction
                 << "smoothSaturated" << m_autoExposure.m_saturatedFraction
                 << "saturationLowLimit" << saturationLowLimit
                 << "saturationHighLimit" << saturationHighLimit
                 << "saturationFrames" << m_autoExposure.m_saturatedFrames
                 << "saturated" << saturated
                 << "error" << error
                 << "correctionError" << correctionError
                 << "deadband" << deadband
                 << "direction" << m_autoExposure.m_adjustDirection
                 << "directionFrames" << m_autoExposure.m_adjustDirectionFrames
                 << "factor" << factor
                 << "exposureMs" << m_settings.m_exposureTimeMs
                 << "newExposureMs" << newExposureMs
                 << "gain" << m_settings.m_cameraGain
                 << "newGain" << newGain
                 << "maxChange" << maxChange;
    };

    if (!saturated && (std::abs(error) < deadband))
    {
        m_autoExposure.m_adjustDirection = 0;
        m_autoExposure.m_adjustDirectionFrames = 0;
        if (debugThisFrame) {
            logAutoExposure("deadband", 1.0, m_settings.m_exposureTimeMs, m_settings.m_cameraGain);
        }
        reportAutoExposureGainToGUI(m_autoExposure.m_brightness, m_autoExposure.m_saturatedFraction);
        return;
    }

    const int adjustDirection = saturated ? -1 : (error > 0.0 ? 1 : -1);
    if (m_autoExposure.m_adjustDirection == adjustDirection) {
        ++m_autoExposure.m_adjustDirectionFrames;
    } else {
        m_autoExposure.m_adjustDirection = adjustDirection;
        m_autoExposure.m_adjustDirectionFrames = 1;
    }

    const int requiredDirectionFrames = error > 0.0 ? 3 : 5;
    if (!saturated && (std::abs(error) < 0.18) && (m_autoExposure.m_adjustDirectionFrames < requiredDirectionFrames))
    {
        if (debugThisFrame) {
            logAutoExposure("waiting-direction-confirm", 1.0, m_settings.m_exposureTimeMs, m_settings.m_cameraGain);
        }
        reportAutoExposureGainToGUI(m_autoExposure.m_brightness, m_autoExposure.m_saturatedFraction);
        return;
    }

    double factor = std::exp(qBound(-maxLogChange, correctionError * 0.35, maxLogChange));

    if (saturated)
    {
        const double excess = std::max(0.0, m_autoExposure.m_saturatedFraction - saturationHighLimit);
        const double severity = qBound(0.15, excess / std::max(0.001, saturationHighLimit), 0.6);
        factor = std::min(factor, std::exp(-maxLogChange * severity));
    }

    double exposureMinMs = std::max(CameraSettings::m_minExposureTimeMs, m_settings.m_autoExposureMinMs);
    double exposureMaxMs = std::max(exposureMinMs, m_settings.m_autoExposureMaxMs);
    if (m_settings.isAlpacaCamera())
    {
        exposureMinMs = std::max(exposureMinMs, m_alpaca.m_exposureMinMs);
        exposureMaxMs = std::min(exposureMaxMs, m_alpaca.m_exposureMaxMs);
    }
#ifdef ASICAMERA_FOUND
    else if (m_settings.isAsiCamera())
    {
        exposureMinMs = std::max(exposureMinMs, m_asi.exposureMinMs());
        exposureMaxMs = std::min(exposureMaxMs, m_asi.exposureMaxMs());
    }
#endif
    exposureMaxMs = std::max(exposureMinMs, exposureMaxMs);

    const int gainMin = std::max(0, m_settings.m_autoExposureMinGain);
    const int gainMax = std::max(gainMin, m_settings.m_autoExposureMaxGain);
    const int currentGain = qBound(gainMin, m_settings.m_cameraGain >= 0 ? m_settings.m_cameraGain : gainMin, gainMax);
    const double currentExposureMs = qBound(exposureMinMs, m_settings.m_exposureTimeMs, exposureMaxMs);
    double newExposureMs = currentExposureMs;
    int newGain = currentGain;

    auto adjustExposure = [&]() {
        static constexpr double minExposureLogChange = 0.005;
        if (std::abs(std::log(factor)) < minExposureLogChange) {
            return false;
        }
        const double proposed = qBound(exposureMinMs, newExposureMs * factor, exposureMaxMs);
        const bool changed = std::abs(proposed - newExposureMs) >= 0.0005;
        newExposureMs = proposed;
        return changed;
    };
    auto adjustGain = [&]() {
        if (gainMax <= gainMin) {
            return false;
        }
        const int gainRange = gainMax - gainMin;
        int delta = static_cast<int>(std::lround((factor - 1.0) * static_cast<double>(gainRange) * 0.25));
        if ((delta == 0) || ((std::abs(error) < 0.14) && !saturated)) {
            return false;
        }
        const int proposed = qBound(gainMin, newGain + delta, gainMax);
        const bool changed = proposed != newGain;
        newGain = proposed;
        return changed;
    };

    switch (m_settings.m_autoExposureGainMode)
    {
    case CameraSettings::AutoExposureGainGainFirst:
        if (!adjustGain()) {
            adjustExposure();
        }
        break;
    case CameraSettings::AutoExposureGainExposureOnly:
        adjustExposure();
        break;
    case CameraSettings::AutoExposureGainGainOnly:
        adjustGain();
        break;
    case CameraSettings::AutoExposureGainExposureFirst:
    default:
        if (!adjustExposure()) {
            adjustGain();
        }
        break;
    }

    const bool exposureChanged = std::abs(newExposureMs - m_settings.m_exposureTimeMs) >= 0.0005;
    const bool gainChanged = newGain != m_settings.m_cameraGain;

    if (exposureChanged || gainChanged)
    {
        logAutoExposure("adjust", factor, newExposureMs, newGain);
        m_settings.m_exposureTimeMs = newExposureMs;
        m_settings.m_cameraGain = newGain;
        m_autoExposure.m_settleFramesRemaining = 2;
#ifdef ASICAMERA_FOUND
        if (m_settings.isAsiCamera()) {
            invalidateAsiSettings();
        }
#endif
    }
    else if (debugThisFrame)
    {
        logAutoExposure("no-change", factor, newExposureMs, newGain);
    }

    reportAutoExposureGainToGUI(m_autoExposure.m_brightness, m_autoExposure.m_saturatedFraction);
}

void CameraWorker::reportAutoFocusToGUI(const QString& status, bool active, int position, double score, int stepIndex, int stepCount) const
{
    if (m_msgQueueToGUI) {
        m_msgQueueToGUI->push(MsgReportAutoFocus::create(status, active, position, score, stepIndex, stepCount));
    }
}

void CameraWorker::startAutoFocus()
{
    if (!m_settings.isAlpacaCamera() || !m_settings.m_alpacaFocuserEnabled)
    {
        reportAutoFocusToGUI(tr("Enable an Alpaca focuser first"), false, m_settings.m_alpacaFocusPosition, 0.0, 0, 0);
        return;
    }

    if (!m_capturing)
    {
        reportAutoFocusToGUI(tr("Start capture before auto focus"), false, m_settings.m_alpacaFocusPosition, 0.0, 0, 0);
        return;
    }

    const int stepSize = std::max(1, m_settings.m_alpacaFocusStepSize);
    const int center = std::max(0, m_settings.m_alpacaFocusPosition);
    QVector<int> positions;
    positions.reserve(7);
    for (int offset = -3; offset <= 3; ++offset)
    {
        const int position = std::max(0, center + offset * stepSize);
        if (!positions.contains(position)) {
            positions.append(position);
        }
    }
    std::sort(positions.begin(), positions.end());

    m_autoFocus.m_active = true;
    m_autoFocus.m_movePending = false;
    m_autoFocus.m_originalPosition = center;
    m_autoFocus.m_bestPosition = center;
    m_autoFocus.m_bestScore = -1.0;
    m_autoFocus.m_currentIndex = -1;
    m_autoFocus.m_positions = positions;

    reportAutoFocusToGUI(tr("Auto focus started"), true, center, 0.0, 0, positions.size());
    moveAutoFocusToNextPosition();
}

void CameraWorker::cancelAutoFocus(const QString& status)
{
    if (!m_autoFocus.m_active) {
        return;
    }

    const int position = m_autoFocus.m_bestScore >= 0.0 ? m_autoFocus.m_bestPosition : m_settings.m_alpacaFocusPosition;
    m_autoFocus = AutoFocusState();
    if (!status.isEmpty()) {
        reportAutoFocusToGUI(status, false, position, 0.0, 0, 0);
    }
}

void CameraWorker::moveAutoFocusToNextPosition()
{
    if (!m_autoFocus.m_active || m_autoFocus.m_movePending) {
        return;
    }

    ++m_autoFocus.m_currentIndex;
    if (m_autoFocus.m_currentIndex >= m_autoFocus.m_positions.size())
    {
        const int bestPosition = std::max(0, m_autoFocus.m_bestPosition);
        const double bestScore = m_autoFocus.m_bestScore;
        m_autoFocus.m_movePending = true;
        m_alpaca.moveFocuserToPosition(
            m_networkManager,
            m_settings,
            bestPosition,
            [this]() { reportAlpacaStatusToGUI(); },
            [this, bestPosition, bestScore]() {
                m_settings.m_alpacaFocusPosition = bestPosition;
                m_autoFocus = AutoFocusState();
                reportAutoFocusToGUI(tr("Auto focus complete"), false, bestPosition, bestScore, 0, 0);
            },
            [this]() {
                cancelAutoFocus(tr("Auto focus failed moving to best position"));
            });
        return;
    }

    const int position = m_autoFocus.m_positions.at(m_autoFocus.m_currentIndex);
    m_autoFocus.m_movePending = true;
    reportAutoFocusToGUI(tr("Moving focuser"), true, position, 0.0, m_autoFocus.m_currentIndex + 1, m_autoFocus.m_positions.size());
    m_alpaca.moveFocuserToPosition(
        m_networkManager,
        m_settings,
        position,
        [this]() { reportAlpacaStatusToGUI(); },
        [this, position]() {
            m_settings.m_alpacaFocusPosition = position;
            m_autoFocus.m_movePending = false;
            m_autoFocus.m_settleFramesRemaining = 1;
            reportAutoFocusToGUI(tr("Waiting for next frame"), true, position, 0.0, m_autoFocus.m_currentIndex + 1, m_autoFocus.m_positions.size());
        },
        [this]() {
            cancelAutoFocus(tr("Auto focus failed moving focuser"));
        });
}

void CameraWorker::sampleAutoFocusFrame(const CameraPipelineFrame& frame)
{
    if (!m_autoFocus.m_active || m_autoFocus.m_movePending || (m_autoFocus.m_currentIndex < 0)) {
        return;
    }

    const int position = m_autoFocus.m_positions.at(m_autoFocus.m_currentIndex);
    if (m_autoFocus.m_settleFramesRemaining > 0)
    {
        --m_autoFocus.m_settleFramesRemaining;
        reportAutoFocusToGUI(tr("Settling focuser"), true, position, 0.0, m_autoFocus.m_currentIndex + 1, m_autoFocus.m_positions.size());
        return;
    }

    const double score = measureAutoFocusScore(frame.m_image);
    if (score > m_autoFocus.m_bestScore)
    {
        m_autoFocus.m_bestScore = score;
        m_autoFocus.m_bestPosition = position;
    }

    reportAutoFocusToGUI(tr("Sampled focus"), true, position, score, m_autoFocus.m_currentIndex + 1, m_autoFocus.m_positions.size());
    moveAutoFocusToNextPosition();
}

double CameraWorker::measureAutoFocusScore(const QImage& image) const
{
    if (image.isNull()) {
        return 0.0;
    }

    cv::Mat gray;
    switch (image.format())
    {
    case QImage::Format_Grayscale8:
        gray = cv::Mat(image.height(), image.width(), CV_8UC1, const_cast<uchar*>(image.constBits()), image.bytesPerLine()).clone();
        break;
    case QImage::Format_Grayscale16:
    {
        cv::Mat gray16(image.height(), image.width(), CV_16UC1, const_cast<uchar*>(image.constBits()), image.bytesPerLine());
        gray16.convertTo(gray, CV_8UC1, 1.0 / 256.0);
        break;
    }
    default:
    {
        QImage rgb = image.convertToFormat(QImage::Format_RGB888);
        cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3, const_cast<uchar*>(rgb.constBits()), rgb.bytesPerLine());
        cv::cvtColor(rgbMat, gray, cv::COLOR_RGB2GRAY);
        break;
    }
    }

    if (gray.empty()) {
        return 0.0;
    }

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F, 3);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
}

bool CameraWorker::handleMessage(const Message& cmd)
{
    if (Camera::MsgConfigureCamera::match(cmd))
    {
        QMutexLocker locker(&m_mutex);
        const Camera::MsgConfigureCamera& cfg = (const Camera::MsgConfigureCamera&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgStartStop::match(cmd))
    {
        MsgStartStop& cfg = (MsgStartStop&) cmd;
        m_captureEpoch = cfg.getCaptureEpoch();
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
    else if (MsgStartAutoFocus::match(cmd))
    {
        startAutoFocus();
        return true;
    }
    else if (MsgVideoFileControl::match(cmd))
    {
        const MsgVideoFileControl& msg = (const MsgVideoFileControl&) cmd;
        switch (msg.getAction())
        {
        case MsgVideoFileControl::Play:
            setVideoFilePlaying(true);
            break;
        case MsgVideoFileControl::Pause:
            setVideoFilePlaying(false);
            break;
        case MsgVideoFileControl::Restart:
            seekVideoFile(0, true);
            setVideoFilePlaying(true);
            break;
        case MsgVideoFileControl::StepBack:
            stepVideoFile(-1);
            break;
        case MsgVideoFileControl::StepForward:
            stepVideoFile(1);
            break;
        case MsgVideoFileControl::Seek:
            seekVideoFile(msg.getPositionMs(), true);
            break;
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
        || (m_settings.isVideoFileCamera() && (force
            || settingsKeys.contains("videoFileCameraPath")
            || settingsKeys.contains("audioDeviceName")))
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
        disconnectControllerCamera(m_settings);
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (force || settingsKeys.contains("audioMute")) {
        m_qtAudio.setMuted(m_settings.m_audioMute);
    }
    if (force || settingsKeys.contains("videoPlaybackAudioOffsetMs")) {
        m_qtAudio.setFilePlaybackAudioOffsetMs(m_settings.m_videoPlaybackAudioOffsetMs);
    }

    if (hdrSettingsChanged) {
        resetHdrBracketState();
    }
    if (stackCadenceChanged) {
        m_stackFrameIndex = 0;
    }
    if (force
        || cameraSourceChanged
        || settingsKeys.contains("autoExposureGainEnabled")
        || settingsKeys.contains("autoExposureGainMode")
        || settingsKeys.contains("autoExposureTargetBrightness")
        || settingsKeys.contains("autoExposureTargetPercentile")
        || settingsKeys.contains("autoExposureMaxChangePercent")
        || settingsKeys.contains("autoExposureMinMs")
        || settingsKeys.contains("autoExposureMaxMs")
        || settingsKeys.contains("autoExposureMinGain")
        || settingsKeys.contains("autoExposureMaxGain"))
    {
        m_autoExposure = AutoExposureState();
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

    if (!recapture && m_capturing && m_settings.isVideoFileCamera()
        && (captureCadenceChanged || settingsKeys.contains("videoPlaybackRate")))
    {
        if (m_videoFilePlaying) {
            m_captureTimer.start(videoFileFrameIntervalMs());
        }
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
        bootstrapControllerCamera();
    }

    if (m_settings.isAlpacaCamera()
        && m_networkManager
        && m_settings.m_alpacaFocuserEnabled
        && (alpacaFocuserEndpointChanged || alpacaFocuserPositionChanged))
    {
        if (settingsKeys.contains("alpacaFocusPosition")) {
            moveControllerFocuser();
        }
    }

    if (m_settings.isAlpacaCamera()
        && m_networkManager
        && m_settings.m_alpacaFilterWheelEnabled
        && (alpacaFilterWheelEndpointChanged || alpacaFilterWheelPositionChanged))
    {
        if (alpacaFilterWheelEndpointChanged) {
            queryControllerFilterWheelInfo();
        } else if (settingsKeys.contains("alpacaFilterWheelPosition")) {
            setControllerFilterWheelPosition();
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
            || settingsKeys.contains("autoExposureGainEnabled")
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
    m_captureTimer.setSingleShot(false);
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

        if (m_alpaca.m_connected && !m_alpaca.bootstrapPending() && (m_alpaca.m_cameraSizeX > 0) && (m_alpaca.m_cameraSizeY > 0)) {
            captureTick();
        } else {
            bootstrapControllerCamera();
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
    else if (m_settings.isVideoFileCamera())
    {
        ++m_videoFileFrameSubmitGeneration;
        if (openVideoFileDecoder())
        {
            readVideoFileFrame();
            setVideoFilePlaying(true);
        }
        else
        {
            m_capturing = false;
        }
    }
}

void CameraWorker::stopCapture()
{
    m_capturing = false;
    ++m_videoFileFrameSubmitGeneration;
    cancelAutoFocus(tr("Auto focus cancelled"));
    m_captureTimer.stop();
    m_alpaca.m_captureTimer.invalidate();
    resetHdrBracketState();

    if (m_settings.isAlpacaCamera() && m_networkManager && m_alpaca.m_frameRequestPending) {
        abortControllerExposure();
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
    closeVideoFileDecoder();
}

void CameraWorker::captureTick()
{
    if (!m_capturing) {
        return;
    }

    if (m_settings.isVideoFileCamera())
    {
        readVideoFileFrame();
        if (m_capturing && m_videoFilePlaying) {
            scheduleNextVideoFileTick();
        }
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

    if (!m_alpaca.m_connected || m_alpaca.m_connectionPending || m_alpaca.bootstrapPending())
    {
        bootstrapControllerCamera();
        return;
    }

    if (!m_alpaca.m_captureTimer.isValid()) {
        m_alpaca.m_captureTimer.start();
    }
    if (useStackIntervalCadence()) {
        m_captureTimer.stop();
    }
    m_alpaca.m_frameRequestPending = true;
    applyControllerCameraParams();
}

bool CameraWorker::openVideoFileDecoder()
{
    closeVideoFileDecoder();

    if (!m_settings.isVideoFileCamera() || m_settings.m_videoFileCameraPath.isEmpty()) {
        return false;
    }

    const int audioOutputSampleRate = m_qtAudio.startFilePlayback(m_settings, getInputMessageQueue());
    m_videoFileDecoder.reset(new CameraVideoFileDecoder());
    QString errorMessage;
    if (!m_videoFileDecoder->open(m_settings.m_videoFileCameraPath, errorMessage, audioOutputSampleRate))
    {
        reportErrorToFeature(
            QStringLiteral("videoFileOpen:%1").arg(m_settings.m_videoFileCameraPath),
            tr("Video file could not be opened"),
            errorMessage);
        m_videoFileDecoder.reset();
        m_qtAudio.stop();
        reportVideoFilePlaybackToGUI();
        return false;
    }

    m_videoFilePositionMs = 0;
    m_videoFileDurationMs = m_videoFileDecoder->durationMs();
    m_videoFilePlaying = false;
    resetVideoFilePlaybackStats();
    reportVideoFilePlaybackToGUI();
    return true;
}

void CameraWorker::closeVideoFileDecoder()
{
    m_captureTimer.stop();
    m_videoFileDecoder.reset();
    m_videoFilePositionMs = 0;
    m_videoFileDurationMs = 0;
    m_videoFilePlaying = false;
    m_videoFilePlaybackClock.invalidate();
    m_videoFilePlaybackTick = 0;
    resetVideoFilePlaybackStats();
    if (m_settings.isVideoFileCamera()) {
        m_qtAudio.stop();
    }
    reportVideoFilePlaybackToGUI();
}

void CameraWorker::setVideoFilePlaying(bool playing)
{
    if (!m_capturing || !m_settings.isVideoFileCamera() || !m_videoFileDecoder)
    {
        m_videoFilePlaying = false;
        reportVideoFilePlaybackToGUI();
        return;
    }

    m_videoFilePlaying = playing;
    if (m_videoFilePlaying)
    {
        if (!m_videoFileStatsTimer.isValid()) {
            resetVideoFilePlaybackStats();
        }
        m_captureTimer.setSingleShot(true);
        resetVideoFilePlaybackSchedule();
        scheduleNextVideoFileTick();
    }
    else
    {
        m_captureTimer.stop();
        m_videoFilePlaybackClock.invalidate();
        m_videoFilePlaybackTick = 0;
    }
    reportVideoFilePlaybackToGUI();
}

void CameraWorker::submitVideoFileFrame(const CameraPipelineFramePtr& frame, bool applyPlaybackOffset)
{
    if (!frame || !m_framePreprocessor) {
        return;
    }

    const int videoDelayMs = (applyPlaybackOffset && (m_settings.m_videoPlaybackAudioOffsetMs < 0))
        ? qBound(0, -m_settings.m_videoPlaybackAudioOffsetMs, -CameraSettings::m_minVideoPlaybackAudioOffsetMs)
        : 0;

    if (videoDelayMs <= 0)
    {
        m_framePreprocessor->submitFrame(frame);
        return;
    }

    const quint64 captureEpoch = frame->m_captureEpoch;
    const quint64 submitGeneration = m_videoFileFrameSubmitGeneration;

    QTimer::singleShot(videoDelayMs, this, [this, frame, captureEpoch, submitGeneration]() {
        if (!m_capturing
            || !m_settings.isVideoFileCamera()
            || !m_videoFilePlaying
            || !m_framePreprocessor
            || (m_captureEpoch != captureEpoch)
            || (m_videoFileFrameSubmitGeneration != submitGeneration)) {
            return;
        }

        m_framePreprocessor->submitFrame(frame);
    });
}

void CameraWorker::readVideoFileFrame(bool submitAudio, qint64 minimumPositionMs)
{
    if (!m_capturing || !m_settings.isVideoFileCamera() || !m_videoFileDecoder) {
        return;
    }

    m_videoFileDecoder->setAudioPaceFrameRate(
        qMax(1.0, m_videoFileDecoder->frameRate()) * qMax(0.1, m_settings.m_videoPlaybackRate));

    QImage image;
    qint64 positionMs = -1;
    QByteArray pcmS16Stereo;
    int audioSampleRate = 0;
    QString errorMessage;
    QElapsedTimer decodeTimer;
    decodeTimer.start();
    const bool readOk = minimumPositionMs >= 0
        ? m_videoFileDecoder->readNextFrameAtOrAfter(minimumPositionMs, image, positionMs, errorMessage)
        : m_videoFileDecoder->readNextFrame(image, positionMs, pcmS16Stereo, audioSampleRate, errorMessage);
    const qint64 decodeMs = decodeTimer.elapsed();
    if (!readOk)
    {
        reportErrorToFeature(
            QStringLiteral("videoFileDecode:%1").arg(m_settings.m_videoFileCameraPath),
            tr("Video file decode failed"),
            errorMessage);
        setVideoFilePlaying(false);
        return;
    }

    if (image.isNull())
    {
        if (m_settings.m_videoLoop)
        {
            seekVideoFile(0, false);
            readVideoFileFrame();
        }
        else
        {
            setVideoFilePlaying(false);
        }
        return;
    }

    if (positionMs >= 0) {
        m_videoFilePositionMs = positionMs;
    } else {
        m_videoFilePositionMs += videoFileFrameIntervalMs();
    }

    if (submitAudio && !pcmS16Stereo.isEmpty())
    {
        m_qtAudio.submitMonitorPcmSamples(pcmS16Stereo, audioSampleRate);
        m_qtAudio.submitRecordingPcmSamples(pcmS16Stereo, audioSampleRate);
    }

    if (minimumPositionMs < 0) {
        updateVideoFilePlaybackStats(decodeMs, m_videoFilePositionMs, pcmS16Stereo.size());
    }

    if (m_framePreprocessor)
    {
        CameraPipelineFramePtr frame(new CameraPipelineFrame);
        frame->m_image = image;
        populateFrameExposureMetadata(*frame);
        frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        frame->m_playbackPositionMs = m_videoFilePositionMs;
        frame->m_playbackFrameRate = m_videoFileDecoder
            ? qMax(1.0, m_videoFileDecoder->frameRate()) * qMax(0.1, m_settings.m_videoPlaybackRate)
            : 0.0;
        submitVideoFileFrame(frame, submitAudio && (minimumPositionMs < 0));
    }

    reportVideoFilePlaybackToGUI();
}

void CameraWorker::seekVideoFile(qint64 positionMs, bool displayFrame)
{
    if (!m_videoFileDecoder) {
        return;
    }

    m_videoFilePositionMs = qBound<qint64>(0, positionMs, m_videoFileDurationMs > 0 ? m_videoFileDurationMs : std::numeric_limits<qint64>::max());
    ++m_videoFileFrameSubmitGeneration;
    m_videoFileDecoder->seek(m_videoFilePositionMs);
    m_qtAudio.clearMonitorAudio();
    m_qtAudio.prefillMonitorAudio(m_qtAudio.filePlaybackMonitorPrefillForOffsetMs());
    reportVideoFilePlaybackToGUI();
    if (displayFrame) {
        readVideoFileFrame(false, m_videoFilePositionMs);
    }
}

void CameraWorker::stepVideoFile(int direction)
{
    setVideoFilePlaying(false);
    ++m_videoFileFrameSubmitGeneration;
    if (direction >= 0)
    {
        readVideoFileFrame(false);
    }
    else
    {
        const qint64 maxPosition = m_videoFileDurationMs > 0 ? m_videoFileDurationMs : std::numeric_limits<qint64>::max();
        const qint64 position = qBound<qint64>(
            0,
            m_videoFilePositionMs - videoFileFrameIntervalMs(),
            maxPosition);
        seekVideoFile(position, true);
    }
}

int CameraWorker::videoFileFrameIntervalMs() const
{
    return qMax(1, static_cast<int>(std::round(videoFileExactFrameIntervalMs())));
}

double CameraWorker::videoFileExactFrameIntervalMs() const
{
    const double decoderFps = m_videoFileDecoder ? m_videoFileDecoder->frameRate() : m_settings.m_framesPerSecond;
    return 1000.0 / (qMax(1.0, decoderFps) * qMax(0.1, m_settings.m_videoPlaybackRate));
}

void CameraWorker::resetVideoFilePlaybackSchedule()
{
    m_videoFilePlaybackClock.restart();
    m_videoFilePlaybackTick = 1;
    m_videoFileTickTimer.restart();
}

void CameraWorker::scheduleNextVideoFileTick()
{
    if (!m_capturing || !m_videoFilePlaying || !m_settings.isVideoFileCamera() || !m_videoFileDecoder) {
        return;
    }

    const double intervalMs = qMax(1.0, videoFileExactFrameIntervalMs());
    if (!m_videoFilePlaybackClock.isValid()) {
        resetVideoFilePlaybackSchedule();
    }

    quint64 tick = m_videoFilePlaybackTick > 0 ? m_videoFilePlaybackTick : 1;
    const qint64 elapsedMs = m_videoFilePlaybackClock.elapsed();
    qint64 targetMs = static_cast<qint64>(std::llround(static_cast<double>(tick) * intervalMs));

    if (targetMs <= elapsedMs)
    {
        tick = static_cast<quint64>(std::floor(static_cast<double>(elapsedMs) / intervalMs)) + 1;
        targetMs = static_cast<qint64>(std::llround(static_cast<double>(tick) * intervalMs));
    }

    const qint64 delayMs = qMax<qint64>(1, targetMs - elapsedMs);
    const int timerDelayMs = static_cast<int>(qMin<qint64>(std::numeric_limits<int>::max(), delayMs));
    m_videoFilePlaybackTick = tick + 1;
    m_captureTimer.setSingleShot(true);
    m_captureTimer.start(timerDelayMs);
}

void CameraWorker::resetVideoFilePlaybackStats()
{
    m_videoFileStatsTimer.invalidate();
    m_videoFileTickTimer.invalidate();
    m_videoFileStatsFrames = 0;
    m_videoFileStatsEmptyAudioFrames = 0;
    m_videoFileStatsDecodeMsTotal = 0;
    m_videoFileStatsDecodeMsMax = 0;
    m_videoFileStatsTickDeltaMsTotal = 0;
    m_videoFileStatsTickDeltaMsMax = 0;
    m_videoFileStatsPositionDeltaMsTotal = 0;
    m_videoFileStatsPositionDeltaMsMin = 0;
    m_videoFileStatsPositionDeltaMsMax = 0;
    m_videoFileStatsLastPositionMs = -1;
    m_videoFileStatsAudioBytes = 0;
    m_videoFileStatsLastDroppedAudioFrames = m_qtAudio.monitorDroppedFrames();
    m_videoFileStatsLastAudioUnderflows = m_qtAudio.monitorUnderflows();

    if (m_videoFileDecoder)
    {
        const CameraVideoFileDecoder::DebugStats& stats = m_videoFileDecoder->debugStats();
        m_videoFileStatsLastDecoderReadAheadCalls = stats.m_readAheadCalls;
        m_videoFileStatsLastDecoderReadAheadPackets = stats.m_readAheadPackets;
        m_videoFileStatsLastDecoderReadAheadVideoPackets = stats.m_readAheadVideoPackets;
        m_videoFileStatsLastDecoderEagain = stats.m_sendVideoPacketEagain;
        m_videoFileStatsLastDecoderQueuedFrames = stats.m_queuedVideoFrames;
        m_videoFileStatsLastDecoderParkedVideoPackets = stats.m_parkedVideoPackets;
        m_videoFileStatsLastDecoderPacketCapHits = stats.m_readAheadPacketCapHits;
        m_videoFileStatsLastDecoderAudioBytes = stats.m_audioBytes;
        m_videoFileStatsLastDecoderPacedAudioCalls = stats.m_pacedAudioCalls;
        m_videoFileStatsLastDecoderPacedAudioTargetFrames = stats.m_pacedAudioTargetFrames;
        m_videoFileStatsLastDecoderPacedAudioOutputFrames = stats.m_pacedAudioOutputFrames;
        m_videoFileStatsLastDecoderPacedAudioShortCalls = stats.m_pacedAudioShortCalls;
        m_videoFileStatsLastDecoderAudioTimestampJumps = stats.m_audioTimestampJumps;
        m_videoFileStatsLastDecoderConvertFrames = stats.m_videoConvertFrames;
        m_videoFileStatsLastDecoderConvertMs = stats.m_videoConvertMs;
    }
    else
    {
        m_videoFileStatsLastDecoderReadAheadCalls = 0;
        m_videoFileStatsLastDecoderReadAheadPackets = 0;
        m_videoFileStatsLastDecoderReadAheadVideoPackets = 0;
        m_videoFileStatsLastDecoderEagain = 0;
        m_videoFileStatsLastDecoderQueuedFrames = 0;
        m_videoFileStatsLastDecoderParkedVideoPackets = 0;
        m_videoFileStatsLastDecoderPacketCapHits = 0;
        m_videoFileStatsLastDecoderAudioBytes = 0;
        m_videoFileStatsLastDecoderPacedAudioCalls = 0;
        m_videoFileStatsLastDecoderPacedAudioTargetFrames = 0;
        m_videoFileStatsLastDecoderPacedAudioOutputFrames = 0;
        m_videoFileStatsLastDecoderPacedAudioShortCalls = 0;
        m_videoFileStatsLastDecoderAudioTimestampJumps = 0;
        m_videoFileStatsLastDecoderConvertFrames = 0;
        m_videoFileStatsLastDecoderConvertMs = 0;
    }
    m_qtAudio.resetMonitorDebugStats();
}

void CameraWorker::updateVideoFilePlaybackStats(qint64 decodeMs, qint64 positionMs, qsizetype audioBytes)
{
    if (!m_videoFileStatsTimer.isValid()) {
        m_videoFileStatsTimer.start();
    }

    if (m_videoFileTickTimer.isValid())
    {
        const qint64 tickDeltaMs = m_videoFileTickTimer.restart();
        m_videoFileStatsTickDeltaMsTotal += tickDeltaMs;
        m_videoFileStatsTickDeltaMsMax = std::max(m_videoFileStatsTickDeltaMsMax, tickDeltaMs);
    }
    else
    {
        m_videoFileTickTimer.start();
    }

    ++m_videoFileStatsFrames;
    m_videoFileStatsDecodeMsTotal += decodeMs;
    m_videoFileStatsDecodeMsMax = std::max(m_videoFileStatsDecodeMsMax, decodeMs);
    m_videoFileStatsAudioBytes += static_cast<quint64>(std::max<qsizetype>(0, audioBytes));
    if (audioBytes <= 0) {
        ++m_videoFileStatsEmptyAudioFrames;
    }

    if ((positionMs >= 0) && (m_videoFileStatsLastPositionMs >= 0))
    {
        const qint64 deltaMs = positionMs - m_videoFileStatsLastPositionMs;
        m_videoFileStatsPositionDeltaMsTotal += deltaMs;
        if (m_videoFileStatsPositionDeltaMsMin == 0) {
            m_videoFileStatsPositionDeltaMsMin = deltaMs;
        } else {
            m_videoFileStatsPositionDeltaMsMin = std::min(m_videoFileStatsPositionDeltaMsMin, deltaMs);
        }
        m_videoFileStatsPositionDeltaMsMax = std::max(m_videoFileStatsPositionDeltaMsMax, deltaMs);
    }
    if (positionMs >= 0) {
        m_videoFileStatsLastPositionMs = positionMs;
    }

    maybeReportVideoFilePlaybackStats();
}

void CameraWorker::maybeReportVideoFilePlaybackStats()
{
    if (!m_videoFileStatsTimer.isValid() || (m_videoFileStatsTimer.elapsed() < 2000) || (m_videoFileStatsFrames == 0)) {
        return;
    }

    const double frames = static_cast<double>(m_videoFileStatsFrames);
    const double avgDecodeMs = static_cast<double>(m_videoFileStatsDecodeMsTotal) / frames;
    const double avgTickDeltaMs = static_cast<double>(m_videoFileStatsTickDeltaMsTotal) / frames;
    const double avgPositionDeltaMs = m_videoFileStatsFrames > 1
        ? static_cast<double>(m_videoFileStatsPositionDeltaMsTotal) / static_cast<double>(m_videoFileStatsFrames - 1)
        : 0.0;
    const quint64 droppedAudioFrames = m_qtAudio.monitorDroppedFrames();
    const quint64 droppedAudioDelta = droppedAudioFrames - m_videoFileStatsLastDroppedAudioFrames;
    const quint64 audioUnderflows = m_qtAudio.monitorUnderflows();
    const quint64 audioUnderflowDelta = audioUnderflows - m_videoFileStatsLastAudioUnderflows;
    const int pendingFrames = m_videoFileDecoder ? m_videoFileDecoder->pendingVideoFrameCount() : 0;
    const int pendingPackets = m_videoFileDecoder ? m_videoFileDecoder->pendingVideoPacketCount() : 0;
    const qint64 audioLeadMs = m_videoFileDecoder
        ? m_videoFileDecoder->audioDecodedPositionMs() - m_videoFilePositionMs
        : 0;
    const int pendingAudioBytes = m_videoFileDecoder ? m_videoFileDecoder->pendingAudioBytes() : 0;

    quint64 readAheadCalls = 0;
    quint64 readAheadPackets = 0;
    quint64 readAheadVideoPackets = 0;
    quint64 decoderEagain = 0;
    quint64 queuedFrames = 0;
    quint64 parkedVideoPackets = 0;
    quint64 packetCapHits = 0;
    quint64 decoderAudioBytes = 0;
    quint64 pacedAudioCalls = 0;
    quint64 pacedAudioTargetFrames = 0;
    quint64 pacedAudioOutputFrames = 0;
    quint64 pacedAudioShortCalls = 0;
    quint64 audioTimestampJumps = 0;
    qint64 audioTimestampJumpMaxAbsMs = 0;
    quint64 convertFrames = 0;
    quint64 convertMs = 0;
    qint64 convertMaxMs = 0;
    if (m_videoFileDecoder)
    {
        const CameraVideoFileDecoder::DebugStats& stats = m_videoFileDecoder->debugStats();
        readAheadCalls = stats.m_readAheadCalls - m_videoFileStatsLastDecoderReadAheadCalls;
        readAheadPackets = stats.m_readAheadPackets - m_videoFileStatsLastDecoderReadAheadPackets;
        readAheadVideoPackets = stats.m_readAheadVideoPackets - m_videoFileStatsLastDecoderReadAheadVideoPackets;
        decoderEagain = stats.m_sendVideoPacketEagain - m_videoFileStatsLastDecoderEagain;
        queuedFrames = stats.m_queuedVideoFrames - m_videoFileStatsLastDecoderQueuedFrames;
        parkedVideoPackets = stats.m_parkedVideoPackets - m_videoFileStatsLastDecoderParkedVideoPackets;
        packetCapHits = stats.m_readAheadPacketCapHits - m_videoFileStatsLastDecoderPacketCapHits;
        decoderAudioBytes = stats.m_audioBytes - m_videoFileStatsLastDecoderAudioBytes;
        pacedAudioCalls = stats.m_pacedAudioCalls - m_videoFileStatsLastDecoderPacedAudioCalls;
        pacedAudioTargetFrames = stats.m_pacedAudioTargetFrames - m_videoFileStatsLastDecoderPacedAudioTargetFrames;
        pacedAudioOutputFrames = stats.m_pacedAudioOutputFrames - m_videoFileStatsLastDecoderPacedAudioOutputFrames;
        pacedAudioShortCalls = stats.m_pacedAudioShortCalls - m_videoFileStatsLastDecoderPacedAudioShortCalls;
        audioTimestampJumps = stats.m_audioTimestampJumps - m_videoFileStatsLastDecoderAudioTimestampJumps;
        audioTimestampJumpMaxAbsMs = audioTimestampJumps > 0 ? stats.m_audioTimestampJumpMaxAbsMs : 0;
        convertFrames = stats.m_videoConvertFrames - m_videoFileStatsLastDecoderConvertFrames;
        convertMs = stats.m_videoConvertMs - m_videoFileStatsLastDecoderConvertMs;
        convertMaxMs = stats.m_videoConvertMaxMs;
        m_videoFileStatsLastDecoderReadAheadCalls = stats.m_readAheadCalls;
        m_videoFileStatsLastDecoderReadAheadPackets = stats.m_readAheadPackets;
        m_videoFileStatsLastDecoderReadAheadVideoPackets = stats.m_readAheadVideoPackets;
        m_videoFileStatsLastDecoderEagain = stats.m_sendVideoPacketEagain;
        m_videoFileStatsLastDecoderQueuedFrames = stats.m_queuedVideoFrames;
        m_videoFileStatsLastDecoderParkedVideoPackets = stats.m_parkedVideoPackets;
        m_videoFileStatsLastDecoderPacketCapHits = stats.m_readAheadPacketCapHits;
        m_videoFileStatsLastDecoderAudioBytes = stats.m_audioBytes;
        m_videoFileStatsLastDecoderPacedAudioCalls = stats.m_pacedAudioCalls;
        m_videoFileStatsLastDecoderPacedAudioTargetFrames = stats.m_pacedAudioTargetFrames;
        m_videoFileStatsLastDecoderPacedAudioOutputFrames = stats.m_pacedAudioOutputFrames;
        m_videoFileStatsLastDecoderPacedAudioShortCalls = stats.m_pacedAudioShortCalls;
        m_videoFileStatsLastDecoderAudioTimestampJumps = stats.m_audioTimestampJumps;
        m_videoFileStatsLastDecoderConvertFrames = stats.m_videoConvertFrames;
        m_videoFileStatsLastDecoderConvertMs = stats.m_videoConvertMs;
    }
    const CameraQtAudioController::MonitorDebugStats& monitorStats = m_qtAudio.monitorDebugStats();
    const double convertAvgMs = convertFrames > 0
        ? static_cast<double>(convertMs) / static_cast<double>(convertFrames)
        : 0.0;

    qDebug() << "CameraWorker: video playback stats"
             << "frames" << m_videoFileStatsFrames
             << "timerMs" << videoFileFrameIntervalMs()
             << "timerExactMs" << videoFileExactFrameIntervalMs()
             << "fps" << (m_videoFileDecoder ? m_videoFileDecoder->frameRate() : 0.0)
             << "rate" << m_settings.m_videoPlaybackRate
             << "decodeAvgMs" << avgDecodeMs
             << "decodeMaxMs" << m_videoFileStatsDecodeMsMax
             << "convertAvgMs" << convertAvgMs
             << "convertMaxMs" << convertMaxMs
             << "tickAvgMs" << avgTickDeltaMs
             << "tickMaxMs" << m_videoFileStatsTickDeltaMsMax
             << "posDeltaAvgMs" << avgPositionDeltaMs
             << "posDeltaMinMaxMs" << m_videoFileStatsPositionDeltaMsMin << m_videoFileStatsPositionDeltaMsMax
             << "audioBytes" << m_videoFileStatsAudioBytes
             << "emptyAudioFrames" << m_videoFileStatsEmptyAudioFrames
             << "monitorFill" << m_qtAudio.monitorAudioFill() << "/" << m_qtAudio.monitorAudioSize()
             << "monitorDroppedFrames" << droppedAudioDelta
             << "monitorUnderflows" << audioUnderflowDelta
             << "monitorSubmitCalls" << monitorStats.m_submitCalls
             << "monitorSubmittedFrames" << monitorStats.m_submittedFrames
             << "monitorSilenceFrames" << monitorStats.m_silenceFrames
             << "monitorOverflowDrainFrames" << monitorStats.m_overflowDrainFrames
             << "monitorFillBeforeMinMax" << monitorStats.m_minFillBefore << monitorStats.m_maxFillBefore
             << "monitorFillAfterMinMax" << monitorStats.m_minFillAfter << monitorStats.m_maxFillAfter
             << "audioLeadMs" << audioLeadMs
             << "pendingAudioBytes" << pendingAudioBytes
             << "pacedAudioCalls" << pacedAudioCalls
             << "pacedAudioTargetFrames" << pacedAudioTargetFrames
             << "pacedAudioOutputFrames" << pacedAudioOutputFrames
             << "pacedAudioShortCalls" << pacedAudioShortCalls
             << "audioTimestampJumps" << audioTimestampJumps
             << "audioTimestampJumpMaxAbsMs" << audioTimestampJumpMaxAbsMs
             << "pendingVideoFrames" << pendingFrames
             << "pendingVideoPackets" << pendingPackets
             << "readAheadCalls" << readAheadCalls
             << "readAheadPackets" << readAheadPackets
             << "readAheadVideoPackets" << readAheadVideoPackets
             << "decoderEagain" << decoderEagain
             << "queuedVideoFrames" << queuedFrames
             << "parkedVideoPackets" << parkedVideoPackets
             << "packetCapHits" << packetCapHits
             << "decoderAudioBytes" << decoderAudioBytes;

    m_videoFileStatsFrames = 0;
    m_videoFileStatsEmptyAudioFrames = 0;
    m_videoFileStatsDecodeMsTotal = 0;
    m_videoFileStatsDecodeMsMax = 0;
    m_videoFileStatsTickDeltaMsTotal = 0;
    m_videoFileStatsTickDeltaMsMax = 0;
    m_videoFileStatsPositionDeltaMsTotal = 0;
    m_videoFileStatsPositionDeltaMsMin = 0;
    m_videoFileStatsPositionDeltaMsMax = 0;
    m_videoFileStatsAudioBytes = 0;
    m_videoFileStatsLastDroppedAudioFrames = droppedAudioFrames;
    m_videoFileStatsLastAudioUnderflows = audioUnderflows;
    m_qtAudio.resetMonitorDebugStats();
    m_videoFileStatsTimer.restart();
}

void CameraWorker::reportVideoFilePlaybackToGUI() const
{
    if (!m_msgQueueToGUI) {
        return;
    }

    m_msgQueueToGUI->push(MsgReportVideoFilePlayback::create(
        m_videoFilePositionMs,
        m_videoFileDurationMs,
        m_videoFilePlaying,
        m_videoFileDecoder != nullptr));
}

void CameraWorker::disconnectControllerCamera(const CameraSettings& settings)
{
    m_alpaca.disconnectCamera(m_networkManager, settings);
}

void CameraWorker::setControllerCameraConnected(bool connected, std::function<void()> continuation)
{
    m_alpaca.setConnected(
        m_networkManager,
        m_settings,
        connected,
        [this]() { reportAlpacaStatusToGUI(); },
        continuation);
}

void CameraWorker::setControllerFocuserConnected(bool connected, std::function<void()> continuation)
{
    m_alpaca.setFocuserConnected(
        m_networkManager,
        m_settings,
        connected,
        [this]() { reportAlpacaStatusToGUI(); },
        continuation);
}

void CameraWorker::moveControllerFocuser()
{
    m_alpaca.moveFocuser(
        m_networkManager,
        m_settings,
        [this]() { reportAlpacaStatusToGUI(); });
}

void CameraWorker::setControllerFilterWheelConnected(bool connected, std::function<void()> continuation)
{
    m_alpaca.setFilterWheelConnected(
        m_networkManager,
        m_settings,
        connected,
        [this]() { reportAlpacaStatusToGUI(); },
        continuation);
}

void CameraWorker::queryControllerFilterWheelInfo()
{
    m_alpaca.queryFilterWheelInfo(
        m_networkManager,
        m_settings,
        [this](const CameraAlpacaController::FilterWheelInfo& info) {
            if (m_msgQueueToGUI) {
                m_msgQueueToGUI->push(MsgReportAlpacaFilterWheelInfo::create(info.m_names, info.m_position));
            }
        });
}

void CameraWorker::queryControllerFilterWheelPosition(std::function<void(int)> continuation)
{
    m_alpaca.queryFilterWheelPosition(m_networkManager, m_settings, continuation);
}

void CameraWorker::waitForControllerFilterWheelPosition(int retriesRemaining)
{
    if (retriesRemaining <= 0 || !m_networkManager || !m_settings.isAlpacaCamera() || !m_settings.m_alpacaFilterWheelEnabled) {
        return;
    }

    queryControllerFilterWheelPosition([this, retriesRemaining](int position) {
        if (position >= 0)
        {
            if (m_msgQueueToGUI) {
                m_msgQueueToGUI->push(MsgReportAlpacaFilterWheelInfo::create(QStringList(), position));
            }
            return;
        }

        QTimer::singleShot(250, this, [this, retriesRemaining]() {
            waitForControllerFilterWheelPosition(retriesRemaining - 1);
        });
    });
}

void CameraWorker::setControllerFilterWheelPosition()
{
    m_alpaca.setFilterWheelPosition(
        m_networkManager,
        m_settings,
        [this]() { waitForControllerFilterWheelPosition(20); },
        [this]() { reportAlpacaStatusToGUI(); });
}

void CameraWorker::bootstrapControllerCamera(std::function<void()> continuation)
{
    m_alpaca.bootstrap(
        m_networkManager,
        m_settings,
        [this]() { reportAlpacaStatusToGUI(); },
        [this]() {
            if (!m_statusTimer.isActive()) {
                m_statusTimer.start(m_alpacaStatusPollIntervalMs);
            }
        },
        [this](const CameraAlpacaController::CapabilitiesReport& report) { reportAlpacaCameraInfoToGUI(report); },
        [this](const CameraAlpacaController::StatusReport& status) {
            reportAlpacaStatusToGUI(status.m_cameraState, status.m_ccdTemperature, status.m_ccdTemperatureValid);
        },
        [this]() {
            if (m_capturing && !m_alpaca.m_frameRequestPending) {
                captureTick();
            }
        },
        continuation);
}

void CameraWorker::applyControllerCameraParams()
{
    m_alpaca.setCameraParams(
        m_networkManager,
        m_settings,
        [this]() { return m_capturing; },
        [this]() { startControllerExposure(); },
        [this]() {
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
        });
}

void CameraWorker::startControllerExposure()
{
    const double exposureTimeMs = currentCaptureExposureTimeMs();
    m_alpaca.startExposure(
        m_networkManager,
        m_settings,
        exposureTimeMs,
        [this, exposureTimeMs]() {
            // Wait for the exposure duration before polling imageready.
            QTimer::singleShot(static_cast<int>(std::ceil(exposureTimeMs)), this, [this]() {
                if (m_capturing) {
                    checkControllerImageReady();
                } else {
                    m_alpaca.m_frameRequestPending = false;
                }
            });
        },
        [this]() {
            if (m_capturing) {
                m_alpaca.m_frameRequestPending = false;
                reportAlpacaStatusToGUI();
                scheduleNextCaptureAfterFailure();
            }
        });
}

void CameraWorker::abortControllerExposure()
{
    m_alpaca.abortExposure(m_networkManager, m_settings);
}

void CameraWorker::checkControllerImageReady()
{
    m_alpaca.checkImageReady(
        m_networkManager,
        m_settings,
        [this](bool ready) {
            if (!m_capturing) {
                m_alpaca.m_frameRequestPending = false;
                return;
            }

            if (ready) {
                fetchControllerImage();
            } else {
                // Some Alpaca devices keep ImageReady false after exposure; CameraState gives us a fallback.
                checkControllerCameraStateForImageReady();
            }
        },
        [this]() {
            if (!m_capturing) {
                m_alpaca.m_frameRequestPending = false;
                return;
            }

            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
        });
}

void CameraWorker::checkControllerCameraStateForImageReady()
{
    m_alpaca.checkCameraStateForImageReady(
        m_networkManager,
        m_settings,
        [this](int cameraState) {
            if (!m_capturing) {
                m_alpaca.m_frameRequestPending = false;
                return;
            }

            if ((cameraState == 0) && m_alpaca.m_exposureSeenActive) {
                fetchControllerImage();
            } else {
                QTimer::singleShot(m_alpacaImageReadyPollIntervalMs, this, [this]() {
                    if (m_capturing) {
                        checkControllerImageReady();
                    } else {
                        m_alpaca.m_frameRequestPending = false;
                    }
                });
            }
        },
        [this]() {
            if (!m_capturing) {
                m_alpaca.m_frameRequestPending = false;
                return;
            }

            m_alpaca.m_frameRequestPending = false;
            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
        });
}

void CameraWorker::fetchControllerImage()
{
    m_alpaca.fetchImageArray(
        m_networkManager,
        m_settings,
        createPlaceholderFrame(),
        [this](const CameraAlpacaController::ImageFetchResult& result) {
            if (!m_capturing) {
                return;
            }

            if (m_framePreprocessor) {
                CameraPipelineFramePtr frame(new CameraPipelineFrame);
                frame->m_image = result.m_image;
                populateFrameExposureMetadata(*frame);
                frame->m_bayerPattern = result.m_bayerPattern;
                sampleAutoFocusFrame(*frame);
                maybeAdjustAutoExposureGain(*frame);
                m_framePreprocessor->submitFrame(frame);
            }
            advanceStackBurstState();
            scheduleNextCaptureAfterFrame();
        },
        [this]() {
            if (!m_capturing) {
                return;
            }

            reportAlpacaStatusToGUI();
            scheduleNextCaptureAfterFailure();
        });
}

void CameraWorker::statusTick()
{
    if (m_networkManager && m_settings.isAlpacaCamera()) {
        pollControllerStatus();
    }
#ifdef ASICAMERA_FOUND
    else if (m_settings.isAsiCamera()) {
        asiPollStatus();
    }
#endif
}

void CameraWorker::pollControllerStatus()
{
    m_alpaca.pollStatus(
        m_networkManager,
        m_settings,
        [this](const CameraAlpacaController::StatusReport& status) {
            reportAlpacaStatusToGUI(status.m_cameraState, status.m_ccdTemperature, status.m_ccdTemperatureValid);
        });
}

void CameraWorker::reportAlpacaCameraInfoToGUI(const CameraAlpacaController::CapabilitiesReport& report)
{
    if (!m_msgQueueToGUI) {
        return;
    }

    m_msgQueueToGUI->push(MsgReportAlpacaCameraInfo::create(
        report.m_name, report.m_description,
        report.m_maxBinX, report.m_maxBinY,
        report.m_gains, report.m_gainMin, report.m_gainMax,
        report.m_offsets, report.m_offsetMin, report.m_offsetMax,
        report.m_readoutModes,
        report.m_sensorName, report.m_sensorType,
        report.m_pixelSizeX, report.m_pixelSizeY,
        report.m_cameraSizeX, report.m_cameraSizeY,
        report.m_ccdTemperature, report.m_ccdTemperatureValid,
        report.m_exposureMinMs, report.m_exposureMaxMs, report.m_exposureResolutionMs));
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
        sampleAutoFocusFrame(*frame);
        maybeAdjustAutoExposureGain(*frame);
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
            sampleAutoFocusFrame(*frame);
            maybeAdjustAutoExposureGain(*frame);
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
