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
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include <QColor>
#include <QDebug>
#include <QDateTime>
#include <QMetaObject>
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
    m_mediaPlayback(this),
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
    QObject::connect(&m_mediaPlayback.m_delayedSubmitTimer, &QTimer::timeout, this, &CameraWorker::releaseDelayedVideoFileFrames);
    QObject::connect(&m_mediaPlayback.m_streamFrameRetryTimer, &QTimer::timeout, this, &CameraWorker::releasePendingStreamVideoFileFrame);
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
    QObject::disconnect(&m_mediaPlayback.m_delayedSubmitTimer, &QTimer::timeout, this, &CameraWorker::releaseDelayedVideoFileFrames);
    QObject::disconnect(&m_mediaPlayback.m_streamFrameRetryTimer, &QTimer::timeout, this, &CameraWorker::releasePendingStreamVideoFileFrame);
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
        if ((delta == 0) || ((std::abs(correctionError) < 0.01) && !saturated)) {
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
            if (m_settings.isStreamCamera())
            {
                closeVideoFileDecoder();
                if (m_capturing && openVideoFileDecoder())
                {
                    readVideoFileFrame();
                    setVideoFilePlaying(true);
                }
            }
            else
            {
                seekVideoFile(0, true);
                setVideoFilePlaying(true);
            }
            break;
        case MsgVideoFileControl::StepBack:
            if (!m_settings.isStreamCamera()) {
                stepVideoFile(-1);
            }
            break;
        case MsgVideoFileControl::StepForward:
            if (!m_settings.isStreamCamera()) {
                stepVideoFile(1);
            }
            break;
        case MsgVideoFileControl::Seek:
            if (!m_settings.isStreamCamera()) {
                seekVideoFile(msg.getPositionMs(), true);
            }
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
        || (m_settings.isFfmpegMediaSource() && (force
            || settingsKeys.contains("videoFileCameraPath")
            || settingsKeys.contains("streamUrl")
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
        ++m_mediaPlayback.m_frameSubmitGeneration;
        clearDelayedVideoFileFrames();
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

    if (!recapture && m_capturing && m_settings.isFfmpegMediaSource()
        && (captureCadenceChanged || settingsKeys.contains("videoPlaybackRate")))
    {
        const bool playing = videoFilePlaybackIsPlaying();
        if (playing && m_settings.isStreamCamera() && m_mediaPlayback.m_decoder)
        {
            m_mediaPlayback.m_decoder->setAudioPaceFrameRate(qMax(1.0, m_mediaPlayback.m_frameRate) * qMax(0.1, m_settings.m_videoPlaybackRate));
        }
        else if (playing)
        {
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
    else if (m_settings.isFfmpegMediaSource())
    {
        ++m_mediaPlayback.m_frameSubmitGeneration;
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
    ++m_mediaPlayback.m_frameSubmitGeneration;
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

    if (m_settings.isFfmpegMediaSource())
    {
        if (!videoFilePlaybackIsPlaying()) {
            return;
        }
        const bool frameRead = readVideoFileFrame();
        // Don't top up the audio monitor while rebuffering: video is held to
        // refill its cushion, so pumping audio ahead here would push audio ~one
        // buffer-depth in front of video and break A/V sync (audio cannot resync
        // afterwards because the per-frame pacing never re-establishes).
        if (m_settings.isStreamCamera()
            && !frameRead
            && (m_mediaPlayback.m_basePositionMs >= 0)
            && !m_mediaPlayback.m_streamRebuffering)
        {
            const int audioSampleRate = streamPlaybackAudioSampleRate();
            if (audioSampleRate > 0) {
                submitVideoFileAudio(QByteArray(), audioSampleRate);
            }
        }
        if (m_capturing
            && videoFilePlaybackIsPlaying()
            && (m_settings.isStreamCamera() || frameRead))
        {
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

    const QString mediaSourcePath = m_settings.ffmpegMediaSourcePath();
    if (!m_settings.isFfmpegMediaSource() || mediaSourcePath.isEmpty()) {
        return false;
    }

    const int audioOutputSampleRate = m_qtAudio.startFilePlayback(m_settings, getInputMessageQueue());
    m_mediaPlayback.m_decoder.reset(new CameraVideoFileDecoder());
    QString errorMessage;
    qDebug() << "CameraWorker: opening FFmpeg media source"
             << (m_settings.isStreamCamera() ? QStringLiteral("stream") : QStringLiteral("video"))
             << mediaSourcePath;
    if (!m_mediaPlayback.m_decoder->open(
        mediaSourcePath,
        errorMessage,
        audioOutputSampleRate,
        m_settings.m_streamInputBufferSizeKiB))
    {
        qWarning() << "CameraWorker: FFmpeg media source open failed"
                   << mediaSourcePath
                   << errorMessage;
        reportErrorToFeature(
            QStringLiteral("videoFileOpen:%1").arg(mediaSourcePath),
            m_settings.isStreamCamera() ? tr("Stream could not be opened") : tr("Video file could not be opened"),
            errorMessage);
        m_mediaPlayback.m_decoder.reset();
        m_qtAudio.stop();
        reportVideoFilePlaybackToGUI();
        return false;
    }

    m_mediaPlayback.m_positionMs = 0;
    m_mediaPlayback.m_durationMs = m_mediaPlayback.m_decoder->durationMs();
    m_mediaPlayback.m_frameRate = m_mediaPlayback.m_decoder->frameRate();
    setVideoFilePlaybackPlayingState(false);
    resetVideoFilePlaybackStats();
    if (m_settings.isStreamCamera()) {
        m_mediaPlayback.m_decoder->setAudioPaceFrameRate(qMax(1.0, m_mediaPlayback.m_frameRate) * qMax(0.1, m_settings.m_videoPlaybackRate));
        startVideoFileDecodeThread();
    }
    reportVideoFilePlaybackToGUI();
    qDebug() << "CameraWorker: FFmpeg media source opened"
             << mediaSourcePath
             << "durationMs" << m_mediaPlayback.m_durationMs
             << "fps" << m_mediaPlayback.m_frameRate
             << "streamInitialFrames" << (m_settings.isStreamCamera() ? streamInitialBufferFrameCount() : 0)
             << "streamMaxFrames" << (m_settings.isStreamCamera() ? maxDecodedStreamFrameCount() : 0);
    return true;
}

void CameraWorker::closeVideoFileDecoder()
{
    m_captureTimer.stop();
    stopVideoFileDecodeThread();
    clearPendingStreamVideoFileFrame();
    clearDelayedVideoFileFrames();
    m_mediaPlayback.resetClosed();
    resetVideoFilePlaybackStats();
    if (m_settings.isFfmpegMediaSource()) {
        m_qtAudio.stop();
    }
    reportVideoFilePlaybackToGUI();
}

void CameraWorker::setVideoFilePlaying(bool playing)
{
    if (!m_capturing || !m_settings.isFfmpegMediaSource() || !m_mediaPlayback.m_decoder)
    {
        setVideoFilePlaybackPlayingState(false);
        reportVideoFilePlaybackToGUI();
        return;
    }

    if (m_settings.isStreamCamera() && !playing) {
        setVideoFilePlaybackPlayingState(false);
    }

    if (m_settings.isStreamCamera())
    {
        clearPendingStreamVideoFileFrame();
        clearDelayedVideoFileFrames();
        clearDecodedVideoFileFrames();
        clearStreamPlaybackAudio();
    }

    if (!m_settings.isStreamCamera() || playing) {
        setVideoFilePlaybackPlayingState(playing);
    }
    if (playing)
    {
        if (!m_mediaPlayback.m_statsTimer.isValid()) {
            resetVideoFilePlaybackStats();
        }
        m_captureTimer.setSingleShot(true);
        resetVideoFilePlaybackSchedule();
        scheduleNextVideoFileTick();
    }
    else
    {
        m_captureTimer.stop();
        clearDelayedVideoFileFrames();
        m_mediaPlayback.resetClock();
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
        if (m_settings.isStreamCamera()) {
            submitOrQueueStreamVideoFileFrame(frame);
            return;
        }

        frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        m_framePreprocessor->submitFrame(frame);
        return;
    }

    if (!m_mediaPlayback.m_delayedSubmitClock.isValid()) {
        m_mediaPlayback.m_delayedSubmitClock.start();
    }

    CameraMediaPlaybackState::DelayedFrame delayedFrame;
    delayedFrame.m_frame = frame;
    delayedFrame.m_dueMs = m_mediaPlayback.m_delayedSubmitClock.elapsed() + videoDelayMs;
    delayedFrame.m_captureEpoch = frame->m_captureEpoch;
    delayedFrame.m_generation = m_mediaPlayback.m_frameSubmitGeneration;
    m_mediaPlayback.m_delayedFrames.append(delayedFrame);

    static constexpr qint64 maxDelayedFrameBytes = 512LL * 1024LL * 1024LL;
    static constexpr int hardMaxDelayedFrameCount = 64;
    const qint64 frameBytes = frame->m_image.isNull()
        ? qint64(0)
        : qMax<qint64>(
            1,
            static_cast<qint64>(frame->m_image.bytesPerLine()) * static_cast<qint64>(frame->m_image.height()));
    const int maxFramesByMemory = frameBytes > 0
        ? static_cast<int>(qMax<qint64>(1, maxDelayedFrameBytes / frameBytes))
        : hardMaxDelayedFrameCount;
    const int maxFramesByDelay = qMax(1, static_cast<int>(std::ceil(static_cast<double>(videoDelayMs) / videoFileExactFrameIntervalMs())) + 2);
    const int maxDelayedFrameCount = qBound(1, std::min({hardMaxDelayedFrameCount, maxFramesByMemory, maxFramesByDelay}), hardMaxDelayedFrameCount);
    if (m_mediaPlayback.m_delayedFrames.size() > maxDelayedFrameCount) {
        m_mediaPlayback.m_delayedFrames.remove(0, m_mediaPlayback.m_delayedFrames.size() - maxDelayedFrameCount);
    }

    scheduleDelayedVideoFileFrameSubmit();
}

bool CameraWorker::submitOrQueueStreamVideoFileFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !m_framePreprocessor) {
        return false;
    }

    if (!m_capturing
        || !m_settings.isStreamCamera()
        || !videoFilePlaybackIsPlaying()
        || (frame->m_captureEpoch != m_captureEpoch))
    {
        return false;
    }

    if (m_framePreprocessor->wouldReplacePendingFrame())
    {
        if (m_mediaPlayback.m_pendingStreamFrame) {
            ++m_mediaPlayback.m_statsDroppedPipelineFrames;
        }
        m_mediaPlayback.m_pendingStreamFrame = frame;
        m_mediaPlayback.m_pendingStreamFrameGeneration = m_mediaPlayback.m_frameSubmitGeneration;
        if (!m_mediaPlayback.m_streamFrameRetryTimer.isActive()) {
            m_mediaPlayback.m_streamFrameRetryTimer.start(5);
        }
        return false;
    }

    if (m_mediaPlayback.m_pendingStreamFrame)
    {
        ++m_mediaPlayback.m_statsDroppedPipelineFrames;
        m_mediaPlayback.m_pendingStreamFrame.clear();
        m_mediaPlayback.m_pendingStreamFrameGeneration = 0;
    }

    frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
    m_framePreprocessor->submitFrame(frame);
    return true;
}

void CameraWorker::clearPendingStreamVideoFileFrame()
{
    m_mediaPlayback.m_streamFrameRetryTimer.stop();
    m_mediaPlayback.m_pendingStreamFrame.clear();
    m_mediaPlayback.m_pendingStreamFrameGeneration = 0;
}

void CameraWorker::releasePendingStreamVideoFileFrame()
{
    if (!m_mediaPlayback.m_pendingStreamFrame)
    {
        clearPendingStreamVideoFileFrame();
        return;
    }

    if (!m_capturing
        || !m_settings.isStreamCamera()
        || !videoFilePlaybackIsPlaying()
        || !m_framePreprocessor
        || (m_mediaPlayback.m_pendingStreamFrame->m_captureEpoch != m_captureEpoch)
        || (m_mediaPlayback.m_pendingStreamFrameGeneration != m_mediaPlayback.m_frameSubmitGeneration))
    {
        clearPendingStreamVideoFileFrame();
        return;
    }

    if (m_framePreprocessor->wouldReplacePendingFrame())
    {
        m_mediaPlayback.m_streamFrameRetryTimer.start(5);
        return;
    }

    CameraPipelineFramePtr frame = m_mediaPlayback.m_pendingStreamFrame;
    m_mediaPlayback.m_pendingStreamFrame.clear();
    m_mediaPlayback.m_pendingStreamFrameGeneration = 0;
    frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
    m_framePreprocessor->submitFrame(frame);
}

void CameraWorker::clearDelayedVideoFileFrames()
{
    m_mediaPlayback.m_delayedSubmitTimer.stop();
    m_mediaPlayback.m_delayedFrames.clear();
    m_mediaPlayback.m_delayedSubmitClock.invalidate();
}

void CameraWorker::scheduleDelayedVideoFileFrameSubmit()
{
    if (m_mediaPlayback.m_delayedFrames.isEmpty()) {
        clearDelayedVideoFileFrames();
        return;
    }

    if (!m_mediaPlayback.m_delayedSubmitClock.isValid()) {
        m_mediaPlayback.m_delayedSubmitClock.start();
    }

    qint64 nextDueMs = m_mediaPlayback.m_delayedFrames.constFirst().m_dueMs;
    for (const CameraMediaPlaybackState::DelayedFrame& delayedFrame : m_mediaPlayback.m_delayedFrames) {
        nextDueMs = std::min(nextDueMs, delayedFrame.m_dueMs);
    }

    const int delayMs = static_cast<int>(qBound(
        qint64(1),
        nextDueMs - m_mediaPlayback.m_delayedSubmitClock.elapsed(),
        qint64(1000)));
    m_mediaPlayback.m_delayedSubmitTimer.setSingleShot(true);
    m_mediaPlayback.m_delayedSubmitTimer.start(delayMs);
}

void CameraWorker::releaseDelayedVideoFileFrames()
{
    if (!m_mediaPlayback.m_delayedSubmitClock.isValid() || m_mediaPlayback.m_delayedFrames.isEmpty()) {
        clearDelayedVideoFileFrames();
        return;
    }

    const qint64 nowMs = m_mediaPlayback.m_delayedSubmitClock.elapsed();
    int dueCount = 0;
    while ((dueCount < m_mediaPlayback.m_delayedFrames.size()) && (m_mediaPlayback.m_delayedFrames[dueCount].m_dueMs <= nowMs)) {
        ++dueCount;
    }

    if (dueCount <= 0)
    {
        scheduleDelayedVideoFileFrameSubmit();
        return;
    }

    const CameraMediaPlaybackState::DelayedFrame delayedFrame = m_mediaPlayback.m_delayedFrames[dueCount - 1];
    m_mediaPlayback.m_delayedFrames.remove(0, dueCount);

    if (m_capturing
        && m_settings.isFfmpegMediaSource()
        && videoFilePlaybackIsPlaying()
        && m_framePreprocessor
        && (m_captureEpoch == delayedFrame.m_captureEpoch)
        && (m_mediaPlayback.m_frameSubmitGeneration == delayedFrame.m_generation)
        && delayedFrame.m_frame)
    {
        if (m_settings.isStreamCamera() && m_framePreprocessor->wouldReplacePendingFrame())
        {
            ++m_mediaPlayback.m_statsDroppedPipelineFrames;
            scheduleDelayedVideoFileFrameSubmit();
            return;
        }
        delayedFrame.m_frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        m_framePreprocessor->submitFrame(delayedFrame.m_frame);
    }

    scheduleDelayedVideoFileFrameSubmit();
}

void CameraWorker::startVideoFileDecodeThread()
{
    stopVideoFileDecodeThread();
    clearDecodedVideoFileFrames();

    if (!m_mediaPlayback.m_decoder || !m_settings.isStreamCamera()) {
        return;
    }

    m_mediaPlayback.m_decodeThreadStop.store(false);
    m_mediaPlayback.m_decodeThread = std::thread([this]()
    {
        int consecutiveReadErrors = 0;
        static constexpr int maxConsecutiveReadErrors = 25;
        while (!m_mediaPlayback.m_decodeThreadStop.load())
        {
            CameraMediaPlaybackState::DecodedFrame decodedFrame;
            QString errorMessage;
            QElapsedTimer decodeTimer;
            decodeTimer.start();
            const bool readOk = m_mediaPlayback.m_decoder->readNextFrame(
                decodedFrame.m_image,
                decodedFrame.m_positionMs,
                decodedFrame.m_pcmS16Stereo,
                decodedFrame.m_audioSampleRate,
                errorMessage);
            decodedFrame.m_decodeMs = decodeTimer.elapsed();

            {
                QMutexLocker locker(&m_mediaPlayback.m_decodeStatsMutex);
                m_mediaPlayback.m_decodeStatsSnapshot = m_mediaPlayback.m_decoder->debugStats();
                m_mediaPlayback.m_decodeAudioPositionMs = m_mediaPlayback.m_decoder->audioDecodedPositionMs();
                m_mediaPlayback.m_decodePendingAudioBytes = m_mediaPlayback.m_decoder->pendingAudioBytes();
                m_mediaPlayback.m_decodePendingVideoFrames = m_mediaPlayback.m_decoder->pendingVideoFrameCount();
                m_mediaPlayback.m_decodePendingVideoPackets = m_mediaPlayback.m_decoder->pendingVideoPacketCount();
            }

            if (!readOk)
            {
                if (consecutiveReadErrors < maxConsecutiveReadErrors)
                {
                    ++consecutiveReadErrors;
                    if ((consecutiveReadErrors == 1) || ((consecutiveReadErrors % 10) == 0)) {
                        qWarning() << "CameraWorker: stream decode read failed; retrying"
                                   << consecutiveReadErrors
                                   << errorMessage;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
                decodedFrame.m_errorMessage = errorMessage;
                queueDecodedVideoFileFrame(std::move(decodedFrame));
                break;
            }
            consecutiveReadErrors = 0;

            if (decodedFrame.m_image.isNull())
            {
                decodedFrame.m_eof = true;
                queueDecodedVideoFileFrame(std::move(decodedFrame));
                break;
            }

            queueDecodedVideoFileFrame(std::move(decodedFrame));
        }
    });
}

void CameraWorker::stopVideoFileDecodeThread()
{
    m_mediaPlayback.m_decodeThreadStop.store(true);
    if (m_mediaPlayback.m_decodeThread.joinable() && m_mediaPlayback.m_decoder) {
        m_mediaPlayback.m_decoder->requestAbort();
    }
    m_mediaPlayback.m_decodedFramesNotFull.wakeAll();
    m_mediaPlayback.m_decodedFramesAvailable.wakeAll();
    if (m_mediaPlayback.m_decodeThread.joinable()) {
        m_mediaPlayback.m_decodeThread.join();
    }
    m_mediaPlayback.m_decodeThreadStop.store(false);
    clearDecodedVideoFileFrames();
    clearStreamPlaybackAudio();

    m_mediaPlayback.resetDecodeSnapshot();
}

void CameraWorker::clearDecodedVideoFileFrames()
{
    QMutexLocker locker(&m_mediaPlayback.m_decodedFramesMutex);
    m_mediaPlayback.m_decodedFrames.clear();
    m_mediaPlayback.m_decodeFrameWakeQueued.store(false);
    m_mediaPlayback.m_decodedFramesNotFull.wakeAll();
}

bool CameraWorker::videoFilePlaybackIsPlaying() const
{
    QMutexLocker locker(&m_mediaPlayback.m_decodedFramesMutex);
    return m_mediaPlayback.m_playing;
}

void CameraWorker::setVideoFilePlaybackPlayingState(bool playing)
{
    QMutexLocker locker(&m_mediaPlayback.m_decodedFramesMutex);
    m_mediaPlayback.m_playing = playing;
    m_mediaPlayback.m_decodedFramesNotFull.wakeAll();
}

void CameraWorker::queueDecodedVideoFileFrame(CameraMediaPlaybackState::DecodedFrame&& frame)
{
    QMutexLocker locker(&m_mediaPlayback.m_decodedFramesMutex);
    const bool streamPlayback = m_settings.isStreamCamera();
    while (!streamPlayback
        && (m_mediaPlayback.m_decodedFrames.size() >= static_cast<size_t>(maxDecodedStreamFrameCount(frame.m_image)))
        && !m_mediaPlayback.m_decodeThreadStop.load())
    {
        m_mediaPlayback.m_decodedFramesNotFull.wait(&m_mediaPlayback.m_decodedFramesMutex, 20);
    }

    if (m_mediaPlayback.m_decodeThreadStop.load()) {
        return;
    }

    if (streamPlayback)
    {
        bool pausedWhileWaiting = false;
        while (!m_mediaPlayback.m_playing && !m_mediaPlayback.m_decodeThreadStop.load())
        {
            pausedWhileWaiting = true;
            m_mediaPlayback.m_decodedFramesNotFull.wait(&m_mediaPlayback.m_decodedFramesMutex, 20);
        }

        if (m_mediaPlayback.m_decodeThreadStop.load()) {
            return;
        }
        if (pausedWhileWaiting) {
            return;
        }

        int droppedFrames = 0;
        qint64 firstDroppedPositionMs = -1;
        qint64 nextKeptPositionMs = -1;
        const int maxDecodedFrames = maxDecodedStreamFrameCount(frame.m_image);
        while (m_mediaPlayback.m_playing
            && (m_mediaPlayback.m_decodedFrames.size() >= static_cast<size_t>(maxDecodedFrames)))
        {
            if ((firstDroppedPositionMs < 0) && (m_mediaPlayback.m_decodedFrames.front().m_positionMs >= 0)) {
                firstDroppedPositionMs = m_mediaPlayback.m_decodedFrames.front().m_positionMs;
            }
            m_mediaPlayback.m_decodedFrames.pop_front();
            m_mediaPlayback.m_decodeDroppedFrames.fetch_add(1);
            m_mediaPlayback.m_decodeDroppedSinceLastSubmit.fetch_add(1);
            ++droppedFrames;
        }
        if (droppedFrames > 0)
        {
            const int audioSampleRate = streamPlaybackAudioSampleRate();
            if (audioSampleRate > 0)
            {
                if (!m_mediaPlayback.m_decodedFrames.empty()) {
                    nextKeptPositionMs = m_mediaPlayback.m_decodedFrames.front().m_positionMs;
                } else {
                    nextKeptPositionMs = frame.m_positionMs;
                }

                if ((firstDroppedPositionMs >= 0) && (nextKeptPositionMs > firstDroppedPositionMs)) {
                    dropTimedStreamPlaybackAudio(nextKeptPositionMs - firstDroppedPositionMs, audioSampleRate);
                } else {
                    const double playbackFrameRate = qMax(1.0, m_mediaPlayback.m_frameRate) * qMax(0.1, m_settings.m_videoPlaybackRate);
                    dropPacedStreamPlaybackAudio(droppedFrames, audioSampleRate, playbackFrameRate);
                }
            }
        }

        appendStreamPlaybackAudio(frame.m_pcmS16Stereo, frame.m_audioSampleRate);
        frame.m_pcmS16Stereo.clear();
    }

    const bool wasEmpty = m_mediaPlayback.m_decodedFrames.empty();
    m_mediaPlayback.m_decodedFrames.push_back(std::move(frame));
    m_mediaPlayback.m_decodedFramesAvailable.wakeAll();
    if (wasEmpty && !m_mediaPlayback.m_decodeFrameWakeQueued.exchange(true)) {
        QMetaObject::invokeMethod(this, "captureTick", Qt::QueuedConnection);
    }
}

bool CameraWorker::takeDecodedVideoFileFrame(CameraMediaPlaybackState::DecodedFrame& frame)
{
    QMutexLocker locker(&m_mediaPlayback.m_decodedFramesMutex);
    if (m_mediaPlayback.m_decodedFrames.empty()) {
        return false;
    }

    frame = std::move(m_mediaPlayback.m_decodedFrames.front());
    m_mediaPlayback.m_decodedFrames.pop_front();
    m_mediaPlayback.m_decodedFramesNotFull.wakeAll();
    return true;
}

void CameraWorker::clearStreamPlaybackAudio()
{
    QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
    m_mediaPlayback.m_streamAudioPcmS16Stereo.clear();
    m_mediaPlayback.m_streamAudioSampleRate = 0;
    m_mediaPlayback.m_streamAudioPaceRemainderFrames = 0.0;
}

void CameraWorker::appendStreamPlaybackAudio(const QByteArray& pcmS16Stereo, int audioSampleRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if (pcmS16Stereo.isEmpty() || (audioSampleRate <= 0)) {
        return;
    }

    const int alignedBytes = (pcmS16Stereo.size() / bytesPerSampleFrame) * bytesPerSampleFrame;
    if (alignedBytes <= 0) {
        return;
    }

    QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
    if (m_mediaPlayback.m_streamAudioSampleRate != audioSampleRate)
    {
        m_mediaPlayback.m_streamAudioPcmS16Stereo.clear();
        m_mediaPlayback.m_streamAudioPaceRemainderFrames = 0.0;
        m_mediaPlayback.m_streamAudioSampleRate = audioSampleRate;
    }

    m_mediaPlayback.m_streamAudioPcmS16Stereo.append(pcmS16Stereo.constData(), alignedBytes);

    const int maxBufferedBytes = audioSampleRate * bytesPerSampleFrame * 2;
    if (m_mediaPlayback.m_streamAudioPcmS16Stereo.size() > maxBufferedBytes)
    {
        const int dropBytes = ((m_mediaPlayback.m_streamAudioPcmS16Stereo.size() - maxBufferedBytes) / bytesPerSampleFrame) * bytesPerSampleFrame;
        if (dropBytes > 0)
        {
            m_mediaPlayback.m_streamAudioPcmS16Stereo.remove(0, dropBytes);
            m_mediaPlayback.m_streamAudioDroppedFrames += static_cast<quint64>(dropBytes / bytesPerSampleFrame);
        }
    }
}

int CameraWorker::takeStreamPlaybackAudio(QByteArray& pcmS16Stereo, int audioSampleRate, int maxSampleFrames)
{
    static constexpr int bytesPerSampleFrame = 4;
    pcmS16Stereo.clear();
    if ((audioSampleRate <= 0) || (maxSampleFrames <= 0)) {
        return 0;
    }

    QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
    if ((m_mediaPlayback.m_streamAudioSampleRate != audioSampleRate) || m_mediaPlayback.m_streamAudioPcmS16Stereo.isEmpty()) {
        return 0;
    }

    const int requestedBytes = maxSampleFrames * bytesPerSampleFrame;
    const int takeBytes = qMin(
        requestedBytes,
        (m_mediaPlayback.m_streamAudioPcmS16Stereo.size() / bytesPerSampleFrame) * bytesPerSampleFrame);
    if (takeBytes <= 0) {
        return 0;
    }

    pcmS16Stereo = m_mediaPlayback.m_streamAudioPcmS16Stereo.left(takeBytes);
    m_mediaPlayback.m_streamAudioPcmS16Stereo.remove(0, takeBytes);
    return takeBytes / bytesPerSampleFrame;
}

int CameraWorker::takePacedStreamPlaybackAudio(QByteArray& pcmS16Stereo, int audioSampleRate, double playbackFrameRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    pcmS16Stereo.clear();
    if ((audioSampleRate <= 0) || (playbackFrameRate <= 0.0)) {
        return 0;
    }

    QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
    if ((m_mediaPlayback.m_streamAudioSampleRate != audioSampleRate) || m_mediaPlayback.m_streamAudioPcmS16Stereo.isEmpty()) {
        return 0;
    }

    const double targetFramesExact = static_cast<double>(audioSampleRate) / playbackFrameRate
        + m_mediaPlayback.m_streamAudioPaceRemainderFrames;
    const int targetFrames = qMax(1, static_cast<int>(std::floor(targetFramesExact)));
    m_mediaPlayback.m_streamAudioPaceRemainderFrames = targetFramesExact - static_cast<double>(targetFrames);

    const int takeBytes = qMin(
        targetFrames * bytesPerSampleFrame,
        (m_mediaPlayback.m_streamAudioPcmS16Stereo.size() / bytesPerSampleFrame) * bytesPerSampleFrame);
    if (takeBytes <= 0) {
        return 0;
    }

    pcmS16Stereo = m_mediaPlayback.m_streamAudioPcmS16Stereo.left(takeBytes);
    m_mediaPlayback.m_streamAudioPcmS16Stereo.remove(0, takeBytes);
    return takeBytes / bytesPerSampleFrame;
}

int CameraWorker::dropPacedStreamPlaybackAudio(int droppedVideoFrames, int audioSampleRate, double playbackFrameRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if ((droppedVideoFrames <= 0) || (audioSampleRate <= 0) || (playbackFrameRate <= 0.0)) {
        return 0;
    }

    QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
    if ((m_mediaPlayback.m_streamAudioSampleRate != audioSampleRate) || m_mediaPlayback.m_streamAudioPcmS16Stereo.isEmpty()) {
        return 0;
    }

    const double targetFramesExact = (static_cast<double>(audioSampleRate) * static_cast<double>(droppedVideoFrames) / playbackFrameRate)
        + m_mediaPlayback.m_streamAudioPaceRemainderFrames;
    const int targetFrames = qMax(1, static_cast<int>(std::floor(targetFramesExact)));
    m_mediaPlayback.m_streamAudioPaceRemainderFrames = targetFramesExact - static_cast<double>(targetFrames);

    const int dropBytes = qMin(
        targetFrames * bytesPerSampleFrame,
        (m_mediaPlayback.m_streamAudioPcmS16Stereo.size() / bytesPerSampleFrame) * bytesPerSampleFrame);
    if (dropBytes <= 0) {
        return 0;
    }

    m_mediaPlayback.m_streamAudioPcmS16Stereo.remove(0, dropBytes);
    m_mediaPlayback.m_streamAudioDroppedFrames += static_cast<quint64>(dropBytes / bytesPerSampleFrame);
    return dropBytes / bytesPerSampleFrame;
}

int CameraWorker::dropTimedStreamPlaybackAudio(qint64 durationMs, int audioSampleRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if ((durationMs <= 0) || (audioSampleRate <= 0)) {
        return 0;
    }

    QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
    if ((m_mediaPlayback.m_streamAudioSampleRate != audioSampleRate) || m_mediaPlayback.m_streamAudioPcmS16Stereo.isEmpty()) {
        return 0;
    }

    const double targetFramesExact = (static_cast<double>(audioSampleRate) * static_cast<double>(durationMs) / 1000.0)
        + m_mediaPlayback.m_streamAudioPaceRemainderFrames;
    const int targetFrames = qMax(1, static_cast<int>(std::floor(targetFramesExact)));
    m_mediaPlayback.m_streamAudioPaceRemainderFrames = targetFramesExact - static_cast<double>(targetFrames);

    const int dropBytes = qMin(
        targetFrames * bytesPerSampleFrame,
        (m_mediaPlayback.m_streamAudioPcmS16Stereo.size() / bytesPerSampleFrame) * bytesPerSampleFrame);
    if (dropBytes <= 0) {
        return 0;
    }

    m_mediaPlayback.m_streamAudioPcmS16Stereo.remove(0, dropBytes);
    m_mediaPlayback.m_streamAudioDroppedFrames += static_cast<quint64>(dropBytes / bytesPerSampleFrame);
    return dropBytes / bytesPerSampleFrame;
}

int CameraWorker::streamPlaybackAudioBytes() const
{
    QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
    return m_mediaPlayback.m_streamAudioPcmS16Stereo.size();
}

int CameraWorker::streamPlaybackAudioSampleRate() const
{
    QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
    return m_mediaPlayback.m_streamAudioSampleRate;
}

int CameraWorker::streamInitialBufferFrameCount() const
{
    const double frameRate = qMax(1.0, m_mediaPlayback.m_frameRate);
    const int frameCount = static_cast<int>(std::ceil(
        frameRate * static_cast<double>(CameraMediaPlaybackState::m_streamInitialBufferMs) / 1000.0));
    return qMax(CameraMediaPlaybackState::m_minDecodedStreamFrames, frameCount);
}

int CameraWorker::decodedStreamFrameQueueDepth() const
{
    QMutexLocker locker(&m_mediaPlayback.m_decodedFramesMutex);
    return static_cast<int>(m_mediaPlayback.m_decodedFrames.size());
}

int CameraWorker::maxDecodedStreamFrameCount(const QImage& frameImage) const
{
    const double frameRate = qMax(1.0, m_mediaPlayback.m_frameRate);
    const int timeFrameCount = static_cast<int>(std::ceil(
        frameRate * static_cast<double>(CameraMediaPlaybackState::m_maxDecodedStreamBufferMs) / 1000.0));
    int maxFrameCount = qMax(CameraMediaPlaybackState::m_minDecodedStreamFrames, timeFrameCount);

    if (!frameImage.isNull())
    {
        const qsizetype frameBytes = qMax<qsizetype>(
            1,
            static_cast<qsizetype>(frameImage.bytesPerLine()) * static_cast<qsizetype>(frameImage.height()));
        const int memoryFrameCount = static_cast<int>(qMax<qsizetype>(
            CameraMediaPlaybackState::m_minDecodedStreamFrames,
            CameraMediaPlaybackState::m_maxDecodedStreamBufferBytes / frameBytes));
        maxFrameCount = qMin(maxFrameCount, memoryFrameCount);
    }

    return qMax(maxFrameCount, streamInitialBufferFrameCount());
}

CameraVideoFileDecoder::DebugStats CameraWorker::videoFileDecoderStatsSnapshot() const
{
    if (m_settings.isStreamCamera())
    {
        QMutexLocker locker(&m_mediaPlayback.m_decodeStatsMutex);
        return m_mediaPlayback.m_decodeStatsSnapshot;
    }

    return m_mediaPlayback.m_decoder ? m_mediaPlayback.m_decoder->debugStats() : CameraVideoFileDecoder::DebugStats();
}

qint64 CameraWorker::updateVideoFilePlaybackPosition(
    qint64 decodedPositionMs,
    qint64 decodeMs,
    bool repairTimestampDiscontinuities,
    bool resetClockOnLargeDrift)
{
    qint64 positionMs = decodedPositionMs;
    const bool droppedStreamFrames = m_mediaPlayback.m_decodeDroppedSinceLastSubmit.exchange(0) > 0;

    if (positionMs >= 0)
    {
        if (repairTimestampDiscontinuities && (m_mediaPlayback.m_lastFramePtsMs >= 0))
        {
            const qint64 frameIntervalMs = videoFileFrameIntervalMs();
            const qint64 positionDeltaMs = positionMs - m_mediaPlayback.m_lastFramePtsMs;
            if ((positionDeltaMs <= 0) || ((positionDeltaMs > frameIntervalMs * 3) && !droppedStreamFrames)) {
                positionMs = m_mediaPlayback.m_lastFramePtsMs + frameIntervalMs;
            }
        }
        m_mediaPlayback.m_positionMs = positionMs;
    }
    else
    {
        m_mediaPlayback.m_positionMs += videoFileFrameIntervalMs();
    }

    m_mediaPlayback.m_lastDecodeMs = decodeMs;
    m_mediaPlayback.m_lastFramePtsMs = m_mediaPlayback.m_positionMs;
    if (m_mediaPlayback.m_basePositionMs < 0)
    {
        m_mediaPlayback.m_basePositionMs = m_mediaPlayback.m_positionMs;
        if (!m_mediaPlayback.m_clock.isValid()) {
            m_mediaPlayback.m_clock.start();
        } else {
            m_mediaPlayback.m_clock.restart();
        }
    }

    qint64 videoLateMs = videoFilePlaybackClockMs() - m_mediaPlayback.m_positionMs;
    if (resetClockOnLargeDrift && (std::abs(videoLateMs) > 150))
    {
        m_mediaPlayback.m_basePositionMs = m_mediaPlayback.m_positionMs;
        if (!m_mediaPlayback.m_clock.isValid()) {
            m_mediaPlayback.m_clock.start();
        } else {
            m_mediaPlayback.m_clock.restart();
        }
        m_mediaPlayback.m_tick = 1;
        videoLateMs = 0;
    }
    m_mediaPlayback.m_statsVideoLateMsTotal += videoLateMs;
    m_mediaPlayback.m_statsVideoLateMsMax = std::max(m_mediaPlayback.m_statsVideoLateMsMax, videoLateMs);
    return m_mediaPlayback.m_positionMs;
}

void CameraWorker::submitDecodedVideoFileFrame(
    const QImage& image,
    qint64 decodedPositionMs,
    qint64 decodeMs,
    const QByteArray& pcmS16Stereo,
    int audioSampleRate,
    bool submitAudio,
    bool updateStats,
    bool applyPlaybackOffset,
    bool repairTimestampDiscontinuities,
    bool resetClockOnLargeDrift)
{
    const qint64 playbackPositionMs = updateVideoFilePlaybackPosition(
        decodedPositionMs,
        decodeMs,
        repairTimestampDiscontinuities,
        resetClockOnLargeDrift);

    if (submitAudio && (!pcmS16Stereo.isEmpty() || m_settings.isStreamCamera())) {
        submitVideoFileAudio(pcmS16Stereo, audioSampleRate);
    }

    if (updateStats) {
        updateVideoFilePlaybackStats(decodeMs, playbackPositionMs, pcmS16Stereo.size());
    }

    if (m_framePreprocessor)
    {
        CameraPipelineFramePtr frame(new CameraPipelineFrame);
        frame->m_image = image;
        populateFrameExposureMetadata(*frame);
        frame->m_captureEpoch = m_captureEpoch;
        frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        frame->m_playbackPositionMs = playbackPositionMs;
        frame->m_playbackFrameRate = qMax(1.0, m_mediaPlayback.m_frameRate) * qMax(0.1, m_settings.m_videoPlaybackRate);
        submitVideoFileFrame(frame, applyPlaybackOffset);
    }

    reportVideoFilePlaybackToGUI();
}

bool CameraWorker::readQueuedVideoFileFrame(bool submitAudio)
{
    if (m_settings.isStreamCamera())
    {
        if (!videoFilePlaybackIsPlaying()) {
            return false;
        }

        m_mediaPlayback.m_decodeFrameWakeQueued.store(false);
        int queuedFrames = 0;
        {
            QMutexLocker locker(&m_mediaPlayback.m_decodedFramesMutex);
            queuedFrames = static_cast<int>(m_mediaPlayback.m_decodedFrames.size());
            m_mediaPlayback.m_decodedFramesNotFull.wakeAll();
        }

        // (Re)buffer a cushion of frames before presenting: at startup
        // (basePositionMs < 0) and again whenever a network stall drains the
        // queue mid-playback. Without rebuffering, an empty queue makes the decode
        // thread wake captureTick on every arriving frame (see
        // queueDecodedVideoFileFrame), so frames are presented at the producer's
        // bursty rate, the buffer never refills, and the downstream post-processor
        // drops frames continuously. Holding here lets the fill servo regain
        // control once the cushion is rebuilt.
        const int rebufferTarget = streamInitialBufferFrameCount();
        if (queuedFrames <= 0) {
            if (m_mediaPlayback.m_basePositionMs >= 0) {
                m_mediaPlayback.m_streamRebuffering = true;
            }
            return false;
        }

        const bool buffering = (m_mediaPlayback.m_basePositionMs < 0) || m_mediaPlayback.m_streamRebuffering;
        if (buffering && (queuedFrames < rebufferTarget))
        {
            m_captureTimer.setSingleShot(true);
            m_captureTimer.start(qMax(1, videoFileFrameIntervalMs() / 2));
            return false;
        }
        m_mediaPlayback.m_streamRebuffering = false;
    }

    CameraMediaPlaybackState::DecodedFrame decodedFrame;
    if (!takeDecodedVideoFileFrame(decodedFrame)) {
        return false;
    }

    if (!decodedFrame.m_errorMessage.isEmpty())
    {
        reportErrorToFeature(
            QStringLiteral("videoFileDecode:%1").arg(m_settings.ffmpegMediaSourcePath()),
            tr("Stream decode failed"),
            decodedFrame.m_errorMessage);
        setVideoFilePlaying(false);
        return true;
    }

    if (decodedFrame.m_eof || decodedFrame.m_image.isNull())
    {
        setVideoFilePlaying(false);
        return true;
    }

    QByteArray pcmS16Stereo = decodedFrame.m_pcmS16Stereo;
    int audioSampleRate = decodedFrame.m_audioSampleRate;
    if (m_settings.isStreamCamera() && submitAudio)
    {
        audioSampleRate = streamPlaybackAudioSampleRate();
        if (audioSampleRate > 0)
        {
            const double playbackFrameRate = qMax(1.0, m_mediaPlayback.m_frameRate) * qMax(0.1, m_settings.m_videoPlaybackRate);
            takePacedStreamPlaybackAudio(pcmS16Stereo, audioSampleRate, playbackFrameRate);
        }
    }

    submitDecodedVideoFileFrame(
        decodedFrame.m_image,
        decodedFrame.m_positionMs,
        decodedFrame.m_decodeMs,
        pcmS16Stereo,
        audioSampleRate,
        submitAudio,
        true,
        submitAudio,
        true,
        true);
    return true;
}

bool CameraWorker::readVideoFileFrame(bool submitAudio, qint64 minimumPositionMs)
{
    if (!m_capturing || !m_settings.isFfmpegMediaSource() || !m_mediaPlayback.m_decoder) {
        return false;
    }

    if (m_settings.isStreamCamera() && (minimumPositionMs < 0))
    {
        return readQueuedVideoFileFrame(submitAudio);
    }

    m_mediaPlayback.m_decoder->setAudioPaceFrameRate(
        qMax(1.0, m_mediaPlayback.m_decoder->frameRate()) * qMax(0.1, m_settings.m_videoPlaybackRate));

    QImage image;
    qint64 positionMs = -1;
    QByteArray pcmS16Stereo;
    int audioSampleRate = 0;
    QString errorMessage;
    QElapsedTimer decodeTimer;
    decodeTimer.start();
    bool readOk = false;
    qint64 decodeMs = 0;
    qint64 videoLateMs = 0;
    int droppedLateFrames = 0;
    static constexpr int maxLateDropFrames = 1;
    static constexpr qint64 maxLateDropDecodeMs = 80;
    // Live FLV/HTTP streams can report short timestamp discontinuities after
    // packet loss; avoid compounding those with aggressive catch-up drops.
    static constexpr qint64 liveFrameLateThresholdMs = 3000;
    for (;;)
    {
        image = QImage();
        positionMs = -1;
        pcmS16Stereo.clear();
        audioSampleRate = 0;
        errorMessage.clear();
        readOk = minimumPositionMs >= 0
            ? m_mediaPlayback.m_decoder->readNextFrameAtOrAfter(minimumPositionMs, image, positionMs, errorMessage)
            : m_mediaPlayback.m_decoder->readNextFrame(image, positionMs, pcmS16Stereo, audioSampleRate, errorMessage);
        decodeMs = decodeTimer.elapsed();
        if (!readOk || image.isNull() || (minimumPositionMs >= 0)) {
            break;
        }

        const qint64 framePtsMs = positionMs >= 0 ? positionMs : (m_mediaPlayback.m_lastFramePtsMs + videoFileFrameIntervalMs());
        if (m_mediaPlayback.m_basePositionMs < 0)
        {
            m_mediaPlayback.m_basePositionMs = framePtsMs;
            if (!m_mediaPlayback.m_clock.isValid()) {
                m_mediaPlayback.m_clock.start();
            } else {
                m_mediaPlayback.m_clock.restart();
            }
        }

        videoLateMs = videoFilePlaybackClockMs() - framePtsMs;
        if (!m_settings.isStreamCamera()
            || (videoLateMs <= liveFrameLateThresholdMs)
            || (droppedLateFrames >= maxLateDropFrames)
            || (decodeMs >= maxLateDropDecodeMs))
        {
            break;
        }

        if (submitAudio && (!pcmS16Stereo.isEmpty() || m_settings.isStreamCamera())) {
            submitVideoFileAudio(pcmS16Stereo, audioSampleRate);
        }
        ++droppedLateFrames;
        ++m_mediaPlayback.m_statsDroppedLateFrames;
    }
    if (!readOk)
    {
        reportErrorToFeature(
            QStringLiteral("videoFileDecode:%1").arg(m_settings.ffmpegMediaSourcePath()),
            m_settings.isStreamCamera() ? tr("Stream decode failed") : tr("Video file decode failed"),
            errorMessage);
        setVideoFilePlaying(false);
        return true;
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
        return true;
    }

    submitDecodedVideoFileFrame(
        image,
        positionMs,
        decodeMs,
        pcmS16Stereo,
        audioSampleRate,
        submitAudio,
        minimumPositionMs < 0,
        submitAudio && (minimumPositionMs < 0),
        m_settings.isStreamCamera(),
        m_settings.isStreamCamera());
    return true;
}

void CameraWorker::submitVideoFileAudio(const QByteArray& pcmS16Stereo, int audioSampleRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if (audioSampleRate <= 0) {
        return;
    }

    QByteArray monitorAudio = pcmS16Stereo;
    if (m_mediaPlayback.m_decoder)
    {
        const uint32_t currentFill = m_qtAudio.monitorAudioFill();
        const int targetFillFrames = m_qtAudio.monitorTargetFillFrames(audioSampleRate);
        if (currentFill < static_cast<uint32_t>(targetFillFrames))
        {
            const int neededFrames = targetFillFrames - static_cast<int>(currentFill);
            // For streams, cap the per-call top-up to a couple of video frames'
            // worth of audio. A large burst (the old sampleRate/5 ≈ 200 ms) lets a
            // post-stall recovery pull a whole buffer of not-yet-shown frames'
            // audio into the monitor at once, leaving audio running ~1 s ahead of
            // video with the per-frame buffer permanently drained. A small cap
            // recovers the monitor gradually while keeping audio within ~2 frames
            // of video.
            const double playbackFrameRate = qMax(1.0, m_mediaPlayback.m_frameRate) * qMax(0.1, m_settings.m_videoPlaybackRate);
            const int audioFramesPerVideoFrame = std::max(1, static_cast<int>(audioSampleRate / playbackFrameRate));
            const int maxExtraFrames = m_settings.isStreamCamera()
                ? (2 * audioFramesPerVideoFrame)
                : audioSampleRate / 2;
            QByteArray extraAudio;
            const int extraFrames = m_settings.isStreamCamera()
                ? takeStreamPlaybackAudio(extraAudio, audioSampleRate, std::min(neededFrames, maxExtraFrames))
                : m_mediaPlayback.m_decoder->takePendingAudio(extraAudio, std::min(neededFrames, maxExtraFrames));
            if (extraFrames > 0)
            {
                monitorAudio.append(extraAudio);
                m_mediaPlayback.m_statsMonitorExtraAudioFrames += static_cast<quint64>(extraFrames);
            }
        }
    }

    if (monitorAudio.isEmpty()) {
        return;
    }
    m_qtAudio.submitMonitorPcmSamples(monitorAudio, audioSampleRate);
    const QByteArray& recordingAudio = m_settings.isStreamCamera() ? monitorAudio : pcmS16Stereo;
    if (!recordingAudio.isEmpty()) {
        m_qtAudio.submitRecordingPcmSamples(recordingAudio.left((recordingAudio.size() / bytesPerSampleFrame) * bytesPerSampleFrame), audioSampleRate);
    }
}

void CameraWorker::seekVideoFile(qint64 positionMs, bool displayFrame)
{
    if (!m_mediaPlayback.m_decoder || m_settings.isStreamCamera()) {
        return;
    }

    m_mediaPlayback.m_positionMs = qBound<qint64>(0, positionMs, m_mediaPlayback.m_durationMs > 0 ? m_mediaPlayback.m_durationMs : std::numeric_limits<qint64>::max());
    ++m_mediaPlayback.m_frameSubmitGeneration;
    clearDelayedVideoFileFrames();
    m_mediaPlayback.m_decoder->seek(m_mediaPlayback.m_positionMs);
    m_qtAudio.clearMonitorAudio();
    m_qtAudio.prefillMonitorAudio(m_qtAudio.filePlaybackMonitorPrefillForOffsetMs());
    resetVideoFilePlaybackSchedule();
    reportVideoFilePlaybackToGUI();
    if (displayFrame) {
        readVideoFileFrame(false, m_mediaPlayback.m_positionMs);
    }
}

void CameraWorker::stepVideoFile(int direction)
{
    setVideoFilePlaying(false);
    ++m_mediaPlayback.m_frameSubmitGeneration;
    if (direction >= 0)
    {
        readVideoFileFrame(false);
    }
    else
    {
        const qint64 maxPosition = m_mediaPlayback.m_durationMs > 0 ? m_mediaPlayback.m_durationMs : std::numeric_limits<qint64>::max();
        const qint64 position = qBound<qint64>(
            0,
            m_mediaPlayback.m_positionMs - videoFileFrameIntervalMs(),
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
    const double decoderFps = m_mediaPlayback.m_decoder ? m_mediaPlayback.m_frameRate : m_settings.m_framesPerSecond;
    return 1000.0 / (qMax(1.0, decoderFps) * qMax(0.1, m_settings.m_videoPlaybackRate));
}

void CameraWorker::resetVideoFilePlaybackSchedule()
{
    m_mediaPlayback.m_clock.restart();
    m_mediaPlayback.m_tick = 1;
    m_mediaPlayback.m_basePositionMs = -1;
    m_mediaPlayback.m_lastFramePtsMs = -1;
    m_mediaPlayback.m_lastDecodeMs = 0;
    m_mediaPlayback.m_streamRebuffering = false;
    m_mediaPlayback.m_tickTimer.restart();
}

qint64 CameraWorker::videoFilePlaybackClockMs() const
{
    static constexpr int bytesPerSampleFrame = 4;
    if (m_settings.isStreamCamera()
        && m_mediaPlayback.m_decoder
        && (m_qtAudio.monitorSampleRate() > 0))
    {
        // Slave stream video to the audio device clock. Audio is consumed at the
        // sound card's true rate, so deriving the playback clock from the audio
        // playback position paces video to the real content rate instead of the
        // reported frame rate (which is slightly inaccurate and otherwise slowly
        // drains the video jitter buffer). The decode thread owns the decoder, so
        // read its snapshotted audio position rather than the decoder directly.
        qint64 audioDecodedPositionMs;
        qint64 decoderPendingAudioBytes;
        {
            QMutexLocker statsLocker(&m_mediaPlayback.m_decodeStatsMutex);
            audioDecodedPositionMs = m_mediaPlayback.m_decodeAudioPositionMs;
            decoderPendingAudioBytes = m_mediaPlayback.m_decodePendingAudioBytes;
        }
        if (audioDecodedPositionMs >= 0)
        {
            const qint64 queuedAudioFrames =
                static_cast<qint64>((decoderPendingAudioBytes + streamPlaybackAudioBytes()) / bytesPerSampleFrame)
                + static_cast<qint64>(m_qtAudio.monitorPlaybackClockFill());
            const qint64 queuedAudioMs = static_cast<qint64>(
                (static_cast<double>(queuedAudioFrames) * 1000.0 / static_cast<double>(m_qtAudio.monitorSampleRate())) + 0.5);
            return audioDecodedPositionMs - queuedAudioMs;
        }
    }
    if (!m_settings.isStreamCamera()
        && m_mediaPlayback.m_decoder
        && (m_mediaPlayback.m_decoder->audioDecodedPositionMs() >= 0)
        && (m_qtAudio.monitorSampleRate() > 0))
    {
        const qint64 queuedAudioFrames =
            static_cast<qint64>(m_mediaPlayback.m_decoder->pendingAudioBytes() / bytesPerSampleFrame)
            + static_cast<qint64>(m_qtAudio.monitorPlaybackClockFill());
        const qint64 queuedAudioMs = static_cast<qint64>(
            (static_cast<double>(queuedAudioFrames) * 1000.0 / static_cast<double>(m_qtAudio.monitorSampleRate())) + 0.5);
        return m_mediaPlayback.m_decoder->audioDecodedPositionMs() - queuedAudioMs;
    }

    if (!m_mediaPlayback.m_clock.isValid() || (m_mediaPlayback.m_basePositionMs < 0)) {
        return m_mediaPlayback.m_positionMs;
    }

    const double rate = qMax(0.1, m_settings.m_videoPlaybackRate);
    return m_mediaPlayback.m_basePositionMs
        + static_cast<qint64>(std::llround(static_cast<double>(m_mediaPlayback.m_clock.elapsed()) * rate));
}

void CameraWorker::scheduleNextVideoFileTick()
{
    if (!m_capturing || !videoFilePlaybackIsPlaying() || !m_settings.isFfmpegMediaSource() || !m_mediaPlayback.m_decoder) {
        return;
    }

    const double intervalMs = qMax(1.0, videoFileExactFrameIntervalMs());
    if (!m_mediaPlayback.m_clock.isValid()) {
        resetVideoFilePlaybackSchedule();
    }

    qint64 delayMs = static_cast<qint64>(std::llround(intervalMs));
    if ((m_mediaPlayback.m_basePositionMs >= 0) && (m_mediaPlayback.m_lastFramePtsMs >= 0))
    {
        if (m_settings.isStreamCamera()) {
            // Buffer-fill servo: hold the decoded-frame queue near a target depth
            // so the present rate tracks the true producer (content) rate. Present
            // a touch faster when over-full and a touch slower when under-full;
            // since fill is the integral of (produce - consume), holding it
            // constant forces consumer rate == producer rate. This avoids both
            // buffer drain (underrun) and overflow (dropped frames) without
            // depending on the reported frame rate, and naturally refills after a
            // network stall. Pacing video to an absolute audio/PTS clock instead
            // does not regulate fill, so the queue rides a rail and drops frames.
            const int fill = decodedStreamFrameQueueDepth();
            const int target = streamInitialBufferFrameCount();
            const double slackMs = qMax(5.0, intervalMs * 0.25);
            const double gainMsPerFrame = qMax(0.5, intervalMs * 0.05);
            const double adjustedMs = intervalMs - static_cast<double>(fill - target) * gainMsPerFrame;
            delayMs = static_cast<qint64>(std::llround(
                qBound(intervalMs - slackMs, adjustedMs, intervalMs + slackMs)));
            delayMs = qMax<qint64>(1, delayMs);
        } else {
            const qint64 nextFramePtsMs = m_mediaPlayback.m_lastFramePtsMs + static_cast<qint64>(std::llround(intervalMs));
            delayMs = nextFramePtsMs - videoFilePlaybackClockMs();
            delayMs -= m_mediaPlayback.m_lastDecodeMs;
            delayMs = qMax<qint64>(1, delayMs);
        }
    }
    else
    {
        quint64 tick = m_mediaPlayback.m_tick > 0 ? m_mediaPlayback.m_tick : 1;
        const qint64 elapsedMs = m_mediaPlayback.m_clock.elapsed();
        qint64 targetMs = static_cast<qint64>(std::llround(static_cast<double>(tick) * intervalMs));

        if (targetMs <= elapsedMs)
        {
            tick = static_cast<quint64>(std::floor(static_cast<double>(elapsedMs) / intervalMs)) + 1;
            targetMs = static_cast<qint64>(std::llround(static_cast<double>(tick) * intervalMs));
        }

        delayMs = qMax<qint64>(1, targetMs - elapsedMs);
        m_mediaPlayback.m_tick = tick + 1;
    }
    const int timerDelayMs = static_cast<int>(qMin<qint64>(std::numeric_limits<int>::max(), delayMs));
    m_captureTimer.setSingleShot(true);
    m_captureTimer.start(timerDelayMs);
}

void CameraWorker::resetVideoFilePlaybackStats()
{
    m_mediaPlayback.m_statsTimer.invalidate();
    m_mediaPlayback.m_tickTimer.invalidate();
    m_mediaPlayback.m_statsFrames = 0;
    m_mediaPlayback.m_statsEmptyAudioFrames = 0;
    m_mediaPlayback.m_statsMonitorExtraAudioFrames = 0;
    m_mediaPlayback.m_statsDecodeMsTotal = 0;
    m_mediaPlayback.m_statsDecodeMsMax = 0;
    m_mediaPlayback.m_statsTickDeltaMsTotal = 0;
    m_mediaPlayback.m_statsTickDeltaMsMax = 0;
    m_mediaPlayback.m_statsPositionDeltaMsTotal = 0;
    m_mediaPlayback.m_statsPositionDeltaMsMin = 0;
    m_mediaPlayback.m_statsPositionDeltaMsMax = 0;
    m_mediaPlayback.m_statsLastPositionMs = -1;
    m_mediaPlayback.m_statsAudioBytes = 0;
    m_mediaPlayback.m_statsDroppedLateFrames = 0;
    m_mediaPlayback.m_statsDroppedPipelineFrames = 0;
    m_mediaPlayback.m_statsVideoLateMsTotal = 0;
    m_mediaPlayback.m_statsVideoLateMsMax = 0;
    m_mediaPlayback.m_decodeDroppedFrames.store(0);
    m_mediaPlayback.m_decodeDroppedSinceLastSubmit.store(0);
    m_mediaPlayback.m_statsLastDroppedAudioFrames = m_qtAudio.monitorDroppedFrames();
    m_mediaPlayback.m_statsLastAudioUnderflows = m_qtAudio.monitorUnderflows();
    {
        QMutexLocker locker(&m_mediaPlayback.m_streamAudioMutex);
        m_mediaPlayback.m_statsLastStreamAudioDroppedFrames = m_mediaPlayback.m_streamAudioDroppedFrames;
    }

    if (m_mediaPlayback.m_decoder)
    {
        const CameraVideoFileDecoder::DebugStats stats = videoFileDecoderStatsSnapshot();
        m_mediaPlayback.m_statsLastDecoderReadAheadCalls = stats.m_readAheadCalls;
        m_mediaPlayback.m_statsLastDecoderReadAheadPackets = stats.m_readAheadPackets;
        m_mediaPlayback.m_statsLastDecoderReadAheadVideoPackets = stats.m_readAheadVideoPackets;
        m_mediaPlayback.m_statsLastDecoderReadAheadAudioPackets = stats.m_readAheadAudioPackets;
        m_mediaPlayback.m_statsLastDecoderReadAheadOtherPackets = stats.m_readAheadOtherPackets;
        m_mediaPlayback.m_statsLastDecoderInputVideoPackets = stats.m_inputVideoPackets;
        m_mediaPlayback.m_statsLastDecoderInputAudioPackets = stats.m_inputAudioPackets;
        m_mediaPlayback.m_statsLastDecoderInputOtherPackets = stats.m_inputOtherPackets;
        m_mediaPlayback.m_statsLastDecoderEagain = stats.m_sendVideoPacketEagain;
        m_mediaPlayback.m_statsLastDecoderQueuedFrames = stats.m_queuedVideoFrames;
        m_mediaPlayback.m_statsLastDecoderParkedVideoPackets = stats.m_parkedVideoPackets;
        m_mediaPlayback.m_statsLastDecoderPacketCapHits = stats.m_readAheadPacketCapHits;
        m_mediaPlayback.m_statsLastDecoderAudioBytes = stats.m_audioBytes;
        m_mediaPlayback.m_statsLastDecoderAudioFrames = stats.m_audioFrames;
        m_mediaPlayback.m_statsLastDecoderPacedAudioCalls = stats.m_pacedAudioCalls;
        m_mediaPlayback.m_statsLastDecoderPacedAudioTargetFrames = stats.m_pacedAudioTargetFrames;
        m_mediaPlayback.m_statsLastDecoderPacedAudioOutputFrames = stats.m_pacedAudioOutputFrames;
        m_mediaPlayback.m_statsLastDecoderPacedAudioShortCalls = stats.m_pacedAudioShortCalls;
        m_mediaPlayback.m_statsLastDecoderDroppedPendingAudioBytes = stats.m_droppedPendingAudioBytes;
        m_mediaPlayback.m_statsLastDecoderDroppedPendingAudioFrames = stats.m_droppedPendingAudioFrames;
        m_mediaPlayback.m_statsLastDecoderAudioTimestampJumps = stats.m_audioTimestampJumps;
        m_mediaPlayback.m_statsLastDecoderReadFrameCalls = stats.m_readFrameCalls;
        m_mediaPlayback.m_statsLastDecoderReadFrameMs = stats.m_readFrameMs;
        m_mediaPlayback.m_statsLastDecoderMainReadPackets = stats.m_mainReadPackets;
        m_mediaPlayback.m_statsLastDecoderMainReadMs = stats.m_mainReadMs;
        m_mediaPlayback.m_statsLastDecoderSendVideoPackets = stats.m_sendVideoPackets;
        m_mediaPlayback.m_statsLastDecoderSendVideoMs = stats.m_sendVideoMs;
        m_mediaPlayback.m_statsLastDecoderReceiveVideoCalls = stats.m_receiveVideoCalls;
        m_mediaPlayback.m_statsLastDecoderReceiveVideoMs = stats.m_receiveVideoMs;
        m_mediaPlayback.m_statsLastDecoderFinishAudioCalls = stats.m_finishAudioCalls;
        m_mediaPlayback.m_statsLastDecoderFinishAudioMs = stats.m_finishAudioMs;
        m_mediaPlayback.m_statsLastDecoderReadAheadAudioMs = stats.m_readAheadAudioMs;
        m_mediaPlayback.m_statsLastDecoderReadAheadReadMs = stats.m_readAheadReadMs;
        m_mediaPlayback.m_statsLastDecoderSendAudioPackets = stats.m_sendAudioPackets;
        m_mediaPlayback.m_statsLastDecoderSendAudioMs = stats.m_sendAudioMs;
        m_mediaPlayback.m_statsLastDecoderConvertFrames = stats.m_videoConvertFrames;
        m_mediaPlayback.m_statsLastDecoderConvertMs = stats.m_videoConvertMs;
    }
    else
    {
        m_mediaPlayback.m_statsLastDecoderReadAheadCalls = 0;
        m_mediaPlayback.m_statsLastDecoderReadAheadPackets = 0;
        m_mediaPlayback.m_statsLastDecoderReadAheadVideoPackets = 0;
        m_mediaPlayback.m_statsLastDecoderReadAheadAudioPackets = 0;
        m_mediaPlayback.m_statsLastDecoderReadAheadOtherPackets = 0;
        m_mediaPlayback.m_statsLastDecoderInputVideoPackets = 0;
        m_mediaPlayback.m_statsLastDecoderInputAudioPackets = 0;
        m_mediaPlayback.m_statsLastDecoderInputOtherPackets = 0;
        m_mediaPlayback.m_statsLastDecoderEagain = 0;
        m_mediaPlayback.m_statsLastDecoderQueuedFrames = 0;
        m_mediaPlayback.m_statsLastDecoderParkedVideoPackets = 0;
        m_mediaPlayback.m_statsLastDecoderPacketCapHits = 0;
        m_mediaPlayback.m_statsLastDecoderAudioBytes = 0;
        m_mediaPlayback.m_statsLastDecoderAudioFrames = 0;
        m_mediaPlayback.m_statsLastDecoderPacedAudioCalls = 0;
        m_mediaPlayback.m_statsLastDecoderPacedAudioTargetFrames = 0;
        m_mediaPlayback.m_statsLastDecoderPacedAudioOutputFrames = 0;
        m_mediaPlayback.m_statsLastDecoderPacedAudioShortCalls = 0;
        m_mediaPlayback.m_statsLastDecoderDroppedPendingAudioBytes = 0;
        m_mediaPlayback.m_statsLastDecoderDroppedPendingAudioFrames = 0;
        m_mediaPlayback.m_statsLastDecoderAudioTimestampJumps = 0;
        m_mediaPlayback.m_statsLastDecoderReadFrameCalls = 0;
        m_mediaPlayback.m_statsLastDecoderReadFrameMs = 0;
        m_mediaPlayback.m_statsLastDecoderMainReadPackets = 0;
        m_mediaPlayback.m_statsLastDecoderMainReadMs = 0;
        m_mediaPlayback.m_statsLastDecoderSendVideoPackets = 0;
        m_mediaPlayback.m_statsLastDecoderSendVideoMs = 0;
        m_mediaPlayback.m_statsLastDecoderReceiveVideoCalls = 0;
        m_mediaPlayback.m_statsLastDecoderReceiveVideoMs = 0;
        m_mediaPlayback.m_statsLastDecoderFinishAudioCalls = 0;
        m_mediaPlayback.m_statsLastDecoderFinishAudioMs = 0;
        m_mediaPlayback.m_statsLastDecoderReadAheadAudioMs = 0;
        m_mediaPlayback.m_statsLastDecoderReadAheadReadMs = 0;
        m_mediaPlayback.m_statsLastDecoderSendAudioPackets = 0;
        m_mediaPlayback.m_statsLastDecoderSendAudioMs = 0;
        m_mediaPlayback.m_statsLastDecoderConvertFrames = 0;
        m_mediaPlayback.m_statsLastDecoderConvertMs = 0;
    }
    m_qtAudio.resetMonitorDebugStats();
}

void CameraWorker::updateVideoFilePlaybackStats(qint64 decodeMs, qint64 positionMs, qsizetype audioBytes)
{
    if (!m_mediaPlayback.m_statsTimer.isValid()) {
        m_mediaPlayback.m_statsTimer.start();
    }

    if (m_mediaPlayback.m_tickTimer.isValid())
    {
        const qint64 tickDeltaMs = m_mediaPlayback.m_tickTimer.restart();
        m_mediaPlayback.m_statsTickDeltaMsTotal += tickDeltaMs;
        m_mediaPlayback.m_statsTickDeltaMsMax = std::max(m_mediaPlayback.m_statsTickDeltaMsMax, tickDeltaMs);
    }
    else
    {
        m_mediaPlayback.m_tickTimer.start();
    }

    ++m_mediaPlayback.m_statsFrames;
    m_mediaPlayback.m_statsDecodeMsTotal += decodeMs;
    m_mediaPlayback.m_statsDecodeMsMax = std::max(m_mediaPlayback.m_statsDecodeMsMax, decodeMs);
    m_mediaPlayback.m_statsAudioBytes += static_cast<quint64>(std::max<qsizetype>(0, audioBytes));
    if (audioBytes <= 0) {
        ++m_mediaPlayback.m_statsEmptyAudioFrames;
    }

    if ((positionMs >= 0) && (m_mediaPlayback.m_statsLastPositionMs >= 0))
    {
        const qint64 deltaMs = positionMs - m_mediaPlayback.m_statsLastPositionMs;
        m_mediaPlayback.m_statsPositionDeltaMsTotal += deltaMs;
        if (m_mediaPlayback.m_statsPositionDeltaMsMin == 0) {
            m_mediaPlayback.m_statsPositionDeltaMsMin = deltaMs;
        } else {
            m_mediaPlayback.m_statsPositionDeltaMsMin = std::min(m_mediaPlayback.m_statsPositionDeltaMsMin, deltaMs);
        }
        m_mediaPlayback.m_statsPositionDeltaMsMax = std::max(m_mediaPlayback.m_statsPositionDeltaMsMax, deltaMs);
    }
    if (positionMs >= 0) {
        m_mediaPlayback.m_statsLastPositionMs = positionMs;
    }

    maybeReportVideoFilePlaybackStats();
}

void CameraWorker::maybeReportVideoFilePlaybackStats()
{
    if (!m_mediaPlayback.m_statsTimer.isValid() || (m_mediaPlayback.m_statsTimer.elapsed() < 2000) || (m_mediaPlayback.m_statsFrames == 0)) {
        return;
    }

    const double frames = static_cast<double>(m_mediaPlayback.m_statsFrames);
    const double avgDecodeMs = static_cast<double>(m_mediaPlayback.m_statsDecodeMsTotal) / frames;
    const double avgTickDeltaMs = static_cast<double>(m_mediaPlayback.m_statsTickDeltaMsTotal) / frames;
    const double avgPositionDeltaMs = m_mediaPlayback.m_statsFrames > 1
        ? static_cast<double>(m_mediaPlayback.m_statsPositionDeltaMsTotal) / static_cast<double>(m_mediaPlayback.m_statsFrames - 1)
        : 0.0;
    const double avgVideoLateMs = m_mediaPlayback.m_statsFrames > 0
        ? static_cast<double>(m_mediaPlayback.m_statsVideoLateMsTotal) / static_cast<double>(m_mediaPlayback.m_statsFrames)
        : 0.0;
    const quint64 droppedAudioFrames = m_qtAudio.monitorDroppedFrames();
    const quint64 droppedAudioDelta = droppedAudioFrames - m_mediaPlayback.m_statsLastDroppedAudioFrames;
    const quint64 audioUnderflows = m_qtAudio.monitorUnderflows();
    const quint64 audioUnderflowDelta = audioUnderflows - m_mediaPlayback.m_statsLastAudioUnderflows;
    const quint64 droppedDecodeQueueFrames = m_mediaPlayback.m_decodeDroppedFrames.exchange(0);
    int pendingFrames = 0;
    int pendingPackets = 0;
    qint64 audioLeadMs = 0;
    int pendingAudioBytes = 0;
    quint64 streamAudioDroppedFrames = 0;
    if (m_mediaPlayback.m_decoder)
    {
        if (m_settings.isStreamCamera())
        {
            QMutexLocker queueLocker(&m_mediaPlayback.m_decodedFramesMutex);
            pendingFrames = static_cast<int>(m_mediaPlayback.m_decodedFrames.size());
            queueLocker.unlock();
            QMutexLocker statsLocker(&m_mediaPlayback.m_decodeStatsMutex);
            pendingFrames += m_mediaPlayback.m_decodePendingVideoFrames;
            pendingPackets = m_mediaPlayback.m_decodePendingVideoPackets;
            pendingAudioBytes = m_mediaPlayback.m_decodePendingAudioBytes + streamPlaybackAudioBytes();
            audioLeadMs = m_mediaPlayback.m_decodeAudioPositionMs - m_mediaPlayback.m_positionMs;
            QMutexLocker audioLocker(&m_mediaPlayback.m_streamAudioMutex);
            streamAudioDroppedFrames = m_mediaPlayback.m_streamAudioDroppedFrames - m_mediaPlayback.m_statsLastStreamAudioDroppedFrames;
            m_mediaPlayback.m_statsLastStreamAudioDroppedFrames = m_mediaPlayback.m_streamAudioDroppedFrames;
        }
        else
        {
            pendingFrames = m_mediaPlayback.m_decoder->pendingVideoFrameCount();
            pendingPackets = m_mediaPlayback.m_decoder->pendingVideoPacketCount();
            audioLeadMs = m_mediaPlayback.m_decoder->audioDecodedPositionMs() - m_mediaPlayback.m_positionMs;
            pendingAudioBytes = m_mediaPlayback.m_decoder->pendingAudioBytes();
        }
    }
    const qint64 playbackClockMs = videoFilePlaybackClockMs();

    quint64 readAheadCalls = 0;
    quint64 readAheadPackets = 0;
    quint64 readAheadVideoPackets = 0;
    quint64 readAheadAudioPackets = 0;
    quint64 readAheadOtherPackets = 0;
    quint64 inputVideoPackets = 0;
    quint64 inputAudioPackets = 0;
    quint64 inputOtherPackets = 0;
    quint64 decoderEagain = 0;
    quint64 queuedFrames = 0;
    quint64 parkedVideoPackets = 0;
    quint64 packetCapHits = 0;
    quint64 decoderAudioBytes = 0;
    quint64 decoderAudioFrames = 0;
    quint64 pacedAudioCalls = 0;
    quint64 pacedAudioTargetFrames = 0;
    quint64 pacedAudioOutputFrames = 0;
    quint64 pacedAudioShortCalls = 0;
    quint64 droppedPendingAudioBytes = 0;
    quint64 droppedPendingAudioFrames = 0;
    quint64 audioTimestampJumps = 0;
    qint64 audioTimestampJumpMaxAbsMs = 0;
    quint64 readFrameCalls = 0;
    quint64 readFrameMs = 0;
    qint64 readFrameMaxMs = 0;
    quint64 mainReadPackets = 0;
    quint64 mainReadMs = 0;
    qint64 mainReadMaxMs = 0;
    quint64 sendVideoPackets = 0;
    quint64 sendVideoMs = 0;
    qint64 sendVideoMaxMs = 0;
    quint64 receiveVideoCalls = 0;
    quint64 receiveVideoMs = 0;
    qint64 receiveVideoMaxMs = 0;
    quint64 finishAudioCalls = 0;
    quint64 finishAudioMs = 0;
    qint64 finishAudioMaxMs = 0;
    quint64 readAheadAudioMs = 0;
    qint64 readAheadAudioMaxMs = 0;
    quint64 readAheadReadMs = 0;
    qint64 readAheadReadMaxMs = 0;
    quint64 sendAudioPackets = 0;
    quint64 sendAudioMs = 0;
    qint64 sendAudioMaxMs = 0;
    quint64 convertFrames = 0;
    quint64 convertMs = 0;
    qint64 convertMaxMs = 0;
    int avioBufferSize = 0;
    int avioBufferFill = 0;
    qint64 avioBytesRead = 0;
    if (m_mediaPlayback.m_decoder)
    {
        const CameraVideoFileDecoder::DebugStats stats = videoFileDecoderStatsSnapshot();
        readAheadCalls = stats.m_readAheadCalls - m_mediaPlayback.m_statsLastDecoderReadAheadCalls;
        readAheadPackets = stats.m_readAheadPackets - m_mediaPlayback.m_statsLastDecoderReadAheadPackets;
        readAheadVideoPackets = stats.m_readAheadVideoPackets - m_mediaPlayback.m_statsLastDecoderReadAheadVideoPackets;
        readAheadAudioPackets = stats.m_readAheadAudioPackets - m_mediaPlayback.m_statsLastDecoderReadAheadAudioPackets;
        readAheadOtherPackets = stats.m_readAheadOtherPackets - m_mediaPlayback.m_statsLastDecoderReadAheadOtherPackets;
        inputVideoPackets = stats.m_inputVideoPackets - m_mediaPlayback.m_statsLastDecoderInputVideoPackets;
        inputAudioPackets = stats.m_inputAudioPackets - m_mediaPlayback.m_statsLastDecoderInputAudioPackets;
        inputOtherPackets = stats.m_inputOtherPackets - m_mediaPlayback.m_statsLastDecoderInputOtherPackets;
        decoderEagain = stats.m_sendVideoPacketEagain - m_mediaPlayback.m_statsLastDecoderEagain;
        queuedFrames = stats.m_queuedVideoFrames - m_mediaPlayback.m_statsLastDecoderQueuedFrames;
        parkedVideoPackets = stats.m_parkedVideoPackets - m_mediaPlayback.m_statsLastDecoderParkedVideoPackets;
        packetCapHits = stats.m_readAheadPacketCapHits - m_mediaPlayback.m_statsLastDecoderPacketCapHits;
        decoderAudioBytes = stats.m_audioBytes - m_mediaPlayback.m_statsLastDecoderAudioBytes;
        decoderAudioFrames = stats.m_audioFrames - m_mediaPlayback.m_statsLastDecoderAudioFrames;
        pacedAudioCalls = stats.m_pacedAudioCalls - m_mediaPlayback.m_statsLastDecoderPacedAudioCalls;
        pacedAudioTargetFrames = stats.m_pacedAudioTargetFrames - m_mediaPlayback.m_statsLastDecoderPacedAudioTargetFrames;
        pacedAudioOutputFrames = stats.m_pacedAudioOutputFrames - m_mediaPlayback.m_statsLastDecoderPacedAudioOutputFrames;
        pacedAudioShortCalls = stats.m_pacedAudioShortCalls - m_mediaPlayback.m_statsLastDecoderPacedAudioShortCalls;
        droppedPendingAudioBytes = stats.m_droppedPendingAudioBytes - m_mediaPlayback.m_statsLastDecoderDroppedPendingAudioBytes;
        droppedPendingAudioFrames = stats.m_droppedPendingAudioFrames - m_mediaPlayback.m_statsLastDecoderDroppedPendingAudioFrames;
        audioTimestampJumps = stats.m_audioTimestampJumps - m_mediaPlayback.m_statsLastDecoderAudioTimestampJumps;
        audioTimestampJumpMaxAbsMs = audioTimestampJumps > 0 ? stats.m_audioTimestampJumpMaxAbsMs : 0;
        readFrameCalls = stats.m_readFrameCalls - m_mediaPlayback.m_statsLastDecoderReadFrameCalls;
        readFrameMs = stats.m_readFrameMs - m_mediaPlayback.m_statsLastDecoderReadFrameMs;
        readFrameMaxMs = stats.m_readFrameMaxMs;
        mainReadPackets = stats.m_mainReadPackets - m_mediaPlayback.m_statsLastDecoderMainReadPackets;
        mainReadMs = stats.m_mainReadMs - m_mediaPlayback.m_statsLastDecoderMainReadMs;
        mainReadMaxMs = stats.m_mainReadMaxMs;
        sendVideoPackets = stats.m_sendVideoPackets - m_mediaPlayback.m_statsLastDecoderSendVideoPackets;
        sendVideoMs = stats.m_sendVideoMs - m_mediaPlayback.m_statsLastDecoderSendVideoMs;
        sendVideoMaxMs = stats.m_sendVideoMaxMs;
        receiveVideoCalls = stats.m_receiveVideoCalls - m_mediaPlayback.m_statsLastDecoderReceiveVideoCalls;
        receiveVideoMs = stats.m_receiveVideoMs - m_mediaPlayback.m_statsLastDecoderReceiveVideoMs;
        receiveVideoMaxMs = stats.m_receiveVideoMaxMs;
        finishAudioCalls = stats.m_finishAudioCalls - m_mediaPlayback.m_statsLastDecoderFinishAudioCalls;
        finishAudioMs = stats.m_finishAudioMs - m_mediaPlayback.m_statsLastDecoderFinishAudioMs;
        finishAudioMaxMs = stats.m_finishAudioMaxMs;
        readAheadAudioMs = stats.m_readAheadAudioMs - m_mediaPlayback.m_statsLastDecoderReadAheadAudioMs;
        readAheadAudioMaxMs = stats.m_readAheadAudioMaxMs;
        readAheadReadMs = stats.m_readAheadReadMs - m_mediaPlayback.m_statsLastDecoderReadAheadReadMs;
        readAheadReadMaxMs = stats.m_readAheadReadMaxMs;
        sendAudioPackets = stats.m_sendAudioPackets - m_mediaPlayback.m_statsLastDecoderSendAudioPackets;
        sendAudioMs = stats.m_sendAudioMs - m_mediaPlayback.m_statsLastDecoderSendAudioMs;
        sendAudioMaxMs = stats.m_sendAudioMaxMs;
        convertFrames = stats.m_videoConvertFrames - m_mediaPlayback.m_statsLastDecoderConvertFrames;
        convertMs = stats.m_videoConvertMs - m_mediaPlayback.m_statsLastDecoderConvertMs;
        convertMaxMs = stats.m_videoConvertMaxMs;
        avioBufferSize = stats.m_avioBufferSize;
        avioBufferFill = stats.m_avioBufferFill;
        avioBytesRead = stats.m_avioBytesRead;
        m_mediaPlayback.m_statsLastDecoderReadAheadCalls = stats.m_readAheadCalls;
        m_mediaPlayback.m_statsLastDecoderReadAheadPackets = stats.m_readAheadPackets;
        m_mediaPlayback.m_statsLastDecoderReadAheadVideoPackets = stats.m_readAheadVideoPackets;
        m_mediaPlayback.m_statsLastDecoderReadAheadAudioPackets = stats.m_readAheadAudioPackets;
        m_mediaPlayback.m_statsLastDecoderReadAheadOtherPackets = stats.m_readAheadOtherPackets;
        m_mediaPlayback.m_statsLastDecoderInputVideoPackets = stats.m_inputVideoPackets;
        m_mediaPlayback.m_statsLastDecoderInputAudioPackets = stats.m_inputAudioPackets;
        m_mediaPlayback.m_statsLastDecoderInputOtherPackets = stats.m_inputOtherPackets;
        m_mediaPlayback.m_statsLastDecoderEagain = stats.m_sendVideoPacketEagain;
        m_mediaPlayback.m_statsLastDecoderQueuedFrames = stats.m_queuedVideoFrames;
        m_mediaPlayback.m_statsLastDecoderParkedVideoPackets = stats.m_parkedVideoPackets;
        m_mediaPlayback.m_statsLastDecoderPacketCapHits = stats.m_readAheadPacketCapHits;
        m_mediaPlayback.m_statsLastDecoderAudioBytes = stats.m_audioBytes;
        m_mediaPlayback.m_statsLastDecoderAudioFrames = stats.m_audioFrames;
        m_mediaPlayback.m_statsLastDecoderPacedAudioCalls = stats.m_pacedAudioCalls;
        m_mediaPlayback.m_statsLastDecoderPacedAudioTargetFrames = stats.m_pacedAudioTargetFrames;
        m_mediaPlayback.m_statsLastDecoderPacedAudioOutputFrames = stats.m_pacedAudioOutputFrames;
        m_mediaPlayback.m_statsLastDecoderPacedAudioShortCalls = stats.m_pacedAudioShortCalls;
        m_mediaPlayback.m_statsLastDecoderDroppedPendingAudioBytes = stats.m_droppedPendingAudioBytes;
        m_mediaPlayback.m_statsLastDecoderDroppedPendingAudioFrames = stats.m_droppedPendingAudioFrames;
        m_mediaPlayback.m_statsLastDecoderAudioTimestampJumps = stats.m_audioTimestampJumps;
        m_mediaPlayback.m_statsLastDecoderReadFrameCalls = stats.m_readFrameCalls;
        m_mediaPlayback.m_statsLastDecoderReadFrameMs = stats.m_readFrameMs;
        m_mediaPlayback.m_statsLastDecoderMainReadPackets = stats.m_mainReadPackets;
        m_mediaPlayback.m_statsLastDecoderMainReadMs = stats.m_mainReadMs;
        m_mediaPlayback.m_statsLastDecoderSendVideoPackets = stats.m_sendVideoPackets;
        m_mediaPlayback.m_statsLastDecoderSendVideoMs = stats.m_sendVideoMs;
        m_mediaPlayback.m_statsLastDecoderReceiveVideoCalls = stats.m_receiveVideoCalls;
        m_mediaPlayback.m_statsLastDecoderReceiveVideoMs = stats.m_receiveVideoMs;
        m_mediaPlayback.m_statsLastDecoderFinishAudioCalls = stats.m_finishAudioCalls;
        m_mediaPlayback.m_statsLastDecoderFinishAudioMs = stats.m_finishAudioMs;
        m_mediaPlayback.m_statsLastDecoderReadAheadAudioMs = stats.m_readAheadAudioMs;
        m_mediaPlayback.m_statsLastDecoderReadAheadReadMs = stats.m_readAheadReadMs;
        m_mediaPlayback.m_statsLastDecoderSendAudioPackets = stats.m_sendAudioPackets;
        m_mediaPlayback.m_statsLastDecoderSendAudioMs = stats.m_sendAudioMs;
        m_mediaPlayback.m_statsLastDecoderConvertFrames = stats.m_videoConvertFrames;
        m_mediaPlayback.m_statsLastDecoderConvertMs = stats.m_videoConvertMs;
    }
    const CameraQtAudioController::MonitorDebugStats& monitorStats = m_qtAudio.monitorDebugStats();
    const auto averageMs = [](quint64 totalMs, quint64 count) -> double {
        return count > 0 ? static_cast<double>(totalMs) / static_cast<double>(count) : 0.0;
    };
    const double readFrameAvgMs = averageMs(readFrameMs, readFrameCalls);
    const double mainReadAvgMs = averageMs(mainReadMs, mainReadPackets);
    const double sendVideoAvgMs = averageMs(sendVideoMs, sendVideoPackets);
    const double receiveVideoAvgMs = averageMs(receiveVideoMs, receiveVideoCalls);
    const double finishAudioAvgMs = averageMs(finishAudioMs, finishAudioCalls);
    const double readAheadAudioAvgMs = averageMs(readAheadAudioMs, readAheadCalls);
    const double readAheadReadAvgMs = averageMs(readAheadReadMs, readAheadPackets);
    const double sendAudioAvgMs = averageMs(sendAudioMs, sendAudioPackets);
    const double convertAvgMs = convertFrames > 0
        ? static_cast<double>(convertMs) / static_cast<double>(convertFrames)
        : 0.0;

    qDebug() << "CameraWorker: video playback stats"
             << "frames" << m_mediaPlayback.m_statsFrames
             << "timerMs" << videoFileFrameIntervalMs()
             << "timerExactMs" << videoFileExactFrameIntervalMs()
             << "fps" << (m_mediaPlayback.m_decoder ? m_mediaPlayback.m_frameRate : 0.0)
             << "rate" << m_settings.m_videoPlaybackRate
             << "decodeAvgMs" << avgDecodeMs
             << "decodeMaxMs" << m_mediaPlayback.m_statsDecodeMsMax
             << "readFrameAvgMaxMs" << readFrameAvgMs << readFrameMaxMs
             << "mainReadAvgMaxMs" << mainReadAvgMs << mainReadMaxMs
             << "sendVideoAvgMaxMs" << sendVideoAvgMs << sendVideoMaxMs
             << "receiveVideoAvgMaxMs" << receiveVideoAvgMs << receiveVideoMaxMs
             << "finishAudioAvgMaxMs" << finishAudioAvgMs << finishAudioMaxMs
             << "readAheadAudioAvgMaxMs" << readAheadAudioAvgMs << readAheadAudioMaxMs
             << "readAheadReadAvgMaxMs" << readAheadReadAvgMs << readAheadReadMaxMs
             << "sendAudioAvgMaxMs" << sendAudioAvgMs << sendAudioMaxMs
             << "convertAvgMs" << convertAvgMs
             << "convertMaxMs" << convertMaxMs
             << "tickAvgMs" << avgTickDeltaMs
             << "tickMaxMs" << m_mediaPlayback.m_statsTickDeltaMsMax
             << "posDeltaAvgMs" << avgPositionDeltaMs
             << "posDeltaMinMaxMs" << m_mediaPlayback.m_statsPositionDeltaMsMin << m_mediaPlayback.m_statsPositionDeltaMsMax
             << "videoLateAvgMs" << avgVideoLateMs
             << "videoLateMaxMs" << m_mediaPlayback.m_statsVideoLateMsMax
             << "droppedLateVideoFrames" << m_mediaPlayback.m_statsDroppedLateFrames
             << "droppedDecodeQueueFrames" << droppedDecodeQueueFrames
             << "droppedPipelineVideoFrames" << m_mediaPlayback.m_statsDroppedPipelineFrames
             << "playbackClockMs" << playbackClockMs
             << "audioBytes" << m_mediaPlayback.m_statsAudioBytes
             << "emptyAudioFrames" << m_mediaPlayback.m_statsEmptyAudioFrames
             << "monitorExtraAudioFrames" << m_mediaPlayback.m_statsMonitorExtraAudioFrames
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
             << "streamAudioDroppedFrames" << streamAudioDroppedFrames
             << "pacedAudioCalls" << pacedAudioCalls
             << "pacedAudioTargetFrames" << pacedAudioTargetFrames
             << "pacedAudioOutputFrames" << pacedAudioOutputFrames
             << "pacedAudioShortCalls" << pacedAudioShortCalls
             << "droppedPendingAudioBytes" << droppedPendingAudioBytes
             << "droppedPendingAudioFrames" << droppedPendingAudioFrames
             << "audioTimestampJumps" << audioTimestampJumps
             << "audioTimestampJumpMaxAbsMs" << audioTimestampJumpMaxAbsMs
             << "pendingVideoFrames" << pendingFrames
             << "pendingVideoPackets" << pendingPackets
             << "readAheadCalls" << readAheadCalls
             << "readAheadPackets" << readAheadPackets
             << "readAheadVideoPackets" << readAheadVideoPackets
             << "readAheadAudioPackets" << readAheadAudioPackets
             << "readAheadOtherPackets" << readAheadOtherPackets
             << "mainReadPackets" << mainReadPackets
             << "sendVideoPackets" << sendVideoPackets
             << "receiveVideoCalls" << receiveVideoCalls
             << "finishAudioCalls" << finishAudioCalls
             << "sendAudioPackets" << sendAudioPackets
             << "inputVideoPackets" << inputVideoPackets
             << "inputAudioPackets" << inputAudioPackets
             << "inputOtherPackets" << inputOtherPackets
             << "decoderEagain" << decoderEagain
             << "queuedVideoFrames" << queuedFrames
             << "parkedVideoPackets" << parkedVideoPackets
             << "packetCapHits" << packetCapHits
             << "decoderAudioBytes" << decoderAudioBytes
             << "decoderAudioFrames" << decoderAudioFrames
             << "avioBufferFillSize" << avioBufferFill << avioBufferSize
             << "avioBytesRead" << avioBytesRead;

    m_mediaPlayback.m_statsFrames = 0;
    m_mediaPlayback.m_statsEmptyAudioFrames = 0;
    m_mediaPlayback.m_statsMonitorExtraAudioFrames = 0;
    m_mediaPlayback.m_statsDecodeMsTotal = 0;
    m_mediaPlayback.m_statsDecodeMsMax = 0;
    m_mediaPlayback.m_statsTickDeltaMsTotal = 0;
    m_mediaPlayback.m_statsTickDeltaMsMax = 0;
    m_mediaPlayback.m_statsPositionDeltaMsTotal = 0;
    m_mediaPlayback.m_statsPositionDeltaMsMin = 0;
    m_mediaPlayback.m_statsPositionDeltaMsMax = 0;
    m_mediaPlayback.m_statsAudioBytes = 0;
    m_mediaPlayback.m_statsDroppedLateFrames = 0;
    m_mediaPlayback.m_statsDroppedPipelineFrames = 0;
    m_mediaPlayback.m_statsVideoLateMsTotal = 0;
    m_mediaPlayback.m_statsVideoLateMsMax = 0;
    m_mediaPlayback.m_statsLastDroppedAudioFrames = droppedAudioFrames;
    m_mediaPlayback.m_statsLastAudioUnderflows = audioUnderflows;
    m_qtAudio.resetMonitorDebugStats();
    m_mediaPlayback.m_statsTimer.restart();
}

void CameraWorker::reportVideoFilePlaybackToGUI() const
{
    if (!m_msgQueueToGUI) {
        return;
    }

    m_msgQueueToGUI->push(MsgReportVideoFilePlayback::create(
        m_mediaPlayback.m_positionMs,
        m_mediaPlayback.m_durationMs,
        videoFilePlaybackIsPlaying(),
        m_mediaPlayback.m_decoder != nullptr));
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
