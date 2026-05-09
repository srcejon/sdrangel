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

#include <QDebug>
#include <QFileInfo>
#include <QTimer>

#include "util/profiler.h"
#include "util/fits.h"
#include "cameraframestacker.h"
#include "cameraimageprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraFrameStacker::MsgConfigureCameraFrameStacker, Message)
MESSAGE_CLASS_DEFINITION(CameraFrameStacker::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraFrameStacker::MsgCaptureActive, Message)

CameraFrameStacker::CameraFrameStacker() :
    m_nextStage(nullptr),
    m_captureActive(false),
    m_processingFrame(false)
{
}

CameraFrameStacker::~CameraFrameStacker() = default;

void CameraFrameStacker::startWork()
{
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()), Qt::QueuedConnection);
}

void CameraFrameStacker::stopWork()
{
    disconnect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
}

void CameraFrameStacker::resetFrameHistoryState()
{
    m_stackFrameHistory.clear();
    m_stackAccumulator.release();
}

void CameraFrameStacker::trimFrameHistoryToCurrentLimit()
{
    const int maxFrames = qBound(1, m_settings.m_stackFrameCount, 256);

    while (static_cast<int>(m_stackFrameHistory.size()) > maxFrames)
    {
        if ((m_settings.m_stackMethod == CameraSettings::StackMethodAverage) && !m_stackAccumulator.empty())
        {
            cv::Mat oldestFloatFrame;
            m_stackFrameHistory.front().convertTo(oldestFloatFrame, CV_32FC3);
            m_stackAccumulator -= oldestFloatFrame;
        }

        m_stackFrameHistory.pop_front();
    }

    if (m_stackFrameHistory.empty()) {
        m_stackAccumulator.release();
    }
}

void CameraFrameStacker::reloadCalibrationFrames()
{
    m_darkCalibrationFrame.release();
    m_flatCalibrationFrame.release();
    m_biasCalibrationFrame.release();

    if (!m_settings.m_stackDarkFileName.isEmpty()) {
        m_darkCalibrationFrame = loadFitsCalibrationFrame(m_settings.m_stackDarkFileName, QStringLiteral("dark"), false);
    }

    if (!m_settings.m_stackFlatFileName.isEmpty()) {
        m_flatCalibrationFrame = loadFitsCalibrationFrame(m_settings.m_stackFlatFileName, QStringLiteral("flat"), true);
    }

    if (!m_settings.m_stackBiasFileName.isEmpty()) {
        m_biasCalibrationFrame = loadFitsCalibrationFrame(m_settings.m_stackBiasFileName, QStringLiteral("bias"), false);
    }
}

cv::Mat CameraFrameStacker::loadFitsCalibrationFrame(const QString& fileName, const QString& calibrationType, bool normalizeFlat) const
{
    FITS fits(fileName);

    if (!fits.valid())
    {
        qWarning() << "CameraFrameStacker: Failed to load" << calibrationType << "calibration FITS:" << fileName;
        return cv::Mat();
    }

    cv::Mat monoFrame(fits.height(), fits.width(), CV_32FC1);
    for (int y = 0; y < fits.height(); ++y)
    {
        float *outputLine = monoFrame.ptr<float>(y);
        for (int x = 0; x < fits.width(); ++x) {
            outputLine[x] = fits.scaledValue(x, y);
        }
    }

    if (normalizeFlat)
    {
        const double meanValue = cv::mean(monoFrame)[0];
        constexpr double epsilon = 1.0e-6;

        if (std::abs(meanValue) < epsilon)
        {
            qWarning() << "CameraFrameStacker:" << calibrationType << "calibration FITS has near-zero mean and cannot be normalized:" << fileName;
            return cv::Mat();
        }

        monoFrame /= static_cast<float>(meanValue);
    }

    return monoFrame;
}

void CameraFrameStacker::validateCalibrationFrame(cv::Mat& calibrationFrame, const cv::Size& expectedSize, const QString& calibrationType, const QString& fileName)
{
    if (calibrationFrame.empty()) {
        return;
    }

    if (calibrationFrame.size() != expectedSize)
    {
        qWarning() << "CameraFrameStacker:" << calibrationType << "calibration FITS size"
                   << calibrationFrame.cols << "x" << calibrationFrame.rows
                   << "does not match frame size" << expectedSize.width << "x" << expectedSize.height
                   << "- disabling calibration file:" << fileName;
        calibrationFrame.release();
    }
}

cv::Mat CameraFrameStacker::applyCalibration(const cv::Mat& input)
{
    if (input.empty()) {
        return input;
    }

    validateCalibrationFrame(m_darkCalibrationFrame, input.size(), QStringLiteral("dark"), m_settings.m_stackDarkFileName);
    validateCalibrationFrame(m_flatCalibrationFrame, input.size(), QStringLiteral("flat"), m_settings.m_stackFlatFileName);
    validateCalibrationFrame(m_biasCalibrationFrame, input.size(), QStringLiteral("bias"), m_settings.m_stackBiasFileName);

    if (m_darkCalibrationFrame.empty() && m_flatCalibrationFrame.empty() && m_biasCalibrationFrame.empty()) {
        return input;
    }

    const int channels = input.channels();
    const int floatType = CV_MAKETYPE(CV_32F, channels);
    cv::Mat calibratedFloat;
    input.convertTo(calibratedFloat, floatType);

    auto expandedCalibration = [channels](const cv::Mat& calibrationFrame) -> cv::Mat {
        if (calibrationFrame.empty()) {
            return cv::Mat();
        }

        if (calibrationFrame.channels() == channels) {
            return calibrationFrame;
        }

        if ((calibrationFrame.channels() == 1) && (channels == 3))
        {
            std::vector<cv::Mat> mats(3, calibrationFrame);
            cv::Mat merged;
            cv::merge(mats, merged);
            return merged;
        }

        return cv::Mat();
    };

    const cv::Mat biasFrame = expandedCalibration(m_biasCalibrationFrame);
    const cv::Mat darkFrame = expandedCalibration(m_darkCalibrationFrame);
    const cv::Mat flatFrame = expandedCalibration(m_flatCalibrationFrame);

    if (!m_biasCalibrationFrame.empty() && biasFrame.empty()) {
        qWarning() << "CameraFrameStacker: bias calibration channel count does not match input";
    }
    if (!m_darkCalibrationFrame.empty() && darkFrame.empty()) {
        qWarning() << "CameraFrameStacker: dark calibration channel count does not match input";
    }
    if (!m_flatCalibrationFrame.empty() && flatFrame.empty()) {
        qWarning() << "CameraFrameStacker: flat calibration channel count does not match input";
    }

    if (!biasFrame.empty()) {
        calibratedFloat -= biasFrame;
    }

    if (!darkFrame.empty()) {
        calibratedFloat -= darkFrame;
    }

    if (!flatFrame.empty())
    {
        cv::Mat safeFlatFrame;
        cv::max(flatFrame, cv::Scalar::all(1.0e-6), safeFlatFrame);
        cv::divide(calibratedFloat, safeFlatFrame, calibratedFloat);
    }

    const double maxValue = input.depth() == CV_16U ? 65535.0 : 255.0;
    cv::max(calibratedFloat, cv::Scalar::all(0.0), calibratedFloat);
    cv::min(calibratedFloat, cv::Scalar::all(maxValue), calibratedFloat);

    cv::Mat calibratedFrame;
    calibratedFloat.convertTo(calibratedFrame, input.type());
    return calibratedFrame;
}

int CameraFrameStacker::bayerPatternToOpenCvCode(CameraPipelineFrame::BayerPattern bayerPattern)
{
    switch (bayerPattern)
    {
    case CameraPipelineFrame::BayerRGGB:
        return cv::COLOR_BayerRGGB2BGR;
    case CameraPipelineFrame::BayerBGGR:
        return cv::COLOR_BayerBGGR2BGR;
    case CameraPipelineFrame::BayerGRBG:
        return cv::COLOR_BayerGRBG2BGR;
    case CameraPipelineFrame::BayerGBRG:
        return cv::COLOR_BayerGBRG2BGR;
    case CameraPipelineFrame::BayerNone:
    default:
        return -1;
    }
}

cv::Mat CameraFrameStacker::imageToWorkingMat(const QImage& input, bool& highBitDepthInput)
{
    highBitDepthInput = (input.format() == QImage::Format_RGBA64)
        || (input.format() == QImage::Format_RGBX64)
        || (input.format() == QImage::Format_Grayscale16);

    if (input.format() == QImage::Format_Grayscale16)
    {
        cv::Mat frameMat(input.height(), input.width(), CV_16UC1,
            const_cast<uchar*>(input.bits()),
            static_cast<size_t>(input.bytesPerLine()));
        return frameMat.clone();
    }

    if (input.format() == QImage::Format_Grayscale8)
    {
        cv::Mat frameMat(input.height(), input.width(), CV_8UC1,
            const_cast<uchar*>(input.bits()),
            static_cast<size_t>(input.bytesPerLine()));
        return frameMat.clone();
    }

    if ((input.format() == QImage::Format_RGBA64) || (input.format() == QImage::Format_RGBX64))
    {
        cv::Mat frameMat(input.height(), input.width(), CV_16UC3);
        for (int y = 0; y < input.height(); ++y)
        {
            const QRgba64 *inputLine = reinterpret_cast<const QRgba64*>(input.constScanLine(y));
            cv::Vec<uint16_t, 3> *outputLine = frameMat.ptr<cv::Vec<uint16_t, 3>>(y);

            for (int x = 0; x < input.width(); ++x)
            {
                outputLine[x][0] = inputLine[x].red();
                outputLine[x][1] = inputLine[x].green();
                outputLine[x][2] = inputLine[x].blue();
            }
        }
        return frameMat;
    }

    const QImage rgb = input.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgbMat(rgb.height(), rgb.width(), CV_8UC3,
                   const_cast<uchar*>(rgb.bits()),
                   static_cast<size_t>(rgb.bytesPerLine()));
    return rgbMat.clone();
}

QImage CameraFrameStacker::workingMatToImage(const cv::Mat& frameMat)
{
    if (frameMat.channels() == 1)
    {
        if (frameMat.depth() == CV_16U)
        {
            QImage image(frameMat.cols, frameMat.rows, QImage::Format_Grayscale16);
            for (int row = 0; row < frameMat.rows; ++row) {
                std::memcpy(image.scanLine(row), frameMat.ptr(row), static_cast<size_t>(frameMat.cols * sizeof(quint16)));
            }
            return image;
        }

        QImage image(frameMat.cols, frameMat.rows, QImage::Format_Grayscale8);
        for (int row = 0; row < frameMat.rows; ++row) {
            std::memcpy(image.scanLine(row), frameMat.ptr(row), static_cast<size_t>(frameMat.cols));
        }
        return image;
    }

    if (frameMat.depth() == CV_16U)
    {
        QImage image(frameMat.cols, frameMat.rows, QImage::Format_RGBA64);
        for (int y = 0; y < frameMat.rows; ++y)
        {
            const cv::Vec<uint16_t, 3> *inputLine = frameMat.ptr<cv::Vec<uint16_t, 3>>(y);
            QRgba64 *outputLine = reinterpret_cast<QRgba64*>(image.scanLine(y));

            for (int x = 0; x < frameMat.cols; ++x) {
                outputLine[x] = qRgba64(inputLine[x][0], inputLine[x][1], inputLine[x][2], 65535);
            }
        }
        return image;
    }

    QImage image(frameMat.cols, frameMat.rows, QImage::Format_RGB888);
    for (int row = 0; row < frameMat.rows; ++row) {
        std::memcpy(image.scanLine(row), frameMat.ptr(row), static_cast<size_t>(frameMat.cols * 3));
    }
    return image;
}

cv::Mat CameraFrameStacker::debayerRawMat(const cv::Mat& input, CameraPipelineFrame::BayerPattern bayerPattern)
{
    const int cvCode = bayerPatternToOpenCvCode(bayerPattern);
    if ((cvCode < 0) || (input.channels() != 1)) {
        return input;
    }

    cv::Mat debayered;
    cv::cvtColor(input, debayered, cvCode);
    if (debayered.channels() == 3) {
        cv::cvtColor(debayered, debayered, cv::COLOR_BGR2RGB);
    }
    return debayered;
}

bool CameraFrameStacker::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraFrameStacker::match(cmd))
    {
        const MsgConfigureCameraFrameStacker& cfg = (const MsgConfigureCameraFrameStacker&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgProcessFrame::match(cmd))
    {
        const MsgProcessFrame& frameMsg = (const MsgProcessFrame&) cmd;
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (MsgCaptureActive::match(cmd))
    {
        const MsgCaptureActive& activeMsg = (const MsgCaptureActive&) cmd;
        m_captureActive = activeMsg.isActive();
        if (m_captureActive) {
            resetFrameHistoryState();
        }
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrame.reset();
        if (!m_captureActive) {
            m_processingFrame = false;
        }
        return true;
    }

    return false;
}

void CameraFrameStacker::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraFrameStacker::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraFrameStacker::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    const bool sourceChanged = force
        || settingsKeys.contains("cameraId")
        || settingsKeys.contains("cameraProtocol")
        || settingsKeys.contains("resolutionWidth")
        || settingsKeys.contains("resolutionHeight")
        || settingsKeys.contains("cameraBinX")
        || settingsKeys.contains("cameraBinY")
        || settingsKeys.contains("cameraNumX")
        || settingsKeys.contains("cameraNumY")
        || settingsKeys.contains("cameraStartX")
        || settingsKeys.contains("cameraStartY")
        || settingsKeys.contains("cameraGain")
        || settingsKeys.contains("cameraOffset")
        || settingsKeys.contains("cameraReadoutMode")
        || settingsKeys.contains("exposureTimeMs")
        || settingsKeys.contains("stackEnabled")
        || settingsKeys.contains("stackMethod")
        || settingsKeys.contains("stackAlignmentMethod")
        || settingsKeys.contains("stackDarkFileName")
        || settingsKeys.contains("stackFlatFileName")
        || settingsKeys.contains("stackBiasFileName");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (force
        || settingsKeys.contains("stackDarkFileName")
        || settingsKeys.contains("stackFlatFileName")
        || settingsKeys.contains("stackBiasFileName"))
    {
        reloadCalibrationFrames();
    }

    if (sourceChanged) {
        resetFrameHistoryState();
    } else if (settingsKeys.contains("stackFrameCount")) {
        trimFrameHistoryToCurrentLimit();
    }
}

void CameraFrameStacker::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            qDebug() << "CameraFrameStacker: Dropping pending frame in favor of new frame";
        }
        m_pendingFrame = frame;
        if (!m_processingFrame)
        {
            m_processingFrame = true;
            schedule = true;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraFrameStacker::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraFrameStacker::processNextFrame()
{
    CameraPipelineFramePtr frame;

    {
        QMutexLocker locker(&m_frameMutex);
        frame = m_pendingFrame;
        m_pendingFrame.reset();

        if (!frame)
        {
            m_processingFrame = false;
            return;
        }
    }

    processNewFrame(frame);

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            schedule = true;
        } else {
            m_processingFrame = false;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraFrameStacker::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraFrameStacker::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || frame->m_image.isNull()) {
        return;
    }

    frame->m_image = applyFrameStacking(frame->m_image, frame->m_bayerPattern);
    frame->m_bayerPattern = CameraPipelineFrame::BayerNone;
    frame->m_unprocessedImage = frame->m_image;
    frame->m_stackCount = std::max(1, static_cast<int>(m_stackFrameHistory.size()));

    if (m_nextStage) {
        m_nextStage->submitFrame(frame);
    }
}

QImage CameraFrameStacker::applyFrameStacking(const QImage& input, CameraPipelineFrame::BayerPattern bayerPattern)
{
    PROFILER_START();
    const bool stackEnabled = m_settings.m_stackEnabled && (m_settings.m_stackFrameCount > 1);

    const bool calibrationEnabled = !m_darkCalibrationFrame.empty()
        || !m_flatCalibrationFrame.empty()
        || !m_biasCalibrationFrame.empty();

    if (!stackEnabled && !calibrationEnabled && (bayerPattern == CameraPipelineFrame::BayerNone)) {
        return input;
    }

    bool highBitDepthInput = false;
    cv::Mat frameMat = imageToWorkingMat(input, highBitDepthInput);

    frameMat = applyCalibration(frameMat);
    frameMat = debayerRawMat(frameMat, bayerPattern);

    if (!stackEnabled)
    {
        PROFILER_STOP(__FUNCTION__);
        return workingMatToImage(frameMat);
    }

    if (frameMat.channels() == 1) {
        cv::cvtColor(frameMat, frameMat, cv::COLOR_GRAY2RGB);
    }

    cv::Mat alignedFrameMat = frameMat;

    if (m_stackFrameHistory.empty()
        || m_stackFrameHistory.front().size() != alignedFrameMat.size()
        || m_stackFrameHistory.front().type() != alignedFrameMat.type())
    {
        m_stackFrameHistory.clear();
        m_stackAccumulator.release();
    }

    m_stackFrameHistory.push_back(alignedFrameMat.clone());
    trimFrameHistoryToCurrentLimit();

    const double scaleTo8Bit = alignedFrameMat.depth() == CV_16U ? (255.0 / 65535.0) : 1.0;

    if (m_settings.m_stackMethod == CameraSettings::StackMethodAverage)
    {
        if (m_stackAccumulator.empty()) {
            m_stackAccumulator = cv::Mat::zeros(alignedFrameMat.size(), CV_32FC3);
        }

        cv::Mat floatFrame;
        alignedFrameMat.convertTo(floatFrame, CV_32FC3);
        m_stackAccumulator += floatFrame;

        cv::Mat averagedFloat;
        m_stackAccumulator.convertTo(averagedFloat, CV_32FC3, 1.0 / static_cast<double>(m_stackFrameHistory.size()));
        cv::Mat averaged8u;
        averagedFloat.convertTo(averaged8u, CV_8UC3, scaleTo8Bit);

        QImage stackedImage(averaged8u.cols, averaged8u.rows, QImage::Format_RGB888);
        for (int row = 0; row < averaged8u.rows; ++row) {
            std::memcpy(stackedImage.scanLine(row), averaged8u.ptr(row), static_cast<size_t>(averaged8u.cols * 3));
        }

        PROFILER_STOP(__FUNCTION__);
        return stackedImage;
    }

    m_stackAccumulator.release();

    QImage stackedImage(alignedFrameMat.cols, alignedFrameMat.rows, QImage::Format_RGB888);
    const size_t frameCount = m_stackFrameHistory.size();
    constexpr double sigmaThreshold = 2.0;

    std::vector<int> channelSamples[3];
    for (std::vector<int>& samples : channelSamples) {
        samples.reserve(frameCount);
    }

    for (int row = 0; row < alignedFrameMat.rows; ++row)
    {
        uchar *output = stackedImage.scanLine(row);

        for (int col = 0; col < alignedFrameMat.cols; ++col)
        {
            for (std::vector<int>& samples : channelSamples) {
                samples.clear();
            }

            for (const cv::Mat& frame : m_stackFrameHistory)
            {
                if (highBitDepthInput)
                {
                    const cv::Vec<uint16_t, 3>& pixel = frame.at<cv::Vec<uint16_t, 3>>(row, col);
                    channelSamples[0].push_back(pixel[0]);
                    channelSamples[1].push_back(pixel[1]);
                    channelSamples[2].push_back(pixel[2]);
                }
                else
                {
                    const cv::Vec3b& pixel = frame.at<cv::Vec3b>(row, col);
                    channelSamples[0].push_back(pixel[0]);
                    channelSamples[1].push_back(pixel[1]);
                    channelSamples[2].push_back(pixel[2]);
                }
            }

            for (int channel = 0; channel < 3; ++channel)
            {
                int channelValue = 0;

                if (m_settings.m_stackMethod == CameraSettings::StackMethodMedian)
                {
                    std::vector<int>& samples = channelSamples[channel];
                    const size_t medianIndex = samples.size() / 2;
                    std::nth_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(medianIndex), samples.end());
                    channelValue = samples[medianIndex];
                }
                else
                {
                    const std::vector<int>& samples = channelSamples[channel];
                    double sum = 0.0;
                    double sumSquares = 0.0;

                    for (int sample : samples)
                    {
                        sum += sample;
                        sumSquares += static_cast<double>(sample) * sample;
                    }

                    const double mean = sum / static_cast<double>(samples.size());
                    const double variance = std::max(0.0, (sumSquares / static_cast<double>(samples.size())) - (mean * mean));
                    const double sigma = std::sqrt(variance);
                    const double minValue = mean - sigmaThreshold * sigma;
                    const double maxValue = mean + sigmaThreshold * sigma;

                    double clippedSum = 0.0;
                    int clippedCount = 0;
                    for (int sample : samples)
                    {
                        if ((sample >= minValue) && (sample <= maxValue))
                        {
                            clippedSum += sample;
                            ++clippedCount;
                        }
                    }

                    channelValue = clippedCount > 0
                        ? static_cast<int>(std::lround(clippedSum / static_cast<double>(clippedCount)))
                        : static_cast<int>(std::lround(mean));
                }

                const int outputValue = highBitDepthInput
                    ? static_cast<int>(std::lround((channelValue * 255.0) / 65535.0))
                    : channelValue;
                output[col * 3 + channel] = static_cast<uchar>(qBound(0, outputValue, 255));
            }
        }
    }

    PROFILER_STOP(__FUNCTION__);
    return stackedImage;
}
