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

#include <opencv2/photo.hpp>
#ifdef CAMERA_OPENCV_CUDA_STACKING
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

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
    m_processingFrame(false),
    m_droppedFrameCount(0)
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
    m_hdrFrameSamples.clear();
    m_stackAccumulator.release();
#ifdef CAMERA_OPENCV_CUDA_STACKING
    m_cudaStackAccumulator.release();
#endif
}

bool CameraFrameStacker::preserveFrameOrder() const
{
    return m_captureActive
        && ((m_settings.isHdrStackingEnabled() && (m_settings.getHdrExposureCount() > 1))
            || (m_settings.m_stackEnabled && (m_settings.m_stackFrameCount > 1)));
}

int CameraFrameStacker::pendingFrameLimit() const
{
    const int stackFrameCount = m_settings.isHdrStackingEnabled()
        ? m_settings.getHdrExposureCount()
        : m_settings.m_stackFrameCount;
    return qBound(2, stackFrameCount * 2, 512);
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
#ifdef CAMERA_OPENCV_CUDA_STACKING
        if ((m_settings.m_stackMethod == CameraSettings::StackMethodAverage) && canUseCudaStacking() && !m_cudaStackAccumulator.empty()) {
            subtractFromCudaAccumulator(m_stackFrameHistory.front());
        }
#endif

        m_stackFrameHistory.pop_front();
    }

    if (m_stackFrameHistory.empty())
    {
        m_stackAccumulator.release();
#ifdef CAMERA_OPENCV_CUDA_STACKING
        m_cudaStackAccumulator.release();
#endif
    }
}

void CameraFrameStacker::reloadCalibrationFrames()
{
    m_darkCalibrationFrame.release();
    m_flatCalibrationFrame.release();
    m_biasCalibrationFrame.release();
#ifdef CAMERA_OPENCV_CUDA_STACKING
    invalidateCudaCalibrationFrames();
#endif

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

#ifdef CAMERA_OPENCV_CUDA_STACKING
bool CameraFrameStacker::canUseCudaStacking() const
{
    static bool warnedNoDevice = false;

    if (!m_settings.m_postProcessUseCuda) {
        return false;
    }

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraFrameStacker: CUDA stacking requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    return true;
}

void CameraFrameStacker::invalidateCudaCalibrationFrames()
{
    m_cudaDarkCalibrationFrame = CudaCalibrationFrame();
    m_cudaFlatCalibrationFrame = CudaCalibrationFrame();
    m_cudaBiasCalibrationFrame = CudaCalibrationFrame();
}

cv::cuda::GpuMat CameraFrameStacker::uploadCalibrationFrameCuda(
    CudaCalibrationFrame& cachedFrame,
    const cv::Mat& calibrationFrame,
    int channels)
{
    if (calibrationFrame.empty()) {
        return cv::cuda::GpuMat();
    }

    if (!cachedFrame.m_frame.empty()
        && (cachedFrame.m_sourceSize == calibrationFrame.size())
        && (cachedFrame.m_sourceType == calibrationFrame.type())
        && (cachedFrame.m_channels == channels))
    {
        return cachedFrame.m_frame;
    }

    cv::cuda::GpuMat uploaded;
    if (calibrationFrame.channels() == channels)
    {
        uploaded.upload(calibrationFrame, m_cudaStackingStream);
    }
    else if ((calibrationFrame.channels() == 1) && (channels == 3))
    {
        cv::cuda::GpuMat monoGpu;
        monoGpu.upload(calibrationFrame, m_cudaStackingStream);
        std::vector<cv::cuda::GpuMat> planes(3, monoGpu);
        cv::cuda::merge(planes, uploaded, m_cudaStackingStream);
    }

    cachedFrame.m_frame = uploaded;
    cachedFrame.m_sourceSize = calibrationFrame.size();
    cachedFrame.m_sourceType = calibrationFrame.type();
    cachedFrame.m_channels = channels;
    return cachedFrame.m_frame;
}

bool CameraFrameStacker::applyCalibrationCuda(cv::cuda::GpuMat& frameGpu, const cv::Size& inputSize, int inputType)
{
    validateCalibrationFrame(m_darkCalibrationFrame, inputSize, QStringLiteral("dark"), m_settings.m_stackDarkFileName);
    validateCalibrationFrame(m_flatCalibrationFrame, inputSize, QStringLiteral("flat"), m_settings.m_stackFlatFileName);
    validateCalibrationFrame(m_biasCalibrationFrame, inputSize, QStringLiteral("bias"), m_settings.m_stackBiasFileName);

    if (m_darkCalibrationFrame.empty() && m_flatCalibrationFrame.empty() && m_biasCalibrationFrame.empty()) {
        return true;
    }

    try
    {
        const int channels = CV_MAT_CN(inputType);
        const int floatType = CV_MAKETYPE(CV_32F, channels);

        cv::cuda::GpuMat calibratedGpu;
        frameGpu.convertTo(calibratedGpu, floatType, m_cudaStackingStream);

        const cv::cuda::GpuMat biasGpu = uploadCalibrationFrameCuda(m_cudaBiasCalibrationFrame, m_biasCalibrationFrame, channels);
        const cv::cuda::GpuMat darkGpu = uploadCalibrationFrameCuda(m_cudaDarkCalibrationFrame, m_darkCalibrationFrame, channels);
        const cv::cuda::GpuMat flatGpu = uploadCalibrationFrameCuda(m_cudaFlatCalibrationFrame, m_flatCalibrationFrame, channels);

        if (!m_biasCalibrationFrame.empty() && biasGpu.empty()) {
            qWarning() << "CameraFrameStacker: bias calibration channel count does not match input";
        }
        if (!m_darkCalibrationFrame.empty() && darkGpu.empty()) {
            qWarning() << "CameraFrameStacker: dark calibration channel count does not match input";
        }
        if (!m_flatCalibrationFrame.empty() && flatGpu.empty()) {
            qWarning() << "CameraFrameStacker: flat calibration channel count does not match input";
        }

        if (!biasGpu.empty()) {
            cv::cuda::subtract(calibratedGpu, biasGpu, calibratedGpu, cv::noArray(), -1, m_cudaStackingStream);
        }
        if (!darkGpu.empty()) {
            cv::cuda::subtract(calibratedGpu, darkGpu, calibratedGpu, cv::noArray(), -1, m_cudaStackingStream);
        }
        if (!flatGpu.empty())
        {
            cv::cuda::GpuMat safeFlatGpu;
            cv::cuda::GpuMat epsilonGpu(flatGpu.size(), flatGpu.type(), cv::Scalar::all(1.0e-6));
            cv::cuda::max(flatGpu, epsilonGpu, safeFlatGpu, m_cudaStackingStream);
            cv::cuda::divide(calibratedGpu, safeFlatGpu, calibratedGpu, 1.0, -1, m_cudaStackingStream);
        }

        calibratedGpu.convertTo(frameGpu, inputType, m_cudaStackingStream);
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFrameStacker: CUDA calibration failed; falling back to CPU:" << error.what();
        invalidateCudaCalibrationFrames();
    }

    return false;
}

bool CameraFrameStacker::debayerRawMatCuda(cv::cuda::GpuMat& frameGpu, CameraPipelineFrame::BayerPattern bayerPattern)
{
    const int cvCode = bayerPatternToOpenCvCode(bayerPattern);
    if (cvCode < 0) {
        return true;
    }
    if (frameGpu.channels() != 1) {
        return true;
    }

    try
    {
        cv::cuda::GpuMat debayeredGpu;
        cv::cuda::demosaicing(frameGpu, debayeredGpu, cvCode, 0, m_cudaStackingStream);
        if (debayeredGpu.channels() == 3)
        {
            cv::cuda::GpuMat rgbGpu;
            cv::cuda::cvtColor(debayeredGpu, rgbGpu, cv::COLOR_BGR2RGB, 0, m_cudaStackingStream);
            debayeredGpu = rgbGpu;
        }

        frameGpu = debayeredGpu;
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFrameStacker: CUDA debayer failed; falling back to CPU:" << error.what();
    }

    return false;
}

bool CameraFrameStacker::prepareFrameCuda(
    const cv::Mat& input,
    CameraPipelineFrame::BayerPattern bayerPattern,
    cv::Mat& output,
    cv::cuda::GpuMat& outputGpu)
{
    if (input.empty()) {
        output = input;
        return true;
    }

    try
    {
        outputGpu.upload(input, m_cudaStackingStream);
        if (!applyCalibrationCuda(outputGpu, input.size(), input.type())) {
            output = debayerRawMat(applyCalibration(input), bayerPattern);
            outputGpu.release();
            return false;
        }
        if (!debayerRawMatCuda(outputGpu, bayerPattern))
        {
            cv::Mat calibrated;
            outputGpu.download(calibrated, m_cudaStackingStream);
            m_cudaStackingStream.waitForCompletion();
            output = debayerRawMat(calibrated, bayerPattern);
            outputGpu.release();
            return false;
        }

        outputGpu.download(output, m_cudaStackingStream);
        m_cudaStackingStream.waitForCompletion();
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFrameStacker: CUDA frame preparation failed; falling back to CPU:" << error.what();
    }

    output = debayerRawMat(applyCalibration(input), bayerPattern);
    outputGpu.release();
    return false;
}

void CameraFrameStacker::subtractFromCudaAccumulator(const cv::Mat& frameMat)
{
    if (m_cudaStackAccumulator.empty()) {
        return;
    }

    cv::cuda::GpuMat frameGpu;
    cv::cuda::GpuMat floatGpu;
    frameGpu.upload(frameMat, m_cudaStackingStream);
    frameGpu.convertTo(floatGpu, CV_32FC3, m_cudaStackingStream);
    cv::cuda::subtract(m_cudaStackAccumulator, floatGpu, m_cudaStackAccumulator, cv::noArray(), -1, m_cudaStackingStream);
}

bool CameraFrameStacker::applyAverageStackingCuda(const cv::Mat& frameMat, const cv::cuda::GpuMat* sourceFrameGpu, double scaleTo8Bit, QImage& outputImage)
{
    try
    {
        if (m_cudaStackAccumulator.empty() || (m_cudaStackAccumulator.size() != frameMat.size())) {
            m_cudaStackAccumulator = cv::cuda::GpuMat(frameMat.size(), CV_32FC3, cv::Scalar::all(0.0));
        }

        cv::cuda::GpuMat frameGpu;
        cv::cuda::GpuMat floatGpu;
        if (sourceFrameGpu && !sourceFrameGpu->empty()) {
            frameGpu = *sourceFrameGpu;
        } else {
            frameGpu.upload(frameMat, m_cudaStackingStream);
        }
        frameGpu.convertTo(floatGpu, CV_32FC3, m_cudaStackingStream);
        cv::cuda::add(m_cudaStackAccumulator, floatGpu, m_cudaStackAccumulator, cv::noArray(), -1, m_cudaStackingStream);

        cv::cuda::GpuMat averagedGpu;
        cv::cuda::GpuMat averaged8uGpu;
        m_cudaStackAccumulator.convertTo(averagedGpu, CV_32FC3, 1.0 / static_cast<double>(m_stackFrameHistory.size()), 0.0, m_cudaStackingStream);
        averagedGpu.convertTo(averaged8uGpu, CV_8UC3, scaleTo8Bit, 0.0, m_cudaStackingStream);

        cv::Mat averaged8u;
        averaged8uGpu.download(averaged8u, m_cudaStackingStream);
        m_cudaStackingStream.waitForCompletion();
        outputImage = workingMatToImage(averaged8u);
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFrameStacker: CUDA average stacking failed; falling back to CPU:" << error.what();
        m_cudaStackAccumulator.release();
    }

    return false;
}
#endif

int CameraFrameStacker::bayerPatternToOpenCvCode(CameraPipelineFrame::BayerPattern bayerPattern)
{
    switch (bayerPattern)
    {
    case CameraPipelineFrame::BayerRGGB:
        return cv::COLOR_BayerBG2BGR;
    case CameraPipelineFrame::BayerBGGR:
        return cv::COLOR_BayerRG2BGR;
    case CameraPipelineFrame::BayerGRBG:
        return cv::COLOR_BayerGB2BGR;
    case CameraPipelineFrame::BayerGBRG:
        return cv::COLOR_BayerGR2BGR;
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
        m_pendingFrames.clear();
        m_droppedFrameCount = 0;
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
        || settingsKeys.contains("postProcessUseCuda")
        || settingsKeys.contains("stackHdrAlgorithm")
        || settingsKeys.contains("stackHdrExposureCount")
        || settingsKeys.contains("stackHdrExposure1Ms")
        || settingsKeys.contains("stackHdrExposure2Ms")
        || settingsKeys.contains("stackHdrExposure3Ms")
        || settingsKeys.contains("stackHdrExposure4Ms")
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

    if (sourceChanged)
    {
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrames.clear();
        m_droppedFrameCount = 0;
        resetFrameHistoryState();
    }
    else if (settingsKeys.contains("stackFrameCount"))
    {
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

        if (preserveFrameOrder())
        {
            const int frameLimit = pendingFrameLimit();
            if (static_cast<int>(m_pendingFrames.size()) >= frameLimit)
            {
                qDebug() << "CameraFrameStacker: Dropping oldest queued stacking frame";
                m_pendingFrames.pop_front();
                ++m_droppedFrameCount;
            }
            m_pendingFrames.push_back(frame);
        }
        else
        {
            if (!m_pendingFrames.empty()) {
                qDebug() << "CameraFrameStacker: Dropping pending frame in favor of new frame";
                m_droppedFrameCount += static_cast<int>(m_pendingFrames.size());
            }
            m_pendingFrames.clear();
            m_pendingFrames.push_back(frame);
        }

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
        if (!m_pendingFrames.empty())
        {
            frame = m_pendingFrames.front();
            m_pendingFrames.pop_front();
            frame->m_stackQueuedCount += static_cast<int>(m_pendingFrames.size());
            frame->m_stackDroppedCount += m_droppedFrameCount;
        }

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
        if (!m_pendingFrames.empty()) {
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

    QImage stackedImage;
    int stackCount = 1;

    if (!applyFrameStacking(*frame, stackedImage, stackCount)) {
        return;
    }

    frame->m_image = stackedImage;
    frame->m_bayerPattern = CameraPipelineFrame::BayerNone;
    frame->m_unprocessedImage = frame->m_image;
    frame->m_stackCount = std::max(1, stackCount);

    if (m_nextStage) {
        m_nextStage->submitFrame(frame);
    }
}

bool CameraFrameStacker::applyFrameStacking(const CameraPipelineFrame& inputFrame, QImage& outputImage, int& stackCount)
{
    PROFILER_START();
    const bool hdrStackingEnabled = m_settings.isHdrStackingEnabled() && (m_settings.getHdrExposureCount() > 1);
    const bool stackEnabled = hdrStackingEnabled
        || (m_settings.m_stackEnabled && (m_settings.m_stackFrameCount > 1));

    const bool calibrationEnabled = !m_darkCalibrationFrame.empty()
        || !m_flatCalibrationFrame.empty()
        || !m_biasCalibrationFrame.empty();

    if (!stackEnabled && !calibrationEnabled && (inputFrame.m_bayerPattern == CameraPipelineFrame::BayerNone))
    {
        outputImage = inputFrame.m_image;
        stackCount = 1;
        return true;
    }

    bool highBitDepthInput = false;
    cv::Mat frameMat = imageToWorkingMat(inputFrame.m_image, highBitDepthInput);

#ifdef CAMERA_OPENCV_CUDA_STACKING
    const bool useCudaStacking = canUseCudaStacking();
    cv::cuda::GpuMat cudaFrameMat;
#else
    const bool useCudaStacking = false;
#endif

#ifdef CAMERA_OPENCV_CUDA_STACKING
    if (useCudaStacking)
    {
        prepareFrameCuda(frameMat, inputFrame.m_bayerPattern, frameMat, cudaFrameMat);
    }
    else
    {
        frameMat = applyCalibration(frameMat);
        frameMat = debayerRawMat(frameMat, inputFrame.m_bayerPattern);
    }
#else
    frameMat = applyCalibration(frameMat);
    frameMat = debayerRawMat(frameMat, inputFrame.m_bayerPattern);
#endif

    if (!stackEnabled)
    {
        PROFILER_STOP(__FUNCTION__);
        outputImage = workingMatToImage(frameMat);
        stackCount = 1;
        return true;
    }

    if (hdrStackingEnabled)
    {
        if (frameMat.channels() == 1) {
            cv::cvtColor(frameMat, frameMat, cv::COLOR_GRAY2RGB);
        }

        const bool validHdrMetadata = (inputFrame.m_hdrExposureCount >= CameraSettings::m_minHdrExposureCount)
            && (inputFrame.m_hdrExposureCount <= CameraSettings::m_maxHdrExposureCount)
            && (inputFrame.m_hdrExposureIndex >= 0)
            && (inputFrame.m_hdrExposureIndex < inputFrame.m_hdrExposureCount);

        if (!validHdrMetadata)
        {
            qWarning() << "CameraFrameStacker: HDR stacking selected without valid HDR frame metadata; passing through current frame";
            resetFrameHistoryState();
            PROFILER_STOP(__FUNCTION__);
            outputImage = workingMatToImage(frameMat);
            stackCount = 1;
            return true;
        }

        const bool resetHdrSequence = m_hdrFrameSamples.empty()
            || (m_hdrFrameSamples.front().m_frameMat.size() != frameMat.size())
            || (m_hdrFrameSamples.front().m_frameMat.type() != frameMat.type())
            || (static_cast<int>(m_hdrFrameSamples.size()) >= inputFrame.m_hdrExposureCount)
            || (inputFrame.m_hdrExposureIndex == 0);

        if (resetHdrSequence) {
            m_hdrFrameSamples.clear();
        }

        if (!m_hdrFrameSamples.empty() && (inputFrame.m_hdrExposureIndex != static_cast<int>(m_hdrFrameSamples.size())))
        {
            qWarning() << "CameraFrameStacker: HDR exposure sequence mismatch; restarting bracket at index" << inputFrame.m_hdrExposureIndex;
            m_hdrFrameSamples.clear();
        }

        if (inputFrame.m_hdrExposureIndex != static_cast<int>(m_hdrFrameSamples.size()))
        {
            PROFILER_STOP(__FUNCTION__);
            return false;
        }

        m_hdrFrameSamples.push_back({frameMat.clone(), std::max(CameraSettings::m_minExposureTimeMs, inputFrame.m_exposureTimeMs), inputFrame.m_hdrExposureIndex});
        stackCount = static_cast<int>(m_hdrFrameSamples.size());

        if (static_cast<int>(m_hdrFrameSamples.size()) < inputFrame.m_hdrExposureCount)
        {
            PROFILER_STOP(__FUNCTION__);
            return false;
        }

        try
        {
            auto sanitizeFloatImage = [](cv::Mat& floatFrame, bool clampUpper)
            {
                for (int y = 0; y < floatFrame.rows; ++y)
                {
                    cv::Vec3f *row = floatFrame.ptr<cv::Vec3f>(y);

                    for (int x = 0; x < floatFrame.cols; ++x)
                    {
                        for (int c = 0; c < 3; ++c)
                        {
                            if (!std::isfinite(row[x][c])) {
                                row[x][c] = 0.0f;
                            } else if (row[x][c] < 0.0f) {
                                row[x][c] = 0.0f;
                            } else if (clampUpper && (row[x][c] > 1.0f)) {
                                row[x][c] = 1.0f;
                            }
                        }
                    }
                }
            };

            auto isUsefulFloatImage = [](const cv::Mat& floatFrame) -> bool
            {
                if (floatFrame.empty() || (floatFrame.type() != CV_32FC3)) {
                    return false;
                }

                double minValue = 0.0;
                double maxValue = 0.0;
                cv::minMaxLoc(floatFrame.reshape(1), &minValue, &maxValue);
                return std::isfinite(minValue) && std::isfinite(maxValue) && (maxValue > 1.0e-4);
            };

            auto useMiddleExposureFrame = [](const std::vector<cv::Mat>& ldrFrames, cv::Mat& tonemapped)
            {
                const int middleIndex = static_cast<int>(ldrFrames.size() / 2);
                ldrFrames[middleIndex].convertTo(tonemapped, CV_32FC3, 1.0 / 255.0);
            };

            std::vector<const HdrFrameSample *> sortedSamples;
            sortedSamples.reserve(m_hdrFrameSamples.size());
            for (const HdrFrameSample& sample : m_hdrFrameSamples) {
                sortedSamples.push_back(&sample);
            }
            std::stable_sort(sortedSamples.begin(), sortedSamples.end(),
                [](const HdrFrameSample *left, const HdrFrameSample *right)
                {
                    return left->m_exposureTimeMs < right->m_exposureTimeMs;
                });

            std::vector<cv::Mat> ldrFrames;
            std::vector<float> exposureTimesSeconds;
            ldrFrames.reserve(m_hdrFrameSamples.size());
            exposureTimesSeconds.reserve(m_hdrFrameSamples.size());

            for (const HdrFrameSample *sample : sortedSamples)
            {
                cv::Mat ldrFrame8u;

                if (sample->m_frameMat.depth() == CV_16U) {
                    sample->m_frameMat.convertTo(ldrFrame8u, CV_8UC3, 255.0 / 65535.0);
                } else if (sample->m_frameMat.depth() == CV_8U) {
                    ldrFrame8u = sample->m_frameMat.clone();
                } else {
                    sample->m_frameMat.convertTo(ldrFrame8u, CV_8UC3);
                }

                cv::cvtColor(ldrFrame8u, ldrFrame8u, cv::COLOR_RGB2BGR);
                ldrFrames.push_back(ldrFrame8u);

                float exposureSeconds = static_cast<float>(std::max(1.0e-6, sample->m_exposureTimeMs / 1000.0));
                if (!exposureTimesSeconds.empty() && (exposureSeconds <= exposureTimesSeconds.back())) {
                    exposureSeconds = exposureTimesSeconds.back() + std::max(1.0e-6f, exposureTimesSeconds.back() * 1.0e-4f);
                }
                exposureTimesSeconds.push_back(exposureSeconds);
            }

            cv::Mat timesMat(static_cast<int>(exposureTimesSeconds.size()), 1, CV_32F, exposureTimesSeconds.data());

            cv::Mat tonemapped;

            if (m_settings.m_stackHdrAlgorithm == CameraSettings::StackHdrAlgorithmMertens)
            {
                cv::Ptr<cv::MergeMertens> mergeMertens = cv::createMergeMertens();
                mergeMertens->process(ldrFrames, tonemapped);
            }
            else
            {
                cv::Mat responseCurve;
                cv::Mat hdrRadiance;

                if (m_settings.m_stackHdrAlgorithm == CameraSettings::StackHdrAlgorithmRobertson)
                {
                    cv::Ptr<cv::CalibrateRobertson> calibrateRobertson = cv::createCalibrateRobertson();
                    calibrateRobertson->process(ldrFrames, responseCurve, timesMat);

                    cv::Ptr<cv::MergeRobertson> mergeRobertson = cv::createMergeRobertson();
                    mergeRobertson->process(ldrFrames, hdrRadiance, timesMat, responseCurve);
                }
                else
                {
                    cv::Ptr<cv::MergeDebevec> mergeDebevec = cv::createMergeDebevec();
                    // Live brackets are too small and scene-dependent for a reliable per-stack CRF fit.
                    mergeDebevec->process(ldrFrames, hdrRadiance, timesMat);
                }

                if (hdrRadiance.depth() != CV_32F) {
                    hdrRadiance.convertTo(hdrRadiance, CV_32FC3);
                }
                sanitizeFloatImage(hdrRadiance, false);

                cv::Ptr<cv::TonemapReinhard> tonemap = cv::createTonemapReinhard(2.2f, 0.0f, 1.0f, 0.0f);
                tonemap->process(hdrRadiance, tonemapped);
            }

            if (tonemapped.depth() != CV_32F) {
                tonemapped.convertTo(tonemapped, CV_32FC3);
            }
            sanitizeFloatImage(tonemapped, true);
            if (!isUsefulFloatImage(tonemapped))
            {
                if (m_settings.m_stackHdrAlgorithm == CameraSettings::StackHdrAlgorithmMertens)
                {
                    qWarning() << "CameraFrameStacker: HDR exposure fusion produced an unusable image; falling back to middle exposure";
                    useMiddleExposureFrame(ldrFrames, tonemapped);
                }
                else
                {
                    qWarning() << "CameraFrameStacker: HDR merge produced an unusable image; falling back to exposure fusion";
                    cv::Ptr<cv::MergeMertens> mergeMertens = cv::createMergeMertens();
                    mergeMertens->process(ldrFrames, tonemapped);
                    if (tonemapped.depth() != CV_32F) {
                        tonemapped.convertTo(tonemapped, CV_32FC3);
                    }
                    sanitizeFloatImage(tonemapped, true);

                    if (!isUsefulFloatImage(tonemapped)) {
                        useMiddleExposureFrame(ldrFrames, tonemapped);
                    }
                }
            }

            cv::cvtColor(tonemapped, tonemapped, cv::COLOR_BGR2RGB);

            cv::Mat clampedTonemapped;
            cv::max(tonemapped, cv::Scalar::all(0.0f), clampedTonemapped);
            cv::min(clampedTonemapped, cv::Scalar::all(1.0f), clampedTonemapped);

            cv::Mat ldr8u;
            clampedTonemapped.convertTo(ldr8u, CV_8UC3, 255.0);
            outputImage = workingMatToImage(ldr8u);
        }
        catch (const cv::Exception& error)
        {
            qWarning() << "CameraFrameStacker: HDR merge failed:" << error.what();
            outputImage = workingMatToImage(frameMat);
            stackCount = 1;
        }

        m_hdrFrameSamples.clear();
        PROFILER_STOP(__FUNCTION__);
        return true;
    }

    if (frameMat.channels() == 1) {
        cv::cvtColor(frameMat, frameMat, cv::COLOR_GRAY2RGB);
#ifdef CAMERA_OPENCV_CUDA_STACKING
        if (!cudaFrameMat.empty() && (cudaFrameMat.channels() == 1))
        {
            cv::cuda::GpuMat rgbGpu;
            cv::cuda::cvtColor(cudaFrameMat, rgbGpu, cv::COLOR_GRAY2RGB, 0, m_cudaStackingStream);
            cudaFrameMat = rgbGpu;
        }
#endif
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
#ifdef CAMERA_OPENCV_CUDA_STACKING
        if (useCudaStacking && applyAverageStackingCuda(alignedFrameMat, cudaFrameMat.empty() ? nullptr : &cudaFrameMat, scaleTo8Bit, outputImage))
        {
            stackCount = static_cast<int>(m_stackFrameHistory.size());
            PROFILER_STOP(__FUNCTION__);
            return true;
        }
#endif
        bool accumulatorIncludesCurrentFrame = false;
        if (m_stackAccumulator.empty() && (m_stackFrameHistory.size() > 1))
        {
            m_stackAccumulator = cv::Mat::zeros(alignedFrameMat.size(), CV_32FC3);
            for (const cv::Mat& historyFrame : m_stackFrameHistory)
            {
                cv::Mat floatFrame;
                historyFrame.convertTo(floatFrame, CV_32FC3);
                m_stackAccumulator += floatFrame;
            }
            accumulatorIncludesCurrentFrame = true;
        }
        else if (m_stackAccumulator.empty()) {
            m_stackAccumulator = cv::Mat::zeros(alignedFrameMat.size(), CV_32FC3);
        }

        if (!accumulatorIncludesCurrentFrame)
        {
            cv::Mat floatFrame;
            alignedFrameMat.convertTo(floatFrame, CV_32FC3);
            m_stackAccumulator += floatFrame;
        }

        cv::Mat averagedFloat;
        m_stackAccumulator.convertTo(averagedFloat, CV_32FC3, 1.0 / static_cast<double>(m_stackFrameHistory.size()));
        cv::Mat averaged8u;
        averagedFloat.convertTo(averaged8u, CV_8UC3, scaleTo8Bit);

        outputImage = workingMatToImage(averaged8u);
        stackCount = static_cast<int>(m_stackFrameHistory.size());

        PROFILER_STOP(__FUNCTION__);
        return true;
    }

    m_stackAccumulator.release();

    QImage stackedImage(alignedFrameMat.cols, alignedFrameMat.rows, QImage::Format_RGB888);
    const size_t frameCount = m_stackFrameHistory.size();
    constexpr double sigmaThreshold = 2.0;
    const bool medianStacking = m_settings.m_stackMethod == CameraSettings::StackMethodMedian;
    std::vector<int> medianSamples[3];
    if (medianStacking)
    {
        for (std::vector<int>& samples : medianSamples) {
            samples.resize(frameCount);
        }
    }

    for (int row = 0; row < alignedFrameMat.rows; ++row)
    {
        uchar *output = stackedImage.scanLine(row);

        for (int col = 0; col < alignedFrameMat.cols; ++col)
        {
            if (medianStacking)
            {
                size_t frameIndex = 0;
                for (const cv::Mat& frame : m_stackFrameHistory)
                {
                    if (highBitDepthInput)
                    {
                        const cv::Vec<uint16_t, 3>& pixel = frame.ptr<cv::Vec<uint16_t, 3>>(row)[col];
                        medianSamples[0][frameIndex] = pixel[0];
                        medianSamples[1][frameIndex] = pixel[1];
                        medianSamples[2][frameIndex] = pixel[2];
                    }
                    else
                    {
                        const cv::Vec3b& pixel = frame.ptr<cv::Vec3b>(row)[col];
                        medianSamples[0][frameIndex] = pixel[0];
                        medianSamples[1][frameIndex] = pixel[1];
                        medianSamples[2][frameIndex] = pixel[2];
                    }
                    ++frameIndex;
                }

                const size_t medianIndex = frameCount / 2;
                for (int channel = 0; channel < 3; ++channel)
                {
                    std::vector<int>& samples = medianSamples[channel];
                    std::nth_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(medianIndex), samples.end());
                    const int outputValue = highBitDepthInput
                        ? static_cast<int>(std::lround((samples[medianIndex] * 255.0) / 65535.0))
                        : samples[medianIndex];
                    output[col * 3 + channel] = static_cast<uchar>(qBound(0, outputValue, 255));
                }
                continue;
            }

            double sum[3] = {0.0, 0.0, 0.0};
            double sumSquares[3] = {0.0, 0.0, 0.0};

            for (const cv::Mat& frame : m_stackFrameHistory)
            {
                if (highBitDepthInput)
                {
                    const cv::Vec<uint16_t, 3>& pixel = frame.ptr<cv::Vec<uint16_t, 3>>(row)[col];
                    for (int channel = 0; channel < 3; ++channel)
                    {
                        const double sample = pixel[channel];
                        sum[channel] += sample;
                        sumSquares[channel] += sample * sample;
                    }
                }
                else
                {
                    const cv::Vec3b& pixel = frame.ptr<cv::Vec3b>(row)[col];
                    for (int channel = 0; channel < 3; ++channel)
                    {
                        const double sample = pixel[channel];
                        sum[channel] += sample;
                        sumSquares[channel] += sample * sample;
                    }
                }
            }

            double mean[3];
            double minValue[3];
            double maxValue[3];
            for (int channel = 0; channel < 3; ++channel)
            {
                mean[channel] = sum[channel] / static_cast<double>(frameCount);
                const double variance = std::max(0.0, (sumSquares[channel] / static_cast<double>(frameCount)) - (mean[channel] * mean[channel]));
                const double sigma = std::sqrt(variance);
                minValue[channel] = mean[channel] - sigmaThreshold * sigma;
                maxValue[channel] = mean[channel] + sigmaThreshold * sigma;
            }

            double clippedSum[3] = {0.0, 0.0, 0.0};
            int clippedCount[3] = {0, 0, 0};

            for (const cv::Mat& frame : m_stackFrameHistory)
            {
                if (highBitDepthInput)
                {
                    const cv::Vec<uint16_t, 3>& pixel = frame.ptr<cv::Vec<uint16_t, 3>>(row)[col];
                    for (int channel = 0; channel < 3; ++channel)
                    {
                        const int sample = pixel[channel];
                        if ((sample >= minValue[channel]) && (sample <= maxValue[channel]))
                        {
                            clippedSum[channel] += sample;
                            ++clippedCount[channel];
                        }
                    }
                }
                else
                {
                    const cv::Vec3b& pixel = frame.ptr<cv::Vec3b>(row)[col];
                    for (int channel = 0; channel < 3; ++channel)
                    {
                        const int sample = pixel[channel];
                        if ((sample >= minValue[channel]) && (sample <= maxValue[channel]))
                        {
                            clippedSum[channel] += sample;
                            ++clippedCount[channel];
                        }
                    }
                }
            }

            for (int channel = 0; channel < 3; ++channel)
            {
                const int channelValue = clippedCount[channel] > 0
                    ? static_cast<int>(std::lround(clippedSum[channel] / static_cast<double>(clippedCount[channel])))
                    : static_cast<int>(std::lround(mean[channel]));
                const int outputValue = highBitDepthInput
                    ? static_cast<int>(std::lround((channelValue * 255.0) / 65535.0))
                    : channelValue;
                output[col * 3 + channel] = static_cast<uchar>(qBound(0, outputValue, 255));
            }
        }
    }

    PROFILER_STOP(__FUNCTION__);
    outputImage = stackedImage;
    stackCount = static_cast<int>(m_stackFrameHistory.size());
    return true;
}
