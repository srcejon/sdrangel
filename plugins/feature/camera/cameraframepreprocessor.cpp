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
#include <QTimer>

#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

#include "util/fits.h"
#include "util/profiler.h"
#include "cameraframealigner.h"
#include "cameraframepreprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraFramePreprocessor::MsgConfigureCameraFramePreprocessor, Message)
MESSAGE_CLASS_DEFINITION(CameraFramePreprocessor::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraFramePreprocessor::MsgCaptureActive, Message)

CameraFramePreprocessor::CameraFramePreprocessor() :
    m_nextStage(nullptr),
    m_captureActive(false),
    m_processingFrame(false),
    m_droppedFrameCount(0)
{
}

CameraFramePreprocessor::~CameraFramePreprocessor() = default;

void CameraFramePreprocessor::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraFramePreprocessor::handleInputMessages);
    handleInputMessages();
}

void CameraFramePreprocessor::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraFramePreprocessor::handleInputMessages);
}

bool CameraFramePreprocessor::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraFramePreprocessor::match(cmd))
    {
        const MsgConfigureCameraFramePreprocessor& cfg = (const MsgConfigureCameraFramePreprocessor&) cmd;
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

void CameraFramePreprocessor::handleInputMessages()
{
    Message *message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraFramePreprocessor::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraFramePreprocessor::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

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
        || settingsKeys.contains("postProcessUseCuda")
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
    }
}

bool CameraFramePreprocessor::preserveFrameOrder() const
{
    return m_captureActive
        && ((m_settings.isHdrStackingEnabled() && (m_settings.getHdrExposureCount() > 1))
            || (m_settings.m_stackEnabled && (m_settings.m_stackFrameCount > 1)));
}

int CameraFramePreprocessor::pendingFrameLimit() const
{
    const int stackFrameCount = m_settings.isHdrStackingEnabled()
        ? m_settings.getHdrExposureCount()
        : m_settings.m_stackFrameCount;
    return qBound(2, stackFrameCount * 2, 512);
}

void CameraFramePreprocessor::submitFrame(const CameraPipelineFramePtr& frame)
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
                qDebug() << "CameraFramePreprocessor: Dropping oldest queued stacking frame";
                m_pendingFrames.pop_front();
                ++m_droppedFrameCount;
            }
            m_pendingFrames.push_back(frame);
        }
        else
        {
            if (!m_pendingFrames.empty())
            {
                qDebug() << "CameraFramePreprocessor: Dropping pending frame in favor of new frame";
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
        QMetaObject::invokeMethod(this, &CameraFramePreprocessor::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraFramePreprocessor::processNextFrame()
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
        QMetaObject::invokeMethod(this, &CameraFramePreprocessor::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraFramePreprocessor::reloadCalibrationFrames()
{
    m_darkCalibrationFrame.release();
    m_flatCalibrationFrame.release();
    m_biasCalibrationFrame.release();
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
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

cv::Mat CameraFramePreprocessor::loadFitsCalibrationFrame(const QString& fileName, const QString& calibrationType, bool normalizeFlat) const
{
    FITS fits(fileName);

    if (!fits.valid())
    {
        qWarning() << "CameraFramePreprocessor: Failed to load" << calibrationType << "calibration FITS:" << fileName;
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
            qWarning() << "CameraFramePreprocessor:" << calibrationType << "calibration FITS has near-zero mean and cannot be normalized:" << fileName;
            return cv::Mat();
        }

        monoFrame /= static_cast<float>(meanValue);
    }

    return monoFrame;
}

void CameraFramePreprocessor::validateCalibrationFrame(cv::Mat& calibrationFrame, const cv::Size& expectedSize, const QString& calibrationType, const QString& fileName)
{
    if (calibrationFrame.empty()) {
        return;
    }

    if (calibrationFrame.size() != expectedSize)
    {
        qWarning() << "CameraFramePreprocessor:" << calibrationType << "calibration FITS size"
                   << calibrationFrame.cols << "x" << calibrationFrame.rows
                   << "does not match frame size" << expectedSize.width << "x" << expectedSize.height
                   << "- disabling calibration file:" << fileName;
        calibrationFrame.release();
    }
}

cv::Mat CameraFramePreprocessor::applyCalibration(const cv::Mat& input)
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
        qWarning() << "CameraFramePreprocessor: bias calibration channel count does not match input";
    }
    if (!m_darkCalibrationFrame.empty() && darkFrame.empty()) {
        qWarning() << "CameraFramePreprocessor: dark calibration channel count does not match input";
    }
    if (!m_flatCalibrationFrame.empty() && flatFrame.empty()) {
        qWarning() << "CameraFramePreprocessor: flat calibration channel count does not match input";
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

void CameraFramePreprocessor::processNewFrame(const CameraPipelineFramePtr& frame)
{
    PROFILER_START();

    if (!frame || !frame->hasImageData()) {
        return;
    }

    frame->ensureCpuImageFromCuda();
    if (frame->m_image.isNull()) {
        return;
    }

    cv::Mat frameMat = imageToWorkingMat(frame->m_image);

#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    if (m_settings.m_postProcessUseCuda && canUseCudaPreprocessing() && preprocessFrameCuda(*frame, frameMat))
    {
        if (m_nextStage) {
            m_nextStage->submitFrame(frame);
        }
        PROFILER_STOP(__FUNCTION__);
        return;
    }
#endif

    frameMat = applyCalibration(frameMat);
    frameMat = debayerRawMat(frameMat, frame->m_bayerPattern);
    if (frameMat.channels() == 1) {
        cv::cvtColor(frameMat, frameMat, cv::COLOR_GRAY2RGB);
    }

    frame->m_image = workingMatToImage(frameMat);
    frame->m_bayerPattern = CameraPipelineFrame::BayerNone;
    frame->clearCudaCache();
    if (shouldMaterializeUnprocessedImage(*frame)) {
        frame->m_unprocessedImage = frame->m_image;
    } else {
        frame->m_unprocessedImage = QImage();
    }

    if (m_nextStage) {
        m_nextStage->submitFrame(frame);
    }

    PROFILER_STOP(__FUNCTION__);
}

#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
bool CameraFramePreprocessor::canUseCudaPreprocessing() const
{
    static bool warnedNoDevice = false;

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraFramePreprocessor: CUDA preprocessing requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    return true;
}

void CameraFramePreprocessor::invalidateCudaCalibrationFrames()
{
    m_cudaDarkCalibrationFrame = CudaCalibrationFrame();
    m_cudaFlatCalibrationFrame = CudaCalibrationFrame();
    m_cudaBiasCalibrationFrame = CudaCalibrationFrame();
}

cv::cuda::GpuMat CameraFramePreprocessor::uploadCalibrationFrameCuda(
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
        uploaded.upload(calibrationFrame, m_cudaStream);
    }
    else if ((calibrationFrame.channels() == 1) && (channels == 3))
    {
        cv::cuda::GpuMat monoGpu;
        monoGpu.upload(calibrationFrame, m_cudaStream);
        std::vector<cv::cuda::GpuMat> planes(3, monoGpu);
        cv::cuda::merge(planes, uploaded, m_cudaStream);
    }

    cachedFrame.m_frame = uploaded;
    cachedFrame.m_sourceSize = calibrationFrame.size();
    cachedFrame.m_sourceType = calibrationFrame.type();
    cachedFrame.m_channels = channels;
    return cachedFrame.m_frame;
}

bool CameraFramePreprocessor::applyCalibrationCuda(cv::cuda::GpuMat& frameGpu, const cv::Size& inputSize, int inputType)
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
        frameGpu.convertTo(calibratedGpu, floatType, m_cudaStream);

        const cv::cuda::GpuMat biasGpu = uploadCalibrationFrameCuda(m_cudaBiasCalibrationFrame, m_biasCalibrationFrame, channels);
        const cv::cuda::GpuMat darkGpu = uploadCalibrationFrameCuda(m_cudaDarkCalibrationFrame, m_darkCalibrationFrame, channels);
        const cv::cuda::GpuMat flatGpu = uploadCalibrationFrameCuda(m_cudaFlatCalibrationFrame, m_flatCalibrationFrame, channels);

        if (!biasGpu.empty()) {
            cv::cuda::subtract(calibratedGpu, biasGpu, calibratedGpu, cv::noArray(), -1, m_cudaStream);
        }
        if (!darkGpu.empty()) {
            cv::cuda::subtract(calibratedGpu, darkGpu, calibratedGpu, cv::noArray(), -1, m_cudaStream);
        }
        if (!flatGpu.empty())
        {
            cv::cuda::GpuMat safeFlatGpu;
            cv::cuda::GpuMat epsilonGpu(flatGpu.size(), flatGpu.type(), cv::Scalar::all(1.0e-6));
            cv::cuda::max(flatGpu, epsilonGpu, safeFlatGpu, m_cudaStream);
            cv::cuda::divide(calibratedGpu, safeFlatGpu, calibratedGpu, 1.0, -1, m_cudaStream);
        }

        calibratedGpu.convertTo(frameGpu, inputType, m_cudaStream);
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFramePreprocessor: CUDA calibration failed; falling back to CPU:" << error.what();
        invalidateCudaCalibrationFrames();
    }

    return false;
}

bool CameraFramePreprocessor::preprocessFrameCuda(CameraPipelineFrame& frame, const cv::Mat& inputMat)
{
    try
    {
        cv::cuda::GpuMat frameGpu;
        frameGpu.upload(inputMat, m_cudaStream);

        if (!applyCalibrationCuda(frameGpu, inputMat.size(), inputMat.type())) {
            return false;
        }

        cv::cuda::GpuMat bgrGpu;
        const int cvCode = bayerPatternToOpenCvCode(frame.m_bayerPattern);
        if ((cvCode >= 0) && (frameGpu.channels() == 1))
        {
            cv::cuda::GpuMat debayeredGpu;
            cv::cuda::demosaicing(frameGpu, debayeredGpu, cvCode, 0, m_cudaStream);
            cv::cuda::cvtColor(debayeredGpu, bgrGpu, cv::COLOR_RGB2BGR, 0, m_cudaStream);
        }
        else if (frameGpu.channels() == 1)
        {
            cv::cuda::cvtColor(frameGpu, bgrGpu, cv::COLOR_GRAY2BGR, 0, m_cudaStream);
        }
        else
        {
            cv::cuda::cvtColor(frameGpu, bgrGpu, cv::COLOR_RGB2BGR, 0, m_cudaStream);
        }

        if (bgrGpu.type() != CV_8UC3)
        {
            cv::cuda::GpuMat bgr8uGpu;
            const double scale = bgrGpu.depth() == CV_16U ? (255.0 / 65535.0) : 1.0;
            bgrGpu.convertTo(bgr8uGpu, CV_8UC3, scale, 0.0, m_cudaStream);
            bgrGpu = bgr8uGpu;
        }

        bgrGpu.copyTo(frame.m_cudaBgrImage, m_cudaStream);
        frame.m_cudaGrayImage.release();
        frame.m_bayerPattern = CameraPipelineFrame::BayerNone;
        if (shouldMaterializeUnprocessedImage(frame)) {
            frame.m_unprocessedImage = downloadCudaBgrImage(frame.m_cudaBgrImage);
        } else {
            frame.m_unprocessedImage = QImage();
        }
        m_cudaStream.waitForCompletion();
        frame.clearCpuImage();
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFramePreprocessor: CUDA preprocessing failed; falling back to CPU:" << error.what();
        frame.clearCudaCache();
    }

    return false;
}

QImage CameraFramePreprocessor::downloadCudaBgrImage(const cv::cuda::GpuMat& bgrGpu)
{
    cv::Mat bgrMat;
    bgrGpu.download(bgrMat, m_cudaStream);
    m_cudaStream.waitForCompletion();
    QImage image(bgrMat.cols, bgrMat.rows, QImage::Format_RGB888);
    cv::Mat rgbMat(image.height(), image.width(), CV_8UC3, image.bits(), static_cast<size_t>(image.bytesPerLine()));
    cv::cvtColor(bgrMat, rgbMat, cv::COLOR_BGR2RGB);
    return image;
}
#endif

bool CameraFramePreprocessor::shouldMaterializeUnprocessedImage(const CameraPipelineFrame& frame) const
{
    const bool rawMediaRequested = (m_settings.m_recordMode == CameraSettings::SavedMediaRaw)
        || (m_settings.m_recordMode == CameraSettings::SavedMediaBoth);

    if (!rawMediaRequested) {
        return false;
    }

    return m_settings.m_saveImage
        || m_settings.m_saveVideo
        || frame.m_saveCurrentImage
        || (m_settings.m_videoPreRecordBufferSeconds > 0);
}

int CameraFramePreprocessor::bayerPatternToOpenCvCode(CameraPipelineFrame::BayerPattern bayerPattern)
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

cv::Mat CameraFramePreprocessor::imageToWorkingMat(const QImage& input)
{
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

QImage CameraFramePreprocessor::workingMatToImage(const cv::Mat& frameMat)
{
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

cv::Mat CameraFramePreprocessor::debayerRawMat(const cv::Mat& input, CameraPipelineFrame::BayerPattern bayerPattern)
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
