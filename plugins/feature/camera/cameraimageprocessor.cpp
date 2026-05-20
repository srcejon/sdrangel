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

#include <QDebug>
#include <QImage>

#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/profiler.h"
#include "cameradetector.h"
#include "cameraimageprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraImageProcessor::MsgConfigureCameraImageProcessor, Message)
MESSAGE_CLASS_DEFINITION(CameraImageProcessor::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(CameraImageProcessor::MsgCaptureActive, Message)

CameraImageProcessor::CameraImageProcessor() :
    m_nextStage(nullptr),
    m_captureActive(false),
    m_autoWhiteBalanceGains(1.0, 1.0, 1.0),
    m_autoWhiteBalanceInitialized(false),
    m_unwarpSourceProjection(CameraSettings::LensProjectionRectilinear),
    m_unwarpSourceFov(0.0),
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    m_cudaUnwarpMapSize(),
    m_cudaHistogramStretchMode(CameraSettings::HistogramStretchOff),
    m_cudaHistogramStretchBlackPoint(0.0),
    m_cudaHistogramStretchWhitePoint(0.0),
    m_cudaHistogramStretchGamma(0.0),
    m_cudaHistogramStretchAsinhStrength(0.0),
    m_cudaHistogramStretchLogStrength(0.0),
    m_cudaGamma(0.0),
    m_cudaGaussianBlurKernelSize(0),
    m_cudaGaussianBlurType(-1),
    m_cudaMedianBlurKernelSize(0),
    m_cudaMedianBlurChannelType(-1),
    m_cudaSharpenBlurType(-1),
    m_cudaSobelInputType(-1),
#endif
    m_processingFrame(false)
{
}

CameraImageProcessor::~CameraImageProcessor() = default;

void CameraImageProcessor::startWork()
{
    QObject::connect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraImageProcessor::handleInputMessages);
    handleInputMessages();
}

void CameraImageProcessor::stopWork()
{
    QObject::disconnect(&m_inputMessageQueue, &MessageQueue::messageEnqueued, this, &CameraImageProcessor::handleInputMessages);
}

bool CameraImageProcessor::handleMessage(const Message& cmd)
{
    if (MsgConfigureCameraImageProcessor::match(cmd))
    {
        const MsgConfigureCameraImageProcessor& cfg = (const MsgConfigureCameraImageProcessor&) cmd;
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
        if (m_captureActive)
        {
            m_lastInputFrame = CameraPipelineFrame();
            m_autoWhiteBalanceGains = cv::Vec3d(1.0, 1.0, 1.0);
            m_autoWhiteBalanceInitialized = false;
            invalidateUnwarpMaps();
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

void CameraImageProcessor::handleInputMessages()
{
    Message *message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraImageProcessor::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "CameraImageProcessor::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    static const QStringList kImageProcessingKeys = {
        "postProcessWhiteBalanceMode",
        "postProcessWhiteBalanceRedGain",
        "postProcessWhiteBalanceGreenGain",
        "postProcessWhiteBalanceBlueGain",
        "postProcessWhiteBalanceHighlightProtection",
        "postProcessUseCuda",
        "postProcessUnwarp",
        "histogramStretch",
        "histogramStretchBlackPoint",
        "histogramStretchWhitePoint",
        "histogramStretchGamma",
        "histogramStretchAsinhStrength",
        "histogramStretchLogStrength",
        "postProcessGreyscale",
        "saturation", "gamma", "gaussianBlur", "medianBlur", "sharpen", "edgeDisplayMode", "sobelEdge", "cannyEdge", "flipX", "flipY",
        "brightness", "contrast", "invertColors"
    };
    const bool imageProcessingChanged = force || std::any_of(kImageProcessingKeys.cbegin(), kImageProcessingKeys.cend(),
        [&settingsKeys](const QString& k) { return settingsKeys.contains(k); });
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
        || settingsKeys.contains("stackFrameCount")
        || settingsKeys.contains("stackMethod")
        || settingsKeys.contains("stackAlignmentMethod")
        || settingsKeys.contains("lensProjection")
        || settingsKeys.contains("fov");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sourceChanged) {
        m_lastInputFrame = CameraPipelineFrame();
        invalidateUnwarpMaps();
    }

    if (force
        || settingsKeys.contains("postProcessWhiteBalanceMode")
        || settingsKeys.contains("postProcessWhiteBalanceRedGain")
        || settingsKeys.contains("postProcessWhiteBalanceGreenGain")
        || settingsKeys.contains("postProcessWhiteBalanceBlueGain")
        || settingsKeys.contains("postProcessWhiteBalanceHighlightProtection")
        || settingsKeys.contains("postProcessGreyscale"))
    {
        m_autoWhiteBalanceGains = cv::Vec3d(1.0, 1.0, 1.0);
        m_autoWhiteBalanceInitialized = false;
    }

    if (force
        || settingsKeys.contains("postProcessUnwarp")
        || settingsKeys.contains("lensProjection")
        || settingsKeys.contains("fov"))
    {
        invalidateUnwarpMaps();
    }

#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    if (force
        || settingsKeys.contains("postProcessUseCuda")
        || settingsKeys.contains("postProcessUnwarp")
        || settingsKeys.contains("lensProjection")
        || settingsKeys.contains("fov")
        || settingsKeys.contains("histogramStretch")
        || settingsKeys.contains("histogramStretchBlackPoint")
        || settingsKeys.contains("histogramStretchWhitePoint")
        || settingsKeys.contains("histogramStretchGamma")
        || settingsKeys.contains("histogramStretchAsinhStrength")
        || settingsKeys.contains("histogramStretchLogStrength")
        || settingsKeys.contains("gamma")
        || settingsKeys.contains("gaussianBlur")
        || settingsKeys.contains("medianBlur")
        || settingsKeys.contains("sharpen")
        || settingsKeys.contains("sobelEdge")
        || settingsKeys.contains("cannyEdge"))
    {
        invalidateCudaProcessingCaches();
    }
#endif

    if (imageProcessingChanged && m_lastInputFrame.hasImageData()) {
        CameraPipelineFramePtr frame(new CameraPipelineFrame(m_lastInputFrame));
        submitFrame(frame);
    }
}

void CameraImageProcessor::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }

    bool schedule = false;
    {
        QMutexLocker locker(&m_frameMutex);
        if (m_pendingFrame) {
            qDebug() << "CameraImageProcessor: Dropping pending frame in favor of new frame";
        }
        m_pendingFrame = frame;
        if (!m_processingFrame)
        {
            m_processingFrame = true;
            schedule = true;
        }
    }

    if (schedule) {
        QMetaObject::invokeMethod(this, &CameraImageProcessor::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraImageProcessor::processNextFrame()
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
        QMetaObject::invokeMethod(this, &CameraImageProcessor::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraImageProcessor::processNewFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !frame->hasImageData()) {
        return;
    }

    m_lastInputFrame = *frame;

    applyImageProcessing(*frame);
    frame->m_histogramData = m_settings.m_histogramVisible
        ? computeHistogramData(*frame)
        : CameraHistogramData();

    if (m_nextStage) {
        m_nextStage->submitFrame(frame);
    }
}

CameraHistogramData CameraImageProcessor::computeHistogramData(const CameraPipelineFrame& frame)
{
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    if (m_settings.m_postProcessUseCuda && (cv::cuda::getCudaEnabledDeviceCount() > 0))
    {
        const CameraHistogramData histogramData = computeHistogramDataCuda(frame);
        if (histogramData.isValid()) {
            return histogramData;
        }
    }
#endif

    CameraPipelineFrame& mutableFrame = const_cast<CameraPipelineFrame&>(frame);
    if (mutableFrame.ensureCpuImageFromCuda()) {
        return computeHistogramDataCpu(mutableFrame.m_image);
    }
    return CameraHistogramData();
}

CameraHistogramData CameraImageProcessor::computeHistogramDataCpu(const QImage& image)
{
    PROFILER_START();
    CameraHistogramData histogramData;

    if (image.isNull()) {
        return histogramData;
    }

    const QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(rgb.height(), rgb.width(), CV_8UC3,
                const_cast<uchar*>(rgb.bits()),
                static_cast<size_t>(rgb.bytesPerLine()));
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);

    std::vector<cv::Mat> channels;
    cv::split(bgrMat, channels);

    constexpr int histSize = 256;
    const float range[] = {0.0f, 256.0f};
    const float* histRange = range;

    auto fillBins = [&](int channelIndex, QVector<float>& bins)
    {
        cv::Mat hist;
        cv::calcHist(&channels[channelIndex], 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange);
        bins.resize(histSize);
        for (int i = 0; i < histSize; ++i) {
            bins[i] = hist.at<float>(i);
        }
    };

    fillBins(2, histogramData.m_redBins);
    fillBins(1, histogramData.m_greenBins);
    fillBins(0, histogramData.m_blueBins);

    PROFILER_STOP(__FUNCTION__);
    return histogramData;
}

#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
CameraHistogramData CameraImageProcessor::computeHistogramDataCuda(const CameraPipelineFrame& frame)
{
    PROFILER_START();
    CameraHistogramData histogramData;

    if (!frame.hasImageData()) {
        return histogramData;
    }

    try
    {
        cv::cuda::GpuMat bgrGpu;

        if (frame.hasCudaBgrImage())
        {
            bgrGpu = frame.m_cudaBgrImage;
        }
        else
        {
            QImage convertedRgb;
            const QImage& rgb = ensureRgb888(frame.m_image, convertedRgb);
            cv::Mat rgbMat = wrapRgb888Image(rgb);
            cv::cuda::GpuMat rgbGpu;
            rgbGpu.upload(rgbMat, m_cudaStream);
            cv::cuda::cvtColor(rgbGpu, bgrGpu, cv::COLOR_RGB2BGR, 0, m_cudaStream);
        }

        std::vector<cv::cuda::GpuMat> channels;
        cv::cuda::split(bgrGpu, channels, m_cudaStream);

        if (channels.size() < 3) {
            return histogramData;
        }

        cv::cuda::GpuMat blueHistGpu;
        cv::cuda::GpuMat greenHistGpu;
        cv::cuda::GpuMat redHistGpu;
        cv::cuda::calcHist(channels[0], blueHistGpu, m_cudaStream);
        cv::cuda::calcHist(channels[1], greenHistGpu, m_cudaStream);
        cv::cuda::calcHist(channels[2], redHistGpu, m_cudaStream);

        cv::Mat blueHist;
        cv::Mat greenHist;
        cv::Mat redHist;
        blueHistGpu.download(blueHist, m_cudaStream);
        greenHistGpu.download(greenHist, m_cudaStream);
        redHistGpu.download(redHist, m_cudaStream);
        m_cudaStream.waitForCompletion();

        fillHistogramBinsFromCudaMat(redHist, histogramData.m_redBins);
        fillHistogramBinsFromCudaMat(greenHist, histogramData.m_greenBins);
        fillHistogramBinsFromCudaMat(blueHist, histogramData.m_blueBins);
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraImageProcessor: CUDA histogram failed; falling back to CPU:" << error.what();
        histogramData = CameraHistogramData();
    }

    PROFILER_STOP(__FUNCTION__);
    return histogramData;
}

void CameraImageProcessor::fillHistogramBinsFromCudaMat(const cv::Mat& hist, QVector<float>& bins)
{
    constexpr int histSize = 256;
    if (hist.empty()) {
        return;
    }

    bins.resize(histSize);
    cv::Mat histRow = hist.reshape(1, 1);
    for (int i = 0; i < histSize; ++i) {
        bins[i] = static_cast<float>(histRow.at<int>(i));
    }
}

#endif

void CameraImageProcessor::applyImageProcessing(CameraPipelineFrame& frame)
{
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    if (m_settings.m_postProcessUseCuda && canUseCudaImageProcessing())
    {
        applyImageProcessingCuda(frame);
        return;
    }
#endif

    applyImageProcessingCpu(frame);
}

void CameraImageProcessor::applyImageProcessingCpu(CameraPipelineFrame& frame)
{
    PROFILER_START();

    if (!frame.ensureCpuImageFromCuda())
    {
        qWarning() << "CameraImageProcessor: CPU image processing received a frame with no CPU image data";
        PROFILER_STOP("CameraImageProcessor::applyImageProcessing");
        return;
    }

    frame.clearCudaCache();
    const QImage& input = frame.m_image;
    const bool needsWhiteBalance = m_settings.m_postProcessWhiteBalanceMode != 0;
    const bool needsUnwarp = m_settings.m_postProcessUnwarp && (m_settings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
    const bool needsHistogramStretch = (m_settings.m_histogramStretch != CameraSettings::HistogramStretchOff)
        && (m_settings.m_histogramStretchWhitePoint > m_settings.m_histogramStretchBlackPoint + 1e-6);
    const bool needsGreyscale = m_settings.m_postProcessGreyscale;
    const bool needsSaturation = !needsGreyscale && (std::abs(m_settings.m_saturation - 1.0) > 1e-4);
    const bool needsGamma = std::abs(m_settings.m_gamma - 1.0) > 1e-4;
    const bool needsGaussianBlur = m_settings.m_gaussianBlur > 0;
    const bool needsMedianBlur = m_settings.m_medianBlur > 0;
    const bool needsSharpen = m_settings.m_sharpen > 1e-4;
    const bool needsSobelEdge = m_settings.m_sobelEdge > 1e-4;
    const bool needsCannyEdge = m_settings.m_cannyEdge > 1e-4;
    const bool needsFlip = m_settings.m_flipX || m_settings.m_flipY;
    const bool needsBrightContrast = (m_settings.m_brightness != 0.0 || m_settings.m_contrast != 1.0);
    const bool needsAny = needsWhiteBalance
        || needsUnwarp
        || needsHistogramStretch
        || needsSaturation
        || needsGamma
        || needsGaussianBlur
        || needsMedianBlur
        || needsSharpen
        || needsSobelEdge
        || needsCannyEdge
        || needsFlip
        || needsBrightContrast
        || needsGreyscale
        || m_settings.m_invertColors;

    if (!needsAny)
    {
        PROFILER_STOP("CameraImageProcessor::applyImageProcessing");
        return;
    }

    QImage convertedRgb;
    const QImage& rgb = ensureRgb888(input, convertedRgb);
    cv::Mat mat = wrapRgb888Image(rgb);
    cv::Mat bgrMat;
    cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);

    if (needsUnwarp) {
        applyLensUnwarp(bgrMat);
    }
    if (needsWhiteBalance) {
        applyWhiteBalance(bgrMat);
    }
    if (needsHistogramStretch) {
        applyHistogramStretch(bgrMat);
    }
    if (needsGreyscale) {
        applyGreyscale(bgrMat);
    }
    if (needsSaturation) {
        applySaturation(bgrMat);
    }
    if (needsGamma) {
        applyGamma(bgrMat);
    }
    if (needsGaussianBlur) {
        applyGaussianBlur(bgrMat);
    }
    if (needsMedianBlur) {
        applyMedianBlur(bgrMat);
    }
    if (needsSharpen) {
        applySharpen(bgrMat);
    }
    if (needsSobelEdge) {
        applySobelEdge(bgrMat);
    }
    if (needsCannyEdge) {
        applyCannyEdge(bgrMat);
    }
    if (needsFlip) {
        applyFlip(bgrMat);
    }
    if (needsBrightContrast) {
        applyBrightnessContrast(bgrMat);
    }
    if (m_settings.m_invertColors) {
        applyInvertColors(bgrMat);
    }

    QImage result = convertBgrToRgbImage(bgrMat);
    PROFILER_STOP("CameraImageProcessor::applyImageProcessing");
    frame.m_image = result;
}

#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
bool CameraImageProcessor::canUseCudaImageProcessing() const
{
    static bool warnedNoDevice = false;
    static bool warnedUnsupportedSettings = false;

    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraImageProcessor: CUDA post-processing requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    const bool manualHighlightProtectedWhiteBalance =
        (m_settings.m_postProcessWhiteBalanceMode == 2)
        && (m_settings.m_postProcessWhiteBalanceHighlightProtection > 1e-6);
    const bool unsupported =
        (m_settings.m_postProcessWhiteBalanceMode == 1)
        || manualHighlightProtectedWhiteBalance;

    if (unsupported)
    {
        if (!warnedUnsupportedSettings)
        {
            qDebug() << "CameraImageProcessor: CUDA post-processing requested, but current settings need CPU-only post-processing";
            warnedUnsupportedSettings = true;
        }
        return false;
    }

    return true;
}

void CameraImageProcessor::applyImageProcessingCuda(CameraPipelineFrame& frame)
{
    PROFILER_START();

    const QImage& input = frame.m_image;
    const bool needsWhiteBalance = m_settings.m_postProcessWhiteBalanceMode != 0;
    const bool needsUnwarp = m_settings.m_postProcessUnwarp && (m_settings.m_lensProjection != CameraSettings::LensProjectionRectilinear);
    const bool needsHistogramStretch = (m_settings.m_histogramStretch != CameraSettings::HistogramStretchOff)
        && (m_settings.m_histogramStretchWhitePoint > m_settings.m_histogramStretchBlackPoint + 1e-6);
    const bool needsGreyscale = m_settings.m_postProcessGreyscale;
    const bool needsSaturation = !needsGreyscale && (std::abs(m_settings.m_saturation - 1.0) > 1e-4);
    const bool needsGamma = std::abs(m_settings.m_gamma - 1.0) > 1e-4;
    const bool needsGaussianBlur = m_settings.m_gaussianBlur > 0;
    const bool needsMedianBlur = m_settings.m_medianBlur > 0;
    const bool needsSharpen = m_settings.m_sharpen > 1e-4;
    const bool needsSobelEdge = m_settings.m_sobelEdge > 1e-4;
    const bool needsCannyEdge = m_settings.m_cannyEdge > 1e-4;
    const bool needsFlip = m_settings.m_flipX || m_settings.m_flipY;
    const bool needsBrightContrast = (m_settings.m_brightness != 0.0 || m_settings.m_contrast != 1.0);
    const bool needsAny = needsWhiteBalance
        || needsUnwarp
        || needsHistogramStretch
        || needsSaturation
        || needsGamma
        || needsGaussianBlur
        || needsMedianBlur
        || needsSharpen
        || needsSobelEdge
        || needsCannyEdge
        || needsFlip
        || needsBrightContrast
        || needsGreyscale
        || m_settings.m_invertColors;

    if (!needsAny) {
        return;
    }

    try
    {
        cv::cuda::GpuMat bgrGpu;
        if (frame.hasCudaBgrImage())
        {
            bgrGpu = frame.m_cudaBgrImage;
        }
        else
        {
            QImage convertedRgb;
            const QImage& rgb = ensureRgb888(input, convertedRgb);
            cv::Mat rgbMat = wrapRgb888Image(rgb);
            cv::cuda::GpuMat gpuRgb;
            gpuRgb.upload(rgbMat, m_cudaStream);
            cv::cuda::cvtColor(gpuRgb, bgrGpu, cv::COLOR_RGB2BGR, 0, m_cudaStream);
        }

        if (needsUnwarp) {
            applyLensUnwarpCuda(bgrGpu, m_cudaStream);
        }
        if (needsWhiteBalance) {
            applyWhiteBalanceCuda(bgrGpu, m_cudaStream);
        }
        if (needsHistogramStretch) {
            applyHistogramStretchCuda(bgrGpu, m_cudaStream);
        }
        if (needsGreyscale) {
            applyGreyscaleCuda(bgrGpu, m_cudaStream);
        }
        if (needsSaturation) {
            applySaturationCuda(bgrGpu, m_cudaStream);
        }
        if (needsGamma) {
            applyGammaCuda(bgrGpu, m_cudaStream);
        }
        if (needsGaussianBlur) {
            applyGaussianBlurCuda(bgrGpu, m_cudaStream);
        }
        if (needsMedianBlur) {
            applyMedianBlurCuda(bgrGpu, m_cudaStream);
        }
        if (needsSharpen) {
            applySharpenCuda(bgrGpu, m_cudaStream);
        }
        if (needsSobelEdge) {
            applySobelEdgeCuda(bgrGpu, m_cudaStream);
        }
        if (needsCannyEdge) {
            applyCannyEdgeCuda(bgrGpu, m_cudaStream);
        }
        if (needsFlip)
        {
            const int flipCode = m_settings.m_flipX && m_settings.m_flipY ? -1 : (m_settings.m_flipX ? 1 : 0);
            cv::cuda::GpuMat flippedGpu;
            cv::cuda::flip(bgrGpu, flippedGpu, flipCode, m_cudaStream);
            bgrGpu = flippedGpu;
        }
        if (needsBrightContrast) {
            bgrGpu.convertTo(bgrGpu, -1, m_settings.m_contrast, m_settings.m_brightness, m_cudaStream);
        }
        if (m_settings.m_invertColors) {
            cv::cuda::bitwise_not(bgrGpu, bgrGpu, cv::noArray(), m_cudaStream);
        }

        bgrGpu.copyTo(frame.m_cudaBgrImage, m_cudaStream);
        m_cudaStream.waitForCompletion();
        frame.m_cudaGrayImage.release();
        frame.clearCpuImage();
        PROFILER_STOP("CameraImageProcessor::applyImageProcessingCuda");
        return;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraImageProcessor: CUDA post-processing failed; falling back to CPU:" << error.what();
    }

    applyImageProcessingCpu(frame);
}

void CameraImageProcessor::applyLensUnwarpCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream)
{
    PROFILER_START();

    ensureUnwarpMaps(bgrGpu.size());
    if (!m_unwarpMapX.empty() && !m_unwarpMapY.empty())
    {
        cv::cuda::GpuMat unwarpedGpu;
        if (m_cudaUnwarpMapX.empty()
            || m_cudaUnwarpMapY.empty()
            || (m_cudaUnwarpMapSize != bgrGpu.size()))
        {
            m_cudaUnwarpMapX.upload(m_unwarpMapX, stream);
            m_cudaUnwarpMapY.upload(m_unwarpMapY, stream);
            m_cudaUnwarpMapSize = bgrGpu.size();
        }
        cv::cuda::remap(bgrGpu, unwarpedGpu, m_cudaUnwarpMapX, m_cudaUnwarpMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), stream);
        bgrGpu = unwarpedGpu;
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyWhiteBalanceCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    std::vector<cv::cuda::GpuMat> channels;
    cv::cuda::split(bgrGpu, channels, stream);
    channels[0].convertTo(channels[0], -1, m_settings.m_postProcessWhiteBalanceBlueGain, 0.0, stream);
    channels[1].convertTo(channels[1], -1, m_settings.m_postProcessWhiteBalanceGreenGain, 0.0, stream);
    channels[2].convertTo(channels[2], -1, m_settings.m_postProcessWhiteBalanceRedGain, 0.0, stream);
    cv::cuda::merge(channels, bgrGpu, stream);

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyHistogramStretchCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    if (m_settings.m_histogramStretch == CameraSettings::HistogramStretchCLAHE)
    {
        cv::cuda::GpuMat labGpu;
        std::vector<cv::cuda::GpuMat> labChannels;
        cv::cuda::cvtColor(bgrGpu, labGpu, cv::COLOR_BGR2Lab, 0, stream);
        cv::cuda::split(labGpu, labChannels, stream);

        if (!m_cudaClahe) {
            m_cudaClahe = cv::cuda::createCLAHE(2.0, cv::Size(8, 8));
        }
        cv::cuda::GpuMat equalizedL;
        m_cudaClahe->apply(labChannels[0], equalizedL, stream);
        labChannels[0] = equalizedL;

        cv::cuda::merge(labChannels, labGpu, stream);
        cv::cuda::cvtColor(labGpu, bgrGpu, cv::COLOR_Lab2BGR, 0, stream);
        PROFILER_STOP(__FUNCTION__);
        return;
    }

    const float blackPoint = static_cast<float>(m_settings.m_histogramStretchBlackPoint);
    const float whitePoint = static_cast<float>(m_settings.m_histogramStretchWhitePoint);
    const float rangeScale = 1.0f / std::max(0.001f, whitePoint - blackPoint);

    const float gammaValue = static_cast<float>(m_settings.m_histogramStretchGamma);
    const float asinhStrength = static_cast<float>(m_settings.m_histogramStretchAsinhStrength);
    const float logStrength = static_cast<float>(m_settings.m_histogramStretchLogStrength);
    const float asinhNorm = std::asinh(asinhStrength);
    const float logNorm = std::log1p(logStrength);

    if (!m_cudaHistogramStretchLookup
        || (m_cudaHistogramStretchMode != m_settings.m_histogramStretch)
        || (m_cudaHistogramStretchBlackPoint != m_settings.m_histogramStretchBlackPoint)
        || (m_cudaHistogramStretchWhitePoint != m_settings.m_histogramStretchWhitePoint)
        || (m_cudaHistogramStretchGamma != m_settings.m_histogramStretchGamma)
        || (m_cudaHistogramStretchAsinhStrength != m_settings.m_histogramStretchAsinhStrength)
        || (m_cudaHistogramStretchLogStrength != m_settings.m_histogramStretchLogStrength))
    {
        cv::Mat lut(1, 256, CV_8U);
        uchar *lutData = lut.ptr<uchar>();

        for (int i = 0; i < 256; ++i)
        {
            float value = ((static_cast<float>(i) / 255.0f) - blackPoint) * rangeScale;
            value = std::clamp(value, 0.0f, 1.0f);

            switch (m_settings.m_histogramStretch)
            {
            case CameraSettings::HistogramStretchLinear:
                break;
            case CameraSettings::HistogramStretchGamma:
                value = std::pow(value, gammaValue);
                break;
            case CameraSettings::HistogramStretchAsinh:
                value = (asinhNorm > 0.0f) ? (std::asinh(asinhStrength * value) / asinhNorm) : value;
                break;
            case CameraSettings::HistogramStretchLog:
                value = (logNorm > 0.0f) ? (std::log1p(logStrength * value) / logNorm) : value;
                break;
            case CameraSettings::HistogramStretchCLAHE:
            case CameraSettings::HistogramStretchOff:
            default:
                break;
            }

            lutData[i] = cv::saturate_cast<uchar>(std::clamp(value, 0.0f, 1.0f) * 255.0f);
        }

        m_cudaHistogramStretchLookup = cv::cuda::createLookUpTable(lut);
        m_cudaHistogramStretchMode = m_settings.m_histogramStretch;
        m_cudaHistogramStretchBlackPoint = m_settings.m_histogramStretchBlackPoint;
        m_cudaHistogramStretchWhitePoint = m_settings.m_histogramStretchWhitePoint;
        m_cudaHistogramStretchGamma = m_settings.m_histogramStretchGamma;
        m_cudaHistogramStretchAsinhStrength = m_settings.m_histogramStretchAsinhStrength;
        m_cudaHistogramStretchLogStrength = m_settings.m_histogramStretchLogStrength;
    }

    cv::cuda::GpuMat stretchedGpu;
    m_cudaHistogramStretchLookup->transform(bgrGpu, stretchedGpu, stream);
    bgrGpu = stretchedGpu;

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyGreyscaleCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    cv::cuda::GpuMat grayGpu;
    cv::cuda::cvtColor(bgrGpu, grayGpu, cv::COLOR_BGR2GRAY, 0, stream);
    cv::cuda::cvtColor(grayGpu, bgrGpu, cv::COLOR_GRAY2BGR, 0, stream);

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applySaturationCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    cv::cuda::GpuMat hsvGpu;
    std::vector<cv::cuda::GpuMat> channels;
    cv::cuda::cvtColor(bgrGpu, hsvGpu, cv::COLOR_BGR2HSV, 0, stream);
    cv::cuda::split(hsvGpu, channels, stream);
    channels[1].convertTo(channels[1], -1, m_settings.m_saturation, 0.0, stream);
    cv::cuda::merge(channels, hsvGpu, stream);
    cv::cuda::cvtColor(hsvGpu, bgrGpu, cv::COLOR_HSV2BGR, 0, stream);

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyGammaCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();
    if (!m_cudaGammaLookup || (m_cudaGamma != m_settings.m_gamma))
    {
        cv::Mat lut(1, 256, CV_8U);
        uchar *lutData = lut.ptr<uchar>();

        for (int i = 0; i < 256; ++i) {
            lutData[i] = cv::saturate_cast<uchar>(std::pow(static_cast<double>(i) / 255.0, m_settings.m_gamma) * 255.0);
        }

        m_cudaGammaLookup = cv::cuda::createLookUpTable(lut);
        m_cudaGamma = m_settings.m_gamma;
    }

    cv::cuda::GpuMat correctedGpu;
    m_cudaGammaLookup->transform(bgrGpu, correctedGpu, stream);
    bgrGpu = correctedGpu;
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyGaussianBlurCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    const int kernelSize = 2 * m_settings.m_gaussianBlur + 1;
    cv::cuda::GpuMat blurredGpu;
    if (!m_cudaGaussianBlurFilter
        || (m_cudaGaussianBlurKernelSize != kernelSize)
        || (m_cudaGaussianBlurType != bgrGpu.type()))
    {
        m_cudaGaussianBlurFilter = cv::cuda::createGaussianFilter(
            bgrGpu.type(), bgrGpu.type(), cv::Size(kernelSize, kernelSize), 0.0);
        m_cudaGaussianBlurKernelSize = kernelSize;
        m_cudaGaussianBlurType = bgrGpu.type();
    }
    m_cudaGaussianBlurFilter->apply(bgrGpu, blurredGpu, stream);
    bgrGpu = blurredGpu;

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyMedianBlurCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    const int kernelSize = 2 * m_settings.m_medianBlur + 1;
    std::vector<cv::cuda::GpuMat> channels;
    cv::cuda::split(bgrGpu, channels, stream);
    const int channelType = channels[0].type();
    if (!m_cudaMedianBlurFilter
        || (m_cudaMedianBlurKernelSize != kernelSize)
        || (m_cudaMedianBlurChannelType != channelType))
    {
        m_cudaMedianBlurFilter = cv::cuda::createMedianFilter(channelType, kernelSize);
        m_cudaMedianBlurKernelSize = kernelSize;
        m_cudaMedianBlurChannelType = channelType;
    }

    for (cv::cuda::GpuMat& channel : channels)
    {
        cv::cuda::GpuMat filteredChannel;
        m_cudaMedianBlurFilter->apply(channel, filteredChannel, stream);
        channel = filteredChannel;
    }
    cv::cuda::merge(channels, bgrGpu, stream);

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::invalidateMedianBlurCudaFilter() const
{
    m_cudaMedianBlurFilter.release();
    m_cudaMedianBlurKernelSize = 0;
    m_cudaMedianBlurChannelType = -1;
}

void CameraImageProcessor::invalidateCudaProcessingCaches() const
{
    m_cudaUnwarpMapX.release();
    m_cudaUnwarpMapY.release();
    m_cudaUnwarpMapSize = cv::Size();
    m_cudaClahe.release();
    m_cudaHistogramStretchLookup.release();
    m_cudaHistogramStretchMode = CameraSettings::HistogramStretchOff;
    m_cudaHistogramStretchBlackPoint = 0.0;
    m_cudaHistogramStretchWhitePoint = 0.0;
    m_cudaHistogramStretchGamma = 0.0;
    m_cudaHistogramStretchAsinhStrength = 0.0;
    m_cudaHistogramStretchLogStrength = 0.0;
    m_cudaGammaLookup.release();
    m_cudaGamma = 0.0;
    m_cudaGaussianBlurFilter.release();
    m_cudaGaussianBlurKernelSize = 0;
    m_cudaGaussianBlurType = -1;
    invalidateMedianBlurCudaFilter();
    m_cudaSharpenBlurFilter.release();
    m_cudaSharpenBlurType = -1;
    m_cudaSobelXFilter.release();
    m_cudaSobelYFilter.release();
    m_cudaSobelInputType = -1;
    m_cudaCannyDetector.release();
}

void CameraImageProcessor::applySharpenCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    cv::cuda::GpuMat blurredGpu;
    if (!m_cudaSharpenBlurFilter || (m_cudaSharpenBlurType != bgrGpu.type()))
    {
        m_cudaSharpenBlurFilter = cv::cuda::createGaussianFilter(
            bgrGpu.type(), bgrGpu.type(), cv::Size(3, 3), 1.0);
        m_cudaSharpenBlurType = bgrGpu.type();
    }
    m_cudaSharpenBlurFilter->apply(bgrGpu, blurredGpu, stream);
    cv::cuda::addWeighted(bgrGpu, 1.0 + m_settings.m_sharpen, blurredGpu, -m_settings.m_sharpen, 0.0, bgrGpu, -1, stream);

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applySobelEdgeCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    cv::cuda::GpuMat grayGpu;
    cv::cuda::cvtColor(bgrGpu, grayGpu, cv::COLOR_BGR2GRAY, 0, stream);

    cv::cuda::GpuMat gradX;
    cv::cuda::GpuMat gradY;
    if (!m_cudaSobelXFilter || !m_cudaSobelYFilter || (m_cudaSobelInputType != grayGpu.type()))
    {
        m_cudaSobelXFilter = cv::cuda::createSobelFilter(grayGpu.type(), CV_16SC1, 1, 0, 3);
        m_cudaSobelYFilter = cv::cuda::createSobelFilter(grayGpu.type(), CV_16SC1, 0, 1, 3);
        m_cudaSobelInputType = grayGpu.type();
    }
    m_cudaSobelXFilter->apply(grayGpu, gradX, stream);
    m_cudaSobelYFilter->apply(grayGpu, gradY, stream);

    cv::cuda::GpuMat absGradX;
    cv::cuda::GpuMat absGradY;
    cv::cuda::GpuMat absGradX8u;
    cv::cuda::GpuMat absGradY8u;
    cv::cuda::abs(gradX, absGradX, stream);
    cv::cuda::abs(gradY, absGradY, stream);
    absGradX.convertTo(absGradX8u, CV_8U, stream);
    absGradY.convertTo(absGradY8u, CV_8U, stream);

    cv::cuda::GpuMat edgesGray;
    cv::cuda::addWeighted(absGradX8u, 0.5, absGradY8u, 0.5, 0.0, edgesGray, -1, stream);

    cv::cuda::GpuMat edgesBgr;
    cv::cuda::cvtColor(edgesGray, edgesBgr, cv::COLOR_GRAY2BGR, 0, stream);
    if (m_settings.m_edgeDisplayMode == CameraSettings::EdgeDisplayEdgesOnly) {
        bgrGpu = edgesBgr;
    } else {
        cv::cuda::addWeighted(bgrGpu, 1.0, edgesBgr, m_settings.m_sobelEdge, 0.0, bgrGpu, -1, stream);
    }

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyCannyEdgeCuda(cv::cuda::GpuMat& bgrGpu, cv::cuda::Stream& stream) const
{
    PROFILER_START();

    cv::cuda::GpuMat grayGpu;
    cv::cuda::cvtColor(bgrGpu, grayGpu, cv::COLOR_BGR2GRAY, 0, stream);

    if (!m_cudaCannyDetector) {
        m_cudaCannyDetector = cv::cuda::createCannyEdgeDetector(50.0, 150.0);
    }
    cv::cuda::GpuMat edgesGray;
    m_cudaCannyDetector->detect(grayGpu, edgesGray, stream);

    cv::cuda::GpuMat edgesBgr;
    cv::cuda::cvtColor(edgesGray, edgesBgr, cv::COLOR_GRAY2BGR, 0, stream);
    if (m_settings.m_edgeDisplayMode == CameraSettings::EdgeDisplayEdgesOnly) {
        bgrGpu = edgesBgr;
    } else {
        cv::cuda::addWeighted(bgrGpu, 1.0, edgesBgr, m_settings.m_cannyEdge, 0.0, bgrGpu, -1, stream);
    }

    PROFILER_STOP(__FUNCTION__);
}
#endif

void CameraImageProcessor::applyWhiteBalance(cv::Mat& bgrMat)
{
    PROFILER_START();
    cv::Vec3d gains(
        m_settings.m_postProcessWhiteBalanceBlueGain,
        m_settings.m_postProcessWhiteBalanceGreenGain,
        m_settings.m_postProcessWhiteBalanceRedGain);

    if (m_settings.m_postProcessWhiteBalanceMode == 1)
    {
        const cv::Scalar means = cv::mean(bgrMat);
        const double blueMean = std::max(1.0, means[0]);
        const double greenMean = std::max(1.0, means[1]);
        const double redMean = std::max(1.0, means[2]);
        const double targetMean = (blueMean + greenMean + redMean) / 3.0;

        cv::Vec3d targetGains(
            qBound(0.25, targetMean / blueMean, 4.0),
            qBound(0.25, targetMean / greenMean, 4.0),
            qBound(0.25, targetMean / redMean, 4.0));

        if (!m_autoWhiteBalanceInitialized)
        {
            m_autoWhiteBalanceGains = targetGains;
            m_autoWhiteBalanceInitialized = true;
        }
        else
        {
            static constexpr double kAutoWhiteBalanceSmoothing = 0.2;
            m_autoWhiteBalanceGains =
                (1.0 - kAutoWhiteBalanceSmoothing) * m_autoWhiteBalanceGains
                + kAutoWhiteBalanceSmoothing * targetGains;
        }

        gains = m_autoWhiteBalanceGains;
    }

    const double highlightProtection = (m_settings.m_postProcessWhiteBalanceMode == 2)
        ? qBound(0.0, m_settings.m_postProcessWhiteBalanceHighlightProtection, 1.0)
        : 0.0;

    if (highlightProtection <= 1e-6)
    {
        std::vector<cv::Mat> channels;
        cv::split(bgrMat, channels);
        channels[0].convertTo(channels[0], -1, gains[0], 0.0);
        channels[1].convertTo(channels[1], -1, gains[1], 0.0);
        channels[2].convertTo(channels[2], -1, gains[2], 0.0);
        cv::merge(channels, bgrMat);
    }
    else
    {
        static constexpr double kHighlightRolloffStart = 0.85 * 255.0;
        static constexpr double kHighlightRolloffRange = 255.0 - kHighlightRolloffStart;

        for (int y = 0; y < bgrMat.rows; ++y)
        {
            cv::Vec3b *row = bgrMat.ptr<cv::Vec3b>(y);

            for (int x = 0; x < bgrMat.cols; ++x)
            {
                const cv::Vec3b src = row[x];
                const double highlight = std::max({
                    static_cast<double>(src[0]),
                    static_cast<double>(src[1]),
                    static_cast<double>(src[2])
                });
                double rolloff = qBound(0.0, (highlight - kHighlightRolloffStart) / kHighlightRolloffRange, 1.0);
                rolloff = rolloff * rolloff * (3.0 - 2.0 * rolloff);
                rolloff *= highlightProtection;

                for (int c = 0; c < 3; ++c)
                {
                    const double effectiveGain = gains[c] + (1.0 - gains[c]) * rolloff;
                    row[x][c] = cv::saturate_cast<uchar>(src[c] * effectiveGain);
                }
            }
        }
    }
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyHistogramStretch(cv::Mat& bgrMat) const
{
    PROFILER_START();

    cv::Mat floatMat;
    bgrMat.convertTo(floatMat, CV_32FC3, 1.0 / 255.0);

    const float blackPoint = static_cast<float>(m_settings.m_histogramStretchBlackPoint);
    const float whitePoint = static_cast<float>(m_settings.m_histogramStretchWhitePoint);
    const float rangeScale = 1.0f / std::max(0.001f, whitePoint - blackPoint);

    const float gammaValue = static_cast<float>(m_settings.m_histogramStretchGamma);
    const float asinhStrength = static_cast<float>(m_settings.m_histogramStretchAsinhStrength);
    const float logStrength = static_cast<float>(m_settings.m_histogramStretchLogStrength);
    const float asinhNorm = std::asinh(asinhStrength);
    const float logNorm = std::log1p(logStrength);

    if (m_settings.m_histogramStretch == CameraSettings::HistogramStretchCLAHE)
    {
        cv::Mat labMat;
        cv::cvtColor(bgrMat, labMat, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> labChannels;
        cv::split(labMat, labChannels);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(labChannels[0], labChannels[0]);
        cv::merge(labChannels, labMat);
        cv::cvtColor(labMat, bgrMat, cv::COLOR_Lab2BGR);
        PROFILER_STOP(__FUNCTION__);
        return;
    }

    for (int row = 0; row < floatMat.rows; ++row)
    {
        cv::Vec3f* pixelRow = floatMat.ptr<cv::Vec3f>(row);
        for (int col = 0; col < floatMat.cols; ++col)
        {
            cv::Vec3f& pixel = pixelRow[col];
            for (int channel = 0; channel < 3; ++channel)
            {
                float value = (pixel[channel] - blackPoint) * rangeScale;
                value = std::clamp(value, 0.0f, 1.0f);

                switch (m_settings.m_histogramStretch)
                {
                case CameraSettings::HistogramStretchLinear:
                    break;
                case CameraSettings::HistogramStretchGamma:
                    value = std::pow(value, gammaValue);
                    break;
                case CameraSettings::HistogramStretchAsinh:
                    value = (asinhNorm > 0.0f) ? (std::asinh(asinhStrength * value) / asinhNorm) : value;
                    break;
                case CameraSettings::HistogramStretchLog:
                    value = (logNorm > 0.0f) ? (std::log1p(logStrength * value) / logNorm) : value;
                    break;
                case CameraSettings::HistogramStretchCLAHE:
                case CameraSettings::HistogramStretchOff:
                default:
                    break;
                }

                pixel[channel] = std::clamp(value, 0.0f, 1.0f);
            }
        }
    }

    floatMat.convertTo(bgrMat, CV_8UC3, 255.0);
    PROFILER_STOP(__FUNCTION__);
}

double CameraImageProcessor::degreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

double CameraImageProcessor::sourceRadiusForTheta(double thetaRadians, CameraSettings::LensProjection projection, double focalPixels)
{
    switch (projection)
    {
    case CameraSettings::LensProjectionEquidistant:
        return focalPixels * thetaRadians;
    case CameraSettings::LensProjectionEquisolid:
        return 2.0 * focalPixels * std::sin(thetaRadians * 0.5);
    case CameraSettings::LensProjectionRectilinear:
    default:
        return focalPixels * std::tan(thetaRadians);
    }
}

void CameraImageProcessor::invalidateUnwarpMaps()
{
    m_unwarpMapX.release();
    m_unwarpMapY.release();
    m_unwarpMapSize = cv::Size();
    m_unwarpSourceProjection = CameraSettings::LensProjectionRectilinear;
    m_unwarpSourceFov = 0.0;
#ifdef CAMERA_OPENCV_CUDA_IMAGE_PROCESSING
    m_cudaUnwarpMapX.release();
    m_cudaUnwarpMapY.release();
    m_cudaUnwarpMapSize = cv::Size();
#endif
}

void CameraImageProcessor::ensureUnwarpMaps(const cv::Size& frameSize)
{
    const double sourceFovDegrees = std::clamp(static_cast<double>(m_settings.m_fov), 1.0, 359.0);
    const CameraSettings::LensProjection sourceProjection = m_settings.m_lensProjection;

    if (!m_unwarpMapX.empty()
        && !m_unwarpMapY.empty()
        && (m_unwarpMapSize == frameSize)
        && (m_unwarpSourceProjection == sourceProjection)
        && (std::abs(m_unwarpSourceFov - sourceFovDegrees) < 1e-6))
    {
        return;
    }

    invalidateUnwarpMaps();

    if ((frameSize.width <= 1) || (frameSize.height <= 1)) {
        return;
    }

    const double sourceHalfFovRadians = degreesToRadians(sourceFovDegrees * 0.5);
    if (!std::isfinite(sourceHalfFovRadians) || (sourceHalfFovRadians <= 1e-6)) {
        return;
    }

    double sourceFocalPixels = 0.0;
    switch (sourceProjection)
    {
    case CameraSettings::LensProjectionEquidistant:
        sourceFocalPixels = (frameSize.width * 0.5) / sourceHalfFovRadians;
        break;
    case CameraSettings::LensProjectionEquisolid:
    {
        const double denom = 2.0 * std::sin(sourceHalfFovRadians * 0.5);
        if (std::abs(denom) <= 1e-9) {
            return;
        }
        sourceFocalPixels = (frameSize.width * 0.5) / denom;
        break;
    }
    case CameraSettings::LensProjectionRectilinear:
    default:
    {
        const double denom = std::tan(sourceHalfFovRadians);
        if (std::abs(denom) <= 1e-9) {
            return;
        }
        sourceFocalPixels = (frameSize.width * 0.5) / denom;
        break;
    }
    }

    const double outputFovDegrees = std::min(sourceFovDegrees, 170.0);
    const double outputHalfFovRadians = degreesToRadians(outputFovDegrees * 0.5);
    const double outputFocalPixels = (frameSize.width * 0.5) / std::tan(outputHalfFovRadians);
    if (!std::isfinite(outputFocalPixels) || (outputFocalPixels <= 0.0)) {
        return;
    }

    m_unwarpMapX.create(frameSize, CV_32FC1);
    m_unwarpMapY.create(frameSize, CV_32FC1);

    const double centerX = 0.5 * static_cast<double>(frameSize.width - 1);
    const double centerY = 0.5 * static_cast<double>(frameSize.height - 1);

    for (int y = 0; y < frameSize.height; ++y)
    {
        float *mapXRow = m_unwarpMapX.ptr<float>(y);
        float *mapYRow = m_unwarpMapY.ptr<float>(y);
        const double yNorm = (centerY - static_cast<double>(y)) / outputFocalPixels;

        for (int x = 0; x < frameSize.width; ++x)
        {
            const double xNorm = (static_cast<double>(x) - centerX) / outputFocalPixels;
            const double radiusNorm = std::sqrt((xNorm * xNorm) + (yNorm * yNorm));
            const double theta = std::atan(radiusNorm);

            if (!std::isfinite(theta) || (theta > sourceHalfFovRadians)) {
                mapXRow[x] = -1.0f;
                mapYRow[x] = -1.0f;
                continue;
            }

            const double radiusPixels = sourceRadiusForTheta(theta, sourceProjection, sourceFocalPixels);
            if (!std::isfinite(radiusPixels)) {
                mapXRow[x] = -1.0f;
                mapYRow[x] = -1.0f;
                continue;
            }

            if (radiusNorm <= 1e-9)
            {
                mapXRow[x] = static_cast<float>(centerX);
                mapYRow[x] = static_cast<float>(centerY);
                continue;
            }

            const double scale = radiusPixels / radiusNorm;
            mapXRow[x] = static_cast<float>(centerX + (xNorm * scale));
            mapYRow[x] = static_cast<float>(centerY - (yNorm * scale));
        }
    }

    m_unwarpMapSize = frameSize;
    m_unwarpSourceProjection = sourceProjection;
    m_unwarpSourceFov = sourceFovDegrees;
}

void CameraImageProcessor::applyLensUnwarp(cv::Mat& bgrMat)
{
    PROFILER_START();

    ensureUnwarpMaps(bgrMat.size());
    if (m_unwarpMapX.empty() || m_unwarpMapY.empty()) {
        PROFILER_STOP(__FUNCTION__);
        return;
    }

    cv::Mat unwarped;
    cv::remap(bgrMat, unwarped, m_unwarpMapX, m_unwarpMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    bgrMat = std::move(unwarped);

    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyGreyscale(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat grayMat;
    cv::cvtColor(bgrMat, grayMat, cv::COLOR_BGR2GRAY);
    cv::cvtColor(grayMat, bgrMat, cv::COLOR_GRAY2BGR);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applySaturation(cv::Mat& bgrMat)
{
    PROFILER_START();
    cv::Mat hsvMat;
    cv::cvtColor(bgrMat, hsvMat, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvMat, hsvChannels);
    hsvChannels[1].convertTo(hsvChannels[1], -1, m_settings.m_saturation, 0.0);
    cv::merge(hsvChannels, hsvMat);
    cv::cvtColor(hsvMat, bgrMat, cv::COLOR_HSV2BGR);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyGamma(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat lut(1, 256, CV_8U);
    uchar* lutData = lut.ptr<uchar>();
    for (int i = 0; i < 256; ++i) {
        lutData[i] = static_cast<uchar>(qBound(0, static_cast<int>(std::pow(i / 255.0, m_settings.m_gamma) * 255.0 + 0.5), 255));
    }
    cv::LUT(bgrMat, lut, bgrMat);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyGaussianBlur(cv::Mat& bgrMat) const
{
    PROFILER_START();
    const int kernelSize = 2 * m_settings.m_gaussianBlur + 1;
    cv::GaussianBlur(bgrMat, bgrMat, cv::Size(kernelSize, kernelSize), 0);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyMedianBlur(cv::Mat& bgrMat) const
{
    PROFILER_START();
    const int kernelSize = 2 * m_settings.m_medianBlur + 1;
    cv::medianBlur(bgrMat, bgrMat, kernelSize);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applySharpen(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat blurred;
    cv::GaussianBlur(bgrMat, blurred, cv::Size(0, 0), 1.0);
    cv::addWeighted(bgrMat, 1.0 + m_settings.m_sharpen, blurred, -m_settings.m_sharpen, 0.0, bgrMat);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applySobelEdge(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat grayMat;
    cv::cvtColor(bgrMat, grayMat, cv::COLOR_BGR2GRAY);

    cv::Mat gradX;
    cv::Mat gradY;
    cv::Sobel(grayMat, gradX, CV_16S, 1, 0, 3);
    cv::Sobel(grayMat, gradY, CV_16S, 0, 1, 3);

    cv::Mat absGradX;
    cv::Mat absGradY;
    cv::convertScaleAbs(gradX, absGradX);
    cv::convertScaleAbs(gradY, absGradY);

    cv::Mat edgesGray;
    cv::addWeighted(absGradX, 0.5, absGradY, 0.5, 0.0, edgesGray);

    cv::Mat edgesBgr;
    cv::cvtColor(edgesGray, edgesBgr, cv::COLOR_GRAY2BGR);
    if (m_settings.m_edgeDisplayMode == CameraSettings::EdgeDisplayEdgesOnly) {
        bgrMat = std::move(edgesBgr);
    } else {
        cv::addWeighted(bgrMat, 1.0, edgesBgr, m_settings.m_sobelEdge, 0.0, bgrMat);
    }
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyCannyEdge(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat grayMat;
    cv::cvtColor(bgrMat, grayMat, cv::COLOR_BGR2GRAY);

    cv::Mat edgesGray;
    cv::Canny(grayMat, edgesGray, 50.0, 150.0);

    cv::Mat edgesBgr;
    cv::cvtColor(edgesGray, edgesBgr, cv::COLOR_GRAY2BGR);
    if (m_settings.m_edgeDisplayMode == CameraSettings::EdgeDisplayEdgesOnly) {
        bgrMat = std::move(edgesBgr);
    } else {
        cv::addWeighted(bgrMat, 1.0, edgesBgr, m_settings.m_cannyEdge, 0.0, bgrMat);
    }
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyFlip(cv::Mat& bgrMat) const
{
    PROFILER_START();
    const int flipCode = m_settings.m_flipX && m_settings.m_flipY ? -1 : (m_settings.m_flipX ? 1 : 0);
    cv::flip(bgrMat, bgrMat, flipCode);
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyBrightnessContrast(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::Mat adjusted;
    cv::convertScaleAbs(bgrMat, adjusted, m_settings.m_contrast, m_settings.m_brightness);
    bgrMat = adjusted;
    PROFILER_STOP(__FUNCTION__);
}

void CameraImageProcessor::applyInvertColors(cv::Mat& bgrMat) const
{
    PROFILER_START();
    cv::bitwise_not(bgrMat, bgrMat);
    PROFILER_STOP(__FUNCTION__);
}

const QImage& CameraImageProcessor::ensureRgb888(const QImage& image, QImage& convertedImage)
{
    if (image.format() == QImage::Format_RGB888) {
        return image;
    }

    convertedImage = image.convertToFormat(QImage::Format_RGB888);
    return convertedImage;
}

cv::Mat CameraImageProcessor::wrapRgb888Image(const QImage& image)
{
    return cv::Mat(image.height(), image.width(), CV_8UC3,
                   const_cast<uchar*>(image.constBits()),
                   static_cast<size_t>(image.bytesPerLine()));
}

QImage CameraImageProcessor::convertBgrToRgbImage(const cv::Mat& bgrMat)
{
    PROFILER_START();
    QImage result(bgrMat.cols, bgrMat.rows, QImage::Format_RGB888);
    cv::Mat rgbMat(result.height(), result.width(), CV_8UC3,
                   result.bits(),
                   static_cast<size_t>(result.bytesPerLine()));
    cv::cvtColor(bgrMat, rgbMat, cv::COLOR_BGR2RGB);
    return result;
}
