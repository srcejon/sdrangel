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
#include <QElapsedTimer>
#include <QPainter>
#include <QTimer>

#include <opencv2/photo.hpp>
#ifdef CAMERA_OPENCV_CUDA_STACKING
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/profiler.h"
#include "camera.h"
#include "camerahdrfusion.h"
#include "cameraframestacker.h"
#include "cameraimageutils.h"
#include "cameraimageprocessor.h"

MESSAGE_CLASS_DEFINITION(CameraFrameStacker::MsgDeleteStackFrame, Message)

CameraFrameStacker::CameraFrameStacker() :
    m_nextStage(nullptr),
    m_captureActive(false),
#ifdef CAMERA_OPENCV_CUDA_STACKING
    m_cudaStackAccumulatorInputType(-1),
    m_cudaQualityLaplacianFilterType(-1),
#endif
    m_processingFrame(false),
    m_droppedFrameCount(0),
    m_rejectedFrameCount(0)
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
    m_stackFrameQualityHistory.clear();
    m_stackFrameThumbnails.clear();
    m_hdrFrameSamples.clear();
    m_lastFrameTemplate.clear();
    m_lastStackedImage = QImage();
    m_stackAccumulator.release();
#ifdef CAMERA_OPENCV_CUDA_STACKING
    m_cudaStackAccumulator.release();
    m_cudaStackAccumulatorInputType = -1;
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
    if (m_settings.isHdrStackingEnabled()) {
        return qBound(8, stackFrameCount * 8, 512);
    }
    return qBound(2, stackFrameCount * 2, 512);
}

int CameraFrameStacker::dropOldestPendingFramesForOverflow()
{
    if (m_pendingFrames.empty()) {
        return 0;
    }

    int dropped = 0;

    if (m_settings.isHdrStackingEnabled())
    {
        do
        {
            m_pendingFrames.pop_front();
            ++dropped;
        }
        while (!m_pendingFrames.empty()
            && m_pendingFrames.front()
            && (m_pendingFrames.front()->m_hdrExposureIndex != 0));
    }
    else
    {
        m_pendingFrames.pop_front();
        dropped = 1;
    }

    return dropped;
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
        if (!m_stackFrameQualityHistory.empty()) {
            m_stackFrameQualityHistory.pop_front();
        }
        if (!m_stackFrameThumbnails.empty()) {
            m_stackFrameThumbnails.pop_front();
        }
    }

    if (m_stackFrameHistory.empty())
    {
        m_stackAccumulator.release();
#ifdef CAMERA_OPENCV_CUDA_STACKING
        m_cudaStackAccumulator.release();
        m_cudaStackAccumulatorInputType = -1;
#endif
    }
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

bool CameraFrameStacker::rebuildCudaAverageAccumulator()
{
    if (m_stackFrameHistory.empty()) {
        m_cudaStackAccumulator.release();
        return false;
    }

    const cv::Size accumulatorSize = m_stackFrameHistory.front().size();
    const int accumulatorInputType = m_stackFrameHistory.front().type();
    m_cudaStackAccumulator = cv::cuda::GpuMat(accumulatorSize, CV_32FC3, cv::Scalar::all(0.0));
    m_cudaStackAccumulatorInputType = accumulatorInputType;

    cv::cuda::GpuMat frameGpu;
    cv::cuda::GpuMat floatGpu;
    for (const cv::Mat& historyFrame : m_stackFrameHistory)
    {
        frameGpu.upload(historyFrame, m_cudaStackingStream);
        frameGpu.convertTo(floatGpu, CV_32FC3, m_cudaStackingStream);
        cv::cuda::add(m_cudaStackAccumulator, floatGpu, m_cudaStackAccumulator, cv::noArray(), -1, m_cudaStackingStream);
    }

    return true;
}

bool CameraFrameStacker::applyAverageStackingCuda(const cv::Mat& frameMat, const cv::cuda::GpuMat* sourceFrameGpu, cv::cuda::GpuMat& outputRgbGpu)
{
    try
    {
        bool accumulatorIncludesCurrentFrame = false;
        if (m_cudaStackAccumulator.empty()
            || (m_cudaStackAccumulator.size() != frameMat.size())
            || (m_cudaStackAccumulatorInputType != frameMat.type()))
        {
            if (!rebuildCudaAverageAccumulator()) {
                return false;
            }
            accumulatorIncludesCurrentFrame = true;
        }

        if (!accumulatorIncludesCurrentFrame)
        {
            cv::cuda::GpuMat frameGpu;
            cv::cuda::GpuMat floatGpu;
            if (sourceFrameGpu && !sourceFrameGpu->empty()) {
                frameGpu = *sourceFrameGpu;
            } else {
                frameGpu.upload(frameMat, m_cudaStackingStream);
            }
            frameGpu.convertTo(floatGpu, CV_32FC3, m_cudaStackingStream);
            cv::cuda::add(m_cudaStackAccumulator, floatGpu, m_cudaStackAccumulator, cv::noArray(), -1, m_cudaStackingStream);
        }

        cv::cuda::GpuMat averagedGpu;
        cv::cuda::GpuMat averagedOutputGpu;
        const int outputType = (frameMat.depth() == CV_16U) ? CV_16UC3 : CV_8UC3;
        m_cudaStackAccumulator.convertTo(averagedGpu, CV_32FC3, 1.0 / static_cast<double>(m_stackFrameHistory.size()), 0.0, m_cudaStackingStream);
        averagedGpu.convertTo(averagedOutputGpu, outputType, 1.0, 0.0, m_cudaStackingStream);
        averagedOutputGpu.copyTo(outputRgbGpu, m_cudaStackingStream);
        m_cudaStackingStream.waitForCompletion();
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFrameStacker: CUDA average stacking failed; falling back to CPU:" << error.what();
        m_cudaStackAccumulator.release();
        m_cudaStackAccumulatorInputType = -1;
    }

    return false;
}

bool CameraFrameStacker::applyMertensFusionCuda(const std::vector<const HdrFrameSample *>& sortedSamples, cv::cuda::GpuMat& tonemappedRgbGpu)
{
    tonemappedRgbGpu.release();
    if (!canUseCudaStacking() || sortedSamples.empty()) {
        return false;
    }

    std::vector<cv::Mat> rgbFrames;
    rgbFrames.reserve(sortedSamples.size());
#if defined(CAMERA_OPENCV_CUDA_STACKING)
    std::vector<cv::cuda::GpuMat> rgbFramesGpu;
    rgbFramesGpu.reserve(sortedSamples.size());
    bool allFramesHaveGpu = true;
#endif
    for (const HdrFrameSample *sample : sortedSamples)
    {
        if (!sample) {
            return false;
        }
        rgbFrames.push_back(sample->m_frameMat);
#if defined(CAMERA_OPENCV_CUDA_STACKING)
        if (sample->m_frameGpu.empty()) {
            allFramesHaveGpu = false;
        } else {
            rgbFramesGpu.push_back(sample->m_frameGpu);
        }
#endif
    }

    QString errorMessage;
    const bool success =
#if defined(CAMERA_OPENCV_CUDA_STACKING)
        allFramesHaveGpu
            ? CameraHdrFusion::mergeMertensCudaRgb(rgbFramesGpu, tonemappedRgbGpu, m_cudaStackingStream, m_cudaHdrLaplacianFilter, &errorMessage)
            :
#endif
            CameraHdrFusion::mergeMertensCudaRgb(rgbFrames, tonemappedRgbGpu, m_cudaStackingStream, m_cudaHdrLaplacianFilter, &errorMessage);
    if (!success)
    {
        if (!errorMessage.isEmpty()) {
            qWarning() << "CameraFrameStacker: CUDA Mertens exposure fusion failed; falling back to CPU:" << errorMessage;
        }
        return false;
    }

    return true;
}

bool CameraFrameStacker::applyDebevecFusionCuda(
    const std::vector<const HdrFrameSample *>& sortedSamples,
    const std::vector<float>& exposureTimesSeconds,
    cv::cuda::GpuMat& tonemappedRgbGpu)
{
    tonemappedRgbGpu.release();
    if (!canUseCudaStacking() || sortedSamples.empty()) {
        return false;
    }

    std::vector<cv::Mat> rgbFrames;
    rgbFrames.reserve(sortedSamples.size());
    std::vector<cv::cuda::GpuMat> rgbFramesGpu;
    rgbFramesGpu.reserve(sortedSamples.size());
    bool allFramesHaveGpu = true;
    for (const HdrFrameSample *sample : sortedSamples)
    {
        if (!sample) {
            return false;
        }
        rgbFrames.push_back(sample->m_frameMat);
        if (sample->m_frameGpu.empty()) {
            allFramesHaveGpu = false;
        } else {
            rgbFramesGpu.push_back(sample->m_frameGpu);
        }
    }

    QString errorMessage;
    const bool success = allFramesHaveGpu
        ? CameraHdrFusion::mergeDebevecCudaRgb(rgbFramesGpu, exposureTimesSeconds, tonemappedRgbGpu, m_cudaStackingStream, &errorMessage)
        : CameraHdrFusion::mergeDebevecCudaRgb(rgbFrames, exposureTimesSeconds, tonemappedRgbGpu, m_cudaStackingStream, &errorMessage);
    if (!success)
    {
        if (!errorMessage.isEmpty()) {
            qWarning() << "CameraFrameStacker: CUDA Debevec HDR merge failed; falling back to CPU:" << errorMessage;
        }
        return false;
    }

    return true;
}

void CameraFrameStacker::setCudaBgrOutputFromRgbGpu(CameraPipelineFrame& inputFrame, const cv::cuda::GpuMat& rgbGpu, int outputDepth)
{
    cv::cuda::GpuMat rgbOutputGpu;
    if (rgbGpu.depth() == outputDepth) {
        rgbOutputGpu = rgbGpu;
    } else {
        const double scale = ((rgbGpu.depth() == CV_32F) && (outputDepth == CV_8U)) ? 255.0 : 1.0;
        rgbGpu.convertTo(rgbOutputGpu, CV_MAKETYPE(outputDepth, rgbGpu.channels()), scale, 0.0, m_cudaStackingStream);
    }

    cv::cuda::cvtColor(rgbOutputGpu, inputFrame.m_cudaBgrImage, cv::COLOR_RGB2BGR, 0, m_cudaStackingStream);
    inputFrame.m_cudaGrayImage.release();
    inputFrame.clearCpuImage();
}

bool CameraFrameStacker::setCudaBgrOutputFromRgbMat(CameraPipelineFrame& inputFrame, const cv::Mat& rgbMat)
{
    if (rgbMat.empty() || (rgbMat.channels() != 3)) {
        return false;
    }

    cv::cuda::GpuMat rgbGpu;
    rgbGpu.upload(rgbMat, m_cudaStackingStream);
    setCudaBgrOutputFromRgbGpu(inputFrame, rgbGpu, rgbMat.depth());
    m_cudaStackingStream.waitForCompletion();
    return inputFrame.hasCudaBgrImage();
}
#endif

// Lifetime contract: the returned Mat may alias the source QImage's pixel buffer when no
// format conversion was needed. Callers must keep `input` alive for at least as long as
// the returned Mat is read, or explicitly clone the result if they need it to outlive the
// input. Where convertToFormat() is needed (a non-RGB888 colour input) the helper still
// has to clone, because the converted QImage is a function-local temporary that would
// otherwise leave a dangling pointer behind.
//
// All current storage sites in this file (m_stackFrameHistory.push_back at ~1123 and
// m_hdrFrameSamples.push_back at ~925) clone explicitly, so the new contract is honoured.
// Avoiding the clone here saves ~32 MB/frame for 4K Grayscale16.
cv::Mat CameraFrameStacker::imageToWorkingMat(const QImage& input, bool& highBitDepthInput)
{
    return CameraImageUtils::imageToWorkingMat(input, &highBitDepthInput);
}

QImage CameraFrameStacker::workingMatToImage(const cv::Mat& frameMat)
{
    return CameraImageUtils::workingMatToImage(frameMat);
}

QImage CameraFrameStacker::makeHistoryThumbnail(const cv::Mat& frameMat)
{
    const QImage image = workingMatToImage(frameMat);
    if (image.isNull()) {
        return QImage();
    }

    constexpr int tileMaxWidth = 320;
    if (image.width() <= tileMaxWidth) {
        return image.copy();
    }

    return image.scaledToWidth(tileMaxWidth, Qt::SmoothTransformation);
}

QImage CameraFrameStacker::makeHistoryTilesImage(const std::deque<QImage>& thumbnails, const std::deque<StackFrameQuality>& qualities)
{
    if (thumbnails.empty() || thumbnails.front().isNull()) {
        return QImage();
    }

    const int frameCount = static_cast<int>(thumbnails.size());
    const int columns = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(frameCount)))));
    const int rows = (frameCount + columns - 1) / columns;
    const int tileWidth = thumbnails.front().width();
    const int tileHeight = thumbnails.front().height();
    const int labelHeight = 18;

    QImage mosaic(columns * tileWidth, rows * (tileHeight + labelHeight), QImage::Format_RGB888);
    mosaic.fill(Qt::black);

    QPainter painter(&mosaic);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.setPen(Qt::white);

    const double medianSharpness = medianQualityValue(qualities, &StackFrameQuality::m_laplacianVariance);

    for (int i = 0; i < frameCount; ++i)
    {
        const QImage& frameImage = thumbnails[static_cast<size_t>(i)];
        if (frameImage.isNull()) {
            continue;
        }

        const int col = i % columns;
        const int row = i / columns;
        const QRect imageRect(col * tileWidth, row * (tileHeight + labelHeight), tileWidth, tileHeight);
        painter.drawImage(imageRect, frameImage);

        QColor borderColor(80, 220, 120);
        if (i < static_cast<int>(qualities.size()))
        {
            const StackFrameQuality& quality = qualities[static_cast<size_t>(i)];
            if (!quality.m_valid
                || (quality.m_blackFraction > 0.98)
                || (quality.m_saturatedFraction > 0.80))
            {
                borderColor = QColor(255, 80, 80);
            }
            else if ((quality.m_blackFraction > 0.50)
                || (quality.m_saturatedFraction > 0.35)
                || ((medianSharpness > 10.0) && (quality.m_laplacianVariance < medianSharpness * 0.50)))
            {
                borderColor = QColor(255, 190, 70);
            }
        }
        painter.setPen(QPen(borderColor, 3));
        painter.drawRect(imageRect.adjusted(1, 1, -2, -2));
        painter.setPen(Qt::white);
        painter.drawText(QRect(imageRect.left(), imageRect.bottom() + 1, tileWidth, labelHeight),
            Qt::AlignCenter, QString::number(i + 1));
    }

    return mosaic;
}

#ifdef CAMERA_OPENCV_CUDA_STACKING
CameraFrameStacker::StackFrameQuality CameraFrameStacker::computeStackFrameQualityCuda(const CameraPipelineFrame& inputFrame)
{
    StackFrameQuality quality;

#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
    if (!canUseCudaStacking()) {
        return quality;
    }

    try
    {
        cv::cuda::GpuMat grayGpu;
        if (inputFrame.hasCudaGrayImage())
        {
            grayGpu = inputFrame.m_cudaGrayImage;
        }
        else if (inputFrame.hasCudaBgrImage())
        {
            cv::cuda::cvtColor(inputFrame.m_cudaBgrImage, grayGpu, cv::COLOR_BGR2GRAY, 0, m_cudaStackingStream);
        }
        else
        {
            return quality;
        }

        if (grayGpu.empty()) {
            return quality;
        }

        cv::cuda::GpuMat gray8uGpu;
        if (grayGpu.type() == CV_8UC1) {
            gray8uGpu = grayGpu;
        } else if (grayGpu.depth() == CV_16U) {
            grayGpu.convertTo(gray8uGpu, CV_8U, 255.0 / 65535.0, 0.0, m_cudaStackingStream);
        } else {
            grayGpu.convertTo(gray8uGpu, CV_8U, 1.0, 0.0, m_cudaStackingStream);
        }
        cv::cuda::GpuMat meanStdDevGpu;
        cv::cuda::meanStdDev(gray8uGpu, meanStdDevGpu, m_cudaStackingStream);

        cv::cuda::GpuMat blackMask;
        cv::cuda::GpuMat saturatedMask;
        cv::cuda::inRange(gray8uGpu, cv::Scalar(0), cv::Scalar(2), blackMask, m_cudaStackingStream);
        cv::cuda::inRange(gray8uGpu, cv::Scalar(253), cv::Scalar(255), saturatedMask, m_cudaStackingStream);
        cv::cuda::GpuMat blackCountGpu;
        cv::cuda::GpuMat saturatedCountGpu;
        cv::cuda::countNonZero(blackMask, blackCountGpu, m_cudaStackingStream);
        cv::cuda::countNonZero(saturatedMask, saturatedCountGpu, m_cudaStackingStream);
        const double pixelCount = static_cast<double>(std::max(1, gray8uGpu.rows * gray8uGpu.cols));

        if (!m_cudaQualityLaplacianFilter
            || (m_cudaQualityLaplacianFilterType != gray8uGpu.type()))
        {
            m_cudaQualityLaplacianFilter = cv::cuda::createLaplacianFilter(gray8uGpu.type(), CV_32F, 1);
            m_cudaQualityLaplacianFilterType = gray8uGpu.type();
        }

        cv::cuda::GpuMat laplacianGpu;
        m_cudaQualityLaplacianFilter->apply(gray8uGpu, laplacianGpu, m_cudaStackingStream);
        cv::cuda::GpuMat laplacianMeanStdDevGpu;
        cv::cuda::meanStdDev(laplacianGpu, laplacianMeanStdDevGpu, m_cudaStackingStream);

        cv::Mat meanStdDev;
        cv::Mat laplacianMeanStdDev;
        cv::Mat blackCount;
        cv::Mat saturatedCount;
        meanStdDevGpu.download(meanStdDev, m_cudaStackingStream);
        laplacianMeanStdDevGpu.download(laplacianMeanStdDev, m_cudaStackingStream);
        blackCountGpu.download(blackCount, m_cudaStackingStream);
        saturatedCountGpu.download(saturatedCount, m_cudaStackingStream);
        m_cudaStackingStream.waitForCompletion();

        const double *meanStdDevValues = meanStdDev.ptr<double>();
        const double *laplacianMeanStdDevValues = laplacianMeanStdDev.ptr<double>();
        quality.m_mean = meanStdDevValues[0];
        quality.m_stdDev = meanStdDevValues[1];
        quality.m_blackFraction = static_cast<double>(blackCount.at<int>(0, 0)) / pixelCount;
        quality.m_saturatedFraction = static_cast<double>(saturatedCount.at<int>(0, 0)) / pixelCount;
        quality.m_laplacianVariance = laplacianMeanStdDevValues[1] * laplacianMeanStdDevValues[1];
        quality.m_valid = std::isfinite(quality.m_mean)
            && std::isfinite(quality.m_stdDev)
            && std::isfinite(quality.m_laplacianVariance)
            && std::isfinite(quality.m_blackFraction)
            && std::isfinite(quality.m_saturatedFraction);
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFrameStacker: CUDA stack quality metrics failed; falling back to CPU:" << error.what();
        quality = StackFrameQuality();
    }
#else
    (void) inputFrame;
#endif

    return quality;
}
#endif

CameraFrameStacker::StackFrameQuality CameraFrameStacker::computeStackFrameQualityForFrame(const CameraPipelineFrame& inputFrame, const cv::Mat& frameMat)
{
#ifdef CAMERA_OPENCV_CUDA_STACKING
    StackFrameQuality cudaQuality = computeStackFrameQualityCuda(inputFrame);
    if (cudaQuality.m_valid) {
        return cudaQuality;
    }
#else
    (void) inputFrame;
#endif

    return computeStackFrameQuality(frameMat);
}

void CameraFrameStacker::ensureStackFrameQualityHistory()
{
    while (m_stackFrameQualityHistory.size() < m_stackFrameHistory.size())
    {
        const size_t index = m_stackFrameQualityHistory.size();
        m_stackFrameQualityHistory.push_back(computeStackFrameQuality(m_stackFrameHistory[index]));
    }

    while (m_stackFrameQualityHistory.size() > m_stackFrameHistory.size()) {
        m_stackFrameQualityHistory.pop_back();
    }
}

std::vector<size_t> CameraFrameStacker::selectedSharpFrameIndices() const
{
    std::vector<size_t> indices;
    indices.reserve(m_stackFrameHistory.size());
    for (size_t i = 0; i < m_stackFrameHistory.size(); ++i) {
        indices.push_back(i);
    }

    std::sort(indices.begin(), indices.end(),
        [this](size_t lhs, size_t rhs)
        {
            const bool lhsValid = (lhs < m_stackFrameQualityHistory.size()) && m_stackFrameQualityHistory[lhs].m_valid;
            const bool rhsValid = (rhs < m_stackFrameQualityHistory.size()) && m_stackFrameQualityHistory[rhs].m_valid;
            if (lhsValid != rhsValid) {
                return lhsValid;
            }

            const double lhsSharpness = lhsValid ? m_stackFrameQualityHistory[lhs].m_laplacianVariance : 0.0;
            const double rhsSharpness = rhsValid ? m_stackFrameQualityHistory[rhs].m_laplacianVariance : 0.0;
            if (lhsSharpness != rhsSharpness) {
                return lhsSharpness > rhsSharpness;
            }

            return lhs > rhs;
        });

    const size_t keepCount = std::max<size_t>(1, (m_stackFrameHistory.size() + 1) / 2);
    if (indices.size() > keepCount) {
        indices.resize(keepCount);
    }

    return indices;
}

CameraFrameStacker::StackFrameQuality CameraFrameStacker::computeStackFrameQuality(const cv::Mat& frameMat)
{
    StackFrameQuality quality;
    if (frameMat.empty()) {
        return quality;
    }

    cv::Mat gray;
    if (frameMat.channels() == 1) {
        gray = frameMat;
    } else {
        cv::cvtColor(frameMat, gray, cv::COLOR_RGB2GRAY);
    }

    cv::Mat gray8u;
    if (gray.depth() == CV_8U) {
        gray8u = gray;
    } else if (gray.depth() == CV_16U) {
        gray.convertTo(gray8u, CV_8U, 255.0 / 65535.0);
    } else {
        gray.convertTo(gray8u, CV_8U);
    }

    cv::Scalar mean;
    cv::Scalar stdDev;
    cv::meanStdDev(gray8u, mean, stdDev);
    quality.m_mean = mean[0];
    quality.m_stdDev = stdDev[0];

    cv::Mat blackMask;
    cv::Mat saturatedMask;
    cv::inRange(gray8u, cv::Scalar(0), cv::Scalar(2), blackMask);
    cv::inRange(gray8u, cv::Scalar(253), cv::Scalar(255), saturatedMask);
    const double pixelCount = static_cast<double>(std::max(1, gray8u.rows * gray8u.cols));
    quality.m_blackFraction = static_cast<double>(cv::countNonZero(blackMask)) / pixelCount;
    quality.m_saturatedFraction = static_cast<double>(cv::countNonZero(saturatedMask)) / pixelCount;

    cv::Mat laplacian;
    cv::Laplacian(gray8u, laplacian, CV_64F);
    cv::meanStdDev(laplacian, mean, stdDev);
    quality.m_laplacianVariance = stdDev[0] * stdDev[0];
    quality.m_valid = std::isfinite(quality.m_mean)
        && std::isfinite(quality.m_stdDev)
        && std::isfinite(quality.m_laplacianVariance)
        && std::isfinite(quality.m_blackFraction)
        && std::isfinite(quality.m_saturatedFraction);
    return quality;
}

double CameraFrameStacker::medianQualityValue(const std::deque<StackFrameQuality>& qualities, double StackFrameQuality::*member)
{
    std::vector<double> values;
    values.reserve(qualities.size());
    for (const StackFrameQuality& quality : qualities)
    {
        if (quality.m_valid) {
            values.push_back(quality.*member);
        }
    }

    if (values.empty()) {
        return 0.0;
    }

    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

bool CameraFrameStacker::handleMessage(const Message& cmd)
{
    if (Camera::MsgConfigureCamera::match(cmd))
    {
        const Camera::MsgConfigureCamera& cfg = (const Camera::MsgConfigureCamera&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (Camera::MsgProcessFrame::match(cmd))
    {
        const Camera::MsgProcessFrame& frameMsg = (const Camera::MsgProcessFrame&) cmd;
        submitFrame(frameMsg.getFrame());
        return true;
    }
    else if (Camera::MsgCaptureActive::match(cmd))
    {
        const Camera::MsgCaptureActive& activeMsg = (const Camera::MsgCaptureActive&) cmd;
        Camera::discardQueuedProcessFrames(m_inputMessageQueue);
        m_captureActive = activeMsg.isActive();
        m_captureEpoch = activeMsg.getCaptureEpoch();
        if (m_captureActive) {
            resetFrameHistoryState();
        }
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrames.clear();
        m_droppedFrameCount = 0;
        m_rejectedFrameCount = 0;
        if (!m_captureActive) {
            m_processingFrame = false;
        }
        return true;
    }
    else if (MsgDeleteStackFrame::match(cmd))
    {
        const MsgDeleteStackFrame& deleteMsg = (const MsgDeleteStackFrame&) cmd;
        deleteStackFrame(deleteMsg.getFrameIndex());
        return true;
    }

    return false;
}

void CameraFrameStacker::handleInputMessages()
{
    Message* message;

    Camera::discardQueuedProcessFramesOnCaptureActive(m_inputMessageQueue);

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

    const bool displayChanged = settingsKeys.contains("stackDisplayMode")
        || settingsKeys.contains("stackDisplayFrameIndex");
    const bool scaleChanged = settingsKeys.contains("scaleEnabled")
        || settingsKeys.contains("scaleWidth")
        || settingsKeys.contains("scaleHeight")
        || settingsKeys.contains("scaleKeepAspectRatio");
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
        || settingsKeys.contains("stackRejectBadFrames")
        || settingsKeys.contains("stackAlignmentMethod");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sourceChanged)
    {
        Camera::discardQueuedProcessFrames(m_inputMessageQueue);
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrames.clear();
        m_droppedFrameCount = 0;
        m_rejectedFrameCount = 0;
        resetFrameHistoryState();
    }
    else if (settingsKeys.contains("stackFrameCount"))
    {
        trimFrameHistoryToCurrentLimit();
    }

    if (!sourceChanged && (displayChanged || scaleChanged)) {
        emitHistoryPreviewFrame();
    }
}

void CameraFrameStacker::submitFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame) {
        return;
    }
    if (!Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
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
                const int dropped = dropOldestPendingFramesForOverflow();
                qDebug() << "CameraFrameStacker: Dropping oldest queued stacking"
                    << (m_settings.isHdrStackingEnabled() ? "HDR bracket frames" : "frame")
                    << dropped;
                m_droppedFrameCount += dropped;
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
            frame->m_stack.m_queuedCount += static_cast<int>(m_pendingFrames.size());
            frame->m_stack.m_droppedCount += m_droppedFrameCount;
            frame->m_stack.m_rejectedCount += m_rejectedFrameCount;
        }

        if (!frame)
        {
            m_processingFrame = false;
            return;
        }
    }

    if (Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
        processNewFrame(frame);
    }

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
    if (!frame || !frame->hasImageData()) {
        return;
    }

    const bool passThroughFrame = canPassThroughFrame(*frame);
    if (passThroughFrame)
    {
        frame->m_stack.m_count = 1;
        if (!applyOutputScaling(*frame)) {
            return;
        }
        m_lastFrameTemplate.reset(new CameraPipelineFrame(*frame));
        if (m_nextStage && Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
            m_nextStage->submitFrame(frame);
        }
        return;
    }

    if (!frame->ensureCpuImageFromCuda()) {
        return;
    }

    frame->m_stack.m_rejectedCount = m_rejectedFrameCount;

    QImage stackedImage;
    int stackCount = 1;

    if (!applyFrameStacking(*frame, stackedImage, stackCount)) {
        return;
    }

    frame->m_stack.m_rejectedCount = m_rejectedFrameCount;
    if (!stackedImage.isNull())
    {
        frame->m_image = stackedImage;
        frame->clearCudaCache();
    }
    else if (!frame->hasCudaBgrImage() && !frame->hasCudaGrayImage())
    {
        return;
    }
    frame->m_bayerPattern = CameraPipelineFrame::BayerNone;
    frame->m_stack.m_count = std::max(1, stackCount);
    if (!applyOutputScaling(*frame)) {
        return;
    }
    m_lastFrameTemplate.reset(new CameraPipelineFrame(*frame));

    if (m_nextStage && Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
        m_nextStage->submitFrame(frame);
    }
}

bool CameraFrameStacker::canPassThroughFrame(const CameraPipelineFrame& inputFrame) const
{
    const bool hdrStackingEnabled = m_settings.isHdrStackingEnabled() && (m_settings.getHdrExposureCount() > 1);
    const bool stackEnabled = hdrStackingEnabled
        || (m_settings.m_stackEnabled && (m_settings.m_stackFrameCount > 1));

    return !stackEnabled && inputFrame.hasImageData();
}

QSize CameraFrameStacker::scaledOutputSize(const QSize& inputSize) const
{
    if (!m_settings.m_scaleEnabled
        || inputSize.isEmpty()
        || (m_settings.m_scaleWidth <= 0)
        || (m_settings.m_scaleHeight <= 0))
    {
        return inputSize;
    }

    int targetWidth = qBound(1, m_settings.m_scaleWidth, 65535);
    int targetHeight = qBound(1, m_settings.m_scaleHeight, 65535);

    if (m_settings.m_scaleKeepAspectRatio)
    {
        const double scale = std::min(
            static_cast<double>(targetWidth) / static_cast<double>(inputSize.width()),
            static_cast<double>(targetHeight) / static_cast<double>(inputSize.height()));
        targetWidth = std::max(1, static_cast<int>(std::lround(static_cast<double>(inputSize.width()) * scale)));
        targetHeight = std::max(1, static_cast<int>(std::lround(static_cast<double>(inputSize.height()) * scale)));
    }

    return QSize(targetWidth, targetHeight);
}

bool CameraFrameStacker::applyOutputScaling(CameraPipelineFrame& frame)
{
    const QSize inputSize = frame.imageSize();
    const QSize targetSize = scaledOutputSize(inputSize);
    if (targetSize == inputSize) {
        return true;
    }
    if (targetSize.isEmpty()) {
        return false;
    }

#ifdef CAMERA_OPENCV_CUDA_STACKING
    if (canUseCudaStacking() && applyOutputScalingCuda(frame, targetSize)) {
        return true;
    }
#endif

    if (!frame.ensureCpuImageFromCuda() || frame.m_image.isNull()) {
        return false;
    }

    bool highBitDepthInput = false;
    const cv::Mat inputMat = imageToWorkingMat(frame.m_image, highBitDepthInput);
    Q_UNUSED(highBitDepthInput);
    if (inputMat.empty()) {
        return false;
    }

    cv::Mat scaledMat;
    const bool downscale = (targetSize.width() < inputMat.cols) || (targetSize.height() < inputMat.rows);
    const int interpolation = downscale ? cv::INTER_AREA : cv::INTER_CUBIC;
    cv::resize(inputMat, scaledMat, cv::Size(targetSize.width(), targetSize.height()), 0.0, 0.0, interpolation);
    frame.m_image = workingMatToImage(scaledMat);
    frame.clearCudaCache();
    return !frame.m_image.isNull();
}

#ifdef CAMERA_OPENCV_CUDA_STACKING
bool CameraFrameStacker::applyOutputScalingCuda(CameraPipelineFrame& frame, const QSize& targetSize)
{
    const cv::Size targetCvSize(targetSize.width(), targetSize.height());
    constexpr int interpolation = cv::INTER_LINEAR;

    if (frame.hasCudaBgrImage())
    {
        cv::cuda::GpuMat scaledGpu;
        cv::cuda::resize(frame.m_cudaBgrImage, scaledGpu, targetCvSize, 0.0, 0.0, interpolation, m_cudaStackingStream);
        frame.m_cudaBgrImage = scaledGpu;
        frame.m_cudaGrayImage.release();
        frame.clearCpuImage();
        m_cudaStackingStream.waitForCompletion();
        return true;
    }

    if (frame.hasCudaGrayImage())
    {
        cv::cuda::GpuMat scaledGpu;
        cv::cuda::resize(frame.m_cudaGrayImage, scaledGpu, targetCvSize, 0.0, 0.0, interpolation, m_cudaStackingStream);
        frame.m_cudaGrayImage = scaledGpu;
        frame.m_cudaBgrImage.release();
        frame.clearCpuImage();
        m_cudaStackingStream.waitForCompletion();
        return true;
    }

    return false;
}
#endif

void CameraFrameStacker::ensureHistoryThumbnails()
{
    while (m_stackFrameThumbnails.size() < m_stackFrameHistory.size())
    {
        const size_t index = m_stackFrameThumbnails.size();
        m_stackFrameThumbnails.push_back(makeHistoryThumbnail(m_stackFrameHistory[index]));
    }

    while (m_stackFrameThumbnails.size() > m_stackFrameHistory.size()) {
        m_stackFrameThumbnails.pop_back();
    }
}

bool CameraFrameStacker::renderStackDisplayImage(const QImage& stackedImage, QImage& outputImage)
{
    if (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayHistoryFrame)
    {
        if (m_stackFrameHistory.empty()) {
            return false;
        }

        const int frameIndex = qBound(0, m_settings.m_stackDisplayFrameIndex, static_cast<int>(m_stackFrameHistory.size()) - 1);
        outputImage = workingMatToImage(m_stackFrameHistory[static_cast<size_t>(frameIndex)]);
        return !outputImage.isNull();
    }

    if (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayHistoryTiles)
    {
        ensureHistoryThumbnails();
        outputImage = makeHistoryTilesImage(m_stackFrameThumbnails, m_stackFrameQualityHistory);
        return !outputImage.isNull();
    }

    outputImage = stackedImage;
    return !outputImage.isNull();
}

bool CameraFrameStacker::shouldRejectStackFrame(const StackFrameQuality& quality, QString& reason) const
{
    if (!quality.m_valid)
    {
        reason = QStringLiteral("invalid quality metrics");
        return true;
    }

    if ((quality.m_mean < 1.0) && (quality.m_stdDev < 1.0))
    {
        reason = QStringLiteral("near-empty frame");
        return true;
    }

    if (quality.m_blackFraction > 0.98)
    {
        reason = QStringLiteral("mostly black frame");
        return true;
    }

    if (quality.m_saturatedFraction > 0.80)
    {
        reason = QStringLiteral("mostly saturated frame");
        return true;
    }

    if (m_stackFrameQualityHistory.size() < 3) {
        return false;
    }

    const double medianMean = medianQualityValue(m_stackFrameQualityHistory, &StackFrameQuality::m_mean);
    const double medianStdDev = medianQualityValue(m_stackFrameQualityHistory, &StackFrameQuality::m_stdDev);
    const double medianSharpness = medianQualityValue(m_stackFrameQualityHistory, &StackFrameQuality::m_laplacianVariance);
    const double medianBlackFraction = medianQualityValue(m_stackFrameQualityHistory, &StackFrameQuality::m_blackFraction);
    const double medianSaturatedFraction = medianQualityValue(m_stackFrameQualityHistory, &StackFrameQuality::m_saturatedFraction);

    if ((medianSharpness > 10.0) && (quality.m_laplacianVariance < medianSharpness * 0.30))
    {
        reason = QStringLiteral("sharpness dropped");
        return true;
    }

    if ((medianMean > 5.0) && (std::abs(quality.m_mean - medianMean) > 25.0)
        && ((quality.m_mean < medianMean * 0.35) || (quality.m_mean > medianMean * 2.5)))
    {
        reason = QStringLiteral("brightness outlier");
        return true;
    }

    if ((medianStdDev > 3.0) && (quality.m_stdDev < medianStdDev * 0.25))
    {
        reason = QStringLiteral("contrast collapsed");
        return true;
    }

    if ((quality.m_blackFraction > std::max(0.50, medianBlackFraction * 4.0 + 0.05))
        && (quality.m_blackFraction > medianBlackFraction + 0.20))
    {
        reason = QStringLiteral("black pixel fraction outlier");
        return true;
    }

    if ((quality.m_saturatedFraction > std::max(0.35, medianSaturatedFraction * 4.0 + 0.05))
        && (quality.m_saturatedFraction > medianSaturatedFraction + 0.20))
    {
        reason = QStringLiteral("saturated pixel fraction outlier");
        return true;
    }

    return false;
}

bool CameraFrameStacker::shouldRejectStackAlignment(const CameraPipelineFrame& inputFrame, QString& reason) const
{
    if (!inputFrame.m_stack.m_alignmentAttempted || inputFrame.m_stack.m_alignmentAccepted) {
        return false;
    }

    reason = inputFrame.m_stack.m_alignmentRejectReason.isEmpty()
        ? QStringLiteral("alignment quality check failed")
        : inputFrame.m_stack.m_alignmentRejectReason;
    return true;
}

void CameraFrameStacker::deleteStackFrame(int frameIndex)
{
    if (m_stackFrameHistory.empty()) {
        return;
    }

    const int boundedFrameIndex = qBound(0, frameIndex, static_cast<int>(m_stackFrameHistory.size()) - 1);
    m_stackFrameHistory.erase(m_stackFrameHistory.begin() + boundedFrameIndex);
    if (boundedFrameIndex < static_cast<int>(m_stackFrameQualityHistory.size())) {
        m_stackFrameQualityHistory.erase(m_stackFrameQualityHistory.begin() + boundedFrameIndex);
    }
    if (boundedFrameIndex < static_cast<int>(m_stackFrameThumbnails.size())) {
        m_stackFrameThumbnails.erase(m_stackFrameThumbnails.begin() + boundedFrameIndex);
    }
    m_stackAccumulator.release();
#ifdef CAMERA_OPENCV_CUDA_STACKING
    m_cudaStackAccumulator.release();
    m_cudaStackAccumulatorInputType = -1;
#endif

    if (m_settings.m_stackDisplayFrameIndex >= static_cast<int>(m_stackFrameHistory.size())) {
        m_settings.m_stackDisplayFrameIndex = std::max(0, static_cast<int>(m_stackFrameHistory.size()) - 1);
    }

    emitHistoryPreviewFrame();
}

void CameraFrameStacker::emitHistoryPreviewFrame()
{
    if (!m_nextStage || !m_lastFrameTemplate || m_stackFrameHistory.empty()) {
        return;
    }

    QImage outputImage;
    if (!renderStackDisplayImage(m_lastStackedImage, outputImage)) {
        return;
    }

    CameraPipelineFramePtr previewFrame(new CameraPipelineFrame(*m_lastFrameTemplate));
    previewFrame->m_manualPreviewFrame = true;
    previewFrame->m_image = outputImage;
    previewFrame->m_unprocessedImage = outputImage;
    previewFrame->clearCudaCache();
    previewFrame->m_stack.m_count = static_cast<int>(m_stackFrameHistory.size());
    previewFrame->m_stack.m_queuedCount = 0;
    previewFrame->m_stack.m_rejectedCount = m_rejectedFrameCount;
    if (!applyOutputScaling(*previewFrame)) {
        return;
    }
    previewFrame->m_unprocessedImage = previewFrame->m_image;
    m_nextStage->submitFrame(previewFrame);
}

bool CameraFrameStacker::applyFrameStacking(CameraPipelineFrame& inputFrame, QImage& outputImage, int& stackCount)
{
    PROFILER_START();
    const bool hdrStackingEnabled = m_settings.isHdrStackingEnabled() && (m_settings.getHdrExposureCount() > 1);
    const bool stackEnabled = hdrStackingEnabled
        || (m_settings.m_stackEnabled && (m_settings.m_stackFrameCount > 1));

    if (!stackEnabled)
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
#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
    if (useCudaStacking)
    {
        if (inputFrame.hasCudaBgrImage()
            && (inputFrame.m_cudaBgrImage.size() == frameMat.size()))
        {
            cv::cuda::cvtColor(inputFrame.m_cudaBgrImage, cudaFrameMat, cv::COLOR_BGR2RGB, 0, m_cudaStackingStream);
        }
        else if (inputFrame.hasCudaGrayImage()
            && (inputFrame.m_cudaGrayImage.size() == frameMat.size()))
        {
            cudaFrameMat = inputFrame.m_cudaGrayImage;
        }
    }
#endif
#else
    const bool useCudaStacking = false;
#endif

    if (hdrStackingEnabled)
    {
        if (frameMat.channels() == 1) {
            // See companion comment at the second GRAY2RGB site below. frameMat may
            // alias inputFrame.m_image's bits at this point (clone elided in
            // imageToWorkingMat). Using an explicit dst Mat keeps the reallocation
            // visible and decouples us from OpenCV's internal in-place rules.
            cv::Mat colorMat;
            cv::cvtColor(frameMat, colorMat, cv::COLOR_GRAY2RGB);
            frameMat = colorMat;
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

        HdrFrameSample hdrSample;
        hdrSample.m_frameMat = frameMat.clone();
#ifdef CAMERA_OPENCV_CUDA_STACKING
        if (!cudaFrameMat.empty()
            && (cudaFrameMat.size() == frameMat.size())
            && (cudaFrameMat.channels() == frameMat.channels()))
        {
            hdrSample.m_frameGpu = cudaFrameMat;
        }
#endif
        hdrSample.m_exposureTimeMs = std::max(CameraSettings::m_minExposureTimeMs, inputFrame.m_exposureTimeMs);
        m_hdrFrameSamples.push_back(hdrSample);
        stackCount = static_cast<int>(m_hdrFrameSamples.size());

        if (static_cast<int>(m_hdrFrameSamples.size()) < inputFrame.m_hdrExposureCount)
        {
            PROFILER_STOP(__FUNCTION__);
            return false;
        }

        try
        {
            QElapsedTimer hdrTimer;
            hdrTimer.start();
            qint64 hdrSortMs = 0;
            qint64 hdrPrepareMs = 0;
            qint64 hdrMergeMs = 0;
            qint64 hdrTonemapMs = 0;
            qint64 hdrValidateMs = 0;
            qint64 hdrFallbackMs = 0;
            qint64 hdrOutputMs = 0;
            const char *hdrFallback = "none";
            bool hdrCudaUsed = false;
            auto elapsedSince = [&hdrTimer](qint64& previousMs) {
                const qint64 nowMs = hdrTimer.elapsed();
                const qint64 deltaMs = nowMs - previousMs;
                previousMs = nowMs;
                return deltaMs;
            };
            qint64 previousHdrMs = 0;

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

            auto useMiddleExposureFrame = [](const std::vector<cv::Mat>& ldrFrames, cv::Mat& tonemapped) -> bool
            {
                if (ldrFrames.empty()) {
                    return false;
                }

                const int middleIndex = static_cast<int>(ldrFrames.size() / 2);
                ldrFrames[middleIndex].convertTo(tonemapped, CV_32FC3, 1.0 / 255.0);
                return true;
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
            hdrSortMs = elapsedSince(previousHdrMs);

            std::vector<cv::Mat> ldrFrames;
            std::vector<float> exposureTimesSeconds;
            bool exposureTimesPrepared = false;
            auto prepareExposureTimes = [&]()
            {
                if (exposureTimesPrepared) {
                    return;
                }

                exposureTimesSeconds.reserve(m_hdrFrameSamples.size());
                for (const HdrFrameSample *sample : sortedSamples)
                {
                    float exposureSeconds = static_cast<float>(std::max(1.0e-6, sample->m_exposureTimeMs / 1000.0));
                    if (!exposureTimesSeconds.empty() && (exposureSeconds <= exposureTimesSeconds.back())) {
                        exposureSeconds = exposureTimesSeconds.back() + std::max(1.0e-6f, exposureTimesSeconds.back() * 1.0e-4f);
                    }
                    exposureTimesSeconds.push_back(exposureSeconds);
                }
                exposureTimesPrepared = true;
            };
            bool ldrFramesPrepared = false;
            auto prepareLdrFrames = [&]()
            {
                if (ldrFramesPrepared) {
                    return;
                }

                prepareExposureTimes();
                ldrFrames.reserve(m_hdrFrameSamples.size());

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
                }

                ldrFramesPrepared = true;
                hdrPrepareMs = elapsedSince(previousHdrMs);
            };

            cv::Mat tonemapped;

            if (m_settings.m_stackHdrAlgorithm == CameraSettings::StackHdrAlgorithmMertens)
            {
#ifdef CAMERA_OPENCV_CUDA_STACKING
                cv::cuda::GpuMat tonemappedGpu;
                if (useCudaStacking && applyMertensFusionCuda(sortedSamples, tonemappedGpu))
                {
                    hdrCudaUsed = true;
                    hdrMergeMs = elapsedSince(previousHdrMs);

                    if (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayStacked)
                    {
                        const int completedHdrFrameCount = static_cast<int>(m_hdrFrameSamples.size());
                        setCudaBgrOutputFromRgbGpu(inputFrame, tonemappedGpu, CV_8U);
                        m_cudaStackingStream.waitForCompletion();
                        outputImage = QImage();
                        stackCount = completedHdrFrameCount;
                        m_hdrFrameSamples.clear();
                        m_lastStackedImage = QImage();
                        hdrOutputMs = elapsedSince(previousHdrMs);

                        qDebug() << "CameraFrameStacker: HDR processing timing"
                                 << "algorithm" << m_settings.m_stackHdrAlgorithm
                                 << "frames" << completedHdrFrameCount
                                 << "size" << frameMat.cols << "x" << frameMat.rows
                                 << "inputDepth" << frameMat.depth()
                                 << "cudaAvailable" << useCudaStacking
                                 << "cudaUsed" << hdrCudaUsed
                                 << "sortMs" << hdrSortMs
                                 << "prepareMs" << hdrPrepareMs
                                 << "mergeMs" << hdrMergeMs
                                 << "tonemapMs" << hdrTonemapMs
                                 << "validateMs" << hdrValidateMs
                                 << "fallbackMs" << hdrFallbackMs
                                 << "outputMs" << hdrOutputMs
                                 << "totalMs" << hdrTimer.elapsed()
                                 << "fallback" << hdrFallback;
                        PROFILER_STOP(__FUNCTION__);
                        return true;
                    }

                    tonemappedGpu.download(tonemapped, m_cudaStackingStream);
                    m_cudaStackingStream.waitForCompletion();
                }
                else
#endif
                {
                    prepareLdrFrames();
                    cv::Ptr<cv::MergeMertens> mergeMertens = cv::createMergeMertens();
                    mergeMertens->process(ldrFrames, tonemapped);
                    hdrMergeMs = elapsedSince(previousHdrMs);
                }
            }
            else
            {
                cv::Mat responseCurve;
                cv::Mat hdrRadiance;

                if (m_settings.m_stackHdrAlgorithm == CameraSettings::StackHdrAlgorithmRobertson)
                {
                    prepareLdrFrames();
                    cv::Mat timesMat(static_cast<int>(exposureTimesSeconds.size()), 1, CV_32F, exposureTimesSeconds.data());
                    cv::Ptr<cv::CalibrateRobertson> calibrateRobertson = cv::createCalibrateRobertson();
                    calibrateRobertson->process(ldrFrames, responseCurve, timesMat);

                    cv::Ptr<cv::MergeRobertson> mergeRobertson = cv::createMergeRobertson();
                    mergeRobertson->process(ldrFrames, hdrRadiance, timesMat, responseCurve);
                }
                else
                {
#ifdef CAMERA_OPENCV_CUDA_STACKING
                    cv::cuda::GpuMat tonemappedGpu;
                    prepareExposureTimes();
                    if (useCudaStacking && applyDebevecFusionCuda(sortedSamples, exposureTimesSeconds, tonemappedGpu))
                    {
                        hdrCudaUsed = true;
                        hdrMergeMs = elapsedSince(previousHdrMs);

                        if (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayStacked)
                        {
                            const int completedHdrFrameCount = static_cast<int>(m_hdrFrameSamples.size());
                            setCudaBgrOutputFromRgbGpu(inputFrame, tonemappedGpu, CV_8U);
                            m_cudaStackingStream.waitForCompletion();
                            outputImage = QImage();
                            stackCount = completedHdrFrameCount;
                            m_hdrFrameSamples.clear();
                            m_lastStackedImage = QImage();
                            hdrOutputMs = elapsedSince(previousHdrMs);

                            qDebug() << "CameraFrameStacker: HDR processing timing"
                                     << "algorithm" << m_settings.m_stackHdrAlgorithm
                                     << "frames" << completedHdrFrameCount
                                     << "size" << frameMat.cols << "x" << frameMat.rows
                                     << "inputDepth" << frameMat.depth()
                                     << "cudaAvailable" << useCudaStacking
                                     << "cudaUsed" << hdrCudaUsed
                                     << "sortMs" << hdrSortMs
                                     << "prepareMs" << hdrPrepareMs
                                     << "mergeMs" << hdrMergeMs
                                     << "tonemapMs" << hdrTonemapMs
                                     << "validateMs" << hdrValidateMs
                                     << "fallbackMs" << hdrFallbackMs
                                     << "outputMs" << hdrOutputMs
                                     << "totalMs" << hdrTimer.elapsed()
                                     << "fallback" << hdrFallback;
                            PROFILER_STOP(__FUNCTION__);
                            return true;
                        }

                        tonemappedGpu.download(tonemapped, m_cudaStackingStream);
                        m_cudaStackingStream.waitForCompletion();
                    }
                    else
#endif
                    {
                        prepareLdrFrames();
                        cv::Mat timesMat(static_cast<int>(exposureTimesSeconds.size()), 1, CV_32F, exposureTimesSeconds.data());
                        cv::Ptr<cv::MergeDebevec> mergeDebevec = cv::createMergeDebevec();
                        // Live brackets are too small and scene-dependent for a reliable per-stack CRF fit.
                        mergeDebevec->process(ldrFrames, hdrRadiance, timesMat);

                        if (hdrRadiance.depth() != CV_32F) {
                            hdrRadiance.convertTo(hdrRadiance, CV_32FC3);
                        }
                        sanitizeFloatImage(hdrRadiance, false);
                        hdrMergeMs = elapsedSince(previousHdrMs);

                        cv::Ptr<cv::TonemapReinhard> tonemap = cv::createTonemapReinhard(2.2f, 0.0f, 1.0f, 0.0f);
                        tonemap->process(hdrRadiance, tonemapped);
                        hdrTonemapMs = elapsedSince(previousHdrMs);
                    }
                }
            }

            if (tonemapped.depth() != CV_32F) {
                tonemapped.convertTo(tonemapped, CV_32FC3);
            }
            sanitizeFloatImage(tonemapped, true);
            hdrValidateMs = elapsedSince(previousHdrMs);
            if (!isUsefulFloatImage(tonemapped))
            {
                if (m_settings.m_stackHdrAlgorithm == CameraSettings::StackHdrAlgorithmMertens)
                {
                    qWarning() << "CameraFrameStacker: HDR exposure fusion produced an unusable image; falling back to middle exposure";
                    prepareLdrFrames();
                    if (useMiddleExposureFrame(ldrFrames, tonemapped)) {
                        hdrFallback = "middle";
                    }
                }
                else
                {
                    qWarning() << "CameraFrameStacker: HDR merge produced an unusable image; falling back to exposure fusion";
                    prepareLdrFrames();

                    if (!ldrFrames.empty())
                    {
                        cv::Ptr<cv::MergeMertens> mergeMertens = cv::createMergeMertens();
                        mergeMertens->process(ldrFrames, tonemapped);
                        hdrFallback = "mertens";
                        if (tonemapped.depth() != CV_32F) {
                            tonemapped.convertTo(tonemapped, CV_32FC3);
                        }
                        sanitizeFloatImage(tonemapped, true);
                    }

                    if (!isUsefulFloatImage(tonemapped))
                    {
                        if (useMiddleExposureFrame(ldrFrames, tonemapped)) {
                            hdrFallback = "middle";
                        }
                    }
                }
            }
            hdrFallbackMs = elapsedSince(previousHdrMs);

            if (!hdrCudaUsed) {
                cv::cvtColor(tonemapped, tonemapped, cv::COLOR_BGR2RGB);
            }

            cv::Mat ldr8u;
            tonemapped.convertTo(ldr8u, CV_8UC3, 255.0);
#ifdef CAMERA_OPENCV_CUDA_STACKING
            if (useCudaStacking
                && (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayStacked)
                && setCudaBgrOutputFromRgbMat(inputFrame, ldr8u))
            {
                outputImage = QImage();
                stackCount = static_cast<int>(m_hdrFrameSamples.size());
                m_lastStackedImage = QImage();
                hdrOutputMs = elapsedSince(previousHdrMs);

                qDebug() << "CameraFrameStacker: HDR processing timing"
                         << "algorithm" << m_settings.m_stackHdrAlgorithm
                         << "frames" << static_cast<int>(m_hdrFrameSamples.size())
                         << "size" << frameMat.cols << "x" << frameMat.rows
                         << "inputDepth" << frameMat.depth()
                         << "cudaAvailable" << useCudaStacking
                         << "cudaUsed" << hdrCudaUsed
                         << "sortMs" << hdrSortMs
                         << "prepareMs" << hdrPrepareMs
                         << "mergeMs" << hdrMergeMs
                         << "tonemapMs" << hdrTonemapMs
                         << "validateMs" << hdrValidateMs
                         << "fallbackMs" << hdrFallbackMs
                         << "outputMs" << hdrOutputMs
                         << "totalMs" << hdrTimer.elapsed()
                         << "fallback" << hdrFallback;
                m_hdrFrameSamples.clear();
                PROFILER_STOP(__FUNCTION__);
                return true;
            }
#endif
            outputImage = workingMatToImage(ldr8u);
            hdrOutputMs = elapsedSince(previousHdrMs);

            qDebug() << "CameraFrameStacker: HDR processing timing"
                     << "algorithm" << m_settings.m_stackHdrAlgorithm
                     << "frames" << static_cast<int>(ldrFrames.size())
                     << "size" << frameMat.cols << "x" << frameMat.rows
                     << "inputDepth" << frameMat.depth()
                     << "cudaAvailable" << useCudaStacking
                     << "cudaUsed" << hdrCudaUsed
                     << "sortMs" << hdrSortMs
                     << "prepareMs" << hdrPrepareMs
                     << "mergeMs" << hdrMergeMs
                     << "tonemapMs" << hdrTonemapMs
                     << "validateMs" << hdrValidateMs
                     << "fallbackMs" << hdrFallbackMs
                     << "outputMs" << hdrOutputMs
                     << "totalMs" << hdrTimer.elapsed()
                     << "fallback" << hdrFallback;
        }
        catch (const cv::Exception& error)
        {
            qWarning() << "CameraFrameStacker: HDR merge failed:" << error.what();
            outputImage = workingMatToImage(frameMat);
            stackCount = 1;
        }

        m_hdrFrameSamples.clear();
        m_lastStackedImage = outputImage;
        QImage displayImage;
        if (renderStackDisplayImage(outputImage, displayImage)) {
            outputImage = displayImage;
        }
        PROFILER_STOP(__FUNCTION__);
        return true;
    }

    if (frameMat.channels() == 1) {
        // frameMat may alias inputFrame.m_image's bits when no upstream stage
        // reallocated (imageToWorkingMat no longer clones the Grayscale paths). The
        // aliasing in-place form cvtColor(frameMat, frameMat, ...) happens to be safe
        // today only because cvtColor's internal Mat::create reallocates whenever the
        // destination type differs from the source. Make that allocation explicit so
        // we don't depend on OpenCV internals.
        cv::Mat colorMat;
        cv::cvtColor(frameMat, colorMat, cv::COLOR_GRAY2RGB);
        frameMat = colorMat;
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
        m_stackFrameQualityHistory.clear();
        m_stackFrameThumbnails.clear();
        m_stackAccumulator.release();
#ifdef CAMERA_OPENCV_CUDA_STACKING
        m_cudaStackAccumulator.release();
        m_cudaStackAccumulatorInputType = -1;
#endif
    }

    StackFrameQuality quality;
    const bool needQuality = (m_settings.m_stackMethod == CameraSettings::StackMethodLuckySharpAverage)
        || m_settings.m_stackRejectBadFrames
        || (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayHistoryTiles);
    if (needQuality) {
        quality = computeStackFrameQualityForFrame(inputFrame, alignedFrameMat);
    }
    QString rejectReason;
    const bool rejectForAlignment = m_settings.m_stackRejectBadFrames && shouldRejectStackAlignment(inputFrame, rejectReason);
    const bool rejectForQuality = !rejectForAlignment
        && m_settings.m_stackRejectBadFrames
        && shouldRejectStackFrame(quality, rejectReason);
    if (rejectForAlignment || rejectForQuality)
    {
        ++m_rejectedFrameCount;
        inputFrame.m_stack.m_rejectReason = rejectReason;
        qDebug() << "CameraFrameStacker: Rejecting frame from stack:" << rejectReason
                 << "alignmentResponse" << inputFrame.m_stack.m_alignmentResponse
                 << "alignmentShift" << inputFrame.m_stack.m_alignmentShiftPixels
                 << "alignmentStars" << inputFrame.m_stack.m_alignmentMatchedStars
                 << "mean" << quality.m_mean
                 << "stddev" << quality.m_stdDev
                 << "sharpness" << quality.m_laplacianVariance
                 << "black" << quality.m_blackFraction
                 << "saturated" << quality.m_saturatedFraction;
        stackCount = static_cast<int>(m_stackFrameHistory.size());

        if (!m_stackFrameHistory.empty() && renderStackDisplayImage(m_lastStackedImage, outputImage))
        {
            PROFILER_STOP(__FUNCTION__);
            return true;
        }

        outputImage = workingMatToImage(alignedFrameMat);
        PROFILER_STOP(__FUNCTION__);
        return true;
    }
    inputFrame.m_stack.m_rejectReason.clear();

    m_stackFrameHistory.push_back(alignedFrameMat.clone());
    if (needQuality) {
        m_stackFrameQualityHistory.push_back(quality);
    }
    if (!m_stackFrameThumbnails.empty()
        || (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayHistoryTiles))
    {
        m_stackFrameThumbnails.push_back(makeHistoryThumbnail(m_stackFrameHistory.back()));
    }
    trimFrameHistoryToCurrentLimit();

    if (m_settings.m_stackMethod == CameraSettings::StackMethodLuckySharpAverage)
    {
        ensureStackFrameQualityHistory();
        const std::vector<size_t> selectedIndices = selectedSharpFrameIndices();
        cv::Mat luckyAccumulator = cv::Mat::zeros(alignedFrameMat.size(), CV_32FC3);

        for (size_t index : selectedIndices)
        {
            cv::Mat floatFrame;
            m_stackFrameHistory[index].convertTo(floatFrame, CV_32FC3);
            luckyAccumulator += floatFrame;
        }

        cv::Mat averagedFloat;
        luckyAccumulator.convertTo(averagedFloat, CV_32FC3, 1.0 / static_cast<double>(std::max<size_t>(1, selectedIndices.size())));
        cv::Mat averagedOutput;
        const int outputType = (alignedFrameMat.depth() == CV_16U) ? CV_16UC3 : CV_8UC3;
        averagedFloat.convertTo(averagedOutput, outputType);

#ifdef CAMERA_OPENCV_CUDA_STACKING
        if (useCudaStacking
            && (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayStacked)
            && setCudaBgrOutputFromRgbMat(inputFrame, averagedOutput))
        {
            m_lastStackedImage = QImage();
            outputImage = QImage();
            stackCount = static_cast<int>(selectedIndices.size());
            PROFILER_STOP(__FUNCTION__);
            return true;
        }
#endif

        const QImage stackedImage = workingMatToImage(averagedOutput);
        m_lastStackedImage = stackedImage;
        if (!renderStackDisplayImage(stackedImage, outputImage)) {
            outputImage = stackedImage;
        }
        stackCount = static_cast<int>(selectedIndices.size());

        PROFILER_STOP(__FUNCTION__);
        return true;
    }

    if (m_settings.m_stackMethod == CameraSettings::StackMethodAverage)
    {
#ifdef CAMERA_OPENCV_CUDA_STACKING
        cv::cuda::GpuMat averagedRgbGpu;
        if (useCudaStacking && applyAverageStackingCuda(alignedFrameMat, cudaFrameMat.empty() ? nullptr : &cudaFrameMat, averagedRgbGpu))
        {
            if (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayStacked)
            {
                cv::cuda::cvtColor(averagedRgbGpu, inputFrame.m_cudaBgrImage, cv::COLOR_RGB2BGR, 0, m_cudaStackingStream);
                inputFrame.m_cudaGrayImage.release();
                inputFrame.clearCpuImage();
                m_cudaStackingStream.waitForCompletion();
                m_lastStackedImage = QImage();
                outputImage = QImage();
            }
            else
            {
                cv::Mat averagedOutput;
                averagedRgbGpu.download(averagedOutput, m_cudaStackingStream);
                m_cudaStackingStream.waitForCompletion();
                const QImage stackedImage = workingMatToImage(averagedOutput);
                QImage displayImage;
                m_lastStackedImage = stackedImage;
                if (renderStackDisplayImage(stackedImage, displayImage)) {
                    outputImage = displayImage;
                } else {
                    outputImage = stackedImage;
                }
            }
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
        cv::Mat averagedOutput;
        const int outputType = (alignedFrameMat.depth() == CV_16U) ? CV_16UC3 : CV_8UC3;
        averagedFloat.convertTo(averagedOutput, outputType);

#ifdef CAMERA_OPENCV_CUDA_STACKING
        if (useCudaStacking
            && (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayStacked)
            && setCudaBgrOutputFromRgbMat(inputFrame, averagedOutput))
        {
            m_lastStackedImage = QImage();
            outputImage = QImage();
            stackCount = static_cast<int>(m_stackFrameHistory.size());
            PROFILER_STOP(__FUNCTION__);
            return true;
        }
#endif

        const QImage stackedImage = workingMatToImage(averagedOutput);
        m_lastStackedImage = stackedImage;
        if (!renderStackDisplayImage(stackedImage, outputImage)) {
            outputImage = stackedImage;
        }
        stackCount = static_cast<int>(m_stackFrameHistory.size());

        PROFILER_STOP(__FUNCTION__);
        return true;
    }

    m_stackAccumulator.release();

    const int stackedOutputType = highBitDepthInput ? CV_16UC3 : CV_8UC3;
    cv::Mat stackedMat(alignedFrameMat.rows, alignedFrameMat.cols, stackedOutputType);
    const size_t frameCount = m_stackFrameHistory.size();
    constexpr double sigmaThreshold = 2.0;
    const bool medianStacking = m_settings.m_stackMethod == CameraSettings::StackMethodMedian;

    cv::parallel_for_(cv::Range(0, alignedFrameMat.rows),
        [&](const cv::Range& range)
        {
            std::vector<int> medianSamples[3];
            if (medianStacking)
            {
                for (std::vector<int>& samples : medianSamples) {
                    samples.resize(frameCount);
                }
            }

            for (int row = range.start; row < range.end; ++row)
            {
                cv::Vec<uint16_t, 3> *output16 = highBitDepthInput
                    ? stackedMat.ptr<cv::Vec<uint16_t, 3>>(row)
                    : nullptr;
                cv::Vec3b *output8 = highBitDepthInput
                    ? nullptr
                    : stackedMat.ptr<cv::Vec3b>(row);

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
                            if (highBitDepthInput) {
                                output16[col][channel] = static_cast<uint16_t>(qBound(0, samples[medianIndex], 65535));
                            } else {
                                output8[col][channel] = static_cast<uchar>(qBound(0, samples[medianIndex], 255));
                            }
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
                        if (highBitDepthInput) {
                            output16[col][channel] = static_cast<uint16_t>(qBound(0, channelValue, 65535));
                        } else {
                            output8[col][channel] = static_cast<uchar>(qBound(0, channelValue, 255));
                        }
                    }
                }
            }
        });

    PROFILER_STOP(__FUNCTION__);
#ifdef CAMERA_OPENCV_CUDA_STACKING
    if (useCudaStacking && (m_settings.m_stackDisplayMode == CameraSettings::StackDisplayStacked))
    {
        if (setCudaBgrOutputFromRgbMat(inputFrame, stackedMat))
        {
            m_lastStackedImage = QImage();
            outputImage = QImage();
            stackCount = static_cast<int>(m_stackFrameHistory.size());
            return true;
        }
    }
#endif
    const QImage stackedImage = workingMatToImage(stackedMat);
    m_lastStackedImage = stackedImage;
    if (!renderStackDisplayImage(stackedImage, outputImage)) {
        outputImage = stackedImage;
    }
    stackCount = static_cast<int>(m_stackFrameHistory.size());
    return true;
}
