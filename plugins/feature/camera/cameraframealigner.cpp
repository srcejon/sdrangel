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
#include <limits>

#include <QDebug>
#ifdef CAMERA_OPENCV_CUDA_STACKING
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "util/profiler.h"
#include "camera.h"
#include "cameraframealigner.h"
#include "cameraframestacker.h"
#include "cameraimageutils.h"


CameraFrameAligner::CameraFrameAligner() :
    m_nextStage(nullptr),
    m_captureActive(false),
    m_alignmentReferenceGeneration(0),
    m_processingFrame(false),
    m_droppedFrameCount(0)
{
}

CameraFrameAligner::~CameraFrameAligner() = default;

void CameraFrameAligner::startWork()
{
    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()), Qt::QueuedConnection);
}

void CameraFrameAligner::stopWork()
{
    disconnect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()));
}

void CameraFrameAligner::resetAlignmentState()
{
    m_alignmentReference = cv::Mat();
    m_alignmentReferenceGeometry = CameraPipelineFrameGeometry();
    m_previousStarAlignmentTransform = cv::Mat();
    m_lastStarAlignmentTransform = cv::Mat();
    m_lastStarAlignmentTargetStars.clear();
    m_recentStarAlignmentReferences.clear();
}

bool CameraFrameAligner::preserveFrameOrder() const
{
    return m_captureActive
        && ((m_settings.isHdrStackingEnabled() && (m_settings.getHdrExposureCount() > 1))
            || (m_settings.m_stackEnabled && (m_settings.m_stackFrameCount > 1)));
}

int CameraFrameAligner::pendingFrameLimit() const
{
    const int stackFrameCount = m_settings.isHdrStackingEnabled()
        ? m_settings.getHdrExposureCount()
        : m_settings.m_stackFrameCount;
    return qBound(2, stackFrameCount * 2, 512);
}

bool CameraFrameAligner::handleMessage(const Message& cmd)
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
            resetAlignmentState();
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

void CameraFrameAligner::handleInputMessages()
{
    Message *message;

    Camera::discardQueuedProcessFramesOnCaptureActive(m_inputMessageQueue);

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        if (handleMessage(*message)) {
            delete message;
        }
    }
}

void CameraFrameAligner::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    cameraLogSettingsChange("CameraFrameAligner::applySettings:", settings, settingsKeys, force);

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
        || settingsKeys.contains("postProcessUseCuda")
        || settingsKeys.contains("stackAlignmentMethod");

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (sourceChanged)
    {
        QMutexLocker locker(&m_frameMutex);
        m_pendingFrames.clear();
        m_droppedFrameCount = 0;
        resetAlignmentState();
    }
    // stackFrameCount changes no longer affect alignment — the reference is a single fixed
    // frame and is unaffected by the rolling stack window size. (Previously this branch
    // trimmed an over-loaded deque of warped frames, which was itself the source of the
    // alignment-drift bug.)
}

void CameraFrameAligner::submitFrame(const CameraPipelineFramePtr& frame)
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
                qDebug() << "CameraFrameAligner: Dropping oldest queued stacking frame";
                m_pendingFrames.pop_front();
                ++m_droppedFrameCount;
            }
            m_pendingFrames.push_back(frame);
        }
        else
        {
            if (!m_pendingFrames.empty()) {
                qDebug() << "CameraFrameAligner: Dropping pending frame in favor of new frame";
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
        QMetaObject::invokeMethod(this, &CameraFrameAligner::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraFrameAligner::processNextFrame()
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
        QMetaObject::invokeMethod(this, &CameraFrameAligner::processNextFrame, Qt::QueuedConnection);
    }
}

void CameraFrameAligner::processNewFrame(const CameraPipelineFramePtr& frame)
{
    PROFILER_START();
    if (!frame || !frame->hasImageData()) {
        return;
    }

    if (m_settings.m_stackEnabled
        && (m_settings.m_stackFrameCount > 1)
        && (m_settings.m_stackAlignmentMethod != CameraSettings::StackAlignmentNone))
    {
        if (!frame->ensureCpuImageFromCuda()) {
            return;
        }
        const QImage alignedImage = applyAlignment(*frame);
        if (!alignedImage.isNull())
        {
            frame->m_image = alignedImage;
            frame->clearCudaCache();
        }
    }

    if (m_nextStage && Camera::acceptsPipelineFrame(frame, m_captureActive, m_captureEpoch)) {
        m_nextStage->submitFrame(frame);
    }
    PROFILER_STOP(__FUNCTION__);
}

// Lifetime contract: the returned Mat may alias the source QImage's pixel buffer when no
// format conversion was needed. Callers must keep `input` alive for at least as long as
// the returned Mat is read, or explicitly clone the result if they need it to outlive the
// input. Where convertToFormat() is needed (a non-RGB888 colour input) the helper still
// has to clone, because the converted QImage is a function-local temporary that would
// otherwise leave a dangling pointer behind.
//
// Avoiding the clone saves ~32 MB/frame for 4K Grayscale16, multiplied by three pipeline
// stages — the biggest single throughput win at high resolution / high bit depth.
cv::Mat CameraFrameAligner::imageToWorkingMat(const QImage& input, bool& highBitDepthInput)
{
    return CameraImageUtils::imageToWorkingMat(input, &highBitDepthInput);
}

QImage CameraFrameAligner::workingMatToImage(const cv::Mat& frameMat, bool highBitDepthInput)
{
    (void) highBitDepthInput;
    return CameraImageUtils::workingMatToImage(frameMat);
}

QImage CameraFrameAligner::applyAlignment(CameraPipelineFrame& frame)
{
    PROFILER_START();

    bool highBitDepthInput = false;
    cv::Mat frameMat = imageToWorkingMat(frame.m_image, highBitDepthInput);
    frame.m_stack.m_alignmentAttempted = true;
    frame.m_stack.m_alignmentAccepted = true;
    frame.m_stack.m_alignmentResponse = 0.0f;
    frame.m_stack.m_alignmentShiftPixels = 0.0f;
    frame.m_stack.m_alignmentMatchedStars = 0;
    frame.m_stack.m_alignmentRejectReason.clear();

    // If the frame geometry/type has changed (e.g. ROI, binning, depth), drop the
    // stale reference so we re-anchor on the next frame.
    if (!m_alignmentReference.empty()
        && (m_alignmentReference.size() != frameMat.size()
            || m_alignmentReference.type() != frameMat.type()))
    {
        resetAlignmentState();
    }

#ifdef CAMERA_OPENCV_CUDA_STACKING
    const bool preferCudaWarp = m_settings.m_postProcessUseCuda && frame.hasCudaBgrImage();
#else
    const bool preferCudaWarp = false;
#endif
    cv::Mat appliedTransform;
    cv::Mat alignedFrameMat = alignFrame(frameMat, frame, &appliedTransform, preferCudaWarp);

    // Anchor the reference once on the first frame after a reset. Use the *input*
    // (unaligned) frame so subsequent warps don't compound. We keep this reference for
    // the whole sequence — there is no drift, no discontinuity when the stack rolls.
    if (m_alignmentReference.empty())
    {
        m_alignmentReference = frameMat.clone();
        ++m_alignmentReferenceGeneration;
        if (m_alignmentReferenceGeneration == 0) {
            ++m_alignmentReferenceGeneration;
        }
        m_alignmentReferenceGeometry = frame.captureGeometry(m_alignmentReferenceGeneration);
    }
    frame.m_alignmentReferenceGeometry = m_alignmentReferenceGeometry;

#ifdef CAMERA_OPENCV_CUDA_STACKING
    if (m_settings.m_postProcessUseCuda
        && frame.hasCudaBgrImage()
        && !appliedTransform.empty())
    {
        cv::cuda::GpuMat alignedGpu;
        if (warpFrameAffineCuda(frame.m_cudaBgrImage, appliedTransform, alignedGpu))
        {
            frame.m_cudaBgrImage = alignedGpu;
            frame.m_cudaGrayImage.release();
            frame.clearCpuImage();
            PROFILER_STOP(__FUNCTION__);
            return QImage();
        }
    }
#endif

    if (preferCudaWarp && !appliedTransform.empty()) {
        alignedFrameMat = warpFrameAffine(frameMat, appliedTransform);
    }

    QImage outputImage = workingMatToImage(alignedFrameMat, highBitDepthInput);
    PROFILER_STOP(__FUNCTION__);
    return outputImage;
}

cv::Mat CameraFrameAligner::alignFrame(const cv::Mat& frameMat, CameraPipelineFrame& frame, cv::Mat *appliedTransform, bool deferWarp)
{
    if (appliedTransform) {
        appliedTransform->release();
    }

    if (m_alignmentReference.empty()) {
        return frameMat.clone();
    }

    const cv::Mat& referenceFrame = m_alignmentReference;
    if (referenceFrame.size() != frameMat.size() || referenceFrame.type() != frameMat.type()) {
        return frameMat.clone();
    }

    switch (m_settings.m_stackAlignmentMethod)
    {
    case CameraSettings::StackAlignmentPhaseCorrelation:
        return alignWithPhaseCorrelation(referenceFrame, frameMat, frame, appliedTransform, deferWarp);
    case CameraSettings::StackAlignmentStarCentroidMatching:
        return alignWithStarCentroids(referenceFrame, frameMat, frame, appliedTransform, deferWarp);
    case CameraSettings::StackAlignmentNone:
    default:
        return frameMat.clone();
    }
}

cv::Mat CameraFrameAligner::frameToAlignmentGray(const cv::Mat& frameMat) const
{
    cv::Mat gray;
    if (frameMat.channels() == 1)
    {
        gray = frameMat;
    }
    else
    {
        cv::cvtColor(frameMat, gray, cv::COLOR_RGB2GRAY);
    }

    if (gray.depth() == CV_8U) {
        return gray;
    }

    cv::Mat gray8;
    if (gray.depth() == CV_16U) {
        gray.convertTo(gray8, CV_8U, 255.0 / 65535.0);
    } else {
        gray.convertTo(gray8, CV_8U);
    }

    return gray8;
}

#ifdef CAMERA_OPENCV_CUDA_STACKING
bool CameraFrameAligner::warpFrameAffineCuda(const cv::cuda::GpuMat& inputGpu, const cv::Mat& transform, cv::cuda::GpuMat& outputGpu)
{
    if (inputGpu.empty() || transform.empty()) {
        return false;
    }

    static bool warnedNoDevice = false;
    if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
    {
        if (!warnedNoDevice)
        {
            qWarning() << "CameraFrameAligner: CUDA alignment requested, but no CUDA-enabled OpenCV device is available";
            warnedNoDevice = true;
        }
        return false;
    }

    try
    {
        cv::cuda::warpAffine(inputGpu, outputGpu, transform, inputGpu.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(), m_cudaAlignmentStream);
        m_cudaAlignmentStream.waitForCompletion();
        return true;
    }
    catch (const cv::Exception& error)
    {
        qWarning() << "CameraFrameAligner: CUDA affine warp failed; falling back to CPU:" << error.what();
    }

    return false;
}
#endif

cv::Mat CameraFrameAligner::warpFrameAffine(const cv::Mat& frameMat, const cv::Mat& transform)
{
    if (transform.empty()) {
        return frameMat.clone();
    }

    cv::Mat aligned;
    cv::warpAffine(frameMat, aligned, transform, frameMat.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    return aligned;
}

cv::Mat CameraFrameAligner::alignWithPhaseCorrelation(const cv::Mat& referenceFrame, const cv::Mat& targetFrame, CameraPipelineFrame& frame, cv::Mat *appliedTransform, bool deferWarp)
{
    const cv::Mat referenceGray = frameToAlignmentGray(referenceFrame);
    const cv::Mat targetGray = frameToAlignmentGray(targetFrame);

    cv::Mat referenceFloat;
    cv::Mat targetFloat;
    referenceGray.convertTo(referenceFloat, CV_32F);
    targetGray.convertTo(targetFloat, CV_32F);

    double response = 0.0;
    const cv::Point2d shift = cv::phaseCorrelate(targetFloat, referenceFloat, cv::noArray(), &response);
    const double shiftPixels = std::hypot(shift.x, shift.y);
    const double maxReasonableShift = 0.25 * static_cast<double>(std::min(targetFrame.cols, targetFrame.rows));
    frame.m_stack.m_alignmentResponse = static_cast<float>(response);
    frame.m_stack.m_alignmentShiftPixels = static_cast<float>(shiftPixels);

    if (!std::isfinite(response) || !std::isfinite(shiftPixels))
    {
        frame.m_stack.m_alignmentAccepted = false;
        frame.m_stack.m_alignmentRejectReason = QStringLiteral("phase correlation returned invalid metrics");
        return targetFrame.clone();
    }

    if (response < 0.08)
    {
        frame.m_stack.m_alignmentAccepted = false;
        frame.m_stack.m_alignmentRejectReason = QStringLiteral("phase correlation response too low (%1)").arg(response, 0, 'f', 3);
        return targetFrame.clone();
    }

    if (shiftPixels > maxReasonableShift)
    {
        frame.m_stack.m_alignmentAccepted = false;
        frame.m_stack.m_alignmentRejectReason = QStringLiteral("alignment shift too large (%1 px)").arg(shiftPixels, 0, 'f', 1);
        return targetFrame.clone();
    }

    cv::Mat transform = (cv::Mat_<double>(2, 3) << 1.0, 0.0, shift.x, 0.0, 1.0, shift.y);
    if (appliedTransform) {
        *appliedTransform = transform.clone();
    }
    if (deferWarp) {
        return targetFrame.clone();
    }
    return warpFrameAffine(targetFrame, transform);
}

std::vector<cv::Point2f> CameraFrameAligner::detectStarCentroids(const cv::Mat& grayFrame) const
{
    std::vector<cv::Point2f> stars;
    if (grayFrame.empty()) {
        return stars;
    }

    cv::Mat gray8;
    if (grayFrame.depth() == CV_8U) {
        gray8 = grayFrame;
    } else if (grayFrame.depth() == CV_16U) {
        grayFrame.convertTo(gray8, CV_8U, 255.0 / 65535.0);
    } else {
        grayFrame.convertTo(gray8, CV_8U);
    }

    cv::Mat blurred;
    cv::GaussianBlur(gray8, blurred, cv::Size(0, 0), 1.0);

    cv::Mat background;
    cv::GaussianBlur(gray8, background, cv::Size(0, 0), 10.0);

    cv::Mat blurredFloat;
    cv::Mat backgroundFloat;
    blurred.convertTo(blurredFloat, CV_32F);
    background.convertTo(backgroundFloat, CV_32F);

    cv::Mat enhancedFloat;
    cv::subtract(blurredFloat, backgroundFloat, enhancedFloat);
    cv::max(enhancedFloat, cv::Scalar::all(0.0), enhancedFloat);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(enhancedFloat, mean, stddev);

    double maxValue = 0.0;
    cv::minMaxLoc(enhancedFloat, nullptr, &maxValue);
    const double thresholdValue = std::max(mean[0] + (2.0 * stddev[0]), maxValue * 0.18);
    if (thresholdValue <= 0.0) {
        return stars;
    }

    cv::Mat binary;
    cv::threshold(enhancedFloat, binary, thresholdValue, 255.0, cv::THRESH_BINARY);
    binary.convertTo(binary, CV_8U);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int componentCount = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);

    struct StarCandidate {
        cv::Point2f centroid;
        double brightness;
    };

    std::vector<StarCandidate> candidates;
    for (int component = 1; component < componentCount; ++component)
    {
        const int area = stats.at<int>(component, cv::CC_STAT_AREA);
        if (area < 1 || area > 500) {
            continue;
        }

        const int left = stats.at<int>(component, cv::CC_STAT_LEFT);
        const int top = stats.at<int>(component, cv::CC_STAT_TOP);
        const int width = stats.at<int>(component, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(component, cv::CC_STAT_HEIGHT);
        const cv::Rect roi(left, top, width, height);

        cv::Mat componentMask = (labels(roi) == component);
        cv::Mat weightedComponent = cv::Mat::zeros(roi.size(), CV_32F);
        enhancedFloat(roi).copyTo(weightedComponent, componentMask);
        const cv::Moments moments = cv::moments(weightedComponent, false);
        if (moments.m00 <= 0.0) {
            continue;
        }

        const double brightness = cv::sum(weightedComponent)[0];
        candidates.push_back({
            cv::Point2f(static_cast<float>(left + (moments.m10 / moments.m00)),
                        static_cast<float>(top + (moments.m01 / moments.m00))),
            brightness
        });
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const StarCandidate& lhs, const StarCandidate& rhs) { return lhs.brightness > rhs.brightness; });

    const size_t maxStars = std::min<size_t>(candidates.size(), 64);
    stars.reserve(maxStars);
    for (size_t i = 0; i < maxStars; ++i) {
        stars.push_back(candidates[i].centroid);
    }

    return stars;
}

cv::Mat CameraFrameAligner::alignWithStarCentroids(const cv::Mat& referenceFrame, const cv::Mat& targetFrame, CameraPipelineFrame& frame, cv::Mat *appliedTransform, bool deferWarp)
{
    const cv::Mat referenceGray = frameToAlignmentGray(referenceFrame);
    const cv::Mat targetGray = frameToAlignmentGray(targetFrame);

    const std::vector<cv::Point2f> referenceStars = detectStarCentroids(referenceGray);
    const std::vector<cv::Point2f> targetStars = detectStarCentroids(targetGray);
    constexpr size_t minTentativeMatches = 4;
    constexpr int minInlierMatches = 4;
    constexpr double minInlierRatio = 0.50;
    constexpr double maxInlierRmsPixels = 3.0;
    constexpr double maxInlierResidualPixels = 8.0;

    if (referenceStars.size() < minTentativeMatches || targetStars.size() < minTentativeMatches) {
        qDebug() << "CameraFrameAligner: star alignment fallback: insufficient centroids"
                 << "reference" << referenceStars.size()
                 << "target" << targetStars.size();
        return alignWithPhaseCorrelation(referenceFrame, targetFrame, frame, appliedTransform, deferWarp);
    }

    cv::Mat referenceFloat;
    cv::Mat targetFloat;
    referenceGray.convertTo(referenceFloat, CV_32F);
    targetGray.convertTo(targetFloat, CV_32F);
    double response = 0.0;
    const cv::Point2d shift = cv::phaseCorrelate(targetFloat, referenceFloat, cv::noArray(), &response);
    frame.m_stack.m_alignmentResponse = static_cast<float>(response);

    constexpr float maxMatchDistance = 12.0f;
    constexpr size_t maxSeedStars = 12;
    constexpr size_t maxPairSeedStars = 8;

    auto makeTranslationTransform = [](const cv::Point2f& candidateShift) -> cv::Mat
    {
        cv::Mat transform = (cv::Mat_<double>(2, 3) << 1.0, 0.0, candidateShift.x, 0.0, 1.0, candidateShift.y);
        return transform;
    };

    auto makePairSimilarityTransform = [](const cv::Point2f& referenceA,
                                           const cv::Point2f& referenceB,
                                           const cv::Point2f& targetA,
                                           const cv::Point2f& targetB) -> cv::Mat
    {
        const double referenceDx = static_cast<double>(referenceB.x - referenceA.x);
        const double referenceDy = static_cast<double>(referenceB.y - referenceA.y);
        const double targetDx = static_cast<double>(targetB.x - targetA.x);
        const double targetDy = static_cast<double>(targetB.y - targetA.y);
        const double referenceLength = std::hypot(referenceDx, referenceDy);
        const double targetLength = std::hypot(targetDx, targetDy);
        if ((referenceLength < 8.0) || (targetLength < 8.0)) {
            return cv::Mat();
        }

        const double scale = referenceLength / targetLength;
        if ((scale < 0.90) || (scale > 1.10)) {
            return cv::Mat();
        }

        const double angle = std::atan2(referenceDy, referenceDx) - std::atan2(targetDy, targetDx);
        const double a = scale * std::cos(angle);
        const double b = scale * std::sin(angle);
        const double tx = static_cast<double>(referenceA.x) - (a * static_cast<double>(targetA.x)) + (b * static_cast<double>(targetA.y));
        const double ty = static_cast<double>(referenceA.y) - (b * static_cast<double>(targetA.x)) - (a * static_cast<double>(targetA.y));

        return (cv::Mat_<double>(2, 3) << a, -b, tx, b, a, ty);
    };

    auto isValidSeedTransform = [](const cv::Mat& candidateTransform)
    {
        return !candidateTransform.empty()
            && (candidateTransform.rows == 2)
            && (candidateTransform.cols == 3)
            && (candidateTransform.type() == CV_64F);
    };

    auto makePredictedTransform = [&](const cv::Mat& previousTransform, const cv::Mat& lastTransform) -> cv::Mat
    {
        if (!isValidSeedTransform(previousTransform) || !isValidSeedTransform(lastTransform)) {
            return cv::Mat();
        }

        cv::Mat predicted = lastTransform + (lastTransform - previousTransform);
        if (predicted.type() != CV_64F) {
            predicted.convertTo(predicted, CV_64F);
        }
        return predicted;
    };

    auto transformPoint = [](const cv::Mat& candidateTransform, const cv::Point2f& point)
    {
        return cv::Point2f(
            static_cast<float>((candidateTransform.at<double>(0, 0) * point.x)
                + (candidateTransform.at<double>(0, 1) * point.y)
                + candidateTransform.at<double>(0, 2)),
            static_cast<float>((candidateTransform.at<double>(1, 0) * point.x)
                + (candidateTransform.at<double>(1, 1) * point.y)
                + candidateTransform.at<double>(1, 2)));
    };

    auto composeTransforms = [](const cv::Mat& referenceFromIntermediate, const cv::Mat& intermediateFromTarget) -> cv::Mat
    {
        cv::Mat reference3x3 = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat intermediate3x3 = cv::Mat::eye(3, 3, CV_64F);
        referenceFromIntermediate.copyTo(reference3x3(cv::Rect(0, 0, 3, 2)));
        intermediateFromTarget.copyTo(intermediate3x3(cv::Rect(0, 0, 3, 2)));

        const cv::Mat composed3x3 = reference3x3 * intermediate3x3;
        return composed3x3(cv::Rect(0, 0, 3, 2)).clone();
    };

    auto makeRelativeTransform = [&](const cv::Mat& referenceFromIntermediate, const cv::Mat& referenceFromTarget) -> cv::Mat
    {
        if (!isValidSeedTransform(referenceFromIntermediate) || !isValidSeedTransform(referenceFromTarget)) {
            return cv::Mat();
        }

        cv::Mat intermediateFromReference;
        cv::invertAffineTransform(referenceFromIntermediate, intermediateFromReference);
        if (intermediateFromReference.type() != CV_64F) {
            intermediateFromReference.convertTo(intermediateFromReference, CV_64F);
        }
        return composeTransforms(intermediateFromReference, referenceFromTarget);
    };

    auto matchForTransform = [&](const std::vector<cv::Point2f>& candidateReferenceStars,
                                 const cv::Mat& candidateTransform,
                                 std::vector<cv::Point2f>& candidateTargetPoints,
                                 std::vector<cv::Point2f>& candidateReferencePoints) -> double
    {
        candidateTargetPoints.clear();
        candidateReferencePoints.clear();
        if (!isValidSeedTransform(candidateTransform)) {
            return std::numeric_limits<double>::infinity();
        }

        std::vector<bool> targetUsed(targetStars.size(), false);
        double totalDistance = 0.0;

        for (const cv::Point2f& referenceStar : candidateReferenceStars)
        {
            int bestIndex = -1;
            float bestDistance = maxMatchDistance;

            for (size_t i = 0; i < targetStars.size(); ++i)
            {
                if (targetUsed[i]) {
                    continue;
                }

                const cv::Point2f shiftedTarget = transformPoint(candidateTransform, targetStars[i]);
                const float dx = shiftedTarget.x - referenceStar.x;
                const float dy = shiftedTarget.y - referenceStar.y;
                const float distance = std::sqrt((dx * dx) + (dy * dy));

                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestIndex = static_cast<int>(i);
                }
            }

            if (bestIndex >= 0)
            {
                targetUsed[bestIndex] = true;
                candidateTargetPoints.push_back(targetStars[static_cast<size_t>(bestIndex)]);
                candidateReferencePoints.push_back(referenceStar);
                totalDistance += bestDistance;
            }
        }

        return totalDistance;
    };

    std::vector<cv::Point2f> matchedTargetPoints;
    std::vector<cv::Point2f> matchedReferencePoints;
    double bestMatchDistance = std::numeric_limits<double>::infinity();
    cv::Point2f bestSeedShift(static_cast<float>(shift.x), static_cast<float>(shift.y));
    QString bestSeedKind = QStringLiteral("phase");

    auto considerTransform = [&](const cv::Mat& candidateTransform, const QString& seedKind)
    {
        if (!isValidSeedTransform(candidateTransform)) {
            return;
        }

        std::vector<cv::Point2f> candidateTargetPoints;
        std::vector<cv::Point2f> candidateReferencePoints;
        const double candidateDistance = matchForTransform(referenceStars, candidateTransform, candidateTargetPoints, candidateReferencePoints);

        if ((candidateTargetPoints.size() > matchedTargetPoints.size())
            || ((candidateTargetPoints.size() == matchedTargetPoints.size()) && (candidateDistance < bestMatchDistance)))
        {
            matchedTargetPoints = std::move(candidateTargetPoints);
            matchedReferencePoints = std::move(candidateReferencePoints);
            bestMatchDistance = candidateDistance;
            bestSeedShift = cv::Point2f(static_cast<float>(candidateTransform.at<double>(0, 2)),
                                        static_cast<float>(candidateTransform.at<double>(1, 2)));
            bestSeedKind = seedKind;
        }
    };

    considerTransform(makeTranslationTransform(cv::Point2f(static_cast<float>(shift.x), static_cast<float>(shift.y))), QStringLiteral("phase"));
    const cv::Mat predictedTransform = makePredictedTransform(m_previousStarAlignmentTransform, m_lastStarAlignmentTransform);
    if (isValidSeedTransform(predictedTransform)) {
        considerTransform(predictedTransform, QStringLiteral("predicted"));
    }
    if (isValidSeedTransform(m_lastStarAlignmentTransform)) {
        considerTransform(m_lastStarAlignmentTransform, QStringLiteral("previous"));
    } else if (!m_lastStarAlignmentTransform.empty()) {
        m_lastStarAlignmentTransform.release();
    }
    if (!m_previousStarAlignmentTransform.empty() && !isValidSeedTransform(m_previousStarAlignmentTransform)) {
        m_previousStarAlignmentTransform.release();
    }

    const size_t referenceSeedCount = std::min(referenceStars.size(), maxSeedStars);
    const size_t targetSeedCount = std::min(targetStars.size(), maxSeedStars);
    for (size_t referenceIndex = 0; referenceIndex < referenceSeedCount; ++referenceIndex)
    {
        for (size_t targetIndex = 0; targetIndex < targetSeedCount; ++targetIndex)
        {
            considerTransform(makeTranslationTransform(
                cv::Point2f(referenceStars[referenceIndex].x - targetStars[targetIndex].x,
                            referenceStars[referenceIndex].y - targetStars[targetIndex].y)),
                QStringLiteral("pair"));
        }
    }

    if (matchedTargetPoints.size() < minTentativeMatches)
    {
        if (isValidSeedTransform(m_lastStarAlignmentTransform)
            && (m_lastStarAlignmentTargetStars.size() >= minTentativeMatches))
        {
            std::vector<cv::Point2f> rollingReferenceStars;
            rollingReferenceStars.reserve(m_lastStarAlignmentTargetStars.size());
            for (const cv::Point2f& previousTargetStar : m_lastStarAlignmentTargetStars) {
                rollingReferenceStars.push_back(transformPoint(m_lastStarAlignmentTransform, previousTargetStar));
            }

            auto considerRollingTransform = [&](const cv::Mat& candidateTransform, const QString& seedKind)
            {
                if (!isValidSeedTransform(candidateTransform)) {
                    return;
                }

                std::vector<cv::Point2f> candidateTargetPoints;
                std::vector<cv::Point2f> candidateReferencePoints;
                const double candidateDistance = matchForTransform(rollingReferenceStars, candidateTransform, candidateTargetPoints, candidateReferencePoints);

                if ((candidateTargetPoints.size() > matchedTargetPoints.size())
                    || ((candidateTargetPoints.size() == matchedTargetPoints.size()) && (candidateDistance < bestMatchDistance)))
                {
                    matchedTargetPoints = std::move(candidateTargetPoints);
                    matchedReferencePoints = std::move(candidateReferencePoints);
                    bestMatchDistance = candidateDistance;
                    bestSeedShift = cv::Point2f(static_cast<float>(candidateTransform.at<double>(0, 2)),
                                                static_cast<float>(candidateTransform.at<double>(1, 2)));
                    bestSeedKind = seedKind;
                }
            };

            considerRollingTransform(predictedTransform, QStringLiteral("predicted-rolling"));
            considerRollingTransform(m_lastStarAlignmentTransform, QStringLiteral("previous-rolling"));

            const size_t rollingReferenceSeedCount = std::min(rollingReferenceStars.size(), maxSeedStars);
            for (size_t referenceIndex = 0; referenceIndex < rollingReferenceSeedCount; ++referenceIndex)
            {
                for (size_t targetIndex = 0; targetIndex < targetSeedCount; ++targetIndex)
                {
                    considerRollingTransform(makeTranslationTransform(
                        cv::Point2f(rollingReferenceStars[referenceIndex].x - targetStars[targetIndex].x,
                                    rollingReferenceStars[referenceIndex].y - targetStars[targetIndex].y)),
                        QStringLiteral("pair-rolling"));
                }
            }

        }

        if (matchedTargetPoints.size() < minTentativeMatches)
        {
            for (const StarAlignmentReference& rollingReference : m_recentStarAlignmentReferences)
            {
                if (!isValidSeedTransform(rollingReference.transform)
                    || (rollingReference.targetStars.size() < minTentativeMatches))
                {
                    continue;
                }

                std::vector<cv::Point2f> rollingTargetPoints;
                std::vector<cv::Point2f> rollingPreviousPoints;
                double bestRollingRawDistance = std::numeric_limits<double>::infinity();
                QString bestRollingRawSeedKind;

                auto considerRawRollingTransform = [&](const cv::Mat& currentToPreviousTransform, const QString& seedKind)
                {
                    if (!isValidSeedTransform(currentToPreviousTransform)) {
                        return;
                    }

                    std::vector<cv::Point2f> candidateTargetPoints;
                    std::vector<cv::Point2f> candidatePreviousPoints;
                    const double candidateDistance = matchForTransform(rollingReference.targetStars, currentToPreviousTransform, candidateTargetPoints, candidatePreviousPoints);

                    if ((candidateTargetPoints.size() > rollingTargetPoints.size())
                        || ((candidateTargetPoints.size() == rollingTargetPoints.size()) && (candidateDistance < bestRollingRawDistance)))
                    {
                        rollingTargetPoints = std::move(candidateTargetPoints);
                        rollingPreviousPoints = std::move(candidatePreviousPoints);
                        bestRollingRawDistance = candidateDistance;
                        bestRollingRawSeedKind = seedKind;
                    }
                };

                considerRawRollingTransform(makeRelativeTransform(rollingReference.transform, predictedTransform), QStringLiteral("predicted-affine-raw-rolling"));
                considerRawRollingTransform(makeRelativeTransform(rollingReference.transform, m_lastStarAlignmentTransform), QStringLiteral("previous-affine-raw-rolling"));

                const size_t previousSeedCount = std::min(rollingReference.targetStars.size(), maxSeedStars);
                for (size_t previousIndex = 0; previousIndex < previousSeedCount; ++previousIndex)
                {
                    for (size_t targetIndex = 0; targetIndex < targetSeedCount; ++targetIndex)
                    {
                        considerRawRollingTransform(makeTranslationTransform(
                            cv::Point2f(rollingReference.targetStars[previousIndex].x - targetStars[targetIndex].x,
                                        rollingReference.targetStars[previousIndex].y - targetStars[targetIndex].y)),
                            QStringLiteral("pair-raw-rolling"));
                    }
                }

                const size_t previousPairSeedCount = std::min(rollingReference.targetStars.size(), maxPairSeedStars);
                const size_t targetPairSeedCount = std::min(targetStars.size(), maxPairSeedStars);
                for (size_t previousA = 0; previousA < previousPairSeedCount; ++previousA)
                {
                    for (size_t previousB = previousA + 1; previousB < previousPairSeedCount; ++previousB)
                    {
                        for (size_t targetA = 0; targetA < targetPairSeedCount; ++targetA)
                        {
                            for (size_t targetB = targetA + 1; targetB < targetPairSeedCount; ++targetB)
                            {
                                considerRawRollingTransform(makePairSimilarityTransform(
                                    rollingReference.targetStars[previousA],
                                    rollingReference.targetStars[previousB],
                                    targetStars[targetA],
                                    targetStars[targetB]),
                                    QStringLiteral("pair-affine-raw-rolling"));
                                considerRawRollingTransform(makePairSimilarityTransform(
                                    rollingReference.targetStars[previousA],
                                    rollingReference.targetStars[previousB],
                                    targetStars[targetB],
                                    targetStars[targetA]),
                                    QStringLiteral("pair-affine-raw-rolling"));
                            }
                        }
                    }
                }

                if (rollingTargetPoints.size() >= minTentativeMatches)
                {
                    cv::Mat rollingInliers;
                    const cv::Mat currentToPreviousTransform = cv::estimateAffinePartial2D(
                        rollingTargetPoints, rollingPreviousPoints, rollingInliers, cv::RANSAC, 3.0);

                    if (isValidSeedTransform(currentToPreviousTransform))
                    {
                        matchedTargetPoints = rollingTargetPoints;
                        matchedReferencePoints.clear();
                        matchedReferencePoints.reserve(rollingPreviousPoints.size());
                        for (const cv::Point2f& previousPoint : rollingPreviousPoints) {
                            matchedReferencePoints.push_back(transformPoint(rollingReference.transform, previousPoint));
                        }

                        const cv::Mat composedTransform = composeTransforms(rollingReference.transform, currentToPreviousTransform);
                        bestSeedShift = cv::Point2f(static_cast<float>(composedTransform.at<double>(0, 2)),
                                                    static_cast<float>(composedTransform.at<double>(1, 2)));
                        bestSeedKind = bestRollingRawSeedKind;
                        break;
                    }
                }
            }
        }

        if (matchedTargetPoints.size() < minTentativeMatches)
        {
            qDebug() << "CameraFrameAligner: star alignment fallback: insufficient tentative matches"
                     << "reference" << referenceStars.size()
                     << "target" << targetStars.size()
                     << "matches" << matchedTargetPoints.size()
                     << "phaseResponse" << response
                     << "phaseShift" << shift.x << shift.y
                     << "bestSeedShift" << bestSeedShift.x << bestSeedShift.y
                     << "bestSeedKind" << bestSeedKind;
            return alignWithPhaseCorrelation(referenceFrame, targetFrame, frame, appliedTransform, deferWarp);
        }
    }

    cv::Mat inliers;
    cv::Mat transform = cv::estimateAffinePartial2D(
        matchedTargetPoints, matchedReferencePoints, inliers, cv::RANSAC, 3.0);

    if (transform.empty())
    {
        qDebug() << "CameraFrameAligner: star alignment fallback: affine estimate failed"
                 << "reference" << referenceStars.size()
                 << "target" << targetStars.size()
                 << "matches" << matchedTargetPoints.size()
                 << "phaseResponse" << response;
        return alignWithPhaseCorrelation(referenceFrame, targetFrame, frame, appliedTransform, deferWarp);
    }

    const int inlierCount = inliers.empty() ? 0 : cv::countNonZero(inliers);
    frame.m_stack.m_alignmentMatchedStars = inlierCount;
    const double inlierRatio = matchedTargetPoints.empty()
        ? 0.0
        : static_cast<double>(inlierCount) / static_cast<double>(matchedTargetPoints.size());

    double residualSquaredSum = 0.0;
    double maxResidual = 0.0;
    for (int i = 0; i < static_cast<int>(matchedTargetPoints.size()); ++i)
    {
        if (inliers.empty() || (inliers.at<uchar>(i, 0) == 0)) {
            continue;
        }

        const cv::Point2f& targetPoint = matchedTargetPoints[static_cast<size_t>(i)];
        const cv::Point2f& referencePoint = matchedReferencePoints[static_cast<size_t>(i)];
        const double alignedX = (transform.at<double>(0, 0) * targetPoint.x)
            + (transform.at<double>(0, 1) * targetPoint.y)
            + transform.at<double>(0, 2);
        const double alignedY = (transform.at<double>(1, 0) * targetPoint.x)
            + (transform.at<double>(1, 1) * targetPoint.y)
            + transform.at<double>(1, 2);
        const double residual = std::hypot(alignedX - referencePoint.x, alignedY - referencePoint.y);
        residualSquaredSum += residual * residual;
        maxResidual = std::max(maxResidual, residual);
    }

    const double residualRms = inlierCount > 0
        ? std::sqrt(residualSquaredSum / static_cast<double>(inlierCount))
        : std::numeric_limits<double>::infinity();

    const double shiftPixels = std::hypot(transform.at<double>(0, 2), transform.at<double>(1, 2));
    const double scaleX = std::hypot(transform.at<double>(0, 0), transform.at<double>(1, 0));
    const double scaleY = std::hypot(transform.at<double>(0, 1), transform.at<double>(1, 1));
    const double scale = (scaleX + scaleY) * 0.5;
    const double maxReasonableShift = 0.25 * static_cast<double>(std::min(targetFrame.cols, targetFrame.rows));
    frame.m_stack.m_alignmentShiftPixels = static_cast<float>(shiftPixels);

    const double rotationDegrees = std::atan2(transform.at<double>(1, 0), transform.at<double>(0, 0)) * 180.0 / CV_PI;
    auto logStarAlignment = [&](const char *status)
    {
        qDebug() << "CameraFrameAligner: star alignment" << status
                 << "reference" << referenceStars.size()
                 << "target" << targetStars.size()
                 << "matches" << matchedTargetPoints.size()
                 << "inliers" << inlierCount
                 << "inlierRatio" << inlierRatio
                 << "rms" << residualRms
                 << "maxResidual" << maxResidual
                 << "shift" << transform.at<double>(0, 2) << transform.at<double>(1, 2)
                 << "seedShift" << bestSeedShift.x << bestSeedShift.y
                 << "seedKind" << bestSeedKind
                 << "rotation" << rotationDegrees
                 << "scale" << scale
                 << "phaseResponse" << response;
    };

    if (!std::isfinite(shiftPixels) || !std::isfinite(scale))
    {
        logStarAlignment("fallback-invalid-transform");
        return alignWithPhaseCorrelation(referenceFrame, targetFrame, frame, appliedTransform, deferWarp);
    }

    if (shiftPixels > maxReasonableShift)
    {
        logStarAlignment("fallback-shift-too-large");
        return alignWithPhaseCorrelation(referenceFrame, targetFrame, frame, appliedTransform, deferWarp);
    }

    if ((scale < 0.80) || (scale > 1.25))
    {
        logStarAlignment("fallback-scale-out-of-range");
        return alignWithPhaseCorrelation(referenceFrame, targetFrame, frame, appliedTransform, deferWarp);
    }

    if ((inlierCount < minInlierMatches)
        || (inlierRatio < minInlierRatio)
        || !std::isfinite(residualRms)
        || (residualRms > maxInlierRmsPixels)
        || (maxResidual > maxInlierResidualPixels))
    {
        logStarAlignment("fallback-poor-fit");
        return alignWithPhaseCorrelation(referenceFrame, targetFrame, frame, appliedTransform, deferWarp);
    }

    logStarAlignment("accepted");
    if (isValidSeedTransform(m_lastStarAlignmentTransform)) {
        m_previousStarAlignmentTransform = m_lastStarAlignmentTransform.clone();
    } else {
        m_previousStarAlignmentTransform.release();
    }
    m_lastStarAlignmentTransform = transform.clone();
    m_lastStarAlignmentTargetStars = targetStars;
    m_recentStarAlignmentReferences.push_front({m_lastStarAlignmentTransform.clone(), m_lastStarAlignmentTargetStars});
    constexpr size_t maxRecentStarAlignmentReferences = 6;
    while (m_recentStarAlignmentReferences.size() > maxRecentStarAlignmentReferences) {
        m_recentStarAlignmentReferences.pop_back();
    }

    if (appliedTransform) {
        *appliedTransform = transform.clone();
    }
    if (deferWarp) {
        return targetFrame.clone();
    }
    return warpFrameAffine(targetFrame, transform);
}
