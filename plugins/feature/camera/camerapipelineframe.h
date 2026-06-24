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

#ifndef INCLUDE_FEATURE_CAMERAPIPELINEFRAME_H_
#define INCLUDE_FEATURE_CAMERAPIPELINEFRAME_H_

#include <cstring>
#include <limits>

#include <QDateTime>
#include <QByteArray>
#include <QImage>
#include <QPointF>
#include <QRect>
#include <QSharedPointer>
#include <QSize>
#include <QString>
#include <QVector>

#include <opencv2/imgproc.hpp>

#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
#include <opencv2/core/cuda.hpp>
#endif

/**
 * \brief A single object detection (bounding box, class label, confidence).
 *
 * Produced by CameraObjectDetector (YOLO/TensorRT backend) and stored in
 * CameraPipelineFrame::m_detections as it flows down the pipeline. The box is in
 * full-frame image pixel coordinates; the label is the resolved class name and
 * the score is the detector confidence.
 */
struct CameraPipelineDetection
{
    QRect m_box;
    QString m_label;
    float m_score;
};

/**
 * \brief A named target being tracked across frames, with optional sky coordinates.
 *
 * Lightweight value type used to carry a tracked object's identity, label, image
 * position and (when a plate solve is available) its derived azimuth/elevation. Pure
 * data with no behaviour; copied freely through the pipeline and to the GUI.
 */
struct CameraPipelineTrackedObject
{
    QString m_name;
    QString m_label;
    QPointF m_position;
    double m_azimuth = 0.0;
    double m_elevation = 0.0;
};

/**
 * \brief Per-channel RGB histogram bins for a frame.
 *
 * Holds the red/green/blue bin counts computed during image processing and is
 * attached to CameraPipelineFrame::m_histogramData for display in the GUI.
 *
 * \note isValid() requires non-empty bins with matching sizes across all three
 *       channels; consumers should check it before reading the bins.
 */
struct CameraHistogramData
{
    QVector<float> m_redBins;
    QVector<float> m_greenBins;
    QVector<float> m_blueBins;

    bool isValid() const
    {
        return !m_redBins.isEmpty() && (m_redBins.size() == m_greenBins.size()) && (m_redBins.size() == m_blueBins.size());
    }
};

/**
 * \brief One detected star with its photometric, morphological and catalog-match data.
 *
 * Produced by CameraStarDetector and stored in CameraPipelineFrame::m_starDetections.
 * Carries the measured centroid and shape metrics (flux, SNR, FWHM, roundness, etc.) used
 * to score and filter candidates, plus quality flags (saturation, hot-pixel suspicion). When
 * the plate solver matches the star against a catalog, the catalog fields (label, magnitude,
 * RA/Dec, spectral type), the projected position and m_solved are populated.
 *
 * \note Catalog RA/Dec default to NaN until a match is made; check m_solved before relying
 *       on the catalog fields or m_projectedCenter.
 */
struct CameraPipelineStarDetection
{
    QPointF m_center;
    QPointF m_projectedCenter;
    float m_peakValue;
    float m_radius;
    float m_flux = 0.0f;
    float m_snr = 0.0f;
    float m_fwhm = 0.0f;
    float m_centroidUncertainty = 0.0f;
    float m_qualityScore = 0.0f;
    float m_roundness = 0.0f;
    float m_fillRatio = 0.0f;
    float m_aspectRatio = 0.0f;
    bool m_saturated = false;
    bool m_hotPixelSuspect = false;
    QString m_label;
    float m_matchDistancePixels = 0.0f;
    float m_catalogMagnitude = 0.0f;
    double m_catalogRightAscensionDegrees = std::numeric_limits<double>::quiet_NaN();
    double m_catalogDeclinationDegrees = std::numeric_limits<double>::quiet_NaN();
    QString m_catalogSpectralType;
    bool m_solved = false;
};

/**
 * \brief Result of a plate-solve attempt for a frame: pointing, geometry and diagnostics.
 *
 * Filled in by the plate solver (driven from CameraStarDetector) and attached to
 * CameraPipelineFrame::m_plateSolve. When m_solved is true it gives the derived camera
 * pointing (azimuth/elevation/roll), field of view, optical-centre offset and distortion,
 * along with match-quality counts and RMS/max residuals. When solving fails, m_failureReason
 * explains why; the diagnostic summary strings are intended for the GUI/status display.
 *
 * \note Treat the geometry/pointing fields as meaningful only when m_solved is true.
 */
struct CameraPipelinePlateSolve
{
    bool m_solved = false;
    int m_matchedStars = 0;
    int m_detectedStarsConsidered = 0;
    int m_catalogStarsLoaded = 0;
    int m_catalogCandidateStars = 0;
    int m_outlierStars = 0;
    float m_rmsError = 0.0f;
    float m_maxError = 0.0f;
    float m_azimuth = 0.0f;
    float m_elevation = 0.0f;
    float m_roll = 0.0f;
    float m_fov = 0.0f;
    float m_centerOffsetX = 0.0f;
    float m_centerOffsetY = 0.0f;
    float m_distortionK1 = 0.0f;
    QString m_catalogSource;
    QString m_failureReason;
    QString m_matchSummary;
    QString m_profileSummary;
    int m_requiredMatches = 0;
};

/**
 * \brief Bookkeeping for the frame-stacking stage: counts and alignment outcome.
 *
 * Attached to CameraPipelineFrame::m_stack to describe how the (possibly stacked) frame was
 * produced: how many frames were combined, how many were queued/dropped/rejected, and whether
 * star-based alignment was attempted and accepted. The alignment fields (response, shift,
 * matched stars, reject reason) record why a contributing frame was kept or discarded.
 */
struct CameraPipelineStacking
{
    int m_count = 1;
    int m_queuedCount = 0;
    int m_droppedCount = 0;
    int m_rejectedCount = 0;
    bool m_alignmentAttempted = false;
    bool m_alignmentAccepted = true;
    float m_alignmentResponse = 0.0f;
    float m_alignmentShiftPixels = 0.0f;
    int m_alignmentMatchedStars = 0;
    QString m_alignmentRejectReason;
    QString m_rejectReason;
};

/**
 * \brief The unit of data flowing through the camera processing pipeline.
 *
 * Carries everything about one captured (or played-back) frame as it travels from image
 * processing through stacking, the detection stages (motion/star/object/diff), plate solving
 * and recording. Holds the image(s) in their various forms (processed, unprocessed, raw input,
 * post-processed) plus capture metadata (timestamps, exposure, HDR/playback info, Bayer
 * pattern) and the accumulated detection/solve/stack results. When built with OpenCV CUDA
 * support it also caches GPU-resident BGR/grayscale mats so successive GPU stages can avoid
 * re-uploading; the cache helpers lazily materialise a CPU QImage on demand.
 *
 * \note Frames are passed by CameraPipelineFramePtr (a QSharedPointer) between pipeline stages,
 *       each running on its own QThread. A frame may be referenced concurrently, so stages
 *       generally treat received frames as shared/read-mostly and mutate only their own added
 *       fields; do not assume exclusive ownership unless the refcount guarantees it.
 * \warning The CUDA mat cache (m_cudaBgrImage/m_cudaGrayImage) and the CPU QImage are kept in
 *          sync only via the provided helpers (ensureCpuImageFromCuda, clearCudaCache,
 *          clearCpuImage); clear or rebuild caches through these rather than touching the
 *          members directly.
 */
struct CameraPipelineFrame
{
    enum BayerPattern
    {
        BayerNone = 0,
        BayerRGGB,
        BayerBGGR,
        BayerGRBG,
        BayerGBRG
    };

    QImage m_image;
    QImage m_unprocessedImage;
    QImage m_postProcessedImage;
    QImage m_rawInputImage;
    CameraHistogramData m_histogramData;
    QDateTime m_captureDateTime;
    quint64 m_captureEpoch = 0;
    qint64 m_pipelineInputWallClockMs = 0;
    bool m_manualPreviewFrame = false;
    bool m_playbackActiveFrame = false;
    qint64 m_playbackPositionMs = -1;
    int m_playbackFrameNumber = -1;
    double m_playbackFrameRate = 0.0;
    double m_exposureTimeMs = 0.0;
    int m_hdrExposureIndex = -1;
    int m_hdrExposureCount = 0;
    QVector<QRect> m_motionBoxes;
    QVector<CameraPipelineDetection> m_detections;
    QVector<CameraPipelineStarDetection> m_starDetections;
    CameraPipelinePlateSolve m_plateSolve;
    bool m_saveCurrentImage = false;
    CameraPipelineStacking m_stack;
    BayerPattern m_bayerPattern = BayerNone;
    BayerPattern m_rawInputBayerPattern = BayerNone;

#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
    cv::cuda::GpuMat m_cudaBgrImage;
    cv::cuda::GpuMat m_cudaGrayImage;

    void clearCudaCache()
    {
        m_cudaBgrImage.release();
        m_cudaGrayImage.release();
    }

    bool hasCudaBgrImage() const
    {
        return !m_cudaBgrImage.empty()
            && (m_cudaBgrImage.channels() == 3)
            && ((m_cudaBgrImage.depth() == CV_8U) || (m_cudaBgrImage.depth() == CV_16U));
    }

    bool hasCudaGrayImage() const
    {
        return !m_cudaGrayImage.empty()
            && (m_cudaGrayImage.type() == CV_8UC1);
    }

    bool ensureCpuImageFromCuda()
    {
        if (!m_image.isNull()) {
            return true;
        }

        if (hasCudaBgrImage())
        {
            cv::Mat bgrMat;
            m_cudaBgrImage.download(bgrMat);
            m_image = bgrMatToRgbImage(bgrMat);
            return !m_image.isNull();
        }

        if (hasCudaGrayImage())
        {
            cv::Mat grayMat;
            m_cudaGrayImage.download(grayMat);
            m_image = grayMatToImage(grayMat);
            return !m_image.isNull();
        }

        return false;
    }
#else
    void clearCudaCache() {}

    bool hasCudaBgrImage() const { return false; }
    bool hasCudaGrayImage() const { return false; }

    bool ensureCpuImageFromCuda()
    {
        return !m_image.isNull();
    }
#endif

    bool hasImageData() const
    {
        return !m_image.isNull()
#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
            || hasCudaBgrImage()
            || hasCudaGrayImage()
#endif
            ;
    }

    QSize imageSize() const
    {
        if (!m_image.isNull()) {
            return m_image.size();
        }
#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION)
        if (hasCudaBgrImage()) {
            return QSize(m_cudaBgrImage.cols, m_cudaBgrImage.rows);
        }
        if (hasCudaGrayImage()) {
            return QSize(m_cudaGrayImage.cols, m_cudaGrayImage.rows);
        }
#endif
        return QSize();
    }

    void clearCpuImage()
    {
        m_image = QImage();
    }

    static QImage bgrMatToRgbImage(const cv::Mat& bgrMat)
    {
        if (bgrMat.empty()) {
            return QImage();
        }

        if ((bgrMat.depth() == CV_16U) && (bgrMat.channels() == 3))
        {
            QImage result(bgrMat.cols, bgrMat.rows, QImage::Format_RGBA64);
            for (int y = 0; y < bgrMat.rows; ++y)
            {
                const cv::Vec<uint16_t, 3> *inputLine = bgrMat.ptr<cv::Vec<uint16_t, 3>>(y);
                QRgba64 *outputLine = reinterpret_cast<QRgba64*>(result.scanLine(y));

                for (int x = 0; x < bgrMat.cols; ++x) {
                    outputLine[x] = qRgba64(inputLine[x][2], inputLine[x][1], inputLine[x][0], 65535);
                }
            }
            return result;
        }

        QImage result(bgrMat.cols, bgrMat.rows, QImage::Format_RGB888);
        cv::Mat rgbMat(result.height(), result.width(), CV_8UC3,
                       result.bits(),
                       static_cast<size_t>(result.bytesPerLine()));
        cv::cvtColor(bgrMat, rgbMat, cv::COLOR_BGR2RGB);
        return result;
    }

    static QImage grayMatToImage(const cv::Mat& grayMat)
    {
        if (grayMat.empty()) {
            return QImage();
        }

        QImage result(grayMat.cols, grayMat.rows, QImage::Format_Grayscale8);
        for (int row = 0; row < grayMat.rows; ++row) {
            std::memcpy(result.scanLine(row), grayMat.ptr(row), static_cast<size_t>(grayMat.cols));
        }
        return result;
    }
};

/**
 * \brief Shared-ownership handle to a CameraPipelineFrame passed between pipeline stages.
 *
 * Pipeline stages and message queues hold frames via this QSharedPointer; the frame is
 * destroyed when the last stage/consumer releases it. Passing by this pointer avoids deep
 * copies of the image data as frames are forwarded down the chain.
 */
using CameraPipelineFramePtr = QSharedPointer<CameraPipelineFrame>;

#endif // INCLUDE_FEATURE_CAMERAPIPELINEFRAME_H_
