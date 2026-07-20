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
#include <QTransform>
#include <QVector>

#include <opencv2/imgproc.hpp>

#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION) || defined(CAMERA_OPENCV_CUDA_CLOUD_DETECTION)
#include <opencv2/core/cuda.hpp>
#endif

#include "cameraopticalspectrum.h"

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

/** Camera pointing sampled when a frame enters the capture pipeline. */
struct CameraPipelineDirection
{
    float m_azimuth = 0.0f;
    float m_elevation = 0.0f;
    float m_roll = 0.0f;
    bool m_valid = false;
};

/** Raw mapped bytes and decoded measurements for a radiometric UVC frame. */
struct CameraPipelineThermalRawFrame
{
    QByteArray m_bytes;
    int m_width = 0;
    int m_height = 0;
    int m_bytesPerLine = 0;
    int m_pixelFormat = -1;
    QString m_pixelFormatName;
};

struct CameraPipelineThermal
{
    CameraPipelineThermalRawFrame m_rawFrame;
    cv::Mat m_temperatureC; // CV_32FC1, sensor coordinates
    QString m_decoderName;
    QString m_status;
    QPoint m_markerPosition;
    QPoint m_minimumPosition;
    QPoint m_maximumPosition;
    float m_markerTemperatureC = 0.0f;
    float m_minimumC = 0.0f;
    float m_maximumC = 0.0f;
    float m_meanC = 0.0f;
    bool m_valid = false;
    bool m_calibrationFrame = false;
    QTransform m_sensorToImage;

    void clearDecoded()
    {
        m_temperatureC.release();
        m_decoderName.clear();
        m_status.clear();
        m_markerPosition = QPoint();
        m_minimumPosition = QPoint();
        m_maximumPosition = QPoint();
        m_markerTemperatureC = 0.0f;
        m_minimumC = 0.0f;
        m_maximumC = 0.0f;
        m_meanC = 0.0f;
        m_valid = false;
        m_calibrationFrame = false;
        m_sensorToImage.reset();
    }
};

/**
 * \brief Photometry result for one detected meteor object.
 *
 * Produced from a YOLO "meteor" detection by measuring the calibrated frame pixels inside
 * the detection box, subtracting a robust local background and calibrating against solved
 * reference stars from the same frame when available.
 */
struct CameraPipelineMeteorPhotometry
{
    QRect m_box;
    QPointF m_center;
    double m_flux = 0.0;
    double m_background = 0.0;
    double m_backgroundSigma = 0.0;
    double m_magnitude = 0.0;
    double m_magnitudeError = 0.0;
    double m_zeroPoint = 0.0;
    double m_zeroPointRms = 0.0;
    int m_referenceStars = 0;
    int m_signalPixels = 0;
    bool m_validMagnitude = false;
    bool m_saturated = false;
    QString m_failureReason;
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
    // True when the pose was recovered on the horizontally-mirrored (handedness-flipped)
    // detection set — an up-looking all-sky fisheye images the sky reflected. The az/el/roll
    // pose is expressed in the mirrored frame, so any projector built to overlay this pose onto
    // the ORIGINAL image must set SkyProjector::mirrorX. Detection positions on the frame are
    // already in original-image coordinates.
    bool m_mirrored = false;
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
 * \brief Cloud segmentation result for a frame: per-pixel mask and coverage statistics.
 *
 * Produced by CameraCloudDetector and attached to CameraPipelineFrame::m_cloud. The mask is a
 * CV_8UC1 image in the (possibly downscaled) detection-ROI working space where 255 = cloud and
 * 0 = clear sky or excluded region; m_roi gives the full-image rectangle the mask covers.
 * Downstream consumers (star/motion detectors, post-processor) use isCloudAtImagePoint() or map
 * their own rectangles into mask space; m_coveragePercent is the percentage of evaluated
 * (non-excluded) sky classified as cloud.
 *
 * \note The mask may be shared between frames via cv::Mat refcounting when the detector reuses
 *       a cached result on intermediate frames, so consumers must treat it as read-only.
 * \note Check m_valid before using any field; frames that bypass the cloud detector stage (or
 *       frames processed while cloud detection is disabled) carry an invalid result.
 */
struct CameraPipelineCloud
{
    cv::Mat m_mask;                  ///< CV_8UC1, 255 = cloud, 0 = clear/excluded; detection-ROI working space
    cv::Rect m_roi;                  ///< Full-image detection ROI the mask covers
    float m_coveragePercent = 0.0f;  ///< Percent of evaluated (non-excluded) sky classified as cloud
    bool m_night = false;            ///< True when the night-sky path classified this frame
    bool m_valid = false;

    /// Returns true when the full-image point falls inside a cloud-classified mask cell.
    bool isCloudAtImagePoint(const QPointF& point) const
    {
        if (!m_valid || m_mask.empty() || (m_roi.width <= 0) || (m_roi.height <= 0)) {
            return false;
        }

        const int col = static_cast<int>((point.x() - m_roi.x) * m_mask.cols / m_roi.width);
        const int row = static_cast<int>((point.y() - m_roi.y) * m_mask.rows / m_roi.height);
        if ((col < 0) || (col >= m_mask.cols) || (row < 0) || (row >= m_mask.rows)) {
            return false;
        }

        return m_mask.at<uchar>(row, col) != 0;
    }
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
 * \brief Maps between the optical image coordinates and the current frame image.
 *
 * Stages such as output scaling can place the real image inside a larger canvas. Later
 * stages still need the original optical coordinate system for plate solving and sky
 * projection, while overlays need to be drawn in the displayed frame coordinates.
 */
struct CameraPipelineImageTransform
{
    QSize m_opticalSize;
    QTransform m_opticalToImage;
    bool m_enabled = false;

    void clear()
    {
        m_opticalSize = QSize();
        m_opticalToImage.reset();
        m_enabled = false;
    }

    bool isValid() const
    {
        return m_enabled
            && !m_opticalSize.isEmpty()
            && m_opticalToImage.isAffine()
            && m_opticalToImage.isInvertible();
    }

    QSize opticalSize(const QSize& imageSize) const
    {
        return isValid() ? m_opticalSize : imageSize;
    }

    void setScaled(const QSize& opticalSize, const QRect& contentRect)
    {
        clear();
        if (opticalSize.isEmpty() || contentRect.isEmpty()) {
            return;
        }

        m_opticalSize = opticalSize;
        m_opticalToImage.translate(contentRect.x(), contentRect.y());
        m_opticalToImage.scale(
            static_cast<double>(contentRect.width()) / static_cast<double>(opticalSize.width()),
            static_cast<double>(contentRect.height()) / static_cast<double>(opticalSize.height()));
        m_enabled = true;
    }

    void applyImageTransform(const QTransform& oldImageToNewImage)
    {
        if (!isValid()) {
            return;
        }

        m_opticalToImage = oldImageToNewImage * m_opticalToImage;
    }

    void applyFlip(bool flipX, bool flipY, const QSize& imageSize)
    {
        if (!isValid() || imageSize.isEmpty() || (!flipX && !flipY)) {
            return;
        }

        const QTransform transform(
            flipX ? -1.0 : 1.0,
            0.0,
            0.0,
            0.0,
            flipY ? -1.0 : 1.0,
            0.0,
            flipX ? imageSize.width() - 1 : 0.0,
            flipY ? imageSize.height() - 1 : 0.0,
            1.0);
        applyImageTransform(transform);
    }

    void applyRotation(int degrees, const QSize& imageSize)
    {
        if (!isValid() || imageSize.isEmpty()) {
            return;
        }

        QTransform transform;
        switch (degrees)
        {
        case 90:
            transform = QTransform(0.0, 1.0, 0.0, -1.0, 0.0, 0.0, imageSize.height() - 1, 0.0, 1.0);
            break;
        case 180:
            transform = QTransform(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, imageSize.width() - 1, imageSize.height() - 1, 1.0);
            break;
        case 270:
            transform = QTransform(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, imageSize.width() - 1, 1.0);
            break;
        case 0:
        default:
            return;
        }
        applyImageTransform(transform);
    }

    QPointF mapOpticalToImage(const QPointF& point) const
    {
        return isValid() ? m_opticalToImage.map(point) : point;
    }

    QPointF mapImageToOptical(const QPointF& point) const
    {
        if (!isValid()) {
            return point;
        }

        bool invertible = false;
        const QTransform imageToOptical = m_opticalToImage.inverted(&invertible);
        return invertible ? imageToOptical.map(point) : point;
    }
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
    CameraOpticalSpectrumData m_opticalSpectrumData;
    QDateTime m_captureDateTime;
    CameraPipelineDirection m_captureDirection;
    CameraPipelineThermal m_thermal;
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
    CameraPipelineCloud m_cloud;
    QVector<CameraPipelineDetection> m_detections;
    QVector<CameraPipelineMeteorPhotometry> m_meteorPhotometry;
    QVector<CameraPipelineStarDetection> m_starDetections;
    CameraPipelinePlateSolve m_plateSolve;
    CameraPipelineImageTransform m_imageTransform;
    bool m_saveCurrentImage = false;
    CameraPipelineStacking m_stack;
    BayerPattern m_bayerPattern = BayerNone;
    BayerPattern m_rawInputBayerPattern = BayerNone;

#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION) || defined(CAMERA_OPENCV_CUDA_CLOUD_DETECTION)
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
#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION) || defined(CAMERA_OPENCV_CUDA_CLOUD_DETECTION)
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
#if defined(CAMERA_OPENCV_CUDA_IMAGE_PROCESSING) || defined(CAMERA_OPENCV_CUDA_DETECTION) || defined(CAMERA_OPENCV_CUDA_MOTION_DETECTION) || defined(CAMERA_OPENCV_CUDA_CLOUD_DETECTION)
        if (hasCudaBgrImage()) {
            return QSize(m_cudaBgrImage.cols, m_cudaBgrImage.rows);
        }
        if (hasCudaGrayImage()) {
            return QSize(m_cudaGrayImage.cols, m_cudaGrayImage.rows);
        }
#endif
        return QSize();
    }

    QSize opticalImageSize() const
    {
        return m_imageTransform.opticalSize(imageSize());
    }

    QPointF mapOpticalToImage(const QPointF& point) const
    {
        return m_imageTransform.mapOpticalToImage(point);
    }

    QPointF mapImageToOptical(const QPointF& point) const
    {
        return m_imageTransform.mapImageToOptical(point);
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
