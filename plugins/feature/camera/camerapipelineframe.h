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

#include <QDateTime>
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

struct CameraPipelineDetection
{
    QRect m_box;
    QString m_label;
    float m_score;
};

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
    QString m_catalogSpectralType;
    bool m_solved = false;
};

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
    CameraHistogramData m_histogramData;
    QDateTime m_captureDateTime;
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
            && (m_cudaBgrImage.type() == CV_8UC3);
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

using CameraPipelineFramePtr = QSharedPointer<CameraPipelineFrame>;

#endif // INCLUDE_FEATURE_CAMERAPIPELINEFRAME_H_
