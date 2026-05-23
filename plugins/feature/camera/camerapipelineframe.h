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
    bool m_plateSolved = false;
    int m_plateSolvedMatches = 0;
    int m_plateSolveDetectedStarsConsidered = 0;
    int m_plateSolveCatalogStarsLoaded = 0;
    int m_plateSolveCatalogCandidateStars = 0;
    int m_plateSolveOutlierStars = 0;
    float m_plateSolveRmsError = 0.0f;
    float m_plateSolveMaxError = 0.0f;
    float m_plateSolveAzimuth = 0.0f;
    float m_plateSolveElevation = 0.0f;
    float m_plateSolveRoll = 0.0f;
    float m_plateSolveFov = 0.0f;
    float m_plateSolveCenterOffsetX = 0.0f;
    float m_plateSolveCenterOffsetY = 0.0f;
    float m_plateSolveDistortionK1 = 0.0f;
    QString m_plateSolveCatalogSource;
    QString m_plateSolveFailureReason;
    QString m_plateSolveMatchSummary;
    int m_plateSolveRequiredMatches = 0;
    bool m_saveCurrentImage = false;
    int m_stackCount = 1;
    int m_stackQueuedCount = 0;
    int m_stackDroppedCount = 0;
    int m_stackRejectedCount = 0;
    bool m_stackAlignmentAttempted = false;
    bool m_stackAlignmentAccepted = true;
    float m_stackAlignmentResponse = 0.0f;
    float m_stackAlignmentShiftPixels = 0.0f;
    int m_stackAlignmentMatchedStars = 0;
    QString m_stackAlignmentRejectReason;
    QString m_stackRejectReason;
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
