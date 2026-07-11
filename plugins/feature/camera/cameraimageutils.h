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

#ifndef INCLUDE_FEATURE_CAMERAIMAGEUTILS_H_
#define INCLUDE_FEATURE_CAMERAIMAGEUTILS_H_

#include <QImage>

#include <opencv2/core.hpp>

#include "camerapipelineframe.h"

struct CameraSettings;

/**
 * \brief Stateless helpers for converting between QImage and OpenCV cv::Mat in the camera pipeline.
 *
 * Collects the static conversion utilities shared across the pipeline stages: wrapping an
 * RGB888 QImage as a cv::Mat without copying, converting BGR cv::Mat back to a QImage,
 * producing a working cv::Mat from an arbitrary QImage (tracking whether the input was
 * high bit depth), turning a working cv::Mat back into a QImage, and debayering a raw
 * single-channel cv::Mat for a given Bayer pattern.
 *
 * \note All members are static; the class is never instantiated.
 * \warning wrapRgb888Image() returns a cv::Mat that aliases the QImage's pixel buffer
 *          (no copy). The cv::Mat is only valid while the source QImage outlives it and
 *          is not detached/modified.
 */
class CameraImageUtils
{
public:
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    [[nodiscard]] static QImage convertBgrToRgbImage(const cv::Mat& bgrMat);
    [[nodiscard]] static cv::Mat imageToWorkingMat(const QImage& input, bool *highBitDepthInput = nullptr);
    [[nodiscard]] static QImage workingMatToImage(const cv::Mat& frameMat);
    [[nodiscard]] static int bayerPatternToOpenCvCode(CameraPipelineFrame::BayerPattern bayerPattern);
    [[nodiscard]] static cv::Mat debayerRawMat(const cv::Mat& input, CameraPipelineFrame::BayerPattern bayerPattern);
    static void captureDirection(CameraPipelineFrame& frame, const CameraSettings& settings);
    [[nodiscard]] static CameraSettings projectionSettingsForFrame(const CameraSettings& settings, const CameraPipelineFrame& frame);
    static void applyPlaybackProjectionTransform(CameraPipelineFrame& frame, const CameraSettings& settings, bool replaceExisting = false);
};

#endif // INCLUDE_FEATURE_CAMERAIMAGEUTILS_H_
