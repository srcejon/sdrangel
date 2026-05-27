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
};

#endif // INCLUDE_FEATURE_CAMERAIMAGEUTILS_H_
