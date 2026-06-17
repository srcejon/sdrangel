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

#ifndef INCLUDE_FEATURE_CAMERAHDRFUSION_H_
#define INCLUDE_FEATURE_CAMERAHDRFUSION_H_

#include <vector>

#include <QString>

#include <opencv2/core/core.hpp>

#ifdef CAMERA_OPENCV_CUDA_STACKING
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#endif

class CameraHdrFusion
{
public:
#ifdef CAMERA_OPENCV_CUDA_STACKING
    static bool mergeMertensCudaRgb(
        const std::vector<cv::Mat>& rgbFrames,
        cv::Mat& tonemappedRgb,
        cv::cuda::Stream& stream,
        cv::Ptr<cv::cuda::Filter>& laplacianFilter,
        QString *errorMessage = nullptr);
#endif
};

#endif // INCLUDE_FEATURE_CAMERAHDRFUSION_H_
