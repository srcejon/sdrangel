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

/**
 * \brief Stateless helpers that fuse multiple exposures into one tone-mapped HDR image.
 *
 * Provides the HDR exposure-fusion routines used by the frame stacker's HDR mode. Mertens
 * fusion (mergeMertensCudaRgb) blends a set of equally-weighted exposures by local
 * contrast/saturation/well-exposedness without needing exposure times; Debevec fusion
 * (mergeDebevecCudaRgb) recovers a radiance map from frames with known exposure times and
 * tone-maps it. Overloads accept inputs either as host cv::Mat or device cv::cuda::GpuMat
 * and likewise return either, so the caller can keep data on the GPU across stages.
 *
 * \note All members are static; the class is never instantiated.
 * \note These routines are only compiled when built with CAMERA_OPENCV_CUDA_STACKING and
 *       run on the GPU using OpenCV-CUDA; the caller supplies the cv::cuda::Stream and a
 *       reusable Laplacian filter for the Mertens pyramid. Failures are reported via the
 *       return value and the optional errorMessage out-parameter.
 */
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
    static bool mergeMertensCudaRgb(
        const std::vector<cv::Mat>& rgbFrames,
        cv::cuda::GpuMat& tonemappedRgbGpu,
        cv::cuda::Stream& stream,
        cv::Ptr<cv::cuda::Filter>& laplacianFilter,
        QString *errorMessage = nullptr);
    static bool mergeMertensCudaRgb(
        const std::vector<cv::cuda::GpuMat>& rgbFramesGpu,
        cv::Mat& tonemappedRgb,
        cv::cuda::Stream& stream,
        cv::Ptr<cv::cuda::Filter>& laplacianFilter,
        QString *errorMessage = nullptr);
    static bool mergeMertensCudaRgb(
        const std::vector<cv::cuda::GpuMat>& rgbFramesGpu,
        cv::cuda::GpuMat& tonemappedRgbGpu,
        cv::cuda::Stream& stream,
        cv::Ptr<cv::cuda::Filter>& laplacianFilter,
        QString *errorMessage = nullptr);
    static bool mergeDebevecCudaRgb(
        const std::vector<cv::Mat>& rgbFrames,
        const std::vector<float>& exposureTimesSeconds,
        cv::cuda::GpuMat& tonemappedRgbGpu,
        cv::cuda::Stream& stream,
        QString *errorMessage = nullptr);
    static bool mergeDebevecCudaRgb(
        const std::vector<cv::cuda::GpuMat>& rgbFramesGpu,
        const std::vector<float>& exposureTimesSeconds,
        cv::cuda::GpuMat& tonemappedRgbGpu,
        cv::cuda::Stream& stream,
        QString *errorMessage = nullptr);
#endif
};

#endif // INCLUDE_FEATURE_CAMERAHDRFUSION_H_
