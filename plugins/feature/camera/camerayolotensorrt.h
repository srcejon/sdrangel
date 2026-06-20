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

#ifndef INCLUDE_FEATURE_CAMERAYOLOTENSORRT_H_
#define INCLUDE_FEATURE_CAMERAYOLOTENSORRT_H_

#include <memory>
#include <vector>
#include <functional>

#include <QString>
#include <opencv2/core.hpp>

/**
 * \brief Optional NVIDIA TensorRT backend for batched YOLO inference.
 *
 * Wraps the TensorRT runtime used by CameraObjectDetector when CAMERA_TENSORRT_YOLO is enabled.
 * ensureLoaded() builds (or loads a cached) serialized engine from an ONNX model for a given
 * input size, max batch and FP16 mode, and infer() runs a batch of letterboxed images through
 * that engine, returning the raw output tensors for the detector to decode. A progress callback
 * reports engine-build activity so the GUI can surface the (potentially slow) conversion.
 *
 * \note All TensorRT/CUDA details are hidden behind a private Impl (pImpl); when built without
 *       CAMERA_TENSORRT_YOLO the class is an empty stub. This is a plain helper, not a QObject,
 *       and is owned directly by CameraObjectDetector, so it shares that detector's thread
 *       affinity and is not safe to call concurrently from multiple threads.
 */
class CameraYoloTensorRt
{
public:
    CameraYoloTensorRt();
    ~CameraYoloTensorRt();

    void reset();
    using ProgressCallback = std::function<void(bool active, const QString& modelPath, const QString& enginePath)>;
    void setProgressCallback(ProgressCallback callback);
    bool ensureLoaded(const QString& modelPath, const cv::Size& inputSize, int maxBatch, bool fp16, QString& error);
    bool infer(const std::vector<cv::Mat>& letterboxes, std::vector<cv::Mat>& outputs, QString& error);

    int maxBatch() const;
    QString enginePath() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif // INCLUDE_FEATURE_CAMERAYOLOTENSORRT_H_
