///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
// Some code by AI                                                               //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERAYOLOLITERT_H_
#define INCLUDE_FEATURE_CAMERAYOLOLITERT_H_

#include <memory>
#include <vector>

#include <QString>
#include <opencv2/core.hpp>

/**
 * Android LiteRT backend for YOLO .tflite models.
 *
 * The implementation uses LiteRT's C API and optional GPU delegate. The model,
 * interpreter, delegate and tensor buffers persist on the detector thread.
 */
class CameraYoloLiteRt
{
public:
    CameraYoloLiteRt();
    ~CameraYoloLiteRt();

    void reset();
    bool ensureLoaded(const QString& modelPath, bool useGpu, QString& error);
    bool infer(const cv::Mat& letterbox, std::vector<cv::Mat>& outputs, QString& error);
    cv::Size inputSize() const;
    bool gpuActive() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif // INCLUDE_FEATURE_CAMERAYOLOLITERT_H_
