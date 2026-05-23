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

#include <QString>
#include <opencv2/core.hpp>

class CameraYoloTensorRt
{
public:
    CameraYoloTensorRt();
    ~CameraYoloTensorRt();

    void reset();
    bool ensureLoaded(const QString& modelPath, const cv::Size& inputSize, int maxBatch, bool fp16, QString& error);
    bool infer(const std::vector<cv::Mat>& letterboxes, std::vector<cv::Mat>& outputs, QString& error);

    int maxBatch() const;
    QString enginePath() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

#endif // INCLUDE_FEATURE_CAMERAYOLOTENSORRT_H_
