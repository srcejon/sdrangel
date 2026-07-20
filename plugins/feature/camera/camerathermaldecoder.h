///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERATHERMALDECODER_H_
#define INCLUDE_FEATURE_CAMERATHERMALDECODER_H_

#include <memory>

#include <QByteArray>
#include <QImage>
#include <QString>

#include <opencv2/core.hpp>

#include "camerasettings.h"
#include "camerapipelineframe.h"

using CameraThermalRawFrame = CameraPipelineThermalRawFrame;

struct CameraThermalDecodeResult
{
    cv::Mat m_temperatureC; // CV_32FC1
    QString m_decoderName;
    QString m_diagnostic;
    bool m_calibrationFrame = false;
};

class CameraThermalDecoder
{
public:
    virtual ~CameraThermalDecoder() = default;
    virtual QString name() const = 0;
    virtual bool probe(const CameraThermalRawFrame& frame) const = 0;
    virtual bool decode(const CameraThermalRawFrame& frame, CameraThermalDecodeResult& result) const = 0;

    static std::unique_ptr<CameraThermalDecoder> create(CameraSettings::ThermalDecoder decoder);
    static std::unique_ptr<CameraThermalDecoder> autoDetect(const CameraThermalRawFrame& frame);
};

#endif // INCLUDE_FEATURE_CAMERATHERMALDECODER_H_
