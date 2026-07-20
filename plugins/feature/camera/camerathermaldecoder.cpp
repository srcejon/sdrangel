///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License.                     //
///////////////////////////////////////////////////////////////////////////////////

#include "camerathermaldecoder.h"

#include <cmath>
#include <limits>

namespace {

class StackedRadiometricDecoder : public CameraThermalDecoder
{
public:
    explicit StackedRadiometricDecoder(const QString& decoderName) :
        m_name(decoderName)
    {}

    QString name() const override { return m_name; }

    bool probe(const CameraThermalRawFrame& frame) const override
    {
        // Both supported cameras expose a 256-pixel-wide stacked UVC frame.
        // Keeping this strict prevents an ordinary 640x480 packed-YUV webcam
        // from being auto-detected as radiometric data.
        if ((frame.m_width != 256)
            || (frame.m_height < 382)
            || (frame.m_height > 390)
            || (frame.m_bytesPerLine < frame.m_width * 2))
        {
            return false;
        }

        const int thermalRows = thermalRowCount(frame);
        return (thermalRows >= 120) && (thermalRows <= 256)
            && (frame.m_bytes.size() >= static_cast<qsizetype>(frame.m_bytesPerLine) * frame.m_height);
    }

    bool decode(const CameraThermalRawFrame& frame, CameraThermalDecodeResult& result) const override
    {
        if (!probe(frame)) {
            return false;
        }

        const int rows = thermalRowCount(frame);
        const int firstThermalRow = frame.m_height - rows;
        cv::Mat temperature(rows, frame.m_width, CV_32FC1);
        float minTemperature = std::numeric_limits<float>::max();
        float maxTemperature = std::numeric_limits<float>::lowest();
        double sum = 0.0;
        int validCount = 0;

        for (int y = 0; y < rows; ++y)
        {
            const uchar *line = reinterpret_cast<const uchar*>(frame.m_bytes.constData())
                + static_cast<qsizetype>(firstThermalRow + y) * frame.m_bytesPerLine;
            float *output = temperature.ptr<float>(y);

            for (int x = 0; x < frame.m_width; ++x)
            {
                const quint16 raw = static_cast<quint16>(line[2 * x])
                    | (static_cast<quint16>(line[2 * x + 1]) << 8);
                const float value = static_cast<float>(raw) / 64.0f - 273.15f;
                if (std::isfinite(value) && (value > -100.0f) && (value < 1000.0f))
                {
                    output[x] = value;
                    minTemperature = std::min(minTemperature, value);
                    maxTemperature = std::max(maxTemperature, value);
                    sum += value;
                    ++validCount;
                }
                else
                {
                    output[x] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }

        // A shutter/NUC frame is nearly uniform. Do not feed it into charts or recordings.
        const bool calibrationFrame = validCount > 0 && (maxTemperature - minTemperature < 0.015f);
        if ((validCount < frame.m_width * rows * 9 / 10) || calibrationFrame) {
            result.m_calibrationFrame = calibrationFrame;
            return false;
        }
        if (validCount < frame.m_width * rows) {
            cv::patchNaNs(temperature, static_cast<float>(sum / validCount));
        }

        result.m_temperatureC = temperature;
        result.m_decoderName = name();
        result.m_diagnostic = QStringLiteral("%1x%2 thermal rows %3..%4 mean %5 C")
            .arg(frame.m_width).arg(rows)
            .arg(minTemperature, 0, 'f', 2).arg(maxTemperature, 0, 'f', 2)
            .arg(sum / validCount, 0, 'f', 2);
        return true;
    }

private:
    QString m_name;

    static int thermalRowCount(const CameraThermalRawFrame& frame)
    {
        // TC001/P2-family UVC streams normally stack a visual 256x192 plane above
        // a 16-bit radiometric plane. Some firmware adds two metadata rows.
        if ((frame.m_width == 256) && (frame.m_height >= 382) && (frame.m_height <= 390)) {
            return 192;
        }
        if ((frame.m_height % 2) == 0) {
            return frame.m_height / 2;
        }
        return (frame.m_height - 1) / 2;
    }
};

}

std::unique_ptr<CameraThermalDecoder> CameraThermalDecoder::create(CameraSettings::ThermalDecoder decoder)
{
    switch (decoder)
    {
    case CameraSettings::ThermalDecoderThermalMasterP2:
        return std::make_unique<StackedRadiometricDecoder>(QStringLiteral("Thermal Master P2"));
    case CameraSettings::ThermalDecoderTopdonTc001:
        return std::make_unique<StackedRadiometricDecoder>(QStringLiteral("TOPDON TC001"));
    case CameraSettings::ThermalDecoderAuto:
    case CameraSettings::ThermalDecoderOff:
    default:
        return nullptr;
    }
}

std::unique_ptr<CameraThermalDecoder> CameraThermalDecoder::autoDetect(const CameraThermalRawFrame& frame)
{
    auto decoder = std::make_unique<StackedRadiometricDecoder>(QStringLiteral("TC001/P2 radiometric UVC"));
    if (!decoder->probe(frame)) {
        return nullptr;
    }
    return decoder;
}
