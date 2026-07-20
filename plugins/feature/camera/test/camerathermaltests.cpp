///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
///////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <iostream>

#include <QCoreApplication>

#include "camerathermaldecoder.h"

namespace {

CameraThermalRawFrame makeFrame(float baseC)
{
    CameraThermalRawFrame frame;
    frame.m_width = 256;
    frame.m_height = 384;
    frame.m_bytesPerLine = 512;
    frame.m_pixelFormatName = QStringLiteral("synthetic Y16");
    frame.m_bytes.resize(frame.m_bytesPerLine * frame.m_height);
    frame.m_bytes.fill(char(0));
    for (int y = 192; y < 384; ++y)
    {
        uchar *line = reinterpret_cast<uchar*>(frame.m_bytes.data()) + y * frame.m_bytesPerLine;
        for (int x = 0; x < 256; ++x)
        {
            const float celsius = baseC + 0.01f * x + 0.02f * (y - 192);
            const quint16 raw = static_cast<quint16>(std::lround((celsius + 273.15f) * 64.0f));
            line[2 * x] = static_cast<uchar>(raw & 0xff);
            line[2 * x + 1] = static_cast<uchar>(raw >> 8);
        }
    }
    return frame;
}

bool closeEnough(float actual, float expected)
{
    return std::abs(actual - expected) <= 1.0f / 64.0f + 0.001f;
}

}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    const CameraThermalRawFrame raw = makeFrame(20.0f);
    auto decoder = CameraThermalDecoder::autoDetect(raw);
    if (!decoder)
    {
        std::cerr << "FAIL: synthetic stacked radiometric frame was not detected\n";
        return 1;
    }
    CameraThermalDecodeResult result;
    if (!decoder->decode(raw, result))
    {
        std::cerr << "FAIL: synthetic stacked radiometric frame did not decode\n";
        return 1;
    }
    if ((result.m_temperatureC.type() != CV_32FC1)
        || (result.m_temperatureC.cols != 256) || (result.m_temperatureC.rows != 192))
    {
        std::cerr << "FAIL: unexpected temperature map shape/type\n";
        return 1;
    }
    if (!closeEnough(result.m_temperatureC.at<float>(0, 0), 20.0f)
        || !closeEnough(result.m_temperatureC.at<float>(191, 255), 26.37f))
    {
        std::cerr << "FAIL: raw 1/64 Kelvin conversion is incorrect\n";
        return 1;
    }
    CameraThermalRawFrame malformed = raw;
    malformed.m_bytes.resize(100);
    if (decoder->probe(malformed))
    {
        std::cerr << "FAIL: truncated frame was accepted\n";
        return 1;
    }

    CameraThermalRawFrame webcam;
    webcam.m_width = 640;
    webcam.m_height = 480;
    webcam.m_bytesPerLine = 1280;
    webcam.m_bytes.resize(webcam.m_bytesPerLine * webcam.m_height);
    if (CameraThermalDecoder::autoDetect(webcam))
    {
        std::cerr << "FAIL: ordinary 640x480 packed video was identified as radiometric\n";
        return 1;
    }

    CameraThermalRawFrame calibration = raw;
    const quint16 calibrationRaw = static_cast<quint16>(std::lround((25.0f + 273.15f) * 64.0f));
    for (int y = 192; y < 384; ++y)
    {
        uchar *line = reinterpret_cast<uchar*>(calibration.m_bytes.data()) + y * calibration.m_bytesPerLine;
        for (int x = 0; x < 256; ++x)
        {
            line[2 * x] = static_cast<uchar>(calibrationRaw & 0xff);
            line[2 * x + 1] = static_cast<uchar>(calibrationRaw >> 8);
        }
    }
    CameraThermalDecodeResult calibrationResult;
    if (decoder->decode(calibration, calibrationResult) || !calibrationResult.m_calibrationFrame)
    {
        std::cerr << "FAIL: uniform shutter-calibration frame was not suppressed\n";
        return 1;
    }

    std::cout << "PASS: radiometric UVC decoder\n";
    return 0;
}
