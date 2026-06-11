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

#include "cameraffmpegaudio.h"

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/error.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}
#endif

QString CameraFFmpegAudio::avErrorString(int errorCode)
{
#ifdef CAMERA_FFMPEG_STREAMING
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(errorCode, buffer, sizeof(buffer));
    return QString::fromLocal8Bit(buffer);
#else
    Q_UNUSED(errorCode)
    return QStringLiteral("FFmpeg support is not available in this build");
#endif
}

bool CameraFFmpegAudio::resamplePcmS16Stereo(const QByteArray& input,
                                             int inputSampleRate,
                                             int outputSampleRate,
                                             QByteArray& output,
                                             QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(input)
    Q_UNUSED(inputSampleRate)
    Q_UNUSED(outputSampleRate)
    Q_UNUSED(output)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    output.clear();
    if (input.isEmpty()) {
        return true;
    }
    if ((inputSampleRate <= 0) || (outputSampleRate <= 0))
    {
        errorMessage = QStringLiteral("Invalid audio sample rate");
        return false;
    }
    if ((input.size() % 4) != 0)
    {
        errorMessage = QStringLiteral("Stereo 16-bit PCM buffer has an invalid size");
        return false;
    }
    if (inputSampleRate == outputSampleRate)
    {
        output = input;
        return true;
    }

    SwrContext *context = swr_alloc_set_opts(
        nullptr,
        AV_CH_LAYOUT_STEREO,
        AV_SAMPLE_FMT_S16,
        outputSampleRate,
        AV_CH_LAYOUT_STEREO,
        AV_SAMPLE_FMT_S16,
        inputSampleRate,
        0,
        nullptr);
    if (!context)
    {
        errorMessage = QStringLiteral("Cannot allocate audio resampler");
        return false;
    }

    int ret = swr_init(context);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot initialise audio resampler: %1").arg(avErrorString(ret));
        swr_free(&context);
        return false;
    }

    const int inputFrames = input.size() / 4;
    const int outputCapacityFrames = static_cast<int>(av_rescale_rnd(
        swr_get_delay(context, inputSampleRate) + inputFrames,
        outputSampleRate,
        inputSampleRate,
        AV_ROUND_UP));
    if (outputCapacityFrames <= 0)
    {
        swr_free(&context);
        return true;
    }

    output.resize(outputCapacityFrames * 4);
    const uint8_t *inputData[1] = { reinterpret_cast<const uint8_t*>(input.constData()) };
    uint8_t *outputData[1] = { reinterpret_cast<uint8_t*>(output.data()) };
    ret = swr_convert(context, outputData, outputCapacityFrames, inputData, inputFrames);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot resample audio: %1").arg(avErrorString(ret));
        swr_free(&context);
        output.clear();
        return false;
    }

    output.resize(ret * 4);
    swr_free(&context);
    return true;
#endif
}
