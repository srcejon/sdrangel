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

#ifndef INCLUDE_CAMERAFFMPEGCOMPAT_H
#define INCLUDE_CAMERAFFMPEGCOMPAT_H

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/channel_layout.h>
#include <libavutil/frame.h>
#include <libswresample/swresample.h>
}

#if LIBAVUTIL_VERSION_MAJOR >= 59
#define CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT 1
#else
#define CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT 0
#endif

inline int cameraFFmpegCodecContextChannels(const AVCodecContext *context)
{
#if CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT
    return context ? context->ch_layout.nb_channels : 0;
#else
    return context ? context->channels : 0;
#endif
}

inline int cameraFFmpegCodecParametersChannels(const AVCodecParameters *parameters)
{
#if CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT
    return parameters ? parameters->ch_layout.nb_channels : 0;
#else
    return parameters ? parameters->channels : 0;
#endif
}

inline bool cameraFFmpegCodecParametersContextChannelLayoutCompatible(
    const AVCodecParameters *parameters,
    const AVCodecContext *context)
{
    if (!parameters || !context) {
        return false;
    }

#if CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT
    if ((parameters->ch_layout.nb_channels <= 0) || (context->ch_layout.nb_channels <= 0)) {
        return true;
    }
    if (parameters->ch_layout.nb_channels != context->ch_layout.nb_channels) {
        return false;
    }
    if ((parameters->ch_layout.order == AV_CHANNEL_ORDER_UNSPEC)
        || (context->ch_layout.order == AV_CHANNEL_ORDER_UNSPEC))
    {
        return true;
    }
    return av_channel_layout_compare(&parameters->ch_layout, &context->ch_layout) == 0;
#else
    return (parameters->channel_layout == 0)
        || (context->channel_layout == 0)
        || (parameters->channel_layout == context->channel_layout);
#endif
}

inline void cameraFFmpegSetStereoChannelLayout(AVCodecContext *context)
{
    if (!context) {
        return;
    }

#if CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT
    av_channel_layout_default(&context->ch_layout, 2);
#else
    context->channel_layout = AV_CH_LAYOUT_STEREO;
    context->channels = 2;
#endif
}

inline int cameraFFmpegSetFrameChannelLayoutFromContext(AVFrame *frame, const AVCodecContext *context)
{
    if (!frame || !context) {
        return AVERROR(EINVAL);
    }

#if CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT
    return av_channel_layout_copy(&frame->ch_layout, &context->ch_layout);
#else
    frame->channel_layout = context->channel_layout;
    return 0;
#endif
}

inline int cameraFFmpegAllocStereoS16Resampler(
    SwrContext **resampler,
    const AVCodecContext *inputContext,
    int outputSampleRate)
{
    if (!resampler || !inputContext) {
        return AVERROR(EINVAL);
    }

#if CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT
    AVChannelLayout inputLayout;
    AVChannelLayout outputLayout;
    av_channel_layout_default(&inputLayout, cameraFFmpegCodecContextChannels(inputContext) > 0
        ? cameraFFmpegCodecContextChannels(inputContext)
        : 1);
    av_channel_layout_default(&outputLayout, 2);

    if (inputContext->ch_layout.nb_channels > 0)
    {
        av_channel_layout_uninit(&inputLayout);
        int ret = av_channel_layout_copy(&inputLayout, &inputContext->ch_layout);
        if (ret < 0)
        {
            av_channel_layout_uninit(&outputLayout);
            return ret;
        }
    }

    const int ret = swr_alloc_set_opts2(
        resampler,
        &outputLayout,
        AV_SAMPLE_FMT_S16,
        outputSampleRate,
        &inputLayout,
        inputContext->sample_fmt,
        inputContext->sample_rate,
        0,
        nullptr);
    av_channel_layout_uninit(&inputLayout);
    av_channel_layout_uninit(&outputLayout);
    return ret;
#else
    const int64_t inputChannelLayout = inputContext->channel_layout != 0
        ? inputContext->channel_layout
        : av_get_default_channel_layout(inputContext->channels);
    *resampler = swr_alloc_set_opts(
        nullptr,
        AV_CH_LAYOUT_STEREO,
        AV_SAMPLE_FMT_S16,
        outputSampleRate,
        inputChannelLayout,
        inputContext->sample_fmt,
        inputContext->sample_rate,
        0,
        nullptr);
    return *resampler ? 0 : AVERROR(ENOMEM);
#endif
}

inline int cameraFFmpegAllocS16StereoRateResampler(
    SwrContext **resampler,
    int inputSampleRate,
    int outputSampleRate)
{
    if (!resampler) {
        return AVERROR(EINVAL);
    }

#if CAMERA_FFMPEG_USE_AV_CHANNEL_LAYOUT
    AVChannelLayout stereoLayout;
    av_channel_layout_default(&stereoLayout, 2);
    const int ret = swr_alloc_set_opts2(
        resampler,
        &stereoLayout,
        AV_SAMPLE_FMT_S16,
        outputSampleRate,
        &stereoLayout,
        AV_SAMPLE_FMT_S16,
        inputSampleRate,
        0,
        nullptr);
    av_channel_layout_uninit(&stereoLayout);
    return ret;
#else
    *resampler = swr_alloc_set_opts(
        nullptr,
        AV_CH_LAYOUT_STEREO,
        AV_SAMPLE_FMT_S16,
        outputSampleRate,
        AV_CH_LAYOUT_STEREO,
        AV_SAMPLE_FMT_S16,
        inputSampleRate,
        0,
        nullptr);
    return *resampler ? 0 : AVERROR(ENOMEM);
#endif
}

#endif

#endif // INCLUDE_CAMERAFFMPEGCOMPAT_H
