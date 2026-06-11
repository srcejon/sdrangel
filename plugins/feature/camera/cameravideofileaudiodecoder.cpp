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

#include "cameravideofileaudiodecoder.h"

#include <algorithm>

#include "cameraffmpegaudio.h"

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}
#endif

CameraVideoFileAudioDecoder::CameraVideoFileAudioDecoder()
{
}

CameraVideoFileAudioDecoder::~CameraVideoFileAudioDecoder()
{
    close();
}

bool CameraVideoFileAudioDecoder::isOpen() const
{
    return m_formatContext && m_codecContext && (m_audioStreamIndex >= 0);
}

bool CameraVideoFileAudioDecoder::open(const QString& fileName, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(fileName)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    close();

    const QByteArray fileNameUtf8 = fileName.toUtf8();
    int ret = avformat_open_input(&m_formatContext, fileNameUtf8.constData(), nullptr, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open video file audio: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }

    ret = avformat_find_stream_info(m_formatContext, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot read video file stream info: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }

    ret = av_find_best_stream(m_formatContext, AVMEDIA_TYPE_AUDIO, -1, -1, nullptr, 0);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Video file has no audio stream");
        close();
        return false;
    }
    m_audioStreamIndex = ret;

    AVStream *stream = m_formatContext->streams[m_audioStreamIndex];
    const AVCodec *codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec)
    {
        errorMessage = QStringLiteral("No decoder is available for the video file audio stream");
        close();
        return false;
    }

    m_codecContext = avcodec_alloc_context3(codec);
    if (!m_codecContext)
    {
        errorMessage = QStringLiteral("Cannot allocate video file audio decoder context");
        close();
        return false;
    }

    ret = avcodec_parameters_to_context(m_codecContext, stream->codecpar);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy video file audio decoder parameters: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }

    ret = avcodec_open2(m_codecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open video file audio decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }

    if (!openResampler(errorMessage))
    {
        close();
        return false;
    }

    m_frame = av_frame_alloc();
    m_packet = av_packet_alloc();
    if (!m_frame || !m_packet)
    {
        errorMessage = QStringLiteral("Cannot allocate video file audio decode buffers");
        close();
        return false;
    }

    m_outputPositionMs = 0;
    m_eof = false;
    return true;
#endif
}

void CameraVideoFileAudioDecoder::close()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_resampler) {
        swr_free(&m_resampler);
    }
    if (m_frame) {
        av_frame_free(&m_frame);
    }
    if (m_packet) {
        av_packet_free(&m_packet);
    }
    if (m_codecContext) {
        avcodec_free_context(&m_codecContext);
    }
    if (m_formatContext) {
        avformat_close_input(&m_formatContext);
    }
#endif
    m_audioStreamIndex = -1;
    m_outputPositionMs = 0;
    m_eof = false;
}

void CameraVideoFileAudioDecoder::seek(qint64 positionMs)
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (!isOpen()) {
        return;
    }

    AVStream *stream = m_formatContext->streams[m_audioStreamIndex];
    const qint64 timestamp = av_rescale_q(
        std::max<qint64>(0, positionMs),
        AVRational{1, 1000},
        stream->time_base);
    if (av_seek_frame(m_formatContext, m_audioStreamIndex, timestamp, AVSEEK_FLAG_BACKWARD) >= 0) {
        avcodec_flush_buffers(m_codecContext);
    }
    m_outputPositionMs = std::max<qint64>(0, positionMs);
    m_eof = false;
#else
    Q_UNUSED(positionMs)
#endif
}

bool CameraVideoFileAudioDecoder::readPcmTo(qint64 playbackPositionMs, QByteArray& pcmS16Stereo, int& sampleRate, QString& errorMessage)
{
    pcmS16Stereo.clear();
    sampleRate = m_outputSampleRate;

    if (!isOpen()) {
        return true;
    }

    const qint64 targetMs = std::max<qint64>(0, playbackPositionMs);
    if ((targetMs + 250 < m_outputPositionMs) || (targetMs > m_outputPositionMs + 5000)) {
        seek(targetMs);
    }

    const qint64 audioLeadMs = 120;
    while (!m_eof && (m_outputPositionMs < targetMs + audioLeadMs))
    {
        const int oldSize = pcmS16Stereo.size();
        if (!decodeNextFrame(pcmS16Stereo, errorMessage)) {
            return false;
        }
        if (pcmS16Stereo.size() == oldSize) {
            break;
        }
    }

    return true;
}

bool CameraVideoFileAudioDecoder::openResampler(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    const int64_t inputChannelLayout = m_codecContext->channel_layout != 0
        ? m_codecContext->channel_layout
        : av_get_default_channel_layout(m_codecContext->channels);
    m_resampler = swr_alloc_set_opts(
        nullptr,
        AV_CH_LAYOUT_STEREO,
        AV_SAMPLE_FMT_S16,
        m_outputSampleRate,
        inputChannelLayout,
        m_codecContext->sample_fmt,
        m_codecContext->sample_rate,
        0,
        nullptr);
    if (!m_resampler)
    {
        errorMessage = QStringLiteral("Cannot allocate video file audio resampler");
        return false;
    }

    const int ret = swr_init(m_resampler);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot initialise video file audio resampler: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }

    return true;
#endif
}

bool CameraVideoFileAudioDecoder::decodeNextFrame(QByteArray& pcmS16Stereo, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(pcmS16Stereo)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    for (;;)
    {
        int ret = avcodec_receive_frame(m_codecContext, m_frame);
        if (ret == 0)
        {
            const bool ok = appendFrameAudio(m_frame, pcmS16Stereo, errorMessage);
            av_frame_unref(m_frame);
            return ok;
        }
        if (ret == AVERROR_EOF)
        {
            m_eof = true;
            return true;
        }
        if (ret != AVERROR(EAGAIN))
        {
            errorMessage = QStringLiteral("Cannot decode video file audio frame: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        ret = av_read_frame(m_formatContext, m_packet);
        if (ret < 0)
        {
            avcodec_send_packet(m_codecContext, nullptr);
            if (ret == AVERROR_EOF) {
                continue;
            }
            errorMessage = QStringLiteral("Cannot read video file audio packet: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        if (m_packet->stream_index == m_audioStreamIndex)
        {
            ret = avcodec_send_packet(m_codecContext, m_packet);
            av_packet_unref(m_packet);
            if (ret < 0)
            {
                errorMessage = QStringLiteral("Cannot send video file audio packet to decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
                return false;
            }
        }
        else
        {
            av_packet_unref(m_packet);
        }
    }
#endif
}

bool CameraVideoFileAudioDecoder::appendFrameAudio(const AVFrame *frame, QByteArray& pcmS16Stereo, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(frame)
    Q_UNUSED(pcmS16Stereo)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!frame || (frame->nb_samples <= 0)) {
        return true;
    }

    const int outputCapacityFrames = static_cast<int>(av_rescale_rnd(
        swr_get_delay(m_resampler, m_codecContext->sample_rate) + frame->nb_samples,
        m_outputSampleRate,
        m_codecContext->sample_rate,
        AV_ROUND_UP));
    if (outputCapacityFrames <= 0) {
        return true;
    }

    QByteArray output;
    output.resize(outputCapacityFrames * 4);
    uint8_t *outputData[1] = { reinterpret_cast<uint8_t*>(output.data()) };
    const int convertedFrames = swr_convert(
        m_resampler,
        outputData,
        outputCapacityFrames,
        const_cast<const uint8_t**>(frame->extended_data),
        frame->nb_samples);
    if (convertedFrames < 0)
    {
        errorMessage = QStringLiteral("Cannot resample video file audio: %1").arg(CameraFFmpegAudio::avErrorString(convertedFrames));
        return false;
    }

    output.resize(convertedFrames * 4);
    pcmS16Stereo.append(output);
    m_outputPositionMs += av_rescale_q(convertedFrames, AVRational{1, m_outputSampleRate}, AVRational{1, 1000});
    return true;
#endif
}
