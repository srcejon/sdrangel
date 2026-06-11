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

#include "cameravideofilevideodecoder.h"

#include <algorithm>

#include "cameraffmpegaudio.h"

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}
#endif

CameraVideoFileVideoDecoder::CameraVideoFileVideoDecoder()
{
}

CameraVideoFileVideoDecoder::~CameraVideoFileVideoDecoder()
{
    close();
}

bool CameraVideoFileVideoDecoder::isOpen() const
{
    return m_formatContext && m_codecContext && (m_videoStreamIndex >= 0);
}

bool CameraVideoFileVideoDecoder::open(const QString& fileName, QString& errorMessage)
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
        errorMessage = QStringLiteral("Cannot open video file: %1").arg(CameraFFmpegAudio::avErrorString(ret));
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

    ret = av_find_best_stream(m_formatContext, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Video file has no video stream");
        close();
        return false;
    }
    m_videoStreamIndex = ret;

    AVStream *stream = m_formatContext->streams[m_videoStreamIndex];
    const AVCodec *codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec)
    {
        errorMessage = QStringLiteral("No decoder is available for the video file stream");
        close();
        return false;
    }

    m_codecContext = avcodec_alloc_context3(codec);
    if (!m_codecContext)
    {
        errorMessage = QStringLiteral("Cannot allocate video file decoder context");
        close();
        return false;
    }

    ret = avcodec_parameters_to_context(m_codecContext, stream->codecpar);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy video file decoder parameters: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }

    ret = avcodec_open2(m_codecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open video file decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }

    m_frame = av_frame_alloc();
    m_packet = av_packet_alloc();
    if (!m_frame || !m_packet)
    {
        errorMessage = QStringLiteral("Cannot allocate video file decode buffers");
        close();
        return false;
    }

    if (m_formatContext->duration > 0) {
        m_durationMs = av_rescale_q(m_formatContext->duration, AVRational{1, AV_TIME_BASE}, AVRational{1, 1000});
    } else if (stream->duration > 0) {
        m_durationMs = av_rescale_q(stream->duration, stream->time_base, AVRational{1, 1000});
    }

    const AVRational rate = stream->avg_frame_rate.num > 0 ? stream->avg_frame_rate : stream->r_frame_rate;
    if ((rate.num > 0) && (rate.den > 0)) {
        m_frameRate = qBound(1.0, av_q2d(rate), 240.0);
    }

    m_eof = false;
    return true;
#endif
}

void CameraVideoFileVideoDecoder::close()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_swsContext)
    {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
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
    m_videoStreamIndex = -1;
    m_durationMs = 0;
    m_frameRate = 25.0;
    m_eof = false;
}

void CameraVideoFileVideoDecoder::seek(qint64 positionMs)
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (!isOpen()) {
        return;
    }

    AVStream *stream = m_formatContext->streams[m_videoStreamIndex];
    const qint64 timestamp = av_rescale_q(
        std::max<qint64>(0, positionMs),
        AVRational{1, 1000},
        stream->time_base);
    if (av_seek_frame(m_formatContext, m_videoStreamIndex, timestamp, AVSEEK_FLAG_BACKWARD) >= 0) {
        avcodec_flush_buffers(m_codecContext);
    }
    m_eof = false;
#else
    Q_UNUSED(positionMs)
#endif
}

bool CameraVideoFileVideoDecoder::readNextFrame(QImage& image, qint64& positionMs, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(image)
    Q_UNUSED(positionMs)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    image = QImage();
    positionMs = -1;
    if (!isOpen() || m_eof) {
        return true;
    }

    for (;;)
    {
        int ret = avcodec_receive_frame(m_codecContext, m_frame);
        if (ret == 0)
        {
            const int64_t bestTimestamp = m_frame->best_effort_timestamp;
            if (bestTimestamp != AV_NOPTS_VALUE) {
                positionMs = av_rescale_q(bestTimestamp, m_formatContext->streams[m_videoStreamIndex]->time_base, AVRational{1, 1000});
            }
            const bool ok = convertFrameToImage(m_frame, image, errorMessage);
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
            errorMessage = QStringLiteral("Cannot decode video file frame: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        ret = av_read_frame(m_formatContext, m_packet);
        if (ret < 0)
        {
            avcodec_send_packet(m_codecContext, nullptr);
            if (ret == AVERROR_EOF) {
                continue;
            }
            errorMessage = QStringLiteral("Cannot read video file packet: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        if (m_packet->stream_index == m_videoStreamIndex)
        {
            ret = avcodec_send_packet(m_codecContext, m_packet);
            av_packet_unref(m_packet);
            if (ret < 0)
            {
                errorMessage = QStringLiteral("Cannot send video file packet to decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
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

bool CameraVideoFileVideoDecoder::convertFrameToImage(const AVFrame *frame, QImage& image, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(frame)
    Q_UNUSED(image)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!frame || (frame->width <= 0) || (frame->height <= 0))
    {
        errorMessage = QStringLiteral("Decoded video file frame is empty");
        return false;
    }

    if (!m_swsContext
        || (m_codecContext->width != frame->width)
        || (m_codecContext->height != frame->height)
        || (m_codecContext->pix_fmt != static_cast<AVPixelFormat>(frame->format)))
    {
        if (m_swsContext) {
            sws_freeContext(m_swsContext);
        }
        m_swsContext = sws_getContext(
            frame->width,
            frame->height,
            static_cast<AVPixelFormat>(frame->format),
            frame->width,
            frame->height,
            AV_PIX_FMT_RGB24,
            SWS_BILINEAR,
            nullptr,
            nullptr,
            nullptr);
        if (!m_swsContext)
        {
            errorMessage = QStringLiteral("Cannot create video file colour converter");
            return false;
        }
    }

    image = QImage(frame->width, frame->height, QImage::Format_RGB888);
    if (image.isNull())
    {
        errorMessage = QStringLiteral("Cannot allocate decoded video file image");
        return false;
    }

    uint8_t *dstData[1] = { image.bits() };
    int dstLinesize[1] = { static_cast<int>(image.bytesPerLine()) };
    sws_scale(m_swsContext, frame->data, frame->linesize, 0, frame->height, dstData, dstLinesize);
    return true;
#endif
}
