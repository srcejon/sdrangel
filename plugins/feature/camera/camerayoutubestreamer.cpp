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

#include "camerayoutubestreamer.h"

#include <algorithm>

#include <QByteArray>
#include <QDebug>
#include <QPainter>

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#endif

CameraYouTubeStreamer::CameraYouTubeStreamer()
{
}

CameraYouTubeStreamer::~CameraYouTubeStreamer()
{
    close();
}

bool CameraYouTubeStreamer::isOpen() const
{
    return m_formatContext != nullptr;
}

QString CameraYouTubeStreamer::avErrorString(int errorCode)
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

QString CameraYouTubeStreamer::streamTargetUrl(const Settings& settings)
{
    QString url = settings.m_url.trimmed();
    const QString key = settings.m_key.trimmed();

    if (url.startsWith(QStringLiteral("rtmp://a.rtmps.youtube.com/"), Qt::CaseInsensitive)) {
        url.replace(0, 7, QStringLiteral("rtmps://"));
    }
    else if (url.startsWith(QStringLiteral("rtmps://a.rtmp.youtube.com/"), Qt::CaseInsensitive)) {
        url.replace(0, 8, QStringLiteral("rtmp://"));
    }

    if (!key.isEmpty())
    {
        if (!url.endsWith('/')) {
            url += '/';
        }
        url += key;
    }

    return url;
}

QSize CameraYouTubeStreamer::evenSize(const QSize& size)
{
    return QSize(std::max(2, size.width() & ~1), std::max(2, size.height() & ~1));
}

QImage CameraYouTubeStreamer::prepareRgbImage(const QImage& image, const QSize& size)
{
    if (image.isNull()) {
        return QImage();
    }

    QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    if (rgb.size() == size) {
        return rgb;
    }

    QImage scaled(size, QImage::Format_RGB888);
    scaled.fill(Qt::black);
    QPainter painter(&scaled);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    painter.drawImage(QRect(QPoint(0, 0), size), rgb);
    return scaled;
}

bool CameraYouTubeStreamer::open(const Settings& settings, const QImage& firstFrame, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(settings)
    Q_UNUSED(firstFrame)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    close();

    const QString targetUrl = streamTargetUrl(settings);
    if (targetUrl.isEmpty())
    {
        errorMessage = QStringLiteral("YouTube stream URL is empty");
        return false;
    }
    if (settings.m_key.trimmed().isEmpty())
    {
        errorMessage = QStringLiteral("YouTube stream key is empty");
        return false;
    }

    QSize streamSize = settings.m_size.isValid() ? settings.m_size : firstFrame.size();
    streamSize = evenSize(streamSize);
    if (!streamSize.isValid())
    {
        errorMessage = QStringLiteral("Invalid YouTube stream size");
        return false;
    }

    const int fps = std::max(1, settings.m_fps);
    const AVCodec *codec = avcodec_find_encoder_by_name("libx264");
    if (!codec) {
        codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    }
    if (!codec)
    {
        errorMessage = QStringLiteral("No H.264 encoder is available in FFmpeg");
        return false;
    }

    const QByteArray targetUrlUtf8 = targetUrl.toUtf8();
    int ret = avformat_alloc_output_context2(&m_formatContext, nullptr, "flv", targetUrlUtf8.constData());
    if ((ret < 0) || !m_formatContext)
    {
        errorMessage = QStringLiteral("Cannot create FLV output context: %1").arg(avErrorString(ret));
        close();
        return false;
    }
    m_formatContext->flags |= AVFMT_FLAG_FLUSH_PACKETS;

    AVStream *stream = avformat_new_stream(m_formatContext, nullptr);
    if (!stream)
    {
        errorMessage = QStringLiteral("Cannot create YouTube video stream");
        close();
        return false;
    }
    m_streamIndex = stream->index;

    m_codecContext = avcodec_alloc_context3(codec);
    if (!m_codecContext)
    {
        errorMessage = QStringLiteral("Cannot allocate H.264 encoder context");
        close();
        return false;
    }

    m_codecContext->codec_id = codec->id;
    m_codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
    m_codecContext->width = streamSize.width();
    m_codecContext->height = streamSize.height();
    m_codecContext->time_base = AVRational{1, fps};
    m_codecContext->framerate = AVRational{fps, 1};
    m_codecContext->gop_size = fps * 2;
    m_codecContext->max_b_frames = 0;
    m_codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
    m_codecContext->bit_rate = static_cast<int64_t>(std::max(100, settings.m_bitrateKbps)) * 1000;

    if (m_formatContext->oformat->flags & AVFMT_GLOBALHEADER) {
        m_codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    av_opt_set(m_codecContext->priv_data, "preset", "veryfast", 0);
    av_opt_set(m_codecContext->priv_data, "tune", "zerolatency", 0);

    ret = avcodec_open2(m_codecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open H.264 encoder: %1").arg(avErrorString(ret));
        close();
        return false;
    }

    ret = avcodec_parameters_from_context(stream->codecpar, m_codecContext);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy stream encoder parameters: %1").arg(avErrorString(ret));
        close();
        return false;
    }
    stream->time_base = m_codecContext->time_base;

    AVDictionary *ioOptions = nullptr;
    av_dict_set(&ioOptions, "rtmp_live", "live", 0);
    ret = avio_open2(&m_formatContext->pb, targetUrlUtf8.constData(), AVIO_FLAG_WRITE, nullptr, &ioOptions);
    av_dict_free(&ioOptions);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open YouTube stream URL: %1").arg(avErrorString(ret));
        close();
        return false;
    }

    AVDictionary *formatOptions = nullptr;
    av_dict_set(&formatOptions, "flvflags", "no_duration_filesize", 0);
    ret = avformat_write_header(m_formatContext, &formatOptions);
    av_dict_free(&formatOptions);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write YouTube stream header: %1").arg(avErrorString(ret));
        close();
        return false;
    }
    m_headerWritten = true;
    avio_flush(m_formatContext->pb);

    m_frame = av_frame_alloc();
    if (!m_frame)
    {
        errorMessage = QStringLiteral("Cannot allocate YouTube stream frame");
        close();
        return false;
    }
    m_frame->format = m_codecContext->pix_fmt;
    m_frame->width = m_codecContext->width;
    m_frame->height = m_codecContext->height;

    ret = av_frame_get_buffer(m_frame, 32);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot allocate YouTube stream frame buffer: %1").arg(avErrorString(ret));
        close();
        return false;
    }

    m_swsContext = sws_getContext(
        m_codecContext->width,
        m_codecContext->height,
        AV_PIX_FMT_RGB24,
        m_codecContext->width,
        m_codecContext->height,
        m_codecContext->pix_fmt,
        SWS_BILINEAR,
        nullptr,
        nullptr,
        nullptr);
    if (!m_swsContext)
    {
        errorMessage = QStringLiteral("Cannot create YouTube stream colour converter");
        close();
        return false;
    }

    m_settings = settings;
    m_streamSize = streamSize;
    m_frameIndex = 0;
    m_lastFrameElapsedMs = -1;
    m_nextFrameElapsedMs = 0;
    m_packetsWritten = 0;
    m_bytesWritten = 0;
    m_lastStatsElapsedMs = 0;
    m_streamTimer.restart();
    qDebug() << "CameraYouTubeStreamer: opened stream" << targetUrl
             << streamSize << "fps" << fps << "bitrateKbps" << settings.m_bitrateKbps;
    return true;
#endif
}

bool CameraYouTubeStreamer::encodeAndWriteRgbFrame(const QImage& rgb, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(rgb)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    int ret = av_frame_make_writable(m_frame);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write to YouTube stream frame buffer: %1").arg(avErrorString(ret));
        return false;
    }

    const uint8_t *srcData[1] = { rgb.constBits() };
    const int srcLineSize[1] = { static_cast<int>(rgb.bytesPerLine()) };
    sws_scale(m_swsContext, srcData, srcLineSize, 0, m_codecContext->height, m_frame->data, m_frame->linesize);
    m_frame->pts = m_frameIndex++;

    ret = avcodec_send_frame(m_codecContext, m_frame);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot encode YouTube stream frame: %1").arg(avErrorString(ret));
        return false;
    }

    AVPacket *packet = av_packet_alloc();
    if (!packet)
    {
        errorMessage = QStringLiteral("Cannot allocate YouTube stream packet");
        return false;
    }

    bool ok = true;
    while ((ret = avcodec_receive_packet(m_codecContext, packet)) >= 0)
    {
        packet->stream_index = m_streamIndex;
        av_packet_rescale_ts(packet, m_codecContext->time_base, m_formatContext->streams[m_streamIndex]->time_base);
        packet->duration = av_rescale_q(1, m_codecContext->time_base, m_formatContext->streams[m_streamIndex]->time_base);
        const int packetSize = packet->size;
        ret = av_interleaved_write_frame(m_formatContext, packet);
        if ((ret >= 0) && m_formatContext->pb) {
            avio_flush(m_formatContext->pb);
        }
        av_packet_unref(packet);
        if (ret < 0)
        {
            errorMessage = QStringLiteral("Cannot write YouTube stream packet: %1").arg(avErrorString(ret));
            ok = false;
            break;
        }
        ++m_packetsWritten;
        m_bytesWritten += packetSize;

        const qint64 elapsedMs = m_streamTimer.elapsed();
        if ((elapsedMs - m_lastStatsElapsedMs) >= 5000)
        {
            qDebug() << "CameraYouTubeStreamer: wrote packets" << m_packetsWritten
                     << "bytes" << m_bytesWritten
                     << "frames" << m_frameIndex
                     << "elapsedMs" << elapsedMs;
            m_lastStatsElapsedMs = elapsedMs;
        }
    }

    if (ok && (ret != AVERROR(EAGAIN)) && (ret != AVERROR_EOF))
    {
        errorMessage = QStringLiteral("Cannot receive YouTube stream packet: %1").arg(avErrorString(ret));
        ok = false;
    }

    av_packet_free(&packet);
    return ok;
#endif
}

bool CameraYouTubeStreamer::writeFrame(const QImage& image, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(image)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!isOpen())
    {
        errorMessage = QStringLiteral("YouTube stream is not open");
        return false;
    }

    const int fps = std::max(1, m_settings.m_fps);
    const int framePeriodMs = std::max(1, 1000 / fps);
    const qint64 elapsedMs = m_streamTimer.elapsed();
    if ((m_lastFrameElapsedMs >= 0) && (elapsedMs < m_nextFrameElapsedMs)) {
        return true;
    }

    const QImage rgb = prepareRgbImage(image, m_streamSize);
    if (rgb.isNull())
    {
        errorMessage = QStringLiteral("Cannot prepare frame for YouTube streaming");
        return false;
    }

    if (m_nextFrameElapsedMs <= 0) {
        m_nextFrameElapsedMs = elapsedMs;
    }
    if ((elapsedMs - m_nextFrameElapsedMs) > 2000) {
        m_nextFrameElapsedMs = elapsedMs;
    }

    const int maxFramesPerInputFrame = std::max(1, fps);
    int framesWritten = 0;
    do
    {
        if (!encodeAndWriteRgbFrame(rgb, errorMessage)) {
            return false;
        }

        m_lastFrameElapsedMs = elapsedMs;
        m_nextFrameElapsedMs += framePeriodMs;
        ++framesWritten;
    } while ((m_nextFrameElapsedMs <= elapsedMs) && (framesWritten < maxFramesPerInputFrame));

    return true;
#endif
}

void CameraYouTubeStreamer::close()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_headerWritten && m_codecContext && m_formatContext)
    {
        avcodec_send_frame(m_codecContext, nullptr);
        AVPacket *packet = av_packet_alloc();
        if (packet)
        {
            while (avcodec_receive_packet(m_codecContext, packet) >= 0)
            {
                packet->stream_index = m_streamIndex;
                av_packet_rescale_ts(packet, m_codecContext->time_base, m_formatContext->streams[m_streamIndex]->time_base);
                av_interleaved_write_frame(m_formatContext, packet);
                av_packet_unref(packet);
            }
            av_packet_free(&packet);
        }
    }

    if (m_formatContext)
    {
        if (m_headerWritten && m_formatContext->pb) {
            av_write_trailer(m_formatContext);
        }
        if (!(m_formatContext->oformat->flags & AVFMT_NOFILE) && m_formatContext->pb) {
            avio_closep(&m_formatContext->pb);
        }
        avformat_free_context(m_formatContext);
        m_formatContext = nullptr;
    }
    if (m_codecContext) {
        avcodec_free_context(&m_codecContext);
    }
    if (m_frame) {
        av_frame_free(&m_frame);
    }
    if (m_swsContext)
    {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
    }
#endif
    m_streamIndex = -1;
    m_frameIndex = 0;
    m_lastFrameElapsedMs = -1;
    m_nextFrameElapsedMs = 0;
    m_packetsWritten = 0;
    m_bytesWritten = 0;
    m_lastStatsElapsedMs = 0;
    m_headerWritten = false;
    m_streamSize = QSize();
}
