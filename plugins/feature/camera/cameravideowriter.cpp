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

#include "cameravideowriter.h"

#include <algorithm>
#include <cmath>

#include <QByteArray>
#include <QDebug>
#include <QPainter>

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#endif

CameraVideoWriter::CameraVideoWriter()
{
}

CameraVideoWriter::~CameraVideoWriter()
{
    close();
}

bool CameraVideoWriter::isOpen() const
{
    return m_formatContext != nullptr;
}

QString CameraVideoWriter::avErrorString(int errorCode)
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

QString CameraVideoWriter::codecName(CameraSettings::VideoCodec codec)
{
    switch (codec)
    {
    case CameraSettings::VideoCodecH265:
        return QStringLiteral("H.265");
    case CameraSettings::VideoCodecH264:
    default:
        return QStringLiteral("H.264");
    }
}

QString CameraVideoWriter::codecName() const
{
    return codecName(m_settings.m_codec);
}

QSize CameraVideoWriter::evenSize(const QSize& size)
{
    return QSize(std::max(2, size.width() & ~1), std::max(2, size.height() & ~1));
}

QImage CameraVideoWriter::prepareRgbImage(const QImage& image, const QSize& size)
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

bool CameraVideoWriter::open(const Settings& settings, const QImage& firstFrame, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(settings)
    Q_UNUSED(firstFrame)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    close();

    const QString fileName = settings.m_fileName.trimmed();
    if (fileName.isEmpty())
    {
        errorMessage = QStringLiteral("Video file name is empty");
        return false;
    }

    const QSize videoSize = evenSize(firstFrame.size());
    if (!videoSize.isValid())
    {
        errorMessage = QStringLiteral("Invalid video frame size");
        return false;
    }

    const double requestedFps = std::max(0.001, settings.m_fps);
    const int fpsDen = 1000;
    const int fpsNum = std::max(1, static_cast<int>(std::llround(requestedFps * fpsDen)));
    const AVRational timeBase{fpsDen, fpsNum};
    const AVRational frameRate{fpsNum, fpsDen};

    const AVCodecID codecId = settings.m_codec == CameraSettings::VideoCodecH265
        ? AV_CODEC_ID_HEVC
        : AV_CODEC_ID_H264;
    const char *hardwareEncoderName = settings.m_codec == CameraSettings::VideoCodecH265
        ? "hevc_nvenc"
        : "h264_nvenc";
    const char *softwareEncoderName = settings.m_codec == CameraSettings::VideoCodecH265
        ? "libx265"
        : "libx264";

    const AVCodec *codec = settings.m_preferHardwareEncoding
        ? avcodec_find_encoder_by_name(hardwareEncoderName)
        : nullptr;
    if (!codec) {
        codec = avcodec_find_encoder_by_name(softwareEncoderName);
    }
    if (!codec) {
        codec = avcodec_find_encoder(codecId);
    }
    if (!codec)
    {
        errorMessage = QStringLiteral("No %1 encoder is available in FFmpeg").arg(codecName(settings.m_codec));
        return false;
    }

    const QByteArray fileNameUtf8 = fileName.toUtf8();
    int ret = avformat_alloc_output_context2(&m_formatContext, nullptr, nullptr, fileNameUtf8.constData());
    if ((ret < 0) || !m_formatContext)
    {
        errorMessage = QStringLiteral("Cannot create video output context: %1").arg(avErrorString(ret));
        close();
        return false;
    }

    AVStream *stream = avformat_new_stream(m_formatContext, nullptr);
    if (!stream)
    {
        errorMessage = QStringLiteral("Cannot create video stream");
        close();
        return false;
    }
    m_streamIndex = stream->index;

    m_codecContext = avcodec_alloc_context3(codec);
    if (!m_codecContext)
    {
        errorMessage = QStringLiteral("Cannot allocate %1 encoder context").arg(codecName(settings.m_codec));
        close();
        return false;
    }

    m_codecContext->codec_id = codec->id;
    m_codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
    m_codecContext->width = videoSize.width();
    m_codecContext->height = videoSize.height();
    m_codecContext->time_base = timeBase;
    m_codecContext->framerate = frameRate;
    m_codecContext->gop_size = std::max(1, static_cast<int>(std::llround(requestedFps * 2.0)));
    m_codecContext->max_b_frames = 0;
    m_codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
    m_codecContext->bit_rate = 10000000;

    if (m_formatContext->oformat->flags & AVFMT_GLOBALHEADER) {
        m_codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    av_opt_set(m_codecContext->priv_data, "preset", "veryfast", 0);
    if (settings.m_codec == CameraSettings::VideoCodecH264) {
        av_opt_set(m_codecContext->priv_data, "tune", "zerolatency", 0);
    }

    ret = avcodec_open2(m_codecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open %1 encoder: %2").arg(codecName(settings.m_codec), avErrorString(ret));
        close();
        return false;
    }

    ret = avcodec_parameters_from_context(stream->codecpar, m_codecContext);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy video encoder parameters: %1").arg(avErrorString(ret));
        close();
        return false;
    }
    stream->codecpar->codec_tag = 0;
    stream->time_base = m_codecContext->time_base;

    if (!(m_formatContext->oformat->flags & AVFMT_NOFILE))
    {
        ret = avio_open(&m_formatContext->pb, fileNameUtf8.constData(), AVIO_FLAG_WRITE);
        if (ret < 0)
        {
            errorMessage = QStringLiteral("Cannot open video file: %1").arg(avErrorString(ret));
            close();
            return false;
        }
    }

    ret = avformat_write_header(m_formatContext, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write video file header: %1").arg(avErrorString(ret));
        close();
        return false;
    }
    m_headerWritten = true;

    m_frame = av_frame_alloc();
    if (!m_frame)
    {
        errorMessage = QStringLiteral("Cannot allocate video frame");
        close();
        return false;
    }
    m_frame->format = m_codecContext->pix_fmt;
    m_frame->width = m_codecContext->width;
    m_frame->height = m_codecContext->height;

    ret = av_frame_get_buffer(m_frame, 32);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot allocate video frame buffer: %1").arg(avErrorString(ret));
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
        errorMessage = QStringLiteral("Cannot create video colour converter");
        close();
        return false;
    }

    m_settings = settings;
    m_videoSize = videoSize;
    m_frameIndex = 0;

    qDebug() << "CameraVideoWriter: opened" << fileName << codecName(settings.m_codec)
             << videoSize << "fps" << requestedFps << "encoder" << codec->name;
    return true;
#endif
}

bool CameraVideoWriter::writeEncodedPacket(AVPacket *packet, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(packet)
    Q_UNUSED(errorMessage)
    return false;
#else
    packet->stream_index = m_streamIndex;
    av_packet_rescale_ts(packet, m_codecContext->time_base, m_formatContext->streams[m_streamIndex]->time_base);
    if (packet->duration <= 0) {
        packet->duration = av_rescale_q(1, m_codecContext->time_base, m_formatContext->streams[m_streamIndex]->time_base);
    }
    const int ret = av_interleaved_write_frame(m_formatContext, packet);
    av_packet_unref(packet);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write video packet: %1").arg(avErrorString(ret));
        return false;
    }

    return true;
#endif
}

bool CameraVideoWriter::writeFrame(const QImage& image, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(image)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!isOpen())
    {
        errorMessage = QStringLiteral("Video file is not open");
        return false;
    }

    const QImage rgb = prepareRgbImage(image, m_videoSize);
    if (rgb.isNull())
    {
        errorMessage = QStringLiteral("Cannot prepare frame for video recording");
        return false;
    }

    int ret = av_frame_make_writable(m_frame);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write to video frame buffer: %1").arg(avErrorString(ret));
        return false;
    }

    const uint8_t *srcData[1] = { rgb.constBits() };
    const int srcLineSize[1] = { static_cast<int>(rgb.bytesPerLine()) };
    sws_scale(m_swsContext, srcData, srcLineSize, 0, m_codecContext->height, m_frame->data, m_frame->linesize);
    m_frame->pts = m_frameIndex++;

    ret = avcodec_send_frame(m_codecContext, m_frame);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot encode video frame: %1").arg(avErrorString(ret));
        return false;
    }

    AVPacket *packet = av_packet_alloc();
    if (!packet)
    {
        errorMessage = QStringLiteral("Cannot allocate video packet");
        return false;
    }

    bool ok = true;
    while ((ret = avcodec_receive_packet(m_codecContext, packet)) >= 0)
    {
        if (!writeEncodedPacket(packet, errorMessage))
        {
            ok = false;
            break;
        }
    }

    if (ok && (ret != AVERROR(EAGAIN)) && (ret != AVERROR_EOF))
    {
        errorMessage = QStringLiteral("Cannot receive video packet: %1").arg(avErrorString(ret));
        ok = false;
    }

    av_packet_free(&packet);
    return ok;
#endif
}

void CameraVideoWriter::flushEncoder()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (!m_codecContext || !m_formatContext || (m_streamIndex < 0)) {
        return;
    }

    avcodec_send_frame(m_codecContext, nullptr);
    AVPacket *packet = av_packet_alloc();
    if (packet)
    {
        QString ignoredError;
        while (avcodec_receive_packet(m_codecContext, packet) >= 0) {
            const bool ignoredOk = writeEncodedPacket(packet, ignoredError);
            Q_UNUSED(ignoredOk)
        }
        av_packet_free(&packet);
    }
#endif
}

void CameraVideoWriter::close()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_headerWritten && m_formatContext) {
        flushEncoder();
    }

    if (m_formatContext)
    {
        if (m_headerWritten) {
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
    m_headerWritten = false;
    m_videoSize = QSize();
}
