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

#include "cameraffmpegaudio.h"

#include <algorithm>
#include <cstring>

#include <QByteArray>
#include <QDebug>
#include <QPainter>
#include <QUrl>

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}
#endif

static constexpr int kCameraYouTubeAudioSampleRate = 48000;

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

QString CameraYouTubeStreamer::redactedStreamTargetUrl(const QString& targetUrl)
{
    QUrl url(targetUrl);

    if (!url.isValid()) {
        return QStringLiteral("<invalid>");
    }

    QString path = url.path();
    const int slash = path.lastIndexOf('/');
    if (slash >= 0) {
        path = path.left(slash + 1) + QStringLiteral("redacted");
    }
    url.setPath(path);
    url.setQuery(QString());
    url.setFragment(QString());
    return url.toString(QUrl::RemovePassword);
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

    const QString encoderName = QString::fromLatin1(codec->name);
    const bool nvencEncoder = encoderName.contains(QStringLiteral("nvenc"), Qt::CaseInsensitive);
    if (!nvencEncoder)
    {
        av_opt_set(m_codecContext->priv_data, "preset", "veryfast", 0);
        av_opt_set(m_codecContext->priv_data, "tune", "zerolatency", 0);
    }

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

    if (!openAudioStream(errorMessage))
    {
        close();
        return false;
    }

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
    m_audioFrameIndex = 0;
    m_audioInputBuffer.clear();
    m_lastFrameElapsedMs = -1;
    m_nextFrameElapsedMs = 0;
    m_streamTimer.restart();
    qDebug() << "CameraYouTubeStreamer: opened stream" << redactedStreamTargetUrl(targetUrl)
             << streamSize << "fps" << fps << "bitrateKbps" << settings.m_bitrateKbps;
    return true;
#endif
}

bool CameraYouTubeStreamer::openAudioStream(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
    if (!codec)
    {
        errorMessage = QStringLiteral("No AAC encoder is available in FFmpeg");
        return false;
    }

    AVStream *stream = avformat_new_stream(m_formatContext, nullptr);
    if (!stream)
    {
        errorMessage = QStringLiteral("Cannot create YouTube audio stream");
        return false;
    }
    m_audioStreamIndex = stream->index;

    m_audioCodecContext = avcodec_alloc_context3(codec);
    if (!m_audioCodecContext)
    {
        errorMessage = QStringLiteral("Cannot allocate AAC encoder context");
        return false;
    }

    m_audioCodecContext->codec_id = codec->id;
    m_audioCodecContext->codec_type = AVMEDIA_TYPE_AUDIO;
    m_audioCodecContext->sample_rate = kCameraYouTubeAudioSampleRate;
    m_audioCodecContext->sample_fmt = codec->sample_fmts ? codec->sample_fmts[0] : AV_SAMPLE_FMT_FLTP;
    m_audioCodecContext->bit_rate = 128000;
    m_audioCodecContext->channel_layout = AV_CH_LAYOUT_STEREO;
    m_audioCodecContext->channels = 2;
    m_audioCodecContext->time_base = AVRational{1, m_audioCodecContext->sample_rate};

    if (m_formatContext->oformat->flags & AVFMT_GLOBALHEADER) {
        m_audioCodecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    int ret = avcodec_open2(m_audioCodecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open AAC encoder: %1").arg(avErrorString(ret));
        return false;
    }

    ret = avcodec_parameters_from_context(stream->codecpar, m_audioCodecContext);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy audio encoder parameters: %1").arg(avErrorString(ret));
        return false;
    }
    stream->time_base = m_audioCodecContext->time_base;

    m_audioFrame = av_frame_alloc();
    if (!m_audioFrame)
    {
        errorMessage = QStringLiteral("Cannot allocate YouTube stream audio frame");
        return false;
    }
    m_audioFrame->format = m_audioCodecContext->sample_fmt;
    m_audioFrame->channel_layout = m_audioCodecContext->channel_layout;
    m_audioFrame->sample_rate = m_audioCodecContext->sample_rate;
    m_audioFrame->nb_samples = m_audioCodecContext->frame_size > 0 ? m_audioCodecContext->frame_size : 1024;

    ret = av_frame_get_buffer(m_audioFrame, 0);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot allocate YouTube stream audio buffer: %1").arg(avErrorString(ret));
        return false;
    }

    return true;
#endif
}

bool CameraYouTubeStreamer::writeEncodedPacket(AVPacket *packet, AVCodecContext *codecContext, int streamIndex, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(packet)
    Q_UNUSED(codecContext)
    Q_UNUSED(streamIndex)
    Q_UNUSED(errorMessage)
    return false;
#else
    packet->stream_index = streamIndex;
    av_packet_rescale_ts(packet, codecContext->time_base, m_formatContext->streams[streamIndex]->time_base);
    if (packet->duration <= 0) {
        packet->duration = av_rescale_q(1, codecContext->time_base, m_formatContext->streams[streamIndex]->time_base);
    }
    const int ret = av_interleaved_write_frame(m_formatContext, packet);
    if ((ret >= 0) && m_formatContext->pb) {
        avio_flush(m_formatContext->pb);
    }
    av_packet_unref(packet);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write YouTube stream packet: %1").arg(avErrorString(ret));
        return false;
    }

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
        if (!writeEncodedPacket(packet, m_codecContext, m_streamIndex, errorMessage))
        {
            ok = false;
            break;
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

bool CameraYouTubeStreamer::fillAudioFrameFromS16Stereo(const char *pcm, int sampleFrames, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(pcm)
    Q_UNUSED(sampleFrames)
    Q_UNUSED(errorMessage)
    return false;
#else
    if (!m_audioFrame || !m_audioCodecContext) {
        return false;
    }
    if (sampleFrames != m_audioFrame->nb_samples)
    {
        errorMessage = QStringLiteral("Audio frame size mismatch");
        return false;
    }

    const qint16 *samples = reinterpret_cast<const qint16*>(pcm);

    switch (m_audioCodecContext->sample_fmt)
    {
    case AV_SAMPLE_FMT_FLTP:
    {
        float *left = reinterpret_cast<float*>(m_audioFrame->data[0]);
        float *right = reinterpret_cast<float*>(m_audioFrame->data[1]);
        for (int i = 0; i < sampleFrames; ++i)
        {
            left[i] = samples[i * 2] / 32768.0f;
            right[i] = samples[i * 2 + 1] / 32768.0f;
        }
        return true;
    }
    case AV_SAMPLE_FMT_S16P:
    {
        qint16 *left = reinterpret_cast<qint16*>(m_audioFrame->data[0]);
        qint16 *right = reinterpret_cast<qint16*>(m_audioFrame->data[1]);
        for (int i = 0; i < sampleFrames; ++i)
        {
            left[i] = samples[i * 2];
            right[i] = samples[i * 2 + 1];
        }
        return true;
    }
    case AV_SAMPLE_FMT_S16:
        std::memcpy(m_audioFrame->data[0], pcm, static_cast<size_t>(sampleFrames) * 4U);
        return true;
    default:
        errorMessage = QStringLiteral("Unsupported AAC sample format %1").arg(av_get_sample_fmt_name(m_audioCodecContext->sample_fmt));
        return false;
    }
#endif
}

bool CameraYouTubeStreamer::encodeAndWriteAudioFrame(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    m_audioFrame->pts = m_audioFrameIndex;
    m_audioFrameIndex += m_audioFrame->nb_samples;

    int ret = avcodec_send_frame(m_audioCodecContext, m_audioFrame);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot encode YouTube stream audio frame: %1").arg(avErrorString(ret));
        return false;
    }

    AVPacket *packet = av_packet_alloc();
    if (!packet)
    {
        errorMessage = QStringLiteral("Cannot allocate YouTube stream audio packet");
        return false;
    }

    bool ok = true;
    while ((ret = avcodec_receive_packet(m_audioCodecContext, packet)) >= 0)
    {
        if (!writeEncodedPacket(packet, m_audioCodecContext, m_audioStreamIndex, errorMessage))
        {
            ok = false;
            break;
        }
    }

    if (ok && (ret != AVERROR(EAGAIN)) && (ret != AVERROR_EOF))
    {
        errorMessage = QStringLiteral("Cannot receive YouTube stream audio packet: %1").arg(avErrorString(ret));
        ok = false;
    }

    av_packet_free(&packet);
    return ok;
#endif
}

bool CameraYouTubeStreamer::encodeAndWriteSilentAudioFrame(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    int ret = av_frame_make_writable(m_audioFrame);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write to YouTube stream audio buffer: %1").arg(avErrorString(ret));
        return false;
    }

    ret = av_samples_set_silence(
        m_audioFrame->data,
        0,
        m_audioFrame->nb_samples,
        m_audioCodecContext->channels,
        m_audioCodecContext->sample_fmt);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot clear YouTube stream audio buffer: %1").arg(avErrorString(ret));
        return false;
    }

    return encodeAndWriteAudioFrame(errorMessage);
#endif
}

bool CameraYouTubeStreamer::writePcmS16Stereo(const QByteArray& pcm, int sampleRate, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(pcm)
    Q_UNUSED(sampleRate)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!m_audioCodecContext || !m_audioFrame) {
        return true;
    }
    if (!pcm.isEmpty())
    {
        QByteArray streamPcm;
        if (!CameraFFmpegAudio::resamplePcmS16Stereo(
                pcm,
                sampleRate,
                m_audioCodecContext->sample_rate,
                streamPcm,
                errorMessage)) {
            return false;
        }
        m_audioInputBuffer.append(streamPcm);
    }
    const int bytesPerSampleFrame = 4;
    const int audioFrameBytes = m_audioFrame->nb_samples * bytesPerSampleFrame;

    while (m_audioInputBuffer.size() >= audioFrameBytes)
    {
        int ret = av_frame_make_writable(m_audioFrame);
        if (ret < 0)
        {
            errorMessage = QStringLiteral("Cannot write to YouTube stream audio buffer: %1").arg(avErrorString(ret));
            return false;
        }
        if (!fillAudioFrameFromS16Stereo(m_audioInputBuffer.constData(), m_audioFrame->nb_samples, errorMessage)) {
            return false;
        }
        m_audioInputBuffer.remove(0, audioFrameBytes);

        if (!encodeAndWriteAudioFrame(errorMessage)) {
            return false;
        }
    }

    return true;
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

    if (!m_audioCodecContext || !m_audioFrame) {
        return true;
    }

    qint64 currentAudioMs = av_rescale_q(
        m_audioFrameIndex,
        m_audioCodecContext->time_base,
        AVRational{1, 1000});
    int audioFramesWritten = 0;

    while ((currentAudioMs <= elapsedMs + framePeriodMs) && (audioFramesWritten < fps * 2))
    {
        if (m_audioInputBuffer.size() >= (m_audioFrame->nb_samples * 4))
        {
            if (!writePcmS16Stereo(QByteArray(), m_audioCodecContext->sample_rate, errorMessage)) {
                return false;
            }
        }
        else
        {
            if (!encodeAndWriteSilentAudioFrame(errorMessage)) {
                return false;
            }
        }
        currentAudioMs = av_rescale_q(
            m_audioFrameIndex,
            m_audioCodecContext->time_base,
            AVRational{1, 1000});
        ++audioFramesWritten;
    }

    return true;
#endif
}

void CameraYouTubeStreamer::flushEncoder(AVCodecContext *codecContext, int streamIndex)
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (!codecContext || !m_formatContext || (streamIndex < 0)) {
        return;
    }

    avcodec_send_frame(codecContext, nullptr);
    AVPacket *packet = av_packet_alloc();
    if (packet)
    {
        QString ignoredError;
        while (avcodec_receive_packet(codecContext, packet) >= 0) {
            const bool ignoredOk = writeEncodedPacket(packet, codecContext, streamIndex, ignoredError);
            Q_UNUSED(ignoredOk)
        }
        av_packet_free(&packet);
    }
#endif
}

void CameraYouTubeStreamer::close()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_headerWritten && m_formatContext)
    {
        flushEncoder(m_codecContext, m_streamIndex);
        flushEncoder(m_audioCodecContext, m_audioStreamIndex);
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
    if (m_audioCodecContext) {
        avcodec_free_context(&m_audioCodecContext);
    }
    if (m_frame) {
        av_frame_free(&m_frame);
    }
    if (m_audioFrame) {
        av_frame_free(&m_audioFrame);
    }
    if (m_swsContext)
    {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
    }
#endif
    m_streamIndex = -1;
    m_audioStreamIndex = -1;
    m_frameIndex = 0;
    m_audioFrameIndex = 0;
    m_audioInputBuffer.clear();
    m_lastFrameElapsedMs = -1;
    m_nextFrameElapsedMs = 0;
    m_headerWritten = false;
    m_streamSize = QSize();
}
