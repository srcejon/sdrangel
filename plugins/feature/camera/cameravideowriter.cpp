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

#include "cameraffmpegaudio.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <QByteArray>
#include <QDebug>
#include <QPainter>

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswscale/swscale.h>
}
#endif

static constexpr int kCameraVideoWriterAudioSampleRate = 48000;

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

static qint64 recordingBitrateBps(const QSize& size, double fps)
{
    const int width = size.width();
    const int height = size.height();
    const bool highFrameRate = fps > 30.0;
    int bitrateKbps;

    if ((width >= 7680) || (height >= 4320)) {
        bitrateKbps = highFrameRate ? 240000 : 160000;
    } else if ((width >= 3840) || (height >= 2160)) {
        bitrateKbps = highFrameRate ? 68000 : 45000;
    } else if ((width >= 2560) || (height >= 1440)) {
        bitrateKbps = highFrameRate ? 24000 : 16000;
    } else if ((width >= 1920) || (height >= 1080)) {
        bitrateKbps = highFrameRate ? 12000 : 8000;
    } else if ((width >= 1280) || (height >= 720)) {
        bitrateKbps = highFrameRate ? 7500 : 5000;
    } else if ((width >= 854) || (height >= 480)) {
        bitrateKbps = highFrameRate ? 4000 : 2500;
    } else {
        bitrateKbps = highFrameRate ? 1500 : 1000;
    }

    return static_cast<qint64>(bitrateKbps) * 1000;
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
    const AVRational timeBase{1, 1000};
    const AVRational frameRate{fpsNum, fpsDen};
    const qint64 frameDurationPts = std::max<qint64>(1, static_cast<qint64>((1000.0 / requestedFps) + 0.5));

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

    const auto tryOpenEncoder = [&](const AVCodec *candidateCodec, QString& openError) -> bool
    {
        m_codecContext = avcodec_alloc_context3(candidateCodec);
        if (!m_codecContext)
        {
            openError = QStringLiteral("Cannot allocate %1 encoder context").arg(codecName(settings.m_codec));
            return false;
        }

        m_codecContext->codec_id = candidateCodec->id;
        m_codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
        m_codecContext->width = videoSize.width();
        m_codecContext->height = videoSize.height();
        m_codecContext->time_base = timeBase;
        m_codecContext->framerate = frameRate;
        m_codecContext->gop_size = std::max(1, static_cast<int>(std::llround(requestedFps * 2.0)));
        m_codecContext->max_b_frames = 0;
        m_codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
        m_codecContext->bit_rate = recordingBitrateBps(videoSize, requestedFps);

        if (m_formatContext->oformat->flags & AVFMT_GLOBALHEADER) {
            m_codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        }

        const QString encoderName = QString::fromLatin1(candidateCodec->name);
        const bool nvencEncoder = encoderName.contains(QStringLiteral("nvenc"), Qt::CaseInsensitive);
        if (!nvencEncoder)
        {
            av_opt_set(m_codecContext->priv_data, "preset", "veryfast", 0);
            if (settings.m_codec == CameraSettings::VideoCodecH264) {
                av_opt_set(m_codecContext->priv_data, "tune", "zerolatency", 0);
            }
        }

        const int openRet = avcodec_open2(m_codecContext, candidateCodec, nullptr);
        if (openRet < 0)
        {
            openError = QStringLiteral("Cannot open %1 encoder %2: %3")
                .arg(codecName(settings.m_codec), encoderName, avErrorString(openRet));
            avcodec_free_context(&m_codecContext);
            return false;
        }

        return true;
    };

    QString openError;
    if (!tryOpenEncoder(codec, openError))
    {
        const QString failedEncoderName = QString::fromLatin1(codec->name);
        const bool failedHardwareEncoder = failedEncoderName.contains(QStringLiteral("nvenc"), Qt::CaseInsensitive);
        const AVCodec *softwareCodec = failedHardwareEncoder ? avcodec_find_encoder_by_name(softwareEncoderName) : nullptr;
        if (softwareCodec && (softwareCodec != codec))
        {
            qWarning() << "CameraVideoWriter: hardware encoder failed, retrying software encoder:" << openError;
            if (!tryOpenEncoder(softwareCodec, openError))
            {
                errorMessage = openError;
                close();
                return false;
            }
            codec = softwareCodec;
        }
        else
        {
            errorMessage = openError;
            close();
            return false;
        }
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

    if (settings.m_audioEnabled)
    {
        QString audioWarning;
        if (!openAudioStream(audioWarning)) {
            qWarning() << "CameraVideoWriter: audio disabled:" << audioWarning;
        }
    }

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
    m_frameDurationPts = frameDurationPts;
    m_firstFrameTimestampMs = -1;
    m_lastVideoPts = -1;
    m_audioFrameIndex = 0;

    qDebug() << "CameraVideoWriter: opened" << fileName << codecName(settings.m_codec)
             << videoSize << "fps" << requestedFps << "encoder" << codec->name;
    return true;
#endif
}

bool CameraVideoWriter::openAudioStream(QString& warningMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    warningMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
    if (!codec)
    {
        warningMessage = QStringLiteral("No AAC encoder is available in FFmpeg");
        return false;
    }

    AVStream *stream = avformat_new_stream(m_formatContext, nullptr);
    if (!stream)
    {
        warningMessage = QStringLiteral("Cannot create audio stream");
        return false;
    }
    m_audioStreamIndex = stream->index;

    m_audioCodecContext = avcodec_alloc_context3(codec);
    if (!m_audioCodecContext)
    {
        warningMessage = QStringLiteral("Cannot allocate AAC encoder context");
        return false;
    }

    m_audioCodecContext->codec_id = codec->id;
    m_audioCodecContext->codec_type = AVMEDIA_TYPE_AUDIO;
    m_audioCodecContext->sample_rate = kCameraVideoWriterAudioSampleRate;
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
        warningMessage = QStringLiteral("Cannot open AAC encoder: %1").arg(avErrorString(ret));
        avcodec_free_context(&m_audioCodecContext);
        m_audioStreamIndex = -1;
        return false;
    }

    ret = avcodec_parameters_from_context(stream->codecpar, m_audioCodecContext);
    if (ret < 0)
    {
        warningMessage = QStringLiteral("Cannot copy audio encoder parameters: %1").arg(avErrorString(ret));
        avcodec_free_context(&m_audioCodecContext);
        m_audioStreamIndex = -1;
        return false;
    }
    stream->time_base = m_audioCodecContext->time_base;

    m_audioFrame = av_frame_alloc();
    if (!m_audioFrame)
    {
        warningMessage = QStringLiteral("Cannot allocate audio frame");
        avcodec_free_context(&m_audioCodecContext);
        m_audioStreamIndex = -1;
        return false;
    }
    m_audioFrame->format = m_audioCodecContext->sample_fmt;
    m_audioFrame->channel_layout = m_audioCodecContext->channel_layout;
    m_audioFrame->sample_rate = m_audioCodecContext->sample_rate;
    m_audioFrame->nb_samples = m_audioCodecContext->frame_size > 0 ? m_audioCodecContext->frame_size : 1024;

    ret = av_frame_get_buffer(m_audioFrame, 0);
    if (ret < 0)
    {
        warningMessage = QStringLiteral("Cannot allocate audio frame buffer: %1").arg(avErrorString(ret));
        av_frame_free(&m_audioFrame);
        avcodec_free_context(&m_audioCodecContext);
        m_audioStreamIndex = -1;
        return false;
    }

    return true;
#endif
}

bool CameraVideoWriter::writeEncodedPacket(AVPacket *packet, AVCodecContext *codecContext, int streamIndex, QString& errorMessage)
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
    av_packet_unref(packet);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write video packet: %1").arg(avErrorString(ret));
        return false;
    }

    return true;
#endif
}

bool CameraVideoWriter::fillAudioFrameFromS16Stereo(const char *pcm, int sampleFrames, QString& errorMessage)
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

bool CameraVideoWriter::encodeAndWriteAudioFrame(QString& errorMessage)
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
        errorMessage = QStringLiteral("Cannot encode audio frame: %1").arg(avErrorString(ret));
        return false;
    }

    AVPacket *packet = av_packet_alloc();
    if (!packet)
    {
        errorMessage = QStringLiteral("Cannot allocate audio packet");
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
        errorMessage = QStringLiteral("Cannot receive audio packet: %1").arg(avErrorString(ret));
        ok = false;
    }

    av_packet_free(&packet);
    return ok;
#endif
}

bool CameraVideoWriter::writePcmS16Stereo(const QByteArray& pcm, int sampleRate, QString& errorMessage)
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
        QByteArray writerPcm;
        if (!CameraFFmpegAudio::resamplePcmS16Stereo(
                pcm,
                sampleRate,
                m_audioCodecContext->sample_rate,
                writerPcm,
                errorMessage)) {
            return false;
        }
        m_audioInputBuffer.append(writerPcm);
    }
    const int bytesPerSampleFrame = 4;
    const int audioFrameBytes = m_audioFrame->nb_samples * bytesPerSampleFrame;

    while (m_audioInputBuffer.size() >= audioFrameBytes)
    {
        int ret = av_frame_make_writable(m_audioFrame);
        if (ret < 0)
        {
            errorMessage = QStringLiteral("Cannot write to audio frame buffer: %1").arg(avErrorString(ret));
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

bool CameraVideoWriter::writeFrame(const QImage& image, QString& errorMessage, qint64 timestampMs)
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

    qint64 pts = m_frameIndex;
    if (timestampMs >= 0)
    {
        if (m_firstFrameTimestampMs < 0) {
            m_firstFrameTimestampMs = timestampMs;
        }
        pts = av_rescale_q(
            std::max<qint64>(0, timestampMs - m_firstFrameTimestampMs),
            AVRational{1, 1000},
            m_codecContext->time_base);
        if (pts <= m_lastVideoPts) {
            pts = m_lastVideoPts + 1;
        }
    }

    m_frame->pts = pts;
    m_lastVideoPts = pts;
    m_frameIndex = pts + m_frameDurationPts;

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
        if (!writeEncodedPacket(packet, m_codecContext, m_streamIndex, errorMessage))
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

void CameraVideoWriter::flushEncoder(AVCodecContext *codecContext, int streamIndex)
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

void CameraVideoWriter::close()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_headerWritten && m_formatContext) {
        flushEncoder(m_codecContext, m_streamIndex);
        flushEncoder(m_audioCodecContext, m_audioStreamIndex);
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
    m_headerWritten = false;
    m_videoSize = QSize();
}
