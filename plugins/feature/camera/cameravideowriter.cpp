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
#include <array>
#include <cmath>
#include <cstring>

#include <QByteArray>
#include <QDebug>
#include <QPainter>
#include <QStringList>

#include "cameraffmpegcompat.h"

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

#ifdef CAMERA_FFMPEG_STREAMING
namespace {

QStringList cameraVideoWriterOptionConstants(void *privData, const char *optionName)
{
    QStringList constants;
    const AVOption *option = av_opt_find(privData, optionName, nullptr, 0, 0);
    if (!option || !option->unit) {
        return constants;
    }

    const AVOption *current = nullptr;
    while ((current = av_opt_next(privData, current)) != nullptr)
    {
        if ((current->type == AV_OPT_TYPE_CONST)
            && current->name
            && current->unit
            && (std::strcmp(current->unit, option->unit) == 0))
        {
            constants.append(QString::fromLatin1(current->name));
        }
    }
    return constants;
}

QStringList cameraVideoWriterEncoderOptionConstants(const AVCodec *codec, const char *optionName)
{
    AVCodecContext *context = avcodec_alloc_context3(codec);
    if (!context) {
        return QStringList();
    }

    QStringList constants;
    if (context->priv_data) {
        constants = cameraVideoWriterOptionConstants(context->priv_data, optionName);
    }
    avcodec_free_context(&context);
    return constants;
}

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
    const AVRational timeBase{fpsDen, fpsNum};
    const AVRational frameRate{fpsNum, fpsDen};
    const qint64 frameDurationPts = 1;

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

    const auto tryOpenEncoder = [&](const AVCodec *candidateCodec, const char *nvencPreset, QString& openError) -> bool
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
        m_codecContext->bit_rate = settings.m_bitrateKbps > 0
            ? static_cast<int64_t>(settings.m_bitrateKbps) * 1000
            : recordingBitrateBps(videoSize, requestedFps);
        if (settings.m_bitrateKbps > 0)
        {
            m_codecContext->rc_min_rate = m_codecContext->bit_rate;
            m_codecContext->rc_max_rate = m_codecContext->bit_rate;
            m_codecContext->rc_buffer_size = static_cast<int>(std::min<int64_t>(
                std::numeric_limits<int>::max(),
                std::max<int64_t>(m_codecContext->bit_rate, m_codecContext->bit_rate * 2)));
            m_codecContext->bit_rate_tolerance = static_cast<int>(std::min<int64_t>(
                std::numeric_limits<int>::max(),
                std::max<int64_t>(1, m_codecContext->bit_rate / 20)));
        }

        if (m_formatContext->oformat->flags & AVFMT_GLOBALHEADER) {
            m_codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        }

        const QString encoderName = QString::fromLatin1(candidateCodec->name);
        const bool nvencEncoder = encoderName.contains(QStringLiteral("nvenc"), Qt::CaseInsensitive);
        if (nvencEncoder)
        {
            if (nvencPreset && *nvencPreset)
            {
                const int presetRet = av_opt_set(m_codecContext->priv_data, "preset", nvencPreset, 0);
                if (presetRet < 0)
                {
                    openError = QStringLiteral("Cannot set %1 encoder %2 preset %3: %4")
                        .arg(codecName(settings.m_codec), encoderName, QString::fromLatin1(nvencPreset), avErrorString(presetRet));
                    avcodec_free_context(&m_codecContext);
                    return false;
                }
            }
        }
        else
        {
            av_opt_set(m_codecContext->priv_data, "preset", "veryfast", 0);
            if (settings.m_bitrateKbps > 0) {
                av_opt_set(m_codecContext->priv_data, "nal-hrd", "vbr", 0);
            }
        }

        const int openRet = avcodec_open2(m_codecContext, candidateCodec, nullptr);
        if (openRet < 0)
        {
            openError = nvencEncoder && nvencPreset && *nvencPreset
                ? QStringLiteral("Cannot open %1 encoder %2 preset %3: %4")
                    .arg(codecName(settings.m_codec), encoderName, QString::fromLatin1(nvencPreset), avErrorString(openRet))
                : QStringLiteral("Cannot open %1 encoder %2: %3")
                    .arg(codecName(settings.m_codec), encoderName, avErrorString(openRet));
            avcodec_free_context(&m_codecContext);
            return false;
        }

        return true;
    };

    QString openError;
    const QString selectedEncoderName = QString::fromLatin1(codec->name);
    const bool selectedHardwareEncoder = selectedEncoderName.contains(QStringLiteral("nvenc"), Qt::CaseInsensitive);
    bool encoderOpen = false;
    if (selectedHardwareEncoder)
    {
        const QStringList supportedNvencPresets = cameraVideoWriterEncoderOptionConstants(codec, "preset");
        static constexpr std::array<const char*, 10> kNvencPresetFallbacks = {{
            // Newer FFmpeg/NVENC presets. Older FFmpeg builds reject these at
            // av_opt_set(), so keep the legacy names below as fallbacks.
            "p4",
            "p3",
            "p2",
            "p1",
            "p5",
            "p6",
            // Legacy FFmpeg/NVENC presets.
            "default",
            "fast",
            "hp",
            "hq"
        }};
        for (const char *preset : kNvencPresetFallbacks)
        {
            if (!supportedNvencPresets.isEmpty()
                && !supportedNvencPresets.contains(QString::fromLatin1(preset)))
            {
                continue;
            }
            if (tryOpenEncoder(codec, preset, openError))
            {
                qDebug() << "CameraVideoWriter: opened NVENC encoder"
                         << selectedEncoderName
                         << "preset" << preset;
                encoderOpen = true;
                break;
            }
        }
    }
    else
    {
        encoderOpen = tryOpenEncoder(codec, nullptr, openError);
    }

    if (!encoderOpen)
    {
        const QString failedEncoderName = QString::fromLatin1(codec->name);
        const bool failedHardwareEncoder = failedEncoderName.contains(QStringLiteral("nvenc"), Qt::CaseInsensitive);
        const AVCodec *softwareCodec = failedHardwareEncoder ? avcodec_find_encoder_by_name(softwareEncoderName) : nullptr;
        if (softwareCodec && (softwareCodec != codec))
        {
            qWarning() << "CameraVideoWriter: recording hardware acceleration requested but encoder"
                       << failedEncoderName
                       << "failed; falling back to software encoder"
                       << softwareEncoderName
                       << ":" << openError;
            if (!tryOpenEncoder(softwareCodec, nullptr, openError))
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
        // Non-fatal: the shared writer cleans up after itself on failure, so the file is muxed
        // video-only if the AAC stream can't be created.
        if (!m_audioWriter.open(m_formatContext, kCameraVideoWriterAudioSampleRate, /*flushAfterWrite=*/false, audioWarning)) {
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

    qDebug() << "CameraVideoWriter: opened" << fileName << codecName(settings.m_codec)
             << videoSize << "fps" << requestedFps << "bitrateKbps" << (m_codecContext->bit_rate / 1000)
             << "encoder" << codec->name;
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
        const qint64 duration = codecContext == m_codecContext ? m_frameDurationPts : 1;
        packet->duration = av_rescale_q(duration, codecContext->time_base, m_formatContext->streams[streamIndex]->time_base);
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

bool CameraVideoWriter::writePcmS16Stereo(const QByteArray& pcm, int sampleRate, QString& errorMessage)
{
    return m_audioWriter.writePcmS16Stereo(pcm, sampleRate, errorMessage);
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
        QString ignoredAudioError;
        const bool ignoredAudioOk = m_audioWriter.flush(ignoredAudioError);   // drain resampler + AAC encoder
        Q_UNUSED(ignoredAudioOk)
        flushEncoder(m_codecContext, m_streamIndex);
    }

    if (m_formatContext)
    {
        if (m_headerWritten) {
            const qint64 durationMs = m_codecContext && (m_lastVideoPts >= 0)
                ? av_rescale_q(m_lastVideoPts + m_frameDurationPts, m_codecContext->time_base, AVRational{1, 1000})
                : 0;
            qDebug() << "CameraVideoWriter: closing" << m_settings.m_fileName
                     << "frames" << (m_frameDurationPts > 0 ? m_frameIndex / m_frameDurationPts : m_frameIndex)
                     << "durationMs" << durationMs
                     << "fps" << m_settings.m_fps
                     << "bitrateKbps" << m_settings.m_bitrateKbps;
            av_write_trailer(m_formatContext);
        }
        if (!(m_formatContext->oformat->flags & AVFMT_NOFILE) && m_formatContext->pb) {
            avio_closep(&m_formatContext->pb);
        }
        avformat_free_context(m_formatContext);
        m_formatContext = nullptr;
    }
    m_audioWriter.close();   // frees the AAC codec context + frame (does not touch m_formatContext)
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
