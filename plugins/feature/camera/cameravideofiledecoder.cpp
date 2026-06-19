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

#include "cameravideofiledecoder.h"

#include <algorithm>
#include <cmath>

#include <QElapsedTimer>
#include <QDebug>
#include <QMutexLocker>
#include <QStringList>
#include <QUrl>

#include "cameraffmpegaudio.h"

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}
#endif

namespace {

#ifdef CAMERA_FFMPEG_STREAMING
int cameraVideoFileDecoderInterruptCallback(void *opaque)
{
    CameraVideoFileDecoder *decoder = static_cast<CameraVideoFileDecoder*>(opaque);
    return decoder ? (decoder->abortRequested() ? 1 : 0) : 0;
}
#endif

}

CameraVideoFileDecoder::CameraVideoFileDecoder()
{
}

CameraVideoFileDecoder::~CameraVideoFileDecoder()
{
    close();
}

bool CameraVideoFileDecoder::isOpen() const
{
    return m_formatContext && m_videoCodecContext && (m_videoStreamIndex >= 0);
}

bool CameraVideoFileDecoder::open(
    const QString& fileName,
    QString& errorMessage,
    int outputSampleRate)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(fileName)
    Q_UNUSED(outputSampleRate)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    close();
    m_abortRequested.store(false);
    m_outputSampleRate = std::max(1000, outputSampleRate);

    const QByteArray fileNameUtf8 = fileName.toUtf8();
    avformat_network_init();

    const QString scheme = QUrl(fileName).scheme().toLower();
    m_urlSource = scheme.size() > 1;
    const bool rtspSource = scheme == QLatin1String("rtsp");
    QStringList openErrors;

    const auto openInput = [&](const char *rtspTransport) -> bool
    {
        QElapsedTimer openTimer;
        openTimer.start();
        const QString transportName = rtspTransport && rtspTransport[0] != '\0'
            ? QString::fromLatin1(rtspTransport)
            : QStringLiteral("auto");

        AVDictionary *options = nullptr;
        if (!scheme.isEmpty())
        {
            av_dict_set(&options, "analyzeduration", "10000000", 0);
            av_dict_set(&options, "probesize", "5000000", 0);
            av_dict_set(&options, "rw_timeout", "5000000", 0);
            if ((scheme == QLatin1String("http")) || (scheme == QLatin1String("https"))) {
                av_dict_set(&options, "flv_ignore_prevtag", "1", 0);
                // NB: do NOT enable FFmpeg's byte-level `reconnect`/`reconnect_streamed`
                // here. For a live FLV-over-HTTP stream the server restarts the FLV
                // stream (fresh `FLV\x01` header) on a re-GET, and FFmpeg splices it
                // mid-stream, so the flv demuxer never re-aligns (manifests as
                // "Audio codec (f) is not implemented" / "Invalid NAL unit size" /
                // "Error splitting the input into NAL units") and never recovers.
                // Recovery from a dropped connection is handled by fully reopening
                // the decoder in the worker decode thread, which re-reads the FLV
                // header cleanly from the current live edge.
            }
        }
        if (rtspSource)
        {
            // Do NOT set the RTSP demuxer's `timeout` option here. For the RTSP
            // demuxer `timeout` is the *incoming-connection* (listen) timeout and
            // it implies rtsp_flags=listen, i.e. it puts FFmpeg into RTSP SERVER
            // mode waiting for a client to connect to us. That breaks pulling
            // from a server like MediaMTX: the demuxer rejects the server's
            // responses as a client would never see them ("Unexpected command in
            // Idle State DESCRIBE" / "RTSP: Unexpected Command") and the open
            // blocks for ~2 min then fails with "Protocol not found". The
            // client-side socket I/O timeout is `stimeout` (microseconds), which
            // does NOT imply listen. prefer_tcp keeps us a TCP client even on the
            // auto/fallback attempt.
            av_dict_set(&options, "stimeout", "5000000", 0);
            av_dict_set(&options, "rtsp_flags", "prefer_tcp", 0);
            if (rtspTransport && rtspTransport[0] != '\0') {
                av_dict_set(&options, "rtsp_transport", rtspTransport, 0);
            }
        }

        qDebug() << "CameraVideoFileDecoder: opening media source" << fileName
                 << "scheme" << scheme
                 << "rtspTransport" << (rtspSource ? transportName : QStringLiteral("n/a"));
        m_formatContext = avformat_alloc_context();
        if (!m_formatContext)
        {
            av_dict_free(&options);
            openErrors.append(QStringLiteral("%1: cannot allocate FFmpeg format context").arg(transportName));
            return false;
        }
        m_formatContext->interrupt_callback.callback = cameraVideoFileDecoderInterruptCallback;
        m_formatContext->interrupt_callback.opaque = this;

        int ret = avformat_open_input(&m_formatContext, fileNameUtf8.constData(), nullptr, &options);
        av_dict_free(&options);
        if (ret < 0)
        {
            const QString error = CameraFFmpegAudio::avErrorString(ret);
            qWarning() << "CameraVideoFileDecoder: open media source failed"
                       << fileName
                       << "rtspTransport" << (rtspSource ? transportName : QStringLiteral("n/a"))
                       << "elapsedMs" << openTimer.elapsed()
                       << error;
            openErrors.append(QStringLiteral("%1: %2").arg(transportName, error));
            avformat_close_input(&m_formatContext);
            return false;
        }

        qDebug() << "CameraVideoFileDecoder: media source opened"
                 << fileName
                 << "rtspTransport" << (rtspSource ? transportName : QStringLiteral("n/a"))
                 << "elapsedMs" << openTimer.elapsed();
        QElapsedTimer streamInfoTimer;
        streamInfoTimer.start();
        ret = avformat_find_stream_info(m_formatContext, nullptr);
        if (ret < 0)
        {
            const QString error = CameraFFmpegAudio::avErrorString(ret);
            qWarning() << "CameraVideoFileDecoder: read media stream info failed"
                       << fileName
                       << "rtspTransport" << (rtspSource ? transportName : QStringLiteral("n/a"))
                       << "elapsedMs" << streamInfoTimer.elapsed()
                       << error;
            openErrors.append(QStringLiteral("%1 stream info: %2").arg(transportName, error));
            avformat_close_input(&m_formatContext);
            return false;
        }

        qDebug() << "CameraVideoFileDecoder: media stream info ready"
                 << fileName
                 << "rtspTransport" << (rtspSource ? transportName : QStringLiteral("n/a"))
                 << "elapsedMs" << streamInfoTimer.elapsed();
        for (unsigned int streamIndex = 0; streamIndex < m_formatContext->nb_streams; ++streamIndex)
        {
            const AVStream *stream = m_formatContext->streams[streamIndex];
            const AVCodecParameters *codecParameters = stream ? stream->codecpar : nullptr;
            const AVMediaType mediaType = codecParameters ? codecParameters->codec_type : AVMEDIA_TYPE_UNKNOWN;
            const char *mediaTypeName = av_get_media_type_string(mediaType);
            const AVCodecDescriptor *codecDescriptor = codecParameters
                ? avcodec_descriptor_get(codecParameters->codec_id)
                : nullptr;
            qDebug() << "CameraVideoFileDecoder: stream"
                     << static_cast<int>(streamIndex)
                     << (mediaTypeName ? QString::fromLatin1(mediaTypeName) : QStringLiteral("unknown"))
                     << "codec" << (codecDescriptor && codecDescriptor->name
                         ? QString::fromLatin1(codecDescriptor->name)
                         : QString::number(codecParameters ? codecParameters->codec_id : AV_CODEC_ID_NONE))
                     << "timeBase" << (stream ? stream->time_base.num : 0) << "/" << (stream ? stream->time_base.den : 0)
                     << "sampleRate" << (codecParameters ? codecParameters->sample_rate : 0)
                     << "channels" << (codecParameters ? codecParameters->channels : 0)
                     << "size" << (codecParameters ? codecParameters->width : 0) << "x" << (codecParameters ? codecParameters->height : 0);
        }
        return true;
    };

    bool opened = false;
    if (rtspSource)
    {
        opened = openInput("tcp") || openInput("udp") || openInput("");
    }
    else
    {
        opened = openInput("");
    }

    if (!opened)
    {
        errorMessage = QStringLiteral("Cannot open media source: %1").arg(openErrors.join(QStringLiteral("; ")));
        qWarning() << "CameraVideoFileDecoder:" << errorMessage;
        close();
        return false;
    }

    int ret = av_find_best_stream(m_formatContext, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Media source has no video stream");
        close();
        return false;
    }
    m_videoStreamIndex = ret;

    if (!openVideoDecoder(errorMessage))
    {
        close();
        return false;
    }

    QString audioError;
    if (!openAudioDecoder(audioError))
    {
        if (audioError.contains(QStringLiteral("no audio stream"), Qt::CaseInsensitive)) {
            qDebug() << "CameraVideoFileDecoder:" << audioError;
        } else {
            qWarning() << "CameraVideoFileDecoder: audio decoder unavailable:" << audioError;
        }
        closeAudioDecoder();
    }

    m_videoFrame = av_frame_alloc();
    m_audioFrame = av_frame_alloc();
    m_packet = av_packet_alloc();
    if (!m_videoFrame || !m_audioFrame || !m_packet)
    {
        errorMessage = QStringLiteral("Cannot allocate media decode buffers");
        close();
        return false;
    }

    AVStream *stream = m_formatContext->streams[m_videoStreamIndex];
    if (m_formatContext->duration > 0) {
        m_durationMs = av_rescale_q(m_formatContext->duration, AVRational{1, AV_TIME_BASE}, AVRational{1, 1000});
    } else if (stream->duration > 0) {
        m_durationMs = av_rescale_q(stream->duration, stream->time_base, AVRational{1, 1000});
    }

    const AVRational rate = stream->avg_frame_rate.num > 0 ? stream->avg_frame_rate : stream->r_frame_rate;
    if ((rate.num > 0) && (rate.den > 0))
    {
        const double reportedFrameRate = av_q2d(rate);
        if (!scheme.isEmpty() && ((reportedFrameRate <= 0.0) || (reportedFrameRate > 120.0)))
        {
            qWarning() << "CameraVideoFileDecoder: ignoring implausible stream frame rate"
                       << reportedFrameRate << "for" << fileName;
            m_frameRate = 25.0;
        }
        else
        {
            m_frameRate = qBound(1.0, reportedFrameRate, 240.0);
        }
    }

    m_eof = false;
    m_videoDraining = false;
    m_audioDraining = false;
    return true;
#endif
}

void CameraVideoFileDecoder::close()
{
#ifdef CAMERA_FFMPEG_STREAMING
    requestAbort();
    if (m_swsContext)
    {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
    }
    m_swsSrcWidth = 0;
    m_swsSrcHeight = 0;
    m_swsSrcFormat = -1;
    closeAudioDecoder();
    if (m_videoFrame) {
        av_frame_free(&m_videoFrame);
    }
    if (m_audioFrame) {
        av_frame_free(&m_audioFrame);
    }
    if (m_packet) {
        av_packet_free(&m_packet);
    }
    clearPendingVideoPackets();
    if (m_videoCodecContext) {
        avcodec_free_context(&m_videoCodecContext);
    }
    if (m_formatContext) {
        avformat_close_input(&m_formatContext);
    }
#endif
    m_videoStreamIndex = -1;
    m_audioStreamIndex = -1;
    m_durationMs = 0;
    m_frameRate = 25.0;
    m_outputSampleRate = 48000;
    m_audioPaceFrameRate = 0.0;
    m_audioPaceRemainderFrames = 0.0;
    m_audioPaceFrameRateApplied = 0.0;
    m_eof = false;
    m_videoDraining = false;
    m_audioDraining = false;
    m_urlSource = false;
    m_audioDecodedPositionMs = -1;
    clearPendingAudio();
    m_pendingVideoFrames.clear();
    // Release pooled free buffers (frames still held downstream return their own
    // buffers when their last copy dies; the pool state outlives this via its
    // refcount). Prevents one source's free list lingering into the next.
    m_imagePool.clear();
}

void CameraVideoFileDecoder::requestAbort()
{
    m_abortRequested.store(true);
}

void CameraVideoFileDecoder::setAudioPaceFrameRate(double frameRate)
{
    // Called from the worker thread. Only publish the rate atomically; the decode
    // thread owns m_audioPaceRemainderFrames and resets it when it observes the
    // rate change (see takePacedAudio), so the worker never writes the remainder.
    m_audioPaceFrameRate.store(std::max(0.0, frameRate), std::memory_order_relaxed);
}

int CameraVideoFileDecoder::pendingAudioBytes() const
{
    QMutexLocker locker(&m_pendingAudioMutex);
    return m_pendingAudioPcm.size();
}

void CameraVideoFileDecoder::clearPendingAudio()
{
    QMutexLocker locker(&m_pendingAudioMutex);
    m_pendingAudioPcm.clear();
}

void CameraVideoFileDecoder::appendPendingAudio(const QByteArray& pcmS16Stereo)
{
    if (pcmS16Stereo.isEmpty()) {
        return;
    }

    QMutexLocker locker(&m_pendingAudioMutex);
    m_pendingAudioPcm.append(pcmS16Stereo);
}

void CameraVideoFileDecoder::seek(qint64 positionMs)
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
    if (av_seek_frame(m_formatContext, m_videoStreamIndex, timestamp, AVSEEK_FLAG_BACKWARD) >= 0)
    {
        avcodec_flush_buffers(m_videoCodecContext);
        if (m_audioCodecContext) {
            avcodec_flush_buffers(m_audioCodecContext);
        }
    }
    clearPendingVideoPackets();
    m_eof = false;
    m_videoDraining = false;
    m_audioDraining = false;
    clearPendingAudio();
    m_audioPaceRemainderFrames = 0.0;
    m_pendingVideoFrames.clear();
#else
    Q_UNUSED(positionMs)
#endif
}

bool CameraVideoFileDecoder::readNextFrame(
    QImage& image,
    qint64& positionMs,
    QByteArray& pcmS16Stereo,
    int& audioSampleRate,
    QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(image)
    Q_UNUSED(positionMs)
    Q_UNUSED(pcmS16Stereo)
    Q_UNUSED(audioSampleRate)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    image = QImage();
    positionMs = -1;
    pcmS16Stereo.clear();
    audioSampleRate = m_outputSampleRate;
    if (!isOpen() || m_eof) {
        return true;
    }

    if (!m_pendingVideoFrames.empty())
    {
        PendingVideoFrame pending = std::move(m_pendingVideoFrames.front());
        m_pendingVideoFrames.pop_front();
        image = std::move(pending.m_image);
        positionMs = pending.m_positionMs;
        QByteArray decodedAudio;
        return finishFrameAudio(decodedAudio, pcmS16Stereo, positionMs, errorMessage);
    }

    QByteArray decodedAudio;
    for (;;)
    {
        if (receiveVideoFrame(image, positionMs, errorMessage)) {
            return finishFrameAudio(decodedAudio, pcmS16Stereo, positionMs, errorMessage);
        }
        if (!errorMessage.isEmpty()) {
            return false;
        }

        int ret;
        if (!m_pendingVideoPackets.empty())
        {
            AVPacket *packet = m_pendingVideoPackets.front();
            m_pendingVideoPackets.pop_front();
            const bool sent = sendVideoPacket(packet, errorMessage);
            av_packet_free(&packet);
            if (!sent) {
                return false;
            }
            continue;
        }

        ret = av_read_frame(m_formatContext, m_packet);
        if (ret < 0)
        {
            if (ret == AVERROR_EOF)
            {
                if (!m_videoDraining)
                {
                    avcodec_send_packet(m_videoCodecContext, nullptr);
                    m_videoDraining = true;
                }
                if (m_audioCodecContext && !m_audioDraining)
                {
                    m_audioDraining = true;
                    if (!sendAudioPacket(nullptr, decodedAudio, errorMessage)) {
                        return false;
                    }
                }
                if (receiveVideoFrame(image, positionMs, errorMessage)) {
                    return finishFrameAudio(decodedAudio, pcmS16Stereo, positionMs, errorMessage);
                }
                if (!errorMessage.isEmpty()) {
                    return false;
                }
                m_eof = true;
                return true;
            }
            errorMessage = QStringLiteral("Cannot read video file packet: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        if (m_packet->stream_index == m_videoStreamIndex)
        {
            const bool sent = sendVideoPacket(m_packet, errorMessage);
            av_packet_unref(m_packet);
            if (!sent) {
                return false;
            }
        }
        else if (m_audioCodecContext && (m_packet->stream_index == m_audioStreamIndex))
        {
            const bool sent = sendAudioPacket(m_packet, decodedAudio, errorMessage);
            av_packet_unref(m_packet);
            if (!sent) {
                return false;
            }
        }
        else if (m_urlSource && isAudioStream(m_packet->stream_index))
        {
            if (isCompatibleAudioStream(m_packet->stream_index))
            {
                const bool sent = sendAudioPacket(m_packet, decodedAudio, errorMessage);
                av_packet_unref(m_packet);
                if (!sent) {
                    return false;
                }
            }
            else
            {
                av_packet_unref(m_packet);
            }
        }
        else
        {
            av_packet_unref(m_packet);
        }
    }
#endif
}

bool CameraVideoFileDecoder::readNextFrameAtOrAfter(
    qint64 targetPositionMs,
    QImage& image,
    qint64& positionMs,
    QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(targetPositionMs)
    Q_UNUSED(image)
    Q_UNUSED(positionMs)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    QByteArray discardedAudio;
    int discardedSampleRate = 0;
    const qint64 toleranceMs = std::max<qint64>(1, static_cast<qint64>((500.0 / std::max(1.0, m_frameRate)) + 0.5));

    for (;;)
    {
        if (!readNextFrame(image, positionMs, discardedAudio, discardedSampleRate, errorMessage)) {
            return false;
        }
        discardedAudio.clear();

        if (image.isNull() || (positionMs < 0) || (positionMs + toleranceMs >= targetPositionMs))
        {
            clearPendingAudio();
            m_audioPaceRemainderFrames = 0.0;
            m_audioDecodedPositionMs = -1;
            return true;
        }
    }
#endif
}

bool CameraVideoFileDecoder::openVideoDecoder(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    AVStream *stream = m_formatContext->streams[m_videoStreamIndex];
    const AVCodec *codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec)
    {
        errorMessage = QStringLiteral("No decoder is available for the video file stream");
        return false;
    }

    m_videoCodecContext = avcodec_alloc_context3(codec);
    if (!m_videoCodecContext)
    {
        errorMessage = QStringLiteral("Cannot allocate video file decoder context");
        return false;
    }

    int ret = avcodec_parameters_to_context(m_videoCodecContext, stream->codecpar);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy video file decoder parameters: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }

    m_videoCodecContext->thread_count = 0;
    m_videoCodecContext->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;

    ret = avcodec_open2(m_videoCodecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open video file decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }

    return true;
#endif
}

bool CameraVideoFileDecoder::openAudioDecoder(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    int ret = av_find_best_stream(m_formatContext, AVMEDIA_TYPE_AUDIO, -1, -1, nullptr, 0);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Video file has no audio stream");
        return false;
    }
    return openAudioDecoderForStream(ret, errorMessage);
#endif
}

bool CameraVideoFileDecoder::openAudioDecoderForStream(int streamIndex, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(streamIndex)
    Q_UNUSED(errorMessage)
    return false;
#else
    if (!m_formatContext || (streamIndex < 0) || (streamIndex >= static_cast<int>(m_formatContext->nb_streams)))
    {
        errorMessage = QStringLiteral("Invalid video file audio stream");
        return false;
    }

    AVStream *stream = m_formatContext->streams[streamIndex];
    if (!stream || !stream->codecpar || (stream->codecpar->codec_type != AVMEDIA_TYPE_AUDIO))
    {
        errorMessage = QStringLiteral("Selected media stream is not audio");
        return false;
    }

    const AVCodec *codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec)
    {
        errorMessage = QStringLiteral("No decoder is available for the video file audio stream");
        return false;
    }

    m_audioCodecContext = avcodec_alloc_context3(codec);
    if (!m_audioCodecContext)
    {
        errorMessage = QStringLiteral("Cannot allocate video file audio decoder context");
        return false;
    }

    int ret = avcodec_parameters_to_context(m_audioCodecContext, stream->codecpar);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy video file audio decoder parameters: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        avcodec_free_context(&m_audioCodecContext);
        return false;
    }

    ret = avcodec_open2(m_audioCodecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open video file audio decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        avcodec_free_context(&m_audioCodecContext);
        return false;
    }

    m_audioStreamIndex = streamIndex;
    qDebug() << "CameraVideoFileDecoder: opened audio stream"
             << m_audioStreamIndex
             << "codec" << (codec && codec->name ? QString::fromLatin1(codec->name) : QString())
             << "sampleRate" << m_audioCodecContext->sample_rate
             << "channels" << m_audioCodecContext->channels
             << "sampleFormat" << av_get_sample_fmt_name(m_audioCodecContext->sample_fmt)
             << "timeBase" << stream->time_base.num << "/" << stream->time_base.den;

    if (!openResampler(errorMessage))
    {
        avcodec_free_context(&m_audioCodecContext);
        m_audioStreamIndex = -1;
        return false;
    }
    return true;
#endif
}

bool CameraVideoFileDecoder::isAudioStream(int streamIndex) const
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(streamIndex)
    return false;
#else
    if (!m_formatContext || (streamIndex < 0) || (streamIndex >= static_cast<int>(m_formatContext->nb_streams))) {
        return false;
    }
    const AVStream *stream = m_formatContext->streams[streamIndex];
    return stream && stream->codecpar && (stream->codecpar->codec_type == AVMEDIA_TYPE_AUDIO);
#endif
}

bool CameraVideoFileDecoder::isCompatibleAudioStream(int streamIndex) const
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(streamIndex)
    return false;
#else
    if (!m_audioCodecContext || !isAudioStream(streamIndex)) {
        return false;
    }

    const AVCodecParameters *codecParameters = m_formatContext->streams[streamIndex]->codecpar;
    return (codecParameters->codec_id == m_audioCodecContext->codec_id)
        && (codecParameters->sample_rate == m_audioCodecContext->sample_rate)
        && (codecParameters->channels == m_audioCodecContext->channels)
        && ((codecParameters->channel_layout == 0)
            || (m_audioCodecContext->channel_layout == 0)
            || (codecParameters->channel_layout == m_audioCodecContext->channel_layout));
#endif
}

bool CameraVideoFileDecoder::sendAudioPacket(AVPacket *packet, QByteArray& pcmS16Stereo, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(packet)
    Q_UNUSED(pcmS16Stereo)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    static constexpr int maxDrainAttempts = 16;

    for (int attempt = 0; attempt <= maxDrainAttempts; ++attempt)
    {
        const int ret = avcodec_send_packet(m_audioCodecContext, packet);
        if (ret == 0) {
            return drainAudio(pcmS16Stereo, errorMessage);
        }

        if (ret != AVERROR(EAGAIN))
        {
            errorMessage = QStringLiteral("Cannot send video file audio packet to decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        if (!drainAudio(pcmS16Stereo, errorMessage)) {
            return false;
        }
    }

    errorMessage = QStringLiteral("Cannot send video file audio packet to decoder: decoder output queue did not drain");
    return false;
#endif
}

bool CameraVideoFileDecoder::openResampler(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    const int64_t inputChannelLayout = m_audioCodecContext->channel_layout != 0
        ? m_audioCodecContext->channel_layout
        : av_get_default_channel_layout(m_audioCodecContext->channels);
    m_resampler = swr_alloc_set_opts(
        nullptr,
        AV_CH_LAYOUT_STEREO,
        AV_SAMPLE_FMT_S16,
        m_outputSampleRate,
        inputChannelLayout,
        m_audioCodecContext->sample_fmt,
        m_audioCodecContext->sample_rate,
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

bool CameraVideoFileDecoder::sendVideoPacket(AVPacket *packet, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(packet)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    static constexpr int maxDrainAttempts = 16;

    for (int attempt = 0; attempt <= maxDrainAttempts; ++attempt)
    {
        const int ret = avcodec_send_packet(m_videoCodecContext, packet);
        if (ret == 0) {
            return true;
        }

        if (ret != AVERROR(EAGAIN))
        {
            errorMessage = QStringLiteral("Cannot send video file packet to decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        if (!queueOneDecodedVideoFrame(errorMessage)) {
            return false;
        }
    }

    errorMessage = QStringLiteral("Cannot send video file packet to decoder: decoder output queue did not drain");
    return false;
#endif
}

bool CameraVideoFileDecoder::receiveVideoFrame(QImage& image, qint64& positionMs, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(image)
    Q_UNUSED(positionMs)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    const int ret = avcodec_receive_frame(m_videoCodecContext, m_videoFrame);
    if (ret == 0)
    {
        const int64_t bestTimestamp = m_videoFrame->best_effort_timestamp;
        if (bestTimestamp != AV_NOPTS_VALUE) {
            positionMs = av_rescale_q(bestTimestamp, m_formatContext->streams[m_videoStreamIndex]->time_base, AVRational{1, 1000});
        }
        const bool ok = convertFrameToImage(m_videoFrame, image, errorMessage);
        av_frame_unref(m_videoFrame);
        return ok;
    }
    if (ret == AVERROR_EOF) {
        return false;
    }
    if (ret != AVERROR(EAGAIN)) {
        errorMessage = QStringLiteral("Cannot decode video file frame: %1").arg(CameraFFmpegAudio::avErrorString(ret));
    }
    return false;
#endif
}

bool CameraVideoFileDecoder::queueDecodedVideoFrames(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    for (;;)
    {
        const size_t maxPendingFrames = m_urlSource ? m_maxPendingStreamVideoFrames : m_maxPendingVideoFrames;
        if (m_pendingVideoFrames.size() >= maxPendingFrames) {
            return true;
        }

        QImage image;
        qint64 positionMs = -1;
        if (receiveVideoFrame(image, positionMs, errorMessage))
        {
            PendingVideoFrame pending;
            pending.m_image = std::move(image);
            pending.m_positionMs = positionMs;
            m_pendingVideoFrames.push_back(std::move(pending));
            continue;
        }
        return errorMessage.isEmpty();
    }
#endif
}

bool CameraVideoFileDecoder::queueOneDecodedVideoFrame(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    const size_t maxPendingFrames = m_urlSource ? m_maxPendingStreamVideoFrames : m_maxPendingVideoFrames;
    if (m_pendingVideoFrames.size() >= maxPendingFrames) {
        return true;
    }

    QImage image;
    qint64 positionMs = -1;
    if (receiveVideoFrame(image, positionMs, errorMessage))
    {
        PendingVideoFrame pending;
        pending.m_image = std::move(image);
        pending.m_positionMs = positionMs;
        m_pendingVideoFrames.push_back(std::move(pending));
        return true;
    }
    return errorMessage.isEmpty();
#endif
}

bool CameraVideoFileDecoder::finishFrameAudio(
    QByteArray& decodedAudio,
    QByteArray& pcmS16Stereo,
    qint64 videoPositionMs,
    QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(decodedAudio)
    Q_UNUSED(pcmS16Stereo)
    Q_UNUSED(videoPositionMs)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (m_urlSource)
    {
        appendPendingAudio(decodedAudio);
        const int maxFrames = m_outputSampleRate > 0 ? m_outputSampleRate : 48000;
        takePendingAudio(pcmS16Stereo, maxFrames);
        trimLivePendingAudio();
        return true;
    }
    if (!readAheadAudio(decodedAudio, videoPositionMs, errorMessage)) {
        return false;
    }
    appendPendingAudio(decodedAudio);
    takePacedAudio(pcmS16Stereo);
    trimLivePendingAudio();
    return true;
#endif
}

bool CameraVideoFileDecoder::readAheadAudio(QByteArray& pcmS16Stereo, qint64 videoPositionMs, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(pcmS16Stereo)
    Q_UNUSED(videoPositionMs)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!m_audioCodecContext) {
        return true;
    }

    static constexpr qint64 audioLeadMs = 50;
    static constexpr int bytesPerSampleFrame = 4;
    // Keep a small decoder-side cushion; the monitor FIFO is the main live
    // jitter buffer. Large per-frame read-ahead can block video on av_read_frame.
    static constexpr int streamTargetAudioMs = 250;
    static constexpr qint64 streamReadAheadBudgetMs = 12;
    const int maxPacketsRead = m_urlSource ? 96 : 32;
    const int frameAudioFrames = static_cast<int>((m_outputSampleRate / std::max(1.0, m_frameRate)) + 0.5);
    const int targetAudioFrames = m_urlSource
        ? std::max(frameAudioFrames, static_cast<int>((static_cast<qint64>(m_outputSampleRate) * streamTargetAudioMs) / 1000))
        : std::max(frameAudioFrames, m_outputSampleRate / 50);
    const int targetAudioBytes = targetAudioFrames * bytesPerSampleFrame;
    const qint64 targetAudioPositionMs = videoPositionMs >= 0
        ? videoPositionMs + audioLeadMs
        : -1;
    int packetsRead = 0;
    QElapsedTimer readAheadBudgetTimer;
    if (m_urlSource) {
        readAheadBudgetTimer.start();
    }

    auto needsAudio = [&]() {
        if (m_urlSource && ((pendingAudioBytes() + pcmS16Stereo.size()) < targetAudioBytes)) {
            return true;
        }
        if ((targetAudioPositionMs >= 0) && (m_audioDecodedPositionMs >= targetAudioPositionMs)) {
            return false;
        }
        return (pendingAudioBytes() + pcmS16Stereo.size()) < targetAudioBytes;
    };

    const size_t maxPendingVideoPackets = m_urlSource ? m_maxPendingStreamVideoPackets : m_maxPendingVideoPackets;
    const size_t maxPendingVideoFrames = m_urlSource ? m_maxPendingStreamVideoFrames : m_maxPendingVideoFrames;
    while (needsAudio()
        && !m_eof
        && (packetsRead < maxPacketsRead)
        && (m_pendingVideoPackets.size() < maxPendingVideoPackets)
        && (!m_urlSource || (m_pendingVideoFrames.size() < maxPendingVideoFrames)))
    {
        int ret = av_read_frame(m_formatContext, m_packet);
        if (ret < 0)
        {
            if (ret == AVERROR_EOF)
            {
                if (!m_videoDraining)
                {
                    avcodec_send_packet(m_videoCodecContext, nullptr);
                    m_videoDraining = true;
                }
                if (!m_audioDraining)
                {
                    m_audioDraining = true;
                    if (!sendAudioPacket(nullptr, pcmS16Stereo, errorMessage)) {
                        return false;
                    }
                }
                if (!queueDecodedVideoFrames(errorMessage)) {
                    return false;
                }
                m_eof = m_pendingVideoFrames.empty();
                return true;
            }
            errorMessage = QStringLiteral("Cannot read video file packet: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        if (m_packet->stream_index == m_audioStreamIndex)
        {
            const bool sent = sendAudioPacket(m_packet, pcmS16Stereo, errorMessage);
            av_packet_unref(m_packet);
            if (!sent) {
                return false;
            }
        }
        else if (m_urlSource && isAudioStream(m_packet->stream_index))
        {
            if (isCompatibleAudioStream(m_packet->stream_index))
            {
                const bool sent = sendAudioPacket(m_packet, pcmS16Stereo, errorMessage);
                av_packet_unref(m_packet);
                if (!sent) {
                    return false;
                }
            }
            else
            {
                av_packet_unref(m_packet);
            }
        }
        else if (m_packet->stream_index == m_videoStreamIndex)
        {
            if (m_urlSource)
            {
                const bool sent = sendVideoPacket(m_packet, errorMessage);
                av_packet_unref(m_packet);
                if (!sent) {
                    return false;
                }
                if (!queueOneDecodedVideoFrame(errorMessage)) {
                    return false;
                }
                ++packetsRead;
                if (!needsAudio() || (m_pendingVideoFrames.size() >= maxPendingVideoFrames)) {
                    return true;
                }
                if (readAheadBudgetTimer.isValid()
                    && (readAheadBudgetTimer.elapsed() >= streamReadAheadBudgetMs)
                    && ((pendingAudioBytes() + pcmS16Stereo.size()) >= (frameAudioFrames * bytesPerSampleFrame)))
                {
                    return true;
                }
                continue;
            }

            AVPacket *parkedPacket = av_packet_clone(m_packet);
            av_packet_unref(m_packet);
            if (!parkedPacket)
            {
                errorMessage = QStringLiteral("Cannot allocate parked video file packet");
                return false;
            }
            m_pendingVideoPackets.push_back(parkedPacket);
        }
        else
        {
            av_packet_unref(m_packet);
        }

        ++packetsRead;
        if (m_urlSource
            && readAheadBudgetTimer.isValid()
            && (readAheadBudgetTimer.elapsed() >= streamReadAheadBudgetMs)
            && ((pendingAudioBytes() + pcmS16Stereo.size()) >= (frameAudioFrames * bytesPerSampleFrame)))
        {
            return true;
        }
    }

    return true;
#endif
}

bool CameraVideoFileDecoder::drainAudio(QByteArray& pcmS16Stereo, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(pcmS16Stereo)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!m_audioCodecContext) {
        return true;
    }

    for (;;)
    {
        const int ret = avcodec_receive_frame(m_audioCodecContext, m_audioFrame);
        if (ret == 0)
        {
            const bool ok = appendFrameAudio(m_audioFrame, pcmS16Stereo, errorMessage);
            av_frame_unref(m_audioFrame);
            if (!ok) {
                return false;
            }
            continue;
        }
        if ((ret == AVERROR(EAGAIN)) || (ret == AVERROR_EOF)) {
            return true;
        }
        errorMessage = QStringLiteral("Cannot decode video file audio frame: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }
#endif
}

bool CameraVideoFileDecoder::appendFrameAudio(const AVFrame *frame, QByteArray& pcmS16Stereo, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(frame)
    Q_UNUSED(pcmS16Stereo)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!frame || !m_resampler || (frame->nb_samples <= 0)) {
        return true;
    }

    const int outputCapacityFrames = static_cast<int>(av_rescale_rnd(
        swr_get_delay(m_resampler, m_audioCodecContext->sample_rate) + frame->nb_samples,
        m_outputSampleRate,
        m_audioCodecContext->sample_rate,
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

    const qint64 audioDurationMs = static_cast<qint64>(
        (static_cast<double>(convertedFrames) * 1000.0 / std::max(1, m_outputSampleRate)) + 0.5);
    qint64 audioStartMs = -1;
    if ((frame->best_effort_timestamp != AV_NOPTS_VALUE) && (m_audioStreamIndex >= 0))
    {
        audioStartMs = av_rescale_q(
            frame->best_effort_timestamp,
            m_formatContext->streams[m_audioStreamIndex]->time_base,
            AVRational{1, 1000});
    }
    if ((audioStartMs >= 0) && (m_audioDecodedPositionMs >= 0))
    {
        if (m_urlSource) {
            audioStartMs = -1;
        }
    }

    if (audioStartMs >= 0) {
        m_audioDecodedPositionMs = audioStartMs + audioDurationMs;
    } else if (m_audioDecodedPositionMs >= 0) {
        m_audioDecodedPositionMs += audioDurationMs;
    }
    return true;
#endif
}

void CameraVideoFileDecoder::trimLivePendingAudio()
{
#ifdef CAMERA_FFMPEG_STREAMING
    static constexpr int bytesPerSampleFrame = 4;
    static constexpr int maxLivePendingAudioMs = 1200;

    if (!m_urlSource || (m_outputSampleRate <= 0) || (pendingAudioBytes() <= 0)) {
        return;
    }

    const int maxBytes = std::max(
        bytesPerSampleFrame,
        static_cast<int>((static_cast<qint64>(m_outputSampleRate) * maxLivePendingAudioMs * bytesPerSampleFrame) / 1000));

    int dropBytes = 0;
    {
        QMutexLocker locker(&m_pendingAudioMutex);
        if (m_pendingAudioPcm.size() <= maxBytes) {
            return;
        }

        dropBytes = ((m_pendingAudioPcm.size() - maxBytes) / bytesPerSampleFrame) * bytesPerSampleFrame;
        if (dropBytes <= 0) {
            return;
        }

        m_pendingAudioPcm.remove(0, dropBytes);
    }
#endif
}

int CameraVideoFileDecoder::takePendingAudio(QByteArray& pcmS16Stereo, int maxSampleFrames)
{
    static constexpr int bytesPerSampleFrame = 4;
    pcmS16Stereo.clear();
#ifdef CAMERA_FFMPEG_STREAMING
    if (maxSampleFrames <= 0) {
        return 0;
    }

    const int maxBytes = maxSampleFrames * bytesPerSampleFrame;
    int alignedByteCount = 0;
    {
        QMutexLocker locker(&m_pendingAudioMutex);
        if (m_pendingAudioPcm.isEmpty()) {
            return 0;
        }

        const int byteCount = std::min(maxBytes, static_cast<int>(m_pendingAudioPcm.size()));
        alignedByteCount = (byteCount / bytesPerSampleFrame) * bytesPerSampleFrame;
        if (alignedByteCount <= 0) {
            return 0;
        }

        pcmS16Stereo = m_pendingAudioPcm.left(alignedByteCount);
        m_pendingAudioPcm.remove(0, alignedByteCount);
    }
    return alignedByteCount / bytesPerSampleFrame;
#else
    Q_UNUSED(maxSampleFrames)
    return 0;
#endif
}

void CameraVideoFileDecoder::takePacedAudio(QByteArray& pcmS16Stereo)
{
    pcmS16Stereo.clear();
#ifdef CAMERA_FFMPEG_STREAMING
    static constexpr int bytesPerSampleFrame = 4;
    if ((m_outputSampleRate <= 0) || (pendingAudioBytes() <= 0)) {
        return;
    }

    // Load the worker-published rate, and reset the (decode-thread-owned)
    // remainder here when the rate changes, so the worker never touches it.
    const double paceRate = m_audioPaceFrameRate.load(std::memory_order_relaxed);
    if (std::abs(paceRate - m_audioPaceFrameRateApplied) > 0.001) {
        m_audioPaceRemainderFrames = 0.0;
        m_audioPaceFrameRateApplied = paceRate;
    }
    const double paceFrameRate = paceRate > 0.0 ? paceRate : m_frameRate;
    const double exactTargetFrames = static_cast<double>(m_outputSampleRate) / std::max(1.0, paceFrameRate);
    const double availableTargetFrames = exactTargetFrames + m_audioPaceRemainderFrames;
    const int targetFrames = std::max(1, static_cast<int>(availableTargetFrames));
    m_audioPaceRemainderFrames = availableTargetFrames - static_cast<double>(targetFrames);
    const int targetBytes = targetFrames * bytesPerSampleFrame;
    int byteCount = 0;
    {
        QMutexLocker locker(&m_pendingAudioMutex);
        byteCount = std::min(targetBytes, static_cast<int>(m_pendingAudioPcm.size()));
        pcmS16Stereo = m_pendingAudioPcm.left(byteCount);
        m_pendingAudioPcm.remove(0, byteCount);
    }
#endif
}

bool CameraVideoFileDecoder::convertFrameToImage(const AVFrame *frame, QImage& image, QString& errorMessage)
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
        qWarning() << "CameraVideoFileDecoder:" << errorMessage
                   << "codecSize" << (m_videoCodecContext ? m_videoCodecContext->width : 0)
                   << "x" << (m_videoCodecContext ? m_videoCodecContext->height : 0);
        return false;
    }

    // Rebuild against the actual decoded frame's geometry/format (cached in
    // m_swsSrc*), not the codec context: a mid-stream format change can leave the
    // codec context already matching the new frame, which would wrongly skip the
    // rebuild and scale through a stale converter.
    if (!m_swsContext
        || (m_swsSrcWidth != frame->width)
        || (m_swsSrcHeight != frame->height)
        || (m_swsSrcFormat != frame->format))
    {
        if (m_swsContext) {
            sws_freeContext(m_swsContext);
            m_swsContext = nullptr;
        }
        m_swsContext = sws_getContext(
            frame->width,
            frame->height,
            static_cast<AVPixelFormat>(frame->format),
            frame->width,
            frame->height,
            AV_PIX_FMT_RGB24,
        SWS_FAST_BILINEAR,
        nullptr,
        nullptr,
        nullptr);
        if (!m_swsContext)
        {
            errorMessage = QStringLiteral("Cannot create video file colour converter");
            return false;
        }
        m_swsSrcWidth = frame->width;
        m_swsSrcHeight = frame->height;
        m_swsSrcFormat = frame->format;
    }

    image = m_imagePool.acquire(frame->width, frame->height, QImage::Format_RGB888);
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

void CameraVideoFileDecoder::clearPendingVideoPackets()
{
#ifdef CAMERA_FFMPEG_STREAMING
    for (AVPacket *packet : m_pendingVideoPackets) {
        av_packet_free(&packet);
    }
#endif
    m_pendingVideoPackets.clear();
}

void CameraVideoFileDecoder::closeAudioDecoder()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_resampler) {
        swr_free(&m_resampler);
    }
    if (m_audioCodecContext) {
        avcodec_free_context(&m_audioCodecContext);
    }
#endif
    m_audioStreamIndex = -1;
    m_audioDraining = false;
}
