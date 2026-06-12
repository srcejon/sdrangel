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

#include "cameraffmpegaudio.h"

#ifdef CAMERA_FFMPEG_STREAMING
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}
#endif

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

bool CameraVideoFileDecoder::open(const QString& fileName, QString& errorMessage, int outputSampleRate)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(fileName)
    Q_UNUSED(outputSampleRate)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    close();
    m_outputSampleRate = std::max(1000, outputSampleRate);

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

    if (!openVideoDecoder(errorMessage))
    {
        close();
        return false;
    }

    QString audioError;
    if (!openAudioDecoder(audioError)) {
        closeAudioDecoder();
    }

    m_videoFrame = av_frame_alloc();
    m_audioFrame = av_frame_alloc();
    m_packet = av_packet_alloc();
    if (!m_videoFrame || !m_audioFrame || !m_packet)
    {
        errorMessage = QStringLiteral("Cannot allocate video file decode buffers");
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
    if ((rate.num > 0) && (rate.den > 0)) {
        m_frameRate = qBound(1.0, av_q2d(rate), 240.0);
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
    if (m_swsContext)
    {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
    }
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
    m_audioDecodedPositionMs = -1;
    m_pendingAudioPcm.clear();
    m_eof = false;
    m_videoDraining = false;
    m_audioDraining = false;
    m_audioDecodedPositionMs = -1;
    m_pendingAudioPcm.clear();
    m_pendingVideoFrames.clear();
    m_debugStats = DebugStats();
}

void CameraVideoFileDecoder::setAudioPaceFrameRate(double frameRate)
{
    const double clampedFrameRate = std::max(0.0, frameRate);
    if (std::abs(m_audioPaceFrameRate - clampedFrameRate) > 0.001) {
        m_audioPaceRemainderFrames = 0.0;
    }
    m_audioPaceFrameRate = clampedFrameRate;
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
    m_pendingAudioPcm.clear();
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
                    avcodec_send_packet(m_audioCodecContext, nullptr);
                    m_audioDraining = true;
                    if (!drainAudio(decodedAudio, errorMessage)) {
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
            ret = avcodec_send_packet(m_audioCodecContext, m_packet);
            av_packet_unref(m_packet);
            if ((ret < 0) && (ret != AVERROR(EAGAIN)))
            {
                errorMessage = QStringLiteral("Cannot send video file audio packet to decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
                return false;
            }
            if (!drainAudio(decodedAudio, errorMessage)) {
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

        if (image.isNull() || (positionMs < 0) || (positionMs + toleranceMs >= targetPositionMs)) {
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
    m_audioStreamIndex = ret;

    AVStream *stream = m_formatContext->streams[m_audioStreamIndex];
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

    ret = avcodec_parameters_to_context(m_audioCodecContext, stream->codecpar);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy video file audio decoder parameters: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }

    ret = avcodec_open2(m_audioCodecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open video file audio decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }

    return openResampler(errorMessage);
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

        ++m_debugStats.m_sendVideoPacketEagain;
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
    static constexpr size_t maxPendingVideoFrames = 3;

    for (;;)
    {
        if (m_pendingVideoFrames.size() >= maxPendingVideoFrames) {
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
            ++m_debugStats.m_queuedVideoFrames;
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
    QImage image;
    qint64 positionMs = -1;
    if (receiveVideoFrame(image, positionMs, errorMessage))
    {
        PendingVideoFrame pending;
        pending.m_image = std::move(image);
        pending.m_positionMs = positionMs;
        m_pendingVideoFrames.push_back(std::move(pending));
        ++m_debugStats.m_queuedVideoFrames;
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
    if (!readAheadAudio(decodedAudio, videoPositionMs, errorMessage)) {
        return false;
    }
    if (!decodedAudio.isEmpty()) {
        m_pendingAudioPcm.append(decodedAudio);
    }
    takePacedAudio(pcmS16Stereo);
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

    ++m_debugStats.m_readAheadCalls;
    static constexpr size_t maxPendingVideoPackets = 30;
    static constexpr qint64 audioLeadMs = 50;
    static constexpr int bytesPerSampleFrame = 4;
    const int frameAudioFrames = static_cast<int>((m_outputSampleRate / std::max(1.0, m_frameRate)) + 0.5);
    const int targetAudioFrames = std::max(frameAudioFrames, m_outputSampleRate / 50);
    const int targetAudioBytes = targetAudioFrames * bytesPerSampleFrame;
    const qint64 targetAudioPositionMs = videoPositionMs >= 0
        ? videoPositionMs + audioLeadMs
        : -1;
    int packetsRead = 0;

    auto needsAudio = [&]() {
        if ((targetAudioPositionMs >= 0) && (m_audioDecodedPositionMs >= targetAudioPositionMs)) {
            return false;
        }
        return (m_pendingAudioPcm.size() + pcmS16Stereo.size()) < targetAudioBytes;
    };

    while (needsAudio()
        && !m_eof
        && (packetsRead < 32)
        && (m_pendingVideoPackets.size() < maxPendingVideoPackets))
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
                    avcodec_send_packet(m_audioCodecContext, nullptr);
                    m_audioDraining = true;
                }
                if (!drainAudio(pcmS16Stereo, errorMessage) || !queueDecodedVideoFrames(errorMessage)) {
                    return false;
                }
                m_eof = m_pendingVideoFrames.empty();
                return true;
            }
            errorMessage = QStringLiteral("Cannot read video file packet: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }

        ++m_debugStats.m_readAheadPackets;
        if (m_packet->stream_index == m_audioStreamIndex)
        {
            ret = avcodec_send_packet(m_audioCodecContext, m_packet);
            av_packet_unref(m_packet);
            if ((ret < 0) && (ret != AVERROR(EAGAIN)))
            {
                errorMessage = QStringLiteral("Cannot send video file audio packet to decoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
                return false;
            }
            if (!drainAudio(pcmS16Stereo, errorMessage)) {
                return false;
            }
        }
        else if (m_packet->stream_index == m_videoStreamIndex)
        {
            ++m_debugStats.m_readAheadVideoPackets;
            AVPacket *parkedPacket = av_packet_clone(m_packet);
            av_packet_unref(m_packet);
            if (!parkedPacket)
            {
                errorMessage = QStringLiteral("Cannot allocate parked video file packet");
                return false;
            }
            m_pendingVideoPackets.push_back(parkedPacket);
            ++m_debugStats.m_parkedVideoPackets;
        }
        else
        {
            av_packet_unref(m_packet);
        }

        ++packetsRead;
    }

    if (m_pendingVideoPackets.size() >= maxPendingVideoPackets) {
        ++m_debugStats.m_readAheadPacketCapHits;
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
    m_debugStats.m_audioBytes += static_cast<quint64>(output.size());

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
        const qint64 audioTimestampGapMs = audioStartMs - m_audioDecodedPositionMs;
        if (std::abs(audioTimestampGapMs) > 2)
        {
            ++m_debugStats.m_audioTimestampJumps;
            m_debugStats.m_audioTimestampJumpMaxAbsMs = std::max(
                m_debugStats.m_audioTimestampJumpMaxAbsMs,
                static_cast<qint64>(std::abs(audioTimestampGapMs)));
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

void CameraVideoFileDecoder::takePacedAudio(QByteArray& pcmS16Stereo)
{
    pcmS16Stereo.clear();
#ifdef CAMERA_FFMPEG_STREAMING
    static constexpr int bytesPerSampleFrame = 4;
    if (m_pendingAudioPcm.isEmpty() || (m_outputSampleRate <= 0)) {
        return;
    }

    const double paceFrameRate = m_audioPaceFrameRate > 0.0 ? m_audioPaceFrameRate : m_frameRate;
    const double exactTargetFrames = static_cast<double>(m_outputSampleRate) / std::max(1.0, paceFrameRate);
    const double availableTargetFrames = exactTargetFrames + m_audioPaceRemainderFrames;
    const int targetFrames = std::max(1, static_cast<int>(availableTargetFrames));
    m_audioPaceRemainderFrames = availableTargetFrames - static_cast<double>(targetFrames);
    const int targetBytes = targetFrames * bytesPerSampleFrame;
    const int byteCount = std::min(targetBytes, static_cast<int>(m_pendingAudioPcm.size()));
    ++m_debugStats.m_pacedAudioCalls;
    m_debugStats.m_pacedAudioTargetFrames += static_cast<quint64>(targetFrames);
    m_debugStats.m_pacedAudioOutputFrames += static_cast<quint64>(byteCount / bytesPerSampleFrame);
    if (byteCount < targetBytes) {
        ++m_debugStats.m_pacedAudioShortCalls;
    }
    pcmS16Stereo = m_pendingAudioPcm.left(byteCount);
    m_pendingAudioPcm.remove(0, byteCount);
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
        return false;
    }

    if (!m_swsContext
        || (m_videoCodecContext->width != frame->width)
        || (m_videoCodecContext->height != frame->height)
        || (m_videoCodecContext->pix_fmt != static_cast<AVPixelFormat>(frame->format)))
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
        SWS_FAST_BILINEAR,
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
    QElapsedTimer convertTimer;
    convertTimer.start();
    sws_scale(m_swsContext, frame->data, frame->linesize, 0, frame->height, dstData, dstLinesize);
    const qint64 convertMs = convertTimer.elapsed();
    ++m_debugStats.m_videoConvertFrames;
    m_debugStats.m_videoConvertMs += static_cast<quint64>(std::max<qint64>(0, convertMs));
    m_debugStats.m_videoConvertMaxMs = std::max(m_debugStats.m_videoConvertMaxMs, convertMs);
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
