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

#include "cameraffmpegaudio.h"
#include "cameraffmpegcompat.h"

#include <cstring>

#include <QDebug>

#ifdef CAMERA_FFMPEG_STREAMING
#include <cerrno>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}
#endif

CameraFFmpegAudio::PcmS16StereoResampler::~PcmS16StereoResampler()
{
    reset();
}

void CameraFFmpegAudio::PcmS16StereoResampler::reset()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_context) {
        swr_free(&m_context);
    }
#endif
    m_inputSampleRate = 0;
    m_outputSampleRate = 0;
}

bool CameraFFmpegAudio::PcmS16StereoResampler::resample(const QByteArray& input,
                                                        int inputSampleRate,
                                                        int outputSampleRate,
                                                        QByteArray& output,
                                                        QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(input)
    Q_UNUSED(inputSampleRate)
    Q_UNUSED(outputSampleRate)
    Q_UNUSED(output)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    output.clear();
    if (input.isEmpty()) {
        return true;
    }
    if ((inputSampleRate <= 0) || (outputSampleRate <= 0))
    {
        errorMessage = QStringLiteral("Invalid audio sample rate");
        return false;
    }
    if ((input.size() % 4) != 0)
    {
        errorMessage = QStringLiteral("Stereo 16-bit PCM buffer has an invalid size");
        return false;
    }
    if (inputSampleRate == outputSampleRate)
    {
        output = input;
        return true;
    }

    if (!m_context || (m_inputSampleRate != inputSampleRate) || (m_outputSampleRate != outputSampleRate))
    {
        reset();
        int ret = cameraFFmpegAllocS16StereoRateResampler(&m_context, inputSampleRate, outputSampleRate);
        if ((ret < 0) || !m_context)
        {
            errorMessage = QStringLiteral("Cannot allocate audio resampler: %1").arg(avErrorString(ret));
            return false;
        }

        ret = swr_init(m_context);
        if (ret < 0)
        {
            errorMessage = QStringLiteral("Cannot initialise audio resampler: %1").arg(avErrorString(ret));
            reset();
            return false;
        }
        m_inputSampleRate = inputSampleRate;
        m_outputSampleRate = outputSampleRate;
    }

    const int inputFrames = input.size() / 4;
    const int outputCapacityFrames = static_cast<int>(av_rescale_rnd(
        swr_get_delay(m_context, inputSampleRate) + inputFrames,
        outputSampleRate,
        inputSampleRate,
        AV_ROUND_UP));
    if (outputCapacityFrames <= 0) {
        return true;
    }

    output.resize(outputCapacityFrames * 4);
    const uint8_t *inputData[1] = { reinterpret_cast<const uint8_t*>(input.constData()) };
    uint8_t *outputData[1] = { reinterpret_cast<uint8_t*>(output.data()) };
    const int convertedFrames = swr_convert(m_context, outputData, outputCapacityFrames, inputData, inputFrames);
    if (convertedFrames < 0)
    {
        errorMessage = QStringLiteral("Cannot resample audio: %1").arg(avErrorString(convertedFrames));
        output.clear();
        return false;
    }

    output.resize(convertedFrames * 4);
    return true;
#endif
}

bool CameraFFmpegAudio::PcmS16StereoResampler::flush(QByteArray& output, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(output)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    output.clear();
    if (!m_context || (m_inputSampleRate <= 0) || (m_outputSampleRate <= 0)) {
        return true;
    }

    const int outputCapacityFrames = static_cast<int>(av_rescale_rnd(
        swr_get_delay(m_context, m_inputSampleRate),
        m_outputSampleRate,
        m_inputSampleRate,
        AV_ROUND_UP));
    if (outputCapacityFrames <= 0) {
        return true;
    }

    output.resize(outputCapacityFrames * 4);
    uint8_t *outputData[1] = { reinterpret_cast<uint8_t*>(output.data()) };
    const int convertedFrames = swr_convert(m_context, outputData, outputCapacityFrames, nullptr, 0);
    if (convertedFrames < 0)
    {
        errorMessage = QStringLiteral("Cannot flush audio resampler: %1").arg(avErrorString(convertedFrames));
        output.clear();
        return false;
    }

    output.resize(convertedFrames * 4);
    return true;
#endif
}

QString CameraFFmpegAudio::avErrorString(int errorCode)
{
#ifdef CAMERA_FFMPEG_STREAMING
    // av_strerror() cannot stringify AVERROR(errno) system codes on Windows - the CRT has no
    // strerror() text for ETIMEDOUT (138), ECONNREFUSED (107), etc. - so it falls back to the
    // unhelpful "Error number -138 occurred". Map the common network errors to readable text first;
    // the AVERROR() macro makes the case values portable (it negates the platform's errno value).
    switch (errorCode)
    {
#ifdef ETIMEDOUT
    case AVERROR(ETIMEDOUT):    return QStringLiteral("Connection timed out");
#endif
#ifdef ECONNREFUSED
    case AVERROR(ECONNREFUSED): return QStringLiteral("Connection refused");
#endif
#ifdef ECONNRESET
    case AVERROR(ECONNRESET):   return QStringLiteral("Connection reset by peer");
#endif
#ifdef ECONNABORTED
    case AVERROR(ECONNABORTED): return QStringLiteral("Connection aborted");
#endif
#ifdef EHOSTUNREACH
    case AVERROR(EHOSTUNREACH): return QStringLiteral("No route to host");
#endif
#ifdef ENETUNREACH
    case AVERROR(ENETUNREACH):  return QStringLiteral("Network unreachable");
#endif
#ifdef ENETDOWN
    case AVERROR(ENETDOWN):     return QStringLiteral("Network is down");
#endif
#ifdef EPIPE
    case AVERROR(EPIPE):        return QStringLiteral("Connection closed by peer");
#endif
    default:
        break;
    }
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(errorCode, buffer, sizeof(buffer));
    return QString::fromLocal8Bit(buffer);
#else
    Q_UNUSED(errorCode)
    return QStringLiteral("FFmpeg support is not available in this build");
#endif
}

bool CameraFFmpegAudio::resamplePcmS16Stereo(const QByteArray& input,
                                             int inputSampleRate,
                                             int outputSampleRate,
                                             QByteArray& output,
                                             QString& errorMessage)
{
    PcmS16StereoResampler resampler;
    return resampler.resample(input, inputSampleRate, outputSampleRate, output, errorMessage);
}

// ===================== CameraFFmpegAacStreamWriter =====================
// Shared AAC encode+mux path factored out of CameraVideoWriter (file) and CameraYouTubeStreamer
// (live RTMP). Behaviour is byte-for-byte the two paths' previous duplicated code, parameterised by
// the caller's format context, the output sample rate, and flushAfterWrite.

namespace { constexpr int kAacBytesPerSampleFrame = 4; }   // S16 stereo

CameraFFmpegAacStreamWriter::~CameraFFmpegAacStreamWriter()
{
    close();
}

bool CameraFFmpegAacStreamWriter::open(AVFormatContext* formatContext, int outputSampleRate, bool flushAfterWrite, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(formatContext)
    Q_UNUSED(outputSampleRate)
    Q_UNUSED(flushAfterWrite)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    close();
    if (!formatContext) {
        errorMessage = QStringLiteral("No format context for the audio stream");
        return false;
    }

    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
    if (!codec)
    {
        errorMessage = QStringLiteral("No AAC encoder is available in FFmpeg");
        return false;
    }

    m_codecContext = avcodec_alloc_context3(codec);
    if (!m_codecContext)
    {
        errorMessage = QStringLiteral("Cannot allocate AAC encoder context");
        return false;
    }

    m_codecContext->codec_id = codec->id;
    m_codecContext->codec_type = AVMEDIA_TYPE_AUDIO;
    m_codecContext->sample_rate = outputSampleRate;
    m_codecContext->sample_fmt = codec->sample_fmts ? codec->sample_fmts[0] : AV_SAMPLE_FMT_FLTP;
    m_codecContext->bit_rate = 128000;
    cameraFFmpegSetStereoChannelLayout(m_codecContext);
    m_codecContext->time_base = AVRational{1, m_codecContext->sample_rate};

    if (formatContext->oformat->flags & AVFMT_GLOBALHEADER) {
        m_codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    int ret = avcodec_open2(m_codecContext, codec, nullptr);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot open AAC encoder: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }

    m_frame = av_frame_alloc();
    if (!m_frame)
    {
        errorMessage = QStringLiteral("Cannot allocate audio frame");
        close();
        return false;
    }
    m_frame->format = m_codecContext->sample_fmt;
    ret = cameraFFmpegSetFrameChannelLayoutFromContext(m_frame, m_codecContext);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot set audio channel layout: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }
    m_frame->sample_rate = m_codecContext->sample_rate;
    m_frame->nb_samples = m_codecContext->frame_size > 0 ? m_codecContext->frame_size : 1024;

    ret = av_frame_get_buffer(m_frame, 0);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot allocate audio frame buffer: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }

    AVStream *stream = avformat_new_stream(formatContext, nullptr);
    if (!stream)
    {
        errorMessage = QStringLiteral("Cannot create audio stream");
        close();
        return false;
    }
    ret = avcodec_parameters_from_context(stream->codecpar, m_codecContext);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot copy audio encoder parameters: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        close();
        return false;
    }
    stream->time_base = m_codecContext->time_base;

    m_formatContext = formatContext;
    m_streamIndex = stream->index;
    m_flushAfterWrite = flushAfterWrite;
    return true;
#endif
}

int CameraFFmpegAacStreamWriter::frameSampleCount() const
{
#ifdef CAMERA_FFMPEG_STREAMING
    return m_frame ? m_frame->nb_samples : 0;
#else
    return 0;
#endif
}

qint64 CameraFFmpegAacStreamWriter::encodedDurationMs() const
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (!m_codecContext) {
        return 0;
    }
    return av_rescale_q(m_frameIndex, m_codecContext->time_base, AVRational{1, 1000});
#else
    return 0;
#endif
}

bool CameraFFmpegAacStreamWriter::writeEncodedPacket(AVPacket *packet, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(packet)
    Q_UNUSED(errorMessage)
    return false;
#else
    packet->stream_index = m_streamIndex;
    AVStream *stream = m_formatContext->streams[m_streamIndex];
    av_packet_rescale_ts(packet, m_codecContext->time_base, stream->time_base);
    if (packet->duration <= 0) {
        packet->duration = av_rescale_q(1, m_codecContext->time_base, stream->time_base);
    }
    const int ret = av_interleaved_write_frame(m_formatContext, packet);
    if ((ret >= 0) && m_flushAfterWrite && m_formatContext->pb) {
        avio_flush(m_formatContext->pb);
    }
    av_packet_unref(packet);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write audio packet: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }
    return true;
#endif
}

bool CameraFFmpegAacStreamWriter::fillFrameFromS16Stereo(const char *pcm, int sampleFrames, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(pcm)
    Q_UNUSED(sampleFrames)
    Q_UNUSED(errorMessage)
    return false;
#else
    if (!m_frame || !m_codecContext) {
        return false;
    }
    if (sampleFrames != m_frame->nb_samples)
    {
        errorMessage = QStringLiteral("Audio frame size mismatch");
        return false;
    }

    const qint16 *samples = reinterpret_cast<const qint16*>(pcm);

    switch (m_codecContext->sample_fmt)
    {
    case AV_SAMPLE_FMT_FLTP:
    {
        float *left = reinterpret_cast<float*>(m_frame->data[0]);
        float *right = reinterpret_cast<float*>(m_frame->data[1]);
        for (int i = 0; i < sampleFrames; ++i)
        {
            left[i] = samples[i * 2] / 32768.0f;
            right[i] = samples[i * 2 + 1] / 32768.0f;
        }
        return true;
    }
    case AV_SAMPLE_FMT_S16P:
    {
        qint16 *left = reinterpret_cast<qint16*>(m_frame->data[0]);
        qint16 *right = reinterpret_cast<qint16*>(m_frame->data[1]);
        for (int i = 0; i < sampleFrames; ++i)
        {
            left[i] = samples[i * 2];
            right[i] = samples[i * 2 + 1];
        }
        return true;
    }
    case AV_SAMPLE_FMT_S16:
        std::memcpy(m_frame->data[0], pcm, static_cast<size_t>(sampleFrames) * 4U);
        return true;
    default:
        errorMessage = QStringLiteral("Unsupported AAC sample format %1").arg(av_get_sample_fmt_name(m_codecContext->sample_fmt));
        return false;
    }
#endif
}

bool CameraFFmpegAacStreamWriter::encodeAndWriteFrame(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    m_frame->pts = m_frameIndex;
    m_frameIndex += m_frame->nb_samples;

    int ret = avcodec_send_frame(m_codecContext, m_frame);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot encode audio frame: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }

    AVPacket *packet = av_packet_alloc();
    if (!packet)
    {
        errorMessage = QStringLiteral("Cannot allocate audio packet");
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
        errorMessage = QStringLiteral("Cannot receive audio packet: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        ok = false;
    }

    av_packet_free(&packet);
    return ok;
#endif
}

bool CameraFFmpegAacStreamWriter::writePcmS16Stereo(const QByteArray& pcm, int inputSampleRate, QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(pcm)
    Q_UNUSED(inputSampleRate)
    errorMessage = QStringLiteral("FFmpeg support is not available in this build");
    return false;
#else
    if (!m_codecContext || !m_frame) {
        return true;
    }
    // One-shot A/V-sync lead silence before any real audio (see setLeadSilenceMs). The buffer holds
    // post-resample data at the encoder rate, so size the silence at the encoder sample rate.
    if (!m_leadSilenceConsumed)
    {
        m_leadSilenceConsumed = true;
        if (m_leadSilenceMs > 0)
        {
            const qint64 silenceFrames =
                (static_cast<qint64>(m_codecContext->sample_rate) * m_leadSilenceMs) / 1000;
            if (silenceFrames > 0) {
                m_inputBuffer.append(QByteArray(static_cast<int>(silenceFrames) * kAacBytesPerSampleFrame, 0));
            }
        }
        qDebug() << "CameraFFmpegAacStreamWriter: applied audio lead silence (ms)" << m_leadSilenceMs;
    }
    if (!pcm.isEmpty())
    {
        QByteArray streamPcm;
        if (!m_resampler.resample(pcm, inputSampleRate, m_codecContext->sample_rate, streamPcm, errorMessage)) {
            return false;
        }
        m_inputBuffer.append(streamPcm);
    }
    const int audioFrameBytes = m_frame->nb_samples * kAacBytesPerSampleFrame;

    while (m_inputBuffer.size() >= audioFrameBytes)
    {
        int ret = av_frame_make_writable(m_frame);
        if (ret < 0)
        {
            errorMessage = QStringLiteral("Cannot write to audio frame buffer: %1").arg(CameraFFmpegAudio::avErrorString(ret));
            return false;
        }
        if (!fillFrameFromS16Stereo(m_inputBuffer.constData(), m_frame->nb_samples, errorMessage)) {
            return false;
        }
        m_inputBuffer.remove(0, audioFrameBytes);
        if (!encodeAndWriteFrame(errorMessage)) {
            return false;
        }
    }
    return true;
#endif
}

bool CameraFFmpegAacStreamWriter::writeSilentFrame(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    if (!m_codecContext || !m_frame) {
        return true;
    }
    int ret = av_frame_make_writable(m_frame);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot write to audio frame buffer: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }
    ret = av_samples_set_silence(
        m_frame->data,
        0,
        m_frame->nb_samples,
        cameraFFmpegCodecContextChannels(m_codecContext),
        m_codecContext->sample_fmt);
    if (ret < 0)
    {
        errorMessage = QStringLiteral("Cannot clear audio buffer: %1").arg(CameraFFmpegAudio::avErrorString(ret));
        return false;
    }
    return encodeAndWriteFrame(errorMessage);
#endif
}

void CameraFFmpegAacStreamWriter::drainEncoder()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (!m_codecContext || !m_formatContext) {
        return;
    }
    avcodec_send_frame(m_codecContext, nullptr);
    AVPacket *packet = av_packet_alloc();
    if (packet)
    {
        QString ignored;
        while (avcodec_receive_packet(m_codecContext, packet) >= 0) {
            const bool ok = writeEncodedPacket(packet, ignored);
            Q_UNUSED(ok)
        }
        av_packet_free(&packet);
    }
#endif
}

bool CameraFFmpegAacStreamWriter::flush(QString& errorMessage)
{
#ifndef CAMERA_FFMPEG_STREAMING
    Q_UNUSED(errorMessage)
    return false;
#else
    if (!m_codecContext || !m_frame) {
        return true;
    }
    QByteArray delayedPcm;
    if (!m_resampler.flush(delayedPcm, errorMessage)) {
        return false;
    }
    if (!delayedPcm.isEmpty()) {
        m_inputBuffer.append(delayedPcm);
    }
    if (!m_inputBuffer.isEmpty())
    {
        const int audioFrameBytes = m_frame->nb_samples * kAacBytesPerSampleFrame;
        const int paddingBytes = audioFrameBytes - (m_inputBuffer.size() % audioFrameBytes);
        if ((paddingBytes > 0) && (paddingBytes < audioFrameBytes)) {
            m_inputBuffer.append(QByteArray(paddingBytes, 0));
        }
        if (!writePcmS16Stereo(QByteArray(), m_codecContext->sample_rate, errorMessage)) {
            return false;
        }
    }
    drainEncoder();
    return true;
#endif
}

void CameraFFmpegAacStreamWriter::close()
{
#ifdef CAMERA_FFMPEG_STREAMING
    if (m_codecContext) {
        avcodec_free_context(&m_codecContext);
    }
    if (m_frame) {
        av_frame_free(&m_frame);
    }
    m_resampler.reset();
#endif
    m_formatContext = nullptr;
    m_codecContext = nullptr;
    m_frame = nullptr;
    m_streamIndex = -1;
    m_flushAfterWrite = false;
    m_inputBuffer.clear();
    m_frameIndex = 0;
    m_leadSilenceMs = 0;
    m_leadSilenceConsumed = false;
}
