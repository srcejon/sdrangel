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

#ifndef INCLUDE_FEATURE_CAMERA_FFMPEG_AUDIO_H_
#define INCLUDE_FEATURE_CAMERA_FFMPEG_AUDIO_H_

#include <QByteArray>
#include <QString>

struct SwrContext;
struct AVFormatContext;
struct AVCodecContext;
struct AVFrame;
struct AVPacket;

/**
 * \brief FFmpeg audio decode/resample helpers for the camera feature.
 *
 * A small utility facade over libswresample for converting interleaved S16
 * stereo PCM between sample rates, plus an avErrorString() helper. The nested
 * PcmS16StereoResampler holds a persistent SwrContext for streaming use: it
 * keeps resampler state (and any carried fractional samples) across successive
 * resample() calls so a continuous stream stays click-free, with flush() to
 * drain trailing samples and reset() to start over. The static
 * resamplePcmS16Stereo() is a stateless one-shot convenience for an isolated
 * buffer.
 *
 * \note When the build lacks CAMERA_FFMPEG_STREAMING, the resample/flush methods
 *       fail with an explanatory errorMessage rather than doing any work.
 * \warning PcmS16StereoResampler is non-copyable and owns its SwrContext; it is
 *          not internally synchronised, so confine each instance to one thread
 *          (the owning encoder/writer typically holds its own).
 */
class CameraFFmpegAudio
{
public:
    class PcmS16StereoResampler
    {
    public:
        PcmS16StereoResampler() = default;
        ~PcmS16StereoResampler();
        PcmS16StereoResampler(const PcmS16StereoResampler&) = delete;
        PcmS16StereoResampler& operator=(const PcmS16StereoResampler&) = delete;

        [[nodiscard]] bool resample(const QByteArray& input,
                                    int inputSampleRate,
                                    int outputSampleRate,
                                    QByteArray& output,
                                    QString& errorMessage);
        [[nodiscard]] bool flush(QByteArray& output, QString& errorMessage);
        void reset();

    private:
        SwrContext *m_context = nullptr;
        int m_inputSampleRate = 0;
        int m_outputSampleRate = 0;
    };

    [[nodiscard]] static QString avErrorString(int errorCode);
    [[nodiscard]] static bool resamplePcmS16Stereo(const QByteArray& input,
                                                   int inputSampleRate,
                                                   int outputSampleRate,
                                                   QByteArray& output,
                                                   QString& errorMessage);
};

/**
 * \brief Shared AAC audio stream encoder/muxer for the camera FFmpeg outputs.
 *
 * Factors the audio half that CameraVideoWriter (file) and CameraYouTubeStreamer (live RTMP) used to
 * duplicate: it creates an AAC stream in a caller-owned AVFormatContext, resamples interleaved S16
 * stereo to the encoder rate (via PcmS16StereoResampler), buffers + chunks it into encoder frames,
 * prepends a one-shot A/V-sync lead silence, encodes, and muxes the packets back into that context.
 *
 * The caller owns the AVFormatContext and the video stream; this owns only the audio codec context +
 * frame. flushAfterWrite=true avio_flushes after every packet (needed for live RTMP). The live caller
 * also uses writeSilentFrame()/encodedDurationMs()/bufferedInputBytes() to pace silence fill.
 *
 * \note Single-threaded, mirrors the resampler's contract. flush() drains the resampler tail +
 *       encoder and must run while the format context is still valid; close() then frees the codec
 *       context/frame (it never touches the format context, so the caller may free that first).
 */
class CameraFFmpegAacStreamWriter
{
public:
    CameraFFmpegAacStreamWriter() = default;
    ~CameraFFmpegAacStreamWriter();
    CameraFFmpegAacStreamWriter(const CameraFFmpegAacStreamWriter&) = delete;
    CameraFFmpegAacStreamWriter& operator=(const CameraFFmpegAacStreamWriter&) = delete;

    // Create the AAC stream in formatContext (non-owning; must outlive flush()/close()). On failure
    // returns false + errorMessage and leaves nothing allocated (isOpen() == false).
    [[nodiscard]] bool open(AVFormatContext* formatContext, int outputSampleRate, bool flushAfterWrite, QString& errorMessage);
    [[nodiscard]] bool isOpen() const { return m_codecContext != nullptr; }
    [[nodiscard]] int streamIndex() const { return m_streamIndex; }
    // Encoder frame size in sample-frames (0 if not open).
    [[nodiscard]] int frameSampleCount() const;
    // Pending un-encoded S16-stereo bytes at the encoder rate (for the live silence-fill decision).
    [[nodiscard]] int bufferedInputBytes() const { return static_cast<int>(m_inputBuffer.size()); }
    // Content duration (ms) encoded so far = PTS of the next sample (for live pacing).
    [[nodiscard]] qint64 encodedDurationMs() const;
    // Prepend this many ms of silence before the first real audio (A/V-sync). Latches once.
    void setLeadSilenceMs(int milliseconds) { if (!m_leadSilenceConsumed) { m_leadSilenceMs = milliseconds > 0 ? milliseconds : 0; } }
    // Resample + buffer + chunk + encode + mux. Empty pcm just drains any full buffered frames.
    [[nodiscard]] bool writePcmS16Stereo(const QByteArray& pcm, int inputSampleRate, QString& errorMessage);
    // Encode + mux one frame of silence (live silence fill).
    [[nodiscard]] bool writeSilentFrame(QString& errorMessage);
    // Flush the resampler tail (zero-padded to a frame) + drain the encoder. Format context must be valid.
    [[nodiscard]] bool flush(QString& errorMessage);
    // Free the codec context + frame + resampler. Does not touch the format context.
    void close();

private:
    AVFormatContext *m_formatContext = nullptr;   // non-owning
    AVCodecContext *m_codecContext = nullptr;
    AVFrame *m_frame = nullptr;
    int m_streamIndex = -1;
    bool m_flushAfterWrite = false;
    CameraFFmpegAudio::PcmS16StereoResampler m_resampler;
    QByteArray m_inputBuffer;
    qint64 m_frameIndex = 0;                       // cumulative encoded sample-frames = next PTS
    int m_leadSilenceMs = 0;
    bool m_leadSilenceConsumed = false;

    [[nodiscard]] bool fillFrameFromS16Stereo(const char *pcm, int sampleFrames, QString& errorMessage);
    [[nodiscard]] bool encodeAndWriteFrame(QString& errorMessage);
    [[nodiscard]] bool writeEncodedPacket(AVPacket *packet, QString& errorMessage);
    void drainEncoder();
};

#endif // INCLUDE_FEATURE_CAMERA_FFMPEG_AUDIO_H_
