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

#ifndef INCLUDE_FEATURE_CAMERA_VIDEO_FILE_DECODER_H_
#define INCLUDE_FEATURE_CAMERA_VIDEO_FILE_DECODER_H_

#include <atomic>
#include <deque>

#include <QByteArray>
#include <QImage>
#include <QMutex>
#include <QString>
#include <QWaitCondition>

#include "cameraaudiobytequeue.h"
#include "cameraimagepool.h"

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct SwrContext;
struct SwsContext;
class QThread;

/**
 * \brief FFmpeg-based decoder for video files and live network streams.
 *
 * Wraps libav* to demux/decode a media source (a local file, or an
 * http/rtsp/rtmp live stream) into RGB888 QImages and interleaved S16 stereo
 * PCM resampled to a fixed output rate. Video frames are colour-converted via
 * an SwsContext that is lazily (re)built when the source geometry/format
 * changes mid-stream; audio is resampled via an SwrContext. Pending decoded
 * video frames/packets and decoded audio are staged in internal queues so the
 * caller can read frame-by-frame and pull audio paced to the video rate.
 *
 * \note Intended to run on a single decode thread that drives open()/seek()/
 *       readNextFrame()/close(). The pending-audio staging buffer is guarded by
 *       m_pendingAudioMutex (takePendingAudio/pendingAudioBytes may be called
 *       from another thread). m_audioPaceFrameRate is atomic because it is set
 *       from the worker thread (setAudioPaceFrameRate) while the decode thread
 *       reads it; the pace remainder is decode-thread-owned only.
 * \note m_abortRequested is atomic: requestAbort() can be called from another
 *       thread to unblock a decode stuck in FFmpeg I/O (notably a stalled live
 *       stream); the decode thread polls it via abortRequested().
 * \note Decoded QImages share recycled RGB888 backing buffers from the internal
 *       CameraImagePool, so a returned QImage's pixels are only valid until the
 *       pool reuses that buffer; downstream consumers must copy if they retain
 *       it beyond the pool's in-flight window.
 * \warning For live sources the decoder caps its pending-audio staging buffer
 *          (setMaxLivePendingAudioMs) so deep buffering happens downstream
 *          rather than here; see trimLivePendingAudio.
 */
class CameraVideoFileDecoder
{
public:
    CameraVideoFileDecoder();
    ~CameraVideoFileDecoder();

    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] bool abortRequested() const { return m_abortRequested.load(); }
    [[nodiscard]] bool open(
        const QString& fileName,
        QString& errorMessage,
        int outputSampleRate = 48000,
        double readAheadSeconds = 1.0,
        bool streaming = false);
    void requestAbort();
    void close();
    void seek(qint64 positionMs);
    void setAudioPaceFrameRate(double frameRate);
    // Cap on the decoder-side pending-audio staging buffer for live sources (see
    // trimLivePendingAudio). Defaults to 1200 ms; the controller raises it to track
    // streamBufferingSeconds so a deep buffer setting isn't undercut by the decoder
    // trimming audio before it reaches the (larger) downstream stream-audio buffer.
    void setMaxLivePendingAudioMs(int ms) { m_maxLivePendingAudioMs = ms < 1 ? 1 : ms; }
    [[nodiscard]] bool readNextFrame(
        QImage& image,
        qint64& positionMs,
        QByteArray& pcmS16Stereo,
        int& audioSampleRate,
        QString& errorMessage);
    [[nodiscard]] bool readNextFrameAtOrAfter(
        qint64 targetPositionMs,
        QImage& image,
        qint64& positionMs,
        QString& errorMessage);
    [[nodiscard]] qint64 durationMs() const { return m_durationMs; }
    [[nodiscard]] double frameRate() const { return m_frameRate; }
    [[nodiscard]] qint64 audioDecodedPositionMs() const { return m_audioDecodedPositionMs; }
    // Source content position (ms) of the FIRST sample of the audio handed back by the
    // most recent readNextFrame(). This is the audio's own PTS, which runs ahead of the
    // paired video frame's position by the monitor read-ahead — use it (not the video
    // position) to place recorded audio on the source timeline. -1 if unknown.
    [[nodiscard]] qint64 lastReturnedAudioStartMs() const { return m_lastReturnedAudioStartMs; }
    // Per-frame timing of the most recent readNextFrame() stages (microseconds),
    // for diagnosing decode throughput. m_lastConvertUSecs = colour conversion
    // (sws_scale + QImage); m_lastFrameAudioUSecs = finishFrameAudio.
    [[nodiscard]] qint64 lastConvertUSecs() const { return m_lastConvertUSecs; }
    [[nodiscard]] qint64 lastFrameAudioUSecs() const { return m_lastFrameAudioUSecs; }
    // Diagnostic: current compressed-bitstream read-ahead depth (video packets
    // buffered). ~0 means the decode path is waiting on the network (server-paced);
    // ~cap means the buffer is full and decode is the bottleneck.
    [[nodiscard]] int readAheadVideoPacketCount() const;
    [[nodiscard]] int pendingAudioBytes() const;
    int takePendingAudio(QByteArray& pcmS16Stereo, int maxSampleFrames);
    [[nodiscard]] int pendingVideoFrameCount() const { return static_cast<int>(m_pendingVideoFrames.size()); }
    [[nodiscard]] int pendingVideoPacketCount() const { return static_cast<int>(m_pendingVideoPackets.size()); }

    // ---- Clean streaming path (URL sources) — see STREAM_REWRITE_PLAN.md ----------------
    // Separate demux + audio-decode + video-decode threads, each paced by its own consumer:
    // audio decode blocks when the audio sample buffer is full (so it is paced by the audio
    // OUTPUT pulling at the device rate), video decode blocks when the decoded-frame queue is
    // full (paced by the present). This replaces the old single combined-decode loop whose
    // pace was tied to the video present. Enabled by open(..., streaming=true).
    //
    // Decoder owns the whole audio path: decode → buffer → drift resampler → DEVICE-rate
    // output. streamTakeAudio runs the slow drift servo (holding the buffer near
    // streamAudioTargetSeconds) and returns up to maxSampleFrames of device-rate S16 stereo.
    // It reports playedPtsMs = the content PTS (ms) most recently CONSUMED from the buffer
    // (i.e. handed toward the device); the consumer derives the master clock as
    // playedPtsMs − monitorFIFO_ms − sinkLatency_ms (single buffer ⇒ exact, no summing of
    // estimated queue depths). Returns frames produced. playedPtsMs = −1 if nothing consumed.
    int streamTakeAudio(QByteArray& pcmS16Stereo, int maxSampleFrames, qint64& playedPtsMs);
    void setStreamAudioTargetSeconds(double seconds);
    [[nodiscard]] int streamAudioBufferedFrames() const;
    // True if the source has an audio track. Set at open() and stable during playback; a video-only
    // stream (no audio) must drive the present from a wall clock, not the audio clock.
    [[nodiscard]] bool streamHasAudio() const { return m_audioStreamIndex >= 0; }
    // Pop the next decoded video frame (oldest). Returns false if the queue is empty.
    bool streamTakeVideoFrame(QImage& image, qint64& positionMs, bool& eof);
    // PTS (ms) of the next decoded video frame without popping; −1 if the queue is empty.
    [[nodiscard]] qint64 streamPeekVideoFramePtsMs() const;
    // Copy the oldest queued frame's image + PTS without consuming it (for a buffering poster).
    [[nodiscard]] bool streamPeekVideoFrameImage(QImage& image, qint64& positionMs) const;
    [[nodiscard]] int streamDecodedVideoFrameCount() const;
    [[nodiscard]] int streamVideoPacketCount() const;
    // True once the demux thread has stopped (read error / rw_timeout / EOF) - the source is dead
    // and will not recover without a reconnect (see streamSourceFailed body).
    [[nodiscard]] bool streamSourceFailed() const;
    // Decode-edge PTS (ms): the latest content timestamp each decoder has produced, straight from
    // the packet PTS (best_effort_timestamp), BEFORE any present-side coupling. Comparing these two
    // is a non-circular A/V sync probe — if the audio and video timelines drift apart these diverge,
    // independent of which frame the present chooses to show. −1 until the first frame/packet.
    [[nodiscard]] qint64 streamVideoDecodedEdgePtsMs() const;
    [[nodiscard]] qint64 streamAudioDecodedEdgePtsMs() const;

private:
    struct PendingVideoFrame
    {
        QImage m_image;
        qint64 m_positionMs = -1;
    };

    static constexpr size_t m_maxPendingVideoFrames = 3;
    static constexpr size_t m_maxPendingStreamVideoFrames = 4;
    static constexpr size_t m_maxPendingVideoPackets = 30;
    // Keep a modest packet cushion for live streams; the monitor FIFO provides
    // the main audio jitter buffer, so excessive packet parking makes video lag.
    static constexpr size_t m_maxPendingStreamVideoPackets = 4;

    AVFormatContext *m_formatContext = nullptr;
    AVCodecContext *m_videoCodecContext = nullptr;
    AVCodecContext *m_audioCodecContext = nullptr;
    AVFrame *m_videoFrame = nullptr;
    AVFrame *m_audioFrame = nullptr;
    AVPacket *m_packet = nullptr;
    SwsContext *m_swsContext = nullptr;
    // Cached stream time bases (num/den), captured at open so the decode thread can
    // rescale frame PTS WITHOUT touching m_formatContext — for URL sources that
    // context is owned exclusively by the read-ahead thread (av_read_frame), and
    // AVFormatContext is not safe for concurrent access.
    int m_videoTimeBaseNum = 0;
    int m_videoTimeBaseDen = 1;
    int m_audioTimeBaseNum = 0;
    int m_audioTimeBaseDen = 1;
    // Read-ahead packet (bitstream) buffer for URL/live sources: a background thread
    // pulls av_read_frame into this compressed-packet queue (capped at
    // readAheadSeconds of video), so network jitter stalls only the read thread, not
    // the decode/present. The decode path (readNextFrame) pops packets from here
    // instead of calling av_read_frame. KB-sized vs the old decoded-image buffer.
    QThread *m_readThread = nullptr;
    mutable QMutex m_readAheadMutex;
    QWaitCondition m_readAheadNotEmpty;
    QWaitCondition m_readAheadNotFull;
    std::deque<AVPacket*> m_readAheadPackets;
    int m_readAheadVideoCount = 0;
    int m_readAheadCapVideoPackets = 60;
    bool m_readAheadEof = false;
    bool m_readAheadError = false;
    // Source geometry/format the current m_swsContext was built for. The
    // converter is rebuilt when a decoded frame's actual width/height/format
    // differs from these (NOT from the codec context, which may already match
    // after a mid-stream change and would skip a needed rebuild). -1 = none.
    int m_swsSrcWidth = 0;
    int m_swsSrcHeight = 0;
    int m_swsSrcFormat = -1;
    SwrContext *m_resampler = nullptr;
    int m_videoStreamIndex = -1;
    int m_audioStreamIndex = -1;
    qint64 m_durationMs = 0;
    double m_frameRate = 25.0;
    int m_outputSampleRate = 48000;
    // Cap (ms) on the live-source pending-audio staging buffer; settable so it can
    // track the controller's streamBufferingSeconds (see setMaxLivePendingAudioMs).
    int m_maxLivePendingAudioMs = 1200;
    // Published from the worker thread (setAudioPaceFrameRate) while the decode
    // thread reads it in takePacedAudio, so it must be atomic. m_audioPaceRemainderFrames
    // is owned solely by the decode thread; a rate change is detected there by
    // comparing against m_audioPaceFrameRateApplied, so the remainder is never
    // written cross-thread.
    std::atomic<double> m_audioPaceFrameRate { 0.0 };
    double m_audioPaceRemainderFrames = 0.0;
    double m_audioPaceFrameRateApplied = 0.0;
    qint64 m_audioDecodedPositionMs = -1;
    qint64 m_lastReturnedAudioStartMs = -1;
    qint64 m_lastConvertUSecs = 0;
    qint64 m_lastFrameAudioUSecs = 0;
    CameraAudioByteQueue m_pendingAudioPcm;
    mutable QMutex m_pendingAudioMutex;
    bool m_eof = false;
    bool m_videoDraining = false;
    bool m_audioDraining = false;
    bool m_urlSource = false;
    std::atomic_bool m_abortRequested { false };
    std::deque<AVPacket*> m_pendingVideoPackets;
    std::deque<PendingVideoFrame> m_pendingVideoFrames;
    // Recycles the per-frame RGB888 backing buffers produced in
    // convertFrameToImage, cutting malloc/page-fault churn at frame rate. Sized
    // for the in-flight frame count (≤4 stream pending + worker/pipeline copies).
    CameraImagePool m_imagePool { 8 };

    // ---- Clean streaming path state (URL sources) — see STREAM_REWRITE_PLAN.md ----------
    struct StreamDecodedFrame
    {
        QImage m_image;
        qint64 m_ptsMs = -1;
        bool m_eof = false;
    };
    bool m_streamingMode = false;
    QThread *m_demuxThread = nullptr;
    QThread *m_audioDecodeThread = nullptr;
    QThread *m_videoDecodeThread = nullptr;
    // One mutex guards all the streaming queues + their wait conditions. The three threads
    // only ever briefly lock it to move a packet/frame in or out, so contention is low.
    mutable QMutex m_streamMutex;
    // Demux → decode: separate compressed packet queues (the jitter/cushion buffer). Capped
    // by video-packet count (≈ streamBufferingSeconds); audio cap derived from it.
    std::deque<AVPacket*> m_streamVideoPktQ;
    std::deque<AVPacket*> m_streamAudioPktQ;
    int m_streamVideoPktCap = 60;
    int m_streamAudioPktCap = 300;
    QWaitCondition m_streamVideoPktNotEmpty;
    QWaitCondition m_streamVideoPktNotFull;
    QWaitCondition m_streamAudioPktNotEmpty;
    QWaitCondition m_streamAudioPktNotFull;
    bool m_streamDemuxEof = false;
    bool m_streamDemuxError = false;
    // Separate mutex for the decoded-audio buffer domain (m_streamAudioBuf, end-PTS, drift servo,
    // cap). The resampler (streamTakeAudio) holds this for the whole interpolation, so it must NOT
    // be the same lock as the packet/video queues or a big audio pull would block demux/video.
    // Lock ordering: m_streamMutex BEFORE m_streamAudioMutex (only clearStreamQueues nests both);
    // nothing locks audio-then-stream.
    mutable QMutex m_streamAudioMutex;
    // Audio decode → output: device-nominal S16 stereo, content-rate, paced by the consumer.
    CameraAudioByteQueue m_streamAudioBuf;
    // Content PTS (ms) of the END of decoded audio currently in the buffer (the back).
    // head-PTS = this − buffered-ms, so it advances exactly as the consumer drains the
    // buffer. Updated by the audio decode thread under m_streamAudioMutex.
    qint64 m_streamAudioEndPtsMs = -1;
    int m_streamAudioBufCapFrames = 0;
    QWaitCondition m_streamAudioBufNotFull;
    QWaitCondition m_streamAudioBufNotEmpty;
    // Drift servo (slow PI on buffer depth → output ratio) + interpolation phase, owned by
    // the consumer (streamTakeAudio). Matches the source content rate to the soundcard rate.
    double m_streamDriftRatio = 1.0;
    double m_streamDriftPhase = 0.0;
    double m_streamAudioTargetSeconds = 0.30;
    // Video decode → present: decoded frames + PTS.
    std::deque<StreamDecodedFrame> m_streamVideoFrameQ;
    // Small just-in-time queue holding pool-backed frames a little ahead of the present for timer
    // jitter; the cushion + latency live in the compressed packet queue. The image pool is
    // refcounted so queued frames stay valid without copying (see streamVideoDecodeLoop).
    size_t m_streamVideoFrameCap = 6;
    QWaitCondition m_streamVideoFrameNotEmpty;
    QWaitCondition m_streamVideoFrameNotFull;
    // Latest decoded video PTS (ms) — the video decode edge (see streamVideoDecodedEdgePtsMs).
    qint64 m_streamVideoDecodedEdgePtsMs = -1;

    void startStreamThreads();
    void stopStreamThreads();
    void clearStreamQueues();       // call with m_streamMutex held
    void streamDemuxLoop();
    void streamAudioDecodeLoop();
    void streamVideoDecodeLoop();

    void startReadAhead();
    void stopReadAhead();
    void clearReadAheadPackets();   // call with m_readAheadMutex held
    // Pop the next packet from the read-ahead queue into pkt (URL sources). Returns
    // 0 on success, AVERROR_EOF on end-of-stream, AVERROR_EXIT on abort, or a generic
    // error if the read thread failed. Blocks (with a re-checking timeout) while the
    // queue is empty and the reader is still running.
    [[nodiscard]] int takeReadAheadPacket(AVPacket *pkt);
    [[nodiscard]] bool openVideoDecoder(QString& errorMessage);
    [[nodiscard]] bool openAudioDecoder(QString& errorMessage);
    [[nodiscard]] bool openAudioDecoderForStream(int streamIndex, QString& errorMessage);
    [[nodiscard]] bool isAudioStream(int streamIndex) const;
    [[nodiscard]] bool isCompatibleAudioStream(int streamIndex) const;
    [[nodiscard]] bool openResampler(QString& errorMessage);
    [[nodiscard]] bool sendAudioPacket(AVPacket *packet, QByteArray& pcmS16Stereo, QString& errorMessage);
    [[nodiscard]] bool sendVideoPacket(AVPacket *packet, QString& errorMessage);
    [[nodiscard]] bool receiveVideoFrame(QImage& image, qint64& positionMs, QString& errorMessage);
    [[nodiscard]] bool queueOneDecodedVideoFrame(QString& errorMessage);
    [[nodiscard]] bool queueDecodedVideoFrames(QString& errorMessage);
    [[nodiscard]] bool finishFrameAudio(QByteArray& decodedAudio, QByteArray& pcmS16Stereo, qint64 videoPositionMs, QString& errorMessage);
    [[nodiscard]] bool readAheadAudio(QByteArray& pcmS16Stereo, qint64 videoPositionMs, QString& errorMessage);
    [[nodiscard]] bool drainAudio(QByteArray& pcmS16Stereo, QString& errorMessage);
    [[nodiscard]] bool appendFrameAudio(const AVFrame *frame, QByteArray& pcmS16Stereo, QString& errorMessage);
    [[nodiscard]] bool convertFrameToImage(const AVFrame *frame, QImage& image, QString& errorMessage);
    void trimLivePendingAudio();
    void takePacedAudio(QByteArray& pcmS16Stereo);
    void clearPendingAudio();
    void appendPendingAudio(const QByteArray& pcmS16Stereo);
    void clearPendingVideoPackets();
    void closeAudioDecoder();
};

#endif // INCLUDE_FEATURE_CAMERA_VIDEO_FILE_DECODER_H_
