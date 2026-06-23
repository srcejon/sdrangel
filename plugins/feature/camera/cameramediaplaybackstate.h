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

#ifndef INCLUDE_FEATURE_CAMERA_MEDIA_PLAYBACK_STATE_H_
#define INCLUDE_FEATURE_CAMERA_MEDIA_PLAYBACK_STATE_H_

#include <atomic>
#include <deque>
#include <memory>

#include <QByteArray>
#include <QElapsedTimer>
#include <QImage>
#include <QMutex>
#include <QTimer>
#include <QVector>
#include <QWaitCondition>

#include "cameraaudiobytequeue.h"
#include "camerapipelineframe.h"
#include "cameravideofiledecoder.h"

class QObject;
class QThread;

/**
 * \brief Plain state container shared between the decode thread and the worker
 *        thread for camera video/stream playback.
 *
 * Holds everything CameraMediaPlaybackController needs to drive playback: the
 * owned CameraVideoFileDecoder, the playback clock/position bookkeeping, the
 * present/delayed-submit/stream-retry timers, the decoded-frame and stream-audio
 * queues, and the adaptive stream-audio resampler servo state. It has no logic
 * of its own beyond resetClosed()/resetClock(); the controller mutates it.
 *
 * \note This struct is the boundary between two threads. The decode thread
 *       produces into m_decodedFrames (guarded by m_decodedFramesMutex, with
 *       m_decodedFramesAvailable/NotFull wait conditions) and into the decoder
 *       pending-state snapshot (guarded by m_decodeSnapshotMutex); the worker
 *       thread consumes both. The stream-audio buffer is guarded by
 *       m_streamAudioMutex. Cross-thread flags (m_decodeThreadStop,
 *       m_decodeFramesProduced, m_decodeReopening, m_streamReopenResetPending,
 *       etc.) are atomics.
 * \warning Fields are split by ownership: the schedule/clock/delayed-frame state
 *          (m_clock, m_basePositionMs, m_delayedFrames, rebuffering flags) is
 *          worker-owned and must not be written from the decode thread — the
 *          decode thread signals the worker via m_streamReopenResetPending
 *          instead of touching that state directly. The per-field inline
 *          comments document each item's threading contract.
 */
class CameraMediaPlaybackState
{
public:
    struct DelayedFrame
    {
        CameraPipelineFramePtr m_frame;
        qint64 m_dueMs = 0;
        quint64 m_captureEpoch = 0;
        quint64 m_generation = 0;
    };

    struct DecodedFrame
    {
        QImage m_image;
        qint64 m_positionMs = -1;
        QByteArray m_pcmS16Stereo;
        int m_audioSampleRate = 0;
        qint64 m_decodeMs = 0;
        bool m_eof = false;
        QString m_errorMessage;
    };

    explicit CameraMediaPlaybackState(QObject *timerParent = nullptr);

    void resetClosed();
    void resetClock();

    std::unique_ptr<CameraVideoFileDecoder> m_decoder;
    double m_frameRate = 25.0;
    qint64 m_positionMs = 0;
    qint64 m_durationMs = 0;
    bool m_playing = false;
    QElapsedTimer m_clock;
    quint64 m_tick = 0;
    qint64 m_basePositionMs = -1;
    qint64 m_lastFramePtsMs = -1;
    qint64 m_lastDecodeMs = 0;
    // Sub-millisecond residual carried between stream present ticks so the integer
    // QTimer averages the exact content frame interval (e.g. 33.33 ms at 30 fps)
    // instead of rounding to 33 ms and drifting ~1% fast — a drift the audio
    // resampler would otherwise track as slow pitch wander.
    double m_presentResidualMs = 0.0;
    // Diagnostic: video frames dropped in the last second to follow the audio clock. A
    // small steady rate is normal (source slightly faster than display); a large/growing
    // rate means video is chronically behind audio.
    int m_streamFramesDroppedThisSecond = 0;
    // Clean streaming present (see STREAM_REWRITE_PLAN.md): content PTS (ms) most recently
    // handed toward the device by streamTakeAudio. The master clock the video present follows
    // is this minus the monitor-FIFO + sink-latency. −1 until audio starts.
    qint64 m_streamPlayedPtsMs = -1;
    quint64 m_frameSubmitGeneration = 0;

    QTimer m_delayedSubmitTimer;
    QElapsedTimer m_delayedSubmitClock;
    QVector<DelayedFrame> m_delayedFrames;

    QTimer m_streamFrameRetryTimer;
    CameraPipelineFramePtr m_pendingStreamFrame;
    quint64 m_pendingStreamFrameGeneration = 0;

    QThread* m_decodeThread = nullptr;
    std::atomic_bool m_decodeThreadStop { false };
    std::atomic_bool m_decodeFrameWakeQueued { false };
    std::atomic<quint64> m_decodeDroppedSinceLastSubmit { 0 };
    // Monotonic count of real frames the decode thread has produced. The worker
    // thread watches it to detect a frozen decode (stuck inside readNextFrame on a
    // desynced live stream, which never surfaces as a read error) and forces a
    // reopen. m_decodeReopening guards the watchdog from firing mid-reopen.
    std::atomic<quint64> m_decodeFramesProduced { 0 };
    std::atomic_bool m_decodeReopening { false };
    // Set by the decode thread after it reopens a live stream at a new edge (it
    // has already cleared the stale audio + decoded-frame queue). The worker
    // thread observes this on its next present tick and resets its own playback
    // schedule (clock/rebuffer/pending/delayed frames) — that state is worker-
    // owned and must not be written from the decode thread.
    std::atomic_bool m_streamReopenResetPending { false };
    // Set by the worker: true while presentation is actively consuming frames, false while
    // (re)buffering. The decode thread reads it to switch behaviour: while buffering it
    // block-to-accumulates (lets the read-ahead build the bitstream cushion while the present
    // is held); during active playback it SELF-PACES at the frame rate and does NOT block on a
    // full queue, so the audio (decoded alongside the video) is fed at the full source rate
    // rather than throttled to the present's achievable display rate (~59 at 60 fps, which
    // starved the audio and pitched the resampler down). The present's own frame drop sheds
    // the small surplus the slow present can't display.
    std::atomic_bool m_streamPresentActive { false };
    // Decode-thread-owned self-pacing clock + frame counter (only used during active playback).
    QElapsedTimer m_decodePaceClock;
    quint64 m_decodePaceFrameCount = 0;
    bool m_decodePacePrevActive = false;
    quint64 m_streamWatchdogLastProduced = 0;
    QElapsedTimer m_streamWatchdogClock;
    mutable QMutex m_decodedFramesMutex;
    QWaitCondition m_decodedFramesAvailable;
    QWaitCondition m_decodedFramesNotFull;
    std::deque<DecodedFrame> m_decodedFrames;
    // Snapshot of decoder pending-buffer state, written by the decode thread and
    // read by the worker thread (to derive the audio-slaved playback clock), so
    // the worker never touches the decoder (owned by the decode thread) directly.
    mutable QMutex m_decodeSnapshotMutex;
    qint64 m_decodeAudioPositionMs = -1;
    int m_decodePendingAudioBytes = 0;
    int m_decodePendingVideoFrames = 0;
    int m_decodePendingVideoPackets = 0;
    mutable QMutex m_streamAudioMutex;
    CameraAudioByteQueue m_streamAudioPcmS16Stereo;
    int m_streamAudioSampleRate = 0;
    // Carried fractional-sample remainder for the audio-drop helpers
    // (dropPaced/dropTimedStreamPlaybackAudio), which discard the stream audio
    // matching dropped video frames so A/V stay aligned.
    double m_streamAudioPaceRemainderFrames = 0.0;
    // Adaptive resampler state for matching the source's audio content rate to the
    // sound-card output rate. The ratio is servoed to hold the stream-audio buffer
    // at a fixed depth, so audio is consumed at the producer's content rate (kept
    // in sync with the fill-servo-paced video) while the monitor is fed at the
    // device rate. This absorbs the source-vs-soundcard clock difference smoothly
    // (no overflow-drain clicks). m_streamAudioResamplePhase is the carried
    // fractional input position for inter-call continuity.
    double m_streamAudioResampleRatio = 1.0;
    double m_streamAudioResamplePhase = 0.0;
    // Diagnostic: per-second stats of the applied resample ratio (= audio playback
    // pitch) and audio buffer fill, so the "wander" (ratio swing) can be measured
    // rather than judged by ear.
    QElapsedTimer m_audioWanderClock;
    double m_audioWanderRatioMin = 2.0;
    double m_audioWanderRatioMax = 0.0;
    double m_audioWanderRatioSum = 0.0;
    double m_audioWanderFillMsSum = 0.0;
    int m_audioWanderCount = 0;
    // Set when a (re)buffering phase ends so the next resampler call trims the
    // stream-audio buffer back to its target depth. While presentation is paused
    // to rebuild the cushion the decoder keeps appending audio, so the buffer
    // sits well above target when playback resumes; trimming it once on resume
    // lets the soft-deadband servo start in-band instead of riding out a large
    // startup excursion.
    bool m_streamAudioTrimToTargetPending = false;
    // True while presentation is paused to (re)build the decoded-frame cushion,
    // at startup and after a stall drains the queue mid-playback.
    bool m_streamRebuffering = false;
    static constexpr int m_minDecodedStreamFrames = 6;
};

#endif // INCLUDE_FEATURE_CAMERA_MEDIA_PLAYBACK_STATE_H_
