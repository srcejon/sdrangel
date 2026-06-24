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
#include <memory>

#include <QByteArray>
#include <QElapsedTimer>
#include <QMutex>
#include <QTimer>
#include <QVector>

#include "cameraaudiobytequeue.h"
#include "camerapipelineframe.h"
#include "cameravideofiledecoder.h"

class QObject;

/**
 * \brief Plain state container shared between the decode thread and the worker
 *        thread for camera video/stream playback.
 *
 * Holds everything CameraMediaPlaybackController needs to drive playback: the
 * owned CameraVideoFileDecoder, the playback clock/position bookkeeping, the
 * present/delayed-submit/stream-retry timers, and the stream-audio buffer. It
 * has no logic of its own beyond resetClosed()/resetClock(); the controller
 * mutates it.
 *
 * \note Streams run the clean present: the decoder owns its own demux/decode
 *       threads (see CameraVideoFileDecoder) and the controller is the consumer.
 *       This container only crosses threads via the decoder pending-state
 *       snapshot (guarded by m_decodeSnapshotMutex), the stream-audio buffer
 *       (m_streamAudioMutex), and m_playing (m_playingMutex). The schedule/clock
 *       state is worker-owned; the per-field inline comments document the rest.
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
    // Smoothed video master clock (ffplay model). The exact audio clock (streamMasterClockMs)
    // is correct on average but advances in a staircase — it jumps ~2.5 frames each time the
    // audio device pulls a chunk from the monitor (fifo drops instantly while the sink-latency
    // EMA lags), then sits flat. Following it directly drops the skipped frames. Instead advance
    // this at wall-clock rate and gently pull it toward the exact clock (hard-resync on a large
    // gap), so the present shows one frame per tick. −1 until audio starts.
    double m_streamVideoClockMs = -1.0;
    QElapsedTimer m_streamVideoClockWall;
    // Video-only streams (no audio track) have no audio clock to slave to, so the master clock
    // free-runs at wall rate from this anchor (= the oldest decoded frame's PTS at each (re)start).
    // −1 while unanchored (buffering); set on the first playing tick.
    qint64 m_streamVideoOnlyAnchorMs = -1;
    QElapsedTimer m_streamVideoOnlyClock;
    // Set false at playback start; lets the present show the first available frame once as a
    // poster so the view isn't blank during the audio-latency gap before the clock reaches it.
    bool m_streamPosterShown = false;
    // Diagnostic: per-second MAX duration (ms) of presentStreamTick stages, to find the
    // bottleneck capping the present rate.
    double m_streamTickTotalMaxMs = 0.0;
    double m_streamTickAudioMaxMs = 0.0;
    double m_streamTickSubmitMaxMs = 0.0;
    // Diagnostic: actual present cadence — ticks per second (= effective present fps) and the
    // worst inter-tick gap, to see directly whether the timer is being delivered late.
    int m_streamPresentTicksThisSecond = 0;
    QElapsedTimer m_streamTickGapClock;
    double m_streamTickGapMaxMs = 0.0;
    // Non-circular A/V sync probe: baseline of (videoDecodeEdgePTS - audioDecodeEdgePTS) captured
    // at start. The two decode edges come from independent decoders' packet PTS, before any
    // present-side coupling, so DRIFT of the live skew away from this baseline is true A/V drift.
    bool m_streamAvSkewBaselineSet = false;
    qint64 m_streamAvSkewBaselineMs = 0;
    quint64 m_frameSubmitGeneration = 0;

    QTimer m_delayedSubmitTimer;
    QElapsedTimer m_delayedSubmitClock;
    QVector<DelayedFrame> m_delayedFrames;

    QTimer m_streamFrameRetryTimer;
    CameraPipelineFramePtr m_pendingStreamFrame;
    quint64 m_pendingStreamFrameGeneration = 0;

    // Count of stream frames the present dropped to catch up to the clock, consumed once per submit.
    std::atomic<quint64> m_decodeDroppedSinceLastSubmit { 0 };
    // Guards m_playing (the transport play/pause flag).
    mutable QMutex m_playingMutex;
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
    // Per-second throttle for the clean stream present diagnostic (presentStreamTick).
    QElapsedTimer m_audioWanderClock;
    // True while presentation is paused to (re)build the decoded-frame cushion,
    // at startup and after a stall drains the queue mid-playback.
    bool m_streamRebuffering = false;
    static constexpr int m_minDecodedStreamFrames = 6;
};

#endif // INCLUDE_FEATURE_CAMERA_MEDIA_PLAYBACK_STATE_H_
