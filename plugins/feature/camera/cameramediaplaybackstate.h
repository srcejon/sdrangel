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
#include <thread>

#include <QByteArray>
#include <QElapsedTimer>
#include <QImage>
#include <QMutex>
#include <QTimer>
#include <QVector>
#include <QWaitCondition>

#include "camerapipelineframe.h"
#include "cameravideofiledecoder.h"

class QObject;

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
    quint64 m_frameSubmitGeneration = 0;

    QTimer m_delayedSubmitTimer;
    QElapsedTimer m_delayedSubmitClock;
    QVector<DelayedFrame> m_delayedFrames;

    QTimer m_streamFrameRetryTimer;
    CameraPipelineFramePtr m_pendingStreamFrame;
    quint64 m_pendingStreamFrameGeneration = 0;

    std::thread m_decodeThread;
    std::atomic_bool m_decodeThreadStop { false };
    std::atomic_bool m_decodeFrameWakeQueued { false };
    std::atomic<quint64> m_decodeDroppedSinceLastSubmit { 0 };
    // Monotonic count of real frames the decode thread has produced. The worker
    // thread watches it to detect a frozen decode (stuck inside readNextFrame on a
    // desynced live stream, which never surfaces as a read error) and forces a
    // reopen. m_decodeReopening guards the watchdog from firing mid-reopen.
    std::atomic<quint64> m_decodeFramesProduced { 0 };
    std::atomic_bool m_decodeReopening { false };
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
    QByteArray m_streamAudioPcmS16Stereo;
    int m_streamAudioSampleRate = 0;
    double m_streamAudioPaceRemainderFrames = 0.0;
    QElapsedTimer m_streamAudioPaceClock;
    // Content position (frame PTS, ms) of the last video frame whose audio was
    // pulled to the monitor. Audio is paced by the advance of the presented video
    // frame's position rather than wall-clock time, so it follows the video
    // timeline exactly and self-corrects after a stall/rebuffer instead of
    // accumulating a permanent lead. -1 until the first frame is presented.
    qint64 m_streamAudioLastPresentedPositionMs = -1;
    // Adaptive resampler state for matching the source's audio content rate to the
    // sound-card output rate. The ratio is servoed to hold the stream-audio buffer
    // at a fixed depth, so audio is consumed at the producer's content rate (kept
    // in sync with the fill-servo-paced video) while the monitor is fed at the
    // device rate. This absorbs the source-vs-soundcard clock difference smoothly
    // (no overflow-drain clicks). m_streamAudioResamplePhase is the carried
    // fractional input position for inter-call continuity.
    double m_streamAudioResampleRatio = 1.0;
    double m_streamAudioResamplePhase = 0.0;
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
