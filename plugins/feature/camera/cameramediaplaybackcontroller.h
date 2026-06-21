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

#ifndef INCLUDE_FEATURE_CAMERA_MEDIA_PLAYBACK_CONTROLLER_H_
#define INCLUDE_FEATURE_CAMERA_MEDIA_PLAYBACK_CONTROLLER_H_

#include <functional>

#include <QObject>
#include <QString>
#include <QTimer>

#include "cameramediaplaybackstate.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class QImage;
class CameraQtAudioController;
class MessageQueue;

/**
 * \brief Owns all video/stream media-playback logic for the camera feature.
 *
 * Created on (and lives on) the worker thread, mirroring
 * CameraQtAudioController's affinity. It owns the CameraMediaPlaybackState
 * (m_state) and its own present QTimer (m_presentTimer) — the playback present
 * cadence no longer borrows the worker's capture timer (which the worker keeps
 * for ASI/Alpaca capture). A background decode thread (started by
 * startVideoFileDecodeThread) wakes the present path by queuing presentTick()
 * onto this object across threads, so presentTick is a slot.
 *
 * \note Slaves video presentation to the audio device clock (via the audio
 *       controller's monitor FIFO), runs an adaptive stream-audio resampler
 *       servo to absorb the source-vs-soundcard clock difference, and handles
 *       live-stream stall/reopen recovery.
 * \note Per-frame collaborator hooks back into the worker are kept as
 *       std::function callbacks (Callbacks), not signals, so the submit/exposure
 *       path stays a direct synchronous call on the worker thread.
 * \warning The decode thread runs in the background; cross-thread coordination
 *          goes through m_state's mutex-/atomic-guarded fields. The schedule and
 *          clock state in m_state is worker-owned — see CameraMediaPlaybackState.
 */
class CameraMediaPlaybackController : public QObject
{
    Q_OBJECT
public:
    // Per-frame / latency-sensitive collaborator hooks back into the worker. Kept
    // as std::function (not signals) so the pipeline submit/exposure path stays a
    // direct synchronous call on the worker thread.
    struct Callbacks
    {
        std::function<void(const CameraPipelineFramePtr&)> submitFrame;
        std::function<bool()> wouldReplacePending;
        std::function<void(CameraPipelineFrame&)> populateExposureMeta;
        std::function<quint64()> captureEpoch;
        std::function<bool()> capturing;
    };

    CameraMediaPlaybackController(
        CameraQtAudioController* audio,
        const CameraSettings* settings,
        QObject* parent = nullptr);
    ~CameraMediaPlaybackController() override;

    void setCallbacks(Callbacks callbacks) { m_callbacks = std::move(callbacks); }
    // The worker's input message queue, passed to the audio controller as the
    // AudioFifo notification queue when (re)starting file/stream playback audio.
    void setWorkerInputMessageQueue(MessageQueue* messageQueue) { m_workerInputMessageQueue = messageQueue; }

    // Lifecycle
    bool open();
    void close();
    bool isOpen() const { return m_state.m_decoder != nullptr; }

    // Transport
    void play();
    void pause();
    void restart();
    void seek(qint64 positionMs);
    void stepForward();
    void stepBackward();

    // Settings reactions
    void onPlaybackRateOrCadenceChanged();
    void onAudioOffsetChanged();

    bool isPlaying() const { return videoFilePlaybackIsPlaying(); }

signals:
    void playbackReport(qint64 positionMs, qint64 durationMs, bool playing, bool open);
    void error(const QString& key, const QString& title, const QString& msg);

public slots:
    // Queued from the decode thread (and fired by m_presentTimer) to advance the
    // present/submit A/V-sync chain.
    void presentTick();

private slots:
    void onDelayedSubmitTimer();      // releaseDelayedVideoFileFrames
    void onStreamRetryTimer();        // releasePendingStreamVideoFileFrame

private:
    // Moved verbatim from CameraWorker (bodies unchanged except m_mediaPlayback.->
    // m_state., collaborator rewiring and present-timer swap).
    bool openVideoFileDecoder();
    void closeVideoFileDecoder();
    void setVideoFilePlaying(bool playing);
    void submitVideoFileFrame(const CameraPipelineFramePtr& frame, bool applyPlaybackOffset);
    bool submitOrQueueStreamVideoFileFrame(const CameraPipelineFramePtr& frame);
    void clearPendingStreamVideoFileFrame();
    void releasePendingStreamVideoFileFrame();
    void clearDelayedVideoFileFrames();
    void scheduleDelayedVideoFileFrameSubmit();
    void releaseDelayedVideoFileFrames();
    void startVideoFileDecodeThread();
    void stopVideoFileDecodeThread();
    // Called from the decode thread to recover a live stream after a persistent
    // read failure by reopening the source from the current live edge. Returns
    // true once reopened, false if it gave up or a stop was requested.
    bool reopenStreamVideoFileDecoder(const QString& mediaSourcePath, int audioOutputSampleRate, double playbackFrameRate);
    void clearDecodedVideoFileFrames();
    bool videoFilePlaybackIsPlaying() const;
    void setVideoFilePlaybackPlayingState(bool playing);
    void queueDecodedVideoFileFrame(
        CameraMediaPlaybackState::DecodedFrame&& frame,
        int minBufferedFrames,
        int maxBufferedFrames,
        double playbackFrameRate);
    bool takeDecodedVideoFileFrame(CameraMediaPlaybackState::DecodedFrame& frame);
    void clearStreamPlaybackAudio();
    void appendStreamPlaybackAudio(const QByteArray& pcmS16Stereo, int audioSampleRate);
    // Pull up to maxOutputFrames of monitor audio, linear-resampling the stream
    // buffer by a ratio servoed to hold the buffer near targetBufferSeconds. Keeps
    // audio consumption at the producer's content rate (synced to video) while
    // emitting at the device sample rate, so the source/soundcard clock mismatch
    // is absorbed smoothly instead of overflow-drained (clicks).
    int takeResampledStreamPlaybackAudio(QByteArray& pcmS16Stereo, int audioSampleRate, int maxOutputFrames, double targetBufferSeconds);
    void submitResampledStreamAudio();
    int dropPacedStreamPlaybackAudio(int droppedVideoFrames, int audioSampleRate, double playbackFrameRate);
    int dropTimedStreamPlaybackAudio(qint64 durationMs, int audioSampleRate);
    int streamPlaybackAudioBytes() const;
    int streamPlaybackAudioSampleRate() const;
    int streamInitialBufferFrameCount() const;
    int decodedStreamFrameQueueDepth() const;
    int maxDecodedStreamFrameCount() const;
    bool readQueuedVideoFileFrame(bool submitAudio);
    qint64 updateVideoFilePlaybackPosition(qint64 decodedPositionMs, qint64 decodeMs, bool repairTimestampDiscontinuities, bool resetClockOnLargeDrift);
    void submitDecodedVideoFileFrame(
        const QImage& image,
        qint64 decodedPositionMs,
        qint64 decodeMs,
        const QByteArray& pcmS16Stereo,
        int audioSampleRate,
        bool submitAudio,
        bool applyPlaybackOffset,
        bool repairTimestampDiscontinuities,
        bool resetClockOnLargeDrift);
    void submitVideoFileAudio(const QByteArray& pcmS16Stereo, int audioSampleRate, qint64 contentPositionMs = -1);
    bool readVideoFileFrame(bool submitAudio = true, qint64 minimumPositionMs = -1);
    void seekVideoFile(qint64 positionMs, bool displayFrame);
    void stepVideoFile(int direction);
    double videoFileExactFrameIntervalMs() const;
    int videoFileFrameIntervalMs() const;
    void resetVideoFilePlaybackSchedule();
    qint64 videoFilePlaybackClockMs() const;
    void scheduleNextVideoFileTick();
    void reportVideoFilePlaybackToGUI();

    CameraQtAudioController* m_audio;
    const CameraSettings* m_settings;
    MessageQueue* m_workerInputMessageQueue;
    Callbacks m_callbacks;
    CameraMediaPlaybackState m_state;
    QTimer m_presentTimer;
};

#endif // INCLUDE_FEATURE_CAMERA_MEDIA_PLAYBACK_CONTROLLER_H_
