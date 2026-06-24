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

#include <algorithm>
#include <cmath>
#include <limits>

#include <QByteArray>
#include <QDateTime>
#include <QDebug>
#include <QElapsedTimer>
#include <QImage>
#include <QMetaObject>
#include <QThread>

#include "cameraqtaudiocontroller.h"
#include "cameravideofiledecoder.h"
#include "cameramediaplaybackcontroller.h"

#ifdef _WIN32
// Raise the system timer resolution to 1 ms while a stream plays so the present QTimer
// (Qt::PreciseTimer, ~16 ms at 60 fps) is actually delivered on time. At the default ~15.6 ms
// Windows scheduler granularity a 16 ms single-shot lands ~31–47 ms later, capping the present
// at ~24 fps (heavy frame-drop / judder) even though each tick's work is < 1 ms. Declared here
// (instead of pulling in <windows.h>) to keep this large TU clean; winmm is linked in CMake.
extern "C" __declspec(dllimport) unsigned int __stdcall timeBeginPeriod(unsigned int uPeriod);
extern "C" __declspec(dllimport) unsigned int __stdcall timeEndPeriod(unsigned int uPeriod);
#endif

CameraMediaPlaybackController::CameraMediaPlaybackController(
    CameraQtAudioController* audio,
    const CameraSettings* settings,
    QObject* parent) :
    QObject(parent),
    m_audio(audio),
    m_settings(settings),
    m_workerInputMessageQueue(nullptr),
    m_state(this),
    m_presentTimer(this)
{
    m_presentTimer.setTimerType(Qt::PreciseTimer);
    QObject::connect(&m_presentTimer, &QTimer::timeout, this, &CameraMediaPlaybackController::presentTick);
    QObject::connect(&m_state.m_delayedSubmitTimer, &QTimer::timeout, this, &CameraMediaPlaybackController::onDelayedSubmitTimer);
    QObject::connect(&m_state.m_streamFrameRetryTimer, &QTimer::timeout, this, &CameraMediaPlaybackController::onStreamRetryTimer);
}

CameraMediaPlaybackController::~CameraMediaPlaybackController()
{
    closeVideoFileDecoder();
}

// ---------------------------------------------------------------------------
// Lifecycle / transport entry points (thin glue called by the worker)
// ---------------------------------------------------------------------------

bool CameraMediaPlaybackController::open()
{
    ++m_state.m_frameSubmitGeneration;
    if (openVideoFileDecoder())
    {
        readVideoFileFrame();
        setVideoFilePlaying(true);
        return true;
    }
    return false;
}

void CameraMediaPlaybackController::close()
{
    ++m_state.m_frameSubmitGeneration;
    closeVideoFileDecoder();
}

void CameraMediaPlaybackController::play()
{
    setVideoFilePlaying(true);
}

void CameraMediaPlaybackController::pause()
{
    setVideoFilePlaying(false);
}

void CameraMediaPlaybackController::restart()
{
    if (m_settings->isStreamCamera())
    {
        closeVideoFileDecoder();
        if (m_callbacks.capturing() && openVideoFileDecoder())
        {
            readVideoFileFrame();
            setVideoFilePlaying(true);
        }
    }
    else
    {
        seekVideoFile(0, true);
        setVideoFilePlaying(true);
    }
}

void CameraMediaPlaybackController::seek(qint64 positionMs)
{
    if (!m_settings->isStreamCamera()) {
        seekVideoFile(positionMs, true);
    }
}

void CameraMediaPlaybackController::stepForward()
{
    if (!m_settings->isStreamCamera()) {
        stepVideoFile(1);
    }
}

void CameraMediaPlaybackController::stepBackward()
{
    if (!m_settings->isStreamCamera()) {
        stepVideoFile(-1);
    }
}

void CameraMediaPlaybackController::onAudioOffsetChanged()
{
    ++m_state.m_frameSubmitGeneration;
    clearDelayedVideoFileFrames();
}

void CameraMediaPlaybackController::onPlaybackRateOrCadenceChanged()
{
    const bool playing = videoFilePlaybackIsPlaying();
    if (playing && m_settings->isStreamCamera() && m_state.m_decoder)
    {
        m_state.m_decoder->setAudioPaceFrameRate(qMax(1.0, m_state.m_frameRate) * qMax(0.1, m_settings->m_videoPlaybackRate));
    }
    else if (playing)
    {
        m_presentTimer.start(videoFileFrameIntervalMs());
    }
}

void CameraMediaPlaybackController::onDelayedSubmitTimer()
{
    releaseDelayedVideoFileFrames();
}

void CameraMediaPlaybackController::onStreamRetryTimer()
{
    releasePendingStreamVideoFileFrame();
}

// ---------------------------------------------------------------------------
// Present tick (the FFmpeg branch of the worker's old captureTick)
// ---------------------------------------------------------------------------

void CameraMediaPlaybackController::presentTick()
{
    if (!m_callbacks.capturing()) {
        return;
    }

    if (!m_settings->isFfmpegMediaSource()) {
        return;
    }

    if (!videoFilePlaybackIsPlaying()) {
        return;
    }
    // Streams use the clean decoder-engine present (decoder owns demux + A/V decode threads).
    if (m_settings->isStreamCamera())
    {
        presentStreamTick();
        return;
    }
    // File playback only (streams returned above via presentStreamTick).
    const bool frameRead = readVideoFileFrame();
    if (m_callbacks.capturing()
        && videoFilePlaybackIsPlaying()
        && frameRead)
    {
        scheduleNextVideoFileTick();
    }
}

// ===================== Clean streaming present (URL sources) ===========================
// See STREAM_REWRITE_PLAN.md. The decoder owns the demux + audio/video decode threads; here
// we are just the consumer: feed the monitor from the decoder's audio buffer, derive the
// master clock from the consumed audio PTS, and show the decoder's video frames at PTS ≤
// clock (drop late, hold early). No controller-side decode thread / self-pace / pacing saga.

void CameraMediaPlaybackController::submitStreamAudio()
{
    if (!m_settings->isStreamCamera() || !m_state.m_decoder) {
        return;
    }
    const int audioSampleRate = qMax(1, m_audio->monitorSampleRate());
    // Top the monitor up to its target fill each tick; the device drains it at the real
    // sample rate, so refilling to target makes the output rate the device rate.
    const int targetFill = m_audio->monitorTargetFillFrames(audioSampleRate);
    const int currentFill = static_cast<int>(m_audio->monitorAudioFill());
    const int need = targetFill - currentFill;
    if (need <= 0) {
        return;
    }
    QByteArray audio;
    qint64 playedPtsMs = -1;
    const int got = m_state.m_decoder->streamTakeAudio(audio, need, playedPtsMs);
    if (got <= 0) {
        return;
    }
    m_audio->submitMonitorPcmSamples(audio, audioSampleRate);
    if (playedPtsMs >= 0) {
        m_state.m_streamPlayedPtsMs = playedPtsMs;
        // Anchor recorded audio on the source content timeline (the content just handed to
        // the device); the recorder lays it alongside the video PTS.
        m_audio->submitRecordingPcmSamples(audio, audioSampleRate, playedPtsMs);
    }
}

qint64 CameraMediaPlaybackController::streamMasterClockMs() const
{
    // Master clock = content PTS handed toward the device (streamTakeAudio) minus the audio
    // still ahead of the speaker: the monitor FIFO + the device's own output (sink) buffer.
    // = the content position at the SPEAKER now. Single buffer in the decoder ⇒ exact.
    if (m_state.m_streamPlayedPtsMs < 0) {
        return -1;
    }
    const int rate = qMax(1, m_audio->monitorSampleRate());
    const qint64 fifoMs = static_cast<qint64>(m_audio->monitorAudioFill()) * 1000 / rate;
    const qint64 sinkMs = m_audio->monitorSinkLatencyUSecs() / 1000;
    return m_state.m_streamPlayedPtsMs - fifoMs - sinkMs;
}

qint64 CameraMediaPlaybackController::streamVideoOnlyClockMs()
{
    // No audio track to slave to: free-run the master clock at wall rate, anchored to the oldest
    // decoded frame's PTS at (re)start (re-anchored each time we leave buffering, so a stall resumes
    // cleanly instead of fast-forwarding). Returns −1 until the first frame is decoded.
    if (m_state.m_streamVideoOnlyAnchorMs < 0)
    {
        const qint64 firstPts = m_state.m_decoder->streamPeekVideoFramePtsMs();
        if (firstPts < 0) {
            return -1;                                        // nothing decoded yet — hold
        }
        m_state.m_streamVideoOnlyAnchorMs = firstPts;
        m_state.m_streamVideoOnlyClock.start();
        return firstPts;
    }
    const double playbackRate = qMax(0.1, m_settings->m_videoPlaybackRate);
    return m_state.m_streamVideoOnlyAnchorMs
        + static_cast<qint64>(m_state.m_streamVideoOnlyClock.nsecsElapsed() / 1.0e6 * playbackRate);
}

void CameraMediaPlaybackController::setHighTimerResolution(bool enable)
{
    if (enable == m_timerResolutionRaised) {
        return;
    }
    m_timerResolutionRaised = enable;
#ifdef _WIN32
    if (enable) {
        timeBeginPeriod(1);
    } else {
        timeEndPeriod(1);
    }
#endif
}

void CameraMediaPlaybackController::submitStreamPresentFrame(const QImage& image, qint64 ptsMs)
{
    if (!m_callbacks.submitFrame) {
        return;
    }
    m_state.m_positionMs = ptsMs;
    CameraPipelineFramePtr frame(new CameraPipelineFrame);
    frame->m_image = image;
    m_callbacks.populateExposureMeta(*frame);
    frame->m_captureEpoch = m_callbacks.captureEpoch();
    frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
    frame->m_playbackActiveFrame = true;
    frame->m_playbackPositionMs = ptsMs;
    frame->m_playbackFrameRate = qMax(1.0, m_state.m_frameRate) * qMax(0.1, m_settings->m_videoPlaybackRate);
    submitVideoFileFrame(frame, false);
    reportVideoFilePlaybackToGUI();
}

bool CameraMediaPlaybackController::reconnectStreamDecoder()
{
    if (!m_state.m_decoder || !m_settings->isStreamCamera()) {
        return false;
    }
    const QString mediaSourcePath = m_settings->ffmpegMediaSourcePath();
    if (mediaSourcePath.isEmpty()) {
        return false;
    }
    const int audioOutputSampleRate = qMax(1, m_audio->monitorSampleRate());
    m_state.m_decoder->close();
    QString errorMessage;
    if (!m_state.m_decoder->open(
        mediaSourcePath, errorMessage, audioOutputSampleRate,
        qBound(CameraSettings::m_minStreamBufferingSeconds,
               m_settings->m_streamBufferingSeconds,
               CameraSettings::m_maxStreamBufferingSeconds),
        /*streaming=*/ true))
    {
        return false;
    }
    m_state.m_decoder->setStreamAudioTargetSeconds(0.30);
    m_state.m_frameRate = m_state.m_decoder->frameRate();
    // Re-gate the present from the reconnected live edge.
    m_state.m_streamPlayedPtsMs = -1;
    m_state.m_streamVideoClockMs = -1.0;
    m_state.m_streamAvSkewBaselineSet = false;
    m_state.m_streamRebuffering = false;
    m_state.m_basePositionMs = -1;
    m_state.m_streamPosterShown = false;
    return true;
}

void CameraMediaPlaybackController::presentStreamTick()
{
    if (!m_state.m_decoder) {
        return;
    }
    if (!m_state.m_streamTickGapClock.isValid()) {
        m_state.m_streamTickGapClock.start();
    } else {
        const double gapMs = m_state.m_streamTickGapClock.nsecsElapsed() / 1.0e6;
        m_state.m_streamTickGapClock.restart();
        m_state.m_streamTickGapMaxMs = qMax(m_state.m_streamTickGapMaxMs, gapMs);
    }
    ++m_state.m_streamPresentTicksThisSecond;

    const double intervalMs = qMax(1.0, videoFileExactFrameIntervalMs());

    // ===================== BUFFERING / REBUFFERING =====================
    // Fill to the streamBufferingSeconds cushion before (re)starting playback - at startup AND after
    // a stall. Don't feed the monitor (audio goes silent) and don't consume video, so the demux can
    // build the packet cushion and the decoders fill ahead; show the oldest decoded frame as a still
    // poster meanwhile. Once the cushion is full, start playing from that oldest frame. If the source
    // died, reconnect (with bounded back-off) - this is where a dead source surfaces (its cushion can
    // never fill).
    if (m_state.m_basePositionMs < 0)
    {
        if (m_settings->isStreamCamera() && m_state.m_decoder->streamSourceFailed())
        {
            if (reconnectStreamDecoder())
            {
                qDebug() << "CameraMediaPlayback: stream reconnected";
                m_streamReconnectAttempts = 0;
                m_state.m_streamPosterShown = false;          // poster the new source's first frame
            }
            else if (++m_streamReconnectAttempts <= m_maxStreamReconnectAttempts)
            {
                qDebug() << "CameraMediaPlayback: stream reconnect failed, attempt"
                         << m_streamReconnectAttempts << "of" << m_maxStreamReconnectAttempts;
                m_presentTimer.setSingleShot(true);
                m_presentTimer.start(500);                    // back off before retrying
                return;
            }
            else
            {
                qWarning() << "CameraMediaPlayback: stream reconnect gave up";
                emit error(QStringLiteral("streamReconnect:%1").arg(m_settings->ffmpegMediaSourcePath()),
                           tr("Stream lost"), tr("The stream ended and could not be reconnected."));
                setVideoFilePlaying(false);
                return;
            }
        }

        const int cushionFrames = streamBufferingCushionFrameCount();
        const int buffered = m_state.m_decoder->streamVideoPacketCount()
                           + m_state.m_decoder->streamDecodedVideoFrameCount();
        const bool haveAudio = m_state.m_decoder->streamHasAudio();
        const int monRate = qMax(1, m_audio->monitorSampleRate());
        const int audioBufMs = haveAudio
            ? (m_state.m_decoder->streamAudioBufferedFrames() * 1000 / monRate) : 0;
        // Keep the video-only wall clock unanchored while buffering so it re-anchors to the new
        // oldest frame on resume (no fast-forward after a stall).
        m_state.m_streamVideoOnlyAnchorMs = -1;
        // Show the oldest decoded frame as a still poster, once. At startup it is the first content
        // (and becomes the first played frame, so no jump); during a rebuffer m_streamPosterShown is
        // already set, so the last shown frame is held instead.
        if (!m_state.m_streamPosterShown)
        {
            QImage poster;
            qint64 posterPts = -1;
            if (m_state.m_decoder->streamPeekVideoFrameImage(poster, posterPts))
            {
                submitStreamPresentFrame(poster, posterPts);
                m_state.m_streamPosterShown = true;
            }
        }
        // Start once the video cushion is built AND - for audio streams - enough audio is buffered to
        // drive the master clock; otherwise the first playing tick underruns and bounces straight back
        // here. Video-only streams (no audio track) start on the video cushion alone.
        const bool audioReady = !haveAudio || (audioBufMs >= 50);
        if ((buffered >= cushionFrames) && audioReady)
        {
            const bool wasRebuffering = m_state.m_streamRebuffering;
            m_state.m_basePositionMs = 0;
            m_state.m_streamRebuffering = false;
            m_state.m_streamVideoClockMs = -1.0;              // re-anchor the clock on resume
            qDebug() << (wasRebuffering ? "CameraMediaPlayback: stream rebuffering done - resume"
                                        : "CameraMediaPlayback: stream playback start")
                     << "- bufferedFrames" << buffered << "cushion" << cushionFrames
                     << "haveAudio" << haveAudio << "audioBufMs" << audioBufMs;
        }
        m_presentTimer.setSingleShot(true);
        m_presentTimer.start(qMax(1, static_cast<int>(intervalMs / 2.0)));
        return;
    }

    // ===================== PLAYING =====================
    QElapsedTimer tickTimer;
    tickTimer.start();
    submitStreamAudio();
    const double tAudioMs = tickTimer.nsecsElapsed() / 1.0e6;

    // Master clock: the audio-at-speaker position for streams with an audio track; a wall clock
    // free-running from the first decoded frame for video-only streams (no audio to slave to).
    const bool haveAudio = m_state.m_decoder->streamHasAudio();
    // Smooth the staircase audio clock into a steady video master clock (see m_streamVideoClockMs).
    const qint64 exactClockMs = haveAudio ? streamMasterClockMs() : streamVideoOnlyClockMs();
    qint64 clockMs = exactClockMs;
    if (exactClockMs >= 0)
    {
        if ((m_state.m_streamVideoClockMs < 0.0) || !m_state.m_streamVideoClockWall.isValid())
        {
            m_state.m_streamVideoClockMs = exactClockMs;             // anchor
            m_state.m_streamVideoClockWall.start();
        }
        else
        {
            const double wallElapsedMs = m_state.m_streamVideoClockWall.nsecsElapsed() / 1.0e6;
            m_state.m_streamVideoClockWall.restart();
            const double playbackRate = qMax(0.1, m_settings->m_videoPlaybackRate);
            double c = m_state.m_streamVideoClockMs + wallElapsedMs * playbackRate;   // free-run at wall rate
            const double err = static_cast<double>(exactClockMs) - c;
            if (qAbs(err) > 250.0) {
                c = exactClockMs;                                    // large gap (start/stall/seek) → snap
            } else {
                c += err * 0.05;                                     // gentle pull: averages the staircase, tracks drift
            }
            // Keep the smoothed clock tethered to the true audio position: it may sit a little
            // below the staircase top (smoothing lag) but must never free-run ahead — when the
            // audio stalls, exactMs freezes while wall time keeps advancing, which would otherwise
            // push the video ahead of the (frozen) audio. Cap the lead so video freezes with audio.
            c = qBound(static_cast<double>(exactClockMs) - 150.0,
                       c,
                       static_cast<double>(exactClockMs) + 40.0);
            m_state.m_streamVideoClockMs = c;
        }
        clockMs = llround(m_state.m_streamVideoClockMs);
    }
    // Manual A/V trim: the single audio-offset slider (shared with file playback - see
    // submitVideoFileFrame applyPlaybackOffset) trims sync. Negative delays the video. Applied
    // always, including while the clock is still negative at startup, so the video gate opens at the
    // trimmed position and video + audio start together. Does not affect audio or recording.
    clockMs += m_settings->m_videoPlaybackAudioOffsetMs;

    // Rebuffer trigger: a mid-playback underrun (source fell behind) drops back into the buffering
    // phase to rebuild the cushion (one clean pause) and resume, or reconnect if the source died.
    // Audio streams signal it via the audio buffer draining; video-only streams via the video frame
    // queue AND packet read-ahead both running dry (no audio buffer to watch).
    const int monRate = qMax(1, m_audio->monitorSampleRate());
    const int audioBufMs = haveAudio ? (m_state.m_decoder->streamAudioBufferedFrames() * 1000 / monRate) : 0;
    const bool underrun = haveAudio
        ? (audioBufMs < 50)
        : ((m_state.m_decoder->streamDecodedVideoFrameCount()
            + m_state.m_decoder->streamVideoPacketCount()) <= 0);
    if (underrun)
    {
        m_state.m_streamRebuffering = true;
        m_state.m_basePositionMs = -1;
        qDebug() << "CameraMediaPlayback: stream rebuffering - underrun, haveAudio" << haveAudio
                 << "audioBufMs" << audioBufMs;
        m_presentTimer.setSingleShot(true);
        m_presentTimer.start(qMax(1, static_cast<int>(intervalMs / 2.0)));
        return;
    }

    // Show frames due by the master clock: drop late (keep the newest due), hold early.
    QImage shown;
    qint64 shownPts = -1;
    bool gotFrame = false;
    bool eof = false;
    int dropped = 0;
    for (;;)
    {
        const qint64 nextPts = m_state.m_decoder->streamPeekVideoFramePtsMs();
        if (nextPts < 0) {
            break;                                                   // nothing decoded yet
        }
        // Hold frames that aren't due yet - INCLUDING while the clock is still negative at startup
        // (audio handed but monitor+sink still ahead of the speaker). The old `clockMs >= 0` guard
        // here drained the whole queue every tick while the clock was negative, racing the video
        // silently through the cushion (the "frames with no audio" burst) and then stranding it
        // ahead of the clock (the pause) until the clock caught up. Holding from the poster until
        // the clock reaches the oldest frame starts video and audio together, no burst.
        if (nextPts > clockMs + static_cast<qint64>(intervalMs / 2.0)) {
            break;                                                   // not due yet — hold
        }
        QImage img;
        qint64 pts = -1;
        bool e = false;
        if (!m_state.m_decoder->streamTakeVideoFrame(img, pts, e)) {
            break;
        }
        if (e) { eof = true; break; }
        if (gotFrame) {
            ++dropped;                                               // previous one was late
        }
        shown = std::move(img);
        shownPts = pts;
        gotFrame = true;
    }
    if (eof)
    {
        if (m_settings->isStreamCamera())
        {
            // Live source ended → drop into the buffering phase, which reconnects (with back-off)
            // and then refills the cushion. (A dead source usually surfaces there directly via
            // streamSourceFailed; this covers the rarer clean-EOF case.)
            m_state.m_streamRebuffering = true;
            m_state.m_basePositionMs = -1;
            m_presentTimer.setSingleShot(true);
            m_presentTimer.start(qMax(1, static_cast<int>(intervalMs / 2.0)));
            return;
        }
        setVideoFilePlaying(false);
        return;
    }
    if (dropped > 0) {
        m_state.m_streamFramesDroppedThisSecond += dropped;
    }
    double tSubmitMs = 0.0;
    if (gotFrame)
    {
        const double tBeforeSubmit = tickTimer.nsecsElapsed() / 1.0e6;
        submitStreamPresentFrame(shown, shownPts);
        tSubmitMs = tickTimer.nsecsElapsed() / 1.0e6 - tBeforeSubmit;
    }

    const double tTotalMs = tickTimer.nsecsElapsed() / 1.0e6;
    m_state.m_streamTickTotalMaxMs = qMax(m_state.m_streamTickTotalMaxMs, tTotalMs);
    m_state.m_streamTickAudioMaxMs = qMax(m_state.m_streamTickAudioMaxMs, tAudioMs);
    m_state.m_streamTickSubmitMaxMs = qMax(m_state.m_streamTickSubmitMaxMs, tSubmitMs);

    // Per-second diagnostic.
    if (!m_state.m_audioWanderClock.isValid()) {
        m_state.m_audioWanderClock.start();
    }
    if (m_state.m_audioWanderClock.elapsed() >= 1000)
    {
        // Non-circular A/V sync probe: compare the two decoders' independent PTS edges. The
        // baseline absorbs the constant decode-order/interleave offset; avDriftMs is how far the
        // live skew has drifted from it = real A/V drift (was ~1.3%/s before the audio-PTS fix).
        const qint64 vEdge = m_state.m_decoder->streamVideoDecodedEdgePtsMs();
        const qint64 aEdge = m_state.m_decoder->streamAudioDecodedEdgePtsMs();
        qint64 avSkewMs = 0;
        qint64 avDriftMs = 0;
        if ((vEdge >= 0) && (aEdge >= 0))
        {
            avSkewMs = vEdge - aEdge;
            if (!m_state.m_streamAvSkewBaselineSet)
            {
                m_state.m_streamAvSkewBaselineSet = true;
                m_state.m_streamAvSkewBaselineMs = avSkewMs;
            }
            avDriftMs = avSkewMs - m_state.m_streamAvSkewBaselineMs;
        }
        qDebug().nospace()
            << "CameraMediaPlayback: stream - clockMs " << clockMs
            << " exactMs " << exactClockMs
            << " avSkewMs " << avSkewMs
            << " avDriftMs " << avDriftMs
            << " playedPts " << m_state.m_streamPlayedPtsMs
            << " monFifoMs " << (static_cast<qint64>(m_audio->monitorAudioFill()) * 1000 / qMax(1, m_audio->monitorSampleRate()))
            << " sinkMs " << (m_audio->monitorSinkLatencyUSecs() / 1000)
            << " offsetMs " << m_settings->m_videoPlaybackAudioOffsetMs
            << " audioBufMs " << (m_state.m_decoder->streamAudioBufferedFrames() * 1000 / qMax(1, m_audio->monitorSampleRate()))
            << " videoPkts " << m_state.m_decoder->streamVideoPacketCount()
            << " videoFrames " << m_state.m_decoder->streamDecodedVideoFrameCount()
            << " framesDropped/s " << m_state.m_streamFramesDroppedThisSecond
            << " rebuf " << (m_state.m_streamRebuffering ? 1 : 0)
            << " presentTicks/s " << m_state.m_streamPresentTicksThisSecond
            << " gapMaxMs " << qRound(m_state.m_streamTickGapMaxMs * 10) / 10.0
            << " tickMaxMs " << qRound(m_state.m_streamTickTotalMaxMs * 10) / 10.0
            << " (audio " << qRound(m_state.m_streamTickAudioMaxMs * 10) / 10.0
            << " submit " << qRound(m_state.m_streamTickSubmitMaxMs * 10) / 10.0 << ")";
        m_state.m_streamFramesDroppedThisSecond = 0;
        m_state.m_streamPresentTicksThisSecond = 0;
        m_state.m_streamTickGapMaxMs = 0.0;
        m_state.m_streamTickTotalMaxMs = 0.0;
        m_state.m_streamTickAudioMaxMs = 0.0;
        m_state.m_streamTickSubmitMaxMs = 0.0;
        m_state.m_audioWanderClock.restart();
    }

    // Reschedule at the frame interval (the buffering phase returns earlier with its own faster
    // re-check, so here playback is running).
    m_presentTimer.setSingleShot(true);
    m_presentTimer.start(qMax(1, static_cast<int>(intervalMs)));
}

// ---------------------------------------------------------------------------
// Moved verbatim from CameraWorker
// ---------------------------------------------------------------------------

bool CameraMediaPlaybackController::openVideoFileDecoder()
{
    closeVideoFileDecoder();

    const QString mediaSourcePath = m_settings->ffmpegMediaSourcePath();
    if (!m_settings->isFfmpegMediaSource() || mediaSourcePath.isEmpty()) {
        qWarning() << "CameraWorker: not opening FFmpeg media source -"
                   << (m_settings->isFfmpegMediaSource()
                       ? (m_settings->isStreamCamera()
                          ? QStringLiteral("stream URL is empty (the entered URL did not reach the worker settings)")
                          : QStringLiteral("video file path is empty"))
                       : QStringLiteral("current camera protocol is not an FFmpeg media source"));
        return false;
    }

    const int audioOutputSampleRate = m_audio->startFilePlayback(*m_settings, m_workerInputMessageQueue);
    m_state.m_decoder.reset(new CameraVideoFileDecoder());
    QString errorMessage;
    qDebug() << "CameraWorker: opening FFmpeg media source"
             << (m_settings->isStreamCamera() ? QStringLiteral("stream") : QStringLiteral("video"))
             << mediaSourcePath;
    if (!m_state.m_decoder->open(
        mediaSourcePath,
        errorMessage,
        audioOutputSampleRate,
        qBound(CameraSettings::m_minStreamBufferingSeconds,
               m_settings->m_streamBufferingSeconds,
               CameraSettings::m_maxStreamBufferingSeconds),
        /*streaming=*/ m_settings->isStreamCamera()))
    {
        qWarning() << "CameraWorker: FFmpeg media source open failed"
                   << mediaSourcePath
                   << errorMessage;
        emit error(
            QStringLiteral("videoFileOpen:%1").arg(mediaSourcePath),
            m_settings->isStreamCamera() ? tr("Stream could not be opened") : tr("Video file could not be opened"),
            errorMessage);
        m_state.m_decoder.reset();
        m_audio->stop();
        reportVideoFilePlaybackToGUI();
        return false;
    }

    m_state.m_positionMs = 0;
    m_state.m_durationMs = m_state.m_decoder->durationMs();
    m_state.m_frameRate = m_state.m_decoder->frameRate();
    setVideoFilePlaybackPlayingState(false);
    if (m_settings->isStreamCamera()) {
        // Clean streaming engine: the decoder runs its own demux + audio/video decode threads
        // (started inside open()). No controller-side decode thread. Hold the audio buffer at a
        // modest target — the cushion + the audio/sink latency live in the decoder's compressed
        // packet queues, not in the audio sample buffer.
        m_state.m_decoder->setStreamAudioTargetSeconds(0.30);
        m_state.m_streamPlayedPtsMs = -1;
        m_state.m_streamVideoClockMs = -1.0;
        m_state.m_streamAvSkewBaselineSet = false;
        m_state.m_streamRebuffering = false;
        m_streamReconnectAttempts = 0;
        m_state.m_basePositionMs = -1;
        m_state.m_streamPosterShown = false;
        // Make the ~16 ms present timer reliable (see setHighTimerResolution); without this the
        // present is delivered at ~42 ms on Windows and caps at ~24 fps.
        setHighTimerResolution(true);
    }
    reportVideoFilePlaybackToGUI();
    qDebug() << "CameraWorker: FFmpeg media source opened"
             << mediaSourcePath
             << "durationMs" << m_state.m_durationMs
             << "fps" << m_state.m_frameRate
             << "streamBufferingSeconds" << (m_settings->isStreamCamera() ? m_settings->m_streamBufferingSeconds : 0.0);
    return true;
}

void CameraMediaPlaybackController::closeVideoFileDecoder()
{
    setHighTimerResolution(false);
    m_presentTimer.stop();
    resetVideoFileDecodeState();
    clearPendingStreamVideoFileFrame();
    clearDelayedVideoFileFrames();
    m_state.resetClosed();
    if (m_settings->isFfmpegMediaSource()) {
        m_audio->stop();
    }
    reportVideoFilePlaybackToGUI();
}

void CameraMediaPlaybackController::setVideoFilePlaying(bool playing)
{
    if (!m_callbacks.capturing() || !m_settings->isFfmpegMediaSource() || !m_state.m_decoder)
    {
        setVideoFilePlaybackPlayingState(false);
        reportVideoFilePlaybackToGUI();
        return;
    }

    if (m_settings->isStreamCamera() && !playing) {
        setVideoFilePlaybackPlayingState(false);
    }

    if (m_settings->isStreamCamera())
    {
        clearPendingStreamVideoFileFrame();
        clearDelayedVideoFileFrames();
        clearStreamPlaybackAudio();
    }

    if (!m_settings->isStreamCamera() || playing) {
        setVideoFilePlaybackPlayingState(playing);
    }
    if (playing)
    {
        m_presentTimer.setSingleShot(true);
        resetVideoFilePlaybackSchedule();
        if (m_settings->isStreamCamera())
        {
            // Clean streaming present: the decoder threads (started in open) are already
            // filling the buffers; just kick the present loop, which gates on the cushion then
            // shows frames at the audio clock and re-schedules itself (presentStreamTick).
            m_state.m_streamPlayedPtsMs = -1;
            m_state.m_streamVideoClockMs = -1.0;
            m_state.m_streamAvSkewBaselineSet = false;
            m_state.m_streamRebuffering = false;
            m_streamReconnectAttempts = 0;
            m_state.m_basePositionMs = -1;
            m_state.m_streamPosterShown = false;
            m_presentTimer.start(qMax(1, videoFileFrameIntervalMs()));
        }
        else
        {
            scheduleNextVideoFileTick();
        }
    }
    else
    {
        m_presentTimer.stop();
        clearDelayedVideoFileFrames();
        m_state.resetClock();
    }
    reportVideoFilePlaybackToGUI();
}

void CameraMediaPlaybackController::submitVideoFileFrame(const CameraPipelineFramePtr& frame, bool applyPlaybackOffset)
{
    if (!frame || !m_callbacks.submitFrame) {
        return;
    }

    // Auto A/V-sync: the audio device output buffer holds ~one bufferSize of audio
    // that has been submitted but not yet played (monitorSinkLatencyUSecs, ~250 ms),
    // so without compensation video is presented that much ahead of the audio the
    // viewer actually hears. Delay the video frames by that latency here (file
    // playback only - streams are paced by the fill servo and stay in sync without
    // it). This uses the same delayed-frame path as the manual offset but does NOT
    // touch the pacing clock, so audio submission to the monitor is unaffected. A
    // negative manual offset still adds extra video delay on top as a residual trim.
    const int sinkLatencyMs = (applyPlaybackOffset && !m_settings->isStreamCamera())
        ? static_cast<int>(m_audio->monitorSinkLatencyUSecs() / 1000)
        : 0;
    const int manualVideoDelayMs = (applyPlaybackOffset && (m_settings->m_videoPlaybackAudioOffsetMs < 0))
        ? qBound(0, -m_settings->m_videoPlaybackAudioOffsetMs, -CameraSettings::m_minVideoPlaybackAudioOffsetMs)
        : 0;
    const int videoDelayMs = qBound(0, sinkLatencyMs + manualVideoDelayMs, -CameraSettings::m_minVideoPlaybackAudioOffsetMs);

    if (videoDelayMs <= 0)
    {
        if (m_settings->isStreamCamera()) {
            submitOrQueueStreamVideoFileFrame(frame);
            return;
        }

        frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        m_callbacks.submitFrame(frame);
        return;
    }

    if (!m_state.m_delayedSubmitClock.isValid()) {
        m_state.m_delayedSubmitClock.start();
    }

    CameraMediaPlaybackState::DelayedFrame delayedFrame;
    delayedFrame.m_frame = frame;
    delayedFrame.m_dueMs = m_state.m_delayedSubmitClock.elapsed() + videoDelayMs;
    delayedFrame.m_captureEpoch = frame->m_captureEpoch;
    delayedFrame.m_generation = m_state.m_frameSubmitGeneration;
    m_state.m_delayedFrames.append(delayedFrame);

    static constexpr qint64 maxDelayedFrameBytes = 512LL * 1024LL * 1024LL;
    static constexpr int hardMaxDelayedFrameCount = 64;
    const qint64 frameBytes = frame->m_image.isNull()
        ? qint64(0)
        : qMax<qint64>(
            1,
            static_cast<qint64>(frame->m_image.bytesPerLine()) * static_cast<qint64>(frame->m_image.height()));
    const int maxFramesByMemory = frameBytes > 0
        ? static_cast<int>(qMax<qint64>(1, maxDelayedFrameBytes / frameBytes))
        : hardMaxDelayedFrameCount;
    const int maxFramesByDelay = qMax(1, static_cast<int>(std::ceil(static_cast<double>(videoDelayMs) / videoFileExactFrameIntervalMs())) + 2);
    const int maxDelayedFrameCount = qBound(1, std::min({hardMaxDelayedFrameCount, maxFramesByMemory, maxFramesByDelay}), hardMaxDelayedFrameCount);
    if (m_state.m_delayedFrames.size() > maxDelayedFrameCount) {
        m_state.m_delayedFrames.remove(0, m_state.m_delayedFrames.size() - maxDelayedFrameCount);
    }

    scheduleDelayedVideoFileFrameSubmit();
}

bool CameraMediaPlaybackController::submitOrQueueStreamVideoFileFrame(const CameraPipelineFramePtr& frame)
{
    if (!frame || !m_callbacks.submitFrame) {
        return false;
    }

    if (!m_callbacks.capturing()
        || !m_settings->isStreamCamera()
        || !videoFilePlaybackIsPlaying()
        || (frame->m_captureEpoch != m_callbacks.captureEpoch()))
    {
        return false;
    }

    if (m_callbacks.wouldReplacePending())
    {
        m_state.m_pendingStreamFrame = frame;
        m_state.m_pendingStreamFrameGeneration = m_state.m_frameSubmitGeneration;
        if (!m_state.m_streamFrameRetryTimer.isActive()) {
            m_state.m_streamFrameRetryTimer.start(5);
        }
        return false;
    }

    if (m_state.m_pendingStreamFrame)
    {
        m_state.m_pendingStreamFrame.clear();
        m_state.m_pendingStreamFrameGeneration = 0;
    }

    frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
    m_callbacks.submitFrame(frame);
    return true;
}

void CameraMediaPlaybackController::clearPendingStreamVideoFileFrame()
{
    m_state.m_streamFrameRetryTimer.stop();
    m_state.m_pendingStreamFrame.clear();
    m_state.m_pendingStreamFrameGeneration = 0;
}

void CameraMediaPlaybackController::releasePendingStreamVideoFileFrame()
{
    if (!m_state.m_pendingStreamFrame)
    {
        clearPendingStreamVideoFileFrame();
        return;
    }

    if (!m_callbacks.capturing()
        || !m_settings->isStreamCamera()
        || !videoFilePlaybackIsPlaying()
        || !m_callbacks.submitFrame
        || (m_state.m_pendingStreamFrame->m_captureEpoch != m_callbacks.captureEpoch())
        || (m_state.m_pendingStreamFrameGeneration != m_state.m_frameSubmitGeneration))
    {
        clearPendingStreamVideoFileFrame();
        return;
    }

    if (m_callbacks.wouldReplacePending())
    {
        m_state.m_streamFrameRetryTimer.start(5);
        return;
    }

    CameraPipelineFramePtr frame = m_state.m_pendingStreamFrame;
    m_state.m_pendingStreamFrame.clear();
    m_state.m_pendingStreamFrameGeneration = 0;
    frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
    m_callbacks.submitFrame(frame);
}

void CameraMediaPlaybackController::clearDelayedVideoFileFrames()
{
    m_state.m_delayedSubmitTimer.stop();
    m_state.m_delayedFrames.clear();
    m_state.m_delayedSubmitClock.invalidate();
}

void CameraMediaPlaybackController::scheduleDelayedVideoFileFrameSubmit()
{
    if (m_state.m_delayedFrames.isEmpty()) {
        clearDelayedVideoFileFrames();
        return;
    }

    if (!m_state.m_delayedSubmitClock.isValid()) {
        m_state.m_delayedSubmitClock.start();
    }

    qint64 nextDueMs = m_state.m_delayedFrames.constFirst().m_dueMs;
    for (const CameraMediaPlaybackState::DelayedFrame& delayedFrame : m_state.m_delayedFrames) {
        nextDueMs = std::min(nextDueMs, delayedFrame.m_dueMs);
    }

    const int delayMs = static_cast<int>(qBound(
        qint64(1),
        nextDueMs - m_state.m_delayedSubmitClock.elapsed(),
        qint64(1000)));
    m_state.m_delayedSubmitTimer.setSingleShot(true);
    m_state.m_delayedSubmitTimer.start(delayMs);
}

void CameraMediaPlaybackController::releaseDelayedVideoFileFrames()
{
    if (!m_state.m_delayedSubmitClock.isValid() || m_state.m_delayedFrames.isEmpty()) {
        clearDelayedVideoFileFrames();
        return;
    }

    const qint64 nowMs = m_state.m_delayedSubmitClock.elapsed();
    int dueCount = 0;
    while ((dueCount < m_state.m_delayedFrames.size()) && (m_state.m_delayedFrames[dueCount].m_dueMs <= nowMs)) {
        ++dueCount;
    }

    if (dueCount <= 0)
    {
        scheduleDelayedVideoFileFrameSubmit();
        return;
    }

    const CameraMediaPlaybackState::DelayedFrame delayedFrame = m_state.m_delayedFrames[dueCount - 1];
    m_state.m_delayedFrames.remove(0, dueCount);

    if (m_callbacks.capturing()
        && m_settings->isFfmpegMediaSource()
        && videoFilePlaybackIsPlaying()
        && m_callbacks.submitFrame
        && (m_callbacks.captureEpoch() == delayedFrame.m_captureEpoch)
        && (m_state.m_frameSubmitGeneration == delayedFrame.m_generation)
        && delayedFrame.m_frame)
    {
        if (m_settings->isStreamCamera() && m_callbacks.wouldReplacePending())
        {
            scheduleDelayedVideoFileFrameSubmit();
            return;
        }
        delayedFrame.m_frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        m_callbacks.submitFrame(delayedFrame.m_frame);
    }

    scheduleDelayedVideoFileFrameSubmit();
}

void CameraMediaPlaybackController::resetVideoFileDecodeState()
{
    clearStreamPlaybackAudio();

    QMutexLocker snapshotLocker(&m_state.m_decodeSnapshotMutex);
    m_state.m_decodeAudioPositionMs = -1;
    m_state.m_decodePendingAudioBytes = 0;
    m_state.m_decodePendingVideoFrames = 0;
    m_state.m_decodePendingVideoPackets = 0;
}

bool CameraMediaPlaybackController::videoFilePlaybackIsPlaying() const
{
    QMutexLocker locker(&m_state.m_playingMutex);
    return m_state.m_playing;
}

void CameraMediaPlaybackController::setVideoFilePlaybackPlayingState(bool playing)
{
    QMutexLocker locker(&m_state.m_playingMutex);
    m_state.m_playing = playing;
}

void CameraMediaPlaybackController::clearStreamPlaybackAudio()
{
    QMutexLocker locker(&m_state.m_streamAudioMutex);
    m_state.m_streamAudioPcmS16Stereo.clear();
    m_state.m_streamAudioSampleRate = 0;
}

int CameraMediaPlaybackController::streamPlaybackAudioBytes() const
{
    QMutexLocker locker(&m_state.m_streamAudioMutex);
    return m_state.m_streamAudioPcmS16Stereo.size();
}

int CameraMediaPlaybackController::streamPlaybackAudioSampleRate() const
{
    QMutexLocker locker(&m_state.m_streamAudioMutex);
    return m_state.m_streamAudioSampleRate;
}

int CameraMediaPlaybackController::streamBufferingCushionFrameCount() const
{
    const double frameRate = qMax(1.0, m_state.m_frameRate);
    const double bufferingSeconds = qBound(
        CameraSettings::m_minStreamBufferingSeconds,
        m_settings->m_streamBufferingSeconds,
        CameraSettings::m_maxStreamBufferingSeconds);
    const int frameCount = static_cast<int>(std::ceil(frameRate * bufferingSeconds));
    return qMax(CameraMediaPlaybackState::m_minDecodedStreamFrames, frameCount);
}

qint64 CameraMediaPlaybackController::updateVideoFilePlaybackPosition(
    qint64 decodedPositionMs,
    qint64 decodeMs,
    bool repairTimestampDiscontinuities,
    bool resetClockOnLargeDrift)
{
    qint64 positionMs = decodedPositionMs;
    const bool droppedStreamFrames = m_state.m_decodeDroppedSinceLastSubmit.exchange(0) > 0;

    if (positionMs >= 0)
    {
        if (repairTimestampDiscontinuities && (m_state.m_lastFramePtsMs >= 0))
        {
            const qint64 frameIntervalMs = videoFileFrameIntervalMs();
            const qint64 positionDeltaMs = positionMs - m_state.m_lastFramePtsMs;
            if ((positionDeltaMs <= 0) || ((positionDeltaMs > frameIntervalMs * 3) && !droppedStreamFrames)) {
                positionMs = m_state.m_lastFramePtsMs + frameIntervalMs;
            }
        }
        m_state.m_positionMs = positionMs;
    }
    else
    {
        m_state.m_positionMs += videoFileFrameIntervalMs();
    }

    m_state.m_lastDecodeMs = decodeMs;
    m_state.m_lastFramePtsMs = m_state.m_positionMs;
    if (m_state.m_basePositionMs < 0)
    {
        m_state.m_basePositionMs = m_state.m_positionMs;
        if (!m_state.m_clock.isValid()) {
            m_state.m_clock.start();
        } else {
            m_state.m_clock.restart();
        }
    }

    qint64 videoLateMs = videoFilePlaybackClockMs() - m_state.m_positionMs;
    if (resetClockOnLargeDrift && (std::abs(videoLateMs) > 150))
    {
        m_state.m_basePositionMs = m_state.m_positionMs;
        if (!m_state.m_clock.isValid()) {
            m_state.m_clock.start();
        } else {
            m_state.m_clock.restart();
        }
        m_state.m_tick = 1;
        videoLateMs = 0;
    }
    return m_state.m_positionMs;
}

void CameraMediaPlaybackController::submitDecodedVideoFileFrame(
    const QImage& image,
    qint64 decodedPositionMs,
    qint64 decodeMs,
    const QByteArray& pcmS16Stereo,
    int audioSampleRate,
    bool submitAudio,
    bool applyPlaybackOffset,
    bool repairTimestampDiscontinuities,
    bool resetClockOnLargeDrift,
    qint64 audioPositionMs)
{
    const qint64 playbackPositionMs = updateVideoFilePlaybackPosition(
        decodedPositionMs,
        decodeMs,
        repairTimestampDiscontinuities,
        resetClockOnLargeDrift);

    if (submitAudio && (!pcmS16Stereo.isEmpty() || m_settings->isStreamCamera())) {
        // Tag the recording audio with the audio's OWN source PTS (audioPositionMs)
        // when the decoder provides it, so the recorder lays it on the source timeline;
        // it leads the video frame position by the monitor read-ahead. Fall back to the
        // video position when unknown. See CameraRecorder::appendAudioSamples.
        const qint64 audioContentMs = (audioPositionMs >= 0) ? audioPositionMs : playbackPositionMs;
        submitVideoFileAudio(pcmS16Stereo, audioSampleRate, audioContentMs);
    }

    if (m_callbacks.submitFrame)
    {
        CameraPipelineFramePtr frame(new CameraPipelineFrame);
        frame->m_image = image;
        m_callbacks.populateExposureMeta(*frame);
        frame->m_captureEpoch = m_callbacks.captureEpoch();
        frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        frame->m_playbackActiveFrame = true;
        frame->m_playbackPositionMs = playbackPositionMs;
        frame->m_playbackFrameRate = qMax(1.0, m_state.m_frameRate) * qMax(0.1, m_settings->m_videoPlaybackRate);
        submitVideoFileFrame(frame, applyPlaybackOffset);
    }

    reportVideoFilePlaybackToGUI();
}

bool CameraMediaPlaybackController::readVideoFileFrame(bool submitAudio, qint64 minimumPositionMs)
{
    if (!m_callbacks.capturing() || !m_settings->isFfmpegMediaSource() || !m_state.m_decoder) {
        return false;
    }

    if (m_settings->isStreamCamera())
    {
        // Streams are driven entirely by the clean present (presentStreamTick); the old
        // per-frame file-read path below does not apply to them.
        return false;
    }

    m_state.m_decoder->setAudioPaceFrameRate(
        qMax(1.0, m_state.m_decoder->frameRate()) * qMax(0.1, m_settings->m_videoPlaybackRate));

    QImage image;
    qint64 positionMs = -1;
    QByteArray pcmS16Stereo;
    int audioSampleRate = 0;
    QString errorMessage;
    QElapsedTimer decodeTimer;
    decodeTimer.start();
    bool readOk = false;
    qint64 decodeMs = 0;
    qint64 videoLateMs = 0;
    int droppedLateFrames = 0;
    static constexpr int maxLateDropFrames = 1;
    static constexpr qint64 maxLateDropDecodeMs = 80;
    // Live FLV/HTTP streams can report short timestamp discontinuities after
    // packet loss; avoid compounding those with aggressive catch-up drops.
    static constexpr qint64 liveFrameLateThresholdMs = 3000;
    for (;;)
    {
        image = QImage();
        positionMs = -1;
        pcmS16Stereo.clear();
        audioSampleRate = 0;
        errorMessage.clear();
        readOk = minimumPositionMs >= 0
            ? m_state.m_decoder->readNextFrameAtOrAfter(minimumPositionMs, image, positionMs, errorMessage)
            : m_state.m_decoder->readNextFrame(image, positionMs, pcmS16Stereo, audioSampleRate, errorMessage);
        decodeMs = decodeTimer.elapsed();
        if (!readOk || image.isNull() || (minimumPositionMs >= 0)) {
            break;
        }

        const qint64 framePtsMs = positionMs >= 0 ? positionMs : (m_state.m_lastFramePtsMs + videoFileFrameIntervalMs());
        if (m_state.m_basePositionMs < 0)
        {
            m_state.m_basePositionMs = framePtsMs;
            if (!m_state.m_clock.isValid()) {
                m_state.m_clock.start();
            } else {
                m_state.m_clock.restart();
            }
        }

        videoLateMs = videoFilePlaybackClockMs() - framePtsMs;
        if (!m_settings->isStreamCamera()
            || (videoLateMs <= liveFrameLateThresholdMs)
            || (droppedLateFrames >= maxLateDropFrames)
            || (decodeMs >= maxLateDropDecodeMs))
        {
            break;
        }

        if (submitAudio && (!pcmS16Stereo.isEmpty() || m_settings->isStreamCamera())) {
            submitVideoFileAudio(pcmS16Stereo, audioSampleRate);
        }
        ++droppedLateFrames;
    }
    if (!readOk)
    {
        emit error(
            QStringLiteral("videoFileDecode:%1").arg(m_settings->ffmpegMediaSourcePath()),
            m_settings->isStreamCamera() ? tr("Stream decode failed") : tr("Video file decode failed"),
            errorMessage);
        setVideoFilePlaying(false);
        return true;
    }

    if (image.isNull())
    {
        if (m_settings->m_videoLoop)
        {
            seekVideoFile(0, false);
            readVideoFileFrame();
        }
        else
        {
            setVideoFilePlaying(false);
        }
        return true;
    }

    // Source PTS of the audio just returned (front of the decoder's pending buffer),
    // so the recorder can place this audio on the source timeline rather than against
    // the read-ahead-leading video frame position.
    const qint64 audioStartMs = m_state.m_decoder ? m_state.m_decoder->lastReturnedAudioStartMs() : -1;
    submitDecodedVideoFileFrame(
        image,
        positionMs,
        decodeMs,
        pcmS16Stereo,
        audioSampleRate,
        submitAudio,
        submitAudio && (minimumPositionMs < 0),
        m_settings->isStreamCamera(),
        m_settings->isStreamCamera(),
        audioStartMs);
    return true;
}

void CameraMediaPlaybackController::submitVideoFileAudio(const QByteArray& pcmS16Stereo, int audioSampleRate, qint64 contentPositionMs)
{
    static constexpr int bytesPerSampleFrame = 4;
    if (audioSampleRate <= 0) {
        return;
    }

    QByteArray monitorAudio = pcmS16Stereo;
    // Maintain a small monitor cushion for FILE playback. The per-frame submission
    // feeds one video frame of audio per tick while the device consumes one per
    // tick, so without a top-up the FIFO drains to zero on any jitter and underruns
    // (glitching). The top-up gradually rebuilds the cushion to target. Streams do NOT use this
    // path (they run the clean present, presentStreamTick, and feed the monitor in submitStreamAudio).
    if (m_state.m_decoder && !m_settings->isStreamCamera())
    {
        const uint32_t currentFill = m_audio->monitorAudioFill();
        const int targetFillFrames = m_audio->monitorTargetFillFrames(audioSampleRate);
        if (currentFill < static_cast<uint32_t>(targetFillFrames))
        {
            const int neededFrames = targetFillFrames - static_cast<int>(currentFill);
            const int maxExtraFrames = audioSampleRate / 2;
            QByteArray extraAudio;
            const int extraFrames = m_state.m_decoder->takePendingAudio(extraAudio, std::min(neededFrames, maxExtraFrames));
            if (extraFrames > 0)
            {
                monitorAudio.append(extraAudio);
            }
        }
    }

    if (monitorAudio.isEmpty()) {
        return;
    }
    m_audio->submitMonitorPcmSamples(monitorAudio, audioSampleRate);
    // Record exactly what the monitor plays: monitorAudio, NOT pcmS16Stereo. For
    // file playback monitorAudio also carries the cushion top-up pulled above from
    // the decoder (extraAudio). That audio is consumed from the decoder's pending
    // buffer and never reappears in a later frame's pcmS16Stereo, so recording only
    // pcmS16Stereo drops it: the recorded track ends up shorter than the video and
    // plays too fast (most visible on heavy 4K playback, where irregular present
    // ticks drain the monitor and make the top-up frequent and large). For streams monitorAudio
    // carries no per-frame audio (the clean present feeds + records audio in submitStreamAudio), so
    // this stays a no-op there.
    const QByteArray& recordingAudio = monitorAudio;
    if (!recordingAudio.isEmpty()) {
        m_audio->submitRecordingPcmSamples(recordingAudio.left((recordingAudio.size() / bytesPerSampleFrame) * bytesPerSampleFrame), audioSampleRate, contentPositionMs);
    }
}

void CameraMediaPlaybackController::seekVideoFile(qint64 positionMs, bool displayFrame)
{
    if (!m_state.m_decoder || m_settings->isStreamCamera()) {
        return;
    }

    m_state.m_positionMs = qBound<qint64>(0, positionMs, m_state.m_durationMs > 0 ? m_state.m_durationMs : std::numeric_limits<qint64>::max());
    ++m_state.m_frameSubmitGeneration;
    clearDelayedVideoFileFrames();
    m_state.m_decoder->seek(m_state.m_positionMs);
    m_audio->clearMonitorAudio();
    m_audio->prefillMonitorAudio(m_audio->filePlaybackMonitorPrefillForOffsetMs());
    resetVideoFilePlaybackSchedule();
    reportVideoFilePlaybackToGUI();
    if (displayFrame) {
        readVideoFileFrame(false, m_state.m_positionMs);
    }
}

void CameraMediaPlaybackController::stepVideoFile(int direction)
{
    setVideoFilePlaying(false);
    ++m_state.m_frameSubmitGeneration;
    if (direction >= 0)
    {
        readVideoFileFrame(false);
    }
    else
    {
        const qint64 maxPosition = m_state.m_durationMs > 0 ? m_state.m_durationMs : std::numeric_limits<qint64>::max();
        const qint64 position = qBound<qint64>(
            0,
            m_state.m_positionMs - videoFileFrameIntervalMs(),
            maxPosition);
        seekVideoFile(position, true);
    }
}

int CameraMediaPlaybackController::videoFileFrameIntervalMs() const
{
    return qMax(1, static_cast<int>(std::round(videoFileExactFrameIntervalMs())));
}

double CameraMediaPlaybackController::videoFileExactFrameIntervalMs() const
{
    const double decoderFps = m_state.m_decoder ? m_state.m_frameRate : m_settings->m_framesPerSecond;
    return 1000.0 / (qMax(1.0, decoderFps) * qMax(0.1, m_settings->m_videoPlaybackRate));
}

void CameraMediaPlaybackController::resetVideoFilePlaybackSchedule()
{
    m_state.m_clock.restart();
    m_state.m_tick = 1;
    m_state.m_basePositionMs = -1;
    m_state.m_lastFramePtsMs = -1;
    m_state.m_lastDecodeMs = 0;
    m_state.m_presentResidualMs = 0.0;
    m_state.m_streamRebuffering = false;
}

qint64 CameraMediaPlaybackController::videoFilePlaybackClockMs() const
{
    static constexpr int bytesPerSampleFrame = 4;
    if (m_settings->isStreamCamera()
        && m_state.m_decoder
        && (m_audio->monitorSampleRate() > 0))
    {
        // Audio-master clock (the ffplay/VLC model): the playback clock IS the position of
        // the audio currently being HEARD at the speaker — decoded-audio position minus
        // everything still queued ahead of it (app queues + the device's own ~250 ms output
        // buffer). The present shows each video frame when this clock reaches its PTS and
        // DROPS late frames to catch up; video follows audio,
        // and audio is never rate-warped for sync. A feed underrun stalls this clock, but the
        // present bounds its wait (scheduleNextVideoFileTick) so the decoded queue drains into
        // a clean rebuffer instead of the old present<->decode deadlock.
        //
        // This replaced a long line of device-clock + rate-servo schemes (free-running
        // processedUSecs, PLL/integral-slope/clamped-trim). They all coupled the present rate
        // into the resampler buffer servo and either drained the buffer (clicks) or drifted
        // (the device crystal vs the resampled audio). Following the audio position directly,
        // with frame drop, is what real players do and sidesteps that coupling.
        const qint64 heardMs = streamDecodeDerivedContentMs();
        if (heardMs != std::numeric_limits<qint64>::min()) {
            return heardMs;
        }
        // Video-only stream (no audio to derive position from): fall through to wall clock.
    }
    if (!m_settings->isStreamCamera()
        && m_state.m_decoder
        && (m_state.m_decoder->audioDecodedPositionMs() >= 0)
        && (m_audio->monitorSampleRate() > 0))
    {
        const qint64 queuedAudioFrames =
            static_cast<qint64>(m_state.m_decoder->pendingAudioBytes() / bytesPerSampleFrame)
            + static_cast<qint64>(m_audio->monitorPlaybackClockFill());
        const qint64 queuedAudioMs = static_cast<qint64>(
            (static_cast<double>(queuedAudioFrames) * 1000.0 / static_cast<double>(m_audio->monitorSampleRate())) + 0.5);
        // NB: the audio device's own output buffer (~250 ms, see monitorSinkLatencyUSecs)
        // is deliberately NOT subtracted here. This clock also paces the decode/audio-submit
        // tick, so shifting it starves the monitor FIFO. The sink-buffer A/V skew is instead
        // compensated by delaying the video frames in submitVideoFileFrame().
        return m_state.m_decoder->audioDecodedPositionMs() - queuedAudioMs;
    }

    if (!m_state.m_clock.isValid() || (m_state.m_basePositionMs < 0)) {
        return m_state.m_positionMs;
    }

    const double rate = qMax(0.1, m_settings->m_videoPlaybackRate);
    return m_state.m_basePositionMs
        + static_cast<qint64>(std::llround(static_cast<double>(m_state.m_clock.elapsed()) * rate));
}

qint64 CameraMediaPlaybackController::streamDecodeDerivedContentMs() const
{
    // The audio content actually being HEARD at the speaker right now: the decoder's
    // decoded-audio position, minus everything still queued ahead of the speaker — the
    // app-side queues (decoder pending + stream resampler buffer + monitor FIFO) AND the
    // sound device's own output buffer (~250 ms, monitorSinkLatencyUSecs). Accurate while
    // the buffers are healthy, and (unlike the device clock) it reflects the resampled
    // SOURCE content rate, so it is the correct sync reference. Returns INT64_MIN when there
    // is no audio to derive it from (e.g. a video-only stream). Decode thread owns the
    // decoder, so this reads the snapshot the decode thread publishes.
    static constexpr int bytesPerSampleFrame = 4;
    if (!m_settings->isStreamCamera() || !m_state.m_decoder || (m_audio->monitorSampleRate() <= 0)) {
        return std::numeric_limits<qint64>::min();
    }
    qint64 audioDecodedPositionMs;
    qint64 decoderPendingAudioBytes;
    {
        QMutexLocker snapshotLocker(&m_state.m_decodeSnapshotMutex);
        audioDecodedPositionMs = m_state.m_decodeAudioPositionMs;
        decoderPendingAudioBytes = m_state.m_decodePendingAudioBytes;
    }
    if (audioDecodedPositionMs < 0) {
        return std::numeric_limits<qint64>::min();
    }
    const qint64 queuedAudioFrames =
        static_cast<qint64>((decoderPendingAudioBytes + streamPlaybackAudioBytes()) / bytesPerSampleFrame)
        + static_cast<qint64>(m_audio->monitorPlaybackClockFill());
    const qint64 queuedAudioMs = static_cast<qint64>(
        (static_cast<double>(queuedAudioFrames) * 1000.0 / static_cast<double>(m_audio->monitorSampleRate())) + 0.5);
    const qint64 sinkLatencyMs = m_audio->monitorSinkLatencyUSecs() / 1000;
    return audioDecodedPositionMs - queuedAudioMs - sinkLatencyMs;
}

void CameraMediaPlaybackController::scheduleNextVideoFileTick()
{
    if (!m_callbacks.capturing() || !videoFilePlaybackIsPlaying() || !m_settings->isFfmpegMediaSource() || !m_state.m_decoder) {
        return;
    }

    const double intervalMs = qMax(1.0, videoFileExactFrameIntervalMs());
    if (!m_state.m_clock.isValid()) {
        resetVideoFilePlaybackSchedule();
    }

    qint64 delayMs = static_cast<qint64>(std::llround(intervalMs));
    if ((m_state.m_basePositionMs >= 0) && (m_state.m_lastFramePtsMs >= 0))
    {
        // Present each frame when videoFilePlaybackClockMs() (the audio position being heard)
        // reaches its PTS — so video FOLLOWS audio (A/V sync), for both files and streams. For
        // streams this is now safe because the DECODE self-paces and drops-when-full
        // (queueDecodedVideoFileFrame), so the present no longer gates the feed; pacing it off
        // the audio clock therefore can't starve the audio. (Pacing streams off a steady wall
        // clock instead — needed before the decode decouple — left the displayed frame at a
        // fixed queue depth unrelated to the audio position, so video LED the audio by a
        // constant ~80ms+; the frame drop only fixes video that's behind, not ahead.)
        const qint64 nextFramePtsMs = m_state.m_lastFramePtsMs + static_cast<qint64>(std::llround(intervalMs));
        delayMs = nextFramePtsMs - videoFilePlaybackClockMs();
        delayMs -= m_state.m_lastDecodeMs;
        delayMs = qMax<qint64>(1, delayMs);
        // Bound the wait for streams: if the audio-heard clock stalls (full underrun) don't
        // wait ever longer — show within ~2 frames so the decoded queue drains into a clean
        // rebuffer. The decode is self-paced (not gated by this present), so it keeps feeding
        // audio and the clock soon resumes; this is just a safety, not the old deadlock path.
        if (m_settings->isStreamCamera()) {
            delayMs = qMin<qint64>(delayMs, static_cast<qint64>(std::llround(2.0 * intervalMs)));
        }
    }
    else
    {
        // Startup (no frame presented yet): pace off the wall clock until the audio clock and
        // lastFramePts are established.
        quint64 tick = m_state.m_tick > 0 ? m_state.m_tick : 1;
        const qint64 elapsedMs = m_state.m_clock.elapsed();
        qint64 targetMs = static_cast<qint64>(std::llround(static_cast<double>(tick) * intervalMs));

        if (elapsedMs - targetMs > static_cast<qint64>(std::llround(2.0 * intervalMs)))
        {
            tick = static_cast<quint64>(std::floor(static_cast<double>(elapsedMs) / intervalMs)) + 1;
            targetMs = static_cast<qint64>(std::llround(static_cast<double>(tick) * intervalMs));
        }

        delayMs = qMax<qint64>(1, targetMs - elapsedMs);
        m_state.m_tick = tick + 1;
    }
    const int timerDelayMs = static_cast<int>(qMin<qint64>(std::numeric_limits<int>::max(), delayMs));
    m_presentTimer.setSingleShot(true);
    m_presentTimer.start(timerDelayMs);
}

void CameraMediaPlaybackController::reportVideoFilePlaybackToGUI()
{
    emit playbackReport(
        m_state.m_positionMs,
        m_state.m_durationMs,
        videoFilePlaybackIsPlaying(),
        m_state.m_decoder != nullptr);
}
