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
    const bool frameRead = readVideoFileFrame();
    // Feed stream audio from the adaptive resampler once per tick (decoupled
    // from whether a video frame was presented), but not during rebuffering:
    // video is held to refill its cushion, so audio holds too and resyncs with
    // it on resume. The resampler tops the monitor to target at the device
    // rate while its ratio absorbs the source-vs-soundcard clock drift, so the
    // monitor never overfills (no overflow-drain clicks).
    if (m_settings->isStreamCamera()
        && (m_state.m_basePositionMs >= 0)
        && !m_state.m_streamRebuffering)
    {
        submitResampledStreamAudio();
    }
    // Watchdog: a desynced live stream can leave the decode thread stuck
    // inside readNextFrame chewing through garbage for a long time without
    // ever returning a read error, so its own reopen-on-failure path never
    // fires and video freezes indefinitely. The worker thread keeps ticking,
    // so detect "no new decoded frames for too long" here and force the decode
    // thread out via requestAbort(), which routes it into the reopen path.
    if (m_settings->isStreamCamera()
        && videoFilePlaybackIsPlaying()
        && (m_state.m_basePositionMs >= 0)
        && !m_state.m_decodeReopening.load())
    {
        static constexpr qint64 streamDecodeWatchdogMs = 6000;
        const quint64 produced = m_state.m_decodeFramesProduced.load();
        if (!m_state.m_streamWatchdogClock.isValid()
            || (produced != m_state.m_streamWatchdogLastProduced))
        {
            m_state.m_streamWatchdogLastProduced = produced;
            m_state.m_streamWatchdogClock.restart();
        }
        else if (m_state.m_streamWatchdogClock.elapsed() > streamDecodeWatchdogMs)
        {
            qWarning() << "CameraWorker: stream decode produced no frames for"
                       << m_state.m_streamWatchdogClock.elapsed()
                       << "ms; forcing decoder reopen";
            if (m_state.m_decoder) {
                m_state.m_decoder->requestAbort();
            }
            m_state.m_streamWatchdogClock.restart();
        }
    }
    if (m_callbacks.capturing()
        && videoFilePlaybackIsPlaying()
        && (m_settings->isStreamCamera() || frameRead))
    {
        scheduleNextVideoFileTick();
    }
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
        audioOutputSampleRate))
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
        m_state.m_decoder->setAudioPaceFrameRate(qMax(1.0, m_state.m_frameRate) * qMax(0.1, m_settings->m_videoPlaybackRate));
        // Let the decoder's live pending-audio cap track the buffering setting so a
        // deep buffer isn't undercut by the decoder trimming audio before it reaches
        // the (larger) downstream stream-audio buffer.
        m_state.m_decoder->setMaxLivePendingAudioMs(qMax(1200, static_cast<int>(m_settings->m_streamBufferingSeconds * 1000.0)));
        startVideoFileDecodeThread();
    }
    reportVideoFilePlaybackToGUI();
    qDebug() << "CameraWorker: FFmpeg media source opened"
             << mediaSourcePath
             << "durationMs" << m_state.m_durationMs
             << "fps" << m_state.m_frameRate
             << "streamBufferingSeconds" << (m_settings->isStreamCamera() ? m_settings->m_streamBufferingSeconds : 0.0)
             << "streamInitialFrames" << (m_settings->isStreamCamera() ? streamInitialBufferFrameCount() : 0)
             << "streamMaxFrames" << (m_settings->isStreamCamera() ? maxDecodedStreamFrameCount() : 0);
    return true;
}

void CameraMediaPlaybackController::closeVideoFileDecoder()
{
    m_presentTimer.stop();
    stopVideoFileDecodeThread();
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
        clearDecodedVideoFileFrames();
        clearStreamPlaybackAudio();
    }

    if (!m_settings->isStreamCamera() || playing) {
        setVideoFilePlaybackPlayingState(playing);
    }
    if (playing)
    {
        m_presentTimer.setSingleShot(true);
        resetVideoFilePlaybackSchedule();
        scheduleNextVideoFileTick();
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

void CameraMediaPlaybackController::startVideoFileDecodeThread()
{
    stopVideoFileDecodeThread();
    clearDecodedVideoFileFrames();

    if (!m_state.m_decoder || !m_settings->isStreamCamera()) {
        return;
    }

    const int minBufferedFrames = streamInitialBufferFrameCount();
    const int maxBufferedFrames = maxDecodedStreamFrameCount();
    const double playbackFrameRate = qMax(1.0, m_state.m_frameRate) * qMax(0.1, m_settings->m_videoPlaybackRate);
    const QString mediaSourcePath = m_settings->ffmpegMediaSourcePath();
    const int audioOutputSampleRate = qMax(1000, m_audio->monitorSampleRate());
    m_state.m_streamWatchdogClock.invalidate();
    m_state.m_streamWatchdogLastProduced = m_state.m_decodeFramesProduced.load();
    m_state.m_decodeReopening.store(false);
    m_state.m_decodeThreadStop.store(false);
    m_state.m_decodeThread = QThread::create([this, minBufferedFrames, maxBufferedFrames, playbackFrameRate, mediaSourcePath, audioOutputSampleRate]()
    {
        int consecutiveReadErrors = 0;
        static constexpr int maxConsecutiveReadErrors = 25;
        while (!m_state.m_decodeThreadStop.load())
        {
            CameraMediaPlaybackState::DecodedFrame decodedFrame;
            QString errorMessage;
            QElapsedTimer decodeTimer;
            decodeTimer.start();
            const bool readOk = m_state.m_decoder->readNextFrame(
                decodedFrame.m_image,
                decodedFrame.m_positionMs,
                decodedFrame.m_pcmS16Stereo,
                decodedFrame.m_audioSampleRate,
                errorMessage);
            decodedFrame.m_decodeMs = decodeTimer.elapsed();

            {
                QMutexLocker locker(&m_state.m_decodeSnapshotMutex);
                m_state.m_decodeAudioPositionMs = m_state.m_decoder->audioDecodedPositionMs();
                m_state.m_decodePendingAudioBytes = m_state.m_decoder->pendingAudioBytes();
                m_state.m_decodePendingVideoFrames = m_state.m_decoder->pendingVideoFrameCount();
                m_state.m_decodePendingVideoPackets = m_state.m_decoder->pendingVideoPacketCount();
            }

            if (!readOk)
            {
                if (consecutiveReadErrors < maxConsecutiveReadErrors)
                {
                    ++consecutiveReadErrors;
                    if ((consecutiveReadErrors == 1) || ((consecutiveReadErrors % 10) == 0)) {
                        qWarning() << "CameraWorker: stream decode read failed; retrying"
                                   << consecutiveReadErrors
                                   << errorMessage;
                    }
                    QThread::msleep(20);
                    continue;
                }
                // Persistent read failure on a live stream. Rather than ending
                // playback, reopen the source from the current live edge (a live
                // FLV/HTTP stream cannot be resumed byte-for-byte). If recovery
                // succeeds, resume decoding; only give up if it cannot be reopened.
                if (!m_state.m_decodeThreadStop.load()
                    && reopenStreamVideoFileDecoder(mediaSourcePath, audioOutputSampleRate, playbackFrameRate))
                {
                    consecutiveReadErrors = 0;
                    continue;
                }
                decodedFrame.m_errorMessage = errorMessage;
                queueDecodedVideoFileFrame(std::move(decodedFrame), minBufferedFrames, maxBufferedFrames, playbackFrameRate);
                break;
            }
            consecutiveReadErrors = 0;

            if (decodedFrame.m_image.isNull())
            {
                decodedFrame.m_eof = true;
                queueDecodedVideoFileFrame(std::move(decodedFrame), minBufferedFrames, maxBufferedFrames, playbackFrameRate);
                break;
            }

            m_state.m_decodeFramesProduced.fetch_add(1);
            queueDecodedVideoFileFrame(std::move(decodedFrame), minBufferedFrames, maxBufferedFrames, playbackFrameRate);
        }
    });
    m_state.m_decodeThread->start();
}

bool CameraMediaPlaybackController::reopenStreamVideoFileDecoder(const QString& mediaSourcePath, int audioOutputSampleRate, double playbackFrameRate)
{
    if (!m_state.m_decoder || mediaSourcePath.isEmpty()) {
        return false;
    }

    m_state.m_decodeReopening.store(true);
    struct ReopenGuard {
        std::atomic_bool& flag;
        ~ReopenGuard() { flag.store(false); }
    } reopenGuard { m_state.m_decodeReopening };

    // ~20 s of attempts with an abortable ~500 ms backoff between them, then give
    // up so a genuinely dead source still surfaces an error instead of spinning.
    static constexpr int maxReopenAttempts = 40;
    for (int attempt = 0; (attempt < maxReopenAttempts) && !m_state.m_decodeThreadStop.load(); ++attempt)
    {
        m_state.m_decoder->close();
        if (m_state.m_decodeThreadStop.load()) {
            return false;
        }

        QString errorMessage;
        if (m_state.m_decoder->open(mediaSourcePath, errorMessage, audioOutputSampleRate))
        {
            m_state.m_decoder->setAudioPaceFrameRate(playbackFrameRate);
            m_state.m_decoder->setMaxLivePendingAudioMs(qMax(1200, static_cast<int>(m_settings->m_streamBufferingSeconds * 1000.0)));
            // Drop stale pre-failure state so nothing from before the stall plays
            // against the new live edge: clear the stale audio AND the already
            // decoded video frames still queued (both safe from this thread — the
            // decoded-frame queue is mutex-guarded). The emptied queue makes the
            // worker rebuffer the cushion naturally. The worker-owned playback
            // schedule (clock/rebuffer/pending/delayed frames) can't be touched
            // from here, so flag it for the worker to reset on its next tick.
            clearStreamPlaybackAudio();
            clearDecodedVideoFileFrames();
            m_state.m_streamReopenResetPending.store(true);
            qDebug() << "CameraWorker: stream decoder reopened after read failure"
                     << mediaSourcePath << "attempt" << (attempt + 1);
            return true;
        }

        qWarning() << "CameraWorker: stream decoder reopen failed"
                   << mediaSourcePath << "attempt" << (attempt + 1) << errorMessage;
        for (int i = 0; (i < 25) && !m_state.m_decodeThreadStop.load(); ++i) {
            QThread::msleep(20);
        }
    }
    return false;
}

void CameraMediaPlaybackController::stopVideoFileDecodeThread()
{
    m_state.m_decodeThreadStop.store(true);
    if (m_state.m_decodeThread && m_state.m_decoder) {
        m_state.m_decoder->requestAbort();
    }
    m_state.m_decodedFramesNotFull.wakeAll();
    m_state.m_decodedFramesAvailable.wakeAll();
    if (m_state.m_decodeThread) {
        m_state.m_decodeThread->wait();
        delete m_state.m_decodeThread;
        m_state.m_decodeThread = nullptr;
    }
    m_state.m_decodeThreadStop.store(false);
    m_state.m_streamReopenResetPending.store(false);
    clearDecodedVideoFileFrames();
    clearStreamPlaybackAudio();

    {
        QMutexLocker snapshotLocker(&m_state.m_decodeSnapshotMutex);
        m_state.m_decodeAudioPositionMs = -1;
        m_state.m_decodePendingAudioBytes = 0;
        m_state.m_decodePendingVideoFrames = 0;
        m_state.m_decodePendingVideoPackets = 0;
    }
}

void CameraMediaPlaybackController::clearDecodedVideoFileFrames()
{
    QMutexLocker locker(&m_state.m_decodedFramesMutex);
    m_state.m_decodedFrames.clear();
    m_state.m_decodeFrameWakeQueued.store(false);
    m_state.m_decodedFramesNotFull.wakeAll();
}

bool CameraMediaPlaybackController::videoFilePlaybackIsPlaying() const
{
    QMutexLocker locker(&m_state.m_decodedFramesMutex);
    return m_state.m_playing;
}

void CameraMediaPlaybackController::setVideoFilePlaybackPlayingState(bool playing)
{
    QMutexLocker locker(&m_state.m_decodedFramesMutex);
    m_state.m_playing = playing;
    m_state.m_decodedFramesNotFull.wakeAll();
}

void CameraMediaPlaybackController::queueDecodedVideoFileFrame(
    CameraMediaPlaybackState::DecodedFrame&& frame,
    int minBufferedFrames,
    int maxBufferedFrames,
    double playbackFrameRate)
{
    QMutexLocker locker(&m_state.m_decodedFramesMutex);
    while (!m_state.m_playing && !m_state.m_decodeThreadStop.load())
    {
        m_state.m_decodedFramesNotFull.wait(&m_state.m_decodedFramesMutex, 20);
    }

    if (m_state.m_decodeThreadStop.load()) {
        return;
    }

    int droppedFrames = 0;
    qint64 firstDroppedPositionMs = -1;
    qint64 nextKeptPositionMs = -1;
    const int maxDecodedFrames = qMax(minBufferedFrames, maxBufferedFrames);
    while (m_state.m_playing
        && (m_state.m_decodedFrames.size() >= static_cast<size_t>(maxDecodedFrames)))
    {
        if ((firstDroppedPositionMs < 0) && (m_state.m_decodedFrames.front().m_positionMs >= 0)) {
            firstDroppedPositionMs = m_state.m_decodedFrames.front().m_positionMs;
        }
        m_state.m_decodedFrames.pop_front();
        m_state.m_decodeDroppedSinceLastSubmit.fetch_add(1);
        ++droppedFrames;
    }
    if (droppedFrames > 0)
    {
        const int audioSampleRate = streamPlaybackAudioSampleRate();
        if (audioSampleRate > 0)
        {
            if (!m_state.m_decodedFrames.empty()) {
                nextKeptPositionMs = m_state.m_decodedFrames.front().m_positionMs;
            } else {
                nextKeptPositionMs = frame.m_positionMs;
            }

            if ((firstDroppedPositionMs >= 0) && (nextKeptPositionMs > firstDroppedPositionMs)) {
                dropTimedStreamPlaybackAudio(nextKeptPositionMs - firstDroppedPositionMs, audioSampleRate);
            } else {
                dropPacedStreamPlaybackAudio(droppedFrames, audioSampleRate, playbackFrameRate);
            }
        }
    }

    appendStreamPlaybackAudio(frame.m_pcmS16Stereo, frame.m_audioSampleRate);
    frame.m_pcmS16Stereo.clear();

    const bool wasEmpty = m_state.m_decodedFrames.empty();
    m_state.m_decodedFrames.push_back(std::move(frame));
    m_state.m_decodedFramesAvailable.wakeAll();
    if (wasEmpty && !m_state.m_decodeFrameWakeQueued.exchange(true)) {
        QMetaObject::invokeMethod(this, "presentTick", Qt::QueuedConnection);
    }
}

bool CameraMediaPlaybackController::takeDecodedVideoFileFrame(CameraMediaPlaybackState::DecodedFrame& frame)
{
    QMutexLocker locker(&m_state.m_decodedFramesMutex);
    if (m_state.m_decodedFrames.empty()) {
        return false;
    }

    frame = std::move(m_state.m_decodedFrames.front());
    m_state.m_decodedFrames.pop_front();
    m_state.m_decodedFramesNotFull.wakeAll();
    return true;
}

void CameraMediaPlaybackController::clearStreamPlaybackAudio()
{
    QMutexLocker locker(&m_state.m_streamAudioMutex);
    m_state.m_streamAudioPcmS16Stereo.clear();
    m_state.m_streamAudioSampleRate = 0;
    m_state.m_streamAudioPaceRemainderFrames = 0.0;
    m_state.m_streamAudioResampleRatio = 1.0;
    m_state.m_streamAudioResamplePhase = 0.0;
    m_state.m_streamAudioTrimToTargetPending = false;
}

void CameraMediaPlaybackController::appendStreamPlaybackAudio(const QByteArray& pcmS16Stereo, int audioSampleRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if (pcmS16Stereo.isEmpty() || (audioSampleRate <= 0)) {
        return;
    }

    const int alignedBytes = (pcmS16Stereo.size() / bytesPerSampleFrame) * bytesPerSampleFrame;
    if (alignedBytes <= 0) {
        return;
    }

    QMutexLocker locker(&m_state.m_streamAudioMutex);
    if (m_state.m_streamAudioSampleRate != audioSampleRate)
    {
        m_state.m_streamAudioPcmS16Stereo.clear();
        m_state.m_streamAudioPaceRemainderFrames = 0.0;
        m_state.m_streamAudioSampleRate = audioSampleRate;
    }

    m_state.m_streamAudioPcmS16Stereo.append(pcmS16Stereo.constData(), alignedBytes);

    // Cap the stream-audio buffer well ABOVE the resampler's target depth
    // (~streamBufferingSeconds), otherwise the decoder's read-ahead bursts push
    // the buffer over the cap and drop the oldest audio every tick (audible
    // glitches). A fixed 2 s cap collided with the ~1.88 s target once buffering
    // was raised to 2 s. Scale the cap with the buffering setting (2x, floored at
    // 2 s so the historical 1 s-buffering default is unchanged) to leave room for
    // the read-ahead to be drained by the resampler instead of dropped.
    const double cappedBufferingSeconds = qBound(
        CameraSettings::m_minStreamBufferingSeconds,
        m_settings->m_streamBufferingSeconds,
        CameraSettings::m_maxStreamBufferingSeconds);
    const int maxBufferedBytes = static_cast<int>(
        static_cast<double>(audioSampleRate) * bytesPerSampleFrame * qMax(2.0, cappedBufferingSeconds * 2.0));
    if (m_state.m_streamAudioPcmS16Stereo.size() > maxBufferedBytes)
    {
        const int dropBytes = ((m_state.m_streamAudioPcmS16Stereo.size() - maxBufferedBytes) / bytesPerSampleFrame) * bytesPerSampleFrame;
        if (dropBytes > 0)
        {
            m_state.m_streamAudioPcmS16Stereo.consume(dropBytes);
        }
    }
}

int CameraMediaPlaybackController::takeResampledStreamPlaybackAudio(QByteArray& pcmS16Stereo, int audioSampleRate, int maxOutputFrames, double targetBufferSeconds)
{
    static constexpr int bytesPerSampleFrame = 4;
    pcmS16Stereo.clear();
    if ((audioSampleRate <= 0) || (maxOutputFrames <= 0)) {
        return 0;
    }

    QMutexLocker locker(&m_state.m_streamAudioMutex);
    if (m_state.m_streamAudioSampleRate != audioSampleRate) {
        return 0;
    }
    int availFrames = m_state.m_streamAudioPcmS16Stereo.size() / bytesPerSampleFrame;
    // Need at least two input frames to interpolate across.
    if (availFrames < 2) {
        return 0;
    }

    // One-shot trim to target on resume from a (re)buffering phase. The buffer
    // fills past target while presentation is paused; drop the excess oldest
    // audio once so playback resumes in-band (within the soft deadband) and the
    // servo holds steady instead of working off a large startup excursion.
    // Dropping the oldest samples also tightens the audio lead toward the
    // intended A/V offset rather than leaving audio running ahead of video.
    if (m_state.m_streamAudioTrimToTargetPending)
    {
        m_state.m_streamAudioTrimToTargetPending = false;
        const int targetFramesInt = static_cast<int>(
            qMax(1.0, static_cast<double>(audioSampleRate) * qMax(0.05, targetBufferSeconds)));
        if (availFrames > targetFramesInt)
        {
            const int dropFrames = availFrames - targetFramesInt;
            m_state.m_streamAudioPcmS16Stereo.consume(dropFrames * bytesPerSampleFrame);
            availFrames = targetFramesInt;
        }
    }

    // PI servo on the buffer level → resample ratio. A full buffer (source faster
    // than the device) raises the ratio so we consume input faster (audio rises
    // slightly in pitch to match); an empty buffer lowers it (audio stretches).
    // The integral state (m_streamAudioResampleRatio) converges to the true
    // source/device rate ratio and holds the buffer at target with NO steady-state
    // drift — a proportional-only servo left a small residual that slowly drained
    // the buffer to underrun over minutes. The proportional term gives transient
    // response; the integral is clamped for anti-windup.
    //
    // SOFT DEADBAND so the audible pitch is steady. The applied ratio IS the
    // playback speed, so a PI that never stops correcting couples buffer ripple
    // straight into an audible "wander". A HARD freeze inside the band (zero
    // correction) removes the ripple but lets the buffer drift unchecked to an
    // edge and back — a slow ~40 s edge-to-edge limit cycle with ~2.5% pitch
    // steps. Instead, scale BOTH gains by |err|/deadband (capped at 1): near
    // target the pull is ~0 (steady pitch, minimal ripple) and firms up smoothly
    // only as the buffer strays toward the band edge (full gains outside it for
    // fast recovery). Because a small restoring pull AND the proportional damping
    // are always present, the buffer SETTLES at target — converging to a constant
    // ratio instead of cycling — while the integral still nulls steady-state error
    // (no drift/drain). Continuous in |err|, so no pitch step anywhere.
    static constexpr double deadband = 0.10;
    const double targetFrames = qMax(1.0, static_cast<double>(audioSampleRate) * qMax(0.05, targetBufferSeconds));
    const double err = (static_cast<double>(availFrames) - targetFrames) / targetFrames;
    const double gainScale = qMin(1.0, std::abs(err) / deadband);
    m_state.m_streamAudioResampleRatio = qBound(0.90,
        m_state.m_streamAudioResampleRatio + 0.002 * gainScale * err, 1.10);
    const double ratio = qBound(0.88, m_state.m_streamAudioResampleRatio + 0.10 * gainScale * err, 1.12);

    const qint16 *in = reinterpret_cast<const qint16*>(m_state.m_streamAudioPcmS16Stereo.constData());
    const double phase = m_state.m_streamAudioResamplePhase;
    pcmS16Stereo.reserve(maxOutputFrames * bytesPerSampleFrame);
    int produced = 0;
    for (; produced < maxOutputFrames; ++produced)
    {
        const double pos = phase + static_cast<double>(produced) * ratio;
        const int idx = static_cast<int>(std::floor(pos));
        if ((idx < 0) || (idx + 1 >= availFrames)) {
            break;
        }
        const double frac = pos - static_cast<double>(idx);
        const auto lerp = [frac](qint16 a, qint16 b) -> qint16 {
            const double v = static_cast<double>(a) * (1.0 - frac) + static_cast<double>(b) * frac;
            return static_cast<qint16>(std::lround(qBound(-32768.0, v, 32767.0)));
        };
        const qint16 l = lerp(in[2 * idx], in[2 * (idx + 1)]);
        const qint16 r = lerp(in[2 * idx + 1], in[2 * (idx + 1) + 1]);
        const qint16 frame[2] = { l, r };
        pcmS16Stereo.append(reinterpret_cast<const char*>(frame), bytesPerSampleFrame);
    }
    if (produced <= 0) {
        return 0;
    }

    const double endPos = phase + static_cast<double>(produced) * ratio;
    const int consumed = static_cast<int>(std::floor(endPos));
    if (consumed > 0) {
        m_state.m_streamAudioPcmS16Stereo.consume(consumed * bytesPerSampleFrame);
        m_state.m_streamAudioResamplePhase = endPos - static_cast<double>(consumed);
    } else {
        m_state.m_streamAudioResamplePhase = endPos;
    }
    return produced;
}

void CameraMediaPlaybackController::submitResampledStreamAudio()
{
    if (!m_settings->isStreamCamera()) {
        return;
    }
    static constexpr int bytesPerSampleFrame = 4;
    const int audioSampleRate = streamPlaybackAudioSampleRate();
    if (audioSampleRate <= 0) {
        return;
    }
    // Top the monitor up to its target each tick: the device drains it at the
    // real sample rate, so refilling to target makes the output rate the device
    // rate while the resampler ratio absorbs the source/soundcard clock drift.
    const int targetFill = m_audio->monitorTargetFillFrames(audioSampleRate);
    const int currentFill = static_cast<int>(m_audio->monitorAudioFill());
    const int need = targetFill - currentFill;
    if (need <= 0) {
        return;
    }
    // Hold the audio buffer at (video buffering − monitor cushion) so the total
    // audio latency (this buffer + the ~120 ms monitor FIFO) matches the video
    // jitter-buffer latency and A/V stay lip-synced. Any residual offset is
    // trimmable with the playback audio offset setting.
    const double monitorCushionSeconds =
        static_cast<double>(m_audio->monitorTargetFillFrames(audioSampleRate)) / static_cast<double>(audioSampleRate);
    const double targetBufferSeconds = qMax(0.25, m_settings->m_streamBufferingSeconds - monitorCushionSeconds);
    QByteArray audio;
    const int got = takeResampledStreamPlaybackAudio(audio, audioSampleRate, need, targetBufferSeconds);
    if (got <= 0) {
        return;
    }
    m_audio->submitMonitorPcmSamples(audio, audioSampleRate);
    m_audio->submitRecordingPcmSamples(audio.left((audio.size() / bytesPerSampleFrame) * bytesPerSampleFrame), audioSampleRate);
}

int CameraMediaPlaybackController::dropPacedStreamPlaybackAudio(int droppedVideoFrames, int audioSampleRate, double playbackFrameRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if ((droppedVideoFrames <= 0) || (audioSampleRate <= 0) || (playbackFrameRate <= 0.0)) {
        return 0;
    }

    QMutexLocker locker(&m_state.m_streamAudioMutex);
    if ((m_state.m_streamAudioSampleRate != audioSampleRate) || m_state.m_streamAudioPcmS16Stereo.isEmpty()) {
        return 0;
    }

    const double targetFramesExact = (static_cast<double>(audioSampleRate) * static_cast<double>(droppedVideoFrames) / playbackFrameRate)
        + m_state.m_streamAudioPaceRemainderFrames;
    const int targetFrames = qMax(1, static_cast<int>(std::floor(targetFramesExact)));
    m_state.m_streamAudioPaceRemainderFrames = targetFramesExact - static_cast<double>(targetFrames);

    const int dropBytes = qMin(
        targetFrames * bytesPerSampleFrame,
        (m_state.m_streamAudioPcmS16Stereo.size() / bytesPerSampleFrame) * bytesPerSampleFrame);
    if (dropBytes <= 0) {
        return 0;
    }

    m_state.m_streamAudioPcmS16Stereo.consume(dropBytes);
    return dropBytes / bytesPerSampleFrame;
}

int CameraMediaPlaybackController::dropTimedStreamPlaybackAudio(qint64 durationMs, int audioSampleRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if ((durationMs <= 0) || (audioSampleRate <= 0)) {
        return 0;
    }

    QMutexLocker locker(&m_state.m_streamAudioMutex);
    if ((m_state.m_streamAudioSampleRate != audioSampleRate) || m_state.m_streamAudioPcmS16Stereo.isEmpty()) {
        return 0;
    }

    const double targetFramesExact = (static_cast<double>(audioSampleRate) * static_cast<double>(durationMs) / 1000.0)
        + m_state.m_streamAudioPaceRemainderFrames;
    const int targetFrames = qMax(1, static_cast<int>(std::floor(targetFramesExact)));
    m_state.m_streamAudioPaceRemainderFrames = targetFramesExact - static_cast<double>(targetFrames);

    const int dropBytes = qMin(
        targetFrames * bytesPerSampleFrame,
        (m_state.m_streamAudioPcmS16Stereo.size() / bytesPerSampleFrame) * bytesPerSampleFrame);
    if (dropBytes <= 0) {
        return 0;
    }

    m_state.m_streamAudioPcmS16Stereo.consume(dropBytes);
    return dropBytes / bytesPerSampleFrame;
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

int CameraMediaPlaybackController::streamInitialBufferFrameCount() const
{
    const double frameRate = qMax(1.0, m_state.m_frameRate);
    const double bufferingSeconds = qBound(
        CameraSettings::m_minStreamBufferingSeconds,
        m_settings->m_streamBufferingSeconds,
        CameraSettings::m_maxStreamBufferingSeconds);
    const int frameCount = static_cast<int>(std::ceil(frameRate * bufferingSeconds));
    return qMax(CameraMediaPlaybackState::m_minDecodedStreamFrames, frameCount);
}

int CameraMediaPlaybackController::decodedStreamFrameQueueDepth() const
{
    QMutexLocker locker(&m_state.m_decodedFramesMutex);
    return static_cast<int>(m_state.m_decodedFrames.size());
}

int CameraMediaPlaybackController::maxDecodedStreamFrameCount() const
{
    const double frameRate = qMax(1.0, m_state.m_frameRate);
    const double bufferingSeconds = qBound(
        CameraSettings::m_minStreamBufferingSeconds,
        m_settings->m_streamBufferingSeconds,
        CameraSettings::m_maxStreamBufferingSeconds);
    const int timeFrameCount = static_cast<int>(std::ceil(frameRate * bufferingSeconds * 2.0));
    return qMax(CameraMediaPlaybackState::m_minDecodedStreamFrames, timeFrameCount);
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
    bool resetClockOnLargeDrift)
{
    const qint64 playbackPositionMs = updateVideoFilePlaybackPosition(
        decodedPositionMs,
        decodeMs,
        repairTimestampDiscontinuities,
        resetClockOnLargeDrift);

    if (submitAudio && (!pcmS16Stereo.isEmpty() || m_settings->isStreamCamera())) {
        // Tag the recording audio with this frame's content position so the recorder
        // can re-align it with the (content-timestamped) video; see
        // CameraRecorder::appendAudioSamples / setAudioLeadSilenceMs.
        submitVideoFileAudio(pcmS16Stereo, audioSampleRate, playbackPositionMs);
    }

    if (m_callbacks.submitFrame)
    {
        CameraPipelineFramePtr frame(new CameraPipelineFrame);
        frame->m_image = image;
        m_callbacks.populateExposureMeta(*frame);
        frame->m_captureEpoch = m_callbacks.captureEpoch();
        frame->m_pipelineInputWallClockMs = QDateTime::currentMSecsSinceEpoch();
        frame->m_playbackPositionMs = playbackPositionMs;
        frame->m_playbackFrameRate = qMax(1.0, m_state.m_frameRate) * qMax(0.1, m_settings->m_videoPlaybackRate);
        submitVideoFileFrame(frame, applyPlaybackOffset);
    }

    reportVideoFilePlaybackToGUI();
}

bool CameraMediaPlaybackController::readQueuedVideoFileFrame(bool submitAudio)
{
    if (m_settings->isStreamCamera())
    {
        if (!videoFilePlaybackIsPlaying()) {
            return false;
        }

        // The decode thread reopened the stream at a new live edge (and cleared the
        // stale audio + decoded-frame queue). Reset the worker-owned playback
        // schedule so presentation restarts cleanly from the live edge instead of
        // replaying pre-stall clock/pending-frame state against the new audio.
        if (m_state.m_streamReopenResetPending.exchange(false))
        {
            clearPendingStreamVideoFileFrame();
            clearDelayedVideoFileFrames();
            resetVideoFilePlaybackSchedule();
        }

        m_state.m_decodeFrameWakeQueued.store(false);
        int queuedFrames = 0;
        {
            QMutexLocker locker(&m_state.m_decodedFramesMutex);
            queuedFrames = static_cast<int>(m_state.m_decodedFrames.size());
            m_state.m_decodedFramesNotFull.wakeAll();
        }

        // (Re)buffer a cushion of frames before presenting: at startup
        // (basePositionMs < 0) and again whenever a network stall drains the
        // queue mid-playback. Without rebuffering, an empty queue makes the decode
        // thread wake presentTick on every arriving frame (see
        // queueDecodedVideoFileFrame), so frames are presented at the producer's
        // bursty rate, the buffer never refills, and the downstream post-processor
        // drops frames continuously. Holding here lets the fill servo regain
        // control once the cushion is rebuilt.
        const int rebufferTarget = streamInitialBufferFrameCount();
        if (queuedFrames <= 0) {
            if (m_state.m_basePositionMs >= 0) {
                m_state.m_streamRebuffering = true;
            }
            return false;
        }

        const bool buffering = (m_state.m_basePositionMs < 0) || m_state.m_streamRebuffering;
        if (buffering && (queuedFrames < rebufferTarget))
        {
            m_presentTimer.setSingleShot(true);
            m_presentTimer.start(qMax(1, videoFileFrameIntervalMs() / 2));
            return false;
        }
        if (buffering)
        {
            // The cushion has been rebuilt and presentation is about to resume.
            // The stream-audio buffer filled past the resampler's target depth
            // while we were paused, so flag it for a one-shot trim back to target
            // (see takeResampledStreamPlaybackAudio) — covers both the initial
            // startup and any mid-playback rebuffer after a stall.
            QMutexLocker audioLocker(&m_state.m_streamAudioMutex);
            m_state.m_streamAudioTrimToTargetPending = true;
        }
        m_state.m_streamRebuffering = false;
    }

    CameraMediaPlaybackState::DecodedFrame decodedFrame;
    if (!takeDecodedVideoFileFrame(decodedFrame)) {
        return false;
    }

    if (!decodedFrame.m_errorMessage.isEmpty())
    {
        emit error(
            QStringLiteral("videoFileDecode:%1").arg(m_settings->ffmpegMediaSourcePath()),
            tr("Stream decode failed"),
            decodedFrame.m_errorMessage);
        setVideoFilePlaying(false);
        return true;
    }

    if (decodedFrame.m_eof || decodedFrame.m_image.isNull())
    {
        setVideoFilePlaying(false);
        return true;
    }

    QByteArray pcmS16Stereo = decodedFrame.m_pcmS16Stereo;
    int audioSampleRate = decodedFrame.m_audioSampleRate;
    if (m_settings->isStreamCamera())
    {
        // Stream audio is no longer pulled per presented frame. It is fed to the
        // monitor by the adaptive resampler (submitResampledStreamAudio, run every
        // tick), which matches the source's content rate to the sound-card rate
        // and keeps audio synced to the fill-servo-paced video. Submit the frame
        // with no audio here so the resampler is the single audio source.
        pcmS16Stereo.clear();
        audioSampleRate = streamPlaybackAudioSampleRate();
    }

    submitDecodedVideoFileFrame(
        decodedFrame.m_image,
        decodedFrame.m_positionMs,
        decodedFrame.m_decodeMs,
        pcmS16Stereo,
        audioSampleRate,
        submitAudio,
        submitAudio,
        true,
        true);
    return true;
}

bool CameraMediaPlaybackController::readVideoFileFrame(bool submitAudio, qint64 minimumPositionMs)
{
    if (!m_callbacks.capturing() || !m_settings->isFfmpegMediaSource() || !m_state.m_decoder) {
        return false;
    }

    if (m_settings->isStreamCamera() && (minimumPositionMs < 0))
    {
        return readQueuedVideoFileFrame(submitAudio);
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

    submitDecodedVideoFileFrame(
        image,
        positionMs,
        decodeMs,
        pcmS16Stereo,
        audioSampleRate,
        submitAudio,
        submitAudio && (minimumPositionMs < 0),
        m_settings->isStreamCamera(),
        m_settings->isStreamCamera());
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
    // (glitching). The top-up gradually rebuilds the cushion to target. Streams do
    // NOT use this path: their audio is fed entirely by submitResampledStreamAudio
    // (the adaptive resampler), which is the single stream-audio source; here a
    // stream call carries empty audio and is a no-op.
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
    // ticks drain the monitor and make the top-up frequent and large). For streams
    // monitorAudio carries no per-frame audio (the resampler in
    // submitResampledStreamAudio is the single stream recording-audio source), so
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
    m_state.m_streamRebuffering = false;
}

qint64 CameraMediaPlaybackController::videoFilePlaybackClockMs() const
{
    static constexpr int bytesPerSampleFrame = 4;
    if (m_settings->isStreamCamera()
        && m_state.m_decoder
        && (m_audio->monitorSampleRate() > 0))
    {
        // Slave stream video to the audio device clock. Audio is consumed at the
        // sound card's true rate, so deriving the playback clock from the audio
        // playback position paces video to the real content rate instead of the
        // reported frame rate (which is slightly inaccurate and otherwise slowly
        // drains the video jitter buffer). The decode thread owns the decoder, so
        // read its snapshotted audio position rather than the decoder directly.
        qint64 audioDecodedPositionMs;
        qint64 decoderPendingAudioBytes;
        {
            QMutexLocker snapshotLocker(&m_state.m_decodeSnapshotMutex);
            audioDecodedPositionMs = m_state.m_decodeAudioPositionMs;
            decoderPendingAudioBytes = m_state.m_decodePendingAudioBytes;
        }
        if (audioDecodedPositionMs >= 0)
        {
            const qint64 queuedAudioFrames =
                static_cast<qint64>((decoderPendingAudioBytes + streamPlaybackAudioBytes()) / bytesPerSampleFrame)
                + static_cast<qint64>(m_audio->monitorPlaybackClockFill());
            const qint64 queuedAudioMs = static_cast<qint64>(
                (static_cast<double>(queuedAudioFrames) * 1000.0 / static_cast<double>(m_audio->monitorSampleRate())) + 0.5);
            return audioDecodedPositionMs - queuedAudioMs;
        }
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
        if (m_settings->isStreamCamera()) {
            // Buffer-fill servo: hold the decoded-frame queue near a target depth
            // so the present rate tracks the true producer (content) rate. Present
            // a touch faster when over-full and a touch slower when under-full;
            // since fill is the integral of (produce - consume), holding it
            // constant forces consumer rate == producer rate. This avoids both
            // buffer drain (underrun) and overflow (dropped frames) without
            // depending on the reported frame rate, and naturally refills after a
            // network stall. Pacing video to an absolute audio/PTS clock instead
            // does not regulate fill, so the queue rides a rail and drops frames.
            const int fill = decodedStreamFrameQueueDepth();
            const int target = streamInitialBufferFrameCount();
            const double slackMs = qMax(5.0, intervalMs * 0.25);
            const double gainMsPerFrame = qMax(0.5, intervalMs * 0.05);
            const double adjustedMs = intervalMs - static_cast<double>(fill - target) * gainMsPerFrame;
            delayMs = static_cast<qint64>(std::llround(
                qBound(intervalMs - slackMs, adjustedMs, intervalMs + slackMs)));
            delayMs = qMax<qint64>(1, delayMs);
        } else {
            const qint64 nextFramePtsMs = m_state.m_lastFramePtsMs + static_cast<qint64>(std::llround(intervalMs));
            delayMs = nextFramePtsMs - videoFilePlaybackClockMs();
            delayMs -= m_state.m_lastDecodeMs;
            delayMs = qMax<qint64>(1, delayMs);
        }
    }
    else
    {
        quint64 tick = m_state.m_tick > 0 ? m_state.m_tick : 1;
        const qint64 elapsedMs = m_state.m_clock.elapsed();
        qint64 targetMs = static_cast<qint64>(std::llround(static_cast<double>(tick) * intervalMs));

        if (targetMs <= elapsedMs)
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
