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
    void resetDecodeSnapshot();

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
    std::atomic<quint64> m_decodeDroppedFrames { 0 };
    std::atomic<quint64> m_decodeDroppedSinceLastSubmit { 0 };
    mutable QMutex m_decodedFramesMutex;
    QWaitCondition m_decodedFramesAvailable;
    QWaitCondition m_decodedFramesNotFull;
    std::deque<DecodedFrame> m_decodedFrames;
    CameraVideoFileDecoder::DebugStats m_decodeStatsSnapshot;
    mutable QMutex m_decodeStatsMutex;
    qint64 m_decodeAudioPositionMs = -1;
    int m_decodePendingAudioBytes = 0;
    int m_decodePendingVideoFrames = 0;
    int m_decodePendingVideoPackets = 0;
    // Keep enough live video headroom to read interleaved audio packets without
    // letting normal stream jitter create unbounded preview latency.
    static constexpr size_t m_streamInitialBufferFrames = 4;
    static constexpr size_t m_maxDecodedStreamFrames = 6;

    QElapsedTimer m_statsTimer;
    QElapsedTimer m_tickTimer;
    quint64 m_statsFrames = 0;
    quint64 m_statsEmptyAudioFrames = 0;
    quint64 m_statsMonitorExtraAudioFrames = 0;
    qint64 m_statsDecodeMsTotal = 0;
    qint64 m_statsDecodeMsMax = 0;
    qint64 m_statsTickDeltaMsTotal = 0;
    qint64 m_statsTickDeltaMsMax = 0;
    qint64 m_statsPositionDeltaMsTotal = 0;
    qint64 m_statsPositionDeltaMsMin = 0;
    qint64 m_statsPositionDeltaMsMax = 0;
    qint64 m_statsLastPositionMs = -1;
    quint64 m_statsAudioBytes = 0;
    quint64 m_statsDroppedLateFrames = 0;
    quint64 m_statsDroppedPipelineFrames = 0;
    qint64 m_statsVideoLateMsTotal = 0;
    qint64 m_statsVideoLateMsMax = 0;
    quint64 m_statsLastDroppedAudioFrames = 0;
    quint64 m_statsLastAudioUnderflows = 0;
    quint64 m_statsLastDecoderReadAheadCalls = 0;
    quint64 m_statsLastDecoderReadAheadPackets = 0;
    quint64 m_statsLastDecoderReadAheadVideoPackets = 0;
    quint64 m_statsLastDecoderReadAheadAudioPackets = 0;
    quint64 m_statsLastDecoderReadAheadOtherPackets = 0;
    quint64 m_statsLastDecoderInputVideoPackets = 0;
    quint64 m_statsLastDecoderInputAudioPackets = 0;
    quint64 m_statsLastDecoderInputOtherPackets = 0;
    quint64 m_statsLastDecoderEagain = 0;
    quint64 m_statsLastDecoderQueuedFrames = 0;
    quint64 m_statsLastDecoderParkedVideoPackets = 0;
    quint64 m_statsLastDecoderPacketCapHits = 0;
    quint64 m_statsLastDecoderAudioBytes = 0;
    quint64 m_statsLastDecoderAudioFrames = 0;
    quint64 m_statsLastDecoderPacedAudioCalls = 0;
    quint64 m_statsLastDecoderPacedAudioTargetFrames = 0;
    quint64 m_statsLastDecoderPacedAudioOutputFrames = 0;
    quint64 m_statsLastDecoderPacedAudioShortCalls = 0;
    quint64 m_statsLastDecoderDroppedPendingAudioBytes = 0;
    quint64 m_statsLastDecoderDroppedPendingAudioFrames = 0;
    quint64 m_statsLastDecoderAudioTimestampJumps = 0;
    quint64 m_statsLastDecoderReadFrameCalls = 0;
    quint64 m_statsLastDecoderReadFrameMs = 0;
    quint64 m_statsLastDecoderMainReadPackets = 0;
    quint64 m_statsLastDecoderMainReadMs = 0;
    quint64 m_statsLastDecoderSendVideoPackets = 0;
    quint64 m_statsLastDecoderSendVideoMs = 0;
    quint64 m_statsLastDecoderReceiveVideoCalls = 0;
    quint64 m_statsLastDecoderReceiveVideoMs = 0;
    quint64 m_statsLastDecoderFinishAudioCalls = 0;
    quint64 m_statsLastDecoderFinishAudioMs = 0;
    quint64 m_statsLastDecoderReadAheadAudioMs = 0;
    quint64 m_statsLastDecoderReadAheadReadMs = 0;
    quint64 m_statsLastDecoderSendAudioPackets = 0;
    quint64 m_statsLastDecoderSendAudioMs = 0;
    quint64 m_statsLastDecoderConvertFrames = 0;
    quint64 m_statsLastDecoderConvertMs = 0;
};

#endif // INCLUDE_FEATURE_CAMERA_MEDIA_PLAYBACK_STATE_H_
