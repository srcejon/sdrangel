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

#include "cameramediaplaybackstate.h"

CameraMediaPlaybackState::CameraMediaPlaybackState(QObject *timerParent) :
    m_delayedSubmitTimer(timerParent),
    m_streamFrameRetryTimer(timerParent)
{
}

void CameraMediaPlaybackState::resetClosed()
{
    m_decoder.reset();
    m_decodeFrameWakeQueued.store(false);
    m_decodeDroppedSinceLastSubmit.store(0);
    {
        QMutexLocker locker(&m_streamAudioMutex);
        m_streamAudioPcmS16Stereo.clear();
        m_streamAudioSampleRate = 0;
        m_streamAudioPaceRemainderFrames = 0.0;
        m_streamAudioTrimToTargetPending = false;
    }
    m_frameRate = 25.0;
    m_positionMs = 0;
    m_durationMs = 0;
    m_playing = false;
    resetClock();
}

void CameraMediaPlaybackState::resetClock()
{
    m_decodeFrameWakeQueued.store(false);
    m_decodeDroppedSinceLastSubmit.store(0);
    {
        QMutexLocker locker(&m_streamAudioMutex);
        m_streamAudioPcmS16Stereo.clear();
        m_streamAudioPaceRemainderFrames = 0.0;
    }
    m_clock.invalidate();
    m_tick = 0;
    m_basePositionMs = -1;
    m_lastFramePtsMs = -1;
    m_lastDecodeMs = 0;
}
