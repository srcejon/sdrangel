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

#ifndef INCLUDE_FEATURE_CAMERAQTAUDIOCONTROLLER_H_
#define INCLUDE_FEATURE_CAMERAQTAUDIOCONTROLLER_H_

#include <atomic>

#include <QObject>
#include <QString>
#include <QVector>

#include "audio/audiofifo.h"
#include "camerasettings.h"

class AudioDeviceManager;
class MessageQueue;

class CameraQtAudioController : public QObject
{
    Q_OBJECT
public:
    struct MonitorDebugStats
    {
        quint64 m_submitCalls = 0;
        quint64 m_submittedFrames = 0;
        quint64 m_silenceFrames = 0;
        quint64 m_overflowDrainFrames = 0;
        quint64 m_minFillBefore = 0;
        quint64 m_maxFillBefore = 0;
        quint64 m_minFillAfter = 0;
        quint64 m_maxFillAfter = 0;
    };

    explicit CameraQtAudioController(QObject *parent = nullptr);

    bool isCapturing() const { return m_capturing; }
    void setRecordingMessageQueue(MessageQueue *messageQueue) { m_recordingMessageQueue = messageQueue; }
    void start(const CameraSettings& settings, MessageQueue *messageQueue);
    int startFilePlayback(const CameraSettings& settings, MessageQueue *messageQueue);
    void stop();
    void setMuted(bool muted);
    void clearMonitorAudio();
    void resetMonitorDebugStats();
    void submitPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate);
    uint32_t monitorAudioFill() const { return m_outputAudioFifo.fill(); }
    uint32_t monitorAudioSize() const { return m_outputAudioFifo.size(); }
    quint64 monitorDroppedFrames() const { return m_monitorDroppedFrames; }
    quint64 monitorUnderflows() const { return m_monitorUnderflows.load(); }
    const MonitorDebugStats& monitorDebugStats() const { return m_monitorDebugStats; }

private slots:
    void onCaptureAudioDataReady();

private:
    static QString normalizeAudioMatchName(QString text);
    static int scoreAudioDeviceMatch(const QString& cameraName, const QString& audioName);
    static void alignInputRate(AudioDeviceManager *audioDeviceManager, int inputDeviceIndex, int outputDeviceIndex);
    static int findInputIndex(const CameraSettings& settings);

    bool m_capturing;
    bool m_muted;
    bool m_captureSourceActive;
    int m_sampleRate;
    MessageQueue *m_recordingMessageQueue;
    AudioFifo m_captureAudioFifo;
    AudioFifo m_outputAudioFifo;
    QVector<quint8> m_audioTransferBuffer;
    quint64 m_monitorDroppedFrames;
    std::atomic<quint64> m_monitorUnderflows;
    MonitorDebugStats m_monitorDebugStats;
};

#endif // INCLUDE_FEATURE_CAMERAQTAUDIOCONTROLLER_H_
