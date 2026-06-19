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
    explicit CameraQtAudioController(QObject *parent = nullptr);

    bool isCapturing() const { return m_capturing; }
    void setRecordingMessageQueue(MessageQueue *messageQueue) { m_recordingMessageQueue = messageQueue; }
    void start(const CameraSettings& settings, MessageQueue *messageQueue);
    int startFilePlayback(const CameraSettings& settings, MessageQueue *messageQueue);
    void stop();
    void setMuted(bool muted);
    void setFilePlaybackAudioOffsetMs(int offsetMs);
    void clearMonitorAudio();
    void prefillMonitorAudio(int milliseconds);
    [[nodiscard]] int filePlaybackMonitorPrefillForOffsetMs() const;
    static int filePlaybackMonitorTargetFillMs() { return 250; }
    [[nodiscard]] int monitorTargetFillFrames(int sampleRate) const;
    void submitPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate);
    void submitMonitorPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate);
    void submitRecordingPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate);
    int monitorSampleRate() const { return m_sampleRate; }
    // Audio still queued in the sound device (output latency not otherwise
    // accounted for in the A/V playback clock); used to compensate the video-vs-
    // audio skew in file playback. 0 if unavailable.
    [[nodiscard]] qint64 monitorSinkLatencyUSecs() const;
    uint32_t monitorAudioFill() const { return m_outputAudioFifo.fill(); }
    uint32_t monitorPlaybackClockFill() const;
    uint32_t monitorAudioSize() const { return m_outputAudioFifo.size(); }

private slots:
    void onCaptureAudioDataReady();

private:
    static QString normalizeAudioMatchName(QString text);
    static int scoreAudioDeviceMatch(const QString& cameraName, const QString& audioName);
    static void alignInputRate(AudioDeviceManager *audioDeviceManager, int inputDeviceIndex, int outputDeviceIndex);
    static int findInputIndex(const CameraSettings& settings);
    void resetFilePlaybackAudioOffset();
    [[nodiscard]] int filePlaybackMonitorTargetFillForOffsetMs() const;
    [[nodiscard]] int filePlaybackMonitorJitterForOffsetMs() const;
    // Keep the live monitor cushion small. It only needs to absorb per-tick
    // jitter (~33 ms presentation interval, a few ms of scheduling slop) so the
    // FIFO never drains to zero between submissions. The cushion is fixed pipeline
    // latency: audio sits this far ahead of its matching video, so a large value
    // (the old 850/250 ms) is an audible lip-sync offset. ~120 ms is enough to
    // stop underruns while keeping the audio-ahead near the video pipeline latency
    // (~16 ms), so A/V look in sync without needing a compensating audio offset.
    // (Network stalls longer than this are ridden out by the video decode buffer,
    // not the monitor; the monitor can't paper over a multi-hundred-ms stall
    // without adding that much latency to every frame.)
    static int streamPlaybackMonitorTargetFillMs() { return 120; }
    static int streamPlaybackMonitorJitterMs() { return 80; }
    static int filePlaybackMonitorJitterMs() { return 80; }
    void applyFilePlaybackAudioOffset(QByteArray& pcmS16Stereo, int sampleRate);

    // Set from the GUI thread (start/stop/setMuted) and read from the worker
    // thread in submitMonitorPcmSamples, so these must be atomic to avoid a
    // data race.
    std::atomic<bool> m_capturing;
    std::atomic<bool> m_muted;
    std::atomic<bool> m_captureSourceActive;
    int m_sampleRate;
    int m_outputDeviceIndex = -1;
    MessageQueue *m_recordingMessageQueue;
    AudioFifo m_captureAudioFifo;
    AudioFifo m_outputAudioFifo;
    QVector<quint8> m_audioTransferBuffer;
    int m_filePlaybackAudioOffsetMs;
    int m_filePlaybackAudioOffsetRemainingFrames;
    bool m_filePlaybackStreamSource = false;
};

#endif // INCLUDE_FEATURE_CAMERAQTAUDIOCONTROLLER_H_
