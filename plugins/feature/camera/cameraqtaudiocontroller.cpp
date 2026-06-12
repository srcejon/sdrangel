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

#include "cameraqtaudiocontroller.h"

#include <algorithm>

#include <QByteArray>
#include <QDebug>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QCameraDevice>
#include <QMediaDevices>
#endif

#include "audio/audiodevicemanager.h"
#include "camerarecorder.h"
#include "dsp/dspengine.h"
#include "util/messagequeue.h"

CameraQtAudioController::CameraQtAudioController(QObject *parent) :
    QObject(parent),
    m_capturing(false),
    m_muted(false),
    m_captureSourceActive(false),
    m_sampleRate(AudioDeviceManager::m_defaultAudioSampleRate),
    m_recordingMessageQueue(nullptr),
    m_filePlaybackAudioOffsetMs(0),
    m_filePlaybackAudioOffsetRemainingFrames(0),
    m_monitorDroppedFrames(0),
    m_monitorUnderflows(0)
{
    // Audio FIFO: stereo 16-bit PCM at 48 kHz; keep enough decoded file audio to absorb timer jitter.
    static constexpr int audioFifoFrames = 48000 * 2;
    static constexpr int bytesPerSampleFrame = 4;
    m_captureAudioFifo.setSize(audioFifoFrames);
    m_outputAudioFifo.setSize(audioFifoFrames);
    m_audioTransferBuffer.resize(audioFifoFrames * bytesPerSampleFrame);
    QObject::connect(&m_outputAudioFifo, &AudioFifo::underflow, this, [this]() {
        ++m_monitorUnderflows;
    });
}

void CameraQtAudioController::start(const CameraSettings& settings, MessageQueue *messageQueue)
{
    stop();

    AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
    const int outputDeviceIndex = audioDeviceManager->getOutputDeviceIndex(settings.m_audioDeviceName);
    const int outputSampleRate = audioDeviceManager->getOutputSampleRate(outputDeviceIndex);
    int inputDeviceIndex = -1;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    inputDeviceIndex = findInputIndex(settings);
    alignInputRate(audioDeviceManager, inputDeviceIndex, outputDeviceIndex);
#endif
    qDebug() << "CameraQtAudioController: starting audio capture: outputDeviceIndex" << outputDeviceIndex
             << "inputDeviceIndex" << inputDeviceIndex;
    audioDeviceManager->addAudioSink(&m_outputAudioFifo, messageQueue, outputDeviceIndex);
    m_monitorDroppedFrames = 0;
    m_monitorUnderflows.store(0);
    m_monitorDebugStats = MonitorDebugStats();
    QObject::connect(&m_captureAudioFifo, &AudioFifo::dataReady, this, &CameraQtAudioController::onCaptureAudioDataReady);
    audioDeviceManager->addAudioSource(&m_captureAudioFifo, messageQueue, inputDeviceIndex);
    m_muted = settings.m_audioMute;
    m_sampleRate = outputSampleRate > 0 ? outputSampleRate : AudioDeviceManager::m_defaultAudioSampleRate;
    m_captureSourceActive = true;
    m_capturing = true;
}

int CameraQtAudioController::startFilePlayback(const CameraSettings& settings, MessageQueue *messageQueue)
{
    stop();

    AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
    const int outputDeviceIndex = audioDeviceManager->getOutputDeviceIndex(settings.m_audioDeviceName);
    const int outputSampleRate = audioDeviceManager->getOutputSampleRate(outputDeviceIndex);
    qDebug() << "CameraQtAudioController: starting file audio monitor: outputDeviceIndex" << outputDeviceIndex
             << "prefillMs" << filePlaybackMonitorPrefillMs()
             << "targetFillMs" << filePlaybackMonitorTargetFillMs()
             << "audioOffsetMs" << settings.m_videoPlaybackAudioOffsetMs;
    audioDeviceManager->addAudioSink(&m_outputAudioFifo, messageQueue, outputDeviceIndex);
    m_monitorDroppedFrames = 0;
    m_monitorUnderflows.store(0);
    m_monitorDebugStats = MonitorDebugStats();
    m_muted = settings.m_audioMute;
    m_sampleRate = outputSampleRate > 0 ? outputSampleRate : AudioDeviceManager::m_defaultAudioSampleRate;
    m_filePlaybackAudioOffsetMs = qBound(
        CameraSettings::m_minVideoPlaybackAudioOffsetMs,
        settings.m_videoPlaybackAudioOffsetMs,
        CameraSettings::m_maxVideoPlaybackAudioOffsetMs);
    resetFilePlaybackAudioOffset();
    m_captureSourceActive = false;
    m_outputAudioFifo.clear();
    prefillMonitorAudio(filePlaybackMonitorPrefillMs());
    m_capturing = true;
    return m_sampleRate;
}

void CameraQtAudioController::stop()
{
    if (!m_capturing) {
        return;
    }

    qDebug() << "CameraQtAudioController: stopping audio capture";
    AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
    if (m_captureSourceActive)
    {
        QObject::disconnect(&m_captureAudioFifo, &AudioFifo::dataReady, this, &CameraQtAudioController::onCaptureAudioDataReady);
        audioDeviceManager->removeAudioSource(&m_captureAudioFifo);
    }
    audioDeviceManager->removeAudioSink(&m_outputAudioFifo);
    m_captureAudioFifo.clear();
    m_outputAudioFifo.clear();
    m_captureSourceActive = false;
    m_capturing = false;
}

void CameraQtAudioController::setMuted(bool muted)
{
    m_muted = muted;
    if (m_muted) {
        m_captureAudioFifo.clear();
        m_outputAudioFifo.clear();
    }
}

void CameraQtAudioController::setFilePlaybackAudioOffsetMs(int offsetMs)
{
    const int boundedOffsetMs = qBound(
        CameraSettings::m_minVideoPlaybackAudioOffsetMs,
        offsetMs,
        CameraSettings::m_maxVideoPlaybackAudioOffsetMs);
    if (m_filePlaybackAudioOffsetMs == boundedOffsetMs) {
        return;
    }

    m_filePlaybackAudioOffsetMs = boundedOffsetMs;
    resetFilePlaybackAudioOffset();
}

void CameraQtAudioController::clearMonitorAudio()
{
    m_outputAudioFifo.clear();
    resetFilePlaybackAudioOffset();
}

void CameraQtAudioController::prefillMonitorAudio(int milliseconds)
{
    static constexpr int bytesPerSampleFrame = 4;
    if ((milliseconds <= 0) || (m_sampleRate <= 0)) {
        return;
    }

    const uint32_t fifoSize = m_outputAudioFifo.size();
    const uint32_t prefillFrames = std::min<uint32_t>(
        fifoSize - std::min(fifoSize, m_outputAudioFifo.fill()),
        static_cast<uint32_t>((static_cast<qint64>(m_sampleRate) * milliseconds) / 1000));
    if (prefillFrames <= 0) {
        return;
    }

    QByteArray silence;
    silence.resize(static_cast<qsizetype>(prefillFrames) * bytesPerSampleFrame);
    silence.fill(0);
    m_outputAudioFifo.write(reinterpret_cast<const quint8*>(silence.constData()), prefillFrames);
    m_monitorDebugStats.m_silenceFrames += prefillFrames;
}

void CameraQtAudioController::resetMonitorDebugStats()
{
    m_monitorDebugStats = MonitorDebugStats();
}

void CameraQtAudioController::submitPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate)
{
    submitMonitorPcmSamples(pcmS16Stereo, sampleRate);
    submitRecordingPcmSamples(pcmS16Stereo, sampleRate);
}

void CameraQtAudioController::submitMonitorPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if (pcmS16Stereo.isEmpty() || (sampleRate <= 0)) {
        return;
    }

    if (!m_muted && m_capturing)
    {
        QByteArray monitorPcmS16Stereo = pcmS16Stereo;
        if (!m_captureSourceActive) {
            applyFilePlaybackAudioOffset(monitorPcmS16Stereo, sampleRate);
        }

        const int sampleFrames = monitorPcmS16Stereo.size() / bytesPerSampleFrame;
        if (sampleFrames <= 0) {
            return;
        }

        const uint32_t fifoSize = m_outputAudioFifo.size();
        const int monitorFrames = std::min(sampleFrames, static_cast<int>(fifoSize));
        uint32_t maxFillFrames = std::min<uint32_t>(
            fifoSize,
            static_cast<uint32_t>(std::max(m_sampleRate / 2, monitorFrames * 4)));
        const int skippedInputFrames = sampleFrames - monitorFrames;
        if (skippedInputFrames > 0) {
            m_monitorDroppedFrames += static_cast<quint64>(skippedInputFrames);
        }
        ++m_monitorDebugStats.m_submitCalls;
        m_monitorDebugStats.m_submittedFrames += static_cast<quint64>(monitorFrames);

        uint32_t fill = m_outputAudioFifo.fill();
        if ((m_monitorDebugStats.m_submitCalls == 1) || (fill < m_monitorDebugStats.m_minFillBefore)) {
            m_monitorDebugStats.m_minFillBefore = fill;
        }
        m_monitorDebugStats.m_maxFillBefore = std::max<quint64>(m_monitorDebugStats.m_maxFillBefore, fill);

        if (!m_captureSourceActive)
        {
            const uint32_t targetFillFrames = std::min<uint32_t>(
                fifoSize,
                static_cast<uint32_t>((static_cast<qint64>(m_sampleRate) * filePlaybackMonitorTargetFillMs()) / 1000));
            const uint32_t jitterFrames = std::min<uint32_t>(
                fifoSize,
                static_cast<uint32_t>((static_cast<qint64>(m_sampleRate) * 20) / 1000));
            maxFillFrames = std::min<uint32_t>(
                fifoSize,
                targetFillFrames + static_cast<uint32_t>(monitorFrames) + jitterFrames);
        }

        const quint8 *monitorData = reinterpret_cast<const quint8*>(monitorPcmS16Stereo.constData())
            + static_cast<qsizetype>(skippedInputFrames) * bytesPerSampleFrame;
        const uint32_t overflowFrames = fill + static_cast<uint32_t>(monitorFrames) > maxFillFrames
            ? fill + static_cast<uint32_t>(monitorFrames) - maxFillFrames
            : 0;
        if (overflowFrames > 0)
        {
            m_outputAudioFifo.drain(overflowFrames);
            m_monitorDroppedFrames += overflowFrames;
            m_monitorDebugStats.m_overflowDrainFrames += overflowFrames;
        }
        if (monitorFrames > 0) {
            m_outputAudioFifo.write(monitorData, static_cast<uint32_t>(monitorFrames));
        }
        const uint32_t fillAfter = m_outputAudioFifo.fill();
        if ((m_monitorDebugStats.m_submitCalls == 1) || (fillAfter < m_monitorDebugStats.m_minFillAfter)) {
            m_monitorDebugStats.m_minFillAfter = fillAfter;
        }
        m_monitorDebugStats.m_maxFillAfter = std::max<quint64>(m_monitorDebugStats.m_maxFillAfter, fillAfter);
    }
}

void CameraQtAudioController::resetFilePlaybackAudioOffset()
{
    if (m_sampleRate <= 0) {
        m_filePlaybackAudioOffsetRemainingFrames = 0;
        return;
    }

    m_filePlaybackAudioOffsetRemainingFrames =
        static_cast<int>((static_cast<qint64>(m_sampleRate) * m_filePlaybackAudioOffsetMs) / 1000);
}

void CameraQtAudioController::applyFilePlaybackAudioOffset(QByteArray& pcmS16Stereo, int sampleRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if ((m_filePlaybackAudioOffsetRemainingFrames == 0) || (sampleRate <= 0) || pcmS16Stereo.isEmpty()) {
        return;
    }

    if (m_filePlaybackAudioOffsetRemainingFrames > 0)
    {
        QByteArray silence;
        silence.resize(static_cast<qsizetype>(m_filePlaybackAudioOffsetRemainingFrames) * bytesPerSampleFrame);
        silence.fill(0);
        pcmS16Stereo.prepend(silence);
        m_monitorDebugStats.m_silenceFrames += static_cast<quint64>(m_filePlaybackAudioOffsetRemainingFrames);
        m_filePlaybackAudioOffsetRemainingFrames = 0;
    }
    else
    {
        const int sampleFrames = pcmS16Stereo.size() / bytesPerSampleFrame;
        const int skipFrames = std::min(sampleFrames, -m_filePlaybackAudioOffsetRemainingFrames);
        if (skipFrames <= 0) {
            return;
        }

        pcmS16Stereo.remove(0, static_cast<qsizetype>(skipFrames) * bytesPerSampleFrame);
        m_monitorDroppedFrames += static_cast<quint64>(skipFrames);
        m_filePlaybackAudioOffsetRemainingFrames += skipFrames;
    }
}

void CameraQtAudioController::submitRecordingPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate)
{
    if (pcmS16Stereo.isEmpty() || (sampleRate <= 0)) {
        return;
    }
    if (m_recordingMessageQueue) {
        m_recordingMessageQueue->push(CameraRecorder::MsgAudioSamples::create(pcmS16Stereo, sampleRate));
    }
}

QString CameraQtAudioController::normalizeAudioMatchName(QString text)
{
    text = text.toLower();

    for (int i = 0; i < text.size(); ++i)
    {
        if (!text[i].isLetterOrNumber()) {
            text[i] = QLatin1Char(' ');
        }
    }

    const QStringList skipTokens = {
        QStringLiteral("audio"),
        QStringLiteral("camera"),
        QStringLiteral("device"),
        QStringLiteral("input"),
        QStringLiteral("microphone"),
        QStringLiteral("mic"),
        QStringLiteral("video"),
        QStringLiteral("webcam")
    };

    QStringList tokens = text.simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
    tokens.erase(
        std::remove_if(
            tokens.begin(),
            tokens.end(),
            [&skipTokens](const QString& token) { return skipTokens.contains(token); }),
        tokens.end());
    return tokens.join(QLatin1Char(' '));
}

int CameraQtAudioController::scoreAudioDeviceMatch(const QString& cameraName, const QString& audioName)
{
    if (cameraName.isEmpty() || audioName.isEmpty()) {
        return -1;
    }

    const QString cameraLower = cameraName.toLower();
    const QString audioLower = audioName.toLower();

    if (cameraLower == audioLower) {
        return 1000;
    }

    int score = 0;

    if (audioLower.contains(cameraLower) || cameraLower.contains(audioLower)) {
        score += 400;
    }

    const QString normalizedCamera = normalizeAudioMatchName(cameraName);
    const QString normalizedAudio = normalizeAudioMatchName(audioName);

    if (!normalizedCamera.isEmpty() && normalizedCamera == normalizedAudio) {
        score += 300;
    } else if (!normalizedCamera.isEmpty() && !normalizedAudio.isEmpty()
            && (normalizedAudio.contains(normalizedCamera) || normalizedCamera.contains(normalizedAudio))) {
        score += 150;
    }

    const QStringList cameraTokens = normalizedCamera.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    const QStringList audioTokens = normalizedAudio.split(QLatin1Char(' '), Qt::SkipEmptyParts);
    int tokenMatches = 0;

    for (const QString& token : cameraTokens)
    {
        if (audioTokens.contains(token)) {
            ++tokenMatches;
        }
    }

    score += tokenMatches * 25;
    return score;
}

void CameraQtAudioController::alignInputRate(AudioDeviceManager *audioDeviceManager, int inputDeviceIndex, int outputDeviceIndex)
{
    if ((audioDeviceManager == nullptr) || (inputDeviceIndex < 0)) {
        return;
    }

    const int outputSampleRate = audioDeviceManager->getOutputSampleRate(outputDeviceIndex);

    if (outputSampleRate <= 0) {
        return;
    }

    const QList<AudioDeviceInfo>& inputDevices = AudioDeviceInfo::availableInputDevices();

    if (inputDeviceIndex >= inputDevices.size()) {
        return;
    }

    const AudioDeviceInfo& inputDeviceInfo = inputDevices.at(inputDeviceIndex);
    const QList<int> supportedSampleRates = inputDeviceInfo.supportedSampleRates();

    if (!supportedSampleRates.contains(outputSampleRate))
    {
        qWarning() << "CameraQtAudioController: input audio device" << inputDeviceInfo.deviceName()
                   << "does not support output sample rate" << outputSampleRate
                   << "supported sample rates:" << supportedSampleRates;
        return;
    }

    AudioDeviceManager::InputDeviceInfo configuredInputInfo;
    audioDeviceManager->getInputDeviceInfo(inputDeviceInfo.deviceName(), configuredInputInfo);

    if (configuredInputInfo.sampleRate == outputSampleRate) {
        return;
    }

    configuredInputInfo.sampleRate = outputSampleRate;
    audioDeviceManager->setInputDeviceInfo(inputDeviceIndex, configuredInputInfo);

    qDebug() << "CameraQtAudioController: aligned input audio device" << inputDeviceInfo.deviceName()
             << "sample rate to output rate" << outputSampleRate;
}

int CameraQtAudioController::findInputIndex(const CameraSettings& settings)
{
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    if (!settings.isQtCamera()) {
        return -1;
    }

    const QString targetId = settings.cameraIdString();
    const QString targetDescription = settings.cameraDescription();
    const QList<QCameraDevice> cameras = QMediaDevices::videoInputs();

    QString cameraDescription = targetDescription;

    for (const QCameraDevice& device : cameras)
    {
        const QString id = QString::fromUtf8(device.id());

        if ((id == targetId) || (device.description() == targetDescription))
        {
            cameraDescription = device.description();
            break;
        }
    }

    const QList<AudioDeviceInfo>& audioInputs = AudioDeviceInfo::availableInputDevices();
    int bestIndex = -1;
    int bestScore = -1;

    for (int i = 0; i < audioInputs.size(); ++i)
    {
        const QString audioName = audioInputs[i].deviceName();
        const int descriptionScore = scoreAudioDeviceMatch(cameraDescription, audioName);
        const int idScore = scoreAudioDeviceMatch(targetId, audioName);
        const int score = std::max(descriptionScore, idScore);

        if (score > bestScore)
        {
            bestScore = score;
            bestIndex = i;
        }
    }

    return bestScore >= 150 ? bestIndex : -1;
#else
    (void) settings;
    return -1;
#endif
}

void CameraQtAudioController::onCaptureAudioDataReady()
{
    // Each audio sample frame is 4 bytes: stereo 16-bit PCM (2 channels x 2 bytes).
    static constexpr int bytesPerSampleFrame = 4;
    unsigned int nbRead;

    while ((nbRead = m_captureAudioFifo.read(m_audioTransferBuffer.data(), m_audioTransferBuffer.size() / bytesPerSampleFrame)) != 0)
    {
        if (!m_muted) {
            m_outputAudioFifo.write(m_audioTransferBuffer.data(), nbRead);
        }

        if (m_recordingMessageQueue)
        {
            const int byteCount = static_cast<int>(nbRead) * bytesPerSampleFrame;
            m_recordingMessageQueue->push(CameraRecorder::MsgAudioSamples::create(
                QByteArray(reinterpret_cast<const char*>(m_audioTransferBuffer.constData()), byteCount),
                m_sampleRate));
        }
    }
}
