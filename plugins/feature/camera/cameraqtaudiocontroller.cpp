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
    m_previewVolumePercent(100),
    m_sampleRate(AudioDeviceManager::m_defaultAudioSampleRate),
    m_recordingMessageQueue(nullptr),
    m_filePlaybackAudioOffsetMs(0),
    m_filePlaybackAudioOffsetRemainingFrames(0),
    m_filePlaybackStreamSource(false)
{
    // Audio FIFO: stereo 16-bit PCM at 48 kHz; keep enough decoded file audio to absorb timer jitter.
    static constexpr int audioFifoFrames = 48000 * 2;
    static constexpr int bytesPerSampleFrame = 4;
    m_captureAudioFifo.setSize(audioFifoFrames);
    m_outputAudioFifo.setSize(audioFifoFrames);
    m_audioTransferBuffer.resize(audioFifoFrames * bytesPerSampleFrame);
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
    m_outputDeviceIndex = outputDeviceIndex;
    QObject::connect(&m_captureAudioFifo, &AudioFifo::dataReady, this, &CameraQtAudioController::onCaptureAudioDataReady);
    audioDeviceManager->addAudioSource(&m_captureAudioFifo, messageQueue, inputDeviceIndex);
    m_muted = settings.m_audioMute;
    m_previewVolumePercent = qBound(0, settings.m_audioPreviewVolume, 100);
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
    m_muted = settings.m_audioMute;
    m_previewVolumePercent = qBound(0, settings.m_audioPreviewVolume, 100);
    m_sampleRate = outputSampleRate > 0 ? outputSampleRate : AudioDeviceManager::m_defaultAudioSampleRate;
    m_filePlaybackAudioOffsetMs = qBound(
        CameraSettings::m_minVideoPlaybackAudioOffsetMs,
        settings.m_videoPlaybackAudioOffsetMs,
        CameraSettings::m_maxVideoPlaybackAudioOffsetMs);
    m_filePlaybackStreamSource = settings.isStreamCamera();
    resetFilePlaybackAudioOffset();
    m_captureSourceActive = false;
    m_outputAudioFifo.clear();
    prefillMonitorAudio(filePlaybackMonitorPrefillForOffsetMs());
    qDebug() << "CameraQtAudioController: starting file audio monitor: outputDeviceIndex" << outputDeviceIndex
             << "prefillMs" << filePlaybackMonitorPrefillForOffsetMs()
             << "targetFillMs" << filePlaybackMonitorTargetFillForOffsetMs()
             << "stream" << m_filePlaybackStreamSource
             << "audioOffsetMs" << m_filePlaybackAudioOffsetMs;
    audioDeviceManager->addAudioSink(&m_outputAudioFifo, messageQueue, outputDeviceIndex);
    m_outputDeviceIndex = outputDeviceIndex;
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
    m_filePlaybackStreamSource = false;
    m_capturing = false;
}

void CameraQtAudioController::setMuted(bool muted)
{
    m_muted = muted;
    if (m_muted) {
        // Drop the live-capture path, but DO NOT clear the monitor (output)
        // FIFO: it now keeps flowing with silence while muted (see
        // submitMonitorPcmSamples). Clearing it would make the resampler servo
        // gulp ~1s out of the stream-audio buffer in one tick to refill, which
        // dips the ratio and reintroduces the post-unmute pitch artifact.
        m_captureAudioFifo.clear();
    }
}

void CameraQtAudioController::setPreviewVolume(int volumePercent)
{
    m_previewVolumePercent = qBound(0, volumePercent, 100);
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

    if (m_capturing)
    {
        QByteArray monitorPcmS16Stereo = pcmS16Stereo;
        if (m_muted) {
            // Output silence while muted, but keep filling the monitor FIFO so
            // the pipeline (and the worker's stream-audio resampler servo that
            // tops it up) keeps running. Skipping submission entirely lets the
            // device drain the FIFO to 0; the buffer-depth servo then sees a
            // perpetually starved monitor, over-drains the stream-audio buffer
            // and winds the resample ratio down to its floor, which plays back
            // at a lowered pitch for ~1s after unmute while the ratio recovers.
            monitorPcmS16Stereo.fill('\0');
        } else if (!m_captureSourceActive) {
            applyFilePlaybackAudioOffset(monitorPcmS16Stereo, sampleRate);
        }
        applyPreviewVolume(monitorPcmS16Stereo);

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

        if (!m_captureSourceActive)
        {
            const uint32_t targetFillFrames = std::min<uint32_t>(
                fifoSize,
                static_cast<uint32_t>((static_cast<qint64>(m_sampleRate) * filePlaybackMonitorTargetFillForOffsetMs()) / 1000));
            const uint32_t jitterFrames = std::min<uint32_t>(
                fifoSize,
                static_cast<uint32_t>((static_cast<qint64>(m_sampleRate) * filePlaybackMonitorJitterForOffsetMs()) / 1000));
            maxFillFrames = std::min<uint32_t>(
                fifoSize,
                targetFillFrames + static_cast<uint32_t>(monitorFrames) + jitterFrames);
        }

        const quint8 *monitorData = reinterpret_cast<const quint8*>(monitorPcmS16Stereo.constData())
            + static_cast<qsizetype>(skippedInputFrames) * bytesPerSampleFrame;
        if (monitorFrames > 0) {
            m_outputAudioFifo.write(monitorData, static_cast<uint32_t>(monitorFrames));
        }
        // Bound the FIFO to maxFillFrames AFTER writing, using a fresh fill()
        // reading taken immediately before the trim. The audio device thread
        // drains the FIFO concurrently, so the earlier `fill` snapshot (used for
        // the before-stats) is stale and computing the overflow from it would
        // over-drain and discard live audio. Reading fill() here and draining
        // only the genuine excess is safe: the device can only make fill smaller,
        // so drain() trims at most the real overflow (and returns the count it
        // actually removed). Drain trims the oldest (head) frames, keeping the
        // newest audio we just wrote.
        const uint32_t fillBeforeTrim = m_outputAudioFifo.fill();
        if (fillBeforeTrim > maxFillFrames)
        {
            m_outputAudioFifo.drain(fillBeforeTrim - maxFillFrames);
        }
    }
}

void CameraQtAudioController::resetFilePlaybackAudioOffset()
{
    if (m_sampleRate <= 0) {
        m_filePlaybackAudioOffsetRemainingFrames = 0;
        return;
    }

    m_filePlaybackAudioOffsetRemainingFrames = m_filePlaybackAudioOffsetMs > 0
        ? static_cast<int>((static_cast<qint64>(m_sampleRate) * m_filePlaybackAudioOffsetMs) / 1000)
        : 0;
}

int CameraQtAudioController::filePlaybackMonitorPrefillForOffsetMs() const
{
    // Prefill the monitor to its steady-state operating level (the target fill the
    // audio-submit servo holds anyway). The audio device primes its own output
    // buffer (~bufferSize) at startup; if the monitor is prefilled below that, the
    // device drains the FIFO to zero before the first ticks refill it, giving a
    // brief startup underrun. Priming at the target removes that ramp without
    // changing steady-state A/V latency (the servo governs the fill regardless).
    return filePlaybackMonitorTargetFillForOffsetMs();
}

int CameraQtAudioController::filePlaybackMonitorTargetFillForOffsetMs() const
{
    return m_filePlaybackStreamSource ? streamPlaybackMonitorTargetFillMs() : filePlaybackMonitorTargetFillMs();
}

int CameraQtAudioController::filePlaybackMonitorJitterForOffsetMs() const
{
    return m_filePlaybackStreamSource ? streamPlaybackMonitorJitterMs() : filePlaybackMonitorJitterMs();
}

int CameraQtAudioController::monitorTargetFillFrames(int sampleRate) const
{
    if (sampleRate <= 0) {
        return 0;
    }

    return static_cast<int>(
        (static_cast<qint64>(sampleRate) * filePlaybackMonitorTargetFillForOffsetMs()) / 1000);
}

qint64 CameraQtAudioController::monitorSinkLatencyUSecs() const
{
    AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
    if (!audioDeviceManager) {
        return 0;
    }
    return audioDeviceManager->getOutputDeviceSinkLatencyUSecs(m_outputDeviceIndex);
}

uint32_t CameraQtAudioController::monitorPlaybackClockFill() const
{
    const uint32_t fill = m_outputAudioFifo.fill();
    if (m_captureSourceActive || !m_filePlaybackStreamSource || (m_sampleRate <= 0)) {
        return fill;
    }

    const int targetFrames = monitorTargetFillFrames(m_sampleRate);
    const int jitterFrames = static_cast<int>(
        (static_cast<qint64>(m_sampleRate) * filePlaybackMonitorJitterForOffsetMs()) / 1000);
    const uint32_t floorFrames = static_cast<uint32_t>(std::max(0, targetFrames - jitterFrames));
    return std::max(fill, floorFrames);
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
        m_filePlaybackAudioOffsetRemainingFrames = 0;
    }
}

void CameraQtAudioController::applyPreviewVolume(QByteArray& pcmS16Stereo) const
{
    const int volumePercent = m_previewVolumePercent.load();

    if (volumePercent >= 100) {
        return;
    }

    if (volumePercent <= 0)
    {
        pcmS16Stereo.fill('\0');
        return;
    }

    qint16 *samples = reinterpret_cast<qint16*>(pcmS16Stereo.data());
    const int sampleCount = pcmS16Stereo.size() / static_cast<int>(sizeof(qint16));

    for (int i = 0; i < sampleCount; ++i) {
        samples[i] = static_cast<qint16>((static_cast<int>(samples[i]) * volumePercent) / 100);
    }
}

void CameraQtAudioController::submitRecordingPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate, qint64 contentPositionMs)
{
    if (pcmS16Stereo.isEmpty() || (sampleRate <= 0)) {
        return;
    }
    if (m_recordingMessageQueue) {
        m_recordingMessageQueue->push(CameraRecorder::MsgAudioSamples::create(pcmS16Stereo, sampleRate, contentPositionMs));
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
        if (!m_muted)
        {
            const int byteCount = static_cast<int>(nbRead) * bytesPerSampleFrame;
            QByteArray monitorPcmS16Stereo(reinterpret_cast<const char*>(m_audioTransferBuffer.constData()), byteCount);
            applyPreviewVolume(monitorPcmS16Stereo);
            m_outputAudioFifo.write(reinterpret_cast<const quint8*>(monitorPcmS16Stereo.constData()), nbRead);
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
