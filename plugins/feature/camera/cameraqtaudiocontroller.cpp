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
    m_sampleRate(AudioDeviceManager::m_defaultAudioSampleRate),
    m_recordingMessageQueue(nullptr)
{
    // Audio FIFO: stereo 16-bit PCM at 48 kHz; 4800 sample frames x 4 bytes each.
    static constexpr int audioFifoFrames = 4800 * 4;
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
    QObject::connect(&m_captureAudioFifo, &AudioFifo::dataReady, this, &CameraQtAudioController::onCaptureAudioDataReady);
    audioDeviceManager->addAudioSource(&m_captureAudioFifo, messageQueue, inputDeviceIndex);
    m_muted = settings.m_audioMute;
    m_sampleRate = outputSampleRate > 0 ? outputSampleRate : AudioDeviceManager::m_defaultAudioSampleRate;
    m_capturing = true;
}

void CameraQtAudioController::startFilePlayback(const CameraSettings& settings, MessageQueue *messageQueue)
{
    stop();

    AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
    const int outputDeviceIndex = audioDeviceManager->getOutputDeviceIndex(settings.m_audioDeviceName);
    const int outputSampleRate = audioDeviceManager->getOutputSampleRate(outputDeviceIndex);
    qDebug() << "CameraQtAudioController: starting file audio monitor: outputDeviceIndex" << outputDeviceIndex;
    audioDeviceManager->addAudioSink(&m_outputAudioFifo, messageQueue, outputDeviceIndex);
    m_muted = settings.m_audioMute;
    m_sampleRate = outputSampleRate > 0 ? outputSampleRate : AudioDeviceManager::m_defaultAudioSampleRate;
    m_outputAudioFifo.clear();
    m_capturing = true;
}

void CameraQtAudioController::stop()
{
    if (!m_capturing) {
        return;
    }

    qDebug() << "CameraQtAudioController: stopping audio capture";
    QObject::disconnect(&m_captureAudioFifo, &AudioFifo::dataReady, this, &CameraQtAudioController::onCaptureAudioDataReady);
    AudioDeviceManager *audioDeviceManager = DSPEngine::instance()->getAudioDeviceManager();
    audioDeviceManager->removeAudioSource(&m_captureAudioFifo);
    audioDeviceManager->removeAudioSink(&m_outputAudioFifo);
    m_captureAudioFifo.clear();
    m_outputAudioFifo.clear();
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

void CameraQtAudioController::submitPcmSamples(const QByteArray& pcmS16Stereo, int sampleRate)
{
    static constexpr int bytesPerSampleFrame = 4;
    if (pcmS16Stereo.isEmpty() || (sampleRate <= 0)) {
        return;
    }

    const int sampleFrames = pcmS16Stereo.size() / bytesPerSampleFrame;
    if (!m_muted && m_capturing && (sampleFrames > 0)) {
        m_outputAudioFifo.write(reinterpret_cast<const quint8*>(pcmS16Stereo.constData()), sampleFrames);
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
