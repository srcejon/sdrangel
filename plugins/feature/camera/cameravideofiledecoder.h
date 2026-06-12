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

#ifndef INCLUDE_FEATURE_CAMERA_VIDEO_FILE_DECODER_H_
#define INCLUDE_FEATURE_CAMERA_VIDEO_FILE_DECODER_H_

#include <deque>

#include <QByteArray>
#include <QImage>
#include <QString>

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct SwrContext;
struct SwsContext;

class CameraVideoFileDecoder
{
public:
    CameraVideoFileDecoder();
    ~CameraVideoFileDecoder();

    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] bool open(const QString& fileName, QString& errorMessage, int outputSampleRate = 48000);
    void close();
    void seek(qint64 positionMs);
    void setAudioPaceFrameRate(double frameRate);
    [[nodiscard]] bool readNextFrame(
        QImage& image,
        qint64& positionMs,
        QByteArray& pcmS16Stereo,
        int& audioSampleRate,
        QString& errorMessage);
    [[nodiscard]] bool readNextFrameAtOrAfter(
        qint64 targetPositionMs,
        QImage& image,
        qint64& positionMs,
        QString& errorMessage);
    [[nodiscard]] qint64 durationMs() const { return m_durationMs; }
    [[nodiscard]] double frameRate() const { return m_frameRate; }
    [[nodiscard]] qint64 audioDecodedPositionMs() const { return m_audioDecodedPositionMs; }
    [[nodiscard]] int pendingAudioBytes() const { return m_pendingAudioPcm.size(); }
    [[nodiscard]] int pendingVideoFrameCount() const { return static_cast<int>(m_pendingVideoFrames.size()); }
    [[nodiscard]] int pendingVideoPacketCount() const { return static_cast<int>(m_pendingVideoPackets.size()); }

    struct DebugStats
    {
        quint64 m_readAheadCalls = 0;
        quint64 m_readAheadPackets = 0;
        quint64 m_readAheadVideoPackets = 0;
        quint64 m_sendVideoPacketEagain = 0;
        quint64 m_queuedVideoFrames = 0;
        quint64 m_parkedVideoPackets = 0;
        quint64 m_readAheadPacketCapHits = 0;
        quint64 m_audioBytes = 0;
        quint64 m_pacedAudioCalls = 0;
        quint64 m_pacedAudioTargetFrames = 0;
        quint64 m_pacedAudioOutputFrames = 0;
        quint64 m_pacedAudioShortCalls = 0;
        quint64 m_audioTimestampJumps = 0;
        qint64 m_audioTimestampJumpMaxAbsMs = 0;
        quint64 m_videoConvertFrames = 0;
        quint64 m_videoConvertMs = 0;
        qint64 m_videoConvertMaxMs = 0;
    };

    [[nodiscard]] const DebugStats& debugStats() const { return m_debugStats; }

private:
    struct PendingVideoFrame
    {
        QImage m_image;
        qint64 m_positionMs = -1;
    };

    AVFormatContext *m_formatContext = nullptr;
    AVCodecContext *m_videoCodecContext = nullptr;
    AVCodecContext *m_audioCodecContext = nullptr;
    AVFrame *m_videoFrame = nullptr;
    AVFrame *m_audioFrame = nullptr;
    AVPacket *m_packet = nullptr;
    SwsContext *m_swsContext = nullptr;
    SwrContext *m_resampler = nullptr;
    int m_videoStreamIndex = -1;
    int m_audioStreamIndex = -1;
    qint64 m_durationMs = 0;
    double m_frameRate = 25.0;
    int m_outputSampleRate = 48000;
    double m_audioPaceFrameRate = 0.0;
    double m_audioPaceRemainderFrames = 0.0;
    qint64 m_audioDecodedPositionMs = -1;
    QByteArray m_pendingAudioPcm;
    bool m_eof = false;
    bool m_videoDraining = false;
    bool m_audioDraining = false;
    std::deque<AVPacket*> m_pendingVideoPackets;
    std::deque<PendingVideoFrame> m_pendingVideoFrames;
    DebugStats m_debugStats;

    [[nodiscard]] bool openVideoDecoder(QString& errorMessage);
    [[nodiscard]] bool openAudioDecoder(QString& errorMessage);
    [[nodiscard]] bool openResampler(QString& errorMessage);
    [[nodiscard]] bool sendAudioPacket(AVPacket *packet, QByteArray& pcmS16Stereo, QString& errorMessage);
    [[nodiscard]] bool sendVideoPacket(AVPacket *packet, QString& errorMessage);
    [[nodiscard]] bool receiveVideoFrame(QImage& image, qint64& positionMs, QString& errorMessage);
    [[nodiscard]] bool queueOneDecodedVideoFrame(QString& errorMessage);
    [[nodiscard]] bool queueDecodedVideoFrames(QString& errorMessage);
    [[nodiscard]] bool finishFrameAudio(QByteArray& decodedAudio, QByteArray& pcmS16Stereo, qint64 videoPositionMs, QString& errorMessage);
    [[nodiscard]] bool readAheadAudio(QByteArray& pcmS16Stereo, qint64 videoPositionMs, QString& errorMessage);
    [[nodiscard]] bool drainAudio(QByteArray& pcmS16Stereo, QString& errorMessage);
    [[nodiscard]] bool appendFrameAudio(const AVFrame *frame, QByteArray& pcmS16Stereo, QString& errorMessage);
    [[nodiscard]] bool convertFrameToImage(const AVFrame *frame, QImage& image, QString& errorMessage);
    void takePacedAudio(QByteArray& pcmS16Stereo);
    void clearPendingVideoPackets();
    void closeAudioDecoder();
};

#endif // INCLUDE_FEATURE_CAMERA_VIDEO_FILE_DECODER_H_
