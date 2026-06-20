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

#include <atomic>
#include <deque>

#include <QByteArray>
#include <QImage>
#include <QMutex>
#include <QString>

#include "cameraimagepool.h"

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
    [[nodiscard]] bool abortRequested() const { return m_abortRequested.load(); }
    [[nodiscard]] bool open(
        const QString& fileName,
        QString& errorMessage,
        int outputSampleRate = 48000);
    void requestAbort();
    void close();
    void seek(qint64 positionMs);
    void setAudioPaceFrameRate(double frameRate);
    // Cap on the decoder-side pending-audio staging buffer for live sources (see
    // trimLivePendingAudio). Defaults to 1200 ms; the controller raises it to track
    // streamBufferingSeconds so a deep buffer setting isn't undercut by the decoder
    // trimming audio before it reaches the (larger) downstream stream-audio buffer.
    void setMaxLivePendingAudioMs(int ms) { m_maxLivePendingAudioMs = ms < 1 ? 1 : ms; }
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
    [[nodiscard]] int pendingAudioBytes() const;
    int takePendingAudio(QByteArray& pcmS16Stereo, int maxSampleFrames);
    [[nodiscard]] int pendingVideoFrameCount() const { return static_cast<int>(m_pendingVideoFrames.size()); }
    [[nodiscard]] int pendingVideoPacketCount() const { return static_cast<int>(m_pendingVideoPackets.size()); }

private:
    struct PendingVideoFrame
    {
        QImage m_image;
        qint64 m_positionMs = -1;
    };

    static constexpr size_t m_maxPendingVideoFrames = 3;
    static constexpr size_t m_maxPendingStreamVideoFrames = 4;
    static constexpr size_t m_maxPendingVideoPackets = 30;
    // Keep a modest packet cushion for live streams; the monitor FIFO provides
    // the main audio jitter buffer, so excessive packet parking makes video lag.
    static constexpr size_t m_maxPendingStreamVideoPackets = 4;

    AVFormatContext *m_formatContext = nullptr;
    AVCodecContext *m_videoCodecContext = nullptr;
    AVCodecContext *m_audioCodecContext = nullptr;
    AVFrame *m_videoFrame = nullptr;
    AVFrame *m_audioFrame = nullptr;
    AVPacket *m_packet = nullptr;
    SwsContext *m_swsContext = nullptr;
    // Source geometry/format the current m_swsContext was built for. The
    // converter is rebuilt when a decoded frame's actual width/height/format
    // differs from these (NOT from the codec context, which may already match
    // after a mid-stream change and would skip a needed rebuild). -1 = none.
    int m_swsSrcWidth = 0;
    int m_swsSrcHeight = 0;
    int m_swsSrcFormat = -1;
    SwrContext *m_resampler = nullptr;
    int m_videoStreamIndex = -1;
    int m_audioStreamIndex = -1;
    qint64 m_durationMs = 0;
    double m_frameRate = 25.0;
    int m_outputSampleRate = 48000;
    // Cap (ms) on the live-source pending-audio staging buffer; settable so it can
    // track the controller's streamBufferingSeconds (see setMaxLivePendingAudioMs).
    int m_maxLivePendingAudioMs = 1200;
    // Published from the worker thread (setAudioPaceFrameRate) while the decode
    // thread reads it in takePacedAudio, so it must be atomic. m_audioPaceRemainderFrames
    // is owned solely by the decode thread; a rate change is detected there by
    // comparing against m_audioPaceFrameRateApplied, so the remainder is never
    // written cross-thread.
    std::atomic<double> m_audioPaceFrameRate { 0.0 };
    double m_audioPaceRemainderFrames = 0.0;
    double m_audioPaceFrameRateApplied = 0.0;
    qint64 m_audioDecodedPositionMs = -1;
    QByteArray m_pendingAudioPcm;
    mutable QMutex m_pendingAudioMutex;
    bool m_eof = false;
    bool m_videoDraining = false;
    bool m_audioDraining = false;
    bool m_urlSource = false;
    std::atomic_bool m_abortRequested { false };
    std::deque<AVPacket*> m_pendingVideoPackets;
    std::deque<PendingVideoFrame> m_pendingVideoFrames;
    // Recycles the per-frame RGB888 backing buffers produced in
    // convertFrameToImage, cutting malloc/page-fault churn at frame rate. Sized
    // for the in-flight frame count (≤4 stream pending + worker/pipeline copies).
    CameraImagePool m_imagePool { 8 };

    [[nodiscard]] bool openVideoDecoder(QString& errorMessage);
    [[nodiscard]] bool openAudioDecoder(QString& errorMessage);
    [[nodiscard]] bool openAudioDecoderForStream(int streamIndex, QString& errorMessage);
    [[nodiscard]] bool isAudioStream(int streamIndex) const;
    [[nodiscard]] bool isCompatibleAudioStream(int streamIndex) const;
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
    void trimLivePendingAudio();
    void takePacedAudio(QByteArray& pcmS16Stereo);
    void clearPendingAudio();
    void appendPendingAudio(const QByteArray& pcmS16Stereo);
    void clearPendingVideoPackets();
    void closeAudioDecoder();
};

#endif // INCLUDE_FEATURE_CAMERA_VIDEO_FILE_DECODER_H_
