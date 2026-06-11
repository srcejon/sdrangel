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

#ifndef INCLUDE_FEATURE_CAMERA_VIDEO_FILE_AUDIO_DECODER_H_
#define INCLUDE_FEATURE_CAMERA_VIDEO_FILE_AUDIO_DECODER_H_

#include <QByteArray>
#include <QString>

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct SwrContext;

class CameraVideoFileAudioDecoder
{
public:
    CameraVideoFileAudioDecoder();
    ~CameraVideoFileAudioDecoder();

    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] bool open(const QString& fileName, QString& errorMessage);
    void close();
    void seek(qint64 positionMs);
    [[nodiscard]] bool readPcmTo(qint64 playbackPositionMs, QByteArray& pcmS16Stereo, int& sampleRate, QString& errorMessage);

private:
    static constexpr int m_outputSampleRate = 48000;

    AVFormatContext *m_formatContext = nullptr;
    AVCodecContext *m_codecContext = nullptr;
    AVFrame *m_frame = nullptr;
    AVPacket *m_packet = nullptr;
    SwrContext *m_resampler = nullptr;
    int m_audioStreamIndex = -1;
    qint64 m_outputPositionMs = 0;
    bool m_eof = false;

    [[nodiscard]] bool openResampler(QString& errorMessage);
    [[nodiscard]] bool decodeNextFrame(QByteArray& pcmS16Stereo, QString& errorMessage);
    [[nodiscard]] bool appendFrameAudio(const AVFrame *frame, QByteArray& pcmS16Stereo, QString& errorMessage);
};

#endif // INCLUDE_FEATURE_CAMERA_VIDEO_FILE_AUDIO_DECODER_H_
