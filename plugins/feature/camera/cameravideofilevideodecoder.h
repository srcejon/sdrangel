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

#ifndef INCLUDE_FEATURE_CAMERA_VIDEO_FILE_VIDEO_DECODER_H_
#define INCLUDE_FEATURE_CAMERA_VIDEO_FILE_VIDEO_DECODER_H_

#include <QImage>
#include <QString>

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

class CameraVideoFileVideoDecoder
{
public:
    CameraVideoFileVideoDecoder();
    ~CameraVideoFileVideoDecoder();

    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] bool open(const QString& fileName, QString& errorMessage);
    void close();
    void seek(qint64 positionMs);
    [[nodiscard]] bool readNextFrame(QImage& image, qint64& positionMs, QString& errorMessage);
    [[nodiscard]] qint64 durationMs() const { return m_durationMs; }
    [[nodiscard]] double frameRate() const { return m_frameRate; }

private:
    AVFormatContext *m_formatContext = nullptr;
    AVCodecContext *m_codecContext = nullptr;
    AVFrame *m_frame = nullptr;
    AVPacket *m_packet = nullptr;
    SwsContext *m_swsContext = nullptr;
    int m_videoStreamIndex = -1;
    qint64 m_durationMs = 0;
    double m_frameRate = 25.0;
    bool m_eof = false;

    [[nodiscard]] bool convertFrameToImage(const AVFrame *frame, QImage& image, QString& errorMessage);
};

#endif // INCLUDE_FEATURE_CAMERA_VIDEO_FILE_VIDEO_DECODER_H_
