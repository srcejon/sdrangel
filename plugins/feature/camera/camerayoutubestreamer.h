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

#ifndef INCLUDE_FEATURE_CAMERA_YOUTUBE_STREAMER_H_
#define INCLUDE_FEATURE_CAMERA_YOUTUBE_STREAMER_H_

#include <QElapsedTimer>
#include <QImage>
#include <QString>
#include <QSize>

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct SwsContext;

class CameraYouTubeStreamer
{
public:
    struct Settings
    {
        QString m_url;
        QString m_key;
        int m_bitrateKbps = 2500;
        int m_fps = 25;
        QSize m_size;
    };

    CameraYouTubeStreamer();
    ~CameraYouTubeStreamer();

    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] bool open(const Settings& settings, const QImage& firstFrame, QString& errorMessage);
    [[nodiscard]] bool writeFrame(const QImage& image, QString& errorMessage);
    void close();

private:
    Settings m_settings;
    QSize m_streamSize;
    AVFormatContext *m_formatContext = nullptr;
    AVCodecContext *m_codecContext = nullptr;
    AVFrame *m_frame = nullptr;
    SwsContext *m_swsContext = nullptr;
    int m_streamIndex = -1;
    qint64 m_frameIndex = 0;
    qint64 m_lastFrameElapsedMs = -1;
    qint64 m_nextFrameElapsedMs = 0;
    bool m_headerWritten = false;
    QElapsedTimer m_streamTimer;

    [[nodiscard]] static QString avErrorString(int errorCode);
    [[nodiscard]] static QString streamTargetUrl(const Settings& settings);
    [[nodiscard]] static QString redactedStreamTargetUrl(const QString& targetUrl);
    [[nodiscard]] static QSize evenSize(const QSize& size);
    [[nodiscard]] static QImage prepareRgbImage(const QImage& image, const QSize& size);
    [[nodiscard]] bool encodeAndWriteRgbFrame(const QImage& rgb, QString& errorMessage);
};

#endif // INCLUDE_FEATURE_CAMERA_YOUTUBE_STREAMER_H_
