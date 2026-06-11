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

#ifndef INCLUDE_FEATURE_CAMERA_VIDEO_WRITER_H_
#define INCLUDE_FEATURE_CAMERA_VIDEO_WRITER_H_

#include <QImage>
#include <QSize>
#include <QString>

#include "camerasettings.h"

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

class CameraVideoWriter
{
public:
    struct Settings
    {
        QString m_fileName;
        CameraSettings::VideoCodec m_codec = CameraSettings::VideoCodecH264;
        double m_fps = 25.0;
        bool m_preferHardwareEncoding = true;
    };

    CameraVideoWriter();
    ~CameraVideoWriter();

    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] QSize size() const { return m_videoSize; }
    [[nodiscard]] CameraSettings::VideoCodec codec() const { return m_settings.m_codec; }
    [[nodiscard]] static QString codecName(CameraSettings::VideoCodec codec);
    [[nodiscard]] QString codecName() const;
    [[nodiscard]] bool open(const Settings& settings, const QImage& firstFrame, QString& errorMessage);
    [[nodiscard]] bool writeFrame(const QImage& image, QString& errorMessage);
    void close();

private:
    Settings m_settings;
    QSize m_videoSize;
    AVFormatContext *m_formatContext = nullptr;
    AVCodecContext *m_codecContext = nullptr;
    AVFrame *m_frame = nullptr;
    SwsContext *m_swsContext = nullptr;
    int m_streamIndex = -1;
    qint64 m_frameIndex = 0;
    bool m_headerWritten = false;

    [[nodiscard]] static QString avErrorString(int errorCode);
    [[nodiscard]] static QSize evenSize(const QSize& size);
    [[nodiscard]] static QImage prepareRgbImage(const QImage& image, const QSize& size);
    [[nodiscard]] bool writeEncodedPacket(AVPacket *packet, QString& errorMessage);
    void flushEncoder();
};

#endif // INCLUDE_FEATURE_CAMERA_VIDEO_WRITER_H_
