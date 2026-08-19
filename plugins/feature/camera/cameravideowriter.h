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
#include <QByteArray>
#include <QSize>
#include <QString>

#include "cameraffmpegaudio.h"
#include "camerasettings.h"

struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

/**
 * \brief FFmpeg muxer/encoder that writes camera frames + audio to a video file.
 *
 * Wraps libav* to open an output container (codec/bitrate/fps/audio from
 * Settings, with optional hardware encoding) sized to the first frame, then
 * encodes successive QImages (converted to the encoder pixel format via an
 * SwsContext) and interleaved S16 stereo PCM (resampled to the encoder's audio
 * rate via the embedded PcmS16StereoResampler) into the muxed file. Frame
 * timestamps can be driven explicitly (timestampMs) or derived from a fixed
 * frame duration. close() flushes the encoders and finalises the container.
 *
 * \note Single-threaded: open()/writeFrame()/writePcmS16Stereo()/close() are
 *       meant to be called in sequence from one thread (the recorder thread that
 *       owns the writer). The instance is non-reentrant and holds raw FFmpeg
 *       contexts freed in close()/the destructor.
 * \warning In a build without CAMERA_FFMPEG_STREAMING the writer is inert; the
 *          open/write methods report an error via errorMessage instead of
 *          producing output.
 */
class CameraVideoWriter
{
public:
    struct Settings
    {
        QString m_fileName;
        CameraSettings::VideoCodec m_codec = CameraSettings::VideoCodecH264;
        int m_bitrateKbps = 0;
        double m_fps = 25.0;
        bool m_preferHardwareEncoding = true;
        bool m_audioEnabled = true;
        QByteArray m_cameraMetadataJson;
    };

    CameraVideoWriter();
    ~CameraVideoWriter();

    [[nodiscard]] bool isOpen() const;
    [[nodiscard]] QSize size() const { return m_videoSize; }
    [[nodiscard]] CameraSettings::VideoCodec codec() const { return m_settings.m_codec; }
    [[nodiscard]] int bitrateKbps() const { return m_settings.m_bitrateKbps; }
    [[nodiscard]] double fps() const { return m_settings.m_fps; }
    [[nodiscard]] QString fileName() const { return m_settings.m_fileName; }
    [[nodiscard]] static QString codecName(CameraSettings::VideoCodec codec);
    [[nodiscard]] static bool updateFileMetadata(
        const QString& fileName,
        const QByteArray& cameraMetadataJson,
        QString& errorMessage);
    [[nodiscard]] QString codecName() const;
    [[nodiscard]] bool open(const Settings& settings, const QImage& firstFrame, QString& errorMessage);
    [[nodiscard]] bool writeFrame(const QImage& image, QString& errorMessage, qint64 timestampMs = -1);
    [[nodiscard]] bool writePcmS16Stereo(const QByteArray& pcm, int sampleRate, QString& errorMessage);
    // Prepend this many milliseconds of silence to the audio track, once, before the
    // first real audio samples are written. Used to re-align recorded audio with the
    // (content-timestamped) video: the recorder feeds audio at presentation time but
    // the video frames reach it later (sink-latency video delay + processing
    // pipeline), so the audio otherwise leads the video. Ignored once audio has
    // started; set it before the first writePcmS16Stereo call.
    void setAudioLeadSilenceMs(int milliseconds) { m_audioWriter.setLeadSilenceMs(milliseconds); }
    void close();

private:
    Settings m_settings;
    QSize m_videoSize;
    AVFormatContext *m_formatContext = nullptr;
    AVCodecContext *m_codecContext = nullptr;
    AVFrame *m_frame = nullptr;
    SwsContext *m_swsContext = nullptr;
    CameraFFmpegAacStreamWriter m_audioWriter;   // owns the AAC stream in m_formatContext
    int m_streamIndex = -1;
    qint64 m_frameIndex = 0;
    qint64 m_frameDurationPts = 40;
    qint64 m_firstFrameTimestampMs = -1;
    qint64 m_lastVideoPts = -1;
    bool m_headerWritten = false;

    [[nodiscard]] static QString avErrorString(int errorCode);
    [[nodiscard]] static QSize evenSize(const QSize& size);
    [[nodiscard]] static QImage prepareRgbImage(const QImage& image, const QSize& size);
    [[nodiscard]] bool writeEncodedPacket(AVPacket *packet, AVCodecContext *codecContext, int streamIndex, QString& errorMessage);
    void flushEncoder(AVCodecContext *codecContext, int streamIndex);
};

#endif // INCLUDE_FEATURE_CAMERA_VIDEO_WRITER_H_
