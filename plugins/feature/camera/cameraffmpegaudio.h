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

#ifndef INCLUDE_FEATURE_CAMERA_FFMPEG_AUDIO_H_
#define INCLUDE_FEATURE_CAMERA_FFMPEG_AUDIO_H_

#include <QByteArray>
#include <QString>

struct SwrContext;

/**
 * \brief FFmpeg audio decode/resample helpers for the camera feature.
 *
 * A small utility facade over libswresample for converting interleaved S16
 * stereo PCM between sample rates, plus an avErrorString() helper. The nested
 * PcmS16StereoResampler holds a persistent SwrContext for streaming use: it
 * keeps resampler state (and any carried fractional samples) across successive
 * resample() calls so a continuous stream stays click-free, with flush() to
 * drain trailing samples and reset() to start over. The static
 * resamplePcmS16Stereo() is a stateless one-shot convenience for an isolated
 * buffer.
 *
 * \note When the build lacks CAMERA_FFMPEG_STREAMING, the resample/flush methods
 *       fail with an explanatory errorMessage rather than doing any work.
 * \warning PcmS16StereoResampler is non-copyable and owns its SwrContext; it is
 *          not internally synchronised, so confine each instance to one thread
 *          (the owning encoder/writer typically holds its own).
 */
class CameraFFmpegAudio
{
public:
    class PcmS16StereoResampler
    {
    public:
        PcmS16StereoResampler() = default;
        ~PcmS16StereoResampler();
        PcmS16StereoResampler(const PcmS16StereoResampler&) = delete;
        PcmS16StereoResampler& operator=(const PcmS16StereoResampler&) = delete;

        [[nodiscard]] bool resample(const QByteArray& input,
                                    int inputSampleRate,
                                    int outputSampleRate,
                                    QByteArray& output,
                                    QString& errorMessage);
        [[nodiscard]] bool flush(QByteArray& output, QString& errorMessage);
        void reset();

    private:
        SwrContext *m_context = nullptr;
        int m_inputSampleRate = 0;
        int m_outputSampleRate = 0;
    };

    [[nodiscard]] static QString avErrorString(int errorCode);
    [[nodiscard]] static bool resamplePcmS16Stereo(const QByteArray& input,
                                                   int inputSampleRate,
                                                   int outputSampleRate,
                                                   QByteArray& output,
                                                   QString& errorMessage);
};

#endif // INCLUDE_FEATURE_CAMERA_FFMPEG_AUDIO_H_
