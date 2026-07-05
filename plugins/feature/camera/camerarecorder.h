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

#ifndef INCLUDE_FEATURE_CAMERARECORDER_H_
#define INCLUDE_FEATURE_CAMERARECORDER_H_

#include <QObject>
#include <deque>
#include <memory>

#include <QByteArray>
#include <QDateTime>
#include <QImage>
#include <QMutex>
#include <QSet>
#include <QSize>

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraPostProcessor;
class CameraVideoWriter;
class CameraYouTubeStreamer;

/**
 * \brief Pipeline stage that records camera output to video/image files and
 *        streams it to YouTube.
 *
 * Receives processed pipeline frames (submitFrame) and audio (via MsgAudioSamples
 * on its input message queue), then drives the recording outputs: one or more
 * CameraVideoWriter instances (for the calibrated/filtered/post-processed image
 * variants), per-frame image files (FITS for raw), a rolling keogram, and an
 * optional CameraYouTubeStreamer. It maintains a pre-record ring buffer so a
 * recording can include frames captured just before it was armed, and buffers
 * pending audio to mux against the video. Settings/control arrive as Messages;
 * state and the keogram are reported back via the GUI/feature message queues.
 *
 * \note Runs on its own dedicated recorder thread (moved there in Camera setup,
 *       started via startWork). Cross-thread input arrives through
 *       m_inputMessageQueue (handleInputMessages slot) and submitFrame, which
 *       appends to m_pendingFrames under m_frameMutex; actual processing happens
 *       on the recorder thread in processNextFrames.
 * \warning The pre-record and pending-audio buffers are byte-capped; the video
 *          output queue is frame-capped. When a cap is exceeded frames are
 *          dropped (m_droppedOutputFrames) rather than allowed to grow without
 *          bound.
 */
class CameraRecorder : public QObject
{
    Q_OBJECT
public:

    class MsgSetVideoRecordingEnabled : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool getEnabled() const { return m_enabled; }

        static MsgSetVideoRecordingEnabled* create(bool enabled)
        {
            return new MsgSetVideoRecordingEnabled(enabled);
        }

    private:
        bool m_enabled;

        MsgSetVideoRecordingEnabled(bool enabled) :
            Message(),
            m_enabled(enabled)
        { }
    };


    class MsgReportSaveVideoState : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool getSaveVideo() const { return m_saveVideo; }

        static MsgReportSaveVideoState* create(bool saveVideo)
        {
            return new MsgReportSaveVideoState(saveVideo);
        }

    private:
        bool m_saveVideo;

        MsgReportSaveVideoState(bool saveVideo) :
            Message(),
            m_saveVideo(saveVideo)
        { }
    };

    class MsgReportSaveImageState : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool getSaveImage() const { return m_saveImage; }

        static MsgReportSaveImageState* create(bool saveImage)
        {
            return new MsgReportSaveImageState(saveImage);
        }

    private:
        bool m_saveImage;

        MsgReportSaveImageState(bool saveImage) :
            Message(),
            m_saveImage(saveImage)
        { }
    };

    class MsgReportKeogram : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QImage& getImage() const { return m_image; }
        const QString& getFileName() const { return m_fileName; }
        bool getVisible() const { return m_visible; }

        static MsgReportKeogram* create(const QImage& image, const QString& fileName, bool visible)
        {
            return new MsgReportKeogram(image, fileName, visible);
        }

    private:
        QImage m_image;
        QString m_fileName;
        bool m_visible;

        MsgReportKeogram(const QImage& image, const QString& fileName, bool visible) :
            Message(),
            m_image(image),
            m_fileName(fileName),
            m_visible(visible)
        { }
    };

    class MsgReportPreRecordPreview : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QImage& getImage() const { return m_image; }
        qint64 getOffsetMs() const { return m_offsetMs; }
        qint64 getBufferDurationMs() const { return m_bufferDurationMs; }

        static MsgReportPreRecordPreview* create(const QImage& image, qint64 offsetMs, qint64 bufferDurationMs)
        {
            return new MsgReportPreRecordPreview(image, offsetMs, bufferDurationMs);
        }

    private:
        QImage m_image;
        qint64 m_offsetMs;
        qint64 m_bufferDurationMs;

        MsgReportPreRecordPreview(const QImage& image, qint64 offsetMs, qint64 bufferDurationMs) :
            Message(),
            m_image(image),
            m_offsetMs(offsetMs),
            m_bufferDurationMs(bufferDurationMs)
        { }
    };

    class MsgRequestPreRecordPreview : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        qint64 getOffsetMs() const { return m_offsetMs; }

        static MsgRequestPreRecordPreview* create(qint64 offsetMs)
        {
            return new MsgRequestPreRecordPreview(offsetMs);
        }

    private:
        qint64 m_offsetMs;

        MsgRequestPreRecordPreview(qint64 offsetMs) :
            Message(),
            m_offsetMs(offsetMs)
        { }
    };

    class MsgAudioSamples : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QByteArray& getPcmS16Stereo() const { return m_pcmS16Stereo; }
        int getSampleRate() const { return m_sampleRate; }
        // Source content position (ms) this audio belongs to, or -1 for live capture
        // (no content clock). Used to re-align recorded file-playback audio with the
        // content-timestamped video; see CameraRecorder::appendAudioSamples.
        qint64 getContentPositionMs() const { return m_contentPositionMs; }

        static MsgAudioSamples* create(const QByteArray& pcmS16Stereo, int sampleRate, qint64 contentPositionMs = -1)
        {
            return new MsgAudioSamples(pcmS16Stereo, sampleRate, contentPositionMs);
        }

    private:
        QByteArray m_pcmS16Stereo;
        int m_sampleRate;
        qint64 m_contentPositionMs;

        MsgAudioSamples(const QByteArray& pcmS16Stereo, int sampleRate, qint64 contentPositionMs) :
            Message(),
            m_pcmS16Stereo(pcmS16Stereo),
            m_sampleRate(sampleRate),
            m_contentPositionMs(contentPositionMs)
        { }
    };

    CameraRecorder();
    ~CameraRecorder();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setNextStage(CameraPostProcessor *nextStage) { m_nextStage = nextStage; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }
    void setMessageQueueToFeature(MessageQueue *messageQueue) { m_msgQueueToFeature = messageQueue; }

private:
    struct BufferedVideoFrame
    {
        QImage m_calibratedImage;
        QImage m_filteredImage;
        QImage m_processedImage;
    };

    struct AudioChunk
    {
        QByteArray m_pcmS16Stereo;
        int m_sampleRate = 48000;
    };

    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_msgQueueToFeature;
    CameraPostProcessor *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    quint64 m_captureEpoch = 0;
    std::unique_ptr<CameraVideoWriter> m_calibratedVideoWriter;
    std::unique_ptr<CameraVideoWriter> m_filteredVideoWriter;
    std::unique_ptr<CameraVideoWriter> m_processedVideoWriter;
    QSize m_calibratedVideoWriterSize;
    QSize m_filteredVideoWriterSize;
    QSize m_processedVideoWriterSize;
    std::deque<BufferedVideoFrame> m_preRecordVideoFrames;
    bool m_preRecordBufferFlushed;
    int m_recordedImageFrames;
    QDateTime m_videoRecordingStartDateTime;
    QImage m_keogramImage;
    QDateTime m_keogramWindowStartUtc;
    QSize m_keogramSourceSize;
    QString m_keogramOutputFileName;
    int m_keogramLastSampleIndex;
    std::deque<AudioChunk> m_pendingAudioChunks;
    qint64 m_pendingAudioBytes;
    // A/V-sync alignment for file-playback recording: the content position of the
    // first recorded video frame and of the first recorded audio chunk. Their
    // difference is the audio lead (the recorder receives audio at presentation time
    // but the video frame later, after the sink-latency delay + processing pipeline),
    // which is prepended as silence to each writer's audio. -1 until known; reset per
    // recording in closeVideoWriters().
    qint64 m_recordAudioLeadRefVideoMs = -1;
    qint64 m_recordAudioFirstChunkMs = -1;
    // YouTube-stream-scoped equivalent of m_recordAudioLeadRefVideoMs (the file path's anchor is set
    // only when saving video). Used to prepend the same read-ahead audio lead to the YouTube stream.
    qint64 m_youtubeAudioLeadRefVideoMs = -1;
    bool m_youtubeAudioLeadApplied = false;
    // Duration (ms) of the video-only pre-record lead-in flushed at the front of the
    // recording. Those frames carry no audio, so this is added to the audio lead
    // silence to keep the live portion lip-synced. 0 when there is no pre-record.
    qint64 m_recordPreRecordLeadMs = 0;
    bool m_recordAudioLeadLogged = false;   // one-shot diagnostic for the measured A/V lead
    std::unique_ptr<CameraYouTubeStreamer> m_youtubeStreamer;
    bool m_youtubeStreamErrorReported;
    QMutex m_frameMutex;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
    bool m_processingFrames;
    int m_droppedOutputFrames;
    QSet<QString> m_reportedVideoWriterErrorKeys;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    void forwardFrame(const CameraPipelineFramePtr& frame);
    void setVideoRecordingEnabled(bool enabled);
    void setImageRecordingEnabled(bool enabled);
    void resetRecordingLimits();
    void updateRecordingLimitsAfterFrame(bool savedImageFrame, bool savedVideoFrame);
    void appendAudioSamples(const QByteArray& pcmS16Stereo, int sampleRate, qint64 contentPositionMs = -1);
    bool writePendingAudio(CameraVideoWriter& writer, const QString& variant);
    void trimPendingAudio();
    [[nodiscard]] static QString createTimestampedOutputFilename(const QString& baseFileName, const QString& variant, const QString& suffixOverride = QString());
    [[nodiscard]] bool shouldSaveRawFits() const;
    [[nodiscard]] bool shouldSaveCalibratedMedia() const;
    [[nodiscard]] bool shouldSaveFilteredMedia() const;
    [[nodiscard]] bool shouldSavePostProcessedMedia() const;
    [[nodiscard]] bool saveRawFits(const QString& fileName,
                                   const QImage& image,
                                   CameraPipelineFrame::BayerPattern bayerPattern,
                                   const CameraPipelineFrame& frame) const;
    void closeVideoWriters();
    void closeYouTubeStream();
    void updateYouTubeStream(const QImage& calibratedImage, const QImage& processedImage, qint64 videoContentMs);
    bool ensureVideoWriter(std::unique_ptr<CameraVideoWriter>& writer, const QString& baseFileName, const QImage& frameForSize, const QString& variant, double frameRate);
    bool writeVideoFrame(CameraVideoWriter& writer, const QImage& frameToWrite, const QString& variant, qint64 timestampMs = -1);
    void reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage);
    int preRecordBufferFrameLimit() const;
    int outputQueueFrameLimit() const;
    void trimPreRecordBuffer();
    void appendPreRecordFrame(const QImage& calibratedImage, const QImage& filteredImage, const QImage& processedImage);
    void flushPreRecordFrames(const QImage& currentCalibratedImage, const QImage& currentFilteredImage, const QImage& currentProcessedImage, double frameRate);
    void reportPreRecordPreviewFrame(qint64 offsetMs);
    void updateKeogram(const QImage& calibratedImage, const QDateTime& captureDateTime);
    void resetKeogram();
    [[nodiscard]] QDateTime keogramWindowStartUtc(const QDateTime& captureDateTime) const;
    [[nodiscard]] int keogramSampleCount() const;
    [[nodiscard]] int keogramSampleIndex(const QDateTime& captureDateTime) const;
    [[nodiscard]] QString keogramOutputFileName(const QDateTime& windowStartUtc) const;
    [[nodiscard]] static QImage rgbImageForKeogram(const QImage& image);

private slots:
    void handleInputMessages();
    void processNextFrames();
};

#endif // INCLUDE_FEATURE_CAMERARECORDER_H_
