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

#include <QDateTime>
#include <QImage>
#include <QMutex>
#include <QSet>
#include <QSize>

#include <opencv2/videoio.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class CameraPostProcessor;

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
        QImage m_processedImage;
    };

    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_msgQueueToFeature;
    CameraPostProcessor *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    quint64 m_captureEpoch = 0;
    cv::VideoWriter m_calibratedVideoWriter;
    cv::VideoWriter m_processedVideoWriter;
    QSize m_calibratedVideoWriterSize;
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
    [[nodiscard]] static QString createTimestampedOutputFilename(const QString& baseFileName, const QString& variant, const QString& suffixOverride = QString());
    [[nodiscard]] bool shouldSaveRawFits() const;
    [[nodiscard]] bool shouldSaveCalibratedMedia() const;
    [[nodiscard]] bool shouldSavePostProcessedMedia() const;
    [[nodiscard]] bool saveRawFits(const QString& fileName,
                                   const QImage& image,
                                   CameraPipelineFrame::BayerPattern bayerPattern,
                                   const CameraPipelineFrame& frame) const;
    void closeVideoWriters();
    bool ensureVideoWriter(cv::VideoWriter& writer, const QString& baseFileName, const QImage& frameForSize, const QString& variant);
    void writeVideoFrame(cv::VideoWriter& writer, const QImage& frameToWrite);
    void reportErrorToFeature(const QString& errorKey, const QString& title, const QString& errorMessage);
    int preRecordBufferFrameLimit() const;
    int outputQueueFrameLimit() const;
    void trimPreRecordBuffer();
    void appendPreRecordFrame(const QImage& calibratedImage, const QImage& processedImage);
    void flushPreRecordFrames(const QImage& currentCalibratedImage, const QImage& currentProcessedImage);
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
