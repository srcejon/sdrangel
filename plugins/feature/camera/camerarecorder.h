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
    class MsgConfigureCameraRecorder : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraRecorder* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraRecorder(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraRecorder(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    class MsgProcessFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraPipelineFramePtr& getFrame() const { return m_frame; }

        static MsgProcessFrame* create(const CameraPipelineFramePtr& frame)
        {
            return new MsgProcessFrame(frame);
        }

    private:
        CameraPipelineFramePtr m_frame;

        MsgProcessFrame(const CameraPipelineFramePtr& frame) :
            Message(),
            m_frame(frame)
        { }
    };

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

    class MsgCaptureActive : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool isActive() const { return m_active; }

        static MsgCaptureActive* create(bool active)
        {
            return new MsgCaptureActive(active);
        }

    private:
        bool m_active;

        MsgCaptureActive(bool active) :
            Message(),
            m_active(active)
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

private:
    struct BufferedVideoFrame
    {
        QImage m_rawImage;
        QImage m_processedImage;
    };

    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    CameraPostProcessor *m_nextStage;
    CameraSettings m_settings;
    bool m_captureActive;
    cv::VideoWriter m_rawVideoWriter;
    cv::VideoWriter m_processedVideoWriter;
    QSize m_rawVideoWriterSize;
    QSize m_processedVideoWriterSize;
    std::deque<BufferedVideoFrame> m_preRecordVideoFrames;
    bool m_preRecordBufferFlushed;
    int m_recordedImageFrames;
    QDateTime m_videoRecordingStartDateTime;
    QMutex m_frameMutex;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
    bool m_processingFrames;
    int m_droppedOutputFrames;

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
    int preRecordBufferFrameLimit() const;
    int outputQueueFrameLimit() const;
    void trimPreRecordBuffer();
    void appendPreRecordFrame(const QImage& rawImage, const QImage& processedImage);
    void flushPreRecordFrames(const QImage& currentRawImage, const QImage& currentProcessedImage);

private slots:
    void handleInputMessages();
    void processNextFrames();
};

#endif // INCLUDE_FEATURE_CAMERARECORDER_H_
