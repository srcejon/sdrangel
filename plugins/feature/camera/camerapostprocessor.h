///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE <jon@beniston.com>                     //
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

#ifndef INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_
#define INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_

#include <QObject>
#include <QHash>
#include <QImage>
#include <QDateTime>
#include <QSet>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/dnn/dnn.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerasettings.h"

class CameraPostProcessor : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraPostProcessor : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraPostProcessor* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraPostProcessor(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraPostProcessor(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    class MsgProcessFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QImage& getImage() const { return m_image; }

        static MsgProcessFrame* create(const QImage& image)
        {
            return new MsgProcessFrame(image);
        }

    private:
        QImage m_image;

        MsgProcessFrame(const QImage& image) :
            Message(),
            m_image(image)
        { }
    };

    class MsgSpectrumFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QImage& getImage() const { return m_image; }

        static MsgSpectrumFrame* create(const QImage& image)
        {
            return new MsgSpectrumFrame(image);
        }

    private:
        QImage m_image;

        MsgSpectrumFrame(const QImage& image) :
            Message(),
            m_image(image)
        { }
    };

    class MsgReportFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QImage& getImage() const { return m_image; }

        static MsgReportFrame* create(const QImage& image)
        {
            return new MsgReportFrame(image);
        }

    private:
        QImage m_image;

        MsgReportFrame(const QImage& image) :
            Message(),
            m_image(image)
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

    CameraPostProcessor();
    ~CameraPostProcessor();

    void startWork();
    void stopWork();
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }

private:
    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    CameraSettings m_settings;
    bool m_captureActive;
    bool m_imageSaved;
    QImage m_lastRawFrame;
    QImage m_previousRawFrame;
    QDateTime m_captureDateTime;
    cv::Ptr<cv::BackgroundSubtractorMOG2> m_bgSubtractor;
    cv::dnn::Net m_yoloNet;
    QString m_yoloLoadedModelPath;
    QStringList m_yoloLabels;
    QString m_yoloLoadedLabelsPath;
    QSet<QString> m_detectedObjectClasses;
    QHash<QString, QDateTime> m_pendingDisappearDeadlines;
    cv::VideoWriter m_videoWriter;
    QImage m_spectrumViewImage;

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const QImage& image);
    [[nodiscard]] QImage applyPostProcessing(const QImage& input);
    void runYoloDetections(cv::Mat& bgrMat);
    void processObjectDetections(const QSet<QString>& currentDetectedClasses, const QDateTime& now);
    void applyObjectDetectedSettings(const QString& className);
    void applyObjectDisappearedSettings(const QString& className);
    void executeCommand(const QString& command, const QString& className);
    bool shouldRecordVideoForDetectedObjects() const;
    void setVideoRecordingEnabled(bool enabled);
    void reportFrameToGUI(const QImage& image);

private slots:
    void handleInputMessages();
};

#endif // INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_
