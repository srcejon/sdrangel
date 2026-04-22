///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Edouard Griffiths, F4EXB <f4exb06@gmail.com>               //
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

#ifndef INCLUDE_FEATURE_CAMERAWORKER_H_
#define INCLUDE_FEATURE_CAMERAWORKER_H_

#include <QObject>
#include <QTimer>
#include <QImage>
#include <QRecursiveMutex>

#include "util/message.h"
#include "util/messagequeue.h"
#include "camerasettings.h"

class QNetworkAccessManager;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
class QCamera;
class QVideoSink;
class QMediaCaptureSession;
class QMediaRecorder;
class QVideoFrame;
#endif

class CameraWorker : public QObject
{
    Q_OBJECT
public:
    class MsgConfigureCameraWorker : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCameraWorker* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCameraWorker(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCameraWorker(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
            Message(),
            m_settings(settings),
            m_settingsKeys(settingsKeys),
            m_force(force)
        { }
    };

    class MsgStartStop : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool getStartStop() const { return m_startStop; }

        static MsgStartStop* create(bool startStop)
        {
            return new MsgStartStop(startStop);
        }

    private:
        bool m_startStop;

        MsgStartStop(bool startStop) :
            Message(),
            m_startStop(startStop)
        { }
    };

    class MsgRefreshCameraList : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgRefreshCameraList* create()
        {
            return new MsgRefreshCameraList();
        }

    private:
        MsgRefreshCameraList() : Message() {}
    };

    class MsgReportCameraList : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QStringList& getCameraIds() const { return m_cameraIds; }

        static MsgReportCameraList* create(const QStringList& cameraIds)
        {
            return new MsgReportCameraList(cameraIds);
        }

    private:
        QStringList m_cameraIds;

        MsgReportCameraList(const QStringList& cameraIds) :
            Message(),
            m_cameraIds(cameraIds)
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

    CameraWorker();
    ~CameraWorker();

    void startWork();
    void stopWork();
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }

private:
    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    QRecursiveMutex m_mutex;
    CameraSettings m_settings;
    bool m_capturing;
    bool m_imageSaved;
    QTimer m_captureTimer;
    QNetworkAccessManager *m_networkManager;

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    QCamera *m_qtCamera;
    QVideoSink *m_videoSink;
    QMediaCaptureSession *m_captureSession;
    QMediaRecorder *m_mediaRecorder;
#endif

    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void reportCameraList();
    void startCapture();
    void stopCapture();
    void processNewFrame(const QImage& image);
    QImage createPlaceholderFrame() const;

    void reportFrameToGUI(const QImage& image);
    QString buildAlpacaBaseUrl() const;
    QStringList queryAlpacaCameras();
    QImage captureAlpacaImage();

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    void setupQtCapture();
    void cleanupQtCapture();
    void processQtVideoFrame(const QVideoFrame& frame);
#endif

private slots:
    void handleInputMessages();
    void captureTick();
};

#endif // INCLUDE_FEATURE_CAMERAWORKER_H_
