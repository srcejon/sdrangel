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

#ifndef INCLUDE_FEATURE_CAMERA_H_
#define INCLUDE_FEATURE_CAMERA_H_

#include <QThread>

#include "feature/feature.h"
#include "util/message.h"
#include "camerasettings.h"
#include "camerapostprocessor.h"
#include "cameraworker.h"

class WebAPIAdapterInterface;
class CameraFinder;

class Camera : public Feature
{
    Q_OBJECT
public:
    class MsgConfigureCamera : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const CameraSettings& getSettings() const { return m_settings; }
        const QList<QString>& getSettingsKeys() const { return m_settingsKeys; }
        bool getForce() const { return m_force; }

        static MsgConfigureCamera* create(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
        {
            return new MsgConfigureCamera(settings, settingsKeys, force);
        }

    private:
        CameraSettings m_settings;
        QList<QString> m_settingsKeys;
        bool m_force;

        MsgConfigureCamera(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force) :
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

    Camera(WebAPIAdapterInterface *webAPIAdapterInterface);
    ~Camera() override;
    void destroy() override { delete this; }
    bool handleMessage(const Message& cmd) override;

    void getIdentifier(QString& id) const override { id = objectName(); }
    QString getIdentifier() const override { return objectName(); }
    void getTitle(QString& title) const override { title = m_settings.m_title; }

    QByteArray serialize() const override;
    bool deserialize(const QByteArray& data) override;

    static const char* const m_featureIdURI;
    static const char* const m_featureId;

    MessageQueue* getWorkerInputMessageQueue() { return m_worker ? m_worker->getInputMessageQueue() : nullptr; }
    MessageQueue* getPostProcessorInputMessageQueue() { return m_postProcessor ? m_postProcessor->getInputMessageQueue() : nullptr; }
    void setMessageQueueToGUI(MessageQueue *queue) override;

private:
    QThread *m_workerThread;
    CameraWorker *m_worker;
    QThread *m_postProcessorThread;
    CameraPostProcessor *m_postProcessor;
    CameraFinder *m_cameraFinder;
    CameraSettings m_settings;

    void start();
    void stop();
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
};

#endif // INCLUDE_FEATURE_CAMERA_H_
