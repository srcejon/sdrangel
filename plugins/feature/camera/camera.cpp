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

#include <QCoreApplication>
#include <QDebug>

#include "camera.h"
#include "cameraworker.h"

MESSAGE_CLASS_DEFINITION(Camera::MsgConfigureCamera, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgRefreshCameraList, Message)

const char* const Camera::m_featureIdURI = "sdrangel.feature.camera";
const char* const Camera::m_featureId = "Camera";

Camera::Camera(WebAPIAdapterInterface *webAPIAdapterInterface) :
    Feature(m_featureIdURI, webAPIAdapterInterface),
    m_thread(nullptr),
    m_worker(nullptr)
{
    setObjectName(m_featureId);
    m_state = StIdle;
    m_errorMessage = "Camera error";
}

Camera::~Camera()
{
    stop();
}

void Camera::start()
{
    qDebug("Camera::start");
    if (m_thread) {
        return;
    }
    m_thread = new QThread();
    m_worker = new CameraWorker();
    m_worker->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::started, m_worker, &CameraWorker::startWork);
    QObject::connect(m_thread, &QThread::finished, m_worker, &QObject::deleteLater);
    QObject::connect(m_thread, &QThread::finished, m_thread, &QThread::deleteLater);

    m_worker->setMessageQueueToGUI(getMessageQueueToGUI());
    m_thread->start();
    m_state = StRunning;

    m_worker->getInputMessageQueue()->push(CameraWorker::MsgConfigureCameraWorker::create(m_settings, QList<QString>(), true));

    // Notify GUI that the worker has started so it can (re)start the Qt camera
    if (m_guiMessageQueue) {
        m_guiMessageQueue->push(MsgConfigureCamera::create(m_settings, QList<QString>(), true));
    }

    if (m_settings.m_captureActive) {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgStartStop::create(true));
    }
}

void Camera::stop()
{
    qDebug("Camera::stop");
    m_state = StIdle;

    if (m_thread)
    {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgStartStop::create(false));
        m_thread->quit();
        m_thread->wait();
        m_thread = nullptr;
        m_worker = nullptr;
    }
}

bool Camera::handleMessage(const Message& cmd)
{
    if (MsgConfigureCamera::match(cmd))
    {
        MsgConfigureCamera& cfg = (MsgConfigureCamera&) cmd;
        applySettings(cfg.getSettings(), cfg.getSettingsKeys(), cfg.getForce());
        return true;
    }
    else if (MsgStartStop::match(cmd))
    {
        MsgStartStop& msg = (MsgStartStop&) cmd;

        if (msg.getStartStop()) {
            start();
        } else {
            stop();
        }

        return true;
    }
    else if (MsgRefreshCameraList::match(cmd))
    {
        // FIXME: Move camera detection to this thread, so we don't have to start the worker?
        start();

        if (m_worker) {
            m_worker->getInputMessageQueue()->push(CameraWorker::MsgRefreshCameraList::create());
        }

        return true;
    }

    return false;
}

QByteArray Camera::serialize() const
{
    return m_settings.serialize();
}

bool Camera::deserialize(const QByteArray& data)
{
    if (m_settings.deserialize(data))
    {
        MsgConfigureCamera *msg = MsgConfigureCamera::create(m_settings, QList<QString>(), true);
        m_inputMessageQueue.push(msg);
        return true;
    }
    else
    {
        m_settings.resetToDefaults();
        MsgConfigureCamera *msg = MsgConfigureCamera::create(m_settings, QList<QString>(), true);
        m_inputMessageQueue.push(msg);
        return false;
    }
}

void Camera::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "Camera::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    if (m_worker) {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgConfigureCameraWorker::create(settings, settingsKeys, force));
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}
