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
    stopWorker();
}

void Camera::startWorker()
{
    if (m_thread)
    {
        if (m_worker) {
            m_worker->setMessageQueueToGUI(getMessageQueueToGUI());
        }
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

    m_worker->getInputMessageQueue()->push(CameraWorker::MsgConfigureCameraWorker::create(m_settings, QList<QString>(), true));

    if (m_settings.m_captureActive) {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgStartStop::create(true));
        m_state = StRunning;
    }
}

void Camera::stopWorker()
{
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
        m_settings.m_captureActive = msg.getStartStop();
        startWorker();

        if (m_worker) {
            m_worker->getInputMessageQueue()->push(CameraWorker::MsgStartStop::create(msg.getStartStop()));
        }

        m_state = msg.getStartStop() ? StRunning : StIdle;
        return true;
    }
    else if (MsgRefreshCameraList::match(cmd))
    {
        startWorker();

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
    const bool success = m_settings.deserialize(data);

    if (!success) {
        m_settings.resetToDefaults();
    }

    m_inputMessageQueue.push(MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    if (m_settings.m_captureActive) {
        m_inputMessageQueue.push(MsgStartStop::create(true));
    }

    return success;
}

void Camera::applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    qDebug() << "Camera::applySettings:" << settings.getDebugString(settingsKeys, force) << "force:" << force;

    startWorker();

    if (m_worker) {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgConfigureCameraWorker::create(settings, settingsKeys, force));
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (m_settings.m_captureActive) {
        m_state = StRunning;
    } else {
        m_state = StIdle;
    }

    if (m_guiMessageQueue)
    {
        m_guiMessageQueue->push(MsgConfigureCamera::create(m_settings, settingsKeys, force));
    }
}
