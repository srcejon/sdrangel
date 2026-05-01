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

#include <QCoreApplication>
#include <QDebug>

#include "camera.h"
#include "camerafinder.h"
#include "cameraworker.h"

MESSAGE_CLASS_DEFINITION(Camera::MsgConfigureCamera, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgRefreshCameraList, Message)

const char* const Camera::m_featureIdURI = "sdrangel.feature.camera";
const char* const Camera::m_featureId = "Camera";

Camera::Camera(WebAPIAdapterInterface *webAPIAdapterInterface) :
    Feature(m_featureIdURI, webAPIAdapterInterface),
    m_workerThread(new QThread()),
    m_worker(new CameraWorker()),
    m_postProcessorThread(new QThread()),
    m_postProcessor(new CameraPostProcessor()),
    m_cameraFinder(new CameraFinder(this))
{
    setObjectName(m_featureId);
    m_state = StIdle;
    m_errorMessage = "Camera error";
    m_cameraFinder->setMessageQueueToGUI(getMessageQueueToGUI());

    // The worker needs to run continuously, to be able to query camera capabilities.
    m_worker->moveToThread(m_workerThread);
    QObject::connect(m_workerThread, &QThread::started, m_worker, &CameraWorker::startWork);
    QObject::connect(m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);
    QObject::connect(m_workerThread, &QThread::finished, m_workerThread, &QThread::deleteLater);
    m_worker->setMessageQueueToGUI(getMessageQueueToGUI());
    m_worker->setPostProcessorInputMessageQueue(getPostProcessorInputMessageQueue());
    m_workerThread->start();
    m_worker->getInputMessageQueue()->push(CameraWorker::MsgConfigureCameraWorker::create(m_settings, QList<QString>(), true));

    // The post-processor runs continously, to be able to update the image when post processing settings are changed.
    m_postProcessor->moveToThread(m_postProcessorThread);
    QObject::connect(m_postProcessorThread, &QThread::started, m_postProcessor, &CameraPostProcessor::startWork);
    QObject::connect(m_postProcessorThread, &QThread::finished, m_postProcessor, &QObject::deleteLater);
    QObject::connect(m_postProcessorThread, &QThread::finished, m_postProcessorThread, &QThread::deleteLater);
    m_postProcessor->setMessageQueueToGUI(getMessageQueueToGUI());
    m_postProcessorThread->start();
    m_postProcessor->getInputMessageQueue()->push(CameraPostProcessor::MsgConfigureCameraPostProcessor::create(m_settings, QList<QString>(), true));
}

Camera::~Camera()
{
    stop();

    if (m_workerThread)
    {
        m_workerThread->quit();
        m_workerThread->wait();
        m_workerThread = nullptr;
        m_worker = nullptr;
    }

    if (m_postProcessorThread)
    {
        m_postProcessorThread->quit();
        m_postProcessorThread->wait();
        m_postProcessorThread = nullptr;
        m_postProcessor = nullptr;
    }
}

void Camera::start()
{
    qDebug("Camera::start");
    if (m_state == StRunning) {
        return;
    }

    m_state = StRunning;

    if (m_worker) {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgStartStop::create(true));
    }
    if (m_postProcessor) {
        m_postProcessor->getInputMessageQueue()->push(CameraPostProcessor::MsgCaptureActive::create(true));
    }
    if (m_guiMessageQueue) {
        m_guiMessageQueue->push(Camera::MsgStartStop::create(true));
    }
}

void Camera::stop()
{
    qDebug("Camera::stop");
    if (m_state == StIdle) {
        return;
    }

    m_state = StIdle;

    if (m_guiMessageQueue) {
        m_guiMessageQueue->push(Camera::MsgStartStop::create(false));
    }
    if (m_postProcessor) {
        m_postProcessor->getInputMessageQueue()->push(CameraPostProcessor::MsgCaptureActive::create(false));
    }
    if (m_worker) {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgStartStop::create(false));
    }
}

void Camera::setMessageQueueToGUI(MessageQueue *queue)
{
    Feature::setMessageQueueToGUI(queue);
    if (m_worker) {
        m_worker->setMessageQueueToGUI(queue);
    }
    if (m_postProcessor) {
        m_postProcessor->setMessageQueueToGUI(queue);
    }
    if (m_cameraFinder) {
        m_cameraFinder->setMessageQueueToGUI(queue);
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
        if (m_cameraFinder)
        {
            m_cameraFinder->setMessageQueueToGUI(getMessageQueueToGUI());
            m_cameraFinder->reportCameraList(m_settings);
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
    if (m_postProcessor) {
        m_postProcessor->getInputMessageQueue()->push(CameraPostProcessor::MsgConfigureCameraPostProcessor::create(settings, settingsKeys, force));
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if ((settingsKeys.contains("alpacaDiscoveryEnabled") || settingsKeys.contains("alpacaHost") || settingsKeys.contains("alpacaPort") || force) && m_cameraFinder) {
        m_cameraFinder->reportCameraList(m_settings);
    }
}
