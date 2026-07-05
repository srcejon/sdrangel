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

#include <memory>

#include <QCoreApplication>
#include <QDebug>
#include <QJsonObject>
#include <QList>

#include "SWGFeatureSettings.h"
#include "SWGFeatureReport.h"
#include "SWGFeatureActions.h"
#include "SWGDeviceState.h"
#include "SWGCameraSettings.h"
#include "SWGCameraRect.h"
#include "SWGCameraReport.h"
#include "SWGCameraActions.h"

#include "settings/serializable.h"
#include "util/messagequeue.h"

#include "camera.h"
#include "cameraworker.h"

MESSAGE_CLASS_DEFINITION(Camera::MsgConfigureCamera, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgProcessFrame, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgCaptureActive, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgRefreshCameraList, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgReportError, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgDeleteStackFrame, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgClearTrackedObjectHeatMap, Message)
MESSAGE_CLASS_DEFINITION(Camera::MsgSaveCurrentImage, Message)

const char* const Camera::m_featureIdURI = "sdrangel.feature.camera";
const char* const Camera::m_featureId = "Camera";

static int discardQueuedProcessFrames(MessageQueue& queue, bool requireCaptureActive)
{
    QList<Message*> messages;
    Message *message = nullptr;
    bool hasCaptureActive = false;

    while ((message = queue.pop()) != nullptr)
    {
        hasCaptureActive = hasCaptureActive || Camera::MsgCaptureActive::match(*message);
        messages.append(message);
    }

    int dropped = 0;

    for (Message *queuedMessage : messages)
    {
        if ((!requireCaptureActive || hasCaptureActive) && Camera::MsgProcessFrame::match(*queuedMessage))
        {
            delete queuedMessage;
            ++dropped;
        }
        else
        {
            queue.push(queuedMessage, false);
        }
    }

    return dropped;
}

int Camera::discardQueuedProcessFrames(MessageQueue& queue)
{
    return ::discardQueuedProcessFrames(queue, false);
}

int Camera::discardQueuedProcessFramesOnCaptureActive(MessageQueue& queue)
{
    return ::discardQueuedProcessFrames(queue, true);
}

bool Camera::acceptsPipelineFrame(const CameraPipelineFramePtr& frame, bool captureActive, quint64 captureEpoch)
{
    return frame
        && (frame->m_manualPreviewFrame || (captureActive && (frame->m_captureEpoch == captureEpoch)));
}

Camera::Camera(WebAPIAdapterInterface *webAPIAdapterInterface) :
    Feature(m_featureIdURI, webAPIAdapterInterface),
    m_workerThread(new QThread()),
    m_worker(new CameraWorker()),
    m_framePreprocessorThread(new QThread()),
    m_framePreprocessor(new CameraFramePreprocessor()),
    m_frameAlignerThread(new QThread()),
    m_frameAligner(new CameraFrameAligner()),
    m_frameStackerThread(new QThread()),
    m_frameStacker(new CameraFrameStacker()),
    m_imageProcessorThread(new QThread()),
    m_imageProcessor(new CameraImageProcessor()),
    m_motionDetectorThread(new QThread()),
    m_motionDetector(new CameraMotionDetector(this)),
    m_starDetectorThread(new QThread()),
    m_starDetector(new CameraStarDetector()),
    m_objectDetectorThread(new QThread()),
    m_objectDetector(new CameraObjectDetector(this)),
    m_diffDetectorThread(new QThread()),
    m_diffDetector(new CameraDiffDetector()),
    m_recorderThread(new QThread()),
    m_recorder(new CameraRecorder()),
    m_postProcessorThread(new QThread()),
    m_postProcessor(new CameraPostProcessor()),
    m_captureEpoch(0)
{
    setObjectName(m_featureId);
    m_state = StIdle;
    m_errorMessage = "Camera error";

    // The worker needs to run continuously, to be able to query camera capabilities.
    m_worker->moveToThread(m_workerThread);
    QObject::connect(m_workerThread, &QThread::started, m_worker, &CameraWorker::startWork);
    QObject::connect(m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);
    QObject::connect(m_workerThread, &QThread::finished, m_workerThread, &QThread::deleteLater);
    m_worker->setMessageQueueToGUI(getMessageQueueToGUI());
    m_worker->setMessageQueueToFeature(getInputMessageQueue());
    m_worker->setFramePreprocessor(getFramePreprocessor());
    m_worker->setPostProcessorInputMessageQueue(getPostProcessorInputMessageQueue());
    m_worker->setRecorderInputMessageQueue(getRecorderInputMessageQueue());
    m_workerThread->start();
    m_worker->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_framePreprocessor->moveToThread(m_framePreprocessorThread);
    QObject::connect(m_framePreprocessorThread, &QThread::started, m_framePreprocessor, &CameraFramePreprocessor::startWork);
    QObject::connect(m_framePreprocessorThread, &QThread::finished, m_framePreprocessor, &QObject::deleteLater);
    QObject::connect(m_framePreprocessorThread, &QThread::finished, m_framePreprocessorThread, &QThread::deleteLater);
    m_framePreprocessor->setNextStage(m_frameAligner);
    m_framePreprocessorThread->start();
    m_framePreprocessor->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_frameAligner->moveToThread(m_frameAlignerThread);
    QObject::connect(m_frameAlignerThread, &QThread::started, m_frameAligner, &CameraFrameAligner::startWork);
    QObject::connect(m_frameAlignerThread, &QThread::finished, m_frameAligner, &QObject::deleteLater);
    QObject::connect(m_frameAlignerThread, &QThread::finished, m_frameAlignerThread, &QThread::deleteLater);
    m_frameAligner->setNextStage(m_frameStacker);
    m_frameAlignerThread->start();
    m_frameAligner->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_frameStacker->moveToThread(m_frameStackerThread);
    QObject::connect(m_frameStackerThread, &QThread::started, m_frameStacker, &CameraFrameStacker::startWork);
    QObject::connect(m_frameStackerThread, &QThread::finished, m_frameStacker, &QObject::deleteLater);
    QObject::connect(m_frameStackerThread, &QThread::finished, m_frameStackerThread, &QThread::deleteLater);
    m_frameStacker->setNextStage(m_imageProcessor);
    m_frameStackerThread->start();
    m_frameStacker->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_imageProcessor->moveToThread(m_imageProcessorThread);
    QObject::connect(m_imageProcessorThread, &QThread::started, m_imageProcessor, &CameraImageProcessor::startWork);
    QObject::connect(m_imageProcessorThread, &QThread::finished, m_imageProcessor, &QObject::deleteLater);
    QObject::connect(m_imageProcessorThread, &QThread::finished, m_imageProcessorThread, &QThread::deleteLater);
    m_imageProcessor->setNextStage(m_motionDetector);
    m_imageProcessorThread->start();
    m_imageProcessor->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_motionDetector->moveToThread(m_motionDetectorThread);
    QObject::connect(m_motionDetectorThread, &QThread::started, m_motionDetector, &CameraMotionDetector::startWork);
    QObject::connect(m_motionDetectorThread, &QThread::finished, m_motionDetector, &QObject::deleteLater);
    QObject::connect(m_motionDetectorThread, &QThread::finished, m_motionDetectorThread, &QThread::deleteLater);
    m_motionDetector->setNextStage(m_starDetector);
    m_motionDetectorThread->start();
    m_motionDetector->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_starDetector->moveToThread(m_starDetectorThread);
    QObject::connect(m_starDetectorThread, &QThread::started, m_starDetector, &CameraStarDetector::startWork);
    QObject::connect(m_starDetectorThread, &QThread::finished, m_starDetector, &QObject::deleteLater);
    QObject::connect(m_starDetectorThread, &QThread::finished, m_starDetectorThread, &QThread::deleteLater);
    m_starDetector->setNextStage(m_objectDetector);
    m_starDetector->setCoalesceForwardedFrames(true);
    m_starDetector->setMessageQueueToGUI(getMessageQueueToGUI());
    m_starDetectorThread->start();
    m_starDetector->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_objectDetector->moveToThread(m_objectDetectorThread);
    QObject::connect(m_objectDetectorThread, &QThread::started, m_objectDetector, &CameraObjectDetector::startWork);
    QObject::connect(m_objectDetectorThread, &QThread::finished, m_objectDetector, &QObject::deleteLater);
    QObject::connect(m_objectDetectorThread, &QThread::finished, m_objectDetectorThread, &QThread::deleteLater);
    m_objectDetector->setNextStage(m_diffDetector);
    m_objectDetector->setMessageQueueToGUI(getMessageQueueToGUI());
    m_objectDetector->setMessageQueueToFeature(getInputMessageQueue());
    m_objectDetectorThread->start();
    m_objectDetector->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_diffDetector->moveToThread(m_diffDetectorThread);
    QObject::connect(m_diffDetectorThread, &QThread::started, m_diffDetector, &CameraDiffDetector::startWork);
    QObject::connect(m_diffDetectorThread, &QThread::finished, m_diffDetector, &QObject::deleteLater);
    QObject::connect(m_diffDetectorThread, &QThread::finished, m_diffDetectorThread, &QThread::deleteLater);
    m_diffDetector->setNextStageInputMessageQueue(getPostProcessorInputMessageQueue());
    m_diffDetectorThread->start();
    m_diffDetector->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    m_recorder->moveToThread(m_recorderThread);
    QObject::connect(m_recorderThread, &QThread::started, m_recorder, &CameraRecorder::startWork);
    QObject::connect(m_recorderThread, &QThread::finished, m_recorder, &QObject::deleteLater);
    QObject::connect(m_recorderThread, &QThread::finished, m_recorderThread, &QThread::deleteLater);
    m_recorder->setMessageQueueToGUI(getMessageQueueToGUI());
    m_recorder->setMessageQueueToFeature(getInputMessageQueue());
    m_recorderThread->start();
    m_recorder->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));

    // The post-processor runs continously, to be able to update the image when post processing settings are changed.
    m_postProcessor->moveToThread(m_postProcessorThread);
    QObject::connect(m_postProcessorThread, &QThread::started, m_postProcessor, &CameraPostProcessor::startWork);
    QObject::connect(m_postProcessorThread, &QThread::finished, m_postProcessor, &QObject::deleteLater);
    QObject::connect(m_postProcessorThread, &QThread::finished, m_postProcessorThread, &QThread::deleteLater);
    m_postProcessor->setMessageQueueToGUI(getMessageQueueToGUI());
    m_postProcessor->setNextStageInputMessageQueue(getRecorderInputMessageQueue());
    m_postProcessor->setWorkerInputMessageQueue(m_worker->getInputMessageQueue());
    m_postProcessorThread->start();
    m_postProcessor->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(m_settings, QList<QString>(), true));
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

    if (m_framePreprocessorThread)
    {
        m_framePreprocessorThread->quit();
        m_framePreprocessorThread->wait();
        m_framePreprocessorThread = nullptr;
        m_framePreprocessor = nullptr;
    }

    if (m_frameAlignerThread)
    {
        m_frameAlignerThread->quit();
        m_frameAlignerThread->wait();
        m_frameAlignerThread = nullptr;
        m_frameAligner = nullptr;
    }

    if (m_frameStackerThread)
    {
        m_frameStackerThread->quit();
        m_frameStackerThread->wait();
        m_frameStackerThread = nullptr;
        m_frameStacker = nullptr;
    }

    if (m_imageProcessorThread)
    {
        m_imageProcessorThread->quit();
        m_imageProcessorThread->wait();
        m_imageProcessorThread = nullptr;
        m_imageProcessor = nullptr;
    }

    if (m_motionDetectorThread)
    {
        m_motionDetectorThread->quit();
        m_motionDetectorThread->wait();
        m_motionDetectorThread = nullptr;
        m_motionDetector = nullptr;
    }

    if (m_starDetectorThread)
    {
        m_starDetectorThread->quit();
        m_starDetectorThread->wait();
        m_starDetectorThread = nullptr;
        m_starDetector = nullptr;
    }

    if (m_objectDetectorThread)
    {
        m_objectDetectorThread->quit();
        m_objectDetectorThread->wait();
        m_objectDetectorThread = nullptr;
        m_objectDetector = nullptr;
    }

    if (m_diffDetectorThread)
    {
        m_diffDetectorThread->quit();
        m_diffDetectorThread->wait();
        m_diffDetectorThread = nullptr;
        m_diffDetector = nullptr;
    }

    // Tear down stages in pipeline (producer-before-consumer) order so that no
    // running stage can push a frame into a downstream stage's input queue
    // after that stage (which owns the queue) has been destroyed. The
    // post-processor feeds the recorder's input queue, so it must be stopped
    // before the recorder is deleted.
    if (m_postProcessorThread)
    {
        m_postProcessorThread->quit();
        m_postProcessorThread->wait();
        m_postProcessorThread = nullptr;
        m_postProcessor = nullptr;
    }

    if (m_recorderThread)
    {
        m_recorderThread->quit();
        m_recorderThread->wait();
        m_recorderThread = nullptr;
        m_recorder = nullptr;
    }
}

void Camera::start()
{
    qDebug("Camera::start");
    if (m_state == StRunning) {
        return;
    }

    const quint64 captureEpoch = ++m_captureEpoch;
    m_state = StRunning;

    if (m_worker) {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgStartStop::create(true, captureEpoch));
    }
    if (m_framePreprocessor) {
        m_framePreprocessor->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_frameAligner) {
        m_frameAligner->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_frameStacker) {
        m_frameStacker->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_imageProcessor) {
        m_imageProcessor->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_motionDetector) {
        m_motionDetector->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_starDetector) {
        m_starDetector->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_objectDetector) {
        m_objectDetector->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_diffDetector) {
        m_diffDetector->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_recorder) {
        m_recorder->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_postProcessor) {
        m_postProcessor->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(true, captureEpoch));
    }
    if (m_guiMessageQueue) {
        m_guiMessageQueue->push(Camera::MsgStartStop::create(true, captureEpoch));
    }
}

void Camera::stop()
{
    qDebug("Camera::stop");
    if (m_state == StIdle) {
        return;
    }

    const quint64 captureEpoch = ++m_captureEpoch;
    m_state = StIdle;

    if (m_guiMessageQueue) {
        m_guiMessageQueue->push(Camera::MsgStartStop::create(false, captureEpoch));
    }
    if (m_postProcessor) {
        m_postProcessor->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_recorder) {
        m_recorder->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_diffDetector) {
        m_diffDetector->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_objectDetector) {
        m_objectDetector->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_starDetector) {
        m_starDetector->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_motionDetector) {
        m_motionDetector->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_imageProcessor) {
        m_imageProcessor->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_frameStacker) {
        m_frameStacker->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_frameAligner) {
        m_frameAligner->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_framePreprocessor) {
        m_framePreprocessor->getInputMessageQueue()->push(Camera::MsgCaptureActive::create(false, captureEpoch));
    }
    if (m_worker) {
        m_worker->getInputMessageQueue()->push(CameraWorker::MsgStartStop::create(false, captureEpoch));
    }
}

void Camera::setMessageQueueToGUI(MessageQueue *queue)
{
    Feature::setMessageQueueToGUI(queue);
    if (m_worker) {
        m_worker->setMessageQueueToGUI(queue);
        m_worker->setMessageQueueToFeature(getInputMessageQueue());
    }
    if (m_objectDetector) {
        m_objectDetector->setMessageQueueToGUI(queue);
        m_objectDetector->setMessageQueueToFeature(getInputMessageQueue());
    }
    if (m_starDetector) {
        m_starDetector->setMessageQueueToGUI(queue);
    }
    if (m_recorder) {
        m_recorder->setMessageQueueToGUI(queue);
        m_recorder->setMessageQueueToFeature(getInputMessageQueue());
    }
    if (m_postProcessor) {
        m_postProcessor->setMessageQueueToGUI(queue);
    }
}

void Camera::submitRecorderAudioSamples(const QByteArray& pcmS16Stereo, int sampleRate)
{
    if (m_recorder) {
        m_recorder->getInputMessageQueue()->push(CameraRecorder::MsgAudioSamples::create(pcmS16Stereo, sampleRate));
    }
}

void Camera::requestPreRecordPreview(qint64 offsetMs)
{
    if (m_recorder) {
        m_recorder->getInputMessageQueue()->push(CameraRecorder::MsgRequestPreRecordPreview::create(offsetMs));
    }
}

void Camera::submitWindowOverlayFrames(const QVector<CameraPostProcessor::WindowOverlayFrame>& frames)
{
    if (m_postProcessor) {
        m_postProcessor->getInputMessageQueue()->push(CameraPostProcessor::MsgWindowOverlayFrames::create(frames));
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
    else if (MsgProcessFrame::match(cmd))
    {
        const MsgProcessFrame& msg = (const MsgProcessFrame&) cmd;
        if (m_starDetector && msg.getFrame()) {
            m_starDetector->getInputMessageQueue()->push(Camera::MsgProcessFrame::create(msg.getFrame()));
        }
        return true;
    }
    else if (MsgRefreshCameraList::match(cmd))
    {
        const MsgRefreshCameraList& msg = (const MsgRefreshCameraList&) cmd;
        if (m_worker) {
            m_worker->getInputMessageQueue()->push(CameraWorker::MsgRefreshCameraList::create(msg.getForceRefresh()));
        }

        return true;
    }
    else if (MsgReportError::match(cmd))
    {
        const MsgReportError& report = (const MsgReportError&) cmd;
        m_errorMessage = report.getTitle().isEmpty()
            ? report.getErrorMessage()
            : QStringLiteral("%1\n\n%2").arg(report.getTitle(), report.getErrorMessage());
        m_state = StError;
        return true;
    }
    else if (MsgDeleteStackFrame::match(cmd))
    {
        const MsgDeleteStackFrame& msg = (const MsgDeleteStackFrame&) cmd;
        if (m_frameStacker) {
            m_frameStacker->getInputMessageQueue()->push(CameraFrameStacker::MsgDeleteStackFrame::create(msg.getFrameIndex()));
        }
        return true;
    }
    else if (MsgClearTrackedObjectHeatMap::match(cmd))
    {
        if (m_postProcessor) {
            m_postProcessor->getInputMessageQueue()->push(CameraPostProcessor::MsgClearTrackedObjectHeatMap::create());
        }
        return true;
    }
    else if (MsgSaveCurrentImage::match(cmd))
    {
        if (m_postProcessor) {
            m_postProcessor->getInputMessageQueue()->push(CameraPostProcessor::MsgSaveCurrentImage::create());
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

    const bool cancellingPlateSolve = (force && !settings.m_plateSolve)
        || (!force && settingsKeys.contains("plateSolve") && !settings.m_plateSolve)
        || (!force && settings.m_plateSolve && CameraStarDetector::plateSolveInputSettingsChanged(settingsKeys));

    if (m_starDetector && cancellingPlateSolve)
    {
        m_starDetector->requestPlateSolveCancellation();
    }

    if (m_worker) {
        m_worker->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_framePreprocessor) {
        m_framePreprocessor->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_frameAligner) {
        m_frameAligner->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_frameStacker) {
        m_frameStacker->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_imageProcessor) {
        m_imageProcessor->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_motionDetector) {
        m_motionDetector->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_starDetector) {
        m_starDetector->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_objectDetector) {
        m_objectDetector->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_diffDetector) {
        m_diffDetector->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_recorder) {
        m_recorder->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }
    if (m_postProcessor) {
        m_postProcessor->getInputMessageQueue()->push(Camera::MsgConfigureCamera::create(settings, settingsKeys, force));
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }
}

int Camera::webapiRun(bool run, SWGSDRangel::SWGDeviceState& response, QString& errorMessage)
{
    (void) errorMessage;
    getFeatureStateStr(*response.getState());
    MsgStartStop *msg = MsgStartStop::create(run);
    getInputMessageQueue()->push(msg);
    return 202;
}

int Camera::webapiSettingsGet(SWGSDRangel::SWGFeatureSettings& response, QString& errorMessage)
{
    (void) errorMessage;
    response.setCameraSettings(new SWGSDRangel::SWGCameraSettings());
    response.getCameraSettings()->init();
    webapiFormatFeatureSettings(response, m_settings);
    return 200;
}

int Camera::webapiSettingsPutPatch(
        bool force,
        const QStringList& featureSettingsKeys,
        SWGSDRangel::SWGFeatureSettings& response,
        QString& errorMessage)
{
    (void) errorMessage;
    CameraSettings settings = m_settings;
    webapiUpdateFeatureSettings(settings, featureSettingsKeys, response);

    MsgConfigureCamera *msg = MsgConfigureCamera::create(settings, featureSettingsKeys, force);
    m_inputMessageQueue.push(msg);

    if (m_guiMessageQueue)
    {
        MsgConfigureCamera *msgToGUI = MsgConfigureCamera::create(settings, featureSettingsKeys, force);
        m_guiMessageQueue->push(msgToGUI);
    }

    webapiFormatFeatureSettings(response, settings);
    return 200;
}

int Camera::webapiReportGet(SWGSDRangel::SWGFeatureReport& response, QString& errorMessage)
{
    (void) errorMessage;
    response.setCameraReport(new SWGSDRangel::SWGCameraReport());
    response.getCameraReport()->init();
    webapiFormatFeatureReport(response);
    return 200;
}

void Camera::webapiFormatFeatureReport(SWGSDRangel::SWGFeatureReport& response)
{
    response.getCameraReport()->setRunningState(getState());
}

int Camera::webapiActionsPost(
        const QStringList& featureActionsKeys,
        SWGSDRangel::SWGFeatureActions& query,
        QString& errorMessage)
{
    SWGSDRangel::SWGCameraActions *swgCameraActions = query.getCameraActions();

    if (swgCameraActions)
    {
        bool accepted = false;
        bool startCamera = false;
        CameraSettings settings = m_settings;
        QList<QString> settingsKeys;
        auto addSettingsKey = [&settingsKeys](const QString& key)
        {
            if (!settingsKeys.contains(key)) {
                settingsKeys.append(key);
            }
        };

        if (featureActionsKeys.contains("run"))
        {
            bool featureRun = swgCameraActions->getRun() != 0;
            MsgStartStop *msg = MsgStartStop::create(featureRun);
            getInputMessageQueue()->push(msg);
            accepted = true;
        }

        if (featureActionsKeys.contains("saveImage"))
        {
            SWGSDRangel::SWGCameraActions_saveImage *saveImage = swgCameraActions->getSaveImage();

            if (!saveImage)
            {
                errorMessage = "Missing saveImage action";
                return 400;
            }

            // Determine which fields the caller actually supplied. Without this gate, a
            // request like {"saveImage":{}} would clobber the user's saved recording
            // output selection with the SWG defaults (0).
            // asJsonObject() emits a key only when the corresponding *_isSet flag is true.
            std::unique_ptr<QJsonObject> saveImageJson(saveImage->asJsonObject());
            const bool hasRecordRawFits = saveImageJson && saveImageJson->contains("recordRawFits");
            const bool hasRecordCalibratedMedia = saveImageJson && saveImageJson->contains("recordCalibratedMedia");
            const bool hasRecordFilteredMedia = saveImageJson && saveImageJson->contains("recordFilteredMedia");
            const bool hasRecordPostProcessedMedia = saveImageJson && saveImageJson->contains("recordPostProcessedMedia");
            const bool hasImages = saveImageJson && saveImageJson->contains("images");

            if (saveImage->getFilename() && !saveImage->getFilename()->isEmpty())
            {
                settings.m_imageFileName = *saveImage->getFilename();
                addSettingsKey("imageFileName");
            }

            if (hasRecordRawFits) {
                settings.m_recordRawFits = saveImage->getRecordRawFits() != 0;
                addSettingsKey("recordRawFits");
            }
            if (hasRecordCalibratedMedia) {
                settings.m_recordCalibratedMedia = saveImage->getRecordCalibratedMedia() != 0;
                addSettingsKey("recordCalibratedMedia");
            }
            if (hasRecordFilteredMedia) {
                settings.m_recordFilteredMedia = saveImage->getRecordFilteredMedia() != 0;
                addSettingsKey("recordFilteredMedia");
            }
            if (hasRecordPostProcessedMedia) {
                settings.m_recordPostProcessedMedia = saveImage->getRecordPostProcessedMedia() != 0;
                addSettingsKey("recordPostProcessedMedia");
            }
            if (hasImages)
            {
                const int imageCount = qMax(CameraSettings::m_minNonNegative, saveImage->getImages());
                if (imageCount != 1)
                {
                    settings.m_imageRecordLimit = imageCount > 1 ? imageCount - 1 : 0;
                    addSettingsKey("imageRecordLimit");
                    settings.m_saveImage = true;
                    addSettingsKey("saveImage");
                    startCamera = m_state == StIdle;
                }
            }
            accepted = true;
        }

        if (featureActionsKeys.contains("recordVideo"))
        {
            SWGSDRangel::SWGCameraActions_recordVideo *recordVideo = swgCameraActions->getRecordVideo();

            if (!recordVideo)
            {
                errorMessage = "Missing recordVideo action";
                return 400;
            }

            // Same isSet-via-JSON pattern as saveImage above — see the comment there.
            std::unique_ptr<QJsonObject> recordVideoJson(recordVideo->asJsonObject());
            const bool hasRecordCalibratedMedia = recordVideoJson && recordVideoJson->contains("recordCalibratedMedia");
            const bool hasRecordFilteredMedia = recordVideoJson && recordVideoJson->contains("recordFilteredMedia");
            const bool hasRecordPostProcessedMedia = recordVideoJson && recordVideoJson->contains("recordPostProcessedMedia");
            const bool hasDuration = recordVideoJson && recordVideoJson->contains("duration");

            if (recordVideo->getFilename() && !recordVideo->getFilename()->isEmpty())
            {
                settings.m_videoFileName = *recordVideo->getFilename();
                addSettingsKey("videoFileName");
            }

            if (hasRecordCalibratedMedia) {
                settings.m_recordCalibratedMedia = recordVideo->getRecordCalibratedMedia() != 0;
                addSettingsKey("recordCalibratedMedia");
            }
            if (hasRecordFilteredMedia) {
                settings.m_recordFilteredMedia = recordVideo->getRecordFilteredMedia() != 0;
                addSettingsKey("recordFilteredMedia");
            }
            if (hasRecordPostProcessedMedia) {
                settings.m_recordPostProcessedMedia = recordVideo->getRecordPostProcessedMedia() != 0;
                addSettingsKey("recordPostProcessedMedia");
            }
            if (hasDuration)
            {
                settings.m_videoRecordLimitSeconds = qMax(CameraSettings::m_minNonNegative, recordVideo->getDuration());
                addSettingsKey("videoRecordLimitSeconds");
            }
            settings.m_saveVideo = true;
            addSettingsKey("saveVideo");
            startCamera = m_state == StIdle;
            accepted = true;
        }

        if (!accepted)
        {
            errorMessage = "Unknown action";
            return 400;
        }

        if (!settingsKeys.isEmpty())
        {
            MsgConfigureCamera *msg = MsgConfigureCamera::create(settings, settingsKeys, false);
            m_inputMessageQueue.push(msg);

            if (m_guiMessageQueue)
            {
                MsgConfigureCamera *msgToGUI = MsgConfigureCamera::create(settings, settingsKeys, false);
                m_guiMessageQueue->push(msgToGUI);
            }
        }

        if (featureActionsKeys.contains("saveImage"))
        {
            MsgSaveCurrentImage *msg = MsgSaveCurrentImage::create();
            getInputMessageQueue()->push(msg);
        }

        if (startCamera && !featureActionsKeys.contains("run"))
        {
            MsgStartStop *msg = MsgStartStop::create(true);
            getInputMessageQueue()->push(msg);
        }

        return 202;
    }
    else
    {
        errorMessage = "Missing CameraActions in query";
        return 400;
    }
}

void Camera::webapiFormatFeatureSettings(
        SWGSDRangel::SWGFeatureSettings& response,
        const CameraSettings& settings)
{
    SWGSDRangel::SWGCameraSettings *swg = response.getCameraSettings();

    // Basic
    if (swg->getTitle()) {
        *swg->getTitle() = settings.m_title;
    } else {
        swg->setTitle(new QString(settings.m_title));
    }
    swg->setRgbColor(settings.m_rgbColor);

    // Camera selection
    swg->setCameraProtocol(new QString(settings.m_cameraProtocol));
    swg->setCameraId(new QString(settings.m_cameraId));
    swg->setCameraDescription(new QString(settings.m_cameraDescription));
    swg->setResolutionWidth(settings.m_resolutionWidth);
    swg->setResolutionHeight(settings.m_resolutionHeight);
    swg->setFramesPerSecond(settings.m_framesPerSecond);
    swg->setCaptureMode((int) settings.m_captureMode);
    swg->setCaptureInterval(settings.m_captureInterval);
    swg->setCaptureIntervalUnits((int) settings.m_captureIntervalUnits);
    swg->setExposureTimeMs(settings.m_exposureTimeMs);
    swg->setIsoSensitivity(settings.m_isoSensitivity);

    // ALPACA
    swg->setAlpacaDiscoveryEnabled(settings.m_alpacaDiscoveryEnabled ? 1 : 0);
    swg->setAlpacaApiLogEnabled(settings.m_alpacaApiLogEnabled ? 1 : 0);
    swg->setAlpacaHost(new QString(settings.m_alpacaHost));
    swg->setAlpacaPort(settings.m_alpacaPort);
    swg->setAlpacaFocuserEnabled(settings.m_alpacaFocuserEnabled ? 1 : 0);
    swg->setAlpacaFocuserHost(new QString(settings.m_alpacaFocuserHost));
    swg->setAlpacaFocuserPort(settings.m_alpacaFocuserPort);
    swg->setAlpacaFocuserDeviceNumber(settings.m_alpacaFocuserDeviceNumber);
    swg->setAlpacaFocusPosition(settings.m_alpacaFocusPosition);
    swg->setAlpacaFocusStepSize(settings.m_alpacaFocusStepSize);
    swg->setAlpacaFilterWheelEnabled(settings.m_alpacaFilterWheelEnabled ? 1 : 0);
    swg->setAlpacaFilterWheelHost(new QString(settings.m_alpacaFilterWheelHost));
    swg->setAlpacaFilterWheelPort(settings.m_alpacaFilterWheelPort);
    swg->setAlpacaFilterWheelDeviceNumber(settings.m_alpacaFilterWheelDeviceNumber);
    swg->setAlpacaFilterWheelPosition(settings.m_alpacaFilterWheelPosition);
    swg->setCameraBinX(settings.m_cameraBinX);
    swg->setCameraBinY(settings.m_cameraBinY);
    swg->setCameraNumX(settings.m_cameraNumX);
    swg->setCameraNumY(settings.m_cameraNumY);
    swg->setCameraStartX(settings.m_cameraStartX);
    swg->setCameraStartY(settings.m_cameraStartY);
    swg->setCameraGain(settings.m_cameraGain);
    swg->setCameraOffset(settings.m_cameraOffset);
    swg->setCameraReadoutMode(settings.m_cameraReadoutMode);
    swg->setAsiCoolerOn(settings.m_asiCoolerOn);
    swg->setAsiTargetTemp(settings.m_asiTargetTemp);
    swg->setAsiUsbBandwidth(settings.m_asiUsbBandwidth);
    swg->setAsiHighSpeedMode(settings.m_asiHighSpeedMode);
    swg->setAsiAutoExposureGain(settings.m_asiAutoExposureGain ? 1 : 0);
    swg->setAutoExposureGainEnabled(settings.m_autoExposureGainEnabled ? 1 : 0);
    swg->setAutoExposureGainMode((int) settings.m_autoExposureGainMode);
    swg->setAutoExposureTargetPercentile(settings.m_autoExposureTargetPercentile);
    swg->setAutoExposureTargetBrightness(settings.m_autoExposureTargetBrightness);
    swg->setAutoExposureMaxChangePercent(settings.m_autoExposureMaxChangePercent);
    swg->setAutoExposureMinMs(settings.m_autoExposureMinMs);
    swg->setAutoExposureMaxMs(settings.m_autoExposureMaxMs);
    swg->setAutoExposureMinGain(settings.m_autoExposureMinGain);
    swg->setAutoExposureMaxGain(settings.m_autoExposureMaxGain);
    swg->setAsiColorImageType((int) settings.m_asiColorImageType);

    // Image / video output
    swg->setSaveImage(settings.m_saveImage ? 1 : 0);
    swg->setImageFileName(new QString(settings.m_imageFileName));
    swg->setSaveVideo(settings.m_saveVideo ? 1 : 0);
    swg->setVideoFileCameraPath(new QString(settings.m_videoFileCameraPath));
    swg->setStreamUrl(new QString(settings.m_streamUrl));
    auto *streamUrlHistory = new QList<QString*>();
    for (const QString& url : settings.m_streamUrlHistory) {
        streamUrlHistory->append(new QString(url));
    }
    swg->setStreamUrlHistory(streamUrlHistory);
    auto *imageFileCameraPaths = new QList<QString*>();
    for (const QString& path : settings.m_imageFileCameraPaths) {
        imageFileCameraPaths->append(new QString(path));
    }
    swg->setImageFileCameraPaths(imageFileCameraPaths);
    swg->setVideoFileName(new QString(settings.m_videoFileName));
    swg->setVideoCodec((int) settings.m_videoCodec);
    swg->setVideoRecordBitrateKbps(settings.m_videoRecordBitrateKbps);
    swg->setRecordRawFits(settings.m_recordRawFits ? 1 : 0);
    swg->setRecordCalibratedMedia(settings.m_recordCalibratedMedia ? 1 : 0);
    swg->setRecordFilteredMedia(settings.m_recordFilteredMedia ? 1 : 0);
    swg->setRecordPostProcessedMedia(settings.m_recordPostProcessedMedia ? 1 : 0);
    swg->setKeogramEnabled(settings.m_keogramEnabled ? 1 : 0);
    swg->setKeogramFileName(new QString(settings.m_keogramFileName));
    swg->setKeogramDirection((int) settings.m_keogramDirection);
    swg->setKeogramDayMode((int) settings.m_keogramDayMode);
    swg->setKeogramSamplePeriodMinutes(settings.m_keogramSamplePeriodMinutes);
    swg->setKeogramShowPreview(settings.m_keogramShowPreview ? 1 : 0);
    swg->setYoutubeStreamEnabled(settings.m_youtubeStreamEnabled ? 1 : 0);
    swg->setYoutubeStreamUrl(new QString(settings.m_youtubeStreamUrl));
    swg->setYoutubeStreamKey(new QString(settings.m_youtubeStreamKey));
    swg->setYoutubeStreamPostProcessed(settings.m_youtubeStreamPostProcessed ? 1 : 0);
    swg->setYoutubeStreamBitrateKbps(settings.m_youtubeStreamBitrateKbps);
    swg->setYoutubeStreamFps(settings.m_youtubeStreamFps);
    swg->setYoutubeStreamWidth(settings.m_youtubeStreamWidth);
    swg->setYoutubeStreamHeight(settings.m_youtubeStreamHeight);
    swg->setVideoHwAcceleration(settings.m_videoHwAcceleration ? 1 : 0);
    swg->setVideoLoop(settings.m_videoLoop ? 1 : 0);
    swg->setVideoPlaybackRate(settings.m_videoPlaybackRate);
    swg->setVideoPlaybackAudioOffsetMs(settings.m_videoPlaybackAudioOffsetMs);
    swg->setVideoPreRecordBufferSeconds(settings.m_videoPreRecordBufferSeconds);
    swg->setImageRecordLimit(settings.m_imageRecordLimit);
    swg->setVideoRecordLimitSeconds(settings.m_videoRecordLimitSeconds);

    // Frame stacking / calibration
    swg->setStackEnabled(settings.m_stackEnabled ? 1 : 0);
    swg->setStackFrameCount(settings.m_stackFrameCount);
    swg->setStackMethod((int) settings.m_stackMethod);
    swg->setStackHdrAlgorithm((int) settings.m_stackHdrAlgorithm);
    swg->setStackHdrExposureCount(settings.m_stackHdrExposureCount);
    auto *swgHdrExposures = new QList<double>();
    for (double exposureTimeMs : settings.m_stackHdrExposureTimesMs) {
        swgHdrExposures->append(exposureTimeMs);
    }
    swg->setStackHdrExposureTimesMs(swgHdrExposures);
    swg->setStackAlignmentMethod((int) settings.m_stackAlignmentMethod);
    swg->setStackDisplayMode((int) settings.m_stackDisplayMode);
    swg->setStackDisplayFrameIndex(settings.m_stackDisplayFrameIndex);
    swg->setStackRejectBadFrames(settings.m_stackRejectBadFrames ? 1 : 0);
    swg->setStackDarkFileName(new QString(settings.m_stackDarkFileName));
    swg->setStackFlatFileName(new QString(settings.m_stackFlatFileName));
    swg->setStackBiasFileName(new QString(settings.m_stackBiasFileName));

    // Sky location / lens model
    swg->setLatitude(settings.m_latitude);
    swg->setLongitude(settings.m_longitude);
    swg->setAltitude(settings.m_altitude);
    swg->setPositionSync(settings.m_positionSync ? 1 : 0);
    swg->setOwmApiKey(new QString(settings.m_owmAPIKey));
    swg->setAzimuth(settings.m_azimuth);
    swg->setElevation(settings.m_elevation);
    swg->setRoll(settings.m_roll);
    swg->setRotator(new QString(settings.m_rotator));
    swg->setFov(settings.m_fov);
    swg->setFovMode((int) settings.m_fovMode);
    swg->setFovSensorWidthMm(settings.m_fovSensorWidthMm);
    swg->setFovSensorHeightMm(settings.m_fovSensorHeightMm);
    swg->setFovFocalLengthMm(settings.m_fovFocalLengthMm);
    swg->setLensProjection((int) settings.m_lensProjection);
    swg->setLensCenterOffsetX(settings.m_lensCenterOffsetX);
    swg->setLensCenterOffsetY(settings.m_lensCenterOffsetY);
    swg->setLensDistortionK1(settings.m_lensDistortionK1);

    // Post-processing – tone / colour
    swg->setPostProcessWhiteBalanceMode(settings.m_postProcessWhiteBalanceMode);
    swg->setPostProcessWhiteBalanceRedGain(settings.m_postProcessWhiteBalanceRedGain);
    swg->setPostProcessWhiteBalanceGreenGain(settings.m_postProcessWhiteBalanceGreenGain);
    swg->setPostProcessWhiteBalanceBlueGain(settings.m_postProcessWhiteBalanceBlueGain);
    swg->setPostProcessWhiteBalanceHighlightProtection(settings.m_postProcessWhiteBalanceHighlightProtection);
    swg->setPostProcessUseCuda(settings.m_postProcessUseCuda ? 1 : 0);
    swg->setPostProcessUnwarp(settings.m_postProcessUnwarp ? 1 : 0);
    swg->setHistogramStretch((int) settings.m_histogramStretch);
    swg->setHistogramStretchBlackPoint(settings.m_histogramStretchBlackPoint);
    swg->setHistogramStretchWhitePoint(settings.m_histogramStretchWhitePoint);
    swg->setHistogramStretchGamma(settings.m_histogramStretchGamma);
    swg->setHistogramStretchAsinhStrength(settings.m_histogramStretchAsinhStrength);
    swg->setHistogramStretchLogStrength(settings.m_histogramStretchLogStrength);
    swg->setPostProcessGreyscale(settings.m_postProcessGreyscale ? 1 : 0);
    swg->setSaturation(settings.m_saturation);
    swg->setGamma(settings.m_gamma);
    swg->setBrightness(settings.m_brightness);
    swg->setContrast(settings.m_contrast);
    swg->setGaussianBlur(settings.m_gaussianBlur);
    swg->setMedianBlur(settings.m_medianBlur);
    swg->setSharpen(settings.m_sharpen);
    swg->setEdgeDisplayMode((int) settings.m_edgeDisplayMode);
    swg->setSobelEdge(settings.m_sobelEdge);
    swg->setCannyEdge(settings.m_cannyEdge);
    swg->setFlipX(settings.m_flipX ? 1 : 0);
    swg->setFlipY(settings.m_flipY ? 1 : 0);
    swg->setImageRotation(settings.m_imageRotation);
    swg->setInvertColors(settings.m_invertColors ? 1 : 0);

    // Date/time overlay
    swg->setOverlayDateTime(settings.m_overlayDateTime ? 1 : 0);
    swg->setDateTimeColor((qint32) settings.m_dateTimeColor.rgb());
    swg->setDateTimeFormat(new QString(settings.m_dateTimeFormat));
    swg->setDateTimePosX(settings.m_dateTimePosX);
    swg->setDateTimePosY(settings.m_dateTimePosY);

    // Text overlay
    swg->setOverlayText(settings.m_overlayText ? 1 : 0);
    swg->setEquatorialGrid(settings.m_equatorialGrid ? 1 : 0);
    swg->setEquatorialGridColor((qint32) settings.m_equatorialGridColor.rgb());
    swg->setAltAzGrid(settings.m_altAzGrid ? 1 : 0);
    swg->setAltAzGridColor((qint32) settings.m_altAzGridColor.rgb());
    swg->setConstellation(settings.m_constellation ? 1 : 0);
    swg->setConstellationColor((qint32) settings.m_constellationColor.rgb());
    swg->setConstellationOverlay((int) settings.m_constellationOverlay);
    swg->setTrackObjects(settings.m_trackObjects ? 1 : 0);
    swg->setTrackObjectMinElevation(settings.m_trackObjectMinElevation);
    swg->setTrackObjectColor((qint32) settings.m_trackObjectColor.rgb());
    swg->setTrackObjectFontScale(settings.m_trackObjectFontScale);
    swg->setTrackObjectTrails(settings.m_trackObjectTrails ? 1 : 0);
    swg->setTrackObjectHeatMap(settings.m_trackObjectHeatMap ? 1 : 0);
    swg->setGridLabelFontFamily(new QString(settings.m_gridLabelFontFamily));
    swg->setGridLabelFontScale(settings.m_gridLabelFontScale);
    swg->setOverlayTextString(new QString(settings.m_overlayTextString));
    swg->setOverlayTextColor((qint32) settings.m_overlayTextColor.rgb());
    swg->setOverlayTextFontFamily(new QString(settings.m_overlayTextFontFamily));
    swg->setOverlayTextFontScale(settings.m_overlayTextFontScale);
    swg->setOverlayTextPosX(settings.m_overlayTextPosX);
    swg->setOverlayTextPosY(settings.m_overlayTextPosY);

    // Diff-mask
    swg->setDiffMask(settings.m_diffMask ? 1 : 0);
    swg->setDiffThreshold(settings.m_diffThreshold);
    swg->setDiffMaskOpenSize(settings.m_diffMaskOpenSize);
    swg->setDilationSize(settings.m_dilationSize);
    swg->setDiffMaskHistoryFrames(settings.m_diffMaskHistoryFrames);
    swg->setDiffMaskCloseSize(settings.m_diffMaskCloseSize);
    swg->setOverlayFontFamily(new QString(settings.m_overlayFontFamily));
    swg->setOverlayFontScale(settings.m_overlayFontScale);

    // Detection ROI
    swg->setDetectionRoiX(settings.m_detectionRoiX);
    swg->setDetectionRoiY(settings.m_detectionRoiY);
    swg->setDetectionRoiWidth(settings.m_detectionRoiWidth);
    swg->setDetectionRoiHeight(settings.m_detectionRoiHeight);
    swg->setShowDetectionRoi(settings.m_showDetectionRoi ? 1 : 0);

    // Motion detection
    swg->setMotionDetect(settings.m_motionDetect ? 1 : 0);
    swg->setMotionBackgroundSubtractor((int) settings.m_motionBackgroundSubtractor);
    swg->setMotionMaskView((int) settings.m_motionMaskView);
    swg->setMotionHistory(settings.m_motionHistory);
    swg->setMotionVarThreshold(settings.m_motionVarThreshold);
    swg->setMotionLearningRate(settings.m_motionLearningRate);
    swg->setMotionConfirmFrames(settings.m_motionConfirmFrames);
    swg->setMotionDownscale(settings.m_motionDownscale);
    swg->setMotionDetectShadows(settings.m_motionDetectShadows ? 1 : 0);
    swg->setMotionOpenSize(settings.m_motionOpenSize);
    swg->setMotionCloseSize(settings.m_motionCloseSize);
    swg->setMotionPersistenceFrames(settings.m_motionPersistenceFrames);
    swg->setMotionBoxColor((qint32) settings.m_motionBoxColor.rgb());
    swg->setMinContourArea(settings.m_minContourArea);
    auto *swgMotionExclusionRects = new QList<SWGSDRangel::SWGCameraRect*>();
    for (const QRect& rect : settings.m_motionExclusionRects)
    {
        auto *swgRect = new SWGSDRangel::SWGCameraRect();
        swgRect->init();
        swgRect->setX(rect.x());
        swgRect->setY(rect.y());
        swgRect->setWidth(rect.width());
        swgRect->setHeight(rect.height());
        swgMotionExclusionRects->append(swgRect);
    }
    swg->setMotionExclusionRects(swgMotionExclusionRects);
    swg->setStarDetect(settings.m_starDetect ? 1 : 0);
    swg->setStarThreshold(settings.m_starThreshold);
    swg->setStarBackgroundBlur(settings.m_starBackgroundBlur);
    swg->setStarMinArea(settings.m_starMinArea);
    swg->setStarMaxArea(settings.m_starMaxArea);
    swg->setStarMaxAspectRatio(settings.m_starMaxAspectRatio);
    swg->setStarDebugView((int) settings.m_starDebugView);
    swg->setStarColor((qint32) settings.m_starColor.rgb());
    swg->setPlateSolve(settings.m_plateSolve ? 1 : 0);
    swg->setPlateSolveMaxMagnitude(settings.m_plateSolveMaxMagnitude);
    swg->setPlateSolveMinMatches(settings.m_plateSolveMinMatches);
    swg->setPlateSolveMatchRadius(settings.m_plateSolveMatchRadius);
    swg->setPlateSolveFinalMatchRadius(settings.m_plateSolveFinalMatchRadius);
    swg->setPlateSolveSearchRadius(settings.m_plateSolveAzElSearchRadius);
    swg->setPlateSolveFovTolerance(settings.m_plateSolveFovTolerance);
    swg->setPlateSolveStartMode((int) settings.m_plateSolveStartMode);
    swg->setPlateSolveLabelMode((int) settings.m_plateSolveLabelMode);
    swg->setPlateSolveUseCaptureDateTime(settings.m_plateSolveUseCaptureDateTime ? 1 : 0);
    swg->setPlateSolveDateTime(new QString(settings.m_plateSolveDateTime.toString(Qt::ISODateWithMs)));
    swg->setPlateSolveDateTimeUtc(settings.m_plateSolveDateTimeUtc ? 1 : 0);
    swg->setPlateSolveUseDownloadedCatalog(settings.m_plateSolveUseDownloadedCatalog ? 1 : 0);
    swg->setPlateSolveCatalogSource((int) settings.m_plateSolveCatalogSource);
    swg->setPlateSolveApplyMode((int) settings.m_plateSolveApplyMode);
    swg->setStarCatalogDiskCacheSizeGb(settings.m_starCatalogDiskCacheSizeGb);

    // Spectrum overlay
    swg->setOverlaySpectrum(settings.m_overlaySpectrum ? 1 : 0);
    swg->setSpectrumDevice(new QString(settings.m_spectrumDevice));
    swg->setSpectrumOffsetX(settings.m_spectrumOffsetX);
    swg->setSpectrumOffsetY(settings.m_spectrumOffsetY);
    swg->setSpectrumScale(settings.m_spectrumScale);

    // YOLO
    swg->setYoloEnabled(settings.m_yoloEnabled ? 1 : 0);
    swg->setYoloModelPath(new QString(settings.m_yoloModelPath));
    swg->setYoloLabelsPath(new QString(settings.m_yoloLabelsPath));
    swg->setYoloConfThreshold(settings.m_yoloConfThreshold);
    swg->setYoloNmsThreshold(settings.m_yoloNmsThreshold);
    swg->setYoloBoxColor((qint32) settings.m_yoloBoxColor.rgb());
    swg->setYoloTileLargeImages(settings.m_yoloTileLargeImages ? 1 : 0);
    swg->setYoloTileOverlapPercent(settings.m_yoloTileOverlapPercent);
    auto *yoloIgnoredClassNames = new QList<QString*>();
    for (const QString& className : settings.m_yoloIgnoredClassNames) {
        yoloIgnoredClassNames->append(new QString(className));
    }
    swg->setYoloIgnoredClassNames(yoloIgnoredClassNames);
    swg->setYoloDnnTarget((int) settings.m_yoloDnnTarget);

    // Audio (Qt camera)
    swg->setAudioMute(settings.m_audioMute ? 1 : 0);
    swg->setAudioDeviceName(new QString(settings.m_audioDeviceName));

    // Qt camera controls
    swg->setWhiteBalanceMode(settings.m_whiteBalanceMode);
    swg->setExposureCompensation(settings.m_exposureCompensation);
    swg->setFocusMode(settings.m_focusMode);
    swg->setFocusDistance(settings.m_focusDistance);
    swg->setZoomFactor(settings.m_zoomFactor);

    // Reverse API
    swg->setUseReverseApi(settings.m_useReverseAPI ? 1 : 0);
    if (swg->getReverseApiAddress()) {
        *swg->getReverseApiAddress() = settings.m_reverseAPIAddress;
    } else {
        swg->setReverseApiAddress(new QString(settings.m_reverseAPIAddress));
    }
    swg->setReverseApiPort(settings.m_reverseAPIPort);
    swg->setReverseApiFeatureSetIndex(settings.m_reverseAPIFeatureSetIndex);
    swg->setReverseApiFeatureIndex(settings.m_reverseAPIFeatureIndex);

    // Rollup state
    if (settings.m_rollupState)
    {
        if (swg->getRollupState()) {
            settings.m_rollupState->formatTo(swg->getRollupState());
        } else {
            SWGSDRangel::SWGRollupState *swgRollupState = new SWGSDRangel::SWGRollupState();
            settings.m_rollupState->formatTo(swgRollupState);
            swg->setRollupState(swgRollupState);
        }
    }

    swg->setWorkspaceIndex(settings.m_workspaceIndex);
}

void Camera::webapiUpdateFeatureSettings(
        CameraSettings& settings,
        const QStringList& featureSettingsKeys,
        SWGSDRangel::SWGFeatureSettings& response)
{
    SWGSDRangel::SWGCameraSettings *swg = response.getCameraSettings();

    // Basic
    if (featureSettingsKeys.contains("title")) {
        settings.m_title = *swg->getTitle();
    }
    if (featureSettingsKeys.contains("rgbColor")) {
        settings.m_rgbColor = swg->getRgbColor();
    }

    // Camera selection
    if (featureSettingsKeys.contains("cameraProtocol")) {
        settings.m_cameraProtocol = *swg->getCameraProtocol();
    }
    if (featureSettingsKeys.contains("cameraId")) {
        settings.m_cameraId = *swg->getCameraId();
    }
    if (featureSettingsKeys.contains("cameraDescription")) {
        settings.m_cameraDescription = *swg->getCameraDescription();
    }
    if (featureSettingsKeys.contains("resolutionWidth")) {
        settings.m_resolutionWidth = swg->getResolutionWidth();
    }
    if (featureSettingsKeys.contains("resolutionHeight")) {
        settings.m_resolutionHeight = swg->getResolutionHeight();
    }
    if (featureSettingsKeys.contains("framesPerSecond")) {
        settings.m_framesPerSecond = swg->getFramesPerSecond();
    }
    if (featureSettingsKeys.contains("captureMode")) {
        settings.m_captureMode = (CameraSettings::CaptureMode) swg->getCaptureMode();
    }
    if (featureSettingsKeys.contains("captureInterval")) {
        settings.m_captureInterval = swg->getCaptureInterval();
    }
    if (featureSettingsKeys.contains("captureIntervalUnits")) {
        settings.m_captureIntervalUnits = (CameraSettings::CaptureIntervalUnits) swg->getCaptureIntervalUnits();
    }
    if (featureSettingsKeys.contains("exposureTimeMs")) {
        settings.m_exposureTimeMs = swg->getExposureTimeMs();
    }
    if (featureSettingsKeys.contains("isoSensitivity")) {
        settings.m_isoSensitivity = swg->getIsoSensitivity();
    }

    // ALPACA
    if (featureSettingsKeys.contains("alpacaDiscoveryEnabled")) {
        settings.m_alpacaDiscoveryEnabled = swg->getAlpacaDiscoveryEnabled() != 0;
    }
    if (featureSettingsKeys.contains("alpacaApiLogEnabled")) {
        settings.m_alpacaApiLogEnabled = swg->getAlpacaApiLogEnabled() != 0;
    }
    if (featureSettingsKeys.contains("alpacaHost")) {
        settings.m_alpacaHost = *swg->getAlpacaHost();
    }
    if (featureSettingsKeys.contains("alpacaPort")) {
        settings.m_alpacaPort = (uint16_t) swg->getAlpacaPort();
    }
    if (featureSettingsKeys.contains("alpacaFocuserEnabled")) {
        settings.m_alpacaFocuserEnabled = swg->getAlpacaFocuserEnabled() != 0;
    }
    if (featureSettingsKeys.contains("alpacaFocuserHost")) {
        settings.m_alpacaFocuserHost = *swg->getAlpacaFocuserHost();
    }
    if (featureSettingsKeys.contains("alpacaFocuserPort")) {
        settings.m_alpacaFocuserPort = (uint16_t) swg->getAlpacaFocuserPort();
    }
    if (featureSettingsKeys.contains("alpacaFocuserDeviceNumber")) {
        settings.m_alpacaFocuserDeviceNumber = swg->getAlpacaFocuserDeviceNumber();
    }
    if (featureSettingsKeys.contains("alpacaFocusPosition")) {
        settings.m_alpacaFocusPosition = swg->getAlpacaFocusPosition();
    }
    if (featureSettingsKeys.contains("alpacaFocusStepSize")) {
        settings.m_alpacaFocusStepSize = swg->getAlpacaFocusStepSize();
    }
    if (featureSettingsKeys.contains("alpacaFilterWheelEnabled")) {
        settings.m_alpacaFilterWheelEnabled = swg->getAlpacaFilterWheelEnabled() != 0;
    }
    if (featureSettingsKeys.contains("alpacaFilterWheelHost")) {
        settings.m_alpacaFilterWheelHost = *swg->getAlpacaFilterWheelHost();
    }
    if (featureSettingsKeys.contains("alpacaFilterWheelPort")) {
        settings.m_alpacaFilterWheelPort = (uint16_t) swg->getAlpacaFilterWheelPort();
    }
    if (featureSettingsKeys.contains("alpacaFilterWheelDeviceNumber")) {
        settings.m_alpacaFilterWheelDeviceNumber = swg->getAlpacaFilterWheelDeviceNumber();
    }
    if (featureSettingsKeys.contains("alpacaFilterWheelPosition")) {
        settings.m_alpacaFilterWheelPosition = swg->getAlpacaFilterWheelPosition();
    }
    if (featureSettingsKeys.contains("cameraBinX")) {
        settings.m_cameraBinX = swg->getCameraBinX();
    }
    if (featureSettingsKeys.contains("cameraBinY")) {
        settings.m_cameraBinY = swg->getCameraBinY();
    }
    if (featureSettingsKeys.contains("cameraNumX")) {
        settings.m_cameraNumX = swg->getCameraNumX();
    }
    if (featureSettingsKeys.contains("cameraNumY")) {
        settings.m_cameraNumY = swg->getCameraNumY();
    }
    if (featureSettingsKeys.contains("cameraStartX")) {
        settings.m_cameraStartX = swg->getCameraStartX();
    }
    if (featureSettingsKeys.contains("cameraStartY")) {
        settings.m_cameraStartY = swg->getCameraStartY();
    }
    if (featureSettingsKeys.contains("cameraGain")) {
        settings.m_cameraGain = swg->getCameraGain();
    }
    if (featureSettingsKeys.contains("cameraOffset")) {
        settings.m_cameraOffset = swg->getCameraOffset();
    }
    if (featureSettingsKeys.contains("cameraReadoutMode")) {
        settings.m_cameraReadoutMode = swg->getCameraReadoutMode();
    }
    if (featureSettingsKeys.contains("asiCoolerOn")) {
        settings.m_asiCoolerOn = swg->getAsiCoolerOn();
    }
    if (featureSettingsKeys.contains("asiTargetTemp")) {
        settings.m_asiTargetTemp = swg->getAsiTargetTemp();
    }
    if (featureSettingsKeys.contains("asiUsbBandwidth")) {
        settings.m_asiUsbBandwidth = swg->getAsiUsbBandwidth();
    }
    if (featureSettingsKeys.contains("asiHighSpeedMode")) {
        settings.m_asiHighSpeedMode = swg->getAsiHighSpeedMode();
    }
    if (featureSettingsKeys.contains("asiAutoExposureGain")) {
        settings.m_asiAutoExposureGain = swg->getAsiAutoExposureGain() != 0;
    }
    if (featureSettingsKeys.contains("autoExposureGainEnabled")) {
        settings.m_autoExposureGainEnabled = swg->getAutoExposureGainEnabled() != 0;
    }
    if (featureSettingsKeys.contains("autoExposureGainMode")) {
        settings.m_autoExposureGainMode = (CameraSettings::AutoExposureGainMode) swg->getAutoExposureGainMode();
    }
    if (featureSettingsKeys.contains("autoExposureTargetPercentile")) {
        settings.m_autoExposureTargetPercentile = swg->getAutoExposureTargetPercentile();
    }
    if (featureSettingsKeys.contains("autoExposureTargetBrightness")) {
        settings.m_autoExposureTargetBrightness = swg->getAutoExposureTargetBrightness();
    }
    if (featureSettingsKeys.contains("autoExposureMaxChangePercent")) {
        settings.m_autoExposureMaxChangePercent = swg->getAutoExposureMaxChangePercent();
    }
    if (featureSettingsKeys.contains("autoExposureMinMs")) {
        settings.m_autoExposureMinMs = swg->getAutoExposureMinMs();
    }
    if (featureSettingsKeys.contains("autoExposureMaxMs")) {
        settings.m_autoExposureMaxMs = swg->getAutoExposureMaxMs();
    }
    if (featureSettingsKeys.contains("autoExposureMinGain")) {
        settings.m_autoExposureMinGain = swg->getAutoExposureMinGain();
    }
    if (featureSettingsKeys.contains("autoExposureMaxGain")) {
        settings.m_autoExposureMaxGain = swg->getAutoExposureMaxGain();
    }
    if (featureSettingsKeys.contains("asiColorImageType")) {
        settings.m_asiColorImageType = (CameraSettings::AsiColorImageType) swg->getAsiColorImageType();
    }

    // Image / video output
    if (featureSettingsKeys.contains("saveImage")) {
        settings.m_saveImage = swg->getSaveImage() != 0;
    }
    if (featureSettingsKeys.contains("imageFileName")) {
        settings.m_imageFileName = *swg->getImageFileName();
    }
    if (featureSettingsKeys.contains("saveVideo")) {
        settings.m_saveVideo = swg->getSaveVideo() != 0;
    }
    if (featureSettingsKeys.contains("videoFileCameraPath")) {
        settings.m_videoFileCameraPath = *swg->getVideoFileCameraPath();
    }
    if (featureSettingsKeys.contains("streamUrl")) {
        settings.m_streamUrl = *swg->getStreamUrl();
    }
    if (featureSettingsKeys.contains("streamUrlHistory"))
    {
        settings.m_streamUrlHistory.clear();
        if (swg->getStreamUrlHistory()) {
            for (const QString *url : *swg->getStreamUrlHistory()) {
                if (url) {
                    settings.m_streamUrlHistory.append(*url);
                }
            }
        }
    }
    if (featureSettingsKeys.contains("imageFileCameraPaths"))
    {
        settings.m_imageFileCameraPaths.clear();
        if (swg->getImageFileCameraPaths()) {
            for (const QString *path : *swg->getImageFileCameraPaths()) {
                if (path) {
                    settings.m_imageFileCameraPaths.append(*path);
                }
            }
        }
    }
    if (featureSettingsKeys.contains("videoFileName")) {
        settings.m_videoFileName = *swg->getVideoFileName();
    }
    if (featureSettingsKeys.contains("videoCodec")) {
        settings.m_videoCodec = (CameraSettings::VideoCodec) swg->getVideoCodec();
    }
    if (featureSettingsKeys.contains("videoRecordBitrateKbps")) {
        settings.m_videoRecordBitrateKbps = swg->getVideoRecordBitrateKbps();
    }
    if (featureSettingsKeys.contains("recordRawFits")) {
        settings.m_recordRawFits = swg->getRecordRawFits() != 0;
    }
    if (featureSettingsKeys.contains("recordCalibratedMedia")) {
        settings.m_recordCalibratedMedia = swg->getRecordCalibratedMedia() != 0;
    }
    if (featureSettingsKeys.contains("recordFilteredMedia")) {
        settings.m_recordFilteredMedia = swg->getRecordFilteredMedia() != 0;
    }
    if (featureSettingsKeys.contains("recordPostProcessedMedia")) {
        settings.m_recordPostProcessedMedia = swg->getRecordPostProcessedMedia() != 0;
    }
    if (featureSettingsKeys.contains("keogramEnabled")) {
        settings.m_keogramEnabled = swg->getKeogramEnabled() != 0;
    }
    if (featureSettingsKeys.contains("keogramFileName")) {
        settings.m_keogramFileName = *swg->getKeogramFileName();
    }
    if (featureSettingsKeys.contains("keogramDirection")) {
        settings.m_keogramDirection = (CameraSettings::KeogramDirection) swg->getKeogramDirection();
    }
    if (featureSettingsKeys.contains("keogramDayMode")) {
        settings.m_keogramDayMode = (CameraSettings::KeogramDayMode) swg->getKeogramDayMode();
    }
    if (featureSettingsKeys.contains("keogramSamplePeriodMinutes")) {
        settings.m_keogramSamplePeriodMinutes = swg->getKeogramSamplePeriodMinutes();
    }
    if (featureSettingsKeys.contains("keogramShowPreview")) {
        settings.m_keogramShowPreview = swg->getKeogramShowPreview() != 0;
    }
    if (featureSettingsKeys.contains("youtubeStreamEnabled")) {
        settings.m_youtubeStreamEnabled = swg->getYoutubeStreamEnabled() != 0;
    }
    if (featureSettingsKeys.contains("youtubeStreamUrl")) {
        settings.m_youtubeStreamUrl = *swg->getYoutubeStreamUrl();
    }
    if (featureSettingsKeys.contains("youtubeStreamKey")) {
        settings.m_youtubeStreamKey = *swg->getYoutubeStreamKey();
    }
    if (featureSettingsKeys.contains("youtubeStreamPostProcessed")) {
        settings.m_youtubeStreamPostProcessed = swg->getYoutubeStreamPostProcessed() != 0;
    }
    if (featureSettingsKeys.contains("youtubeStreamBitrateKbps")) {
        settings.m_youtubeStreamBitrateKbps = swg->getYoutubeStreamBitrateKbps();
    }
    if (featureSettingsKeys.contains("youtubeStreamFps")) {
        settings.m_youtubeStreamFps = swg->getYoutubeStreamFps();
    }
    if (featureSettingsKeys.contains("youtubeStreamWidth")) {
        settings.m_youtubeStreamWidth = swg->getYoutubeStreamWidth();
    }
    if (featureSettingsKeys.contains("youtubeStreamHeight")) {
        settings.m_youtubeStreamHeight = swg->getYoutubeStreamHeight();
    }
    if (featureSettingsKeys.contains("videoHwAcceleration")) {
        settings.m_videoHwAcceleration = swg->getVideoHwAcceleration() != 0;
    }
    if (featureSettingsKeys.contains("videoLoop")) {
        settings.m_videoLoop = swg->getVideoLoop() != 0;
    }
    if (featureSettingsKeys.contains("videoPlaybackRate")) {
        settings.m_videoPlaybackRate = swg->getVideoPlaybackRate();
    }
    if (featureSettingsKeys.contains("videoPlaybackAudioOffsetMs")) {
        settings.m_videoPlaybackAudioOffsetMs = swg->getVideoPlaybackAudioOffsetMs();
    }
    if (featureSettingsKeys.contains("videoPreRecordBufferSeconds")) {
        settings.m_videoPreRecordBufferSeconds = swg->getVideoPreRecordBufferSeconds();
    }
    if (featureSettingsKeys.contains("imageRecordLimit")) {
        settings.m_imageRecordLimit = swg->getImageRecordLimit();
    }
    if (featureSettingsKeys.contains("videoRecordLimitSeconds")) {
        settings.m_videoRecordLimitSeconds = swg->getVideoRecordLimitSeconds();
    }
    if (featureSettingsKeys.contains("stackEnabled")) {
        settings.m_stackEnabled = swg->getStackEnabled() != 0;
    }
    if (featureSettingsKeys.contains("stackFrameCount")) {
        settings.m_stackFrameCount = swg->getStackFrameCount();
    }
    if (featureSettingsKeys.contains("stackMethod")) {
        settings.m_stackMethod = (CameraSettings::StackMethod) swg->getStackMethod();
    }
    if (featureSettingsKeys.contains("stackHdrAlgorithm")) {
        settings.m_stackHdrAlgorithm = (CameraSettings::StackHdrAlgorithm) swg->getStackHdrAlgorithm();
    }
    if (featureSettingsKeys.contains("stackHdrExposureCount")) {
        settings.m_stackHdrExposureCount = swg->getStackHdrExposureCount();
    }
    if (featureSettingsKeys.contains("stackHdrExposureTimesMs") && swg->getStackHdrExposureTimesMs())
    {
        const int count = qMin((int) settings.m_stackHdrExposureTimesMs.size(), swg->getStackHdrExposureTimesMs()->size());
        for (int i = 0; i < count; ++i) {
            settings.m_stackHdrExposureTimesMs[static_cast<size_t>(i)] = swg->getStackHdrExposureTimesMs()->at(i);
        }
    }
    if (featureSettingsKeys.contains("stackAlignmentMethod")) {
        settings.m_stackAlignmentMethod = (CameraSettings::StackAlignmentMethod) swg->getStackAlignmentMethod();
    }
    if (featureSettingsKeys.contains("stackDisplayMode")) {
        settings.m_stackDisplayMode = (CameraSettings::StackDisplayMode) swg->getStackDisplayMode();
    }
    if (featureSettingsKeys.contains("stackDisplayFrameIndex")) {
        settings.m_stackDisplayFrameIndex = swg->getStackDisplayFrameIndex();
    }
    if (featureSettingsKeys.contains("stackRejectBadFrames")) {
        settings.m_stackRejectBadFrames = swg->getStackRejectBadFrames() != 0;
    }
    if (featureSettingsKeys.contains("stackDarkFileName")) {
        settings.m_stackDarkFileName = *swg->getStackDarkFileName();
    }
    if (featureSettingsKeys.contains("stackFlatFileName")) {
        settings.m_stackFlatFileName = *swg->getStackFlatFileName();
    }
    if (featureSettingsKeys.contains("stackBiasFileName")) {
        settings.m_stackBiasFileName = *swg->getStackBiasFileName();
    }
    if (featureSettingsKeys.contains("latitude")) {
        settings.m_latitude = swg->getLatitude();
    }
    if (featureSettingsKeys.contains("longitude")) {
        settings.m_longitude = swg->getLongitude();
    }
    if (featureSettingsKeys.contains("altitude")) {
        settings.m_altitude = swg->getAltitude();
    }
    if (featureSettingsKeys.contains("positionSync")) {
        settings.m_positionSync = swg->getPositionSync() != 0;
    }
    if (featureSettingsKeys.contains("owmAPIKey")) {
        settings.m_owmAPIKey = *swg->getOwmApiKey();
    }
    if (featureSettingsKeys.contains("azimuth")) {
        settings.m_azimuth = swg->getAzimuth();
    }
    if (featureSettingsKeys.contains("elevation")) {
        settings.m_elevation = swg->getElevation();
    }
    if (featureSettingsKeys.contains("roll")) {
        settings.m_roll = swg->getRoll();
    }
    if (featureSettingsKeys.contains("rotator")) {
        settings.m_rotator = *swg->getRotator();
    }
    if (featureSettingsKeys.contains("fov")) {
        settings.m_fov = swg->getFov();
    }
    if (featureSettingsKeys.contains("fovMode")) {
        settings.m_fovMode = (CameraSettings::FovMode) swg->getFovMode();
    }
    if (featureSettingsKeys.contains("fovSensorWidthMm")) {
        settings.m_fovSensorWidthMm = swg->getFovSensorWidthMm();
    }
    if (featureSettingsKeys.contains("fovSensorHeightMm")) {
        settings.m_fovSensorHeightMm = swg->getFovSensorHeightMm();
    }
    if (featureSettingsKeys.contains("fovFocalLengthMm")) {
        settings.m_fovFocalLengthMm = swg->getFovFocalLengthMm();
    }
    if (featureSettingsKeys.contains("lensProjection")) {
        settings.m_lensProjection = (CameraSettings::LensProjection) swg->getLensProjection();
    }
    if (featureSettingsKeys.contains("lensCenterOffsetX")) {
        settings.m_lensCenterOffsetX = swg->getLensCenterOffsetX();
    }
    if (featureSettingsKeys.contains("lensCenterOffsetY")) {
        settings.m_lensCenterOffsetY = swg->getLensCenterOffsetY();
    }
    if (featureSettingsKeys.contains("lensDistortionK1")) {
        settings.m_lensDistortionK1 = swg->getLensDistortionK1();
    }

    // Post-processing – tone / colour
    if (featureSettingsKeys.contains("postProcessWhiteBalanceMode")) {
        settings.m_postProcessWhiteBalanceMode = swg->getPostProcessWhiteBalanceMode();
    }
    if (featureSettingsKeys.contains("postProcessWhiteBalanceRedGain")) {
        settings.m_postProcessWhiteBalanceRedGain = swg->getPostProcessWhiteBalanceRedGain();
    }
    if (featureSettingsKeys.contains("postProcessWhiteBalanceGreenGain")) {
        settings.m_postProcessWhiteBalanceGreenGain = swg->getPostProcessWhiteBalanceGreenGain();
    }
    if (featureSettingsKeys.contains("postProcessWhiteBalanceBlueGain")) {
        settings.m_postProcessWhiteBalanceBlueGain = swg->getPostProcessWhiteBalanceBlueGain();
    }
    if (featureSettingsKeys.contains("postProcessWhiteBalanceHighlightProtection")) {
        settings.m_postProcessWhiteBalanceHighlightProtection = swg->getPostProcessWhiteBalanceHighlightProtection();
    }
    if (featureSettingsKeys.contains("postProcessUseCuda")) {
        settings.m_postProcessUseCuda = swg->getPostProcessUseCuda() != 0;
    }
    if (featureSettingsKeys.contains("postProcessUnwarp")) {
        settings.m_postProcessUnwarp = swg->getPostProcessUnwarp() != 0;
    }
    if (featureSettingsKeys.contains("histogramStretch")) {
        settings.m_histogramStretch = (CameraSettings::HistogramStretch) swg->getHistogramStretch();
    }
    if (featureSettingsKeys.contains("histogramStretchBlackPoint")) {
        settings.m_histogramStretchBlackPoint = swg->getHistogramStretchBlackPoint();
    }
    if (featureSettingsKeys.contains("histogramStretchWhitePoint")) {
        settings.m_histogramStretchWhitePoint = swg->getHistogramStretchWhitePoint();
    }
    if (featureSettingsKeys.contains("histogramStretchGamma")) {
        settings.m_histogramStretchGamma = swg->getHistogramStretchGamma();
    }
    if (featureSettingsKeys.contains("histogramStretchAsinhStrength")) {
        settings.m_histogramStretchAsinhStrength = swg->getHistogramStretchAsinhStrength();
    }
    if (featureSettingsKeys.contains("histogramStretchLogStrength")) {
        settings.m_histogramStretchLogStrength = swg->getHistogramStretchLogStrength();
    }
    if (featureSettingsKeys.contains("postProcessGreyscale")) {
        settings.m_postProcessGreyscale = swg->getPostProcessGreyscale() != 0;
    }
    if (featureSettingsKeys.contains("saturation")) {
        settings.m_saturation = swg->getSaturation();
    }
    if (featureSettingsKeys.contains("gamma")) {
        settings.m_gamma = swg->getGamma();
    }
    if (featureSettingsKeys.contains("brightness")) {
        settings.m_brightness = swg->getBrightness();
    }
    if (featureSettingsKeys.contains("contrast")) {
        settings.m_contrast = swg->getContrast();
    }
    if (featureSettingsKeys.contains("gaussianBlur")) {
        settings.m_gaussianBlur = swg->getGaussianBlur();
    }
    if (featureSettingsKeys.contains("medianBlur")) {
        settings.m_medianBlur = swg->getMedianBlur();
    }
    if (featureSettingsKeys.contains("sharpen")) {
        settings.m_sharpen = swg->getSharpen();
    }
    if (featureSettingsKeys.contains("edgeDisplayMode")) {
        settings.m_edgeDisplayMode = (CameraSettings::EdgeDisplayMode) swg->getEdgeDisplayMode();
    }
    if (featureSettingsKeys.contains("sobelEdge")) {
        settings.m_sobelEdge = swg->getSobelEdge();
    }
    if (featureSettingsKeys.contains("cannyEdge")) {
        settings.m_cannyEdge = swg->getCannyEdge();
    }
    if (featureSettingsKeys.contains("flipX")) {
        settings.m_flipX = swg->getFlipX() != 0;
    }
    if (featureSettingsKeys.contains("flipY")) {
        settings.m_flipY = swg->getFlipY() != 0;
    }
    if (featureSettingsKeys.contains("imageRotation")) {
        settings.m_imageRotation = swg->getImageRotation();
    }
    if (featureSettingsKeys.contains("invertColors")) {
        settings.m_invertColors = swg->getInvertColors() != 0;
    }

    // Date/time overlay
    if (featureSettingsKeys.contains("overlayDateTime")) {
        settings.m_overlayDateTime = swg->getOverlayDateTime() != 0;
    }
    if (featureSettingsKeys.contains("dateTimeColor")) {
        settings.m_dateTimeColor = QColor(swg->getDateTimeColor());
    }
    if (featureSettingsKeys.contains("dateTimeFormat")) {
        settings.m_dateTimeFormat = *swg->getDateTimeFormat();
    }
    if (featureSettingsKeys.contains("dateTimePosX")) {
        settings.m_dateTimePosX = swg->getDateTimePosX();
    }
    if (featureSettingsKeys.contains("dateTimePosY")) {
        settings.m_dateTimePosY = swg->getDateTimePosY();
    }

    // Text overlay
    if (featureSettingsKeys.contains("overlayText")) {
        settings.m_overlayText = swg->getOverlayText() != 0;
    }
    if (featureSettingsKeys.contains("equatorialGrid")) {
        settings.m_equatorialGrid = swg->getEquatorialGrid() != 0;
    }
    if (featureSettingsKeys.contains("equatorialGridColor")) {
        settings.m_equatorialGridColor = QColor(swg->getEquatorialGridColor());
    }
    if (featureSettingsKeys.contains("altAzGrid")) {
        settings.m_altAzGrid = swg->getAltAzGrid() != 0;
    }
    if (featureSettingsKeys.contains("altAzGridColor")) {
        settings.m_altAzGridColor = QColor(swg->getAltAzGridColor());
    }
    if (featureSettingsKeys.contains("constellation")) {
        settings.m_constellation = swg->getConstellation() != 0;
    }
    if (featureSettingsKeys.contains("constellationColor")) {
        settings.m_constellationColor = QColor(swg->getConstellationColor());
    }
    if (featureSettingsKeys.contains("constellationOverlay")) {
        settings.m_constellationOverlay = (CameraSettings::ConstellationOverlay) swg->getConstellationOverlay();
    }
    if (featureSettingsKeys.contains("trackObjects")) {
        settings.m_trackObjects = swg->getTrackObjects() != 0;
    }
    if (featureSettingsKeys.contains("trackObjectMinElevation")) {
        settings.m_trackObjectMinElevation = swg->getTrackObjectMinElevation();
    }
    if (featureSettingsKeys.contains("trackObjectColor")) {
        settings.m_trackObjectColor = QColor(swg->getTrackObjectColor());
    }
    if (featureSettingsKeys.contains("trackObjectFontScale")) {
        settings.m_trackObjectFontScale = swg->getTrackObjectFontScale();
    }
    if (featureSettingsKeys.contains("trackObjectTrails")) {
        settings.m_trackObjectTrails = swg->getTrackObjectTrails() != 0;
    }
    if (featureSettingsKeys.contains("trackObjectHeatMap")) {
        settings.m_trackObjectHeatMap = swg->getTrackObjectHeatMap() != 0;
    }
    if (featureSettingsKeys.contains("gridLabelFontFamily")) {
        settings.m_gridLabelFontFamily = *swg->getGridLabelFontFamily();
    }
    if (featureSettingsKeys.contains("gridLabelFontScale")) {
        settings.m_gridLabelFontScale = swg->getGridLabelFontScale();
    }
    if (featureSettingsKeys.contains("overlayTextString")) {
        settings.m_overlayTextString = *swg->getOverlayTextString();
    }
    if (featureSettingsKeys.contains("overlayTextColor")) {
        settings.m_overlayTextColor = QColor(swg->getOverlayTextColor());
    }
    if (featureSettingsKeys.contains("overlayTextFontFamily")) {
        settings.m_overlayTextFontFamily = *swg->getOverlayTextFontFamily();
    }
    if (featureSettingsKeys.contains("overlayTextFontScale")) {
        settings.m_overlayTextFontScale = swg->getOverlayTextFontScale();
    }
    if (featureSettingsKeys.contains("overlayTextPosX")) {
        settings.m_overlayTextPosX = swg->getOverlayTextPosX();
    }
    if (featureSettingsKeys.contains("overlayTextPosY")) {
        settings.m_overlayTextPosY = swg->getOverlayTextPosY();
    }

    // Diff-mask
    if (featureSettingsKeys.contains("diffMask")) {
        settings.m_diffMask = swg->getDiffMask() != 0;
    }
    if (featureSettingsKeys.contains("diffThreshold")) {
        settings.m_diffThreshold = swg->getDiffThreshold();
    }
    if (featureSettingsKeys.contains("diffMaskOpenSize")) {
        settings.m_diffMaskOpenSize = swg->getDiffMaskOpenSize();
    }
    if (featureSettingsKeys.contains("dilationSize")) {
        settings.m_dilationSize = swg->getDilationSize();
    }
    if (featureSettingsKeys.contains("diffMaskHistoryFrames")) {
        settings.m_diffMaskHistoryFrames = swg->getDiffMaskHistoryFrames();
    }
    if (featureSettingsKeys.contains("diffMaskCloseSize")) {
        settings.m_diffMaskCloseSize = swg->getDiffMaskCloseSize();
    }
    if (featureSettingsKeys.contains("overlayFontFamily")) {
        settings.m_overlayFontFamily = *swg->getOverlayFontFamily();
    }
    if (featureSettingsKeys.contains("overlayFontScale")) {
        settings.m_overlayFontScale = swg->getOverlayFontScale();
    }

    // Detection ROI
    if (featureSettingsKeys.contains("detectionRoiX")) {
        settings.m_detectionRoiX = swg->getDetectionRoiX();
    }
    if (featureSettingsKeys.contains("detectionRoiY")) {
        settings.m_detectionRoiY = swg->getDetectionRoiY();
    }
    if (featureSettingsKeys.contains("detectionRoiWidth")) {
        settings.m_detectionRoiWidth = swg->getDetectionRoiWidth();
    }
    if (featureSettingsKeys.contains("detectionRoiHeight")) {
        settings.m_detectionRoiHeight = swg->getDetectionRoiHeight();
    }
    if (featureSettingsKeys.contains("showDetectionRoi")) {
        settings.m_showDetectionRoi = swg->getShowDetectionRoi() != 0;
    }

    // Motion detection
    if (featureSettingsKeys.contains("motionDetect")) {
        settings.m_motionDetect = swg->getMotionDetect() != 0;
    }
    if (featureSettingsKeys.contains("motionBackgroundSubtractor")) {
        settings.m_motionBackgroundSubtractor = (CameraSettings::MotionBackgroundSubtractor) swg->getMotionBackgroundSubtractor();
    }
    if (featureSettingsKeys.contains("motionMaskView")) {
        settings.m_motionMaskView = (CameraSettings::MotionMaskView) swg->getMotionMaskView();
    }
    if (featureSettingsKeys.contains("motionHistory")) {
        settings.m_motionHistory = swg->getMotionHistory();
    }
    if (featureSettingsKeys.contains("motionVarThreshold")) {
        settings.m_motionVarThreshold = swg->getMotionVarThreshold();
    }
    if (featureSettingsKeys.contains("motionLearningRate")) {
        settings.m_motionLearningRate = swg->getMotionLearningRate();
    }
    if (featureSettingsKeys.contains("motionConfirmFrames")) {
        settings.m_motionConfirmFrames = swg->getMotionConfirmFrames();
    }
    if (featureSettingsKeys.contains("motionDownscale")) {
        settings.m_motionDownscale = swg->getMotionDownscale();
    }
    if (featureSettingsKeys.contains("motionDetectShadows")) {
        settings.m_motionDetectShadows = swg->getMotionDetectShadows() != 0;
    }
    if (featureSettingsKeys.contains("motionOpenSize")) {
        settings.m_motionOpenSize = swg->getMotionOpenSize();
    }
    if (featureSettingsKeys.contains("motionCloseSize")) {
        settings.m_motionCloseSize = swg->getMotionCloseSize();
    }
    if (featureSettingsKeys.contains("motionPersistenceFrames")) {
        settings.m_motionPersistenceFrames = swg->getMotionPersistenceFrames();
    }
    if (featureSettingsKeys.contains("motionBoxColor")) {
        settings.m_motionBoxColor = QColor(swg->getMotionBoxColor());
    }
    if (featureSettingsKeys.contains("minContourArea")) {
        settings.m_minContourArea = swg->getMinContourArea();
    }
    if (featureSettingsKeys.contains("motionExclusionRects"))
    {
        settings.m_motionExclusionRects.clear();

        if (swg->getMotionExclusionRects())
        {
            for (auto *swgRect : *swg->getMotionExclusionRects())
            {
                if (swgRect) {
                    settings.m_motionExclusionRects.append(QRect(
                        swgRect->getX(),
                        swgRect->getY(),
                        swgRect->getWidth(),
                        swgRect->getHeight()));
                }
            }
        }
    }
    if (featureSettingsKeys.contains("starDetect")) {
        settings.m_starDetect = swg->getStarDetect() != 0;
    }
    if (featureSettingsKeys.contains("starThreshold")) {
        settings.m_starThreshold = swg->getStarThreshold();
    }
    if (featureSettingsKeys.contains("starBackgroundBlur")) {
        settings.m_starBackgroundBlur = swg->getStarBackgroundBlur();
    }
    if (featureSettingsKeys.contains("starMinArea")) {
        settings.m_starMinArea = swg->getStarMinArea();
    }
    if (featureSettingsKeys.contains("starMaxArea")) {
        settings.m_starMaxArea = swg->getStarMaxArea();
    }
    if (featureSettingsKeys.contains("starMaxAspectRatio")) {
        settings.m_starMaxAspectRatio = swg->getStarMaxAspectRatio();
    }
    if (featureSettingsKeys.contains("starDebugView")) {
        settings.m_starDebugView = (CameraSettings::StarDebugView) swg->getStarDebugView();
    }
    if (featureSettingsKeys.contains("starColor")) {
        settings.m_starColor = QColor(swg->getStarColor());
    }
    if (featureSettingsKeys.contains("plateSolve")) {
        settings.m_plateSolve = swg->getPlateSolve() != 0;
    }
    if (featureSettingsKeys.contains("plateSolveMaxMagnitude")) {
        settings.m_plateSolveMaxMagnitude = swg->getPlateSolveMaxMagnitude();
    }
    if (featureSettingsKeys.contains("plateSolveMinMatches")) {
        settings.m_plateSolveMinMatches = swg->getPlateSolveMinMatches();
    }
    if (featureSettingsKeys.contains("plateSolveMatchRadius")) {
        settings.m_plateSolveMatchRadius = swg->getPlateSolveMatchRadius();
    }
    if (featureSettingsKeys.contains("plateSolveFinalMatchRadius")) {
        settings.m_plateSolveFinalMatchRadius = swg->getPlateSolveFinalMatchRadius();
    }
    if (featureSettingsKeys.contains("plateSolveSearchRadius")) {
        settings.m_plateSolveAzElSearchRadius = swg->getPlateSolveSearchRadius();
    }
    if (featureSettingsKeys.contains("plateSolveFovTolerance")) {
        settings.m_plateSolveFovTolerance = swg->getPlateSolveFovTolerance();
    }
    if (featureSettingsKeys.contains("plateSolveStartMode")) {
        settings.m_plateSolveStartMode = (CameraSettings::PlateSolveStartMode) swg->getPlateSolveStartMode();
    }
    if (featureSettingsKeys.contains("plateSolveLabelMode")) {
        settings.m_plateSolveLabelMode = (CameraSettings::PlateSolveLabelMode) swg->getPlateSolveLabelMode();
    }
    if (featureSettingsKeys.contains("plateSolveUseCaptureDateTime")) {
        settings.m_plateSolveUseCaptureDateTime = swg->getPlateSolveUseCaptureDateTime() != 0;
    }
    if (featureSettingsKeys.contains("plateSolveDateTime"))
    {
        settings.m_plateSolveDateTime = swg->getPlateSolveDateTime()
            ? QDateTime::fromString(*swg->getPlateSolveDateTime(), Qt::ISODateWithMs)
            : QDateTime();
        if (!settings.m_plateSolveDateTime.isValid() && swg->getPlateSolveDateTime()) {
            settings.m_plateSolveDateTime = QDateTime::fromString(*swg->getPlateSolveDateTime(), Qt::ISODate);
        }
    }
    if (featureSettingsKeys.contains("plateSolveDateTimeUtc")) {
        settings.m_plateSolveDateTimeUtc = swg->getPlateSolveDateTimeUtc() != 0;
    }
    if (featureSettingsKeys.contains("plateSolveUseDownloadedCatalog")) {
        settings.m_plateSolveUseDownloadedCatalog = swg->getPlateSolveUseDownloadedCatalog() != 0;
    }
    if (featureSettingsKeys.contains("plateSolveCatalogSource")) {
        settings.m_plateSolveCatalogSource = (CameraSettings::PlateSolveCatalogSource) swg->getPlateSolveCatalogSource();
    }
    if (featureSettingsKeys.contains("plateSolveApplyMode")) {
        settings.m_plateSolveApplyMode = (CameraSettings::PlateSolveApplyMode) swg->getPlateSolveApplyMode();
    }
    if (featureSettingsKeys.contains("starCatalogDiskCacheSizeGb")) {
        settings.m_starCatalogDiskCacheSizeGb = swg->getStarCatalogDiskCacheSizeGb();
    }

    // Spectrum overlay
    if (featureSettingsKeys.contains("overlaySpectrum")) {
        settings.m_overlaySpectrum = swg->getOverlaySpectrum() != 0;
    }
    if (featureSettingsKeys.contains("spectrumDevice")) {
        settings.m_spectrumDevice = *swg->getSpectrumDevice();
    }
    if (featureSettingsKeys.contains("spectrumOffsetX")) {
        settings.m_spectrumOffsetX = swg->getSpectrumOffsetX();
    }
    if (featureSettingsKeys.contains("spectrumOffsetY")) {
        settings.m_spectrumOffsetY = swg->getSpectrumOffsetY();
    }
    if (featureSettingsKeys.contains("spectrumScale")) {
        settings.m_spectrumScale = swg->getSpectrumScale();
    }

    // YOLO
    if (featureSettingsKeys.contains("yoloEnabled")) {
        settings.m_yoloEnabled = swg->getYoloEnabled() != 0;
    }
    if (featureSettingsKeys.contains("yoloModelPath")) {
        settings.m_yoloModelPath = *swg->getYoloModelPath();
    }
    if (featureSettingsKeys.contains("yoloLabelsPath")) {
        settings.m_yoloLabelsPath = *swg->getYoloLabelsPath();
    }
    if (featureSettingsKeys.contains("yoloConfThreshold")) {
        settings.m_yoloConfThreshold = swg->getYoloConfThreshold();
    }
    if (featureSettingsKeys.contains("yoloNmsThreshold")) {
        settings.m_yoloNmsThreshold = swg->getYoloNmsThreshold();
    }
    if (featureSettingsKeys.contains("yoloBoxColor")) {
        settings.m_yoloBoxColor = QColor(swg->getYoloBoxColor());
    }
    if (featureSettingsKeys.contains("yoloTileLargeImages")) {
        settings.m_yoloTileLargeImages = swg->getYoloTileLargeImages() != 0;
    }
    if (featureSettingsKeys.contains("yoloTileOverlapPercent")) {
        settings.m_yoloTileOverlapPercent = swg->getYoloTileOverlapPercent();
    }
    if (featureSettingsKeys.contains("yoloIgnoredClassNames"))
    {
        settings.m_yoloIgnoredClassNames.clear();
        if (swg->getYoloIgnoredClassNames()) {
            for (const QString *className : *swg->getYoloIgnoredClassNames()) {
                if (className) {
                    settings.m_yoloIgnoredClassNames.append(*className);
                }
            }
        }
    }
    if (featureSettingsKeys.contains("yoloDnnTarget")) {
        settings.m_yoloDnnTarget = (CameraSettings::DNNTarget) swg->getYoloDnnTarget();
    }

    // Audio (Qt camera)
    if (featureSettingsKeys.contains("audioMute")) {
        settings.m_audioMute = swg->getAudioMute() != 0;
    }
    if (featureSettingsKeys.contains("audioDeviceName")) {
        settings.m_audioDeviceName = *swg->getAudioDeviceName();
    }

    // Qt camera controls
    if (featureSettingsKeys.contains("whiteBalanceMode")) {
        settings.m_whiteBalanceMode = swg->getWhiteBalanceMode();
    }
    if (featureSettingsKeys.contains("exposureCompensation")) {
        settings.m_exposureCompensation = swg->getExposureCompensation();
    }
    if (featureSettingsKeys.contains("focusMode")) {
        settings.m_focusMode = swg->getFocusMode();
    }
    if (featureSettingsKeys.contains("focusDistance")) {
        settings.m_focusDistance = swg->getFocusDistance();
    }
    if (featureSettingsKeys.contains("zoomFactor")) {
        settings.m_zoomFactor = swg->getZoomFactor();
    }

    // Reverse API
    if (featureSettingsKeys.contains("useReverseAPI")) {
        settings.m_useReverseAPI = swg->getUseReverseApi() != 0;
    }
    if (featureSettingsKeys.contains("reverseAPIAddress")) {
        settings.m_reverseAPIAddress = *swg->getReverseApiAddress();
    }
    if (featureSettingsKeys.contains("reverseAPIPort")) {
        settings.m_reverseAPIPort = (uint16_t) swg->getReverseApiPort();
    }
    if (featureSettingsKeys.contains("reverseAPIFeatureSetIndex")) {
        settings.m_reverseAPIFeatureSetIndex = swg->getReverseApiFeatureSetIndex();
    }
    if (featureSettingsKeys.contains("reverseAPIFeatureIndex")) {
        settings.m_reverseAPIFeatureIndex = swg->getReverseApiFeatureIndex();
    }

    if (settings.m_rollupState && featureSettingsKeys.contains("rollupState")) {
        settings.m_rollupState->updateFrom(featureSettingsKeys, swg->getRollupState());
    }

    if (featureSettingsKeys.contains("workspaceIndex")) {
        settings.m_workspaceIndex = swg->getWorkspaceIndex();
    }
}
