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

#include <atomic>

#include <QByteArray>
#include <QDateTime>
#include <QMutex>
#include <QStringList>
#include <QThread>

#include "feature/feature.h"
#include "util/message.h"
#include "camerasettings.h"
#include "cameradetector.h"
#include "cameraclouddetector.h"
#include "cameramotiondetector.h"
#include "camerastardetector.h"
#include "cameraobjectdetector.h"
#include "cameradiffdetector.h"
#include "cameraframepreprocessor.h"
#include "cameraframealigner.h"
#include "cameraframestacker.h"
#include "cameraimageprocessor.h"
#include "camerarecorder.h"
#include "camerapostprocessor.h"
#include "cameraworker.h"

class WebAPIAdapterInterface;
class MessageQueue;

namespace SWGSDRangel {
    class SWGDeviceState;
    class SWGFeatureSettings;
    class SWGFeatureReport;
    class SWGFeatureActions;
}

/**
 * \brief SDRangel Feature that drives the camera capture and image-processing pipeline.
 *
 * Camera is the top-level Feature object for the camera plugin. It owns the whole
 * processing chain and the capture orchestrator: a CameraWorker plus a sequence of
 * QObject stages (frame preprocessor -> frame aligner -> frame stacker -> image
 * processor -> cloud/motion/star/object/diff detectors -> recorder, with a continuously
 * running post-processor). Each stage lives on its own dedicated QThread and frames
 * flow down the chain as CameraPipelineFramePtr passed via per-stage message queues.
 * The class handles configuration (CameraSettings), start/stop, serialization, and the
 * REST/WebAPI surface (settings/report/actions/run), and routes GUI-bound reports.
 *
 * \note The Camera object itself lives on the main thread; every owned pipeline stage
 *       runs on its own QThread and communicates only through message queues, so cross-
 *       thread access goes through those queues rather than direct calls.
 * \note Threads are torn down (quit/wait) in the destructor; each stage QObject is
 *       deleteLater-ed when its thread finishes, so do not delete stages directly.
 * \note m_captureEpoch tags frames for the current capture session; acceptsPipelineFrame
 *       and the discardQueuedProcessFrames* helpers use it to drop stale frames from a
 *       previous capture run. Manual-preview frames bypass the epoch check.
 * \see CameraWorker, CameraSettings, CameraPlugin, CameraWebAPIAdapter
 */
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
        quint64 getCaptureEpoch() const { return m_captureEpoch; }

        static MsgStartStop* create(bool startStop, quint64 captureEpoch = 0)
        {
            return new MsgStartStop(startStop, captureEpoch);
        }

    private:
        bool m_startStop;
        quint64 m_captureEpoch;

        MsgStartStop(bool startStop, quint64 captureEpoch) :
            Message(),
            m_startStop(startStop),
            m_captureEpoch(captureEpoch)
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

    class MsgCaptureActive : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool isActive() const { return m_active; }
        quint64 getCaptureEpoch() const { return m_captureEpoch; }

        static MsgCaptureActive* create(bool active, quint64 captureEpoch = 0)
        {
            return new MsgCaptureActive(active, captureEpoch);
        }

    private:
        bool m_active;
        quint64 m_captureEpoch;

        MsgCaptureActive(bool active, quint64 captureEpoch) :
            Message(),
            m_active(active),
            m_captureEpoch(captureEpoch)
        { }
    };

    class MsgRefreshCameraList : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        bool getForceRefresh() const { return m_forceRefresh; }

        static MsgRefreshCameraList* create(bool forceRefresh = false)
        {
            return new MsgRefreshCameraList(forceRefresh);
        }

    private:
        bool m_forceRefresh;

        MsgRefreshCameraList(bool forceRefresh) :
            Message(),
            m_forceRefresh(forceRefresh)
        {}
    };

    class MsgReportError : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QString& getTitle() const { return m_title; }
        const QString& getErrorMessage() const { return m_errorMessage; }

        static MsgReportError* create(const QString& title, const QString& errorMessage)
        {
            return new MsgReportError(title, errorMessage);
        }

    private:
        QString m_title;
        QString m_errorMessage;

        MsgReportError(const QString& title, const QString& errorMessage) :
            Message(),
            m_title(title),
            m_errorMessage(errorMessage)
        { }
    };

    static int discardQueuedProcessFrames(MessageQueue& queue);
    static int discardQueuedProcessFramesOnCaptureActive(MessageQueue& queue);
    static bool acceptsPipelineFrame(const CameraPipelineFramePtr& frame, bool captureActive, quint64 captureEpoch);

    class MsgDeleteStackFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        int getFrameIndex() const { return m_frameIndex; }

        static MsgDeleteStackFrame* create(int frameIndex)
        {
            return new MsgDeleteStackFrame(frameIndex);
        }

    private:
        int m_frameIndex;

        MsgDeleteStackFrame(int frameIndex) :
            Message(),
            m_frameIndex(frameIndex)
        { }
    };

    class MsgSaveClearSkyReference : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgSaveClearSkyReference* create()
        {
            return new MsgSaveClearSkyReference();
        }

    private:
        MsgSaveClearSkyReference() :
            Message()
        { }
    };

    class MsgClearTrackedObjectHeatMap : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgClearTrackedObjectHeatMap* create()
        {
            return new MsgClearTrackedObjectHeatMap();
        }

    private:
        MsgClearTrackedObjectHeatMap() :
            Message()
        { }
    };

    class MsgSaveCurrentImage : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgSaveCurrentImage* create()
        {
            return new MsgSaveCurrentImage();
        }

    private:
        MsgSaveCurrentImage() :
            Message()
        { }
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

    virtual int webapiRun(bool run,
            SWGSDRangel::SWGDeviceState& response,
            QString& errorMessage) override;

    virtual int webapiSettingsGet(
            SWGSDRangel::SWGFeatureSettings& response,
            QString& errorMessage) override;

    virtual int webapiSettingsPutPatch(
            bool force,
            const QStringList& featureSettingsKeys,
            SWGSDRangel::SWGFeatureSettings& response,
            QString& errorMessage) override;

    virtual int webapiReportGet(
            SWGSDRangel::SWGFeatureReport& response,
            QString& errorMessage) override;

    virtual int webapiActionsPost(
            const QStringList& featureActionsKeys,
            SWGSDRangel::SWGFeatureActions& query,
            QString& errorMessage) override;

    static void webapiFormatFeatureSettings(
            SWGSDRangel::SWGFeatureSettings& response,
            const CameraSettings& settings);

    static void webapiUpdateFeatureSettings(
            CameraSettings& settings,
            const QStringList& featureSettingsKeys,
            SWGSDRangel::SWGFeatureSettings& response);

    static const char* const m_featureIdURI;
    static const char* const m_featureId;

    MessageQueue* getWorkerInputMessageQueue() { return m_worker ? m_worker->getInputMessageQueue() : nullptr; }
    CameraFrameAligner* getFrameAligner() { return m_frameAligner; }
    MessageQueue* getDetectorInputMessageQueue() { return m_objectDetector ? m_objectDetector->getInputMessageQueue() : nullptr; }
    void submitRecorderAudioSamples(const QByteArray& pcmS16Stereo, int sampleRate);
    void requestPreRecordPreview(qint64 offsetMs);
    void submitWindowOverlayFrames(const QVector<CameraPostProcessor::WindowOverlayFrame>& frames);
    void setMessageQueueToGUI(MessageQueue *queue) override;

private:
    QThread *m_workerThread;
    CameraWorker *m_worker;
    QThread *m_framePreprocessorThread;
    CameraFramePreprocessor *m_framePreprocessor;
    QThread *m_frameAlignerThread;
    CameraFrameAligner *m_frameAligner;
    QThread *m_frameStackerThread;
    CameraFrameStacker *m_frameStacker;
    QThread *m_imageProcessorThread;
    CameraImageProcessor *m_imageProcessor;
    QThread *m_cloudDetectorThread;
    CameraCloudDetector *m_cloudDetector;
    QThread *m_motionDetectorThread;
    CameraMotionDetector *m_motionDetector;
    QThread *m_starDetectorThread;
    CameraStarDetector *m_starDetector;
    QThread *m_objectDetectorThread;
    CameraObjectDetector *m_objectDetector;
    QThread *m_diffDetectorThread;
    CameraDiffDetector *m_diffDetector;
    QThread *m_recorderThread;
    CameraRecorder *m_recorder;
    QThread *m_postProcessorThread;
    CameraPostProcessor *m_postProcessor;
    CameraSettings m_settings;
    quint64 m_captureEpoch;
    // Latest cloud coverage from MsgReportCloudCoverage; atomics because webapiReportGet
    // may read them from a different thread than handleMessage
    std::atomic<float> m_lastCloudCoveragePercent{0.0f};
    std::atomic<bool> m_lastCloudCoverageValid{false};
    QMutex m_reportMutex;
    QDateTime m_reportCaptureDateTime;
    QStringList m_reportDetectedObjectClasses;
    bool m_reportMotionDetected = false;
    CameraCloudEventTracker m_cloudEventTracker;

    void start();
    void stop();
    CameraFramePreprocessor* getFramePreprocessor() { return m_framePreprocessor; }
    MessageQueue* getRecorderInputMessageQueue() { return m_recorder ? m_recorder->getInputMessageQueue() : nullptr; }
    MessageQueue* getPostProcessorInputMessageQueue() { return m_postProcessor ? m_postProcessor->getInputMessageQueue() : nullptr; }
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void webapiFormatFeatureReport(SWGSDRangel::SWGFeatureReport& response);
};

#endif // INCLUDE_FEATURE_CAMERA_H_
