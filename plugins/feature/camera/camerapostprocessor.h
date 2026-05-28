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

#ifndef INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_
#define INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_

#include <QObject>
#include <deque>
#include <limits>
#include <QHash>
#include <QMutex>
#include <QImage>
#include <QDateTime>
#include <QTextDocument>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "util/message.h"
#include "util/messagequeue.h"
#include "availablechannelorfeaturehandler.h"
#include "camerapipelineframe.h"
#include "camerasettings.h"

class Weather;
namespace SWGSDRangel {
    class SWGMapItem;
}

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
        const CameraHistogramData& getHistogramData() const { return m_histogramData; }
        int getStackCount() const { return m_stack.m_count; }
        int getStackQueuedCount() const { return m_stack.m_queuedCount; }
        int getStackDroppedCount() const { return m_stack.m_droppedCount; }
        int getStackRejectedCount() const { return m_stack.m_rejectedCount; }
        const QString& getStackRejectReason() const { return m_stack.m_rejectReason; }
        const QVector<CameraPipelineStarDetection>& getStarDetections() const { return m_starDetections; }
        bool isPlateSolved() const { return m_plateSolve.m_solved; }
        int getPlateSolvedMatches() const { return m_plateSolve.m_matchedStars; }
        int getPlateSolveDetectedStarsConsidered() const { return m_plateSolve.m_detectedStarsConsidered; }
        int getPlateSolveCatalogStarsLoaded() const { return m_plateSolve.m_catalogStarsLoaded; }
        int getPlateSolveCatalogCandidateStars() const { return m_plateSolve.m_catalogCandidateStars; }
        int getPlateSolveOutlierStars() const { return m_plateSolve.m_outlierStars; }
        float getPlateSolveRmsError() const { return m_plateSolve.m_rmsError; }
        float getPlateSolveMaxError() const { return m_plateSolve.m_maxError; }
        float getPlateSolveAzimuth() const { return m_plateSolve.m_azimuth; }
        float getPlateSolveElevation() const { return m_plateSolve.m_elevation; }
        float getPlateSolveRoll() const { return m_plateSolve.m_roll; }
        float getPlateSolveFov() const { return m_plateSolve.m_fov; }
        float getPlateSolveCenterOffsetX() const { return m_plateSolve.m_centerOffsetX; }
        float getPlateSolveCenterOffsetY() const { return m_plateSolve.m_centerOffsetY; }
        float getPlateSolveDistortionK1() const { return m_plateSolve.m_distortionK1; }
        const QString& getPlateSolveCatalogSource() const { return m_plateSolve.m_catalogSource; }

        static MsgReportFrame* create(const QImage& image,
                                      const CameraHistogramData& histogramData,
                                      const CameraPipelineStacking& stack,
                                      const QVector<CameraPipelineStarDetection>& starDetections,
                                      const CameraPipelinePlateSolve& plateSolve)
        {
            return new MsgReportFrame(
                image,
                histogramData,
                stack,
                starDetections,
                plateSolve);
        }

    private:
        QImage m_image;
        CameraHistogramData m_histogramData;
        CameraPipelineStacking m_stack;
        QVector<CameraPipelineStarDetection> m_starDetections;
        CameraPipelinePlateSolve m_plateSolve;

        MsgReportFrame(const QImage& image,
                       const CameraHistogramData& histogramData,
                       const CameraPipelineStacking& stack,
                       const QVector<CameraPipelineStarDetection>& starDetections,
                       const CameraPipelinePlateSolve& plateSolve) :
            Message(),
            m_image(image),
            m_histogramData(histogramData),
            m_stack(stack),
            m_starDetections(starDetections),
            m_plateSolve(plateSolve)
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

    CameraPostProcessor();
    ~CameraPostProcessor();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }
    void setNextStageInputMessageQueue(MessageQueue *messageQueue) { m_nextStageQueue = messageQueue; }

private:
    struct TrackedMapObject
    {
        QString m_label;
        double m_latitude = 0.0;
        double m_longitude = 0.0;
        double m_altitude = 0.0;
        QDateTime m_availableUntil;
    };

    struct BufferedVideoFrame
    {
        QImage m_rawImage;
        QImage m_processedImage;
    };

    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_nextStageQueue;
    AvailableChannelOrFeatureHandler m_availableChannelOrFeatureHandler;
    CameraSettings m_settings;
    bool m_captureActive;
    CameraPipelineFrame m_lastFrame;
    QDateTime m_captureDateTime;
    cv::VideoWriter m_rawVideoWriter;
    cv::VideoWriter m_processedVideoWriter;
    // Frame size each writer was opened with. We re-open on size mismatch so a stray
    // resolution change between when the writer was opened and the next frame doesn't
    // silently corrupt the file (the cv::VideoWriter has no API to query its own size).
    QSize m_rawVideoWriterSize;
    QSize m_processedVideoWriterSize;
    std::deque<BufferedVideoFrame> m_preRecordVideoFrames;
    bool m_preRecordBufferFlushed;
    int m_recordedImageFrames;
    QDateTime m_videoRecordingStartDateTime;
    QImage m_spectrumViewImage;
    Weather *m_weather = nullptr;
    float m_weatherTemperature = std::numeric_limits<float>::quiet_NaN();
    float m_weatherPressure = std::numeric_limits<float>::quiet_NaN();
    float m_weatherHumidity = std::numeric_limits<float>::quiet_NaN();
    float m_weatherCloudiness = std::numeric_limits<float>::quiet_NaN();
    float m_weatherWindSpeed = std::numeric_limits<float>::quiet_NaN();
    float m_weatherWindDirection = std::numeric_limits<float>::quiet_NaN();
    QHash<QString, TrackedMapObject> m_trackedMapObjects;
    QMutex m_frameMutex;
    CameraPipelineFramePtr m_pendingFrame;
    bool m_processingFrame;
    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void processNewFrame(const CameraPipelineFramePtr& frame);
    [[nodiscard]] QImage applyPostProcessing(const CameraPipelineFrame& frame);
    void applyMotionOverlay(QImage& image, const QVector<QRect>& motionBoxes) const;
    void applyDetectionOverlay(QImage& image, const QVector<CameraPipelineDetection>& detections) const;
    void applyStarOverlay(QImage& image, const QVector<CameraPipelineStarDetection>& starDetections) const;
    void applySpectrumOverlay(QImage& image) const;
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    void applySkyGridOverlay(QImage& image) const;
    void applyConstellationOverlay(QImage& image) const;
    void applyTrackedObjectOverlay(QImage& image) const;
    void applyDateTimeOverlay(QImage& image) const;
    void applyTextOverlay(QImage& image, QTextDocument& overlayTextDocument) const;
    [[nodiscard]] QString expandOverlayTextTemplate() const;
    void updateTrackedMapObject(const QObject* pipeSource, SWGSDRangel::SWGMapItem* swgMapItem);
    void restartWeatherUpdates();
    void setVideoRecordingEnabled(bool enabled);
    void setImageRecordingEnabled(bool enabled);
    void resetRecordingLimits();
    void updateRecordingLimitsAfterFrame(bool savedImageFrame, bool savedVideoFrame);
    void reportFrameToGUI(const QImage& image, const CameraPipelineFrame& frame);
    [[nodiscard]] static QString createTimestampedOutputFilename(const QString& baseFileName, bool rawVariant);
    [[nodiscard]] bool shouldSaveRawMedia() const;
    [[nodiscard]] bool shouldSaveProcessedMedia() const;
    void closeVideoWriters();
    bool ensureVideoWriter(cv::VideoWriter& writer, const QString& baseFileName, const QImage& frameForSize, bool rawVariant);
    void writeVideoFrame(cv::VideoWriter& writer, const QImage& frameToWrite);
    int preRecordBufferFrameLimit() const;
    void trimPreRecordBuffer();
    void appendPreRecordFrame(const QImage& rawImage, const QImage& processedImage);
    void flushPreRecordFrames(const QImage& currentRawImage, const QImage& currentProcessedImage);
private slots:
    void handleInputMessages();
    void handlePipeMessageQueue(MessageQueue* messageQueue);
    void processNextFrame();
    void weatherUpdated(float temperature, float pressure, float humidity, float cloudiness, float windSpeed, float windDirection);

};

#endif // INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_
