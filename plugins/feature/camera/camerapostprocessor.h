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
#include <limits>
#include <QHash>
#include <QMutex>
#include <QImage>
#include <QColor>
#include <QDateTime>
#include <QPointF>
#include <QTextDocument>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

    struct PreviewTextLabel
    {
        QString m_text;
        QPointF m_position;
        QColor m_color;
        QString m_fontFamily;
        double m_fontPointSize = 9.0;
        bool m_positionIsTopLeft = false;
        bool m_background = false;
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
        const QVector<PreviewTextLabel>& getPreviewTextLabels() const { return m_previewTextLabels; }
        const QVector<QRect>& getMotionBoxes() const { return m_motionBoxes; }
        const QVector<CameraPipelineDetection>& getDetections() const { return m_detections; }
        const QDateTime& getCaptureDateTime() const { return m_captureDateTime; }
        quint64 getCaptureEpoch() const { return m_captureEpoch; }
        bool isManualPreviewFrame() const { return m_manualPreviewFrame; }

        static MsgReportFrame* create(const QImage& image,
                                      const CameraHistogramData& histogramData,
                                      const CameraPipelineStacking& stack,
                                      const QVector<CameraPipelineStarDetection>& starDetections,
                                      const CameraPipelinePlateSolve& plateSolve,
                                      const QVector<QRect>& motionBoxes,
                                      const QVector<CameraPipelineDetection>& detections,
                                      const QDateTime& captureDateTime,
                                      quint64 captureEpoch,
                                      bool manualPreviewFrame,
                                      const QVector<PreviewTextLabel>& previewTextLabels)
        {
            return new MsgReportFrame(
                image,
                histogramData,
                stack,
                starDetections,
                plateSolve,
                motionBoxes,
                detections,
                captureDateTime,
                captureEpoch,
                manualPreviewFrame,
                previewTextLabels);
        }

    private:
        QImage m_image;
        CameraHistogramData m_histogramData;
        CameraPipelineStacking m_stack;
        QVector<CameraPipelineStarDetection> m_starDetections;
        CameraPipelinePlateSolve m_plateSolve;
        QVector<QRect> m_motionBoxes;
        QVector<CameraPipelineDetection> m_detections;
        QDateTime m_captureDateTime;
        quint64 m_captureEpoch;
        bool m_manualPreviewFrame;
        QVector<PreviewTextLabel> m_previewTextLabels;

        MsgReportFrame(const QImage& image,
                       const CameraHistogramData& histogramData,
                       const CameraPipelineStacking& stack,
                       const QVector<CameraPipelineStarDetection>& starDetections,
                       const CameraPipelinePlateSolve& plateSolve,
                       const QVector<QRect>& motionBoxes,
                       const QVector<CameraPipelineDetection>& detections,
                       const QDateTime& captureDateTime,
                       quint64 captureEpoch,
                       bool manualPreviewFrame,
                       const QVector<PreviewTextLabel>& previewTextLabels) :
            Message(),
            m_image(image),
            m_histogramData(histogramData),
            m_stack(stack),
            m_starDetections(starDetections),
            m_plateSolve(plateSolve),
            m_motionBoxes(motionBoxes),
            m_detections(detections),
            m_captureDateTime(captureDateTime),
            m_captureEpoch(captureEpoch),
            m_manualPreviewFrame(manualPreviewFrame),
            m_previewTextLabels(previewTextLabels)
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

    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_nextStageQueue;
    AvailableChannelOrFeatureHandler m_availableChannelOrFeatureHandler;
    CameraSettings m_settings;
    CameraPipelineFrame m_lastFrame;
    QDateTime m_captureDateTime;
    bool m_captureActive = false;
    quint64 m_captureEpoch = 0;
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
    [[nodiscard]] QImage applyPostProcessing(
        const CameraPipelineFrame& frame,
        bool drawPreviewText = true,
        QVector<PreviewTextLabel> *previewTextLabels = nullptr);
    void applyMotionOverlay(QImage& image, const QVector<QRect>& motionBoxes) const;
    void applyDetectionOverlay(QImage& image, const QVector<CameraPipelineDetection>& detections, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyStarOverlay(QImage& image, const QVector<CameraPipelineStarDetection>& starDetections, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyPreviewTextLabels(QImage& image, const QVector<PreviewTextLabel>& labels) const;
    void applySpectrumOverlay(QImage& image) const;
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    void applySkyGridOverlay(QImage& image, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyConstellationOverlay(QImage& image) const;
    void applyTrackedObjectOverlay(QImage& image, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyDateTimeOverlay(QImage& image, bool drawLabel, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyTextOverlay(QImage& image, QTextDocument& overlayTextDocument) const;
    [[nodiscard]] QString expandOverlayTextTemplate() const;
    void updateTrackedMapObject(const QObject* pipeSource, SWGSDRangel::SWGMapItem* swgMapItem);
    void restartWeatherUpdates();
    void reportFrameToGUI(const QImage& image, const CameraPipelineFrame& frame, const QVector<PreviewTextLabel>& previewTextLabels = {});
private slots:
    void handleInputMessages();
    void handlePipeMessageQueue(MessageQueue* messageQueue);
    void processNextFrame();
    void weatherUpdated(float temperature, float pressure, float humidity, float cloudiness, float windSpeed, float windDirection);

};

#endif // INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_
