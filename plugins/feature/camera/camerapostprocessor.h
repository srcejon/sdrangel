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
#include "cameraimagepool.h"

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

    struct PreviewRectItem
    {
        QRectF m_rect;
        QColor m_color;
        double m_lineWidth = 2.0;
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
        const QVector<PreviewRectItem>& getPreviewRectItems() const { return m_previewRectItems; }
        const QVector<QRect>& getMotionBoxes() const { return m_motionBoxes; }
        const QVector<CameraPipelineDetection>& getDetections() const { return m_detections; }
        const QVector<CameraPipelineTrackedObject>& getTrackedObjects() const { return m_trackedObjects; }
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
                                      const QVector<CameraPipelineTrackedObject>& trackedObjects,
                                      const QDateTime& captureDateTime,
                                      quint64 captureEpoch,
                                      bool manualPreviewFrame,
                                      const QVector<PreviewTextLabel>& previewTextLabels,
                                      const QVector<PreviewRectItem>& previewRectItems)
        {
            return new MsgReportFrame(
                image,
                histogramData,
                stack,
                starDetections,
                plateSolve,
                motionBoxes,
                detections,
                trackedObjects,
                captureDateTime,
                captureEpoch,
                manualPreviewFrame,
                previewTextLabels,
                previewRectItems);
        }

    private:
        QImage m_image;
        CameraHistogramData m_histogramData;
        CameraPipelineStacking m_stack;
        QVector<CameraPipelineStarDetection> m_starDetections;
        CameraPipelinePlateSolve m_plateSolve;
        QVector<QRect> m_motionBoxes;
        QVector<CameraPipelineDetection> m_detections;
        QVector<CameraPipelineTrackedObject> m_trackedObjects;
        QDateTime m_captureDateTime;
        quint64 m_captureEpoch;
        bool m_manualPreviewFrame;
        QVector<PreviewTextLabel> m_previewTextLabels;
        QVector<PreviewRectItem> m_previewRectItems;

        MsgReportFrame(const QImage& image,
                       const CameraHistogramData& histogramData,
                       const CameraPipelineStacking& stack,
                       const QVector<CameraPipelineStarDetection>& starDetections,
                       const CameraPipelinePlateSolve& plateSolve,
                       const QVector<QRect>& motionBoxes,
                       const QVector<CameraPipelineDetection>& detections,
                       const QVector<CameraPipelineTrackedObject>& trackedObjects,
                       const QDateTime& captureDateTime,
                       quint64 captureEpoch,
                       bool manualPreviewFrame,
                       const QVector<PreviewTextLabel>& previewTextLabels,
                       const QVector<PreviewRectItem>& previewRectItems) :
            Message(),
            m_image(image),
            m_histogramData(histogramData),
            m_stack(stack),
            m_starDetections(starDetections),
            m_plateSolve(plateSolve),
            m_motionBoxes(motionBoxes),
            m_detections(detections),
            m_trackedObjects(trackedObjects),
            m_captureDateTime(captureDateTime),
            m_captureEpoch(captureEpoch),
            m_manualPreviewFrame(manualPreviewFrame),
            m_previewTextLabels(previewTextLabels),
            m_previewRectItems(previewRectItems)
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

    CameraPostProcessor();
    ~CameraPostProcessor();

    void startWork();
    void stopWork();
    void submitFrame(const CameraPipelineFramePtr& frame);
    MessageQueue *getInputMessageQueue() { return &m_inputMessageQueue; }
    void setMessageQueueToGUI(MessageQueue *messageQueue) { m_msgQueueToGUI = messageQueue; }
    void setNextStageInputMessageQueue(MessageQueue *messageQueue) { m_nextStageQueue = messageQueue; }
    void setWorkerInputMessageQueue(MessageQueue *messageQueue) { m_workerInputMessageQueue = messageQueue; }

private:
    struct TrackedMapObject
    {
        struct TrackPoint
        {
            double m_latitude = 0.0;
            double m_longitude = 0.0;
            double m_altitude = 0.0;
        };

        QString m_name;
        QString m_label;
        double m_latitude = 0.0;
        double m_longitude = 0.0;
        double m_altitude = 0.0;
        QDateTime m_availableUntil;
        QVector<TrackPoint> m_track;
    };

    // Cache for the (expensive) sky-grid line overlay. The grid is thousands of
    // antialiased trig-projected segments that are identical frame-to-frame
    // unless the projection/observer parameters change, so the lines are rendered
    // once into a transparent overlay and re-composited each frame. Only accessed
    // from the post-processor worker thread, so it needs no locking.
    struct SkyGridOverlayCache
    {
        // Quantise the equatorial grid's sidereal-time dependence so the cached
        // overlay is reused for a whole second instead of being re-rendered every
        // frame. The grid moves ~0.004 deg/frame at 30 fps, so a 1 s bucket is
        // visually indistinguishable while removing the per-frame render cost.
        static constexpr qint64 m_equatorialQuantumMs = 1000;

        struct Key
        {
            QSize m_size;
            bool m_drawEquatorial = false;
            bool m_drawAltAz = false;
            QRgb m_altAzColor = 0;
            QRgb m_equatorialColor = 0;
            int m_lensProjection = 0;
            double m_azimuth = 0.0;
            double m_elevation = 0.0;
            double m_roll = 0.0;
            double m_fov = 0.0;
            double m_lensCenterOffsetX = 0.0;
            double m_lensCenterOffsetY = 0.0;
            double m_lensDistortionK1 = 0.0;
            double m_latitude = 0.0;
            double m_longitude = 0.0;
            qint64 m_equatorialTimeBucket = 0;

            bool operator==(const Key& other) const
            {
                return m_size == other.m_size
                    && m_drawEquatorial == other.m_drawEquatorial
                    && m_drawAltAz == other.m_drawAltAz
                    && m_altAzColor == other.m_altAzColor
                    && m_equatorialColor == other.m_equatorialColor
                    && m_lensProjection == other.m_lensProjection
                    && m_azimuth == other.m_azimuth
                    && m_elevation == other.m_elevation
                    && m_roll == other.m_roll
                    && m_fov == other.m_fov
                    && m_lensCenterOffsetX == other.m_lensCenterOffsetX
                    && m_lensCenterOffsetY == other.m_lensCenterOffsetY
                    && m_lensDistortionK1 == other.m_lensDistortionK1
                    && m_latitude == other.m_latitude
                    && m_longitude == other.m_longitude
                    && m_equatorialTimeBucket == other.m_equatorialTimeBucket;
            }
            bool operator!=(const Key& other) const { return !(*this == other); }
        };

        QImage m_overlay;
        Key m_key;
        bool m_valid = false;
    };

    MessageQueue m_inputMessageQueue;
    MessageQueue *m_msgQueueToGUI;
    MessageQueue *m_nextStageQueue;
    MessageQueue *m_workerInputMessageQueue = nullptr;
    AvailableChannelOrFeatureHandler m_availableChannelOrFeatureHandler;
    CameraSettings m_settings;
    CameraPipelineFrame m_lastFrame;
    // Recycles the full-frame overlay-composite buffers produced every frame in
    // processNewFrame/applyPostProcessing (the RGB32 convert target and the
    // preview deep-copy). Used only on the post-processor thread; cross-thread
    // release (the GUI holds the preview) is handled by CameraImagePool.
    CameraImagePool m_overlayImagePool;
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
    QImage m_trackedObjectHeatMap;
    QSize m_trackedObjectHeatMapSize;
    QHash<QString, QPointF> m_trackedObjectHeatMapLastPoints;
    bool m_trackedObjectHeatMapSkipSeed = false;
    QMutex m_frameMutex;
    // Small bounded backlog of frames waiting to be post-processed. The processor
    // averages well under the frame interval, so a short queue lets an occasional
    // processing spike (e.g. the once-per-second equatorial grid re-render) be
    // absorbed instead of costing a frame; a sustained overrun still trims the
    // oldest frame so latency stays bounded.
    static constexpr int m_maxPendingFrames = 3;
    std::deque<CameraPipelineFramePtr> m_pendingFrames;
    // Diagnostic: throttle for logging the submit->GUI display pipeline latency.
    qint64 m_lastPipelineLatencyLogMs = 0;
    bool m_processingFrame;
    mutable SkyGridOverlayCache m_skyGridOverlayCache;
    bool handleMessage(const Message& cmd);
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false);
    void saveCurrentImage();
    void processNewFrame(const CameraPipelineFramePtr& frame);
    [[nodiscard]] QImage applyPostProcessing(
        const CameraPipelineFrame& frame,
        bool drawPreviewText = true,
        QVector<PreviewTextLabel> *previewTextLabels = nullptr,
        QVector<PreviewRectItem> *previewRectItems = nullptr,
        QVector<CameraPipelineTrackedObject> *trackedObjects = nullptr);
    void applyMotionOverlay(QImage& image, const QVector<QRect>& motionBoxes, bool drawBoxes, QVector<PreviewRectItem> *previewRectItems) const;
    void applyDetectionOverlay(QImage& image, const QVector<CameraPipelineDetection>& detections, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels, QVector<PreviewRectItem> *previewRectItems) const;
    void applyStarOverlay(QImage& image, const QVector<CameraPipelineStarDetection>& starDetections, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyPreviewRectItems(QImage& image, const QVector<PreviewRectItem>& items) const;
    void applyPreviewTextLabels(QImage& image, const QVector<PreviewTextLabel>& labels) const;
    void applySpectrumOverlay(QImage& image) const;
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    void applySkyGridOverlay(QImage& image, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyConstellationOverlay(QImage& image) const;
    void applyTrackedObjectOverlay(QImage& image, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels, QVector<CameraPipelineTrackedObject> *trackedObjects = nullptr);
    void applyDateTimeOverlay(QImage& image, bool drawLabel, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyTextOverlay(QImage& image, QTextDocument& overlayTextDocument) const;
    [[nodiscard]] QString expandOverlayTextTemplate() const;
    void updateTrackedMapObject(const QObject* pipeSource, SWGSDRangel::SWGMapItem* swgMapItem);
    void restartWeatherUpdates();
    void reportFrameToGUI(const QImage& image, const CameraPipelineFrame& frame, const QVector<PreviewTextLabel>& previewTextLabels = {}, const QVector<PreviewRectItem>& previewRectItems = {}, const QVector<CameraPipelineTrackedObject>& trackedObjects = {});
private slots:
    void handleInputMessages();
    void handlePipeMessageQueue(MessageQueue* messageQueue);
    void processNextFrame();
    void weatherUpdated(float temperature, float pressure, float humidity, float cloudiness, float windSpeed, float windDirection);

};

#endif // INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_
