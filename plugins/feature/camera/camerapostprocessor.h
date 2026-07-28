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
#include <QSet>
#include <QStringList>
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
        bool m_ellipse = false;
    };

    struct WindowOverlayFrame
    {
        QImage m_image;
        int m_offsetX = 0;
        int m_offsetY = 0;
        double m_scale = 1.0;
    };

    class MsgSpectrumFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QImage& getImage() const { return m_image; }
        const QString& getSourceId() const { return m_sourceId; }

        static MsgSpectrumFrame* create(const QString& sourceId, const QImage& image)
        {
            return new MsgSpectrumFrame(sourceId, image);
        }

    private:
        QString m_sourceId;
        QImage m_image;

        MsgSpectrumFrame(const QString& sourceId, const QImage& image) :
            Message(),
            m_sourceId(sourceId),
            m_image(image)
        { }
    };

    class MsgWindowOverlayFrames : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QVector<WindowOverlayFrame>& getFrames() const { return m_frames; }

        static MsgWindowOverlayFrames* create(const QVector<WindowOverlayFrame>& frames)
        {
            return new MsgWindowOverlayFrames(frames);
        }

    private:
        QVector<WindowOverlayFrame> m_frames;

        MsgWindowOverlayFrames(const QVector<WindowOverlayFrame>& frames) :
            Message(),
            m_frames(frames)
        { }
    };

    class MsgReportFrame : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QImage& getImage() const { return m_image; }
        const CameraHistogramData& getHistogramData() const { return m_histogramData; }
        const CameraOpticalSpectrumData& getOpticalSpectrumData() const { return m_opticalSpectrumData; }
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
        float getPlateSolveTimeMs() const { return m_plateSolve.m_solveTimeMs; }
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
        const QVector<WindowOverlayFrame>& getPreviewImageOverlays() const { return m_previewImageOverlays; }
        const QVector<QRect>& getMotionBoxes() const { return m_motionBoxes; }
        bool isCloudValid() const { return m_cloud.m_valid; }
        float getCloudCoveragePercent() const { return m_cloud.m_coveragePercent; }
        bool isCloudNight() const { return m_cloud.m_night; }
        const QVector<CameraPipelineDetection>& getDetections() const { return m_detections; }
        const QVector<CameraPipelineMeteorPhotometry>& getMeteorPhotometry() const { return m_meteorPhotometry; }
        const QVector<CameraPipelineTrackedObject>& getTrackedObjects() const { return m_trackedObjects; }
        const CameraPipelineThermal& getThermal() const { return m_thermal; }
        const QDateTime& getCaptureDateTime() const { return m_captureDateTime; }
        quint64 getCaptureEpoch() const { return m_captureEpoch; }
        bool isManualPreviewFrame() const { return m_manualPreviewFrame; }

        static MsgReportFrame* create(const QImage& image,
                                      const CameraHistogramData& histogramData,
                                      const CameraOpticalSpectrumData& opticalSpectrumData,
                                      const CameraPipelineStacking& stack,
                                      const QVector<CameraPipelineStarDetection>& starDetections,
                                      const CameraPipelinePlateSolve& plateSolve,
                                      const QVector<QRect>& motionBoxes,
                                      const CameraPipelineCloud& cloud,
                                      const QVector<CameraPipelineDetection>& detections,
                                      const QVector<CameraPipelineMeteorPhotometry>& meteorPhotometry,
                                      const QVector<CameraPipelineTrackedObject>& trackedObjects,
                                      const CameraPipelineThermal& thermal,
                                      const QDateTime& captureDateTime,
                                      quint64 captureEpoch,
                                      bool manualPreviewFrame,
                                      const QVector<PreviewTextLabel>& previewTextLabels,
                                      const QVector<PreviewRectItem>& previewRectItems,
                                      const QVector<WindowOverlayFrame>& previewImageOverlays)
        {
            return new MsgReportFrame(
                image,
                histogramData,
                opticalSpectrumData,
                stack,
                starDetections,
                plateSolve,
                motionBoxes,
                cloud,
                detections,
                meteorPhotometry,
                trackedObjects,
                thermal,
                captureDateTime,
                captureEpoch,
                manualPreviewFrame,
                previewTextLabels,
                previewRectItems,
                previewImageOverlays);
        }

    private:
        QImage m_image;
        CameraHistogramData m_histogramData;
        CameraOpticalSpectrumData m_opticalSpectrumData;
        CameraPipelineStacking m_stack;
        QVector<CameraPipelineStarDetection> m_starDetections;
        CameraPipelinePlateSolve m_plateSolve;
        QVector<QRect> m_motionBoxes;
        CameraPipelineCloud m_cloud;
        QVector<CameraPipelineDetection> m_detections;
        QVector<CameraPipelineMeteorPhotometry> m_meteorPhotometry;
        QVector<CameraPipelineTrackedObject> m_trackedObjects;
        CameraPipelineThermal m_thermal;
        QDateTime m_captureDateTime;
        quint64 m_captureEpoch;
        bool m_manualPreviewFrame;
        QVector<PreviewTextLabel> m_previewTextLabels;
        QVector<PreviewRectItem> m_previewRectItems;
        QVector<WindowOverlayFrame> m_previewImageOverlays;

        MsgReportFrame(const QImage& image,
                       const CameraHistogramData& histogramData,
                       const CameraOpticalSpectrumData& opticalSpectrumData,
                       const CameraPipelineStacking& stack,
                       const QVector<CameraPipelineStarDetection>& starDetections,
                       const CameraPipelinePlateSolve& plateSolve,
                       const QVector<QRect>& motionBoxes,
                       const CameraPipelineCloud& cloud,
                       const QVector<CameraPipelineDetection>& detections,
                       const QVector<CameraPipelineMeteorPhotometry>& meteorPhotometry,
                       const QVector<CameraPipelineTrackedObject>& trackedObjects,
                       const CameraPipelineThermal& thermal,
                       const QDateTime& captureDateTime,
                       quint64 captureEpoch,
                       bool manualPreviewFrame,
                       const QVector<PreviewTextLabel>& previewTextLabels,
                       const QVector<PreviewRectItem>& previewRectItems,
                       const QVector<WindowOverlayFrame>& previewImageOverlays) :
            Message(),
            m_image(image),
            m_histogramData(histogramData),
            m_opticalSpectrumData(opticalSpectrumData),
            m_stack(stack),
            m_starDetections(starDetections),
            m_plateSolve(plateSolve),
            m_motionBoxes(motionBoxes),
            m_cloud(cloud),
            m_detections(detections),
            m_meteorPhotometry(meteorPhotometry),
            m_trackedObjects(trackedObjects),
            m_thermal(thermal),
            m_captureDateTime(captureDateTime),
            m_captureEpoch(captureEpoch),
            m_manualPreviewFrame(manualPreviewFrame),
            m_previewTextLabels(previewTextLabels),
            m_previewRectItems(previewRectItems),
            m_previewImageOverlays(previewImageOverlays)
        { }
    };

    class MsgReportFrameSummary : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QDateTime& getCaptureDateTime() const { return m_captureDateTime; }
        const QStringList& getDetectedObjectClasses() const { return m_detectedObjectClasses; }
        bool getMotionDetected() const { return m_motionDetected; }
        const CameraPipelineThermal& getThermal() const { return m_thermal; }

        static MsgReportFrameSummary* create(const QDateTime& captureDateTime, const QStringList& detectedObjectClasses,
            bool motionDetected, const CameraPipelineThermal& thermal)
        {
            return new MsgReportFrameSummary(captureDateTime, detectedObjectClasses, motionDetected, thermal);
        }

    private:
        QDateTime m_captureDateTime;
        QStringList m_detectedObjectClasses;
        bool m_motionDetected;
        CameraPipelineThermal m_thermal;

        MsgReportFrameSummary(const QDateTime& captureDateTime, const QStringList& detectedObjectClasses,
            bool motionDetected, const CameraPipelineThermal& thermal) :
            Message(),
            m_captureDateTime(captureDateTime),
            m_detectedObjectClasses(detectedObjectClasses),
            m_motionDetected(motionDetected),
            m_thermal(thermal)
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
    void setMessageQueueToFeature(MessageQueue *messageQueue) { m_msgQueueToFeature = messageQueue; }
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
            QSize m_opticalSize;
            double m_opticalToImageM11 = 1.0;
            double m_opticalToImageM12 = 0.0;
            double m_opticalToImageM21 = 0.0;
            double m_opticalToImageM22 = 1.0;
            double m_opticalToImageDx = 0.0;
            double m_opticalToImageDy = 0.0;
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
                    && m_opticalSize == other.m_opticalSize
                    && m_opticalToImageM11 == other.m_opticalToImageM11
                    && m_opticalToImageM12 == other.m_opticalToImageM12
                    && m_opticalToImageM21 == other.m_opticalToImageM21
                    && m_opticalToImageM22 == other.m_opticalToImageM22
                    && m_opticalToImageDx == other.m_opticalToImageDx
                    && m_opticalToImageDy == other.m_opticalToImageDy
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
    MessageQueue *m_msgQueueToFeature;
    MessageQueue *m_nextStageQueue;
    MessageQueue *m_workerInputMessageQueue = nullptr;
    AvailableChannelOrFeatureHandler m_availableChannelOrFeatureHandler;
    QSet<QObject*> m_trackedObjectPipeSources;
    CameraSettings m_settings;
    CameraPipelineFrame m_lastFrame;
    // Recycles the full-frame overlay-composite buffers produced every frame in
    // processNewFrame/applyPostProcessing (the RGB32 convert target and the
    // preview deep-copy). Used only on the post-processor thread; cross-thread
    // release (the GUI holds the preview) is handled by CameraImagePool.
    CameraImagePool m_overlayImagePool;
    QDateTime m_captureDateTime;
    float m_cloudCoveragePercent = std::numeric_limits<float>::quiet_NaN();
    bool m_captureActive = false;
    quint64 m_captureEpoch = 0;
    QHash<QString, QImage> m_spectrumViewImages;
    QVector<WindowOverlayFrame> m_windowOverlayFrames;
    Weather *m_weather = nullptr;
    float m_weatherTemperature = std::numeric_limits<float>::quiet_NaN();
    float m_weatherPressure = std::numeric_limits<float>::quiet_NaN();
    float m_weatherHumidity = std::numeric_limits<float>::quiet_NaN();
    float m_weatherCloudiness = std::numeric_limits<float>::quiet_NaN();
    float m_weatherWindSpeed = std::numeric_limits<float>::quiet_NaN();
    float m_weatherWindDirection = std::numeric_limits<float>::quiet_NaN();
    QHash<QString, TrackedMapObject> m_trackedMapObjects;
    QImage m_trackedObjectHeatMap;
    QVector<float> m_trackedObjectHeatMapDensity;
    QSize m_trackedObjectHeatMapSize;
    CameraPipelineImageTransform m_trackedObjectHeatMapTransform;
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
        QVector<CameraPipelineTrackedObject> *trackedObjects = nullptr,
        bool drawImageOverlays = true);
    void applyMotionOverlay(QImage& image, const QVector<QRect>& motionBoxes, bool drawBoxes, QVector<PreviewRectItem> *previewRectItems) const;
    void applyCloudOverlay(QImage& image, const CameraPipelineCloud& cloud) const;
    void applyDetectionOverlay(QImage& image, const QVector<CameraPipelineDetection>& detections, const QVector<CameraPipelineMeteorPhotometry>& meteorPhotometry, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels, QVector<PreviewRectItem> *previewRectItems) const;
    void applyStarOverlay(QImage& image, const QVector<CameraPipelineStarDetection>& starDetections, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyPreviewRectItems(QImage& image, const QVector<PreviewRectItem>& items) const;
    void applyPreviewTextLabels(QImage& image, const QVector<PreviewTextLabel>& labels) const;
    void applySpectrumOverlay(QImage& image) const;
    void applyWindowOverlays(QImage& image) const;
    void applyDrawingOverlay(QImage& image) const;
    void applyThermalOverlay(const CameraPipelineFrame& frame, QImage& image, bool drawLabels,
        QVector<PreviewTextLabel> *previewTextLabels, QVector<PreviewRectItem> *previewRectItems) const;
    [[nodiscard]] QVector<WindowOverlayFrame> currentImageOverlays() const;
    [[nodiscard]] static QSize overlayCompositionSize(const QImage& image, double scale);
    [[nodiscard]] static QImage normaliseOverlayImageForComposition(const QImage& image, double scale);
    [[nodiscard]] static const QImage& ensureRgb888(const QImage& image, QImage& convertedImage);
    [[nodiscard]] static cv::Mat wrapRgb888Image(const QImage& image);
    void applySkyGridOverlay(const CameraPipelineFrame& frame, QImage& image, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyConstellationOverlay(const CameraPipelineFrame& frame, QImage& image) const;
    void applyMessierOverlay(const CameraPipelineFrame& frame, QImage& image) const;
    void applyTrackedObjectOverlay(const CameraPipelineFrame& frame, QImage& image, bool drawLabels, QVector<PreviewTextLabel> *previewTextLabels, QVector<CameraPipelineTrackedObject> *trackedObjects = nullptr);
    void applyDateTimeOverlay(QImage& image, bool drawLabel, QVector<PreviewTextLabel> *previewTextLabels) const;
    void applyTextOverlay(QImage& image, QTextDocument& overlayTextDocument) const;
    [[nodiscard]] QString expandOverlayTextTemplate(const CameraSettings& settings) const;
    void updateTrackedMapObject(const QObject* pipeSource, SWGSDRangel::SWGMapItem* swgMapItem);
    void updateTrackedObjectPipeRegistration();
    void registerTrackedObjectPipes();
    void unregisterTrackedObjectPipes();
    void restartWeatherUpdates();
    void reportFrameToGUI(const QImage& image, const CameraPipelineFrame& frame, const QVector<PreviewTextLabel>& previewTextLabels = {}, const QVector<PreviewRectItem>& previewRectItems = {}, const QVector<CameraPipelineTrackedObject>& trackedObjects = {});
private slots:
    void handleInputMessages();
    void handlePipeMessageQueue(MessageQueue* messageQueue);
    void handleTrackedObjectSourcesChanged();
    void processNextFrame();
    void weatherUpdated(float temperature, float pressure, float humidity, float cloudiness, float windSpeed, float windDirection);

};

#endif // INCLUDE_FEATURE_CAMERAPOSTPROCESSOR_H_
