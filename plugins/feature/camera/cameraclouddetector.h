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

#ifndef INCLUDE_FEATURE_CAMERACLOUDDETECTOR_H_
#define INCLUDE_FEATURE_CAMERACLOUDDETECTOR_H_

#include <algorithm>

#include <QHash>

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
#include <opencv2/cudafilters.hpp>
#endif

#include "cameraclearskyreference.h"
#include "cameraplatesolver.h"

#include "cameradetector.h"

/**
 * \brief Tracks cloud coverage against the event threshold and decides when to emit
 * Scheduler coverage-high/low events.
 *
 * The state starts unknown at capture start, so the first coverage report emits an event
 * describing the initial sky state (an already-overcast sky fires High immediately).
 * Thereafter events fire only on transitions, with a hysteresis band below the threshold
 * so coverage hovering around it does not chatter. The band shrinks with a low threshold
 * (half of it at most): a fixed band would put the Low transition below 0 % coverage for
 * thresholds under the band width, making Low unreachable once High had fired.
 */
struct CameraCloudEventTracker
{
    enum Event { None, High, Low };

    static constexpr double m_hysteresisPercent = 10.0;

    void reset() { m_state = StateUnknown; }

    Event update(double coveragePercent, double thresholdPercent)
    {
        // Cap the hysteresis at half the threshold so the Low transition always sits at a
        // reachable coverage; the full band below a small threshold would sit below 0 %
        const double hysteresis = std::min(m_hysteresisPercent, thresholdPercent / 2.0);
        switch (m_state)
        {
        case StateUnknown:
            m_state = (coveragePercent >= thresholdPercent) ? StateHigh : StateLow;
            return (m_state == StateHigh) ? High : Low;
        case StateLow:
            if (coveragePercent >= thresholdPercent)
            {
                m_state = StateHigh;
                return High;
            }
            return None;
        case StateHigh:
        default:
            if (coveragePercent <= thresholdPercent - hysteresis)
            {
                m_state = StateLow;
                return Low;
            }
            return None;
        }
    }

private:
    enum State { StateUnknown, StateLow, StateHigh };
    State m_state = StateUnknown;
};

/**
 * \brief Detection stage that segments clouds into a per-pixel mask and coverage percentage.
 *
 * Classifies the detection ROI of each frame as cloud/clear using one of two classical paths:
 * at night, clouds appear as low-frequency brightness structure against an otherwise smooth sky,
 * so a heavily blurred background estimate is compared against a robust global sky level and the
 * absolute deviation is thresholded; by day, cloud is white/grey against blue sky, so a per-pixel
 * red/blue ratio is thresholded. Auto mode picks the path from overall frame brightness with
 * hysteresis. The result is written into CameraPipelineFrame::m_cloud before forwarding, so
 * downstream stages can optionally suppress false star detections and cloud-drift motion boxes.
 *
 * \note Clouds evolve slowly, so the mask is recomputed only every
 *       m_cloudUpdateIntervalFrames frames (or on ROI/size/settings changes); intermediate
 *       frames are stamped with the cached result via a shallow cv::Mat copy. Downstream stages
 *       must treat the shared mask as read-only.
 * \note When output scaling pads the image inside a larger canvas, the frame's image transform
 *       records where the real content sits; the padded borders are excluded from
 *       classification, the coverage denominator, the sky-level median and the auto day/night
 *       brightness decision.
 * \note Derives from CameraDetectionStage and runs on its own QThread; see that base class for
 *       threading, frame-backlog and ROI/exclusion-mask behaviour. Whenever a fresh mask is
 *       computed a MsgReportCloudCoverage is pushed to the feature for GUI/WebAPI reporting.
 * \note The only full-resolution work is the prepare step (ROI crop, downscale, luminance,
 *       median blur); classification then runs on the small downscaled images. When
 *       CAMERA_OPENCV_CUDA_CLOUD_DETECTION is defined and the frame carries a GPU-resident BGR
 *       image, the prepare step runs on the GPU so the full-resolution frame never has to be
 *       downloaded to the CPU; only the downscaled work images are. Classification is shared
 *       between the two paths, so CPU and CUDA results differ only by resize/filter rounding.
 */
class CameraCloudDetector : public CameraDetectionStage
{
    Q_OBJECT
public:
    class MsgReportCloudCoverage : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        float getCoveragePercent() const { return m_coveragePercent; }
        bool isNight() const { return m_night; }
        const QDateTime& getCaptureDateTime() const { return m_captureDateTime; }

        static MsgReportCloudCoverage* create(float coveragePercent, bool night, const QDateTime& captureDateTime)
        {
            return new MsgReportCloudCoverage(coveragePercent, night, captureDateTime);
        }

    private:
        float m_coveragePercent;
        bool m_night;
        QDateTime m_captureDateTime;

        MsgReportCloudCoverage(float coveragePercent, bool night, const QDateTime& captureDateTime) :
            Message(),
            m_coveragePercent(coveragePercent),
            m_night(night),
            m_captureDateTime(captureDateTime)
        { }
    };

    // GUI/API request: save the current frame plus everything needed to reproduce this
    // detection offline (full settings, capture time, clear-sky reference) as a standalone
    // test-case bundle in the given directory (runnable via the test harness --run-case)
    class MsgSaveCloudTestCase : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QString& getDirectory() const { return m_directory; }

        static MsgSaveCloudTestCase* create(const QString& directory)
        {
            return new MsgSaveCloudTestCase(directory);
        }

    private:
        QString m_directory;

        explicit MsgSaveCloudTestCase(const QString& directory) :
            Message(),
            m_directory(directory)
        { }
    };

    // GUI/API request: capture the current frame as the clear-sky reference for the
    // current sky state (sun/moon elevation at the frame's observation time)
    class MsgSaveClearSkyReference : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgSaveClearSkyReference* create() { return new MsgSaveClearSkyReference(); }

    private:
        MsgSaveClearSkyReference() : Message() { }
    };

    // GUI request: delete this camera's clear-sky reference store (recovery from a
    // reference that learned cloud as clear sky)
    class MsgClearClearSkyReference : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        static MsgClearClearSkyReference* create() { return new MsgClearClearSkyReference(); }

    private:
        MsgClearClearSkyReference() : Message() { }
    };

    // Report to the feature/GUI: clear-sky reference store status after a save or
    // auto-learn update ("3/7 refs ... - current: Dark [tick]")
    class MsgReportClearSkyReference : public Message {
        MESSAGE_CLASS_DECLARATION

    public:
        const QString& getSummary() const { return m_summary; }

        static MsgReportClearSkyReference* create(const QString& summary)
        {
            return new MsgReportClearSkyReference(summary);
        }

    private:
        QString m_summary;

        explicit MsgReportClearSkyReference(const QString& summary) :
            Message(),
            m_summary(summary)
        { }
    };

    CameraCloudDetector();
    ~CameraCloudDetector() override;
    void setMessageQueueToFeature(MessageQueue *messageQueue) { m_msgQueueToFeature = messageQueue; }

    // Storage key identifying which camera a clear-sky reference belongs to; also used by
    // the GUI's reference viewer to open the same store
    [[nodiscard]] static QString referenceStorageKey(const CameraSettings& settings);

protected:
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false) override;
    void captureActiveChanged(bool active) override;
    void processNewFrame(const CameraPipelineFramePtr& frame) override;
    bool handleStageMessage(const Message& cmd) override;

private:
    MessageQueue *m_msgQueueToFeature;
    CameraPipelineFramePtr m_lastInputFrame;
    CameraPipelineCloud m_lastCloud;
    cv::Mat m_lastDebugMask;   // Debug view of the last recompute, re-rendered onto intermediate frames
    int m_framesSinceUpdate;
    QSize m_lastFrameSize;
    cv::Rect m_lastContentRect;
    bool m_autoNight;          // Auto-mode day/night decision, kept between frames for hysteresis
    bool m_haveAutoModeState;
    CameraClearSkyReference m_clearSkyReference;
    bool m_saveReferencePending;
    QString m_saveTestCaseDir; // Non-empty: write a test-case bundle here on the next recompute
    // Cached minimum-elevation keep-mask (255 = at or above the configured elevation).
    // Rebuilt when the geometry it was computed for changes; settings changes (lens pose,
    // FoV, the elevation itself) invalidate it through invalidateCache().
    // The debug view rendered at the last recompute, reused verbatim on the intermediate
    // frames that carry the cached mask: the render is a full-resolution canvas fill,
    // colour conversion and QImage build, and its result cannot change while the mask
    // does not. QImage is implicitly shared, so handing it to each frame is free.
    QImage m_lastDebugImage;
    cv::Mat m_elevationKeepMask;
    cv::Rect m_elevationMaskRoi;
    cv::Size m_elevationMaskWorkSize;
    QSize m_elevationMaskImageSize;
    // When each predicted star was last actually detected, keyed by catalog index. The
    // star-blank cue only trusts the DISAPPEARANCE of a recently seen star: on real frames
    // most predicted stars fail detection even under a clear sky (twilight, faintness,
    // lens softness), so absence alone is meaningless. Cleared with the cache, so a pose
    // or settings change never carries sightings across geometries.
    QHash<int, QDateTime> m_starLastVisible;
    // The bright-star catalog filtered to the sensing magnitude. The plate solver caches
    // the catalog itself, but hands out a freshly built vector of every entry (order 1e5)
    // on each call, of which star sensing wants a few hundred. Rebuilt when the magnitude
    // changes and dropped by invalidateCache(), so a catalog downloaded mid-session is
    // picked up at the next settings change or capture start.
    mutable QVector<CameraPlateSolver::BrightStar> m_sensedStarCatalog;
    mutable double m_sensedStarCatalogMagnitude = -1.0;

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
    cv::cuda::Stream m_cudaCloudStream;
    cv::Ptr<cv::cuda::Filter> m_cudaCloudMedianFilter;

    [[nodiscard]] bool canUseCudaCloudDetection() const;
    bool prepareWorkImagesCuda(const cv::cuda::GpuMat& bgrGpu, const cv::Rect& roi, cv::Mat& workBgr, cv::Mat& rawGray, cv::Mat& gray);
#endif
    // Star-visibility sensing: predicted catalog stars checked for a point-source peak in
    // the full-resolution frame, used to veto cloud-mask components that stars shine through
    struct CloudStarSense
    {
        struct Star
        {
            QPointF position; // full-image coordinates
            bool visible;
            float magnitude;    // catalog magnitude, for diagnostics
            int catalogIndex;   // stable index into the bright-star catalog, keys the last-seen record
        };
        QVector<Star> stars;
        bool valid = false;
    };

    // Geometric visibility of the sun's glare at its projected position: Visible proves that
    // line of sight is clear, Obscured proves cloud in front of the sun, Unknown means the
    // check could not run (no time, sun low or outside the frame)
    enum class BodyVisibility { Unknown, Visible, Obscured };

    [[nodiscard]] static bool cloudSettingsChanged(const QList<QString>& settingsKeys);
    [[nodiscard]] bool resolveNightMode(const cv::Mat& medianGray, const cv::Mat& evaluationMask, const QDateTime& captureDateTime);
    void prepareWorkImages(const QImage& image, const cv::Rect& roi, cv::Mat& workBgr, cv::Mat& rawGray, cv::Mat& gray) const;
    void applyCloudDetection(const cv::Mat& workBgr, const cv::Mat& rawGray, const cv::Mat& gray, const cv::Rect& roi, const cv::Rect& contentRect, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime, const CloudStarSense& starSense, CameraPipelineCloud& cloud, cv::Mat* debugMask);
    void applySunMoonMask(cv::Mat& mask, cv::Mat& evaluationMask, const cv::Mat& gray, const cv::Rect& roi, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime) const;
    cv::Mat structureContrastMask(const cv::Mat& gray, const cv::Mat& evaluationMask, bool requirePixelContrast, cv::Mat* debugMask) const;
    [[nodiscard]] CloudStarSense senseStarVisibility(const CameraPipelineFramePtr& frame, const QSize& imageSize) const;
    // fullResBgr, when non-empty, supplies the pixels instead of the frame: the GPU path
    // otherwise pays a synchronous download per star
    static bool samplePatchGray(const CameraPipelineFramePtr& frame, const QPoint& centre, int half, cv::Mat& patch,
                                const cv::Mat& fullResBgr = cv::Mat());
    [[nodiscard]] const QVector<CameraPlateSolver::BrightStar>& sensedStarCatalog() const;
    void applyStarVisibilityVeto(cv::Mat& mask, const CloudStarSense& starSense, const cv::Rect& roi) const;
    void applyStarBlankCue(cv::Mat& mask, const cv::Mat& evaluationMask, const CloudStarSense& starSense, const cv::Rect& roi, const QSize& imageSize, const QDateTime& observationTime) const;
    void recordStarVisibility(const CloudStarSense& starSense, const QDateTime& observationTime);
    [[nodiscard]] BodyVisibility sunVisibility(const cv::Mat& gray, const cv::Rect& roi, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime) const;
    void applyMinElevationMask(cv::Mat& evaluationMask, const cv::Rect& roi, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform);
    void renderDebugView(const CameraPipelineFramePtr& frame, const cv::Size& frameCvSize, const cv::Rect& roi);
    void saveTestCaseBundle(const CameraPipelineFramePtr& frame);
    void invalidateCache();
};

#endif // INCLUDE_FEATURE_CAMERACLOUDDETECTOR_H_
