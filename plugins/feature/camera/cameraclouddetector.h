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

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
#include <opencv2/cudafilters.hpp>
#endif

#include "cameradetector.h"

/**
 * \brief Tracks cloud coverage against the event threshold and decides when to emit
 * Scheduler coverage-high/low events.
 *
 * The state starts unknown at capture start, so the first coverage report emits an event
 * describing the initial sky state (an already-overcast sky fires High immediately).
 * Thereafter events fire only on transitions, with a fixed hysteresis band below the
 * threshold so coverage hovering around it does not chatter.
 */
struct CameraCloudEventTracker
{
    enum Event { None, High, Low };

    static constexpr double m_hysteresisPercent = 10.0;

    void reset() { m_state = StateUnknown; }

    Event update(double coveragePercent, double thresholdPercent)
    {
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
            if (coveragePercent <= thresholdPercent - m_hysteresisPercent)
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

    CameraCloudDetector();
    ~CameraCloudDetector() override;
    void setMessageQueueToFeature(MessageQueue *messageQueue) { m_msgQueueToFeature = messageQueue; }

protected:
    void applySettings(const CameraSettings& settings, const QList<QString>& settingsKeys, bool force = false) override;
    void captureActiveChanged(bool active) override;
    void processNewFrame(const CameraPipelineFramePtr& frame) override;

private:
    MessageQueue *m_msgQueueToFeature;
    CameraPipelineFramePtr m_lastInputFrame;
    CameraPipelineCloud m_lastCloud;
    int m_framesSinceUpdate;
    QSize m_lastFrameSize;
    cv::Rect m_lastContentRect;
    bool m_autoNight;          // Auto-mode day/night decision, kept between frames for hysteresis
    bool m_haveAutoModeState;

#ifdef CAMERA_OPENCV_CUDA_CLOUD_DETECTION
    cv::cuda::Stream m_cudaCloudStream;
    cv::Ptr<cv::cuda::Filter> m_cudaCloudMedianFilter;

    [[nodiscard]] bool canUseCudaCloudDetection() const;
    bool prepareWorkImagesCuda(const cv::cuda::GpuMat& bgrGpu, const cv::Rect& roi, cv::Mat& workBgr, cv::Mat& rawGray, cv::Mat& gray);
#endif
    [[nodiscard]] static bool cloudSettingsChanged(const QList<QString>& settingsKeys);
    [[nodiscard]] bool resolveNightMode(const cv::Mat& medianGray, const cv::Mat& evaluationMask, const QDateTime& captureDateTime);
    void prepareWorkImages(const cv::Mat& bgrMat, const cv::Rect& roi, cv::Mat& workBgr, cv::Mat& rawGray, cv::Mat& gray) const;
    void applyCloudDetection(const cv::Mat& workBgr, const cv::Mat& rawGray, const cv::Mat& gray, const cv::Rect& roi, const cv::Rect& contentRect, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime, CameraPipelineCloud& cloud, cv::Mat* debugMask);
    void applySunMoonMask(cv::Mat& evaluationMask, const cv::Rect& roi, const QSize& imageSize, const CameraPipelineImageTransform& imageTransform, const QDateTime& captureDateTime) const;
    void invalidateCache();
};

#endif // INCLUDE_FEATURE_CAMERACLOUDDETECTOR_H_
