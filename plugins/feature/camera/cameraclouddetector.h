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

#include "cameradetector.h"

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
 *       CPU-only for now; a CUDA path (boxFilter/threshold via cv::cuda, as in the star
 *       detector's preprocessing) is a possible follow-up.
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

        static MsgReportCloudCoverage* create(float coveragePercent, bool night)
        {
            return new MsgReportCloudCoverage(coveragePercent, night);
        }

    private:
        float m_coveragePercent;
        bool m_night;

        MsgReportCloudCoverage(float coveragePercent, bool night) :
            Message(),
            m_coveragePercent(coveragePercent),
            m_night(night)
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

    [[nodiscard]] static bool cloudSettingsChanged(const QList<QString>& settingsKeys);
    [[nodiscard]] bool resolveNightMode(const cv::Mat& medianGray, const cv::Mat& evaluationMask);
    void applyCloudDetection(const cv::Mat& bgrMat, const cv::Rect& roi, const cv::Rect& contentRect, CameraPipelineCloud& cloud, cv::Mat* debugMask);
    void invalidateCache();
};

#endif // INCLUDE_FEATURE_CAMERACLOUDDETECTOR_H_
