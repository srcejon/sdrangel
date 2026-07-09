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
// GNU General Public License V3 for more details.                              //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_FEATURE_CAMERACLEARSKYREFERENCE_H_
#define INCLUDE_FEATURE_CAMERACLEARSKYREFERENCE_H_

#include <QDateTime>
#include <QImage>
#include <QRectF>
#include <QString>

#include <opencv2/core.hpp>

/**
 * \brief Per-camera clear-sky reference model for the cloud detector.
 *
 * Stores what this camera's clear sky looks like - per-pixel brightness and red/blue
 * colour maps at a fixed reference resolution - in a handful of slots keyed by the sky
 * state (sun-elevation band x moon up/down), since the clear sky's appearance changes
 * continuously from day through twilight to dark and with moonlight. At classification
 * time the frame is compared against the matching slot after normalising both by their
 * own robust anchors (absorbing gain/exposure changes), yielding two per-pixel uses:
 *
 *  - Veto: pixels matching the reference within a tolerance tighter than any detection
 *    margin are clear sky whatever the in-frame cues say. This retires static false
 *    positives (glow pockets, rim artifacts) permanently.
 *  - Cue: pixels standing above the reference in brightness or shifted toward white in
 *    colour are cloud, catching deviations the in-frame heuristics cannot anchor.
 *
 * The comparison judges the residual after removing a fitted low-order surface from the
 * deviation: twilight and moon glow evolve within minutes as smooth gradients, and without
 * this a reference only held for the minutes around its capture. The consequence, shared
 * with the per-frame anchoring, is deliberate blindness to smooth whole-sky changes
 * (uniform or gradient haze would need absolute exposure metadata); the model's value is
 * spatial: quirks, patchy cloud, and localised structure.
 *
 * The model also derives a static foreground mask (trees, roofs, window frames) from the
 * captured data: foreground silhouettes are darker than the night sky's airglow in the
 * dark slots and finely textured in the day slot. Foreground is excluded from cloud
 * evaluation automatically, replacing hand-drawn exclusion rectangles for fixed cameras.
 *
 * References persist on disk per camera (keyed by camera id) under the application data
 * directory, overridable via SDRANGEL_CAMERA_CLEARSKY_DIR (used by the test harness).
 *
 * \note Not thread-safe; owned and used by the cloud detector on its own thread.
 */
class CameraClearSkyReference
{
public:
    // Day plus nine 2-degree sun-elevation bins from -4 down to below -20, each split by
    // moon above/below 5 degrees. The bins are deliberately fine: at solstice latitudes one
    // broad twilight band spans a 4x sky-brightness range, so a reference saved late in the
    // band would overwrite one still needed for the earlier conditions.
    static constexpr int kNightBins = 9;
    static constexpr int kSlotCount = 1 + 2 * kNightBins;

    [[nodiscard]] static int slotFor(double sunElevationDeg, double moonElevationDeg);
    [[nodiscard]] static QString slotName(int slot);

    // The slot encoding, defined once: slot 0 is Day; night slots are 1 + 2*bin + moon.
    // All bin/moon arithmetic must go through these so the encoding has a single source.
    [[nodiscard]] static constexpr bool slotIsNight(int slot) { return (slot >= 1) && (slot < kSlotCount); }
    [[nodiscard]] static constexpr int slotFromBin(int bin, bool moonUp) { return 1 + 2 * bin + (moonUp ? 1 : 0); }
    [[nodiscard]] static constexpr int slotBin(int slot) { return (slot - 1) / 2; }
    [[nodiscard]] static constexpr bool slotMoonUp(int slot) { return ((slot - 1) % 2) != 0; }

    // Points the store at the camera identified by the settings' camera id, loading its
    // saved references when the camera changes. Call before any other member.
    void ensureLoaded(const QString& cameraId);

    // Captures the current frame as the clear-sky reference for the slot. gray is the
    // median-blurred work luminance, workBgr the work-resolution BGR image, texture the
    // fine-texture energy, evaluationMask the evaluated-sky mask; roiNorm is the detection
    // ROI normalised to the frame so a later ROI change invalidates the slot.
    void capture(int slot, const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& texture, const cv::Mat& evaluationMask, const QRectF& roiNorm, const QDateTime& when,
                 double sunElevationDeg = 0.0, double moonElevationDeg = 0.0);

    // Applies the reference comparison to the thresholded cloud mask (cue OR, veto AND).
    // Returns false when it abstained: empty/mismatched slot, or the frame globally
    // disagrees with the reference (exposure regime change).
    bool applyCueAndVeto(int slot, cv::Mat& mask, const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& evaluationMask, const QRectF& roiNorm, int nightThreshold) const;

    // Auto-learning: blends a verified-clear frame into its slot (first fill copies).
    // Gated on measured coverage, star-sensing confirmation at night (when star sensing
    // is enabled), and a per-slot time throttle. When star sensing strongly confirms a
    // clear sky (most predicted stars visible everywhere), learning is allowed even at
    // high measured coverage - measured coverage may be exactly the false positives an
    // empty slot cannot yet veto. Returns true when the slot was updated.
    bool autoLearn(int slot, const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& texture, const cv::Mat& evaluationMask, const QRectF& roiNorm, const QDateTime& when,
                   float coveragePercent, bool night, bool starSenseEnabled, int starsExpected, int starsVisible);

    // Static foreground mask (255 = foreground) derived from the darkest filled night
    // slot's silhouettes and the day slot's fine texture, resized to workSize. Empty when
    // insufficient data or when the derivation looks implausible (covers most of the sky).
    [[nodiscard]] cv::Mat foregroundMask(const cv::Size& workSize, const QRectF& roiNorm) const;

    [[nodiscard]] bool slotFilled(int slot) const;
    [[nodiscard]] QString statusSummary(int activeSlot) const;

    // Human-checkable renders of a slot's stored maps, for the reference viewer dialog
    struct SlotPreview
    {
        QImage brightness; // reconstructed clear-sky luminance
        QImage ratio;      // red/blue ratio, 1.0 mapped to mid-grey
        QImage texture;    // fine-texture energy
        QImage sky;        // evaluated-sky mask at capture
        QString info;      // capture time, update count, anchors
        bool valid = false;
    };
    [[nodiscard]] SlotPreview slotPreview(int slot) const;
    [[nodiscard]] QImage foregroundPreview() const; // derived foreground mask; null when none

    // Copies the reference into a test-case bundle directory (canonical file name), so the
    // bundle is a standalone clear-sky store; false when nothing is saved yet
    bool exportTo(const QString& directory) const;

private:
    struct Slot
    {
        cv::Mat brightness;   // CV_32F refSize: box-smoothed luminance / anchor
        cv::Mat ratio;        // CV_32F refSize: red / (blue + 1)
        cv::Mat texture;      // CV_32F refSize: fine-texture energy
        cv::Mat sky;          // CV_8U refSize: evaluated-sky mask at capture
        double brightnessAnchor = 0.0; // robust sky median luminance at capture
        double ratioAnchor = 0.0;      // clear-sky (low percentile) ratio at capture
        double sunElevation = 0.0;     // sky state at capture, for diagnostics
        double moonElevation = 0.0;
        QRectF roiNorm;
        QDateTime updated;
        int updateCount = 0;

        [[nodiscard]] bool valid() const { return !brightness.empty(); }
    };

    [[nodiscard]] const Slot* usableSlot(int slot, const QRectF& roiNorm, double frameAnchor) const;
    [[nodiscard]] static QString storageDir();
    [[nodiscard]] QString storageFileName() const;
    [[nodiscard]] QString storagePath() const;
    void load();
    void save() const;
    bool saveToPath(const QString& path) const;
    [[nodiscard]] static bool roiNormsMatch(const QRectF& a, const QRectF& b);
    [[nodiscard]] static bool roiMatches(const Slot& slot, const QRectF& roiNorm);
    static void buildMaps(const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& texture, const cv::Mat& evaluationMask,
                          cv::Mat& brightnessOut, cv::Mat& ratioOut, cv::Mat& textureOut, cv::Mat& skyOut,
                          double& brightnessAnchorOut, double& ratioAnchorOut);

    QString m_cameraId;
    bool m_loaded = false;
    Slot m_slots[kSlotCount];
    mutable cv::Mat m_foregroundCache; // at reference resolution; invalidated on slot updates
    mutable QRectF m_foregroundRoiNorm; // ROI of the slot the cache was derived from, for the geometry gate
    mutable bool m_foregroundDirty = true;
};

#endif // INCLUDE_FEATURE_CAMERACLEARSKYREFERENCE_H_
