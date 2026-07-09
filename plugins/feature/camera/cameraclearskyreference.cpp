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

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include <QCryptographicHash>
#include <QDataStream>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QSaveFile>
#include <QStandardPaths>

#include <opencv2/imgproc.hpp>

#include "cameraclearskyreference.h"

namespace {

// All comparison happens at this fixed resolution, independent of the detector's work size,
// so references survive downscale-setting changes and stay small on disk
constexpr int kRefSize = 192;
// Box-smoothing radius applied to luminance at reference resolution before any comparison,
// so per-pixel sensor noise cannot defeat the tight veto tolerances on dark skies
constexpr int kSmoothRadius = 2;

// Veto: a pixel matching the reference this closely is clear sky whatever the in-frame cues
// say. Tolerances sit below every detection margin (moonlit ratio margin 0.10, night
// threshold 8) so cloud strong enough to detect is never vetoed.
constexpr double kVetoBrightness = 0.08;    // relative to the sky anchor
constexpr double kVetoAbsBrightness = 6.0;  // 8-bit units; rescues dark skies where the relative tolerance is tighter than noise
constexpr double kVetoRatio = 0.06;

// Cue: deviation from the reference strong enough to flag as cloud on its own
constexpr double kCueBrightness = 0.18;     // relative to the sky anchor
constexpr double kCueRatio = 0.12;

// Frame-level sanity: if the sky disagrees with the reference this much at the median, the
// exposure/white-balance regime changed and the comparison abstains entirely
constexpr double kAbstainMedianDev = 0.35;

// Below this luminance the red/blue ratio is quantisation noise and only brightness is used
constexpr int kDarkPixel = 24;

// Auto-learning gates
constexpr float kAutoLearnMaxCoverageNight = 5.0f; // a camera's static quirks may measure a few percent; genuine overcast is far above
constexpr float kAutoLearnMaxCoverageDay = 2.0f;   // day clear-sky detection is reliable, so be stricter against learning thin haze
constexpr qint64 kAutoLearnThrottleSecs = 600;
constexpr double kAutoLearnAlpha = 0.15;

// Foreground derivation
constexpr double kForegroundDarkLevel = 24.0;  // absolute 8-bit level below which a dark-slot pixel is silhouette, not sky
constexpr double kForegroundTexture = 8.0;     // day-slot fine-texture level above which a pixel is foliage/structure
constexpr double kForegroundMaxFraction = 0.45; // a derivation covering most of the frame is wrong; ignore it

constexpr quint32 kFileMagic = 0x43535231; // "CSR1"

int maskedPercentile8U(const cv::Mat& values, const cv::Mat& mask, double fraction)
{
    int histogram[256] = {0};
    int total = 0;
    for (int row = 0; row < values.rows; ++row)
    {
        const uchar *valueLine = values.ptr<uchar>(row);
        const uchar *maskLine = mask.ptr<uchar>(row);
        for (int col = 0; col < values.cols; ++col)
        {
            if (maskLine[col])
            {
                ++histogram[valueLine[col]];
                ++total;
            }
        }
    }

    int remaining = std::max(1, static_cast<int>(std::ceil(std::clamp(fraction, 0.0, 1.0) * total)));
    for (int bin = 0; bin < 256; ++bin)
    {
        remaining -= histogram[bin];
        if (remaining <= 0) {
            return bin;
        }
    }

    return 0;
}

float maskedPercentile32F(const cv::Mat& values, const cv::Mat& mask, double fraction)
{
    std::vector<float> samples;
    samples.reserve(static_cast<size_t>(values.total()));
    for (int row = 0; row < values.rows; ++row)
    {
        const float *valueLine = values.ptr<float>(row);
        const uchar *maskLine = mask.ptr<uchar>(row);
        for (int col = 0; col < values.cols; ++col)
        {
            if (maskLine[col]) {
                samples.push_back(valueLine[col]);
            }
        }
    }

    if (samples.empty()) {
        return 0.0f;
    }

    const size_t index = static_cast<size_t>(std::clamp(fraction, 0.0, 1.0) * (samples.size() - 1));
    std::nth_element(samples.begin(), samples.begin() + index, samples.end());
    return samples[index];
}

void writeMat(QDataStream& stream, const cv::Mat& mat)
{
    stream << static_cast<qint32>(mat.rows) << static_cast<qint32>(mat.cols) << static_cast<qint32>(mat.type());
    if (!mat.empty())
    {
        cv::Mat continuous = mat.isContinuous() ? mat : mat.clone();
        stream.writeRawData(reinterpret_cast<const char*>(continuous.data),
            static_cast<int>(continuous.total() * continuous.elemSize()));
    }
}

bool readMat(QDataStream& stream, cv::Mat& mat)
{
    qint32 rows = 0, cols = 0, type = 0;
    stream >> rows >> cols >> type;
    if ((rows < 0) || (cols < 0) || (rows > 4096) || (cols > 4096)) {
        return false;
    }
    if ((rows == 0) || (cols == 0))
    {
        mat = cv::Mat();
        return true;
    }
    mat.create(rows, cols, type);
    const int bytes = static_cast<int>(mat.total() * mat.elemSize());
    return stream.readRawData(reinterpret_cast<char*>(mat.data), bytes) == bytes;
}

} // namespace

int CameraClearSkyReference::slotFor(double sunElevationDeg, double moonElevationDeg)
{
    if (sunElevationDeg >= -4.0) {
        return 0; // Day (matches the detector's day/night boundary)
    }
    int band;
    if (sunElevationDeg >= -12.0) {
        band = 0; // Twilight (civil + early nautical)
    } else if (sunElevationDeg >= -18.0) {
        band = 1; // Deep twilight
    } else {
        band = 2; // Dark
    }
    const bool moonUp = moonElevationDeg > 5.0;
    return 1 + 2 * band + (moonUp ? 1 : 0);
}

QString CameraClearSkyReference::slotName(int slot)
{
    switch (slot)
    {
    case 0: return QStringLiteral("Day");
    case 1: return QStringLiteral("Twilight");
    case 2: return QStringLiteral("Twilight+moon");
    case 3: return QStringLiteral("Deep twilight");
    case 4: return QStringLiteral("Deep twilight+moon");
    case 5: return QStringLiteral("Dark");
    case 6: return QStringLiteral("Dark+moon");
    default: return QStringLiteral("?");
    }
}

QString CameraClearSkyReference::storageDir()
{
    const QByteArray overrideDir = qgetenv("SDRANGEL_CAMERA_CLEARSKY_DIR");
    if (!overrideDir.isEmpty()) {
        return QString::fromLocal8Bit(overrideDir);
    }
    const QString baseDir = QStandardPaths::standardLocations(QStandardPaths::AppDataLocation).value(0);
    return QDir(baseDir).filePath(QStringLiteral("camera/clearsky"));
}

QString CameraClearSkyReference::storagePath() const
{
    const QByteArray hash = QCryptographicHash::hash(m_cameraId.toUtf8(), QCryptographicHash::Sha1).toHex().left(16);
    return QDir(storageDir()).filePath(QString::fromLatin1(hash) + QStringLiteral(".csr"));
}

void CameraClearSkyReference::ensureLoaded(const QString& cameraId)
{
    if (m_loaded && (cameraId == m_cameraId)) {
        return;
    }

    m_cameraId = cameraId;
    for (Slot& slot : m_slots) {
        slot = Slot();
    }
    m_foregroundDirty = true;
    load();
    m_loaded = true;
}

void CameraClearSkyReference::load()
{
    QFile file(storagePath());
    if (!file.open(QIODevice::ReadOnly)) {
        return;
    }

    QDataStream stream(&file);
    stream.setVersion(QDataStream::Qt_5_15);
    quint32 magic = 0;
    stream >> magic;
    if (magic != kFileMagic)
    {
        qWarning() << "CameraClearSkyReference: unrecognised reference file" << file.fileName();
        return;
    }

    for (Slot& slot : m_slots)
    {
        bool valid = false;
        stream >> valid;
        if (!valid) {
            continue;
        }
        stream >> slot.brightnessAnchor >> slot.ratioAnchor >> slot.roiNorm >> slot.updated >> slot.updateCount;
        if (!readMat(stream, slot.brightness) || !readMat(stream, slot.ratio)
            || !readMat(stream, slot.texture) || !readMat(stream, slot.sky)
            || (stream.status() != QDataStream::Ok))
        {
            qWarning() << "CameraClearSkyReference: failed to read reference file" << file.fileName();
            for (Slot& reset : m_slots) {
                reset = Slot();
            }
            return;
        }
    }
}

void CameraClearSkyReference::save() const
{
    QDir().mkpath(storageDir());
    // QSaveFile commits atomically via rename, so a crash mid-write (or a concurrent
    // reader in another feature instance) never sees a truncated reference file
    QSaveFile file(storagePath());
    if (!file.open(QIODevice::WriteOnly))
    {
        qWarning() << "CameraClearSkyReference: cannot write reference file" << file.fileName();
        return;
    }

    QDataStream stream(&file);
    stream.setVersion(QDataStream::Qt_5_15);
    stream << kFileMagic;
    for (const Slot& slot : m_slots)
    {
        stream << slot.valid();
        if (!slot.valid()) {
            continue;
        }
        stream << slot.brightnessAnchor << slot.ratioAnchor << slot.roiNorm << slot.updated << slot.updateCount;
        writeMat(stream, slot.brightness);
        writeMat(stream, slot.ratio);
        writeMat(stream, slot.texture);
        writeMat(stream, slot.sky);
    }
    if (!file.commit()) {
        qWarning() << "CameraClearSkyReference: failed to commit reference file" << file.fileName();
    }
}

bool CameraClearSkyReference::roiMatches(const Slot& slot, const QRectF& roiNorm)
{
    return (std::fabs(slot.roiNorm.x() - roiNorm.x()) < 0.02)
        && (std::fabs(slot.roiNorm.y() - roiNorm.y()) < 0.02)
        && (std::fabs(slot.roiNorm.width() - roiNorm.width()) < 0.02)
        && (std::fabs(slot.roiNorm.height() - roiNorm.height()) < 0.02);
}

// Builds the reference-resolution maps from the detector's work images: box-smoothed
// normalised luminance, red/blue ratio, fine texture, and the evaluated-sky mask, plus the
// robust anchors both sides of a later comparison are normalised by
void CameraClearSkyReference::buildMaps(const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& texture, const cv::Mat& evaluationMask,
                                        cv::Mat& brightnessOut, cv::Mat& ratioOut, cv::Mat& textureOut, cv::Mat& skyOut,
                                        double& brightnessAnchorOut, double& ratioAnchorOut)
{
    const cv::Size refSize(kRefSize, kRefSize);
    cv::Mat grayRef, bgrRef, textureRef;
    cv::resize(gray, grayRef, refSize, 0.0, 0.0, cv::INTER_AREA);
    cv::resize(workBgr, bgrRef, refSize, 0.0, 0.0, cv::INTER_AREA);
    cv::resize(texture, textureRef, refSize, 0.0, 0.0, cv::INTER_AREA);
    cv::resize(evaluationMask, skyOut, refSize, 0.0, 0.0, cv::INTER_NEAREST);

    brightnessAnchorOut = std::max(1, maskedPercentile8U(grayRef, skyOut, 0.5));

    cv::Mat grayF;
    grayRef.convertTo(grayF, CV_32F);
    const cv::Size smoothKernel(2 * kSmoothRadius + 1, 2 * kSmoothRadius + 1);
    cv::Mat smooth;
    cv::boxFilter(grayF, smooth, -1, smoothKernel);
    brightnessOut = smooth / brightnessAnchorOut;

    std::vector<cv::Mat> channels;
    cv::split(bgrRef, channels);
    cv::Mat blue, red;
    channels[0].convertTo(blue, CV_32F);
    channels[2].convertTo(red, CV_32F);
    ratioOut = red / (blue + 1.0f);

    cv::Mat brightSky;
    cv::bitwise_and(skyOut, grayRef >= kDarkPixel, brightSky);
    ratioAnchorOut = maskedPercentile32F(ratioOut, brightSky, 0.05);

    textureRef.convertTo(textureOut, CV_32F);
}

void CameraClearSkyReference::capture(int slot, const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& texture, const cv::Mat& evaluationMask, const QRectF& roiNorm, const QDateTime& when)
{
    if ((slot < 0) || (slot >= kSlotCount)) {
        return;
    }

    Slot& target = m_slots[slot];
    buildMaps(gray, workBgr, texture, evaluationMask,
              target.brightness, target.ratio, target.texture, target.sky,
              target.brightnessAnchor, target.ratioAnchor);
    target.roiNorm = roiNorm;
    target.updated = when;
    target.updateCount = 1;
    m_foregroundDirty = true;
    save();
}

bool CameraClearSkyReference::applyCueAndVeto(int slot, cv::Mat& mask, const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& evaluationMask, const QRectF& roiNorm, int nightThreshold) const
{
    if ((slot < 0) || (slot >= kSlotCount)) {
        return false;
    }
    const Slot& reference = m_slots[slot];
    if (!reference.valid() || !roiMatches(reference, roiNorm)) {
        return false;
    }

    // Build the frame's maps exactly as the reference was built, and compare at the
    // reference resolution
    cv::Mat frameBrightness, frameRatio, frameTexture, frameSky;
    double frameBrightnessAnchor = 0.0, frameRatioAnchor = 0.0;
    cv::Mat dummyTexture = cv::Mat::zeros(gray.size(), CV_8UC1);
    buildMaps(gray, workBgr, dummyTexture, evaluationMask,
              frameBrightness, frameRatio, frameTexture, frameSky,
              frameBrightnessAnchor, frameRatioAnchor);

    cv::Mat valid;
    cv::bitwise_and(reference.sky, frameSky, valid);
    if (cv::countNonZero(valid) < (kRefSize * kRefSize) / 20) {
        return false;
    }

    // Normalised brightness deviation, and the same in absolute 8-bit units
    const cv::Mat brightnessDev = frameBrightness - reference.brightness;
    cv::Mat absDev = brightnessDev * frameBrightnessAnchor;

    // Colour deviation with both sides anchored to their own clear-sky ratio, absorbing
    // white-balance and gain shifts (and, unavoidably, perfectly uniform colour changes)
    const cv::Mat ratioDev = (frameRatio - static_cast<float>(frameRatioAnchor))
        - (reference.ratio - static_cast<float>(reference.ratioAnchor));

    // Frame-level sanity: if the sky no longer resembles the reference at the median, the
    // exposure regime changed and per-pixel judgements would be nonsense
    cv::Mat absBrightnessDev = cv::abs(brightnessDev);
    if (maskedPercentile32F(absBrightnessDev, valid, 0.5) > kAbstainMedianDev) {
        return false;
    }

    cv::Mat grayRef;
    cv::resize(gray, grayRef, cv::Size(kRefSize, kRefSize), 0.0, 0.0, cv::INTER_AREA);
    const cv::Mat brightPixels = grayRef >= kDarkPixel;
    cv::Mat darkPixels;
    cv::bitwise_not(brightPixels, darkPixels);
    const cv::Mat absAbsDev = cv::abs(absDev);
    const cv::Mat absRatioDev = cv::abs(ratioDev);

    // Cue: brighter than the clear sky (relative and absolute) or shifted toward white
    const cv::Mat cueBrightnessRel = brightnessDev > kCueBrightness;
    const cv::Mat cueBrightnessAbs = absDev > static_cast<double>(nightThreshold);
    const cv::Mat cueRatioRel = ratioDev > kCueRatio;
    cv::Mat cueBrightness, cueRatio, cue;
    cv::bitwise_and(cueBrightnessRel, cueBrightnessAbs, cueBrightness);
    cv::bitwise_and(cueRatioRel, brightPixels, cueRatio);
    cv::bitwise_or(cueBrightness, cueRatio, cue);
    cv::bitwise_and(cue, valid, cue);

    // Veto: matches the reference within tolerances tighter than any detection margin.
    // Dark pixels rely on brightness alone; their ratio is quantisation noise.
    const cv::Mat vetoBrightnessRel = absBrightnessDev < kVetoBrightness;
    const cv::Mat vetoBrightnessAbs = absAbsDev < kVetoAbsBrightness;
    const cv::Mat vetoRatioRel = absRatioDev < kVetoRatio;
    cv::Mat vetoBrightness, vetoRatio, veto;
    cv::bitwise_or(vetoBrightnessRel, vetoBrightnessAbs, vetoBrightness);
    cv::bitwise_or(vetoRatioRel, darkPixels, vetoRatio);
    cv::bitwise_and(vetoBrightness, vetoRatio, veto);
    cv::bitwise_and(veto, valid, veto);

    // Back to work resolution and into the thresholded mask
    cv::Mat cueWork, vetoWork;
    cv::resize(cue, cueWork, mask.size(), 0.0, 0.0, cv::INTER_NEAREST);
    cv::resize(veto, vetoWork, mask.size(), 0.0, 0.0, cv::INTER_NEAREST);
    cv::bitwise_and(cueWork, evaluationMask, cueWork);
    cv::bitwise_or(mask, cueWork, mask);
    cv::Mat notVeto;
    cv::bitwise_not(vetoWork, notVeto);
    cv::bitwise_and(mask, notVeto, mask);
    return true;
}

bool CameraClearSkyReference::autoLearn(int slot, const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& texture, const cv::Mat& evaluationMask, const QRectF& roiNorm, const QDateTime& when,
                                        float coveragePercent, bool night, bool starSenseEnabled, bool starConfirmed)
{
    if ((slot < 0) || (slot >= kSlotCount)) {
        return false;
    }
    if (coveragePercent > (night ? kAutoLearnMaxCoverageNight : kAutoLearnMaxCoverageDay)) {
        return false;
    }
    // At night, when star sensing runs it must confirm the sky is genuinely clear; low
    // measured coverage alone could be a dark overcast the detector under-reads
    if (night && starSenseEnabled && !starConfirmed) {
        return false;
    }

    Slot& target = m_slots[slot];
    const bool replace = !target.valid() || !roiMatches(target, roiNorm);
    if (!replace && target.updated.isValid() && when.isValid()
        && (target.updated.secsTo(when) < kAutoLearnThrottleSecs)) {
        return false;
    }

    cv::Mat brightness, ratio, textureMap, sky;
    double brightnessAnchor = 0.0, ratioAnchor = 0.0;
    buildMaps(gray, workBgr, texture, evaluationMask, brightness, ratio, textureMap, sky, brightnessAnchor, ratioAnchor);

    if (replace)
    {
        target.brightness = brightness;
        target.ratio = ratio;
        target.texture = textureMap;
        target.sky = sky;
        target.brightnessAnchor = brightnessAnchor;
        target.ratioAnchor = ratioAnchor;
        target.roiNorm = roiNorm;
        target.updateCount = 1;
    }
    else
    {
        // Slow exponential blend so one mis-verified frame cannot poison the reference
        const double alpha = kAutoLearnAlpha;
        target.brightness = target.brightness * (1.0 - alpha) + brightness * alpha;
        target.ratio = target.ratio * (1.0 - alpha) + ratio * alpha;
        target.texture = target.texture * (1.0 - alpha) + textureMap * alpha;
        cv::bitwise_and(target.sky, sky, target.sky);
        target.brightnessAnchor = target.brightnessAnchor * (1.0 - alpha) + brightnessAnchor * alpha;
        target.ratioAnchor = target.ratioAnchor * (1.0 - alpha) + ratioAnchor * alpha;
        ++target.updateCount;
    }
    target.updated = when;
    m_foregroundDirty = true;
    save();
    return true;
}

cv::Mat CameraClearSkyReference::foregroundMask(const cv::Size& workSize, const QRectF& roiNorm) const
{
    if (m_foregroundDirty)
    {
        m_foregroundCache = cv::Mat();

        // Silhouettes: in the darkest filled night slot, foreground (trees, roofs, frames)
        // is darker than the airglow-lit sky
        static constexpr int darkPreference[] = {5, 6, 3, 4};
        const Slot *darkSlot = nullptr;
        for (int slot : darkPreference)
        {
            if (m_slots[slot].valid())
            {
                darkSlot = &m_slots[slot];
                break;
            }
        }

        cv::Mat foreground = cv::Mat::zeros(kRefSize, kRefSize, CV_8UC1);
        bool any = false;
        if (darkSlot)
        {
            const cv::Mat absolute = darkSlot->brightness * darkSlot->brightnessAnchor;
            cv::Mat silhouettes = absolute < kForegroundDarkLevel;
            cv::bitwise_and(silhouettes, darkSlot->sky, silhouettes);
            cv::bitwise_or(foreground, silhouettes, foreground);
            any = true;
        }
        // Structure: in the day slot, foliage and buildings carry dense fine texture that
        // sky and cloud do not
        const Slot& daySlot = m_slots[0];
        if (daySlot.valid() && (!darkSlot || roiMatches(daySlot, darkSlot->roiNorm)))
        {
            cv::Mat textured = daySlot.texture > kForegroundTexture;
            cv::bitwise_and(textured, daySlot.sky, textured);
            cv::bitwise_or(foreground, textured, foreground);
            any = true;
        }

        if (any)
        {
            const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::morphologyEx(foreground, foreground, cv::MORPH_OPEN, kernel);
            const double fraction = static_cast<double>(cv::countNonZero(foreground)) / (kRefSize * kRefSize);
            if ((fraction > 0.0) && (fraction <= kForegroundMaxFraction)) {
                m_foregroundCache = foreground;
            }
        }
        m_foregroundDirty = false;
    }

    if (m_foregroundCache.empty()) {
        return cv::Mat();
    }
    // The mask geometry only holds for the ROI it was learned from
    static constexpr int darkPreference[] = {5, 6, 3, 4, 0};
    for (int slot : darkPreference)
    {
        if (m_slots[slot].valid())
        {
            if (!roiMatches(m_slots[slot], roiNorm)) {
                return cv::Mat();
            }
            break;
        }
    }

    cv::Mat work;
    cv::resize(m_foregroundCache, work, workSize, 0.0, 0.0, cv::INTER_NEAREST);
    return work;
}

bool CameraClearSkyReference::slotFilled(int slot) const
{
    return (slot >= 0) && (slot < kSlotCount) && m_slots[slot].valid();
}

namespace {

QImage grayMatToImage(const cv::Mat& gray8)
{
    QImage image(gray8.cols, gray8.rows, QImage::Format_Grayscale8);
    for (int row = 0; row < gray8.rows; ++row) {
        std::memcpy(image.scanLine(row), gray8.ptr<uchar>(row), static_cast<size_t>(gray8.cols));
    }
    return image;
}

QImage renderFloatMap(const cv::Mat& map, double scale)
{
    cv::Mat scaled;
    map.convertTo(scaled, CV_8U, scale);
    return grayMatToImage(scaled);
}

} // namespace

CameraClearSkyReference::SlotPreview CameraClearSkyReference::slotPreview(int slot) const
{
    SlotPreview preview;
    if ((slot < 0) || (slot >= kSlotCount) || !m_slots[slot].valid()) {
        return preview;
    }

    const Slot& reference = m_slots[slot];
    // Brightness is stored normalised; reconstruct the absolute clear-sky luminance
    preview.brightness = renderFloatMap(reference.brightness, reference.brightnessAnchor);
    // Ratio rendered as in the detector's Signal debug view: 1.0 maps to mid-grey
    preview.ratio = renderFloatMap(reference.ratio, 128.0);
    preview.texture = renderFloatMap(reference.texture, 16.0);
    preview.sky = grayMatToImage(reference.sky);
    preview.info = QStringLiteral("%1\nupdated %2\n%3 update%4\nsky median %5\nclear R/B %6")
        .arg(slotName(slot),
             reference.updated.isValid() ? reference.updated.toString(QStringLiteral("yyyy-MM-dd hh:mm")) : QStringLiteral("-"))
        .arg(reference.updateCount)
        .arg(reference.updateCount == 1 ? QStringLiteral("") : QStringLiteral("s"))
        .arg(reference.brightnessAnchor, 0, 'f', 1)
        .arg(reference.ratioAnchor, 0, 'f', 3);
    preview.valid = true;
    return preview;
}

QImage CameraClearSkyReference::foregroundPreview() const
{
    // Use the source slot's own ROI so the geometry check always passes for the preview
    static constexpr int preference[] = {5, 6, 3, 4, 0};
    for (int slot : preference)
    {
        if (m_slots[slot].valid())
        {
            const cv::Mat foreground = foregroundMask(cv::Size(kRefSize, kRefSize), m_slots[slot].roiNorm);
            return foreground.empty() ? QImage() : grayMatToImage(foreground);
        }
    }
    return QImage();
}

QString CameraClearSkyReference::statusSummary(int activeSlot) const
{
    int filled = 0;
    QStringList names;
    for (int slot = 0; slot < kSlotCount; ++slot)
    {
        if (m_slots[slot].valid())
        {
            ++filled;
            names.append(slotName(slot));
        }
    }

    QString summary = QStringLiteral("%1/%2 refs").arg(filled).arg(kSlotCount);
    if (!names.isEmpty()) {
        summary += QStringLiteral(" (%1)").arg(names.join(QStringLiteral(", ")));
    }
    if ((activeSlot >= 0) && (activeSlot < kSlotCount))
    {
        summary += QStringLiteral(" - current: %1%2")
            .arg(slotName(activeSlot))
            .arg(m_slots[activeSlot].valid() ? QStringLiteral(" ✓") : QStringLiteral(" ✗"));
    }
    return summary;
}
