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
// The veto judges the residual after smooth-surface removal so it survives twilight/moon
// drift, but that must not let it erase a genuinely smooth overcast the in-frame paths
// detected: where the RAW deviation from the clear reference is itself large, the pixel is
// not "matching the reference" however smooth the change, and the veto may not fire.
constexpr double kVetoRawBrightnessCap = 0.30; // raw |brightness deviation| above this blocks the veto
constexpr double kVetoRawRatioCap = 0.12;      // raw |anchored ratio deviation| above this blocks the veto

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

constexpr quint32 kFileMagic = 0x43535232;   // "CSR2": fine sun-elevation bins + per-slot sky state
constexpr quint32 kFileMagicV1 = 0x43535231; // "CSR1": three broad night bands
// A reference is only comparable when frame and reference were exposed similarly; beyond
// this anchor ratio the normalised shapes no longer correspond and the comparison abstains
constexpr double kAnchorRatioLimit = 2.5;
// Strong star confirmation: most predicted stars visible everywhere proves the sky clear
// even when the measured coverage is inflated by the very false positives an empty
// reference slot cannot yet veto
constexpr int kStarConfirmMinExpected = 20;
constexpr double kStarConfirmStrongFraction = 0.7;
constexpr int kStarConfirmMinVisible = 5; // weak confirmation: some stars visible somewhere
constexpr float kAutoLearnStarMaxCoverage = 35.0f;

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

// Fits an order-2 polynomial surface to the masked values (least squares over normalised
// coordinates) and returns it evaluated over the whole map. Used to remove the smooth
// component of a frame-vs-reference deviation: twilight and moon glow evolve within
// minutes as smooth low-order gradients, while camera quirks and genuine cloud are
// localised, so comparing the residual keeps a reference valid across a whole sky-state
// band instead of only for the minutes around its capture.
cv::Mat fitSmoothSurface(const cv::Mat& values, const cv::Mat& mask)
{
    constexpr int kTerms = 6; // 1, x, y, x^2, xy, y^2
    double ata[kTerms][kTerms] = {{0.0}};
    double atb[kTerms] = {0.0};
    int samples = 0;

    const double scaleX = 2.0 / std::max(1, values.cols - 1);
    const double scaleY = 2.0 / std::max(1, values.rows - 1);
    for (int row = 0; row < values.rows; row += 2)
    {
        const float *valueLine = values.ptr<float>(row);
        const uchar *maskLine = mask.ptr<uchar>(row);
        const double y = row * scaleY - 1.0;
        for (int col = 0; col < values.cols; col += 2)
        {
            if (!maskLine[col]) {
                continue;
            }
            const double x = col * scaleX - 1.0;
            const double basis[kTerms] = {1.0, x, y, x * x, x * y, y * y};
            for (int i = 0; i < kTerms; ++i)
            {
                for (int j = i; j < kTerms; ++j) {
                    ata[i][j] += basis[i] * basis[j];
                }
                atb[i] += basis[i] * valueLine[col];
            }
            ++samples;
        }
    }

    cv::Mat surface = cv::Mat::zeros(values.size(), CV_32F);
    if (samples < 4 * kTerms) {
        return surface;
    }

    cv::Mat lhs(kTerms, kTerms, CV_64F);
    cv::Mat rhs(kTerms, 1, CV_64F);
    for (int i = 0; i < kTerms; ++i)
    {
        for (int j = 0; j < kTerms; ++j) {
            lhs.at<double>(i, j) = ata[std::min(i, j)][std::max(i, j)];
        }
        rhs.at<double>(i) = atb[i];
    }
    cv::Mat coefficients;
    if (!cv::solve(lhs, rhs, coefficients, cv::DECOMP_SVD)) {
        return surface;
    }

    for (int row = 0; row < surface.rows; ++row)
    {
        float *line = surface.ptr<float>(row);
        const double y = row * scaleY - 1.0;
        for (int col = 0; col < surface.cols; ++col)
        {
            const double x = col * scaleX - 1.0;
            const double basis[kTerms] = {1.0, x, y, x * x, x * y, y * y};
            double value = 0.0;
            for (int i = 0; i < kTerms; ++i) {
                value += coefficients.at<double>(i) * basis[i];
            }
            line[col] = static_cast<float>(value);
        }
    }
    return surface;
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
    const int bin = std::clamp(static_cast<int>((-4.0 - sunElevationDeg) / 2.0), 0, kNightBins - 1);
    const bool moonUp = moonElevationDeg > 5.0;
    return slotFromBin(bin, moonUp);
}

QString CameraClearSkyReference::slotName(int slot)
{
    if (slot == 0) {
        return QStringLiteral("Day");
    }
    if ((slot < 0) || (slot >= kSlotCount)) {
        return QStringLiteral("?");
    }
    const int bin = slotBin(slot);
    const bool moonUp = slotMoonUp(slot);
    const int upper = -4 - 2 * bin;
    QString name = (bin == kNightBins - 1)
        ? QStringLiteral("Sun < %1\u00b0").arg(upper)
        : QStringLiteral("Sun %1..%2\u00b0").arg(upper).arg(upper - 2);
    if (moonUp) {
        name += QStringLiteral(" +moon");
    }
    return name;
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

QString CameraClearSkyReference::storageFileName() const
{
    const QByteArray hash = QCryptographicHash::hash(m_cameraId.toUtf8(), QCryptographicHash::Sha1).toHex().left(16);
    return QString::fromLatin1(hash) + QStringLiteral(".csr");
}

QString CameraClearSkyReference::storagePath() const
{
    return QDir(storageDir()).filePath(storageFileName());
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
    if ((magic != kFileMagic) && (magic != kFileMagicV1))
    {
        qWarning() << "CameraClearSkyReference: unrecognised reference file" << file.fileName();
        return;
    }

    const bool v1 = (magic == kFileMagicV1);
    // v1 files held Day plus three broad night bands (x moon). Each band is replicated
    // into every fine bin it covered (cv::Mat copies share pixel data, so this is cheap),
    // preserving the coverage the old single band provided; the anchor guard rejects any
    // bin where the approximation no longer corresponds to the frame's exposure.
    static constexpr int kV1FirstBin[] = {0, 0, 0, 4, 4, 7, 7}; // Twilight -4..-12, Deep -12..-18, Dark < -18
    static constexpr int kV1LastBin[] = {0, 3, 3, 6, 6, 8, 8};
    const int slotCount = v1 ? 7 : kSlotCount;

    for (int index = 0; index < slotCount; ++index)
    {
        bool valid = false;
        stream >> valid;
        if (!valid) {
            continue;
        }
        Slot loaded;
        stream >> loaded.brightnessAnchor >> loaded.ratioAnchor;
        if (!v1) {
            stream >> loaded.sunElevation >> loaded.moonElevation;
        }
        stream >> loaded.roiNorm >> loaded.updated >> loaded.updateCount;
        if (!readMat(stream, loaded.brightness) || !readMat(stream, loaded.ratio)
            || !readMat(stream, loaded.texture) || !readMat(stream, loaded.sky)
            || (stream.status() != QDataStream::Ok))
        {
            qWarning() << "CameraClearSkyReference: failed to read reference file" << file.fileName();
            for (Slot& reset : m_slots) {
                reset = Slot();
            }
            return;
        }
        if (!v1)
        {
            if ((index >= 0) && (index < kSlotCount)) {
                m_slots[index] = loaded;
            }
            continue;
        }
        if (index == 0)
        {
            loaded.sunElevation = 10.0;
            loaded.moonElevation = -20.0;
            m_slots[0] = loaded;
            continue;
        }
        const bool moonUp = slotMoonUp(index); // v1 night slots share the same parity encoding
        loaded.moonElevation = moonUp ? 20.0 : -20.0;
        for (int bin = kV1FirstBin[index]; bin <= kV1LastBin[index]; ++bin)
        {
            Slot replica = loaded;
            replica.sunElevation = -4.0 - 2.0 * bin - 1.0; // bin centre
            m_slots[slotFromBin(bin, moonUp)] = replica;
        }
    }
}

void CameraClearSkyReference::save() const
{
    QDir().mkpath(storageDir());
    saveToPath(storagePath());
}

// Exports the in-memory reference into another directory under its canonical file name, so
// a saved test-case bundle can serve as a clear-sky store by pointing
// SDRANGEL_CAMERA_CLEARSKY_DIR at the bundle
bool CameraClearSkyReference::exportTo(const QString& directory) const
{
    bool any = false;
    for (const Slot& slot : m_slots) {
        any = any || slot.valid();
    }
    if (!any) {
        return false;
    }
    return saveToPath(QDir(directory).filePath(storageFileName()));
}

bool CameraClearSkyReference::saveToPath(const QString& path) const
{
    // QSaveFile commits atomically via rename, so a crash mid-write (or a concurrent
    // reader in another feature instance) never sees a truncated reference file
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly))
    {
        qWarning() << "CameraClearSkyReference: cannot write reference file" << file.fileName();
        return false;
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
        stream << slot.brightnessAnchor << slot.ratioAnchor << slot.sunElevation << slot.moonElevation << slot.roiNorm << slot.updated << slot.updateCount;
        writeMat(stream, slot.brightness);
        writeMat(stream, slot.ratio);
        writeMat(stream, slot.texture);
        writeMat(stream, slot.sky);
    }
    if (!file.commit())
    {
        qWarning() << "CameraClearSkyReference: failed to commit reference file" << file.fileName();
        return false;
    }
    return true;
}

// A slot usable for comparison against a frame: filled, same ROI, and exposed similarly
// enough that normalised shapes correspond
const CameraClearSkyReference::Slot* CameraClearSkyReference::usableSlot(int slot, const QRectF& roiNorm, double frameAnchor) const
{
    if ((slot < 0) || (slot >= kSlotCount)) {
        return nullptr;
    }
    const Slot& candidate = m_slots[slot];
    if (!candidate.valid() || !roiMatches(candidate, roiNorm)) {
        return nullptr;
    }
    const double high = std::max(frameAnchor, candidate.brightnessAnchor);
    const double low = std::max(1.0, std::min(frameAnchor, candidate.brightnessAnchor));
    if (high / low > kAnchorRatioLimit) {
        return nullptr;
    }
    return &candidate;
}

bool CameraClearSkyReference::roiNormsMatch(const QRectF& a, const QRectF& b)
{
    return (std::fabs(a.x() - b.x()) < 0.02)
        && (std::fabs(a.y() - b.y()) < 0.02)
        && (std::fabs(a.width() - b.width()) < 0.02)
        && (std::fabs(a.height() - b.height()) < 0.02);
}

bool CameraClearSkyReference::roiMatches(const Slot& slot, const QRectF& roiNorm)
{
    return roiNormsMatch(slot.roiNorm, roiNorm);
}

// Builds the reference-resolution maps from the detector's work images: box-smoothed
// normalised luminance, red/blue ratio, fine texture, and the evaluated-sky mask, plus the
// robust anchors both sides of a later comparison are normalised by
void CameraClearSkyReference::buildMaps(const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& texture, const cv::Mat& evaluationMask,
                                        cv::Mat& brightnessOut, cv::Mat& ratioOut, cv::Mat& textureOut, cv::Mat& skyOut,
                                        double& brightnessAnchorOut, double& ratioAnchorOut)
{
    const cv::Size refSize(kRefSize, kRefSize);
    cv::Mat grayRef, bgrRef;
    cv::resize(gray, grayRef, refSize, 0.0, 0.0, cv::INTER_AREA);
    cv::resize(workBgr, bgrRef, refSize, 0.0, 0.0, cv::INTER_AREA);
    cv::resize(evaluationMask, skyOut, refSize, 0.0, 0.0, cv::INTER_NEAREST);
    // The texture map is only stored at capture; comparisons pass an empty Mat
    cv::Mat textureRef = cv::Mat::zeros(refSize, CV_8UC1);
    if (!texture.empty()) {
        cv::resize(texture, textureRef, refSize, 0.0, 0.0, cv::INTER_AREA);
    }

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

void CameraClearSkyReference::capture(int slot, const cv::Mat& gray, const cv::Mat& workBgr, const cv::Mat& texture, const cv::Mat& evaluationMask, const QRectF& roiNorm, const QDateTime& when,
                                       double sunElevationDeg, double moonElevationDeg)
{
    if ((slot < 0) || (slot >= kSlotCount)) {
        return;
    }

    Slot& target = m_slots[slot];
    buildMaps(gray, workBgr, texture, evaluationMask,
              target.brightness, target.ratio, target.texture, target.sky,
              target.brightnessAnchor, target.ratioAnchor);
    target.sunElevation = sunElevationDeg;
    target.moonElevation = moonElevationDeg;
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
    // Cheap pre-check before building the frame's maps: with an empty or ROI-mismatched
    // store (every fresh install) nothing below could succeed, and the map build is the
    // expensive part of the comparison
    const auto slotCandidate = [&](int candidate) {
        return (candidate >= 0) && (candidate < kSlotCount)
            && m_slots[candidate].valid() && roiMatches(m_slots[candidate], roiNorm);
    };
    bool anyCandidate = slotCandidate(slot);
    if (!anyCandidate && slotIsNight(slot))
    {
        const int bin = slotBin(slot);
        const bool moonUp = slotMoonUp(slot);
        anyCandidate = ((bin > 0) && slotCandidate(slotFromBin(bin - 1, moonUp)))
            || ((bin < kNightBins - 1) && slotCandidate(slotFromBin(bin + 1, moonUp)));
    }
    if (!anyCandidate) {
        return false;
    }

    // Build the frame's maps exactly as the reference was built, and compare at the
    // reference resolution
    cv::Mat frameBrightness, frameRatio, frameTexture, frameSky;
    double frameBrightnessAnchor = 0.0, frameRatioAnchor = 0.0;
    buildMaps(gray, workBgr, cv::Mat(), evaluationMask,
              frameBrightness, frameRatio, frameTexture, frameSky,
              frameBrightnessAnchor, frameRatioAnchor);

    // Exact bin first, then the neighbouring sun-elevation bins with the same moon state.
    // Day (slot 0) never participates in night fallback and has no fallback of its own: a
    // night frame compared against a daylight map produces nonsense either way. The anchor
    // guard rejects any candidate whose exposure no longer corresponds to the frame's.
    const Slot *chosen = usableSlot(slot, roiNorm, frameBrightnessAnchor);
    if (!chosen && slotIsNight(slot))
    {
        const int bin = slotBin(slot);
        const bool moonUp = slotMoonUp(slot);
        if (bin > 0) {
            chosen = usableSlot(slotFromBin(bin - 1, moonUp), roiNorm, frameBrightnessAnchor);
        }
        if (!chosen && (bin < kNightBins - 1)) {
            chosen = usableSlot(slotFromBin(bin + 1, moonUp), roiNorm, frameBrightnessAnchor);
        }
    }
    if (!chosen) {
        return false;
    }
    const Slot& reference = *chosen;

    cv::Mat valid;
    cv::bitwise_and(reference.sky, frameSky, valid);
    if (cv::countNonZero(valid) < (kRefSize * kRefSize) / 20) {
        return false;
    }

    // Normalised brightness deviation
    cv::Mat brightnessDev = frameBrightness - reference.brightness;

    // Colour deviation with both sides anchored to their own clear-sky ratio, absorbing
    // white-balance and gain shifts (and, unavoidably, perfectly uniform colour changes)
    cv::Mat ratioDev = (frameRatio - static_cast<float>(frameRatioAnchor))
        - (reference.ratio - static_cast<float>(reference.ratioAnchor));

    // Frame-level sanity: if the sky no longer resembles the reference at the median, the
    // exposure regime changed and per-pixel judgements would be nonsense
    {
        const cv::Mat rawAbsDev = cv::abs(brightnessDev);
        if (maskedPercentile32F(rawAbsDev, valid, 0.5) > kAbstainMedianDev) {
            return false;
        }
    }

    // Twilight and moon glow evolve within minutes as smooth low-order gradients; without
    // this the veto only holds for the minutes around a save and the cue then misreads the
    // drifted glow as cloud. Judge only the residual, localised deviation - but keep the
    // raw deviations: the veto must not fire where the sky differs strongly from the
    // reference even if the difference is smooth (a gradient overcast is not drift).
    const cv::Mat rawAbsBrightnessDev = cv::abs(brightnessDev);
    const cv::Mat rawAbsRatioDev = cv::abs(ratioDev);
    brightnessDev -= fitSmoothSurface(brightnessDev, valid);
    ratioDev -= fitSmoothSurface(ratioDev, valid);
    cv::Mat absDev = brightnessDev * frameBrightnessAnchor;
    cv::Mat absBrightnessDev = cv::abs(brightnessDev);

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
    const cv::Mat rawBrightnessOk = rawAbsBrightnessDev < kVetoRawBrightnessCap;
    const cv::Mat rawRatioOk = rawAbsRatioDev < kVetoRawRatioCap;
    cv::Mat vetoBrightness, vetoRatio, veto;
    cv::bitwise_or(vetoBrightnessRel, vetoBrightnessAbs, vetoBrightness);
    cv::bitwise_or(vetoRatioRel, darkPixels, vetoRatio);
    cv::bitwise_and(vetoBrightness, vetoRatio, veto);
    cv::bitwise_and(veto, rawBrightnessOk, veto);
    cv::bitwise_and(veto, rawRatioOk, veto);
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
                                        float coveragePercent, bool night, bool starSenseEnabled, int starsExpected, int starsVisible)
{
    if ((slot < 0) || (slot >= kSlotCount)) {
        return false;
    }
    Slot& target = m_slots[slot];
    const bool replace = !target.valid() || !roiMatches(target, roiNorm);

    // Strong star confirmation (most predicted stars visible across the sky) overrides the
    // coverage gate ONLY when bootstrapping an empty slot: there, high measured coverage is
    // usually the false positives the missing reference cannot yet veto. A filled slot is a
    // working reference; it only ever blends genuinely low-coverage frames, so one partly
    // cloudy frame with clear zenith stars cannot progressively poison it. (The bootstrap
    // itself accepts a bounded risk: star sensing samples nothing below the minimum
    // elevation, so low horizon cloud can be baked into a first fill - a later Save ref or
    // low-coverage blends correct it.)
    const bool starStrong = night && starSenseEnabled && (starsExpected >= kStarConfirmMinExpected)
        && (static_cast<double>(starsVisible) >= kStarConfirmStrongFraction * starsExpected);
    const float maxCoverage = (starStrong && replace)
        ? kAutoLearnStarMaxCoverage
        : (night ? kAutoLearnMaxCoverageNight : kAutoLearnMaxCoverageDay);
    if (coveragePercent > maxCoverage) {
        return false;
    }
    // At night, when star sensing runs it must confirm the sky is genuinely clear; low
    // measured coverage alone could be a dark overcast the detector under-reads
    if (night && starSenseEnabled && !starStrong && (starsVisible < kStarConfirmMinVisible)) {
        return false;
    }
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

        // Silhouettes: in the darkest filled night slot (deepest sun-elevation bin,
        // moonless preferred), foreground (trees, roofs, frames) is darker than the
        // airglow-lit sky
        const Slot *darkSlot = nullptr;
        for (int bin = kNightBins - 1; (bin >= 0) && !darkSlot; --bin)
        {
            for (int moon = 0; (moon <= 1) && !darkSlot; ++moon)
            {
                const int slot = slotFromBin(bin, moon != 0);
                if (m_slots[slot].valid()) {
                    darkSlot = &m_slots[slot];
                }
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
            if ((fraction > 0.0) && (fraction <= kForegroundMaxFraction))
            {
                m_foregroundCache = foreground;
                // Remember the ROI of the slot the mask was derived from: the geometry
                // gate below must test the SOURCE slot, not whichever slot a scan order
                // happens to find first
                m_foregroundRoiNorm = darkSlot ? darkSlot->roiNorm : m_slots[0].roiNorm;
            }
        }
        m_foregroundDirty = false;
    }

    if (m_foregroundCache.empty()) {
        return cv::Mat();
    }
    // The mask geometry only holds for the ROI it was learned from
    if (!roiNormsMatch(m_foregroundRoiNorm, roiNorm)) {
        return cv::Mat();
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
    // Derive (or reuse) the cache, then render it with its own source ROI so the
    // geometry gate always passes for the preview
    for (int slot = 0; slot < kSlotCount; ++slot)
    {
        if (m_slots[slot].valid())
        {
            foregroundMask(cv::Size(kRefSize, kRefSize), m_slots[slot].roiNorm); // ensure derived
            const cv::Mat foreground = foregroundMask(cv::Size(kRefSize, kRefSize), m_foregroundRoiNorm);
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
