///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                        //
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

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <QDebug>

#include "dsp/fftengine.h"
#include "dsp/spectrumvis.h"
#include "util/messagequeue.h"
#include "maincore.h"

#include "meteordemodsink.h"

MESSAGE_CLASS_DEFINITION(MeteorDemodSink::MsgMeteorDetected, Message)
MESSAGE_CLASS_DEFINITION(MeteorDemodSink::MsgSatelliteDetected, Message)
MESSAGE_CLASS_DEFINITION(MeteorDemodSink::MsgMeteorDataCollected, Message)

namespace {
    FFTEngine *createMeteorFFT()
    {
        FFTEngine *fft = FFTEngine::create(QString(), QStringLiteral("Kiss"));

        if (!fft) {
            qFatal("MeteorDemodSink: no FFT engine is available");
        }

        return fft;
    }

    void makeHannWindow(std::vector<Real>& window, int size)
    {
        window.resize(size);

        if (size <= 1)
        {
            if (size == 1) {
                window[0] = 1.0f;
            }

            return;
        }

        const double twoPi = 2.0 * std::acos(-1.0);

        for (int i = 0; i < size; i++) {
            window[i] = (Real) (0.5 - 0.5 * std::cos(twoPi * (double) i / (double) (size - 1)));
        }
    }

}

MeteorDemodSink::MeteorDemodSink() :
    m_spectrumSink(nullptr),
    m_secondarySpectrumSink(nullptr),
    m_messageQueueToGUI(nullptr),
    m_channelSampleRate(48000),
    m_channelFrequencyOffset(0),
    m_interpolatorDistance(48.0f),
    m_interpolatorDistanceRemain(0.0f),
    m_spectrumBufferSize(0),
    m_spectralFrameSize(0),
    m_spectralHopSize(0),
    m_detectionSampleRingStart(0),
    m_detectionSampleRingCount(0),
    m_detectionSampleRingStartSample(0),
    m_nextDisplayTimeAnchorSample(0),
    m_nextDataMarkerSample(0),
    m_blobFFT(nullptr),
    m_blobFFTSize(0)
{
    resizeSpectrumBuffer();
    resetDetector();
    applySettings(m_settings, QStringList(), true);
    applyChannelSettings(m_channelSampleRate, m_channelFrequencyOffset, true);
}

MeteorDemodSink::~MeteorDemodSink()
{
    delete m_blobFFT;
}

void MeteorDemodSink::setDetectorTunables(const DetectorTunables& tunables)
{
    m_detectorTunables = tunables;
    // Through the spectral configuration (which itself reconfigures the blob
    // detector), so harness overrides of the STFT frame/hop tunables take effect
    // rather than silently reusing the previous front-end geometry.
    configureSpectralDetector();
}

void MeteorDemodSink::feed(const SampleVector::const_iterator& begin, const SampleVector::const_iterator& end)
{
    if ((m_channelSampleRate <= 0) || (m_settings.m_channelSampleRate <= 0)) {
        return;
    }

    Complex ci;

    for (SampleVector::const_iterator it = begin; it != end; ++it)
    {
        Complex c(it->real(), it->imag());
        c *= m_nco.nextIQ();

        if (m_interpolatorDistance < 1.0f)
        {
            while (!m_interpolator.interpolate(&m_interpolatorDistanceRemain, c, &ci))
            {
                processOneSample(ci);
                m_interpolatorDistanceRemain += m_interpolatorDistance;
            }
        }
        else
        {
            if (m_interpolator.decimate(&m_interpolatorDistanceRemain, c, &ci))
            {
                processOneSample(ci);
                m_interpolatorDistanceRemain += m_interpolatorDistance;
            }
        }
    }
}

bool MeteorDemodSink::flushPendingPulse(bool finalizeSweeps)
{
    // Only claim work was done if the detector analysed something new or an unfinished
    // sweep was retired — otherwise an idle source logs a spurious "flushed" message on
    // every inactivity tick.
    const bool flushedDetector = m_blobDetector.flush();

    drainBlobDetections();

    if (!finalizeSweeps) {
        // Inactivity tick: the stream may resume (the detector keeps its window for
        // exactly that reason). Keep active sweeps alive too, or a >=2 s data stall
        // mid-pass emits a half-pass row now and a second row for the same pass after
        // resume. Sweeps retire through the normal stale path once data flows again,
        // or here when the stream really ends (stop / teardown).
        return flushedDetector;
    }

    const bool hadActiveSweeps = !m_activeSweeps.empty();

    finalizeStaleSweeps(0, true);
    return flushedDetector || hadActiveSweeps;
}

void MeteorDemodSink::processOneSample(Complex& ci, bool realSample)
{
    recordDisplayTimeAnchor();

    if (realSample) {
        emitDataCollectionMarker();
    }

    const Real re = ci.real() / SDR_RX_SCALEF;
    const Real im = ci.imag() / SDR_RX_SCALEF;
    const Complex normalized(re, im);

    appendDetectionSample(normalized);
    processSpectralSample(normalized);
    feedSpectrum(normalized);
    m_sampleCounter++;
}

void MeteorDemodSink::processSpectralSample(const Complex& sample)
{
    if ((m_spectralFrameSize < 8) || (m_spectralHopSize < 1)) {
        return;
    }

    m_spectralFrameBuffer.push_back(sample);

    if ((int) m_spectralFrameBuffer.size() >= m_spectralFrameSize)
    {
        const quint64 frameStartSample = m_sampleCounter + 1 - (quint64) m_spectralFrameBuffer.size();
        processBlobDetectorFrame(frameStartSample);

        const int removeCount = std::min(m_spectralHopSize, (int) m_spectralFrameBuffer.size());
        m_spectralFrameBuffer.erase(m_spectralFrameBuffer.begin(), m_spectralFrameBuffer.begin() + removeCount);
    }

}

void MeteorDemodSink::configureBlobDetector()
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);

    // Zero-pad to nfft = nextPow2(2 * frameSize) so the live bin resolution matches the
    // offline recipe the acceptance/box thresholds were calibrated on (~3.9 Hz @ 1 ksps).
    int nfft = 1;
    while (nfft < 2 * std::max(8, m_spectralFrameSize)) {
        nfft <<= 1;
    }

    if (!m_blobFFT) {
        m_blobFFT = createMeteorFFT();
    }
    if (m_blobFFTSize != nfft)
    {
        m_blobFFT->configure(nfft, false);
        m_blobFFTSize = nfft;
    }
    makeHannWindow(m_blobWindow, m_spectralFrameSize);

    MeteorBlobDetector::Config cfg;
    cfg.m_seedDb = m_detectorTunables.m_blob.m_seedDb;
    cfg.m_growDb = m_detectorTunables.m_blob.m_growDb;
    cfg.m_floorOffsetDb = m_detectorTunables.m_blob.m_floorOffsetDb;
    cfg.m_medianFloor = true;    // per-chunk median floor = offline reference, no self-masking
    cfg.m_extractSweepSegments = m_detectorTunables.m_blob.m_weakSweepExtend;
    cfg.m_faintSeedDb = m_detectorTunables.m_blob.m_faintSeedDb;
    // Extraction is RELAXED (small dashes included) so the sink can dash-walk sweep boxes;
    // the strong shape gates from the tunables are applied sink-side for live extension.
    cfg.m_segMinCols = 2;
    cfg.m_segMinPix = 3;
    cfg.m_segMinR2 = -1e9;
    cfg.m_segSlopeMinHzPerS = 0.0;
    cfg.m_segSlopeMaxHzPerS = 1e12;
    cfg.m_minPix = m_detectorTunables.m_blob.m_minPix;
    cfg.m_closeFreqBins = m_detectorTunables.m_blob.m_closeFreqBins;
    cfg.m_linkMaxDriftHzPerS = m_detectorTunables.m_blob.m_linkMaxDriftHzPerS;
    cfg.m_linkTolHz = m_detectorTunables.m_blob.m_linkTolHz;
    cfg.m_trimKeepTime = m_detectorTunables.m_blob.m_trimKeepTime;
    cfg.m_trimKeepFreq = m_detectorTunables.m_blob.m_trimKeepFreq;
    cfg.m_occLimit = m_detectorTunables.m_blob.m_occLimit;
    cfg.m_minPeakExcessDb = m_detectorTunables.m_blob.m_minPeakExcessDb;
    cfg.m_sweepMinLinR2 = m_detectorTunables.m_blob.m_sweepMinLinR2;
    cfg.m_sweepMinAbsSlopeHzPerS = m_detectorTunables.m_blob.m_sweepMinAbsSlopeHzPerS;
    cfg.m_sweepMinDurationS = m_detectorTunables.m_blob.m_sweepMinDurationS;
    cfg.m_scoreThreshold = m_detectorTunables.m_blob.m_scoreThreshold;
    cfg.m_windowSeconds = m_detectorTunables.m_blob.m_windowSeconds;
    cfg.m_minWindowSeconds = m_detectorTunables.m_blob.m_minWindowSeconds;
    cfg.m_emitStrideS = m_detectorTunables.m_blob.m_emitStrideS;
    cfg.m_finalMarginS = m_detectorTunables.m_blob.m_finalMarginS;
    cfg.m_binHz = (double) sampleRate / (double) nfft;
    cfg.m_dt = (double) std::max(1, m_spectralHopSize) / (double) sampleRate;
    cfg.m_fMinHz = -(double) sampleRate / 2.0;
    // Link gap is specified in time so fragment merging is rate-independent (a column gap
    // would shrink at higher sample rates and over-split long meteors).
    cfg.m_linkMaxGapCols = std::max(1, (int) std::llround(m_detectorTunables.m_blob.m_linkGapSeconds / cfg.m_dt));
    m_blobDetector.configure(cfg);

    m_blobBinPower.assign(nfft, 1e-30);
}

void MeteorDemodSink::processBlobDetectorFrame(quint64 frameStartSample)
{
    if ((m_blobFFTSize < 8) || ((int) m_spectralFrameBuffer.size() < m_spectralFrameSize)) {
        return;
    }

    Complex *fftIn = m_blobFFT->in();
    for (int i = 0; i < m_blobFFTSize; i++) {
        fftIn[i] = (i < m_spectralFrameSize)
            ? m_spectralFrameBuffer[i] * m_blobWindow[i]
            : Complex(0.0f, 0.0f);
    }
    m_blobFFT->transform();
    const Complex *fftOut = m_blobFFT->out();
    const int halfSize = m_blobFFTSize / 2;

    if ((int) m_blobBinPower.size() != m_blobFFTSize) {
        m_blobBinPower.assign(m_blobFFTSize, 1e-30);
    }
    for (int bin = -halfSize; bin < halfSize; bin++)
    {
        const int fftIndex = bin < 0 ? bin + m_blobFFTSize : bin;
        m_blobBinPower[bin + halfSize] = std::max((double) std::norm(fftOut[fftIndex]), 1e-30);
    }

    const quint64 frameCenterSample = frameStartSample + (quint64) (m_spectralFrameSize / 2);

    // The detector buffers columns and derives its own per-chunk median floor, so we just
    // hand it the padded bin power (the noiseFloor argument is ignored in median mode).
    m_blobDetector.processFrame(m_blobBinPower, std::vector<double>(), frameCenterSample);
    drainBlobDetections();
    finalizeStaleSweeps(frameCenterSample, false);
}

void MeteorDemodSink::drainBlobDetections()
{
    std::vector<MeteorBlobDetector::Blob> blobs = m_blobDetector.takeBlobs();
    // Within one chunk blobs come out in raster (frequency) order; sweep consolidation needs
    // fragments in time order, so sort each batch by start sample before routing.
    std::stable_sort(blobs.begin(), blobs.end(),
              [](const MeteorBlobDetector::Blob& a, const MeteorBlobDetector::Blob& b) {
                  return a.m_startSample < b.m_startSample;
              });
    for (const MeteorBlobDetector::Blob& blob : blobs)
    {
        if (blob.m_sweepRejected) {
            // A blob can re-emit grown: if it was already reported as a METEOR and the
            // grown version now classifies as a sweep, a second (satellite) row for the
            // same event must not appear. The meteor row stands.
            if (!overlapsRecentDetection(
                    blob.m_startSample,
                    blob.m_endSample,
                    blob.m_f0,
                    blob.m_f1)) {
                addSweepFragment(blob);      // stitch into a consolidated Doppler sweep
            }
        } else if (absorbedByActiveSweep(blob)) {
            // on an established sweep's line — part of the sweep, not a meteor
        } else {
            emitBlobDetection(blob);         // accepted meteor
        }
    }

    // Weak-sweep extension: STRONG faint segments (shape-gated) attach live to an
    // established sweep's line; every dash (relaxed) is buffered for the finalize-time
    // dash-walk that recovers a pass's faint dashed leading/trailing parts.
    if (m_detectorTunables.m_blob.m_weakSweepExtend)
    {
        std::vector<MeteorBlobDetector::SweepSegment> segs = m_blobDetector.takeSweepSegments();
        for (const MeteorBlobDetector::SweepSegment& seg : segs)
        {
            const bool strong = (seg.m_cols >= m_detectorTunables.m_blob.m_segMinCols)
                && (seg.m_r2 >= m_detectorTunables.m_blob.m_segMinR2)
                && (std::fabs(seg.m_slopeHzPerS) >= m_detectorTunables.m_blob.m_segSlopeMinHzPerS)
                && (std::fabs(seg.m_slopeHzPerS) <= m_detectorTunables.m_blob.m_segSlopeMaxHzPerS);
            if (strong) {
                extendSweepWithSegment(seg);
            }
            m_faintDashes.push_back(seg);
        }

        // prune the dash buffer to a horizon covering finalize-time walks
        const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
        const quint64 keep = (quint64) sampleRate * 90;
        if (m_sampleCounter > keep)
        {
            const quint64 oldest = m_sampleCounter - keep;
            m_faintDashes.erase(
                std::remove_if(m_faintDashes.begin(), m_faintDashes.end(),
                               [oldest](const MeteorBlobDetector::SweepSegment& d) {
                                   return d.m_endSample < oldest;
                               }),
                m_faintDashes.end());
        }
    }
}

// --- Doppler-sweep consolidation ------------------------------------------------
// Sweep fragments arrive across chunks; stitch collinear ones (a fragment continues an
// active sweep's extrapolated drift line within a bounded time gap) into a single sweep.

void MeteorDemodSink::addSweepFragment(const MeteorBlobDetector::Blob& blob)
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const double tc = 0.5 * (blob.m_t0 + blob.m_t1);
    const double fc = 0.5 * (blob.m_f0 + blob.m_f1);
    const double maxGapS = m_detectorTunables.m_blob.m_sweepLinkMaxGapS;
    const double tolHz = m_detectorTunables.m_blob.m_sweepLinkTolHz;

    // Find the active sweep whose drift line best predicts this fragment's centre frequency.
    // The gap test is non-destructive: retirement is finalizeStaleSweeps' job (driven by the
    // live edge with the full settle margin). Retiring here, keyed to an arbitrary incoming
    // fragment's start, prematurely killed sweeps that a later long fragment - emitted later
    // because emission is END-ordered - would have continued, splitting one pass into two rows.
    int best = -1;
    double bestErr = tolHz;
    for (size_t i = 0; i < m_activeSweeps.size(); i++)
    {
        const ActiveSweep& s = m_activeSweeps[i];
        const double gapS = (double) ((blob.m_startSample > s.m_endSample)
            ? (blob.m_startSample - s.m_endSample) : 0) / (double) sampleRate;

        if (gapS > maxGapS) {
            continue;   // too old to continue, but not ours to retire
        }

        double predF;
        if (s.m_nPts >= 2)
        {
            const double denom = s.m_nPts * s.m_sumTT - s.m_sumT * s.m_sumT;
            if (std::fabs(denom) > 1e-9) {
                const double slope = (s.m_nPts * s.m_sumTF - s.m_sumT * s.m_sumF) / denom;
                const double intercept = (s.m_sumF - slope * s.m_sumT) / s.m_nPts;
                predF = slope * tc + intercept;
            } else {
                predF = s.m_lastCentreF;
            }
        }
        else
        {
            predF = s.m_lastCentreF + s.m_lastSlope * (tc - s.m_lastCentreT);
        }
        const double err = std::fabs(fc - predF);
        if (err <= bestErr) { bestErr = err; best = (int) i; }
    }

    if (best >= 0)
    {
        const ActiveSweep& matched = m_activeSweeps[best];

        // A stride re-analysis can re-emit a fragment the sweep already contains (the
        // rolling median shifted and the group grew elsewhere). Nothing new to add —
        // merging again would double-count its power into the line fit and totals.
        if ((blob.m_startSample >= matched.m_startSample)
            && (blob.m_endSample <= matched.m_endSample))
        {
            return;
        }
    }

    if (best < 0)
    {
        ActiveSweep s;
        s.m_t0 = blob.m_t0; s.m_t1 = blob.m_t1;
        s.m_f0 = blob.m_f0; s.m_f1 = blob.m_f1;
        s.m_startSample = blob.m_startSample; s.m_endSample = blob.m_endSample;
        s.m_peakPower = blob.m_peakPower; s.m_totalPower = blob.m_totalPower;
        s.m_backgroundSum = blob.m_backgroundPower;
        s.m_fragCount = 1;
        s.m_sumT = tc; s.m_sumF = fc; s.m_sumTT = tc * tc; s.m_sumTF = tc * fc; s.m_nPts = 1;
        s.m_lastCentreT = tc; s.m_lastCentreF = fc; s.m_lastSlope = blob.m_slopeHzPerS;
        s.m_lastSample = blob.m_endSample;
        m_activeSweeps.push_back(s);
        return;
    }

    mergeSweepFragment(m_activeSweeps[best], blob);
}

void MeteorDemodSink::mergeSweepFragment(ActiveSweep& s, const MeteorBlobDetector::Blob& blob)
{
    const double tc = 0.5 * (blob.m_t0 + blob.m_t1);
    const double fc = 0.5 * (blob.m_f0 + blob.m_f1);
    s.m_t0 = std::min(s.m_t0, blob.m_t0); s.m_t1 = std::max(s.m_t1, blob.m_t1);
    s.m_f0 = std::min(s.m_f0, blob.m_f0); s.m_f1 = std::max(s.m_f1, blob.m_f1);
    s.m_startSample = std::min(s.m_startSample, blob.m_startSample);
    s.m_endSample = std::max(s.m_endSample, blob.m_endSample);
    s.m_peakPower = std::max(s.m_peakPower, blob.m_peakPower);
    s.m_totalPower += blob.m_totalPower;
    s.m_backgroundSum += blob.m_backgroundPower;
    s.m_fragCount++;
    s.m_sumT += tc; s.m_sumF += fc; s.m_sumTT += tc * tc; s.m_sumTF += tc * fc; s.m_nPts++;
    s.m_lastCentreT = tc; s.m_lastCentreF = fc; s.m_lastSlope = blob.m_slopeHzPerS;
    s.m_lastSample = blob.m_endSample;
}

// A blob that is not sweep-shaped on its own but lies ON an established sweep's drift line
// (in time and frequency) is part of that sweep, not a separate meteor — absorb it.
bool MeteorDemodSink::absorbedByActiveSweep(const MeteorBlobDetector::Blob& blob)
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const double tc = 0.5 * (blob.m_t0 + blob.m_t1);
    const double fc = 0.5 * (blob.m_f0 + blob.m_f1);
    // Absorption is restricted to (near) the sweep's own time span: every validated case
    // sits within the span, and a wide slack would let a genuine meteor that merely crosses
    // an unrelated pass's extrapolated line vanish from the meteor counts.
    const double maxGapS = 1.0;
    const double tolHz = m_detectorTunables.m_blob.m_sweepLinkTolHz;

    int best = -1;
    double bestErr = tolHz;
    for (size_t i = 0; i < m_activeSweeps.size(); i++)
    {
        const ActiveSweep& s = m_activeSweeps[i];
        if (s.m_nPts < 2) {
            continue;   // need an established line before attributing meteors to it
        }
        // near the sweep in time (inside its span, or just off either end)
        const double afterS = (blob.m_startSample > s.m_endSample)
            ? (double) (blob.m_startSample - s.m_endSample) / sampleRate : 0.0;
        const double beforeS = (blob.m_endSample < s.m_startSample)
            ? (double) (s.m_startSample - blob.m_endSample) / sampleRate : 0.0;
        if (afterS > maxGapS || beforeS > maxGapS) {
            continue;
        }
        const double denom = s.m_nPts * s.m_sumTT - s.m_sumT * s.m_sumT;
        if (std::fabs(denom) <= 1e-9) {
            continue;
        }
        const double slope = (s.m_nPts * s.m_sumTF - s.m_sumT * s.m_sumF) / denom;
        const double intercept = (s.m_sumF - slope * s.m_sumT) / s.m_nPts;
        const double err = std::fabs(fc - (slope * tc + intercept));
        if (err <= bestErr) { bestErr = err; best = (int) i; }
    }

    if (best < 0) {
        return false;
    }

    // Still absorbed (not a meteor), but a re-emitted copy the sweep already spans
    // must not double-count its power into the sweep.
    if ((blob.m_startSample < m_activeSweeps[best].m_startSample)
        || (blob.m_endSample > m_activeSweeps[best].m_endSample))
    {
        mergeSweepFragment(m_activeSweeps[best], blob);
    }

    return true;
}

// A faint below-seed segment that continues an ESTABLISHED sweep's drift line (frequency
// prediction + slope agreement, within the time gap) extends that sweep. Segments matching no
// established sweep are dropped — there is no faint-only discovery.
void MeteorDemodSink::extendSweepWithSegment(const MeteorBlobDetector::SweepSegment& seg)
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const double tc = 0.5 * (seg.m_t0 + seg.m_t1);
    const double fc = seg.m_fc;
    const double maxGapS = m_detectorTunables.m_blob.m_sweepLinkMaxGapS;
    const double tolHz = m_detectorTunables.m_blob.m_sweepLinkTolHz;

    int best = -1;
    double bestErr = tolHz;
    for (size_t i = 0; i < m_activeSweeps.size(); i++)
    {
        const ActiveSweep& s = m_activeSweeps[i];
        const double afterS = (seg.m_startSample > s.m_endSample)
            ? (double) (seg.m_startSample - s.m_endSample) / sampleRate : 0.0;
        const double beforeS = (seg.m_endSample < s.m_startSample)
            ? (double) (s.m_startSample - seg.m_endSample) / sampleRate : 0.0;
        if (afterS > maxGapS || beforeS > maxGapS) {
            continue;
        }
        // sweep drift line: fitted when >=2 fragments, else the single fragment's own slope
        // (a sweep fragment is already lin_r2>=0.9, so its slope is trustworthy).
        double slope, predF;
        if (s.m_nPts >= 2)
        {
            const double denom = s.m_nPts * s.m_sumTT - s.m_sumT * s.m_sumT;
            if (std::fabs(denom) <= 1e-9) {
                continue;
            }
            slope = (s.m_nPts * s.m_sumTF - s.m_sumT * s.m_sumF) / denom;
            predF = slope * tc + (s.m_sumF - slope * s.m_sumT) / s.m_nPts;
        }
        else
        {
            slope = s.m_lastSlope;
            predF = s.m_lastCentreF + slope * (tc - s.m_lastCentreT);
        }
        // the segment's own drift must agree with the sweep's slope
        if (std::fabs(seg.m_slopeHzPerS - slope) > std::max(30.0, 0.35 * std::fabs(slope))) {
            continue;
        }
        const double err = std::fabs(fc - predF);
        if (err <= bestErr) { bestErr = err; best = (int) i; }
    }

    if (best < 0) {
        return;
    }

    // Re-emitted segments the sweep already spans add nothing (and would double-count).
    if ((seg.m_startSample >= m_activeSweeps[best].m_startSample)
        && (seg.m_endSample <= m_activeSweeps[best].m_endSample))
    {
        return;
    }

    MeteorBlobDetector::Blob fb;   // adapt the segment to the fragment-merge signature
    fb.m_t0 = seg.m_t0; fb.m_t1 = seg.m_t1;
    fb.m_f0 = seg.m_f0; fb.m_f1 = seg.m_f1;
    fb.m_startSample = seg.m_startSample; fb.m_endSample = seg.m_endSample;
    fb.m_peakPower = seg.m_peakPower; fb.m_totalPower = seg.m_totalPower;
    fb.m_backgroundPower = seg.m_backgroundPower;
    fb.m_slopeHzPerS = seg.m_slopeHzPerS;
    mergeSweepFragment(m_activeSweeps[best], fb);
}

void MeteorDemodSink::finalizeStaleSweeps(quint64 currentSample, bool force)
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    // A sweep is complete once no further fragment can extend it: with stride emission a
    // fragment arrives ~(finalMargin + stride) after it ends, so a continuation fragment
    // (starting within the link gap, lasting up to ~a window) is all in by window + gap.
    const double settleS = m_detectorTunables.m_blob.m_windowSeconds
        + m_detectorTunables.m_blob.m_sweepLinkMaxGapS;
    for (size_t i = 0; i < m_activeSweeps.size();)
    {
        const double behindS = (double) ((currentSample > m_activeSweeps[i].m_lastSample)
            ? (currentSample - m_activeSweeps[i].m_lastSample) : 0) / (double) sampleRate;
        if (force || behindS > settleS) {
            retireSweepAt(i);
        } else {
            i++;
        }
    }
}

// Retire one active sweep: merge it into a collinear continuation if one exists (a single
// pass split by its faint mid-section), else dash-walk its box outward and emit it.
void MeteorDemodSink::retireSweepAt(size_t index)
{
    if (index >= m_activeSweeps.size()) {
        return;
    }
    if (!mergeIntoContinuationSweep(index))
    {
        walkSweepAlongDashes(m_activeSweeps[index]);
        emitConsolidatedSweep(m_activeSweeps[index]);
    }
    m_activeSweeps.erase(m_activeSweeps.begin() + index);
}

// Two consolidated sweeps that continue one pass: a satellite's Doppler S-curve sweeps
// fastest (and appears faintest) mid-pass, so the middle can be invisible for longer than
// the fragment link gap and the pass splits in two. Merge when the later sweep is a
// consistent continuation: same drift direction and a frequency step over the gap that
// implies a plausible (typically faster) mid-pass slope.
bool MeteorDemodSink::mergeIntoContinuationSweep(size_t sweepIndex)
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const ActiveSweep& s = m_activeSweeps[sweepIndex];

    double slopeS = s.m_lastSlope, interceptS = 0.0;
    if (s.m_nPts >= 2)
    {
        const double denom = s.m_nPts * s.m_sumTT - s.m_sumT * s.m_sumT;
        if (std::fabs(denom) > 1e-9) {
            slopeS = (s.m_nPts * s.m_sumTF - s.m_sumT * s.m_sumF) / denom;
        }
    }
    interceptS = (s.m_nPts > 0) ? (s.m_sumF - slopeS * s.m_sumT) / s.m_nPts : 0.0;

    for (size_t j = 0; j < m_activeSweeps.size(); j++)
    {
        if (j == sweepIndex) {
            continue;
        }
        ActiveSweep& t = m_activeSweeps[j];
        if (t.m_startSample <= s.m_endSample) {
            continue;   // only a LATER sweep can be the continuation
        }
        const double gapS = (double) (t.m_startSample - s.m_endSample) / (double) sampleRate;
        if (gapS > m_detectorTunables.m_blob.m_sweepMergeMaxGapS) {
            continue;
        }

        double slopeT = t.m_lastSlope;
        if (t.m_nPts >= 2)
        {
            const double denom = t.m_nPts * t.m_sumTT - t.m_sumT * t.m_sumT;
            if (std::fabs(denom) > 1e-9) {
                slopeT = (t.m_nPts * t.m_sumTF - t.m_sumT * t.m_sumF) / denom;
            }
        }
        const double interceptT = (t.m_nPts > 0) ? (t.m_sumF - slopeT * t.m_sumT) / t.m_nPts : 0.0;

        // same drift direction, both actually drifting
        if ((slopeS * slopeT <= 0.0)
            || (std::fabs(slopeS) < 10.0) || (std::fabs(slopeT) < 10.0)) {
            continue;
        }
        // frequency step across the gap must go the same way, at a rate consistent with the
        // (typically faster) unseen middle of the pass
        const double fEndS = slopeS * s.m_t1 + interceptS;
        const double fStartT = slopeT * t.m_t0 + interceptT;
        const double implied = (fStartT - fEndS) / std::max(0.5, gapS);
        if (implied * slopeS <= 0.0) {
            continue;
        }
        const double fastest = std::max(std::fabs(slopeS), std::fabs(slopeT));
        if (std::fabs(implied) > 8.0 * fastest + 50.0) {
            continue;
        }

        // merge s into t (t keeps accumulating; combined line fit spans both pieces)
        t.m_t0 = std::min(t.m_t0, s.m_t0);
        t.m_t1 = std::max(t.m_t1, s.m_t1);
        t.m_f0 = std::min(t.m_f0, s.m_f0);
        t.m_f1 = std::max(t.m_f1, s.m_f1);
        t.m_startSample = std::min(t.m_startSample, s.m_startSample);
        t.m_endSample = std::max(t.m_endSample, s.m_endSample);
        t.m_peakPower = std::max(t.m_peakPower, s.m_peakPower);
        t.m_totalPower += s.m_totalPower;
        t.m_backgroundSum += s.m_backgroundSum;
        t.m_fragCount += s.m_fragCount;
        t.m_sumT += s.m_sumT; t.m_sumF += s.m_sumF;
        t.m_sumTT += s.m_sumTT; t.m_sumTF += s.m_sumTF;
        t.m_nPts += s.m_nPts;

        qDebug() << "MeteorDemodSink::mergeIntoContinuationSweep: merged pass halves"
                 << " gapS:" << gapS
                 << " slopeS:" << slopeS
                 << " slopeT:" << slopeT
                 << " impliedSlope:" << implied;
        return true;
    }

    return false;
}

// Extend a finished sweep's box through its faint dashed leading/trailing parts: walk the
// fitted drift line outward, one on-line dash at a time. Chained steps are required (each
// must find a dash within the step gap), so scattered noise cannot sustain a walk.
void MeteorDemodSink::walkSweepAlongDashes(ActiveSweep& sweep)
{
    if (sweep.m_nPts < 1 || m_faintDashes.empty()) {
        return;
    }

    double slope = sweep.m_lastSlope;
    if (sweep.m_nPts >= 2)
    {
        const double denom = sweep.m_nPts * sweep.m_sumTT - sweep.m_sumT * sweep.m_sumT;
        if (std::fabs(denom) > 1e-9) {
            slope = (sweep.m_nPts * sweep.m_sumTF - sweep.m_sumT * sweep.m_sumF) / denom;
        }
    }
    const double intercept = (sweep.m_sumF - slope * sweep.m_sumT) / sweep.m_nPts;
    const double tolHz = m_detectorTunables.m_blob.m_sweepLinkTolHz;
    const double stepGapS = m_detectorTunables.m_blob.m_dashWalkStepGapS;
    const double maxWalkS = m_detectorTunables.m_blob.m_dashWalkMaxS;

    std::vector<char> used(m_faintDashes.size(), 0);
    // A dash whose midpoint lies inside the sweep's own span was either already merged
    // live (strong segment) or is interior detail - re-adding its power double-counts.
    // Span captured before walking: dashes claimed by the walk sit outside it.
    const double spanT0 = sweep.m_t0;
    const double spanT1 = sweep.m_t1;

    auto walk = [&](bool backward)
    {
        double edgeT = backward ? sweep.m_t0 : sweep.m_t1;
        const double limitT = backward ? (sweep.m_t0 - maxWalkS) : (sweep.m_t1 + maxWalkS);

        for (;;)
        {
            int best = -1;
            double bestGap = stepGapS;
            for (size_t k = 0; k < m_faintDashes.size(); k++)
            {
                if (used[k]) {
                    continue;
                }
                const MeteorBlobDetector::SweepSegment& d = m_faintDashes[k];
                const double tc = 0.5 * (d.m_t0 + d.m_t1);

                if ((tc >= spanT0) && (tc <= spanT1)) {
                    continue;
                }

                const double gap = backward ? (edgeT - d.m_t1) : (d.m_t0 - edgeT);
                if ((gap < -0.3) || (gap > bestGap)) {
                    continue;
                }
                if (backward ? (tc < limitT) : (tc > limitT)) {
                    continue;
                }
                if (std::fabs(d.m_fc - (slope * tc + intercept)) > tolHz) {
                    continue;
                }
                bestGap = std::max(0.0, gap);
                best = (int) k;
            }
            if (best < 0) {
                break;
            }
            const MeteorBlobDetector::SweepSegment& d = m_faintDashes[best];
            used[best] = 1;
            sweep.m_t0 = std::min(sweep.m_t0, d.m_t0);
            sweep.m_t1 = std::max(sweep.m_t1, d.m_t1);
            sweep.m_f0 = std::min(sweep.m_f0, d.m_f0);
            sweep.m_f1 = std::max(sweep.m_f1, d.m_f1);
            sweep.m_startSample = std::min(sweep.m_startSample, d.m_startSample);
            sweep.m_endSample = std::max(sweep.m_endSample, d.m_endSample);
            sweep.m_peakPower = std::max(sweep.m_peakPower, d.m_peakPower);
            sweep.m_totalPower += d.m_totalPower;
            edgeT = backward ? d.m_t0 : d.m_t1;
        }
    };

    walk(true);
    walk(false);
}

void MeteorDemodSink::emitConsolidatedSweep(const ActiveSweep& sweep)
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);

    double slope = sweep.m_lastSlope;
    if (sweep.m_nPts >= 2)
    {
        const double denom = sweep.m_nPts * sweep.m_sumTT - sweep.m_sumT * sweep.m_sumT;
        if (std::fabs(denom) > 1e-9) {
            slope = (sweep.m_nPts * sweep.m_sumTF - sweep.m_sumT * sweep.m_sumF) / denom;
        }
    }

    // Sweep samples are STFT frame centres; extend by half a frame to the signal support.
    const quint64 halfFrame = (quint64) std::max(0, m_spectralFrameSize / 2);
    const quint64 startSample = sweep.m_startSample > halfFrame ? sweep.m_startSample - halfFrame : 0;
    const quint64 endSample = sweep.m_endSample + halfFrame;

    DetectionRecord d;
    d.m_dateTimeUtc = sampleCounterToDateTimeUtc(startSample);
    d.m_displayDateTimeUtc = sampleCounterToDisplayDateTimeUtc(startSample);
    d.m_startSample = startSample;
    d.m_endSample = endSample;
    d.m_displayStartSample = startSample;
    d.m_displayEndSample = endSample;
    d.m_hasDisplaySamples = true;
    d.m_durationS = (endSample >= startSample)
        ? (double) (endSample - startSample) / (double) sampleRate : 0.0;
    // Display duration must be WALL-CLOCK time via the anchors (the waterfall y-axis is wall
    // time): at accelerated file playback, stream seconds occupy less display time and a
    // stream-duration box is drawn too tall (mirrors emitDetectionReport's handling).
    d.m_displayDurationS = d.m_durationS;
    const QDateTime displayEndDateTimeUtc = sampleCounterToDisplayDateTimeUtc(endSample + 1);
    if (d.m_displayDateTimeUtc.isValid() && displayEndDateTimeUtc.isValid())
    {
        const qint64 displayDurationMS = d.m_displayDateTimeUtc.msecsTo(displayEndDateTimeUtc);
        if (displayDurationMS >= 0) {
            d.m_displayDurationS = std::max(0.001, (double) displayDurationMS / 1000.0);
        }
    }
    d.m_centerFrequency = 0.5 * (sweep.m_f0 + sweep.m_f1);
    d.m_frequencySpan = sweep.m_f1 - sweep.m_f0;
    d.m_frequencyDrift = slope * d.m_durationS;   // total Hz swept over the pass
    d.m_peakPower = sweep.m_peakPower;
    d.m_backgroundPower = sweep.m_fragCount > 0
        ? std::max(sweep.m_backgroundSum / sweep.m_fragCount, 1e-20) : 1e-20;
    d.m_totalPower = sweep.m_totalPower;
    d.m_sampleRate = sampleRate;

    qDebug() << "MeteorDemodSink::emitConsolidatedSweep:"
             << " startSample:" << sweep.m_startSample
             << " endSample:" << sweep.m_endSample
             << " durationS:" << d.m_durationS
             << " centerFrequency:" << d.m_centerFrequency
             << " frequencySpan:" << d.m_frequencySpan
             << " frequencyDrift:" << d.m_frequencyDrift
             << " fragments:" << sweep.m_fragCount;

    if (m_messageQueueToGUI) {
        m_messageQueueToGUI->push(MsgSatelliteDetected::create(d));
    }
}

void MeteorDemodSink::emitBlobDetection(const MeteorBlobDetector::Blob& blob)
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);

    PulseReport report;
    report.m_valid = true;
    // Blob samples are STFT frame CENTRES; the physical signal support extends half a frame
    // beyond each end (a one-column blob would otherwise report zero duration).
    const quint64 halfFrame = (quint64) std::max(0, m_spectralFrameSize / 2);
    report.m_startSample = blob.m_startSample > halfFrame ? blob.m_startSample - halfFrame : 0;
    report.m_endSample = blob.m_endSample + halfFrame;
    // Honour the maximum-duration setting: clip and flag rather than report unbounded events.
    const quint64 maxDurationSamples = (quint64) std::max(1,
        (int) ((qint64) m_settings.m_maxDurationMS * sampleRate / 1000));
    if (report.m_endSample - report.m_startSample + 1 > maxDurationSamples)
    {
        report.m_endSample = report.m_startSample + maxDurationSamples - 1;
        report.m_truncated = true;
    }
    report.m_dateTimeUtc = sampleCounterToDateTimeUtc(report.m_startSample);
    report.m_peakPower = blob.m_peakPower;
    report.m_backgroundPower = std::max(blob.m_backgroundPower, 1e-20);
    report.m_totalPower = blob.m_totalPower;    // 0 -> emitDetectionReport estimates it
    report.m_centerFrequency = 0.5 * (blob.m_f0 + blob.m_f1);
    report.m_frequencySpan = blob.m_f1 - blob.m_f0;
    report.m_durationS = report.m_endSample >= report.m_startSample
        ? (double) (report.m_endSample - report.m_startSample) / (double) sampleRate
        : 0.0;
    // total drift over the event from the blob's fitted frequency-track slope
    report.m_frequencyDrift = blob.m_slopeHzPerS * report.m_durationS;
    report.m_sampleRate = sampleRate;
    report.m_reportFrequencySpan = report.m_frequencySpan;

    // Duplicate check for blob emissions: only a GENUINE time overlap counts. The legacy
    // gap-based rule (suppress a compact detection up to 5 s after a similar-frequency one)
    // exists for the legacy path's multi-path re-detections, but the blob detector emits
    // each blob exactly once — and real meteors DO arrive seconds apart near the carrier
    // (measured: 138 such pairs in one overnight recording were being swallowed).
    {
        const double halfSpan = 0.5 * std::max(0.0, report.m_frequencySpan);

        if (overlapsRecentDetection(
                report.m_startSample,
                report.m_endSample,
                report.m_centerFrequency - halfSpan,
                report.m_centerFrequency + halfSpan)) {
            return;   // same event re-reported
        }
    }

    emitDetectionReport(report, "blob-detector");
}

bool MeteorDemodSink::overlapsRecentDetection(
    quint64 startSample,
    quint64 endSample,
    double lowFrequency,
    double highFrequency) const
{
    for (const DetectionRange& range : m_recentDetectionRanges)
    {
        const bool timeOverlap = (endSample >= range.m_startSample)
            && (startSample <= range.m_endSample);
        if (!timeOverlap) {
            continue;
        }
        const quint64 overlapLen = std::min(endSample, range.m_endSample)
            - std::max(startSample, range.m_startSample) + 1;
        const quint64 shorter = std::max<quint64>(1, std::min(
            endSample - startSample + 1,
            range.m_endSample - range.m_startSample + 1));
        const double overlapW = std::min(highFrequency, range.m_highFrequency)
            - std::max(lowFrequency, range.m_lowFrequency);
        const double minW = std::max(1.0, std::min(highFrequency - lowFrequency,
            range.m_highFrequency - range.m_lowFrequency));
        if (((double) overlapLen >= 0.5 * (double) shorter)
            && (overlapW >= m_detectorTunables.m_duplicate.m_duplicateFrequencyOverlapFraction * minW)) {
            return true;
        }
    }

    return false;
}

void MeteorDemodSink::rememberDetection(quint64 startSample, quint64 endSample, double centerFrequency, double frequencySpan)
{
    if (endSample < startSample) {
        return;
    }

    const double halfSpan = 0.5 * std::max(0.0, frequencySpan);
    m_recentDetectionRanges.push_back({startSample, endSample, centerFrequency - halfSpan, centerFrequency + halfSpan});
    pruneRecentDetections();
}

void MeteorDemodSink::pruneRecentDetections()
{
    if (m_recentDetectionRanges.empty()) {
        return;
    }

    const quint64 keepSamples = (quint64) std::max(1, m_settings.m_channelSampleRate) * 180;
    const quint64 oldestSample = m_sampleCounter > keepSamples ? m_sampleCounter - keepSamples : 0;

    m_recentDetectionRanges.erase(
        std::remove_if(
            m_recentDetectionRanges.begin(),
            m_recentDetectionRanges.end(),
            [oldestSample](const DetectionRange& range) { return range.m_endSample < oldestSample; }),
        m_recentDetectionRanges.end()
    );
}

void MeteorDemodSink::appendDetectionSample(const Complex& sample)
{
    const int capacity = (int) m_detectionSampleRing.size();

    if (capacity <= 0) {
        return;
    }

    if (m_detectionSampleRingCount == 0)
    {
        m_detectionSampleRingStart = 0;
        m_detectionSampleRingStartSample = m_sampleCounter;
    }

    if (m_detectionSampleRingCount < capacity)
    {
        const int index = (m_detectionSampleRingStart + m_detectionSampleRingCount) % capacity;
        m_detectionSampleRing[index] = sample;
        m_detectionSampleRingCount++;
    }
    else
    {
        m_detectionSampleRing[m_detectionSampleRingStart] = sample;
        m_detectionSampleRingStart = (m_detectionSampleRingStart + 1) % capacity;
        m_detectionSampleRingStartSample++;
    }
}

bool MeteorDemodSink::copyDetectionSamples(quint64 startSample, quint64 endSample, ComplexVector& samples) const
{
    samples.clear();

    if ((endSample < startSample) || (m_detectionSampleRingCount <= 0) || m_detectionSampleRing.empty()) {
        return false;
    }

    const quint64 historyEndSample = m_detectionSampleRingStartSample + (quint64) m_detectionSampleRingCount - 1;

    if ((startSample < m_detectionSampleRingStartSample) || (endSample > historyEndSample)) {
        return false;
    }

    const int capacity = (int) m_detectionSampleRing.size();
    const int count = (int) (endSample - startSample + 1);
    const int firstOffset = (int) (startSample - m_detectionSampleRingStartSample);
    samples.resize(count);

    for (int i = 0; i < count; i++) {
        samples[i] = m_detectionSampleRing[(m_detectionSampleRingStart + firstOffset + i) % capacity];
    }

    return true;
}

double MeteorDemodSink::estimatePulseTotalPower(quint64 startSample, quint64 endSample, double backgroundPower) const
{
    ComplexVector samples;

    if (!copyDetectionSamples(startSample, endSample, samples)) {
        return 0.0;
    }

    double totalPower = 0.0;

    for (const Complex& sample : samples) {
        totalPower += std::max(0.0, (double) std::norm(sample) - backgroundPower);
    }

    return totalPower;
}

void MeteorDemodSink::emitDetectionReport(const PulseReport& report, const char *source)
{
    emitDetectionReport(report, source, report.m_startSample, report.m_endSample);
}

void MeteorDemodSink::emitDetectionReport(
    const PulseReport& report,
    const char *source,
    quint64 duplicateStartSample,
    quint64 duplicateEndSample)
{
    if (!report.m_valid) {
        return;
    }

    const double peakPowerDB = 10.0 * std::log10(std::max(report.m_peakPower, 1e-20));
    const double backgroundPowerDB = 10.0 * std::log10(std::max(report.m_backgroundPower, 1e-20));
    double totalPower = report.m_totalPower;
    const quint64 sampleDuration = report.m_endSample >= report.m_startSample
        ? report.m_endSample - report.m_startSample + 1
        : 1;

    if (totalPower <= 0.0) {
        totalPower = estimatePulseTotalPower(report.m_startSample, report.m_endSample, report.m_backgroundPower);
    }

    if (totalPower <= 0.0) {
        totalPower = std::max(report.m_peakPower - report.m_backgroundPower, report.m_peakPower) * (double) std::max<quint64>(1, sampleDuration);
    }

    const double totalPowerDB = 10.0 * std::log10(std::max(totalPower, 1e-20));
    const quint64 displayStartSample = report.m_hasDisplaySamples ? report.m_displayStartSample : report.m_startSample;
    const quint64 displayEndSample = report.m_hasDisplaySamples ? report.m_displayEndSample : report.m_endSample;
    const QDateTime displayDateTimeUtc = sampleCounterToDisplayDateTimeUtc(displayStartSample);
    const QDateTime displayEndDateTimeUtc = sampleCounterToDisplayDateTimeUtc(displayEndSample + 1);
    double displayDurationS = report.m_durationS;

    if (displayDateTimeUtc.isValid() && displayEndDateTimeUtc.isValid())
    {
        const qint64 displayDurationMS = displayDateTimeUtc.msecsTo(displayEndDateTimeUtc);

        if (displayDurationMS >= 0) {
            displayDurationS = std::max(0.001, (double) displayDurationMS / 1000.0);
        }
    }

    qDebug() << "MeteorDemodSink::emitDetectionReport:"
             << " source:" << source
             << " dateTimeUtc:" << report.m_dateTimeUtc
             << " displayDateTimeUtc:" << displayDateTimeUtc
             << " durationS:" << report.m_durationS
             << " displayDurationS:" << displayDurationS
             << " peakPowerDB:" << peakPowerDB
             << " backgroundPowerDB:" << backgroundPowerDB
             << " totalPowerDB:" << totalPowerDB
             << " centerFrequency:" << report.m_centerFrequency
             << " frequencySpan:" << report.m_frequencySpan
             << " frequencyDrift:" << report.m_frequencyDrift
             << " startSample:" << report.m_startSample
             << " endSample:" << report.m_endSample
             << " displayStartSample:" << displayStartSample
             << " displayEndSample:" << displayEndSample;

    if (m_messageQueueToGUI)
    {
        DetectionRecord detection = report;
        detection.m_displayDateTimeUtc = displayDateTimeUtc;
        detection.m_displayDurationS = displayDurationS;
        detection.m_displayStartSample = displayStartSample;
        detection.m_displayEndSample = displayEndSample;
        detection.m_totalPower = totalPower;
        detection.m_sampleRate = m_settings.m_channelSampleRate;
        m_messageQueueToGUI->push(MsgMeteorDetected::create(detection));
    }

    rememberDetection(
        duplicateStartSample,
        duplicateEndSample,
        report.m_centerFrequency,
        report.m_duplicateFrequencySpan > 0.0 ? report.m_duplicateFrequencySpan : report.m_frequencySpan);

    // Send to event pipes
    if (!m_channel) {
        return;
    }

    QList<ObjectPipe*> eventPipes;
    MainCore::instance()->getMessagePipes().getMessagePipes(m_channel, "event", eventPipes);

    QString eventData = QString("peakPowerDB=%1,totalPowerDB=%2,duration=%3,truncated=%4")
        .arg(peakPowerDB)
        .arg(totalPowerDB)
        .arg(displayDurationS)
        .arg(report.m_truncated ? 1 : 0);

    for (const auto& pipe : eventPipes)
    {
        MessageQueue *messageQueue = qobject_cast<MessageQueue*>(pipe->m_element);

        if (messageQueue) {
            messageQueue->push(MainCore::MsgEvent::create(m_channel, displayDateTimeUtc, MainCore::MsgEvent::EventType::MeteorScatterEvent, eventData));
        }
    }
}

QDateTime MeteorDemodSink::sampleCounterToDateTimeUtc(quint64 sampleCounter) const
{
    if (!m_streamStartDateTimeUtc.isValid() || (m_settings.m_channelSampleRate <= 0)) {
        return QDateTime::currentDateTimeUtc();
    }

    const qint64 offsetMSecs = (qint64) std::llround(1000.0 * (double) sampleCounter / (double) m_settings.m_channelSampleRate);
    return m_streamStartDateTimeUtc.addMSecs(offsetMSecs);
}

QDateTime MeteorDemodSink::sampleCounterToDisplayDateTimeUtc(quint64 sampleCounter) const
{
    if (m_displayTimeAnchors.empty() || (m_settings.m_channelSampleRate <= 0)) {
        return sampleCounterToDateTimeUtc(sampleCounter);
    }

    auto anchorIt = std::lower_bound(
        m_displayTimeAnchors.begin(),
        m_displayTimeAnchors.end(),
        sampleCounter,
        [](const DisplayTimeAnchor& anchor, quint64 sample)
        {
            return anchor.m_sampleCounter < sample;
        });

    auto interpolateFromAnchor = [this](const DisplayTimeAnchor& anchor, quint64 sample) -> QDateTime
    {
        const double sampleDelta = (double) sample - (double) anchor.m_sampleCounter;
        const qint64 offsetMSecs = (qint64) std::llround(1000.0 * sampleDelta / (double) m_settings.m_channelSampleRate);
        return anchor.m_dateTimeUtc.addMSecs(offsetMSecs);
    };

    if (anchorIt == m_displayTimeAnchors.begin()) {
        return interpolateFromAnchor(*anchorIt, sampleCounter);
    }

    if (anchorIt == m_displayTimeAnchors.end()) {
        return interpolateFromAnchor(m_displayTimeAnchors.back(), sampleCounter);
    }

    if (anchorIt->m_sampleCounter == sampleCounter) {
        return anchorIt->m_dateTimeUtc;
    }

    const DisplayTimeAnchor& right = *anchorIt;
    const DisplayTimeAnchor& left = *(anchorIt - 1);
    const quint64 sampleSpan = right.m_sampleCounter - left.m_sampleCounter;

    if (sampleSpan == 0) {
        return left.m_dateTimeUtc;
    }

    const qint64 timeSpanMSecs = left.m_dateTimeUtc.msecsTo(right.m_dateTimeUtc);
    const double fraction = (double) (sampleCounter - left.m_sampleCounter) / (double) sampleSpan;
    return left.m_dateTimeUtc.addMSecs((qint64) std::llround((double) timeSpanMSecs * fraction));
}

void MeteorDemodSink::recordDisplayTimeAnchor()
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const quint64 anchorInterval = (quint64) std::max(1, sampleRate / 20);

    if (!m_displayTimeAnchors.empty() && (m_sampleCounter < m_nextDisplayTimeAnchorSample)) {
        return;
    }

    m_displayTimeAnchors.push_back({
        m_sampleCounter,
        QDateTime::currentDateTimeUtc()
    });
    m_nextDisplayTimeAnchorSample = m_sampleCounter + anchorInterval;

    const int maxDurationSamples = std::max(1, (int) ((qint64) m_settings.m_maxDurationMS * sampleRate / 1000));
    const quint64 keepSamples = (quint64) std::max(sampleRate * 120, maxDurationSamples + sampleRate * 20);

    if (m_sampleCounter <= keepSamples) {
        return;
    }

    const quint64 oldestSample = m_sampleCounter - keepSamples;
    auto firstKeep = std::lower_bound(
        m_displayTimeAnchors.begin(),
        m_displayTimeAnchors.end(),
        oldestSample,
        [](const DisplayTimeAnchor& anchor, quint64 sample)
        {
            return anchor.m_sampleCounter < sample;
        });

    if ((firstKeep != m_displayTimeAnchors.begin()) && (firstKeep != m_displayTimeAnchors.end())) {
        --firstKeep;
    }

    if (firstKeep != m_displayTimeAnchors.begin()) {
        m_displayTimeAnchors.erase(m_displayTimeAnchors.begin(), firstKeep);
    }
}

void MeteorDemodSink::emitDataCollectionMarker()
{
    if (!m_messageQueueToGUI) {
        return;
    }

    if (m_sampleCounter < m_nextDataMarkerSample) {
        return;
    }

    const QDateTime displayDateTimeUtc = sampleCounterToDisplayDateTimeUtc(m_sampleCounter);

    if (displayDateTimeUtc.isValid()) {
        m_messageQueueToGUI->push(MsgMeteorDataCollected::create(displayDateTimeUtc));
    }

    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    m_nextDataMarkerSample = m_sampleCounter + (quint64) sampleRate;
}

void MeteorDemodSink::configureInterpolator()
{
    if ((m_channelSampleRate <= 0) || (m_settings.m_channelSampleRate <= 0)) {
        return;
    }

    const Real cutoff = std::min<Real>(
        (Real) m_settings.m_channelSampleRate * 0.45f,
        (Real) m_channelSampleRate * 0.45f
    );
    m_interpolator.create(16, m_channelSampleRate, cutoff, 2.0f);
    m_interpolatorDistanceRemain = 0.0f;
    m_interpolatorDistance = (Real) m_channelSampleRate / (Real) m_settings.m_channelSampleRate;
}

void MeteorDemodSink::configureDetectionHistory()
{
    const int sampleRate = std::max(1, m_settings.m_channelSampleRate);
    const int maxDurationSamples = std::max(1, (int) ((qint64) m_settings.m_maxDurationMS * sampleRate / 1000));
    const int historyCapacity = maxDurationSamples + 5 * sampleRate;
    m_detectionSampleRing.assign(historyCapacity, Complex(0.0f, 0.0f));
    m_detectionSampleRingStart = 0;
    m_detectionSampleRingCount = 0;
    m_detectionSampleRingStartSample = m_sampleCounter;
}

void MeteorDemodSink::configureSpectralDetector()
{
    m_spectralFrameSize = std::clamp(
        (int) std::llround(
            m_detectorTunables.m_spectral.m_spectralFrameDurationS
                * (double) std::max(1, m_settings.m_channelSampleRate)),
        32,
        256);
    if ((m_spectralFrameSize % 2) != 0) {
        m_spectralFrameSize++;
    }
    m_spectralHopSize = std::max(
        1,
        (int) std::floor(
            m_detectorTunables.m_spectral.m_spectralHopFraction * (double) m_spectralFrameSize));
    m_spectralFrameBuffer.clear();
    configureDetectionHistory();
    configureBlobDetector();
}

void MeteorDemodSink::resizeSpectrumBuffer()
{
    m_spectrumBufferSize = std::max(16, m_settings.m_channelSampleRate / 10);
    m_spectrumBuffer.clear();
    m_spectrumBuffer.reserve(m_spectrumBufferSize);
}

void MeteorDemodSink::resetDetector()
{
    m_sampleCounter = 0;
    m_streamStartDateTimeUtc = QDateTime::currentDateTimeUtc();
    m_displayTimeAnchors.clear();
    m_nextDisplayTimeAnchorSample = 0;
    m_nextDataMarkerSample = 0;
    m_detectionSampleRingStart = 0;
    m_detectionSampleRingCount = 0;
    m_detectionSampleRingStartSample = 0;
    m_spectralFrameBuffer.clear();
    m_recentDetectionRanges.clear();
    m_blobDetector.reset();
    m_activeSweeps.clear();
    m_faintDashes.clear();
}

void MeteorDemodSink::feedSpectrum(const Complex& sample)
{
    if (!m_spectrumSink && !m_secondarySpectrumSink) {
        return;
    }

    m_spectrumBuffer.push_back(sample);

    if ((int) m_spectrumBuffer.size() >= m_spectrumBufferSize)
    {
        if (m_spectrumSink) {
            m_spectrumSink->feed(m_spectrumBuffer.begin(), m_spectrumBuffer.end(), false);
        }

        if (m_secondarySpectrumSink) {
            m_secondarySpectrumSink->feed(m_spectrumBuffer.begin(), m_spectrumBuffer.end(), false);
        }

        m_spectrumBuffer.clear();
    }
}

void MeteorDemodSink::applyChannelSettings(int channelSampleRate, int channelFrequencyOffset, bool force)
{
    qDebug() << "MeteorDemodSink::applyChannelSettings:"
            << " channelSampleRate:" << channelSampleRate
            << " channelFrequencyOffset:" << channelFrequencyOffset
            << " force:" << force;

    const bool retuned = m_channelFrequencyOffset != channelFrequencyOffset;

    if (retuned || (m_channelSampleRate != channelSampleRate) || force)
    {
        m_nco.setFreq(-channelFrequencyOffset, channelSampleRate);
        m_channelSampleRate = channelSampleRate;
        m_channelFrequencyOffset = channelFrequencyOffset;
        configureInterpolator();
    }

    // A retune changes what every buffered column means: drop the analysis window, active
    // sweeps and history so pre- and post-retune content cannot be linked into one event.
    if (retuned)
    {
        m_spectralFrameBuffer.clear();
        m_blobDetector.reset();
        m_activeSweeps.clear();
        m_faintDashes.clear();
        m_recentDetectionRanges.clear();
    }
}

void MeteorDemodSink::applySettings(const MeteorSettings& settings, const QStringList& settingsKeys, bool force)
{
    qDebug() << "MeteorDemodSink::applySettings:" << settings.getDebugString(settingsKeys, force)
             << " force:" << force;

    const bool sampleRateChanged = (settingsKeys.contains("channelSampleRate")
        && (settings.m_channelSampleRate != m_settings.m_channelSampleRate)) || force;
    const bool historyChanged = (settingsKeys.contains("maxDurationMS")
        && (settings.m_maxDurationMS != m_settings.m_maxDurationMS)) || force;
    const bool blobSensitivityChanged = (settingsKeys.contains("blobSensitivity")
        && (settings.m_blobSensitivity != m_settings.m_blobSensitivity)) || force;

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    // The sensitivity setting is the user-facing control for the blob detector; mirror it
    // into the tunables (tunable overrides applied later, e.g. by the test harness, win).
    if (blobSensitivityChanged) {
        m_detectorTunables.m_blob.m_scoreThreshold = m_settings.m_blobSensitivity;
    }

    if (sampleRateChanged)
    {
        configureInterpolator();
        configureSpectralDetector();
        resizeSpectrumBuffer();
        resetDetector();
    }
    else if (blobSensitivityChanged)
    {
        // live knob: adjust acceptance without resetting the analysis window
        m_blobDetector.setScoreThreshold(m_settings.m_blobSensitivity);
    }

    if (!sampleRateChanged && historyChanged) {
        configureDetectionHistory();
    }
}
