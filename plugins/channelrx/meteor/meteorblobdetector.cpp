///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                          //
//                                                                                 //
// Exact port of the offline golden reference (test/detector2d/blobs.py            //
// detect_in_excess) run per chunk. Any change here must be re-verified against    //
// the Python parity fixture (test/detector2d/cpp).                                //
//                                                                                 //
// This program is free software; you can redistribute it and/or modify            //
// it under the terms of the GNU General Public License as published by            //
// the Free Software Foundation as version 3 of the License, or                    //
// (at your option) any later version.                                             //
///////////////////////////////////////////////////////////////////////////////////

#include "meteorblobdetector.h"

#include <cmath>
#include <algorithm>
#include <deque>

MeteorBlobDetector::MeteorBlobDetector() :
    m_nbins(0),
    m_windowCols(0),
    m_minCols(0),
    m_strideCols(1),
    m_finalMarginCols(0)
{
}

void MeteorBlobDetector::configure(const Config& cfg)
{
    m_cfg = cfg;
    reset();
}

void MeteorBlobDetector::reset()
{
    m_buf.clear();
    m_out.clear();
    m_segmentsOut.clear();
    m_activeBins.assign(m_nbins, 0);
    const double dt = std::max(1e-9, m_cfg.m_dt);
    const long long colCap = (1LL << 30);   // clamp so huge test configs don't overflow int
    m_windowCols = (int) std::min(colCap, std::max(2LL, (long long) std::llround(m_cfg.m_windowSeconds / dt)));
    m_minCols = (int) std::min(colCap, std::max(2LL, (long long) std::llround(m_cfg.m_minWindowSeconds / dt)));
    m_strideCols = (int) std::min(colCap, std::max(1LL, (long long) std::llround(m_cfg.m_emitStrideS / dt)));
    // final margin must exceed every mechanism that could still extend a "finished" blob:
    // the trace-link gap plus the bright-absorb reach (<=5 cols)
    m_finalMarginCols = std::max(m_cfg.m_linkMaxGapCols + 8,
                                 (int) std::llround(m_cfg.m_finalMarginS / dt));
    m_colsSinceProcess = 0;
    m_absStart = 0;
    m_emittedThroughAbs = -1;
}

void MeteorBlobDetector::processFrame(const std::vector<double>& binPower,
                                      const std::vector<double>& noiseFloor,
                                      std::uint64_t frameCenterSample)
{
    const int nb = (int) binPower.size();
    if (nb == 0) {
        return;
    }
    if (nb != m_nbins) {
        m_nbins = nb;
        reset();
    }

    Col col;
    col.m_excess.resize(nb);
    col.m_power.resize(nb);
    col.m_floor.resize(nb);
    col.m_sample = frameCenterSample;
    m_activeBins.assign(nb, 0);

    for (int f = 0; f < nb; f++)
    {
        const double p = std::max(binPower[f], 1e-30);
        col.m_power[f] = (float) p;
        if (m_cfg.m_medianFloor)
        {
            // excess + floor are computed per-chunk from the buffer median (processChunk);
            // no per-frame floor here, so activeBins is unused in this mode.
            col.m_excess[f] = 0.0f;
            col.m_floor[f] = 1.0f;
        }
        else
        {
            const double fl = std::max((f < (int) noiseFloor.size()) ? noiseFloor[f] : 1.0, 1e-30);
            const double ex = 10.0 * std::log10(p / fl) + m_cfg.m_floorOffsetDb;
            col.m_excess[f] = (float) ex;
            col.m_floor[f] = (float) fl;
            if (ex >= m_cfg.m_seedDb) {
                m_activeBins[f] = 1;
            }
        }
    }
    m_buf.push_back(std::move(col));
    m_colsSinceProcess++;

    // stride scheduling: once enough context exists, re-analyse the window every stride and
    // emit blobs finished more than the final margin behind the live edge
    if ((int) m_buf.size() >= m_minCols && m_colsSinceProcess >= m_strideCols)
    {
        const long long lastCol = m_absStart + (long long) m_buf.size() - 1;
        processChunk(lastCol - m_finalMarginCols);
        m_colsSinceProcess = 0;
        const int excess = (int) m_buf.size() - m_windowCols;
        if (excess > 0) {
            m_buf.erase(m_buf.begin(), m_buf.begin() + excess);
            m_absStart += excess;
        }
    }
}

void MeteorBlobDetector::flush()
{
    if (m_buf.empty()) {
        return;
    }
    processChunk(m_absStart + (long long) m_buf.size());   // emit everything left
    m_absStart += (long long) m_buf.size();
    m_buf.clear();
    m_colsSinceProcess = 0;
}

// ---------------------------------------------------------------------------------
// The per-chunk 2D algorithm — mirrors blobs.py detect_in_excess line by line.
// Image layout: y = frequency bin [0, nbins), x = buffer column [0, ncols).
// ---------------------------------------------------------------------------------

namespace {

struct Px { int y, x; };

int findRoot(std::vector<int>& parent, int i)
{
    while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
    return i;
}

} // namespace

void MeteorBlobDetector::processChunk(long long emitThroughAbs)
{
    const int nb = m_nbins;
    const int nc = (int) m_buf.size();
    if (nc == 0 || nb == 0) {
        return;
    }
    if (emitThroughAbs <= m_emittedThroughAbs) {
        return;   // nothing new can be final yet
    }

    // Median floor over the whole chunk (matches offline p - median(p)). Because median
    // commutes with the monotonic log, 10*log10(power/median_linear) == p_dB - median(p_dB)
    // exactly, so this reproduces the offline excess with no reconciliation offset.
    if (m_cfg.m_medianFloor)
    {
        std::vector<double> vals((size_t) nc);
        for (int y = 0; y < nb; y++)
        {
            for (int x = 0; x < nc; x++) {
                vals[x] = m_buf[x].m_power[y];
            }
            const int mid = nc / 2;
            std::nth_element(vals.begin(), vals.begin() + mid, vals.end());
            double floor = vals[mid];
            if ((nc % 2) == 0) {   // numpy median: average the two central order statistics
                const double lower = *std::max_element(vals.begin(), vals.begin() + mid);
                floor = 0.5 * (lower + vals[mid]);
            }
            floor = std::max(floor, 1e-30);
            for (int x = 0; x < nc; x++)
            {
                const double p = std::max((double) m_buf[x].m_power[y], 1e-30);
                m_buf[x].m_excess[y] = (float) (10.0 * std::log10(p / floor) + m_cfg.m_floorOffsetDb);
                m_buf[x].m_floor[y] = (float) floor;
            }
        }
    }

    auto EX = [&](int y, int x) -> double { return m_buf[x].m_excess[y]; };
    auto idx = [&](int y, int x) { return (size_t) y * nc + x; };

    // seed mask + per-column occupancy (fraction of the band at seed level)
    std::vector<std::uint8_t> seed((size_t) nb * nc, 0);
    std::vector<double> colOcc(nc, 0.0);
    for (int x = 0; x < nc; x++)
    {
        int cnt = 0;
        for (int y = 0; y < nb; y++) {
            if (EX(y, x) >= m_cfg.m_seedDb) { seed[idx(y, x)] = 1; cnt++; }
        }
        colOcc[x] = (double) cnt / nb;
    }

    // label seed components (8-connectivity, ndimage-compatible raster numbering)
    std::vector<int> seedLbl((size_t) nb * nc, 0);
    std::vector<long> seedSize(1, 0);
    {
        std::vector<Px> stack;
        int nextId = 1;
        for (int y = 0; y < nb; y++) for (int x = 0; x < nc; x++)
        {
            if (!seed[idx(y, x)] || seedLbl[idx(y, x)]) continue;
            const int id = nextId++;
            seedSize.push_back(0);
            stack.push_back({y, x});
            seedLbl[idx(y, x)] = id;
            while (!stack.empty())
            {
                Px p = stack.back(); stack.pop_back();
                seedSize[id]++;
                for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++)
                {
                    if (!dy && !dx) continue;
                    int yy = p.y + dy, xx = p.x + dx;
                    if (yy < 0 || yy >= nb || xx < 0 || xx >= nc) continue;
                    if (seed[idx(yy, xx)] && !seedLbl[idx(yy, xx)]) {
                        seedLbl[idx(yy, xx)] = id;
                        stack.push_back({yy, xx});
                    }
                }
            }
        }
    }

    // hysteresis: flood from seed components >= minPix into the grow mask (8-conn)
    std::vector<std::uint8_t> mask((size_t) nb * nc, 0);
    {
        std::vector<Px> stack;
        for (int y = 0; y < nb; y++) for (int x = 0; x < nc; x++)
        {
            int id = seedLbl[idx(y, x)];
            if (id && seedSize[id] >= m_cfg.m_minPix && !mask[idx(y, x)]) {
                mask[idx(y, x)] = 1;
                stack.push_back({y, x});
            }
        }
        while (!stack.empty())
        {
            Px p = stack.back(); stack.pop_back();
            for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++)
            {
                if (!dy && !dx) continue;
                int yy = p.y + dy, xx = p.x + dx;
                if (yy < 0 || yy >= nb || xx < 0 || xx >= nc) continue;
                if (!mask[idx(yy, xx)] && EX(yy, xx) >= m_cfg.m_growDb) {
                    mask[idx(yy, xx)] = 1;
                    stack.push_back({yy, xx});
                }
            }
        }
    }

    // vertical morphological close (height m_closeFreqBins), scipy border semantics (border=0)
    if (m_cfg.m_closeFreqBins > 1)
    {
        const int r = m_cfg.m_closeFreqBins / 2;
        std::vector<std::uint8_t> dil((size_t) nb * nc, 0);
        for (int x = 0; x < nc; x++) for (int y = 0; y < nb; y++)
        {
            if (!mask[idx(y, x)]) continue;
            for (int dy = -r; dy <= r; dy++) {
                int yy = y + dy;
                if (yy >= 0 && yy < nb) dil[idx(yy, x)] = 1;
            }
        }
        for (int x = 0; x < nc; x++) for (int y = 0; y < nb; y++)
        {
            bool all = true;
            for (int dy = -r; dy <= r && all; dy++) {
                int yy = y + dy;
                all = (yy >= 0 && yy < nb) ? (dil[idx(yy, x)] != 0) : false;
            }
            mask[idx(y, x)] = all ? 1 : 0;
        }
    }

    // final components (8-conn), keep >= minPix, with linking endpoints
    struct Comp {
        std::vector<Px> px;
        int cmin, cmax;
        double fstart, fend;
    };
    std::vector<Comp> comps;
    {
        std::vector<int> lbl((size_t) nb * nc, 0);
        std::vector<Px> stack, pixels;
        int nextId = 1;
        for (int y = 0; y < nb; y++) for (int x = 0; x < nc; x++)
        {
            if (!mask[idx(y, x)] || lbl[idx(y, x)]) continue;
            const int id = nextId++;
            pixels.clear();
            stack.push_back({y, x});
            lbl[idx(y, x)] = id;
            while (!stack.empty())
            {
                Px p = stack.back(); stack.pop_back();
                pixels.push_back(p);
                for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++)
                {
                    if (!dy && !dx) continue;
                    int yy = p.y + dy, xx = p.x + dx;
                    if (yy < 0 || yy >= nb || xx < 0 || xx >= nc) continue;
                    if (mask[idx(yy, xx)] && !lbl[idx(yy, xx)]) {
                        lbl[idx(yy, xx)] = id;
                        stack.push_back({yy, xx});
                    }
                }
            }
            if ((int) pixels.size() < m_cfg.m_minPix) continue;
            Comp c;
            c.px = pixels;
            c.cmin = 1 << 30; c.cmax = -1;
            for (const Px& p : pixels) { c.cmin = std::min(c.cmin, p.x); c.cmax = std::max(c.cmax, p.x); }
            auto colFreq = [&](int colx) {
                double s = 0; int n = 0;
                for (const Px& p : c.px) if (p.x == colx) { s += p.y; n++; }
                return m_cfg.m_fMinHz + std::rint(s / std::max(1, n)) * m_cfg.m_binHz;
            };
            c.fstart = colFreq(c.cmin);
            c.fend = colFreq(c.cmax);
            comps.push_back(std::move(c));
        }
    }

    // tight trajectory linking of temporal fragments (mirrors link_components)
    std::vector<int> parent(comps.size());
    for (size_t i = 0; i < comps.size(); i++) parent[i] = (int) i;
    {
        std::vector<int> order(comps.size());
        for (size_t i = 0; i < order.size(); i++) order[i] = (int) i;
        std::stable_sort(order.begin(), order.end(),
                         [&](int a, int b) { return comps[a].cmin < comps[b].cmin; });
        for (int bi : order)
        {
            const Comp& B = comps[bi];
            int best = -1; double bestd = 1e18;
            for (size_t ai = 0; ai < comps.size(); ai++)
            {
                if ((int) ai == bi) continue;
                const Comp& A = comps[ai];
                const int gap = B.cmin - A.cmax;
                if (gap < 0 || gap > m_cfg.m_linkMaxGapCols) continue;
                const double reach = m_cfg.m_linkMaxDriftHzPerS * (gap + 1) * m_cfg.m_dt + m_cfg.m_linkTolHz;
                const double d = std::fabs(B.fstart - A.fend);
                if (d <= reach && d < bestd) { bestd = d; best = (int) ai; }
            }
            if (best >= 0) parent[findRoot(parent, bi)] = findRoot(parent, best);
        }
    }

    // groups in first-seen-root order
    std::vector<std::vector<int>> groups;
    {
        std::vector<int> rootSlot((int) comps.size(), -1);
        for (size_t i = 0; i < comps.size(); i++)
        {
            int r = findRoot(parent, (int) i);
            if (rootSlot[r] < 0) { rootSlot[r] = (int) groups.size(); groups.push_back({}); }
            groups[rootSlot[r]].push_back((int) i);
        }
    }

    const double base = m_cfg.m_growDb;
    for (const std::vector<int>& g : groups)
    {
        std::vector<Px> pts;
        for (int ci : g) pts.insert(pts.end(), comps[ci].px.begin(), comps[ci].px.end());
        if ((int) pts.size() < m_cfg.m_minPix) continue;

        // energy trim (keep >= trimKeep) that never trims bright pixels
        {
            int ymin = 1 << 30, ymax = -1, xmin = 1 << 30, xmax = -1;
            double tot = 0;
            for (const Px& p : pts) {
                ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
                xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
                tot += std::max(EX(p.y, p.x) - base, 0.0);
            }
            if (tot > 0)
            {
                const double dropF = (1.0 - m_cfg.m_trimKeepFreq) / 2.0 * tot;   // freq rows
                const double dropT = (1.0 - m_cfg.m_trimKeepTime) / 2.0 * tot;   // time cols
                std::vector<double> yprof(ymax - ymin + 1, 0.0), xprof(xmax - xmin + 1, 0.0);
                for (const Px& p : pts) {
                    const double w = std::max(EX(p.y, p.x) - base, 0.0);
                    yprof[p.y - ymin] += w;
                    xprof[p.x - xmin] += w;
                }
                for (size_t i = 1; i < yprof.size(); i++) yprof[i] += yprof[i - 1];
                for (size_t i = 1; i < xprof.size(); i++) xprof[i] += xprof[i - 1];
                auto lower = [](const std::vector<double>& c, double v) {
                    return (int) (std::lower_bound(c.begin(), c.end(), v) - c.begin());
                };
                const int y0 = ymin + lower(yprof, dropF), y1 = ymin + lower(yprof, tot - dropF);
                const int x0 = xmin + lower(xprof, dropT), x1 = xmin + lower(xprof, tot - dropT);
                std::vector<Px> kept;
                for (const Px& p : pts) {
                    const bool inBox = (p.y >= y0 && p.y <= y1 && p.x >= x0 && p.x <= x1);
                    const bool bright = EX(p.y, p.x) >= m_cfg.m_seedDb;
                    if (inBox || bright) kept.push_back(p);
                }
                pts.swap(kept);
            }
        }
        if ((int) pts.size() < m_cfg.m_minPix) continue;

        // absorb bright pixels adjacent to the box (iterative, matches _absorb_bright)
        {
            std::vector<std::uint8_t> inSet((size_t) nb * nc, 0);
            for (const Px& p : pts) inSet[idx(p.y, p.x)] = 1;
            for (int it = 0; it < 5; it++)
            {
                int ymin = 1 << 30, ymax = -1, xmin = 1 << 30, xmax = -1;
                for (const Px& p : pts) {
                    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
                    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
                }
                const int y0 = std::max(0, ymin - 1), y1 = std::min(nb - 1, ymax + 1);
                const int x0 = std::max(0, xmin - 1), x1 = std::min(nc - 1, xmax + 1);
                bool grew = false;
                for (int y = y0; y <= y1; y++) for (int x = x0; x <= x1; x++)
                {
                    if (!inSet[idx(y, x)] && EX(y, x) >= m_cfg.m_seedDb) {
                        inSet[idx(y, x)] = 1;
                        pts.push_back({y, x});
                        grew = true;
                    }
                }
                if (!grew) break;
            }
        }

        int ymin = 1 << 30, ymax = -1, xmin = 1 << 30, xmax = -1;
        int yminB = 1 << 30, ymaxB = -1;   // frequency extent of BRIGHT (seed-level) pixels
        double score = 0, peakP = 0, totalP = 0, floorSum = 0, maxOcc = 0, maxEx = -1e30;
        for (const Px& p : pts)
        {
            ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
            xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
            const double ex = EX(p.y, p.x);
            if (ex >= m_cfg.m_seedDb) { yminB = std::min(yminB, p.y); ymaxB = std::max(ymaxB, p.y); }
            score += std::max(ex - base, 0.0);
            maxEx = std::max(maxEx, ex);
            peakP = std::max(peakP, (double) m_buf[p.x].m_power[p.y]);
            totalP += m_buf[p.x].m_power[p.y];
            floorSum += m_buf[p.x].m_floor[p.y];
        }
        {
            std::vector<std::uint8_t> seenCol(nc, 0);
            for (const Px& p : pts) {
                if (!seenCol[p.x]) { seenCol[p.x] = 1; maxOcc = std::max(maxOcc, colOcc[p.x]); }
            }
        }
        if (ymaxB < 0) { yminB = ymin; ymaxB = ymax; }   // no bright pixels: fall back to full extent

        // emit each blob exactly once: when its last column first becomes final
        // (older than emitThroughAbs but newer than the previous emission boundary)
        const long long endAbs = m_absStart + xmax;
        if (endAbs > emitThroughAbs || endAbs <= m_emittedThroughAbs) continue;

        // Sweep discrimination: linear fit of the per-column frequency-centroid track.
        // A Doppler sweep is a long, straight, drifting line (lin_r2 ~1, |slope| large);
        // meteors are vertical/irregular. Mirrors blobs.py features_from_mask.
        double slope = 0.0, linR2 = 0.0;
        {
            const int wcols = xmax - xmin + 1;
            std::vector<double> sumY(wcols, 0.0);
            std::vector<int> cnt(wcols, 0);
            for (const Px& p : pts) { sumY[p.x - xmin] += p.y; cnt[p.x - xmin]++; }
            std::vector<double> tv, fv;
            tv.reserve(wcols); fv.reserve(wcols);
            for (int c = 0; c < wcols; c++) {
                if (cnt[c] > 0) {
                    const int yi = (int) std::lround(sumY[c] / cnt[c]);
                    tv.push_back((double) c * m_cfg.m_dt);
                    fv.push_back(m_cfg.m_fMinHz + yi * m_cfg.m_binHz);
                }
            }
            const int n = (int) tv.size();
            if (n >= 3) {
                double mt = 0, mf = 0;
                for (int i = 0; i < n; i++) { mt += tv[i]; mf += fv[i]; }
                mt /= n; mf /= n;
                double sxx = 0, sxy = 0, sff = 0;
                for (int i = 0; i < n; i++) {
                    const double dtv = tv[i] - mt, dfv = fv[i] - mf;
                    sxx += dtv * dtv; sxy += dtv * dfv; sff += dfv * dfv;
                }
                if (sxx > 1e-12) {
                    slope = sxy / sxx;
                    double ssRes = 0;
                    for (int i = 0; i < n; i++) {
                        const double r = fv[i] - (mf + slope * (tv[i] - mt));
                        ssRes += r * r;
                    }
                    linR2 = 1.0 - ssRes / (sff + 1e-9);
                }
            }
        }
        const double durS = (double) (xmax - xmin) * m_cfg.m_dt;
        const bool isSweep = (linR2 >= m_cfg.m_sweepMinLinR2)
                          && (std::fabs(slope) >= m_cfg.m_sweepMinAbsSlopeHzPerS)
                          && (durS >= m_cfg.m_sweepMinDurationS);

        Blob b;
        const double absT0 = (double) (m_absStart + xmin) * m_cfg.m_dt;
        const double absT1 = (double) (m_absStart + xmax) * m_cfg.m_dt;
        b.m_t0 = absT0 - m_cfg.m_dt;                                  // 1-column display pad
        b.m_t1 = absT1 + m_cfg.m_dt;
        // frequency box from bright pixels (the "red blob") + 1-bin display pad
        b.m_f0 = m_cfg.m_fMinHz + yminB * m_cfg.m_binHz - m_cfg.m_binHz;
        b.m_f1 = m_cfg.m_fMinHz + ymaxB * m_cfg.m_binHz + m_cfg.m_binHz;
        b.m_startSample = m_buf[xmin].m_sample;
        b.m_endSample = m_buf[xmax].m_sample;
        b.m_npix = (long) pts.size();
        b.m_score = score;
        b.m_peakExcessDb = maxEx;
        b.m_maxOcc = maxOcc;
        b.m_linR2 = linR2;
        b.m_slopeHzPerS = slope;
        b.m_peakPower = peakP;
        b.m_totalPower = totalP;
        b.m_backgroundPower = pts.empty() ? 1e-30 : floorSum / pts.size();
        b.m_sweepRejected = isSweep;
        b.m_accepted = (score >= m_cfg.m_scoreThreshold) && (maxOcc <= m_cfg.m_occLimit)
                    && (maxEx >= m_cfg.m_minPeakExcessDb) && !isSweep;
        m_out.push_back(b);
    }

    // ---- weak-sweep EXTENSION source: faint linear segments below the seed --------------
    // 8-connected components of faint pixels, kept only if elongated + internally linear +
    // drifting in the satellite band. The sink attaches ones that continue a known sweep's
    // drift line; faint-only discovery is NOT done (calibration: inseparable from noise).
    if (m_cfg.m_extractSweepSegments)
    {
        std::vector<std::uint8_t> fseen((size_t) nb * nc, 0);
        std::vector<Px> stack, pix;
        for (int sy = 0; sy < nb; sy++) for (int sx = 0; sx < nc; sx++)
        {
            if (fseen[idx(sy, sx)] || EX(sy, sx) < m_cfg.m_faintSeedDb) continue;
            pix.clear();
            stack.push_back({sy, sx}); fseen[idx(sy, sx)] = 1;
            while (!stack.empty())
            {
                Px p = stack.back(); stack.pop_back(); pix.push_back(p);
                for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++)
                {
                    if (!dy && !dx) continue;
                    int yy = p.y + dy, xx = p.x + dx;
                    if (yy < 0 || yy >= nb || xx < 0 || xx >= nc) continue;
                    if (!fseen[idx(yy, xx)] && EX(yy, xx) >= m_cfg.m_faintSeedDb) {
                        fseen[idx(yy, xx)] = 1; stack.push_back({yy, xx});
                    }
                }
            }
            if ((int) pix.size() < m_cfg.m_segMinPix) continue;

            int xmn = 1 << 30, xmx = -1, ymn = 1 << 30, ymx = -1;
            for (const Px& p : pix) {
                xmn = std::min(xmn, p.x); xmx = std::max(xmx, p.x);
                ymn = std::min(ymn, p.y); ymx = std::max(ymx, p.y);
            }
            const int wcols = xmx - xmn + 1;
            std::vector<double> sumY(wcols, 0.0);
            std::vector<int> cnt(wcols, 0);
            for (const Px& p : pix) { sumY[p.x - xmn] += p.y; cnt[p.x - xmn]++; }
            std::vector<double> tv, fv;
            for (int c = 0; c < wcols; c++) {
                if (cnt[c] > 0) {
                    const int yi = (int) std::lround(sumY[c] / cnt[c]);
                    tv.push_back((double) c * m_cfg.m_dt);
                    fv.push_back(m_cfg.m_fMinHz + yi * m_cfg.m_binHz);
                }
            }
            if ((int) tv.size() < m_cfg.m_segMinCols) continue;

            const int n = (int) tv.size();
            double mt = 0, mf = 0;
            for (int i = 0; i < n; i++) { mt += tv[i]; mf += fv[i]; }
            mt /= n; mf /= n;
            double sxx = 0, sxy = 0, sff = 0;
            for (int i = 0; i < n; i++) {
                const double dtv = tv[i] - mt, dfv = fv[i] - mf;
                sxx += dtv * dtv; sxy += dtv * dfv; sff += dfv * dfv;
            }
            if (sxx <= 1e-12) continue;
            const double slope = sxy / sxx;
            double ssRes = 0;
            for (int i = 0; i < n; i++) {
                const double r = fv[i] - (mf + slope * (tv[i] - mt));
                ssRes += r * r;
            }
            const double r2 = 1.0 - ssRes / (sff + 1e-9);
            if (r2 < m_cfg.m_segMinR2
                || std::fabs(slope) < m_cfg.m_segSlopeMinHzPerS
                || std::fabs(slope) > m_cfg.m_segSlopeMaxHzPerS) {
                continue;
            }
            const long long segEndAbs = m_absStart + xmx;
            if (segEndAbs > emitThroughAbs || segEndAbs <= m_emittedThroughAbs) continue;

            SweepSegment s;
            s.m_t0 = (double) (m_absStart + xmn) * m_cfg.m_dt;
            s.m_t1 = (double) (m_absStart + xmx) * m_cfg.m_dt;
            s.m_f0 = m_cfg.m_fMinHz + ymn * m_cfg.m_binHz;
            s.m_f1 = m_cfg.m_fMinHz + ymx * m_cfg.m_binHz;
            s.m_fc = mf;
            s.m_slopeHzPerS = slope;
            s.m_r2 = r2;
            s.m_startSample = m_buf[xmn].m_sample;
            s.m_endSample = m_buf[xmx].m_sample;
            s.m_npix = (long) pix.size();
            double segPeak = 0, segTot = 0, segFloor = 0;
            for (const Px& p : pix) {
                segPeak = std::max(segPeak, (double) m_buf[p.x].m_power[p.y]);
                segTot += m_buf[p.x].m_power[p.y];
                segFloor += m_buf[p.x].m_floor[p.y];
            }
            s.m_peakPower = segPeak;
            s.m_totalPower = segTot;
            s.m_backgroundPower = pix.empty() ? 1e-20 : std::max(segFloor / pix.size(), 1e-20);
            m_segmentsOut.push_back(s);
        }
    }

    m_emittedThroughAbs = emitThroughAbs;
}

std::vector<MeteorBlobDetector::Blob> MeteorBlobDetector::takeBlobs()
{
    std::vector<Blob> out;
    for (const Blob& b : m_out) {
        // accepted meteors AND sweep fragments (the sink consolidates sweeps); drop noise.
        if (b.m_accepted || b.m_sweepRejected) out.push_back(b);
    }
    m_out.clear();
    return out;
}

std::vector<MeteorBlobDetector::SweepSegment> MeteorBlobDetector::takeSweepSegments()
{
    std::vector<SweepSegment> out;
    out.swap(m_segmentsOut);
    return out;
}
