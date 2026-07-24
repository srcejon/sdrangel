///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2026 Jon Beniston, M7RCE                                          //
//                                                                                 //
// 2D meteor detector: chunked spectrogram-image blob detection with an            //
// integrated-power acceptance rule (user-calibrated sensitivity threshold).       //
//                                                                                 //
// The algorithm is an exact port of the offline golden reference                  //
// (plugins/channelrx/meteor/test/detector2d/blobs.py detect_in_excess), which was //
// validated in two user review rounds (acceptance calibration + box quality):     //
//   seed >= seedDb (8-connectivity), seed components >= minPix grow into          //
//   >= growDb (hysteresis), vertical morphological close, tight trajectory        //
//   linking of temporal fragments, noise-robust score = sum(excess - growDb),     //
//   energy-trimmed box that never trims bright pixels + absorbs adjacent bright   //
//   pixels, box padded by one column/bin for display alignment.                   //
// Acceptance = score >= scoreThreshold (user knob) AND column occupancy below     //
// the broadband-impulse limit. No ML classifier.                                  //
//                                                                                 //
// Chunked processing: columns are buffered (pad|core|pad) and each chunk is       //
// processed as a whole image with the exact offline algorithm; blobs centred in   //
// the core are emitted, and the trailing pad carries over as the next chunk's     //
// leading context. Worst-case reporting latency = core + pad seconds.             //
//                                                                                 //
// This program is free software; you can redistribute it and/or modify            //
// it under the terms of the GNU General Public License as published by            //
// the Free Software Foundation as version 3 of the License, or                    //
// (at your option) any later version.                                             //
///////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDE_METEORBLOBDETECTOR_H
#define INCLUDE_METEORBLOBDETECTOR_H

#include <cstdint>
#include <vector>

class MeteorBlobDetector
{
public:
    struct Config
    {
        // dB thresholds are over the offline median-floor scale; excess computed from the
        // plugin's EMA floor is calibrated by +m_floorOffsetDb (measured offset +1.54 dB).
        double m_seedDb = 11.0;
        double m_growDb = 6.5;
        double m_floorOffsetDb = 1.54;
        // When true, the noiseFloor argument to processFrame is ignored and the per-bin
        // floor is the MEDIAN power over the whole buffered chunk (exactly the offline
        // reference: excess = power - per-bin-median). A median is non-adaptive, so a
        // meteor's faint decay tail cannot pull the floor up and self-mask (the EMA floor
        // did, truncating long-meteor durations). No offset is needed in this mode.
        bool   m_medianFloor = false;
        int    m_minPix = 3;
        int    m_closeFreqBins = 3;        // vertical morphological close height (1 = off)
        int    m_linkMaxGapCols = 3;       // trajectory linking: max column gap
        double m_linkMaxDriftHzPerS = 300.0;
        double m_linkTolHz = 40.0;
        double m_trimKeepTime = 0.97;      // energy kept along time (loose: preserve tails)
        double m_trimKeepFreq = 0.90;      // energy kept along frequency (tight: hug the core)
        double m_occLimit = 0.5;           // broadband-impulse gate: reject maxOcc above this
        // Peak-brightness gate ("red blob"): a real meteor peaks well above the floor; faint
        // spread noise blobs and weak interference clear the score by area but never get bright.
        double m_minPeakExcessDb = 16.0;
        // Sweep rejection: an elongated, linear, drifting frequency track is a Doppler sweep
        // (satellite), not a meteor. Reject when all three hold (meteors are vertical/irregular:
        // lin_r2 < ~0.5; a flat meteor has slope ~0 so the slope gate spares it).
        double m_sweepMinLinR2 = 0.9;
        double m_sweepMinAbsSlopeHzPerS = 20.0;
        double m_sweepMinDurationS = 0.4;
        double m_scoreThreshold = 145.0;   // USER SENSITIVITY KNOB (integrated excess dB)
        // Low-latency stride scheduling: a rolling window of columns is re-analysed every
        // stride, and a blob is emitted as soon as it is provably FINISHED — its last column
        // is more than the final margin behind the live edge, so no later column can extend
        // it (the margin exceeds the trace-link gap). Latency after a meteor ends is
        // ~finalMargin + stride (~1 s), instead of waiting for a chunk boundary (5–15 s).
        // The window must stay much longer than the longest meteor or the per-window median
        // floor absorbs the meteor and truncates its tail (measured earlier).
        double m_windowSeconds = 20.0;     // rolling stats/analysis window (max intact meteor)
        double m_minWindowSeconds = 10.0;  // don't detect until this much context exists
        double m_emitStrideS = 0.5;        // re-analysis cadence
        double m_finalMarginS = 0.5;       // a blob ending this far behind "now" is final
        // Weak-sweep EXTENSION: extract faint linear segments (below the seed) so the sink can
        // attach ones that continue an already-detected sweep's drift line. Off unless the sink
        // enables it. Discovery of faint-only sweeps is deliberately NOT done — calibration
        // showed faint chains are inseparable from noise (would add false satellite rows).
        bool   m_extractSweepSegments = false;
        double m_faintSeedDb = 9.0;        // faint threshold for segment pixels
        int    m_segMinCols = 4;           // segment must span this many time columns
        int    m_segMinPix = 4;
        double m_segMinR2 = 0.8;           // internal linearity of the segment's own track
        double m_segSlopeMinHzPerS = 20.0; // satellite drift band (spares meteors/carriers)
        double m_segSlopeMaxHzPerS = 1000.0;
        double m_binHz = 1.0;              // FFT bin spacing, set from sink config
        double m_dt = 1.0;                 // hop interval (s), set from sink config
        double m_fMinHz = 0.0;             // frequency of bin 0
    };

    struct Blob
    {
        double m_t0 = 0.0, m_t1 = 0.0;       // padded box, seconds (stream-relative)
        double m_f0 = 0.0, m_f1 = 0.0;       // padded box, Hz
        std::uint64_t m_startSample = 0, m_endSample = 0;
        long   m_npix = 0;
        double m_score = 0.0;                // integrated excess above growDb (the knob metric)
        double m_peakExcessDb = 0.0;         // brightest pixel's excess over the floor
        double m_maxOcc = 0.0;               // max per-column band occupancy over the blob
        double m_linR2 = 0.0;                // linearity of the freq-vs-time centroid track
        double m_slopeHzPerS = 0.0;          // drift of that track (Hz/s)
        double m_peakPower = 0.0, m_backgroundPower = 0.0, m_totalPower = 0.0;
        bool   m_sweepRejected = false;      // looked like a Doppler sweep, not a meteor
        bool   m_accepted = false;
    };

    // A faint linear segment (below the seed) that may continue a known sweep's drift line.
    struct SweepSegment
    {
        double m_t0 = 0.0, m_t1 = 0.0;
        double m_f0 = 0.0, m_f1 = 0.0, m_fc = 0.0;
        double m_slopeHzPerS = 0.0, m_r2 = 0.0;
        std::uint64_t m_startSample = 0, m_endSample = 0;
        int    m_cols = 0;                 // time columns spanned (shape reliability indicator)
        long   m_npix = 0;
        double m_peakPower = 0.0, m_backgroundPower = 1e-20, m_totalPower = 0.0;
    };

    MeteorBlobDetector();
    void configure(const Config& cfg);
    void reset();

    // Feed one STFT frame (linear bin powers + current per-bin EMA noise floor).
    void processFrame(const std::vector<double>& binPower,
                      const std::vector<double>& noiseFloor,
                      std::uint64_t frameCenterSample);

    // Emit everything buffered without waiting for the final margin (end of stream or
    // inactivity). The analysis window is preserved so a resuming stream keeps its noise
    // context. Returns true if anything new was analysed.
    bool flush();

    // Accepted blobs completed since the last call (moved out).
    std::vector<Blob> takeBlobs();

    // Faint linear segments from processed chunks (only when m_extractSweepSegments).
    std::vector<SweepSegment> takeSweepSegments();

    // Bins at seed level in the most recent frame: the sink applies the slow "active"
    // noise-floor alpha to these so a long bright event cannot self-mask.
    const std::vector<std::uint8_t>& activeBins() const { return m_activeBins; }

    const Config& config() const { return m_cfg; }

    // Live sensitivity update: acceptance is evaluated at emission time, so this takes
    // effect immediately without resetting the analysis window.
    void setScoreThreshold(double threshold) { m_cfg.m_scoreThreshold = threshold; }

    // True when any columns are buffered (flush() itself reports whether it did new work).
    bool hasBufferedColumns() const { return !m_buf.empty(); }

private:
    struct Col
    {
        std::vector<float> m_excess;     // calibrated dB over floor
        std::vector<float> m_power;      // linear bin power
        std::vector<float> m_floor;      // linear floor
        std::uint64_t m_sample = 0;      // frame centre sample index
    };

    // Analyse the whole buffer; emit blobs/segments whose last column is <= emitThroughAbs
    // and not yet emitted (tracked via m_emittedThroughAbs).
    void processChunk(long long emitThroughAbs);

    Config m_cfg;
    int m_nbins;
    int m_windowCols, m_minCols, m_strideCols, m_finalMarginCols;
    int m_colsSinceProcess = 0;
    long long m_absStart = 0;            // absolute column index of m_buf[0]
    long long m_emittedThroughAbs = -1;  // blobs ending at or before this column are emitted
    std::vector<Col> m_buf;              // rolling analysis window
    std::vector<std::uint8_t> m_activeBins;
    std::vector<Blob> m_out;
    std::vector<SweepSegment> m_segmentsOut;
};

#endif // INCLUDE_METEORBLOBDETECTOR_H
