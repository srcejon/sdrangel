# Meteor Detector

The Meteor Detector channel is intended for RF meteor-scatter trail monitoring. It channelizes the selected receiver offset and resamples the channel to 100, 300, 1000, or 3000 Hz. The GUI displays the same resampled IQ in two independently configurable waterfalls. The trail-echo view defaults to a 1024-point FFT with 50% overlap for frequency resolution, while the head-echo view defaults to a 128-point FFT with 75% overlap and faster updates for time resolution.

The Geometry rollup records the radar transmitter latitude and longitude, plus the receive antenna azimuth, elevation, and half-power beamwidth. Receive pointing can be entered manually or synchronized once per second from a selected GS232Controller feature. Selecting no rotator restores manual azimuth and elevation entry. The current receive pointing and beamwidth are included in exported RMOB reports.

Detection combines two paths. A scalar power gate finds increases above an adaptive noise floor, while an overlapping-FFT tracker follows narrow spectral bands through time. Candidates are scored from signal strength, integrated support, bandwidth, frequency coherence, duration, and track occupancy.

Accepted spectral components are associated with a parent meteor event before reporting. A parent uses a lower continuation threshold inside its established frequency region, protects that region from noise-floor adaptation, and tolerates short fades. Compatible components and scalar evidence are consolidated into one detection. Duration is measured from the complete parent interval; center frequency, span, and drift use robust weighted observations from its spectral components. Parent state and retained observations are bounded so long unattended runs have fixed memory and processing costs.

The regression harness can write a candidate audit CSV containing the parent event ID and association decision for each spectral candidate. A diagnostic callback is also available to capture the channelized IQ samples belonging to a finalized parent event without adding file I/O to the detector's sample-processing path.

Detector thresholds that represent time or frequency are expressed in seconds and Hz, then resolved and clamped when the channel sample rate changes. The audit also records rate-normalized frequency features so candidate data collected at different supported sample rates can be compared directly.

The detector computes several secondary features used for diagnostics and tightly bounded recovery:

- A block minimum-statistics spectral floor is compared with the active adaptive floor. The audit records their contrast and floor delta.
- An exponential decay template bank records the best underdense-trail envelope score, peak position, decay, and monotonic tail fraction.
- A weighted quadratic frequency fit records curvature and its improvement over the linear fit.
- A frozen standardized logistic model can be evaluated as a dot product when coefficients have been trained and enabled.

Calibrated rescue is limited to otherwise safe three-frame candidates with strong score, contrast, support, frequency coherence, and a decaying envelope. Two-frame candidates are accepted only for tightly bounded compact or wide-band morphologies with strong contrast, support, and frequency coherence. Settled spectral parents that are already at least two seconds long receive one bounded lead/trail envelope pass with hysteresis, allowing weak fireball tails to extend the report without changing short-event timing. Curvature rejection and the learned model remain disabled until a larger labeled corpus demonstrates a benefit.

An otherwise strong one- or two-frame candidate is held for one second before its final frame-gate decision. The detector then performs a bounded envelope reanalysis from the sample history and accepts it only when the expanded interval contains at least 180 ms of prominent, frequency-coherent narrow-line support. Two-frame recovery additionally requires a moderately wide original band, or an exceptionally strong narrower band. A second settled-track check still rejects smooth Doppler sweeps. The broad delayed interval is used only for validation; a tighter component envelope supplies the reported time, duration, and total power. A recovered component that mostly overlaps an active meteor confirms that event without enlarging its measurements or adding another count. Candidates that fail the spectral-evidence gate are not recovered by this path because reviewed weak-evidence sweeps are not yet separated reliably enough.

The standalone regression harness can build such a corpus:

```text
meteor_demod_sink_test --wav recording.wav --candidate-csv candidates.csv --candidate-capture-dir candidate-iq
```

Rejected candidates close to the score boundary are written as stereo signed 16-bit little-endian IQ (`.ci16`) only when a capture directory is requested. Recording-level labels can be supplied separately:

```text
startSample,endSample,label
120000,123000,meteor
240000,245000,interference
```

For overlapping events or recordings containing several simultaneous signals, labels can also include a frequency interval and stable event ID:

```text
startSample,endSample,lowFrequencyHz,highFrequencyHz,label,eventId
120000,123000,35,85,meteor,M001
120000,123000,-160,-80,interference,I001
```

Pass this file with `--candidate-labels labels.csv`. Overlapping candidates inherit the label in the audit CSV. `test/train_candidate_model.py` fits a balanced L2 logistic model, reports leave-one-recording-out metrics, writes boundary cases for review, and prints C++ initializers for a frozen model. Hard duration, usable-bandwidth, duplicate, broadband-impulse, and sweep gates remain in force even when learned rescue is enabled.

`test/render_candidate_review.py` renders prioritized five-second waterfall windows from an audit and its original WAV. It includes newly accepted candidates relative to an optional baseline audit, positive-margin gate failures, sweep and duplicate rejects, and candidate-free control windows. The generated `review.csv` provides blank label, event ID, and notes fields for manual classification.

`test/label_candidates.py` serves a local web page for one-keystroke labeling: it shows one candidate waterfall at a time from an audit and its WAV, with meteor, sweep, interference, noise, and unsure buttons, a prioritized queue (recovery acceptances, then acceptances, then boundary rejects), and resume support. Labels are written directly in the `--candidate-labels` format on every click.

The regression harness accepts repeated `--tunable name=value` overrides for any `DetectorTunables` member (names drop the `m_` prefix). This allows detector operating points to be swept against a labeled corpus without rebuilding; the current defaults were chosen from such a sweep, which recovered roughly one hundred verified meteors over 8.8 hours of labeled recording at a cost of one known false accept.

Regression expectation CSVs may append a `required` column. Required rows must be present and match all measurements. Optional rows describe genuinely ambiguous visual cases: they may be absent, but if detected their measurements must still match. Files without this column retain the original all-required behavior.

Smooth, sustained Doppler sweeps and tracks beyond the configured drift limit are rejected to suppress satellite and other moving-carrier interference. A short 4- or 5-frame sweep can survive this gate only when it is spectrally coherent, locally strong, and compact in time, which retains brief head-echo-like meteor signatures without accepting sustained moving carriers. The detector remains primarily a trail-echo counter.

The maximum duration is a reporting and counting limit. A longer coherent echo that otherwise passes the meteor checks is reported once with its duration clipped to the configured maximum; the detection is marked as truncated in the GUI tooltip. The detector then rearms only after the signal falls below the release threshold, avoiding repeated counts of the same long trail.
