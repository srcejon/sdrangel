# Meteor Detector

The Meteor Detector channel is intended for RF meteor-scatter trail monitoring. It channelizes the selected receiver offset and resamples the channel to 100, 300, 1000, or 3000 Hz. The GUI displays the same resampled IQ in two independently configurable waterfalls. The trail-echo view defaults to a 1024-point FFT with 50% overlap for frequency resolution, while the head-echo view defaults to a 128-point FFT with 75% overlap and faster updates for time resolution.

Detection combines two paths. A scalar power gate finds increases above an adaptive noise floor, while an overlapping-FFT tracker follows narrow spectral bands through time. Candidates are scored from signal strength, integrated support, bandwidth, frequency coherence, duration, and track occupancy.

Accepted spectral components are associated with a parent meteor event before reporting. A parent uses a lower continuation threshold inside its established frequency region, protects that region from noise-floor adaptation, and tolerates short fades. Compatible components and scalar evidence are consolidated into one detection. Duration is measured from the complete parent interval; center frequency, span, and drift use robust weighted observations from its spectral components. Parent state and retained observations are bounded so long unattended runs have fixed memory and processing costs.

The regression harness can write a candidate audit CSV containing the parent event ID and association decision for each spectral candidate. A diagnostic callback is also available to capture the channelized IQ samples belonging to a finalized parent event without adding file I/O to the detector's sample-processing path.

Detector thresholds that represent time or frequency are expressed in seconds and Hz, then resolved and clamped when the channel sample rate changes. The audit also records rate-normalized frequency features so candidate data collected at different supported sample rates can be compared directly.

The detector computes several secondary features used for diagnostics and tightly bounded recovery:

- A block minimum-statistics spectral floor is compared with the active adaptive floor. The audit records their contrast and floor delta.
- An exponential decay template bank records the best underdense-trail envelope score, peak position, decay, and monotonic tail fraction.
- A weighted quadratic frequency fit records curvature and its improvement over the linear fit.
- A frozen standardized logistic model can be evaluated as a dot product when coefficients have been trained and enabled.

Calibrated rescue is limited to otherwise safe three-frame candidates with strong score, contrast, support, frequency coherence, and a decaying envelope. Two-frame candidates remain rejected because the available fixtures do not provide enough temporal evidence to distinguish them reliably. Settled spectral parents that are already at least two seconds long receive one bounded lead/trail envelope pass with hysteresis, allowing weak fireball tails to extend the report without changing short-event timing. Curvature rejection and the learned model remain disabled until a larger labeled corpus demonstrates a benefit.

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

Smooth, sustained Doppler sweeps and tracks beyond the configured drift limit are rejected to suppress satellite and other moving-carrier interference. Consequently, fast meteor head echoes that sweep by kilohertz per second are outside the detector's intended scope and may be rejected. The detector is designed primarily as a trail-echo counter.

The maximum duration is a reporting and counting limit. A longer coherent echo that otherwise passes the meteor checks is reported once with its duration clipped to the configured maximum; the detection is marked as truncated in the GUI tooltip. The detector then rearms only after the signal falls below the release threshold, avoiding repeated counts of the same long trail.
