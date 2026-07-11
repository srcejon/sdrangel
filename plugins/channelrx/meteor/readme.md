# Meteor Detector

The Meteor Detector channel is intended for RF meteor-scatter trail monitoring. It channelizes the selected receiver offset, resamples the channel to 100, 300, 1000, or 3000 Hz, and displays the resampled IQ in a spectrum.

Detection combines two paths. A scalar power gate finds increases above an adaptive noise floor, while an overlapping-FFT tracker follows narrow spectral bands through time. Candidates are scored from signal strength, integrated support, bandwidth, frequency coherence, duration, and track occupancy.

Accepted spectral components are associated with a parent meteor event before reporting. A parent uses a lower continuation threshold inside its established frequency region, protects that region from noise-floor adaptation, and tolerates short fades. Compatible components and scalar evidence are consolidated into one detection. Duration is measured from the complete parent interval; center frequency, span, and drift use robust weighted observations from its spectral components. Parent state and retained observations are bounded so long unattended runs have fixed memory and processing costs.

The regression harness can write a candidate audit CSV containing the parent event ID and association decision for each spectral candidate. A diagnostic callback is also available to capture the channelized IQ samples belonging to a finalized parent event without adding file I/O to the detector's sample-processing path.

Smooth, sustained Doppler sweeps and tracks beyond the configured drift limit are rejected to suppress satellite and other moving-carrier interference. Consequently, fast meteor head echoes that sweep by kilohertz per second are outside the detector's intended scope and may be rejected. The detector is designed primarily as a trail-echo counter.

The maximum duration is a reporting and counting limit. A longer coherent echo that otherwise passes the meteor checks is reported once with its duration clipped to the configured maximum; the detection is marked as truncated in the GUI tooltip. The detector then rearms only after the signal falls below the release threshold, avoiding repeated counts of the same long trail.
