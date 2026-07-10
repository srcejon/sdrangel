# Meteor Detector

The Meteor Detector channel is intended for RF meteor-scatter trail monitoring. It channelizes the selected receiver offset, resamples the channel to 100, 300, 1000, or 3000 Hz, and displays the resampled IQ in a spectrum.

Detection combines two paths. A scalar power gate finds increases above an adaptive noise floor, while an overlapping-FFT tracker follows narrow spectral bands through time. Candidates are scored from signal strength, integrated support, bandwidth, frequency coherence, duration, and track occupancy. Duplicate component reports are merged before a detection is emitted.

Smooth, sustained Doppler sweeps and tracks beyond the configured drift limit are rejected to suppress satellite and other moving-carrier interference. Consequently, fast meteor head echoes that sweep by kilohertz per second are outside the detector's intended scope and may be rejected. The detector is designed primarily as a trail-echo counter.

The maximum duration is a reporting and counting limit. A longer coherent echo that otherwise passes the meteor checks is reported once with its duration clipped to the configured maximum; the detection is marked as truncated in the GUI tooltip. The detector then rearms only after the signal falls below the release threshold, avoiding repeated counts of the same long trail.
