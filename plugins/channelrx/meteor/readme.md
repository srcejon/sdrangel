# Meteor Detector

The Meteor Detector channel is intended for RF meteor scatter monitoring. It channelizes the selected receiver offset, resamples the channel to 100, 300, 1000, or 3000 Hz, and displays the resampled IQ in the spectrum and scope.

The scope streams are:

* IQ
* Instantaneous power in dB
* Low-pass filtered power in dB
* Meteor detection gate
* Tracked noise floor in dB

Pulse detections are based on filtered power rising above a tracked noise floor. Candidate pulses are rejected when their measured frequency span or frequency drift exceeds the configured limit, which helps discard longer Doppler-swept satellite passes.
