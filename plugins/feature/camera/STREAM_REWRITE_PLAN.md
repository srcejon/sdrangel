# Stream playback rewrite — clean separate-thread architecture

Status: IN PROGRESS. Scope = LIVE/URL streams only. File playback (`readNextFrame`,
non-URL) is left untouched and keeps working throughout.

## Why
The old design decoded audio+video **together** (`readNextFrame`) on one decode thread that
was paced by the **video present** (block-to-accumulate). So the audio feed rate was tied to
the present's achievable display rate → endless sync/pitch/buffer trade-offs. The fix is the
standard player shape: decode audio and video on **separate** paths, each paced by **its own
consumer**, with audio as the master clock.

## Target architecture (streams)

Decoder owns the FFmpeg objects and runs three threads:

1. **Demux thread** (rework of the read-ahead): `av_read_frame`, route each packet by stream
   index into TWO bounded packet queues — `m_videoPktQ`, `m_audioPktQ`. The compressed
   **cushion** (≈ streamBufferingSeconds) lives here. Demux blocks when both queues are full.
2. **Audio decode thread**: pop `m_audioPktQ` → decode (aac) → resample to device-rate S16
   stereo → append to a bounded **audio sample buffer** (samples tagged with content PTS).
   BLOCKS when that buffer is full. ⇒ paced by the audio OUTPUT pulling (device rate).
3. **Video decode thread**: pop `m_videoPktQ` → decode (NVDEC) → bounded **decoded frame
   queue** (frame + PTS). BLOCKS when full. ⇒ paced by the PRESENT pulling.

Decoder exposes (for streams):
- `streamTakeAudio(buf, maxFrames)` → device-rate samples; reports the content PTS consumed.
- `streamTakeVideoFrame(img, ptsMs)` / `streamPeekVideoFramePtsMs()`.
- `streamAudioPlayedPtsMs()` is NOT in the decoder; the controller computes the master clock.

Controller:
- **Audio output** (existing resampler→monitor FIFO→QAudioSink) pulls from the decoder audio
  buffer. Resampler stays = SLOW drift corrector only (source vs soundcard crystal). Holds the
  audio buffer near a target depth.
- **Master clock** = content PTS of audio at the speaker = (PTS of last sample consumed by the
  resampler) − (monitor FIFO + sink latency), in content ms.
- **Present**: timer; show the decoder's next frame when its PTS ≤ master clock; DROP late
  frames; HOLD (don't pop) when the next frame is still ahead. No self-pace, no
  block-to-accumulate, no wall-clock pacing.

## Behaviour
- Overrun (a queue full): demux blocks (cushion holds); audio/video decode block on their
  output queue. Resampler trims audio over-target.
- Underrun: audio buffer empty → monitor plays silence (readData), audio decode refills.
  Video queue empty → present holds last frame; if persistently empty → rebuffer.
- Source stall: demux stalls; both decode threads drain the packet cushion at the consumer
  rate (audio output keeps pulling → audio decode keeps draining → rides the stall). >Ns dead
  → reopen at live edge (watchdog).
- Latency: sink latency = EMA(fed−processedUSecs) (already in AudioOutputDevice). Audio buffer
  target sized so (buffer+FIFO+sink) ≤ decoded-frame-queue depth so the present can land on the
  matching frame. (This sizing is the thing that's been fiddly; with the clean clock it's
  exact: the present shows frame at PTS==clock, clock already subtracts FIFO+sink.)

## Build/test gate (each stage)
30fps sync · 60fps sync · source-stall recovery · file playback unaffected.

## Implementation stages
1. Decoder: split read-ahead → two packet queues (demux). [foundation]
2. Decoder: audio decode thread + audio sample buffer (PTS-tagged) + `streamTakeAudio`.
3. Decoder: video decode thread + decoded frame queue + `streamTakeVideoFrame`.
4. Controller: stream audio output pulls decoder buffer; compute master clock.
5. Controller: present shows decoder frames at PTS vs clock (drop late / hold early). Delete
   the old stream decode thread, queueDecodedVideoFileFrame, readQueuedVideoFileFrame
   self-pace, present wall/audio pacing saga, m_streamPresentActive, etc.
6. Strip diagnostics; validate matrix.

Fallback = git: `d0507cbb6` (decouple: 60fps good + stalls ride, but video-leads-audio).
