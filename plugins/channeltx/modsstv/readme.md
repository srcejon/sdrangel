# SSTV Modulator Plugin

## Overview

This plugin modulates PD120 SSTV (Slow Scan Television) images for transmission. It encodes a loaded image using the PD120 protocol and produces either an FM or SSB (USB/LSB) modulated signal.

## PD120 Protocol

PD120 is a colour SSTV mode that transmits:
- 640 × 496 pixel images (248 pairs of scan lines)
- Using YCbCr colour encoding
- Total transmission time: approximately 126 seconds per frame

Each scan line pair consists of:
1. **Sync pulse**: 1200 Hz for 20 ms
2. **Porch**: 1500 Hz for 2.08 ms
3. **Y_odd (luminance)**: 640 pixels at 190 µs/pixel
4. **Cr (red chroma)**: 640 pixels at 190 µs/pixel
5. **Cb (blue chroma)**: 640 pixels at 190 µs/pixel
6. **Y_even (luminance)**: 640 pixels at 190 µs/pixel

Pixel brightness values are mapped linearly to audio frequencies:
- 1500 Hz → black (0)
- 2300 Hz → white (255)

A standard VIS (Vertical Interval Signalling) preamble is transmitted before the image data.

## Interface

![SSTV Modulator GUI](screenshot.png)

- **Δf**: Channel frequency offset from the device centre frequency (Hz)
- **RF BW**: RF bandwidth (kHz)
- **Mode**: Modulation type — FM, USB (upper sideband) or LSB (lower sideband)
- **Dev**: FM peak deviation (kHz) — only shown when FM mode is selected
- **...**: Open a file dialog to load a PNG or JPEG image
- Image path label shows the currently loaded file name
- **Image preview**: Shows the loaded image scaled to fit
- **Start**: Start/stop the SSTV transmission
- **Power**: Channel transmit power (dB)

## Usage

1. Select the modulation type (FM for VHF/UHF, USB for HF)
2. Set appropriate RF bandwidth and FM deviation if using FM
3. Click **...** to load a PNG or JPEG image file
4. Click **Start** to begin transmission
5. The image is automatically scaled to 640 × 496 pixels

Transmission time is approximately 126 seconds for a full PD120 frame.
