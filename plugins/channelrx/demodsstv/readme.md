<h1>SSTV Demodulator Plugin</h1>

<h2>Introduction</h2>

This plugin provides demodulation for **PD120 SSTV (Slow Scan Television)** signals. SSTV is a picture transmission method used by amateur radio operators and space stations to transmit and receive static images.

PD120 is a colour SSTV mode that transmits a 640×496 pixel image over approximately 120 seconds using FM-modulated audio tones in the range 1200–2300 Hz.

<h2>Interface</h2>

![SSTV Demodulator plugin GUI](../../../doc/img/SSTVDemod_plugin.png)

<h3>1. Frequency shift from centre frequency of reception</h3>

Use the dial to adjust the centre frequency of the SSTV channel within the device passband.

<h3>2. Channel power</h3>

Average total power in the channel passband in dB.

<h3>3. Level meter</h3>

Average and peak power level of the channel.

<h3>4. RF bandwidth (BW)</h3>

The bandwidth of the pre-demodulation bandpass filter in kHz.

<h3>5. FM deviation (Dev)</h3>

The FM deviation in kHz used to scale the phase discriminator output to audio frequencies. Adjust this if the decoded image appears washed out or too dark.

<h3>6. Start/Stop</h3>

Toggle image decoding on or off. When stopped, incoming samples are still processed for level display but no image data is accumulated.

<h3>7. Reset decoder</h3>

Clears the current image and resets the decoder state machine so that the next sync pulse starts a fresh image.

<h3>8. Save image</h3>

Save the currently displayed image to disk in PNG, JPEG, or BMP format.

<h3>9. Zoom controls</h3>

Zoom in, zoom out, or fit the image to the available window space.

<h3>10. Image display</h3>

The received PD120 image is drawn progressively as each pair of scan lines is decoded. The image updates in near real-time as the transmission progresses.

<h2>PD120 signal format</h2>

Each "block" of two scan lines has the following structure:

| Section  | Duration | Frequency            |
| -------- | -------- | -------------------- |
| Sync     | 20 ms    | 1200 Hz              |
| Porch    | 2.08 ms  | 1500 Hz              |
| Y (odd)  | 640 px   | 1500–2300 Hz per px  |
| Cr       | 320 px   | 1500–2300 Hz per px  |
| Cb       | 320 px   | 1500–2300 Hz per px  |
| Y (even) | 640 px   | 1500–2300 Hz per px  |

Pixel frequency mapping:
- 1500 Hz → black (0%)
- 2300 Hz → white (100%)

Colour conversion uses the YCbCr model shared by both lines in each block, converted to RGB for display.

<h2>Operation tips</h2>

- Tune to the SSTV carrier frequency. For HF USB SSTV the carrier is typically offset by 1500–2300 Hz from the dial frequency.
- Set FM deviation to approximately 3 kHz (the default). Increase if the image is dark; decrease if it is washed out.
- The decoder automatically synchronises to sync pulses; you do not need to start reception at the beginning of a transmission.
- For best results use an RF bandwidth of 4–8 kHz to cover the full SSTV audio range.
