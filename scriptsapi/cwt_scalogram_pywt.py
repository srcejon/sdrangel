#!/usr/bin/env python3
"""
cwt_scalogram_pywt.py
---------------------
Python 3 example that reads a ``.wav`` audio file and produces a scalogram
using a *traditional* Continuous Wavelet Transform via PyWavelets
(``pywt.cwt``).

Designed to be a direct visual comparison against the SDRangel FFT-based
Morlet CWT scalogram produced by ``cwt_scalogram.py``.

Key differences vs the SDRangel approach
-----------------------------------------
  SDRangel (cwt_scalogram.py)   | PyWavelets (this script)
  ------------------------------+------------------------------------------
  Large-FFT + Gaussian weights  | Convolution with analytic Morlet wavelet
  Constant-Q (log-spaced bins)  | Scales set to give the same log-spaced freqs
  One row per fft_size samples  | One CWT output column per input sample
  ω₀ = 6.0 (hardcoded)         | w = 6.0 via 'cmor<B>-<C>' wavelet param

Dependencies (install via pip):
    numpy scipy matplotlib PyWavelets

Usage
-----
    python3 cwt_scalogram_pywt.py audio.wav
    python3 cwt_scalogram_pywt.py audio.wav --num-freqs 256 --fmin 20 --fmax 8000
    python3 cwt_scalogram_pywt.py audio.wav --wavelet cmor2.0-1.0 --db-range 60
    python3 cwt_scalogram_pywt.py audio.wav --downsample 4
"""

import argparse
import sys
from typing import Tuple

import numpy as np
import scipy.io.wavfile as wavfile
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import pywt


# ---------------------------------------------------------------------------
# Audio helpers
# ---------------------------------------------------------------------------

def load_wav_mono(path: str) -> Tuple[int, np.ndarray]:
    """Load a WAV file and return ``(sample_rate, float32 mono samples)``.

    Integer formats are normalised to the range [-1, 1].  Stereo signals are
    mixed to mono by averaging the two channels.

    Parameters
    ----------
    path : str
        Path to the input ``.wav`` file.

    Returns
    -------
    sample_rate : int
        Sample rate in Hz.
    samples : np.ndarray
        1-D float32 array of normalised audio samples.
    """
    sample_rate, data = wavfile.read(path)

    # Normalise integer PCM formats to [-1, 1].
    if data.dtype == np.int16:
        data = data.astype(np.float32) / 32768.0
    elif data.dtype == np.int32:
        data = data.astype(np.float32) / 2_147_483_648.0
    elif data.dtype == np.uint8:
        data = (data.astype(np.float32) - 128.0) / 128.0
    else:
        data = data.astype(np.float32)

    # Mix stereo to mono.
    if data.ndim == 2:
        data = data.mean(axis=1)

    return sample_rate, data


# ---------------------------------------------------------------------------
# Frequency / scale helpers
# ---------------------------------------------------------------------------

def log_freq_axis(fmin: float, fmax: float, n: int) -> np.ndarray:
    """Return *n* frequencies log-spaced between *fmin* and *fmax* (inclusive).

    Parameters
    ----------
    fmin : float
        Lowest frequency in Hz.
    fmax : float
        Highest frequency in Hz.
    n : int
        Number of frequency bins.

    Returns
    -------
    np.ndarray
        1-D float64 array of log-spaced frequencies.
    """
    return np.geomspace(fmin, fmax, num=n, dtype=np.float64)


def freqs_to_scales(freqs: np.ndarray, wavelet: str, sample_rate: int) -> np.ndarray:
    """Convert physical frequencies (Hz) to pywt scales for *wavelet*.

    ``pywt.scale2frequency(wavelet, scale)`` returns a normalised frequency
    (cycles/sample), so the physical frequency is::

        f_physical = f_normalised * sample_rate

    Rearranging::

        scale = central_frequency(wavelet) * sample_rate / f_physical

    Parameters
    ----------
    freqs : np.ndarray
        Array of physical frequencies in Hz.
    wavelet : str
        PyWavelets wavelet name (e.g. ``"cmor1.5-1.0"``).
    sample_rate : int
        Sample rate in Hz.

    Returns
    -------
    np.ndarray
        Array of pywt scales corresponding to *freqs*.
    """
    central_freq = pywt.central_frequency(wavelet)
    return central_freq * sample_rate / freqs


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """Parse arguments, compute the PyWavelets CWT, and display the scalogram."""
    parser = argparse.ArgumentParser(
        description=(
            "Traditional Morlet CWT scalogram via PyWavelets — "
            "use alongside cwt_scalogram.py for direct comparison with the "
            "SDRangel FFT-based implementation."
        )
    )
    parser.add_argument("wav_file", help="Input .wav file path")
    parser.add_argument(
        "--wavelet", default="cmor1.5-1.0",
        help=(
            "PyWavelets wavelet name.  Complex Morlet 'cmor<B>-<C>' gives the "
            "best comparison with SDRangel (B=bandwidth, C=centre freq).  "
            "Default: cmor1.5-1.0  (ω₀ ≈ 6.28 — comparable to SDRangel ω₀=6)."
        ),
    )
    parser.add_argument(
        "--num-freqs", type=int, default=256,
        help="Number of log-spaced frequency bins (default: 256)",
    )
    parser.add_argument(
        "--fmin", type=float, default=0.0,
        help="Lowest frequency in Hz (default: auto = 1 cycle/signal duration)",
    )
    parser.add_argument(
        "--fmax", type=float, default=0.0,
        help="Highest frequency in Hz (default: auto = Nyquist / 2)",
    )
    parser.add_argument(
        "--downsample", type=int, default=1,
        help=(
            "Downsample the signal by this integer factor before the CWT.  "
            "Reduces computation time roughly quadratically (default: 1 — no downsampling)."
        ),
    )
    parser.add_argument(
        "--db-range", type=float, default=80.0,
        help="Dynamic range in dB for the colour scale (default: 80)",
    )
    parser.add_argument(
        "--cmap", default="inferno",
        help="Matplotlib colour map (default: inferno)",
    )
    args = parser.parse_args()

    # ------------------------------------------------------------------
    # Load audio
    # ------------------------------------------------------------------
    sample_rate, samples = load_wav_mono(args.wav_file)
    print(f"Loaded : {args.wav_file}")
    print(f"  Sample rate : {sample_rate} Hz")
    print(f"  Samples     : {len(samples)}")
    print(f"  Duration    : {len(samples) / sample_rate:.2f} s")

    # Optional simple decimation — good enough for visual display.
    ds = max(1, int(args.downsample))
    if ds > 1:
        samples = samples[::ds]
        sample_rate = sample_rate // ds
        print(f"  After ×{ds} downsample : {len(samples)} samples @ {sample_rate} Hz")

    # ------------------------------------------------------------------
    # Frequency axis
    # ------------------------------------------------------------------
    nyquist = sample_rate / 2.0
    fmin = args.fmin if args.fmin > 0.0 else max(1.0, sample_rate / len(samples))
    fmax = args.fmax if args.fmax > 0.0 else nyquist / 2.0
    fmin = min(fmin, nyquist)
    fmax = min(fmax, nyquist)

    if fmin >= fmax:
        print(
            f"Error: --fmin ({fmin:.1f} Hz) must be less than --fmax ({fmax:.1f} Hz).",
            file=sys.stderr,
        )
        sys.exit(1)

    # Build ascending frequency array; derive pywt scales (descending, as pywt expects).
    freq_axis = log_freq_axis(fmin, fmax, args.num_freqs)   # ascending Hz
    scales = freqs_to_scales(freq_axis, args.wavelet, sample_rate)
    # pywt.cwt returns one row per scale; high scale = low frequency.
    # Pass scales in descending order so the first row is the lowest frequency.
    scales_desc = scales[::-1]      # high scale (low freq) first
    freq_desc = freq_axis[::-1]     # matching frequency order

    print(f"  Frequency range : {fmin:.1f} – {fmax:.1f} Hz  ({args.num_freqs} bins)")
    print(f"  Wavelet         : {args.wavelet}")
    print(f"  Scale range     : {scales_desc[-1]:.2f} – {scales_desc[0]:.2f}")

    # ------------------------------------------------------------------
    # CWT
    # ------------------------------------------------------------------
    print("Computing CWT (this may take a moment for long files)…")
    coeffs, _ = pywt.cwt(
        samples,
        scales_desc,
        args.wavelet,
        sampling_period=1.0 / sample_rate,
    )
    # coeffs shape: (num_freqs, num_samples)

    # Power in dB.
    eps = 1e-30
    power_db = 10.0 * np.log10(np.maximum(np.abs(coeffs) ** 2, eps))

    # Restore ascending-frequency order so low frequencies sit at the bottom.
    power_db = power_db[::-1, :]    # row 0 = fmin
    freq_plot = freq_desc[::-1]     # fmin … fmax

    print(f"  Scalogram shape : {power_db.shape}  (freq_bins × time_samples)")

    # ------------------------------------------------------------------
    # Time axis
    # ------------------------------------------------------------------
    time_axis = np.arange(power_db.shape[1]) / sample_rate  # seconds

    # ------------------------------------------------------------------
    # Plot
    # ------------------------------------------------------------------
    vmax = power_db.max()
    vmin = vmax - args.db_range

    fig, ax = plt.subplots(figsize=(14, 6))
    im = ax.pcolormesh(
        time_axis,
        freq_plot,
        power_db,
        shading="auto",
        cmap=args.cmap,
        norm=mcolors.Normalize(vmin=vmin, vmax=vmax),
    )
    ax.set_yscale("log")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Frequency (Hz)")
    ax.set_title(
        f"PyWavelets CWT scalogram — {args.wav_file}\n"
        f"wavelet={args.wavelet},  {args.num_freqs} bins,  "
        f"{fmin:.0f}–{fmax:.0f} Hz,  Fs={sample_rate} Hz"
    )
    fig.colorbar(im, ax=ax, label="Power (dB)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
