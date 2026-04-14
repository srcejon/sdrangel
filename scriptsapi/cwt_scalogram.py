#!/usr/bin/env python3
"""
cwt_scalogram.py
----------------
Python 3 faithful reimplementation of the SDRangel CWT scalogram as produced
by ``SpectrumVis::performCWT`` and ``SpectrumVis::buildCWTWeightTables`` in
``sdrbase/dsp/spectrumvis.cpp``.

Algorithm summary
-----------------
A sliding history buffer of ``N_total = fft_size * cwt_steps`` samples is
maintained.  Each time ``fft_size`` new samples arrive the full buffer is
windowed and transformed with one large-N FFT.  Pre-computed, log-spaced
Gaussian weight tables then map the N_total-bin power spectrum onto
``fft_size`` output bins, giving constant-Q (scale-dependent) frequency
resolution — the defining property of a Continuous Wavelet Transform.

Key constants (mirror the C++ ``constexpr`` values):
  * Morlet central frequency  ω₀ = 6.0
  * σ-cutoff                  = 4 / ω₀ ≈ 0.667
  * half-ω₀²                  = ω₀² / 2 = 18.0

Dependencies (install via pip):
    numpy scipy matplotlib

Usage
-----
    python3 cwt_scalogram.py audio.wav
    python3 cwt_scalogram.py audio.wav --fft-size 2048 --cwt-steps 16
    python3 cwt_scalogram.py audio.wav --cwt-steps 1024 --window blackman
    python3 cwt_scalogram.py audio.wav --db-range 60 --cmap viridis
"""

import argparse
import sys
from typing import Dict, List, Tuple

import numpy as np
import scipy.io.wavfile as wavfile
import scipy.signal as signal
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


# ---------------------------------------------------------------------------
# C++ mirror constants (spectrumvis.cpp)
# ---------------------------------------------------------------------------
_MORLET_OMEGA0: float = 6.0
_SIGMA_CUTOFF: float = 4.0 / _MORLET_OMEGA0      # ≈ 0.6667
_HALF_OMEGA0_SQ: float = _MORLET_OMEGA0 ** 2 / 2  # = 18.0


# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------
WeightEntry = Tuple[int, float]          # (fft_bin_index, normalised_weight)
WeightTable = List[List[WeightEntry]]    # one list of (bin, weight) per output slot


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def is_power_of_two(n: int) -> bool:
    """Return True when *n* is a strictly positive power of two.

    Parameters
    ----------
    n : int
        Value to test.

    Returns
    -------
    bool
        ``True`` iff ``n >= 1`` and ``n`` is an exact power of two.
    """
    return n >= 1 and (n & (n - 1)) == 0


def log_center_bin(i: int, n: int, lo: int, hi: int) -> int:
    """Map output-bin index *i* (0-based, out of *n*) to a log-spaced integer
    bin in [*lo*, *hi*] (inclusive).

    This is a direct Python translation of the ``logCenterBin`` lambda in
    ``SpectrumVis::buildCWTWeightTables``.

    Parameters
    ----------
    i : int
        Zero-based output-slot index.
    n : int
        Total number of output slots.
    lo : int
        Smallest allowed bin (mapped to ``i=0``).
    hi : int
        Largest allowed bin (mapped to ``i=n-1``).

    Returns
    -------
    int
        Clamped log-spaced centre bin for slot *i*.
    """
    if n <= 1:
        return lo
    t = i / (n - 1)
    val = lo * (hi / lo) ** t
    return max(lo, min(hi, int(round(val))))


def build_cwt_weight_tables(
    fft_size: int,
    cwt_steps: int,
) -> Tuple[WeightTable, WeightTable]:
    """Build the log-spaced Gaussian weight tables used by the SDRangel CWT.

    Mirrors ``SpectrumVis::buildCWTWeightTables`` exactly.

    Parameters
    ----------
    fft_size : int
        Base FFT / display size (number of output frequency bins).
    cwt_steps : int
        History multiplier; the large FFT has ``N_total = fft_size * cwt_steps``
        points.  Must be a power of two in the range [1, *fft_size*].

    Returns
    -------
    weights_full : WeightTable
        Two-sided weight table with shape ``[fft_size]``.
    weights_pos : WeightTable
        Positive-only weight table with shape ``[fft_size // 2]``.

    Raises
    ------
    ValueError
        When *cwt_steps* is not a power of two or exceeds *fft_size*.
    """
    if not is_power_of_two(cwt_steps):
        raise ValueError(f"cwt_steps must be a power of two, got {cwt_steps}")
    if cwt_steps > fft_size:
        raise ValueError(
            f"cwt_steps ({cwt_steps}) must not exceed fft_size ({fft_size})"
        )

    half_size: int = fft_size // 2
    cwt_fft_size: int = fft_size * cwt_steps
    cwt_half: int = cwt_fft_size // 2

    # ------------------------------------------------------------------
    # Full (two-sided) weight table
    # ------------------------------------------------------------------
    # Output layout (centred spectrum):
    #   kOut = 0 .. half_size-1      → negative frequencies (most-negative first)
    #   kOut = half_size             → DC (no weights, left at zero)
    #   kOut = half_size+1 .. fft_size-1 → positive frequencies
    weights_full: WeightTable = [[] for _ in range(fft_size)]

    for k_out in range(fft_size):
        if k_out == half_size:
            continue  # DC bin — leave empty so power stays 0

        if k_out > half_size:
            # Positive side: half_size-1 slots log-spaced in [cwt_steps, cwt_half - cwt_steps]
            i_pos_slot = k_out - half_size - 1
            k = log_center_bin(i_pos_slot, half_size - 1, cwt_steps, cwt_half - cwt_steps)
        else:
            # Negative side: half_size slots log-spaced |k| in [cwt_steps, cwt_half]
            i_neg_slot = half_size - 1 - k_out
            k = -log_center_bin(i_neg_slot, half_size, cwt_steps, cwt_half)

        k_abs = abs(k)
        range_bins = int(_SIGMA_CUTOFF * k_abs) + 2

        if k > 0:
            j_min = max(1, k - range_bins)
            j_max = min(cwt_half - 1, k + range_bins)
        else:
            j_min = max(-cwt_half, k - range_bins)
            j_max = min(-1, k + range_bins)

        # Accumulate raw Gaussian weights then normalise in a single pass.
        raw_weights: List[float] = []
        norm = 0.0
        for j in range(j_min, j_max + 1):
            ratio = j / k
            diff = ratio - 1.0
            w = float(np.exp(-_HALF_OMEGA0_SQ * diff * diff))
            raw_weights.append(w)
            norm += w

        if norm > 0.0:
            for idx, w in enumerate(raw_weights):
                j_nat = (j_min + idx + cwt_fft_size) % cwt_fft_size
                weights_full[k_out].append((j_nat, w / norm))

    # ------------------------------------------------------------------
    # Positive-only weight table
    # ------------------------------------------------------------------
    # half_size output slots, log-spaced k in [cwt_steps, cwt_half].
    weights_pos: WeightTable = [[] for _ in range(half_size)]

    for i in range(half_size):
        k = log_center_bin(i, half_size, cwt_steps, cwt_half)
        range_bins = int(_SIGMA_CUTOFF * k) + 2
        j_min = max(1, k - range_bins)
        j_max = min(cwt_half, k + range_bins)

        raw_weights = []
        norm = 0.0
        for j in range(j_min, j_max + 1):
            ratio = j / k
            diff = ratio - 1.0
            w = float(np.exp(-_HALF_OMEGA0_SQ * diff * diff))
            raw_weights.append(w)
            norm += w

        if norm > 0.0:
            for idx, w in enumerate(raw_weights):
                weights_pos[i].append((j_min + idx, w / norm))

    return weights_full, weights_pos


def make_window(name: str, size: int) -> np.ndarray:
    """Return a normalised analysis window of the requested type and length.

    Parameters
    ----------
    name : str
        Window name accepted by ``scipy.signal.get_window`` (e.g. ``"hann"``,
        ``"hamming"``, ``"blackman"``, ``"flattop"``).
    size : int
        Number of samples.

    Returns
    -------
    np.ndarray
        Float64 window array of length *size*, peak-normalised to 1.
    """
    win = signal.get_window(name, size, fftbins=True).astype(np.float64)
    peak = win.max()
    if peak > 0.0:
        win /= peak
    return win


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


def compute_cwt_scalogram(
    samples: np.ndarray,
    sample_rate: int,
    fft_size: int,
    cwt_steps: int,
    window: np.ndarray,
) -> np.ndarray:
    """Produce a CWT scalogram matrix from audio samples.

    Implements the SDRangel sliding-buffer CWT: every ``fft_size`` new samples
    trigger one large ``N_total``-point FFT over the full history buffer, whose
    power is mapped onto ``fft_size`` log-spaced output bins via pre-computed
    Gaussian weight tables.

    Parameters
    ----------
    samples : np.ndarray
        1-D array of normalised audio samples.
    sample_rate : int
        Sample rate in Hz (used only for the returned frequency axis).
    fft_size : int
        Base FFT / display size.  Must be a power of two.
    cwt_steps : int
        History multiplier; ``N_total = fft_size * cwt_steps``.  Must be a
        power of two in [1, *fft_size*].
    window : np.ndarray
        Analysis window of length ``N_total = fft_size * cwt_steps``.

    Returns
    -------
    scalogram : np.ndarray
        2-D float32 array with shape ``(n_rows, fft_size)``.  Each row is one
        spectrum snapshot in linear power (not dB).
    """
    cwt_fft_size = fft_size * cwt_steps
    half_size = fft_size // 2

    print(f"Building CWT weight tables (fft_size={fft_size}, cwt_steps={cwt_steps})…")
    weights_full, weights_pos = build_cwt_weight_tables(fft_size, cwt_steps)

    # Convert weight tables to NumPy arrays for vectorised power accumulation.
    # For each output bin we need an array of FFT-bin indices and their weights.
    bin_indices: List[np.ndarray] = []
    bin_weights: List[np.ndarray] = []
    for entries in weights_full:
        if entries:
            idx_arr = np.array([e[0] for e in entries], dtype=np.int32)
            wgt_arr = np.array([e[1] for e in entries], dtype=np.float32)
        else:
            idx_arr = np.empty(0, dtype=np.int32)
            wgt_arr = np.empty(0, dtype=np.float32)
        bin_indices.append(idx_arr)
        bin_weights.append(wgt_arr)

    # Pre-allocate the sliding history buffer (zero-padded initially).
    buf = np.zeros(cwt_fft_size, dtype=np.float64)

    rows: List[np.ndarray] = []
    n_samples = len(samples)
    pos = 0  # next unread sample index

    print(f"Processing {n_samples} samples in {n_samples // fft_size} frames…")

    while pos + fft_size <= n_samples:
        # Slide the buffer left by fft_size and insert new samples at the tail.
        buf[:cwt_fft_size - fft_size] = buf[fft_size:]
        buf[cwt_fft_size - fft_size:] = samples[pos: pos + fft_size]
        pos += fft_size

        # Window the full history buffer and compute the large FFT.
        fft_out = np.fft.fft(buf * window)
        power = (fft_out.real ** 2 + fft_out.imag ** 2).astype(np.float32)

        # Accumulate power using the pre-computed log-spaced Gaussian weights.
        row = np.zeros(fft_size, dtype=np.float32)
        for k_out in range(fft_size):
            idx = bin_indices[k_out]
            if idx.size > 0:
                row[k_out] = float(np.dot(power[idx], bin_weights[k_out]))

        rows.append(row)

    return np.array(rows, dtype=np.float32)


def frequency_axis(fft_size: int, cwt_steps: int, sample_rate: int) -> np.ndarray:
    """Return the physical frequency (Hz) for each of the ``fft_size`` output bins.

    The positive side covers ``cwt_steps`` to ``fft_size * cwt_steps / 2``
    bins in the large FFT, which correspond to::

        f_k = k * (sample_rate / (fft_size * cwt_steps))

    Parameters
    ----------
    fft_size : int
        Base FFT / display size.
    cwt_steps : int
        History multiplier.
    sample_rate : int
        Sample rate in Hz.

    Returns
    -------
    np.ndarray
        1-D float64 array of length *fft_size* with the physical centre
        frequency (Hz) of each output bin (positive side only for display).
    """
    cwt_fft_size = fft_size * cwt_steps
    half_size = fft_size // 2
    cwt_half = cwt_fft_size // 2
    bin_hz = sample_rate / cwt_fft_size

    freqs = np.zeros(fft_size, dtype=np.float64)
    for i in range(half_size):
        k = log_center_bin(i, half_size, cwt_steps, cwt_half)
        freqs[i] = k * bin_hz
    return freqs


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """Parse arguments, run the CWT, and display the scalogram."""
    parser = argparse.ArgumentParser(
        description=(
            "SDRangel-faithful CWT scalogram: replicates SpectrumVis::performCWT "
            "using a sliding large-N FFT with log-spaced Gaussian weight tables."
        )
    )
    parser.add_argument("wav_file", help="Input .wav file path")
    parser.add_argument(
        "--fft-size", type=int, default=1024,
        help="Base FFT / display size (power of two, default: 1024)",
    )
    parser.add_argument(
        "--cwt-steps", type=int, default=4,
        help=(
            "History multiplier: N_total = fft_size * cwt_steps.  "
            "Must be a power of two in the range [1, fft_size].  "
            "Larger values increase low-frequency resolution at the cost of "
            "more memory and computation (default: 4)."
        ),
    )
    parser.add_argument(
        "--window", default="hann",
        help="Analysis window function (scipy.signal.get_window name, default: hann)",
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

    fft_size: int = args.fft_size
    cwt_steps: int = args.cwt_steps

    # ------------------------------------------------------------------
    # Validate power-of-two constraints
    # ------------------------------------------------------------------
    if not is_power_of_two(fft_size):
        print(f"Error: --fft-size must be a power of two, got {fft_size}.", file=sys.stderr)
        sys.exit(1)

    if not is_power_of_two(cwt_steps):
        print(f"Error: --cwt-steps must be a power of two, got {cwt_steps}.", file=sys.stderr)
        sys.exit(1)

    if cwt_steps > fft_size:
        print(
            f"Error: --cwt-steps ({cwt_steps}) must not exceed --fft-size ({fft_size}).",
            file=sys.stderr,
        )
        sys.exit(1)

    cwt_fft_size = fft_size * cwt_steps

    # ------------------------------------------------------------------
    # Load audio
    # ------------------------------------------------------------------
    sample_rate, samples = load_wav_mono(args.wav_file)
    print(f"Loaded : {args.wav_file}")
    print(f"  Sample rate  : {sample_rate} Hz")
    print(f"  Samples      : {len(samples)}")
    print(f"  Duration     : {len(samples) / sample_rate:.2f} s")
    print(f"  fft_size     : {fft_size}")
    print(f"  cwt_steps    : {cwt_steps}  (N_total = {cwt_fft_size})")

    if len(samples) < cwt_fft_size:
        print(
            f"Error: signal is shorter ({len(samples)} samples) than the CWT "
            f"buffer ({cwt_fft_size} samples).  Use a smaller --fft-size or "
            f"--cwt-steps, or provide a longer file.",
            file=sys.stderr,
        )
        sys.exit(1)

    # ------------------------------------------------------------------
    # Build analysis window (length = N_total)
    # ------------------------------------------------------------------
    try:
        win = make_window(args.window, cwt_fft_size)
    except Exception as exc:
        print(f"Error creating window '{args.window}': {exc}", file=sys.stderr)
        sys.exit(1)

    # ------------------------------------------------------------------
    # Compute CWT scalogram
    # ------------------------------------------------------------------
    scalogram = compute_cwt_scalogram(samples, sample_rate, fft_size, cwt_steps, win)
    print(f"Scalogram shape : {scalogram.shape}  (frames × frequency bins)")

    # ------------------------------------------------------------------
    # Convert to dB
    # ------------------------------------------------------------------
    eps = 1e-30
    scalogram_db = 10.0 * np.log10(np.maximum(scalogram, eps))

    # ------------------------------------------------------------------
    # Build axes
    # ------------------------------------------------------------------
    # Time axis: one row per fft_size samples, centre-referenced.
    n_rows = scalogram.shape[0]
    time_axis = (np.arange(n_rows) * fft_size + fft_size / 2) / sample_rate  # seconds

    # Frequency axis: positive side only (half_size bins).
    half_size = fft_size // 2
    freqs = frequency_axis(fft_size, cwt_steps, sample_rate)[:half_size]

    # Extract positive-side columns from the full scalogram.
    # In the centred layout, positive bins occupy the upper half of the array.
    # Bin index half_size+1 .. fft_size-1 correspond to log-spaced positive freqs.
    # For display, take the half_size bins that form the positive spectrum.
    # (The positive-only weight table was built for slots 0..half_size-1.)
    # Re-compute using the positive-only table for clarity, or just take
    # columns from the right half of the full scalogram.
    pos_scalogram_db = scalogram_db[:, half_size + 1:]  # shape (n_rows, half_size - 1)
    pos_freqs = freqs[1:]  # skip DC (matches half_size-1 positive slots)

    # If there are no valid frequency values (edge case), fall back.
    if pos_freqs.size == 0 or np.all(pos_freqs == 0):
        pos_scalogram_db = scalogram_db
        pos_freqs = np.linspace(0, sample_rate / 2, fft_size)

    # ------------------------------------------------------------------
    # Plot
    # ------------------------------------------------------------------
    vmax = pos_scalogram_db.max()
    vmin = vmax - args.db_range

    fig, ax = plt.subplots(figsize=(14, 6))
    im = ax.pcolormesh(
        time_axis,
        pos_freqs,
        pos_scalogram_db.T,
        shading="auto",
        cmap=args.cmap,
        norm=mcolors.Normalize(vmin=vmin, vmax=vmax),
    )
    ax.set_yscale("log")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Frequency (Hz)")
    ax.set_title(
        f"SDRangel CWT scalogram — {args.wav_file}\n"
        f"fft_size={fft_size},  cwt_steps={cwt_steps},  "
        f"N_total={cwt_fft_size},  window={args.window},  Fs={sample_rate} Hz"
    )
    fig.colorbar(im, ax=ax, label="Power (dB)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
