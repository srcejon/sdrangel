#!/usr/bin/env python3
"""Count Siril SPCC Gaia DR3 catalog stars in a small FoV.

This is a throwaway validation utility for the camera plate-solver catalog
discussion. It reads Siril's HEALPix level-8 xpsamp catalog over HTTP using
Range requests, fetches only the cells around the requested field, and filters
the returned records by the requested rectangular FoV.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from collections import defaultdict
from dataclasses import dataclass

import requests
from astropy import units as u
from astropy.coordinates import ICRS, SkyCoord
from astropy_healpix import HEALPix


BASE_URL = "https://huggingface.co/datasets/siril-spcc/gaia/resolve/main"
FILENAME_TEMPLATE = "siril_cat1_healpix8_xpsamp_{chunk}.dat"
HEALPIX_LEVEL = 8
NSIDE = 1 << HEALPIX_LEVEL
CHUNK_LEVEL = 1
PIXELS_PER_CHUNK = 4 ** (HEALPIX_LEVEL - CHUNK_LEVEL)
HEADER_SIZE = 128
INDEX_ENTRY_SIZE = 4
RECORD_SIZE = 701
ANGLE_SCALE = 360.0 / ((1 << 31) - 1)


@dataclass(frozen=True)
class CatalogRecord:
    ra_deg: float
    dec_deg: float
    g_mag: float


def parse_hms(text: str) -> float:
    parts = text.replace(":", " ").split()
    if len(parts) == 1:
        return float(parts[0])
    if len(parts) != 3:
        raise ValueError(f"Expected RA as decimal degrees or 'H M S', got {text!r}")
    hours, minutes, seconds = map(float, parts)
    return 15.0 * (hours + minutes / 60.0 + seconds / 3600.0)


def parse_dms(text: str) -> float:
    parts = text.replace(":", " ").replace("+", " ").split()
    if len(parts) == 1:
        return float(parts[0])
    if len(parts) != 3:
        raise ValueError(f"Expected Dec as decimal degrees or 'D M S', got {text!r}")
    sign = -1.0 if parts[0].startswith("-") else 1.0
    deg = abs(float(parts[0]))
    minutes = float(parts[1])
    seconds = float(parts[2])
    return sign * (deg + minutes / 60.0 + seconds / 3600.0)


def range_get(session: requests.Session, url: str, start: int, end_inclusive: int) -> bytes:
    response = session.get(
        url,
        headers={"Range": f"bytes={start}-{end_inclusive}"},
        timeout=60,
    )
    response.raise_for_status()
    if response.status_code != 206:
        raise RuntimeError(f"Server did not return partial content for {url}: {response.status_code}")
    return response.content


def chunk_url(chunk: int) -> str:
    return f"{BASE_URL}/{FILENAME_TEMPLATE.format(chunk=chunk)}"


def fetch_chunk_header(session: requests.Session, chunk: int) -> tuple[int, int]:
    data = range_get(session, chunk_url(chunk), 0, HEADER_SIZE - 1)
    title = data[:48].rstrip(b"\0").decode("ascii", errors="replace")
    gaia_release, healpix_level, catalog_type, chunked, chunk_level = struct.unpack_from("<BBBBB", data, 48)
    chunk_number = struct.unpack_from("<I", data, 53)[0]
    first_pixel, last_pixel = struct.unpack_from("<II", data, 57)
    if healpix_level != HEALPIX_LEVEL or catalog_type != 2 or not chunked or chunk_level != CHUNK_LEVEL:
        raise RuntimeError(f"Unexpected Siril catalog header in chunk {chunk}: {title!r}")
    if chunk_number != chunk:
        raise RuntimeError(f"Chunk file {chunk} reports chunk number {chunk_number}")
    if gaia_release != 3:
        raise RuntimeError(f"Chunk {chunk} is not Gaia DR3")
    return first_pixel, last_pixel


def fetch_cell_counts(session: requests.Session, chunk: int, local_pixels: list[int]) -> dict[int, tuple[int, int]]:
    """Return local_pixel -> (start_record_index, record_count)."""
    result: dict[int, tuple[int, int]] = {}
    if not local_pixels:
        return result

    wanted = sorted(set(local_pixels))
    needed_index_entries = sorted(set(wanted + [p - 1 for p in wanted if p > 0]))

    entries: dict[int, int] = {}
    url = chunk_url(chunk)
    for entry in needed_index_entries:
        start = HEADER_SIZE + entry * INDEX_ENTRY_SIZE
        data = range_get(session, url, start, start + INDEX_ENTRY_SIZE - 1)
        entries[entry] = struct.unpack("<I", data)[0]

    for pixel in wanted:
        previous = entries.get(pixel - 1, 0)
        current = entries[pixel]
        result[pixel] = (previous, current - previous)
    return result


def fetch_records(session: requests.Session, chunk: int, ranges: list[tuple[int, int]]) -> list[CatalogRecord]:
    records: list[CatalogRecord] = []
    if not ranges:
        return records

    data_start = HEADER_SIZE + PIXELS_PER_CHUNK * INDEX_ENTRY_SIZE
    url = chunk_url(chunk)

    for start_index, count in ranges:
        if count <= 0:
            continue
        start = data_start + start_index * RECORD_SIZE
        end = start + count * RECORD_SIZE - 1
        data = range_get(session, url, start, end)
        for offset in range(0, len(data), RECORD_SIZE):
            ra_scaled, dec_scaled, _dra, _ddec, g_mag_scaled, _fexpo = struct.unpack_from("<iihhhb", data, offset)
            records.append(CatalogRecord(
                ra_scaled * ANGLE_SCALE,
                dec_scaled * ANGLE_SCALE,
                g_mag_scaled / 1000.0,
            ))
    return records


def angular_delta_ra_deg(ra_deg: float, center_ra_deg: float) -> float:
    return ((ra_deg - center_ra_deg + 180.0) % 360.0) - 180.0


def in_rect(record: CatalogRecord, center_ra_deg: float, center_dec_deg: float, width_deg: float, height_deg: float) -> bool:
    dra = angular_delta_ra_deg(record.ra_deg, center_ra_deg) * math.cos(math.radians(center_dec_deg))
    ddec = record.dec_deg - center_dec_deg
    return abs(dra) <= width_deg * 0.5 and abs(ddec) <= height_deg * 0.5


def count_field(ra_deg: float, dec_deg: float, width_deg: float, height_deg: float, max_mag: float) -> None:
    session = requests.Session()
    hp = HEALPix(nside=NSIDE, order="nested", frame=ICRS())
    center = SkyCoord(ra=ra_deg * u.deg, dec=dec_deg * u.deg, frame="icrs")

    # Use a circular superset for HEALPix selection, then exact rectangular
    # filtering on decoded records. Add a small cell-size margin.
    half_diagonal = 0.5 * math.hypot(width_deg, height_deg)
    query_radius = half_diagonal + 0.25
    pixels = [int(p) for p in hp.cone_search_skycoord(center, query_radius * u.deg)]

    pixels_by_chunk: dict[int, list[int]] = defaultdict(list)
    for pixel in pixels:
        chunk = pixel // PIXELS_PER_CHUNK
        pixels_by_chunk[chunk].append(pixel % PIXELS_PER_CHUNK)

    total_candidates = 0
    total_in_rect = 0
    total_in_rect_mag = 0
    mags: list[float] = []

    for chunk, local_pixels in sorted(pixels_by_chunk.items()):
        first_pixel, _last_pixel = fetch_chunk_header(session, chunk)
        if first_pixel != chunk * PIXELS_PER_CHUNK:
            raise RuntimeError(f"Unexpected first pixel in chunk {chunk}: {first_pixel}")
        counts = fetch_cell_counts(session, chunk, local_pixels)
        ranges = list(counts.values())
        total_candidates += sum(count for _start, count in ranges)
        records = fetch_records(session, chunk, ranges)
        for record in records:
            if in_rect(record, ra_deg, dec_deg, width_deg, height_deg):
                total_in_rect += 1
                if record.g_mag <= max_mag:
                    total_in_rect_mag += 1
                    mags.append(record.g_mag)

    print(f"Center RA={ra_deg:.8f} deg Dec={dec_deg:.8f} deg")
    print(f"FoV {width_deg:.4f} x {height_deg:.4f} deg, max G mag {max_mag:.2f}")
    print(f"HEALPix cells touched: {len(pixels)} across {len(pixels_by_chunk)} chunk file(s)")
    print(f"Catalog records fetched from touched cells: {total_candidates}")
    print(f"Stars inside rectangular FoV, all magnitudes in SPCC catalog: {total_in_rect}")
    print(f"Stars inside rectangular FoV with G <= {max_mag:.2f}: {total_in_rect_mag}")
    if mags:
        mags.sort()
        print(f"Brightest/faintest counted G mag: {mags[0]:.3f} / {mags[-1]:.3f}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ra", default="15 24 55.77840", help="RA as decimal degrees or 'H M S'")
    parser.add_argument("--dec", default="+58 57 57.8340", help="Dec as decimal degrees or 'D M S'")
    parser.add_argument("--width", type=float, default=1.29, help="Field width in degrees")
    parser.add_argument("--height", type=float, default=0.73, help="Field height in degrees")
    parser.add_argument("--max-mag", type=float, default=11.0, help="Maximum Gaia G magnitude to count")
    args = parser.parse_args()

    count_field(parse_hms(args.ra), parse_dms(args.dec), args.width, args.height, args.max_mag)
    return 0


if __name__ == "__main__":
    sys.exit(main())
