"""Decode Compact type 6/version 1 using the SICK SDK 0.5.0 layout."""

from dataclasses import dataclass
import logging
import math
from pathlib import Path
import struct
from typing import Iterator
import zlib


logger = logging.getLogger(__name__)
HEADER = struct.Struct("<IIQQIII")
META = struct.Struct("<QQHHHHHBHBBBfB")
POINT = struct.Struct("<ffff")


@dataclass(frozen=True)
class Segment:
    sequence: int
    serial: int
    frame: int
    timestamp_us: int
    index: int
    segments_per_frame: int
    columns: int
    rows: int
    echoes: int
    interlace_steps: int
    interlace_index: int
    points: bytes
    zero_distances: int


def decode_type6(telegram: bytes) -> Segment:
    if len(telegram) < HEADER.size + META.size + 91 + 4:
        raise ValueError("Type 6 telegram is too short.")
    start, kind, sequence, _, version, length, serial = HEADER.unpack_from(telegram)
    if (start, kind, version) != (0x02020202, 6, 1):
        raise ValueError("Only Compact type 6/version 1 is supported.")
    if len(telegram) != HEADER.size + length + 4:
        raise ValueError("Telegram length does not match the header.")
    checksum = struct.unpack_from("<I", telegram, len(telegram) - 4)[0]
    if zlib.crc32(telegram[:-4]) != checksum:
        raise ValueError(f"CRC32 mismatch in telegram {sequence}.")

    (
        frame, timestamp, index, segments, columns, frame_columns, rows, echoes,
        ambient_rows, interlace_steps, interlace_index, _, scale, flags,
    ) = META.unpack_from(telegram, HEADER.size)
    if not segments or index >= segments or columns > frame_columns:
        raise ValueError("Invalid segment dimensions or index.")
    if flags & ~7 or not math.isfinite(scale) or scale <= 0:
        raise ValueError("Unsupported echo flags or invalid distance scale.")
    if interlace_steps and interlace_index >= interlace_steps:
        raise ValueError("Invalid interlace index.")
    samples = columns * rows * echoes
    intensity_size = (samples * 12 + 7) // 8 if flags & 1 else 0
    geometry_size = rows * 4 + columns * 10
    expected = (
        META.size + 91 + columns * ambient_rows * 2 + geometry_size
        + samples * 2 + intensity_size
        + samples * (bool(flags & 2) + bool(flags & 4))
    )
    if length != expected:
        raise ValueError(f"Payload layout mismatch in telegram {sequence}.")

    geometry_start = HEADER.size + META.size + 91 + columns * ambient_rows * 2
    elevations = struct.unpack_from(f"<{rows}f", telegram, geometry_start)
    azimuths = struct.unpack_from(
        f"<{columns}f", telegram, geometry_start + rows * 4,
    )
    if not all(math.isfinite(angle) for angle in (*elevations, *azimuths)):
        raise ValueError(f"Non-finite geometry in telegram {sequence}.")
    distance_start = geometry_start + geometry_size
    distances = struct.unpack_from(f"<{samples}H", telegram, distance_start)
    intensity_start = distance_start + samples * 2
    row_trig = [(math.cos(angle), math.sin(angle)) for angle in elevations]
    column_trig = [(math.cos(angle), math.sin(angle)) for angle in azimuths]
    points = bytearray()
    zeros = 0
    # Match the vendor converter's row/column/echo ordering and negative Z sign.
    for row, (cos_elevation, sin_elevation) in enumerate(row_trig):
        for column, (cos_azimuth, sin_azimuth) in enumerate(column_trig):
            for echo in range(echoes):
                sample = echo * columns * rows + column * rows + row
                raw_distance = distances[sample]
                if raw_distance == 0:
                    zeros += 1
                    continue
                distance = raw_distance * scale / 1000.0
                if distance > 3.4028234663852886e38:
                    raise ValueError("Scaled distance exceeds the PCD float32 range.")
                intensity = math.nan
                if flags & 1:
                    position = intensity_start + sample * 12 // 8
                    pair = telegram[position] | (telegram[position + 1] << 8)
                    intensity = ((pair >> (4 if sample % 2 else 0)) & 4095) / 4095
                points.extend(POINT.pack(
                    distance * cos_elevation * cos_azimuth,
                    distance * cos_elevation * sin_azimuth,
                    -distance * sin_elevation,
                    intensity,
                ))
    return Segment(
        sequence, serial, frame, timestamp, index, segments, columns, rows,
        echoes, interlace_steps, interlace_index, bytes(points), zeros,
    )


def read_segments(
    source: Path, allow_truncated_tail: bool = False,
) -> Iterator[Segment]:
    with source.open("rb") as stream:
        total = source.stat().st_size
        while stream.tell() < total:
            offset = stream.tell()
            header = stream.read(HEADER.size)
            if len(header) < HEADER.size:
                if allow_truncated_tail:
                    logger.warning("Skipping truncated header at byte %s", offset)
                    return
                raise ValueError(f"Truncated header at byte {offset}.")
            start, kind, _, _, version, length, _ = HEADER.unpack(header)
            if (start, kind, version) != (0x02020202, 6, 1):
                raise ValueError(f"Unsupported Compact header at byte {offset}.")
            if length < META.size + 91:
                raise ValueError(f"Invalid payload length at byte {offset}.")
            if length + 4 > total - stream.tell():
                if allow_truncated_tail:
                    logger.warning(
                        "Skipping truncated telegram at byte %s (%s bytes remain)",
                        offset, total - offset,
                    )
                    return
                raise ValueError(f"Truncated telegram at byte {offset}.")
            yield decode_type6(header + stream.read(length + 4))


def convert_capture(
    source: Path, output_dir: Path, allow_truncated_tail: bool = False,
) -> tuple[int, int]:
    """Write one binary PCD per segment; never merge different sensor frames."""
    count = 0
    point_count = 0
    previous: Segment | None = None
    for segment in read_segments(source, allow_truncated_tail):
        if not count:
            output_dir.mkdir(parents=True, exist_ok=False)
        if previous and segment.sequence != previous.sequence + 1:
            logger.warning(
                "Telegram sequence gap: %s -> %s",
                previous.sequence, segment.sequence,
            )
        points = len(segment.points) // POINT.size
        name = (
            f"sensor_{segment.serial}_frame_{segment.frame}"
            f"_segment_{segment.index}_telegram_{segment.sequence}.pcd"
        )
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            f"# frame_timestamp_us {segment.timestamp_us}\n"
            f"# segment {segment.index}/{segment.segments_per_frame}\n"
            f"# interlace {segment.interlace_index}/{segment.interlace_steps}\n"
            f"# zero_distances_excluded {segment.zero_distances}\n"
            "VERSION 0.7\nFIELDS x y z intensity\n"
            "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
            f"WIDTH {points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {points}\nDATA binary\n"
        )
        with (output_dir / name).open("xb") as output:
            output.write(header.encode("ascii"))
            output.write(segment.points)
        count += 1
        point_count += points
        previous = segment
        logger.info("Wrote %s (%s points)", name, points)
        if not points:
            logger.warning("Telegram %s contains no nonzero distances", segment.sequence)
    if not count:
        raise ValueError("No complete Type 6 telegrams found.")
    return count, point_count
