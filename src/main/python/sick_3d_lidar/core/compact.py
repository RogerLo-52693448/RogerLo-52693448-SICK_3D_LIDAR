"""Decode Compact type 6/version 1 using the SICK SDK 0.5.0 layout."""

from dataclasses import dataclass
import logging
import math
from pathlib import Path
import struct
from typing import Iterator
import zlib

import numpy as np


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
    frame_columns: int = 0
    scan_configuration: int = 0


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
        ambient_rows, interlace_steps, interlace_index, scan_configuration, scale, flags,
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
    distances = np.frombuffer(telegram, dtype="<u2", count=samples, offset=distance_start)
    intensity_start = distance_start + samples * 2
    # Wire order is echo/column/row; match the vendor's row/column/echo output.
    ranges = (distances.astype(np.float64) * scale / 1000).reshape(
        echoes, columns, rows,
    ).transpose(2, 1, 0)
    if np.any(ranges > np.finfo(np.float32).max):
        raise ValueError("Scaled distance exceeds the PCD float32 range.")
    elevation = np.asarray(elevations)[:, None, None]
    azimuth = np.asarray(azimuths)[None, :, None]
    points = np.empty((rows, columns, echoes, 4), dtype="<f4")
    points[..., 0] = ranges * np.cos(elevation) * np.cos(azimuth)
    points[..., 1] = ranges * np.cos(elevation) * np.sin(azimuth)
    points[..., 2] = -ranges * np.sin(elevation)
    points[..., 3] = np.nan
    if flags & 1:
        packed = np.frombuffer(
            telegram, dtype=np.uint8, count=intensity_size, offset=intensity_start,
        )
        positions = np.arange(samples) * 12 // 8
        pairs = packed[positions].astype(np.uint16) | (
            packed[positions + 1].astype(np.uint16) << 8
        )
        intensities = ((pairs >> ((np.arange(samples) % 2) * 4)) & 4095) / 4095
        points[..., 3] = intensities.reshape(echoes, columns, rows).transpose(2, 1, 0)
    valid = ranges != 0
    zeros = samples - int(np.count_nonzero(valid))
    return Segment(
        sequence, serial, frame, timestamp, index, segments, columns, rows,
        echoes, interlace_steps, interlace_index, points[valid].tobytes(), zeros,
        frame_columns, scan_configuration,
    )


def type6_size(header: bytes) -> int:
    start, kind, _, _, version, length, _ = HEADER.unpack(header)
    if (start, kind, version) != (0x02020202, 6, 1):
        raise ValueError("Only Compact type 6/version 1 is supported.")
    if length < META.size + 91:
        raise ValueError("Invalid Type 6 payload length.")
    return HEADER.size + length + 4


class TelegramStream:
    """Reassemble TCP chunks without assuming receive boundaries are telegrams."""

    def __init__(self, max_size: int = 16 * 1024 * 1024):
        self.buffer = bytearray()
        self.max_size = max_size

    def feed(self, chunk: bytes) -> Iterator[bytes]:
        self.buffer.extend(chunk)
        while len(self.buffer) >= HEADER.size:
            size = type6_size(self.buffer[:HEADER.size])
            if size > self.max_size:
                raise ValueError(f"Telegram exceeds the {self.max_size}-byte stream limit.")
            if len(self.buffer) < size:
                break
            telegram = bytes(self.buffer[:size])
            del self.buffer[:size]
            yield telegram

    def finish(self, allow_truncated_tail: bool = False) -> None:
        if self.buffer:
            if not allow_truncated_tail:
                raise ValueError("Truncated telegram at end of TCP stream.")
            logger.warning("Skipping %s trailing TCP bytes", len(self.buffer))


def write_pcd(path: Path, points: bytes, comments: tuple[str, ...] = ()) -> int:
    if len(points) % POINT.size:
        raise ValueError("Invalid XYZ/intensity point buffer size.")
    count = len(points) // POINT.size
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        + "".join(f"# {comment}\n" for comment in comments)
        + "VERSION 0.7\nFIELDS x y z intensity\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\nDATA binary\n"
    )
    with path.open("xb") as output:
        output.write(header.encode("ascii"))
        output.write(points)
    return count


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
            try:
                remaining = type6_size(header) - HEADER.size
            except ValueError as error:
                raise ValueError(f"Unsupported Compact header at byte {offset}: {error}") from error
            if remaining > total - stream.tell():
                if allow_truncated_tail:
                    logger.warning(
                        "Skipping truncated telegram at byte %s (%s bytes remain)",
                        offset, total - offset,
                    )
                    return
                raise ValueError(f"Truncated telegram at byte {offset}.")
            yield decode_type6(header + stream.read(remaining))


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
        write_pcd(output_dir / name, segment.points, (
            f"frame_timestamp_us {segment.timestamp_us}",
            f"segment {segment.index}/{segment.segments_per_frame}",
            f"interlace {segment.interlace_index}/{segment.interlace_steps}",
            f"zero_distances_excluded {segment.zero_distances}",
        ))
        count += 1
        point_count += points
        previous = segment
        logger.info("Wrote %s (%s points)", name, points)
        if not points:
            logger.warning("Telegram %s contains no nonzero distances", segment.sequence)
    if not count:
        raise ValueError("No complete Type 6 telegrams found.")
    return count, point_count
