from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import math
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
import unittest
import zlib

from sick_3d_lidar.__main__ import main
from sick_3d_lidar.core.compact import (
    HEADER, META, POINT, convert_capture, decode_type6, read_segments,
)


def telegram(
    rows=2, columns=2, echoes=2, flags=7, distances=None, intensities=None,
    sequence=1, ambient_rows=1, frame=123, timestamp_us=456,
) -> bytes:
    samples = rows * columns * echoes
    distances = distances if distances is not None else [1000] * samples
    intensities = intensities if intensities is not None else list(range(samples))
    metadata = META.pack(
        frame, timestamp_us, 0, 1, columns, columns, rows, echoes, ambient_rows, 1, 0,
        12, 2.0, flags,
    ) + bytes(91)
    elevations = [0.0, math.pi / 2][:rows]
    azimuths = [0.0, math.pi / 2][:columns]
    payload = metadata + bytes(columns * ambient_rows * 2)
    payload += struct.pack(f"<{rows}f", *elevations)
    payload += struct.pack(f"<{columns}f", *azimuths)
    payload += bytes(columns * 6)
    payload += struct.pack(f"<{samples}H", *distances)
    if flags & 1:
        packed = sum(value << (12 * i) for i, value in enumerate(intensities))
        payload += packed.to_bytes((samples * 12 + 7) // 8, "little")
    payload += bytes(samples * (bool(flags & 2) + bool(flags & 4)))
    data = HEADER.pack(0x02020202, 6, sequence, 100, 1, len(payload), 789) + payload
    return data + struct.pack("<I", zlib.crc32(data))


class CompactTests(unittest.TestCase):
    def test_xyz_negative_z_echo_order_and_packed_intensity(self) -> None:
        data = decode_type6(telegram(distances=list(range(1000, 1008))))
        points = list(POINT.iter_unpack(data.points))
        self.assertEqual(len(points), 8)
        self.assertAlmostEqual(points[0][0], 2.0)
        self.assertAlmostEqual(points[1][0], 2.008, delta=1e-6)
        self.assertAlmostEqual(points[2][1], 2.004, delta=1e-6)
        self.assertAlmostEqual(points[4][2], -2.002, delta=1e-6)
        for output_index, raw_index in enumerate((0, 4, 2, 6, 1, 5, 3, 7)):
            self.assertAlmostEqual(points[output_index][3], raw_index / 4095)

    def test_odd_intensity_and_zero_distance(self) -> None:
        data = decode_type6(telegram(
            rows=1, columns=1, echoes=1, intensities=[0xABC],
        ))
        self.assertAlmostEqual(POINT.unpack(data.points)[3], 0xABC / 4095)
        empty = decode_type6(telegram(
            rows=1, columns=1, echoes=1, distances=[0],
        ))
        self.assertEqual(empty.points, b"")
        self.assertEqual(empty.zero_distances, 1)

    def test_absent_intensity(self) -> None:
        point = POINT.unpack(decode_type6(
            telegram(rows=1, columns=1, echoes=1, flags=0),
        ).points)
        self.assertTrue(math.isnan(point[3]))

    def test_crc_and_payload_errors(self) -> None:
        corrupted = bytearray(telegram())
        corrupted[-5] ^= 1
        with self.assertRaisesRegex(ValueError, "CRC32"):
            decode_type6(bytes(corrupted))
        malformed = bytearray(telegram())
        malformed[HEADER.size + META.size - 1] = 0
        malformed[-4:] = struct.pack("<I", zlib.crc32(malformed[:-4]))
        with self.assertRaisesRegex(ValueError, "layout"):
            decode_type6(bytes(malformed))

    def test_invalid_geometry_and_metadata(self) -> None:
        for offset, replacement, message in (
            (HEADER.size + META.size + 91 + 4, struct.pack("<f", math.nan), "geometry"),
            (HEADER.size + 18, struct.pack("<H", 0), "dimensions"),
            (HEADER.size + META.size - 1, b"\x80", "flags"),
            (HEADER.size + META.size - 5, struct.pack("<f", 3e38), "float32"),
        ):
            with self.subTest(message=message):
                malformed = bytearray(telegram(
                    rows=1, columns=2, echoes=1, distances=[65535, 65535],
                ))
                malformed[offset:offset + len(replacement)] = replacement
                malformed[-4:] = struct.pack("<I", zlib.crc32(malformed[:-4]))
                with self.assertRaisesRegex(ValueError, message):
                    decode_type6(bytes(malformed))

    def test_strict_and_allowed_truncated_tail(self) -> None:
        with TemporaryDirectory() as directory:
            source = Path(directory) / "input.bin"
            for tail in (telegram(sequence=2)[:12], telegram(sequence=2)[:-10]):
                source.write_bytes(telegram() + tail)
                with self.assertRaisesRegex(ValueError, "Truncated"):
                    list(read_segments(source))
                with self.assertLogs(level="WARNING") as logs:
                    self.assertEqual(len(list(read_segments(source, True))), 1)
                self.assertIn("truncated", str(logs.output))

    def test_binary_pcd_shape_no_overwrite_and_sequence_warning(self) -> None:
        with TemporaryDirectory() as directory:
            source = Path(directory) / "input.bin"
            output = Path(directory) / "pcd"
            source.write_bytes(telegram() + telegram(sequence=3))
            with self.assertLogs(level="WARNING") as logs:
                self.assertEqual(convert_capture(source, output), (2, 16))
            self.assertIn("sequence gap", str(logs.output))
            for pcd in output.glob("*.pcd"):
                header, data = pcd.read_bytes().split(b"DATA binary\n", 1)
                self.assertIn(b"POINTS 8\n", header)
                self.assertIn(b"WIDTH 8\nHEIGHT 1", header)
                self.assertEqual(len(data), 8 * 16)
            with self.assertRaises(FileExistsError):
                convert_capture(source, output)

    def test_empty_input_and_unsupported_header(self) -> None:
        with TemporaryDirectory() as directory:
            source = Path(directory) / "input.bin"
            source.write_bytes(b"")
            with self.assertRaisesRegex(ValueError, "No complete"):
                convert_capture(source, Path(directory) / "pcd")
            source.write_bytes(HEADER.pack(0x02020202, 1, 0, 0, 1, 0, 0))
            with self.assertRaisesRegex(ValueError, "Unsupported"):
                list(read_segments(source, True))

    def test_convert_cli(self) -> None:
        with TemporaryDirectory() as directory:
            source = Path(directory) / "input.bin"
            source.write_bytes(telegram())
            args = ["convert", str(source), "--output-dir", str(Path(directory) / "pcd")]
            with redirect_stdout(StringIO()):
                self.assertEqual(main(args), 0)
            with redirect_stderr(StringIO()):
                self.assertEqual(main(args), 1)
