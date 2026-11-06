from contextlib import redirect_stderr, redirect_stdout
from dataclasses import replace
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from sick_3d_lidar.__main__ import main
from sick_3d_lidar.core.compact import Segment
from sick_3d_lidar.core.detection import DetectionConfig, FrameAssembler, ObjectDetector, voxelize


BACKGROUND = [(20.0, 0.0, 0.0, 0.5), (20.0, 1.0, 0.0, 0.5)]


def object_points(x: float, count: int) -> list[tuple[float, float, float, float]]:
    return [(x + i * 0.01, 0.1, 0.1, 0.75) for i in range(count)]


def scene(number: int, objects=(), **changes) -> Segment:
    points = np.asarray([*BACKGROUND, *objects], dtype="<f4").tobytes()
    segment = Segment(
        sequence=number, serial=789, frame=number, timestamp_us=number * 100_000,
        index=0, segments_per_frame=1, columns=100, rows=1, echoes=1,
        interlace_steps=1, interlace_index=0, points=points, zero_distances=0,
        frame_columns=100, scan_configuration=12,
    )
    return replace(segment, **changes)


def config(**changes) -> DetectionConfig:
    return replace(DetectionConfig(
        background_frames=1, min_points=3, min_voxels=1, min_hits=2,
        exit_seconds=0.5, min_missed_frames=2,
    ), **changes)


class FrameTests(unittest.TestCase):
    def test_only_complete_frames_are_assembled(self) -> None:
        assembler = FrameAssembler()
        first = scene(1, segments_per_frame=2, columns=40)
        self.assertIsNone(assembler.add(first))
        second = replace(first, sequence=2, index=1, columns=60)
        complete = assembler.add(second)
        self.assertIsNotNone(complete)
        self.assertEqual(complete.points, first.points + second.points)
        self.assertEqual(complete.columns, 100)

    def test_missing_segments_are_not_empty_frames(self) -> None:
        assembler = FrameAssembler()
        self.assertIsNone(assembler.add(scene(1, segments_per_frame=2, columns=50)))
        with self.assertLogs(level="WARNING") as logs:
            self.assertIsNone(assembler.add(scene(3, segments_per_frame=2, columns=50)))
        self.assertIn("incomplete frame", str(logs.output))

    def test_configuration_changes_and_sensor_reset_fail(self) -> None:
        for change in (
            {"serial": 12}, {"scan_configuration": 13}, {"rows": 2},
            {"timestamp_us": 50_000}, {"frame": 0}, {"sequence": 1},
        ):
            with self.subTest(change=change):
                assembler = FrameAssembler()
                assembler.add(scene(1))
                with self.assertRaises(ValueError):
                    assembler.add(replace(scene(2), **change))

    def test_duplicate_and_inconsistent_segments_fail(self) -> None:
        for changes in ({"index": 0}, {"interlace_index": 1}, {"timestamp_us": 100_001}):
            with self.subTest(changes=changes):
                assembler = FrameAssembler()
                first = scene(1, segments_per_frame=2, columns=50)
                assembler.add(first)
                second = replace(first, sequence=2, index=1)
                with self.assertRaises(ValueError):
                    assembler.add(replace(second, **changes))


class DetectionTests(unittest.TestCase):
    def test_voxel_connectivity_and_integer_keys_preserve_groups(self) -> None:
        with TemporaryDirectory() as directory:
            detector = ObjectDetector(config(min_points=3, min_voxels=3), Path(directory) / "objects")
            points = np.asarray([
                (0.1, 0.1, 0.1, .5), (0.7, 0.7, 0.7, .5), (1.3, 1.3, 1.3, .5),
                (10.1, 0.1, 0.1, .5), (10.7, 0.7, 0.7, .5), (11.3, 1.3, 1.3, .5),
            ], dtype="<f4")
            self.assertEqual(sorted(len(c) for c in detector._clusters(points)), [3, 3])
            repeated = np.asarray([(-.1, 0, 0), (.1, 0, 0), (-.1, 0, 0)], dtype="<f4")
            cells, inverse = voxelize(repeated, .15)
            np.testing.assert_array_equal(cells[inverse], np.floor(repeated / .15))
            with self.assertRaisesRegex(ValueError, "Voxel"):
                voxelize(repeated, 1e-100)

    def test_two_objects_one_best_pcd_each_after_departure(self) -> None:
        with TemporaryDirectory() as directory:
            output = Path(directory) / "objects"
            detector = ObjectDetector(config(), output)
            detector.process(scene(1))
            detector.process(scene(2, object_points(5, 3) + object_points(10, 4)))
            detector.process(scene(3, object_points(5.1, 6) + object_points(10.1, 7)))
            self.assertEqual(list(output.glob("*.pcd")), [])
            detector.process(scene(4))
            self.assertEqual(detector.saved, 0)
            detector.process(scene(10))
            files = list(output.glob("*.pcd"))
            self.assertEqual(len(files), 2)
            counts = []
            for path in files:
                header, payload = path.read_bytes().split(b"DATA binary\n", 1)
                counts.append(len(payload) // 16)
                self.assertIn(b"complete_passage true", header)
                self.assertIn(b"end_reason departed", header)
                self.assertIn(b"frame_3", path.name.encode())
                self.assertTrue(np.all(np.frombuffer(payload, dtype="<f4").reshape(-1, 4)[:, 0] < 11))
            self.assertEqual(sorted(counts), [6, 7])
            detector.process(scene(11))
            detector.finish("replay_end")
            self.assertEqual(len(list(output.glob("*.pcd"))), 2)

    def test_static_background_and_small_jitter_produce_no_objects(self) -> None:
        with TemporaryDirectory() as directory:
            detector = ObjectDetector(config(), Path(directory) / "objects")
            detector.process(scene(1))
            jitter = [(x + .05, y + .02, z, i) for x, y, z, i in BACKGROUND]
            for number in range(2, 8):
                detector.process(scene(number, points=np.asarray(jitter, dtype="<f4").tobytes()))
            detector.finish("replay_end")
            self.assertEqual(detector.saved, 0)
            self.assertEqual(detector.tracks, {})

    def test_transient_noise_and_insufficient_cluster_voxels_are_rejected(self) -> None:
        with TemporaryDirectory() as directory:
            detector = ObjectDetector(config(), Path(directory) / "objects")
            detector.process(scene(1))
            detector.process(scene(2, object_points(5, 2)))
            self.assertEqual(detector.tracks, {})
            detector.process(scene(3, object_points(5, 5)))
            detector.process(scene(10))
            detector.process(scene(11))
            self.assertEqual(detector.saved, 0)
            strict = ObjectDetector(config(min_voxels=2), Path(directory) / "strict")
            strict.process(scene(1))
            strict.process(scene(2, object_points(5, 5)))
            self.assertEqual(strict.tracks, {})

    def test_background_waits_for_every_interlace_phase(self) -> None:
        with TemporaryDirectory() as directory:
            detector = ObjectDetector(config(), Path(directory) / "objects")
            detector.process(scene(1, interlace_steps=2, interlace_index=0))
            detector.process(scene(2, interlace_steps=2, interlace_index=0))
            self.assertIsNone(detector.background)
            detector.process(scene(3, interlace_steps=2, interlace_index=1))
            self.assertIsNotNone(detector.background)
            detector.process(scene(4, interlace_steps=2, interlace_index=0))
            self.assertEqual(detector.tracks, {})

    def test_shutdown_marks_active_object_partial_and_saves_once(self) -> None:
        with TemporaryDirectory() as directory:
            output = Path(directory) / "objects"
            detector = ObjectDetector(config(), output)
            detector.process(scene(1))
            detector.process(scene(2, object_points(5, 5)))
            detector.process(scene(3, object_points(5, 7)))
            detector.finish("interrupted")
            files = list(output.glob("*_partial.pcd"))
            self.assertEqual(len(files), 1)
            self.assertIn(b"complete_passage false", files[0].read_bytes())
            self.assertIn(b"end_reason interrupted", files[0].read_bytes())
            detector.finish("interrupted")
            self.assertEqual(detector.saved, 1)

    def test_single_timestamp_gap_does_not_count_as_departure(self) -> None:
        with TemporaryDirectory() as directory:
            detector = ObjectDetector(config(), Path(directory) / "objects")
            detector.process(scene(1))
            detector.process(scene(2, object_points(5, 4)))
            detector.process(scene(3, object_points(5, 5)))
            detector.process(scene(20))
            self.assertEqual(detector.saved, 0)
            self.assertEqual(len(detector.tracks), 1)

    def test_roi_excludes_outside_objects(self) -> None:
        with TemporaryDirectory() as directory:
            detector = ObjectDetector(config(roi=(0, 30, -1, 2, -1, 1)), Path(directory) / "objects")
            detector.process(scene(1))
            outside = [(x, y + 10, z, i) for x, y, z, i in object_points(5, 5)]
            detector.process(scene(2, outside))
            self.assertEqual(detector.tracks, {})

    def test_invalid_configuration_rejected(self) -> None:
        for changes in (
            {"background_frames": 0}, {"min_hits": -1}, {"min_points": 0},
            {"change_distance": .01}, {"cluster_size": float("nan")},
            {"max_range": float("inf")}, {"roi": (0, 0, 0, 1, 0, 1)},
            {"min_range": 100},
        ):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                config(**changes)

    def test_output_folder_not_overwritten_and_warmup_failure(self) -> None:
        with TemporaryDirectory() as directory:
            with self.assertRaises(FileExistsError):
                ObjectDetector(config(), Path(directory))
            detector = ObjectDetector(config(background_frames=2), Path(directory) / "objects")
            detector.process(scene(1))
            with self.assertRaisesRegex(ValueError, "background training"):
                detector.finish("replay_end")

    def test_detect_cli_reports_invalid_configuration(self) -> None:
        with TemporaryDirectory() as directory, redirect_stderr(StringIO()), redirect_stdout(StringIO()):
            self.assertEqual(main([
                "detect", "--background-frames", "0",
                "--output-dir", str(Path(directory) / "objects"),
            ]), 1)
