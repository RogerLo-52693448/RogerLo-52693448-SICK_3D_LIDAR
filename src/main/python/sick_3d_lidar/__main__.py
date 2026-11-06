"""Command-line entry point for the SICK 3D LiDAR project."""

import argparse
from contextlib import closing
from datetime import datetime, timezone
import logging
from pathlib import Path
import sys

from sick_3d_lidar.services.receiver import capture_tcp, live_segments
from sick_3d_lidar.core.compact import convert_capture, read_segments


def detect_main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Fixed-sensor change detection; save one best PCD per tracked passage.",
    )
    parser.add_argument("--host", default="192.168.0.1")
    parser.add_argument("--port", type=tcp_port, default=2115)
    parser.add_argument("--duration", type=positive_seconds, help="Omit to run until Ctrl+C.")
    parser.add_argument("--connect-timeout", type=positive_seconds, default=5.0)
    parser.add_argument("--idle-timeout", type=positive_seconds, default=10.0)
    parser.add_argument("--input", type=Path, help="Replay a RAW capture instead of using TCP.")
    parser.add_argument("--allow-truncated-tail", action="store_true")
    parser.add_argument("--output-dir", type=Path, help="New folder; default: output/objects_<UTC>.")
    parser.add_argument("--background-frames", type=int, default=10, help="Empty frames per interlace phase.")
    parser.add_argument("--voxel-size", type=float, default=0.15, help="Background voxel size in meters.")
    parser.add_argument("--change-distance", type=float, default=0.45, help="Minimum background separation in meters.")
    parser.add_argument("--cluster-size", type=float, default=0.6, help="26-neighbor cluster cell size in meters.")
    parser.add_argument("--min-points", type=int, default=50)
    parser.add_argument("--min-voxels", type=int, default=3)
    parser.add_argument("--min-hits", type=int, default=3)
    parser.add_argument("--track-distance", type=float, default=2.0, help="Association gate in meters.")
    parser.add_argument("--exit-seconds", type=float, default=1.0, help="Missing sensor time before departure.")
    parser.add_argument("--min-missed-frames", type=int, default=3)
    parser.add_argument("--min-range", type=float, default=0.5)
    parser.add_argument("--max-range", type=float, default=80.0)
    parser.add_argument("--roi", nargs=6, type=float, metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"))
    args = parser.parse_args(argv)
    if args.input is not None and args.duration is not None:
        parser.error("--duration is only available for live TCP detection.")
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    try:
        from sick_3d_lidar.core.detection import DetectionConfig, FrameAssembler, ObjectDetector
    except ModuleNotFoundError as error:
        if error.name != "scipy":
            raise
        print('Detection requires: python -m pip install -e ".[detect]"', file=sys.stderr)
        return 1
    output = args.output_dir or Path("output") / (
        datetime.now(timezone.utc).strftime("objects_%Y%m%dT%H%M%S_%fZ")
    )
    detector = None
    assembler = FrameAssembler()
    reason = "error"
    result = 0
    try:
        config = DetectionConfig(
            background_frames=args.background_frames, voxel_size=args.voxel_size,
            change_distance=args.change_distance, cluster_size=args.cluster_size,
            min_points=args.min_points, min_voxels=args.min_voxels, min_hits=args.min_hits,
            track_distance=args.track_distance, exit_seconds=args.exit_seconds,
            min_missed_frames=args.min_missed_frames, min_range=args.min_range,
            max_range=args.max_range, roi=tuple(args.roi) if args.roi is not None else None,
        )
        detector = ObjectDetector(config, output)
        logging.info("Keep the ROI empty during background training; do not move the LiDAR.")
        segments = (
            read_segments(args.input, args.allow_truncated_tail)
            if args.input is not None
            else live_segments(
                args.host, args.port, args.duration, args.connect_timeout, args.idle_timeout,
            )
        )
        with closing(segments):
            for segment in segments:
                frame = assembler.add(segment)
                if frame is not None:
                    detector.process(frame)
        reason = "replay_end" if args.input is not None else "duration_end"
    except KeyboardInterrupt:
        reason = "interrupted"
        result = 130
        print("Detection interrupted; confirmed active objects will be marked partial.", file=sys.stderr)
    except (OSError, ValueError, RuntimeError) as error:
        result = 1
        print(f"Detection failed: {error}", file=sys.stderr)
    finally:
        assembler.discard_incomplete()
        if detector is not None:
            try:
                detector.finish(reason)
            except (OSError, ValueError) as error:
                result = 1
                print(f"Detection finalization failed: {error}", file=sys.stderr)
    if result == 0 and detector is not None:
        print(f"Detection ended: {detector.saved} object PCD files in {output}.")
    return result


def convert_main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Convert Compact type 6/version 1 captures to binary PCD."
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--allow-truncated-tail", action="store_true",
        help="Warn and skip only an incomplete final telegram.",
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    try:
        files, points = convert_capture(
            args.input, args.output_dir, args.allow_truncated_tail,
        )
    except (OSError, ValueError) as error:
        print(
            f"Conversion failed: {error} "
            f"Any already-written PCD files remain in {args.output_dir}.",
            file=sys.stderr,
        )
        return 1
    print(f"Converted {files} segments, {points} points to {args.output_dir}.")
    return 0


def positive_seconds(value: str) -> float:
    number = float(value)
    if not 0 < number < float("inf"):
        raise argparse.ArgumentTypeError("Must be a finite positive number.")
    return number


def tcp_port(value: str) -> int:
    number = int(value)
    if not 1 <= number <= 65535:
        raise argparse.ArgumentTypeError("Must be between 1 and 65535.")
    return number


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "convert":
        return convert_main(argv[1:])
    if argv and argv[0] == "detect":
        return detect_main(argv[1:])
    parser = argparse.ArgumentParser(
        description="Passively capture TCP bytes; does not request or decode scan data.",
        epilog="Other modes: convert --help (RAW to PCD), detect --help (live object detection).",
    )
    parser.add_argument("--host", default="192.168.0.1")
    parser.add_argument("--port", type=tcp_port, default=2112)
    parser.add_argument("--duration", type=positive_seconds, default=60.0)
    parser.add_argument("--connect-timeout", type=positive_seconds, default=5.0)
    parser.add_argument("--idle-timeout", type=positive_seconds, default=10.0)
    parser.add_argument(
        "--output", type=Path,
        help="New binary capture path (default: output/tcp_<UTC timestamp>.bin).",
    )
    args = parser.parse_args(argv)
    output = args.output or Path("output") / (
        datetime.now(timezone.utc).strftime("tcp_%Y%m%dT%H%M%S_%fZ.bin")
    )
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    try:
        received = capture_tcp(
            args.host, args.port, output, args.duration,
            args.connect_timeout, args.idle_timeout,
        )
    except (OSError, ValueError) as error:
        print(
            f"Capture failed: {error} (TCP {args.host}:{args.port})",
            file=sys.stderr,
        )
        return 1
    except KeyboardInterrupt:
        print(
            f"Capture interrupted; any received bytes remain at {output}.",
            file=sys.stderr,
        )
        return 130
    print(f"Captured {received} TCP bytes to {output}. Scan data is not decoded.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
