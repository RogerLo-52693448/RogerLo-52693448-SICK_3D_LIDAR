"""Command-line entry point for the SICK 3D LiDAR project."""

import argparse
from datetime import datetime, timezone
import logging
from pathlib import Path
import sys

from sick_3d_lidar.services.receiver import capture_tcp
from sick_3d_lidar.core.compact import convert_capture


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
    parser = argparse.ArgumentParser(
        description="Passively capture TCP bytes; does not request or decode scan data."
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
