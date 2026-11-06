"""Passive TCP capture without sending commands to the sensor."""

from contextlib import ExitStack, closing
import logging
import math
from pathlib import Path
import socket
from queue import Empty, Full, Queue
import threading
import time
from typing import Iterator

from sick_3d_lidar.core.compact import Segment, TelegramStream, decode_type6


logger = logging.getLogger(__name__)


def tcp_chunks(
    host: str, port: int, duration: float | None,
    connect_timeout: float, idle_timeout: float,
    stop: threading.Event | None = None,
) -> Iterator[bytes]:
    if not host or not 1 <= port <= 65535:
        raise ValueError("A host and a port between 1 and 65535 are required.")
    if not all(
        0 < value < float("inf")
        for value in (connect_timeout, idle_timeout)
    ) or (duration is not None and not 0 < duration < float("inf")):
        raise ValueError("Duration and timeouts must be finite positive numbers.")
    with socket.create_connection((host, port), timeout=connect_timeout) as connection:
        logger.info("Connected to %s:%s; passive receive only", host, port)
        deadline = time.monotonic() + duration if duration is not None else math.inf
        last_received = time.monotonic()
        while stop is None or not stop.is_set():
            now = time.monotonic()
            remaining = deadline - now
            idle_remaining = idle_timeout - (now - last_received)
            if remaining <= 0:
                return
            if idle_remaining <= 0:
                raise TimeoutError(f"No TCP data for {idle_timeout:g} seconds.")
            connection.settimeout(min(remaining, idle_remaining, 0.2))
            try:
                data = connection.recv(65536)
            except socket.timeout:
                continue
            if not data:
                raise ConnectionError("Device closed the TCP connection.")
            last_received = time.monotonic()
            yield data


def live_segments(
    host: str, port: int, duration: float | None,
    connect_timeout: float, idle_timeout: float,
) -> Iterator[Segment]:
    """Keep receiving during processing; fail explicitly rather than drop frames."""
    pending: Queue[bytes] = Queue(maxsize=8)
    stop = threading.Event()
    done = threading.Event()
    errors: list[Exception] = []

    def receive() -> None:
        stream = TelegramStream()
        try:
            for chunk in tcp_chunks(
                host, port, duration, connect_timeout, idle_timeout, stop,
            ):
                for telegram in stream.feed(chunk):
                    try:
                        pending.put_nowait(telegram)
                    except Full as error:
                        raise RuntimeError(
                            "Detection cannot keep up with incoming data; receive queue "
                            "is full. No silent frame dropping is allowed. Reduce the "
                            "sensor output rate or use a faster computer."
                        ) from error
            stream.finish(allow_truncated_tail=True)
        except (OSError, ValueError, RuntimeError) as error:
            errors.append(error)
        finally:
            done.set()

    worker = threading.Thread(target=receive, name="lidar-receiver")
    worker.start()
    try:
        while True:
            if errors:
                raise errors[0]
            try:
                telegram = pending.get(timeout=0.1)
            except Empty:
                if done.is_set():
                    break
                continue
            yield decode_type6(telegram)
        if errors:
            raise errors[0]
    finally:
        stop.set()
        worker.join()


def capture_tcp(
    host: str,
    port: int,
    output: Path,
    duration: float,
    connect_timeout: float,
    idle_timeout: float,
) -> int:
    """Save received bytes unchanged, preserving TCP stream order.

    TCP receive chunks are not telegram boundaries. An existing capture is never
    overwritten. Partial captures remain available when a connection fails.
    """
    received = 0
    chunks = tcp_chunks(host, port, duration, connect_timeout, idle_timeout)
    with ExitStack() as stack:
        stack.enter_context(closing(chunks))
        capture = None
        try:
            for data in chunks:
                if capture is None:
                    output.parent.mkdir(parents=True, exist_ok=True)
                    capture = stack.enter_context(output.open("xb"))
                capture.write(data)
                received += len(data)
                logger.info("Received %s bytes (%s total)", len(data), received)
        except (TimeoutError, ConnectionError) as error:
            raise type(error)(
                f"{error} {received} bytes saved"
                + (f" to {output}." if capture is not None else "; no capture created.")
            ) from error

    if received == 0:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("xb"):
            pass
        raise TimeoutError(
            f"Capture ended without receiving data; empty file at {output}. "
            "TCP 2112 may require a documented CoLa B request."
        )
    return received
