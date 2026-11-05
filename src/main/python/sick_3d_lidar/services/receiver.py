"""Passive TCP capture without sending commands to the sensor."""

import logging
from pathlib import Path
import socket
import time


logger = logging.getLogger(__name__)


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
    if not host or not 1 <= port <= 65535:
        raise ValueError("A host and a port between 1 and 65535 are required.")
    if not all(
        0 < value < float("inf")
        for value in (duration, connect_timeout, idle_timeout)
    ):
        raise ValueError("Duration and timeouts must be finite positive numbers.")

    received = 0
    with socket.create_connection((host, port), timeout=connect_timeout) as connection:
        logger.info("Connected to %s:%s; passive capture only", host, port)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("xb") as capture:
            deadline = time.monotonic() + duration
            last_received = time.monotonic()
            while True:
                now = time.monotonic()
                remaining = deadline - now
                idle_remaining = idle_timeout - (now - last_received)
                if remaining <= 0:
                    break
                if idle_remaining <= 0:
                    raise TimeoutError(
                        f"No TCP data for {idle_timeout:g} seconds; "
                        f"{received} bytes saved to {output}."
                    )
                connection.settimeout(min(remaining, idle_remaining))
                try:
                    data = connection.recv(65536)
                except socket.timeout:
                    continue
                if not data:
                    raise ConnectionError(
                        f"Device closed the connection; "
                        f"{received} bytes saved to {output}."
                    )
                capture.write(data)
                received += len(data)
                last_received = time.monotonic()
                logger.info("Received %s bytes (%s total)", len(data), received)

    if received == 0:
        raise TimeoutError(
            f"Capture ended without receiving data; empty file at {output}. "
            "TCP 2112 may require a documented CoLa B request."
        )
    return received
