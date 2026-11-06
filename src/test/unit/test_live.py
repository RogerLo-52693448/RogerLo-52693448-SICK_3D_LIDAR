from contextlib import closing, redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import socket
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch

from sick_3d_lidar.__main__ import main
from sick_3d_lidar.core.compact import HEADER, TelegramStream
from sick_3d_lidar.services.receiver import live_segments
from unit.test_compact import telegram
from unit.test_detection import scene, object_points


class StreamTests(unittest.TestCase):
    def test_fragmented_and_coalesced_telegrams(self) -> None:
        messages = [telegram(sequence=1), telegram(sequence=2)]
        data = b"".join(messages)
        stream = TelegramStream()
        parsed = []
        for offset in range(0, len(data), 7):
            parsed.extend(stream.feed(data[offset:offset + 7]))
        stream.finish()
        self.assertEqual(parsed, messages)
        self.assertEqual(list(TelegramStream().feed(data)), messages)

    def test_tail_unknown_type_and_oversize_are_explicit(self) -> None:
        stream = TelegramStream()
        list(stream.feed(telegram()[:12]))
        with self.assertRaisesRegex(ValueError, "Truncated"):
            stream.finish()
        with self.assertLogs(level="WARNING"):
            stream.finish(True)
        with self.assertRaisesRegex(ValueError, "Only Compact"):
            list(TelegramStream().feed(bytes(HEADER.size)))
        with self.assertRaisesRegex(ValueError, "exceeds"):
            list(TelegramStream(max_size=100).feed(telegram()))

    @patch("sick_3d_lidar.services.receiver.tcp_chunks")
    def test_receive_overload_fails_instead_of_silently_dropping(self, chunks) -> None:
        chunks.return_value = iter([telegram() * 100])
        with self.assertRaisesRegex(RuntimeError, "queue is full"):
            list(live_segments("localhost", 2115, 1, 1, 1))

    def test_live_cli_fragmented_tcp_to_departure_pcd(self) -> None:
        wire = b"".join(
            telegram(
                rows=1, columns=1, echoes=1, sequence=number, frame=number,
                timestamp_us=number * 100_000, distances=[distance],
            )
            for number, distance in ((1, 1000), (2, 2000), (3, 2100), (4, 1000), (10, 1000))
        )
        errors = []
        requests = []
        with TemporaryDirectory() as directory, socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(3)

            def serve() -> None:
                try:
                    connection, _ = server.accept()
                    with connection:
                        connection.settimeout(3)
                        for offset in range(0, len(wire), 17):
                            connection.sendall(wire[offset:offset + 17])
                        requests.append(connection.recv(1))
                except OSError as error:
                    errors.append(error)

            worker = threading.Thread(target=serve)
            worker.start()
            output = Path(directory) / "objects"
            try:
                with redirect_stdout(StringIO()):
                    result = main([
                        "detect", "--host", "127.0.0.1", "--port", str(server.getsockname()[1]),
                        "--duration", "0.3", "--idle-timeout", "2",
                        "--background-frames", "1", "--min-points", "1",
                        "--min-voxels", "1", "--min-hits", "2",
                        "--exit-seconds", "0.5", "--min-missed-frames", "2",
                        "--output-dir", str(output),
                    ])
            finally:
                worker.join(timeout=4)
            self.assertFalse(worker.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(requests, [b""])
            self.assertEqual(result, 0)
            files = list(output.glob("*.pcd"))
            self.assertEqual(len(files), 1)
            self.assertIn(b"complete_passage true", files[0].read_bytes())

    @patch("sick_3d_lidar.__main__.live_segments")
    def test_interrupt_and_connection_failure_save_only_flagged_partial_objects(self, live) -> None:
        for error, expected in ((KeyboardInterrupt(), 130), (ConnectionError("offline"), 1)):
            with self.subTest(error=type(error).__name__), TemporaryDirectory() as directory:
                def observations():
                    yield scene(1)
                    yield scene(2, object_points(5, 5))
                    yield scene(3, object_points(5, 7))
                    raise error

                live.return_value = observations()
                output = Path(directory) / "objects"
                with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
                    result = main([
                        "detect", "--background-frames", "1", "--min-points", "3",
                        "--min-voxels", "1", "--min-hits", "2", "--output-dir", str(output),
                    ])
                self.assertEqual(result, expected)
                files = list(output.glob("*_partial.pcd"))
                self.assertEqual(len(files), 1)
                self.assertIn(b"complete_passage false", files[0].read_bytes())

    def test_live_generator_close_stops_socket_thread_promptly(self) -> None:
        errors = []
        with socket.socket() as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(3)

            def serve() -> None:
                try:
                    connection, _ = server.accept()
                    with connection:
                        connection.settimeout(3)
                        connection.sendall(telegram())
                        connection.recv(1)
                except OSError as error:
                    errors.append(error)

            worker = threading.Thread(target=serve)
            worker.start()
            started = time.monotonic()
            try:
                with closing(live_segments(
                    "127.0.0.1", server.getsockname()[1], None, 1, 2,
                )) as stream:
                    self.assertEqual(next(stream).sequence, 1)
            finally:
                worker.join(timeout=4)
            self.assertEqual(errors, [])
            self.assertFalse(worker.is_alive())
            self.assertLess(time.monotonic() - started, 2)
