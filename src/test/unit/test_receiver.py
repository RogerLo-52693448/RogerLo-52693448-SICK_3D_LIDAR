from pathlib import Path
import socket
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

from sick_3d_lidar.services.receiver import capture_tcp


class ReceiverTests(unittest.TestCase):
    @patch(
        "sick_3d_lidar.services.receiver.socket.create_connection",
        side_effect=TimeoutError("connection timed out"),
    )
    def test_failed_connection_does_not_create_capture(self, create_connection) -> None:
        with TemporaryDirectory() as directory:
            output = Path(directory) / "capture.bin"
            with self.assertRaisesRegex(TimeoutError, "connection timed out"):
                capture_tcp("localhost", 2112, output, 1, 1, 1)
            self.assertFalse(output.exists())

    def test_local_tcp_stream_saved_byte_for_byte_without_requests(self) -> None:
        payload = bytes(range(256)) * 512
        server_errors = []
        requests = []
        with socket.socket() as server, TemporaryDirectory() as directory:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(2)

            def serve() -> None:
                try:
                    connection, _ = server.accept()
                    with connection:
                        connection.settimeout(2)
                        connection.sendall(payload[:17])
                        connection.sendall(payload[17:])
                        requests.append(connection.recv(1))
                except OSError as error:
                    server_errors.append(error)

            thread = threading.Thread(target=serve)
            thread.start()
            output = Path(directory) / "output" / "capture.bin"
            try:
                result = capture_tcp(
                    "127.0.0.1", server.getsockname()[1], output, 0.15, 1, 1,
                )
            finally:
                thread.join(timeout=3)
            self.assertFalse(thread.is_alive())
            self.assertEqual(server_errors, [])
            self.assertEqual(requests, [b""])
            self.assertEqual(result, len(payload))
            self.assertEqual(output.read_bytes(), payload)

    @patch("sick_3d_lidar.services.receiver.socket.create_connection")
    def test_empty_capture_is_failure(self, create_connection) -> None:
        connection = create_connection.return_value.__enter__.return_value
        connection.recv.side_effect = socket.timeout()
        with TemporaryDirectory() as directory:
            output = Path(directory) / "capture.bin"
            with self.assertRaisesRegex(TimeoutError, "without receiving data"):
                capture_tcp("localhost", 2112, output, 0.01, 1, 1)
            self.assertEqual(output.read_bytes(), b"")

    @patch("sick_3d_lidar.services.receiver.socket.create_connection")
    def test_partial_capture_survives_disconnect(self, create_connection) -> None:
        connection = create_connection.return_value.__enter__.return_value
        connection.recv.side_effect = [b"\x02\x02", b""]
        with TemporaryDirectory() as directory:
            output = Path(directory) / "capture.bin"
            with self.assertRaisesRegex(ConnectionError, "2 bytes saved"):
                capture_tcp("localhost", 2112, output, 1, 1, 1)
            self.assertEqual(output.read_bytes(), b"\x02\x02")

    @patch("sick_3d_lidar.services.receiver.socket.create_connection")
    def test_existing_file_not_overwritten(self, create_connection) -> None:
        with TemporaryDirectory() as directory:
            output = Path(directory) / "capture.bin"
            output.write_bytes(b"original")
            with self.assertRaises(FileExistsError):
                capture_tcp("localhost", 2112, output, 1, 1, 1)
            self.assertEqual(output.read_bytes(), b"original")

    @patch("sick_3d_lidar.services.receiver.socket.create_connection")
    def test_idle_timeout_is_failure(self, create_connection) -> None:
        connection = create_connection.return_value.__enter__.return_value
        connection.recv.side_effect = socket.timeout()
        with TemporaryDirectory() as directory:
            with self.assertRaisesRegex(TimeoutError, "No TCP data"):
                capture_tcp(
                    "localhost", 2112, Path(directory) / "capture.bin", 1, 1, 0.01,
                )
