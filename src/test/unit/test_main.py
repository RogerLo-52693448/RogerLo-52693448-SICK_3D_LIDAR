from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
import unittest
from unittest.mock import patch

from sick_3d_lidar.__main__ import main


class MainTests(unittest.TestCase):
    @patch("sick_3d_lidar.__main__.capture_tcp", return_value=42)
    def test_default_device_and_port(self, capture) -> None:
        output = StringIO()
        with redirect_stdout(output):
            result = main(["--output", "output/test.bin"])
        self.assertEqual(result, 0)
        capture.assert_called_once_with(
            "192.168.0.1", 2112, Path("output/test.bin"), 60.0, 5.0, 10.0,
        )
        self.assertIn("42 TCP bytes", output.getvalue())

    @patch("sick_3d_lidar.__main__.capture_tcp", side_effect=TimeoutError("no data"))
    def test_connection_error_is_reported(self, capture) -> None:
        errors = StringIO()
        with redirect_stderr(errors):
            self.assertEqual(main([]), 1)
        self.assertIn("Capture failed: no data", errors.getvalue())

    def test_invalid_arguments_rejected(self) -> None:
        for arguments in (
            ["--port", "0"], ["--port", "65536"], ["--duration", "0"],
            ["--idle-timeout", "nan"], ["--connect-timeout", "inf"],
        ):
            with self.subTest(arguments=arguments), redirect_stderr(StringIO()):
                with self.assertRaises(SystemExit) as raised:
                    main(arguments)
                self.assertEqual(raised.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
