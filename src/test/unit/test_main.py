from contextlib import redirect_stdout
from io import StringIO
import unittest

from sick_3d_lidar.__main__ import main


class MainTests(unittest.TestCase):
    def test_main_reports_unconfigured_input_format(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            main()
        self.assertIn("Configure the sensor model and RAW data format", output.getvalue())


if __name__ == "__main__":
    unittest.main()
