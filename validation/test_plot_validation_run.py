import csv
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from validation.plot_validation_run import align_position_origins, state_segments


ROOT = Path(__file__).resolve().parent
SCRIPT = ROOT / 'plot_validation_run.py'
COLUMNS = ['t', 'tag', 'state', 'pitch', 'margin', 'slip', 'odom_x',
           'odom_y', 'gz_x', 'gz_y', 'gz_z']
ROWS = [
    [0.0, 'loaded', 'APPROACH', 0.0, 1.0, 0.02,
     0.0, 0.0, 1.0, 0.0, 0.0],
    [1.0, 'loaded', 'ASCENT', 8.0, 0.72, 0.10,
     0.9, 0.0, 1.85, 0.01, 0.1],
    [2.0, 'loaded', 'ASCENT', 16.0, 0.46, 0.18,
     1.8, 0.0, 2.70, 0.02, 0.2],
    [3.0, 'loaded', 'HALT', 16.0, 0.12, 0.62,
     2.0, 0.0, 2.82, 0.03, 0.2],
    [4.0, 'loaded', 'DESCENT', -8.0, 0.50, 0.22,
     1.2, 0.0, 2.05, 0.04, 0.1],
    [5.0, 'loaded', 'EXIT', 0.0, 0.90, 0.05,
     0.1, 0.0, 1.02, 0.02, 0.0],
]


class PlotValidationRunTest(unittest.TestCase):
    def write_csv(self, directory, columns=COLUMNS, rows=ROWS):
        path = Path(directory) / 'run.csv'
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(columns)
            writer.writerows(rows)
        return path

    def test_cli_writes_png_for_guarded_run_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self.write_csv(directory)
            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(csv_path)],
                capture_output=True,
                text=True,
                check=False,
            )
            png_path = csv_path.with_suffix('.png')

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(str(png_path), result.stdout)
            self.assertTrue(png_path.exists())
            self.assertGreater(png_path.stat().st_size, 10_000)
            self.assertEqual(png_path.read_bytes()[:8], b'\x89PNG\r\n\x1a\n')

    def test_position_origins_are_aligned_without_removing_drift(self):
        rows = [
            {'gz_x': 1.0, 'odom_x': 0.0},
            {'gz_x': 2.0, 'odom_x': 0.8},
            {'gz_x': 3.0, 'odom_x': 1.7},
        ]

        truth, estimate = align_position_origins(rows, 'gz_x', 'odom_x')

        self.assertEqual(truth, [1.0, 2.0, 3.0])
        self.assertEqual(estimate, [1.0, 1.8, 2.7])
        self.assertEqual(truth[0] - estimate[0], 0.0)
        self.assertAlmostEqual(truth[-1] - estimate[-1], 0.3)

    def test_cli_reports_missing_columns_without_a_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self.write_csv(
                directory,
                columns=['t', 'state', 'margin'],
                rows=[[0.0, 'APPROACH', 1.0]],
            )
            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(csv_path)],
                capture_output=True,
                text=True,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn('missing required columns', result.stderr)
            self.assertNotIn('Traceback', result.stderr)

    def test_cli_reports_truncated_row_without_a_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self.write_csv(
                directory,
                rows=[ROWS[0][:-2], ROWS[1]],
            )
            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(csv_path)],
                capture_output=True,
                text=True,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn('missing value for gz_y', result.stderr)
            self.assertNotIn('Traceback', result.stderr)

    def test_cli_rejects_series_with_no_usable_values(self):
        with tempfile.TemporaryDirectory() as directory:
            rows = [
                [0.0, 'loaded', 'APPROACH', 0.0, '', '', '', '', '', '', ''],
                [1.0, 'loaded', 'ASCENT', 1.0, '', '', '', '', '', '', ''],
            ]
            csv_path = self.write_csv(directory, rows=rows)
            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(csv_path)],
                capture_output=True,
                text=True,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn('columns contain no usable values', result.stderr)
            self.assertNotIn('Traceback', result.stderr)

    def test_state_bands_change_at_observed_sample_times(self):
        rows = [
            {'t': 0.0, 'state': 'APPROACH'},
            {'t': 1.0, 'state': 'ASCENT'},
            {'t': 10.0, 'state': 'HALT'},
            {'t': 11.0, 'state': 'HALT'},
        ]
        self.assertEqual(
            state_segments(rows),
            [
                (0.0, 1.0, 'APPROACH'),
                (1.0, 10.0, 'ASCENT'),
                (10.0, 11.0, 'HALT'),
            ],
        )

    def test_output_path_is_normalized_or_rejected_as_png(self):
        with tempfile.TemporaryDirectory() as directory:
            csv_path = self.write_csv(directory)
            bare_output = Path(directory) / 'figure'
            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(csv_path),
                 '--output', str(bare_output)],
                capture_output=True,
                text=True,
                check=False,
            )

            png_output = bare_output.with_suffix('.png')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(str(png_output), result.stdout)
            self.assertTrue(png_output.exists())

            result = subprocess.run(
                [sys.executable, str(SCRIPT), str(csv_path),
                 '--output', str(Path(directory) / 'figure.jpg')],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('output path must use a .png suffix', result.stderr)
            self.assertNotIn('Traceback', result.stderr)


if __name__ == '__main__':
    unittest.main()
