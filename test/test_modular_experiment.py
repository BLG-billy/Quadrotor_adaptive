"""Exercise inter-process experiment completion without a real PX4 vehicle."""

import csv
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from circle_offboard_py.trajectory_results import aligned_results


def test_error_excludes_samples_outside_reference():
    actual = [(0, 0, 0, -1), (1, 0, 0, -1), (2, 1, 0, -1), (3, 2, 0, 0)]
    target = [(1, 0, 0, -1, 1, 0, 0), (2, 1, 0, -1, 1, 0, 0)]
    result = aligned_results(actual, target)
    assert np.isnan(result[[0, 3], 7]).all()
    assert result[:, 8].tolist() == [0, 1, 1, 0]
    assert np.allclose(result[1:3, 7], 0)


def test_circle_state_includes_slow_ramp_and_excludes_hover():
    actual = [(0, 5, 0, -1), (1, 0.2, 0, -1),
              (2, 0.3, 0, -1), (3, 0, 0, -1)]
    target = [(0, 0, 0, -1, 0, 0, 0), (2, 0, 0, -1, 0.01, 0, 0)]
    result = aligned_results(actual, target, circle_start=1.0)
    assert result[:, 8].tolist() == [0, 1, 1, 0]
    assert np.isnan(result[[0, 3], 7]).all()
    assert np.allclose(result[1:3, 7], [0.2, 0.3])


def test_hover_only_has_no_error_samples():
    actual = [(0, 5, 0, 0), (1, 0, 0, -1)]
    target = [(0, 0, 0, -1, 0, 0, 0), (1, 0, 0, -1, 0, 0, 0)]
    result = aligned_results(actual, target)
    assert np.isnan(result[:, 7]).all()
    assert not result[:, 8].any()


@pytest.mark.parametrize('control_mode', ['wrench', 'position', 'ctbr', 'l1', 'l1observe'])
def test_experiment_saves_automatically(tmp_path, control_mode):
    # A separate domain and namespace isolate all commands from other vehicles.
    env = dict(os.environ, ROS_DOMAIN_ID='187', ROS_LOCALHOST_ONLY='1')
    script = Path(__file__).with_name('mock_px4_experiment.py')
    result = subprocess.run(
        [sys.executable, str(script), str(tmp_path), control_mode], env=env,
        capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    pngs = list(tmp_path.glob('*.png'))
    csvs = [p for p in tmp_path.glob('*.csv') if not p.name.endswith(('_l1_diagnostics.csv', '_control_diagnostics.csv'))]
    assert len(pngs) == len(csvs) == 1
    if control_mode == 'ctbr':
        assert csvs[0].name.startswith('ctbr_')
    assert pngs[0].read_bytes().startswith(b'\x89PNG')
    with csvs[0].open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) > 10
    assert any(row['circle_phase'] == '1.0' for row in rows)
