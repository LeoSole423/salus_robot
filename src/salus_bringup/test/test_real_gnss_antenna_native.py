"""Exercise real adapter/selector wiring against native robot_localization."""
import json
import os
from pathlib import Path
import subprocess
import sys


def test_real_raw_gnss_uses_native_lever_arm(tmp_path):
    root = Path(__file__).resolve().parents[3]
    output = tmp_path / 'native.json'
    environment = dict(os.environ, ROS_DOMAIN_ID='159')
    result = subprocess.run(
        [sys.executable, str(root / 'tools/check_real_gnss_antenna.py'),
         '--output', str(output)],
        env=environment, capture_output=True, text=True, timeout=65,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(output.read_text())
    assert report['passed']
    assert len(report['angles']) == 4
    assert max(row['error_m'] for row in report['angles']) < .005
