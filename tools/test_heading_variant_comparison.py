"""Comparison cannot promote an unpaired trial or failed parameter export."""
import unittest
import json
from pathlib import Path
import tempfile
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from summarize_heading_navigation import summarize

from tools.compare_heading_variants import compare


class VariantComparisonTest(unittest.TestCase):
    def test_pair_by_recipe_seed_and_retain_failed_pair(self):
        metric = dict(instrumentation_valid=True, localization={'/odometry/global': dict(
            all=dict(heading_deg=dict(maximum=2, rms=1), position_m=dict(rms=.1)),
            recovery=dict(censored=True))}, continuity={'/odometry/global': {}})
        base = dict(kind='right', speed=1.6, radius=4, profile='clean', seed=1,
                    id='base', returncode=0, metrics=metric)
        variant = dict(base, id='lag', variant='lagged', returncode=1)
        unpaired = dict(base, id='unpaired', seed=2)
        pairs = compare([base, variant, unpaired])
        self.assertEqual(len(pairs), 1)
        self.assertFalse(pairs[0]['comparison_eligible'])
        self.assertEqual(pairs[0]['lagged']['returncode'], 1)
        self.assertEqual(pairs[0]['position_rms_reduction_fraction'], 0)

    def test_navigation_setup_failure_is_retained_without_pose_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            trial = root/'trials'/'failed'
            trial.mkdir(parents=True)
            (trial/'summary.json').write_text(json.dumps(dict(
                reason='matrix_setup_failure', errors=['parameter readback timeout'])))
            rows = summarize(root)
            self.assertEqual(len(rows), 1)
            self.assertFalse(rows[0]['functional_pass'])
            self.assertEqual(rows[0]['failed_gates'], ['matrix_setup'])
            self.assertEqual(rows[0]['errors'], ['parameter readback timeout'])


if __name__ == '__main__':
    unittest.main()
