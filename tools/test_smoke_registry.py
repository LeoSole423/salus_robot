#!/usr/bin/env python3
import re
import unittest
from pathlib import Path

from tools.ci_select_smokes import ALL_SMOKES, PACKAGE_SMOKES
from tools.smoke_registry import BY_ID, SCENARIOS, ids, nightly_scripts

ROOT = Path(__file__).resolve().parents[1]


class SmokeRegistryTest(unittest.TestCase):
    def test_registered_scripts_exist(self):
        for scenario in SCENARIOS:
            with self.subTest(scenario=scenario["id"]):
                self.assertTrue((ROOT / scenario["script"]).is_file())

    def test_selector_only_references_registered_ids(self):
        registered = set(BY_ID)
        self.assertEqual(set(ALL_SMOKES), set(ids(participation="pr")))
        for prefix, selected in PACKAGE_SMOKES.items():
            with self.subTest(prefix=prefix):
                self.assertLessEqual(set(selected), registered)

    def test_pr_registry_matches_ci_smoke_scripts(self):
        workflow = (ROOT / ".github/workflows/ci.yml").read_text(encoding="utf-8")
        referenced = set(
            re.findall(r"run_smoke\.sh \.\/tools\/([^\s]+)", workflow)
        )
        expected = {
            Path(BY_ID[scenario_id]["script"]).name
            for scenario_id in ids(participation="pr")
        }
        self.assertEqual(referenced, expected)

    def test_nightly_runner_is_registry_driven(self):
        runner = (ROOT / "tools/smoke_reliability.sh").read_text(encoding="utf-8")
        self.assertIn("smoke_registry.py --nightly-scripts", runner)
        self.assertNotIn("scenarios=(", runner)
        self.assertGreater(len(nightly_scripts()), 0)

    def test_registry_records_current_known_nightly_drift_explicitly(self):
        self.assertFalse(BY_ID["navigation_canonical"]["participation"]["nightly"])
        self.assertFalse(BY_ID["navigation_no_obstacles"]["participation"]["nightly"])
        self.assertFalse(BY_ID["sensor_selection"]["participation"]["nightly"])
        self.assertFalse(BY_ID["web_cockpit"]["participation"]["nightly"])
        self.assertTrue(BY_ID["sim_operational"]["participation"]["nightly"])
        self.assertTrue(BY_ID["operational_persistence"]["participation"]["nightly"])


if __name__ == "__main__":
    unittest.main()
