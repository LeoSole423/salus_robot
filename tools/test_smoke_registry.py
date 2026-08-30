#!/usr/bin/env python3
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

    def test_pr_workflow_is_registry_matrix_driven(self):
        workflow = (ROOT / ".github/workflows/ci.yml").read_text(encoding="utf-8")
        self.assertIn("fromJSON(needs.classify-changes.outputs.smoke_matrix)", workflow)
        self.assertIn('run_registered_smoke.py "${{ matrix.id }}" --context ci', workflow)
        self.assertNotIn("run_smoke.sh ./tools/smoke_", workflow)

    def test_registered_runner_owns_execution_metadata(self):
        runner = (ROOT / "tools/run_registered_smoke.py").read_text(encoding="utf-8")
        self.assertIn('scenario["timeouts_s"][args.context]', runner)
        self.assertIn('scenario.get("env", {})', runner)
        self.assertIn('scenario["script"]', runner)

    def test_nightly_workflow_is_registry_driven(self):
        workflow = (ROOT / ".github/workflows/nightly-smokes.yml").read_text(encoding="utf-8")
        runner = (ROOT / "tools/smoke_reliability.sh").read_text(encoding="utf-8")
        self.assertIn("smoke_registry.py --nightly-matrix", workflow)
        self.assertIn("SMOKE_SCENARIO_ID", runner)
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
