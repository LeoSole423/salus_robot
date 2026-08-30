#!/usr/bin/env python3
import json
import unittest

from tools.ci_select_smokes import ALL_SMOKES, classify, outputs


class ChangeAwareCiSelectionTest(unittest.TestCase):
    def assert_smokes(self, path, expected):
        selection = classify([path])
        self.assertFalse(selection.full_ci, selection.reasons)
        self.assertEqual(set(selection.smokes), set(expected))

    def test_docs_only_uses_fast_gate_only(self):
        selection = classify(["docs/smoke-testing.md", "README.md"])
        self.assertEqual(selection.classification, "FAST_GATE_ONLY")
        self.assertFalse(selection.full_ci)
        self.assertEqual(selection.smokes, frozenset())

    def test_control_only(self):
        self.assert_smokes(
            "src/salus_control/salus_control/controller.py",
            {"control", "motion", "safety", "integration"},
        )

    def test_localization_only(self):
        self.assert_smokes(
            "src/salus_localization/salus_localization/odometry.py",
            {
                "localization",
                "localization_canonical",
                "sensor_selection",
                "integration",
                "navigation",
                "navigation_canonical",
            },
        )

    def test_navigation_only(self):
        self.assert_smokes(
            "src/salus_navigation/salus_navigation/route_executor.py",
            {
                "safety",
                "integration",
                "navigation",
                "navigation_canonical",
                "navigation_no_obstacles",
                "zones",
                "routes",
                "patrol_battery",
                "snapshot",
                "web_cockpit",
            },
        )

    def test_web_only(self):
        self.assert_smokes(
            "src/salus_web/salus_web/bridge.py",
            {"integration", "web_cockpit"},
        )

    def test_interfaces_force_full(self):
        selection = classify(["src/salus_interfaces/msg/VehicleCommand.msg"])
        self.assertTrue(selection.full_ci)
        self.assertEqual(set(selection.smokes), set(ALL_SMOKES))

    def test_bringup_forces_full(self):
        self.assertTrue(classify(["src/salus_bringup/launch/integration_sim.launch.py"]).full_ci)

    def test_docker_workflow_and_common_tooling_force_full(self):
        for path in (
            "Dockerfile",
            ".github/workflows/ci.yml",
            "tools/run_smoke.sh",
            "entrypoint.sh",
            "docs/package-map.yaml",
        ):
            with self.subTest(path=path):
                self.assertTrue(classify([path]).full_ci)

    def test_unknown_path_falls_back_to_full(self):
        selection = classify(["config/new-runtime-boundary.yaml"])
        self.assertTrue(selection.full_ci)
        self.assertIn("unclassified path", " ".join(selection.reasons))

    def test_perception_runs_lidar_and_navigation_contracts(self):
        self.assert_smokes(
            "src/salus_perception/salus_perception/cloud_normalizer.py",
            {"lidar", "integration", "navigation", "navigation_canonical"},
        )

    def test_navigation_bt_runs_navigation_missions(self):
        selection = classify(["src/salus_navigation_bt/src/path_health_condition.cpp"])
        self.assertFalse(selection.full_ci)
        self.assertTrue(selection.run_navigation_missions)
        self.assertIn("navigation", selection.smokes)
        self.assertIn("web_cockpit", selection.smokes)

    def test_evaluation_has_no_owned_runtime_smoke(self):
        selection = classify(["src/salus_evaluation/salus_evaluation/report.py"])
        self.assertEqual(selection.classification, "FAST_GATE_ONLY")
        self.assertFalse(selection.run_simulation_core)
        self.assertFalse(selection.run_navigation_missions)

    def test_any_full_boundary_dominates_targeted_paths(self):
        selection = classify(
            [
                "src/salus_control/salus_control/controller.py",
                "src/salus_interfaces/msg/VehicleCommand.msg",
            ]
        )
        self.assertTrue(selection.full_ci)
        self.assertEqual(set(selection.smokes), set(ALL_SMOKES))

    def test_empty_change_set_falls_back_to_full(self):
        self.assertTrue(classify([]).full_ci)

    def test_targeted_selection_emits_only_selected_matrix_ids(self):
        selection = classify(["src/salus_web/salus_web/bridge.py"])
        matrix = json.loads(outputs(selection)["smoke_matrix"])
        self.assertTrue(outputs(selection)["run_smokes"] == "true")
        self.assertEqual(
            [entry["id"] for entry in matrix["include"]],
            ["integration", "web_cockpit"],
        )

    def test_fast_gate_only_emits_empty_matrix(self):
        selection = classify(["README.md"])
        matrix = json.loads(outputs(selection)["smoke_matrix"])
        self.assertEqual(outputs(selection)["run_smokes"], "false")
        self.assertEqual(matrix, {"include": []})


if __name__ == "__main__":
    unittest.main()
