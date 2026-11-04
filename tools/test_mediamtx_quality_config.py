"""Host-owned relay configuration: additions must preserve existing consumers."""
import copy
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("quality_config", Path(__file__).parents[1] / "deploy/mediamtx/prepare_quality_config.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class QualityConfigTests(unittest.TestCase):
    def setUp(self):
        self.config = {"apiAddress": "127.0.0.1:9997", "paths": {
            **{f"cam{i}": {"source": f"rtsp://user:password@camera/media{i}/video2", "sourceOnDemand": True} for i in (3, 4, 5)},
            "rec_cam3": {"source": "publisher", "record": True},
            "all_others": {}}}
        self.config["paths"]["cam5"]["rtspTransport"] = "tcp"

    def test_adds_aliases_without_modifying_input_or_recordings(self):
        before = copy.deepcopy(self.config)
        result = module.add_quality_paths(self.config)
        self.assertEqual(self.config, before)
        for name, route in before["paths"].items():
            self.assertEqual(result["paths"][name], route)
        for name in ("cam3", "cam4", "cam5"):
            route = result["paths"][name + "_normal"]
            self.assertTrue(route["source"].endswith("video1"))
            self.assertTrue(route["sourceOnDemand"])
            self.assertNotIn("record", route)
        self.assertEqual(result["paths"]["cam5_normal"]["rtspTransport"], "tcp")

    def test_rejects_unknown_source_instead_of_guessing(self):
        self.config["paths"]["cam3"]["source"] = "publisher"
        with self.assertRaises(ValueError):
            module.add_quality_paths(self.config)

    def test_existing_alias_requires_explicit_review(self):
        self.config["paths"]["cam3_normal"] = {"source": "publisher"}
        with self.assertRaises(ValueError):
            module.add_quality_paths(self.config)


if __name__ == "__main__":
    unittest.main()
