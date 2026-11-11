"""Frontal relay contract: TCP video-only input, stream copy, local publication."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("front_video_relay", Path(__file__).parents[1] / "deploy/mediamtx/front_video_relay.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FrontRelayTests(unittest.TestCase):
    def setUp(self):
        self.config = {"ffmpeg": "/opt/ffmpeg/bin/ffmpeg", "sources": {
            "cam3": "rtsp://user:password@camera/media1/video2",
            "cam3_normal": "rtsp://user:password@camera/media1/video1"}}

    def test_selects_video_before_input_setup_and_never_reencodes(self):
        args = module.build_command(self.config, "cam3_normal")
        self.assertEqual(args[args.index("-allowed_media_types") + 1], "video")
        self.assertLess(args.index("-allowed_media_types"), args.index("-i"))
        self.assertEqual(args[args.index("-c:v") + 1], "copy")
        self.assertEqual(args.count("tcp"), 2)
        self.assertTrue(all(flag in args for flag in ("-an", "-sn", "-dn")))
        self.assertEqual(args[-1], "rtsp://127.0.0.1:8554/cam3_normal")

    def test_saving_uses_its_original_secondary_source(self):
        args = module.build_command(self.config, "cam3")
        self.assertEqual(args[args.index("-i") + 1], self.config["sources"]["cam3"])
        self.assertTrue(args[-1].endswith("/cam3"))

    def test_source_is_one_argument_even_with_shell_metacharacters(self):
        self.config["sources"]["cam3"] += "?literal=$(echo test); value"
        args = module.build_command(self.config, "cam3")
        self.assertEqual(args[args.index("-i") + 1], self.config["sources"]["cam3"])

    def test_rejects_non_frontal_paths_and_invalid_sources(self):
        with self.assertRaises(ValueError):
            module.build_command(self.config, "cam5")
        self.config["sources"]["cam3"] = "file:///private/file"
        with self.assertRaises(ValueError):
            module.build_command(self.config, "cam3")

    def test_rejects_invalid_publish_port(self):
        self.config["publish_port"] = 0
        with self.assertRaises(ValueError):
            module.build_command(self.config, "cam3")


if __name__ == "__main__":
    unittest.main()
