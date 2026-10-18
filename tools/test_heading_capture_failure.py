"""The real shell runner must preserve provenance when its container fails."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

class CaptureFailureTest(unittest.TestCase):
    def test_container_failure_keeps_original_status_and_provenance(self):
        repo=Path(__file__).resolve().parents[1]
        (repo/'artifacts').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=repo/'artifacts',prefix='capture-failure-test-') as temporary:
            root=Path(temporary);bin_dir=root/'bin';bin_dir.mkdir()
            docker=bin_dir/'docker'
            docker.write_text('#!/bin/sh\nif [ "$1" = image ]; then echo sha256:fixture; exit 0; fi\nexit 9\n')
            docker.chmod(0o755)
            git=bin_dir/'git'
            git.write_text('#!/bin/sh\nif [ "$1" = rev-parse ]; then echo fixture-head; fi\nexit 0\n')
            git.chmod(0o755)
            target=root/'trial'
            name=str(target.relative_to(repo/'artifacts'))
            env=dict(os.environ,PATH=str(bin_dir)+os.pathsep+os.environ['PATH'])
            result=subprocess.run(['bash',str(repo/'tools/run_heading_capture.sh'),str(repo),name,'94'],env=env,cwd=repo,capture_output=True,text=True)
            self.assertEqual(result.returncode,9,result.stderr)
            self.assertTrue((target/'run.log').exists())
            provenance=json.loads((target/'provenance.json').read_text())
            self.assertEqual(provenance['image_id'],'sha256:fixture')
            self.assertFalse((target/'samples.jsonl').exists())

if __name__=='__main__':unittest.main()
