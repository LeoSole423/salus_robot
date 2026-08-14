from pathlib import Path
import subprocess


HARNESS = Path(__file__).parents[3] / "tools" / "smoke_harness.sh"


def test_duplicate_artifact_name_is_rejected(tmp_path) -> None:
    command = f"""
      source {HARNESS}
      export SMOKE_ARTIFACT_ROOT={tmp_path}
      smoke_init duplicate-fixture
      smoke_reserve_artifact_name repeated
      smoke_reserve_artifact_name repeated
    """
    result = subprocess.run(["bash", "-c", command], text=True, capture_output=True)
    assert result.returncode != 0
    assert "already reserved" in result.stderr
