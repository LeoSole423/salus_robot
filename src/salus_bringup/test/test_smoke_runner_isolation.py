import os
from pathlib import Path
import subprocess


RUNNER = Path(__file__).parents[3] / "tools" / "run_smoke.sh"


def test_runner_allocates_isolation_and_cleans_runtime(tmp_path) -> None:
    command = [
        str(RUNNER),
        "bash", "-c",
        'printf "%s|%s|%s" "$SMOKE_ROS_DOMAIN_ID" "$SMOKE_GZ_PARTITION" "$SMOKE_RUNTIME_DIR"',
    ]
    environment = os.environ.copy()
    environment["TMPDIR"] = str(tmp_path)
    result = subprocess.run(command, check=True, capture_output=True, text=True, env=environment)
    values = result.stdout.strip().splitlines()[-1].split("|")
    assert 80 <= int(values[0]) <= 199
    assert values[1].startswith("salus-smoke-")
    assert not Path(values[2]).exists()


def test_runner_preserves_explicit_isolation(tmp_path) -> None:
    environment = os.environ.copy()
    environment.update({
        "TMPDIR": str(tmp_path),
        "SMOKE_ROS_DOMAIN_ID": "211",
        "SMOKE_GZ_PARTITION": "fixture-partition",
    })
    result = subprocess.run(
        [str(RUNNER), "bash", "-c", 'printf "%s|%s" "$SMOKE_ROS_DOMAIN_ID" "$SMOKE_GZ_PARTITION"'],
        check=True, capture_output=True, text=True, env=environment,
    )
    assert result.stdout.strip().splitlines()[-1] == "211|fixture-partition"


def test_runner_rotates_domains_between_sequential_runs(tmp_path) -> None:
    environment = os.environ.copy()
    environment["TMPDIR"] = str(tmp_path)
    command = [
        str(RUNNER), "bash", "-c",
        'printf "%s" "$SMOKE_ROS_DOMAIN_ID"',
    ]
    first = subprocess.run(command, check=True, capture_output=True, text=True, env=environment)
    second = subprocess.run(command, check=True, capture_output=True, text=True, env=environment)
    first_domain = first.stdout.strip().splitlines()[-1]
    second_domain = second.stdout.strip().splitlines()[-1]
    assert first_domain != second_domain
