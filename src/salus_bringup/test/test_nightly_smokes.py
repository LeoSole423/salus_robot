from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_nightly_workflow_uses_registry_matrix_per_scenario():
    workflow = (ROOT / '.github/workflows/nightly-smokes.yml').read_text()

    assert 'args=(--nightly-matrix)' in workflow
    assert 'python3 tools/smoke_registry.py "${args[@]}"' in workflow
    assert 'fail-fast: false' in workflow
    assert 'name: nightly / ${{ matrix.id }}' in workflow
    assert 'timeout-minutes: ${{ matrix.job_timeout_minutes }}' in workflow
    assert 'SMOKE_SCENARIO_ID: ${{ matrix.id }}' in workflow
    assert '--nightly-repetitions-override' in workflow
    assert 'SMOKE_REPETITIONS: ${{ matrix.repetitions }}' in workflow
    assert 'nightly-${{ matrix.id }}-artifacts' in workflow
    assert 'timeout-minutes: 90' not in workflow


def test_reliability_runner_writes_per_scenario_incremental_summary():
    runner = (ROOT / 'tools/smoke_reliability.sh').read_text()

    assert 'SMOKE_SCENARIO_ID' in runner
    assert 'configured_repetitions' in runner
    assert 'completed_repetitions' in runner
    assert 'passed_repetitions' in runner
    assert 'failed_repetitions' in runner
    assert 'incomplete_repetitions' in runner
    assert 'write_summary running' in runner
