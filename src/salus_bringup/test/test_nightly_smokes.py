from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_nightly_workflow_has_budget_and_ten_scheduled_suites():
    workflow = (ROOT / '.github/workflows/nightly-smokes.yml').read_text()

    assert 'timeout-minutes: 90' in workflow
    assert 'Repeat isolated smoke scenarios three times' not in workflow
    assert "inputs.repetitions || '10'" in workflow


def test_reliability_runner_writes_incremental_machine_summary():
    runner = (ROOT / 'tools/smoke_reliability.sh').read_text()

    assert 'reliability-summary.json' in runner
    assert 'write_summary running' in runner
    assert 'completed_suites' in runner
    assert 'completed_scenarios' in runner
    assert 'expected_scenarios' in runner
