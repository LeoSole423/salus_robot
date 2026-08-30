# Smoke scenario registry

`tools/smoke_scenarios.json` is the authoritative inventory for SALUS smoke scenarios.

Each entry records the stable scenario id, executable script, family, effective CI/nightly hard-timeout budgets, PR/FULL/main/nightly participation, nightly repetition metadata, ownership tags, and scenario-specific environment when applicable.

Consumers should read the registry through `tools/smoke_registry.py` rather than maintaining another scenario list. The change-aware PR selector derives its scenario ids from this registry, and the current nightly reliability runner derives its ordered scenario list and default repetition count from it.

This first CI v2 step intentionally preserves current execution semantics. In particular, existing differences between PR/main coverage and nightly coverage are represented explicitly rather than silently corrected here. Later CI v2 issues can change participation or orchestration with an attributable diff.

Registry validation is part of the fast classifier tests. It verifies that registered scripts exist, selector-owned ids are registered, PR workflow smoke scripts match the PR registry, and nightly execution is registry-driven.

## PR/main matrix execution

CI v2 consumes the change-aware selection as a dynamic matrix. Each selected scenario becomes an independent GitHub Actions job named `smoke / <id>` with `fail-fast: false`.

The workflow passes only the stable scenario id. `tools/run_registered_smoke.py` resolves the script, hard-timeout budget, and scenario-specific environment from the registry before delegating to `tools/run_smoke.sh`. This keeps execution metadata out of the workflow and prevents a second hand-maintained scenario table.

Each matrix entry receives a fresh GitHub runner and uploads `smoke-<id>-artifacts`. A failed scenario is therefore attributable and independently rerunnable without serially re-executing unrelated smokes.

This stage intentionally rebuilds the ROS workspace in each matrix runner. Commit-scoped workspace sharing is a separate CI v2 experiment so performance optimization cannot obscure the isolation change.


## Compiled workspace artifact experiment

CI v2 evaluated sharing an exact-SHA compiled `build/` + `install/` artifact between matrix runners instead of rebuilding the ROS workspace independently.

The experiment proved the transfer contract was technically viable:
- artifact size was about 4.36 MB;
- packaging took about 1 second;
- upload took about 7 seconds;
- representative downloads took about 2 seconds;
- restore was below 1 second;
- restored `--symlink-install` workspaces executed successfully in isolated smoke runners.

However, the architecture adds a serial producer barrier: every smoke must wait for the build job to finish before it can start. In the measured FULL runs, approximate runner consumption dropped from 51.7 to 36.1 runner-minutes (about 30% lower), but workflow wall-clock increased from about 4.9 to 6.9 minutes (about 42% slower).

Because CI v2 currently prioritizes shorter developer feedback latency, PR/main CI keeps independent per-runner workspace builds. The exact-SHA artifact approach may be reconsidered later if runner cost becomes more important than wall-clock latency.
