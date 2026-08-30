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

The full `web_cockpit` scenario is intentionally registered but has PR/FULL/main/nightly participation disabled. Evidence from #138 showed that its complete operational composition requires materially more CPU than the standard 4-vCPU GitHub runner can provide while preserving simulation time. It therefore acts as a resource/reliability stress workload rather than a deterministic functional gate.

Registered non-gating scenarios remain runnable without bypassing registry-owned execution metadata:

```bash
python3 tools/run_registered_smoke.py web_cockpit --context manual
```

`manual` uses the scenario's CI hard-timeout and environment metadata but does not require PR participation.

This stage intentionally rebuilds the ROS workspace in each matrix runner. Commit-scoped workspace sharing is a separate CI v2 experiment so performance optimization cannot obscure the isolation change.


## Nightly reliability matrix

Nightly reliability is planned from the same registry instead of a hand-maintained shell list. A lightweight plan job emits one matrix entry per nightly-enabled scenario, including its repetition count, hard-timeout metadata, and derived job budget.

Each `nightly / <id>` job:
- runs on a fresh runner with `fail-fast: false`;
- builds the workspace once for that scenario;
- repeats only that registered scenario;
- writes an incremental per-scenario JSON summary with completed/passed/failed/incomplete repetitions;
- uploads artifacts under a scenario-specific name.

This removes the former single 90-minute runner that owned all nightly repetitions and makes reliability attributable per scenario.


### Nightly hard-timeout budgets

Nightly hard timeouts remain scenario metadata rather than a workflow-wide override. Most current nightly scenarios retain the existing 120-second bound. `sim_operational` and `operational_persistence` use 180 seconds because historical healthy executions can consume roughly 90–130 seconds before cleanup, while the previous 120-second global hard wall produced kills around 133–134 seconds. The larger bound is limited to these heavy compositions and reflects their observed execution envelope; it does not add retries or relax functional assertions.

## Compiled workspace artifact experiment

CI v2 evaluated sharing an exact-SHA compiled `build/` + `install/` artifact between matrix runners instead of rebuilding the ROS workspace independently.

The experiment proved the transfer contract was technically viable:
- artifact size was about 4.36 MB;
- packaging took about 1 second;
- upload took about 7 seconds;
- representative downloads took about 2 seconds;
- restore was below 1 second;
- restored `--symlink-install` workspaces executed successfully in isolated smoke runners.

However, the architecture adds a serial producer barrier: every smoke must wait for the build job to finish before it can start. In the measured FULL runs (#105 run 249, `33315182497`, versus #108 experiment run 251, `33316089133`), approximate runner consumption dropped from 51.7 to 36.1 runner-minutes (about 30% lower), but workflow wall-clock increased from about 4.9 to 6.9 minutes (about 42% slower).

Because CI v2 currently prioritizes shorter developer feedback latency, PR/main CI keeps independent per-runner workspace builds. The exact-SHA artifact approach may be reconsidered later if runner cost becomes more important than wall-clock latency.


## Operational smoke contract ownership

CI v2 keeps the operational scenarios focused on their stated boundaries instead of repeating full functional suites.

Coverage ownership after decomposition:

- `sim_operational`: owns canonical full-system composition only. It launches `sim_operational.launch.py` and uses the operational integration probe to verify the composed graph, common source/readiness contracts, TF, required services, and key authorities.
- `routes`: owns route execution and navigation-profile application. `smoke_route_executor_sim.sh` continues to execute both `smoke_route_executor_sim.py` and `smoke_navigation_profiles.py`.
- `web_cockpit`: owns the full Cockpit WebSocket protocol, control lease, camera operations, scan preview, zones, waypoints, manual-safe-stop, snapshot, and safe-operation acknowledgements. It is preserved as a manual/provisioned stress-reliability scenario rather than a standard-runner PR/main gate.
- `operational_persistence`: owns only persistence of Cockpit waypoints and simulated camera presets across owner restart. It launches the minimal `persistence_contract.launch.py` composition containing Web + Camera, seeds state, restarts those owners, and verifies restoration.

The persistence contract intentionally does not require Gazebo, localization, Nav2, keepout, routes, or patrol. Failures in those systems therefore cannot invalidate a persistence assertion.

The existing nightly hard-timeout metadata is left unchanged by this decomposition. After runtime measurements from the focused scenarios are available, budgets may be tightened in a separate evidence-based adjustment.
