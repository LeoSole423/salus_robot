# Smoke readiness evidence

CI v2 treats readiness as a set of causal source-layer observations rather than a single opaque boolean.

## Shared topic states

`tools/smoke_runtime.py` provides `TopicEvidence`, which classifies a required stream as:

- `NO_PUBLISHER`: ROS graph has no producer for the topic;
- `NO_MESSAGES`: a producer exists but the probe received nothing, which points to transport/QoS or producer callback behavior;
- `INVALID`: messages arrive but fail the scenario's validator;
- `NOT_PROGRESSIVE`: valid messages arrive but timestamps do not advance;
- `READY`: valid messages with progressive timestamps are observed.

Reports retain publisher count, received/valid counts, first-message latency, recent timestamps/frames and recent validation errors.

This lets a smoke distinguish, for example, a missing raw source from a normalized topic that exists but never delivers usable data.

## Navigation startup evidence

`NavigationStartupEvidence` records the latest `/navigation_startup/diagnostics` state and the monotonic age of the last diagnostic. `SmokeRuntime.wait_navigation_startup()` is the shared wait primitive for scenarios that require the coordinator to become active.

The age field is diagnostic evidence, not a new production freshness threshold. It exists so artifacts can distinguish an actively publishing `WAITING_INPUTS` reason such as `SCAN_INVALID` from a startup diagnostic that stopped updating while lower-level lifecycle logs continued to progress.

## Composition rule

Shared readiness primitives own only common source/transport evidence and bounded waits. Scenario-specific assertions remain in their owning smoke. CI v2 must not replace functional contracts with a single global `ready=true`.
