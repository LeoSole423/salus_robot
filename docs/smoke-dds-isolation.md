# Smoke DDS transport isolation

SALUS production and interactive development keep the default Fast DDS transport configuration.

The smoke harness is different: `tools/run_smoke.sh` defaults `FASTDDS_BUILTIN_TRANSPORTS` to `UDPv4` for the lifetime of a smoke invocation. `compose.yaml` forwards the environment value into the ROS 2 container but defaults to `DEFAULT` when the harness did not set an override.

This is a CI/runtime-isolation measure. GitHub Actions reproduced Fast DDS shared-memory port/lock failures such as `RTPS_TRANSPORT_SHM Failed init_port ... open_and_lock_file failed` even on fresh scenario runners. Those failures correlated with missing lifecycle heartbeats or stale startup diagnostics.

The smoke harness therefore avoids SHM rather than changing ROS_DOMAIN_ID logic, functional timeouts, retries, or production middleware behavior.

Every smoke artifact records the effective `FASTDDS_BUILTIN_TRANSPORTS` value so a future failure can prove which transport policy was active.

For local diagnosis, `SMOKE_FASTDDS_BUILTIN_TRANSPORTS` can override the smoke default explicitly.
