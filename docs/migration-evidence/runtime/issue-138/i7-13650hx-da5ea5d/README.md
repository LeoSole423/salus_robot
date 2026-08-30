# Issue #138: local CI-wrapper evidence

These are raw artifacts produced locally from the clean baseline checkout at
`da5ea5df7c0beb41629c222786ab2389fb480eb3`.

Command executed for each run:

```bash
python3 tools/run_registered_smoke.py web_cockpit --context ci
```

The command was run without overriding `ROS_DOMAIN_ID`, `GZ_PARTITION`,
`SMOKE_RUN_TOKEN`, `SMOKE_RUNTIME_DIR`, `FASTDDS_BUILTIN_TRANSPORTS`, or
`SMOKE_HARD_TIMEOUT_S`. The registered CI wrapper configured the 240-second
hard timeout, isolated ROS domains, isolated Gazebo partitions/runtime
folders, and Fast DDS `UDPv4`.

Host: Intel Core i7-13650HX, 20 logical CPUs, with the complete captured host
output in `host-info.txt`.

- Run 1: PASS; source artifact directory `web-cockpit-20260830T214853-1`.
- Run 2: PASS; source artifact directory `web-cockpit-20260830T214907-1`.

The files under `run-1/` and `run-2/` are unmodified copies of the raw smoke
artifacts. No metrics were recalculated, normalized, or edited here.
