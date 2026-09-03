# RS16 real owner contract

This cut ports the observed RS16 raw owner into `salus_robot` without starting
the sensor or claiming physical validation. The sources are pinned in
`dependencies.repos`:

- `rslidar_sdk`: RoboSense v1.5.18, `7c4ea25fada93442c3d390aa4ef05e240999b851`;
- its `rs_driver` gitlink: `cd358851ab65bf57fc7e321837be2a425305b298`;
- `rslidar_msg`: `fe8a95cb242bd294cc3d5e3422f2093fb49a56ee`.

`salus_hardware/launch/rs16_real.launch.py` is intentionally a one-node,
raw-only owner. It accepts only the configuration path and starts one
`rslidar_sdk_node`; it does not start perception, Nav2, localization, UART,
MAVROS, NTRIP, RViz or any actuator path. The exact legacy RS16 configuration
is packaged at `salus_hardware/config/rs16.yaml` and produces `/scan_3d` with
frame `lidar_link`. The existing physical mount is frozen at
`base_link -> lidar_link`, `xyz=0.92 0 0.65`, `rpy=0 0.1745 0`.

Build and structural tests prove source pins, configuration, launch ownership
and mount geometry. No RS16 hardware was connected or launched for this cut;
packet rates, point-cloud quality and UDP reachability remain pending physical
validation.
