# Real perception pipeline contract

Issue #184 ports only the software pipeline that consumes the raw RS16 output:

```text
/scan_3d (PointCloud2, lidar_link)
  -> scan_ground_filter
  -> /obstacles_cloud (base_footprint)
  -> pointcloud_to_laserscan_node
  -> /scan
  -> scan_noise_filter
  -> /scan_clean (base_footprint)
```

`perception_real.launch.py` owns exactly those three nodes. It does not start
the RS16 driver, `cloud_normalizer`, TF publishers, robot description,
localization, Nav2, Collision Monitor, UART, MAVROS, NTRIP or camera paths.
The external composition must provide the fixed
`base_footprint -> base_link -> lidar_link` transforms. The parameters are
the values characterized in #184: urban ground filtering with 0.20 m
tolerance and 20 m range, projection from -0.1 to 1.6 m over ±90°, 0.4–20 m
range and 0.00872665 rad increment, and the existing speckle filter profile.

Synthetic runtime tests publish a `lidar_link` cloud with an isolated TF
fixture and verify fresh, finite, monotonic `/scan_clean` output with a
plausible obstacle. A missing transform produces no derived cloud or scan.
No RS16 hardware was connected or launched; physical UDP/point-cloud parity
and the legacy `/scan_clean` stall remain separate validation work.
