# salus_simulation

- Responsabilidad: Gazebo, mundos, bridges y normalizadores simulados.
- No contiene: algoritmos que también necesita el robot real.
- Interfaces vigentes: `/cmd_vel_gazebo`, `/odom_raw`, `/joint_states`,
  `/clock`, `/gps/fix_raw` y `/scan_3d_raw`.
- El `gpu_lidar` de Gazebo publica internamente la nube en `/lidar/points`;
  el bridge la expone como `/scan_3d_raw` (`sensor_msgs/PointCloud2`). La
  nube cruda puede incluir un frame con ámbito interno de Gazebo; el
  normalizador publica la API canónica `/scan_3d` en `lidar_link`.
- Estado: mundo Fortress mínimo, ejecutable con
  `ros2 launch salus_simulation motion_sim.launch.py`, con GPS y LiDAR 3D
  reducidos para pruebas. No representa fidelidad RS16.
- Prueba: `colcon test --packages-select salus_simulation` y
  `./tools/smoke_motion_sim.sh`.
- Migración: los normalizadores y sensores simulados se migrarán junto con sus
  consumidores de localización y percepción.

## Perfiles de sensores de simulación (#64)

`config/sensor_profiles/{clean,independent_nominal,degraded}.yaml` define el
contrato simulation-only para ruido, bias, latencia/jitter, dropout/stale data
y calidad GNSS. El seed no vive en el YAML: las composiciones de simulación
aceptan `sim_sensor_profile` (default `clean`) y `sim_sensor_seed` (default
`6400`) para que cada ejecución sea reproducible. `/odom_raw` sigue siendo sólo
ground truth del evaluador.
