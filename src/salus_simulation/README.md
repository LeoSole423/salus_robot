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

`config/sensor_profiles/{clean,independent_nominal,degraded}.yaml` son archivos
ROS 2 de parámetros y la única fuente de los valores simulation-only de ruido,
bias, latencia/jitter, dropout y calidad GNSS. El seed no vive en el YAML: las
composiciones de simulación aceptan `sim_sensor_profile` (default `clean`) y
`sim_sensor_seed` (default `6400`) para que cada ejecución sea reproducible.
La ausencia de mensajes y los timeouts existentes representan la falta de
freshness; no hay un umbral `stale_after_s` adicional. `/odom_raw` sigue siendo
sólo ground truth del evaluador.

### Referencia física independiente (#328 / ADR0017)

`/odom_raw` es odometría cinemática del plugin Ackermann, integrada a partir de
articulaciones; no es pose física independiente. `/simulation/ground_truth`
(`nav_msgs/Odometry`, map→base_footprint, 50 Hz, reliable/volatile/depth10)
observa worldPose mediante OdometryPublisher y el bridge. Se usa sólo para
evaluación; no alimenta sensores, EKF, compensación, control ni `/tf`.
La pose es referencia primaria; su twist se deriva y filtra en el plugin.
El modelo conserva L0.94/track0.75 y usa claves Fortress wheel_base/kingpin_width.
