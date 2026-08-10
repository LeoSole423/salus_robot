# salus_simulation

- Responsabilidad: Gazebo, mundos, bridges y normalizadores simulados.
- No contiene: algoritmos que también necesita el robot real.
- Interfaces vigentes: `/cmd_vel_gazebo`, `/odom_raw`, `/joint_states` y
  `/clock` para el corte de movimiento.
- Estado: mundo Fortress mínimo, ejecutable con
  `ros2 launch salus_simulation motion_sim.launch.py`; no incluye sensores.
- Prueba: `colcon test --packages-select salus_simulation` y
  `./tools/smoke_motion_sim.sh`.
- Migración: los normalizadores y sensores simulados se migrarán junto con sus
  consumidores de localización y percepción.
