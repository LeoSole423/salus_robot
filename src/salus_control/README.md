# salus_control

- Responsabilidad: arbitraje final, actuación, telemetría y estimación de batería.
- No contiene: decisiones de misión, UI ni drivers de sensores.
- Interfaces migradas: `/cmd_vel_final`, `/controller/status`,
  `/controller/telemetry`, `/controller/drive_telemetry`, `/battery_state`,
  `/battery_mission_guard` y servicios `/sim_battery/*`.
- Estado: primer corte funcional en simulación. El backend UART está preservado
  como código de compatibilidad, pero no fue conectado ni validado en hardware.
- Prueba: `colcon test --packages-select salus_control`.
- Launch parcial: `ros2 launch salus_control control_sim.launch.py`.
- Signo simulado: `sim_invert_actuation_steer_sign` es booleano, default `false`;
  sólo invierte el ángulo enviado al plugin de Gazebo. El perfil canónico lo
  mantiene desactivado para preservar `angular.z > 0 -> yaw > 0`.
- Presets: `full`, `under_load`, `watching`, `return_home_rest`,
  `return_home_load`, `stale`, `suspect` y `unavailable`.
