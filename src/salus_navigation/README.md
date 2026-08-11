# salus_navigation

- Responsabilidad: Nav2, goals, rutas, patrulla, HOME, zonas y observabilidad.
- No contiene: actuación, drivers, bridge web ni plugins BT compilados.
- Interfaces vigentes: `/cmd_vel_safe`, `/cmd_vel_teleop`, `/cmd_vel_final`,
  `/nav_command_server/telemetry`, `/nav_command_server/events`, `set_goal_ll`
  y `cancel_goal`.
- Estado: arbitraje y navegación Nav2 de un único goal LL portados en
  simulación. Zonas, rutas, patrulla y HOME siguen pendientes.
- Prueba: `colcon test --packages-select salus_navigation`,
  `./tools/smoke_safety_sim.sh` y `./tools/smoke_navigation_core_sim.sh`.
- Migración: el contrato de arrays se conserva, pero múltiples waypoints y
  loop se rechazan explícitamente hasta migrar misiones.
