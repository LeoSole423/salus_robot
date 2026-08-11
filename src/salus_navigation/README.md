# salus_navigation

- Responsabilidad: Nav2, goals, rutas, patrulla, HOME, zonas y observabilidad.
- No contiene: actuación, drivers, bridge web ni plugins BT compilados.
- Interfaces vigentes: `/cmd_vel_safe`, `/cmd_vel_teleop`, `/cmd_vel_final`,
  `/nav_command_server/telemetry` y `/nav_command_server/events`.
- Estado: arbitraje manual/automático y `collision_monitor` portados en
  simulación. Goals, Nav2, zonas y misiones siguen pendientes.
- Prueba: `colcon test --packages-select salus_navigation` y
  `./tools/smoke_safety_sim.sh`.
- Migración: Nav2 y objetivos entrarán en un corte posterior sin mezclar la
  lógica de misión con el arbitraje.
