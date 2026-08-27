# salus_control

- Responsabilidad: arbitraje final, actuación, telemetría y estimación de batería.
- No contiene: decisiones de misión, UI ni drivers de sensores.
- Interfaces migradas: `/cmd_vel_final`, `/controller/status`,
  `/controller/telemetry`, `/controller/drive_telemetry`, `/battery_state`,
  `/battery_mission_guard`, la observación `/vehicle/command_shadow` y servicios
  `/sim_battery/*`.
- Compatibilidad de salida: `legacy_vehicle_command_adapter` traduce cada
  `/cmd_vel_final` a `VehicleCommand` en unidades Ackermann SI. La salida
  `*_shadow` es observacional por defecto; sólo adquiere autoridad sobre Gazebo
  cuando se selecciona explícitamente el modo canónico de simulación. Nunca
  tiene conexión habilitada hacia UART o hardware.
  Usa timestamp de recepción, vigencia de `0.7 s` y conserva explícitamente la
  semántica histórica `brake_pct > 0 -> emergency_stop`.
- Comparación shadow: `vehicle_command_shadow_comparison` correlaciona por FIFO
  ambos tópicos con timeout monotónico, tolerancias explícitas y colas acotadas.
  Publica únicamente `DiagnosticArray` en
  `/vehicle/command_shadow/diagnostics`; sus divergencias quedan latched en
  contadores y no bloquean ni modifican el comando autoritativo.
- Consumidor canónico `dry_run`: valida rangos, finitud, enum, timestamp,
  vigencia y monotonía; limita `valid_for` a `0.7 s` y aplica un watchdog de
  recepción monotónico. E-stop, disable y freno inhiben movimiento efectivo.
  Publica sólo `/vehicle/command_dry_run/diagnostics`, con
  `authoritative=false`; no tiene conexión a ningún backend.
- Entrada canónica simulada: `command_input_mode=canonical_vehicle_command`
  conecta la misma política validada al único backend `sim_gazebo`. El default
  continúa siendo `legacy_cmd_vel`; el modo canónico con UART se rechaza al
  iniciar. El ángulo se cuantiza al porcentaje entero del backend existente;
  aceleración, jerk y velocidad de giro de dirección aún no son aplicados por
  el plugin simulado.
- Estado: primer corte funcional en simulación. El backend UART está preservado
  como código de compatibilidad, pero no fue conectado ni validado en hardware.
- Prueba: `colcon test --packages-select salus_control`.
- Launch parcial: `ros2 launch salus_control control_sim.launch.py`.
- Signo simulado: `sim_invert_actuation_steer_sign` es booleano, default `false`;
  sólo invierte el ángulo enviado al plugin de Gazebo. El perfil canónico lo
  mantiene desactivado para preservar `angular.z > 0 -> yaw > 0`.
- Presets: `full`, `under_load`, `watching`, `return_home_rest`,
  `return_home_load`, `stale`, `suspect` y `unavailable`.
