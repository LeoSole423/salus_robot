# salus_control

- Responsabilidad: arbitraje final, actuación, telemetría y estimación de batería.
- No contiene: decisiones de misión, UI ni drivers de sensores.
- Interfaces migradas: `/cmd_vel_final`, `/controller/status`,
  `/controller/telemetry`, `/controller/drive_telemetry`, `/battery_state`,
  `/battery_mission_guard`, la observación `/vehicle/command_shadow` y servicios
  `/sim_battery/*`.
- Compatibilidad de salida: `legacy_vehicle_command_adapter` consume cada
  `/cmd_vel_final` como `interfaces/msg/CmdVelFinal` y lo traduce a
  `salus_interfaces/msg/VehicleCommand` en unidades Ackermann SI. La salida
  `*_shadow` es observacional por defecto; sólo adquiere autoridad sobre Gazebo
  cuando se selecciona explícitamente el modo canónico de simulación. Nunca
  tiene conexión habilitada hacia UART o hardware.
  Usa timestamp de recepción, vigencia de `0.7 s` y conserva explícitamente la
  semántica histórica `brake_pct > 0 -> emergency_stop`.
- `input_wire_type=salus_interfaces` es el default para los productores de
  simulación. La coexistencia con `ROS2_SALUS` fija explícitamente
  `input_wire_type=interfaces` en el perfil real read-only; cada nodo crea una
  sola suscripción del tipo elegido y rechaza cualquier otro valor.
- Comparación shadow: `vehicle_command_shadow_comparison` correlaciona por FIFO
  la entrada legacy `interfaces/msg/CmdVelFinal` y el shadow canónico con
  timeout monotónico, tolerancias explícitas y colas acotadas.
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
- Batería 48 V: la ESP32 entrega una muestra calibrada y estabilizada. SALUS no
  aplica filtros EMA de la batería de plomo anterior: `53.5 V` es referencia
  superior, `47.0 V` LOW, `45.0 V` CRITICAL y `44.5 V` el mínimo. El porcentaje
  es sólo orientación de operador; la guardia de misión se enclava tras `30 s`
  continuos a `<=46.5 V` y se limpia tras `30 s` a `>=48.0 V`.
- Perfil físico MVP: `control_real_uart.launch.py` contiene un único
  `controller_server_node`, con `transport_backend=uart` y
  `command_input_mode=legacy_cmd_vel`. Es una autoridad física deliberada y no
  debe combinarse con perfiles read-only/shadow ni usarse antes del cutover.
- Estado: primer corte funcional en simulación. El perfil UART está especificado
  y probado estructuralmente, pero no fue conectado ni validado en hardware.
- Prueba: `colcon test --packages-select salus_control`.
- Launches parciales: `ros2 launch salus_control control_sim.launch.py` y,
  sólo para el cutover físico autorizado, `ros2 launch salus_control
  control_real_uart.launch.py`.
- Signo simulado: `sim_invert_actuation_steer_sign` es booleano, default `false`;
  sólo invierte el ángulo enviado al plugin de Gazebo. El perfil canónico lo
  mantiene desactivado para preservar `angular.z > 0 -> yaw > 0`.
- Presets: `full`, `under_load`, `watching`, `return_home_rest`,
  `return_home_load`, `stale`, `suspect` y `unavailable`.


### Low-speed command invariant

`vx_min_effective_mps` is not an output speed floor. Positive commands above
the deadband preserve the upstream requested speed (subject only to the configured
maximum). The value is retained only as a virtual steering-reference speed when
linear velocity is effectively zero, so a downstream adapter never increases a
Nav2 or safety slowdown command.

### Ackermann turning authority

`/cmd_vel_final.angular.z` is converted to requested curvature using the
command's linear speed (or the documented near-zero steering reference), then
to steering with the wheelbase. The applied steering is limited by the smaller
of the physical `steering_limit_rad` and the source-specific operational
limit. This saturation remains observable as requested/applied curvature and
steering plus `steer_saturated`. There is intentionally no fixed
`max_abs_angular_z`: such a yaw-rate cap would impose a speed-dependent second
curvature authority rather than a fixed Ackermann steering limit.
