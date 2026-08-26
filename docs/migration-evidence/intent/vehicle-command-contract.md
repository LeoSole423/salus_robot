# Ficha de intención: contrato canónico de comando vehicular

## Alcance

Definir la frontera ROS compartida entre el arbitraje seguro de movimiento y
los adaptadores de actuación. Este corte compila el contrato, pero no cambia el
productor `/cmd_vel_final`, no conecta un consumidor nuevo y no mueve hardware.

## Hechos históricos

- `CmdVelFinal` transporta `geometry_msgs/Twist`, `brake_pct` y `source`.
- `controller_server` convierte velocidad lineal y yaw rate a ángulo Ackermann,
  limita velocidad/dirección y genera `steer_pct` para el protocolo UART.
- El estado interno entregado tanto a UART como a Gazebo mezcla unidades a
  nivel vehículo (`speed_mps`) con una unidad específica del ESP32
  (`steer_pct`).
- Actualmente cualquier `brake_pct > 0` activa también `estop`; freno de
  servicio y parada de emergencia no son conceptos independientes.
- El watchdog se basa en tiempo monotónico de recepción y el protocolo UART
  transmite periódicamente; ambas propiedades son invariantes de seguridad.

## Decisión

`salus_interfaces/VehicleCommand` será un sobre atómico y agnóstico del backend.
Contiene:

- `std_msgs/Header header`: timestamp del productor y frame cinemático;
- `builtin_interfaces/Duration valid_for`: vigencia solicitada, siempre
  positiva y limitada además por el watchdog local del consumidor;
- `source`: `UNKNOWN`, `AUTO`, `MANUAL` o `SAFETY`;
- `drive_enabled`: habilitación ordinaria de tracción;
- `emergency_stop`: parada de emergencia explícita e independiente;
- `brake_ratio`: freno de servicio normalizado en `[0.0, 1.0]`;
- `ackermann_msgs/AckermannDrive drive`: velocidad firmada, aceleración, jerk,
  ángulo de la rueda central virtual y velocidad de dirección en unidades SI.

Se reutiliza `AckermannDrive` porque expresa el movimiento de un vehículo
Ackermann sin imponer Pixhawk, ESP32, PWM, porcentaje de dirección ni protocolo
de transporte. Los adaptadores posteriores convertirán esta semántica física a
sus setpoints nativos.

## Invariantes para consumidores posteriores

- `emergency_stop=true` domina habilitación, movimiento y freno solicitado; el
  backend aplica su estado seguro configurado.
- `drive_enabled=false` prohíbe propulsión, pero no falsea E-stop ni convierte
  el freno de servicio en emergencia.
- Todos los escalares deben ser finitos; `brake_ratio` pertenece a `[0, 1]` y
  `valid_for` debe ser mayor que cero.
- Un consumidor mantiene un watchdog monotónico propio y limita `valid_for` a
  su máximo local; nunca confía sólo en el reloj o plazo del productor.
- Un timeout, timestamp inválido/regresivo, mensaje mal formado o fuente no
  admitida produce comando seguro y diagnóstico explícito.
- La conversión a `steer_pct`, PWM, torque o bytes UART pertenece al adaptador
  de hardware, no al contrato compartido.
- La transmisión real sigue siendo periódica y el cierre del transporte sigue
  enviando múltiples comandos seguros.

## Compatibilidad y migración prevista

`CmdVelFinal` permanece vigente. El siguiente corte deberá crear una traducción
pura y un adaptador ROS en modo sombra antes de cambiar autoridades. Sólo tras
comparación reproducible se podrá seleccionar la nueva entrada del controlador.

La aceleración y el jerk son restricciones cinemáticas del mensaje estándar;
no se interpretan como porcentaje de acelerador. Un backend que no pueda
honrarlas debe declarar esa capacidad y aplicar límites seguros, no fingir que
las ejecutó.

## Pruebas de este corte

- compilación ROSIDL con la dependencia estándar `ackermann_msgs`;
- inventario/documentación coherentes;
- build y suite completa del repositorio.

## Evidencia pendiente

- política pura `CmdVelFinal -> VehicleCommand` y validación de vigencia;
- comparación shadow en simulación;
- adaptador `VehicleCommand -> ESP32 UART` sin porcentajes aguas arriba;
- backend Pixhawk opcional y futuros controladores;
- watchdog, E-stop, freno y calibración validados en banco/robot.
