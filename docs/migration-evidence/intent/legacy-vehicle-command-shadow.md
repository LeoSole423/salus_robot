# Ficha de intención: traducción shadow del comando vehicular

## Alcance

Observar `/cmd_vel_final`, todavía autoritativo, y publicar su traducción física
en `/vehicle/command_shadow`. Este corte no conecta consumidores, no modifica el
backend de simulación o UART y no habilita hardware.

## Semántica de compatibilidad

- El timestamp es el instante ROS de recepción porque `CmdVelFinal` no incluye
  cabecera; `frame_id` es `base_footprint`.
- `valid_for` vale `0.7 s`, alineado con el watchdog legado actual. Un futuro
  consumidor deberá aplicar además su propio límite monotónico.
- La velocidad publicada es la velocidad firmada luego de los límites y
  deadbands existentes.
- La dirección es el ángulo físico de la rueda central virtual Ackermann en
  radianes. La inversión o conversión a porcentaje pertenece al backend.
- La fuente y la habilitación conservan el comportamiento existente.
- Por compatibilidad, cualquier `brake_pct > 0` publica simultáneamente
  `emergency_stop=true`; el porcentaje se satura y normaliza en `[0, 1]`. Esta
  equivalencia histórica queda explícita y no redefine el contrato canónico.
- Un valor cinemático no finito o una fuente fuera del enum produce un comando
  shadow seguro: fuente `SAFETY`, tracción deshabilitada, E-stop, freno completo
  y movimiento nulo.

## Autoridad

El sufijo `_shadow` es parte de la barrera operativa. Ningún launch de este
corte remapea esa salida hacia `controller_server`, Gazebo, UART, ESP32 ni otro
actuador. `/cmd_vel_final` conserva la única ruta de control existente.

## Evidencia

- pruebas puras de límites, marcha atrás, fuentes, freno y entradas inválidas;
- prueba ROS del timestamp, frame, vigencia y serialización del contrato;
- smoke de `control_sim` que inyecta `CmdVelFinal` y observa la traducción;
- build y suite completa del repositorio.

## Pendiente para próximos cortes

- consumidor canónico con watchdog propio, primero en simulación;
- adaptadores de actuación independientes para ESP32 y Pixhawk;
- validación en banco y robot antes de cualquier cambio de autoridad real.
