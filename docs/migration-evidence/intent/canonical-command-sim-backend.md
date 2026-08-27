# Ficha de intención: comando canónico hacia simulación

## Alcance

- Fuente transitoria: `/cmd_vel_final` traducido a `/vehicle/command_shadow`.
- Destino: backend `sim_gazebo` existente y `/cmd_vel_gazebo`.
- Incluido: selección exclusiva de entrada, validación, watchdog, conversión a
  la frontera interna existente y prueba causal en simulación.
- Fuera de alcance: UART, ESP32, Pixhawk, hardware real y cambio de firmware.

## Evidencia histórica

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| `4c15bca` | El controller y backend simulado conservan telemetría y batería. | alta |
| `src/salus_control/salus_control/sim_gazebo_backend.py` | Un único backend publica el Twist de actuación simulado. | alta |
| `docs/migration-evidence/intent/canonical-command-consumer-dry-run.md` | La validación y el watchdog canónicos ya estaban aislados y probados. | alta |

## Contratos e invariantes

- `command_input_mode=legacy_cmd_vel` continúa siendo el valor predeterminado.
- `canonical_vehicle_command` crea sólo la suscripción canónica y se permite
  exclusivamente junto a `transport_backend=sim_gazebo`.
- No existe fallback silencioso ni dos publicadores sobre `/cmd_vel_gazebo`.
- Velocidad conserva signo y unidad m/s. El ángulo es el ángulo virtual central
  Ackermann en radianes y mantiene el signo ROS usado por la simulación.
- Freno de servicio no se convierte en E-stop; E-stop, disable, rechazo y
  timeout dominan el movimiento.

## Diseño y degradación

El `ControllerServerNode` selecciona exactamente una frontera de entrada. En
modo canónico utiliza `CanonicalCommandConsumer` y su watchdog directamente,
sin volver a aplicar el timeout ni la conversión yaw-rate del flujo legacy.
Después traduce a `DesiredCommand` para reutilizar un solo `SimGazeboBackend`,
preservando su telemetría y simulación de batería.

El backend existente representa dirección como porcentaje entero. Esto
cuantiza el ángulo a aproximadamente 0,3 grados cuando el límite es 30 grados.
Los campos de aceleración, jerk y velocidad angular de dirección se validan,
pero el plugin actual no puede aplicarlos; se consideran restricciones no
soportadas y no se reportan como ejecutadas.

| Condición | Respuesta |
| --- | --- |
| modo desconocido | fallo explícito al iniciar |
| modo canónico con UART | fallo explícito antes de crear el backend |
| comando inválido o vencido | E-stop, freno completo y movimiento cero |
| ausencia de mensajes | watchdog monotónico y parada segura |

## Pruebas y estado

- Unitarios: conversión SI, freno, E-stop, límites y selección de modo/backend.
- Smoke: una autoridad, movimiento canónico observado y parada por watchdog.
- Regresión: el perfil legacy continúa predeterminado para las composiciones.
- Composición: el selector se propaga por `integration_sim` y
  `sim_operational`; el movimiento Gazebo canónico se caracteriza sin lanzar
  una segunda autoridad junto a Nav2.
- Estado propuesto: `ported` sólo en simulación.
- No validado: UART, banco, Jetson o robot real.
