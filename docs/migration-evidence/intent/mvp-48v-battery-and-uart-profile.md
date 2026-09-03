# Ficha de intención — batería 48 V y perfil UART MVP (#165)

## Alcance

Este corte porta únicamente la política de batería operativa de 48 V y declara
el perfil de autoridad UART que usará el MVP. No inicia el perfil, no abre un
serial y no valida hardware.

## Hechos de referencia

- `ROS2_SALUS@f35834989b041f51dd325c626d2338e2232d9e53` cambió la guardia de
  misión a un pack 48 V y retiró los filtros EMA propios de la batería de plomo.
- La medición de la ESP32 ya está calibrada y estabilizada; por eso se usa la
  muestra actual directamente.
- La cadena de control MVP conserva el contrato histórico:
  `/cmd_vel_final` (`salus_interfaces/msg/CmdVelFinal`) → `controller_server`
  → UART → ESP32. `VehicleCommand` no adquiere autoridad sobre UART en este
  corte.

## Política portada

| Concepto | Valor |
| --- | ---: |
| Referencia superior | 53.5 V |
| LOW | <=47.0 V |
| RETURN_HOME | <=46.5 V continuos por 30 s |
| Clear de guardia | >=48.0 V continuos por 30 s |
| CRITICAL | <=45.0 V |
| Mínimo | 44.5 V |
| SOC orientativo | 44.5/0%, 46.5/15%, 48/35%, 50/60%, 52/85%, 53.5/100% |

La prioridad de estado es `UNAVAILABLE`, `STALE`, `SUSPECT`, guardia de misión,
`BELOW_MINIMUM`, `CRITICAL`, `LOW`, `OK`. La protección de misión depende de
voltaje y persistencia; el SOC no reemplaza a la guardia tipada.

## Perfil físico futuro

`src/salus_control/launch/control_real_uart.launch.py` declara una sola
instancia de `controller_server_node` con:

- `use_sim_time=false`;
- `transport_backend=uart`;
- `command_input_mode=legacy_cmd_vel`;
- `serial_port=auto`, `serial_baud=115200`, `serial_tx_hz=50`.

No contiene adaptadores shadow, consumidores canónicos ni publicadores
alternativos de `/cmd_vel_final`. Es autoridad física y está excluido de
`real_observation.launch.py` y `real_localization_shadow.launch.py`.

## Validación y límite

Las pruebas unitarias cubren los umbrales exactos, continuidad de los 30 s,
reinicio del temporizador, prioridad fail-safe y presets de simulación. Las
pruebas estructurales demuestran que el perfil UART tiene una sola autoridad y
no entra en perfiles read-only. La activación física, la lectura del serial,
el control de actuadores y el rollback quedan para #168 con el robot presente.
