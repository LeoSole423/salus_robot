# Issue #282 — evaluación PC/sim de `legacy_pair`

## Alcance

Esta evidencia valida el modo opt-in `legacy_pair` integrado por el corte SOL.
No cambia el default `single_checkpoint`, no habilita el modo real y no usa
Jetson ni hardware.

La comparación usa el mismo `integration_sim`, `free.world`, sensor profile
`clean`, seed `6400`, y los defaults de ruta `leg_spacing_m=35`,
`chunk_span_m=120`, `chunk_max_waypoints=5`. Los yaws de la misión son
automáticos para que los checkpoints normales sean elegibles para pair; las
políticas de yaw productivas no se modifican.

## Comandos

Se validó primero el aislamiento:

```bash
./tools/nav_eval.sh isolation artifacts/evaluations/issue282-luna-isolation
```

Resultado: `PASS`, con clocks monótonos, dominios/particiones distintos,
procesos aislados, Nav2 activo, muerte de sibling y cleanup correctos.

Cada brazo se ejecutó serialmente mediante `smoke_harness.sh`, con un dominio y
partición propios. Los artifacts completos quedaron bajo:

```text
artifacts/evaluations/issue282-luna/
```

## Resultado A/B

| Brazo | Dispatches | Requests | Sintéticos | Planes observados | Detour máximo | Max deviation máximo | Self-intersections |
|---|---:|---|---:|---:|---:|---:|---:|
| `single_checkpoint` | 3 | `[P0]`, `[P1]`, `[P2]` | 0 | 6 | 1.00283 | 0.0936 m | 0 |
| `legacy_pair` | 2 | `[P0,P1]`, `[P2]` | 0 | 5 | 1.00000 | 0.0845 m | 0 |

Ambos brazos completaron el smoke de ruta con navegación, yaw comparison,
RPP branch gate, cancelación y cleanup verdes. El brazo pair conservó el
primer checkpoint normal como pose intermedia y el segundo como terminal; el
último checkpoint se despachó en un request singleton. No hubo poses sintéticas
en ninguno de los dos brazos.

La trayectoria observada por el smoke fue separada del `/plan`; la evidencia
incluye odometría, `/received_global_plan`, comandos finales, estados y trazas
de progreso. No se interpretan métricas de planes como sustituto de trayectoria
ejecutada.

## Regresión patrol/HOME

Con `route_execution_mode=legacy_pair`, `smoke_patrol_battery` completó:

```text
JOIN_LOOP -> PATROL -> EXIT_LOOP -> RETURN_HOME -> AT_HOME
```

Esto confirma que el coordinador envía roles `hard` y que patrol/HOME no adopta
accidentalmente la semántica pair. El artifact está en el mismo directorio de
campaña.

## Limitaciones y siguiente corte

- Es una caracterización PC/sim, no validación física.
- Se ejecutó una corrida válida por brazo y una regresión patrol/HOME.
- No se cambia el default ni se recomienda todavía habilitar `legacy_pair` en
  producción.
- Quedan por ampliar en el siguiente corte las pruebas específicas de loops,
  acciones en checkpoints, retry/recovery y takeover/cancel durante un pair.
