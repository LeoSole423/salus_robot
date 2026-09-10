# Intención: ejecutor de rutas

Fuente histórica: `fb54b95`, `eaac77d`, `fd7d977`, `6d94ba3`, `8e826e9` y `d0cd4a7`, junto con `test_route_executor.py` de `ROS2_SALUS`.

- Las rutas abiertas omiten solo el prefijo ya alcanzado y nunca retroceden.
- Los loops se incorporan/rotan conservando los índices originales y sus metadatos.
- Los chunks son finitos, no se solapan y un loop nunca se entrega completo repetidamente a Nav2.
- Los puntos originales son checkpoints. Los puntos expandidos son geometría
  entregada dentro del chunk multi-pose, además de servir para anclaje,
  progreso y diagnóstico: no producen éxito, freno ni acciones.
- Cada chunk termina en el próximo checkpoint original. Los límites de cantidad
  y distancia son suaves dentro de esa pierna: si se cruzan entre puntos
  sintéticos, se conserva la geometría hasta el checkpoint. El chunk finito se
  entrega como `NavigateThroughPoses`; sólo un chunk de una pose usa
  `NavigateToPose`.
- Antes de cada dispatch se avanza sólo sobre el tramo sintético inicial ya
  sobrepasado. Los checkpoints originales nunca se podan: cada uno es una
  frontera observable para `ROUTE_CHECKPOINT_REACHED`, incluida la salida de
  loop que Patrol usa para pasar de `EXIT_LOOP` a `RETURN_HOME`.
- Un yaw automático describe la pierna siguiente de la misión. Durante un
  dispatch finito sólo el terminal automático se adapta al rumbo de llegada;
  esto evita exigir geometría fuera de la ventana. Los yaws explícitos nunca se
  sustituyen.
- No hay freno entre objetivos contiguos; sí al finalizar, cancelar o abortar.
- Esta migración convierte LL una vez para validar/preparar la misión y conserva las poses `map` para diagnóstico. Cada despacho usa el contrato legacy `SetNavGoalLL`, cuyo servidor mantiene su conversión defensiva.

Fuera de alcance: reintentos, acciones, perfiles, patrulla, HOME y batería.

Los defaults caracterizados para esta frontera son `leg_spacing_m=35.0`,
`chunk_span_m=120.0` y `chunk_max_waypoints=5`. Las acciones programadas son
fronteras duras; los límites de span/cantidad son suaves hasta el checkpoint.

Esta separación recupera la intención visible en `fd7d977`, `eaac77d`,
`6d94ba3`, `e190157` y `4a1a2b4`: la densificación describe el recorrido que
Nav2 debe seguir, pero no redefine los hitos de la misión.

## Decisiones del corte #244

- `SetNavGoalLL` conserva su contrato público escalar/array. Arrays finitos se
  traducen a `NavigateThroughPoses`; `loop=true` sigue rechazado porque el loop
  pertenece al executor y nunca se entrega completo a Nav2.
- El BT multi-pose reutiliza `PathHealth` para conservar un path vigente,
  validar el candidato y detenerse ante TF/costmap stale. Recupera poda de
  goals y clears local/global, sin `Spin`, `BackUp`, smoother ni waypoint
  follower.
- El progreso se calcula proyectando sobre segmentos del chunk activo. Esto
  evita reportar como error transversal la distancia al vértice más cercano.
- El smoke de rutas registra el request lógico del chunk, el pose del robot y
  el `/plan`; sus métricas deterministas incluyen longitud, ratio de desvío,
  máxima distancia a la polilínea solicitada y auto-intersecciones. La
  comparación de yaw mantiene separadas la política actual, la preparada sin
  mutación y la terminal entrante; el yaw explícito siempre prevalece. En tres
  ejecuciones del escenario determinista `wide_turn_two_pose_window`, usando
  `ComputePathThroughPoses` y el path devuelto por Nav2, se obtuvo la misma
  conclusión:

  | política | yaw solicitado (°) | longitud (m) | detour | max deviation (m) | auto-intersecciones |
  | --- | ---: | ---: | ---: | ---: | ---: |
  | actual (approach + terminal) | 90, 0 | 61.298 | 6.130 | 8.07 | 1 |
  | preparada estilo legacy | 0, 90 | 62.441 | 6.244 | 10.83 | 2 |
  | sólo terminal entrante | 0, 0 | 39.605 | 3.961 | 8.07 | 0 |

  La variación entre ejecuciones fue menor que 0.03 m en `max deviation` y no
  cambió la clasificación. Se conserva `terminal_incoming`: elimina las
  auto-intersecciones y minimiza longitud/detour sin mutar el primer yaw; un
  yaw explícito continúa siendo intocable. La evidencia queda reproducible con
  `./tools/smoke_route_executor_sim.sh`, que escribe
  `evidence.yaw_policy_comparison` en `route_probe.json`.
- No se incorporan parámetros legacy omitidos ni tuning: requieren evidencia
  independiente y no son necesarios para restaurar el contrato multi-pose.

No validado en hardware en este corte: continuidad física, curva Ackermann y
loop. Esas pruebas sólo se habilitan después de PC/CI y simulación verdes.

## Corrección física posterior a #244

La primera implementación de paridad agrupó hasta cinco checkpoints originales
en un único `NavigateThroughPoses`. Una ruta física corta demostró que eso no
reproducía `build_chunk_waypoints(..., key_stop_indices=...)` del legacy:
Smac/Dubins intentaba satisfacer varios yaws cercanos dentro de un solo plan y
producía bucles de gran radio. El contrato corregido corta en el siguiente
checkpoint y conserva únicamente los sintéticos de esa pierna.

Los paths diagnósticos de misión y chunk son estado, no sensores. Se publican y
consumen con `TRANSIENT_LOCAL`, se proyectan usando el TF actual y permanecen
en el snapshot hasta que el executor publica explícitamente un path vacío. Esto
evita que Nav Live pierda la ruta cuando Nav2 limpia o reemplaza `/plan`.

## Diagnóstico `patrol_battery`

El artifact rojo de CI `34511311407` (`patrol-battery-return-20260910T180024-1`)
terminó esperando `PATROL`/`EXIT_LOOP` con navegación activa: el comando final
era AUTO y positivo, `collision_stop_active=false`, `failure_code` vacío y el
historial de `PathHealth` era `path_healthy`. El artifact rojo local posterior
(`patrol-battery-return-20260910T193206-1`) avanzó hasta `EXIT_LOOP`, completó
dos chunks y volvió a mostrar `collision_stop_active=false`, `failure_code`
vacío y `path_healthy`; sólo registró dos episodios de `progress_stalled` antes
de agotar la espera del tercer chunk. Por tanto no hay evidencia de que
Collision Monitor haya detenido el vehículo simulado ni de un error de
dispatch/authority.

El mismo smoke ya había documentado 5 pasadas y 1 fallo intermitente antes de
este corte. Después del cambio se obtuvieron tres pasadas consecutivas:
`patrol-battery-return-20260910T205001-1`, `205134-1` y `205307-1`, todas con
la secuencia `JOIN_LOOP -> PATROL -> EXIT_LOOP -> RETURN_HOME -> AT_HOME`.
La evidencia disponible clasifica el fallo como un stall transitorio de
progresión del simulador en el escenario de patrulla, no como una regresión
causal del PR ni como un stop de Collision Monitor. No se aumentaron timeouts,
se agregaron retries ni se modificó Collision Monitor.
