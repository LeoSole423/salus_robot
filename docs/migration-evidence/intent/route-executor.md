# Intención: ejecutor de rutas

Fuente histórica: `fb54b95`, `eaac77d`, `fd7d977`, `6d94ba3`, `8e826e9` y `d0cd4a7`, junto con `test_route_executor.py` de `ROS2_SALUS`.

- Las rutas abiertas omiten solo el prefijo ya alcanzado y nunca retroceden.
- Los loops se incorporan/rotan conservando los índices originales y sus metadatos.
- Los chunks son finitos, no se solapan y un loop nunca se entrega completo repetidamente a Nav2.
- Los puntos originales son checkpoints. Los puntos expandidos son geometría
  entregada dentro del chunk multi-pose, además de servir para anclaje,
  progreso y diagnóstico: no producen éxito, freno ni acciones.
- Los límites de cantidad y distancia de un chunk son suaves: después de
  cruzarlos se continúa hasta el próximo checkpoint original. El chunk finito
  completo se entrega como `NavigateThroughPoses`; sólo un chunk de una pose
  usa `NavigateToPose`.
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
- No se incorporan parámetros legacy omitidos ni tuning: requieren evidencia
  independiente y no son necesarios para restaurar el contrato multi-pose.

No validado en hardware en este corte: continuidad física, curva Ackermann y
loop. Esas pruebas sólo se habilitan después de PC/CI y simulación verdes.
