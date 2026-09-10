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
- Un yaw automático describe la pierna siguiente de la misión. Cuando un
  checkpoint automático pasa a ser el terminal de un chunk finito, el request
  a Nav2 usa en esa pose el rumbo de llegada desde el punto anterior. El estado
  y la misión preparada conservan su rumbo saliente; un yaw explícito del
  operador nunca se sustituye.
- Cuando un chunk multi-pose comienza en un checkpoint automático y la pose
  actual del robot es conocida, la primera pose del request usa el rumbo de
  aproximación desde el robot. Esto evita exigir en ese checkpoint un rumbo de
  salida casi opuesto que Smac/Dubins resolvería con un rulo de radio mínimo.
  No modifica un yaw explícito, el terminal ni los checkpoints de misión.
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
