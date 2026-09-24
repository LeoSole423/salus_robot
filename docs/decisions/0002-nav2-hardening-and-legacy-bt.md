# ADR 0002: Path estable y despeje observable en Nav2

- Estado: aceptada
- Fecha: 2026-08-13

## Contexto

El stack legacy contenía `IsPathClearanceValid`, que evitaba reemplazar una
ruta segura por pequeñas desviaciones y pedía replanning si el despeje se
degradaba de forma sostenida. También contenía `TraceReplan`, un trazador
alrededor de `NavigateThroughPoses`.

Replanificar a una frecuencia fija hace difícil depurar Nav2 y puede convertir
ruido de pose, inflación aislada o oscilaciones del costmap en cambios de ruta
sin valor operativo. A la vez, `collision_monitor` sólo protege el movimiento
inmediato: no reemplaza una comprobación anticipada del path global.

## Decisión

- Migrar la capacidad útil como `PathHealth`: una política Python pura en
  `salus_navigation` y un plugin BT C++ delgado en `salus_navigation_bt`.
- Conservar el path activo mientras sea seguro y alcanzable. Solicitar replan
  únicamente por cambio de goal, colisión, inflación sostenida, desviación
  transversal persistente, falta de progreso o fallo de datos.
- Ante costmap o TF no disponibles/vencidos, publicar `STOP_AND_WAIT`, detener
  sólo el comando automático y conservar el path hasta recuperarse.
- Validar una ruta candidata antes de sustituir la ruta vigente. Si la
  candidata tampoco es válida, detenerse y reintentar sin adoptar una ruta
  insegura.
- Mantener la geometría, umbrales, histéresis y métricas fuera del BT. El
  plugin consulta `/path_health/evaluate` con contexto explícito `ACTIVE` o
  `CANDIDATE`; `PathHealth` expone la causa, edad, coste, muestras y error
  transversal.
- No migrar `TraceReplan`. `nav_observer` conserva eventos de replan, bloqueo,
  lifecycle y resultado sin poseer comandos. Se reabrirá sólo si rutas futuras
  con `NavigateThroughPoses` aportan una necesidad reproducible.

## Consecuencias

La navegación mantiene una única autoridad sobre `/cmd_vel_final`, una ruta
estable y evidencia explícita de cada cambio. `salus_navigation_bt` queda
portado para esta coordinación mínima; no incorpora la complejidad ni el ABI
legacy de `TraceReplan`.

## Enmienda #244: rutas multi-pose

La necesidad reproducible apareció durante la validación física de rutas. El
navigator `NavigateThroughPoses` se habilita reutilizando la misma política
`PathHealth`: conserva el path sano, poda goals superados, valida el candidato
antes de reemplazarlo y separa los clears local/global. `nav_observer` resultó
suficiente para observar replans, por lo que `TraceReplan` continúa fuera del
runtime. Tampoco se incorporan `Spin`, `BackUp`, smoother ni waypoint follower.

## Enmienda #303: ocupaciones lejanas transitorias

La caracterización de #303 mostró que una celda letal a 8 m provocaba
`REPLAN` inmediato en `PathHealth`. Además, `IsPathValid` de Nav2 Humble
recorría todo el path restante antes de `PathHealth` y adelantaba un replan por
una única marca lejana. Ambos BT delegan ahora la decisión del path activo en
`PathHealth`, manteniendo la validación estricta del candidato antes de
reemplazarlo.

La política distingue el horizonte cercano de 5,35 m del tramo de observación
hasta 12 m. Una ocupación cercana pide replan de inmediato. La lejana requiere
dos costmaps de stamps distintos, al menos 1,0 s y continuidad espacial dentro
de 1 m; una observación ausente o stale reinicia esa confirmación. Los valores
iniciales toman como referencias la zona de slowdown exterior de Collision
Monitor y la persistencia de recuperación de rutas. Tras observar en Gazebo
un replan demasiado tardío frente a un obstáculo persistente, se elevó la
consulta de ambos BT a 2 Hz, la publicación del costmap global a 1 Hz y se
redujo la confirmación lejana a 1,0 s. Son cotas de configuración; todavía
falta medir la latencia real de detección y planificación. No constituyen una
distancia de frenado física validada. RPP y Collision Monitor conservan sus
propios stops cercanos.

Una revisión nueva de keepout fuerza replan inmediato, sin aplicar la espera
de ocupación lejana. Esto conserva el tratamiento distinto de una zona
operativa explícita y un retorno del sensor dudoso. La validación física de
distancia, latencia y comportamiento de misión sigue pendiente.
