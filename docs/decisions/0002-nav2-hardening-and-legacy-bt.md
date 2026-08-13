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
