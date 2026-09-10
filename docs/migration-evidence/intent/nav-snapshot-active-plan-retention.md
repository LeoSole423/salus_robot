# Intención: retención visual del plan Nav2 activo

## Hechos

- Nav2 publica `/plan` al planificar o recalcular; no es un stream periódico.
- El snapshot actual trataba `/plan` como una capa dinámica de 2 s. Durante
  una navegación sana, o mientras Nav2 prepara un replan, Q podía perder la
  ruta verde aunque el goal seguía activo.
- `ROS2_SALUS` retenía el último plan, pero sin distinguir una acción Nav2
  activa de un resultado terminal.

## Decisión

- Se mantiene la frescura estricta de TF, costmap local, scan y polígonos de
  seguridad de ADR 0004.
- El adaptador de snapshot retiene el último `/plan` sólo mientras
  `/nav_command_server/telemetry` sea fresco y reporte `goal_active=true`.
- Un plan fresco reemplaza el retenido. Un resultado terminal (`goal_active`
  falso) descarta el retenido; no se representa como un plan activo.
- Las rutas `mission_path` y `active_chunk_path` continúan siendo geometría de
  intención retenida y visualmente distinta, no autoridad de movimiento.

## Límites

- Este cambio es exclusivamente diagnóstico; no publica comandos, TF ni
  modifica Nav2, `path_health`, Collision Monitor o readiness.
- Sigue pendiente confirmar en robot que Q mantiene el plan verde durante una
  ejecución larga y lo retira al abortar/cancelar el goal.
