# Intención: recuperación de rutas bloqueadas

Fuentes históricas: `e190157`, `23d56b7`, `3457b65`, `4a1a2b4` y los tests
de bloqueo/reanclaje de `route_executor.py` en `ROS2_SALUS`.

## Invariantes preservados

- Un bloqueo transitorio no consume intentos ni reinicia el tramo.
- TF o costmap vencidos producen espera segura; no justifican limpiar mapas.
- Solo la máquina de misión posee los reintentos. El BT protege el path activo.
- Antes de reintentar se cancelan movimiento y goal, se limpian ambos costmaps
  y se reancla hacia delante desde la pose actual.
- Una ruta abierta nunca retrocede. Un loop no hace wrap dentro de la vuelta
  activa; el cierre lo cruza el avance normal del ejecutor.
- Los baselines heredados son tres intentos, cinco segundos de espera y ocho
  metros de tolerancia de reanclaje.
- Al agotar intentos se expone `NEEDS_OPERATOR`; no hay reinicio silencioso.

## Diseño migrado

`BlockedRecoveryPolicy` y `resolve_forward_reanchor` son lógica pura. Reciben
observaciones y devuelven decisiones tipadas sin conocer ROS, parámetros,
relojes ni servicios. `route_executor_node` adapta `PathHealth`, telemetría y
servicios Nav2; publica estados y eventos, pero no redefine la política.

Los estados son `CLEAR`, `PENDING`, `WAITING_DATA`, `WAITING_RETRY`,
`RECOVERING` y `NEEDS_OPERATOR`. Los campos `blocked_*` del contrato existente
reflejan directamente ese estado, sin añadir una API pública.
