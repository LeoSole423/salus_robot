# Intención: recuperación de rutas bloqueadas

Fuentes históricas: `e190157`, `23d56b7`, `3457b65`, `4a1a2b4` y los tests
de bloqueo/reanclaje de `route_executor.py` en `ROS2_SALUS`.

## Invariantes preservados

- Un bloqueo transitorio no consume intentos ni reinicia el tramo.
- TF o costmap vencidos producen espera segura; no justifican limpiar mapas.
- Solo la máquina de misión posee los reintentos. El BT protege el path activo.
- Antes de reintentar se cancelan movimiento y goal y se limpian ambos costmaps.
- El request pendiente se recorta sólo tras checkpoints acreditados de forma
  consecutiva por `ROUTE_CHECKPOINT_REACHED`. La identidad incluye vuelta e
  índice original; el cierre del loop puede estar dentro del mismo chunk.
- Los baselines heredados son tres intentos y cinco segundos de espera. La
  tolerancia geométrica existente sólo permite completar el terminal.
- Al agotar intentos se expone `NEEDS_OPERATOR`; no hay reinicio silencioso.

## Diseño migrado

`BlockedRecoveryPolicy`, `resolve_forward_reanchor` y
`pending_checkpoint_suffix` son lógica pura. Reciben
observaciones y devuelven decisiones tipadas sin conocer ROS, parámetros,
relojes ni servicios. `route_executor_node` adapta `PathHealth`, telemetría y
servicios Nav2; publica estados y eventos, pero no redefine la política.

Los estados son `CLEAR`, `PENDING`, `WAITING_DATA`, `WAITING_RETRY`,
`RECOVERING` y `NEEDS_OPERATOR`. Los campos `blocked_*` del contrato existente
reflejan directamente ese estado, sin añadir una API pública.

## Corrección #302

Hecho observado en el runtime: un chunk `5(vuelta 0) → 0 → 1 → 2(vuelta 1)`
puede haber acreditado `5` antes del bloqueo. El reanclaje anterior no permitía
envolver el índice; reconstruía el chunk y volvía a despachar `5`. La autoridad
de progreso es el evento acreditado, no la cercanía de la pose a un waypoint.

La política recorta el chunk activo después del último checkpoint acreditado
consecutivamente. Conserva puntos sintéticos, yaws, acciones y el terminal
original. Una acreditación posterior sin las previas no permite saltar ninguna.
El nodo mantiene `mission_id` y `chunk_id`, reconstruye el tracker sólo con los
pendientes y preserva el registro de eventos ya acreditados. No cambia Nav2,
tolerancias, contratos públicos ni el disparador inicial del bloqueo (#303).
Validación en robot permanece pendiente.
