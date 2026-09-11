# Intención caracterizada: recuperación single-pose equivalente al legacy

## Alcance

Este corte recupera únicamente la recuperación de `NavigateToPose` que existe
en `ROS2_SALUS`. No cambia el árbol `NavigateThroughPoses`, planner,
controller, parámetros de Nav2, safety ni la recuperación de misión.

## Contrato

- `FollowPath` tiene un `RecoveryNode` local con un único reintento y espera de
  1 segundo.
- La recuperación exterior responde a `GoalUpdated` inmediatamente o espera
  2 segundos antes de recalcular.
- La ruta recalculada se escribe en `candidate_path`, se valida con
  `PathHealth` en contexto `CANDIDATE` y sólo entonces se copia a `path`.
- `KeepHealthyPath`, `STOP_AND_WAIT`, `AlwaysFailure` y la ausencia de
  `Spin`/`BackUp` permanecen intactos.

## Fallos y seguridad

Un fallo transitorio de controller usa la espera local; un fallo transitorio de
planner usa la recuperación exterior. Una candidata inválida nunca reemplaza
la ruta activa. Si no existe una ruta sana, el árbol no libera una ruta stale
al controller. Cancelación y fallo terminal siguen dependiendo de los límites
de autoridad existentes, sin introducir publishers ni comandos nuevos.

La validación estructural se complementa con el smoke Nav2 existente, que
comprueba la composición, la cadena de comandos y el brake seguro al cancelar.
