# Estado de revisión después de retomar — 17/10/2026

El operador retomó y autorizó completar PC/simulación y preparar revisión.
No hay despliegue ni movimiento físico autorizados.

## Ubicación y ramas

- Backend: worktree salus_robot-maneuver-guard, rama agent/maneuver-guard,
  base c6bed23. Implementación 2c488f5; instrumentación 2dfc0c6; el commit
  de checkpoint que contiene este archivo conserva los últimos ajustes.
- Cockpit: worktree cockpit-maneuver-guard-status, rama
  agent/maneuver-guard-status, commit acc8585, base migration/salus-robot-cockpit
  en 383f3ce. PR draft: https://github.com/AEyeSecurity/cockpit/pull/26.
- Checkouts canónicos y .agents/ preservados. El worktree temporal de baseline
  se eliminó después de copiar evidencia a artifacts/baseline-c6bed23-patrol.

## Completado

Política pura de U/rulos y ventana monotónica, admisión activa/candidata,
stop del follower anterior, bloqueo terminal conservando misión y contratos,
proyección de causa y presentación/polling en Cockpit. ADR 0016, intención,
mapa de paquetes y estado de migración actualizados.

Build Humble: 14 paquetes. Suite completa: 1341 tests, cero errores/fallos,
cuatro omitidos; después del último ajuste de metadata de la respuesta se
repitió salus_navigation (352 aprobados) y el agregado conserva 1341 verdes.
Cockpit: build y 172 tests aprobados, uno omitido.

Smokes aprobados: fault injection ROS sin backend de movimiento, navegación
core, safety, ruta abierta, circuito intencional con route_tangent, perfiles,
Web/Cockpit e integración estructural. Matriz R8: 2/2. Matriz recta/R4: 2/3;
la derecha produjo un rulo real de 31.6168 m y fue bloqueada correctamente
(185 comandos finales todos en cero). Se conserva el fallo funcional del
evaluador y el fixture del path, sin relajar gates.

Patrol/HOME falla antes del despacho por anchor no_near_segment, reproducido
en una compilación limpia de c6bed23 sin este cambio. Evidencia y límites en
../maneuver-guard-2026-10-17.md (relativo al directorio superior de evidencia).
No se atribuye al guard ni se declara ese smoke aprobado.

## Revisión publicada

- Backend: PR draft https://github.com/LeoSole423/salus_robot/pull/329.
- Cockpit: PR draft https://github.com/AEyeSecurity/cockpit/pull/26.
- Ambos PRs están enlazados y adjuntos al chat. Revisión final de contratos,
  fixture y diff completada. El primer CI backend pasó todos sus jobs.
- En el cierre se corrigieron los timers de admisión/recuperación para usar
  STEADY_TIME y se agregó fault injection con /clock inmóvil en cero: PASS.
  La actualización debe conservar CI verde antes de merge.
- Queda revisión humana y decisión de integración de ambos PRs. No hacer
  merge automático ni promoción hardware; mantener explícitos el fallo
  preexistente Patrol/HOME y el rechazo intencional R4.

Los contenedores ajenos/legacy no se tocaron. No se programó seguimiento.
