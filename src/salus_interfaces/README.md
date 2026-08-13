# salus_interfaces

- Responsabilidad: contratos ROS compartidos.
- No contiene: nodos, launches, lógica ni contratos legacy automáticos.
- Interfaces actuales: control/batería, arbitraje y navegación punto a punto:
  `NavTelemetry`, `NavEvent`, `BrakeNav`, `SetManualMode`, `GetNavState`,
  `SetNavGoalLL`, `CancelNavGoal`, `SetRouteMissionLL`, `CancelRouteMission`,
  `GetRouteMissionState`, `SetZonesGeoJson`, `GetZonesState` y
  `PathHealth`.
- `PathHealth` es un contrato interno de runtime: explica si Nav2 conserva,
  recalcula o detiene temporalmente un path; no es una API de Cockpit.
- `EvaluatePathHealth` usa explícitamente el contexto `ACTIVE` o `CANDIDATE`;
  evita inferir la intención a partir del orden de llamadas del BT.
- Estado: los contratos listados conservan nombres, campos y constantes del
  sistema anterior bajo el namespace nuevo `salus_interfaces`.
- Prueba: `colcon test --packages-select salus_interfaces`.
- Migración: el resto de los contratos continúa pendiente y se incorporará por
  corte funcional.
