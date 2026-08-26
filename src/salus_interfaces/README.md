# salus_interfaces

- Responsabilidad: contratos ROS compartidos.
- No contiene: nodos, launches, lógica ni contratos legacy automáticos.
- Interfaces actuales: control/batería, arbitraje y navegación punto a punto:
  `MeasurementMetadata`, `TractionMeasurement`, `SteeringMeasurement`,
  `NavTelemetry`, `NavEvent`, `BrakeNav`, `SetManualMode`, `GetNavState`,
  `SetNavGoalLL`, `CancelNavGoal`, `SetRouteMissionLL`, `CancelRouteMission`,
  `GetRouteMissionState`, `SetPatrolMissionLL`, `CancelPatrolMission`,
  `GetPatrolMissionState`, `RequestReturnHome`, `SetNavigationProfile`,
  `SetZonesGeoJson`, `GetZonesState` y
  `PathHealth`, `NavSnapshotLayers`, `GetNavSnapshot`, `CameraPan`,
  `CameraStatus`, `CameraPtz`, `CameraPreset`, `CameraSavePreset` y
  `CameraPtzState` (más el `Trigger` estándar de zoom).
- `PathHealth` es un contrato interno de runtime: explica si Nav2 conserva,
  recalcula o detiene temporalmente un path; no es una API de Cockpit.
- `EvaluatePathHealth` usa explícitamente el contexto `ACTIVE` o `CANDIDATE`;
  evita inferir la intención a partir del orden de llamadas del BT.
- `GetNavSnapshot` conserva la petición vacía y la respuesta PNG del contrato
  legacy. `NavSnapshotLayers` indica qué información se dibujó realmente; no
  solicita capas ni representa su mera recepción.
- Estado: los contratos listados conservan nombres, campos y constantes del
  sistema anterior bajo el namespace nuevo `salus_interfaces`.
- Las tres interfaces de medición son el primer contrato deliberadamente
  nuevo: separan hechos físicos, conversiones, odometría y localización según
  ADR 0008. Incorporarlas no activa adaptadores ni actuadores.
- Prueba: `colcon test --packages-select salus_interfaces`.
- Migración: el resto de los contratos continúa pendiente y se incorporará por
  corte funcional.
