# Inventario De Contratos Pendientes

Estado: referencia para migración; la familia control/batería ya tiene su primer
corte público en `salus_interfaces`.

## Invariantes funcionales

- TF: `map -> odom -> base_footprint`.
- Sensores normalizados: IMU, GNSS, dirección, velocidad y LiDAR.
- Comando seguro: automático/manual/seguridad con fuente y freno explícitos.
- Estado observable para goals, rutas, patrulla, HOME y batería.
- Paridad de contrato entre robot y simulación.

## Familias a evaluar

| Familia | Contratos anteriores | Decisión necesaria |
| --- | --- | --- |
| Control | `CmdVelFinal`, `DriveTelemetry`, `VehicleCommand`, `DiagnosticArray` | contratos legacy preservados; `VehicleCommand` se publica sólo como observación shadow y su comparación emite diagnóstico, sin consumidor ni autoridad |
| Mediciones de vehículo | `MeasurementMetadata`, `TractionMeasurement`, `SteeringMeasurement` | contrato, adaptador legacy, conversión calibrada y ejecutable de odometría canónica incorporados; quedan selección por perfiles, fuentes directas y validación física según ADR 0008 |
| Batería | `BatteryMissionGuard`, `BatteryState` | migrado; se mantiene separación guardia/SOC |
| Navegación | `NavTelemetry`, `NavEvent`, freno, modo manual, estado, goals LL, cancelación y zonas GeoJSON | arbitraje, goal único Nav2, keepout dinámico, path estable, rutas multi-waypoint y recuperación controlada migrados |
| Salud de path (interna) | `PathHealth` | contrato de diagnóstico entre evaluador y BT; conserva, recalcula o detiene la navegación automática sin exponer una API web |
| Ruta/patrulla | `SetRouteMissionLL`, `SetPatrolMissionLL`, cancelación, estado y `RequestReturnHome` | rutas, acciones y patrulla/HOME migradas; el retorno manual o por batería delega tramos a `route_executor` y conserva un latch de misión |
| Perfiles | `SetNavigationProfile` | `urban`/`rural` migrados mediante aplicación transaccional |
| Zonas | GeoJSON y zonas tipadas | elegir una representación canónica |
| Observabilidad | telemetría, eventos, `NavSnapshotLayers`, `GetNavSnapshot` | contratos de snapshot y semántica fijados por ADR 0004; servidor y renderer pendientes |
| Cámara | `CameraPan`, `CameraStatus`, `CameraPtz`, `CameraPreset`, `CameraSavePreset`, `CameraPtzState` y zoom `Trigger` | contrato caracterizado; control PTZ separado de MediaMTX/WebRTC, implementación pendiente |
| Simulación | inyección de batería | mantener fuera de la API operativa real |
| Datum | set/get dinámico | clasificado legacy por defecto |

## Regla de incorporación

Un contrato solo entra en `salus_interfaces` cuando tiene propietario, clientes,
unidad/semántica, QoS o timeout, errores esperados, test y decisión de
compatibilidad. Hasta entonces permanece únicamente en este inventario.
