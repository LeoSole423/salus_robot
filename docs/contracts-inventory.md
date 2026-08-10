# Inventario De Contratos Pendientes

Estado: referencia para migración; no son APIs de este repositorio todavía.

## Invariantes funcionales

- TF: `map -> odom -> base_footprint`.
- Sensores normalizados: IMU, GNSS, dirección, velocidad y LiDAR.
- Comando seguro: automático/manual/seguridad con fuente y freno explícitos.
- Estado observable para goals, rutas, patrulla, HOME y batería.
- Paridad de contrato entre robot y simulación.

## Familias a evaluar

| Familia | Contratos anteriores | Decisión necesaria |
| --- | --- | --- |
| Control | `CmdVelFinal`, `DriveTelemetry` | conservar semántica; revisar nombres/QoS |
| Batería | `BatteryMissionGuard`, `BatteryState` | mantener separación guardia/SOC |
| Navegación | goals, cancelación, freno, estado | definir API mínima estable |
| Ruta/patrulla | ruta, HOME, retorno, acciones | reemplazar JSON libre por contrato versionado |
| Perfiles | `SetNavigationProfile` | conservar `urban`/`rural` o generalizar |
| Zonas | GeoJSON y zonas tipadas | elegir una representación canónica |
| Observabilidad | telemetría, eventos, snapshots | definir retención y severidades |
| Cámara | PTZ, presets y estado | separar control de transporte WebRTC |
| Simulación | inyección de batería | mantener fuera de la API operativa real |
| Datum | set/get dinámico | clasificado legacy por defecto |

## Regla de incorporación

Un contrato solo entra en `salus_interfaces` cuando tiene propietario, clientes,
unidad/semántica, QoS o timeout, errores esperados, test y decisión de
compatibilidad. Hasta entonces permanece únicamente en este inventario.

