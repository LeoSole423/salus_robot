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
| Control | `CmdVelFinal`, `DriveTelemetry` | migrados con campos originales; QoS actual preservado |
| Batería | `BatteryMissionGuard`, `BatteryState` | migrado; se mantiene separación guardia/SOC |
| Navegación | `NavTelemetry`, `NavEvent`, freno, modo manual, estado, goals LL, cancelación y zonas GeoJSON | arbitraje, goal único Nav2, keepout dinámico y política estable de path migrados; multi-waypoint sigue pendiente |
| Salud de path (interna) | `PathHealth` | contrato de diagnóstico entre evaluador y BT; conserva, recalcula o detiene la navegación automática sin exponer una API web |
| Ruta/patrulla | ruta, HOME, retorno, acciones | reemplazar JSON libre por contrato versionado |
| Perfiles | `SetNavigationProfile` | conservar `urban`/`rural` o generalizar |
| Zonas | GeoJSON y zonas tipadas | elegir una representación canónica |
| Observabilidad | telemetría, eventos, snapshots | telemetría/eventos básicos y observador Nav2 migrados; snapshots pendientes |
| Cámara | PTZ, presets y estado | separar control de transporte WebRTC |
| Simulación | inyección de batería | mantener fuera de la API operativa real |
| Datum | set/get dinámico | clasificado legacy por defecto |

## Regla de incorporación

Un contrato solo entra en `salus_interfaces` cuando tiene propietario, clientes,
unidad/semántica, QoS o timeout, errores esperados, test y decisión de
compatibilidad. Hasta entonces permanece únicamente en este inventario.
