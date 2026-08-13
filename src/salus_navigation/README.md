# salus_navigation

Responsabilidad: navegación segura, Nav2 y zonas no-go. El corte actual ofrece
un goal LL único, rutas abiertas/circulares y zonas dinámicas GeoJSON en
simulación.

- API de zonas: `/zones_manager/set_geojson`, `/zones_manager/get_state` y
  `/zones_manager/reload_from_disk`.
- Las zonas se convierten con `/fromLL`, recargan `/keepout_filter_mask` y
  limpian el costmap global. Los datos operativos persisten en `runtime/zones/`
  y no se versionan.
- API de rutas: `/route_executor/set_route_mission_ll`,
  `/route_executor/cancel_route_mission` y `/route_executor/get_route_mission_state`.
  La preparación LL es asíncrona y atómica; el ejecutor no publica velocidad
  ni invoca Nav2 directamente. Recuperación, acciones, perfiles, patrulla y
  HOME siguen fuera de este corte.
- `path_health` conserva el plan mientras siga sano y evalúa hasta 12 m por
  delante con footprint orientado, colisión, inflación sostenida, progreso y
  desviación transversal. Evalúa la pose desde TF en el frame del path y usa
  `EvaluatePathHealth` con contexto explícito; ante costmap o TF vencidos
  produce `STOP_AND_WAIT`.
- `nav_observer` publica eventos de lifecycle, bloqueo local y replanning sin
  modificar Nav2 ni poseer comandos. La decisión sobre el plugin BT delgado y
  `TraceReplan` está registrada en [ADR 0002](../../docs/decisions/0002-nav2-hardening-and-legacy-bt.md).

Pruebas: `colcon test --packages-select salus_navigation` y
`./tools/smoke_navigation_zones_sim.sh`, `./tools/smoke_navigation_core_sim.sh`
y `./tools/smoke_route_executor_sim.sh`.
