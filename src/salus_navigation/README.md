# salus_navigation

Responsabilidad: navegación segura, Nav2 y zonas no-go. El corte actual ofrece
un goal LL único y zonas dinámicas GeoJSON en simulación.

- API de zonas: `/zones_manager/set_geojson`, `/zones_manager/get_state` y
  `/zones_manager/reload_from_disk`.
- Las zonas se convierten con `/fromLL`, recargan `/keepout_filter_mask` y
  limpian el costmap global. Los datos operativos persisten en `runtime/zones/`
  y no se versionan.
- `use_keepout:=false` publica una máscara vacía; rutas, patrulla, HOME y la
  operación web siguen fuera de este paquete migrado.
- `nav_observer` publica eventos de lifecycle, bloqueo local y replanning sin
  modificar Nav2 ni poseer comandos. La decisión sobre los plugins BT legacy
  está registrada en [ADR 0002](../../docs/decisions/0002-nav2-hardening-and-legacy-bt.md).

Pruebas: `colcon test --packages-select salus_navigation` y
`./tools/smoke_navigation_zones_sim.sh` y `./tools/smoke_navigation_core_sim.sh`.
