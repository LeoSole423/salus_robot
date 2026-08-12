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

Pruebas: `colcon test --packages-select salus_navigation` y
`./tools/smoke_navigation_zones_sim.sh`.
