# Mapa De Migración

Estado: inventario inicial; ninguna unidad está migrada.

## Política

Migrar comportamiento probado y contratos vigentes, no carpetas. Cada unidad
debe pasar por: inventario, contrato, tests de caracterización, implementación,
paridad y retiro del camino anterior.

| Origen en ROS2_SALUS | Destino | Estado inicial | Nota |
| --- | --- | --- | --- |
| `interfaces` | `salus_interfaces` | pendiente | revisar contrato por contrato |
| URDF/RViz de `navegacion_gps` | `salus_description` | pendiente | consolidar un modelo canónico |
| MAVROS, RTK, RS16, cámara | `salus_hardware` | pendiente | wrappers; no vendorizar SDK |
| odometría, EKF, GPS heading | `salus_localization` | pendiente | preservar TF y datum fijo |
| conversión/filtros LiDAR | `salus_perception` | pendiente | separar vigente/experimental |
| `controller_server` y batería | `salus_control` | pendiente | conservar backend real/sim |
| Nav2, command server, rutas, zonas | `salus_navigation` | pendiente | separar misión de arbitraje |
| plugins en `navegacion_gps_bt` | `salus_navigation_bt` | pendiente | caracterizar ABI/BT XML |
| `map_tools/web_zone_server` | `salus_web` | pendiente | no duplicar nodos de navegación |
| Gazebo, mundos, normalizadores | `salus_simulation` | pendiente | paridad con contratos reales |
| launches globales V2 | `salus_bringup` | pendiente | reducir a perfiles finales |

## No migrar de entrada

- `pixhawk_driver` propio reemplazado por MAVROS;
- `real.launch.py`, `simulacion.launch.py` y wrappers equivalentes;
- flujo de datum dinámico;
- rutas LiDAR experimentales no validadas;
- archivos generados, cachés y configuración local;
- dependencias vendorizadas sin evaluar distribución/versión.

## Orden propuesto

1. Contratos e interfaces.
2. Control y telemetría con backend simulado.
3. Descripción y hardware normalizado.
4. Localización y percepción.
5. Navegación/misiones y plugins BT.
6. Simulación integrada y operación web.
7. Bringups reales, pruebas en banco y robot.

