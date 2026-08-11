# Mapa De Migración

Estado: migración en curso; control y batería constituyen el primer corte.

El estado detallado y la evidencia verificable viven en
[`migration-status.yaml`](migration-status.yaml).

## Política

Migrar comportamiento probado y contratos vigentes, no carpetas. Cada unidad
debe pasar por: inventario, contrato, tests de caracterización, implementación,
paridad y retiro del camino anterior.

| Origen en ROS2_SALUS | Destino | Estado inicial | Nota |
| --- | --- | --- | --- |
| `interfaces` | `salus_interfaces` | parcial | cinco contratos de control/batería migrados |
| URDF/RViz de `navegacion_gps` | `salus_description` | portado parcial | Xacro canónico de movimiento; medidas sin validar en hardware |
| MAVROS, RTK, RS16, cámara | `salus_hardware` | pendiente | wrappers; no vendorizar SDK |
| odometría, EKF, GPS heading | `salus_localization` | pendiente | preservar TF y datum fijo |
| conversión/filtros LiDAR | `salus_perception` | portado parcial | cadena 3D local; falta validación con bag RS16 |
| `controller_server` y batería | `salus_control` | paridad sim | UART sin validar en hardware |
| Nav2, command server, rutas, zonas | `salus_navigation` | portado parcial | goal único Nav2 y arbitraje migrados; rutas y zonas siguen pendientes |
| plugins en `navegacion_gps_bt` | `salus_navigation_bt` | pendiente | caracterizar ABI/BT XML |
| `map_tools/web_zone_server` | `salus_web` | pendiente | no duplicar nodos de navegación |
| Gazebo, mundos, normalizadores | `salus_simulation` | portado parcial | mundo Ackermann con GPS/LiDAR reducidos; paridad A/B pendiente |
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
