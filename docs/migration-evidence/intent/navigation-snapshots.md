# Intención: snapshots de navegación

Fuentes históricas: `0777a4c`, `2b88ef2`, `0338ea4`, `0f3305d`, `2e16a32` y
`f4f1e4e` de `ROS2_SALUS`.

## Problema que resolvían

- entregar a Cockpit una vista compacta del estado espacial de Nav2;
- combinar información local de seguridad con contexto global sin transmitir
  costmaps o nubes completas;
- conservar una respuesta tipada que permita saber qué capas sí llegaron a la
  imagen;
- evitar que fallos opcionales impidan observar el costmap local disponible.

## Intención vigente

- el centro de la vista es la pose real obtenida por TF, no el centro supuesto
  de la grilla;
- los paths se recortan contra la ventana, por lo que un tramo que cruza la
  imagen debe verse aunque sus extremos estén fuera;
- keepout se superpone tanto en la vista local como en el inset global;
- scan, polígonos y paths se transforman al frame del costmap local;
- el inset global conserva plan, keepout y posición del robot cuando esos datos
  están disponibles;
- los flags representan contenido dibujado;
- el cleanup `f4f1e4e` es la versión vigente: no migrar vehículo sintético,
  leyenda ni flechas decorativas.

## Arquitectura nueva

El adaptador ROS toma una copia consistente de la caché y resuelve las TF. El
ensamblador puro valida y transforma esa entrada a una escena 2D. El renderer
consume sólo la escena y parámetros explícitos. La codificación PNG es la
última operación y no conoce ROS.

WebSocket, base64, sesiones de misión y rosbag quedan fuera de este corte. El
bridge web futuro convertirá la respuesta ROS al protocolo de Cockpit.

## Fuentes y QoS fijados

| Entrada | Tipo | QoS | Semántica |
| --- | --- | --- | --- |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | reliable, transient-local, depth 1 | base obligatoria |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | reliable, transient-local, depth 1 | inset opcional |
| `/keepout_filter_mask` | `nav_msgs/OccupancyGrid` | reliable, transient-local, depth 1 | overlay estático opcional |
| `/local_costmap/published_footprint` | `geometry_msgs/PolygonStamped` | reliable, depth 10 | footprint opcional |
| `/stop_zone_raw` | `geometry_msgs/PolygonStamped` | reliable, depth 10 | zona reactiva opcional |
| `/collision_monitor/polygons` | `visualization_msgs/MarkerArray` | reliable, depth 10 | polígonos adicionales opcionales |
| `/scan_clean` | `sensor_msgs/LaserScan` | sensor-data best-effort | detecciones 2D opcionales |
| `/plan` | `nav_msgs/Path` | reliable, depth 10 | path activo opcional |

`/collision_monitor/polygons` se conserva configurable por compatibilidad. Si
la versión de Nav2 en uso no lo publica, la capa queda ausente sin crear otro
publisher ni duplicar las zonas individuales.

## Matriz de fallos

| Condición | Resultado |
| --- | --- |
| costmap local ausente, inválido o vencido | `ok=false`, sin PNG |
| TF base→frame local ausente | `ok=false`, sin PNG |
| capa opcional ausente, inválida, vencida o sin TF | se omite, flag `false` |
| keepout vacío | no altera píxeles, flag `false` |
| global costmap válido sin TF del robot | inset válido sin marcador del robot |
| fallo de codificación PNG | `ok=false`, sin PNG |
| render correcto por encima de 500 ms | `ok=true` más diagnóstico de latencia |

Los casos declarativos están en
`test/fixtures/navigation_snapshots/scenarios.json`. Terra debe convertirlos en
tests del ensamblador y renderer antes de conectar el nodo ROS.
