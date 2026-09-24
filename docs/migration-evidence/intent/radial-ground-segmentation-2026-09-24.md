# Intención: recuperar la segmentación radial de suelo del robot real

## Alcance

- Fuente histórica: `ROS2_SALUS/src/navegacion_gps/navegacion_gps/scan_ground_filter.py`, commits `868b2fc` y `b3cdc58`, y sus tests.
- Destino: política pura en `salus_perception` y adaptador ROS existente `scan_ground_filter`.
- Incluido: clasificación de nube 3D, perfiles urban/rural, replay del RS16 real y diagnóstico de latencia.
- Fuera de alcance: driver RS16, TF owner, `pointcloud_to_laserscan`, costmaps, Collision Monitor y cambios de seguridad por tuning.

## Evidencia histórica y contradicción

| Fuente | Hecho observado | Confianza |
| --- | --- | --- |
| Legacy `868b2fc` | Introdujo segmentación radial tipo Autoware, con agrupación por azimut, orden radial y comparación de pendiente local/global. | Alta |
| Legacy `b3cdc58` | Añadió perfiles urban/rural y tests de pendientes y obstáculos bajos. | Alta |
| Legacy `real_global_v2.launch.py` y `real_global_v2_wifi.launch.py` | `enable_scan_ground_filter=True` y `enable_lidar_obstacle_filter=False` por defecto; `/scan_3d/no_ground` alimentaba la proyección y `/scan_clean` a Nav2. | Alta para configuración por defecto; no demuestra parámetros efectivos de toda salida histórica |
| Nuevo `0868c62` y #184 | El filtro inicial de simulación usa una cota absoluta de `z` y luego se conectó al perfil real con tests sintéticos, sin comparación física RS16. | Alta |
| Bag real de 24/09/2026 | Captura de 15 min, 2,8 GB, con nube raw/filtrada, scans, TF, IMU y costmaps. | Alta para señales registradas; sin etiquetas de verdad terreno/obstáculo |

El `split_height_distance=0.20 m` legado mide discontinuidad entre puntos. El `ground_tolerance_m=0.20 m` nuevo elimina todos los puntos con `z <= 0.20 m` dentro de 20 m. Son políticas distintas pese al nombre del nodo. La migración dejó una brecha funcional respecto de la navegación real por defecto.

## Contrato e invariantes

- Entrada: `/scan_3d`, `sensor_msgs/PointCloud2`, `lidar_link`, QoS sensor data best effort, ~10 Hz observado.
- Salida: `/obstacles_cloud`, `sensor_msgs/PointCloud2`, `base_footprint`, mismo `header.stamp`, QoS sensor data best effort. El consumidor proyecta a `/scan`; `/scan_clean` alimenta Nav2 y Collision Monitor.
- TF: `base_footprint <- lidar_link` estático observado con traslación `(0.92, 0, 0.65) m` y pitch ~10°. No crear otro publicador TF.
- Ausencia de TF o nube vencida: no fabricar salida despejada. Mantener el contrato de stop/freshness existente.
- El segmento debe conservar obstáculos reales bajos, incluso cuando una política de suelo más permisiva los podría retirar.

## Primer contraste offline del bag

Se extrajeron 12 frames alrededor de 2026-09-24 23:08:56 UTC (la ventana en que el operador vio franjas radiales). Cada frame raw contiene ~10.000 puntos. La política actual predice, dentro de 20 m, entre 1.374 y 1.599 puntos retenidos; el segmentador legacy sin adaptador ROS retiene entre 1.648 y 1.754. Por frame hay 563–665 puntos retenidos sólo por legacy y 256–545 retenidos sólo por la política actual. La ejecución pura legacy en contenedor x86 tuvo mediana ~6,6 ms/frame; **no** mide Jetson ni costo ROS/TF/publicación.

El contraste prueba desacuerdo sustancial, no que el filtro legacy sea mejor en esas franjas. No existe etiqueta de verdad para suelo, vegetación u obstáculo en el bag. El recorte de 20 m se aplicó antes del segmentador legacy; el filtro actual conserva puntos fuera de rango que no considera suelo, una diferencia adicional que debe medirse por separado. La comparación usó el TF estático observado; pequeñas diferencias de redondeo frente al output real requieren verificación exacta antes de usar métricas por punto como gate.

## Diseño propuesto

1. Congelar un fixture pequeño de frames raw sincronizados con TF, nube filtrada, scans y costmaps; registrar provenance del bag y ventana visual.
2. Especificar una política pura de clasificación radial a partir de la intención y tests legacy, sin copiar el archivo histórico. Configuración tipada: ángulo radial, pendientes local/global, continuidad, salto de altura, punto virtual y rango.
3. Separar clasificación de transformación y serialización. El adaptador ROS conserva topics, QoS, stamps, frame, ausencia de TF y el comportamiento de salida ante nube vacía.
4. Añadir diagnóstico de conteos por sector/rango/altura, latencia, backlog, pérdida de frames y diferencias con la política actual. Evaluar el bag completo o una selección estratificada de terreno plano, pendiente, franjas y obstáculo real.
5. Probar dos casos control: pendiente suave sin obstáculo y obstáculo bajo real al mismo rango/sector. Las etiquetas del bag visual no reemplazan esos controles.
6. Medir CPU y tasa en el contenedor Humble de PC, luego prueba de banco en Jetson sin conectar salida a `/scan_clean` productivo. Conservar la política actual como fallback explícito hasta validar seguridad y throughput.

## Fallos y degradación

| Condición | Respuesta exigida |
| --- | --- |
| TF no disponible o nube stale | No publicar scan despejado fabricado; mantener la detección de freshness/stop existente. |
| Clasificación ambigua cerca de un obstáculo bajo | Mantenerlo como obstáculo hasta demostrar que es suelo con controles independientes. |
| Tiempo de filtrado mayor que periodo de nube | No desplegar en navegación real; medir backlog y tasa efectiva antes de optimizar. |
| Perfil dinámico inválido | Rechazo atómico, sin aplicar una mezcla de parámetros. |

## Decisiones pendientes

- Si preservar todos los campos de PointCloud2 (legacy conserva `intensity`, el filtro nuevo publica XYZ) forma parte del contrato requerido por consumidores futuros.
- Umbrales urban/rural definitivos: los valores legacy son baseline experimental, no validación automática en este terreno.
- Si hace falta un ADR por cambio de política productiva o por cualquier reutilización textual del algoritmo histórico. La implementación preferida es clean-room, basada en comportamiento caracterizado.

## Pruebas y aceptación

- Caracterización previa: suelo plano, pendiente sostenida, borde bajo, poste, rayos desordenados, nube vacía, rango y perfiles.
- Replay: comparar ambos filtros con timestamps iguales; cuantificar franjas dudosas por azimut/rango/altura y retención de controles positivos de obstáculo.
- Runtime: `scan_ground_filter -> pointcloud_to_laserscan -> scan_noise_filter`; preservar frame/stamp/QoS, tasa y ausencia de salida con TF faltante.
- Composición: `scan_clean` sigue siendo único input autoritativo de Nav2/Collision Monitor; no sumar publicadores.
- Hardware: A/B inicialmente observador en Jetson; sólo después prueba controlada de navegación con operador y E-stop.

Estado: **characterized**, sin paridad ni validación de la política candidata en hardware.
