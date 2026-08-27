# Observación pasiva de contratos en la Jetson

## Alcance y seguridad

Fecha: 2026-08-27, zona `America/Argentina/Cordoba`.

Se observó por SSH el robot encendido mientras ejecutaba una prueba con
`ROS2_SALUS`. No se cambiaron archivos, ramas, parámetros, servicios, nodos,
contenedores, conexiones NTRIP, puertos ni lifecycle. No se publicó ningún
mensaje, no se llamó ningún servicio, no se envió movimiento y no se grabó una
bag. Las únicas entidades DDS añadidas fueron suscriptores CLI breves para
headers, estados, covarianzas y metadatos de grafo.

`tools/observe_hardware_contracts.py` conserva este procedimiento para futuras
corridas: por defecto sólo genera un inventario de tópicos y QoS. El muestreo
debe pedirse por tópico y está limitado por whitelist a tipos cuyos resultados
se reducen a header, status y covarianzas; nunca ejecuta `echo` sobre RTCM,
imágenes, nubes o mensajes genéricos.

El checkout operativo estaba limpio en `ROS2_SALUS/main`, commit `8897c84`
(`fix(rtk): connect IGN corrections and harden Jetson service lifecycle`). El
launch real era `real_global_v2_wifi.launch.py` dentro del contenedor
`ros2_salus`. Durante dos snapshots el grafo conservó 55 nodos únicos, 136
tópicos y un productor en cada frontera crítica listada abajo.

`ros2 node list` advirtió nombres repetidos `/mavros_node`, consecuencia de la
superficie compuesta de plugins de MAVROS. Las herramientas de diagnóstico no
deben suponer que ese nombre visible identifica de forma única cada endpoint.

Esto es evidencia real de **caracterización**, no ejecución ni validación de
`salus_robot` en hardware. `hardware_validated` permanece `false`.

## Fronteras observadas

| Hecho real | Productor → consumidor principal | Tipo/QoS observado | Limitación relevante |
| --- | --- | --- | --- |
| IMU Pixhawk | `mavros_node → /imu/data → EKF local/gate global` | `sensor_msgs/Imu`, best-effort, volatile, depth 5 | frame `base_link`; no conserva ID ni mount físico de la IMU |
| GNSS Pixhawk | `mavros_node → /global_position/raw/fix` | `sensor_msgs/NavSatFix`, best-effort, volatile, depth 5 | frame `base_link`; transporte y receptor quedan implícitos |
| heading por movimiento | `gps_course_heading → /gps/course_heading → EKF global` | `sensor_msgs/Imu`, reliable, volatile, depth 10 | sólo válido bajo gates de RTK/movimiento; no es heading estático |
| realimentación del vehículo | `vehicle_controller_server → /controller/drive_telemetry` | `interfaces/DriveTelemetry`, reliable, depth 10 | velocidad Hall y mecanismo de dirección no son odometría directa |
| odometría cinemática | `ackermann_odometry → /wheel/odometry → EKF local` | `nav_msgs/Odometry`, reliable, depth 10 | estimación Ackermann derivada, no encoder ni motor individual |
| localización local/global | EKF local/global | `/odometry/local`, `/odometry/global`, reliable, depth 10 | mantiene autoridades separadas `odom→base` y `map→odom` |
| LiDAR RS16 | driver RS → `/scan_3d` → filtros → `/scan_clean` | `PointCloud2` reliable depth 30; `LaserScan` best-effort depth 1 | cambio de LiDAR pertenece al adaptador/percepción |

La muestra IMU contenía orientación, velocidad angular y aceleración lineal
finitas. Su cuaternión era normalizado y las covarianzas estaban presentes. El
GNSS informó `NavSatStatus.status=0`, `service=GPS`, con diagonal de covarianza
aproximada `[5.15, 5.15, 45.62] m²`. No se guardaron coordenadas.

## Cadena RTK observada

```text
rtk_source_manager
  -> /rtcm (std_msgs/UInt8MultiArray)
  -> rtk_bridge
  -> /mavros_node/send_rtcm (mavros_msgs/RTCM)
  -> mavros_node / Pixhawk
```

`/gps/rtk_status_mavros` publicó `rtcm_ok` y `/gps/rtcm_age_s` informó cerca de
0.50 s. Esto demuestra entrega reciente de correcciones, pero no un fix RTK: el
`NavSatFix` observado seguía siendo un fix GPS autónomo, coherente con el robot
en interior.

Se encontraron dos deudas de contrato:

1. `/rtcm` aparece simultáneamente con `std_msgs/UInt8MultiArray` y
   `mavros_msgs/RTCM`. Aunque el bridge acepta ambas, compartir nombre entre
   tipos hace ambiguo discovery, tooling y replay.
2. `/gps/rtk_status` y `/gps/fix_type` tenían suscriptores pero cero productores;
   el estado efectivo se publicaba en `/gps/rtk_status_mavros`.

La migración debe separar adquisición NTRIP, bytes RTCM normalizados y entrega
específica al backend. No se modificó esta cadena operativa.

## Perfiles de adaptación resultantes

| Perfil | Backend físico | Salidas del adaptador | Consumidores lógicos |
| --- | --- | --- | --- |
| `pixhawk` | MAVROS sensor-only + Pixhawk | `/hardware/imu_primary/data`, `/hardware/gnss_primary/fix` | selectores `/imu/data` y GNSS lógico |
| `direct_jetson` | IMU PCB + receptor RTK USB | los mismos tópicos físicos normalizados | exactamente los mismos selectores |
| `hybrid` | una fuente Pixhawk y otra directa | IDs distintos por dispositivo | selección explícita por eje, nunca fallback |
| `dual_gnss_heading` | receptor/solver doble antena | `/heading/external` con yaw, timestamp y covarianza válidos | selector global de orientación |

El adaptador Pixhawk es permanente, no una compatibilidad temporal. MAVROS
termina dentro de `salus_hardware`; localización no debe conocer MAVLink. Para
usar frames como `imu_pixhawk_link` o `gnss_primary_link` será necesario medir
el mount y publicar TF calibrado. No se debe renombrar `base_link` a ciegas.

La entrada RTK directa deberá exponer por separado:

- estado de adquisición/caster, sin secretos;
- edad y contador de frames;
- bytes RTCM internos con un solo tipo por tópico;
- entrega MAVROS o serial/USB específica del receptor;
- calidad GNSS/fix como estado distinto de “RTCM reciente”.

## Aspectos no caracterizados

- Frecuencias y jitter: se evitó mantener suscriptores de medición porque la
  Jetson estaba bajo carga durante una prueba activa.
- Transformaciones físicas Pixhawk/antena y calibración de orientación.
- Relación real AS5600 → mecanismo → ángulo efectivo de rueda.
- Datos de motor/rueda independientes inexistentes en el protocolo actual.
- Heading dual-antena, IMU PCB y RTK USB directo, todavía no instalados.
- Replay/bag comparable y ejecución de `salus_robot` sobre la Jetson.
- Cualquier salida de actuación canónica hacia UART/ESP32.

También se observó una advertencia no fatal al cargar el entorno por una ruta
ausente de un workspace opcional (`/opt/salus_coverage_ws`). Se registra como
deuda de entorno, sin corregirla durante la prueba.

## Criterio para el siguiente paso real

Antes de habilitar un bringup real de `salus_robot`:

1. implementar adaptadores Pixhawk read-only bajo los tópicos físicos;
2. validar frames, timestamp, QoS y staleness mediante la herramienta pasiva;
3. capturar una bag autorizada con el robot inmóvil y luego en movimiento;
4. ejecutar replay en dominio ROS aislado, sin TF ni comandos globales;
5. habilitar primero `VehicleCommand` en dry-run y sólo después planificar HIL.
