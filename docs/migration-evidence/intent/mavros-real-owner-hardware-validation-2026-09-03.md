# Validación física del owner MAVROS real — 2026-09-03

## Alcance y seguridad

Se validó de forma estacionaria y read-only el owner `salus_hardware/pixhawk_real.launch.py`.
No se publicaron comandos ni metas, no hubo movimiento y no se iniciaron UART del
ESP32, NTRIP, entrega RTCM, RS16, Nav2 ni Collision Monitor. `ROS2_SALUS`
conservó la autoridad operativa antes y después de la prueba.

La fuente probada fue `salus_robot/main@c3829dddaa48631a93525c8fc035a99f73d2d083`.
El legacy observado fue `ROS2_SALUS@f35834989b041f51dd325c626d2338e2232d9e53`.

Para evitar cambiar el despliegue persistente, los paquetes `interfaces`,
`salus_interfaces` y `salus_hardware` se construyeron en un contenedor efímero
con el checkout limpio montado y el runtime ROS Humble/MAVROS disponible en la
imagen operativa. Sólo `/dev/ttyACM0` se expuso a ese contenedor; no se montó ni
se abrió `/dev/ttyUSB0`.

## Owner nuevo y señales observadas

El servicio `salus-real-global-v2-wifi.service` estaba `inactive` antes del
arranque. No había procesos MAVROS legacy y `/dev/ttyACM0` estaba libre.

El launch nuevo abrió exactamente un proceso `mavros_node` con
`/dev/ttyACM0:921600`. MAVROS informó enlace abierto, heartbeat ArduPilot y
`FCU connected`. Durante una ventana de aproximadamente 10 segundos se observó:

| Señal | Tipo y endpoint | Resultado |
| --- | --- | --- |
| IMU | `/imu/data`, `sensor_msgs/msg/Imu`, publisher `/mavros_node` | ~10 Hz; `frame_id: base_link` |
| GNSS | `/global_position/raw/fix`, `sensor_msgs/msg/NavSatFix`, publisher `/mavros_node` | ~2 Hz; `frame_id: base_link`; status 0 |
| GPSRAW | `/mavros_node/mavros_node/gps1/raw`, `mavros_msgs/msg/GPSRAW`, publisher `/mavros_node` | ~2 Hz; `fix_type: 3`; 31 satélites; `frame_id: /wgs84` |

El tópico GPSRAW efectivo tiene el prefijo duplicado
`/mavros_node/mavros_node/`; no apareció el nombre simplificado
`/mavros_node/gps1/raw` indicado en el procedimiento. No se corrigió durante
esta validación.

## Autoridad y TF

- `/tf` mostró un endpoint publisher de MAVROS, pero `ros2 topic echo --once`
  y `ros2 topic hz` no recibieron mensajes durante la ventana; no se observó
  una transformación publicada por el owner nuevo.
- No aparecieron publishers de `/cmd_vel`, `/cmd_vel_final` ni otros comandos.
- El tópico efectivo de entrega `/mavros_node/mavros_node/send_rtcm` tuvo cero
  publishers (sólo el subscriber interno de MAVROS).
- El proceso MAVROS tuvo descriptor únicamente para `/dev/ttyACM0`; no abrió
  `/dev/ttyUSB0`.

## Recursos y anomalías

MAVROS consumió aproximadamente 2,7% CPU y 51 MiB RSS; el contenedor completo
consumió aproximadamente 2,8% CPU y 85 MiB RSS.

Se registraron timeouts de `AUTOPILOT_VERSION`; el FCU permaneció conectado y
las señales continuaron frescas. También quedó registrada la discrepancia entre
los frames configurados (`imu_link`/`gps_link`) y los frames efectivos
(`base_link`/`/wgs84`). No se ajustaron parámetros en respuesta.

## Shutdown y rollback

Se envió SIGINT únicamente al launch nuevo. El contenedor terminó con código 0,
sin procesos MAVROS huérfanos, sin descriptores seriales nuevos y sin restos de
`salus_robot` en ejecución.

Después se restauró `salus-real-global-v2-wifi.service`, que quedó `active
(running)`. El MAVROS legacy recuperó el FCU y volvió a ser el único proceso con
descriptor `/dev/ttyACM0`. Sus señales volvieron a estar frescas:

- `/imu/data`: ~10 Hz, `base_link`;
- `/global_position/raw/fix`: ~2 Hz, `base_link`;
- `/mavros_node/gps1/raw`: ~2 Hz, `mavros_msgs/msg/GPSRAW`, `/wgs84`.

No quedaron procesos ni contenedores de `salus_robot`; sólo permanecieron los
contenedores operativos legacy (`ros2_salus` y `netdata_salus`).

Esta evidencia valida el owner físico MAVROS/Pixhawk y su rollback. No valida
localización, NTRIP propio, RS16, UART/control, Nav2 ni el cutover completo.
