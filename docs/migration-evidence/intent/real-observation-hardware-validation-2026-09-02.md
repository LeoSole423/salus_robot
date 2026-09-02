# Validación física read-only de `real_observation.launch.py`

## Alcance

- Fuente legacy: `salus-real-global-v2-wifi.service` en la Jetson, con
  `ROS2_SALUS/main@f35834989b041f51dd325c626d2338e2232d9e53`.
- Destino nuevo: `salus_robot/main@625948d56894d230cfa88cadab0aa484ae5679a9`.
- Incluido: coexistencia estacionaria, sólo de lectura, del perfil
  `real_observation.launch.py` y comparación de autoridades antes/durante/después.
- Fuera de alcance: movimiento, comandos o metas, UART, MAVROS/NTRIP/RS16
  nuevos, TF/localización, Nav2, safety, Web, cámara y cambios persistentes del
  despliegue legacy.

La prueba se realizó el 2026-09-02 dentro de un galpón. Pixhawk, GNSS/F9P y
RS16 estaban conectados. La recepción GNSS degradada del interior no se trata
como un fallo de la migración.

No se registraron credenciales, correcciones RTCM, coordenadas GNSS, nubes ni
comandos de movimiento.

## Baseline y autoridad legacy

| Frontera | Evidencia observada antes del shadow |
| --- | --- |
| servicio | `salus-real-global-v2-wifi.service` activo; contenedor `ros2_salus` activo |
| MAVROS/Pixhawk | un `mavros_node` publica IMU, GNSS y participa en TF; Pixhawk visible como dos ACM |
| RTCM | `rtk_source_manager -> /rtcm -> rtk_bridge -> /mavros_node/send_rtcm`; un solo publicador `rtk_bridge` y un solo suscriptor MAVROS |
| UART/ESP32 | CP210x visible como `ttyUSB0`; `controller_serv` legacy mantiene el descriptor |
| TF | cuatro publicadores legacy: EKF map, EKF local, MAVROS y `robot_state_publisher` |
| telemetría | `vehicle_controller_server` es el único publicador de `/controller/drive_telemetry` |
| LiDAR | `rslidar_points_destination_0` publica `/scan_3d`; `scan_noise_filter` publica `/scan_clean` |
| comando | `nav_command_server` es el único publicador de `/cmd_vel_final` |

La captura física original 27–28/08 se hizo contra `8897c84`; el servicio
operativo había avanzado a `f358349` en esta sesión. Conservó la composición y
autoridades relevantes, pero no se asume equivalencia de cada cambio intermedio.

## Señales físicas observadas

| Señal | Resultado sanitizado |
| --- | --- |
| `/imu/data` | `base_link`, aproximadamente 10 Hz, QoS best-effort/depth 5 |
| `/global_position/raw/fix` | `base_link`; `NavSatStatus=-1`, service 1; no tasa estable registrada durante la ventana interior |
| GPSRAW | `fix_type=1`; no es RTK float/fixed |
| RTCM legacy | edad observada cercana a 0,36 s antes del shadow |
| `/controller/drive_telemetry` | aproximadamente 10,0 Hz, reliable/depth 10 |
| `/scan_3d` | `lidar_link`, aproximadamente 7,0 Hz |
| `/scan_clean` | `base_footprint`, aproximadamente 9,2 Hz |

La recepción/frescura RTCM y la calidad GNSS se registran por separado. Dentro
del galpón, el estado GNSS degradado es una limitación ambiental válida.

## Coexistencia del perfil Salus

El perfil se compiló y ejecutó en un contenedor separado, con red host y sin
`--privileged`, `--device` ni montaje de `/dev`. Arrancó exactamente:

- `pixhawk_sensor_adapter`, `imu_selector`, `gnss_selector`;
- `legacy_rtk_observer`, `rtcm_dry_run_sink`;
- `legacy_drive_measurement_adapter`;
- `legacy_vehicle_command_adapter`,
  `vehicle_command_shadow_comparison`.

| Frontera nueva | Resultado durante el shadow |
| --- | --- |
| IMU | un publicador en `/hardware/imu_primary/data` y uno en `/salus/imu/data`; ambos entregaron headers con `base_link` |
| GNSS | un publicador en `/hardware/gnss_primary/fix` y uno en `/salus/gps/fix`; ambos entregaron headers con `base_link` |
| RTCM canónico | un publicador `legacy_rtk_observer` en `/salus/hardware/rtcm/corrections`; un único receptor `rtcm_dry_run_sink` |
| estado RTK canónico | `corrections_fresh=true`, edad cerca de 2,04 s, backend y entrega `DISABLED`; `fix_quality=UNKNOWN`, `receiver_fix_type=-1` |
| RTCM hacia Pixhawk | se mantuvo un solo publicador legacy `rtk_bridge` en `/mavros_node/send_rtcm`; Salus no añadió ninguno |
| TF / TF estático | se mantuvieron cuatro/tres publicadores legacy; Salus no añadió ninguno |
| LiDAR | no se inició driver Salus; los productores legacy permanecieron únicos |

El estado RTK canónico `UNKNOWN` no contradice GPSRAW legacy `fix_type=1`:
el perfil de observación fija `delivery_backend=disabled`, por lo que no inicia
el adaptador Pixhawk que traduce GPSRAW. Es una limitación conocida del perfil,
no una deducción de que RTCM fresco produzca solución RTK.

## Anomalía encontrada: contratos legacy incompatibles

La coexistencia encontró dos incompatibilidades de tipo ROS que impiden validar
los adaptadores de telemetría y command shadow sobre el stack instalado:

| Tópico | Publicador legacy | Suscriptor Salus | Consecuencia |
| --- | --- | --- | --- |
| `/controller/drive_telemetry` | `interfaces/msg/DriveTelemetry` | `salus_interfaces/msg/DriveTelemetry` | no llegan `TractionMeasurement` ni `SteeringMeasurement` |
| `/cmd_vel_final` | `interfaces/msg/CmdVelFinal` | `salus_interfaces/msg/CmdVelFinal` | no llega `VehicleCommand` shadow ni diagnóstico de comparación |

Durante el shadow el grafo mostró ambos tipos bajo cada nombre, con un único
publicador legacy. No se inyectaron comandos para provocar tráfico. Esta
incompatibilidad no creó una segunda autoridad ni modificó el flujo legacy,
pero bloquea la siguiente validación de telemetría/comando hasta añadir un
adaptador de compatibilidad tipado en un PR separado.

## Cierre y rollback

- Se detuvo únicamente el contenedor temporal del perfil Salus.
- El primer cierre mediante la señal por defecto de Docker excedió el grace
  period y terminó con código 137; no dejó procesos huérfanos ni alteró el
  legacy.
- Una segunda ejecución se cerró con `SIGINT` y código 0. El log conserva
  trazas `KeyboardInterrupt` de procesos que aún estaban arrancando en la
  Jetson; se debe preferir `SIGINT` y esperar readiness antes de futuros
  cierres controlados.
- Tras la expiración del lease DDS no quedaron nodos Salus; `/cmd_vel_final`
  volvió a un único tipo legacy, dos suscriptores legacy y un único publicador.
- El servicio, MAVROS/RTCM, TF, UART y RS16 legacy permanecieron activos. No
  se observó movimiento.

## Estado de evidencia

- Estado propuesto: `characterized` para coexistencia física segura; el perfil
  sigue `ported` y no alcanza paridad ni `hardware_validated` integral.
- Validado: entradas IMU/GNSS read-only, separación RTCM/fix, ausencia de
  nuevas autoridades, baseline RS16 y rollback sin impacto legacy.
- Pendiente: adaptadores de compatibilidad para los dos mensajes legacy,
  mediciones/command-shadow con tráfico real, localización shadow, drivers
  físicos propios y cualquier cutover de control.
