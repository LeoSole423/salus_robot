# Refresco read-only del despliegue operativo de Jetson

## Alcance

- Fuente legacy: servicio `salus-real-global-v2-wifi.service` de la Jetson.
- Destino nuevo: evidencia para el issue #153; no se implementa bringup real.
- Incluido: inventario pasivo del checkout, systemd, Docker, entorno ROS,
  grafo, dispositivos visibles y activos locales que deben preservarse.
- Fuera de alcance: iniciar/detener servicios, publicar o llamar ROS, abrir
  serial, capturar mensajes/payloads, modificar `ROS2_SALUS` o probar hardware.

La sesión se hizo el 2026-09-02 desde otra máquina mediante SSH. El robot
ejecutaba el servicio legacy ya activo. Pixhawk/GNSS y LiDAR estaban
desconectados por decisión del operador; su falta de señal no es un fallo.

## Evidencia histórica y actual

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| `jetson-hardware-contract-observation.md` (27–28/08) | baseline física y grafo de 55 nodos únicos / 136 tópicos | alta |
| Jetson, checkout `ROS2_SALUS/main@8897c84` | checkout operativo limpio, igual al commit esperado | alta |
| systemd y Docker (02/09) | el servicio real habilitado sigue lanzando el contenedor legacy | alta |
| grafo ROS pasivo (02/09) | conserva 55 nodos únicos, 136 tópicos y autoridades de composición | alta |
| inventario de dispositivos (02/09) | sólo confirma presencia visible, no funcionamiento físico | alta |

No se copiaron credenciales, configuraciones NTRIP locales, tokens, rutas
sensibles, payloads RTCM, coordenadas, imágenes, nubes ni mensajes ROS.

## Despliegue efectivo observado

| Capa | Configuración | Clasificación |
| --- | --- | --- |
| checkout legacy | `~/Desktop/SALUS/ROS2_SALUS`, rama `main`, commit `8897c846`; árbol limpio | configurado; observado ahora; observado previamente |
| servicio principal | `salus-real-global-v2-wifi.service`, `enabled` y `active/running` | configurado; observado ahora |
| arranque efectivo | el servicio ejecuta `tools/launch_real_global_v2_wifi.sh` desde el checkout legacy | configurado; observado ahora |
| launch real | `real_global_v2_wifi.launch.py`, que compone `real_global_v2.launch.py` | configurado; observado ahora; observado previamente |
| contenedor ROS | `ros2_salus`, imagen `ros2-humble-perception-ws-salus`, activo; red host, reinicio `unless-stopped`, acceso a `/dev` | configurado; observado ahora |
| otros servicios | Docker y `mediamtx.service` activos; MediaMTX está habilitado | configurado; observado ahora |
| entorno ROS | Humble, dominio 0, `rmw_cyclonedds_cpp`, localhost deshabilitado; no se observaron perfiles DDS explícitos | configurado; observado ahora |

El contenedor monta el árbol fuente, herramientas, `build`, `install` y `log`
del checkout legacy como volúmenes de lectura/escritura, además de los
dispositivos del host. La configuración de launch referencia las configuraciones
reales de localización, Nav2 rolling WiFi, MAVROS, RS16, filtros de nube/scan,
keepout, colisión y cámara. Esta ficha registra nombres, no contenidos.

Se mantiene una deuda de entorno ya observada: al cargar el setup aparece una
referencia ausente a un workspace opcional de cobertura. No se corrigió ni se
atribuye al robot físico.

## Autoridades y fronteras físicas

| Frontera | Autoridad observada ahora | Estado físico el 02/09 | Clasificación |
| --- | --- | --- | --- |
| IMU | `mavros_node -> /imu/data -> EKF/gates` | Pixhawk no conectado; no se muestreó | configurado; observado previamente; pendiente revalidación física |
| GNSS | `mavros_node -> /global_position/raw/fix -> consumidores GNSS` | Pixhawk/GNSS no conectado; no se muestreó | configurado; observado previamente; pendiente revalidación física |
| RTCM | `rtk_source_manager -> /rtcm (UInt8MultiArray) -> rtk_bridge -> /mavros_node/send_rtcm` | no se inició NTRIP/RTCM ni se inspeccionó payload | configurado; observado previamente; pendiente revalidación física |
| telemetría de vehículo | `vehicle_controller_server -> /controller/drive_telemetry -> ackermann_odometry/gates` | un CP210x aparece como `ttyUSB0`; relación/controlador sin revalidar | configurado; observado ahora (dispositivo); observado previamente; pendiente revalidación física |
| odometría derivada | `ackermann_odometry -> /wheel/odometry -> EKF local` | no se muestreó | configurado; observado previamente; pendiente revalidación física |
| LiDAR | `rslidar_points_destination_0 -> /scan_3d -> scan_ground_filter -> scan_noise_filter -> /scan_clean` | LiDAR desconectado; no se inspeccionó nube ni scan | configurado; observado previamente; pendiente revalidación física |
| localización/TF | EKF local, EKF map y `robot_state_publisher` son autoridades `/tf`; contrato esperado `map -> odom -> base_footprint` | no se consultaron transformaciones ni muestras | configurado; observado previamente; pendiente revalidación física |
| comando | `nav_command_server -> /cmd_vel_final -> vehicle_controller_server` | no se publicó, llamó ni movió | configurado; observado ahora (grafo); pendiente revalidación física y de seguridad |

Los conteos de productor observados ahora fueron uno para cada frontera
enumerada, incluido `/cmd_vel_final` y `/mavros_node/send_rtcm`. `/rtcm`
sigue anunciando los dos tipos legacy ya registrados por la baseline; no se
modificó esa deuda de contrato. Los plugins de MAVROS hacen que `/mavros_node`
aparezca repetido en `ros2 node list`; no debe interpretarse como varias
autoridades de una misma frontera.

## Activos locales que deben preservarse

- Reglas udev locales para el ESP (`esp.rules` y `esp_alt.rules`).
- Archivo local ignorado de fuentes RTK; no se leyó ni versionó.
- Datum, máscara keepout, parámetros de localización/Nav2, configuración de
  MAVROS, RS16, cámara y filtros dentro del checkout legacy.
- Estado persistido de build/install/log de `ROS2_SALUS` hasta tener un plan de
  despliegue y rollback.
- Configuración local de MediaMTX y presets de cámara.
- Servicio systemd, script de lanzamiento e imagen de contenedor vigentes.

La asociación concreta entre los archivos locales y los dispositivos físicos
debe verificarse posteriormente sin sustituirlos ni sobrescribirlos durante el
cutover.

## Diferencias respecto de 27–28/08

- La composición ROS coincide en conteos con la baseline (55 nodos únicos,
  136 tópicos) y conserva las autoridades críticas conocidas.
- La baseline tenía Pixhawk/GNSS y RS16 disponibles para caracterización de
  headers, QoS y tasas; esta sesión no revalida ninguna señal porque ambos
  equipos están desconectados.
- Esta sesión agrega el propietario de arranque efectivo (systemd), contenedor,
  mounts, entorno ROS, reglas udev y activos locales a preservar.
- Se observó el servicio legacy corriendo desde hace múltiples días; no se
  infiere por ello salud de sensores ni capacidad de actuación.

## Checks pendientes antes del bringup real

- Reconectar y comprobar pasivamente Pixhawk/IMU/GNSS, frames, timestamps,
  covarianzas, calidad GNSS y separación RTCM/fix.
- Reconectar RS16 y comprobar flujo, timestamps, TF y tasas sin asumir que el
  nodo configurado implique datos físicos.
- Identificar el controlador asociado al CP210x y caracterizar UART/ESP32 sin
  abrir el puerto mientras el servicio legacy lo use.
- Verificar cámara/red cuando la cámara esté disponible, sin exponer RTSP ni
  credenciales.
- Revalidar la cadena TF completa y la localización con datos físicos.
- Comparar telemetría/odometría y comandos en modo observacional antes de crear
  cualquier segunda autoridad.
- Preparar rollback antes de detener `ROS2_SALUS` o habilitar `salus_robot`.

## Estado de evidencia

- Estado propuesto: `characterized`.
- No valida: bringup real de `salus_robot`, paridad, localización física,
  percepción física, RTCM nuevo, UART/ESP32, actuación, safety ni movimiento.
- Punto de parada: Fase 1A del issue #153 termina con esta caracterización.
