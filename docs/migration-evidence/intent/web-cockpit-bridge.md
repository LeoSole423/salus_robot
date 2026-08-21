# Intención: bridge web y compatibilidad con Cockpit

Fuentes históricas principales: `abcfdff`, `1e48c36`, `99deb86`, `f4f1e4e`,
`1900342`, `0208160`, `73434bc`, `492dd13`, `120660f`, `5decce4` y `05f03eb`
de `ROS2_SALUS`. La contraparte vigente se caracterizó contra
`cockpit/src/packages/nav2`, especialmente `messages.ts`,
`Nav2DispatcherBase.ts`, `RobotDispatcher.ts`, `MapDispatcher.ts` y
`MissionDispatcher.ts`.

## Problema que resolvía

`map_tools/web_zone_server.py` era la única frontera entre Cockpit y ROS. Con
el tiempo acumuló más de 5.300 líneas y cinco responsabilidades distintas:

1. parseo, correlación y envío del protocolo WebSocket;
2. clientes, subscriptions y publishers ROS;
3. lock y heartbeat de seguridad del operador;
4. persistencia de waypoints y sesiones JSONL;
5. control de procesos rosbag, puente HTTP de sensores y cámara.

La intención útil es conservar el contrato visible por Cockpit, no esa
estructura monolítica.

## Contrato de transporte vigente

- Endpoint: WebSocket de texto JSON, default `0.0.0.0:8766`.
- Cada solicitud contiene `op` y puede contener `client_req_id`. Cockpit envía
  simultáneamente `requestId` y el alias legacy `client_req_id`, además de
  conservar `payload` y aplanar sus campos al nivel superior.
- El bridge debe aceptar `client_req_id`, `requestId`, `clientReqId` o
  `request_id`, normalizarlos internamente y responder con
  `client_req_id`. No se correlacionará solamente por nombre de operación:
  puede haber solicitudes concurrentes del mismo tipo.
- La respuesta común es `{"op":"ack","request":<op>,"ok":bool,
  "error":null|string,"client_req_id":...}` más campos específicos.
- Respuestas con contenido propio, como `state`, `nav_snapshot`, `datums` y
  sesiones, conservan su `op` y el mismo identificador de correlación.
- Al conectar, el servidor envía un `state` inicial. Los broadcasts no llevan
  identificador de solicitud.
- JSON inválido y operaciones desconocidas producen un ack negativo; nunca
  deben cerrar el proceso ni quedar sin respuesta.

## Superficie que debe migrarse ahora

| Grupo | Operaciones de entrada | Salida principal | Adaptador ROS nuevo |
| --- | --- | --- | --- |
| Estado | `get_state` | `state` | estados tipados de navegación, rutas, patrulla, batería y zonas |
| Seguridad UI | `set_control_lock`, `control_heartbeat` | `ack`, `state`, `nav_telemetry` | política interna, sin servicio ROS público nuevo |
| Zonas | `set_zones_geojson`, `load_zones_file` | `ack`, `state` | `SetZonesGeoJson`, `GetZonesState`, reload `Trigger` |
| Goal | `set_goal_ll`, `cancel_goal`, `brake` | `ack` | `SetNavGoalLL`, `CancelNavGoal`, `BrakeNav` |
| Rutas | `set_route_ll`, `cancel_route` | `ack`, `state` | `SetRouteMissionLL`, `CancelRouteMission`, `GetRouteMissionState` |
| Patrulla/HOME | `set_patrol_ll`, `cancel_patrol`, `request_return_home` | `ack`, `state` | contratos de patrulla y HOME vigentes |
| Perfiles | `set_navigation_profile` | `ack` | `SetNavigationProfile` |
| Manual | `set_manual_mode`, `set_manual_cmd` | `ack` | `SetManualMode` y `/cmd_vel_teleop` |
| Snapshot | `get_nav_snapshot` | `nav_snapshot` | `GetNavSnapshot`; PNG convertido a base64 sólo en el gateway |
| Waypoints | `save_waypoints_file`, `load_waypoints_file` | `ack` | repositorio atómico independiente de ROS |
| Sensor info | `set_sensor_info_view` | `ack`, `sensor_info` | vista `general`; otras vistas degradadas explícitamente |

`set_manual_cmd` conserva campos top-level `linear_x`, `angular_z` y
`brake_pct`; no se reintroduce `SetManualCmd`. Los comandos se publican como
`CmdVelFinal` en `/cmd_vel_teleop` y el watchdog sigue perteneciendo al
arbitraje, no al WebSocket.

## Broadcasts que Cockpit consume

- `state`: snapshot coherente y de baja frecuencia tras conexión o cambio de
  estado; incluye zonas, pose, GPS, control manual, goal, ruta, patrulla,
  alertas, batería y lock.
- `nav_telemetry`: estado operativo compacto; conserva `cmd_vel_safe`,
  `drive_telemetry`, `manual_control`, goal, ruta, patrulla, alertas y los
  campos de batería consumidos por `RobotDispatcher`.
- `nav_event`, `nav_alerts`, `robot_pose`, `gps_status` y `drive_telemetry`:
  deltas/eventos con su forma legacy vigente.
- `nav_snapshot`: `mime`, dimensiones, frame, stamp, flags, `image_b64` y
  `image_size_bytes`. El gateway no vuelve a renderizar.

El perfil inicial será `telemetry_profile: compact`: coalescerá estados que se
pueden reemplazar y limitará su tasa, pero nunca descartará acknowledgements,
eventos, cambios de lock ni transiciones de misión. `/scan_preview` será una
salida posterior y separada; la nube 3D no atraviesa este bridge.

## Lock de operador y precedencia segura

- Con controles bloqueados se rechazan goal, ruta, patrulla, perfil, HOME,
  comando manual y la activación del modo manual.
- Cancelaciones, `brake`, desactivar modo manual, consultar estado y modificar
  zonas permanecen disponibles para detener o diagnosticar.
- Desbloquear inicia el heartbeat. Si vence el timeout monotónico se vuelve a
  bloquear con causa `UI_HEARTBEAT_TIMEOUT` y se difunden estado y telemetría.
- El lock web no sustituye manual, freno, E-stop, watchdog ni
  `collision_monitor`; estos mantienen precedencia aguas abajo.
- La propiedad multi-cliente quedó resuelta en ADR 0005: la conexión que
  desbloquea adquiere un lease exclusivo; otros clientes conservan operaciones
  seguras de consulta, cancelación, freno y bloqueo, pero no pueden comandar.
  Desconexión o heartbeat vencido liberan el lease y vuelven a bloquear.

## Separación obligatoria en `salus_web`

- `protocol`: codec, aliases, validación, ack y errores; lógica pura.
- `operator_guard`: lock/heartbeat con reloj inyectado; lógica pura.
- `state_projection`: transforma mensajes ROS cacheados en payloads sin I/O.
- `ros_gateway`: clientes/subscriptions/publisher y timeouts; no conoce sockets.
- `websocket_server`: conexiones, colas, correlación y despacho; no conoce
  tipos ROS.
- `waypoint_repository`: lectura/escritura atómica y validada.
- `mission_sessions` y `rosbag_manager`: componentes posteriores, opcionales y
  separados del gateway operativo.

Cada cliente tendrá una cola de salida acotada y un único writer. Un cliente
lento no bloqueará a los demás: estados reemplazables se coalescen; mensajes
no descartables que excedan el límite cierran únicamente ese cliente con
diagnóstico. El parser limitará tamaño de frame, profundidad/forma JSON y
rangos numéricos antes de llamar ROS.

## Capacidades diferidas o degradadas

| Operación | Decisión de esta etapa |
| --- | --- |
| `get_datums` | exponer sólo el datum fijo como lectura compatible |
| `set_datum`, `save_datum`, `delete_datum`, `select_datum`, `capture_current_gps_datum` | rechazo explícito `UNSUPPORTED_FIXED_DATUM`; no reintroducir setters legacy |
| `select_rtk_source`, `upsert_rtk_source` | diferir al adaptador GNSS/RTK |
| `camera_*`, `get_camera_*` | forma de protocolo reservada; implementación en el corte PTZ/cámara |
| `start_rosbag`, `stop_rosbag`, `get_rosbag_status`, `mission.*` | segundo subcorte desacoplado; nunca ejecutar subprocess desde el gateway ROS |
| `start_recording`, `stop_recording`, `clear_recording`, `start_patrol`, `stop_patrol`, `mission.start` | no implementadas por el backend operativo actual; responder `unknown op` por compatibilidad real |
| vistas `topics`, `lidar`, `camera` de `sensor_info` | `implemented=false` hasta tener productor específico |
| video | fuera de ROS y del WebSocket; MediaMTX/WHEP permanece externo |

## Fallos e invariantes

| Condición | Resultado requerido |
| --- | --- |
| JSON no objeto, inválido o `op` vacío | ack negativo acotado |
| mismo `op` concurrente con IDs distintos | respuestas correlacionadas por ID, aunque se reordenen |
| servicio ROS ausente, timeout o rechazo | ack negativo con causa estable; loop WebSocket sigue vivo |
| desconexión durante llamada ROS | future puede finalizar, pero no publica a un socket eliminado |
| cliente lento | no bloquea callbacks ROS ni otros clientes |
| telemetry cache vencida | se marca no disponible; no se inventa estado fresco |
| lock vencido | comandos controlados rechazados; cadena segura aguas abajo intacta |
| snapshot fallido | `nav_snapshot ok=false`, sin base64 parcial |
| archivo de waypoints inválido | se conserva el archivo y estado anteriores |

El bind `0.0.0.0` legacy no implica autenticación ni cifrado. La primera
migración conserva compatibilidad en una red confiable y debe documentarlo.
TLS, token y autorización remota son una decisión de despliegue posterior, no
deben presentarse como seguridad existente.

## Evidencia de implementación

El runtime mantiene separados `ros_gateway`, `websocket_server`, políticas
puras y persistencia. `tools/smoke_web_cockpit.sh` ejecuta un cliente WebSocket
real contra el stack simulado y verifica conexión, lease/heartbeat, estado,
zonas, waypoints, manual seguro, snapshots y operaciones de cancelación. El
smoke también detectó y corrigió una colisión con almacenamiento interno de
`rclpy.Node` y una carrera en la inicialización de la máscara keepout.

Sesiones, rosbag, selección RTK y cámara continúan como subcortes separados;
no se incorporan al gateway monolíticamente.
