# Intención: cámara PTZ y MediaMTX en el runtime real

## Hechos caracterizados

- `salus_hardware.camera_node` ya implementa el contrato PTZ por ISAPI/HTTP
  Digest y degrada a `ok=false` cuando la configuración o la cámara no están
  disponibles.
- `salus_web.ros_gateway` ya es el adaptador de las operaciones Cockpit:
  `camera_pan`, `camera_zoom_toggle`, `get_camera_status`,
  `camera_ptz_move`, `camera_ptz_preset`, `camera_ptz_set_preset` y
  `get_camera_ptz_state`.
- Cockpit consume el video fuera de ROS mediante el endpoint WHEP de MediaMTX
  para el path `cam3`. La configuración operativa del host usa RTSP on-demand,
  WebRTC en el puerto `8889` y RTSP en `8554`.

## Decisión de composición

`camera_real.launch.py` inicia exactamente un `salus_camera` con backend
`isapi`, `use_sim_time=false` y presets bajo el runtime writable
`/ros2_ws/log/runtime/camera/presets.json`. `real_mvp.launch.py` lo incluye una
sola vez.

MediaMTX sigue siendo un servicio del host, fuera de ROS y del container
principal. Este PR no crea otro owner, no proxyfica frames por el WebSocket y
no añade MediaMTX a readiness/authority de navegación.

La configuración no secreta de la cámara llega desde el EnvironmentFile de
systemd. La contraseña sólo se entrega opcionalmente mediante
`SALUS_CAMERA_PASS_FILE`; `real_runtime_exec.sh` la monta read-only en el
container como `CAMERA_PASS_FILE` y nunca la pasa como parámetro ROS ni la
imprime.

## Degradación y límites

- Si falta la cámara, la configuración o MediaMTX, PTZ/video reportan su
  indisponibilidad y Cockpit muestra el estado degradado.
- El runtime principal, readiness, Nav2, safety y la autoridad de comandos no
  dependen de que exista video o una respuesta PTZ válida.
- La validación física de cámara, MediaMTX, credenciales y PTZ queda pendiente
  para Jetson; las pruebas de este corte son de composición y contratos sin
  hardware.
