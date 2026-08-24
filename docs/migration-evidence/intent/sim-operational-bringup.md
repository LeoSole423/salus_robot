# Intención: bringup operacional de simulación remota

## Fuente caracterizada

El perfil legacy `navegacion_gps/sim_global_v2_wifi.launch.py` reunía el robot
Ackermann simulado, localización global, LiDAR, seguridad, Nav2, rutas, patrulla,
HOME y la aplicación web. Sus wrappers añadían CycloneDDS y un scan reducido
para observar el robot a través de un enlace WiFi.

La intención útil no es preservar un archivo monolítico ni seleccionar una
interfaz de red desde ROS. Es disponer de una composición completa, equivalente
al perfil remoto real, que pueda validarse sin hardware y operarse desde
Cockpit con tráfico compacto.

## Reemplazo de launch legacy

`salus_bringup/sim_operational.launch.py` es el **reemplazo de simulación** de
`navegacion_gps/sim_global_v2_wifi.launch.py`. Conserva su propósito operativo:
componer la navegación global completa para operación remota y validación sin
hardware. No es un alias de compatibilidad ni conserva las decisiones legacy de
DDS, interfaz WiFi o `/scan_wifi_debug`.

## Decisiones de migración

- La entrada nueva es `salus_bringup/sim_operational.launch.py`; `V2` y `wifi` no forman
  parte de la nomenclatura canónica.
- El launch incluye el checkpoint `integration_sim.launch.py` y activa rutas,
  patrulla/HOME, snapshots, bridge WebSocket y cámara simulada. Los algoritmos
  siguen perteneciendo a sus paquetes de subsistema.
- `/scan_preview` reemplaza al contrato diagnóstico legacy
  `/scan_wifi_debug`; Nav2 continúa consumiendo `/scan_clean`.
- `compact` es el perfil remoto de telemetría por defecto y el WebSocket conserva
  el puerto `8766`.
- La configuración DDS y la elección de interfaz física quedan fuera del launch.
- Zonas, patrulla, waypoints y presets de cámara comparten la raíz escribible
  `runtime/sim_operational`, pero usan subdirectorios separados.
- `integration_sim.launch.py` permanece como checkpoint liviano compatible; no
  se convierte implícitamente en el perfil operacional.
- Con `--rviz`, la herramienta `2D Goal Pose` publica `/goal_pose`; la meta pasa
  por `nav_command_server` y conserva las políticas de manual, keepout y freno.

## Ownership y readiness

La composición exige una sola instancia de `route_executor`,
`patrol_mission_coordinator`, `nav_snapshot_server`, `salus_web_gateway` y
`salus_camera`, además de un único publisher de `/cmd_vel_final`. El probe
operacional valida esos owners, los servicios públicos requeridos, la cadena TF,
odometría, `/scan_clean`, `/scan_preview` y lifecycle de navegación.

## Evidencia ejecutable

- `tools/smoke_sim_operational.sh` valida en una única sesión el grafo
  operacional, una ruta con movimiento, perfiles de navegación y Cockpit
  (lease, zonas, snapshot, PTZ y parada segura).
- `tools/smoke_operational_persistence.sh` reinicia el bringup con el mismo
  runtime y verifica que waypoints y presets PTZ persistan.
- `tools/smoke_reliability.sh` incorpora ambos escenarios para repeticiones
  nocturnas. Una única ejecución correcta no declara paridad de hardware.

## Límites de evidencia

Esta composición no valida UART/ESP32, MAVROS/Pixhawk, GNSS/RTK real, RS16,
Hikvision, la red de la Jetson ni movimiento físico. La paridad operacional sólo
podrá declararse después del smoke end-to-end repetido del perfil nuevo.
