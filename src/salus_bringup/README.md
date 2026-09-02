# salus_bringup

Propietario de las composiciones completas y perfiles operativos. No contiene
algoritmos, drivers ni contratos.

## Observación RTK/GNSS en coexistencia

`rtk_gnss_observation.launch.py` observa el estado RTK/GNSS del stack legado y
lo expone bajo `/salus/hardware/...` sin intervenir en su cadena activa:

```bash
ros2 launch salus_bringup rtk_gnss_observation.launch.py
```

El perfil predeterminado es `delivery_backend:=disabled` y
`delivery_enabled:=false`. `pixhawk_mavros` añade observación `GPSRAW` y deja al
adaptador como única autoridad del estado canónico; con `delivery_enabled=false`
no crea un publicador RTCM hacia MAVROS. La combinación `pixhawk_mavros` +
`true` está reservada para una validación aislada después de detener el bridge
legado. `direct_usb` falla explícitamente como no implementado. El launch no
inicia NTRIP, MAVROS/FCU, UART, TF ni nodos de movimiento.

El perfil se puede desactivar por completo con `enabled:=false`. En el perfil
predeterminado, el único receptor de correcciones es un contador dry-run. Los
parámetros declaran un solo tipo legado de RTCM
(`uint8_multi_array`), las entradas legacy, las salidas canónicas y el timeout
de frescura (`stale_timeout_s:=5.0`). Este launch no abre clientes de
correcciones, conexiones de receptor ni rutas de actuación; tampoco inicia TF
global ni nodos de movimiento.

## Entradas Pixhawk en coexistencia

`pixhawk_sensor_inputs.launch.py` adapta IMU y GNSS del MAVROS ya operativo sin
iniciar hardware ni actuación:

```bash
ros2 launch salus_bringup pixhawk_sensor_inputs.launch.py
```

Mientras `ROS2_SALUS` es propietario del robot, las salidas lógicas usan
`/salus/imu/data` y `/salus/gps/fix` para no competir con `/imu/data` ni
`/gps/fix`. El launch sólo contiene el adaptador y selectores read-only; no
publica TF, abre UART, llama servicios ni envía comandos.

## Perfil real de observación/coexistencia

`real_observation.launch.py` compone exclusivamente adaptadores que consumen
los tópicos ya publicados por `ROS2_SALUS`; no inicia ni toma propiedad de
hardware. Es el primer perfil que se puede ejecutar junto al servicio legacy:

```bash
ros2 launch salus_bringup real_observation.launch.py
```

Incluye las entradas Pixhawk read-only, la observación RTK/GNSS, telemetría
legacy a mediciones canónicas, la traducción `/cmd_vel_final` a
`/vehicle/command_shadow` y su comparación diagnóstica. La entrega RTCM queda
fijada internamente en `delivery_backend:=disabled` y
`delivery_enabled:=false`; el launch no expone argumentos para cambiarlo.

No contiene UART/serial, MAVROS nuevo, NTRIP, RS16, TF, EKF/navsat, Nav2,
Collision Monitor, arbitraje de comandos, Web/Cockpit ni cámara. Por ello la
ausencia de Pixhawk/GNSS o LiDAR deja sólo datos ausentes/stale según cada
adaptador, sin abrir drivers ni cambiar fuentes. La validación física posterior
será estrictamente read-only y estacionaria.

## Perfil real de shadow de localización

`real_localization_shadow.launch.py` compone el perfil de observación ya
validado más **un solo** EKF local sin autoridad:

```bash
ros2 launch salus_bringup real_localization_shadow.launch.py
python3 tools/observe_localization_shadow.py --duration 60
```

Equivaldría a lanzar por separado `real_observation.launch.py` y
`localization_real_shadow.launch.py`; el wrapper no añade nodos propios ni
capacidad nueva. Su salida es únicamente
`/salus/localization_shadow/odometry/local`, y `ROS2_SALUS` conserva la
autoridad de `/odometry/local`, `odom -> base_footprint`, el resto del TF, el
hardware y el control.

El wrapper **no declara ningún launch argument**: no hay forma de habilitar
entrega RTCM, TF, `use_control`, UART, Nav2 ni propiedad de hardware desde él, y
hereda sin cambios las restricciones demostradas para `real_observation.launch.py`.
Tampoco mueve el robot ni implementa localización global.

Se ejecutó de forma estacionaria junto al `ROS2_SALUS` en vivo el 2026-09-02:
los nueve nodos compusieron correctamente, el shadow publicó de forma continua,
`/odometry/local` y `/tf` conservaron exactamente la autoridad legacy y el
cierre dejó el contenedor en `Exited (0)` sin procesos huérfanos. La evidencia
completa está en
`docs/migration-evidence/intent/physical-local-localization-shadow-hardware-2026-09-02.md`.

## Checkpoint integrado de simulación

El launch actual reúne movimiento Ackermann, controlador simulado, localización
local/global, LiDAR, seguridad y Nav2. Rutas, patrulla, snapshots y Cockpit se
habilitan explícitamente para mantener el diagnóstico básico más liviano. Sigue
siendo un checkpoint de migración, no un bringup operativo final.

```bash
ros2 launch salus_bringup integration_sim.launch.py
```

Para validar la entrada canónica de actuación en simulación:

```bash
ros2 launch salus_bringup integration_sim.launch.py \
  command_input_mode:=canonical_vehicle_command
```

El valor predeterminado continúa siendo `legacy_cmd_vel`. Este selector es
independiente de `vehicle_io_profile`, que elige mediciones y odometría.
El escenario `tools/smoke_navigation_canonical_sim.sh` valida que una meta Nav2
atraviesa el arbitraje único de `nav_command_server`, la traducción temporal a
`VehicleCommand`, el consumidor canónico y el backend Gazebo. No publica
directamente en `/cmd_vel_final`.

## Perfil explícito sin detección local

Para robots donde la detección local de obstáculos no está instalada ni forma
parte del alcance operativo:

```bash
ros2 launch salus_bringup integration_sim.launch.py \
  capability_profile:=no_obstacle_detection
```

El default `obstacle_detection` conserva LiDAR, obstacle layers y collision
monitor. El perfil degradado no inicia la tubería LiDAR ni fabrica scans vacíos;
mantiene keepout, PathHealth, watchdogs, arbitraje, freno y E-stop. Un relay
declarado conserva la frontera `/cmd_vel_safe`, pero no afirma protección
anticolisión. La selección sólo ocurre al arrancar y jamás como fallback ante
una avería.

## Perfiles explícitos de IMU y orientación

Los ejes de sensores se seleccionan por separado y nunca hacen fallback:

```bash
ros2 launch salus_bringup sim_operational.launch.py \
  imu_source:=imu_primary \
  orientation_source:=course_over_ground
```

También se aceptan `imu_secondary` y `external_heading`. En la simulación
actual no existe productor para la IMU secundaria, de modo que elegirla deja
la entrada lógica ausente. El heading externo sí dispone de un fixture de
ground truth para probar el cableado; no representa hardware dual-GNSS.

Con Gazebo y RViz visibles:

```bash
./tools/sim.sh
```

El helper también carga automáticamente ROS dentro del contenedor. Para CI o
depuración sin ventanas se puede usar `./tools/sim.sh --headless`.

Para control manual desde el host: `./tools/cmd_vel_sim.sh straight`, `left`,
`right` o `brake`. La shell de diagnóstico `./tools/shell.sh` usa el mismo
contenedor que la simulación.

## Cockpit en simulación

```bash
./tools/sim.sh --cockpit
cd ../cockpit && npm run dev
```

Usar el preset **Simulation** con `localhost:8766`. El flag activa el bridge
WebSocket y los subsistemas de ruta, patrulla y snapshot que consume Cockpit.
No inicia una cámara ni permite validar hardware real.

Los launches `*_skeleton.launch.py` se conservan como marcadores de los futuros
bringups finales `sim.launch.py` y `real.launch.py`.

## Perfil operacional de simulación remota

`sim_operational.launch.py` es la composición completa destinada a Cockpit y a
la validación end-to-end sin hardware:

```bash
ros2 launch salus_bringup sim_operational.launch.py
```

El perfil operacional acepta el mismo `command_input_mode`. La ruta canónica
continúa limitada a Gazebo y no habilita UART ni hardware.

El helper operativo construye el workspace y expone Cockpit:

```bash
./tools/sim_operational.sh
./tools/sim_operational.sh --headless
```

Activa por defecto navegación, keepout, rutas, patrulla/HOME, snapshots,
WebSocket compacto en el puerto `8766` y cámara PTZ simulada. La persistencia se
agrupa bajo `runtime/sim_operational`; puede cambiarse con `runtime_dir:=...`.
Usar `headless:=true` en automatización y `rviz:=true` sólo para diagnóstico
local. El sufijo `wifi` describe el perfil remoto compacto: no selecciona una
interfaz de red ni configura DDS.

El perfil está listo para validación operacional, pero no sustituye al futuro
bringup real ni demuestra capacidad de mover el robot físico.

Para trazabilidad de la migración, este launch reemplaza en simulación a
`navegacion_gps/sim_global_v2_wifi.launch.py`: conserva la composición
operativa remota, pero no sus configuraciones DDS/WiFi ni `/scan_wifi_debug`.
