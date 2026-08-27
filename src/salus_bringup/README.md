# salus_bringup

Propietario de las composiciones completas y perfiles operativos. No contiene
algoritmos, drivers ni contratos.

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
