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
