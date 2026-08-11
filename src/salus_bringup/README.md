# salus_bringup

Propietario de las composiciones completas y perfiles operativos. No contiene
algoritmos, drivers ni contratos.

## Checkpoint integrado de simulación

El launch actual reúne movimiento Ackermann, controlador simulado, localización
local/global y la cadena LiDAR. Sirve para depuración manual; todavía no incluye
Nav2, misiones, Cockpit ni hardware real.

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

Los launches `*_skeleton.launch.py` se conservan como marcadores de los futuros
bringups finales `sim.launch.py` y `real.launch.py`.
