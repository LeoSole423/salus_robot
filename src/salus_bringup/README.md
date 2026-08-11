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
ros2 launch salus_bringup integration_sim.launch.py gz_args:=-r rviz:=true
```

Los launches `*_skeleton.launch.py` se conservan como marcadores de los futuros
bringups finales `sim.launch.py` y `real.launch.py`.
