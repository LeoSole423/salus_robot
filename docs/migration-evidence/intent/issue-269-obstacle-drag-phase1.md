# Issue #269 — Fase 1 de medición de arrastre de obstáculos

## Alcance

Este corte agrega únicamente infraestructura de medición. No modifica el
pipeline de LiDAR, TF/extrinsics, EKF, costmaps, frecuencias, clearing,
tolerancias ni navegación productiva.

## Escenario reproducible

`src/salus_simulation/worlds/obstacle_drag.world` conserva el world existente
`salus_empty` para que `integration_sim.launch.py` y sus bridges funcionen sin
otro stack. Contiene tres cajas estáticas:

| Objeto | Centro `(x, y)` m | Tamaño `(x, y, z)` m | Clase |
| --- | --- | --- | --- |
| `obstacle_near_post` | `(2.5, 1.5)` | `(0.6, 0.6, 1.5)` | near |
| `obstacle_mid_wall` | `(7.5, -2.5)` | `(2.0, 0.8, 1.6)` | medium |
| `obstacle_far_box` | `(14.0, 5.0)` | `(2.0, 1.2, 1.8)` | far |

La tabla también está versionada en
`src/salus_simulation/config/obstacle_drag_geometry.yaml`, que es la fuente de
geometría consumida por el evaluador. El frame fijo de la medición es `odom`.

## Métricas y topics

`salus_evaluation.static_scan_metrics` recibe `/scan_clean` en
`base_footprint` y la pose de `/odom_raw`. Para cada haz transforma el origen y
la dirección al frame fijo `odom`, calcula la primera intersección con una caja
conocida y puntúa el error absoluto de rango. Haces que no deberían intersectar
una caja no se consideran error. El resultado expone:

- `scan_static_error_rmse_m`;
- `scan_static_error_p95_m`.

El smoke `tools/smoke_obstacle_drag_sim.sh` mantiene un observador ROS activo,
reutiliza `tools/smoke_harness.sh`, inicia el world mediante el launch de
integración existente con `launch_navigation:=false` y guarda provenance de
world, fixture, profile, seed, SHA, dominio, partición, conteos y métricas en
`artifacts/smokes/<run>/`.

## Ejecución headless

```bash
./tools/smoke_obstacle_drag_sim.sh
```

La ejecución de esta fase usa por defecto `ROS_DOMAIN_ID=49`, profile `clean` y
seed `6400`; pueden cambiarse con `SMOKE_ROS_DOMAIN_ID`,
`SMOKE_GZ_PARTITION`, `SMOKE_SENSOR_PROFILE` y `SMOKE_SENSOR_SEED`.

## Inspección gráfica

```bash
docker compose run --rm \
  -e ROS_DOMAIN_ID=49 -e GZ_PARTITION=salus-obstacle-drag-gui \
  -e IGN_PARTITION=salus-obstacle-drag-gui \
  ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 launch salus_bringup integration_sim.launch.py \
      world:=/ros2_ws/install/salus_simulation/share/salus_simulation/worlds/obstacle_drag.world \
      gz_args:="-r" rviz:=true sim_sensor_profile:=clean sim_sensor_seed:=6400'
```

En RViz se deben revisar el robot, `/scan_clean`,
`/local_costmap/costmap` y `/global_costmap/costmap`. Para esta comparación,
usar `odom` como Fixed Frame. La selección de Fixed Frame es una acción visual
del operador; no se persiste ni cambia ningún archivo de producción.

## Limitaciones deliberadas

Esta fase no implementa todavía `trail_width_p95_m` ni
`ghost_persistence_s`, no rastrea obstáculos en el tiempo y no cambia ninguna
configuración para corregir el arrastre. El world permite inspección y una
curva estacionaria/segura en simulación, pero no constituye validación física.
