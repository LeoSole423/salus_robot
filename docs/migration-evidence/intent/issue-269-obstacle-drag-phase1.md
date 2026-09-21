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
una caja no se consideran error. El fixture geométrico permanece en
`schema_version: 1`; únicamente el artefacto `obstacle_drag_metrics.json`
usa `schema_version: 2`.

En la ejecución con movimiento, cada scan se conserva con su timestamp ROS y
se asocia mediante interpolación a `/odom_raw` (yaw por el arco más corto), sin
usar la última pose recibida ni extrapolar. El bundle incluye además
`max_m`, el conteo de muestras y los índices/errores de los tres haces con mayor
error por scan, para distinguir outliers de un sesgo global. El artefacto
v2 conserva provenance, pairing, conteos y `worst_outliers`; sus agregados son
`max_scan_static_error_rmse_m`, `max_scan_static_error_p95_m` y
`max_beam_static_error_m`. `per_scan_metrics` queda ordenado por `stamp_s` y
contiene `stamp_s`, `sample_count`, `rmse_m`, `p95_m` y `max_m`.

El corte de instrumentación también conserva `temporal_offset_sweep`, un
barrido report-only de poses de `/odom_raw` entre `-0.30` y `+0.30` s. Todos
los offsets reutilizan el mismo conjunto de scans y la intersección de haces
scoreables por scan; `scored_beam_support` registra esa población y evita que
un candidato gane por evaluar menos o distintos haces. `best_temporal_offset_s`
es sólo el mínimo observado de mediana RMSE entre entradas con soporte idéntico
y `best_temporal_offset_support` conserva el soporte usado. El artefacto
incluye además `localization_vs_raw` (`/odometry/local` frente a
`/odom_raw`) y `tf_vs_raw` (el TF dinámico `odom -> base_footprint` de `/tf`
frente a `/odom_raw`), ambos comparados por timestamp. Ninguno de estos
diagnósticos constituye una corrección ni un gate. Un mínimo estrecho y
estable sería evidencia de desfase temporal constante; la ausencia de mejora
en toda la grilla mantiene abierta la hipótesis de distorsión geométrica o de
barrido.

El smoke ejecuta una maniobra open-loop única a través del `/cmd_vel` existente:
entrada recta, curva derecha de radio aproximado de 4 m a 0,5 m/s, salida recta
y stop. No agrega otro controller ni modifica la navegación productiva.

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

## Inspección gráfica en dos terminales

En el primer terminal, levantar el world con Gazebo y RViz:

```bash
docker compose run --rm \
  -e ROS_DOMAIN_ID=49 -e GZ_PARTITION=salus-obstacle-drag-gui \
  -e IGN_PARTITION=salus-obstacle-drag-gui \
  ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 launch salus_bringup integration_sim.launch.py \
      world:=/ros2_ws/install/salus_simulation/share/salus_simulation/worlds/obstacle_drag.world \
      gz_args:="-r" rviz:=true sim_sensor_profile:=clean sim_sensor_seed:=6400'
```

En el segundo terminal, ejecutar la misma maniobra open-loop usada por el smoke,
con el mismo dominio ROS y `use_sim_time=true`:

```bash
docker compose run --rm \
  -e ROS_DOMAIN_ID=49 -e GZ_PARTITION=salus-obstacle-drag-gui \
  -e IGN_PARTITION=salus-obstacle-drag-gui \
  ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    python3 /ros2_ws/tools/obstacle_drag_maneuver.py \
      --ros-args -p use_sim_time:=true'
```

En RViz se deben revisar durante la maniobra el robot, `/scan_clean`,
`/local_costmap/costmap` y `/global_costmap/costmap`. Para esta comparación,
usar `odom` como Fixed Frame. La selección de Fixed Frame es una acción visual
del operador; no se persiste ni cambia ningún archivo de producción.

## Limitaciones deliberadas

Esta fase no implementa todavía `trail_width_p95_m` ni
`ghost_persistence_s`, no rastrea obstáculos en el tiempo y no cambia ninguna
configuración para corregir el arrastre. Tampoco usa el signo de dirección como
diagnóstico aislado: el perfil simulado conserva la inversión del backend y su
compensación en la odometría legacy. El world permite inspección y una
curva estacionaria/segura en simulación, pero no constituye validación física.
