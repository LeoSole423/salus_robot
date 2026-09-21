# salus_evaluation

Escenarios, métricas y gates reproducibles de navegación, con un runner ROS
observador que no posee autoridad de comando ni publica TF, según ADR 0007.

Los escenarios instalados usan metros, radianes y segundos. En simulación se
compara la estimación contra `/odom_raw`; esto no valida comportamiento real.

Con una simulación `sim_operational.launch.py` ya levantada:

```bash
./tools/nav_eval.sh run src/salus_evaluation/config/scenarios/right_quarter.yaml
./tools/nav_eval.sh observe
```

`observe` espera el próximo `2D Goal Pose` de RViz e infiere del plan publicado
si la maniobra inicial pide izquierda, derecha o recto. Si no puede inferirlo,
el gate `turn_sign` falla en lugar de omitir esa comprobación. Ambos modos
generan el mismo bundle versionado en `artifacts/evaluations/`.

## Medición de arrastre de obstáculos

La Fase 1 de #269 usa `salus_simulation/worlds/obstacle_drag.world`, un world
con tres cajas estáticas de geometría conocida en `odom`. El fixture
`obstacle_drag_geometry.yaml` conserva sus poses y dimensiones como provenance.
El smoke `./tools/smoke_obstacle_drag_sim.sh` reutiliza
`integration_sim.launch.py` y comprueba que `/scan_clean` vea la geometría sin
iniciar navegación.

`salus_evaluation.static_scan_metrics.scan_static_error_metrics` transforma cada
haz de `/scan_clean` desde `base_footprint` a `odom` usando la pose de `/odom_raw`
interpolada en el timestamp ROS del scan, calcula la intersección esperada con
las cajas conocidas y reporta el error por scan y los índices/errores de los tres
peores haces. Sólo se puntúan haces que deberían intersectar una caja; el espacio
libre no se considera error.

El artefacto `obstacle_drag_metrics.json` usa `schema_version: 2` (el fixture
geométrico sigue en v1). Conserva provenance, pairing, conteos y
`worst_outliers`; sus agregados inequívocos son
`max_scan_static_error_rmse_m`, `max_scan_static_error_p95_m` y
`max_beam_static_error_m`. `per_scan_metrics` está ordenado por `stamp_s` y
contiene `stamp_s`, `sample_count`, `rmse_m`, `p95_m` y `max_m`.

El mismo artefacto agrega diagnósticos report-only para separar causas:
`temporal_offset_sweep` reevalúa los últimos scans con poses de `/odom_raw`
desplazadas entre `-0.30` y `+0.30` s, sin extrapolar ni aplicar el offset al
runtime. Todos los offsets usan exactamente el mismo conjunto de scans y la
intersección de haces scoreables; `scored_beam_support` y
`best_temporal_offset_support` dejan esa población explícita. Por eso
`best_temporal_offset_s` sólo identifica el mínimo de la mediana RMSE entre
entradas realmente comparables. `localization_vs_raw` compara
`/odometry/local` con `/odom_raw`, y `tf_vs_raw` compara el TF dinámico
`odom -> base_footprint` de `/tf` con `/odom_raw`, siempre por timestamp, e
informa divergencia de posición y yaw. El smoke requiere al menos dos muestras
TF emparejadas para considerar válida la captura; una ausencia de TF queda como
fallo de instrumentación, no como una conclusión de baja divergencia. Ninguno
de estos campos es un umbral de aceptación ni modifica TF, EKF, LiDAR o
costmaps.

La captura de lineage del mismo smoke escribe además
`obstacle_drag_stage_metrics.json` (`schema_version: 1`). Sólo considera
cadenas completas emparejadas por igualdad exacta de `header.stamp` y exige al
menos diez antes de declarar la ejecución válida. Conserva los conteos de
pérdida por cada arista de
`/scan_3d_raw → /scan_3d → /obstacles_cloud → /scan → /scan_clean`, latencias
de recepción monotónica respecto de la nube raw, frames, firmas de payload y
métricas geométricas independientes contra `/odom_raw`. La firma de
`PointCloud2` excluye deliberadamente el header para comprobar si el
normalizador cambió el payload; no permite inferir causalidad por sí sola.
Cuando el simulador entrega un frame interno sin TF público, la geometría raw
se reporta mediante el payload idéntico de `/scan_3d` y queda marcada con una
nota explícita, sin inventar una transformación. `/scan` y `/scan_clean`
exponen sus metadatos angulares y el artefacto indica si frame, ángulos y
timestamps se preservaron; stamps, payload raw→normalizado, metadatos y
disponibilidad geométrica son invariantes del smoke y hacen fallar la captura
si no se cumplen. El soporte de haces común se publica para evitar comparar
conjuntos distintos, pero su igualdad entre `/scan` y `/scan_clean` queda como
diagnóstico porque el filtro puede cambiarlo. Este artefacto es evidencia de
simulación y no cierra #269 ni establece un umbral de aceptación.

El smoke ejecuta una única maniobra open-loop por el `/cmd_vel` existente:
entrada recta, curva derecha de radio aproximado de 4 m a 0,5 m/s, salida recta
y stop. Esto mantiene la medición reproducible sin añadir otro controller.
Esto es instrumentación base, no un diagnóstico ni un cambio del pipeline de
LiDAR, TF, EKF, costmaps o Nav2.

Para inspección gráfica, usar dos terminales con el entorno ROS del compose. En
el primero, levantar el world con RViz:

```bash
docker compose run --rm \
  -e ROS_DOMAIN_ID=49 -e GZ_PARTITION=salus-obstacle-drag-gui \
  -e IGN_PARTITION=salus-obstacle-drag-gui \
  ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 launch salus_bringup integration_sim.launch.py \
      world:=/ros2_ws/install/salus_simulation/share/salus_simulation/worlds/obstacle_drag.world \
      gz_args:="-r" rviz:=true sim_sensor_profile:=clean sim_sensor_seed:=6400'
```

En el segundo, ejecutar la misma maniobra con el mismo dominio y
`use_sim_time=true`:

```bash
docker compose run --rm \
  -e ROS_DOMAIN_ID=49 -e GZ_PARTITION=salus-obstacle-drag-gui \
  -e IGN_PARTITION=salus-obstacle-drag-gui \
  ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    python3 /ros2_ws/tools/obstacle_drag_maneuver.py \
      --ros-args -p use_sim_time:=true'
```

En RViz, seleccionar `odom` como Fixed Frame y verificar durante la maniobra
`/scan_clean`, `/local_costmap/costmap`, `/global_costmap/costmap` y el robot.

El bundle v2 conserva `commands.csv` como la solicitud Nav2 en `/cmd_vel` y
agrega las etapas `/cmd_vel_safe`, `/cmd_vel_final`, `VehicleCommand` y
`DriveTelemetry`, junto con los diagnósticos observados de control. Las
correlaciones usan sólo la última muestra causal previa dentro de 0,2 s y
registran su `alignment_gap_s`; la ausencia de una pareja válida se informa sin
alterar los gates funcionales. La dirección medida se persiste en radianes.

La comparación de localización sólo acepta muestras de verdad terreno a menos
de 0.2 s de cada estimación. Los datos no finitos en meta, poses, velocidades,
comandos o plan invalidan la ejecución.

La llegada tiene dos referencias deliberadamente separadas:

- `goal_tolerance_m=1.2`: gate funcional alineado con Nav2 hoy;
- `precision_target_m=0.25`: objetivo futuro, reportado como `calibrating` sin
  fallar CI.

El error final siempre queda registrado, por lo que estos valores pueden
endurecerse después usando distribuciones reales y no una impresión visual.

## Matriz Ackermann velocidad × curvatura

`config/matrices/ackermann_speed_curvature.yaml` define la primera matriz
obstacle-free: velocidades 0,8/1,2/1,6 m/s, recto, llegada corta y arcos de
radio solicitado 8 m y 4 m en ambos sentidos. El radio es la geometría que
solicita el caso; el plan y el ángulo de dirección aplicado siguen siendo
evidencia observada separada en cada bundle. Los valores no modifican Nav2,
los límites Ackermann ni la seguridad.

La ejecución inicia una simulación limpia por trial para evitar que la pose
final, costmaps, plan o estado de Nav2 del trial anterior lo contaminen. Antes
de cada meta aplica y lee `FollowPath.desired_linear_vel` en
`/controller_server`; el resultado y el readback efectivo quedan en los dos
JSON del trial. Si Humble rechaza el cambio runtime, el trial se preserva como
fallido: no se sustituye por un valor supuesto.

Ejecutar la matriz completa y producir el resumen report-only:

```bash
./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/ackermann_speed_curvature.yaml
```

La ejecución conserva `--jobs 1` como default. `--jobs N` limita el número de
trials simultáneos; cada trial recibe su propio `ROS_DOMAIN_ID`,
`IGN_PARTITION == GZ_PARTITION`, `ROS_LOG_DIR`, runtime de zonas y grupo de
procesos. Las identidades se guardan en `matrix_trial` dentro del bundle y del
manifest, y el resumen conserva el orden de la matriz aunque los workers
terminen en otro orden:

```bash
./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/ackermann_speed_curvature.yaml \
  artifacts/evaluations/ackermann-parallel --jobs 2
```

Antes de usar `--jobs 2`, la caracterización de aislamiento de dos
simulaciones debe pasar:

```bash
./tools/nav_eval.sh isolation artifacts/evaluations/isolation
```

Ese comando conserva `isolation-report.json` con las secuencias de `/clock`,
información de publishers, odometría, lifecycle, muerte del worker A y
cleanup. La concurrencia está acotada al harness de evaluación; no cambia el
DDS del robot físico.

También puede agregarse una colección ya capturada de bundles, en el orden
determinista de la matriz:

```bash
./tools/nav_eval.sh matrix-summary \
  src/salus_evaluation/config/matrices/ackermann_speed_curvature.yaml \
  artifacts/evaluations/matrix-baseline \
  artifacts/evaluations/<trial-01> ... artifacts/evaluations/<trial-54>
```

Produce `matrix-manifest.json`, `matrix-summary.json`, CSV y HTML. Los
agregados continuos incluyen mínimo, mediana, máximo y P95 cuando hay al menos
dos muestras. Los performance gates se mantienen explícitamente en
`calibrating/report-only`; los gates funcionales de cada bundle no cambian.
Al finalizar se conservan todas las celdas y el proceso devuelve non-zero si
alguna tuvo setup failure o un gate funcional existente falló. Una métrica
calibrating no cambia ese exit status. El error final de yaw no se aproxima en
esta matriz: queda explícitamente para #63, que definirá su semántica.
