# Probe read-only de observabilidad ROS

`tools/ros_observability_probe.py` hace una sola captura acotada por ventana.
Sólo crea suscripciones y registra metadatos compactos; no publica, no llama
servicios/acciones, no emite TF y no inicia ningún launch. Cada fila conserva,
cuando existe, el timestamp de origen y registra simultáneamente ROS time,
`time.time_ns()` y `time.monotonic_ns()` de recepción. PointCloud2 y paths se
reducen a frame/forma/firma para mantener pequeño el artifact.

La captura incluye `/scan_3d`, `/obstacles_cloud`, `/scan`, `/scan_clean`,
`/cmd_vel_safe`, `/cmd_vel_final`, `/gps/course_heading/debug`,
`/gps/course_heading`, `/localization/orientation`, `/odometry/global`,
`/tf`/`/tf_static` para `map -> odom` y `odom -> base_footprint`, `/plan`,
`/nav_command_server/events`, `/nav_command_server/telemetry`, los estados de
acciones de NavigateToPose/NavigateThroughPoses y transiciones lifecycle de
planner/controller/BT/behavior. Los eventos existentes permiten correlacionar
goals y replans sin inventar una interfaz nueva.

Uso PC/sim o en un equipo de observación, con la composición ya iniciada:

```bash
SOURCE_SHA="$(git rev-parse HEAD)" \
SOURCE_BRANCH="$(git branch --show-current)" \
python3 tools/ros_observability_probe.py \
  --duration-s 30 \
  --json-out artifacts/observability/one-capture.json \
  --csv-out artifacts/observability/one-capture.csv
```

Para la futura ventana física, verificar antes provenance, dominio ROS y E-stop
accesible; ejecutar el mismo comando con la composición ya operativa. No
publicar comandos, cambiar parámetros (`source_timeout`, tolerancias, EKF o
heading policy), reiniciar servicios ni declarar validación física a partir de
una captura sin mensajes. `source_sha` y `source_branch` deben corresponder al
runtime realmente observado; el probe no los adivina.

La salida JSON contiene filas, incluye con cero mensajes los tópicos esperados
que no aparecieron y resume gaps de recepción/origen. Las
funciones puras `age_seconds`, `timestamp_gap_seconds` e
`interpolate_scalar` sirven para correlación offline: la interpolación no
extrapola fuera de la ventana ni oculta timestamps regresivos.
