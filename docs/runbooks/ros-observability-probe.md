# Probe read-only de observabilidad ROS

`tools/ros_observability_probe.py` hace una sola captura acotada por ventana.
Sólo crea suscripciones y registra metadatos compactos; no publica, no llama
servicios/acciones, no emite TF y no inicia ningún launch. Cada fila conserva,
cuando existe, el timestamp de origen y registra simultáneamente ROS time,
`time.time_ns()` y `time.monotonic_ns()` de recepción. PointCloud2 y paths se
reducen a frame/forma/firma para mantener pequeño el artifact.

La captura de PointCloud2 es por etapas para no convertir al observador en otro
consumidor pesado durante la investigación de starvation. Por defecto es
`none`; `selected` captura sólo `/scan_3d` (o los tópicos indicados) y `all`
captura explícitamente `/scan_3d` y `/obstacles_cloud`.

La captura incluye `/scan`, `/scan_clean`,
`/cmd_vel_safe`, `/cmd_vel_final`, `/gps/course_heading/debug`,
`/gps/course_heading`, `/localization/orientation`, `/odometry/global`,
`/tf`/`/tf_static` para `map -> odom` y `odom -> base_footprint`, `/plan`,
`/nav_command_server/events`, `/nav_command_server/telemetry`, los estados de
acciones de NavigateToPose/NavigateThroughPoses y transiciones lifecycle de
planner/controller/BT/behavior. También observa `/rosout`, pero conserva sólo
el warning directo de Collision Monitor cuyo nodo termina en
`collision_monitor` y cuyo mensaje contiene la diferencia de timestamps y
`Ignoring the source.`. Esa fila tiene `log_node`, `log_message`, `log_stamp_ns`
y `source_stamp_ns` (ambos son el stamp del `rcl_interfaces/msg/Log`) y los
tres relojes de recepción, además de
`causal_signal=collision_monitor_source_ignored_by_timestamp`.
Los eventos existentes permiten correlacionar goals y replans sin inventar una
interfaz nueva.

Uso PC/sim o en un equipo de observación, con la composición ya iniciada:

```bash
SOURCE_SHA="$(git rev-parse HEAD)" \
SOURCE_BRANCH="$(git branch --show-current)" \
python3 tools/ros_observability_probe.py \
  --duration-s 30 \
  --json-out artifacts/observability/one-capture.json \
  --csv-out artifacts/observability/one-capture.csv
```

Para una captura escalonada, usar primero el baseline sin nubes, luego una sola
nube y sólo después el modo completo:

```bash
python3 tools/ros_observability_probe.py --duration-s 60 \
  --pointcloud-mode none --json-out artifacts/observability/baseline.json
python3 tools/ros_observability_probe.py --duration-s 60 \
  --pointcloud-mode selected --pointcloud-topic /scan_3d \
  --json-out artifacts/observability/scan-3d.json
python3 tools/ros_observability_probe.py --duration-s 60 \
  --pointcloud-mode all --json-out artifacts/observability/full.json
```

Antes de usar `selected` o `all` durante movimiento, medir la sobrecarga en
ventanas equivalentes y con el mismo caso de navegación. Registrar el PID del
probe, por ejemplo con `pgrep -n -f ros_observability_probe.py`, y en otra
terminal conservar CPU/RSS con `pidstat -p "$PID" -u -r 1` y
`ps -p "$PID" -o pid,pcpu,rss,etime,cmd`. Comparar baseline, una nube y modo
completo usando mediana y máximo/percentil alto de CPU y RSS. Medir también la
tasa de cada etapa con una sola suscripción de `ros2 topic hz` por vez (o con
las cuentas por ventana del JSON), anotando que `ros2 topic hz` también es un
observador y debe mantenerse igual entre comparaciones:

```bash
ros2 topic hz /scan_3d
ros2 topic hz /obstacles_cloud
ros2 topic hz /scan
ros2 topic hz /scan_clean
```

No comparar una ventana idle con otra en movimiento, ni concluir causalidad
por una subida aislada. Si CPU/RSS o la tasa/gap de una etapa cambia de forma
material al pasar de `none` a `selected`, detener la escalada y conservar esa
evidencia antes de `all`. No se deben cambiar parámetros del runtime, rates,
resolución del LiDAR ni `source_timeout` para hacer la comparación.

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
