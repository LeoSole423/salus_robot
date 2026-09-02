# Validación física estacionaria del local EKF en shadow (2026-09-02)

## Alcance

Validación **estacionaria y de sólo lectura** del primer EKF local físico de
Salus en modo shadow (#161), ejecutado junto al stack `ROS2_SALUS` en vivo.

- Destino probado: `salus_robot@63a14a49bd956cdcfea29c5ae8761aff34e84636`
  (cabeza del PR #162, rama `agent/physical-localization-shadow`), sobre
  `main@f5b3a97288e043b1cb7263fa40093450111a1a38`.
- Fuente legacy: `ROS2_SALUS/main@f35834989b041f51dd325c626d2338e2232d9e53`,
  checkout limpio, servicio `salus-real-global-v2-wifi.service` activo.
- Robot dentro del galpón, sin movimiento, operador presente.
- No se registraron coordenadas GNSS, IPs, credenciales ni comandos. Las
  posiciones citadas son del frame `odom`, no geográficas.

## Método de coexistencia

Idéntico al de la revalidación de #157, sin tocar el despliegue legacy:

- worktree detached del commit del PR, montado de **sólo lectura** (`:ro`);
- `build`/`install`/`log` en un directorio temporal aparte, borrado al terminar;
- `Privileged=false`, `Devices=[]`, sin montaje de `/dev`: dentro del contenedor
  no existió ningún `/dev/ttyUSB*` ni `/dev/ttyTHS*`;
- red `host`, `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` y el
  mismo `CYCLONEDDS_URI` del legacy (el `cyclonedds_wifi.xml` de
  `navegacion_gps`, montado `:ro`);
- build inicial con `--network=none` y `--cpus=2`, sin aumentar la carga del
  host; el perfil corrió también con `--cpus=2`.

Se compuso el perfil completo vía `salus_bringup real_localization_shadow.launch.py`
(9 nodos: los 8 de `real_observation.launch.py` más `salus_local_ekf_shadow`).

Las tres ventanas (antes / durante / después) se midieron con el mismo
instrumento, `tools/observe_localization_shadow.py`, un único proceso suscriptor.
Esto es deliberado: las sondas `ros2 topic hz` de la CLI levantan procesos
Python pesados que en este Orin alteraron el load average (subió de 7 a ~17 con
sondeo CLI y volvió a ~5 con el observer), y una medición de tasa tomada con
ruido del propio instrumento no es evidencia.

## Resultado

**VALIDADO en hardware, en modo shadow y estacionario.** Ninguna autoridad
cambió de manos.

| Gate de seguridad | Resultado |
| --- | --- |
| `/odometry/local` conserva un único publicador legacy | **OK** (`ekf_filter_node_local_v2`, antes/durante/después) |
| El shadow no publica en `/odometry/local` | **OK** (0 publishers Salus en las tres ventanas) |
| `/tf` no gana autoridad | **OK** (ver payload abajo) |
| Salida del shadow aislada | **OK** (`/salus/localization_shadow/odometry/local`, 1 publicador: `salus_local_ekf_shadow`) |
| Robot sin movimiento | **OK** (lineal 0.0, yaw rate 0.0001 rad/s) |
| Legado vivo durante toda la prueba | **OK** (servicio activo, contenedor con el mismo uptime, EKF legacy con el mismo RSS) |
| Cierre limpio | **OK** (`Exited (0)`, ~9 s, 0 huérfanos en el host) |

## Tasa, frames y timestamps

| Tópico | Antes (60 s) | Durante (60 s) | Después (30 s) |
| --- | --- | --- | --- |
| `/wheel/odometry` | 601 msgs, 10.02 Hz | 601 msgs, 10.00 Hz | 300 msgs, 10.00 Hz |
| `/odometry/local` (legacy) | 1157 msgs, 19.30 Hz | 621 msgs, 10.35 Hz | 305 msgs, 10.14 Hz |
| Shadow `/salus/localization_shadow/odometry/local` | ausente | **620 msgs, 10.33 Hz** | ausente (0 publishers) |

En los tres registros: `frame_id=odom`, `child_frame_id=base_footprint`,
timestamps monótonos, **0 mensajes con valores no finitos**, antigüedad de la
última muestra 0.09 s (sin huecos ni silencio al final de la ventana).

La cadencia del shadow siguió la de sus entradas (10 Hz de odometría de rueda),
no los 30 Hz nominales de `frequency`, y lo hizo replicando la cadencia observada
del legacy en la misma ventana. No se retuneó nada para forzar 30 Hz.

## Autoridad TF: endpoint contra payload

Este es el punto delicado del corte y se midió explícitamente.

| Ventana | Endpoints publicados en `/tf` | Transforms recibidos | Parejas distintas |
| --- | --- | --- | --- |
| Antes | 4 (todos legacy) | 2357 @ 39.33 Hz | `map -> odom` 1200, `odom -> base_footprint` 1157 |
| Durante | **5** (los 4 legacy + `salus_local_ekf_shadow`) | 1818 @ 30.31 Hz | `map -> odom` 1197, `odom -> base_footprint` 621 |
| Después | 4 (todos legacy) | 904 @ 30.14 Hz | `map -> odom` 600, `odom -> base_footprint` 304 |

`salus_local_ekf_shadow` aparece como **endpoint** `/tf` mientras corre, exactamente
como anticipa la ficha de intención: `robot_localization` construye su
`TransformBroadcaster` de forma incondicional. La autoridad se midió sobre el
**payload** y no sobre el grafo:

- las parejas de `/tf` fueron las mismas dos en las tres ventanas, sin ninguna
  pareja nueva atribuible al shadow;
- el conteo de `odom -> base_footprint` coincidió **uno a uno** con la salida del
  EKF legacy en cada ventana (1157/1157, 621/621, 304/305), es decir el shadow
  aportó **cero** transforms;
- la calibración previa de este detector demostró que la fuga sería visible: en
  el contenedor, un único emisor dió 241 transforms a 30.12 Hz y al inyectar un
  segundo emisor la tasa pasó a 60.24 Hz.

Conclusión: la caída de 39.33 a 30.31 Hz en la tasa de `/tf` **no** proviene del
shadow; es variación del propio `odom -> base_footprint` legacy, que también se
observa en la ventana posterior con Salus ya retirado (ver Anomalías).

## Deltas legacy frente a shadow

Emparejamiento temporal de 620 pares (desajuste máximo dentro de la tolerancia
de 0.25 s), **sin retuneo** y con el robot detenido:

| Métrica | Máximo | Media |
| --- | --- | --- |
| Delta de posición | 0.0000 m | 0.0000 m |
| Delta de yaw | 0.0000 rad | 0.0000 rad |
| Delta de velocidad lineal | 0.0000 m/s | 0.0000 m/s |
| Delta de yaw rate | 0.0000 rad/s | 0.0000 rad/s |

Último par emparejado: `position_delta [0.0, 0.0]`, `yaw_delta 0.0`,
`linear_velocity_delta -0.0`, `yaw_rate_delta 0.0`, `match_skew 0.0`.

Muestra coincidente en reposo, en ambos estimadores: posición
`(6.782, -0.2095)` m en `odom`, yaw `0.2828` rad, lineal `0.0` m/s,
yaw rate `0.0001` rad/s.

Que las diferencias sean nulas a 4 decimales **no** equivale a paridad
validada: con el robot inmóvil ambos filtros consumen la misma odometría de
rueda y una tasa de guiñada nula, y el shadow usa una IMU que es adaptación
leída de la misma fuente legacy. Esta ventana demuestra que el shadow estima de
forma continua, coherente y sin desplazar al legacy; no caracteriza
comportamiento en movimiento ni error de calibración.

## Consumo y carga

| Medida | Valor |
| --- | --- |
| CPU del EKF shadow | **2.3 %** de un núcleo (núcleos = 6) |
| RSS del EKF shadow | **22.9 MiB** (VSZ 635 MiB) |
| CPU del EKF legacy local (`ekf_filter_node_local_v2`) en la misma ventana | 2.9 %, RSS 26.3 MiB (26 284 → 26 324 KiB, sin crecimiento) |
| Contenedor Salus completo (9 nodos) | ~26 % de CPU, ~279 MiB de RAM |
| Load average | 5.63 antes de la ventana; el EKF legacy quedó con el mismo RSS tras 3 h de servicio |

El shadow cuesta esencialmente lo mismo que el estimador legacy equivalente, y
no se observó degradación del stack en vivo.

## Cierre y rollback

- Retiro sólo del perfil Salus con SIGINT al proceso real de launch (PID 44
  dentro del contenedor). Fue necesario apuntar ahí porque el PID 1 del
  contenedor es un wrapper `bash -lc` con cadena `&&`, que no reemplaza al
  proceso de launch: repetir el `docker kill --signal=INT` del contenedor
  mataría al wrapper y dejaría los nodos vivos.
- Apagado en ~9 s, contenedor `Exited (0)`, **0** procesos huérfanos de Salus en
  el host.
- Después del cierre: `/odometry/local` con su único publicador legacy, shadow
  con 0 mensajes y 0 publishers, `/tf` de nuevo con 4 endpoints legacy, y los
  tópicos de Salus desaparecidos.
- `ROS2_SALUS` nunca se reinició ni se modificó: servicio activo, contenedor
  `ros2_salus` con el mismo uptime de 3 h, checkout en `f358349` con 0 cambios.
- El checkout auxiliar de Salus quedó como estaba (`625948d`, 0 cambios); el
  worktree del PR, el directorio de build, los JSON y el contenedor se
  eliminaron. No quedó ninguna traza de la prueba en el disco del robot.
- No hubo movimiento: velocidad medida 0.0 en toda la ventana y ninguna orden,
  meta, comando ni publicación Salus sobre `/cmd_vel*`, `/cmd_vel_final`,
  `/imu/data`, `/tf` ni `/odometry/local`.

## Anomalías registradas (para después, no bloqueantes)

1. **`/odometry/local` legacy bajó de 19.30 Hz a ~10.3 Hz.** Se observa también
   en la ventana **posterior**, con Salus completamente retirado, por lo que no
   es una regresión del shadow sino una variación del propio stack legacy
   (mientras, `map -> odom` del EKF global se mantuvo en ~20 Hz en las tres
   ventanas). Requiere seguimiento aparte; este corte no tocó tuning.
2. **La salida del shadow no alcanzó los 30 Hz nominales** sino 10.33 Hz,
   siguiendo la cadencia de sus entradas a 10 Hz. Se registra como hallazgo del
   primer contacto físico; no se modificó `frequency`, máscaras ni covarianzas
   para disimularlo.
3. **Reproducido el doble `rclpy.shutdown()` ya conocido:** en el cierre,
   `vehicle_command_comparison_node` terminó con código 1 (misma traza que #158).
   Sigue siendo un issue propio, explícitamente fuera de este corte.
4. **Artefacto del arnés, no del perfil:** el primer intento de arranque falló
   con `Package 'salus_bringup' not found` porque el comando no fuenteó el
   workspace, y el `--device-read-only` inicial es inválido sin `--device`.

## Lo que esta validación NO afirma

- No valida odometría física canónica ni su calibración mecánica.
- No valida localización global: sin `navsat_transform`, GPS absoluto en `map`,
  heading externo, `gps_course_heading` ni `map -> odom`.
- No transfiere ninguna autoridad TF, de control ni de hardware.
- No caracteriza comportamiento en movimiento, drift, ni paridad de navegación.
- No hace merge del PR #162, y no promueve el componente amplio `localization`.

## Referencias

- Ficha de intención:
  [`physical-local-localization-shadow.md`](physical-local-localization-shadow.md).
- Issue paraguas: #153. Issue de este corte: #161. PR: #162.
