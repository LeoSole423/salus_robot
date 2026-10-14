# salus_localization

## Responsabilidad

Localización de SALUS: odometría Ackermann, perfiles explícitos de IMU,
orientación global, GPS/datum y EKF local/global en simulación.

## Interfaces vigentes

`ackermann_odometry` conserva la ruta de compatibilidad desde
`/controller/drive_telemetry` hacia `/wheel/odometry` y `/vehicle/twist`.
Es la autoridad seleccionada por defecto en el perfil `legacy`.

`kinematic_ackermann_odometry` es la ruta canónica seleccionable en simulación:
consume `TractionMeasurement` y `SteeringMeasurement` desde
`/vehicle/kinematic_inputs/traction` y `/vehicle/kinematic_inputs/steering`.
Selecciona por defecto `rear_drive_wheel_equivalent` y
`virtual_center_wheel`, exige muestras `OK`, con procedencia válida y un skew
máximo de 0.05 s. Publica las mismas salidas con QoS reliable/volatile depth
10 y nunca TF; el EKF sigue siendo la única autoridad de
`odom -> base_footprint`. La primera pareja y los saltos temporales hacia
adelante publican twist válido sin integrar distancia; timestamps repetidos o
regresivos nunca producen odometría no monótona.

`integration_sim.launch.py vehicle_io_profile:=canonical` compone el adaptador
legacy, la conversión con calibración exclusiva del simulador y esta
odometría. El default `legacy` conserva la ruta histórica. Sólo una de ellas
publica `/wheel/odometry`; la comparación opcional
`compare_legacy_odometry:=true` envía la salida histórica a
`/comparison/legacy/*` y no la conecta al EKF.
En simulación `/odom_raw` alimenta `/hardware/imu_primary/data_raw`, que se
normaliza como `/hardware/imu_primary/data`. `imu_selector` publica la fuente
elegida (`imu_primary` o `imu_secondary`) como `/imu/data`; sólo la primaria
existe en el fixture actual. Elegir la secundaria no fabrica datos ni conmuta
de vuelta. El EKF publica `/odometry/local` y es la única autoridad de
`odom -> base_footprint`.

`gnss_selector` aplica la misma regla de identidad explícita a
`gnss_primary|gnss_secondary`: se suscribe sólo a la elegida, exige frame y
timestamps crecientes y no hace fallback. Conserva muestras `NO_FIX` para que
la ausencia real de posición no se confunda con silencio o con un fix válido.

## Local EKF físico en shadow

`localization_real_shadow.launch.py` arranca **exactamente un**
`robot_localization/ekf_node` llamado `salus_local_ekf_shadow` con el perfil
`config/localization_local_real_shadow.yaml`. Está pensado para correr junto al
stack `ROS2_SALUS` sin desplazarlo:

- consume `/wheel/odometry` legacy y `/salus/imu/data` (la IMU lógica del
  perfil de observación), no la odometría canónica que aún espera calibración;
- publica sólo `/salus/localization_shadow/odometry/local` y
  `/salus/localization_shadow/diagnostics`;
- fija `publish_tf: false`, `use_control: false` y
  `publish_acceleration: false` en el YAML **y** como override explícito del
  nodo, porque `robot_localization` publica TF por defecto;
- no declara ningún launch argument, así que no puede convertirse en autoridad
  desde la línea de comandos;
- no arranca robot, TF, global EKF, `navsat_transform`, heading, Nav2,
  Collision Monitor ni hardware.

Parámetros del perfil (todos heredados de `localization_local_sim.yaml`; no se
inventó tuning):

| Parámetro | Tipo | Valor | Unidad | Sentido operativo |
| --- | --- | --- | --- | --- |
| `frequency` | float | `30.0` | Hz | cadencia de publicación del shadow |
| `sensor_timeout` | float | `0.2` | s | silencio que invalida una fuente |
| `two_d_mode` | bool | `true` | — | restricción plano/yaw del vehicle |
| `publish_tf` | bool | `false` | — | nunca escribe `odom -> base_footprint` |
| `use_control` | bool | `false` | — | no consume `/cmd_vel` como entrada |
| `publish_acceleration` | bool | `false` | — | no publica `OdometryWithCovariance` |
| `odom0` / `imu0` | string | `/wheel/odometry`, `/salus/imu/data` | — | fuentes físicas usadas |
| `odom0_config` | bool[15] | pos x,y,yaw + vel x,y,yaw | — | máscara del modelo local |
| `imu0_config` | bool[15] | sólo `angular_velocity.z` | — | heading rate sin orientación |
| `odom0_queue_size` / `imu0_queue_size` | int | `10` / `20` | muestras | buffering por fuente |

Matiz importante: `robot_localization` construye su `TransformBroadcaster`
siempre, por lo que el nodo anuncia un *endpoint* `/tf` incluso con
`publish_tf=false`. La garantía es de payload — nunca emite transforms — y así
lo comprueba `test/test_localization_real_shadow_runtime.py`, que levanta el
`ekf_node` real con entradas sintéticas tipadas y exige stream `/tf` vacío y
ausencia total de publisher en `/odometry/local`.

```bash
ros2 launch salus_localization localization_real_shadow.launch.py
python3 tools/observe_localization_shadow.py --duration 60
```

`tools/observe_localization_shadow.py` es el recolector de evidencia: sólo se
suscribe, nunca publica, y reporta tasa, frames, monotonía, valores no finitos,
antigüedad de la última muestra, deltas legacy/shadow y publishers de `/tf`.

Validado en hardware de forma estacionaria y en shadow el 2026-09-02 junto al
`ROS2_SALUS` en vivo: salida continua con los frames y stamps esperados, cero
transforms aportados a `/tf` (medido por payload), cero publishers Salus en
`/odometry/local`, deltas nulos frente al estimador legacy sin retuneo y 2.3 %
de un núcleo con 22.9 MiB de RSS. Ver
`docs/migration-evidence/intent/physical-local-localization-shadow-hardware-2026-09-02.md`.
Esta validación no afirma paridad en movimiento ni localización global.

## Perfil local real autoritativo

`localization_local_real.launch.py` es el corte software autoritativo de #179.
Arranca exactamente `ackermann_odometry` y `robot_localization/ekf_node`
(`salus_local_ekf`):

- `DriveTelemetry` desde `/controller/drive_telemetry` alimenta el adaptador
  compatible, que publica `/wheel/odometry` y `/vehicle/twist`;
- el EKF consume `/wheel/odometry` y `/salus/imu/data`, publica
  `/odometry/local` y es la única autoridad de `odom → base_footprint`;
- el perfil fija el wheelbase legacy `0.94`, la inversión de signo medida y
  las covarianzas reales caracterizadas; no usa `kinematic_ackermann_odometry`;
- no arranca GNSS/NTRIP, heading, perception, Nav2, UART ni owners de
  hardware. `use_sim_time=false`, `publish_tf=true`, `use_control=false` y
  `publish_acceleration=false` están fijados en el YAML y en el launch.

El test runtime sintético publica `DriveTelemetry` + IMU y verifica muestras
frescas, finitas y monótonas en ambas odometrías, además de la autoridad TF y
el rechazo de datos stale/invalidos. Este perfil es evidencia de software; no
declara validación física ni habilita movimiento del robot.

## Prueba

```bash
ros2 launch salus_localization localization_local_real.launch.py
ros2 launch salus_localization localization_sim.launch.py
./tools/smoke_localization_sim.sh
VEHICLE_IO_PROFILE=canonical ./tools/smoke_localization_sim.sh
```

## Perfil global real software-only

`global_localization_real.launch.py` compone exactamente `global_stationary_gates`,
`gps_course_heading`, `orientation_source_selector`, `navsat_transform_node` y el
EKF global. Usa el datum seleccionado de `config/datums.yaml`, con el fallback y
los overrides explícitos compatibles con legacy. El heading sólo acepta un
`GnssRtkStatus` fresco con `RTK_FIXED` y el selector queda fijado a
`course_over_ground`.

El EKF global es la única autoridad de `map -> odom` y publica `/odometry/global`;
`navsat_transform_node` consume esa odometría y publica `/odometry/gps` y
`/fromLL`. Este perfil no inicia hardware, Nav2 ni el EKF local, y su validación
es sintética/software-only; no declara validación en Jetson o robot.

`global_localization_sim.launch.py` añade GPS global simulado y el segundo
EKF. El NavSat raw procede de Gazebo en `/gps/fix_raw`, se normaliza a
`/gps/fix` con perfiles `ideal`, `f9p_rtk` o `m8n`, y conserva un datum fijo.
La misma autoridad de datum expone `/fromLL` para convertir coordenadas
geográficas al frame `map` de simulación.
La orientación global se selecciona exclusivamente entre
`course_over_ground` y `external_heading`. Ambos terminan en
`/localization/orientation`, consumido por el EKF global y `navsat_transform`.
El pseudo-yaw estacionario no alimenta el filtro global y la pérdida de la
fuente elegida no activa la otra. El perfil externo usa un fixture ground-truth
de simulación; no modela todavía un receptor dual-GNSS.

La IMU actual se deriva de la odometría de Gazebo para pruebas reproducibles;
no representa un modelo físico de ruido ni se usa con hardware real.

- Responsabilidad: odometría Ackermann, EKF local/global, datum y heading.
- No contiene: drivers, costmaps, planners ni modelo físico.
- Interfaces previstas: `map -> odom -> base_footprint` y odometrías tipadas.
- Estado: perfiles de simulación portados; el EKF local físico en shadow quedó validado en hardware de forma estacionaria, sin autoridad TF ni de control; odometría canónica calibrada, localización global y drivers siguen pendientes.
- Prueba: `colcon test --packages-select salus_localization`.
- Migración: caracterizar primero TF, datum fijo y gating de heading.

### Course GPS durante maniobras

`gps_course_heading` clasifica `NORMAL → STRONG_TURN → REACQUIRE` con política
pura timestamped. Usa yaw-rate **fusionado de `/odometry/local`**, velocidad y
steering medido fresco; ese EKF conserva sus fuentes IMU/ruedas y covarianzas.
No introduce otra integración de orientación publicada ni modifica GPS posición,
TF o safety. La integral relativa interna sólo verifica excursión de la ventana.

En STRONG_TURN no publica ni guarda heading GPS. Invalida una vez la ventana al
entrar y una vez al salir; REACQUIRE reúne exclusivamente fixes posteriores a
la salida confirmada. Acepta la primera ventana fiable de al menos 2 m, sin
cooldown adicional. Señales ausentes, atrasadas, futuras o no monotónicas son
incertidumbre. Un intervalo de movimiento inválido rompe la cobertura temporal.
Pequeñas oscilaciones no reinician la ventana: se verifica el rango de la
integral relativa, no la suma de magnitudes de yaw-rate. Curvas suaves pueden
invalidar geometría GPS sin ser clasificadas como STRONG_TURN.

Parámetros ROS compartidos por sim/real (todos `double`, finitos, rango > 0):

| Parámetro | Default | Unidad | Significado |
| --- | ---: | --- | --- |
| `enter_rate_rps` | 0.18 | rad/s | Magnitud sostenida de entrada |
| `exit_rate_rps` | 0.08 | rad/s | Salida; debe ser menor que entrada |
| `enter_duration_s` | 0.25 | s | Confirmación temporal de entrada |
| `exit_duration_s` | 0.35 | s | Confirmación temporal de salida |
| `turn_min_speed_mps` | 0.3 | m/s | Movimiento mínimo para clasificar giro |
| `min_steer_deg` | 5.0 | grados | Dirección que corrobora giro |
| `max_motion_age_s` | 0.5 | s | Edad máxima y hueco de cobertura |
| `max_window_excursion_rad` | 0.0872665 | rad | Excursión máxima de ventana GPS (5°) |

**La velocidad para heading** sigue siendo `min_speed_mps=0.8` del estimador.
El umbral de movimiento del detector se llama `turn_min_speed_mps` en ROS
para distinguirlo. Estos valores son candidatos de simulación, no calibración
validada de hardware. `max_abs_steer_deg`, `max_abs_yaw_rate_rps` e
`invalid_hold_s` se conservan por compatibilidad de parámetros, sin efecto en
la nueva política; ningún rumbo retenido es publicable.

Debug JSON conserva los campos existentes y añade `turn_state`, `turn_reason`
y `window_generation`; canal `/gps/course_heading/debug`, `std_msgs/String`,
productor `gps_course_heading`, consumidores observadores, QoS reliable depth 10.
El tópico IMU GPS y sus stamps originales/covarianza de yaw 0.05 rad² se conservan.

Ensayos open-loop: `python3 tools/run_heading_queue.py "$PWD" <prefijo>`;
`--jobs 2` sólo después de `tools/nav_eval.sh isolation <salida>`. La cola usa
el mismo asignador y locks de dominios que la matriz Nav2 y preserva la secuencia
recta/curva/recta/stop; `--scenarios right left gentle oscillations` selecciona
controles. Los bundles están en `artifacts/`, ignorados por Git.

### Posición GNSS: antena → base en simulación

`map_gps_absolute_measurement` publica la **posición de base_footprint**, no la
posición de la antena. Obtiene base→frame del fix mediante TF (montaje de la
URDF), rota ese desplazamiento con yaw de `/odometry/global` en frame `map`
alineado al stamp GPS y lo resta a la proyección geográfica. No usa yaw de
`odom` como si fuese yaw de `map`, ni ground truth como entrada.

La primera orientación válida de `/localization/orientation` confirma la
inicialización absoluta. Antes de ella omite GPS posición; después, la orientación
map sigue la estimación EKF durante STRONG_TURN aunque no haya heading GPS nuevo.
No publica orientación fusionable ni TF, y conserva las máscaras EKF.
`/fromLL` transforma un punto geográfico: **no** aplica el montaje de antena.
El cambio sólo está compuesto en simulación; el camino real no se modifica.

La covarianza horizontal GNSS se rota según datum, conserva el piso existente
0.05 m² e incorpora incertidumbre de yaw con `J var(yaw) Jᵀ`, donde
`J=(offset_y,-offset_x)`, más varianza del montaje. La interpolación usa el
máximo de varianza de sus extremos correlacionados; no inventa precisión por
promediar muestras de un mismo filtro. El modelo de compensación es planar.

| Parámetro | Tipo | Default | Unidad / rango | Significado |
| --- | --- | --- | --- | --- |
| `map_odom_topic` | string | `/odometry/global` | tópico no vacío | Orientación/covarianza en map |
| `map_frame` | string | `map` | frame no vacío | Marco de salida y orientación requerida |
| `base_frame` | string | `base_footprint` | frame no vacío | Base destino del montaje TF |
| `orientation_topic` | string | `/localization/orientation` | tópico no vacío | Confirmación de inicialización absoluta |
| `max_fix_age_s` | double | 0.5 | s, >0 | Edad máxima antes de omitir posición |
| `max_odom_age_s` | double | 0.5 | s, >0 | Edad máxima del heading map disponible |
| `max_heading_gap_s` | double | 0.2 | s, >0 | Hueco máximo para interpolar yaw; sin extrapolación |
| `future_tolerance_s` | double | 0.05 | s, ≥0 | Margen de orden de entrega clock/sensores |
| `mount_variance_m2` | double | 0.0 | m², ≥0 | Varianza isotrópica XY del montaje; URDF sim exacta |
| `covariance_xy` | double | 0.05 | m², >0 | Piso XY y fallback explícito para covarianza GNSS desconocida |
| `debug_topic` | string | `/gps/odometry_map/debug` | tópico no vacío | Motivo, yaw map, montaje y covarianza efectivos |

Contratos: entradas `/gps/fix` (`NavSatFix`, reliable10), `/odometry/global`
(`Odometry`, reliable10), orientación (`Imu`, sensor-data) y TF de montaje.
Salida `/gps/odometry_map` (`Odometry`, reliable10), header `map`, child
`base_footprint`, stamp original; consumidor EKF global. Debug (`String`,
reliable10) es observacional. Una cola acotada a 20 fixes espera cobertura/TF,
con timer 0.05 s y expiración según max_fix_age_s. Fix/frames/covarianzas
inválidos no producen posición; un retroceso del reloj borra historial,
inicialización y pendientes. No hay dato retenido publicado con stamp nuevo.
Un reinicio de la autoridad global requiere reinitializar también el adaptador;
la política general de reset/loss de orientación permanece en #272.

La investigación previa con `--shadows` se reproduce sobre el snapshot
`f062c15`, anterior a esta compensación. El harness/helper rechazan compensar
una medición cuyo child frame ya declara la base para impedir aplicar dos veces
el montaje. Para validar producto se usa la cola normal sin shadows.
