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

## Prueba

```bash
ros2 launch salus_localization localization_sim.launch.py
./tools/smoke_localization_sim.sh
VEHICLE_IO_PROFILE=canonical ./tools/smoke_localization_sim.sh
```

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
