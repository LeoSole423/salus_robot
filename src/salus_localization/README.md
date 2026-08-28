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
- Estado: perfiles de simulación portados; drivers y hardware pendientes.
- Prueba: `colcon test --packages-select salus_localization`.
- Migración: caracterizar primero TF, datum fijo y gating de heading.
