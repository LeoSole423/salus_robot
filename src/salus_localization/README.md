# salus_localization

## Responsabilidad

Localización de SALUS. El corte actual contiene odometría Ackermann, IMU
simulada normalizada y EKF local; GPS, datum y localización global aún no se
han migrado.

## Interfaces vigentes

`/controller/drive_telemetry` alimenta `/wheel/odometry` y `/vehicle/twist`.
En simulación `/odom_raw` alimenta `/imu/data_raw`, que se normaliza como
`/imu/data`. El EKF publica `/odometry/local` y es la única autoridad de
`odom -> base_footprint`.

## Prueba

```bash
ros2 launch salus_localization localization_sim.launch.py
./tools/smoke_localization_sim.sh
```

`global_localization_sim.launch.py` añade GPS global simulado y el segundo
EKF. El NavSat raw procede de Gazebo en `/gps/fix_raw`, se normaliza a
`/gps/fix` con perfiles `ideal`, `f9p_rtk` o `m8n`, y conserva un datum fijo.
La misma autoridad de datum expone `/fromLL` para convertir coordenadas
geográficas al frame `map` de simulación.
Los gates estacionarios, brújula, servicios de datum y GNSS real no están
migrados todavía.

La IMU actual se deriva de la odometría de Gazebo para pruebas reproducibles;
no representa un modelo físico de ruido ni se usa con hardware real.

- Responsabilidad: odometría Ackermann, EKF local/global, datum y heading.
- No contiene: drivers, costmaps, planners ni modelo físico.
- Interfaces previstas: `map -> odom -> base_footprint` y odometrías tipadas.
- Estado: esqueleto sin ejecutables.
- Prueba: `colcon test --packages-select salus_localization`.
- Migración: caracterizar primero TF, datum fijo y gating de heading.
