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

La IMU actual se deriva de la odometría de Gazebo para pruebas reproducibles;
no representa un modelo físico de ruido ni se usa con hardware real.

- Responsabilidad: odometría Ackermann, EKF local/global, datum y heading.
- No contiene: drivers, costmaps, planners ni modelo físico.
- Interfaces previstas: `map -> odom -> base_footprint` y odometrías tipadas.
- Estado: esqueleto sin ejecutables.
- Prueba: `colcon test --packages-select salus_localization`.
- Migración: caracterizar primero TF, datum fijo y gating de heading.
