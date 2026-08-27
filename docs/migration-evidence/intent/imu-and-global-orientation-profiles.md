# Intención: perfiles explícitos de IMU y orientación global

## Alcance

Este corte completa la parte de IMU/orientación de PR 5 del plan de fronteras
de hardware. Introduce selección y simulación; no añade drivers reales, fusión
multi-IMU ni validación física.

## Hechos caracterizados

- En el stack histórico, `gps_course_heading` fue añadido como fuente opcional
  `imu1` del EKF global. Los commits `52af057`, `8f49209` y `9dd7ecc` añadieron
  gating RTK/movimiento y un hold interno de 0.8 s para atravesar curvas leves.
- Durante el port a `salus_robot`, `/gps/course_heading` quedó publicado pero
  sin consumidor. El EKF global recibía yaw absoluto únicamente desde
  `/odometry/local_yaw_hold`, derivado de la odometría local.
- La simulación actual deriva una única IMU de `/odom_raw`; no modela ruido ni
  una segunda IMU física. El robot canónico sólo tenía el frame `imu_link`.
- `navsat_transform` no tenía remapeo explícito de orientación y podía consumir
  `/imu/data`, independientemente del heading global pretendido.

## Decisión

La selección de movimiento local y la orientación global son ejes separados:

| Eje | Valores | Salida lógica | Ausencia de la elegida |
| --- | --- | --- | --- |
| IMU local | `imu_primary`, `imu_secondary` | `/imu/data` | no publica; no conmuta |
| orientación global | `course_over_ground`, `external_heading` | `/localization/orientation` | no publica; no conmuta |

Las fuentes conservan identidad por tópico y frame. En simulación la IMU
existente se normaliza como `/hardware/imu_primary/data` en
`imu_primary_link`. No se publica una secundaria ficticia. El selector se
suscribe sólo al tópico elegido, valida frame, timestamp, finitud y orientación
y exige timestamps crecientes.

El selector global se suscribe sólo a `/gps/course_heading` o
`/heading/external`. El EKF global usa esa salida únicamente para yaw absoluto,
y `navsat_transform` recibe la misma autoridad. `/imu/data_global` conserva
exclusivamente yaw-rate. `/odometry/local_yaw_hold` puede seguir existiendo
como diagnóstico, pero deja de alimentar el EKF: no se fabricará rumbo al
desaparecer la fuente seleccionada.

`course_over_ground` conserva el hold de 0.8 s dentro del mismo estimador
histórico. Esto es continuidad temporal de una fuente, no conmutación de
método; queda visible en `/gps/course_heading/debug` y debe reevaluarse con bag
real. `external_heading` usa en simulación un fixture derivado de ground truth,
marcado explícitamente como no representativo de doble antena.

## Contratos

| Tópico | Tipo | Productor | Consumidor | QoS |
| --- | --- | --- | --- | --- |
| `/hardware/imu_primary/data` | `sensor_msgs/Imu` | normalizador sim / adaptador futuro | selector IMU | sensor data |
| `/hardware/imu_secondary/data` | `sensor_msgs/Imu` | adaptador futuro | selector IMU | sensor data |
| `/imu/data` | `sensor_msgs/Imu` | selector IMU | EKF local y gates | reliable, volatile, depth 10 |
| `/gps/course_heading` | `sensor_msgs/Imu` | estimador por movimiento | selector orientación | sensor data |
| `/heading/external` | `sensor_msgs/Imu` | fixture sim / adaptador futuro | selector orientación | sensor data |
| `/localization/orientation` | `sensor_msgs/Imu` | selector orientación | EKF global y `navsat_transform` | reliable, volatile, depth 10 |

`SystemCapabilities` declara `local_motion_imu` y `global_orientation` con el
único `source_id` seleccionado. Empiezan `UNAVAILABLE`, pasan a `READY` al
observar la salida lógica y a `STALE` al vencer; el monitor nunca modifica la
selección.

## Evidencia y límites

- Unitarios cubren valores de perfil inválidos, fuente no elegida, frame,
  timestamp, cuaternión/covarianza y monotonía.
- El smoke verifica por grafo ROS que cada selector tenga una sola entrada y
  que el estimador de curso no se inicie en el perfil externo.
- `hardware_validated: false`: no se observaron Pixhawk, PCB IMU ni GNSS de
  doble antena. No autoriza movimiento ni reemplaza el stack real.
