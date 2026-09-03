# #179 — localización local real autoritativa

## Alcance

Este corte implementa únicamente el perfil software local real especificado
por #179: `DriveTelemetry` → `ackermann_odometry` → `/wheel/odometry` y
`/vehicle/twist` → EKF local → `/odometry/local` y `odom → base_footprint`.

El punto de partida fue `main` en `00a35a07fe6a5cd4226ae1aea4122a61922c0294`.
El nodo compatible existente se conserva; el perfil fija el wheelbase legacy
de `0.94`, la inversión de signo medida y las covarianzas congeladas en #179.

## Evidencia ejecutable

- Los tests estructurales fijan los dos nodos exactos, topics, máscaras EKF,
  flags de autoridad, ausencia de global/Nav2/UART/hardware y parámetros de
  `ackermann_odometry`.
- El runtime sintético publica mensajes ROS tipados `DriveTelemetry` e `Imu` y
  observa `/wheel/odometry`, `/odometry/local` y `/tf`.
- El runtime exige muestras frescas, finitas y monótonas; verifica que el único
  par TF sea `odom → base_footprint` y que el publisher sea `salus_local_ekf`.
- La prueba de datos stale/invalidos exige que la odometría de rueda mantenga
  la pose y publique twist cero, y que los datos inválidos no produzcan nuevas
  muestras.

## Límites

No se conectó Jetson, UART, sensores físicos, GNSS/NTRIP, heading global,
perception, Nav2 ni ningún owner de hardware. Esta evidencia no declara
validación física ni paridad en movimiento.
