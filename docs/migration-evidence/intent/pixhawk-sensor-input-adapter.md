# Adaptador de entradas Pixhawk/MAVROS

## Alcance

Este corte convierte la caracterización pasiva de la Jetson en una ruta de
entrada ejecutable y estrictamente read-only. No inicia MAVROS, NTRIP, UART,
control, TF ni actuadores. Pixhawk permanece como backend soportado de forma
permanente.

## Contratos

| Etapa | Entrada | Salida | Tipo y QoS |
| --- | --- | --- | --- |
| adaptador IMU | `/imu/data` legacy configurable | `/hardware/imu_primary/data` | `sensor_msgs/Imu`, sensor data |
| selector IMU | fuente física elegida | `/salus/imu/data` en coexistencia | `sensor_msgs/Imu`, reliable depth 10 |
| adaptador GNSS | `/global_position/raw/fix` configurable | `/hardware/gnss_primary/fix` | `sensor_msgs/NavSatFix`, sensor data |
| selector GNSS | fuente física elegida | `/salus/gps/fix` en coexistencia | `sensor_msgs/NavSatFix`, reliable depth 10 |

Los nombres `/salus/*` evitan crear una segunda autoridad sobre los tópicos
lógicos mientras `ROS2_SALUS` sigue operativo. Cuando el bringup real nuevo
sea propietario de MAVROS, esos outputs se configurarán como `/imu/data` y
`/gps/fix`, y las entradas MAVROS tendrán nombres raw separados.

## Reglas

- Se conserva `base_link`, porque es el frame observado y todavía no existe
  una calibración de mount que autorice otro TF.
- Timestamp, mensaje y covarianzas se copian sin modificar.
- Muestras con frame, timestamp o valores estructurales inválidos se rechazan.
- `NavSatStatus.STATUS_NO_FIX` se publica honestamente incluso si la posición
  es desconocida/NaN; recibir RTCM no fabrica un fix.
- Cada selector se suscribe sólo a la fuente configurada. No existe `auto` ni
  fallback implícito entre Pixhawk y futuros dispositivos directos.
- El launch parcial no posee puertos, servicios, TF ni comandos de movimiento.

## Evidencia y límites

Las políticas y nodos tienen tests de validez, no-fix, monotonía, identidad de
fuente, copia de mensajes y ausencia de actuación en el launch. La observación
previa verificó los tipos, frames y QoS reales.

El 2026-08-28 se ejecutaron temporalmente los tres nodos nuevos dentro del
contenedor `ros2_salus`, junto al stack legacy y sin iniciar el launch nuevo.
Se observaron un productor único en cada output, `/salus/imu/data` a ~10 Hz,
`/salus/gps/fix` a ~2 Hz, frame `base_link` intacto y `NavSatStatus` propagado.
La terminación por `SIGINT` fue limpia. No se publicó control, TF ni RTCM y se
eliminaron código/logs temporales. Esto valida en hardware exclusivamente la
frontera de entradas Pixhawk; no valida el bringup real, localización completa
ni actuación de `salus_robot`.
