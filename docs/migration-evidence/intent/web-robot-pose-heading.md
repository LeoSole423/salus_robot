# Intención: pose dinámica del robot en Cockpit

## Hechos observados

- El perfil web `compact` agrega `robot_pose` dentro de `nav_telemetry` a una
  cadencia máxima de 2 Hz.
- `/gps/fix` es la autoridad de latitud y longitud que consume el bridge.
- `/odometry/local` publica `nav_msgs/Odometry` en la cadena
  `odom -> base_footprint` y contiene la orientación filtrada usada por Nav2.
- Cockpit necesita posición y orientación durante patrulla, no sólo en el
  estado inicial o en el perfil web `full`.

## Comportamiento elegido

`salus_web` combina la posición GPS más reciente con el yaw ROS de
`/odometry/local`. La pose agregada mantiene el contrato existente:

```json
{"robot_pose": {"lat": -31.0, "lon": -64.0, "heading_deg": 45.0}}
```

La orientación no cambia la autoridad TF ni participa en control o seguridad;
es exclusivamente telemetría de operador. Un cuaternión no finito o degenerado
se descarta. Antes del primer fix GPS no se publica una pose parcial. Si GPS se
actualiza entre muestras de odometría, conserva el último heading válido.

El tópico se configura con `heading_odometry_topic` (string, default
`/odometry/local`, unidad no aplicable). Productor: EKF local. Consumidor:
`salus_web`. Tipo: `nav_msgs/msg/Odometry`. QoS del consumidor: sensor-data
best-effort, volatile. La ausencia del tópico degrada a posición sin heading.

## Evidencia y límite

La simulación operacional permite verificar movimiento y giro del marcador en
Cockpit. Esto no constituye validación del heading ni del GNSS en hardware
real; esa evidencia continúa pendiente de banco, bag o robot.
