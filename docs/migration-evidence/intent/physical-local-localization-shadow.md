# Ficha de intención — local EKF físico en shadow (#161)

## Hechos observados

- La validación física de #157 cerró las dos fronteras wire legacy
  (`interfaces/msg/DriveTelemetry`, `interfaces/msg/CmdVelFinal`) y confirmó
  que el perfil `real_observation.launch.py` convive sin autoridad.
- El perfil de observación ya publica una IMU lógica propia en
  `/salus/imu/data`, derivada del `/imu/data` de MAVROS mediante
  `pixhawk_sensor_adapter` + `imu_selector`.
- El stack legacy sigue produciendo `/wheel/odometry` (Ackermann derivada de la
  telemetría del ESP32) y su propio EKF local en `/odometry/local`, que es la
  única autoridad de `odom -> base_footprint`.
- La calibración mecánica física sigue pendiente: el cuenta-vueltas del motor
  no equivale a velocidad sobre el suelo, y el AS5600 más el vínculo no
  equivale al ángulo efectivo de la rueda central virtual. Por eso la
  odometría canónica (`kinematic_conversion` /
  `kinematic_ackermann_odometry`) **no** se conecta todavía.

## Decisión de diseño

Separar el problema del estimador del problema de calibración.

```text
ROS2_SALUS/MAVROS /imu/data
    -> real_observation Pixhawk adapter
    -> /salus/imu/data ─────────┐
                                ├──> salus_local_ekf_shadow
legacy /wheel/odometry ─────────┘            |
                                             v
                        /salus/localization_shadow/odometry/local

legacy (intacto, en paralelo):
  /wheel/odometry + /imu/data -> EKF legacy -> /odometry/local -> odom->base_footprint
```

El shadow reutiliza la odometría de rueda legacy precisamente para que la
única variable nueva sea el estimador Salus y su IMU lógica. Si el shadow y el
legacy divergen, la divergencia es del estimador, no de una calibración aún no
validada.

## Contratos

| Elemento | Valor | Productor | Consumidor |
| --- | --- | --- | --- |
| `/wheel/odometry` | `nav_msgs/msg/Odometry` | `ROS2_SALUS` (autoridad) | EKF shadow (sólo lectura) |
| `/salus/imu/data` | `sensor_msgs/msg/Imu` | `salus_local_ekf_shadow` inputs vía `real_observation` | EKF shadow |
| `/salus/localization_shadow/odometry/local` | `nav_msgs/msg/Odometry` | `salus_local_ekf_shadow` | herramienta de observación / Cockpit futuro |
| `/salus/localization_shadow/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `salus_local_ekf_shadow` | diagnóstico |
| `/odometry/local` | `nav_msgs/msg/Odometry` | EKF legacy (única autoridad) | sin cambios |

Nuevos archivos, todos sin autoridad:

- `src/salus_localization/config/localization_local_real_shadow.yaml`
- `src/salus_localization/launch/localization_real_shadow.launch.py`
- `src/salus_bringup/launch/real_localization_shadow.launch.py`
- `tools/observe_localization_shadow.py` (sólo suscriptor)

## Semejanzas y diferencias con el perfil simulado

El modelo local ya portado se conserva sin retoque: `frequency 30`,
`sensor_timeout 0.2`, `two_d_mode`, `world_frame odom`, y los mismos masks de
`odom0`/`imu0` que usa la simulación. Diferencias deliberadas:

- `publish_tf: false`, `use_control: false`, `publish_acceleration: false`
  (en simulación el EKF sí es autoridad);
- `imu0` apunta a `/salus/imu/data` en lugar de `/imu/data`;
- nombre de nodo propio (`salus_local_ekf_shadow`), sin colisión con
  `ekf_filter_node_local`;
- salida y diagnósticos remapeados a un namespace de shadow;
- el launch no declara ningún argumento: no existe forma de convertirlo en
  autoridad desde la línea de comandos.

No se modificó process noise, covarianzas, umbrales de rechazo ni máscaras. No
hubo evidencia concreta que lo justificara.

## Límite importante sobre `/tf`

`robot_localization` construye su `TransformBroadcaster` de forma
incondicional, por lo que el nodo **anuncia un endpoint `/tf`** incluso con
`publish_tf=false`. La propiedad que se garantiza es de payload: nunca emite
transforms. Por eso el test de runtime se suscribe a `/tf` y exige stream vacío
en lugar de mirar sólo el grafo, y por eso la validación en la Jetson debe
contar transforms reales, no sólo contar endpoints.

## Fuera de alcance de este corte

- Odometría física canónica y su calibración mecánica.
- EKF global, `navsat_transform`, `gps_course_heading`, heading externo,
  medición GPS absoluta en `map` y `map -> odom`.
- cualquier transferencia de autoridad TF, cambio de tuning o control.

## Validación

Local, sin hardware:

- tests estructurales en `salus_localization` y `salus_bringup` que fallan si
  aparece un segundo EKF, si `publish_tf`/`use_control` dejan de ser `false`,
  si se toca el namespace de salida o si se cuela un owner global/de hardware
  (verificado por mutación: activar `publish_tf` o borrar el override hace
  fallar los tests);
- test de runtime con el `ekf_node` real y ROS en vivo: entradas sintéticas
  tipadas en `/wheel/odometry` y `/salus/imu/data`, verificación de salida,
  frames, monotonía, finitud, continuidad, y ausencia de transforms en `/tf` y
  de cualquier publisher en `/odometry/local`.

Medido en el contenedor Humble (sin hardware, dominio DDS aislado):

| Observación | Resultado |
| --- | --- |
| salida del shadow alimentado | 240 msgs en 8 s, **30.0 Hz** exactos |
| frames del shadow | `frame_id=odom`, `child_frame_id=base_footprint` |
| stamps | monótonos, sin mensajes no finitos |
| `/odometry/local` | 0 publishers (nadie ocupa la salida legacy) |
| diagnósticos | único publisher en `/salus/localization_shadow/diagnostics`; `/diagnostics` global sin publishers del shadow |
| `/tf` con el shadow corriendo | 241 transforms, 30.12 Hz, **una sola pareja** `odom -> base_footprint` (el emisor legacy sintético) |
| `/tf` con un segundo emisor inyectado | 480 transforms, 60.24 Hz → el detector de fuga detecta el doble |
| wrapper `real_localization_shadow.launch.py` | 9 nodos Salus + listener del EKF, apagado completo en ~6 s |

El control con emisor inyectado es deliberado: demuestra que la medición de TF
por payload no es vacuamente cierta. En la Jetson la comprobación equivalente
será que la tasa y las parejas de `/tf` **no cambian** entre baseline y durante,
no que el nodo desaparezca del grafo (el endpoint `/tf` se anuncia siempre).

El recolector `tools/observe_localization_shadow.py` quedó verificado como
sólo-lectura: sus únicos publishers son los implícitos `/rosout` y
`/parameter_events` que rclpy crea para cualquier nodo.

## Hardware

Pendiente al abrir este corte. La validación física estacionaria debe registrar
baseline y después/durante: tasa del shadow (~30 Hz esperados), `frame_id`/
`child_frame_id`, monotonía, finitud, coherencia con robot detenido, deltas
legacy/shadow sin retune, CPU/RSS del EKF, y que `/odometry/local` y `/tf`
conservan exactamente la autoridad legacy. La espera de 30 Hz es orientativa:
si el consumo de las entradas reales cambia la tasa del EKF, se registra como
hallazgo y no se retunea para disimularlo.

No se declaró nada `hardware_validated` en esta ficha hasta ejecutarla.

