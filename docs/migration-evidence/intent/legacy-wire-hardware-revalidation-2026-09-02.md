# Revalidación física de las dos fronteras wire legacy

## Alcance

Revalidación estacionaria, de sólo lectura, de las dos fronteras que quedaron
bloqueadas por incompatibilidad de identidad wire en la validación anterior:

1. `/controller/drive_telemetry` -> mediciones canónicas.
2. `/cmd_vel_final` -> comando en sombra y diagnóstico.

- Destino probado: `salus_robot@436b1ef5e5562282caccb937554f55805167e447`
  (cabeza del PR #157, rama `agent/legacy-wire-type-compat`).
- Fuente legacy: `ROS2_SALUS/main@f35834989b041f51dd325c626d2338e2232d9e53`,
  checkout limpio, servicio `salus-real-global-v2-wifi.service` activo.
- Fecha: 2026-09-02 (UTC). Robot dentro del galpón, estacionario.
- No se repitió la caracterización de Pixhawk/GNSS/LiDAR; sólo se hizo un
  sanity check de que las señales legacy seguían vivas.
- No se registraron coordenadas GNSS, payloads RTCM, credenciales ni comandos.

## Resultado

| Frontera | Resultado |
| --- | --- |
| `interfaces/msg/DriveTelemetry` -> adaptador real -> `TractionMeasurement` -> `SteeringMeasurement` | **validado en hardware** |
| `interfaces/msg/CmdVelFinal` -> adaptador real -> `VehicleCommand` -> diagnóstico | **validado en hardware** |

Esto no valida localización, MAVROS propio, NTRIP propio, RS16 propio, Nav2
real, UART ni control físico.

## Método de coexistencia

El perfil se compiló y ejecutó en contenedores separados de la imagen ROS del
host, sin tocar el despliegue legacy:

- fuente montada de **sólo lectura** (`:ro`);
- `build`/`install` en un directorio temporal aparte, borrado al terminar;
- `Privileged=false`, `Devices=[]`, sin montaje de `/dev`;
- red `host`, `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  (idéntico al contenedor legacy);
- build inicial con red deshabilitada.

Se verificó la procedencia de los tipos: con sólo `/opt/ros/humble` y el
`install` de Salus en `AMENT_PREFIX_PATH`, `ros2 interface package interfaces`
resolvió `/reval/install/interfaces` y expuso exactamente `CmdVelFinal` y
`DriveTelemetry`. Es decir, Salus genera por sí mismo las dos identidades wire
legacy sin montar ni fuentear el workspace de `ROS2_SALUS`.

El selector `input_wire_type=interfaces` quedó explícito en los tres nodos del
launch real; no se dependió del default.

## Baseline antes de iniciar Salus

| Tópico / recurso | Observado |
| --- | --- |
| `/controller/drive_telemetry` | `interfaces/msg/DriveTelemetry`, 1 publicador (`vehicle_controller_server`), 6 suscripciones |
| `/cmd_vel_final` | `interfaces/msg/CmdVelFinal`, 1 publicador (`nav_command_server`), 2 suscripciones |
| `/mavros_node/send_rtcm` | `mavros_msgs/msg/RTCM`, 1 publicador (`rtk_bridge`), 1 suscriptor (`mavros_node`) |
| `/tf` | 21 entradas de nodo, todas legacy |
| UART `/dev/ttyUSB0` | un solo descriptor: `controller_serv` (PID 3865) |
| QoS de entrada | RELIABLE / KEEP_LAST(10) en ambas fronteras |

## Frontera 1: telemetría de conducción

Flujo observado con el perfil activo:

```text
/controller/drive_telemetry [interfaces/msg/DriveTelemetry]
  -> /salus/observation/legacy_drive_measurement_adapter
  -> /vehicle/measurements/traction [salus_interfaces/msg/TractionMeasurement]
  -> /vehicle/measurements/steering [salus_interfaces/msg/SteeringMeasurement]
```

- Suscripciones en el tópico de entrada: 6 legacy + 1 Salus
  (`legacy_drive_measurement_adapter`, namespace `/salus/observation`), con el
  mismo tipo completo `interfaces/msg/DriveTelemetry` y QoS RELIABLE.
- Tasa de salida: tracción ~10,0 Hz, dirección ~10,0 Hz (ventanas sucesivas
  9,99/10,10 y 9,98/10,03), sincronizada con la entrada legacy.
- Muestra representativa de `TractionMeasurement`:
  `stamp` no cero (heredado del mensaje legacy), `source_id=rear_traction_motor`,
  `status=1`, `sequence` monotónico (p. ej. 558),
  `available_fields=4`, `measured_fields=0`, `calculated_fields=0`,
  `inferred_fields=4`, `linear_velocity_mps=0.0`.
- Dirección: `source_id=front_steering_linkage`, tipo
  `salus_interfaces/msg/SteeringMeasurement`.
- Valores coherentes con robot detenido. No se validó calibración física en
  este corte; el objetivo era probar la conexión wire.
- El contenedor de Salus no vio `/dev/ttyUSB*` y `fuser` confirmó que el UART
  siguió exclusivamente en el proceso legacy.

## Frontera 2: comando en sombra

Flujo observado, sin inyectar ningún comando:

```text
/cmd_vel_final [interfaces/msg/CmdVelFinal]
  -> /salus/observation/legacy_vehicle_command_adapter
  -> /vehicle/command_shadow [salus_interfaces/msg/VehicleCommand]
  -> /salus/observation/vehicle_command_shadow_comparison
  -> /vehicle/command_shadow/diagnostics [diagnostic_msgs/msg/DiagnosticArray]
```

- Publicador de entrada: 1, `nav_command_server` (legacy).
- Suscripciones durante la sombra: 4 = 2 legacy (`vehicle_controller_server`,
  `nav_observability`) + 2 Salus (`legacy_vehicle_command_adapter`,
  `vehicle_command_shadow_comparison`).
- Tráfico natural del legacy, sin inyección: ~25-43 Hz medidos en ventanas de
  40 s; `/vehicle/command_shadow` ~30-35 Hz.
- Diagnóstico final tras ~4 minutos:
  `compared=8165`, `matched=8165`, `diverged=0`,
  `legacy_without_shadow=0`, `shadow_without_legacy=0`,
  `legacy_queue_dropped=0`, `shadow_queue_dropped=0`,
  `legacy_pending=0`, `shadow_pending=0`, `last_reasons=none`,
  `authoritative=false`, mensaje `shadow matches legacy translation`, nivel OK.
- No hubo divergencias, por lo que no se tocó ninguna tolerancia.
- Muestra de `VehicleCommand` en reposo: `source=2` (manual),
  `drive_enabled=true`, `emergency_stop=false`, `brake_ratio=0.0`,
  `speed=0.0`, `steering_angle=0.0`.

## Autoridades durante la sombra

- `/cmd_vel_final`: 1 solo publicador (legacy). Salus únicamente suscribió.
- `/controller/drive_telemetry`: 1 solo publicador (legacy).
- UART: descriptor único del legacy; sin montaje de `/dev` en Salus.
- RTCM: `/mavros_node/send_rtcm` siguió con 1 publicador (`rtk_bridge`) y 1
  suscriptor (`mavros_node`); Salus no añadió endpoints.
- Estado RTK canónico de Salus: `delivery_backend=0` y `delivery_state=0`
  (desactivado), `corrections_fresh=true`, `correction_age_s≈0,53`,
  `receiver_fix_type=-1`, `fix_quality=0` (degradación ambiental del galpón,
  no requerida en este corte). Dry-run: `received_count=1170`,
  `rejected_count=0`, `age_s≈0,25`, sin registro de payload.
- `/tf`: 21 entradas de nodo, ninguna de Salus.
- Sin MAVROS, NTRIP, RS16 ni serial nuevos: `ps` dentro del contenedor Salus
  no mostró ninguno de esos procesos ni ejecutables.

Sanity check de señales legacy durante la sombra: `/imu/data` ~10,0 Hz,
`/global_position/raw/fix` ~2,0 Hz, `/scan_3d` ~10,2 Hz,
`/controller/drive_telemetry` ~10,1 Hz.

## Cierre y rollback

- Retiro únicamente del perfil Salus mediante SIGINT al proceso de launch;
  todas las colas de nodos terminaron y el contenedor quedó `exited 0`, sin
  procesos huérfanos de Salus en el host.
- Después del cierre: `/cmd_vel_final` volvió a 2 suscripciones,
  `/controller/drive_telemetry` a 6, `/mavros_node/send_rtcm` a 1/1, `/tf` con
  los mismos publicadores legacy, UART con el mismo descriptor legacy, y los
  tópicos de Salus desaparecieron.
- `ROS2_SALUS` nunca se reinició ni se modificó: servicio activo durante toda
  la prueba, contenedor `ros2_salus` con el mismo uptime, checkout en
  `f358349` limpio, sin cambios en systemd ni en el despliegue persistente.
- No hubo movimiento: velocidad medida 0,0 en toda la ventana y ningún comando
  o meta publicado desde Salus.
- Los contenedores temporales y el directorio de build se eliminaron; el
  checkout auxiliar de Salus en la Jetson se devolvió a su estado previo.

## Anomalías registradas

1. **Doble `rclpy.shutdown()` en el camino de salida por SIGINT.** Los nodos
   `legacy_drive_measurement_node`, `legacy_vehicle_command_node` y
   `vehicle_command_comparison_node` terminaron con código 1 lanzando
   `RCLError: failed to shutdown: rcl_shutdown already called on the given
   context`, mientras que los cinco nodos restantes terminaron limpiamente.
   El cierre funcional es correcto (procesos desaparecidos, sin huérfanos,
   contenedor `exited 0`), pero el código de salida y la traza son ruido real
   de observabilidad. Queda fuera de este PR por la exclusión de "arreglo
   general de shutdown"; corresponde a un issue propio.
2. **`/scan_clean` sin tasa medible.** Con `scan_noise_filter` vivo, 1
   publicador y 5 suscriptores legacy, `ros2 topic hz` no devolvió tasa durante
   la sombra **ni después del cierre de Salus**, por lo que se atribuye a una
   condición preexistente del stack legacy y no a una regresión del perfil de
   observación. Requiere seguimiento aparte.
3. **Artefacto de la prueba, no del sistema.** Los dos primeros intentos de
   cierre enviaron SIGINT al wrapper `bash | tee` del contenedor (PID 1) en vez
   del proceso de launch, por lo que los nodos seguían vivos hasta dirigir la
   señal al PID correcto. No es un defecto de `real_observation.launch.py`.

## Referencias

- Ficha de intención del puente:
  [`legacy-wire-type-compatibility.md`](legacy-wire-type-compatibility.md).
- Validación anterior que detectó el gap:
  [`real-observation-hardware-validation-2026-09-02.md`](real-observation-hardware-validation-2026-09-02.md).
- Issue paraguas: #153. PR: #157.
