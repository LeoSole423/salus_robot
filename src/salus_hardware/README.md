# salus_hardware

- Responsabilidad: adaptadores MAVROS, GNSS/RTK, RS16, cámara y transporte UART.
- No contiene: fusión, percepción, control de misión ni SDKs vendorizados.
- Interfaces previstas: sensores y telemetría normalizados; control PTZ,
  presets y estado de cámara bajo `/camara/*`.
- `pixhawk_sensor_adapter` consume las salidas MAVROS configuradas de IMU y
  GNSS y las publica con identidad estable en `/hardware/imu_primary/data` y
  `/hardware/gnss_primary/fix`. Es read-only: no inicia MAVROS, NTRIP, puertos,
  TF ni actuadores. Conserva `base_link`, timestamps, covarianzas y `NO_FIX`;
  otro frame requiere calibración física.
- `pixhawk_real.launch.py` es el owner físico parcial del MVP: inicia una sola
  instancia sensor-only de `mavros_node` con el FCU configurable (default
  `/dev/ttyACM0:921600`), remaps históricos de IMU/GNSS/velocidad/odometría y
  TF de MAVROS deshabilitado. No inicia NTRIP, RS16, UART, localización, Nav2
  ni actuadores. Debe ejecutarse únicamente tras retirar el owner legacy y
  validar el preflight físico.
- `pixhawk_rtk_delivery_real.launch.py` es el perfil físico aislado de entrega:
  inicia exactamente un `pixhawk_rtk_adapter`, con overrides explícitos para
  los endpoints dobles de MAVROS y entrega habilitada. No inicia MAVROS, FCU,
  NTRIP, RS16, UART, localización, heading, TF, Nav2 ni Cockpit.
- `legacy_rtk_observer` normaliza en modo read-only el estado JSON, el estado
  textual del receptor y exactamente un `/rtcm` legado de tipo
  `UInt8MultiArray`. Publica `RtcmFrame` validado y `GnssRtkStatus`, manteniendo
  separadas la frescura de correcciones y la calidad GNSS. No abre NTRIP ni
  entrega correcciones a MAVROS o USB.
- `ntrip_rtcm_source` es el owner de adquisición real aislado: lee un YAML
  local read-only, abre el caster seleccionado con HTTP/ICY y Basic auth,
  valida chunked/RTCM3/CRC24Q y publica únicamente `RtcmFrame` en
  `/salus/hardware/rtcm/corrections`. Su `GnssRtkStatus` sólo informa
  adquisición/frescura; deja calidad GNSS y entrega en `UNKNOWN`/`DISABLED`.
  `ntrip_rtcm_source_real.launch.py` requiere `config_path` y no inicia
  MAVROS, Pixhawk, RS16, UART, Nav2 ni delivery RTCM.
- `rtcm_dry_run_sink` valida la frontera canónica y publica únicamente
  contadores/edad JSON para diagnóstico; nunca registra el payload ni actúa
  sobre el receptor.
- `pixhawk_rtk_adapter` toma calidad exclusivamente de
  `mavros_msgs/GPSRAW`, publica el estado tipado final y puede convertir
  `RtcmFrame` a `mavros_msgs/RTCM`. La entrega requiere simultáneamente
  `delivery_backend=pixhawk_mavros` y `delivery_enabled=true`; por defecto no
  crea el publicador MAVROS. Rechaza CRC, tamaños mayores a 720 bytes,
  duplicados/regresiones y `direct_usb` mientras no exista su driver.
- `legacy_drive_measurement_node` es un adaptador estrictamente de lectura:
  consume el tipo wire legacy `interfaces/msg/DriveTelemetry` (por defecto
  `/controller/drive_telemetry`) y
  publica `TractionMeasurement` y `SteeringMeasurement` en
  `/vehicle/measurements/traction` y `/vehicle/measurements/steering`. No
  habilita actuadores, servicios ni un launch real. Sus parámetros de tópico y
  `source_id` son configurables; conserva el timestamp legado y marca la
  velocidad firmada deducida de `reverse_requested` como inferida tanto en
  avance como en reversa. `interfaces` es una excepción transitoria de
  coexistencia; las salidas siguen siendo contratos canónicos
  `salus_interfaces`. `input_wire_type=salus_interfaces` es el default de
  simulación; la coexistencia real fija explícitamente `interfaces`.
- `vehicle_kinematic_converter` transforma fuentes físicas seleccionadas en
  entradas cinemáticas mediante una escala de tracción y una curva polinómica
  de dirección explícitas. `calibration_validated` es `false` por defecto: sin
  validación publica `UNAVAILABLE` y ningún campo consumible. Filtra por
  `source_id`, conserva timestamp/secuencia y nunca selecciona fuentes,
  publica odometría ni tiene autoridad sobre actuadores.
- `capability_profile` publica `/system/capabilities` como snapshot latched del
  perfil elegido al arrancar. Además del eje de obstáculos, declara la única
  `imu_source` y `orientation_source` seleccionadas. Sus salidas lógicas pasan
  por `UNAVAILABLE`, `READY` y `STALE` según recepción/frescura, sin cambiar de
  fuente. El estado de obstáculos continúa siendo sólo declarativo.
- Estado: MAVROS y GeographicLib quedan disponibles de forma reproducible en
  la imagen, con configuración sensor-only y tests estructurales; la conexión
  al Pixhawk sigue pendiente de validación física. La cámara PTZ dispone de
  contratos, políticas puras, backends
  simulado/ISAPI, persistencia atómica y el ejecutable `camera_node`; pasó
  pruebas unitarias y el smoke WebSocket en simulación. La validación física
  continúa pendiente.
- Prueba: `colcon test --packages-select salus_hardware`.
- Migración: comenzar por contratos de entrada/salida, no por drivers legacy.
No debe habilitarse la entrega mientras el `rtk_bridge` legado siga publicando
`/mavros_node/send_rtcm`. Este paquete no inicia MAVROS, FCU ni control; la
adquisición NTRIP sólo se inicia mediante su launch real explícito.

## Replaceable battery adapters

`battery_backends.create_battery_backend` selects a read-only adapter implementing
`BatteryBackend.read() -> BatterySample` and `close()`. Add future battery models
here with their own pure parser and tests; the ROS/public consumers do not change.
`pylontech_us2000` supports the characterized single-module low-voltage protocol,
15 cells, standard/extended capacity fields and strict framing/checksums. SOC is
remaining Ah / total Ah; current is positive charging, negative discharging.
The battery USB adapter is independent of the ESP32 drive adapter.

Isolated read-only launch (no drive nodes):

```bash
ros2 launch salus_hardware battery_real.launch.py battery_serial_port:=/dev/ttyUSB1
```

Use an explicit stable device path where possible. CH340 requires host `ch341`;
container dependencies cannot supply a missing host driver. A kernel update may
require rebuilding the locally installed module. RJ45 wiring must match the exact
battery revision; characterized US2000 used pins 7=A, 8=B, 6=GND, 115200 8N1,
address 2. Do not auto-select the controller USB port.

| Node parameter | Type/default | Unit/range | Meaning |
| --- | --- | --- | --- |
| backend | string / pylontech_us2000 | registered name | Adapter selection; unknown names fail |
| serial_port | string / empty | explicit device path | Required battery adapter, no auto fallback |
| baud | int / 115200 | bit/s; 9600 or 115200 | Match active battery DIP setting |
| address | int / 2 | 0–255 | Protocol device address |
| timeout_s | double / 0.5 | s; (0,2] | Bounded serial read/write timeout |
| poll_hz | double / 1.0 | Hz; [0.1,2] | Poll rate; choose control stale timeout longer than poll interval |
| state_topic | string / /battery/backend_state | ROS topic | Private measured-state output |

`/battery/backend_state`: sensor_msgs/BatteryState, producer salus_battery,
consumer salus_controller external source, reliable/volatile depth 10. Node owns
serial while running and closes on exit. Failed reads publish nothing and retry
opening on next poll; control expires the last sample after 3 s. Unknown health, design capacity and individual-cell temperatures remain
unknown (the US2000 provides grouped temperature sensors). Only CID2 0x42 requests are emitted.

Stationary serial evidence on 2026-10-01: five checksum-valid frames captured in
`test/fixtures/pylontech_us2000_readings.json`. This is not ROS deployment or
mission hardware parity.
