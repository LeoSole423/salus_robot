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
- `legacy_drive_measurement_node` es un adaptador estrictamente de lectura:
  consume `DriveTelemetry` (por defecto `/controller/drive_telemetry`) y
  publica `TractionMeasurement` y `SteeringMeasurement` en
  `/vehicle/measurements/traction` y `/vehicle/measurements/steering`. No
  habilita actuadores, servicios ni un launch real. Sus parámetros de tópico y
  `source_id` son configurables; conserva el timestamp legado y marca la
  velocidad firmada deducida de `reverse_requested` como inferida tanto en
  avance como en reversa.
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
- Estado: cámara PTZ dispone de contratos, políticas puras, backends
  simulado/ISAPI, persistencia atómica y el ejecutable `camera_node`; pasó
  pruebas unitarias y el smoke WebSocket en simulación. La validación física
  continúa pendiente.
- Prueba: `colcon test --packages-select salus_hardware`.
- Migración: comenzar por contratos de entrada/salida, no por drivers legacy.
