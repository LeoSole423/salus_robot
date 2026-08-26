# salus_hardware

- Responsabilidad: adaptadores MAVROS, GNSS/RTK, RS16, cámara y transporte UART.
- No contiene: fusión, percepción, control de misión ni SDKs vendorizados.
- Interfaces previstas: sensores y telemetría normalizados; control PTZ,
  presets y estado de cámara bajo `/camara/*`.
- `legacy_drive_measurement_node` es un adaptador estrictamente de lectura:
  consume `DriveTelemetry` (por defecto `/controller/drive_telemetry`) y
  publica `TractionMeasurement` y `SteeringMeasurement` en
  `/vehicle/measurements/traction` y `/vehicle/measurements/steering`. No
  habilita actuadores, servicios ni un launch real. Sus parámetros de tópico y
  `source_id` son configurables; conserva el timestamp legado y marca la
  velocidad firmada deducida de `reverse_requested` como inferida tanto en
  avance como en reversa.
- Estado: cámara PTZ dispone de contratos, políticas puras, backends
  simulado/ISAPI, persistencia atómica y el ejecutable `camera_node`; pasó
  pruebas unitarias y el smoke WebSocket en simulación. La validación física
  continúa pendiente.
- Prueba: `colcon test --packages-select salus_hardware`.
- Migración: comenzar por contratos de entrada/salida, no por drivers legacy.
