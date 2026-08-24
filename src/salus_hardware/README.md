# salus_hardware

- Responsabilidad: adaptadores MAVROS, GNSS/RTK, RS16, cámara y transporte UART.
- No contiene: fusión, percepción, control de misión ni SDKs vendorizados.
- Interfaces previstas: sensores y telemetría normalizados; control PTZ,
  presets y estado de cámara bajo `/camara/*`.
- Estado: cámara PTZ caracterizada en
  `docs/migration-evidence/intent/camera-ptz-presets.md`; todavía sin
  interfaces ni ejecutables.
- Prueba: `colcon test --packages-select salus_hardware`.
- Migración: comenzar por contratos de entrada/salida, no por drivers legacy.
