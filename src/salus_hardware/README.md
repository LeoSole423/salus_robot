# salus_hardware

- Responsabilidad: adaptadores MAVROS, GNSS/RTK, RS16, cámara y transporte UART.
- No contiene: fusión, percepción, control de misión ni SDKs vendorizados.
- Interfaces previstas: sensores y telemetría normalizados.
- Estado: esqueleto sin ejecutables.
- Prueba: `colcon test --packages-select salus_hardware`.
- Migración: comenzar por contratos de entrada/salida, no por drivers legacy.

