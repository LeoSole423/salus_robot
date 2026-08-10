# salus_control

- Responsabilidad: arbitraje final, actuación, telemetría y estimación de batería.
- No contiene: decisiones de misión, UI ni drivers de sensores.
- Interfaces previstas: comando final, drive telemetry, SOC y guardia de batería.
- Estado: esqueleto sin ejecutables.
- Prueba: `colcon test --packages-select salus_control`.
- Migración: portar lógica pura y backend sim antes del transporte UART real.

