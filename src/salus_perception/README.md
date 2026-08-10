# salus_perception

- Responsabilidad: conversión, filtrado y validación de percepción LiDAR.
- No contiene: drivers RS16, costmaps, planners ni visualización remota.
- Interfaces previstas: nube normalizada, scan de navegación y diagnóstico.
- Estado: esqueleto sin ejecutables.
- Prueba: `colcon test --packages-select salus_perception`.
- Migración: portar primero el pipeline conservador; experimentos quedan fuera.

