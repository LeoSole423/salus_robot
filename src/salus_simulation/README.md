# salus_simulation

- Responsabilidad: Gazebo, mundos, bridges y normalizadores simulados.
- No contiene: algoritmos que también necesita el robot real.
- Interfaces previstas: mismos contratos normalizados que los backends reales.
- Estado: esqueleto; no existe un mundo ejecutable.
- Prueba: `colcon test --packages-select salus_simulation`.
- Migración: portar un escenario mínimo después del modelo canónico.

