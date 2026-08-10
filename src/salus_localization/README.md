# salus_localization

- Responsabilidad: odometría Ackermann, EKF local/global, datum y heading.
- No contiene: drivers, costmaps, planners ni modelo físico.
- Interfaces previstas: `map -> odom -> base_footprint` y odometrías tipadas.
- Estado: esqueleto sin ejecutables.
- Prueba: `colcon test --packages-select salus_localization`.
- Migración: caracterizar primero TF, datum fijo y gating de heading.

