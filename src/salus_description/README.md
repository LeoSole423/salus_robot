# salus_description

- Responsabilidad: modelo físico canónico, Xacro/URDF, frames y RViz.
- No contiene: drivers, algoritmos de localización ni mundos de simulación.
- Interfaces previstas: `robot_description` y TF físicos estáticos.
- Estado: esqueleto; no existe todavía un URDF operativo.
- Prueba: `colcon test --packages-select salus_description`.
- Migración: consolidar los modelos históricos y validar medidas/calibraciones.

