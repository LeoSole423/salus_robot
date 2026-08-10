# salus_description

- Responsabilidad: modelo físico canónico, Xacro/URDF, frames y RViz.
- No contiene: drivers, algoritmos de localización ni mundos de simulación.
- Interfaces vigentes: `robot_description` y TF físicos desde
  `base_footprint` a chasis, ruedas y mounts de sensores.
- Estado: Xacro canónico migrado desde `cuatri_real_v2`; sus medidas requieren
  validación final sobre hardware.
- Prueba: `colcon test --packages-select salus_description`.
- Migración: la geometría física se conserva aquí; los plugins de Gazebo viven
  en un componente Xacro separado y los sensores simulados aún no se migraron.
