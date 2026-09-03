# salus_description

- Responsabilidad: modelo físico canónico, Xacro/URDF, frames y RViz.
- `description_real.launch.py` es el owner mínimo del TF físico estático para
  el preflight real: inicia únicamente `robot_state_publisher` con el Xacro
  canónico (`use_sim:=false`, `use_sim_time=false`).
- No contiene: drivers, algoritmos de localización ni mundos de simulación.
- Interfaces vigentes: `robot_description` y TF físicos desde
  `base_footprint` a chasis, ruedas y mounts de sensores.
- Estado: Xacro canónico migrado desde `cuatri_real_v2`; sus medidas requieren
  validación final sobre hardware.
- Prueba: `colcon test --packages-select salus_description`; el runtime real
  del paquete no inicia hardware, localización, navegación ni movimiento.
- Migración: la geometría física se conserva aquí; los plugins de Gazebo viven
  en un componente Xacro separado y los sensores simulados aún no se migraron.
