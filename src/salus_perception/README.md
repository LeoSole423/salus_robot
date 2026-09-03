# salus_perception

Estado: portado en simulación, sin paridad RS16 aún.

La ruta operativa conserva la nube 3D localmente:
`/scan_3d_raw -> /scan_3d -> /obstacles_cloud -> /scan -> /scan_clean`.
`/scan_clean` es la única entrada de `collision_monitor`. El nodo
`scan_preview` genera `/scan_preview` desde ese scan a 2 Hz, con FOV ±90°,
stride 4 y alcance máximo 12 m. Es una salida diagnóstica remota: nunca será
entrada de seguridad, Nav2 ni evidencia 3D.

`/scan_3d_raw` preserva la entrada del bridge para diagnóstico y rosbag. En
Gazebo Fortress puede llevar un frame interno no resoluble por RViz; por eso el
display `Raw 3D` queda apagado por defecto. `/scan_3d` normaliza ese frame a
`lidar_link` y es la primera nube visible y canónica.

Ejecutar: `ros2 launch salus_perception lidar_sim.launch.py`.
Para registrar un bag externo: `python3 tools/replay_lidar_report.py /ruta/al/bag`.

El perfil real aislado se ejecuta con
`ros2 launch salus_perception perception_real.launch.py`. Consume la nube
`/scan_3d` del owner RS16 en `lidar_link` y compone exactamente
`scan_ground_filter -> pointcloud_to_laserscan_node -> scan_noise_filter`,
produciendo `/obstacles_cloud`, `/scan` y `/scan_clean`. No inicia el driver,
TF, percepción adicional, safety ni Nav2; el TF debe pertenecer a la
composición externa.

- Responsabilidad: conversión, filtrado y validación de percepción LiDAR.
- No contiene: drivers RS16, costmaps, planners ni la UI remota.
- Interfaces previstas: nube normalizada, scan de navegación y diagnóstico.
- Estado: pipeline 3D simulado y preview 2D diagnóstico portados; falta replay RS16.
- Prueba: `colcon test --packages-select salus_perception`.
- Migración: portar primero el pipeline conservador; experimentos quedan fuera.
