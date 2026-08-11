# salus_perception

Estado: portado en simulación, sin paridad RS16 aún.

La ruta operativa conserva la nube 3D localmente:
`/scan_3d_raw -> /scan_3d -> /obstacles_cloud -> /scan -> /scan_clean`.
`/scan_clean` es la única entrada de `collision_monitor`; una vista compacta para
Cockpit se incorporará después como `/scan_preview` y nunca será evidencia 3D.

Ejecutar: `ros2 launch salus_perception lidar_sim.launch.py`.
Para registrar un bag externo: `python3 tools/replay_lidar_report.py /ruta/al/bag`.

- Responsabilidad: conversión, filtrado y validación de percepción LiDAR.
- No contiene: drivers RS16, costmaps, planners ni visualización remota.
- Interfaces previstas: nube normalizada, scan de navegación y diagnóstico.
- Estado: esqueleto sin ejecutables.
- Prueba: `colcon test --packages-select salus_perception`.
- Migración: portar primero el pipeline conservador; experimentos quedan fuera.
