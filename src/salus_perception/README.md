# salus_perception

Estado: filtro radial histórico migrado; falta validación de comportamiento en Jetson.

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

## Diagnóstico visual en RViz

`lidar_diagnostics.rviz` usa `odom` como `Fixed Frame` y conserva las nubes,
el scan, TF, odometrías y costmaps. Es una elección de visualización para
percepción local: durante un giro, la localización global puede corregir
`map -> odom`; anclar RViz en `map` hace que esas correcciones desplacen o roten
en pantalla datos que son estables en `odom`. El perfil en `odom` evita que ese
efecto se confunda con una estela del LiDAR o una celda adherida al robot.

Esto no cambia los frames de Nav2 ni sustituye `map` para rutas, GPS u objetivos
globales. Si se necesita investigar la localización global, se puede seleccionar
`map` temporalmente desde RViz y observar explícitamente el transformador
`map -> odom`.

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
- Estado: pipeline 3D y segmentación radial del filtro histórico portados; falta validación de comportamiento en Jetson.
- Prueba: `colcon test --packages-select salus_perception`.
- Migración: la clasificación agrupa puntos por azimut y radio y evalúa pendientes local/global; perfiles urban/rural 10°/13°/0,20 m y 15°/18°/0,25 m. El cambio no modifica el stop por nube vencida.
