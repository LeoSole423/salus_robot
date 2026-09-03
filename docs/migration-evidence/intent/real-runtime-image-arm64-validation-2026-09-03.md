# Validación de imagen física ARM64 — 2026-09-03

## Alcance

Se validó la receta explícita `Dockerfile.real` para el runtime físico de
`salus_robot`. El flujo de simulación (`Dockerfile` y `compose.yaml`) no fue
modificado funcionalmente. No se detuvo `ROS2_SALUS`, no se iniciaron nodos de
hardware ni se conectaron dispositivos físicos.

## Resultados

| Entorno | Resultado | Comprobaciones |
| --- | --- | --- |
| PC x86 | PASS | `./tools/build_real_image.sh`; imagen `salus-robot:humble-real`; arquitectura `amd64` |
| Jetson Orin | PASS | build nativo; arquitectura `arm64`; imagen `salus-robot:humble-real` |
| Workspace en Jetson | PASS | `colcon build --symlink-install` desde checkout limpio montado en la imagen; 14 paquetes |

En ambas arquitecturas se verificaron dentro de la imagen:

- `mavros` y `mavros_extras`;
- `rmw_cyclonedds_cpp`;
- `robot_localization`;
- `nav2_collision_monitor`, `nav2_controller`, `nav2_planner`,
  `nav2_bt_navigator`, `nav2_behaviors`, `nav2_costmap_2d`,
  `nav2_smac_planner` y `nav2_regulated_pure_pursuit_controller`;
- datasets GeographicLib (`egm96-5.wld` presente);
- usuario `ros` con grupos `dialout` y `tty`.

`ros_gz_sim`, `ros_gz_bridge`, `rviz2` y `nav2_rviz_plugins` no están
instalados. Para mantener esta frontera, la receta usa componentes Nav2
individuales: los metapaquetes `navigation2`/`nav2-bringup` arrastran plugins
RViz y por eso no forman parte de la imagen física.

## Integridad y límites

El build normal del workspace (`./tools/build.sh`) y la suite (`./tools/test.sh`)
pasaron en el contenedor de desarrollo: 14 paquetes, 752 tests, 0 errores, 0
fallos y 1 omitido. También pasaron la validación del repositorio, los tests
de selección de CI, `git diff --check` y el parseo YAML.

El build dentro de la imagen ARM64 fue sólo de compilación y comprobación de
paquetes; no se copió el workspace a la imagen, no se inició hardware y no se
modificó el servicio legacy activo de la Jetson. La imagen no constituye aún
un deployment persistente ni valida el cutover físico.
