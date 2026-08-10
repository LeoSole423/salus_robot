# salus_robot

Monorepo ROS 2 del robot terrestre autónomo SALUS.

> **Estado:** migración incremental. El corte de control/batería funciona en
> simulación, pero el sistema completo todavía no es operativo ni debe usarse
> para mover el vehículo.

## Objetivo

Reconstruir el stack de SALUS con límites claros entre hardware, localización,
percepción, navegación, control, operación web y simulación. Este repositorio
reemplazará gradualmente a `ROS2_SALUS`; el repositorio anterior es referencia,
no una fuente para copiar carpetas completas.

## Paquetes

| Paquete | Responsabilidad |
| --- | --- |
| `salus_interfaces` | Mensajes, servicios y acciones compartidos |
| `salus_description` | Xacro/URDF, frames físicos y RViz |
| `salus_hardware` | Adaptadores MAVROS, GNSS, LiDAR, cámara y UART |
| `salus_localization` | Odometría Ackermann, EKF, datum y heading |
| `salus_perception` | Conversión y filtrado de datos LiDAR |
| `salus_control` | Arbitraje final, actuación, telemetría y batería |
| `salus_navigation` | Nav2, rutas, patrulla, zonas y observabilidad |
| `salus_navigation_bt` | Plugins Behavior Tree propios |
| `salus_web` | Puente ROS/WebSocket y herramientas del operador |
| `salus_simulation` | Gazebo, mundos y sensores simulados |
| `salus_bringup` | Únicos launches completos del sistema |

Las dependencias permitidas están en [docs/architecture.md](docs/architecture.md)
y en [docs/package-map.yaml](docs/package-map.yaml).

## Requisitos

- Docker Engine con el plugin Compose.
- Linux con acceso a Docker.
- Para GUI: servidor X11 disponible.
- Opcional para instalación nativa: Ubuntu 22.04, ROS 2 Humble, `colcon` y
  `rosdep`.

## Instalación con Docker

```bash
git clone <URL_DEL_REPOSITORIO> salus_robot
cd salus_robot
./tools/up.sh
./tools/build.sh
./tools/test.sh
```

Abrir una shell ROS:

```bash
./tools/shell.sh
```

Los directorios `build/`, `install/` y `log/` se crean localmente y no se
versionan.

Smoke test del primer corte migrado:

```bash
./tools/smoke_control_sim.sh
```

## Instalación nativa opcional

```bash
source /opt/ros/humble/setup.bash
vcs import < dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

`dependencies.repos` está vacío en este hito; no se vendoriza ningún driver.

## Launches del esqueleto

```bash
ros2 launch salus_bringup real_skeleton.launch.py
ros2 launch salus_bringup sim_skeleton.launch.py
```

Solo muestran una advertencia y terminan. No son bringups funcionales.

## Flujo de desarrollo

1. Elegir una unidad del [mapa de migración](docs/migration-map.md).
2. Registrar cambios arquitectónicos en `docs/decisions/`.
3. Implementar dentro del paquete propietario, sin dependencias circulares.
4. Documentar interfaces públicas y agregar tests.
5. Ejecutar `./tools/test.sh` antes de integrar.

Para orientación rápida de agentes y colaboradores, leer [AGENTS.md](AGENTS.md).

## Licencia

MIT. Consultar [LICENSE](LICENSE).
