# salus_robot

Monorepo ROS 2 del robot terrestre autónomo SALUS.

> **Estado:** migración incremental. El corte de control/batería funciona en
> simulación. También existe un robot Ackermann mínimo en Gazebo Fortress y
> localización local simulada;
> el sistema completo todavía no es operativo ni debe usarse para mover el
> vehículo.

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
./tools/smoke_motion_sim.sh
./tools/smoke_localization_sim.sh
./tools/smoke_navigation_snapshot.sh
./tools/smoke_integration_sim.sh
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

## Checkpoint integrado de simulación

Después del build, el stack migrado hasta ahora se inicia con:

```bash
ros2 launch salus_bringup integration_sim.launch.py
```

Para abrir Gazebo y RViz:

```bash
./tools/sim.sh
```

El helper construye incrementalmente, levanta Docker, carga ROS y ejecuta el
bringup. Para ejecutarlo sin ventanas: `./tools/sim.sh --headless`.

Para probar el bridge completo de Cockpit en simulación:

```bash
./tools/sim.sh --cockpit
cd ../cockpit
npm run dev
```

Elegir **Simulation** y conectar a `localhost:8766`. Este modo activa rutas,
patrulla, snapshots y el bridge WebSocket; no representa un bringup final ni
habilita hardware real.

En otra terminal, los comandos manuales usan el mismo contenedor:

```bash
./tools/cmd_vel_sim.sh straight  # mantener activo; Ctrl+C para detener
./tools/cmd_vel_sim.sh left
./tools/cmd_vel_sim.sh right
./tools/cmd_vel_sim.sh brake
```

`./tools/shell.sh` abre una shell ROS en ese mismo contenedor para inspección
manual de tópicos y TF.

Incluye movimiento, control simulado, localización local/global y LiDAR. Es un
bringup de depuración; todavía no contiene Nav2, misiones ni Cockpit.

## Simulación parcial de movimiento

```bash
ros2 launch salus_simulation motion_sim.launch.py
ros2 launch salus_control control_sim.launch.py
```

Estos launches aislados conectan `/cmd_vel_final` con un vehículo Ackermann
simulado mediante `/cmd_vel_gazebo`, `/odom_raw` y `/joint_states`. No incluyen
localización global, sensores, Nav2 ni un bringup operativo.

## Localización local parcial

```bash
ros2 launch salus_localization localization_sim.launch.py
```

Compone la telemetría del controlador, odometría Ackermann, una IMU simulada
normalizada y el EKF local. Publica `/wheel/odometry`, `/imu/data` y
`/odometry/local`; el EKF es la única autoridad de `odom -> base_footprint`.
No incluye GPS, datum, `map -> odom`, LiDAR ni Nav2.

## Localización GPS global parcial

```bash
ros2 launch salus_localization global_localization_sim.launch.py
```

Con los launches de movimiento, control y localización local ya activos, añade
NavSat de Gazebo, perfiles GPS reproducibles, `/gps/odometry_map`,
`/odometry/gps`, `/odometry/global` y la autoridad `map -> odom`. Usa un datum
fijo de simulación; no incluye brújula, datum dinámico, LiDAR ni GNSS real.

## Flujo de desarrollo

1. Elegir una unidad del [mapa de migración](docs/migration-map.md).
2. Registrar cambios arquitectónicos en `docs/decisions/`.
3. Implementar dentro del paquete propietario, sin dependencias circulares.
4. Documentar interfaces públicas y agregar tests.
5. Ejecutar `./tools/test.sh` antes de integrar.

Para orientación rápida de agentes y colaboradores, leer [AGENTS.md](AGENTS.md).

## Licencia

MIT. Consultar [LICENSE](LICENSE).
