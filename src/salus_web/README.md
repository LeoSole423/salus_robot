# salus_web

- Responsabilidad: bridge ROS/WebSocket, snapshots y herramientas del operador.
- No contiene: la aplicación Cockpit, lógica de misión ni transporte de video.
- Interfaces previstas: cliente de APIs ROS y protocolo WebSocket versionado.
- Estado: runtime ROS/WebSocket compatible con Cockpit portado en simulación,
  incluyendo lease exclusivo, telemetría compacta/full, zonas, navegación/misiones, control
  manual, waypoints y snapshots. Sesiones, rosbag, RTK y cámara siguen diferidos.
- Prueba: `colcon test --packages-select salus_web`.
- Launch parcial:
  `ros2 launch salus_web web_bridge.launch.py ws_port:=8766`.

El bridge publica `robot_pose` dentro de la telemetría web usando posición de
`/gps/fix` y orientación ROS (yaw, grados) de `/odometry/local`. El tópico de
orientación se puede cambiar con `heading_odometry_topic`; si todavía no hay
fix GPS o el cuaternión es inválido, no se fabrica una pose u orientación.
- Smoke integrado: `./tools/run_smoke.sh ./tools/smoke_web_cockpit.sh`.
- `telemetry_profile:=compact` es el default operativo: agrega estado
  reemplazable a 2 Hz, pero mantiene eventos, acks y transiciones inmediatos.
  `full` conserva los deltas de diagnóstico. `/scan_preview` es un
  `LaserScan` reducido y reemplazable; no transporta nube 3D.
- Migración: la intención, superficie compatible y separación obligatoria están
  definidas en
  [web-cockpit-bridge.md](../../docs/migration-evidence/intent/web-cockpit-bridge.md).
  La política compacta está en
  [compact-telemetry-scan-preview.md](../../docs/migration-evidence/intent/compact-telemetry-scan-preview.md).
- `/salus/hardware/gnss_primary/rtk_status` (`GnssRtkStatus`) es la autoridad
  RTK tipada cuando aparece. La proyección `gps_status` conserva por separado
  calidad GNSS, adquisición/frescura RTCM y backend/estado de entrega. El
  string `/gps/rtk_status` queda como fallback de migración y no puede reemplazar
  un estado tipado ya recibido durante la vida del proceso.
  Los módulos puros no abren sockets ni acceden a ROS. El transporte y el
  adaptador ROS permanecen separados y la política multi-cliente está fijada
  por el ADR 0005.
