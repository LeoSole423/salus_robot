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
  Los módulos puros no abren sockets ni acceden a ROS. El transporte y el
  adaptador ROS permanecen separados y la política multi-cliente está fijada
  por el ADR 0005.
