# salus_web

- Responsabilidad: bridge ROS/WebSocket, snapshots y herramientas del operador.
- No contiene: la aplicación Cockpit, lógica de misión ni transporte de video.
- Interfaces previstas: cliente de APIs ROS y protocolo WebSocket versionado.
- Estado: protocolo Cockpit caracterizado; runtime aún sin ejecutables.
- Prueba: `colcon test --packages-select salus_web`.
- Migración: la intención, superficie compatible y separación obligatoria están
  definidas en
  [web-cockpit-bridge.md](../../docs/migration-evidence/intent/web-cockpit-bridge.md).
