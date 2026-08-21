# salus_web

- Responsabilidad: bridge ROS/WebSocket, snapshots y herramientas del operador.
- No contiene: la aplicación Cockpit, lógica de misión ni transporte de video.
- Interfaces previstas: cliente de APIs ROS y protocolo WebSocket versionado.
- Estado: codec, lock/heartbeat, proyección de estado y persistencia atómica
  portados como lógica pura; el runtime ROS/WebSocket aún no tiene ejecutables.
- Prueba: `colcon test --packages-select salus_web`.
- Migración: la intención, superficie compatible y separación obligatoria están
  definidas en
  [web-cockpit-bridge.md](../../docs/migration-evidence/intent/web-cockpit-bridge.md).
  Los módulos puros no abren sockets, no acceden a ROS y no establecen la
  política de ownership multi-cliente; esa frontera pertenece al próximo corte.
