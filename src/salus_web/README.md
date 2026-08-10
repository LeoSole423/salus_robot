# salus_web

- Responsabilidad: bridge ROS/WebSocket, snapshots y herramientas del operador.
- No contiene: la aplicación Cockpit, lógica de misión ni transporte de video.
- Interfaces previstas: cliente de APIs ROS y protocolo WebSocket versionado.
- Estado: esqueleto sin ejecutables.
- Prueba: `colcon test --packages-select salus_web`.
- Migración: definir protocolo web antes de dividir el backend anterior.

