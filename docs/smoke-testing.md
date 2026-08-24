# Smoke tests de simulación

Los smokes son escenarios aislados de frontera. No sustituyen las pruebas
unitarias ni la validación visual/manual de Gazebo.

`tools/smoke_harness.sh` es la única capa compartida para el ciclo de vida de
un escenario: asigna evidencia bajo `artifacts/smokes/`, registra condiciones
de readiness, detiene los launches y guarda topología ROS, lifecycle, TF y
logs incluso si el smoke falla. Cada escenario conserva sus propias
afirmaciones funcionales.

| Escenario | Mundo / entrada | Responsabilidad |
| --- | --- | --- |
| control | sin Gazebo | batería y backend simulado |
| movimiento | `free.world` | actuación, odometría y joints |
| localización | `free.world` | odometría local y TF local |
| LiDAR | `empty.world` con obstáculo | nube 3D y scan derivado |
| seguridad | `free.world` | arbitraje y parada segura |
| Nav2 | `free.world` | goal único y cadena automática |
| zonas | `free.world`, runtime único | máscara keepout y recarga |
| rutas | `free.world` | misión, checkpoints, progreso y cancelación |
| patrulla/HOME | `free.world` | fases de misión y retorno por batería |
| snapshots | composición Nav2 | render y servicio PNG determinista |
| Cockpit/cámara | composición completa | WebSocket, lease, PTZ y contratos web |
| integración | mundo de composición | procesos, lifecycle y contratos, sin repetir escenarios funcionales |

La tolerancia de arranque solamente controla cuánto se espera por una
condición observable. No modifica la frescura funcional de `PathHealth`: TF o
costmap vencidos continúan produciendo `STOP_AND_WAIT`.

En un PR se ejecutan `build-unit`, `simulation-core` y
`navigation-missions`. El workflow nocturno admite repeticiones configurables
(normalmente diez) para detectar flakiness y conserva artefactos de diagnóstico.
