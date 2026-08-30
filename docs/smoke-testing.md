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
| Nav2 canónico | `free.world` | goal Nav2, `VehicleCommand` fresco y actuación Gazebo |
| Nav2 sin obstáculos | `free.world`, sin `/scan_clean` | perfil degradado explícito, zonas y autoridad segura |
| zonas | `free.world`, runtime único | máscara keepout y recarga |
| rutas | `free.world` | misión, checkpoints, progreso y cancelación |
| patrulla/HOME | `free.world` | fases de misión y retorno por batería |
| snapshots | composición Nav2 | render y servicio PNG determinista |
| Cockpit/cámara | composición completa | WebSocket, lease, PTZ y contratos web |
| integración | mundo de composición | procesos, lifecycle y contratos, sin repetir escenarios funcionales |

La tolerancia de arranque solamente controla cuánto se espera por una
condición observable. No modifica la frescura funcional de `PathHealth`: TF o
costmap vencidos continúan produciendo `STOP_AND_WAIT`.

El artefacto de patrulla conserva además la identidad/estado del route
executor, generación y resultado de goals Nav2, eventos con timestamp, pose y
distancia recorrida durante `JOIN_LOOP`, distancia al target, último comando
seguro/final y el historial de path-health. Estos campos son evidencia de
fallo; no sustituyen ni suavizan las aserciones de fase y retorno HOME.

En un PR, `build-unit` es obligatorio y `simulation-core` /
`navigation-missions` ejecutan sólo los escenarios seleccionados por
[`ci-change-aware.md`](ci-change-aware.md). Cambios compartidos o desconocidos
caen en FULL CI. El workflow nocturno conserva la suite completa, admite
repeticiones configurables (normalmente diez) y mantiene artefactos de diagnóstico.
