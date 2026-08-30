# CI change-aware para Pull Requests

El CI de Pull Requests mantiene un `fast gate` obligatorio y selecciona los
smokes de integración según las fronteras modificadas. La selección es
deliberadamente conservadora: cualquier ruta desconocida o frontera compartida
cae en `FULL` y ejecuta la suite completa.

El selector vive en `tools/ci_select_smokes.py` y su matriz está cubierta por
`tools/test_ci_smoke_selection.py`. El job `classify-changes` publica en logs y
en el Job Summary los archivos detectados, la clasificación, smokes
seleccionados/omitidos y las razones.

## Fast gate obligatorio

`build-unit` se ejecuta para todo PR, incluso cambios sólo de documentación, e
incluye:

- `tools/validate_repository.py`;
- `colcon build`;
- lint y tests registrados por `colcon test`;
- `tools/test_smoke_harness.sh`.

Los smokes seleccionados no dependen de que `build-unit` termine. Cada job de
smoke conserva su propio build aislado, por lo que `build-unit`,
`simulation-core` y `navigation-missions` pueden empezar en paralelo después de
`classify-changes`. Un fallo del fast gate sigue fallando el workflow completo.

## Matriz de selección

| Cambio | Smokes seleccionados |
| --- | --- |
| `docs/**`, `README.md`, `AGENTS.md`, metadatos editoriales conocidos | ninguno; sólo fast gate |
| `src/salus_control/**` | control, motion, safety, integration |
| `src/salus_localization/**` | localization, canonical localization, sensor selection, integration, navigation core, canonical navigation |
| `src/salus_navigation/**` | safety, integration, navigation core, canonical navigation, navigation no-obstacles, zones, routes, patrol/HOME, snapshot |
| `src/salus_navigation_bt/**` | integration y los smokes funcionales de `navigation-missions` |
| `src/salus_perception/**` | LiDAR, integration, navigation core, canonical navigation |
| `src/salus_web/**` | integration; protocolo/gateway/lease Web quedan además cubiertos por el fast gate unitario |
| `src/salus_evaluation/**` | ninguno; actualmente no posee un runtime smoke, por lo que queda cubierto por build/lint/unit del fast gate |

La inclusión de navegación para cambios de localización valida que la pose
producida siga alimentando Nav2. La inclusión de navegación para percepción
valida la cadena de obstáculos más allá del smoke LiDAR aislado.
`salus_navigation` incluye el smoke de safety porque ese paquete posee el
arbitraje de seguridad.

## Full operational Cockpit: stress/reliability, no gate funcional

El escenario registrado `web_cockpit` conserva la composición completa
(`sim_operational.launch.py`, Nav2, zones, Snapshot y bridge WebSocket), pero
no participa en PR, FULL ni push a `main` sobre runners estándar.

La decisión está respaldada por #138. El mismo SHA y el mismo entry point
registrado de CI fallaron en un runner GitHub de 4 vCPU saturado cerca de 400%,
con ROS:wall degradado, mientras dos ejecuciones con el wrapper exacto
(`UDPv4`, timeout de 240 s e aislamiento normal) pasaron en un i7-13650HX con
margen de CPU, ROS:wall cercano a 1 y EKF/TF alrededor de 30 Hz.

Por tanto, un fallo del full operational Cockpit bajo una máquina sin margen no
se usa como evidencia suficiente de regresión funcional. El escenario no se
elimina: se ejecuta manualmente, o en infraestructura suficientemente
dimensionada, mediante:

```bash
python3 tools/run_registered_smoke.py web_cockpit --context manual
```

El contexto `manual` conserva script, entorno y hard-timeout del registro sin
convertir el escenario en gate obligatorio. La cobertura funcional Web
determinista permanece en `build-unit` (protocol, gateway, lease, state
projection y WebSocket server) y en el smoke `integration`.

## Fronteras que fuerzan FULL CI

Cualquiera de estas rutas ejecuta todos los smokes:

- `src/salus_interfaces/**`;
- `src/salus_bringup/**`;
- `src/salus_description/**`;
- `src/salus_simulation/**`;
- `src/salus_hardware/**`;
- `.github/workflows/**`;
- `tools/**`;
- `Dockerfile`;
- `compose.yaml`;
- `dependencies.repos`;
- `entrypoint.sh`;
- `docs/package-map.yaml` (fuente de verdad de ownership y dependencias).

También se fuerza `FULL` si no hay lista de cambios disponible o aparece una
ruta que el selector no reconoce. Un path nuevo nunca se interpreta como seguro
por defecto. El diff desactiva la detección de renames para clasificar tanto la
ruta eliminada como la nueva; mover un archivo no puede ocultar su frontera de
origen.

`push` a `main` y `workflow_dispatch` fuerzan siempre `FULL`, independientemente
de los paths. El nightly conserva su workflow y repeticiones existentes.

## Agregación de resultados

Los jobs de smoke conservan los pasos existentes, pero sólo ejecutan los
escenarios seleccionados. El agregador recibe `WORKSPACE` y únicamente los
smokes que debían correr; un paso omitido intencionalmente no se interpreta
como fallo.

## Builds duplicados

Este cambio no cachea ni transfiere `build/`, `install/` o `log/`. En un FULL
CI todavía pueden existir hasta tres builds del workspace: fast gate,
`simulation-core` y `navigation-missions`.

Se prioriza primero el ahorro por selección y paralelismo porque compartir
`install/` exige una clave de invalidación y un formato de artifact
reproducibles. Además, hacer que los smokes esperen el artifact de `build-unit`
volvería a serializar el camino crítico. Una optimización futura debe medir el
costo real de empaquetado/transferencia frente a recompilar y demostrar que no
introduce estado stale.
