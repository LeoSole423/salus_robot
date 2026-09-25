# Intención: orientar el anchor de entrada a un loop

## Alcance

- Fuente de la incidencia: [salus_robot #315](https://github.com/LeoSole423/salus_robot/issues/315) y su plan de trabajo.
- Destino: política pura `salus_navigation.route_anchor` y su adaptador de `route_executor`.
- Incluido: considerar pose y yaw al elegir el tramo de entrada de una misión loop nueva; rechazar entradas sin orientación válida, fuera de un tramo cercano o con candidatos ambiguos; emitir evidencia mediante `NavEvent` existente.
- Fuera de alcance: rutas abiertas, continuidad de una misión activa, Cockpit, cambios a interfaces ROS, Nav2, configuración de obstáculos, Collision Monitor, comandos y hardware.

## Evidencia

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| Issue #315, captura del 24/09/2026 | El robot estaba en `(13403.745, 6808.525)`, yaw aproximado `1.6°`; la selección real despachó índices `[46,47]`, con waypoint 46 en `(13372.705, 6811.113)` y waypoint 47 en `(13346.321, 6813.811)`. | Alta para los valores reportados; el agente no accedió al robot. |
| `route_executor_node.py::_on_pose()` en `origin/main` `0242a18` | `/odometry/global` proporciona la posición y el quaternion de orientación. | Alta, inspección del código. |
| `route_executor_node.py::_activate_prepared()` y `route_anchor.py::select_anchor()` en `0242a18` | Cada ruta preparada empieza una misión nueva; la política anterior sólo comparaba distancias y escogía el segmento cercano sin usar yaw. | Alta, inspección del código. |
| Prueba base del fixture reducido, antes de cambiar la política | Con la pose de la captura y ambos segmentos candidatos, la política anterior devolvió el índice 46. | Alta para el fixture; no es replay del bag. |
| Copia indicada en `/tmp/salus_nav_live_capture_20260924` | No se encontró en este PC; no había bag ni ruta de 51 puntos para reproducir la geometría completa. | Alta sobre disponibilidad local. |

### Hechos e inferencias

- Hecho reportado: 46→47 apunta aproximadamente al oeste, en contra del yaw este del robot.
- Hecho observado en el código: la entrada loop podía elegir un segmento por distancia lateral aunque su sentido apuntara detrás del robot.
- Inferencia de fixture: 10→11 es un tramo cercano hacia el este, como indica la expectativa operacional del issue. Sus coordenadas no están en la copia disponible.
- El fixture conserva la pose y las coordenadas físicas de 46 y 47; aproxima sólo las dos geometrías candidatas y rellena los demás índices para mantener la numeración original. No prueba paridad geométrica de la ruta completa.

## Problema e intención

Al reiniciar una ruta loop desde la pose actual, la cercanía lateral por sí sola puede seleccionar un tramo cuyo checkpoint siguiente queda detrás. La entrada debe avanzar por una proyección finita de la ruta y concordar con la orientación del robot. Si los datos no identifican una opción única, no se inicia la nueva misión ni se envía un goal.

## Contratos e invariantes

- Entradas internas: ruta preparada, posición y yaw de `/odometry/global`, muestra dentro de `route_progress_pose_max_age_s` (0,5 s por defecto), tolerancia de waypoint existente (1,2 m) y tolerancia de segmento existente (`route_segment_start_tolerance_m`, 5 m por defecto).
- Los segmentos deben contener la proyección de la pose. Los candidatos con diferencia angular de 90° o más quedan fuera del semiplano delantero.
- Entre candidatos delanteros, una selección sólo es concluyente cuando un candidato no es peor en distancia lateral ni diferencia angular. Un empate o tradeoff produce `ambiguous_forward_segments`.
- Rutas abiertas conservan su lógica previa. Una misión nueva no reutiliza checkpoints de una misión cancelada ni acredita puntos por haberlos pasado manualmente.
- No cambian tipos, campos ni servicios de `salus_interfaces`. La selección y el rechazo se notifican por el `NavEvent` existente (`ROUTE_ANCHOR_SELECTED` / `ROUTE_ANCHOR_REJECTED`).
- El rechazo ocurre antes de reemplazar la misión y antes del despacho. Nav2 y la cadena de seguridad quedan sin cambios.

## Fallos y degradación

| Condición | Respuesta | Prueba |
| --- | --- | --- |
| Yaw ausente/inválido o muestra de pose vencida | Rechazar entrada loop; notificar causa y candidatos geométricos cercanos. Un quaternion cero no se interpreta como yaw cero. | `test_loop_anchor_rejects_unknown_orientation_and_outside_route`, `test_pose_callback_does_not_treat_zero_quaternion_as_heading_zero`, prueba de frescura y adaptador de rechazo. |
| Ningún segmento cercano o con proyección finita | Rechazar; no usar nearest-waypoint como fallback para loops. | Fixture fuera de ruta y tolerancia de segmento. |
| Candidato únicamente detrás/perpendicular | Rechazar con `no_forward_segment`; no despachar. | Cobertura de selección por sentido en los fixtures. |
| Métricas de distancia y rumbo en conflicto o idénticas | Rechazar como ambiguo. | `test_loop_anchor_rejects_indistinguishable_forward_segments` y `test_loop_anchor_rejects_distance_heading_tradeoff`. |

## Pruebas y aceptación

- Base previa: el fixture reducido confirmó índice 46 con la política posicional original.
- Unitarias actuales: rumbo este selecciona waypoint 11; rumbo oeste selecciona waypoint 46; vértice próximo conserva la entrada al tramo saliente; orientación ausente, pose vencida, tramo lejano y candidatos ambiguos se rechazan. `colcon test --packages-select salus_navigation`: 302 pasaron, 0 fallos.
- Gates completos: `./tools/test.sh` terminó con build de 14 paquetes, 1208 tests pasados, 0 fallos y 2 skips; el smoke harness self-test pasó.
- Adaptador: `ROUTE_ANCHOR_REJECTED` se emite y no se despacha un goal cuando falta orientación.
- Smoke simulado `SMOKE_ROUTE_SCENARIO=loop ./tools/smoke_route_executor_sim.sh`: el check `route_executor` pasó hasta despachar la iteración 1. El artefacto registra cinco chunks, goals completados, pose, checkpoints y 1121 comandos finales en `artifacts/smokes/routes-free-world-20260925T125445-1/route_probe.json`.
- La misma ejecución falló después en `navigation_profiles`: faltó el parámetro de suelo `[0.25]`. Ese check es ajeno a la política anchor y se conserva en la evidencia; el smoke global no queda marcado como verde.

## Estado de evidencia

- Estado: `characterized`.
- Validación en hardware: no realizada. La Jetson y el servicio real no se usaron.
- Pendiente para corroborar la geometría física: disponer de la ruta/bag original y confirmar 10→11. La validación física requiere una sesión posterior con operador y E-stop.
