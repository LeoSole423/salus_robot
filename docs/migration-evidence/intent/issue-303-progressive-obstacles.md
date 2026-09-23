# Intención: ocupaciones lejanas o dudosas (#303)

## Alcance

- Fuente histórica: commits de navegación legacy y la misión real descrita en #303.
- Destino: `salus_navigation`, con evaluación reproducible en `salus_evaluation`.
- Incluido: distinguir observación lejana transitoria, bloqueo persistente y riesgo cercano sin perder la misión lógica.
- Fuera de alcance: segmentación de suelo (#300), freshness del scan (#277) y selección del sufijo de checkpoints tras un retry (#302).

## Evidencia disponible

| Fuente | Hecho observado | Límite |
| --- | --- | --- |
| [#303](https://github.com/LeoSole423/salus_robot/issues/303) | La misión real registró `collision ahead` en RPP, terminó goals e inició recuperación. Collision Monitor mostró sólo slowdown temporal. | El texto del issue no identifica distancia, persistencia ni clase física de la ocupación. |
| [#300](https://github.com/LeoSole423/salus_robot/issues/300) | Una captura de 43,95 s contiene bandas bajas lejanas en `/obstacles_cloud`, anteriores a `/scan_clean`. | No demuestra que esas bandas fueran la causa de cada aborto de #303. |
| `path_health.py` y `test_path_health.py` | En los 12 m inspeccionados, una celda letal provoca `REPLAN` en la primera evaluación, aun a 8 m; al desaparecer vuelve a `KEEP_PATH`. | El test congela la semántica actual, no reproduce el controlador RPP ni la misión real. |
| `nav2_core_real.yaml` | RPP conserva `use_collision_detection: true` y horizonte de colisión de 1,5 s. El costmap local marca `/scan_clean` hasta 15 m. | El YAML no acredita los parámetros efectivos cargados durante la misión. |
| `navigation_through_poses.xml` | El BT intenta path estable, espera/replan y recuperaciones acotadas; `FollowPath` puede terminar en fallo. | La secuencia temporal exacta del aborto necesita logs y bag. |
| `route_recovery.py` y `route_executor_node.py` | El ejecutor trata `CONTROLLER_COLLISION` y `NAV_ABORTED` como bloqueo persistente; tras 1,5 s puede cancelar, frenar y reintentar. | El progreso de checkpoints ya acreditados corresponde a #302. |
| Legacy `nav_command_server.py`, commit `dfda7f3` | El texto `collision ahead` se clasifica como pista `CONTROLLER_COLLISION`. | Es una pista de log, no una medición del obstáculo. |
| Recuerdo del operador, comunicado después de la caracterización inicial | En una pendiente, con el robot inclinado, apareció a su izquierda una ocupación grande en el costmap semejante a una pared. Los puntos rojos de Nav Live se veían dispersos, sin geometría de pared. Había pasto bajo en esa zona. La «pared» desaparecía al enderezarse el robot, sin inclinación. | No hay aún correlación temporal exacta con `collision ahead`, scan, pose, costmap o resultado Nav2. Tampoco se midió cuánto tardaba en desaparecer. |
| `test_tilted_ground_characterization.py` y `test_tilted_low_grass_reaches_clean_scan_then_disappears_when_upright` | Con una escena sintética fija de pasto irregular a la izquierda, el filtro y la composición real de percepción muestran ocupación lateral en `/scan_clean` al aplicar una rotación relativa de 8°; la ocupación no aparece a 0° y desaparece al volver a 0°. Un poste de 0,75 m permanece detectable. | Es un control causal sintético de la geometría relativa en nube y scan. No prueba que el roll real fuera 8°, que la inclinación del chasis por sí sola cause esa geometría, ni que RPP termine el goal. El input del test ROS ya está expresado en `base_footprint`; no ejercita la TF dinámica del LiDAR. |
| `test_navigation_real_pc_runtime_without_clock` | El mismo patrón angular sintético, publicado en `/scan_clean` con Nav2 real activo en PC, produjo al menos 10 celdas letales en el costmap local de la zona izquierda. Al regresar al scan despejado, quedaron como máximo una en 10 s. En una ejecución diagnóstica la secuencia publicada fue 52 → 1 celdas. | Esta etapa inyecta el scan directamente; no ejecuta un goal de movimiento ni demuestra la causa del aborto físico. La celda residual necesita investigación separada antes de afirmar clearing completo. |

El artifact físico citado en #300 está sólo en el Jetson, que no respondió por SSH durante esta investigación. Falta el bag y los logs exactos de la ruta que produjo `collision ahead`.

## Contratos e invariantes

- `/scan_clean` alimenta costmaps y Collision Monitor; el segundo conserva autoridad dura sobre `/cmd_vel_safe`.
- RPP sigue rechazando una trayectoria con riesgo de colisión cercano. La política nueva no puede fabricar un comando positivo cuando RPP o Collision Monitor paran.
- TF, costmap o scan ausente/vencido no equivalen a espacio libre.
- Un replan local no cambia el identificador ni el progreso acreditado de la misión; un goal terminal y un stop cercano siguen siendo estados observables distintos.
- No se reduce globalmente alcance LiDAR, footprint, horizonte de RPP ni tolerancias de freshness para hacer desaparecer falsos positivos.

## Investigación y decisión pendiente

1. Sincronizar `header.stamp` y tiempo de recepción de `/scan_clean`, costmaps local/global, plan, arco RPP, `/cmd_vel`, `/cmd_vel_safe`, `/collision_monitor_state`, `/path_health`, diagnóstico del controlador y resultado Nav2. Registrar el primer eslabón que marca ocupación y el desfase de cada pareja causal.
2. Por evento, medir distancia al obstáculo sobre la trayectoria y al footprint, sector, costo, duración, número de observaciones independientes, velocidad, tiempo disponible para detenerse, clearance cercano y si el punto persiste al cambiar de pose. Reproducir además inclinación del robot y ocupación lateral izquierda: comparar nube original, puntos proyectados y costmap antes de atribuir una pared física. Un costo alto por sí solo no constituye confianza de sensor.
3. Reproducir en PC/sim: obstáculo real a distancias cercana y lejana, celda transitoria lejana, ocupación persistente, pasto/pendiente y datos stale. En el escenario de pasto, mantener la escena fija y variar la inclinación del robot de inclinado a derecho; comprobar si la ocupación con aspecto de pared aparece y desaparece en nube, scan y costmap, y medir la demora de despeje. Comparar con el baseline antes de seleccionar umbrales.
4. Especificar una política pura que devuelva decisión y motivo tipados: observar/replanear sin terminar la misión, aproximación prudente sólo con tramo cercano válido y despejado, o stop/espera ante riesgo cercano o incertidumbre de datos. Definir histéresis y condición de despeje con evidencia; los valores numéricos quedan abiertos hasta el replay.
5. Integrar en el propietario de navegación después de localizar la causa terminal exacta. Mantener RPP y Collision Monitor independientes. Evitar que un nuevo estado de observación agote los retries del BT o del ejecutor sin un fallo persistente demostrado.

## Pruebas y gates

- Caracterización actual: `test_single_distant_lethal_cell_replans_active_path_immediately`.
- Fixture de percepción: `test_tilted_ground_characterization.py` comprueba la clasificación pura y `test_tilted_low_grass_reaches_clean_scan_then_disappears_when_upright` atraviesa `scan_ground_filter -> pointcloud_to_laserscan -> scan_noise_filter`, cotejando cada salida por `header.stamp`.
- El test de composición real de navegación comprueba la marca y el despeje mayoritario del costmap local con el patrón del fixture; el remanente de una celda se registra sin ocultarlo.
- Siguiente frontera: un escenario de movimiento en Gazebo que introduzca inclinación mientras Nav2 ejecuta una meta y mida RPP, resultado del goal y trayectoria. Un robot inmóvil con odometría sintética no permite interpretar un aborto de controlador como causado por la ocupación.
- Un obstáculo real cercano conserva stop y no recibe aproximación positiva desde la política.
- Una ocupación lejana de corta duración se observa sin abortar ni reiniciar la misión; si persiste, produce replan o espera con motivo observable.
- Datos stale/TF inválido mantienen degradación segura. Ningún test debe pasar simulando libre una celda desconocida o vencida.
- El estado de misión y los checkpoints acreditados permanecen monotónicos durante replans; la lógica concreta de sufijo se valida en #302.
- Tests puros, tests del adaptador, replay, smokes de navegación/rutas/safety, build y suite completa antes de CI. La validación física requiere luego prueba controlada con operador y E-stop.

## Estado de evidencia

- Estado: `characterized` parcialmente; no hay selección de umbrales ni implementación de política nueva.
- No validado en hardware: causalidad de la ocupación, distancia de disparo, presupuesto de frenado y efecto de una política progresiva.
- Entrada que falta: bag/logs de la ruta real de #303 y acceso o copia del artifact de #300.
