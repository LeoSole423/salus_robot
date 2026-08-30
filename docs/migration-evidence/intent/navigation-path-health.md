# Intención caracterizada: path estable y despeje

## Evidencia legacy

- `f4f093b` introdujo un guard de clearance para evitar replans sin causa.
- `6634dcf` y `4a1a2b4` estabilizaron la validación y su trazabilidad.
- `3457b65` ajustó Nav2 para Ackermann sin recuperación de giro ni marcha atrás.

## Invariantes migrados

- Conservar el path activo ante errores pequeños y ruido aislado.
- Replanificar por colisión, inflación sostenida, desvío persistente o falta de progreso.
- Parar el movimiento automático y esperar ante costmap o TF no disponibles.
- Validar un candidato antes de reemplazar el path activo.

## Decisión de estructura

`PathHealthPolicy` conserva la lógica pura. `EvaluatePathHealth` declara si la
consulta corresponde al path `ACTIVE` o a un `CANDIDATE`; el BT C++ sólo
coordina estados. Esto impide que el orden o la concurrencia de llamadas ROS
modifique la histéresis del path activo.

El BT calcula `candidate_path`, lo valida y sólo entonces lo copia al path
activo que consume `FollowPath`. No usa `SmoothPath` ni un `smoother_server`;
el planner `SmacPlannerHybrid` mantiene `smooth_path: false`.

## Evidencia de almacenamiento del costmap

En el entorno ROS 2 Humble del proyecto, `nav2_msgs/msg/Costmap.data` está
declarado como `sequence<uint8>` y el código Python generado por
`rosidl_generator_py` lo representa como `array.array('B')`. El setter conserva
ese `array.array` directamente cuando ya tiene el tipo de elemento compatible.
`CostmapView` conserva una referencia a esa secuencia, manteniendo su lifetime
mediante la referencia Python sin depender de un buffer temporal del
middleware.

El test `test_on_costmap_retains_data_after_message_is_collected` construye un
`nav2_msgs.msg.Costmap` real, lo pasa por `_on_costmap()`, verifica identidad,
elimina el mensaje y fuerza GC; los cuatro valores siguen accesibles desde
`CostmapView.data`.

El adaptador no muta el mensaje ni el array: sólo retiene la referencia. Se
eliminó la materialización `tuple(message.data)`, que generaba una copia Python
de todo el global rolling costmap en cada actualización. La política sigue
leyendo los mismos índices y valores, incluyendo free, lethal, unknown y los
bordes fuera de la geometría (que continúan devolviendo costo 0).

En un benchmark local reproducible con Humble, una matriz `1200 × 1200`
(1.440.000 celdas) tomó 4,46 ms por adaptación con `tuple(message.data)` y
0,001 ms reteniendo directamente `message.data` (12 iteraciones, luego de una
iteración de calentamiento). Es una medición del adaptador, no una promesa de
reducción equivalente del CPU total del sistema. Con `publish_frequency: 0.5`,
eso equivale a aproximadamente 2,23 ms de CPU de adaptación ahorrados por
segundo de tiempo de pared.

Midiendo también `PathHealthPolicy.evaluate()` sobre el mismo costmap y una
trayectoria de 12 m, el resultado fue 0,1890 ms con tuple frente a 0,1725 ms
con `array('B')` (500 iteraciones por variante). La representación nueva no
introdujo una regresión medible en el hot path de indexación.

## Pruebas

`test_path_health.py` cubre colisión, inflación, histéresis, stale data,
progreso, aislamiento de candidatos, índices/bordes de `_cell_cost()` y la
retención de una secuencia `array.array` no-tuple. `smoke_navigation_core_sim.sh` verifica
la cadena Nav2 integrada y la interfaz del servicio.
