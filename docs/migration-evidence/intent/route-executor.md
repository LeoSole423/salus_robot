# Intención: ejecutor de rutas

Fuente histórica: `fb54b95`, `eaac77d`, `fd7d977`, `6d94ba3`, `8e826e9` y `d0cd4a7`, junto con `test_route_executor.py` de `ROS2_SALUS`.

- Las rutas abiertas omiten solo el prefijo ya alcanzado y nunca retroceden.
- Los loops se incorporan/rotan conservando los índices originales y sus metadatos.
- Los chunks son finitos, no se solapan y un loop nunca se entrega completo repetidamente a Nav2.
- Los puntos originales son checkpoints. Los puntos expandidos son geometría
  entregada dentro del chunk multi-pose, además de servir para anclaje,
  progreso y diagnóstico: no producen éxito, freno ni acciones.
- Cada chunk termina en el próximo checkpoint original. Los límites de cantidad
  y distancia son suaves dentro de esa pierna: si se cruzan entre puntos
  sintéticos, se conserva la geometría hasta el checkpoint. El chunk finito se
  entrega como `NavigateThroughPoses`; sólo un chunk de una pose usa
  `NavigateToPose`.
- Antes de cada dispatch se avanza sólo sobre el tramo sintético inicial ya
  sobrepasado. Los checkpoints originales nunca se podan: cada uno es una
  frontera observable para `ROUTE_CHECKPOINT_REACHED`, incluida la salida de
  loop que Patrol usa para pasar de `EXIT_LOOP` a `RETURN_HOME`.
- El request de Cockpit selecciona `auto_yaw_policy=route_tangent`: un yaw
  automático de checkpoint sigue la tangente entre las piernas de entrada y
  salida; los extremos usan la única pierna disponible. Los puntos sintéticos
  usan el rumbo de su pierna. El dispatch finito conserva esos valores. Los
  callers que dejan el campo vacío usan ahora la misma bisectriz, incluso
  Patrol/HOME; `legacy` conserva la política previa de rumbo de salida y
  ajuste del terminal. Un yaw explícito nunca se sustituye.
- No hay freno entre objetivos contiguos; sí al finalizar, cancelar o abortar.
- Esta migración convierte LL una vez para validar/preparar la misión y conserva las poses `map` para diagnóstico. Cada despacho usa el contrato legacy `SetNavGoalLL`, cuyo servidor mantiene su conversión defensiva.

Fuera de alcance: reintentos, acciones, perfiles, patrulla, HOME y batería.

Los defaults caracterizados para esta frontera son `leg_spacing_m=35.0`,
`chunk_span_m=120.0` y `chunk_max_waypoints=5`. Las acciones programadas son
fronteras duras; los límites de span/cantidad son suaves hasta el checkpoint.

Esta separación recupera la intención visible en `fd7d977`, `eaac77d`,
`6d94ba3`, `e190157` y `4a1a2b4`: la densificación describe el recorrido que
Nav2 debe seguir, pero no redefine los hitos de la misión.

## Decisiones del corte #244

- `SetNavGoalLL` conserva su contrato público escalar/array. Arrays finitos se
  traducen a `NavigateThroughPoses`; `loop=true` sigue rechazado porque el loop
  pertenece al executor y nunca se entrega completo a Nav2.
- El BT multi-pose reutiliza `PathHealth` para conservar un path vigente,
  validar el candidato y detenerse ante TF/costmap stale. Recupera poda de
  goals y clears local/global, sin `Spin`, `BackUp`, smoother ni waypoint
  follower.
- El progreso se calcula proyectando sobre segmentos del chunk activo. Esto
  evita reportar como error transversal la distancia al vértice más cercano.
- El smoke de rutas registra el request lógico del chunk, el pose del robot y
  el `/plan`; sus métricas deterministas incluyen longitud, ratio de desvío,
  máxima distancia a la polilínea solicitada y auto-intersecciones. La
  comparación histórica de yaw mantuvo separadas la política de approach, la
  preparada sin mutación y la terminal entrante. En tres ejecuciones del
  escenario determinista `wide_turn_two_pose_window`, usando
  `ComputePathThroughPoses` y el path devuelto por Nav2, se obtuvo la misma
  conclusión:

  | política | yaw solicitado (°) | longitud (m) | detour | max deviation (m) | auto-intersecciones |
  | --- | ---: | ---: | ---: | ---: | ---: |
  | actual (approach + terminal) | 90, 0 | 61.298 | 6.130 | 8.07 | 1 |
  | preparada estilo legacy | 0, 90 | 62.441 | 6.244 | 10.83 | 2 |
  | sólo terminal entrante | 0, 0 | 39.605 | 3.961 | 8.07 | 0 |

  La variación entre ejecuciones fue menor que 0.03 m en `max deviation` y no
  cambió la clasificación. En aquel corte se conservó `terminal_incoming`;
  el smoke vigente compara la tangente de ruta con esa política histórica y
  escribe `evidence.yaw_policy_comparison` en `route_probe.json`.
- No se incorporan parámetros legacy omitidos ni tuning: requieren evidencia
  independiente y no son necesarios para restaurar el contrato multi-pose.

No validado en hardware en este corte: continuidad física, curva Ackermann y
loop. Esas pruebas sólo se habilitan después de PC/CI y simulación verdes.

## Yaw automático en curvas de patrulla

La ruta guardada `PatrullaSencillaPolo` no contiene yaws explícitos. En el
Cockpit legacy, `web_zone_server._resolve_waypoint_yaws()` convertía esos
puntos a yaws finitos antes de enviarlos: para un punto interior usaba la
bisectriz de los rumbos de entrada y salida. El executor legacy los recibía
como explícitos y los despachaba sin adaptar el terminal. En el Cockpit nuevo,
el gateway conserva la ausencia de yaw como `NaN`; el executor actual calculaba
el rumbo de salida y después sustituía el yaw del terminal por el de entrada.

En los puntos 3 y 4 del archivo de la patrulla, los rumbos aproximados de
entrada/salida son `-104°/-17°` y `-17°/84°`. El yaw que generaba el Cockpit
legacy era `-60°` y `34°`; el terminal automático actual pedía `-104°` y
`-17°`. Esto explica una diferencia de request, no demuestra todavía la
trayectoria ejecutada. La política pura se ajusta para usar la tangente de la
ruta en los checkpoints automáticos, conservar yaws explícitos y usar el rumbo
del tramo en puntos sintéticos. El dispatch preserva estos yaws al cortar un
chunk. El campo aditivo `SetRouteMissionLL.auto_yaw_policy` lo solicita el
gateway de rutas de Cockpit; vacío también selecciona la bisectriz para
Patrol/HOME y otros callers. `legacy` conserva el cálculo anterior. La prueba
de operador en simulación confirmó que las curvas de la patrulla mejoraron.
La validación en robot sigue pendiente.

### Diferencia causal con ROS2_SALUS para #244

El Cockpit legacy resolvía el yaw automático **antes** del request: usaba la
bisectriz circular entre entrada y salida en cada punto interior
(`ROS2_SALUS/src/map_tools/map_tools/web_zone_server.py::_resolve_waypoint_yaws`).
Enviaba un array de yaws finitos, y el executor legacy los copiaba al chunk
sin recalcular el terminal
(`ROS2_SALUS/src/navegacion_gps/navegacion_gps/route_executor.py::_send_chunk`).
La alternativa interna del executor legacy, rumbo de salida para yaws ausentes,
no era la que usaba esa ruta de Cockpit porque ya recibía yaws explícitos.

El Cockpit nuevo conserva la ausencia de yaw. Antes de la política
`route_tangent`, `salus_robot` asignaba rumbo de salida al checkpoint automático
y sustituía el yaw del terminal de cada chunk por el rumbo de llegada. Esa
combinación era distinta de la bisectriz legacy y explica el request tardío de
giro que degradaba las curvas. `route_tangent` restaura la bisectriz para los
checkpoints automáticos de Cockpit y da a los sintéticos el rumbo del tramo.
Los yaws escritos por el operador mantienen prioridad. La extensión de la
bisectriz a Patrol/HOME fue solicitada después de la prueba de Cockpit y
requiere validación específica de sus transiciones. El primer intento de smoke
produjo un plan con autointersección en `JOIN_LOOP`: el robot estaba a unos
6 m del primer checkpoint del chunk y éste exigía `−90°` aunque la aproximación
era casi `0°`. En el dispatch, ese primer checkpoint automático toma la
orientación actual del robot si la diferencia supera 60°; los demás conservan
su tangente y los yaws manuales quedan intactos.

La simulación georreferenciada de `PatrullaSencillaPolo` reveló también un
caso límite: en la pierna 12→13, de 36,8 m, el espaciamiento de 35 m colocaba
un sintético apenas 1,9 m antes del checkpoint 13. El sintético pedía
aproximadamente −3,68° y el checkpoint, cuya bisectriz anticipa la siguiente
curva, −23,25°. Smac Hybrid/Dubins produjo un `/plan` de 65,17 m para un
desplazamiento directo de unos 39 m, con una autointersección; después de
entrar a 2,5 m del checkpoint todavía dibujaba 28,29 m de vuelta. El
controlador de posición completó el pedido antes de ejecutar esa vuelta.
Esto **no invalida** la bisectriz que mejoró las curvas: muestra que no debe
insertarse una pose sintética casi coincidente con un checkpoint cuyo yaw es
distinto. La expansión ahora descarta la muestra final de una pierna si deja
menos de medio espaciamiento hasta el siguiente checkpoint.

Un segundo fallo independiente apareció en 43→44: un chunk de 149 m contenía
cuatro sintéticos pero terminaba en el checkpoint 44, que quedaba unos 15 m
fuera del costmap global móvil de 300×300 m. Nav2 abortó con
`Goal pose is out of costmap!` y la misión agotó tres retries. El horizonte
`adaptive_dense_horizon_m=60` sólo limita extensiones más allá del par inicial
de checkpoints, por lo que no protegía esa pierna. El executor ahora acota
cada pedido a 120 m radiales desde la pose de dispatch y usa el último
sintético alcanzable como terminal provisional. Ese éxito avanza la ventana
sin acreditar un checkpoint, ejecutar acciones ni alterar los eventos de
Patrol/HOME. La corrección corresponde a #309; su evidencia sigue siendo de PC
y simulación, no de hardware.

## Corrección física posterior a #244

La primera implementación de paridad agrupó hasta cinco checkpoints originales
en un único `NavigateThroughPoses`. Una ruta física corta demostró que eso no
reproducía `build_chunk_waypoints(..., key_stop_indices=...)` del legacy:
Smac/Dubins intentaba satisfacer varios yaws cercanos dentro de un solo plan y
producía bucles de gran radio. El contrato corregido corta en el siguiente
checkpoint y conserva únicamente los sintéticos de esa pierna.

Los paths diagnósticos de misión y chunk son estado, no sensores. Se publican y
consumen con `TRANSIENT_LOCAL`, se proyectan usando el TF actual y permanecen
en el snapshot hasta que el executor publica explícitamente un path vacío. Esto
evita que Nav Live pierda la ruta cuando Nav2 limpia o reemplaza `/plan`.

## Diagnóstico `patrol_battery`

El artifact rojo de CI `34511311407` (`patrol-battery-return-20260910T180024-1`)
terminó esperando `PATROL`/`EXIT_LOOP` con navegación activa: el comando final
era AUTO y positivo, `collision_stop_active=false`, `failure_code` vacío y el
historial de `PathHealth` era `path_healthy`. El artifact rojo local posterior
(`patrol-battery-return-20260910T193206-1`) avanzó hasta `EXIT_LOOP`, completó
dos chunks y volvió a mostrar `collision_stop_active=false`, `failure_code`
vacío y `path_healthy`; sólo registró dos episodios de `progress_stalled` antes
de agotar la espera del tercer chunk. Por tanto no hay evidencia de que
Collision Monitor haya detenido el vehículo simulado ni de un error de
dispatch/authority.

El mismo smoke ya había documentado 5 pasadas y 1 fallo intermitente antes de
este corte. Después del cambio se obtuvieron tres pasadas consecutivas:
`patrol-battery-return-20260910T205001-1`, `205134-1` y `205307-1`, todas con
la secuencia `JOIN_LOOP -> PATROL -> EXIT_LOOP -> RETURN_HOME -> AT_HOME`.
Una repetición posterior volvió a reproducir el mismo stall en
`patrol-battery-return-20260910T210709-1`, sin cambiar la causa observada.
La evidencia disponible clasifica el fallo como un stall transitorio de
progresión del simulador en el escenario de patrulla, no como una regresión
causal del PR ni como un stop de Collision Monitor. No se aumentaron timeouts,
se agregaron retries ni se modificó Collision Monitor.
