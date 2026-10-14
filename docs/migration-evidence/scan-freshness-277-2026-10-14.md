# #277 — timestamps de scans y parada ante fuente vencida

Análisis del 14/10/2026. Horas de capturas en UTC. Base de código `2fc6973`.

## Resultado

Se confirma una brecha actual de safety: recibir continuamente `/scan_clean` con
stamp vencido renovaba la autorización del árbitro final, mientras Collision
Monitor Humble 1.1.20 ignoraba la fuente. La reproducción PC falla antes del
cambio y pasa al conservar el presupuesto de frescura desde el stamp fuente.
No se modifican `source_timeout=1.0`, `transform_tolerance=0.5`, rates ni filtros.

El primer origen de los dos retrasos originales sigue sin demostrarse. La cola
depth 5 del filtro antiguo es un contribuyente histórico con A/B documentado,
ya corregido por #279. Los bags posteriores no reproducen el warning. No cerrar
#277 ni atribuir los eventos al deadlock de tf2 0.25.23.

## Originales del 14/09 recuperados de Jetson

Se copiaron por SSH sólo los dos artefactos solicitados y dos ventanas acotadas
de journal. SHA256 local/remoto idéntico:

- JSON: `1b0d66c4913bc100d794e7312511af0b3aa6225b3624d2c3a15f56f746a408d1`.
- Markdown: `8180c92decb5bd27c3bf304f7c7c97cb036ef028c88adf2f7c5c955541b728f1`.

El gate corresponde a `f918d4b`, 16:26:08–16:29:08 UTC, 180,038 s.
El JSON agrega odometría, TF, paths y comandos, pero **no registra scans ni
recepciones por etapa**. Su lista `errors=[]` no significa ausencia de warnings.

| Evento UTC | ROS del log | Edad declarada | Comando posterior registrado |
|---|---|---|---|
| 16:26:08.548563 | 1789403168.548563198 | 1,055044 s | 7 ms después: auto 1,600 m/s |
| 16:28:06.417938 | 1789403286.417938118 | 1,021683 s | 22 ms después: auto 1,216 m/s |

Las líneas exactas de ambos eventos dicen `Latest source and current collision
monitor node timestamps differ ... Ignoring the source.`. Son rechazos de
**edad de fuente**, previos al lookup TF de `Scan::getData`; ese texto no es
un diagnóstico de extrapolación TF. No conocemos el stamp exacto de la fuente:
restar el desfase al timestamp del log da sólo una aproximación porque el
tiempo usado por el monitor se toma antes de emitir el log.

El journal confirma comandos positivos alrededor de los warnings; no contiene
el tópico `/cmd_vel_safe` ni prueba física de movimiento o parada. La ventana
del segundo evento contiene `Control loop missed its desired rate of 20Hz`.
El informe original menciona load 19,73/15,37/12,64 sobre 6 CPU y filtro ~92%
en una ventana anterior; no es CPU medida simultáneamente por callback.

## Procedencia por captura

| Captura | Ventana UTC | Duración | Versión y cobertura |
|---|---|---:|---|
| `salus_lidar_ground_20260924` | Indeterminada | — | DB parcial, excluida; YAML de filtro antiguo disponible, SHA no demostrado |
| `salus_nav_live_capture_20260924` | 22:50:03.647–22:50:44.337 | 40.690 s | `af9aa72` citado en análisis #307/#315; sin contexto local, sin scans/rosout |
| `salus_nav_loops_20260924T235128Z` | 23:51:30.629–23:53:33.478 | 122.850 s | `a0940cb`, contexto local; cuatro etapas + TF/rosout/comandos |
| `salus_nav_loops_20260924T235727Z` | 23:57:29.681–23:59:00.510 | 90.829 s | `a0940cb`, contexto local; cuatro etapas + TF/rosout/comandos |
| `salus_nav_loops_20260924T235901Z` | 23:59:03.538–00:00:20.761 | 77.223 s | `a0940cb`, contexto local; cuatro etapas + TF/rosout/comandos |
| `salus_nav_loops_20260925T000021Z` | 00:00:23.967–00:15:35.709 | 911.742 s | `a0940cb`, contexto local; cuatro etapas + TF/rosout/comandos |

Todos los ocho DB completos coinciden con hashes de `analysis-307/inventory.json`
y pasan `PRAGMA quick_check`. La metadata y los conteos se contrastaron con las
tablas SQLite originales. El bag corto no permite clasificar su percepción.

Los cuatro contextos de navegación declaran `a0940cb` y la misma imagen
`sha256:a52b2fe21737523b0915e9d1e5049f88a4dca73c4131e51538211b71abaaaec4`.
En ese SHA, el código esperado usa segmentación radial de suelo y depth 1.
Esto identifica la configuración fuente esperada, **no certifica** binarios ni
overrides efectivos: no hay dump de parámetros por cada bag. No se identifica
un bag completo de percepción vieja con procedencia suficiente. La captura
parcial conserva YAML con `ground_tolerance_m`/`wheelbase_m`, compatible con el
filtro anterior, pero no se debe extrapolar ese YAML a los cuatro loops.
No se trata la jornada como un A/B controlado.

## Stamps, Hz y gaps por bag

`age` = timestamp SQLite del grabador menos `header.stamp`, en segundos.
No es tiempo exacto de procesamiento ni recepción en Collision Monitor.
Hz = (N−1)/(última−primera recepción grabada). Percentiles con interpolación
lineal. Stamps y deltas se calculan desde enteros nanosegundo.

| Bag (sufijo) | Tópico | N | Hz | age p50 | age p95 | age máx | gap grabador máx | gap stamp máx |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 20260924T235128Z | `/scan_3d` | 1227 | 10.001 | 0.110 | 0.123 | 0.148 | 0.135 | 0.120 |
| 20260924T235128Z | `/obstacles_cloud` | 973 | 7.944 | 0.274 | 0.373 | 0.526 | 0.386 | 0.399 |
| 20260924T235128Z | `/scan` | 973 | 7.944 | 0.276 | 0.377 | 0.528 | 0.380 | 0.399 |
| 20260924T235128Z | `/scan_clean` | 973 | 7.944 | 0.280 | 0.382 | 0.539 | 0.388 | 0.399 |
| 20260924T235727Z | `/scan_3d` | 906 | 10.001 | 0.112 | 0.126 | 0.151 | 0.139 | 0.124 |
| 20260924T235727Z | `/obstacles_cloud` | 670 | 7.394 | 0.275 | 0.416 | 0.580 | 0.464 | 0.400 |
| 20260924T235727Z | `/scan` | 670 | 7.394 | 0.278 | 0.419 | 0.582 | 0.464 | 0.400 |
| 20260924T235727Z | `/scan_clean` | 670 | 7.393 | 0.284 | 0.425 | 0.584 | 0.470 | 0.400 |
| 20260924T235901Z | `/scan_3d` | 770 | 10.001 | 0.112 | 0.127 | 0.153 | 0.143 | 0.126 |
| 20260924T235901Z | `/obstacles_cloud` | 567 | 7.357 | 0.283 | 0.402 | 0.566 | 0.379 | 0.400 |
| 20260924T235901Z | `/scan` | 567 | 7.358 | 0.285 | 0.410 | 0.568 | 0.385 | 0.400 |
| 20260924T235901Z | `/scan_clean` | 567 | 7.358 | 0.288 | 0.418 | 0.575 | 0.370 | 0.400 |
| 20260925T000021Z | `/scan_3d` | 9116 | 10.000 | 0.112 | 0.127 | 0.161 | 0.152 | 0.123 |
| 20260925T000021Z | `/obstacles_cloud` | 6579 | 7.218 | 0.283 | 0.423 | 0.641 | 0.470 | 0.502 |
| 20260925T000021Z | `/scan` | 6579 | 7.218 | 0.285 | 0.427 | 0.644 | 0.472 | 0.502 |
| 20260925T000021Z | `/scan_clean` | 6579 | 7.218 | 0.289 | 0.434 | 0.651 | 0.473 | 0.502 |

Ninguna de esas series tiene edad >1 s, edad negativa, stamp regresivo ni
duplicado. El bag largo confirma independientemente el age p95=0,434 s ya
calculado en #307. `/scan_3d_raw` no está grabado. No hay mensajes `/clock`
grabados; la configuración real esperada usa `use_sim_time=false`.

## Emparejamiento exacto de stamps por frontera

Se preserva multiplicidad; sólo se emparejan stamps únicos presentes en ambos
tópicos. Los deltas son diferencias de almacenamiento/recepción al grabador,
no duración del callback. No se empareja por cercanía temporal.

| Bag | Frontera | Pares | Delta p50 ms | Delta p95 ms | Delta máximo ms |
|---|---|---:|---:|---:|---:|
| 20260924T235128Z | `/scan_3d -> /obstacles_cloud` | 972 | 162.40 | 260.59 | 403.38 |
| 20260924T235128Z | `/obstacles_cloud -> /scan` | 973 | 1.26 | 5.57 | 15.83 |
| 20260924T235128Z | `/scan -> /scan_clean` | 973 | 3.32 | 12.09 | 37.82 |
| 20260924T235727Z | `/scan_3d -> /obstacles_cloud` | 668 | 163.49 | 301.78 | 444.63 |
| 20260924T235727Z | `/obstacles_cloud -> /scan` | 670 | 1.25 | 6.31 | 22.25 |
| 20260924T235727Z | `/scan -> /scan_clean` | 670 | 3.10 | 14.73 | 50.51 |
| 20260924T235901Z | `/scan_3d -> /obstacles_cloud` | 565 | 171.47 | 288.00 | 452.22 |
| 20260924T235901Z | `/obstacles_cloud -> /scan` | 567 | 1.35 | 5.91 | 19.11 |
| 20260924T235901Z | `/scan -> /scan_clean` | 567 | 3.15 | 14.68 | 44.38 |
| 20260925T000021Z | `/scan_3d -> /obstacles_cloud` | 6578 | 171.29 | 307.56 | 523.73 |
| 20260925T000021Z | `/obstacles_cloud -> /scan` | 6579 | 1.30 | 5.95 | 40.34 |
| 20260925T000021Z | `/scan -> /scan_clean` | 6579 | 3.30 | 14.11 | 63.23 |

Uno o dos stamps de salida de cada bag preceden la cobertura upstream;
todos los unmatched exactos están conservados en `summary.json`. No implican
por sí solos restamping. Los deltas scan→clean incluyen 5/2/5/29 valores
negativos según bag, hasta −0,325 ms: esto confirma que el orden del grabador
no es un reloj causal preciso del pipeline. No recortar esos datos a cero.

El incremento dominante medido ocurre entre `/scan_3d` y `/obstacles_cloud`.
La salida cae a 7,2–7,9 Hz frente a ~10 Hz de entrada. Es compatible con costo
del filtro, TF y scheduling/colas en esa frontera; estos bags no separan esas
contribuciones. Los filtros posteriores añaden milisegundos en p95.

## Pipeline efectivo y política de stamps

- RS16 real: `rs16.yaml` publica directamente `/scan_3d`, con
  `use_lidar_clock=false`, `ts_first_point=true`. Un giro adquirido durante
  ~0,1 s ya puede llegar con esa edad al stamp del primer punto. Es configuración
  esperada, no dump del driver de cada bag.
- `cloud_normalizer` copia el mensaje y sólo cambia el frame. Se usa en el
  pipeline de entrada raw de simulación; el launch RS16 real revisado no lo
  interpone. No inventar una frontera raw→normalizado en estos bags.
- Ground filter: en `f918d4b` transforma/filtra puntos en Python y conserva
  stamp; input sensor-data depth 5. #279 cambia sólo input a depth 1; #291
  vectoriza; `a0940cb` usa además el filtro radial. El lookup del filtro es
  `Time()` (último TF), timeout 0,05 s. No corrige adquisición ni reemplaza stamp.
- `pointcloud_to_laserscan` recibe `/obstacles_cloud` en `base_footprint` y
  publica `/scan`. Todos los stamps de salida grabados coinciden exactamente
  con su cloud: evidencia observada de conservación en estas capturas.
- `scan_noise_filter` hace deepcopy del LaserScan y limpia rangos; conserva
  stamp. Todos los stamps scan→clean coinciden.
- Collision Monitor compara su reloj ROS con `header.stamp` antes del TF
  temporal hacia base mediante odom. Una fuente fuera de tiempo no aporta
  puntos; el procesamiento comienza con acción DO_NOTHING y el comando de
  entrada. La ausencia de puntos por fuente ignorada no equivale a stop.

Fuentes upstream: [sourceValid, Nav2 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_collision_monitor/src/source.cpp),
[Scan::getData](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_collision_monitor/src/scan.cpp),
[CollisionMonitor::process](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_collision_monitor/src/collision_monitor_node.cpp).

## TF, carga y deadlock

En los cuatro loops, `/tf` dinámico mantiene gaps máximos <0,17 s; las edades
máximas de transform recibida son <0,17 s. Los stamps de `/scan_clean` quedan
dentro del intervalo de TF ya grabado en casi todos los casos:

| Bag | odom→base bracketed / scans | map→odom bracketed / scans |
|---|---:|---:|
| 20260924T235128Z | 971/973 | 972/973 |
| 20260924T235727Z | 670/670 | 669/670 |
| 20260924T235901Z | 565/567 | 565/567 |
| 20260925T000021Z | 6578/6579 | 6578/6579 |

Las excepciones son de borde de inicio de bag. Esto sólo verifica cobertura
temporal aproximada del recorder, no disponibilidad en el buffer del consumidor,
interpolación válida, autoridad única ni el lookup base a tiempo actual. TF
estáticos antiguos no están stale: su stamp no limita su validez. Los logs
grabados no contienen `timestamps differ`, `Ignoring the source` ni errores
TF/extrapolación/colas. No hay muestreo CPU/profiling por bag; rate bajo por sí
solo no distingue filtro lento de starvation. No hay evidencia de salto de
reloj en las series medidas; falta estado NTP/clock del sensor por captura.

La imagen local original tiene tf2/tf2_ros 0.25.22 y sólo se usó para extraer
CDR offline. Se construyó otra imagen `salus-robot:issue277-validation`, sin
reemplazar etiquetas existentes: ambos paquetes 0.25.24 y Nav2 1.1.20.

La consulta read-only del 14/10 a Jetson observa servicio activo, checkout
`0b7e667` limpio, contenedor creado el 10/10 con imagen `a52b2fe…`, ambos
paquetes tf2 0.25.22 y Nav2 1.1.20. `docker diff` no registra cambios en dpkg
o rutas tf2. Coincide con la imagen declarada en los loops y capturas posteriores
del 14/09, pero falta manifiesto de paquetes original por evento. El deadlock
ABBA reproducido en #323 requiere tf2 0.25.23; **no hay evidencia de esa versión
en estas capturas**. No usar #323 como causa de #277. Otros bloqueos no quedan
descartados sin trazas. No se cambiaron servicio, fuente, imagen ni parámetros
del robot, ni se publicó ningún comando.

## Corrección y verificación de software

El adaptador calcula edad en reloj ROS desde `header.stamp` y la entrega al
árbitro junto a recepción monotónica. El árbitro proyecta adquisición sobre
su reloj steady: tiempo restante = timeout existente − edad de fuente.
Rechaza edades negativas/no finitas; el adaptador invalida stamp cero.
Llegadas continuas con source vencido no renuevan frescura. Al llegar scans
frescos puede recuperarse. Este contrato no verifica el TF interno del monitor
ni detecta todos los posibles bloqueos de su callback.

La fase añadida a la prueba PC existente mantiene TF, scans y comandos a
frecuencia de fixture pero envejece el stamp 2 s; comprueba el warning real
de Nav2 y que, pasado el presupuesto, **todas** las salidas son safety/zero.
Control positivo fresco antes, recuperación después y obstáculo/ausencia de
scans se ejecutan en la misma composición, sin otra infraestructura.

Validación:

- Baseline previo al fix: falla por comandos positivos con fuente vencida.
- Focalizados: 12 passed (árbitro + composición real PC).
- Build completo: 16 paquetes terminados.
- Suite de paquetes ejecutada: 15 paquetes, CTest sin fallos. Inventario
  separado de resultados de paquetes ejecutados: 1254 tests, 0 errors,
  0 failures, 1 skipped; 36 tests auxiliares host pasan.
- Smoke safety con clock de simulación: pasa; self-test del harness pasa.
- `tools/test.sh` se detiene en el validador por documentación preexistente
  no versionada bajo `.agents/`. Se preservó esa carpeta y se ejecutaron
  por separado todos los pasos restantes. El validador pasa sobre una copia
  de archivos versionados más los documentos nuevos (14 paquetes propios),
  sin mover ni modificar `.agents/`. El reporte global inicial también
  arrastraba 19 tests/15 fallos históricos de `rslidar_msg`, paquete excluido
  por el script; los XML se conservaron, sin borrarlos ni alterar tests.

## Pendientes y criterio de cierre

Confirmado: backlog histórico contribuyente ya corregido; conservación de
stamps en los loops; brecha actual de árbitro reproducida y corregida en PC.
Pendiente: asignar el primer origen de los dos eventos del 14/09. Los originales
recuperados no contienen los mensajes necesarios para hacerlo retrospectivamente.
No afirmar que los loops validan el consumidor ni que el robot ya tiene el fix.

Una futura captura autorizada puede usar el probe existente de #278 con
stamps/recepción por etapa, rosout y TF/carga durante un warning natural;
si la frontera sigue ambigua, registrar entrada/salida del callback de filtro
en una prueba acotada. No requiere cambiar escena, umbrales ni inducir riesgo.
No se propone ensayo físico como requisito para aceptar la corrección PC.

Los scripts, CSV por etapa, controles de deserialización, hashes, resúmenes,
originales, journal y logs de pruebas quedan en el directorio externo
`artifacts/robot-real-2026-09-24/analysis-277/`. `analyze.py` se ejecuta con
inputs read-only, `/output` escribible y red deshabilitada; no inicializa ROS.
Por cada tópico header en cada DB se coteja el parser CDR con deserialización
ROS completa; se conservan deltas negativos y fronteras ausentes. No se ha
desplegado, movido ni modificado el robot; #307/#315/#320 quedan fuera del cambio.
