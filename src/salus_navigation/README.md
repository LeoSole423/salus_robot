# salus_navigation

La observabilidad espacial tiene su contrato y límites definidos en el ADR
0004. `nav_snapshot_server` genera bajo demanda PNGs deterministas de costmaps,
keepout, footprint, zonas de colisión, `/scan_clean` y plan, sin ser dueño de
WebSocket, rosbag ni telemetría compacta.

Responsabilidad: navegación segura, Nav2 y zonas no-go. El corte actual ofrece
goals LL escalares, chunks LL multi-pose, rutas abiertas/circulares y zonas dinámicas GeoJSON; el
runtime real también compone las APIs de rutas y patrol/HOME sin crear otra
autoridad de velocidad.

- API de zonas: `/zones_manager/set_geojson`, `/zones_manager/get_state` y
  `/zones_manager/reload_from_disk`.
- Las zonas se convierten con `/fromLL` y publican
  `/zones_manager/projected_keepouts` como estado vectorial revisionado,
  reliable y transient-local. Los datos operativos persisten en `runtime/zones/`
  y no se versionan; no se genera una máscara global PGM.
- API de rutas: `/route_executor/set_route_mission_ll`,
  `/route_executor/cancel_route_mission` y `/route_executor/get_route_mission_state`.
- Los checkpoints sin yaw manual siguen por defecto la tangente de la curva,
  también en Patrol/HOME. El gateway de Cockpit puede enviar
  `auto_yaw_policy=route_tangent` explícitamente; `legacy` solicita el cálculo
  anterior. El primer checkpoint automático de cada pedido usa la orientación
  actual del robot sólo si difiere más de 60° de la bisectriz; los siguientes
  conservan la bisectriz. Los yaws manuales siempre prevalecen.
- Recuperación de rutas bloqueadas mediante una política pura con espera por
  datos, cooldown, limpieza de costmaps y límite de intentos observable en los
  campos `blocked_*`. El retry conserva el sufijo del chunk activo posterior a
  checkpoints acreditados consecutivamente, incluso al cruzar el cierre de
  un loop; la proximidad no acredita checkpoints originales.
  `./tools/smoke_route_recovery_sim.sh` inyecta un STOP sim después de
  progreso acreditado y registra el plan, la odometría y el comando final
  durante el retry y cancelación.
  La preparación LL es asíncrona y atómica; el ejecutor no publica velocidad
  ni invoca Nav2 directamente. Normalmente el chunk termina en el siguiente
  checkpoint. Si una pose queda fuera del horizonte radial de navegación,
  termina en la última pose alcanzable, que puede ser un sintético, y continúa luego
  hasta el checkpoint real. `nav_goal_horizon_m` es `float`, default `120.0 m`,
  rango finito `(0, +inf)`: limita la distancia de todas las poses del pedido
  desde el robot al despacharlo. El costmap global actual mide 300×300 m y
  deja así 30 m de margen. Si la primera pose pendiente ya está fuera se pausa con
  `ROUTE_GOAL_HORIZON_UNREACHABLE`. La expansión descarta sintéticos situados
  a menos de medio `leg_spacing_m` del checkpoint siguiente para evitar metas
  casi coincidentes con yaws diferentes. El chunk se despacha por
  `nav_command_server`: una pose usa `NavigateToPose` y varias usan
  `NavigateThroughPoses`. Sólo los checkpoints originales incrementan el
  progreso de misión o ejecutan acciones; un sintético terminal no acredita
  un checkpoint.
- Las acciones `brake_hold` y `set_navigation_profile` se ejecutan sólo en
  checkpoints originales. Tienen estado explícito y se cancelan ante takeover,
  collision stop o cancelación de misión.
- `navigation_profile_coordinator` aplica `urban`/`rural` como transacción sobre
  filtro de suelo, inflation local/global y controlador. Ante rechazo restaura
  todos los componentes ya modificados.
- API de patrulla: `/route_executor/set_patrol_mission_ll`,
  `/route_executor/cancel_patrol_mission`,
  `/route_executor/get_patrol_mission_state` y
  `/route_executor/request_return_home`. `patrol_mission_coordinator` conserva
  HOME/salida/loop/retorno, convierte toda la misión antes de reemplazar la
  activa, persiste `runtime/patrol/patrol_mission.json` atómicamente y delega
  los tramos a `route_executor`. La guardia válida en
  `/battery_mission_guard` tiene precedencia sobre el fallback de
  `/battery_state`; un retorno iniciado queda enclavado hasta HOME o
  cancelación explícita según ADR 0003.
- Parámetros de batería de patrulla: `battery_guard_topic` (default
  `/battery_mission_guard`), `battery_state_topic` (default `/battery_state`),
  `battery_guard_timeout_s` (3,0 s, finito y positivo) y
  `low_battery_threshold_pct` (25 %, rango 0–100, sólo fallback).
- `patrol_mission_sim.launch.py` requiere que `route_executor` ya esté activo.
  En el checkpoint integrado se habilitan ambos con
  `launch_routes:=true launch_patrol:=true`. En real,
  `navigation_real.launch.py` inicia exactamente una instancia de
  `route_executor_real.launch.py` y `patrol_mission_real.launch.py`, ambas con
  `use_sim_time=false`. El launch conserva `runtime/patrol/` como contrato; el
  servicio final lo apunta a `/ros2_ws/log/runtime/patrol`, persistente en el
  workspace preparado.
- `navigation_real.launch.py` inicia también exactamente una instancia del
  `nav_snapshot_server` mediante `navigation_snapshot_real.launch.py`, con
  `use_sim_time=false` y el mismo `navigation_snapshot.yaml`. Es una capacidad
  auxiliar: no forma parte del lifecycle ni de los gates de readiness y no
  publica TF ni comandos.
- `path_health` conserva el plan mientras siga sano y evalúa hasta 12 m por
  delante con footprint orientado, colisión, inflación sostenida, progreso y
  desviación transversal. Evalúa la pose desde TF en el frame del path y usa
  `EvaluatePathHealth` con contexto explícito; ante costmap o TF vencidos
  produce `STOP_AND_WAIT`. El BT valida el path candidato antes de copiarlo al
  path activo que consume `FollowPath`; no ejecuta `SmoothPath` ni inicia un
  `smoother_server`. `SmacPlannerHybrid` conserva `smooth_path: false`.
  El BT multi-pose añade poda de goals superados y recuperaciones separadas de
  costmaps local/global, sin maniobras `Spin`/`BackUp` incompatibles con
  Ackermann.
- La política de ocupación progresiva del path activo usa `near_horizon_m`
  (`float`, default 5,35 m, rango abierto `0 < valor < 12`): una marca en ese
  tramo pide replan inmediato. Más lejos, hasta los 12 m inspeccionados,
  `far_persistence_s` (`float`, default 1,0 s, finito y positivo) exige al
  menos dos costmaps con stamps distintos antes de pedir replan. Una marca
  aislada conserva el path y se publica como `far_obstacle_observed`; la marca
  confirmada debe permanecer a menos de 1 m de la primera posición muestreada
  sobre el path. El despeje reinicia la observación. El horizonte inicial corresponde a la zona
  de slowdown más externa del Collision Monitor real, y la persistencia al
  baseline existente de recuperación de ruta; el umbral lejano se redujo tras
  observar en Gazebo que el replan llegaba demasiado cerca del obstáculo. Ambos requieren calibración
  física antes de afirmar distancia segura de frenado. Los paths candidatos
  siguen rechazando cualquier colisión o inflación sostenida dentro de los
  12 m. El BT delega la validez del path activo en `path_health`; la consulta
  global `IsPathValid` de Humble recorría todo el path restante y adelantaba
  un replan por una sola celda letal lejana. RPP y Collision Monitor conservan
  la respuesta de colisión cercana independiente. Una revisión nueva de
  `/zones_manager/projected_keepouts` (`ProjectedKeepoutState`, reliable,
  transient-local; productor `zones_manager`, consumidor `path_health`)
  pide replan inmediato del path activo, sin aplicar la espera de ocupación
  lejana. Si no existe un publicador de zonas, la política usa sólo costmap,
  TF y path como antes.
- `nav_observer` publica eventos de lifecycle, bloqueo local y replanning sin
  modificar Nav2 ni poseer comandos. La decisión sobre el plugin BT delgado y
  `TraceReplan` está registrada en [ADR 0002](../../docs/decisions/0002-nav2-hardening-and-legacy-bt.md).
- `nav2_startup_coordinator` mantiene Nav2 sin activar hasta observar reloj y
  odometría progresivos, TF global reciente, scan válido y la máscara keepout
  cuando está habilitada. Publica la causa y el estado en
  `/navigation_startup/diagnostics`; no altera parámetros de navegación.
- `nav_command_server` mantiene una meta como activa mientras Nav2 procesa su
  cancelación. `CancelNavGoal` espera hasta `cancel_result_timeout_s` (12,0 s
  por defecto, mínimo 0,1 s) para observar un resultado terminal; un reemplazo
  no despacha la meta siguiente antes de esa transición. El takeover manual
  conserva autoridad inmediata de comando y la cancelación termina de forma
  asíncrona y observable.

Pruebas: `colcon test --packages-select salus_navigation` y
`./tools/smoke_navigation_zones_sim.sh`, `./tools/smoke_navigation_core_sim.sh`
y `./tools/smoke_route_executor_sim.sh`. El retorno integrado se valida con
`./tools/smoke_patrol_battery_sim.sh` usando una guardia sintética aislada.
El snapshot se prueba por separado con `./tools/smoke_navigation_snapshot.sh`.
