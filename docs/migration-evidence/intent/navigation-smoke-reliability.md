# Intención: confiabilidad observable de navegación y patrulla/HOME

## Alcance

- Fuente vigente: runs de GitHub Actions `32998562388` (intento 1) y
  `32932132311`, más el código de `salus_navigation` en `main` (`d488c9e`).
- Destino: BT de navegación, coordinación patrulla/route executor y probe de
  patrulla en `salus_robot`.
- Incluido: acuse asíncrono de `FollowPath`, identidad de rutas reemplazadas y
  evidencia causal del smoke.
- Fuera de alcance: cambiar geometría, tolerancias de goal/progress, timeouts
  funcionales del smoke, retries y comportamiento de hardware.

## Evidencia

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| run `32998562388`, job `98274864222` | El goal JOIN_LOOP `(0.67, 0.20) -> (3.68, 0.20)` fue aceptado; BT Navigator venció esperando el acuse de `follow_path`, que llegó después, y el robot quedó sin progreso | alta |
| nightly `32932132311` | Un JOIN_LOOP distinto quedó inmóvil con cuatro abortos del progress checker; el artefacto anterior no conservó comandos/path-health suficientes para atribuirlo al mismo mecanismo | alta |
| nightlies `32552787488`, `32618951074`, `32692130362`, `32932132311` | En fallos RETURN_HOME el goal posterior a cancelar el loop llegó a completarse, mientras el coordinador quedó `PAUSED` o siguió esperando | alta |
| `patrol_mission_coordinator.py` | Al vaciar `_route_mission_id`, cualquier estado no vacío —incluido el de la ruta cancelada— podía convertirse en autoridad para la ruta nueva | alta |
| reproducción local `navigation-free-world-20260826T184324-1` | El probe publicó cuatro goals RViz antes de observar `goal_active`, causó preemptions y evaluó comandos residuales del goal anterior | alta |

Hecho: discovery, lifecycle, scan, odometría, TF y servicios estaban activos en
el fallo citado. Inferencia delimitada: la ausencia de telemetría de comandos y
path-health impide afirmar que todos los JOIN_LOOP inmóviles comparten una sola
causa.

## Problema e intención

Una transición de patrulla no debe confundir el resultado tardío de la ruta
cancelada con la ruta que acaba de despachar. Del mismo modo, un acuse de action
server demorado por carga razonable no debe crear un goal huérfano: el BT debe
esperar dentro de un presupuesto explícito y acotado.

## Contratos e invariantes

- Las fases `JOIN_LOOP -> PATROL -> EXIT_LOOP -> RETURN_HOME -> AT_HOME` y el
  latch de batería no cambian.
- La identidad de la ruta cancelada queda excluida hasta observar un
  `mission_id` nuevo; después sólo ese ID puede gobernar la fase.
- `FollowPath` conserva las mismas acciones, retries y asserts; sólo explicita
  500 ms para el acuse, igual que los servicios de path-health del mismo BT.
- El probe conserva los límites funcionales y añade generaciones, eventos con
  timestamp, pose/distancia, comandos, resultado Nav2, estado de route executor
  y path-health.
- El probe de navigation core espera discovery, limpia sus baselines de
  comandos y publica el goal RViz una sola vez.

## Fallos y degradación

| Condición | Respuesta requerida | Evidencia/test |
| --- | --- | --- |
| poll tardío devuelve la ruta cancelada | ignorarlo hasta aparecer otro ID | `test_replacement_does_not_adopt_cancelled_route_mission_id` |
| acuse `FollowPath` tarda más que el default implícito | esperar hasta 500 ms, sin reintentar ni reducir asserts | `test_navigation_config_and_launch_keep_the_safe_contract` |
| JOIN_LOOP vuelve a quedar inmóvil | fallar y conservar odometría, distancia, comandos, path-health, generación y resultado | `smoke_patrol_battery_sim.py` |
| el poll de estado tarda detrás de `/goal_pose` | no republicar/preemptar el goal; esperar la aceptación del único mensaje | `test_navigation_smoke_probe.py` |

## Decisiones descartadas

- Aumentar los timeouts del smoke o añadir retries: ocultaría el síntoma.
- Relajar progress checker o goal tolerance: la evidencia no demuestra que
  sean incorrectos.
- Culpar Fast DDS SHM o PR #39: no hay causalidad y el nodo de #39 no participa
  en los launches.

## Pruebas y aceptación

- Unitarios focalizados de `salus_navigation`.
- Smoke aislado y cinco repeticiones secuenciales.
- Secuencia zones/route/patrol/snapshot igual al job de CI.
- Suite completa y los tres jobs requeridos del PR draft.

## Estado de evidencia

- Estado: corrección de confiabilidad de simulación; no cambia el estado de
  paridad/hardware del subsistema.
- No validado en hardware.
- Pendiente: clasificar con la nueva evidencia cualquier JOIN_LOOP inmóvil que
  no contenga el timeout de acuse `FollowPath`.
