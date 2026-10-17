# Evidencia PC: admisión de maniobras del planner — 17/10/2026

## Revisión y alcance

Base `c6bed23` (PR #327), rama `agent/maneuver-guard`. El checkout canónico
`6775e24`, `.agents/` y los worktrees anteriores se preservaron. No se conectó
ni desplegó a Jetson y no se inició movimiento físico. Se preservan PR #325/#327,
los anchors de #315, el horizonte de #282 y las semánticas multi-pose de #244.

Diseño: ADR 0016 y ficha de intención del mismo día. El dominio puro no publica
velocidades ni acredita checkpoints. Los candidatos se inspeccionan completos;
la selección del cursor activo no puede saltar a un cruce futuro. Una detección
combina tangente/retorno longitudinal, o giro firmado con retorno a un sector
previo. Una ruta cerrada con sus poses de intención no tiene una prohibición
global de vueltas. Umbrales y fronteras tienen fixtures analíticos, ambos
sentidos, varias densidades, invariancia por rotación y un path real de Smac
capturado en PC. No son umbrales físicos de frenado calibrados.

## Resultados y comandos

| Verificación | Resultado |
| --- | --- |
| tools/build.sh | 14 paquetes compilados en Humble |
| tools/test.sh | 1341 tests, cero errores/fallos, cuatro omitidos; harness aprobado |
| smoke_maneuver_guard_runtime.py, DDS aislado 83 | PASS: freno, adopción, deadline total, causa en payload Cockpit, takeover/cancel y misión preservada |
| mismo fault injection con MANEUVER_SMOKE_PAUSED_CLOCK=1 | PASS: reloj ROS inmóvil en cero, vencimiento y NEEDS_OPERATOR por reloj steady |
| smoke_web_cockpit.sh | PASS: contrato WebSocket/estado operacional |
| smoke_integration_sim.sh | PASS: contratos estructurales integrados |
| smoke_safety_sim.sh | PASS: stop de Collision Monitor y arbitraje |
| smoke_navigation_core_sim.sh | PASS: goal, movimiento simulado, cancelación y manual |
| smoke_route_executor_sim.sh (open) | PASS: checkpoints, continuidad y perfiles |
| SMOKE_ROUTE_SCENARIO=loop smoke_route_executor_sim.sh | PASS con route_tangent: vuelta completa e ingreso a la siguiente |
| matriz maneuver_guard.yaml --jobs 1 | 2/3 gates funcionales aprobados; derecha R4 bloqueada por path con rulo, ver abajo |
| matriz maneuver_guard_gentle.yaml --jobs 1 | 2/2 gates funcionales aprobados (izquierda/derecha R8) |
| matriz maneuver_guard_r4_probe.yaml --jobs 1 | Fallo funcional de llegada conservado; stop correcto de un rulo capturado |
| smoke_patrol_battery_sim.sh | Falla anterior a despacho: anchor no_near_segment; reproducida sin este cambio en c6bed23 |

Los artefactos completos permanecen bajo `artifacts/` (ignorados por Git):

- `maneuver-guard-runtime.json` contiene el payload GetRouteMissionState
  serializado por salus_web, misma misión e índice pendiente, cero créditos;
- `smokes/navigation-free-world-20261017T135002-1`;
- `smokes/routes-free-world-20261017T135931-1` (open);
- `smokes/routes-free-world-20261017T140245-1` (loop con tangentes);
- `evaluations/maneuver-guard`, `maneuver-guard-r4-probe` y
  `maneuver-guard-gentle`, con manifest, configuración efectiva, DDS, semillas,
  comandos, odometría, gates y reports;
- los bundles instrumentados añaden `plans.csv` y `path_health.csv`, sin
  cambiar los gates ni publicar otro plan.

## Derecha R4: fallo de llegada clasificado, sin relajar admisión

El request del caso existente right_tight_valid pide `(4,-4,-pi/2)` desde el
origen. Esto no garantiza que Smac devuelva el cuarto de círculo nominal.
La matriz devolvió un path de 31.6168 m, 82 poses, que alcanza x=10.8543 m e
y=5.2502 m antes de regresar al objetivo. El giro firmado con revisita detecta
PLANNER_LOOP. No es una curva de 90° rechazada sólo por el ángulo.

El probe independiente conservó 15 planes y estados WAITING_REPLAN →
NEEDS_OPERATOR. Sus 185 muestras de comando final tienen velocidad cero.
Se mantiene el fallo del evaluador de llegada/terminal/turn_sign; no se cambia
su criterio para convertir un stop en éxito de navegación. El mismo request
ideal en arco R4 pasa la política pura. El path de Smac capturado se versiona
como `src/salus_navigation/test/fixtures/planner_right_r4_loop.json`.

La protección cumple el objetivo de no ejecutar ese rulo; no asegura que un
planner determinista encuentre una alternativa manteniendo iguales sus
restricciones. Corregir/tunear el planner o el EKF queda fuera de este corte.

## Patrol/HOME y readiness

El smoke de Patrol/HOME queda en JOIN_LOOP porque el anchor rechaza
no_near_segment antes de despachar Nav2. Se reprodujo con una compilación
limpia de c6bed23, sin guard, en un checkout temporal separado. Ambos muestran
IDLE → AT_HOME → IDLE → JOIN_LOOP y el mismo rechazo. Se conserva como fallo
preexistente, no como prueba aprobada de Patrol/HOME para este cambio.

El primer smoke open no alcanzó readiness en 40 s (Nav2 unconfigured). Una
segunda ejecución sin cambios funcionales ni aumento de timeouts pasó. Su
causa no quedó demostrada; se registra como fallo de readiness no reproducido.

El fixture loop histórico forzaba yaw este en todos los vértices. El guard
retuvo el cuarto objetivo, con tres checkpoints realmente alcanzados y freno
100%, sin saltar al siguiente. El smoke de circuito intencional usa ahora la
política route_tangent existente; no cambia yaws del producto ni los explícitos.
La opción SMOKE_ROUTE_AUTO_YAWS=0 conserva el fixture negativo para revisión.

## Cockpit y salida del bloqueo

La rama remota `migration/salus-robot-cockpit@383f3ce` todavía esperaba nombres
BLOCKED_* y no refrescaba la recuperación. El PR compañero
[AEyeSecurity/cockpit#26](https://github.com/AEyeSecurity/cockpit/pull/26), separado
de su main y checkout de cámara, muestra WAITING_DATA/NEEDS_OPERATOR, causa,
plazo y acción de operador; CLEAR retira el bloqueo. No inventa retries para
la maniobra ni publica comandos. Build aprobado; 172 tests aprobados y uno
omitido, incluyendo DOM del mapa y panel. Sin Tauri ni hardware.

Los timers de admisión y recuperación usan explícitamente STEADY_TIME: el
plazo vence y Cockpit recibe el estado terminal incluso con /clock pausado.
El fault injection repetido sin publicador /clock comprueba que todos los
relojes ROS permanecen en cero durante stop, adopción, timeout y cancelación.

El plazo es monotónico y sólo nace con el primer rechazo. Datos stale,
replans rechazados y causas distintas no lo renuevan. Un candidato sano no
libera el path anterior: debe aparecer adoptado como ACTIVE. El BT cancela
el follower viejo aunque planificación continúe RUNNING. La espera terminal
queda enclavada; un path sano tardío no la libera. CANCEL y un nuevo pedido
explícito del operador conservan las reglas existentes. El nuevo dispatch
confirma RESET; un replan interno nunca lo solicita.

## Pendientes y límites

No se midieron distancias físicas de parada, latencia extrema DDS/BT, frenado
con pendiente, precisión angular real ni paridad en robot. El timer de 100 ms
y las consultas BT no son una certificación temporal. La matriz es acotada,
no una campaña de fiabilidad. Este trabajo no despliega ni autoriza movimiento
real, y no hereda confirmaciones de seguridad anteriores. Los fallos descritos
se mantienen visibles para la revisión del PR.
