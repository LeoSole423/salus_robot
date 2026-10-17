# ADR 0016: Admisión geométrica y espera acotada del planner

- Estado: implementada para revisión PC; hardware no validado
- Fecha: 2026-10-17
- Relación: ADR 0002, 0011, 0012; #244, #282, #315, PR #325/#327

## Decisión

`salus_navigation/maneuver_guard.py` contiene geometría pura y `ManeuverWait`.
No infiere progreso de misión ni modifica metas. Las poses pedidas delimitan
piernas de intención: una vuelta explícita de patrulla no es un rulo introducido
entre dos visitas. Se utiliza un radio geométrico de 1.2 m para asociar poses,
igual al waypoint_reached_tolerance_m predeterminado; esto no acredita visitas
ni modifica las tolerancias Nav2/mission. No existe una excepción global para rutas loop.

Una pierna con tangente opuesta >=135° y retorno longitudinal >=1.17 m se
rechaza como PLANNER_U_TURN. Una acumulación firmada >=270° junto con retorno dentro de 1.17 m a un
sector previo se rechaza como PLANNER_LOOP. Un arco abierto no es un rulo
por acumular ángulo solamente. La cuerda de cada pierna es referencia longitudinal; el heading
TF actual también se usa en la primera. La longitud del footprint (1.17 m),
el cuadrante posterior y tres cuartos de vuelta motivan las cotas; los fixtures
analíticos congelan las fronteras. No son cotas de frenado ni calibración física.
Una curva de 120°, un giro lateral y una S sin retorno suficiente pasan.

Los candidatos se inspeccionan completos. El path activo utiliza un cursor
monótono con búsqueda hacia delante de 4 m (igual que RPP), para no proyectarse
sobre una intersección futura ni borrar el rulo antes de comprobarlo. Un salto
de pose fuera de esa ventana no autoriza saltar geometría. La política previa
de coste, frescura, histéresis y replan sigue siendo requisito de aceptación.

La primera detección fija un plazo monotónico (15 s por defecto). Nuevos paths,
causas, TF/costmaps vencidos y fallos del planner no lo reinician. Un candidato
sano se devuelve aceptable al BT, manteniendo STOP_AND_WAIT en el tópico hasta
que se evalúe ese mismo path completo como ACTIVE. Al vencer, la causa original
queda enclavada hasta un nuevo pedido explícito. El timer de 100 ms mantiene el
stop y hace observable el vencimiento aunque dejen de llegar requests del BT.

## BT y comando

Humble PipelineSequence permite continuar FollowPath cuando el planner vuelve
RUNNING. `GuardedPipeline` conserva ese comportamiento sólo mientras no haya
stop. Una respuesta STOP o un timeout del servicio fija un latch local y cancela
síncronamente el follower anterior antes de seguir tickeando planificación.

`GuardedRecovery` mantiene tres retries ordinarios. Durante la espera de maniobra
permite replans sin renovar el plazo; la respuesta terminal finaliza el árbol.
No se utilizan Spin, BackUp, nuevas velocidades ni otra cadena de control.
`nav_command_server` aplica freno automático 100% ante el stop de maniobra,
incluso si ya no llegan Twists, mediante su salida canónica. Manual conserva
precedencia. Collision Monitor y E-stop no se modifican.

## Contrato y salida del bloqueo

EvaluatePathHealth añade `RESET=2` y `goals: PoseStamped[]`. PathHealth añade
`wait_remaining_s`. Es un cambio de tipo ROS: reconstruir productores y
consumidores juntos, sin compatibilidad binaria con instalaciones anteriores.
El servicio sigue siendo reliable, volátil, productor path_health y consumidores
plugin BT/nav_command_server. El tópico sigue reliable/volátil/depth 10, con
productor path_health, consumidores arbiter, route executor y observadores.

RESET se solicita antes del dispatch de un pedido Nav2 nuevo, después de cancelar
el anterior. El pedido falla cerrado si no se confirma RESET en 1 s. RESET deja
STOP hasta evaluar un path; nunca forma parte de un replan automático.

El executor proyecta WAITING_REPLAN a WAITING_DATA y el vencimiento a
NEEDS_OPERATOR, con causa PLANNER_U_TURN o PLANNER_LOOP y texto específico.
No ejecuta retry, reanchor, créditos geométricos ni clear para escapar de esta
espera. Cancela Nav2 al vencer. Un buen path tardío no reanuda la misión.
La salida operativa es CANCEL explícito y START de un pedido revisado, como
intervención del operador existente. Checkpoints originales y contadores no
cambian por detectar o rechazar el path. Una visita física real sigue siendo
responsabilidad exclusiva del tracker existente.

## Límites

No prueba distancia física de parada ni cuantifica errores angulares. La
resolución temporal del BT y DDS existe; no se declara seguridad funcional ni
paridad física. No se cambia EKF, compensación de antena, selección de anchors,
radio Ackermann, horizonte o tolerancias de checkpoint. Los casos de intención
sin poses suficientes se tratan conservadoramente; deben corregirse por el
operador, sin aceptar automáticamente el retorno.
