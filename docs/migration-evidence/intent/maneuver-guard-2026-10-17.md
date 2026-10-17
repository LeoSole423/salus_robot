# Intención: admisión de paths sin retornos introducidos por el planner

## Alcance y evidencia

PC/simulación exclusivamente. Base origin/main c6bed23; el checkout canónico
6775e24 y su .agents/ se preservan. No se modifica localización ni firmware.

Hechos: ADR 0002 mantiene paths sanos y valida candidatos, pero el árbitro sólo
frena STOP_AND_WAIT; PipelineSequence Humble permite seguir al controller durante
RUNNING del planner. La observación PLAN_U_TURN era sólo diagnóstica. #244 ya
incorpora multi-pose y recovery; #315 corrige selección direccional del anchor;
#282 preserva el horizonte de checkpoints. ADR 0011 corrige heading intermedio.
Estos comportamientos no se sustituyen con un detector. PR 325/327 quedan intactos.

Fuentes: código vigente path_health, route_recovery, nav_command_server, ambos
BT, contratos salus_interfaces, state_projection/ros_gateway de salus_web;
issues 244/315/282 y ADR 0002/0011/0012. Cockpit inspeccionado en rama
agent/camera-sweep-recovery: consume blocked_reason_text y blocked_* existentes;
no se edita ese checkout. No se presume que sea la rama de migración vigente.

## Criterios reproducibles (hipótesis conservadora para PC)

No rechazar por ángulo aislado ni por longitud global / distancia al destino.
Separar el path por visitas en orden a las poses pedidas. En cada pierna:
- proyección longitudinal sobre su cuerda y sobre heading actual en la primera;
- retorno longitudinal superior a una longitud de footprint (1.17 m);
- tangente opuesta más de 135 grados (diagonal del semiplano posterior);
- vuelta acumulada firmada de al menos 270 grados y retorno a un sector
  previo dentro de 1.17 m identifican un rulo dentro de una pierna; curvas de 90/120 grados y S no son vueltas.
Un circuito intencional se divide por sus poses; no recibe una excepción global
que permitiría rulos adicionales. Una ruta sin esas poses no permite inferir
intención de un círculo. Las poses no se mueven ni se acreditan por esta división.
Se prueban ambos sentidos, escala/muestreo, transformaciones rígidas y fronteras.
Umbrales son criterios operacionales PC, no radio ni frenado físico validado.

## Estados y autoridad

KEEP -> WAITING_REPLAN -> KEEP, o NEEDS_OPERATOR al vencimiento monotónico.
La primera inadmisibilidad fija el plazo; cambios de causa, paths, fallos de datos
y candidatos rechazados no lo renuevan. Un candidato admisible antes del plazo
se copia al blackboard; el stop sólo se libera al observarlo como ACTIVE.
Un pipeline BT con latch de stop cancela FollowPath cuando la evaluación pide
STOP, aunque la planificación siga RUNNING. El detector no publica velocidades.
El árbitro existente sigue siendo dueño del stop automático; manual, cancel,
E-stop y Collision Monitor conservan precedencia. No se usa Spin.

El vencimiento se proyecta a NEEDS_OPERATOR y cancela Nav2 sin reanchor, clear,
retry ni créditos geométricos. No se reanuda por recibir después un buen path.
Salida: cancelación explícita y nuevo START/SetRouteMission, igual que el bloqueo
terminal existente. El nuevo pedido reinicia la admisión antes de despachar Nav2.
Una pausa por takeover no reinicia plazos ni autoriza movimiento automático.

## Contratos y pruebas

Se amplía EvaluatePathHealth con poses de intención y RESET explícito; requiere
rebuild simultáneo de salus_interfaces, salus_navigation y salus_navigation_bt.
El tópico PathHealth y los campos blocked_* conservan tipos y autoridad.
Los parámetros y límites se documentan en README y ADR.

Pruebas: política geométrica y temporal pura, adaptador/árbitro/recovery,
BT y proyección web; build, suite completa y smokes proporcionales en contenedor.
Registrar resultados efectivos y limitaciones en evidencia. Sin evidencia física
ni declaración de seguridad de frenado antes de una campaña posterior.

## Resultado de la implementación

Ver `../maneuver-guard-2026-10-17.md` y ADR 0016 para resultados, el rulo R4
capturado y fallos preexistentes. La inspección de la rama remota de migración
de Cockpit en 383f3ce confirmó nombres de estado antiguos; el PR separado
AEyeSecurity/cockpit#26 corrige panel/mapa/polling sin modificar su main ni el
checkout activo. La matriz no se suaviza para aprobar una llegada bloqueada.
