# Intención: detectar planes que retroceden sobre un chunk

## Alcance

Se agrega diagnóstico en `salus_navigation/nav_observer` y una política pura de
geometría. También se amplía el horizonte de observación en simulación, sin
cambiar Cockpit ni la aceptación de planes.

## Evidencia observada

En Gazebo el 2026-09-24, con el usuario colocando obstáculos, el chunk activo
iba de `(77.64, 84.68)` a `(127.51, 81.22)` en `map`. Tras
`far_obstacle_persistent`, `/plan` tuvo 221 poses y 85.98 m desde el robot
`(87.30, 84.06)`, con mínimo progreso de -14.6 m respecto a ese robot.
Tras `clearance_degraded`, otro `/plan` tuvo 216 poses y 83.77 m desde
`(91.13, 81.81)`, con mínimo progreso de -18.6 m. Son planes emitidos por
Nav2, no una orden comprobada de volver a un waypoint previo. La captura
read-only está en `/tmp/salus-uturn-live.jsonl` del host de desarrollo; ese
archivo temporal no forma parte del repositorio.

## Política y contrato

Proyectar las poses del plan sobre el sentido de un chunk casi recto (longitud
del chunk ≤ 1.15 veces su cuerda). Marcar un plan si su punto más retrasado
queda ≥ 6 m detrás del robot y la longitud del plan supera 1.5 veces el avance
restante proyectado. Los umbrales son conservadores para observabilidad,
deducidos de los dos planes observados; no son gates de seguridad ni rechazo.
Chunk curvo, ausente, frame desigual u odometría vencida: resultado inconcluso.
Los eventos `PLAN_U_TURN` y `PLAN_U_TURN_CLEARED` reutilizan `NavEvent` y
`/nav_command_server/events`, sin cambiar el contrato público. Se emiten en
las transiciones observado/limpio para no inundar la UI con cada replan.

## Próximo paso separado

Antes de vetar planes, comparar alternativas Nav2 en simulación con el mismo
costmap y medir si existe un desvío hacia delante. Un veto incondicional puede
bloquear un rodeo necesario. Separar además la causa de nuevas replans cuando
el obstáculo sale del campo de visión, mediante scans y costmap sincronizados.
La captura actual no valida sensores ni comportamiento físico.

## Anticipación de replan en simulación

La configuración previa sólo marcaba obstáculos globales a menos de 15 m y
`path_health` examinaba 12 m. Con costmap a 1 Hz y confirmación de dos mapas,
un robot a 1,6 m/s podía iniciar el replan después de avanzar ~1,6 m. El
radio mínimo de 4 m de Smac deja entonces poco margen para un rodeo lateral.
El scan simulado admite 20 m. Se amplía sólo el marking global a 19 m y el
horizonte de `path_health` a 18 m; la confirmación de dos costmaps, el límite
cercano 5,35 m y el stop físico no cambian. La captura demuestra la regresión
de planes, pero no prueba que ampliar el horizonte baste para eliminarla;
requiere comparación A/B en Gazebo con obstáculo de pose fija.

## Reproducción controlada inicial

Se guardó la pose Gazebo del obstáculo frontal `box_7` en
`(94.77, 82.92, 0.5)` y se recreó como caja estática de 1 m en un world
temporal. El robot salió de `(77.64, 84.68)` hacia `(127.5, 81.2)` con el
perfil nuevo. El plan inicial midió 49.89 m. `far_obstacle_persistent` apareció
en `(81.84, 84.67)`, antes de llegar a la caja; el replan midió 46.25 m y
retrocedió sólo 0.03 m en X respecto a la pose del robot. El goal terminó con
estado Nav2 4 (`SUCCEEDED`). El observador no tenía chunk de misión en este
ensayo de goal directo, por lo que no se espera `PLAN_U_TURN`.

El control no recrea las otras cajas ni el estado exacto del costmap de la
sesión del operador. Demuestra que el perfil ampliado inicia un rodeo hacia
delante en este caso, pero no prueba todavía que resuelva todos los ciclos.
