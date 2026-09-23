# #302 — evidencia de recovery en PC/sim

La prueba `tools/smoke_route_recovery_sim.sh` usa el bringup sim y espera dos
hechos antes de inyectar un STOP acotado: `ROUTE_CHECKPOINT_REACHED` para el
prefijo pedido y odometría que ya pasó físicamente el último checkpoint
acreditado. Después comprueba el request de retry, un plan nuevo y al menos
1 m adicional de movimiento. Calcula el mayor retroceso longitudinal desde
el máximo progreso anterior, tanto en `/plan` como en odometría. El umbral de
0,5 m es una aserción del smoke; no modifica Nav2 ni las tolerancias de misión.

| Caso | Original → retry (`input_indices`) | Máximo retroceso `/plan` | Máximo retroceso odometría | Artifact local |
| --- | --- | ---: | ---: | --- |
| Abierta | `[0,1,2] → [1,2]` | 0 m | 0,0014 m | `route-recovery-partial-20260923T200204-1` |
| Cierre de loop, crédito 5 | `[5,0,1,2] → [0,1,2]` | 0 m | 0,0025 m | `route-recovery-partial-20260923T200310-1` |
| Cierre de loop, créditos 5 y 0 | `[5,0,1,2] → [1,2]` | 0 m | 0,0065 m | `route-recovery-partial-20260923T200406-1` |
| Sintético posterior al crédito | `[0,0,1] → [0,1]` | 0 m | 0,0015 m | `route-recovery-partial-20260923T200908-1` |
| Acción y frontera hard previas | `[1,2] → [2]` | 0 m | 0,0001 m | `route-recovery-partial-20260923T201025-1` |

En el caso de loop, las ocurrencias originales fueron `(0,5), (1,0),
(1,1), (1,2)`; el retry conservó sólo las de la vuelta 1. En el caso
sintético, el `0` restante del retry es una muestra `key=false`, no un
checkpoint repetido. La acción hard de índice 0 comenzó y terminó una sola
vez antes del fallo en el chunk `[1,2]`.

La variante `SMOKE_RECOVERY_CANCEL_IN_WAIT=1` canceló durante
`ROUTE_BLOCKED_WAITING`: estado final `CANCELLED` y seis comandos safe-zero
posteriores a la solicitud de cancel en el artifact
`route-recovery-partial-20260923T201100-1`.

El smoke existente de Patrol/HOME pasó con fases `JOIN_LOOP → PATROL →
EXIT_LOOP → RETURN_HOME → AT_HOME` y conserva sus eventos; el artifact es
`patrol-battery-return-20260923T192948-1`. No se inyectó un fallo de recovery
durante Patrol/HOME.

Un primer fixture sintético largo (17 poses) y otro de 7 poses abortaron en
Nav2 antes de acreditar el primer checkpoint; no entraron en el recovery de
#302. El fixture final de tres poses (`[0, sintético, 1]`) sí alcanzó el
checkpoint y permitió medir el retry. Los fallos iniciales no se utilizaron
como evidencia de éxito y no se cambiaron tolerancias ni el disparador del
fallo de #303.

Estos artifacts están en `artifacts/smokes/` del worktree local y no forman
parte del repositorio. Cada `recovery_probe.json` contiene eventos, despachos,
planes, odometría y comandos finales para volver a calcular las métricas. La
evidencia corresponde a las rutas sim ensayadas. No demuestra que Nav2 nunca
elegirá un giro en U con otra geometría de calles ni establece paridad de
hardware. El smoke opt-in anterior de loop amplio agotó 120 s antes de la
segunda vuelta, sin entrar en recovery (`routes-free-world-20260923T193215-1`);
se conserva como límite separado, sin alterar su timeout.
