# Intención: matriz de regresión Ackermann velocidad × curvatura

## Alcance

- Fuente: issue #66 y ADR 0007.
- Destino: `salus_evaluation` y sus artefactos reproducibles.
- Incluido: expansión determinista, casos sin obstáculos, agregación de
  bundles individuales y conservación de fallos.
- Fuera de alcance: tuning de RPP/Nav2, límites de dirección, radio mínimo,
  tolerancias de meta y validación de hardware.

## Evidencia

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| `docs/decisions/0007-navigation-evaluation.md` | performance comienza calibrating; gates funcionales son distintos | alta |
| `salus_evaluation/evaluation_runner.py` | el bundle v2 ya observa la cadena post-safety y Ackermann | alta |
| issue #66 | se necesita evidencia por velocidad, geometría y repetición | alta |

## Diseño y contratos

La matriz v1 usa 0,8/1,2/1,6 m/s y repite cada celda tres veces. Las curvas
usan radios solicitados de 8 m y 4 m, ambos sentidos; el de 4 m coincide con
el radio mínimo configurado, por lo que no se introducen maniobras imposibles.
Recto y llegada corta se mantienen como geometrías separadas. El radio
solicitado no se presenta como curvatura del plan, `angular.z`, ángulo
Ackermann ni steering aplicado: éstos siguen en los streams del trial.

Cada trial conserva el bundle v2 existente. La matriz lo referencia por
`trial_id`, agrega success/failure, métricas de tracking/llegada/replans y la
evidencia de saturación/diferencia de steering ya observada. Las métricas de
performance son sólo reportes `calibrating`. El yaw final no se aproxima ni se
deduce de `heading_p95_rad`; su métrica y semántica pertenecen a #63.

## Fallos y aceptación

Un bundle faltante, una cantidad incorrecta de trials o un valor de matriz
inválido falla antes de emitir un resumen ambiguo. Un trial Nav2 fallido queda
en la tasa de éxito y sus medidas disponibles; no se descarta. Cada ejecución
real debe iniciar una simulación limpia y registrar el resultado de la
mutación/lectura de `FollowPath.desired_linear_vel`; un rechazo runtime es
evidencia, no un motivo para alterar parámetros estáticos.

La matriz completa todas las celdas. Al final devuelve non-zero por setup
failure o por el non-zero de `navigation_evaluation` (gates funcionales ya
existentes); performance `calibrating` no modifica el exit code. El readback
de velocidad se registra como requested/effective numéricos y se verifica con
una tolerancia explícita de 1e-6 m/s; un readback ausente, ambiguo o distinto
preserva un setup failure.

Estado: `ported` para el dominio y pendiente de baseline de simulación; no
validado en hardware.

## Ejecución local

El 2026-08-30 se ejecutó `./tools/nav_eval.sh matrix
src/salus_evaluation/config/matrices/ackermann_speed_curvature_smoke.yaml
artifacts/evaluations/issue-66-smoke-20260830`. El primer trial se preservó en
ese directorio como `matrix_setup_failure`: la readiness de navegación agotó
90 s mientras el log del launch mostraba la activación de Nav2 al límite. No
se aumentó el timeout ni se maquilló como éxito. Había otra composición ROS en
el host, por lo que no constituye baseline. La repetición aislada en
`artifacts/evaluations/issue-66-smoke-isolated-20260830-2` reprodujo el mismo
agotamiento con `ROS_DOMAIN_ID=153` y una `GZ_PARTITION` propia. Este síntoma
debe correlacionarse con #119, #117 y #72; la matrix no aumenta su presupuesto
ni modifica Nav2 para ocultarlo.
