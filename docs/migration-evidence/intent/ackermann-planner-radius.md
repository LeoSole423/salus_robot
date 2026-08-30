# Intención: coherencia de radio Smac y dirección Ackermann

## Alcance

- Fuente: issue #57, control Ackermann y configuración Nav2 de simulación.
- Incluido: modelo cinemático, experimento reproducible y selección fundada de
  un radio nominal del planner.
- Fuera de alcance: tuning de RPP, perfiles #65, tolerancia/final yaw #63 y
  validación de hardware.

## Hechos y geometría

Con batalla `L = 0,94 m`, `R = L / tan(delta)`:

| Frontera | Steering | Radio equivalente |
| --- | ---: | ---: |
| físico/modelo | 30° | 1,63 m |
| autónomo provisional | 25° | 2,02 m |
| Smac actual | 13,22° | 4,00 m |
| candidato conservador a evaluar | 20,61° | 2,50 m |
| candidato intermedio a evaluar | 21,38° | 2,40 m |
| candidato cercano a la frontera | 22,22° | 2,30 m |

25° es un límite seguro provisional validado inicialmente en simulación;
`hardware_validated: false`. El límite físico de 30° se conserva para el
modelo y la protección mecánica. Manual mantiene su límite operativo actual.

## Autoridades separadas

El control calcula curvatura desde el request y limita steering aplicado a
`min(steering_limit_rad, operational_steering_limit_rad)` para auto. Smac usa
`minimum_turning_radius` como restricción geométrica de sus primitivas, por lo
que su radio nominal debe ser mayor al radio autónomo mínimo para retener
corrección de tracking. RPP `regulated_linear_scaling_min_radius` sólo activa
regulación de velocidad para curvas más cerradas; no es un límite de
factibilidad y se analiza separadamente.

Referencias: [Smac Hybrid minimum turning radius](https://docs.nav2.org/configuration/packages/smac/configuring-smac-hybrid.html)
y [RPP curvature regulation](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html).

## Experimento

1. baseline 4,0 m, luego sweep grueso 2,5 / 2,4 / 2,3 m con pocos trials;
2. comparar 1,2 y 1,6 m/s como prioridad, y 0,8 m/s sólo como geometría y
   transitorios en simulación, no como evidencia de tracción desde reposo;
3. usar bundles #66: éxito, tracking, replans, requested/applied steering y
   saturación; añadir steering margin si hace falta;
4. sólo pasar a sweep fino del candidato con margen suficiente, sin saturación
   sistemática ni asimetría izquierda/derecha.

El baseline y el sweep se clasifican como infraestructura/runtime si reaparece
la firma #119/#117/#72. No se aumenta timeout ni se retocan TF, costmaps o
RPP para ocultarlo.

## Criterio de decisión pendiente

No se seleccionará ni aplicará el candidato hasta obtener un baseline y sweep
de simulación válidos. #57 no asume que radio de Smac y radio de regulación de
RPP deban ser iguales.

## Ejecución local inicial

Baseline `4,0 m`: `./tools/nav_eval.sh matrix
src/salus_evaluation/config/matrices/ackermann_speed_curvature_smoke.yaml
artifacts/evaluations/issue-57-baseline-20260830`. Las tres celdas a 1,2 m/s
(recta, izquierda suave y derecha suave) llegaron con éxito. La recta no tuvo
saturación; izquierda registró un intervalo observado de 0,102 s y derecha
ninguno. Es una baseline inicial, no una caracterización suficiente para
seleccionar radio.

El primer candidato `2,5 m` usó el mismo ejecutor con
`--planner-minimum-turning-radius 2.5`; no alcanzó el trial por timeout de
servicio de parámetros durante startup y no generó un bundle terminal. Esa
evidencia no se clasifica todavía: el harness original intentaba mutar el
planner ya arrancado mediante sus servicios de parámetros.

Los siguientes experimentos no mutan `minimum_turning_radius` en runtime. Para
cada trial se genera de forma reproducible un YAML desde
`nav2_core_no_obstacles_sim.yaml`, modificando sólo
`planner_server.ros__parameters.GridBased.minimum_turning_radius`, y se lo
entrega al launch no-obstacle desde el arranque. El artifact conserva ambos
paths y sus hashes SHA-256, el radio solicitado y el readback efectivo del
planner (`requested_radius_m`, `effective_radius_m`, unidad `m`). La readiness
del candidato exige odometría, controller/planner ACTIVE y que el servicio
`get_parameters` responda; no necesita `set_parameters`. No se ajustaron
timeouts, retries, TF, costmaps ni RPP.

Reintento posterior a la corrección:
`artifacts/evaluations/issue-57-radius-2p5-ready-20260830`. El launch alcanzó
`/planner_server` ACTIVE, pero registró timeouts de respuesta de
`/planner_server/get_parameters` y `/planner_server/set_parameters`; no llegó
a publicar una meta ni un bundle terminal. Esa ejecución queda como evidencia
del problema sistémico #119/#117/#72, pero no es una medición cinemática del
radio: fue reemplazada por la configuración de arranque descripta arriba.

## Reintento con configuración de arranque

El baseline se repitió con `4,0 m` mediante el nuevo mecanismo en
`artifacts/evaluations/issue-57-startup-radius-4p0-20260830`. El primer trial
conserva el YAML candidato y hashes de base/efectivo, pero agotó los 90 s de
readiness antes de navegación: no había odometría global, controller ni planner
ACTIVE, ni respuesta/disponibilidad de `planner_server/get_parameters`. Por
tanto no existe aún readback efectivo ni una medición de trayectoria que pueda
atribuirse a `minimum_turning_radius`. La ejecución queda clasificada como la
misma frontera de readiness #119/#117/#72; no se inició `2,5 m`, porque no
habría aislado una variable distinta. Se conservan los mismos thresholds y no
se aplicaron sleeps, retries, tuning Nav2, TF ni costmaps.
