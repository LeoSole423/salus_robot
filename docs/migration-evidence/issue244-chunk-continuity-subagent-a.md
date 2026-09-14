# #244 — continuidad entre chunks en curva amplia

Informe autocontenido del subagente A. Este experimento fue PC/simulación
únicamente; no accedió a Jetson, hardware, deploy ni comandos físicos.

## Identidad

- Base SHA: `f918d4bcfac33f6f8d20d5af02d175f6a1b9a82d`
- Rama: `agent/issue-244-chunk-continuity-subagent-a`
- Worktree: `/home/leosole/Desktop/SALUS-worktrees/salus_robot-issue-244-chunk-continuity-subagent-a`
- Matriz: `src/salus_evaluation/config/matrices/issue244_chunk_continuity.yaml`
- Modo: `salus_evaluation/nav_eval`, simulación fresca por trial

## Diseño constante

Una curva izquierda amplia de 90 grados, con dos piernas de 8 m, un límite
entre el primer y segundo chunk, y un look-ahead sintético de 2 m sólo para la
variante `lookahead`. Todos los trials usaron `DUBIN`,
`minimum_turning_radius=4.0 m`, `free.world`/costmaps sin obstáculos, steering
del perfil existente y velocidad constante nominal de 0.8 m/s. No se cambió
ningún YAML o parámetro productivo; el runner envió sólo requests de evaluación.

Políticas comparadas: `terminal_incoming` (CURRENT), `legacy_outgoing`,
`shared_tangent` y `lookahead`. Repeticiones: 2 por política, 8 celdas en
total. La matriz ejecutó con `--jobs 2` después de pasar aislamiento.

## Comandos exactos y resultado

```text
./tools/nav_eval.sh isolation /home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-isolation
# FAIL antes de ROS: faltaban /ros2_ws/install/setup.bash y salus_evaluation.

./tools/build.sh
./tools/nav_eval.sh isolation /home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-isolation-built
# PASS: dos dominios, clock monótono, planner/controller activos, odometría finita,
# worker A muerto y worker B sobrevivió.

./tools/build.sh
docker compose run --rm ros2 bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && colcon test --packages-select salus_evaluation --event-handlers console_direct+ && colcon test-result --verbose'
# PASS: 86 tests, 0 failures, 2 warnings.

./tools/nav_eval.sh matrix src/salus_evaluation/config/matrices/issue244_chunk_continuity.yaml /home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-isolation-serial
# STOPPED con setup de velocidad en la segunda celda; se conserva como evidencia
# separada y no se usó para la conclusión.

./tools/nav_eval.sh matrix src/salus_evaluation/config/matrices/issue244_chunk_continuity.yaml /home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-final --jobs 2
# exit 1: la matriz terminó y escribió todos los bundles; el exit no-zero refleja
# gates funcionales y un setup failure, no pérdida de artifacts.
```

## Dominios y partitions

La corrida final asignó dominios 66–73, uno por trial, y una partition única por
trial con formato
`salus-nav-salus-nav-matrix-20260914T171834-246272-<trial_id>`.
Los identificadores completos están en cada `manifest.json` y en
`summary/matrix-manifest.json`; `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` fue constante.

## Resultados agregados

Valores de mediana de los trials con métricas disponibles. `valid` cuenta
bundles que llegaron a producir las dos acciones/planes; `gate` es el resultado
funcional existente de ese bundle. `direct` está en metros, `length` en metros,
`detour` es adimensional, desviación en metros, curvatura en 1/m, y los ángulos
en radianes.

| Política | valid / reps | gate | length | direct | detour | max deviation | self intersections | boundary jump | boundary curvature | saturation intervals |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| CURRENT / terminal_incoming | 2/2 | 1/2 | 40.527 | 11.314 | 3.582 | 10.192 | 0 | 0.0013 | 3.268 | 7.0 |
| LEGACY_OUTGOING | 2/2 | 0/2 | 67.453 | 11.314 | 5.962 | 9.010 | 2 | 1.471 | 2.708 | 4.5 |
| SHARED_TANGENT | 2/2 | 2/2 | 65.907 | 11.404 | 5.780 | 10.199 | 9 | 0.857 | 3.268 | 6.5 |
| LOOKAHEAD | 1/2 | 1/1 | 43.668 | 11.314 | 3.860 | 8.696 | 1 | 0.295 | 0.248 | 4.0 |

Todos los trials válidos reportaron `first_chunk_status=4` y
`second_chunk_status=4`; los gates funcionales que fallaron fueron llegada
fuera de tolerancia en algunas repeticiones. `explicit_yaws_changed=false` en
todos los bundles válidos.

## Hipótesis

- La hipótesis de que `terminal_incoming` produce por sí sola una discontinuidad
  grande entre chunks no quedó confirmada en esta geometría: el salto medido fue
  ~0.0013 rad y tuvo 0 auto-intersecciones, aunque mantuvo curvatura local alta
  y un gate funcional fallido.
- Volver a `legacy_outgoing` no mejoró la continuidad: aumentó el salto (~1.47
  rad), el detour (~5.96) y las auto-intersecciones (2).
- `shared_tangent` tampoco fue ganador: aunque pasó sus dos gates funcionales,
  produjo 9 auto-intersecciones y detour ~5.78.
- `lookahead` redujo la curvatura local y fue mejor que outgoing/shared en
  longitud, pero sólo tiene una repetición válida y todavía conserva una
  auto-intersección; no supera de forma concluyente a CURRENT.

Conclusión: no hay ganador claro que preserve simultáneamente geometría,
continuidad, gates y cobertura de repeticiones. No se abrió Draft PR ni se
preparó cambio productivo.

## Artifacts

Resumen final:

- `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-final/summary/matrix-manifest.json`
- `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-final/summary/matrix-summary.json`
- `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-final/summary/matrix-summary.csv`
- `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-final/summary/matrix-report.html`

Trials finales:

- `.../trials/terminal_incoming-wide_turn_boundary-left-r4-v0p8-rep01/`
- `.../trials/terminal_incoming-wide_turn_boundary-left-r4-v0p8-rep02/`
- `.../trials/legacy_outgoing-wide_turn_boundary-left-r4-v0p8-rep01/`
- `.../trials/legacy_outgoing-wide_turn_boundary-left-r4-v0p8-rep02/`
- `.../trials/shared_tangent-wide_turn_boundary-left-r4-v0p8-rep01/`
- `.../trials/shared_tangent-wide_turn_boundary-left-r4-v0p8-rep02/`
- `.../trials/lookahead-wide_turn_boundary-left-r4-v0p8-rep01/`
- `.../trials/lookahead-wide_turn_boundary-left-r4-v0p8-rep02/`

Los tres últimos sufijos son rutas relativas a
`/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-final`.
`lookahead-rep02` conserva `summary.json`/`manifest.json` como `setup_failure`
por timeout de acción; no se mezcló con las métricas válidas.

Artifacts de aislamiento:

- Fallo de infraestructura inicial:
  `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-isolation/`
- Aislamiento PASS:
  `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-isolation-built/isolation-report.json`
- Matriz serial con fallo de setup preservado:
  `/home/leosole/Desktop/SALUS-worktrees/artifacts-issue244-subagent-a-isolation-serial/`

SHA-256 del resumen final: `matrix-manifest.json`
`6847cc5b51c9c3abdfd5d85136830a9a88fa605d42f86e18613b9b47008ae000`,
`matrix-summary.json`
`7c6d78131a7e2b8cd81dc7964dab14488090807fcb233c0308d6dc962bae2c28`.

## Limitaciones

El runner experimental ejercita dos `NavigateThroughPoses` secuenciales para
aislar la continuidad y observa `/plan`, odometría y status del controlador;
no sustituye todavía al `route_executor` real ni genera eventos de checkpoint,
acciones o frenos de misión. Por eso no permite afirmar paridad de checkpoint
en #244. Los fallos de llegada, los timeouts y la variabilidad de simulación
impiden declarar un ganador; la desviación se calcula contra la polilínea de
referencia y puede ser grande cuando Smac elige un arco Dubins largo. Los
hallazgos #272/#276/#277 no se reprodujeron ni se modificaron: esta campaña no
es evidencia sobre heading global, outliers de wheel odometry o timestamps de
Collision Monitor.

No hubo URL/SHA de PR: **sin PR**.
