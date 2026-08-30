# salus_evaluation

Escenarios, métricas y gates reproducibles de navegación, con un runner ROS
observador que no posee autoridad de comando ni publica TF, según ADR 0007.

Los escenarios instalados usan metros, radianes y segundos. En simulación se
compara la estimación contra `/odom_raw`; esto no valida comportamiento real.

Con una simulación `sim_operational.launch.py` ya levantada:

```bash
./tools/nav_eval.sh run src/salus_evaluation/config/scenarios/right_quarter.yaml
./tools/nav_eval.sh observe
```

`observe` espera el próximo `2D Goal Pose` de RViz e infiere del plan publicado
si la maniobra inicial pide izquierda, derecha o recto. Si no puede inferirlo,
el gate `turn_sign` falla en lugar de omitir esa comprobación. Ambos modos
generan el mismo bundle versionado en `artifacts/evaluations/`.

El bundle v2 conserva `commands.csv` como la solicitud Nav2 en `/cmd_vel` y
agrega las etapas `/cmd_vel_safe`, `/cmd_vel_final`, `VehicleCommand` y
`DriveTelemetry`, junto con los diagnósticos observados de control. Las
correlaciones usan sólo la última muestra causal previa dentro de 0,2 s y
registran su `alignment_gap_s`; la ausencia de una pareja válida se informa sin
alterar los gates funcionales. La dirección medida se persiste en radianes.

La comparación de localización sólo acepta muestras de verdad terreno a menos
de 0.2 s de cada estimación. Los datos no finitos en meta, poses, velocidades,
comandos o plan invalidan la ejecución.

La llegada tiene dos referencias deliberadamente separadas:

- `goal_tolerance_m=1.2`: gate funcional alineado con Nav2 hoy;
- `precision_target_m=0.25`: objetivo futuro, reportado como `calibrating` sin
  fallar CI.

El error final siempre queda registrado, por lo que estos valores pueden
endurecerse después usando distribuciones reales y no una impresión visual.

## Matriz Ackermann velocidad × curvatura

`config/matrices/ackermann_speed_curvature.yaml` define la primera matriz
obstacle-free: velocidades 0,8/1,2/1,6 m/s, recto, llegada corta y arcos de
radio solicitado 8 m y 4 m en ambos sentidos. El radio es la geometría que
solicita el caso; el plan y el ángulo de dirección aplicado siguen siendo
evidencia observada separada en cada bundle. Los valores no modifican Nav2,
los límites Ackermann ni la seguridad.

La ejecución inicia una simulación limpia por trial para evitar que la pose
final, costmaps, plan o estado de Nav2 del trial anterior lo contaminen. Antes
de cada meta aplica y lee `FollowPath.desired_linear_vel` en
`/controller_server`; el resultado y el readback efectivo quedan en los dos
JSON del trial. Si Humble rechaza el cambio runtime, el trial se preserva como
fallido: no se sustituye por un valor supuesto.

Ejecutar la matriz completa y producir el resumen report-only:

```bash
./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/ackermann_speed_curvature.yaml
```

También puede agregarse una colección ya capturada de bundles, en el orden
determinista de la matriz:

```bash
./tools/nav_eval.sh matrix-summary \
  src/salus_evaluation/config/matrices/ackermann_speed_curvature.yaml \
  artifacts/evaluations/matrix-baseline \
  artifacts/evaluations/<trial-01> ... artifacts/evaluations/<trial-54>
```

Produce `matrix-manifest.json`, `matrix-summary.json`, CSV y HTML. Los
agregados continuos incluyen mínimo, mediana, máximo y P95 cuando hay al menos
dos muestras. Los performance gates se mantienen explícitamente en
`calibrating/report-only`; los gates funcionales de cada bundle no cambian.
Al finalizar se conservan todas las celdas y el proceso devuelve non-zero si
alguna tuvo setup failure o un gate funcional existente falló. Una métrica
calibrating no cambia ese exit status. El error final de yaw no se aproxima en
esta matriz: queda explícitamente para #63, que definirá su semántica.
