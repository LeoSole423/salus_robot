# salus_evaluation

Dominio puro para escenarios, métricas y gates reproducibles de navegación.
No posee autoridad de comando ni TF. La integración ROS, RViz y el runner se
incorporarán sobre estos contratos según ADR 0007.

Los escenarios instalados usan metros, radianes y segundos. En simulación se
compara la estimación contra `/odom_raw`; esto no valida comportamiento real.

Con una simulación `sim_operational.launch.py` ya levantada:

```bash
./tools/nav_eval.sh run src/salus_evaluation/config/scenarios/right_quarter.yaml
./tools/nav_eval.sh observe
```

`observe` espera el próximo `2D Goal Pose` de RViz. Ambos modos generan el
mismo bundle versionado en `artifacts/evaluations/`.

La llegada tiene dos referencias deliberadamente separadas:

- `goal_tolerance_m=1.2`: gate funcional alineado con Nav2 hoy;
- `precision_target_m=0.25`: objetivo futuro, reportado como `calibrating` sin
  fallar CI.

El error final siempre queda registrado, por lo que estos valores pueden
endurecerse después usando distribuciones reales y no una impresión visual.
