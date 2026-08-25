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
