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
