# #328 — campaña PC del 18/10/2026

Fuente de captura inicial: `7fd0014`, base `d6bf67d`. Sólo simulación. Ver
[ficha de intención](../intent/ekf-turns-2026-10-18.md) y reporte de resultados
cuando finalice la campaña. No se modifican defaults ni hardware.

## Reproducción

Desde un worktree propio, con Docker Humble construido:

```bash
./tools/build.sh
./tools/test.sh
./tools/nav_eval.sh isolation artifacts/issue328-isolation-new
/usr/bin/python3 tools/heading_turn_campaign.py \
  docs/migration-evidence/ekf-turns-20261018/pilot.json artifacts/issue328-pilot-new
/usr/bin/python3 tools/heading_turn_campaign.py \
  docs/migration-evidence/ekf-turns-20261018/baseline-matrix.json \
  artifacts/issue328-reference-new --jobs 2
python3 tools/summarize_turn_campaign.py artifacts/issue328-reference-new \
  artifacts/issue328-summary-new --plot
./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/issue328_navigation_reference.yaml \
  artifacts/issue328-nav-new --jobs 2
```

El intérprete host necesita PyYAML (en esta PC `/usr/bin/python3`; el `python3`
del PATH pertenece a PlatformIO y no lo incluye). El resumen con `--plot`
necesita matplotlib, instalado en un venv temporal para esta campaña. El build
usa sus propios build/install. No reutilizar symlinks de otros worktrees.

La cola asigna dominios con locks existentes y limita concurrencia a 1/2.
Cada captura vive en un Docker propio, sin puertos publicados, particiones
Gazebo, runtime y logs independientes. Esta cola usa bridge/UDPv4 para evitar
el discovery intermitente observado con procesos CLI en network=none. No usa
red ni nodos externos; no cambia el harness de matrices Nav2.
Cada artefacto conserva receta, seed, perfil YAML montado sólo dentro del
contenedor, parámetros leídos y provenance de fuentes/imagen. No se sobrescribe
ninguna carpeta existente. Los fallos permanecen en outcomes.json.

`clean` en esta campaña significa perturbaciones cero, salvo GNSS 2 cm del
perfil clean existente, sobre el **transporte independiente** de sensores.
Esto mantiene el mismo camino de rueda/steering al introducir perturbaciones;
no es el bypass de sensores que usa el launch de producto `clean`. Las máscaras,
covarianzas y compensación del EKF sí son las vigentes. La velocidad mínima de
heading GPS sigue siendo 0.8 m/s: su disponibilidad a la frontera debe medirse.

## Artefactos y semántica

`summary.csv/json`, `aggregates.json`, comparación PNG/SVG y carpetas de trial.
Los errores primarios usan header stamp, yaw envuelto, interpolación truth con
gap máximo 0.2 s, sin extrapolación ni fit de offset. Las fases son ventanas de
comando; `actual_strong_turn` informa por separado el giro físico >=0.18 rad/s.
El lag es diagnóstico con soporte común; nunca corrige las métricas primarias.
La recuperación <=2° y <=0.25 m durante 1 s es una referencia descriptiva,
no un criterio de seguridad. Si no ocurre se conserva como censurada.
Las continuidades incluyen bordes de ventana y timestamps únicos: un stream
congelado no mejora artificialmente la precisión.

El modelo cinemático solicitado sirve para separar tracking de localización;
no representa un path aceptado por Nav2. Los bundles Nav2 tienen sus propias
metas, errores de seguimiento, replans y gates funcionales. Un rechazo del guard
no se convierte en una mejora del EKF.

Los datos compactos `analysis-samples.jsonl.gz` conservan todas las muestras,
header/receive stamps, poses, rates, comandos, GPS y debug; sólo sustituyen
matrices completas de covarianza por sus seis diagonales. Son entrada directa
de `heading_turn_metrics.py` para reproducir los números. `raw-sha256.json`
identifica crudos completos locales, que conservan también covarianzas cruzadas.
No se calcula innovación interna a partir de un error externo: habilitar debug
EKF sólo si hace falta atribuir un fallo no explicado por datos observables.

Ground truth `/odom_raw` lleva nombres internos `salus_ackermann/odom` y
`salus_ackermann/base_footprint`. Son coordenadas de origen del modelo, no TF
públicos. `motion_sim.launch.py` fija spawn x/y/yaw=0; `free.world` y mapper
comparten datum/yaw=0. La comparación fija map=odom del mundo, sin alinear al
EKF ni restar un error inicial. Otros spawns/datums requieren otra transformación
explícita; esta campaña no es un analizador genérico de bags reales.
