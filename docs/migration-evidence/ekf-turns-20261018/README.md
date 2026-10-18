# #328 — navegación y EKF compensado, PC 18/10/2026

Leer [resultados](RESULTS.md), [intención](../intent/ekf-turns-2026-10-18.md) y
[ADR0017](../../decisions/0017-simulation-motion-reference.md). Hardware no validado.

## Dos referencias que no deben mezclarse

`kinematic-precheck/` conserva la exploración inicial previa a corregir el
simulador. Comparaba contra `/odom_raw`: odometría integrada de articulaciones,
correlacionada con rueda/IMU, **no pose física independiente**. Sus errores y
fallos quedan conservados, pero no sirven para seleccionar tuning de precisión
física. El inventario de detención distingue completados, parciales y no ejecutados.
Los históricos del worktree heading-strong-turns permanecen intactos, identificados
en preserved-history-sha256.json; tampoco se reinterpretan como truth físico.

La referencia física parte de `0de5f53`, base main `d6bf67d`: contrato Twist
correcto, claves Fortress wheel_base/kingpin_width y observador worldPose. El
EKF/compensación de producto se conserva. El nuevo observador nunca alimenta
sensores, control, compensación, EKF ni `/tf`. El tracking cambia por corregir
el simulador; no es una mejora del filtro. IMU sigue derivando de odometría
cinemática: los errores comunes siguen siendo una limitación del sensor simulado.

## Reproducción

Desde un worktree propio, con imagen Docker Humble disponible:

```bash
./tools/build.sh
./tools/test.sh
./tools/nav_eval.sh isolation artifacts/issue328-isolation-new
/usr/bin/python3 tools/heading_turn_campaign.py \
  docs/migration-evidence/ekf-turns-20261018/physical-pilot.json \
  artifacts/issue328-physical-pilot-new --jobs 2
/usr/bin/python3 tools/heading_turn_campaign.py \
  docs/migration-evidence/ekf-turns-20261018/physical-reference-matrix.json \
  artifacts/issue328-physical-reference-new --jobs 2
python3 tools/summarize_turn_campaign.py artifacts/issue328-physical-reference-new \
  artifacts/issue328-physical-summary-new --plot
./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/issue328_navigation_reference.yaml \
  artifacts/issue328-nav-r4-new --jobs 2 \
  --evaluation-ground-truth-topic /simulation/ground_truth
./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/issue328_navigation_r3.yaml \
  artifacts/issue328-nav-r3-new --jobs 2 --planner-minimum-turning-radius 3 \
  --evaluation-ground-truth-topic /simulation/ground_truth
```

El intérprete host de la cola necesita PyYAML (en esta PC `/usr/bin/python3`;
`python3` del PATH pertenece a PlatformIO). `--plot` requiere matplotlib;
se instaló en un venv temporal, sin modificar el entorno del proyecto.
Build/install propios; no reutilizar symlinks de otros worktrees. Las carpetas
nuevas son obligatorias: no se sobrescribe ninguna captura.

La cola usa asignador/locks existentes, 1/2 workers, seis CPU cada uno. Docker
bridge sin puertos publicados, dominios/particiones/runtime/logs independientes,
UDPv4. El runner histórico conserva network=none por defecto. La captura lee
parámetros efectivos mediante servicios Humble, incluyendo arrays tipados no
inicializados; un fallo conserva status, datos y provenance.

`clean` significa perturbaciones cero salvo GNSS2 cm existente, sobre el mismo
transporte independiente de sensores utilizado por las intervenciones. No es el
bypass que el launch de producto llama clean. Perfil YAML, seed, receta y
parámetros efectivos están junto a cada trial. `recipe.json` identifica escenario,
velocidad, radio y override manual; el campo scenario legado del probe no debe
reemplazar esa receta. Las máscaras, covarianzas y compensación son las vigentes.

## Velocidad, radio y seguridad geométrica

Por pedido del operador se evalúan1.2/1.6 y1.8 m/s. Producto conserva Nav2 R4,
steering automático18° y manual30°. El wrapper tools/heading_manual_envelope.launch.py
aplica30° sólo en simulación, con readback registrado. R3/R2/R1.65 son solicitudes;
los límites de juntas internas/clamp/dinámica pueden impedir alcanzarlas.
R1.65 sostiene6 s y puede describir una vuelta: es **estimador-only**, sin Nav2,
no evidencia de navegación integral. Las matrices Nav2 mantienen guard de U/rulos;
el override R3 es experimental, sin promoción a defaults. No hay hardware ni deploy.

## Datos y métricas

`summary.csv/json`, `aggregates.json`, comparación PNG/SVG y carpetas por trial.
Errores primarios: header stamp, yaw envuelto, interpolación del referente con
gap<=0.2 s, sin extrapolación ni fit espacial/temporal. Fases son ventanas de
comando; el giro físico>=0.18 rad/s se informa aparte. Lag usa soporte común y
es report-only. Recuperación<=2°/0.25 m durante1 s es descriptiva y se censura
si no ocurre; no es umbral de seguridad. Continuidad incluye bordes y stamps
únicos. Los fallos no se eliminan de tasas ni se convierten en baja precisión.

La referencia cinemática de comandos mide tracking separado del error EKF.
Los bundles Nav2 conservan metas, replans, bloqueos y tracking con pose física;
el remap se aplica **sólo al proceso evaluator** y se registra en metadata/manifest.
Un rechazo del guard no cuenta como llegada ni como mejora de localización.

`analysis-samples.jsonl.gz` conserva todas las muestras, stamps, poses, rates,
comandos, GPS/debug y seis diagonales de covarianza; sólo omite off-diagonales.
Es entrada directa de heading_turn_metrics.py para reproducir los números.
raw-sha256.json identifica crudos completos locales, también con covarianzas
cruzadas. analysis-provenance.json fija código del análisis. No se llama
innovación interna al error externo; debug EKF sólo si lo exige una atribución.

El observador físico usa map/base_footprint y worldPose. El spawn x/y/yaw=0 y
el datum/yaw del world y mapper coinciden. La comparación fija map=odom del
mundo, sin alinear al EKF ni restar sesgo inicial. Otros datums/spawns requieren
transformación explícita; estas herramientas no analizan bags reales genéricos.

## Variante y confirmación

Ejecutar con carpetas nuevas, después de conservar la referencia:

```bash
/usr/bin/python3 tools/heading_turn_campaign.py \
  docs/migration-evidence/ekf-turns-20261018/lagged-variant-matrix.json \
  artifacts/issue328-variant-new --jobs 2
/usr/bin/python3 tools/heading_turn_campaign.py \
  docs/migration-evidence/ekf-turns-20261018/holdout-matrix.json \
  artifacts/issue328-holdout-new --jobs 2
python3 tools/summarize_turn_campaign.py artifacts/issue328-variant-new \
  artifacts/issue328-variant-summary-new --plot
python3 tools/summarize_turn_campaign.py artifacts/issue328-holdout-new \
  artifacts/issue328-holdout-summary-new --plot
python3 tools/compare_heading_variants.py \
  artifacts/issue328-physical-summary-new/summary.json \
  artifacts/issue328-variant-summary-new/summary.json \
  --output artifacts/issue328-paired-new.json --plot
python3 tools/compare_heading_variants.py \
  artifacts/issue328-holdout-summary-new/summary.json \
  --output artifacts/issue328-holdout-paired-new.json --plot
python3 tools/summarize_heading_navigation.py artifacts/issue328-nav-r4-new \
  artifacts/issue328-nav-r4-observations-new.json
```

La selección rechaza pares sin parámetros efectivos/captura válida. Los JSON
paired conservan también esos fallos; las barras rayadas no son elegibles.
`tools/plot_turn_series.py <trial> <output-prefix>` reproduce las series físicas.
`tools/plot_navigation_reference.py <bundle>... --output <prefix>` grafica primer
plan publicado, pose física y meta; no confunde ese plan con el activo interno.
