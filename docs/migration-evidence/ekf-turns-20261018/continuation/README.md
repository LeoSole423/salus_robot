# #328: continuación ejecutada de navegación automática

[Resultados y límites](RESULTS.md), [comparación por intento](final-analysis/summary.csv),
[gráfica](comparison.png). La primera campaña de estimador queda en el directorio
superior. Esta continuación prueba navegación realimentada con Nav2, protecciones
de rulos/U vigentes y pose física independiente del estimador, siempre en PC.

## Reproducir

Desde la raíz del repositorio, con Docker/Humble y el build de esta rama:

```bash
./tools/build.sh
mkdir -p artifacts
cp -a docs/migration-evidence/ekf-turns-20261018/continuation/generated-configs/. artifacts/
python3 tools/run_heading_route_stages.py \
  docs/migration-evidence/ekf-turns-20261018/continuation/confirmation-stages.json \
  artifacts/replay-confirmation-ledger
python3 tools/run_heading_route_stages.py \
  docs/migration-evidence/ekf-turns-20261018/continuation/latency-stages.json \
  artifacts/replay-latency-ledger
python3 tools/run_heading_route_stages.py \
  docs/migration-evidence/ekf-turns-20261018/continuation/extension-stages.json \
  artifacts/replay-extension-ledger
python3 tools/run_heading_route_stages.py \
  docs/migration-evidence/ekf-turns-20261018/continuation/recovery-stages.json \
  artifacts/replay-recovery-ledger
```

Cada etapa usa `nav_eval.sh matrix --jobs 2`, dominios/puertos/artefactos del
harness y exportación de parámetros efectivos. Las colas se ejecutan
**secuencialmente**. Requieren directorios de salida nuevos: el runner rechaza
sobrescribir datos. En un checkout con artefactos originales, elegir outputs
nuevos en una copia del manifiesto. `stages.json` conserva la exploración inicial;
sus dos intervenciones300 inválidas se saltan explícitamente, no se repiten con
la etiqueta incorrecta. El wrapper retirado también falla explícitamente.

La extensión24° exige completar curvas previas en ambos lados sin consigna
nominal bajo1.2. El piloto2.0 de la exploración exige el par1.8/seed6501 definido
previamente; conservar fallos y la restricción de piloto al reproducirlo.

Ejemplo individual de GNSS300 ms con verificación efectiva previa a navegar:

```bash
SALUS_NAV_EVAL_SENSOR_PROFILE_FILE="$PWD/src/salus_evaluation/config/experiments/issue328_sensors_nominal300.yaml" \
./tools/nav_eval.sh matrix \
  src/salus_evaluation/config/matrices/issue328_route_latency300.yaml \
  artifacts/replay-300 --jobs 2 \
  --planner-minimum-turning-radius 3 --trial-start-sim-s 60 \
  --evaluation-ground-truth-topic /simulation/ground_truth
```

El montaje sustituye el perfil sólo dentro del contenedor. No cambia el archivo
nominal del host ni el perfil del producto. Un readback distinto de300 ms causa
setup fallido antes de enviar metas. Para comparar EKF temporal usar la matriz
`issue328_route_latency300_lagged.yaml`, mismo perfil y semillas.

## Datos y análisis

`exploration/`, `confirmation/`, `latency/` y `extension/` y `recovery/` guardan cada bundle
comprimido, SHA256 del archivo y de cada miembro, y resultados de proceso. Los
subdirectorios `reference-r4/`, `reference-r3/`, `radius3-auto18-planner3/` y
`issue328-route-*` conservan los pilotos previos. No se incluyen caches runtime;
sí CSV, planes, debug GPS, IMU cuando estaba instrumentada, parámetros, logs y
setup fallido. Los archivos de análisis por etapa pueden corresponder a una
versión anterior: `final-analysis/analysis-provenance.json` identifica el análisis uniforme.

```bash
mkdir -p artifacts/extracted
# Elegir un bundle de esta campaña:
tar -xzf docs/migration-evidence/ekf-turns-20261018/continuation/confirmation/coherent24/bundle.tar.gz \
  -C artifacts/extracted
python3 tools/summarize_heading_routes.py artifacts/extracted \
  --output artifacts/extracted-analysis
python3 tools/plot_heading_routes.py \
  docs/migration-evidence/ekf-turns-20261018/continuation/final-analysis/summary.json \
  artifacts/comparison.png
```

Plot requiere matplotlib; los analizadores requieren PyYAML. La
herramienta de archivo verifica bytes, no normaliza CRLF ni modifica datos.
`runtime-environment.json` fija imagen y versiones ROS/Fortress. El manifiesto de
cada trial incluye SHA de HEAD al iniciar y hashes de configuraciones. Los ensayos
de época iniciales empezaron mientras se preparaba el commit90982c5: el campo
HEAD anterior no representa por sí solo el código sin commit; ese commit conserva
la corrección de época/instrumentación usada. Los ensayos posteriores registran
la revisión ya comprometida. No se afirma repetibilidad bit a bit del tráfico ROS.
