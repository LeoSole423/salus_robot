# Intención: métricas del EKF local para #60

## Alcance

- Fuente legacy: `ROS2_SALUS/src/navegacion_gps/config/localization_v2.yaml`.
- Destino nuevo: `salus_evaluation` y los bundles de evaluación.
- Incluido: P95 del error de yaw y resumen observado de covarianzas X/Y/yaw.
- Fuera de alcance: configuración EKF, launches, harness, hardware y promoción de variantes.

## Evidencia histórica

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| `src/salus_evaluation/salus_evaluation/metrics.py` | Ya calcula P95 para errores de posición con un helper común. | alta |
| `src/salus_evaluation/salus_evaluation/evaluation_runner.py` | El observador recibe `/odom_raw`, `/odometry/local` y persiste bundles v2. | alta |
| `src/salus_localization/config/localization_local_sim.yaml` | El baseline simulado conserva la selección actual de estados. | alta |

## Problema original e intención

El plan de #60 necesita comparar error de yaw y confianza declarada del EKF sin
convertir esas observaciones en gates. Las nuevas métricas deben ser
deterministas, pequeñas y compatibles con bundles anteriores.

## Contratos e invariantes

- Entradas: muestras observadas de `/odometry/local`, alineadas con el collector.
- Salidas: `yaw_p95_rad` y medianas/P95 de las diagonales X/Y/yaw de pose.
- Unidades: metros cuadrados para X/Y, radianes cuadrados para yaw.
- TF/autoridades: el observador no publica TF ni comandos.
- Compatibilidad pública: el esquema de bundle v2 y sus gates no cambian; el nuevo
  resumen es opcional para consumidores de bundles antiguos.

## Diseño nuevo

- Modelos inmutables: `TimedPose` conserva las tres diagonales como campos opcionales;
  `LocalizationCovarianceSummary` representa el resumen.
- Políticas puras: `localization_metrics()` reutiliza `_percentile()` y
  `covariance_summary()` descarta muestras ausentes/no finitas.
- Adaptador ROS: `_timed_odometry()` copia sólo las diagonales de pose observadas.
- Configuración: no se modifica ninguna configuración ni launch.

## Fallos y degradación

| Condición | Respuesta requerida | Evidencia/test |
| --- | --- | --- |
| Covarianza ausente o no finita | Resumen con `sample_count=0`/campos nulos; no cambia gates | Test unitario |
| Bundle anterior sin el campo nuevo | Consumidores siguen leyendo métricas existentes | Persistencia JSON compatible |

## Decisiones descartadas

- Promover una variante EKF: requiere la campaña A/B definida en #60.
- Añadir NEES o un sistema estadístico nuevo: no es necesario para este corte.

## Pruebas y aceptación

- Unitarios: `test_domain.py`, `test_artifacts.py`, `test_matrix.py`.
- Integración: no requerida para el cambio puro de métricas.
- Smoke: pendiente de la campaña de #60.
- Replay/banco/hardware: no ejecutado.

## Estado de evidencia

- Estado propuesto: `characterized`.
- No validado en hardware: todas las conclusiones actuales son de simulación/artefactos.
- Preguntas pendientes: ejecutar las variantes del plan y decidir si existe candidato claro.
