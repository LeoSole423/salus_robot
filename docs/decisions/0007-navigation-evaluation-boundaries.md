# ADR 0007: Fronteras de evaluación de navegación

## Estado

Aceptado para la base de evaluación reproducible.

## Decisión

`salus_evaluation` es un observador sin autoridad sobre comandos, acciones,
lifecycle ni TF. El mismo dominio puro calculará resultados para ejecuciones
headless, sesiones visibles en RViz y metas interactivas.

Los escenarios v1 usan metas relativas al spawn y nombres lógicos de mundo;
rechazan campos desconocidos y datos no finitos. En simulación, `/odom_raw` es
la referencia de verdad terreno y las odometrías filtradas son estimaciones a
comparar. Esto no constituye validación del robot real.

Los invariantes causales (datos finitos, plan presente, resultado terminal,
llegada, signo comando-respuesta y prohibición de reversa) pueden fallar CI.
Las magnitudes de rendimiento comienzan como `calibrating`: serán informativas
hasta reunir al menos 30 repeticiones válidas por escenario/perfil. Después se
compararán individualmente mediante P95/P99 y límites explícitos; no habrá una
puntuación única que oculte regresiones.

La llegada distingue el contrato operacional vigente del objetivo de mejora.
Mientras Nav2 use `xy_goal_tolerance: 1.2`, ése será el gate funcional. El
objetivo de precisión de 0.25 m se registra separadamente como `calibrating` y
no rompe CI. Ambos valores son explícitos en cada artefacto y podrán reducirse
con evidencia acumulada.

No se crean mensajes ROS propios: el adaptador consumirá tipos estándar y
persistirá artefactos versionados JSON/CSV y un informe visual. Una ausencia de
datos invalida la prueba; nunca se rellena con éxito supuesto.

## Consecuencias

- Nav2 y el bringup permanecen como autoridades operativas.
- Las métricas pueden probarse sin ROS, Gazebo ni RViz.
- Rosbags y gráficos son evidencia opcional; el JSON resumido es contractual.
- Reemplazo/cancelación, keepouts y perturbaciones de sensores se incorporan
  como suites del runner, sin agregar autoridad al evaluador.
