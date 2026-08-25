# Handoff TERRA: integración de evaluación de navegación

## Punto de partida

Rama `agent/navigation-evaluation-foundation`. SOL fijó en ADR 0007 el dominio,
la semántica de escenarios, métricas y gates. No cambiar esos contratos ante un
problema mecánico de integración: devolver a SOL cualquier ambigüedad.

## Trabajo delimitado para TERRA

1. Implementar un colector ROS delgado que traduzca `/plan`, `/cmd_vel`,
   `/odom_raw`, `/odometry/local`, `/odometry/global` y el resultado Nav2 a los
   modelos puros. Validar timestamps, frames y datos finitos.
2. Crear el runner headless y el modo `observe` para una meta RViz arbitraria;
   ambos deben llamar exactamente las mismas funciones puras.
3. Persistir siempre `manifest.json`, `summary.json`, series CSV e informe HTML
   autocontenido bajo `artifacts/evaluations/<run-id>/`; rosbag2 es opcional.
4. Publicar overlays RViz con tipos estándar: trayectoria ejecutada, error,
   entrada a tolerancia y replans. El evaluador no publica comandos ni TF.
5. Añadir `tools/nav_eval.sh` con `run`, `observe`, `compare` y `annotate`, y
   permitir overrides explícitos de archivos Nav2/EKF desde el bringup.
6. Añadir suites de reemplazo/cancelación, manual, keepout y benchmark. En CI
   sólo fallan inicialmente los `functional_gates`; rendimiento se reporta como
   `calibrating`.

## Parada obligatoria

Detenerse si falta una fuente independiente de verdad, hay duda sobre autoridad
de comando/TF, se necesita cambiar schema v1 o un gate de seguridad, o se piensa
compensar ausencia de datos con defaults. Eso requiere revisión SOL.
