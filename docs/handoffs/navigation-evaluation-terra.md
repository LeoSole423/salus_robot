# Handoff TERRA: integración de evaluación de navegación

## Punto de partida

Rama `agent/navigation-evaluation-foundation`. SOL fijó en ADR 0007 el dominio,
la semántica de escenarios, métricas y gates. No cambiar esos contratos ante un
problema mecánico de integración: devolver a SOL cualquier ambigüedad.

## Integración completada por TERRA

- `navigation_evaluation` observa los tópicos estándar definidos, adopta una
  meta RViz o publica una meta de escenario mediante `/goal_pose`, y nunca
  publica comandos ni TF.
- `evaluation_observer.launch.py` permite adjuntarlo a una simulación existente.
- `tools/nav_eval.sh run|observe` crea los bundles JSON/CSV/HTML en
  `artifacts/evaluations/`; los marcadores se publican en
  `/navigation_evaluation/markers`.
- `integration_sim.launch.py` acepta `nav2_params_file:=...` para que perfiles
  de evaluación sean explícitos y reproducibles.

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

## Decisión SOL resuelta

`functional_gates` exige actualmente al menos una muestra angular elegible para
que `turn_sign` pase. Eso es correcto para giros izquierdo/derecho, pero hace
fallar semánticamente `straight_5m`, donde no debería existir tal comando. SOL
resolvió que una recta sin comandos angulares pasa, mientras cualquier comando
angular elegible sigue obligado a producir una respuesta física coherente. En
giros, además, el primer comando relevante debe coincidir con `expected_turn`.

## Decisión operativa pendiente

Nav2 usa actualmente `xy_goal_tolerance: 1.2`, mientras el runner nació con
`goal_tolerance_m: 0.25`. Una ejecución real confirmó que Nav2 puede declarar
éxito cerca de 1 m y el evaluador marcarlo fallido. Se propuso separar el gate
funcional de 1.2 m del objetivo de rendimiento de 0.25 m; falta confirmación del
operador antes de fijar esa semántica.
