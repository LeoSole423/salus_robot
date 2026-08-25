# Intención: evaluación reproducible de navegación

## Alcance

Medir navegación Ackermann en la simulación operacional, tanto headless como
en RViz, y conservar evidencia comparable para futuros ajustes de Nav2, caminos
y fusión EKF.

## Hechos observados

- El smoke de navegación existente comprobaba movimiento y llegada, pero una
  comprobación de magnitud angular no detectó que el robot giraba al lado opuesto.
- La corrección integrada en PR #35 añadió causalidad de signo a través de
  `/cmd_vel`, `/odom_raw`, `/odometry/local` y `/odometry/global`.
- `/plan` y `nav_observer` ya exponen camino y cambios materiales de plan.
- `/odom_raw` proviene de simulación y sirve como referencia, no como evidencia
  de precisión sobre hardware.

## Intención preservada

- Una prueba debe expresar dirección esperada, observar respuesta y medir
  llegada, sobrepaso y movimiento posterior al éxito.
- El escenario y el perfil deben ser repetibles y producir artefactos aun al
  fallar.
- La visualización y el modo headless deben consumir las mismas mediciones.

## Inferencias y límites

- El episodio aislado de sobrepaso no justifica todavía cambiar tolerancias.
- Los umbrales cuantitativos necesitan distribución empírica antes de ser gates.
- No se afirma paridad ni validación física hasta disponer de bags/banco/robot.

## Pruebas exigidas

- Geometría relativa izquierda/derecha y error contra una polilínea conocida.
- Regresión explícita del giro con signo contrario.
- Entrada, salida, sobrepaso y distancia posterior al éxito.
- Error de localización contra una fuente de verdad independiente.
- Parser estricto y política de calibración/regresión.
