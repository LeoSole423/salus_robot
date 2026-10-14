# Baseline de heading: recta, curva fuerte, recta

## Ejecución

Captura ejecutada antes de modificar localización, base main@2fc6973.
Worktree agent/heading-strong-turns. Instrumentación manual, sin tests nuevos
en CI. Producto de localización/control/sim idéntico a origin/main en esta rama.

Gazebo, world free, perfil clean, seed 6400, orientación course_over_ground.
Docker salus-robot:humble-dev, 4 CPU, network none, ROS_LOCALHOST_ONLY=1,
dominio 117 y partición Gazebo exclusiva. Build/install existentes montados
read-only; symlinks Python resuelven al src del worktree. No hardware físico.
No Nav2: maniobra open-loop por /cmd_vel, con control y arbitraje de simulación.

Secuencia por reloj simulado: recta de 8 s a 1,2 m/s; curva derecha de 4,5 s
a 1,2 m/s y petición angular de -0,35 rad/s; recta de salida de 8 s; stop de 3 s.
El giro nominal sería ~90 grados, pero la dinámica de dirección modifica la
trayectoria: giro medido de ~69,3 grados. Al comenzar la recta de salida todavía
continúa girando brevemente; etiquetas son fases de comando, no detección de
curvatura ejecutada. No retocar la captura para hacerla parecer ideal.

## Observaciones

| Fase de comando | Error yaw local máximo | Error yaw global máximo |
| --- | ---: | ---: |
| Recta inicial | <0,001 grados | 1,11 grados |
| Curva fuerte | 3,37 grados | 9,69 grados |
| Recta de salida | 3,90 grados | 9,93 grados |
| Stop | 3,87 grados | 1,86 grados |

Errores absolutos emparejados por header.stamp, interpolando /odom_raw entre
muestras vecinas y sin extrapolar. Agrupación por fase al recibir el mensaje;
mensajes demorados cerca de límites pueden aparecer en fase siguiente.
La gráfica usa tiempo del header respecto del inicio, salvo debug que no tiene
header. No interpretar la exactitud numérica de la recta ideal como física.

GPS course publicó 41 mediciones en recta inicial, 8 al inicio de curva y 41
en salida. Debug registró cinco hold_steer_too_high: se reutiliza el rumbo
anterior mientras la orientación real ya cambia. Después faltan correcciones
GPS durante la curva, pero los EKF siguen publicando heading. Primera corrección
en fase de salida a t=14,481 s, ~1,98 s después de cambiar comando a recta.
Error GPS máximo de 6,07 grados en curva y 3,76 grados en salida.
El global mejora después de volver a recibir correcciones. Hay también error
local: no atribuir todo el desfase a GPS ni prometer eliminarlo con su gating.

## Evidencia y reproducción

Bundle local ignorado por Git: artifacts/heading-baseline-20261014/.
Conserva samples.jsonl, sequence.json, capture.json, metrics.json,
gps-parameters.yaml, provenance.json (image ID y hashes de fuentes), run.log,
heading.svg y heading.html. Report del harness en
artifacts/smokes/heading-baseline-20261014T131527-1/.

Para repetir sin pisar la referencia:

```bash
bash tools/run_heading_capture.sh /ruta/al/workspace-compatible heading-after-change 118
```

Reutilizar build/install sólo si es compatible y los symlinks Python resuelven
al código bajo prueba. Recompilar si cambia configuración instalada, contratos,
entry points o fuentes compiladas. Guardar provenance de cada corrida; no basta
con el nombre de imagen. Los scripts de captura y resumen quedan versionados.

## Límites

La IMU clean se deriva de /odom_raw, sin sesgo/ruido: esta corrida demuestra
comportamiento del software actual en condiciones favorables, no precisión de
la IMU física. GPS clean tiene ruido configurado de 0,02 m. No se mide pared,
costmap, obstáculos fantasma, misión completa ni distancia de frenado. No se
alimenta external_heading ground truth al filtro global. No cambiar todavía
umbrales, covarianzas ni política. Comparación posterior debe conservar esta
secuencia, perfil y seed y mostrar diferencias en heading local/global, GPS,
tiempo de reacquisición y TF. Una sola corrida es baseline, no confiabilidad.
