# Intención caracterizada: observabilidad de la cadena de comandos

## Hechos

- `salus_evaluation` observaba `/cmd_vel` y odometría, pero no las etapas
  posteriores que pueden modificar una orden de Nav2.
- `/cmd_vel_safe` y `/cmd_vel_final` conservan semántica Twist; el último
  agrega fuente y freno.
- `/vehicle/command_shadow` expone la traducción Ackermann observacional.
- `/controller/drive_telemetry` publica velocidad medida y dirección medida en
  grados; el evaluador la normaliza a radianes.
- `/controller/status` publica el comando efectivo y flags como
  `steer_saturated`; `/controller/telemetry` publica la consigna automática y
  límites Ackermann configurados.

## Decisión

El evaluador registra esas fuentes sin publicar comandos ni TF. Correlaciona
sólo la última muestra causal previa dentro de 0,2 s, conserva el
`alignment_gap_s` y declara los pares ausentes como no disponibles. Estas
métricas son diagnósticas y no cambian gates funcionales.

## Límite

La evidencia caracteriza simulación y contratos publicados. No demuestra
respuesta de actuadores reales ni paridad de hardware.
