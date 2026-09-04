# Intención: snapshot ROS único para authority preflight de #168

## Alcance

Corregir el preflight host-side de `real_mvp` después de la aceptación física
abortada: los tópicos ROS se observan desde el runtime preparado con una única
ejecución y un único participant DDS. Los probes de systemd, procesos y devices
siguen siendo host-side y de solo lectura.

## Hecho causal

El checker de `main@53e275605f7aa457a6e8687855ae65bd5a1dcaaf` ejecutaba un
contenedor temporal por tópico crítico con el timeout genérico de host de cinco
segundos. En Jetson, el probe de `/cmd_vel_final` superó ese límite durante F2,
por lo que el checker devolvió exit 2 antes de poder demostrar authority libre.

## Decisión

- Ejecutar un solo snippet `rclpy` por checker mediante
  `tools/real_runtime_exec.sh`.
- Esperar una ventana acotada de discovery y emitir un JSON de publishers para
  todos los tópicos críticos; un tópico ausente es una lista vacía.
- Mantener el fallo cerrado ante runtime/JSON inválido y conservar el timeout
  corto de los probes locales.

## Fuera de alcance

No cambia launches, systemd, hardware, políticas de safety ni timeouts de
producto. La siguiente aceptación física sólo se repite tras CI verde y merge.
