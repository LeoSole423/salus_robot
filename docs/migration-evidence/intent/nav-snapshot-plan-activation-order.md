# Intención: retención visual frente al orden de aceptación Nav2

## Hechos observados

- Nav2 publica el plan inicial en `/plan` antes de que `nav_command_server`
  publique la primera telemetría con `goal_active=true`.
- `/plan` no es periódico durante una navegación sana. Tratarlo como sensor
  de dos segundos hace que Nav Live pierda la ruta verde aunque la acción siga
  activa.
- La primera retención visual sólo aceptaba planes recibidos después de la
  telemetría activa, por lo que no retenía el plan inicial real.

## Decisión

- En el flanco `goal_active=false -> true`, el snapshot conserva el plan ya
  cacheado únicamente si todavía supera la política dinámica de frescura de
  ADR 0004.
- Un plan posterior reemplaza el retenido y una telemetría terminal lo borra.
- No cambia las entradas críticas de TF/costmap, ni publica comandos o TF.

## Fuera de alcance

- Los hallazgos de paridad del BT single-pose, reintentos internos y RPP
  documentados en #244 permanecen para cortes de caracterización separados.
