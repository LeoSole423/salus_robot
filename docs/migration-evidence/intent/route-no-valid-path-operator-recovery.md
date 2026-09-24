# Intención: explicar y recuperar una ruta sin camino válido

## Alcance

- Destino: `salus_navigation` y el estado de misión consumido por Cockpit.
- Incluido: diagnóstico de fallo del planificador, motivo para el operador y cambio de perfil después de agotar la recuperación.
- Fuera de alcance: cambiar los límites de seguridad, limpiar costmaps automáticamente o afirmar la causa física del obstáculo.

## Evidencia observada

- En la simulación del 24/09/2026, Nav2 registró `GridBased: failed to create plan, exceeded maximum iterations` y abortó el goal. La política agotó 3 intentos y quedó en `NEEDS_OPERATOR` con `NAV_ABORTED`.
- Tras retirar las dos cajas, el costmap global en ese sector quedó libre. La misión continuó activa en ROS sin goal Nav2; Cockpit mostraba `Ready`, deshabilitaba CANCEL y el perfil previo al envío de una nueva ruta era rechazado por misión activa.
- Una cancelación explícita seguida de START ROUTE creó otra misión anclada cerca del robot, desde el índice 9, y Cockpit volvió a mostrar CANCEL.

## Contratos e invariantes

- Se conservan los mensajes y servicios públicos. `NavTelemetry.failure_code` puede transportar `NO_VALID_PATH` cuando un aborto coincide temporalmente con un error de `planner_server` del goal actual.
- `GetRouteMissionState.blocked_reason_code` mantiene un código estable; `blocked_reason_text` se presenta en lenguaje de operador.
- El perfil sólo puede cambiar con misión activa si la recuperación está en `NEEDS_OPERATOR`; en los demás estados se conserva la prohibición.
- Las señales de seguridad conservan prioridad sobre la explicación del planificador.

## Diseño y límites

- `PlannerFailureEvidence` correlaciona el log de `planner_server` con el epoch del goal y una ventana de 12 s. Es evidencia diagnóstica de que no hubo plan válido, no prueba que un obstáculo físico sea la causa.
- El adaptador ROS observa `/rosout` (`rcl_interfaces/msg/Log`, QoS depth 10, productor Nav2, consumidor `nav_command_server`) y publica el código en la telemetría existente.
- La política de recuperación conserva sus límites e intentos. Cuando ya requiere operador, se permite aplicar el perfil antes de reemplazar la ruta; no se reanuda la marcha automáticamente.
- No hay bag de este episodio ni validación en robot físico. La aceptación inicial es en simulación y tests del paquete.
