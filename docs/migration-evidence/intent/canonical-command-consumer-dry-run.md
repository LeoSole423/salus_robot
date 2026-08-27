# Ficha de intención: consumidor canónico dry-run

## Alcance

Consumir `/vehicle/command_shadow` mediante la política de seguridad destinada
a los futuros backends, sin producir actuación. El nodo publica exclusivamente
diagnósticos y no reemplaza `/cmd_vel_final` como autoridad.

## Validación de entrada

El consumidor rechaza con comando seguro:

- escalares no finitos, fuente fuera del enum o duración no positiva;
- timestamp nulo, futuro más allá de la tolerancia, repetido o regresivo;
- mensaje que ya llegó vencido;
- freno fuera de `[0, 1]`, velocidad fuera de los límites físicos o dirección
  fuera del máximo configurado.

La vigencia efectiva es el mínimo entre `valid_for` y `max_valid_for_s`. Después
de aceptar una muestra, el watchdog usa exclusivamente tiempo monotónico de
recepción. Al vencer publica estado seguro: fuente `SAFETY`, tracción
deshabilitada, E-stop, freno completo y movimiento nulo.

E-stop y `drive_enabled=false` dominan la intención cinemática. Un freno de
servicio válido inhibe velocidad sin convertirse en E-stop, preservando la
separación del contrato canónico.

## Dry-run y autoridad

`canonical_command_dry_run` no instancia los backends existentes y no publica
Twist, setpoints, porcentajes ni bytes. Su única salida es
`/vehicle/command_dry_run/diagnostics`, identificada con `backend=dry_run` y
`authoritative=false`.

## Evidencia

- pruebas puras de todos los rechazos, dominancia y expiración;
- prueba ROS de aceptación y timeout monotónico;
- smoke causal que observa aceptación y luego estado seguro por watchdog;
- build y suite completa del repositorio.

## Próximo corte

Añadir un backend canónico de simulación seleccionable mediante perfil. La ruta
legacy continuará como default y será imposible habilitar simultáneamente dos
autoridades sobre el tópico de actuación simulado.
