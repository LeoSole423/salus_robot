# ADR 0005: Lease exclusivo para el control desde Cockpit

- Estado: aceptada
- Fecha: 2026-08-21

## Contexto

El bridge legacy mantenía un lock y un único timestamp de heartbeat globales.
Aceptaba varias conexiones WebSocket, pero no registraba cuál había
desbloqueado el control. Por ello una pestaña atrasada podía renovar el
heartbeat o comandar el robot después de que otra conexión asumiera la
operación.

Cockpit no envía una identidad autenticada y esta etapa conserva su protocolo.
El identificador estable disponible es la propia conexión WebSocket. El bind
en `0.0.0.0` continúa suponiendo una red de operación confiable; el lease no
sustituye autenticación ni TLS.

## Decisión

- La conexión que ejecuta correctamente `set_control_lock` con `locked=false`
  adquiere un lease exclusivo.
- Sólo esa conexión puede enviar heartbeat u operaciones controladas.
- Otro cliente recibe `CONTROL_OWNED`; no puede robar ni renovar el lease.
- Cualquier conexión puede frenar, cancelar, consultar estado, modificar zonas
  o solicitar `locked=true`, porque son operaciones de detención o diagnóstico.
- Bloquear libera el lease. La desconexión del propietario bloquea con causa
  `UI_CLIENT_DISCONNECTED`.
- El heartbeat vencido bloquea con `UI_HEARTBEAT_TIMEOUT` y libera el lease.
- El identificador de conexión nunca se publica. Las respuestas sólo indican
  si existe propietario y si el solicitante es ese propietario.
- Si el lock se deshabilita explícitamente por configuración, el lease no
  restringe comandos; las capas manual, freno, E-stop y collision monitor
  conservan su precedencia.

## Consecuencias

Se evita control concurrente sin cambiar el contrato de Cockpit. Una
reconexión debe volver a desbloquear y no hereda autoridad de la conexión
anterior. Autenticación, autorización por usuario y control remoto fuera de una
red confiable requieren otro ADR y no se presentan como capacidades actuales.
