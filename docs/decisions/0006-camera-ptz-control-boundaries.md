# ADR 0006: Fronteras del control PTZ y transporte de video

- Estado: aceptada
- Fecha: 2026-08-24

## Contexto

El nodo de cámara legacy reunía políticas de movimiento, HTTP/XML, secretos,
persistencia y servicios ROS. El bridge web también llegó a transportar frames
de cámara, aunque el despliegue operativo más reciente usa MediaMTX/WebRTC.
Además, un movimiento relativo requiere leer y escribir sin que otro comando
se intercale.

## Decisión

- ROS controla únicamente PTZ, presets y estado; MediaMTX transporta video y
  permanece externo.
- La geometría, aliases y política de presets serán lógica pura.
- El backend ISAPI será un adaptador sin dependencias ROS y el simulador
  implementará el mismo contrato sin producir video.
- Todas las operaciones del backend se serializarán para hacer atómicas las
  secuencias read-modify-write.
- Los presets se persistirán mediante reemplazo atómico fuera del árbol de
  instalación.
- El modo real falla cerrado si faltan credenciales y nunca las registra.
- Las mutaciones WebSocket requieren el lease exclusivo de ADR 0005; las
  consultas de estado no.
- Una cámara no disponible degrada sus servicios, pero no impide arrancar el
  resto del robot.

## Consecuencias

PTZ puede probarse de forma determinista sin cámara ni video, y un fallo de
cámara no se confunde con una pose cero válida. MediaMTX debe desplegarse y
diagnosticarse por separado. El backend real y la paridad permanecen pendientes
hasta una prueba de banco con credenciales suministradas externamente.
