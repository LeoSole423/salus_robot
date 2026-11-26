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

- ROS controla PTZ, presets, luces y estado; MediaMTX transporta video y
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

## Extensión 2026-10-14: paneo y luces

`CameraFeatures` es un servicio aditivo del mismo owner. El paneo tiene dos
fases acotadas (ir al extremo izquierdo, recorrer hacia el derecho), conserva
tilt/zoom y puede detenerse. Los comandos PT manuales cancelan el paneo.
Un timer del nodo serializa sus operaciones con el backend, sin hilo oculto
en Cockpit. Encender/iniciar requiere lease; apagar/detener permanece accesible
sin lease. El recorrido alterna ambos extremos hasta detenerse; desconectar Cockpit no
lo detiene ni crea una nueva orden de inicio.

La luz fija renueva órdenes temporales. La luz móvil usa configuración de
imagen: selecciona blanco y fuerza noche; antes de mutar guarda atómicamente
el perfil completo en un archivo junto a los presets. Apagar, cierre limpio o
arranque posterior restaura el perfil. Un cierre abrupto puede dejar la luz
móvil activa hasta la recuperación posterior: no tiene timeout físico probado.
Ante fallo de red se informa estado desconocido y no se reintenta indefinidamente.
El estado de luces representa intención reconocida, no telemetría óptica.

Actualización 2026-10-14: el paneo alterna derecha/izquierda continuamente
hasta Stop explícito, un comando PTZ manual, error o cierre del nodo.
Mantiene el timeout de 120 s por tramo; cada llegada renueva el plazo.
Cerrar la vista de Cockpit no detiene el recorrido. Velocidad normalizada
ONVIF 0.65 para paneo; flechas manuales conservan 0.15.
