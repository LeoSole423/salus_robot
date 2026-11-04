# Intención: selector de calidad de video

## Objetivo y contrato

El operador solicita un botón debajo del acceso PTZ de Cockpit, compartido por
las cámaras frontal, móvil y trasera de su navegador. Ahorro usa los substreams
actuales; Normal selecciona los principales. La preferencia local persiste y el
primer uso queda en Ahorro. No cambia el modo de otros operadores ni las rutas
de grabación. Las cámaras pausadas permanecen pausadas.

## Evidencia y frontera

Inspección del 2026-11-04: Cockpit configura `cam3/4/5`, y su superficie WebRTC
cierra el reader al cambiar URL. MediaMTX pertenece al host, fuera de ROS.
La Jetson ejecuta backend `e90bc79`; su YAML efectivo conserva rutas separadas
`rec_cam3/4/5`. RTSP/TCP confirmó principales H.264 720p/15 FPS para frontal y
móvil; la trasera usa HEVC 1080p/25 FPS. No se modificaron encoders durante la
inspección. Las fuentes efectivas contienen secretos y no se adjuntan.

Cockpit implementa el estado local compartido y selección de URLs. Este repo
aporta plantilla y preparación privada de configuración incremental bajo
`deploy/mediamtx/`, sin contratos ROS nuevos ni cambios al controlador.

## Fallos y evidencia pendiente

Configuración incompleta o transporte MJPEG deshabilita Normal y usa Ahorro.
Errores WebRTC permanecen visibles y permiten volver al modo Ahorro; no se
anuncia conexión hasta recibir frames. No se abren cámaras pausadas al alternar.

La incompatibilidad inicial de la trasera se resolvió con autorización explícita
del operador, como se registra en el resultado del corte. La aceptación final requiere medir calidad/bitrate, probar los seis
feeds WHEP, otro navegador y coexistencia con grabación. Build y pruebas locales
no constituyen validación integral de hardware.

## Resultado del corte

Autorización explícita del operador para convertir el principal trasero a H.264,
conservando resolución/FPS/substream. Cambio aceptado por ONVIF y confirmado por
RTSP; el substream conserva H.264 640×360/15 FPS. El relay recargó las tres rutas
sin reinicio, sin cambiar definiciones anteriores. Evidencia de los seis streams:
[medición RTSP](../camera-stream-quality-20261104.json). Los archivos privados de
respaldo permanecen en la Jetson y no se incluyen en Git.

Build ROS: 16 paquetes. Suite ROS: 1375 pruebas, cero fallos, tres omitidas;
tres pruebas adicionales de preparación del relay correctas. Smoke web-cockpit
en simulación correcto. `tools/test.sh` se detiene antes de ejecutar la suite por
enlaces inválidos preexistentes dentro de `.agents/` no versionado; se ejecutó
la suite ROS y el selftest del harness directamente, sin modificar esa carpeta.
Cockpit: build correcto, 236 pruebas correctas y una omitida.

La coexistencia con grabación física activa y dos operadores reales requiere
aceptación adicional: no había sesiones activas al aplicar la configuración.

Verificación visual adicional: los tres videos WebRTC de Cockpit local entregan
640×360 en Ahorro y 1280×720 / 1280×720 / 1920×1080 en Normal, con readyState=4 y
paused=false. El mapa frontal también cambia entre 360p y 720p. La preferencia
Normal sobrevivió una recarga. El frontend de producción no fue publicado.
Se verificó reconexión y cierre del reader con dos pruebas adicionales. Las 37
pruebas de herramientas de runtime/despliegue también pasaron.

## Ajuste posterior de los perfiles normales

El operador solicitó mejorar bitrate/FPS y luego bajar la trasera a 720p.
Se configuraron frontal/móvil 720p, 15 FPS, límite 2048 kbit/s; trasera 720p,
25 FPS, límite 4096 kbit/s. Los FPS ya estaban en el máximo anunciado por cada
hardware, por lo que no se aumentaron. La comparación completa de perfiles
ONVIF confirmó conservación de los secundarios y de todos los campos ajenos al
ajuste solicitado. Respaldos privados y procedimiento en el README de despliegue.
Evidencia posterior: [tuning Normal](../camera-normal-tuning-20261104.json).
No se ejecutó movimiento ni se reiniciaron servicios para ajustar encoders.
