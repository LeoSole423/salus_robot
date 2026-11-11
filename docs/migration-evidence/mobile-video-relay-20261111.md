# Móvil/PTZ: relay de video por TCP — 2026-11-11

A pedido del operador se extendió a cam4/cam4_normal la solución de cam3:
FFmpeg copia únicamente H264 por RTSP/TCP, sin decodificar, recodificar,
audio ni metadata. El launcher conserva su nombre front_video_relay.py y
agrega las dos rutas móviles a su allowlist. Las URLs originales de canal 2
permanecen en el JSON privado del host; no se modificaron encoders ni ONVIF.

Antes, cam4 usaba ingreso directo automático con H264/G711/Generic. No hubo
avisos de pérdida o decode en los cinco minutos inspeccionados; por ello no
se atribuye una reducción de delay/cortes a esta muestra. Se aplica el mismo
mecanismo previamente validado en la fija, sin prometer latencia total medida.

## Aplicación

Se respaldaron YAML privado, JSON de fuentes y helper en el directorio privado
`camera-diagnostics/mobile-relay-20261111T223138Z/`, relativo a HOME de admin.
Primero se copiaron 15 frames de cada fuente móvil directamente por TCP/video
para verificar soporte. El candidato se validó en otro proceso MediaMTX con
todos los listeners desactivados y sin hooks de inicio. Se comprobó que no
había cambios concurrentes ni diferencias en las otras rutas.

Se agregaron los originales cam4/cam4_normal al JSON privado y se sustituyeron
sólo esas dos rutas por publishers con hooks bajo demanda: restart true,
start timeout 15 s, close after 10 s y record false. YAML se recargó en caliente.
No se cambiaron rutas frontal/trasera, rutas de grabación ni opciones globales.
La transición corta brevemente el video móvil y sus lectores se reconectan.
PTZ conserva su backend y estado; no se enviaron movimientos de prueba.

## Verificación

- Cam4: 30 frames H264 copiados por RTSP/TCP, 640×360, 15 FPS.
- Cam4_normal: 30 frames H264 copiados por RTSP/TCP, 1280×720, 15 FPS.
- API MediaMTX: sólo pista H264 en ambos caminos durante la prueba.
- Consulta get_camera_ptz_state: ACK ok, sin escribir orientación ni zoom.
- Detector: las tres cámaras en watching, sin reiniciarlo.
- PIDs preservados: MediaMTX 1952, ROS 3712, detector 222320, NRestarts cero.
- Build 16 paquetes; 1377 pruebas ROS correctas (3 omitidas); 26 pruebas de
  herramientas correctas, incluyendo fuentes móviles por modo y rechazo de
  un alias móvil ausente. Validación del árbol Git limpio.

Evidencia local sin credenciales: artifacts/mobile-video-20261111/ de la raíz
SALUS (apply.json, verify.json y lectura PTZ). No se midió latencia de extremo a
extremo ni se probó un reinicio completo, red degradada o grabación activa.
La copia de Ahorro permanece bajo demanda del detector; Normal puede cerrarse
sin lectores, según el timeout del hook.

## Rollback

Restaurar únicamente las entradas cam4/cam4_normal desde mediamtx-before.yml,
tras comparar el YAML actual y conservar cambios posteriores. Retirar las dos
claves móviles del JSON sólo si ya no hay hooks que las usen. El helper nuevo
sigue siendo compatible con las rutas frontales, por lo que no es necesario
restaurarlo para revertir la móvil. Validar y recargar YAML; no reiniciar ROS.
Los respaldos privados contienen secretos y no deben copiarse a Git.
