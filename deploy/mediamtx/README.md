# Calidad de transmisión en vivo

MediaMTX pertenece al host. ROS conserva PTZ y grabaciones; Cockpit selecciona
rutas WHEP por navegador, sin servicios ROS nuevos ni cambios de lease.

## Contrato

| Cámara | Ahorro (existente) | Normal (adición) | Fuente principal |
| --- | --- | --- | --- |
| Frontal | cam3 | cam3_normal | /media1/video1 |
| Móvil | cam4 | cam4_normal | /media2/video1 |
| Trasera | cam5 | cam5_normal | /media/video1 |

Cada ruta termina en `/whep` en el servidor WebRTC ya configurado. Las nuevas
fuentes son bajo demanda, con cierre tras 10 s sin lectores. Ahorro conserva
las fuentes `/video2`. No modificar `rec_cam3/4/5`, hooks de grabación, parámetros
WebRTC globales ni encoders como efecto del botón.

## Preparación y aplicación

1. Inspeccionar servicio y configuración efectiva del host, revisión del backend,
   lectores activos y grabaciones. Respaldar el YAML privado con permisos 0600.
2. Verificar codec/resolución/FPS de los principales. Exigir H.264 para las tres
   fuentes antes de habilitar Normal. Los cambios de encoder requieren decisión
   del operador y respaldo específico, y no los realiza este script.
3. Con Python 3 y PyYAML, ejecutar en el host:
   `python3 prepare_quality_config.py CONFIG_PRIVADA CANDIDATO_PRIVADO`.
   El destino debe ser nuevo; contiene credenciales y se crea con permisos 0600.
   La herramienta agrega rutas sin cambiar las existentes; no aplica ni reinicia.
4. Revisar el candidato privadamente. `quality-paths.example.yaml` documenta
   únicamente las adiciones y no reemplaza el archivo completo del host.
5. Coordinar aplicación y rollback con el operador. Verificar la recarga en la
   versión instalada de MediaMTX; si exige reinicio, interrumpirá sesiones y debe
   hacerse en una ventana acordada. No reiniciar el runtime ROS para este cambio.
6. Configurar en Cockpit `camera_normal_webrtc_url`,
   `camera_mobile_normal_webrtc_url` y `camera_reverse_normal_webrtc_url` con las
   URLs reales. Validar ambas calidades por cámara, reconexión, bitrate recibido,
   sesión de otro operador y una grabación activa. No iniciar movimiento.

Rollback: volver a Ahorro en Cockpit y restaurar el YAML privado anterior,
verificando servicio, rutas y grabaciones. Restaurar cualquier encoder cambiado
mediante su respaldo propio. No guardar configuraciones efectivas ni secretos en Git.

## Estado observado el 2026-11-04

Consulta SSH de solo lectura: MediaMTX y runtime activos; backend `e90bc79`.
`ffprobe` sobre RTSP/TCP confirmó principales frontal/móvil H.264 1280×720,
15 FPS y trasera HEVC 1920×1080, 25 FPS. El bitrate no fue informado por ffprobe.
La incompatibilidad inicial del principal trasero se resolvió con autorización
del operador, como se documenta debajo.
No confundir estas pruebas de fuentes con validación WebRTC integrada.

### Aplicación y comprobación

El operador autorizó cambiar sólo el principal trasero a H.264. Se respaldó
GetProfiles en el directorio privado `camera-diagnostics/2026-11-04-quality` del
host y se envió SetVideoEncoderConfiguration conservando los campos recibidos,
incluidos 1920×1080, 25 FPS, GOP 50 y límite 2048 kbit/s. ONVIF ya anunciaba H.264
antes del cambio aunque RTSP era HEVC; la lectura ONVIF sola no es evidencia
suficiente del codec ni un respaldo fiel del valor HEVC anterior. La comprobación
RTSP posterior confirmó H.264 y conservación de resolución/FPS.

MediaMTX v1.19.1 aceptó el candidato en un proceso aislado con todos los listeners
deshabilitados, incluido MoQ. Se aplicó por recarga de archivo, sin reiniciar el
proceso (PID 1559). La comparación semántica confirmó que sólo se agregaron las
tres rutas. No había lectores ni grabaciones activas en ese momento. La copia
privada `mediamtx-before.yml` permite rollback del relay. Restaurar HEVC en la
cámara requeriría un mecanismo nativo verificado; no afirmar que el respaldo
ONVIF restaura ese codec.

Las seis rutas entregaron H.264 por RTSP/TCP. Ahorro: 640×360/15 FPS en las tres;
Normal: 1280×720/15 FPS frontal y móvil, 1920×1080/25 FPS trasera. La muestra de
aproximadamente cinco segundos midió payload de video, no consumo total de red:
ver `docs/migration-evidence/camera-stream-quality-20261104.json`.

La recarga de archivo está documentada por el proveedor en
[Configuration](https://mediamtx.org/docs/features/configuration). Esta versión
no expone `--validate-conf`; por eso se usó el proceso aislado con listeners
deshabilitados, sin modificar fuentes ni parámetros globales del servicio real.

## Ajuste de Normal — 2026-11-04, posterior a la integración

A pedido del operador se aumentaron los límites de bitrate de los principales y
se consultaron las capacidades ONVIF de FPS. Frontal y móvil admiten como máximo
15 FPS; la trasera, 25 FPS. Se conservaron esos máximos. El operador pidió además
bajar únicamente el principal trasero a 720p, resolución anunciada como soportada.

| Principal | Resolución | FPS | Límite de bitrate |
| --- | --- | --- | --- |
| Frontal | 1280×720 | 15 | 2048 kbit/s (antes 512) |
| Móvil | 1280×720 | 15 | 2048 kbit/s (antes 512) |
| Trasera | 1280×720 (antes 1920×1080) | 25 | 4096 kbit/s (antes 2048) |

H.264, GOP y los demás campos se conservaron. La comparación completa de las
configuraciones ONVIF posteriores confirmó que todos los encoders secundarios
permanecieron idénticos, incluido su límite de 256 kbit/s. No se modificaron
MediaMTX, las rutas de grabación, el frontend ni el runtime ROS.

Respaldos privados previos al tuning: `camera-diagnostics/normal-tuning-20261104T165909Z`
en la Jetson; respaldo adicional de resolución trasera en
`camera-diagnostics/rear-720p-20261104T165945Z`. Para rollback, reaplicar únicamente
las configuraciones de encoder principal respaldadas con ONVIF
SetVideoEncoderConfiguration y ForcePersistence; verificar RTSP después. No
reaplicar snapshots completos del dispositivo ni tocar encoders secundarios.

La medición posterior de los seis feeds está en
`docs/migration-evidence/camera-normal-tuning-20261104.json`. Los límites
configurados no equivalen a consumo constante: la muestra mide payload de video
RTSP, excluye audio y overhead, y depende de la escena. La mejora visual durante
transiciones de iluminación requiere aceptación del operador, no se deduce sólo
de bitrate o de una escena estática.
