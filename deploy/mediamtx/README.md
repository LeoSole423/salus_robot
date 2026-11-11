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
fuentes son bajo demanda, con cierre tras 10 s sin lectores y `record: false`
explícito para no heredar grabación de los defaults globales. Ahorro conserva
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

### Ajuste final: trasera a 15 FPS

Por pedido posterior del operador, el principal trasero conserva 1280×720 y pasa
de 25 a 15 FPS y de 4096 a 2048 kbit/s. Los tres principales de Normal quedan
así en 720p/15 FPS, con límite 2048 kbit/s por cámara. No se modificaron GOP,
codec, otros campos ni el secundario de Ahorro: comparación completa ONVIF
posterior correcta. Respaldo privado en
`camera-diagnostics/rear-15fps-20261104T170334Z` de la Jetson. Verificación RTSP
posterior en `docs/migration-evidence/camera-rear-15fps-20261104.json`.

## Frontal TCP de sólo video — 2026-11-05

La cámara frontal rechaza TCP al abrir todas sus pistas (RTSP 500); la misma
fuente acepta TCP seleccionando sólo video antes de SETUP. La solución evita
recodificación: `front_video_relay.py` copia H.264 por TCP y publica a loopback.
MediaMTX sirve cam3/cam3_normal como publishers con hooks bajo demanda. Se
conservan URLs WHEP y encoders. Los detalles y límites están en
[la ficha de intención](../../docs/migration-evidence/intent/camera-front-low-latency.md).

1. Respaldar el YAML efectivo y guardar las dos URLs RTSP originales en un JSON
   privado 0600 usando `front-video.private.example.json` como estructura.
2. Preparar un FFmpeg compatible con el host y verificar `ffmpeg -version` con
   el mismo `LD_LIBRARY_PATH` del launcher. El ejemplo no instala dependencias.
   En Jetson se copió el binario de grabaciones a un directorio independiente y
   las bibliotecas no-glibc resueltas con ldd desde la imagen existente. No
   reemplazar libc/loader del host ni asumir que el paquete binario aislado
   contiene todas sus dependencias. Documentar versión y procedencia privadas.
3. Instalar el launcher y ajustar rutas en `front-relay.example.yaml`. Esa
   plantilla sustituye únicamente las dos rutas frontales, manteniendo todas
   las demás rutas/globales. No reemplazar todo el YAML con el ejemplo.
4. Validar el candidato en la versión instalada con listeners deshabilitados.
   Aplicar primero cam3_normal y verificar video/estado; luego cam3 con una
   ventana para reconexión de la grabación frontal. No reiniciar navegación.
5. Confirmar una sola pista H264, ambos perfiles de calidad, métricas WebRTC,
   recuperación de rec_cam3/4/5 y que el helper de Normal termine sin lectores.

La fuente publisher usa `runOnDemand*`, no `sourceOnDemand*`. Un lector de
**grabación** también mantiene demanda de cam3 aunque el usuario pause el video.
El launcher no permite otras rutas y no imprime fuentes privadas al fallar;
MediaMTX registra los fallos/reintentos del hook. No se implementa fallback
silencioso a UDP: el error queda visible y el hook reintenta.

Campos privados del launcher: `ffmpeg` es la ruta string a un ejecutable probado;
`ffmpeg_library_path` es un string opcional con directorios separados por `:`;
`sources` mapea exactamente los nombres frontales a URLs RTSP autenticadas;
`publish_port` es un entero entre 1 y 65535, default 8554, para publicar sólo en
127.0.0.1. Los timeouts 15 s de arranque y 10 s sin demanda pertenecen a los hooks
MediaMTX del ejemplo. El relay no expone puertos nuevos ni API ROS.


El [registro de despliegue y operación](../../docs/migration-evidence/camera-front-deployment-20261105.md)
identifica los archivos efectivos del host, comandos de inspección, demanda por
grabación, estado de integración, validaciones y rollback selectivo.

## Mobile/PTZ video-only TCP — 2026-11-11

The same `front_video_relay.py` launcher also accepts `cam4` and `cam4_normal`.
Use `mobile-relay.example.yaml` to replace only those two routes after backing
up the private YAML, helper and source JSON. Preserve original mobile secondary
and main URLs under the corresponding private `sources` keys. Do not derive
credentials or substitute a frontal URL if a mobile entry is missing.

Input selects video before RTSP SETUP, uses TCP and copies H264 without encoding,
audio or metadata. Ahorro remains 360p/15 FPS; Normal 720p/15 FPS; camera encoder
settings and ONVIF/PTZ controls are independent and unchanged. The AI detector
holds saving open; normal closes ten seconds after its last reader leaves.
Hot reload interrupts only mobile video briefly and does not restart MediaMTX,
ROS, the detector or PTZ. Existing recording readers may reconnect during the
transition. Restore only mobile entries from the private snapshot for rollback;
never overwrite concurrent front, rear, recording or global changes.
