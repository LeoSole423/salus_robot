# Despliegue y operación de la frontal — 2026-11-05

## Estado registrado al finalizar la intervención

La corrección está aplicada en el host Jetson. El código de implementación es
`9b73641`, rama `agent/camera-front-low-latency` de salus_robot. Este corte aún no
está fusionado a main. Cockpit conserva main y no necesitó cambios. El backend
ROS desplegado no se sustituyó ni se reinició para este trabajo.

MediaMTX sigue siendo el propietario del video en el host. Las rutas frontales
cam3 y cam3_normal son publishers bajo demanda de una copia H.264 por TCP:

```text
Cámara frontal /media1/video2 (Ahorro) o /media1/video1 (Normal)
  -> RTSP/TCP, selección de sólo video antes de abrir pistas
  -> FFmpeg: copia H.264, sin decodificación/recodificación/GPU
  -> RTSP/TCP a 127.0.0.1:8554/cam3 o cam3_normal
  -> MediaMTX -> WHEP -> Cockpit
```

No cambian URLs WHEP, PTZ, encoders, cámara móvil, trasera, grabación ni contratos
ROS. La recarga de las rutas sí provocó reconexión breve de video/grabación
frontal, con recuperación verificada de las tres rutas de grabación.

| Modo frontal | Perfil de cámara conservado |
| --- | --- |
| Ahorro | H.264, 640×360, 15 FPS, límite 256 kbit/s |
| Normal | H.264, 1280×720, 15 FPS, límite 2048 kbit/s |

## Archivos efectivos del host

Obtener el directorio de MediaMTX con `systemctl show mediamtx -p WorkingDirectory`.
La instalación observada usa el subdirectorio `Desktop/SALUS/mediamtx` del home
del usuario del servicio; confirmar siempre la ruta efectiva antes de intervenir.

| Ubicación | Función |
| --- | --- |
| Directorio MediaMTX: `mediamtx.yml` | Configuración privada, con hooks en cam3/cam3_normal |
| Directorio MediaMTX: `front_video_relay.py` | Copia del launcher versionado en deploy/mediamtx |
| Directorio MediaMTX: `front-video.private.json` | Fuentes autenticadas y rutas de herramientas; permisos 0600 |
| Home del usuario: `.local/share/salus-mediamtx/bin/ffmpeg` | Copia independiente de FFmpeg 4.4.2 |
| Home del usuario: `.local/share/salus-mediamtx/ffmpeg-libs/` | 198 archivos de dependencias no-glibc, copiados de la imagen existente |

El binario y las bibliotecas se copiaron, no se instalaron paquetes ni se
modificaron bibliotecas del sistema. La instalación quedó independiente del
contenedor ROS una vez preparadas las herramientas. No borrar estos archivos
mientras los hooks los referencien. Los ejemplos versionados contienen sólo
marcadores de credenciales; nunca copiar el JSON/YAML efectivo a Git o a logs.

## Ciclo de vida y comprobaciones

- El primer lector inicia el hook. FFmpeg reemplaza al launcher y recibe SIGINT
  cuando termina la demanda; no queda un supervisor adicional.
- Arranque permitido: 15 s. Cierre: 10 s después del último lector. Se reintenta
  cuando el proceso termina con error; no hay fallback silencioso a UDP.
- La grabación también es un lector: cam3 puede permanecer activo aunque el
  operador pause el video. cam3_normal debe liberar su proceso sin lectores.
- El launcher sólo admite las dos rutas frontales y publica exclusivamente a
  loopback. No incorpora servicios/tópicos ROS ni puertos nuevos.

Comprobaciones de sólo lectura en Jetson:

```bash
systemctl is-active mediamtx salus-robot-real
systemctl show mediamtx salus-robot-real -p Id -p MainPID -p NRestarts
curl -fsS http://127.0.0.1:9997/v3/paths/list
journalctl -u mediamtx --since '-5 min' --no-pager
ps -C ffmpeg -o pid,pcpu,pmem,etimes
```

En las rutas frontales listas se espera una sola pista H264. Comprobar además
rec_cam3/rec_cam4/rec_cam5 si la grabación está solicitada. No imprimir el API de
configuración completo: contiene fuentes privadas. El journal puede incluir
metadata de sesiones; revisar/redactar antes de compartir.

## Evidencia y límites

El intento directo de TCP produjo RTSP 500 y se revirtió. FFprobe confirmó que
abrir todas las pistas falla y seleccionar sólo video funciona; no se atribuye
el fallo a una pista concreta sin más evidencia. La copia TCP de sólo video
elimina ese requisito y evita retransmitir audio/metadata de la frontal.

Medianas del buffer incremental WebRTC en ensayos estacionarios de unos 40 s:

| Modo | Frontal | Trasera de referencia |
| --- | ---: | ---: |
| Normal | 56,97 ms | 84,96 ms |
| Ahorro | 52,89 ms | 94,14 ms |

No son latencia captura→pantalla. La red varió entre ensayos y hubo congelamientos
al arranque/red también en trasera; no se afirma ausencia de cortes ni reducción
causal de toda la demora. Quedan pendientes prueba en movimiento, red degradada,
reboot de host/cámara y persistencia tras pérdida de alimentación.

La prueba también corrigió la interpretación inicial del audio: el log MediaMTX
«2 tracks H264/G711» aparece con receiveAudio=false y sin audio en getStats. No
prueba audio negociado. Se verificó que la nueva fuente entrega sólo video aun
cuando el cliente solicita audio; la política de Cockpit permanece intacta.

Validación local: build de 16 paquetes; 1375 pruebas ROS sin fallos (3 omitidas);
16 pruebas de herramientas (5 nuevas del relay); selftest del harness; smoke
web-cockpit y validación del árbol Git limpio correctos. En la captura final,
MediaMTX conservó PID 1559 y el runtime PID 3028, ambos activos y NRestarts=0.

Ver [evidencia estructurada](camera-front-low-latency-20261105.json),
[intención y decisiones](intent/camera-front-low-latency.md) y
[guía/plantillas de instalación](../../deploy/mediamtx/README.md).

## Rollback sin reiniciar navegación

Los respaldos privados están bajo `camera-diagnostics` en el home del usuario:

- `front-relay-20261105T224533Z/mediamtx-before.yml`: anterior a ambos cambios;
  permite recuperar las definiciones RTSP originales de las dos rutas.
- `front-saving-relay-20261105T224715Z/mediamtx-before.yml`: anterior al cambio
  de Ahorro, con Normal ya corregido.

Con el robot detenido y una ventana para reconexión de video/grabación:

1. Respaldar el YAML efectivo actual y comparar con el respaldo elegido.
2. Preparar un candidato privado restaurando únicamente las entradas frontales
   necesarias. Conservar todas las demás rutas, settings globales y cambios
   posteriores del operador; no reemplazar el archivo entero sin esa revisión.
3. Validar en la versión instalada con listeners deshabilitados y aplicar por
   recarga de archivo. No reiniciar el runtime ROS ni cambiar navegación.
4. Verificar ambas calidades, consumidores de grabación y estado de servicios.
   Confirmar que los procesos bajo demanda se liberan cuando corresponda.
5. Conservar launcher, JSON y herramientas hasta que ninguna ruta los use. La
   limpieza de dependencias no forma parte de la reversión de configuración.
