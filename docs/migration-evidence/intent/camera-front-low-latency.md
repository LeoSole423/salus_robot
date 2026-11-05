# Intención: estabilizar la panorámica frontal sin alterar navegación

## Solicitud y hechos previos

El 2026-11-05 el operador reportó varios segundos de desfase y más cortes en
frontal que en trasera. Primero pidió inspección pasiva durante navegación;
luego confirmó robot detenido y autorizó trabajar sólo sobre cámaras.

La inspección encontró pérdidas RTP y errores de fragmentos H.264 en frontal
cam3/cam3_normal y pérdidas simultáneas en cam4, que comparte cámara. Trasera
cam5 fuerza TCP; frontal heredaba automático y recibía medios fuera del canal
TCP de control. El histórico de la frontal mostraba sensibilidad a audio/A-V.

## Caracterización y decisión

Forzar TCP directamente en cam3_normal produjo RTSP 500 y se revirtió. Dos
probes FFprobe sobre la misma fuente separaron la causa: pedir todas las pistas
por TCP falló, mientras `-allowed_media_types video` entregó H.264 720p/15 FPS.
La atribución exacta a audio o metadata no está determinada; no confundir el
fallo de SETUP de pistas con un fallo del codec H.264.

Además, el log MediaMTX «2 tracks H264/G711» también aparece en la prueba con
receiveAudio=false y sin audio recibido en getStats. Por tanto ese mensaje no
prueba negociación de audio por el navegador. La política de audio de Cockpit
permanece intacta, con frontal sin audio por defecto. No se modificó Cockpit.

## Implementación

Un launcher host-owned lee fuentes privadas y reemplaza su proceso por FFmpeg:
TCP, selección de video antes de abrir pistas, H.264 stream copy, publicación
RTSP/TCP a loopback en la misma ruta cam3 o cam3_normal. No decodifica, recodifica
ni usa GPU. MediaMTX administra inicio bajo demanda, reinicio por error y cierre
tras 10 s sin lectores. FFmpeg recibe SIGINT directamente al terminar la demanda.

Se preservan URLs WHEP, encoders y perfiles de calidad, PTZ, rutas de grabación,
servicios y todos los parámetros de navegación. MediaMTX recarga únicamente las
rutas frontales. La grabación que consume cam3 puede reconectar brevemente al
cambiar la fuente; se verificó recuperación de rec_cam3/4/5.

## Runtime y reversión

Se reutilizó FFmpeg 4.4.2 instalado para grabaciones, copiando binario y sus
bibliotecas no-glibc a un directorio privado independiente del host. No se
instalaron paquetes ni se alteraron bibliotecas del sistema, imagen o runtime ROS.
El launcher/configuración son archivos persistentes del host y no dependen del
contenedor una vez preparado FFmpeg. Los secretos permanecen fuera de Git y los
mensajes del launcher/FFmpeg no imprimen URLs autenticadas.

La aplicación inicial a Normal tiene respaldo privado
`camera-diagnostics/front-relay-20261105T224533Z/mediamtx-before.yml`; el cambio
posterior de Ahorro tiene respaldo
`camera-diagnostics/front-saving-relay-20261105T224715Z/mediamtx-before.yml`.
Restaurar ambos cambios con el primer respaldo, verificando antes que no haya
otras ediciones concurrentes. No reiniciar ROS para volver a las fuentes RTSP
anteriores. Conservar launcher, JSON privado y FFmpeg mientras alguna ruta los use.

## Validación y límites

Pruebas puras del launcher verifican selección previa de sólo video, TCP en
ambos tramos, ausencia de encoder, destino loopback, fuentes por modo y rechazo
de entradas no soportadas. Se validó el candidato con el binario MediaMTX
instalado y listeners deshabilitados antes de aplicarlo. Servicios conservaron
PID 1559 y 3028. El runtime no se reinició y no se enviaron comandos de movimiento.

Comparaciones WebRTC y estado de grabaciones se registran en
[resumen de evidencia](../camera-front-low-latency-20261105.json). Las mediciones
de buffer son incrementales por ventana de 1 s, no latencia captura→pantalla.
El ensayo es corto y estacionario; continuidad en movimiento y red degradada
requiere validación del operador.


Validación final local: build de 16 paquetes, 1375 pruebas ROS sin fallos (3
omitidas), 16 pruebas de herramientas sin fallos (5 nuevas del relay), selftest
del harness y smoke web-cockpit correctos. La validación del árbol versionado se
realiza por separado para conservar intacta la carpeta .agents/ no versionada.
