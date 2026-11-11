# Detección de personas y recuperación de GPU — 2026-11-11

## Resultado y alcance

Implementación nueva en ramas `agent/person-watch` de salus_robot y Cockpit,
creadas desde main (`e48e0af` y `aa7daeb`). No se fusionaron ni publicaron a main.
El detector sí está instalado en Jetson como servicio independiente habilitado.
La interfaz se comprobó en Cockpit local; el sitio público no se desplegó.
Código instalado de salus_robot: `1969716`; Cockpit: `1004cbc`. El manifest
`deployment.json` del detector guarda el SHA de fuente y los hashes de los tres
archivos ejecutables, comparados con el checkout local después del despliegue.

El operador pidió alertas exclusivamente en Cockpit y después confirmó que el
robot estaba detenido y disponible para mantenimiento, incluyendo posibles
cambios del sistema y reinicio. No se iniciaron misiones ni movimientos.
La implementación no publica comandos ROS ni modifica parámetros de navegación.

## Reparación previa de GPU

Orin Nano Developer Kit de 8 GB; Ubuntu 24.04, L4T R39.2.0 y kernel
6.8.12-1021-tegra. Antes: CUDA status 100, ausencia de dispositivos GPU,
`nvidia-smi` sin conexión y fallos de servicios NVIDIA de display/potencia/fan.
La entrada `JetsonIO` de extlinux apuntaba al DTB base de 142278 bytes, sin nodo
GPU. Su SHA-256 coincidía exactamente con el DTB base instalado: no tenía
modificaciones UART personalizadas que trasladar.

Se respaldó extlinux y se cambió únicamente la línea FDT:

- anterior: `/boot/dtb/tegra234-p3768-0000+p3767-0005.dtb`;
- efectiva: `/boot/tegra234-p3768-0000+p3767-0005-nv.dtb`.

Respaldo y comparación: `camera-diagnostics/gpu-repair-20261111/` relativo a
HOME de admin, directorio privado. Se reinició la Jetson dentro de la ventana
autorizada. Después aparecen dispositivos GPU, `nvidia-smi` identifica
Orin/nvgpu 595.78, CUDA compatible 13.2, y el ventilador vuelve a estar activo.
No se actualizaron kernel, drivers, imagen ROS ni paquetes JetPack.

El servicio real volvió activo con PID 3712; MediaMTX con PID 1952. Ambos
conservaron esos PIDs durante la instalación/reinicios del detector. Lifecycle
de controller_server y collision_monitor activos. `/scan_clean` observado
aproximadamente a 9.5–10.8 Hz durante una lectura corta con IA activa.
Esto no prueba ausencia de interferencia durante navegación física.

## Diseño finalmente instalado

[ADR 0017](../decisions/0017-advisory-person-watch.md) y
[manual del servicio](../../deploy/person_watch/README.md).

- GStreamer NVIDIA H264 decode para cam3/cam4/cam5, RTSP/TCP local de Ahorro.
- 0.5 FPS por cámara; un slot de imagen reciente por cámara, sin cola histórica.
- YOLOX Nano 416×416, NCNN Vulkan en la GPU; preparación/NMS en CPU.
- Pesos y operaciones FP32; no PyTorch, TensorRT ni DeepStream en runtime.
- Umbral 0.55; dos positivos consecutivos, gap máximo 6 s y cooldown 30 s/cámara.
- Pausa de inferencia con menos de 700 MiB disponibles o temperatura ≥82°C.
- systemd Nice 15, CPUWeight 10, CPUQuota 50% de un core, MemoryHigh 512 MiB,
  MemoryMax 768 MiB. La GPU no tiene cuota dura; la cadencia limita su trabajo.
- API sólo lectura, sin imágenes ni credenciales, ligada a Tailscale puerto 8791.
  Acceso de red del tailnet; CORS limita los orígenes del navegador, no autentica.
- Indicador permanente, panel y avisos con cámara/hora en Cockpit. Errores y
  muestras vencidas muestran vigilancia no disponible, no ausencia de personas.

Inicialmente se midió CPU (~215 ms/imagen). TensorRT se consideró después,
pero su paquete de 2.3 GB no aporta ventajas necesarias para esta cadencia.
Se interrumpió la descarga antes de instalar paquetes; se eliminó únicamente
su parcial de 480865815 bytes. El paquete NCNN oficial ocupa unos 5 MB.
No quedó una instalación de TensorRT en el host.

## Archivos desplegados

Base relativa a HOME de admin: `.local/share/salus-person-watch/`.

- `person_watch.py`, `gstreamer_reader.py`, `ncnn_backend.py`;
- `venv/`: Python 3.12 aislado, acceso a bindings GStreamer del sistema;
- `config.json`, modo 0600; fuentes loopback sin credenciales;
- `yolox_nano.onnx`: original oficial, checksum verificado;
- `salus_folded.ncnn.param` y `.bin`: conversión offline, checksums fijados;
- unidad `/etc/systemd/system/salus-person-watch.service`, habilitada al arranque.

No se requieren los FFmpeg privados existentes ni modificaciones de MediaMTX.
El monitoreo mantiene abiertos los tres streams de ahorro, aunque el operador
pause una vista o elija Normal. Añade tráfico de LAN cámara→Jetson, no sube video
a una nube. Historia de hasta 50 avisos en memoria; reiniciar la borra.

La conversión Focus→Conv6 es matemáticamente equivalente y tiene prueba de
bordes/interior. Comparación de ONNX: error absoluto máximo 0.0001631 y medio
1.56e-7. La prueba GPU final con pesos FP32 reproduce las tres detecciones y
confianzas del original CPU en una imagen pública de prueba; imagen uniforme:
cero detecciones. Los hashes de pesos finales están en `ncnn_backend.py`.
La variante previa con pesos FP16 mostró cambios de confianza y se sustituyó
por FP32; no es la versión final.

## Evidencia y límites

Build de 16 paquetes; 1375 pruebas ROS correctas, tres omitidas. Herramientas:
seis pruebas nuevas de política/conversión, más heading y preparación MediaMTX.
Smoke web y harness correctos. Cockpit: build y 239 pruebas correctas, una
omitida. Árbol Git limpio validado sin modificar `.agents/` local no versionado.

Los lectores hardware entregaron 9/8/8 frames frontal/móvil/trasera en 20 s,
incluyendo arranque. El servicio mostró las tres cámaras vigilando. Reinicios
controlados sólo del detector recuperaron las cámaras y el panel.

Evidencia local en `artifacts/person-watch-20261111/` de la raíz SALUS: fixtures,
comparación numérica, lectores, muestras de servicio, screenshot y chequeo ROS.
Medición final FP32: 11.61% de un core, máximo 162.25 MiB, mediana de inferencia 31.1 ms y máximo 38.0 ms; 20 consultas sin errores durante 38.07 s. Son muestras estacionarias, sin comparación causal de navegación antes/después. La evidencia JSON junto a este documento conserva estos resultados.

No se observó una persona real durante la muestra; la cadena de confirmación y
avisos tiene pruebas, pero falta aceptación de campo con personas delante de
cada cámara, distancia/noche/oclusiones, pérdidas prolongadas y patrulla física.
No se valida frenado ni se conecta este detector a la seguridad de movimiento.
Habilitación al arranque comprobada; reinicio completo con el nuevo detector
instalado todavía no ensayado. La reparación de GPU sí fue validada tras reboot.

## Consulta y rollback

```bash
systemctl status salus-person-watch.service
systemctl show salus-person-watch.service -p CPUUsageNSec -p MemoryCurrent -p NRestarts
curl -fsS http://100.66.15.45:8791/v1/state
journalctl -u salus-person-watch.service -n 30 --no-pager
```

Para retirar IA, ejecutar con privilegios administrativos:

```bash
systemctl disable --now salus-person-watch.service
```

No detener ROS ni MediaMTX. Cockpit indicará detector no disponible. Conservar
archivos/configuración/modelos para diagnóstico. Rollback del arranque GPU es
una operación distinta: restaurar sólo la referencia FDT del respaldo tras
revisar extlinux actual y usar otra ventana de reinicio; eso vuelve a deshabilitar
la GPU y no es necesario para retirar IA.

La publicación HTTPS de Cockpit necesita proxy del endpoint por el mismo
origen HTTPS y actualizar `person_watch_url`; no desactivar seguridad del
navegador ni abrir el puerto a Internet. No se publicó esa infraestructura.
