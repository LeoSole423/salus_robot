# Recuperación de Cockpit tras arranque — 2026-11-11

El operador reportó Error de conexión desde el sitio público. Inspección:
servicio ROS activo, detector accesible en 8791, pero sin listener en 8766.
Docker conservó el traceback de arranque, que journal había omitido en la
consulta inicial: Recordings.start falló con errno 99/EADDRNOTAVAIL al enlazar
el proxy privado 8790 antes de que existiera la IP Tailscale. Esto hizo salir
web_bridge, aunque navegación continuó. No fue un fallo HTTPS del navegador.

Se arrancó sólo web_bridge, conservando rutas persistentes, grabaciones,
fuentes GPS/heading, NTRIP y bloqueo de operador. La página pública recuperó
Conectado · Robot real y telemetría. Esa primera recuperación utilizó el código
anterior; luego se sustituyó sólo el proceso web por el código corregido.
No se desbloqueó control ni se inició movimiento.

## Corrección

Recordings.start no impide el arranque del gateway cuando falta temporalmente
la dirección privada. Reintenta en el worker existente cada dos segundos, con
la misma IP/puerto y sin fallback a wildcard. Otros errores iniciales de bind,
incluido EADDRINUSE, siguen siendo errores explícitos. Cerrar cancela reintentos
y libera un proxy recuperado. No cambia firmas, autenticación, lease ni
parámetros ROS. La navegación no depende de esperar Internet/Tailscale.

Fuente canónica: commit `8b66cc3`, rama agent/web-recording-bind-recovery.
En Jetson se verificó primero que recordings.py coincidía con la fuente base.
Se aplicó únicamente ese archivo sobre el runtime e90bc79, en la nueva rama
`agent/web-bind-recovery-deployed-20261111`, commit `ef05621`. Checkout limpio.
La preparación incremental conservó imagen 3601eeb4be47 y dependencias, y
actualizó el manifest preparado a ef05621 para el siguiente arranque.
No se reemplazó el runtime por todo main ni se reinició el servicio real.

Para esta sesión se ejecuta un único launch parcial web desacoplado dentro del
contenedor existente (docker exec -d), con log `web-recovery-20261111.log` en el
directorio log montado. Tras el siguiente reinicio normal, el launch completo
volverá a iniciar ese mismo bridge con la corrección y manifest preparados.
No se añadió un segundo servicio permanente que compita por el puerto.

## Evidencia

- Build: 16 paquetes; suite ROS: 1377 pruebas sin fallos, tres omitidas.
- Pruebas focalizadas: 14 correctas, incluyendo interfaz ausente/recuperada,
  cierre, conflicto de puerto y rechazo de descarga sin firma.
- Smoke web-cockpit correcto y validación del árbol Git limpio.
- Probe aislado en la imagen real: dos fallos de bind inyectados y tercer
  intento correcto; descarga sin firma 403 y worker detenido al cerrar. Usa
  directorio temporal, tick simulado y puerto loopback efímero: no toca
  grabaciones reales ni cambia la interfaz de red.
- Listeners finales 8766 y 8790 presentes, un solo bridge, conexión de Chrome
  al sitio público y telemetría fresca. Detector: tres cámaras vigilando.
- PIDs preservados: ROS 3712, MediaMTX 1952, detector 32178; NRestarts cero.

La caché quedó preparada para reinicio, pero no se realizó un segundo reboot
físico para reproducir la carrera; la recuperación se validó por inyección en
la implementación instalada. La validación anterior del panel de IA local no
había incluido conexión completa de control tras el reboot: esa omisión fue
reconocida y se comprobó ahora desde la página pública.

## Respaldo y rollback

En HOME de admin, directorio privado `camera-diagnostics/web-recovery-20261111/`:
`recordings-before.py` y `prepared-before.env`. Se conservó la rama anterior.
Rollback de código: regresar al commit e90bc79 y preparar la caché usando la
misma imagen validada, dentro de una ventana de mantenimiento. No sobrescribir
la configuración privada ni los datos persistentes. El código anterior vuelve
a ser vulnerable a la IP tardía; no es necesario retirar la corrección para
reiniciar sólo el bridge. No terminar PID 1 del contenedor ni el servicio ROS
para recuperar únicamente Cockpit.
