# Despliegue de paneo y luces — 2026-10-14

Operador autorizó despliegue con robot detenido y uso de sudo. No se almacenó
la credencial en archivos. Backend de96b87, rama agent/camera-sweep-lights,
activado en /opt/salus_robot. Cockpit local 906bab0, rama
agent/camera-sweep-lights-current, con controles compactos dentro de PTZ.

Candidato compilado de forma aislada mediante prepare_real_runtime.sh:
camera-20261014-source y camera-20261014-cache bajo salus_diagnostics en Jetson.
Se reutilizó la imagen validada y se importaron dependencias fijadas.
Servicio salus-robot-real.service reiniciado; quedó active. Presets y demás
log/runtime copiados del workspace anterior al candidato antes del arranque.
Wrapper, secretos, MediaMTX y perfiles locales conservados.

Respaldo: salus_diagnostics/camera-20261014-backup.
Scripts: camera-20261014-activate.sh y camera-20261014-rollback.sh.
Rollback requiere sudo y robot detenido, restaura rama y manifiesto previos.

Validación posterior: ROS /camara/camera_features responde ok=true,
supported=true, sweep_running=false, phase=idle, luces=-1.
WebSocket get_camera_features devuelve el mismo contrato sin errores:
artifacts/camera-deployment-20261014/web-features.json.
Cockpit conectado al robot reconoce controles; apagado y detener están disponibles,
inicio y encendido requieren tomar control. No se desbloqueó ni inició movimiento.
No se ejecutó paneo físico ni encendido mediante los botones durante despliegue.
Ambos métodos físicos de luz ya fueron caracterizados con el operador.
La lectura /cmd_vel_final previa no entregó muestra dentro del timeout;
no se presenta como prueba de salida cero.

Prueba física del paneo automático nuevo pendiente de aceptación del operador.

## Seguimiento: velocidad y apagado

El operador informó apagado tardío de luces y paneo lento. Las luces terminaron
apagadas; backend respondió fixed_light=0/mobile_light=0 antes del reinicio.
Prueba posterior desde ROS: pan 0→164.36→300.5, transición moving_left→sweeping_right,
lectura pan=77.65 y final idle por WebSocket. Es evidencia de posición reportada;
el operador no observó esa prueba físicamente. No se determinó la causa del
primer intento aparentemente inmóvil.

Actualización 7e343d7 desplegada en agent/camera-speed-update: paneo ONVIF a
velocidad normalizada 0.65, movimientos manuales mantienen 0.15.
Build incremental Jetson completado y servicio active; API sin errores, idle.
Tiempo y suavidad físicos del nuevo recorrido todavía pendientes de medición.
Cockpit 31c2389 conserva los errores de órdenes entre consultas periódicas.
Build y 201 pruebas passed/1 skipped; 15 pruebas focalizadas de backend/política.
Respaldo adicional: salus_diagnostics/camera-speed-20261014-backup.
No afirmar que la demora física de las luces está corregida: no se midió.

## Paneo continuo

aaa2e44 activado en Jetson, rama agent/camera-repeat-update. Iniciar alterna
izquierda/derecha repetidamente hasta detener, comando PTZ manual, fallo o cierre.
Cada tramo conserva deadline 120 s. No se ejecutó barrido físico continuo de
prueba tras esta actualización; API verificada idle, supported=true, sin error.
Cockpit 98eaeef muestra sentido izquierdo/derecho y abre video móvil.
Hardware 188 y web 73 pruebas sin fallos; Cockpit 201 passed/1 skipped, builds y
smoke web-cockpit correctos. Respaldo camera-repeat-20261014-backup en Jetson.
