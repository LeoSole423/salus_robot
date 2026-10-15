# Camera sweep and steady lights

Intent: Cockpit commands continuous mobile pan between physical left and right,
preserving tilt, with an explicit stop. Separate steady light controls use physically verified mechanisms for
the fixed and mobile lenses.
MediaMTX remains the video owner; salus_hardware owns ONVIF/LAPI and secrets.

Evidence: Uniarch firmware DIPC-B6109.1.120.251218 accepts LightCtrl PUT,
Type=1/Mode=1 or 2/Interval=60; operator confirmed flashing and steady light
on general index 0 (2026-10-14). LightCtrl indices 1, 2 and singular Channel/2 all lit only the fixed lens.
Operator confirmed mobile white light only with LampCtrl/Video channel 2 Type=1
and Exposure/Video DayNight.Mode=2. Both complete original profiles were restored
and GET matched the backups after the test.
Existing ONVIF mapping spans pan 0–350 degrees, tilt 0–90. Cockpit's corrected
horizontal arrows make increasing pan physical left. Existing presets stay intact.

Design: additive CameraFeatures service, controlled WebSocket start/on commands,
unrestricted stop/off commands. A timer advances a bounded sweep and renews
60-second light orders every 40 seconds. Light state is last acknowledged intent,
unknown at startup/after failure: GET LightCtrl is not physical telemetry.
Simulation implements the same surface. Manual PT commands cancel an active
sweep. Tests never address real hardware.

Validation: Humble build 16 packages; full first-party colcon results 1308 tests,
0 errors/failures (one skipped); hardware 187, web 73; focused policy/ONVIF/
protocol/mapping 36 passed; Cockpit build and 201 passed/1 skipped; extended
WebSocket smoke passed, including state, sweep start/stop, lights and off/stop
without lease. Host runtime tests 36 passed; smoke harness selftest passed.

The standard tools/test.sh preflight encounters pre-existing untracked .agents
documentation violations; validation of the tracked tree plus this change passes.
Historical rslidar_msg results remain excluded as prescribed; its stale lint
results do not represent this run. No user files or old results were removed.

Physical light mechanisms confirmed; new automatic sweep acceptance and real
runtime activation remain pending. No robot motion authorized. Working source
is based on salus_robot c6bed23 and current Cockpit main fd69a8e, whose camera
grid, corrected PTZ and rear stream are already integrated. The older migration
branch diverges; an attempted integration on a new branch was aborted cleanly
and changes are isolated on agent/camera-sweep-lights-current, preserving main.


Actualización 2026-10-14: el paneo alterna derecha/izquierda continuamente
hasta Stop explícito, un comando PTZ manual, error o cierre del nodo.
Mantiene el timeout de 120 s por tramo; cada llegada renueva el plazo.
Cerrar la vista de Cockpit no detiene el recorrido. Velocidad normalizada
ONVIF 0.65 para paneo; flechas manuales conservan 0.15.


Corrección de fiabilidad 2026-10-14: operador informa paradas que requieren
reiniciar paneo. Estado real observado: sweep_phase=error, sweep_running=false,
error vacío con ambas luces activas. La renovación exitosa de luces borraba
el error compartido; no se puede recuperar la causa de esa parada anterior.
Separar errores por paneo y lente, registrar transiciones/errores en ROS.
La política anterior detenía ante una sola lectura GetStatus fallida. Ahora
permite dos fallos consecutivos de lectura sin emitir nuevas órdenes ni
renovar el deadline; la tercera lectura fallida detiene. Una lectura válida
reinicia el contador. Rechazos de movimiento y timeout de 120 s mantienen
parada inmediata. No reinicia automáticamente recorridos terminados.
Pruebas simulan fallos aislados/persistentes, errores de movimiento y renovación
de luces simultánea; no actúan sobre hardware. Causa física y aceptación
prolongada pendientes de nueva evidencia de runtime.

Validación de corrección: build de 16 paquetes; suite colcon de 15 paquetes,
1313 tests de fuentes propias, cero errores/fallos y un skip; salus_hardware
192 y salus_web 73. Smoke web-cockpit passed. Desplegado 40fd3a7 en Jetson
con operador confirmando robot detenido; servicio active, readiness real
ACTIVE y WebSocket CameraFeatures ok/supported, sweep idle y error vacío.
No se inició paneo físico durante esta verificación. Respaldo de fuente y
prepared.env disponible en el directorio de diagnóstico de la Jetson.


Recuperación solicitada por operador: esperar 5 segundos después de una parada
por fallo, verificar posición y reenviar el destino interrumpido. Máximo tres
intentos durante toda la sesión iniciada por el operador, sin resetear presupuesto
al completar un tramo. Nuevo Start explícito reinicia presupuesto. Estado
recovering mantiene sweep_running=true para que Stop y PTZ manual cancelen
la recuperación pendiente; shutdown también la cancela. Si Stop no se confirma,
no se programa recuperación. El tercer intento fallido o nuevo fallo tras agotar
presupuesto termina en error. No recupera por reinicio del nodo.

Validación recuperación: 16 pruebas puras de política, build 16 paquetes,
suite propia 1318 tests sin fallos/errores (1 skip), smoke web-cockpit passed.
Cockpit build y 205 passed/1 skipped. Backend 38d92d5 desplegado en Jetson,
servicio active y consulta WebSocket ok/supported/idle/error vacío. Cockpit
commits 6a152bc y 5a73fd5 en agent/camera-sweep-recovery; no merge a main.
Fallos de red inyectados sólo en mocks; recuperación física ante fallo real
queda pendiente de observación del operador. Ningún paneo se inició al desplegar.


Paneo sectorial solicitado: partir de Frente, ir 120 grados físicos a izquierda
(incremento pan) y 120 grados a derecha (decremento pan); alternar continuamente.
2026-10-14 operador corrigió referencia: antiguo Frente 0 no representaba frente,
HOME sí. Lectura real con cámara centrada pan=223.94999556, tilt=0, zoom=1;
se guardó Frente por servicio camera_save_preset sin modificar HOME. Límites
resultantes 103.94999556 y 343.94999556, dentro del rango ONVIF leído 0..350.
Start captura Frente guardado, centra primero y luego alterna izquierda/derecha;
conserva tilt y zoom de Frente. Un centro cuyos ±120 no entren en límites se
rechaza antes de mover, sin wrap por detrás. Recuperación conserva destino y
fase centering/moving_left/sweeping_*; cambios de preset no alteran sesión activa.

Validación sector: 17 pruebas de política, build 16 paquetes, suite propia
1319 tests (1 skip); primera ejecución halló estilo corregido y timeout en
prueba RTK ajena bajo smokes concurrentes; repetición aislada de salus_hardware
198 sin errores/fallos. Smoke web-cockpit passed tras calibrar Frente de fixture
simulada; Cockpit build y 205 passed/1 skipped. Backend 4fcd2ff desplegado,
servicio active y consulta CameraFeatures ok/supported/idle. Preset real Frente
223.94999556 guardado por servicio con respuesta ok; HOME intacto. UI 226b1ec
commiteada en agent/camera-sweep-recovery. Recorrido físico nuevo pendiente de
observación del operador; no iniciado automáticamente durante despliegue.
