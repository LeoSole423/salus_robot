# Historial operativo

## Evidencia de partida

El WebSocket de `ROS2_SALUS` guardaba sesiones JSONL en `/tmp/mission_sessions`.
Cockpit aún podía consumir ese contrato desde Diagnóstico, pero el gateway
actual lo declara fuera de soporte. La vista anterior infería la duración de
las filas visibles. Se conserva el patrón de línea de tiempo y detalle. El
historial operativo nuevo obtiene sus tiempos del robot.

El 26/11/2026 se inspeccionó en solo lectura la Jetson por SSH. El servicio
`salus-robot-real.service` estaba activo con `salus_robot@7bb6b8a`. Se verificó
la publicación de `/nav_command_server/events`, `/nav_command_server/telemetry`,
`/controller/drive_telemetry`, `/battery_mission_guard` y
`/system/capabilities`. La comprobación de tópicos establece presencia,
sin demostrar cada transición en hardware. El historial nuevo se prueba
con entradas controladas y en simulación antes de cualquier despliegue real.

## Señales y significado

| Señal | Uso |
| --- | --- |
| `NavEvent` en `/nav_command_server/events` | Inicio, fases y cierre de patrulla. Cambios manuales confirmados, órdenes de freno, watchdog, fallas de navegación y decisiones de retorno. Se conservan código y detalles aprobados. |
| `NavTelemetry` en `/nav_command_server/telemetry` | Entrada y salida de bloqueo por monitor de colisión y LiDAR vencido cuando el árbitro lo reporta. |
| `PathHealth` en `/path_health` | Entrada a espera, cambios de motivo y recuperación con camino válido. Suscripción del gateway con QoS confiable y profundidad 10. La espera inicial de un objetivo es informativa. |
| `DriveTelemetry` en `/controller/drive_telemetry` | Cambios de frescura. Con datos frescos, autoridad RC, freno aplicado y señal de parada reportada. Con velocidad válida, detención medida al pasar del estado en movimiento (≥0,10 m/s) a ≤0,05 m/s. El motivo requiere una señal distinta. |
| `BatteryMissionGuard` en `/battery_mission_guard` | Recomendación y recuperación de batería cuando el guard está listo y fresco. |
| `SystemCapabilities` en `/system/capabilities` | Cambios explícitos de disponibilidad, validez y recuperación para capacidades habilitadas. QoS existente confiable y transient local. |
| Lease del gateway | Adquisición y liberación del permiso de operación, sin identidad del usuario. Independiente de la confirmación del modo manual. |

Se reutilizan las suscripciones del gateway con sus QoS existentes. Un evento
coincidente con la ejecución aporta contexto. Solo sus productores autorizan
atribuir un motivo. La historia no ingiere ROS logs, telemetría continua ni
video. Un robot desconectado puede dejar de aportar señales. Los incidentes
sin ejecución asociada siguen disponibles en Actividad.

## Persistencia y tiempos

El gateway es dueño del registro y su escritura SQLite en segundo plano. El
reloj UTC del robot identifica la recepción. Para `NavEvent` se conserva el
timestamp ROS original como `ros_stamp_ns`. El reloj monotónico mide duración
total y tiempo en fase `PATROL`, sin contar las fases de salida, regreso ni
pausas del coordinador como tiempo de circuito. Una detención puntual no
cambia por sí sola la fase. El tiempo en circuito describe la fase operativa,
no el tiempo con las ruedas girando. Se consolidan tiempos en cada evento y
cada cinco segundos. Una ejecución abierta al reiniciar queda interrumpida,
con los tiempos observados y sin hora de fin. Las consultas pueden devolver
historial degradado si se pierden escrituras. Revisar el almacenamiento y
reiniciar el gateway una vez recuperado. El historial rige desde su activación.

`history_path` es texto vacío para desactivar, o una ruta de base SQLite local
escribible. El gateway aislado parte desactivado. Real MVP usa
`/ros2_ws/state/web/operational-history.sqlite`; el perfil operativo simulado
usa el directorio de runtime del perfil. `history_robot` es la etiqueta
estable de la instancia y vale `salus` por defecto. La base crece con los
cambios registrados. Incluirla en el respaldo del estado del robot con una
copia SQLite coherente.

## API y Cockpit

- `history_list` ofrece `view` (`activity` o `patrols`), límites opcionales
  `start` inclusivo y `end` exclusivo en ISO 8601 con zona, `category` para
  actividad y cursor `before`. Devuelve hasta 50 filas en orden descendente.
- `history_patrol` devuelve resumen y hasta 100 eventos ordenados. El primer
  resultado fija `through`. Las consultas siguientes reutilizan ese límite y
  usan `after` con el valor de `next`.
- Ambas operaciones usan la sesión WebSocket y se pueden consultar sin
  permiso de conducción. Cockpit descarga JSONL UTF-8 de una ejecución,
  encabezado `type=patrol`, `schema_version=1`, seguido por registros
  `type=event`. Los detalles se restringen a una lista explícita.
- Validación local con pruebas de transiciones, reinicio, paginación y
  almacenamiento degradado. El smoke web comprueba persistencia de cambios
  manuales y lectura desde un cliente sin el lease. Quedan pendientes la
  aceptación con operador y la validación de señales durante operación real.

## Resultado local

Pasaron la suite de `salus_robot` y el smoke `web-cockpit`. Una reproducción
aislada publicó eventos ROS de cambio a manual, inicio, entrada al circuito,
freno, pausa con motivo, cancelación y fallo posterior fuera de patrulla.
El gateway devolvió la secuencia y ambos tiempos a través de WebSocket.

La prueba con Gazebo y retorno por batería confirmó el arranque del gateway
pero no completó la patrulla. El ejecutor rechazó el anclaje con
`no_near_segment` y el escenario agotó la espera en `JOIN_LOOP`. El mismo
síntoma se registró anteriormente en
[maneuver-guard-2026-10-17.md](../maneuver-guard-2026-10-17.md).

Las rutas en bucle lanzadas directamente también aparecen en Patrullas. El
ejecutor emite inicio tras activar la ruta y registra pausa, cancelación,
reemplazo o finalización. Para los tramos internos, el coordinador pasa
`parent_patrol_id` en `SetRouteMissionLL` y conserva la única ejecución del
historial. El cliente ROS y el servidor deben usar la misma versión del servicio.

Se verificó en una simulación aislada: la ruta de 51 puntos de
Patrulla Sencilla Polo apareció al activarse y quedó cancelada al cerrar.

El backend simulado levanta la señal de parada cuando aplica freno. El
historial adjunta la señal a la misma fila de freno cuando ambos aparecen
juntos. Si la señal aparece sin freno aplicado se registra por separado.
La señal por sí sola no identifica el accionamiento de un pulsador físico.
Las órdenes de freno manual y la detención medida se registran por separado.
