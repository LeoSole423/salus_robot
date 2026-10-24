# Biblioteca compartida de rutas del robot

Fecha: 2026-10-24. Owner: `salus_web`; consumidor: módulo de navegación de Cockpit.

## Intención y caracterización

Cockpit guardaba las rutas con nombre en `cockpit.navigation.routes.v1` de cada
PC. `saveNamedRoute`, `loadNamedRoute` y `deleteNamedRoute` eran síncronos y sólo
modificaban localStorage. El bridge ya ofrecía `save_waypoints_file` y
`load_waypoints_file`, pero representaban un único documento, sin catálogo,
nombres ni control de concurrencia. `patrol_store` guarda una definición de
misión para recuperación; no es la biblioteca del operador.

El operador solicita que todos compartan las rutas del robot. Decisiones:
importación seleccionable, conservación de copias locales y administración por
todos los operadores conectados. El lease de conducción sigue independiente.

## Contrato

WebSocket, respuestas `ack` con el identificador correlacionado habitual:

- `list_routes` → `routes: [{id, name, revision}]`.
- `get_route {id}` → `route: {id, name, revision, document}`.
- `save_route {name, document}` crea; `{id, expected_revision, name, document}`
  actualiza. Devuelve `route` confirmado en disco.
- `delete_route {id, expected_revision}` elimina.
- `routes_changed` se difunde a todos los clientes normales tras guardar o
  eliminar; los clientes vuelven a consultar el catálogo.

`document` conserva el formato del editor: `waypoints` (`x` latitud, `y`
longitud, `localId`, orientación opcional `yawDeg`, rol y acciones),
`patrolMissionProfileRefs` por índices y `loopRoute`. Hasta 200 puntos y 512000
bytes por documento. Nombres recortados de 1–120 caracteres, únicos y sensibles
a mayúsculas. ID UUID estable; revisión entera creciente desde 1. Una creación
nunca sobrescribe. Conflictos: `ROUTE_NAME_EXISTS`, `ROUTE_CONFLICT`,
`ROUTE_NOT_FOUND`; validación: `INVALID_ROUTE`; E/S: `STORAGE_FAILURE`.
No se añaden interfaces ROS ni se inicia o modifica una misión.

Archivo JSON `schema_version: 1`, bloqueo entre procesos, escritura temporal,
fsync, replace y fsync de directorio. Un documento inválido o un fallo previo
al replace conserva el anterior. No se reemplaza una biblioteca corrupta.

Parámetro `routes_file`: string, default vacío, sin unidad; vacío deriva
`routes.json` junto a `waypoints_file`, de otro modo ruta de archivo escribible.
En real `web_routes_file` default `/ros2_ws/state/web/routes.json`.
`SALUS_ROBOT_STATE_DIR` es absoluto y se monta en `/ros2_ws/state`; por defecto
`${XDG_DATA_HOME:-$HOME/.local/share}/salus_robot`, independiente de caché e imagen.
La simulación conserva almacenamiento aislado según su directorio de waypoints.

## Cliente y fallos

Catálogo remoto al conectar/reconectar y tras eventos; sin fallback local.
Guardado confirmado antes de cambiar baseline; conservar ediciones que ocurren
mientras llega la respuesta. Un conflicto conserva el borrador y exige recargar
o guardar con otro nombre. Cambiar conexión descarta catálogo, identidad de
ruta y respuestas pendientes anteriores, manteniendo los puntos como borrador.
La importación seleccionada informa resultados individuales y deselecciona las
confirmadas; nombres repetidos requieren renombrar. La sesión de destino queda
fijada durante el lote. Exportación/importación JSON permite respaldos.

## Evidencia y límites

Tests: `test_route_repository.py`, escenario multi-cliente en
`test_websocket_server.py`, `sharedRouteLibrary.test.ts` en Cockpit y smoke
`smoke_operational_persistence.sh` con dos clientes y reinicio del bridge.
Inspección de sólo lectura: servicio real activo en `/opt/salus_robot`, sin
montaje estable `/ros2_ws/state` al comenzar la tarea. La nueva configuración
requiere actualizar el wrapper antes de desplegar; no se declara despliegue
ni validación de navegación real por pruebas locales.

Validación local del 2026-10-24:

- Build Humble: 16 paquetes completos.
- Suite ROS: 1319 tests, 0 errores, 0 fallos, 1 omitido tras repetir
  `salus_hardware`. La primera pasada tuvo un fallo temporal en
  `test_stale_corrections_are_reported_before_reconnect`; el test aislado y el
  paquete completo pasaron sin cambios de implementación.
- Tests host de runtime/despliegue: 37 correctos; self-test del harness correcto.
- Cockpit: build correcto; 220 pruebas correctas y 1 omitida.
- Smoke de persistencia: dos clientes y reinicio correctos, mismo ID de ruta
  antes/después. Artefactos locales:
  `artifacts/smokes/operational-persistence-20261024T164811-1/`.
- `tools/test.sh` se detiene en la validación documental por enlaces y ejemplos
  preexistentes del directorio local no versionado `.agents/`; no se modifica.
  Las etapas funcionales restantes se ejecutaron separadamente.
- Jetson: sólo inspección; sigue en `fd86559`, rama de despliegue existente,
  servicio activo. No se instalaron estos cambios ni se reinició el servicio.
