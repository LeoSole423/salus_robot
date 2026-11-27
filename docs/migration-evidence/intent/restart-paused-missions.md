# Nuevo inicio desde una pausa

La Jetson pasa la ruta a PAUSED cuando se toma el control manual y cancela
el objetivo activo. El coordinador conserva una patrulla pausada como
misión pendiente. Este contrato se confirmó por lectura del checkout
desplegado 8fd5ee9. Cockpit agrupaba la pausa con la ejecución y ocultaba
el botón de inicio.

El operador puede enviar una nueva misión explícitamente desde la pausa.
La conversión conserva la misión anterior hasta aceptar el reemplazo.
El coordinador de patrulla publica el resultado correlacionado con la
solicitud. Cancelar invalida la preparación pendiente, incluidos los
resultados tardíos de la cancelación del tramo anterior. No se añade
reanudación de un tramo.

Validación de hardware pendiente. El cambio de interfaces ROS requiere
recompilar y actualizar el backend junto a Cockpit.

## Verificación en simulación

- `SMOKE_ROUTE_SCENARIO=restart ./tools/smoke_route_executor_sim.sh` verificó
  que la ruta reemplaza su ID tras la pausa manual y que se puede cancelar.
- `SMOKE_PATROL_RESTART_ONLY=1 ./tools/smoke_patrol_battery_sim.sh` verificó
  el ID de preparación de patrulla, reemplazo desde la pausa y cancelación
  durante la preparación.
- La primera ejecución del escenario de batería completo quedó en JOIN_LOOP.
  El ejecutor informó `no_near_segment` al seleccionar la entrada al bucle.
