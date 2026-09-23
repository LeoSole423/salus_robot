# ADR 0011: Selección explícita de yaw automático para rutas de Cockpit

- Estado: propuesta para validación en simulación
- Fecha: 2026-09-23

## Contexto

El Cockpit legacy resolvía los waypoints sin yaw manual usando la bisectriz
entre rumbos de entrada y salida antes de enviarlos. El gateway nuevo envía
`NaN` para conservar la distinción entre yaw automático y manual. El executor
actual usa el rumbo de salida y modifica el terminal de cada chunk al rumbo
de entrada. En las curvas de `PatrullaSencillaPolo` esto cambia el yaw pedido
a Nav2 en unos 40–50 grados.

El mismo servicio `SetRouteMissionLL` lo consume Patrol/HOME. Cambiar su
política por defecto alteró el smoke de retorno de batería, por lo que la
selección debe ser por request y conservar el comportamiento existente.

## Decisión propuesta

- Añadir `auto_yaw_policy` al request de `SetRouteMissionLL`.
- Cadena vacía conserva la política anterior para callers existentes;
  `route_tangent` calcula el promedio circular de entrada y salida en
  checkpoints automáticos y preserva ese yaw durante el dispatch.
- El gateway de rutas de Cockpit envía `route_tangent`. Patrol/HOME deja el
  campo vacío. Los yaws finitos manuales prevalecen en ambas políticas.
- Los puntos sintéticos de la política nueva usan el rumbo de su tramo.
- Un valor desconocido se rechaza antes de sustituir una misión activa.

## Compatibilidad y validación

El campo es aditivo en semántica, pero modifica el tipo ROS compilado: los
productores y consumidores deben actualizarse juntos. El WebSocket de
Cockpit no cambia. La rama se mantiene sin merge hasta reproducir la ruta
guardada en simulación y revisar `/plan`, odometría y autointersecciones.
La validación en robot queda fuera de este corte.
