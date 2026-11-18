# Aviso al rechazar el inicio de una ruta en bucle

El 18 de noviembre de 2026, la simulación rechazó tres envíos de una ruta en
bucle con `no_near_segment`. La aceptación de `SetRouteMissionLL` precedía a
la conversión de coordenadas y el resultado sólo quedaba en el diagnóstico
`ROUTE_ANCHOR_REJECTED`.

`SetRouteMissionLL` devuelve el identificador de preparación y
`GetRouteMissionState` expone el mismo identificador y la causa de rechazo.
El bridge transmite ambos campos por el canal existente. Cockpit compara
el identificador con el de su envío para evitar mostrar un rechazo previo
o de otro operador. La selección de entrada y la ejecución de rutas
mantienen sus criterios.

La verificación cubre la publicación de la causa al rechazar un anclaje y
la presentación para un envío coincidente en Cockpit. Los servicios ROS
modificados requieren recompilar y actualizar juntos sus consumidores.
El cambio se verifica en local y simulación.

Cockpit muestra «Preparando ruta…» desde el envío. El resultado se emite
por el canal existente de `NavEvent` y el estado consultable sigue exponiendo
los rechazos para clientes que no hayan recibido el evento.
