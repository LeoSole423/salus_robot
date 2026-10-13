# ADR 0011: Bisectriz automática de yaw para rutas

- Estado: aceptada como default tras validación de Cockpit en simulación; robot pendiente
- Fecha: 2026-09-23

## Contexto

El Cockpit legacy resolvía los waypoints sin yaw manual usando la bisectriz
entre rumbos de entrada y salida antes de enviarlos. El gateway nuevo envía
`NaN` para conservar la distinción entre yaw automático y manual. El executor
actual usa el rumbo de salida y modifica el terminal de cada chunk al rumbo
de entrada. En las curvas de `PatrullaSencillaPolo` esto cambia el yaw pedido
a Nav2 en unos 40–50 grados.

El mismo servicio `SetRouteMissionLL` lo consume Patrol/HOME. La primera
implementación mantuvo su política anterior por compatibilidad, pero el
operador pidió después extender la bisectriz automática a todas las rutas.

## Decisión

- Añadir `auto_yaw_policy` al request de `SetRouteMissionLL`.
- Cadena vacía y `route_tangent` calculan el promedio circular de entrada y
  salida en checkpoints automáticos y preservan ese yaw durante el dispatch.
  `legacy` solicita explícitamente el cálculo anterior.
- El gateway de rutas de Cockpit envía `route_tangent`; Patrol/HOME deja el
  campo vacío y recibe la misma política nueva. Los yaws finitos manuales
  prevalecen en ambas políticas.
- Para el primer checkpoint automático intermedio de un pedido finito, la
  bisectriz se compara con el rumbo geométrico robot→checkpoint. Si difiere
  más de 60°, se usa ese rumbo de aproximación; de lo contrario se conserva
  la bisectriz. Esta referencia geométrica se usa sólo si la aproximación
  difiere como máximo 60° de la orientación actual del robot; una entrada
  lateral más cerrada mantiene la referencia anterior al rumbo del robot.
  Un singleton mantiene también esa referencia. Sin una aproximación
  geométrica válida se mantiene el fallback existente a esa orientación.
  Los checkpoints siguientes conservan la
  bisectriz. Esto conserva la protección de una entrada muy cerrada sin fijar
  el rumbo del robot previo a la curva en una pose intermedia futura.
- Los puntos sintéticos de la política nueva usan el rumbo de su tramo.
- Un valor desconocido se rechaza antes de sustituir una misión activa.

## Compatibilidad y validación

El campo modifica el tipo ROS compilado: productores y consumidores deben
actualizarse juntos. El WebSocket de Cockpit no cambia. La ruta guardada se
reprodujo en simulación y se revisaron `/plan`, odometría y autointersecciones.
El cambio del significado de la cadena vacía es deliberado y requiere smoke
de Patrol/HOME antes de fusionar. La validación en robot queda fuera de este
corte.

## Resultado de la simulación georreferenciada

El operador observó curvas claramente mejores en `PatrullaSencillaPolo`, por
lo que la política se conserva para las rutas de Cockpit. Se capturó un rulo
en el plan 12→13: el sintético a 35 m quedaba sólo 1,9 m antes del checkpoint
y sus yaws diferían unos 19,6°. Nav2 dibujó una autointersección para alcanzar
la orientación terminal, aunque el controlador completó el checkpoint por
posición antes de recorrerla. Se evita insertar un sintético cuando deja
menos de medio espaciamiento hasta el siguiente checkpoint. La bisectriz del
checkpoint y los yaws explícitos no se sustituyen.

La misma sesión mostró el fallo separado #309: un checkpoint terminal fuera
del costmap global móvil. La decisión para ese caso se documenta en ADR 0012.

## Corrección causal del primer checkpoint intermedio (13/10/2026)

El segundo rulo real del 24/09 mostró una bisectriz de −73,03° sustituida
por el rumbo de despacho −11,88° porque su diferencia superaba 60°. Un replan
posterior debía llegar a ese heading fijo cuando el robot ya entraba en la
curva. La reproducción con Smac Humble produce el rodeo incluso con costmap
vacío. Comparar la bisectriz con el rumbo de aproximación conserva −73,03°
y evita el rodeo en ese caso. La aproximación del caso difiere unos 39° del
rumbo inicial; el control de entrada lateral de 90° conserva el comportamiento
anterior. No cambia el umbral, el radio, los yaws explícitos,
la política legacy ni las garantías de checkpoints. Ver
`docs/migration-evidence/intent/loop-corner-replan-heading-2026-10-13.md`.
