# ADR 0010: Transporte RTCM y calidad de solución GNSS ortogonales

## Estado

Aceptado para la migración incremental; backend MAVROS implementado opt-in y
validación de entrega física nueva pendiente.

## Contexto

El robot instalado obtiene correcciones mediante un cliente NTRIP legado,
publica frames como `std_msgs/UInt8MultiArray` en `/rtcm` y los reenvía al
Pixhawk mediante MAVROS. El bridge histórico llegó a suscribirse al mismo
tópico usando tres tipos ROS distintos. Además, estados textuales mezclan la
conectividad y frescura RTCM con la solución que informa el receptor.

La arquitectura futura debe conservar Pixhawk como backend válido y admitir un
receptor RTK conectado directamente por USB, sin que navegación, Web o Cockpit
dependan de MAVROS o de un protocolo de fabricante.

## Decisión

`RtcmFrame` es el contrato canónico de transporte de una trama RTCM3 validada.
Identifica fuente y secuencia; el productor valida preámbulo, longitud y
CRC-24Q antes de publicar. Los payloads no se copian a estados ni logs.

`GnssRtkStatus` representa dimensiones independientes:

- calidad de solución informada por el receptor;
- adquisición y frescura de correcciones;
- backend de entrega seleccionado explícitamente;
- estado observado de esa entrega.

Sólo telemetría del receptor puede producir `DGPS`, `RTK_FLOAT` o `RTK_FIXED`.
La recepción reciente de RTCM nunca eleva la calidad GNSS. Datos ausentes,
stale o reinicios de secuencia quedan explícitos y no habilitan fallback.

Los backends son `disabled`, `pixhawk_mavros` y `direct_usb`. Pixhawk no es una
compatibilidad temporal: continuará siendo una opción soportada. La selección
es de perfil y no cambia automáticamente. La entrega Pixhawk consume únicamente
`RtcmFrame` validado y requiere a la vez `delivery_backend=pixhawk_mavros` y
`delivery_enabled=true`; el segundo parámetro es `false` por defecto y, así, no
se crea un publicador MAVROS. `direct_usb` permanece declarado pero se rechaza
como no implementado, sin fallback.

En el perfil Pixhawk, `mavros_msgs/GPSRAW` es la única autoridad de calidad:
sus constantes oficiales 5 y 6 son `RTK_FLOAT` y `RTK_FIXED`. El observador
legado publica adquisición en un tópico intermedio y el adaptador Pixhawk es el
único publicador del estado canónico final. El launch no inicia MAVROS, FCU,
NTRIP, UART, TF ni control; sólo consume una instancia MAVROS ya existente.

Durante coexistencia, un adaptador puede consumir exactamente un tipo legacy
configurado para `/rtcm` y normalizar sus estados. No replica rutas de archivos,
credenciales ni errores que puedan contener secretos. El estado tipado es la
autoridad Web cuando está disponible; el string histórico queda como fallback.

## Consecuencias

- Se puede cambiar caster, receptor o ruta Pixhawk/USB sin cambiar consumidores.
- “Correcciones presentes” y “RTK Fixed” dejan de ser equivalentes.
- La entrega MAVROS está codificada pero permanece deshabilitada hasta una prueba
  aislada sin el bridge legado; el driver USB requiere un corte posterior.
- El launch parcial puede convivir con `ROS2_SALUS` porque sólo observa y
  publica tópicos namespaced nuevos.
