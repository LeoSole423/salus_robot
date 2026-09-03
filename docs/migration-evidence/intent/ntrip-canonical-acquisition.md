# Ficha de intención: adquisición NTRIP canónica (#178)

## Alcance

Este corte hace que `salus_hardware` sea owner de la adquisición NTRIP propia,
sin entregar RTCM a MAVROS/Pixhawk. El caster local se transforma en frames
RTCM3 completos y CRC24Q válidos; la calidad GNSS continúa siendo desconocida
porque sólo telemetría del receptor puede producir `RTK_FLOAT`/`RTK_FIXED`.

## Contrato

| Frontera | Contrato |
| --- | --- |
| caster → fuente | HTTP/1.0-1.1 o `ICY 200 OK`, Basic auth, chunked opcional |
| fuente → ROS | `salus_interfaces/RtcmFrame` en `/salus/hardware/rtcm/corrections` |
| estado | `GnssRtkStatus` en `/salus/hardware/gnss_primary/rtk_source_status`, sólo adquisición |
| configuración | YAML local `rtk_sources.local.yaml`, ignorado y read-only durante runtime |

Los timeouts son connect 5 s, read 2 s, reconnect inicial 2 s con exponential
backoff hasta 60 s y stale 10 s. `RtcmFrame.sequence` comienza en 1 y es
monótona por proceso/fuente. No se registran headers, Basic auth, payload,
password, mountpoint, host ni ruta de configuración.

## Evidencia

Las pruebas unitarias cubren headers fragmentados, ICY, chunked, sourcetable,
texto, autenticación/respuestas inválidas, resync/CRC y configuración
sanitizada. Un fake caster TCP local entrega un frame válido y uno corrupto:
el primero produce exactamente un `RtcmFrame`; el segundo sólo incrementa el
contador CRC. No se usó Jetson, caster real, MAVROS, Pixhawk, RS16, UART,
heading, Cockpit ni source management.

## Estado y límites

La adquisición queda implementada y validada offline/localmente. La entrega al
Pixhawk, la calidad de solución informada por GPSRAW y la validación física son
trabajo posterior (#166/B3b); `real_observation.launch.py` permanece sin iniciar
NTRIP.
