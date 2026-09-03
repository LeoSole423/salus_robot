# Ficha de intención: contratos, observación y backend Pixhawk RTK/GNSS

## Alcance

Esta migración introduce contratos canónicos y observación RTK/GNSS sin cambiar
por defecto la cadena activa del robot. Incluye políticas puras para CRC24Q,
secuencia, antigüedad, contadores, calidad GNSS y adquisición; un observador
read-only del stack legado; un consumidor dry-run; un backend Pixhawk con doble
opt-in y una proyección web typed-first. El string RTK legado se conserva sólo
durante la migración y no es fuente de verdad física.

No inicia un cliente NTRIP, MAVROS/FCU, un driver `direct_usb`, UART, TF global,
movimiento ni actuadores. La entrega está deshabilitada por defecto y no crea
un publicador MAVROS en ese estado. Tampoco registra credenciales, rutas de
configuración o payloads RTCM en estados, diagnósticos o logs.

## Hechos caracterizados

- El stack legado expone estado NTRIP en JSON, RTCM en `/rtcm` como
  `std_msgs/UInt8MultiArray` y estado de fix textual.
- La observación previa indoor confirmó que la recepción y antigüedad RTCM no
  prueban una solución RTK: hubo correcciones recientes junto a fix autónomo o
  sin fix GNSS.
- El grafo legado acumuló más de un tipo sobre `/rtcm`; esta ambigüedad impide
  discovery, tooling y replay inequívocos.

## Contratos y reglas

| Frontera | Contrato/regla | Límite de este corte |
| --- | --- | --- |
| transporte RTCM | `RtcmFrame` lleva header, fuente lógica, secuencia y bytes ya validados | no replica payload fuera del transporte; MAVROS admite hasta 720 bytes |
| estado canónico | `GnssRtkStatus` separa fix GNSS, adquisición, frescura/contadores RTCM, backend y entrega | no deduce fix a partir de recepción RTCM |
| calidad GNSS | constantes `GPSRAW`: 5 → `RTK_FLOAT`, 6 → `RTK_FIXED` | sólo telemetría GNSS/MAVLink tiene autoridad de calidad |
| entrega Pixhawk | doble opt-in; `delivery_enabled=false` no crea publicador MAVROS | no habilitar junto al bridge legado |
| backend USB | declarado, no implementado y rechazado explícitamente | nunca hace fallback |
| observación legacy | un único tipo legado configurable para `/rtcm` | no se suscribe simultáneamente a los tres tipos históricos |
| dry-run | receptor canónico contabiliza frames y antigüedad | nunca publica `mavros_msgs/RTCM` |
| operación web | estado RTK tipado es preferido, con string legado temporal | no extiende Cockpit en este repositorio |

Los perfiles `pixhawk_mavros`, `direct_usb` y `disabled` se eligen de forma
explícita. No existe fallback automático por fallo, falta de datos o cambio de
frescura.

## Invariantes

- Frescura RTCM, adquisición y entrega son dimensiones distintas de
  `fix_quality`; RTCM reciente nunca crea por sí solo `RTK_FLOAT` ni
  `RTK_FIXED`.
- Frames vacíos, sobredimensionados, malformados o con CRC24Q inválido se
  rechazan y contabilizan. Las regresiones o reinicios de secuencia se exponen
  como tales, sin fabricar un estado físico.
- El launch parcial permanece namespaced y seguro por defecto. Hay exactamente
  una autoridad del estado canónico por perfil y no crea control, TF, NTRIP,
  MAVROS/FCU ni UART.
- `source_id` y `status_detail` son identificadores/detalle sanitizado, nunca
  secretos, rutas locales o bytes RTCM.

## Evidencia y límites

La convivencia read-only se validó en exterior con el robot estacionario. El
contrato observó `GPSRAW.fix_type=6` (`RTK_FIXED`), 32 satélites y correcciones
frescas. El perfil `pixhawk_mavros` con `delivery_enabled=false` mantuvo estado
de entrega `IDLE`, un único publicador del estado canónico y ningún publicador
MAVROS adicional; el bridge legado siguió siendo el único. El perfil real
aislado `pixhawk_rtk_delivery_real.launch.py` fija los endpoints físicos
observados y fue probado con un runtime ROS sintético: un frame válido se
entrega una vez como `mavros_msgs/RTCM`, mientras duplicados, frames inválidos y
payloads sobredimensionados no se entregan; la calidad sigue dependiendo de
GPSRAW y se vuelve `UNKNOWN` al quedar stale. Esto valida la composición y la
política software, no la entrega física: habilitar el backend en el robot
requiere una prueba aislada tras detener el bridge legado. El componente aún
no está `parity_passed` ni tiene entrega `hardware_validated`.
