# Ficha de intención: contratos y observación RTK/GNSS

## Alcance

Este corte introduce contratos canónicos y observación RTK/GNSS sin cambiar la
cadena activa del robot. Incluye políticas puras para validación RTCM CRC24Q,
secuencia, antigüedad, contadores, calidad GNSS y adquisición; un observador
read-only del stack legado; un consumidor canónico en dry-run; un launch parcial
y una proyección web typed-first. El string RTK legado se conserva sólo durante
la migración y no es la fuente de verdad física.

No inicia un cliente NTRIP, entrega hacia MAVROS, un driver `direct_usb`, UART,
TF global, movimiento ni actuadores. Tampoco registra ni publica credenciales,
rutas de configuración o payloads RTCM en estado, diagnósticos o logs.

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
| transporte RTCM | `RtcmFrame` lleva header, fuente lógica, secuencia y bytes ya validados | no replica payload fuera del transporte ni reenvía al Pixhawk |
| estado canónico | `GnssRtkStatus` separa fix GNSS, adquisición, frescura/contadores RTCM, backend y entrega | no deduce fix a partir de recepción RTCM |
| calidad GNSS | MAVLink 5 se mapea a `RTK_FLOAT` y MAVLink 6 a `RTK_FIXED` | sólo telemetría GNSS/MAVLink tiene autoridad de calidad |
| observación legacy | un único tipo legado configurable para `/rtcm` | no se suscribe simultáneamente a los tres tipos históricos |
| entrega | receptor canónico dry-run contabiliza frames y antigüedad | no publica `mavros_msgs/RTCM`; `direct_usb` queda sin driver |
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
- El launch parcial permanece namespaced y seguro por defecto: sólo observa y
  publica sus contratos canónicos; no crea una autoridad de control, TF ni
  backend físico.
- `source_id` y `status_detail` son identificadores/detalle sanitizado, nunca
  secretos, rutas locales o bytes RTCM.

## Evidencia y límites

La evidencia previa en interior pertenece al stack legado y sirve para
caracterizar el transporte y la separación semántica, no para validar este
corte en hardware. La validación de hardware queda pendiente: con el robot
estacionario, propulsión inhibida y un único cliente NTRIP activo se observarán
las transiciones reales de calidad GNSS sin deducirlas de la llegada de
correcciones. Este componente no está `parity_passed` ni `hardware_validated`.
