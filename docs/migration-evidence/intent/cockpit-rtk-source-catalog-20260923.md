# Intención: catálogo RTK visible en Cockpit

## Alcance

- Fuente legacy: `rtk_source_manager` y `web_zone_server` de `ROS2_SALUS`.
- Destino nuevo: `salus_web` dentro del `real_mvp`.
- Incluido: proyectar el catálogo NTRIP ya configurado, sin credenciales, y el
  estado de adquisición canónico como `rtk_sources` y `rtk_source_state` del
  protocolo WebSocket de Cockpit.
- Fuera de alcance: cambiar la fuente activa en caliente, editar credenciales,
  escribir el YAML privado o iniciar/cambiar el runtime real.

## Evidencia histórica

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| `ROS2_SALUS` commit `f4f1e4e` | El modal consume `rtk_sources` y `rtk_source_state`, y el backend conserva la configuración privada. | alta |
| `ROS2_SALUS/src/map_tools/map_tools/web_zone_server.py` | El estado inicial y las actualizaciones incluyen simultáneamente la lista y la fuente activa. | alta |
| Observación SSH Jetson 2026-09-23 | El YAML privado contiene tres fuentes y `ntrip_rtcm_source` recibe RTCM de `ign_ucor`, pero `salus_web_gateway` no se suscribe ni publica el catálogo. | alta |

## Problema original e intención

El runtime nuevo recibía y entregaba correcciones RTCM, pero Cockpit mostraba
"No hay antenas configuradas" porque el bridge sólo enviaba `gps_status`.
La lista vacía no debe interpretarse como pérdida de la configuración privada.

## Contratos e invariantes

- Entrada: YAML NTRIP privado, sólo lectura, indicado por
  `rtk_sources_config`; y `GnssRtkStatus` canónico.
- Salida: campos WebSocket `rtk_sources` (`id`, `label`) y
  `rtk_source_state` en `state`/telemetría; no se exponen host, mountpoint,
  usuario, contraseña ni la ruta del archivo.
- Autoridad: la fuente efectiva, conexión y frescura proceden de
  `GnssRtkStatus`; el YAML sólo aporta el catálogo y las etiquetas.
- Compatibilidad pública: conserva el formato que consume el modal RTK de
  Cockpit.

## Diseño nuevo

- Modelo/política pura: carga y sanea el catálogo mínimo de YAML; proyecta el
  estado de fuente sin inferir calidad RTK a partir de RTCM.
- Adaptador ROS: carga una vez el catálogo al iniciar y lo añade a la caché
  WebSocket; cada `GnssRtkStatus` actualiza el estado y una secuencia.
- Configuración: `real_mvp` entrega al bridge la misma ruta privada que ya
  entrega al dueño NTRIP. No hay escrituras ni secretos en parámetros o logs.

## Fallos y degradación

| Condición | Respuesta requerida | Evidencia/test |
| --- | --- | --- |
| YAML ausente o inválido | Lista vacía; no se expone ruta, contenido ni secreto. | unitario de carga segura |
| Estado GNSS aún ausente | Se muestra la fuente configurada como reconectando; no se finge RTCM. | unitario de proyección |
| RTCM recibido sin RTK Fixed | `receiving_rtcm` puede ser verdadero y la calidad GNSS sigue separada. | pruebas GNSS existentes |

## Decisiones descartadas

- Editar el YAML desde Cockpit: el actual dueño NTRIP lo lee sólo al inicio y
  esa mutación requeriría un contrato, reinicio y revisión de secretos.
- Enviar el YAML completo: expondría endpoints y potencialmente credenciales;
  Cockpit sólo necesita identidad y etiqueta para este alcance.

## Pruebas y aceptación

- Unitarios de saneamiento, proyección y actualización de caché del gateway.
- Tests focalizados de `salus_web` y de wiring de `real_mvp`.
- No se realiza launch, reinicio ni movimiento del robot real.

## Estado de evidencia

- Estado propuesto: `ported`.
- No validado en hardware: visualización WebSocket con el binario desplegado;
  el cambio no altera adquisición ni entrega RTCM.
- Preguntas pendientes: contrato para selección/edición RTK en caliente.
