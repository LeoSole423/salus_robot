# Intención: cámara PTZ y presets

Fuentes históricas principales: `120660f` y `5decce4` de `ROS2_SALUS`,
`docs/camera-webrtc-ptz.md`, `sensores/camara.py` y
`sensores/test/test_camara_presets.py`. El cliente vigente se caracterizó
contra `cockpit:migration/salus-robot-cockpit`, en particular
`RobotDispatcher`, `NavigationService`, la vista de cámara y
`cameraPresetSave.test.tsx`.

## Problema e intención preservada

La cámara IP cumple dos funciones independientes. ROS controla posición y
zoom mediante ISAPI/HTTP Digest; MediaMTX transporta RTSP como WHEP/WebRTC
hacia Cockpit. La migración conserva esa separación: ROS no publica imágenes,
no transcodifica video y no administra MediaMTX. PTZ debe poder diagnosticarse
y operarse aunque no exista viewer o stream disponible.

El nodo legacy mezclaba configuración, HTTP/XML, política geométrica,
persistencia y servicios ROS. La funcionalidad se conservará, pero esas
responsabilidades no se copiarán juntas.

## Contratos públicos compatibles

Los servicios migrarán a `salus_interfaces` sin cambiar campos ni unidades:

| Servicio | Tipo | Finalidad |
| --- | --- | --- |
| `/camara/camera_pan` | `CameraPan` | Compatibilidad: pan absoluto en grados |
| `/camara/camera_zoom_toggle` | `std_srvs/Trigger` | Alternar entre zoom cero y fijo |
| `/camara/camera_status` | `CameraStatus` | Estado compatible completo |
| `/camara/camera_ptz` | `CameraPtz` | Movimiento absoluto o relativo por ejes seleccionados |
| `/camara/camera_preset` | `CameraPreset` | Aplicar un preset lógico |
| `/camara/camera_save_preset` | `CameraSavePreset` | Guardar un preset editable desde la pose vigente |
| `/camara/camera_ptz_state` | `CameraPtzState` | Leer pose, zoom, preset y último comando |

Definiciones exactas bajo el namespace nuevo:

```text
CameraPan:        request angle_deg
                  response ok, error, applied_angle_deg
CameraStatus:     request vacio
                  response ok, error, last_command, zoom_in,
                           pan_deg, tilt_deg, zoom_level, active_preset
CameraPtz:        request relative, apply_pan, pan_deg, apply_tilt, tilt_deg,
                          apply_zoom, zoom_level
                  response ok, error, pan_deg, tilt_deg, zoom_level
CameraPreset:     request preset
                  response ok, error, applied_preset,
                           pan_deg, tilt_deg, zoom_level
CameraSavePreset: request preset, save_zoom
                  response ok, error, saved_preset,
                           pan_deg, tilt_deg, zoom_level
CameraPtzState:   request vacio
                  response ok, error, pan_deg, tilt_deg, zoom_level,
                           zoom_in, last_command, active_preset
```

Los números son `float32`, flags `bool` y textos `string`, idénticos al
legacy. No se añadirán campos de frescura a estas interfaces públicas; la
validez se expresa mediante `ok` y `error`.

Los contratos usan grados para pan/tilt y el nivel nativo de zoom configurado.
`CameraPtz.apply_*` distingue un eje omitido de un valor cero. Una respuesta
fallida conserva `ok=false` y una causa acotada; sus campos numéricos no deben
interpretarse como una medición válida.

Cockpit conserva estas operaciones WebSocket y su payload vigente:

- `camera_pan {angle}`;
- `camera_zoom_toggle {}`;
- `get_camera_status {}`;
- `camera_ptz_move {relative, pan_deg?, tilt_deg?, zoom_level?}`;
- `camera_ptz_preset {preset}`;
- `camera_ptz_set_preset {preset, save_zoom}`;
- `get_camera_ptz_state {}`.

Las respuestas continuarán correlacionadas mediante `client_req_id`. Una
mutación exitosa devolverá un `ack` cuyo `payload` es un
`camera_ptz_state` refrescado; una lectura devuelve el mismo payload sin
fabricar disponibilidad.

## Modelo y políticas puras

`salus_hardware` separará los siguientes tipos inmutables de cualquier I/O:

- `CameraLimits`: mínimo/máximo de pan, tilt y zoom;
- `PtzPose`: pan, tilt y zoom normalizados;
- `PresetDefinition`: nombre canónico, pose y política de guardado;
- `CameraState`: disponibilidad, pose válida opcional, último comando, preset
  activo y error;
- `CameraCommandResult`: éxito, causa y estado posterior opcional.

La política pura deberá:

- rechazar valores no finitos antes de acceder al backend;
- normalizar pan a `[0, 360)` y luego ajustarlo a los límites configurados;
- limitar tilt y zoom;
- resolver un movimiento relativo desde una lectura válida y convertirlo en
  una escritura absoluta;
- mantener sin cambios los ejes no seleccionados;
- reconocer presets con tolerancia inicial de `1,5 grados` en pan/tilt y
  `0,2` en zoom;
- calcular distancia angular circular para comparar pan cerca de 0/360;
- aceptar `center -> home` y `back -> rear`;
- rechazar presets desconocidos sin modificar cámara ni persistencia.

Los límites/defaults de caracterización son pan `0..355 grados`, tilt
`0..90 grados`, zoom `1..4`, zoom cero `1`, zoom fijo `4`, timeout HTTP
`2 s`, y presets base `front/home=0`, `left=90`, `right=270`, `rear=180`,
con tilt neutral `0`. Serán parámetros centralizados con tipo, unidad y rango,
no constantes dispersas. Cambiarlos requiere fixture o evidencia de hardware.

Parámetros no secretos previstos:

| Parámetro | Default | Validación |
| --- | --- | --- |
| `backend` | `sim` en launch sim; `isapi` en real | sólo `sim`/`isapi` |
| `camera_host` | vacío | requerido para ISAPI |
| `camera_port` | `80` | `1..65535` |
| `camera_channel` | `1` | entero positivo |
| `camera_timeout_s` | `2.0` | finito, `0.1..10 s` |
| `camera_probe_cooldown_s` | `5.0` | finito, `0..60 s` |
| `camera_presets_file` | `runtime/camera/presets.json` | path escribible de runtime |
| límites/presets geométricos | valores indicados arriba | finitos y rangos ordenados |

El usuario se obtiene de `CAMERA_USER` y la contraseña exclusivamente de
`CAMERA_PASS` o de un archivo de secreto indicado por
`CAMERA_PASS_FILE`. La contraseña y su contenido no serán parámetros ROS.
La precedencia es archivo de secreto, variable directa y, si ninguno existe,
configuración incompleta. Host/puerto/canal pueden venir de parámetros ROS o
de `CAMERA_HOST`, `CAMERA_PORT` y `CAMERA_CHANNEL`, con el parámetro explícito
como autoridad. No se buscarán `.env` dentro de `install/` ni `src/`.

## Presets y persistencia

Los nombres canónicos son `home`, `front`, `left`, `right` y `rear`. Sólo
`home`, `left` y `right` son editables:

- HOME guarda pan, tilt y zoom actuales;
- LEFT/RIGHT guardan pan y tilt, conservando el zoom que ya tenga el preset;
- FRONT/REAR nunca se sobrescriben desde Cockpit.

Los overrides vivirán en un path runtime configurable, por defecto bajo
`runtime/camera/presets.json`, fuera del árbol instalado y de Git. Se validan
por entrada y se superponen a presets base; nombres desconocidos se ignoran.
Un JSON ilegible o inválido conserva los presets base y genera diagnóstico.

El guardado será transaccional: construir el mapa nuevo, escribir un temporal
en el mismo filesystem, sincronizar/cerrar y reemplazar atómicamente. Sólo
después se actualiza el estado en memoria. Un fallo de lectura, escritura o
replace conserva archivo, overrides y preset activo anteriores.

## Backends y concurrencia

`CameraBackend` expone únicamente operaciones acotadas equivalentes a
`read_state()` y `write_absolute(pose)`. No conoce ROS, presets ni Cockpit.

- `IsapiCameraBackend` es el único propietario de HTTP Digest, XML y
  `/ISAPI/PTZCtrl/channels/{channel}/absoluteEx`.
- `SimCameraBackend` mantiene estado determinista en memoria y permite
  configurar demora o fallo para tests; no simula video.
- El nodo ROS es el único orquestador de servicios y traduce resultados, pero
  no contiene geometría ni parsing XML.

Todas las operaciones de hardware se serializan mediante un único lock/callback
group mutuamente exclusivo. Esto incluye lecturas de estado: un movimiento
relativo y un guardado son secuencias read-modify-write y no pueden intercalarse
con otro comando. El lock no se mantiene fuera del tiempo acotado del backend.

El backend real no tendrá host, usuario ni contraseña inseguros por defecto.
Host, puerto, usuario, contraseña y canal se resolverán como se indica arriba;
host, usuario o contraseña vacíos dejan el nodo en estado degradado. Secretos nunca
aparecen en logs, parámetros volcados, respuestas, excepciones ni artefactos.

Al arrancar se realiza un probe acotado. Si falla, el nodo permanece vivo y
los servicios responden no disponible. Cada operación posterior puede intentar
un único probe si venció un cooldown monotónico configurable; no habrá timer de
reconexión agresivo ni reintentos encadenados. Un fallo conserva el último
estado válido sólo para diagnóstico interno, pero la respuesta vigente queda
`ok=false`.

## Seguridad del bridge y video

Las operaciones que mueven cámara, alternan zoom, aplican o guardan presets son
operaciones controladas por el `OperatorLease` de ADR 0005. Sólo el cliente
propietario puede ejecutarlas cuando el lock está habilitado. Consultar status
o estado PTZ permanece disponible sin lease para diagnóstico. Desconexión,
heartbeat vencido o lock impiden nuevas mutaciones, pero no interrumpen una
petición que el backend ya confirmó.

El stream conserva esta frontera externa:

```text
camara IP (RTSP) -> MediaMTX on-demand -> WHEP/WebRTC -> Cockpit
                    (fuera de ROS y de este corte)
```

No se reintroducen `/camera/image_raw`, MJPEG WebSocket, YOLO/ONNX ni control
de procesos MediaMTX. En simulación, Cockpit podrá probar PTZ contra el backend
simulado mientras el video aparece deshabilitado/no disponible.

## Fallos y evidencia requerida

| Condición | Resultado requerido |
| --- | --- |
| configuración real incompleta | nodo vivo, servicios `ok=false`, sin secretos |
| timeout, HTTP no exitoso o XML inválido | operación falla acotada; estado no se inventa |
| movimiento relativo sin lectura válida | no se escribe una pose absoluta |
| dos comandos concurrentes | ejecución serial; cada resultado corresponde a su estado posterior |
| archivo corrupto | presets base utilizables; warning explícito |
| fallo al guardar | archivo y memoria anteriores intactos |
| cliente sin lease intenta mutar | `CONTROL_LOCKED` o `CONTROL_OWNED`; no se llama ROS |
| consulta sin lease | permitida |
| stream ausente | PTZ sigue disponible |

La primera implementación quedará `ported` y
`hardware_validated: false`. Requiere tests puros, fixtures ISAPI, smoke ROS y
WebSocket con dos clientes, reinicio de persistencia y prueba automatizada en
la rama Cockpit de migración. Sólo una cámara física compatible validada en
banco permitirá declarar paridad/hardware.
