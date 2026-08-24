# Intención: telemetría compacta y preview LiDAR

Fuentes históricas principales: `c2b30aa`, `4b84df9`, `8d3e045`, `75ec6d4`,
`f4f1e4e` y los wrappers `sim_global_v2_wifi`/`real_global_v2_wifi` de
`ROS2_SALUS`. El contrato web vigente se caracteriza en
`web-cockpit-bridge.md` y contra la rama externa
`cockpit:migration/salus-robot-cockpit`.

## Intención preservada

La denominación legacy `_wifi` no seleccionaba una interfaz de red: reducía
frecuencia y volumen de visualización remota sin alterar los datos locales de
seguridad. El sistema nuevo expresa esa intención mediante
`telemetry_profile: compact` y reserva `full` para diagnóstico.

La reducción LiDAR legacy publicaba a 2 Hz, recortaba a ±90 grados, tomaba un
rayo de cada cuatro y limitaba el alcance a 12 m. Se conserva el algoritmo,
pero se corrigen sus fronteras:

- la fuente canónica será `/scan_clean`, no el `/scan` histórico;
- la salida estable será `/scan_preview`, no `/scan_wifi_debug`;
- `/scan_clean` seguirá alimentando Nav2, collision monitor y snapshots;
- `/scan_preview` será sólo observación remota y nunca evidencia de paridad
  del RS16;
- ninguna nube 3D atravesará el WebSocket.

## Perfiles de telemetría

`telemetry_profile` acepta únicamente `compact` o `full` y su valor por
defecto es `compact`. Un valor desconocido impide arrancar el bridge con un
error explícito; no debe seleccionar un comportamiento implícito.

En `compact`, todos los callbacks ROS actualizan la caché, pero el estado
reemplazable se proyecta en un único `nav_telemetry` a un máximo configurable
de 2 Hz. Pose, GPS, drive telemetry y batería viajan dentro de ese agregado;
no se emiten además los deltas redundantes `robot_pose`, `gps_status` y
`drive_telemetry`.

En `full`, se conserva la forma vigente: `nav_telemetry` y los deltas
individuales se emiten al ritmo de sus productores. Este perfil existe para
depuración y compatibilidad, no es el bringup normal.

El limitador usa reloj monotónico y política latest-wins. No duerme dentro de
callbacks ROS ni crea una cola por muestra. El siguiente tick publica una
copia coherente de la caché más reciente.

## Transiciones inmediatas

El límite de 2 Hz no se aplica a acknowledgements, errores, `nav_event`,
snapshots ni respuestas correlacionadas. Tampoco puede retrasar un
`nav_telemetry` cuando cambia cualquiera de estas señales:

- lease/lock del operador o su causa;
- `manual_enabled`, `goal_active` o modo operativo derivado;
- `collision_stop_active`;
- `nav_result_event_id`, estado, texto o código/componente de fallo;
- fase/estado, identidad o resultado de ruta y patrulla;
- estado de acción o perfil de navegación;
- fase HOME, causa de retorno o recomendación de retorno por batería;
- E-stop o habilitación de drive cuando estén presentes en telemetría.

La comparación se realiza sobre una firma inmutable de campos normalizados.
La primera muestra válida se emite inmediatamente. Cambios numéricos
continuos de pose, velocidad, tensión o porcentaje no son transiciones y
esperan al tick compacto. Una emisión inmediata actualiza el límite temporal
para evitar un duplicado instantáneo.

## Contrato `/scan_preview`

El productor ROS usa `sensor_msgs/LaserScan`, QoS best-effort, volatile y
profundidad 1. Los defaults son: 2 Hz, stride 4, sector
`[-1.57079632679, 1.57079632679]` y alcance máximo 12 m. Conserva header,
frame, `scan_time` y ajusta `angle_increment`/`time_increment`. Lecturas
finitas más lejanas que el máximo se convierten en `Inf`. Un scan vacío,
incremento inválido o sector sin muestras no produce salida.

El bridge transforma cada muestra válida al broadcast reemplazable:

```json
{
  "op": "scan_preview",
  "frame_id": "base_footprint",
  "stamp": {"sec": 0, "nanosec": 0},
  "angle_min": -1.5708,
  "angle_increment": 0.01745,
  "range_min": 0.4,
  "range_max": 12.0,
  "ranges": [null, 3.2, 2.8],
  "valid_count": 2
}
```

Todo valor no finito se serializa como `null`; no se envían intensidades.
`valid_count` cuenta únicamente rangos finitos dentro de límites. La ausencia
o caducidad se detecta en Cockpit por tiempo desde la última muestra; el
backend no fabrica un scan vacío como heartbeat.

## UI y límites de este corte

Cockpit mostrará posteriormente un mini radar relativo al robot, no una
proyección Leaflet. Esto evita atribuir precisión geográfica a un producto de
diagnóstico y no depende del heading global, que el bridge aún no publica.
La UI permanecerá exclusivamente en `migration/salus-robot-cockpit`; el
`main` legacy no se modifica.

La implementación se acepta sólo si se prueba que `compact` reduce mensajes y
bytes frente a `full` sin perder transiciones, y que `/scan_preview` nunca
tiene consumidores de seguridad o navegación.
