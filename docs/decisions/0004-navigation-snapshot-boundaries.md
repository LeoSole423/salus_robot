# ADR 0004: Límites del snapshot de navegación

- Estado: aceptada
- Fecha: 2026-08-21

## Contexto

El servidor legacy reunía costmaps, keepout, footprint, zonas de colisión,
scan y plan en una imagen PNG consumida por Cockpit. Los commits `0338ea4` y
`f4f1e4e` muestran una intención importante: mejorar la legibilidad no debía
convertir el snapshot en una segunda visualización completa del robot. El
último commit retiró vehículo artificial, leyenda y marcadores de dirección.

El nodo legacy mezclaba recepción ROS, TF, geometría, render y codificación.
También convivía en el mismo corte histórico con el gateway WebSocket y el
registro de misiones, aunque esas responsabilidades no forman parte del
snapshot.

## Decisión

- `salus_navigation` será propietario de la captura y generación del snapshot.
  `salus_web` será un cliente y no renderizará navegación.
- Se preservan exactamente `NavSnapshotLayers`, `GetNavSnapshot` y el endpoint
  `/nav_snapshot_server/get_nav_snapshot` bajo `salus_interfaces`.
- El runtime se separará en caché ROS thread-safe, ensamblado puro de escena y
  renderer PNG determinista. Sólo el adaptador ROS conocerá tópicos, TF y reloj.
- El costmap local y la transformación de `base_footprint` a su frame son los
  únicos requisitos para responder `ok=true`. Si faltan, la respuesta será un
  error sin imagen.
- Costmap global, keepout, footprint, stop zone, polígonos de colisión, scan y
  plan son capas opcionales. La ausencia o imposibilidad de transformar una
  capa la omite sin invalidar las restantes.
- Cada booleano de `NavSnapshotLayers` significa que la capa aportó píxeles al
  resultado. No significa únicamente que se recibió un mensaje.
- Se conservará el render limpio posterior a `f4f1e4e`: no se dibujarán un
  vehículo sintético, leyenda ni marcadores de dirección.
- La entrada LiDAR será `/scan_clean`. `/scan_preview` pertenece al futuro
  bridge compacto y no será fuente del snapshot.
- La zona de stop se leerá directamente de `/stop_zone_raw`; no se recuperará
  el republisher legacy `/stop_zone`.
- El snapshot no analizará `/rosout`, no iniciará rosbag y no persistirá
  archivos. Esas responsabilidades permanecerán desacopladas.

## Frescura y consistencia

- La escena será una copia inmutable de la última muestra de cada entrada al
  comenzar la solicitud; mensajes posteriores pertenecerán al siguiente
  snapshot.
- La caché de costmaps aplicará `/local_costmap/costmap_updates` y
  `/global_costmap/costmap_updates` sobre el último mapa completo. Esto conserva
  la publicación global compacta del stack operativo sin presentar como actual
  una grilla inicial antigua. El costmap local mantiene
  `always_send_full_costmap=true`, igual que la variante legacy optimizada.
- El costmap local deberá tener stamp válido y una edad ROS no mayor que
  `local_costmap_max_age_s` (default 2.0 s, mínimo 0.1 s).
- Las capas dinámicas opcionales usarán `dynamic_layer_max_age_s` (default
  2.0 s, mínimo 0.1 s). Una capa vencida se omite y su flag queda en `false`.
- Keepout es configuración transitorio-local y no caduca por edad.
- Si el reloj ROS no avanza o una entrada usa stamp cero, se acepta durante
  `startup_grace_s` (default 5.0 s) desde la primera recepción. Después se
  considera inválida. Esta tolerancia no altera umbrales del robot.
- Todas las transformaciones se resolverán para el stamp de la capa, con
  `tf_timeout_s=0.2`. No se reutilizará silenciosamente una TF fallida.

## Límites

- Ventana: 30.0 m centrada en el robot, configurable, mínimo 5.0 m.
- Imagen: 512x512 px, configurable entre 128 y 1024 px.
- Inset global: 160 px, configurable entre 32 px y la mitad del tamaño final.
  Esto mantiene el default legacy y permite una configuración coherente cuando
  la imagen principal usa su mínimo de 128 px.
- Objetivo de generación: 500 ms. Superarlo genera diagnóstico; no cambia una
  imagen correcta por un error una vez generada.
- Formato único: `image/png`; dimensiones y bytes deben ser coherentes.
- La caché conserva una sola muestra por entrada y nunca conserva imágenes.

## Respuesta y errores

- `stamp` será el tiempo ROS de generación de la escena, no el stamp de una
  entrada particular.
- En éxito: `ok=true`, `error=""`, `mime="image/png"`, dimensiones no nulas,
  `frame_id` igual al frame del costmap local y `image_png` no vacío.
- En fallo: `ok=false`, `mime=""`, dimensiones cero e `image_png` vacío. Los
  flags sólo podrán reflejar capas procesadas antes del fallo.
- `error` comenzará con un código estable seguido de `: ` y detalle humano:
  `MISSING_LOCAL_COSTMAP`, `INVALID_LOCAL_COSTMAP`, `STALE_LOCAL_COSTMAP`,
  `MISSING_LOCAL_TF`, `PNG_ENCODE_FAILED` o `INTERNAL_ERROR`.

Los parámetros del adaptador conservarán los nombres legacy para servicio,
tópicos, frame, extensión, tamaño, inset, objetivo de tiempo y timeout TF. Se
agregan los tópicos incrementales de ambos costmaps y las políticas
`local_costmap_max_age_s`, `dynamic_layer_max_age_s` y `startup_grace_s`, con
los defaults definidos arriba. Todos se declararán en un único YAML instalado,
no como constantes dispersas.

## Consecuencias

Los tests de geometría y render pueden ejecutarse sin ROS ni TF. Cockpit sigue
recibiendo el mismo contrato, pero el gateway deja de ser propietario de la
imagen. La frescura evita presentar como actual una escena dinámica antigua y
queda diferenciada del timeout de generación.
