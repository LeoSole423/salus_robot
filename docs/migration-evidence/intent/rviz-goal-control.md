# Intención: control de metas desde RViz

## Alcance

- Fuente legacy: `ROS2_SALUS/navegacion_gps/config/rviz_global_v2_wifi.rviz`.
- Destino nuevo: `salus_navigation/nav_command_server.py` y el RViz diagnóstico.
- Incluido: herramienta `2D Goal Pose`, tópico `/goal_pose`, validación, envío a
  Nav2 y visualización del plan global calculado en `/plan`.
- Fuera de alcance: pose inicial, teleoperación, hardware y cambios de localización.

## Evidencia histórica

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| commit legacy `4b84df9` | El perfil WiFi liviano exponía control RViz | alta |
| `rviz_global_v2_wifi.rviz` | `SetGoal` publicaba `PoseStamped` en `/goal_pose` con frame `map` | alta |
| `rviz_global_v2_wifi.rviz` | El display `Path` mostraba `/plan` como línea verde | alta |
| `nav_command_server.py` | Es la autoridad nueva para reemplazo, keepout, manual y freno | alta |

## Problema original e intención

El operador necesita seleccionar en el mapa un destino para la navegación
automática durante pruebas locales. El botón estándar de RViz es útil, pero no
debe saltarse la autoridad de navegación migrada ni publicar velocidad.

## Contratos e invariantes

- Entrada: `geometry_msgs/PoseStamped` en `/goal_pose`, QoS reliable/volatile.
- Coordenadas: metros y orientación quaternion finita, exclusivamente en `map`.
- Salida: una meta `NavigateToPose` mediante `nav_command_server`.
- Feedback: `nav_msgs/Path` de Nav2 en `/plan`, reliable/volatile, frame `map`.
- Se conservan rechazo en modo manual/keepout, reemplazo de meta y freno al éxito.
- No se añade `/initialpose`: no existe evidencia de un consumidor válido en el perfil global nuevo.

## Diseño nuevo

El perfil RViz incorpora `rviz_default_plugins/SetGoal` y un display `Path`
verde para `/plan`. `nav_command_server` valida la meta y reutiliza el mismo
despacho map que la API LL. Los rechazos quedan observables como eventos
`GOAL_REJECTED` con `source=rviz`.

## Fallos y degradación

| Condición | Respuesta requerida | Evidencia/test |
| --- | --- | --- |
| frame distinto de `map` o valores inválidos | rechazar y emitir evento | test unitario |
| modo manual o keepout | rechazar sin reemplazar la meta | política compartida |
| Nav2 no disponible | rechazar y emitir evento | política compartida |

## Decisiones descartadas

- Publicar directamente a Nav2: omitiría la autoridad y políticas de SALUS.
- Agregar `SetInitialPose`: no está caracterizado para la localización GPS actual.
- Crear un panel RViz propio: complejidad innecesaria frente al contrato estándar.

## Pruebas y aceptación

- Unitarios: validación del `PoseStamped` y presencia del tool/topic.
- Integración: build y suite completa del repositorio.
- Smoke: `smoke_navigation_core_sim.sh` publica `/goal_pose` y verifica que la
  meta sea aceptada, atraviese la cadena segura y produzca movimiento simulado.
- Hardware: no solicitado ni validado.

## Estado de evidencia

- Estado propuesto: `ported`.
- No validado en hardware: toda esta funcionalidad.
