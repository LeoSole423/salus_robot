# Plan: heading continuo y correcciones GPS durante giros fuertes

## Alcance y punto de partida

Plan solicitado por el operador el 2026-10-14. Rama
`agent/heading-strong-turns`, checkout aislado, base
`main@2fc6973beee79c2f1643531acaaccf1ed6a1863c`.
No incorporar cambios del checkout de #277 sin revisar su PR y su integración.
No desplegar ni mover hardware. Este documento no acredita implementación ni
pruebas ejecutadas del cambio. Sí existe una captura baseline del código
original, documentada a continuación; conservarla antes de implementar.

Objetivo: suspender exclusivamente la corrección de orientación GPS durante
giros fuertes. La estimación de yaw local/global sigue actualizándose con la
IMU y odometría. Mantener posición GPS y la cadena de safety. Oscilaciones
pequeñas de recta no deben activar el bloqueo ni reiniciar continuamente el
historial GNSS. Inicialización absoluta sigue requiriendo una fuente válida:
el giroscopio no crea por sí solo una referencia norte.

## Evidencia previa

- CourseHeadingEstimator devuelve heading previo válido hasta 0,8 s al girar;
  el adaptador lo publica con el stamp de un fix nuevo. Un ejemplo a 0,4 rad/s
  conserva 0 grados después de 0,5 s, aunque el giro acumulado sea 11,46 grados.
- Los gates de dirección/yaw-rate consultan valores instantáneos, no la
  trayectoria del intervalo entre fixes. Un ejemplo de salida de curva devuelve
  64,54 grados cuando el último tramo apunta a 90 grados.
- on_fix usa stamp_s como now_s: el gate de edad del fix no mide edad de recepción.
  Odom/dirección se guardan sin control local de edad/alineación temporal.
- Perfil real: EKF local usa yaw-rate de IMU, pero también yaw y yaw-rate de
  odometría Ackermann. EKF global recibe yaw-rate local y yaw absoluto GPS.
  Suspender GPS no equivale a autoridad exclusiva de IMU sobre la fusión local.
- Los gates actuales de 3 grados y 0,05 rad/s son de admisibilidad de rumbo GPS;
  no deben reutilizarse automáticamente como definición de giro fuerte.

## Referencia de simulación ya ejecutada

Leer [heading-baseline-2026-10-14.md](heading-baseline-2026-10-14.md).
Instrumentación y reporte versionados en `0d2da33`, posterior al plan inicial
`b55ab4c`. Ninguno modifica el código de localización.

La captura baseline está en `artifacts/heading-baseline-20261014/` dentro del
worktree. Artefactos ignorados por Git: conservarlos, no sobrescribirlos ni
suponer que estarán disponibles al clonar en otra máquina. Incluye datos crudos
`samples.jsonl`, secuencia, parámetros efectivos de heading GPS, hashes/image ID,
métricas emparejadas por timestamp y gráficas `heading.html` y `heading.svg`.

Secuencia ejecutada en Gazebo, world free, perfil clean y seed 6400:

| Fase | Duración simulada | Velocidad pedida | Giro pedido |
| --- | ---: | ---: | ---: |
| Recta inicial | 8 s | 1,2 m/s | 0 rad/s |
| Curva derecha fuerte | 4,5 s | 1,2 m/s | -0,35 rad/s |
| Recta de salida | 8 s | 1,2 m/s | 0 rad/s |
| Stop | 3 s | 0 m/s | 0 rad/s |

Giro medido de unos 69,3 grados, no los 90 nominales: la dirección responde
gradualmente y el giro continúa brevemente al comenzar la recta de salida.
Evaluar el detector usando el movimiento medido, no solamente fase de comando.

Resultados de referencia:

- Recta inicial: error global máximo 1,11 grados; 41 correcciones GPS.
- Curva: error local máximo 3,37 grados, global 9,69 grados; 8 correcciones GPS
  al inicio, incluyendo cinco estados `hold_steer_too_high` en el debug.
- Recta de salida: errores máximos local 3,90 y global 9,93 grados. Primera
  corrección GPS a t=14,481 s, unos 1,98 s después del cambio de comando a recta.
- Stop: error global máximo 1,86 grados. Los EKF publicaron heading durante
  toda la maniobra; ausencia de corrección GPS no congeló la orientación.

Estos resultados no atribuyen todo el desfase a GPS: ya hay error local.
La IMU clean deriva de ground truth, sin ruido/sesgo. Esta captura es open-loop
sin Nav2; no mide pared, costmap, misión ni obstáculos fantasma.

## Orden de trabajo para el nuevo chat

1. Continuar en este worktree y rama; revisar status, instrucciones y este plan.
   El checkout canónico fue usado por otra tarea de #277: no cambiar su rama,
   resetearlo ni copiar sus modificaciones. Si se actualiza la base, documentar
   qué cambió y recapturar baseline cuando afecte este experimento.
2. Comprobar que el bundle baseline existe y leer provenance/metrics/gráfica.
   Antes de editar, repetir la secuencia original en otra carpeta para observar
   dispersión entre corridas; mismo seed no garantiza scheduling idéntico.
3. Implementar primero gating/ventanas/frescura y regresiones causales. Conservar
   máscaras y covarianzas EKF en la primera comparación para aislar el efecto.
   Revisar covarianza de heading GPS después si la medición lo justifica.
4. Repetir exactamente recta-curva-recta con código nuevo, preservar baseline,
   comparar por timestamp y publicar gráficas con las mismas escalas. Reportar
   error local/global por fase y alineado al giro realmente observado; actividad
   GPS, hold, tiempo de reacquisición y variación map/odom.
5. Añadir los controles de oscilaciones pequeñas, curva suave y curva izquierda.
   Son necesarios: la referencia actual sólo tiene una curva derecha fuerte.
6. Si pasa la comparación principal, ampliar a IMU con sesgo/ruido y después a
   pared/costmap. No confundir esas etapas con evidencia ya obtenida.
7. Completar gates y abrir Draft PR acotado con evidencia y limitaciones; detenerse
   antes de merge o despliegue.

## Diseño propuesto

1. Política pura de clasificación con estados NORMAL, STRONG_TURN y REACQUIRE.
   Entradas timestamped: yaw-rate medido/fusionado, velocidad, dirección medida
   y validez/frescura. Identificar el canal de IMU efectivo y su covarianza.
   Usar magnitudes absolutas para ambos sentidos, pero conservar el signo para
   integrar giro acumulado. Señales inválidas son incertidumbre, no marcha recta.
2. Entrada a STRONG_TURN por giro sostenido, con umbral de yaw-rate, confirmación
   temporal y comprobación de dirección/velocidad. Dirección grande a vehículo
   parado no debe declarar giro. Señal contradictoria debe ser observable.
   Histéresis de salida y confirmación por tiempo: nada de conmutación en cada
   oscilación ni contador dependiente de frecuencia de mensajes.
3. Distinguir clasificación de maniobra de validez de la medición GPS. NORMAL
   no autoriza cualquier chord GPS: comprobar excursión angular durante la
   ventana y coherencia temporal. Tolerar oscilaciones pequeñas, incluyendo un
   breve cruce de los límites actuales, sin reset continuo ni heading retenido
   reetiquetado. Si la ventana no es fiable, omitir esa corrección; el yaw continúa.
4. STRONG_TURN: no publicar heading GPS ni retener un rumbo absoluto viejo con
   stamp nuevo. Invalidar la ventana que atraviesa la curva una sola vez.
   Mantener callbacks IMU, EKFs, GPS posición y salida de TF en funcionamiento.
5. REACQUIRE: formar ventana posterior al giro con desplazamiento suficiente y
   baja excursión angular, sin exigir steering exactamente cero. No fijar otra
   espera larga adicional: aceptar la primera ventana que tenga evidencia válida.
   No mezclar muestras anteriores al giro. Nunca inyectar como IMU una copia del
   yaw filtrado: crearía información duplicada.
6. Arreglar freshness usando reloj actual para edad y stamp original para
   alineación. Rechazar timestamps no monotónicos/futuros fuera de contrato y
   exigir odom/dirección recientes. Una cola atrasada no debe parecer fresca.
7. Covarianza de rumbo vinculada a incertidumbre y baseline, revisando el valor
   fijo actual. No suavizar ni limitar correcciones a ciegas; medir innovación
   y variación de map/odom. No añadir una segunda integración de yaw al GPS.

Valores candidatos exclusivamente para explorar en simulación: entrada de
giro sostenido alrededor de 0,15–0,25 rad/s durante 0,2–0,4 s y salida por debajo
de 0,05–0,10 rad/s durante 0,3–0,5 s. Son hipótesis, no defaults de hardware.
Seleccionar con curvas suaves/fuertes y oscilaciones representativas; revisar
giro real a baja velocidad antes de fijar parámetros. Cada parámetro final
debe documentar tipo, unidad, rango, default y significado.

## Implementación acotada

- Propietario: src/salus_localization. Caracterizar historial relevante antes de
  editar. Nueva política separada del adaptador gps_course_heading.
- Cambiar CourseHeadingEstimator y el adaptador para ventanas/frescura/gating;
  actualizar perfiles sim/real coherentemente y diagnóstico del estado/motivo.
- Primera versión conserva fusión EKF local/global existente. Una modificación
  de pesos o retirada de yaw de ruedas será otro experimento si hay atraso
  aun sin correcciones GPS. No añadir IMU cruda al global duplicando yaw-rate local.
- Actualizar README, evidencia y migration-status. Revisar decisiones existentes
  de autoridad de orientación antes de proponer un nuevo ADR.

## Verificación antes de robot

### Caracterización y pruebas pequeñas

Reproducciones que fallen antes del arreglo: heading retenido en curva, fixes
que mezclan curva/recta, fix recibido tarde aceptado por edad cero. Cubrir también
oscilaciones de signo alternante, bordes de histéresis, giro izquierdo/derecho,
entradas stale y primera alineación. Reutilizar tests existentes y añadir sólo
casos causales; no una gran matriz de unitarios que replique la implementación.

### Simulación comparativa

Leer src/salus_evaluation/README.md y usar harness aislado por dominio/partición,
una versión por ensayo. Misma escena, seed, velocidad, sensores y configuración
salvo política. Baseline y cambio deben recibir las mismas entradas.

Primera comparación obligatoria: reproducir el bundle ya capturado usando
`tools/run_heading_capture.sh` y `tools/summarize_heading_capture.py`. Ejemplo:

```bash
bash tools/run_heading_capture.sh /ruta/al/workspace-compatible heading-after-change 118
```

El primer argumento aporta build/install compatibles; verificar qué fuentes
resuelven los symlinks. Recompilar si cambian entry points, contratos, config
instalada o código compilado. No sobrescribir `heading-baseline-20261014`.
Guardar provenance de la versión nueva y parámetros efectivos; no asumir que
el mismo tag Docker identifica la misma imagen.

Criterios de esta comparación: eliminar rumbo retenido publicado con stamp
nuevo en giro fuerte; orientación continua; ventana post-curva limpia; no
introducir un atraso fijo adicional al de reunir una ventana fiable. Evaluar
mejora global frente a la dispersión baseline y ausencia de regresión en recta.
No usar 9,69 grados como umbral inventado de seguridad. Si el error global no
mejora, investigar y reportarlo antes de añadir otros ajustes al EKF.

- Recta y recta con oscilaciones pequeñas: no STRONG_TURN, sin reinicios
  repetidos de ventana; disponibilidad de correcciones y error de yaw no peores
  que baseline bajo entradas fiables.
- Curva suave: no clasificar como giro fuerte; observar validez geométrica de
  la ventana GPS separadamente.
- Giro fuerte en ambos sentidos y recta posterior: ninguna medición absoluta
  retenida durante STRONG_TURN, yaw sigue el giro sin pausa, reacquisición usa
  sólo muestras posteriores y recupera al reunir el baseline requerido.
- Giro fuerte cerca de pared: usar world/fixture estático conocido y medición
  de ocupaciones fuera de su geometría; medir ambos costmaps y scan. Comparar
  orientaciones en odom y map, no sólo la imagen de Cockpit/RViz.

Registrar ground truth Gazebo, IMU yaw-rate, wheel/odometry, odometry/local y
global, TF map/odom y odom/base, GPS fixes/course/debug, estado del gate,
scan_clean y costmaps. Métricas: error yaw y desfase respecto de ground truth,
continuidad/frecuencia de TF, giro acumulado, variación map/odom, tiempo sin
corrección absoluta, tiempo/baseline de reacquisición y ocupación fantasma.

La IMU actual simulada deriva de ground truth: añadir sesgo/ruido y retardos
controlados representativos en un ensayo separado. Un resultado ideal no prueba
ausencia de deriva en hardware. No usar external_heading ground truth en lugar
del course GPS que queremos probar ni alimentar ground truth al estimador.

Definir bandas de aceptación antes de comparar resultados usando tolerancias
de sensor y resolución del costmap. Si baseline no reproduce desfase/fantasmas,
reportar que se verificaron invariantes, no afirmar mejora del defecto físico.
Mantener safety activo, registrar stops y resultado/progreso de misión.

### Gates de entrega

Build, tests focalizados, suite completa y smokes proporcionales de localización,
navegación/safety; CI verde del HEAD final. Draft PR separado. Comparación
documentada y umbrales seleccionados sólo después de medición. No merge,
despliegue ni afirmación de validación física como parte de este plan.

## Fuentes contrastadas

- https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst
  Fusión de yaw/yaw-rate y riesgo de información derivada duplicada.
- https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/state_estimation_nodes.rst
  Predicción/corrección y separación de frames map/odom. Verificar detalles con
  versión Humble instalada al implementar; las referencias son documentación upstream.
- https://novatel.com/solutions/velocity
  Diferencia entre dirección de movimiento GNSS y orientación del vehículo.

## Ejecución 2026-10-14

Implementación y comparación registradas en [resultados](heading-strong-turns-results-2026-10-14.md). Los invariantes de suspensión/continuidad pasan en simulación; el error global no mejora. Ruido/sesgo y pared/costmap permanecen pendientes según el paso 6. Draft, sin merge ni despliegue.
