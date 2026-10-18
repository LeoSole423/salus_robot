# #328: continuación de localización y navegación automática

PC/ROS2 Humble/Fortress únicamente. Rama `agent/ekf-turns-sim-evaluation`;
base `origin/main@d6bf67d`. No se accedió a Jetson ni robot, no hubo despliegue
ni merge. Los defaults de EKF, planner, controlador y límites reales se conservan.
Esta continuación amplía la [primera campaña](../RESULTS.md).

## Conclusiones que ya tienen evidencia

La compensación de antena mejora la localización también frente a pose física
independiente. En cuatro pares con perfil nominal, 1.6 m/s y comienzo simulado
60.0 s, el máximo global en curva es 2.45–3.71° con compensación y 10.59–11.68°
sin ella; RMS posición0.074–0.083 m frente a0.596–0.632 m. Se completan 4/4 curvas
en ambos grupos; los gates funcionales son 3/4 y 4/4 respectivamente. El fallo de
llegada no se descarta del grupo compensado. Son intervenciones de navegación
con realimentación: las trayectorias físicas pueden cambiar al cambiar la pose
estimada. No son cuatro reproducciones de una misma trayectoria física grabada.
La pérdida de adquisición GPS previa en los controles es un efecto observado,
no una causa para retirarlos de esta comparación causal.

Sí existen giros automáticos más cerrados que con el límite 18° vigente.
En la exploración con rutaR2.5/planner2, cambiar sólo el tope automático18→30°
reduce las medianas de radio físico central de3.13–3.27 a2.32–2.55 m, con4/4
curvas en cada grupo. Aparece un máximo aislado de heading 7.21°. Ese planner 2
es ligeramente inferior al límite de la rueda interior modelada 2.003 m: sirve
para diagnosticar capacidad, no para recomendar sus parámetros al producto.
La confirmación usa planner2.2, compatible con24°/30°, misma expansión20 m y
rutaR2.5. El piloto24° completa 4/4 curvas y gates, radio central2.50–2.62 m,
headingmáximo 3.40–4.63°. El piloto30° coherente también completa 4/4, radio central
2.39–2.65 m y máximo 3.71–4.95°; no hay ventaja concluyente para ampliarlo a 30°.
En cada perfil sólo2/4 cumplen la elegibilidad más estricta: seed6500 registra
un fix `non_monotonic_fix` en cada lado en la ventana previa. El jitter produce
reordenamiento; el mapper rechaza ese fix y conserva 12 publicaciones válidas en
esa ventana. El criterio conservador exige cero rechazos, así que se registra
false, sin describirlo como congelamiento del EKF ni apagón completo de GPS.
RMS heading supera el margen predefinido 2.46° en varios pilotos cerrados; se
preserva como tradeoff y no se amplía el umbral después del ensayo. No equivale a garantizar ese radio en hardware.

La primera campaña ya rechaza convertir `smooth_lagged_data=true`, historial1 s,
en default universal: mejora mucho GNSS300 ms pero empeora recuperación/heading
nominal. No se repiten máscaras de #60/#266 ni se duplica yaw-rate global de
#61/#268. La única variante de EKF es temporal; no se cambian máscaras, Q,
autoridades ni covarianzas de sensores para conseguir mejores números.

## Ensayos y variaciones

| Etapa exploratoria | Intentos | Gates pass | Curvas completas | Lectura |
|---|---:|---:|---:|---|
| RutaR4, planner4, 1.2/1.6/1.8, seeds6500–6502 |18|0|0|17 guard PLANNER_LOOP, 1 timeout|
| Misma ruta, planner 3 |18|16|17|1 setup fallido; no equivale a 18 éxitos|
| Punto intermedio 45°, planner 4, 64 bins |2|0|0|guard bloquea|
| Igual, 128 bins |2|0|0|no resuelve geometría|
| Punto 45°, planner 3 |2|1|1|dependencia de puntos de paso|
| Sin punto 45°, planner 3, limpio 1.6 |2|2|2|fixture sparse factible|
| Expansión analítica 3→20 m, planner 4 |4|2|2|mejora parcial, no solución universal|
| Expansión 20, planner 3 |4|4|4|candidato coherente paraR 4|
| RutaR3/planner3/auto18, 1.2 |4|1|1|no atribuir bloqueo al EKF|
| RutaR3/planner2.5/auto18 |4|4|4|planner incompatible con18°, diagnóstico|
| RutaR2.5/planner2/auto18 |4|4|4|radio físico3.13–3.27 m|
| Igual, auto 30 |4|4|4|radio 2.32–2.55; outlier 7.21°|
| RutaR2/planner1.65/auto30 |4|4|4|radio2.26–2.99; no cumpleR2 universal|
| RutaR1.65/planner1.65/auto30 |4|0|0|rechazos del guard, preservados|
| Antena sin compensación, 1.6/1.8 |8|8|8|máximo heading9.09–12.18°|
| Lookahead 2.2→1.6 s, planner 3 |8|4|6|2 setup fallidos, no ganador reproducible|
| Wrapper llamado latency 300, realmente 100 ms |4|4|4|intervención inválida, reclasificada|
| Igual 100 ms + EKF temporal |4|4|4|NO evidencia 300 ms|
| S nominal 1.2, planner 3 |4|4|4|máximo heading 4.11–4.93°|
| Piloto recta2.0, seed6501 |1|1|1|mediana física1.998 m/s|
| Piloto90°2.0 ambos lados, seed6501 |2|2|2|medianas1.906/1.938 m/s; max3.00/3.57°|

Los pilotos 2.0 sólo se habilitaron después del par 1.8 preespecificado válido;
no constituyen campaña de robustez de 2.0. El control de velocidad adaptativa
existente reduce hacia 1.2 en curvas; ninguna variante fuerza comandos finales
hacia arriba. Frenado/parada de seguridad sigue disponible. Las mediciones de
velocidad física pueden oscilar bajo 1.2 aunque la consigna no lo haga: el ensayo
no modela ni valida la reserva de torque/motor del robot.

## Medición, fallos y criterios

`final-analysis/summary.json` y CSV contienen todos los intentos con una versión
uniforme del analizador; los archivos de cada etapa conservan la versión usada
entonces. `comparison_eligible` exige adquisición, mapper previo, captura de
parámetros y gates. `recommendation_eligible` además exige compensación activa,
modelo/planner compatibles, IMU/EKF frescos y ausencia de paradas nominales.
Ninguna de esas banderas convierte automáticamente un candidato en ganador.
Los errores disponibles se reportan incluso si falla la llegada o adquisición.
Los denominadores incluyen setup fallido, guard y ausencia de movimiento.

Se empareja por timestamp de cabecera con pose física interpolada, brecha máxima
0.2 s; sin ajustar desfase ni pose para reducir artificialmente el error. Mapa y
odom parten del datum/origen fijo de simulación; el error local incluye deriva
acumulada desde el inicio. Las métricas de curva usan ROI espacial fija. Completar
el giro requiere≥81° antes de la salida 6 m, sin recorrido angular desenvuelto
>180°. Una curva más amplia puede completarse con mal seguimiento: se conservan
ambas observaciones. No se llama rulo a la suma de pequeñas oscilaciones; se usa
el rango angular desenvuelto, con regresiones sintéticas para 450° ywiggles.

El radio observado se obtiene por circunferencia de tres poses separadas 0.25 s,
y se resume en el tramo 20–70° con duración de soporte. No es radio mínimo
garantizado ni radio pedido al planner. Tracking válido usa el último plan
recibido causalmente; la comparación contra el snapshot final se conserva sólo
como diagnóstico y puede dar errores enormes por cambiar el plan. La tolerancia
heredada de llegada 2.5 m permite éxito antes de llegar físicamente al punto;
no se relaja para mejorar la tasa de éxito.

Se observó dependencia del instante de arranque respecto a RTK_FLOAT periódico
(últimos 5 de cada 30 s). Las etapas iniciales con igual seed pero arranque variable
son exploratorias. La confirmación fija dispatch a 60±0.25 s; un setup tardío falla
sin enviar metas. No se resetean sensores ni se inicializa el filtro con truth.
El tráfico ROS puede variar el consumo de RNG: la semilla no promete replay
bit a bit, por eso se usan repeticiones y semillas reservadas.

## Correcciones del instrumento conservadas

Además de las correcciones físicas y referencia pasiva de ADR 0017, se implementan
rutas ordenadas por servicio existente; captura IMU; finalización del observer
que no reinicia su ventana con telemetría terminal repetida; overlay/hash/readback
Nav 2; copia de escenario y perfil efectivo; comienzo a época fija; archivos
reproducibles comprimidos con SHA 256. No se toca ninguna autoridad del producto.

El wrapper GNSS 300 anterior no aplicaba la latencia: el YAML explícito ganaba a
SetParameter. Se conservan esos 8 ensayos como 100 ms y se retira el wrapper con
error explícito. El nuevo override monta sólo dentro del contenedor un perfil
completo y verifica `gnss.latency_s` antes de navegar. Un mismatch falla setup.
La campaña aislada anterior 300 ms sí usaba su YAML efectivo y sigue siendo válida.

Los bundles iniciales sin IMU no se relabelan como frescos. La evolución del
analizador corrigió falsos rechazos por variación absoluta y giro fuera de ROI;
no cambió poses, errores ni gates de navegación. Los 93 hashes históricos se
mantienen independientes de esta evidencia nueva.

## Límites y siguiente validación física

La pose de referencia es independiente, pero IMU y rueda simuladas comparten
odometría de articulaciones; ruido independiente no elimina esa correlación.
No hay patinaje/terreno, carga real, torque mínimo, viento, multipath GNSS ni
obstáculos en esta campaña libre. No se introduce truth en compensación, sensores,
EKF o control. No hay demostración cuantitativa en hardware.

Antes de promover un perfil: medir radio manual real ambos lados; comprobar
seguimiento, torque y frenado a 1.2; repetirR4 yR2.5 con adquisición y logs/RTK,
IMU/EKF frescos; progresar1.6→1.8 sólo tras repeticiones estables, zona despejada,
E-stop y autorización del operador.2.0 queda como piloto separado. Mantener las
protecciones de rulos/U y registrar rechazos, sin deshabilitarlas para pasar.

## Confirmación de latencia aplicada

Se repite con YAML completo montado dentro del contenedor, lectura efectiva
300 ms previa al dispatch y época 60 s, manteniendo semillas, ruta y planner3.
La antigüedad mediana observada del GNSS es 312–315 ms en la referencia.

| Perfil temporal | Intentos | Curvas | Gates pass | Heading máximo global | RMS posición global |
|---|---:|---:|---:|---:|---:|
| EKF vigente, GNSS300 |4|3|3|5.04–5.69°|0.335–0.357 m|
| Historial1 s/smooth_lagged, GNSS300 |4|4|3|2.45–4.11°|0.044–0.106 m|

El cuarto baseline falla antes de navegar: el servicio de lectura no responde,
con RC1 y warning de timeout en el log del normalizador. No se reintenta ni se
sustituye por una corrida exitosa. Los tres pares disponibles mejoran RMS de
posición 84–88% con tratamiento temporal. El cuarto lagged sí navega y completa,
pero falla llegada; sus0.106 m/4.11° siguen en el informe aunque no tenga par
baseline. Todos los readbacks finales del grupo lagged confirman historial1 s
y smooth_lagged_data=true, además de la latencia efectiva300 ms previa.

Esto reproduce la ventaja para datos atrasados en navegación completa. La
referencia nominal100 ms de la exploración (antes mal llamada300) presenta
máximos2.24–3.76° frente a2.79–4.20° con tratamiento temporal; la primera campaña
con varias semillas también mostró regresión nominal/recuperación. Se conserva
como **perfil experimental condicionado a latencia alta medida**, no default
universal ni ajuste de máscara. Medir latencia real es requisito para siquiera
considerar ese cambio en una validación posterior del robot.

## Extensión de velocidad, semillas reservadas y recomendación final

El perfil24°/planner2.2/expansión20 completa8/8 curvas con límites1.6/1.8,
pero sólo5/8 gates: las tres fallas son llegada a1.8, pese al terminal Nav2
success. No se relaja la tolerancia ni se oculta el error del estimador al
puntuar llegada. A1.6 las cuatro pasan; a1.8 pasa una de cuatro. Heading máximo
global3.32–4.81°, RMS2.05–2.69°; posición RMS0.078–0.106 m. Varias repeticiones
exceden los márgenes relativos predefinidos de RMS. El límite superior configurado
no es velocidad sostenida: el control adaptativo entrega medianas físicas de
curva1.23–1.31 m/s, respetando consignas nominales>=1.2.

La verificación reservada usa6510–6514, ambos lados, límite1.6 y el mismo perfil:
9/10 intentos completan y pasan gates; el restante no navega porque falla la
actualización/readback de velocidad. No se reintenta para convertirlo en10/10.
En los nueve con datos: máximo global3.09–4.72°, RMS2.03–2.84°, posición
RMS0.066–0.090 m; radio central mediano2.58–2.91 m y velocidad1.23–1.24 m/s.
La dispersión confirma capacidad de giro más cerrado, pero no no-regresión
universal de heading frente a la referencia amplia ni1.6 sostenido en curva.

La salida16 m termina a menudo antes de ver readquisición del heading GPS;
null significa observación censurada, no recuperación inmediata. El diagnóstico
con salida32 m y seed6515 completa2/2 y observa primer heading válido8.469/8.580 s
después del ROI de curva, con GPS suspendido por strong_turn y ventana reconstruida.
El EKF/IMU continúa publicando; no se considera la suspensión del heading GPS
como congelación del estimador. Esa recuperación merece validación física y
no se elimina reduciendo gates para mejorar estadísticas.

**Recomendación:** conservar defaults y máscaras del EKF. Mantener24°/planner2.2/
expansión20 como perfil experimental reproducible para estudiar rutas cerradas;
no promoverlo aún como default de navegación: mejora capacidad de radio, con
tradeoff de heading, llegada y recuperación.30° no demuestra ventaja suficiente
sobre24° en la confirmación y se conserva como techo manual de investigación.
1.8/2.0 son límites explorados;2.0 tiene sólo dos pilotos amplios exitosos.
No queda demostrado poder sostener1.6/1.8 en las curvas más cerradas. La mejora
de antena sí está reproducida; aprovecharla requiere coordinar geometría del
planner y margen del controlador, no retocar EKF para esconder tracking.

## Validación y conservación

Build final:14 paquetes. Suite completa:1375 tests,0 errores,0 fallos,4 skipped.
Smokes finales aislados y secuenciales: navegación core, localización con backend
canónico, safety y route_executor, todos RC0. Logs y comandos en `validation/`.
Pruebas nuevas cubren época sin dispatch tardío, latencia no aplicada, captura
IMU, finalización terminal, servicios, rulos reales frente a oscilaciones,
rechazo GPS visible, recuperación ausente, geometría y archivo/replay idéntico.
`replay-integrity.json` verifica142 archivos y métricas idénticas de cuatro trials.
`history-recheck.json`:93 hashes históricos sin cambios. El checkout canónico
conserva su `.agents/` sin tocar. No se abre otro PR ni se espera CI, conforme a
la instrucción posterior del operador; cambios y evidencia se pushean a la rama.
