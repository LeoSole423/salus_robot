# Investigación causal del exceso de yaw durante giro fuerte

Pedido del operador: investigar qué actualizaciones agregan error al EKF global,
tras comparar baseline (9.48–9.69°) y bloqueo GPS (11.07°). Base `dbef09c`,
misma rama/worktree de #325, sin modificar #277 ni hardware. No seguimiento CI.

Se conservan máscaras, covarianzas, fuentes y gating productivos. #60/#266 ya
comparó local pose/twist sin candidato claro; #61/#268 eligió yaw-rate local
como fuente única del global. No repetir esas campañas como una solución nueva.

Instrumentación: activar únicamente debug nativo robot_localization 3.5.4 en
los EKF local/global mediante YAMLs transitorios de ensayo; capturar entradas
exactas, covarianzas y salidas por stamp. Parsear cada corrección: fuente,
stamp, estado previo/posterior, innovación, fila yaw de K y cambio de yaw.
Verificar delta yaw contra K·innovación; descartar bloques incompletos.
Distinguir predicción de corrección, alineación temporal y resultado publicado.

Repetir recta → giro derecho fuerte → recta con clean/seed6400. Comparar el
ensayo debug contra las capturas sin debug para evaluar perturbación del logging.
Si aparece una causa, realizar una ablación diagnóstica aislada en simulación
con el mismo stream de entradas antes de proponer cambios productivos.
Ninguna ablación autoriza eliminar GPS posición en operación real.

Verificación: tests causales del parser, captura de matrices/innovaciones,
reconciliación de contribuciones y reporte reproducible. Los experimentos quedan
en artifacts sin sobrescribir referencias. Documentar hechos e hipótesis;
no dar por demostrada una mejora física ni promover configuraciones.

## Resultado: causa dominante identificada en simulación

**La posición GPS de la antena está entrando al global como posición de la
base, sin compensación del montaje.** No es necesario cambiar las máscaras
seleccionadas en #60/#61 para demostrar este problema geométrico.

Hechos contrastados:

- `salus_description/urdf/components/sensor_mounts.xacro` coloca `gps_link`
  respecto de `base_link` en **(0.66, 0.20, 0.62) m**; base_link y
  base_footprint no tienen desplazamiento XY en esta descripción.
- Gazebo define el navsat en ese gps_link. El fix corresponde a la antena.
- `map_gps_absolute_measurement.py` proyecta lat/lon y publica esa posición
  directamente con frame `map`, orientación identidad y child_frame_id vacío.
  No consulta ni aplica el montaje de gps_link. El global fusiona X/Y de esa
  salida; no recibe la posición compensada de navsat_transform.
- El log nativo muestra ganancias yaw no nulas para innovaciones X/Y.
  Por tanto **suspender heading GPS no impide que posición GPS corrija yaw**.
  Al girar, el desplazamiento de antena rota respecto del mundo; tratarlo como
  posición de base introduce una inconsistencia dinámica, no sólo un offset fijo.

Primera captura con logging (sin shadows): `heading-innovations-right-20261014`,
dominio 124. Error global de curva 9.84°, salida 10.12°. En t=8–14.3 s:

| Filtro | Fuente | Corrección yaw neta |
| --- | --- | ---: |
| Local | Pose wheel | −3.863° |
| Local | IMU yaw-rate | −0.143° |
| Local | Twist wheel | −0.057° |
| Global | Posición GPS | **−6.763°** |
| Global | Twist local | −0.087° |
| Global | Heading GPS | +0.416° |

Estos son cambios internos de corrección, separados de predicción, no errores
contra ground truth ni cantidades que deban sumarse directamente entre filtros.
El global tuvo 42 correcciones GPS X/Y en ese intervalo; mayor salto 2.435°.
De su aporte por componentes, X agregó +8.874° e Y −15.628° (neto ≈−6.754°
calculado con K·innovación; delta de estado impreso −6.763°).

En un primer ensayo simultáneo, `heading-innovations-shadows-20261014`, compensar
montaje bajó el máximo de curva de 9.388° a 2.089°. Se añadió después el control
de retardo para no atribuir a geometría un efecto de ordenar mensajes.

## Controles con el mismo stream de sensores

Cola serial (para reducir perturbación del logging), dominios 74 y 75:

```bash
python3 tools/run_heading_queue.py "$PWD" heading-geometry-controls-20261014 \
  --jobs 1 --scenarios right left --ekf-debug --shadows
python3 tools/analyze_heading_innovations.py artifacts/heading-geometry-controls-20261014-right
python3 tools/analyze_heading_innovations.py artifacts/heading-geometry-controls-20261014-left
```

Clean, seed6400, world free, misma secuencia 8/4.5/8/3 s. Cuatro globales
simultáneos observan el mismo movimiento físico y las mismas fuentes de
velocidad/course GPS, con diferencias deliberadas sólo en posición:

1. Global operativo sin cambiar ningún parámetro numérico; debug habilitado.
2. Shadow sin X/Y GPS: ablación para atribución, nunca candidato operativo.
3. Shadow GPS compensado: resta R(yaw_local)·(0.66,0.20) al GPS map, usando
   yaw local interpolado al stamp GPS; no utiliza ground truth como entrada.
4. Shadow referencia demorada: recibe GPS sin compensar en la misma pasada del
   publicador que (3), con el mismo stamp y espera por cobertura de yaw local.

Los shadows tienen `publish_tf: false`, outputs `/diagnostic/global_*` y no
servicios/comandos de control. El helper sólo publica mediciones en
`/diagnostic/gps_base_position` y `/diagnostic/gps_ant_position_delayed`,
`nav_msgs/Odometry`, reliable depth10. Covarianzas originales se conservan para
**aislar geometría**, no como propuesta final de incertidumbre. Se omiten
muestras sin cobertura temporal (sin extrapolación; hueco máximo 0.2 s).
Los filtros arrancan por separado antes de la maniobra: no se promete identidad
bit a bit de histories iniciales, orden DDS o estado interno.

Máximos de error yaw global, grados, interpolados contra /odom_raw por stamp:

| Sentido | Configuración | Recta inicial | Curva pedida | Recta salida | Stop |
| --- | --- | ---: | ---: | ---: | ---: |
| Derecha | Actual | 2.07 | **9.59** | 10.25 | 1.28 |
| Derecha | Sin X/Y GPS (diagnóstico) | 1.49 | 2.45 | 2.45 | 1.29 |
| Derecha | Montaje compensado | 1.78 | **2.06** | 2.55 | 1.28 |
| Derecha | GPS original, mismo retardo | 1.77 | **9.59** | 10.25 | 1.28 |
| Izquierda | Actual | 0.95 | **10.98** | 10.98 | 1.67 |
| Izquierda | Sin X/Y GPS (diagnóstico) | 0.85 | 1.47 | 2.39 | 1.58 |
| Izquierda | Montaje compensado | 0.95 | **2.34** | 3.09 | 1.67 |
| Izquierda | GPS original, mismo retardo | 0.95 | **10.41** | 11.00 | 1.67 |

El retardo no reproduce la mejora geométrica: error sigue ≈10–11° sin
compensación. En ambos sentidos el compensado queda ≈2–3°. Es evidencia causal
de la contribución dominante del montaje en esta escena, no una campaña de
promoción ni una garantía sobre robot real.

Corrección global neta por posición, t=8–14.3 s:

| Sentido | Actual | Compensado | Original con retardo |
| --- | ---: | ---: | ---: |
| Derecha | −6.971° | −2.283° | −6.774° |
| Izquierda | +6.261° | +1.196° | +6.078° |

Correcciones pose wheel locales: −3.890° derecha, +3.914° izquierda.
Esto identifica también una contribución local, pero no invalida la campaña
previa #60 ni autoriza cambiar su default sin una evaluación nueva acotada.

Gráficas con escalas compartidas ±15° error y ±20° corrección acumulada:
[derecha](../heading-innovations-20261014/right-innovations-comparison.svg),
[izquierda](../heading-innovations-20261014/left-innovations-comparison.svg).
[Matrices e innovaciones derecha](../heading-innovations-20261014/right-innovations.json),
[izquierda](../heading-innovations-20261014/left-innovations.json).
Métricas, parámetros efectivos, versión instalada, provenance y SHA-256 de
crudos en el mismo directorio. Logs y samples completos preservados en artifacts,
ignorados por Git. No se sobrescribió ningún baseline previo.

## Calidad de atribución y verificación

- Versión instalada: `ros-humble-robot-localization 3.5.4-1jammy.20260804.195527`.
  Formato y operación contrastados con código upstream tag 3.5.4:
  [ekf.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/ekf.cpp),
  [filter_base.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/3.5.4/src/filter_base.cpp).
- Parser streaming: identifica cada processMeasurement y su stamp, fuente,
  índices fusionados, innovación, fila yaw de ganancia K y estado antes/después.
  Reconciliación K·innovación contra delta angular; tolerancia 0.0003 rad por
  precisión limitada del log nativo. No hay atribución basada sólo en cercanía
  temporal de callbacks. Inicialización se cuenta aparte.
- Ambas capturas finales: **cero bloques malformados o incompletos** en los
  cinco filtros (local, global y tres shadows). Máximo residuo de reconciliación
  de orden 0.006°, dentro de precisión impresa. Datos completos de captura
  llegaron antes de cleanup.
- Siete tests causales: ganancia X/Y que cambia yaw, wrap angular, impresión
  nativa con columnas contiguas, bloques incompletos/malformados, igualdad de
  configs numéricos debug, publish_tf falso y geometría del montaje en recta/90°.
- `tools/test.sh`: build 14 paquetes, 1253 tests colcon sin errores/fallos,
  2 skipped, más siete tests del diagnóstico y selftest del harness pasados.
  Validación de repositorio, py_compile y bash -n pasados.
- No cambios en `src/`, máscaras/covarianzas productivas, descripción, launches
  operativos, fuentes reales, hardware ni #277. No se observó CI por pedido.
- Logging perturba carga/scheduling: nuevos controles sin compensar varían
  entre ≈9.4 y 11°. La comparación 9.69→11.07 de una corrida anterior no debe
  interpretarse como un empeoramiento determinista de exactamente 1.38°.
  El efecto de geometría se evalúa con controles simultáneos, no sólo entre runs.

## Recomendación concreta, todavía sin aplicar a producto

Corregir la frontera **antena GNSS → pose de base** del adaptador simulado,
preservando posición GPS activa y la autoridad EKF elegida. Preferir TF/montaje
existente y una conversión geodésica/frame coherente; no trasladar hardcodes
experimentales a código productivo. Verificar si el camino navsat_transform ya
ofrece la transformación adecuada antes de duplicarla.

El cambio final deberá:

- usar orientación y montaje expresados en el mismo marco y timestamp que GPS;
  yaw de odom no es automáticamente yaw de map en el robot real;
- propagar incertidumbre de orientación y montaje a covarianza de posición;
- definir inicialización sin orientación absoluta, datos stale y recuperación;
- conservar datos originales y no usar ground truth ni publicar yaw filtrado
  como una segunda IMU;
- comparar ambos giros, rectas/oscilaciones, posición y heading con sensores
  independent_nominal y montajes alternativos; no sólo mirar el mínimo de yaw;
- revisar separately el adaptador real: esta prueba demuestra la causa en el
  publicador de simulación, no que el robot físico tenga idéntico defecto.

El resto del error (≈2–3° global, ≈4° local) permanece abierto. No merge ni
hardware; esta entrega es investigación y tooling de diagnóstico en Draft #325.
