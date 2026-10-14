# Heading GPS en giros fuertes: implementación y comparación

Fecha: 2026-10-14. Worktree y rama solicitados, base `3bbbbf5`.
Implementación limitada a `salus_localization` e instrumentación. No se modificó
el checkout de #277, no se fusionó ni se desplegó en hardware.

## Intención y caracterización

Aplica [el plan](heading-strong-turns-plan.md). El nuevo historial
`32a7487` introdujo gates instantáneos y hold; `bf39a51` añadió buffering de
discovery, que se conserva con expiración y cancelación. `0ac84d1` introdujo
el RTK tipado, sin cambios en su autoridad. En el histórico
`ROS2_SALUS/src/navegacion_gps/navegacion_gps/gps_course_heading_core.py`,
los commits `8f49209` y `9dd7ecc` ya usan hold configurable, default cero,
y separan estimador del adaptador. Se consultaron como evidencia, sin copiar código.
Las decisiones de autoridad ya existentes permanecen: EKF local y global
publican sus TF; el estimador GPS no publica yaw integrado ni copia yaw EKF.

Se caracteriza y reemplaza la publicación del último rumbo válido durante
steering excesivo con timestamp nuevo. Se conservan RTK, GPS posición,
contratos públicos, máscaras y covarianzas de los dos EKF. La orientación GPS
válida conserva covarianza de yaw 0.05 rad².

## Implementado

Política pura `HeadingTurnPolicy`: entrada por yaw-rate fusionado ≥0.18 rad/s,
steering ≥5° y velocidad ≥0.3 m/s durante 0.25 s; salida ≤0.08 rad/s durante
0.35 s. Magnitudes simétricas, integral relativa con signo y reloj explícito.
Parámetros compartidos sim/real documentados en el README. Son candidatos
para simulación; no tienen validación física. El umbral de velocidad de heading
GPS sigue siendo 0.8 m/s y el del detector tiene un nombre ROS independiente.

`STRONG_TURN` cancela correcciones/pending GPS y limpia la ventana una vez.
`REACQUIRE` limpia al salir y reúne sólo fixes posteriores; se publica al
reunir ≥2 m dentro de 2.5 s con excursión ≤5°. La geometría GPS tiene su propio
gate, separado de la maniobra. Oscilaciones pequeñas no reinician la ventana.
No hay hold ni espera fija posterior adicional.

Fixes atrasados/futuros/duplicados/no monotónicos se rechazan usando reloj de
recepción y stamp original. Odom y drive deben tener cobertura reciente ≤0.5 s;
un intervalo inválido invalida la ventana una vez. Cero timestamps no reciben
un stamp artificial. No se cambia ningún callback IMU, EKF, TF, safety o GPS posición.

## Comparación reproducible

El baseline original `artifacts/heading-baseline-20261014/` permanece intacto.
Su manifiesto SHA-256 y copia de gráfica/métricas están
[versionados](../heading-strong-turns-20261014/preserved-baseline-sha256.json).
Los crudos siguen sólo en el worktree, ignorados por Git. Repetición antes de
editar: `heading-baseline-repeat-20261014`, dominio 118, fuentes originales.
Build compatible original reutilizado de sólo lectura; después se construyeron
los 14 paquetes en este worktree y se usó su propio build/install.

Cola final: `python3 tools/run_heading_queue.py "$PWD" heading-delivery-20261014 --jobs 2`.
Reutiliza el asignador/locks de dominios de `salus_evaluation`; cada captura
Docker tiene `network=none`, 4 CPUs, dominio/partición propios y cleanup.
`tools/nav_eval.sh isolation artifacts/heading-isolation-20261014` pasó los nueve
checks y cleanup antes de usar paralelismo. El runner de matrices Nav2 existente
posee metas closed-loop; la cola de captura mantiene la referencia open-loop.

Los cuatro trials finales pasaron (dominios 70–73): right, left, gentle y
oscillations. Mismo world free, perfil clean, seed 6400 y secuencia de duraciones
8 / 4.5 / 8 / 3 s. Right reproduce exactamente las velocidades 1.2 m/s y giro
−0.35 rad/s. Left cambia sólo el signo; gentle pide −0.10 rad/s;
oscillations pide 0.06·sin(2πt) rad/s durante las tres fases móviles.

[Comparación completa por timestamp](../heading-strong-turns-20261014/comparison.json).
Cada celda es máximo de error yaw **local / global**, grados, según fase de comando:

| Ensayo | Recta inicial | Curva | Recta salida | Stop | Primer GPS salida, t desde inicio |
| --- | --- | --- | --- | --- | --- |
| baseline | 0.00 / 1.11 | 3.37 / 9.69 | 3.90 / 9.93 | 3.87 / 1.86 | 14.480999999999998 |
| baseline-repeat | 0.00 / 1.11 | 3.40 / 9.48 | 3.91 / 11.11 | 3.88 / 1.85 | 14.379 |
| right | 0.00 / 1.33 | 3.32 / 11.07 | 3.85 / 11.24 | 3.81 / 1.59 | 16.055999999999997 |
| left | 0.00 / 2.01 | 3.37 / 10.42 | 3.89 / 10.73 | 3.86 / 2.35 | 16.059 |
| gentle | 0.00 / 1.28 | 1.09 / 3.54 | 1.27 / 4.02 | 1.27 / 0.26 | 14.040000000000001 |
| oscillations | 0.04 / 1.07 | 0.01 / 0.79 | 0.02 / 1.24 | 0.01 / 0.41 | 12.578999999999999 |
Gráficas con escalas idénticas baseline/right:
[baseline](../heading-strong-turns-20261014/baseline-heading.svg),
[repetición](../heading-strong-turns-20261014/baseline-repeat-heading.svg),
[right](../heading-strong-turns-20261014/right-heading.svg).
[Left](../heading-strong-turns-20261014/left-heading.svg) usa escalas reflejadas;
[gentle](../heading-strong-turns-20261014/gentle-heading.svg) y
[oscillations](../heading-strong-turns-20261014/oscillations-heading.svg) conservan
las escalas right. GPS se muestra como puntos, sin interpolar outages.

En right: STRONG_TURN observado entre t=9.756 y 14.361 s; cero correcciones
válidas en los 31 diagnósticos del estado, cero holds. La maniobra física
≥0.18 rad/s transcurre entre 9.273 y 12.969 s, incluyendo salida del comando;
error máximo local/global alineado a ese intervalo: **3.69 / 11.24°**.
Los baselines equivalentes fueron **3.74 / 9.93°** y **3.74 / 9.87°**.
Hay 8 correcciones al comienzo de la fase de curva, antes del estado fuerte;
no se presentan como correcciones durante todo el giro fuerte.

Primera corrección limpia right a t=16.056 s: 1.695 s después de REACQUIRE,
baseline 2.040 m. Left: 1.701 s y 2.042 m. Es tiempo de reunir desplazamiento
post-salida confirmada, sin cooldown fijo. Baseline reanudaba antes usando un
historial que no comprobaba la trayectoria de la ventana. La nueva espera es
observable y debe revisarse en hardware; no se promete recuperación instantánea.

En right, EKF local/global y ambos TF siguen publicando durante la maniobra;
huecos máximos 0.063 / 0.099 s. Máximo paso yaw map/odom 2.00°, frente a 2.40°
y 3.89° en los baselines. Esto no implica ausencia de innovaciones bruscas.
Left también tiene cero heading válido en STRONG_TURN y cero holds.
Gentle y oscillations permanecen NORMAL, generación de ventana 0; gentle
rechaza 25 ventanas geométricas, oscillations publica 127 correcciones y no
reinicia su ventana. Un fix futuro respecto del reloj recibido fue rechazado
correctamente en oscillations. La publicación IMU y estimación continúan.

## Interpretación y límites

**La comparación principal NO demuestra mejora del error global.** El máximo
durante la curva right sube de 9.48–9.69° a 11.07°. En recta inicial también
hay dispersión, 1.11° frente a 1.33°. No se seleccionan esos números como límites
de seguridad ni se declara ausencia de regresión estadística con dos baselines.
La suspensión y continuidad pasan; el objetivo de mejorar yaw global queda abierto.

Investigación adicional sobre crudos: entre t=8 y 14.3 s, la integral de
velocidad de giro global es aproximadamente −66.14°, −66.51° y −65.29°
(baseline, repetición, right), mientras sus cambios de yaw son −72.56°, −73.54°
y −69.83°. Diferencia no explicada por integrar rate: −6.41°, −7.03°, −4.54°.
El pico right ocurre a t=12.717 s, dentro del bloqueo GPS. Por tanto, el hold GPS
no explica por sí solo todo el error. El EKF local mantiene error y el global
sigue fusionando posición GPS y velocidad local; sus covarianzas cruzadas
pueden afectar yaw (hipótesis, no atribución demostrada). Se requiere registrar
innovaciones/estados internos o replay EKF para separar esas causas.
No se alteran pesos/covarianzas para ocultarlo.

El punto 6 del plan condiciona ruido/sesgo y pared/costmap a pasar la comparación
principal: **no se amplían aquí**, quedan pendientes. No hay evidencia sobre
obstáculos fantasma, Nav2 frente a pared, drift de IMU real ni robot físico.
El diagnóstico usa yaw-rate del EKF local, no autoridad IMU exclusiva.

## Verificación y fallos preservados

- Build final: 14 paquetes, `tools/build.sh`.
- Suite completa final: **1253 tests, 0 errores, 0 fallos, 2 skipped**;
  `tools/test.sh`, incluye regresiones de ambos sentidos, oscilaciones,
  ventana post-curva, fixes tardíos/futuros/duplicados y velocidades separadas.
- Smokes localization y safety pasaron. Navigation-core tuvo un primer timeout
  de respuesta física derecha; se preservó el bundle. Repetición aislada dominio
  122 pasó todos los gates de navegación. El proceso launch terminó con SIGSEGV
  durante cleanup posterior; no se presenta como un runtime sin incidencias.
- Algunos ensayos exploratorios tuvieron timeout de discovery del nodo en
  60 s; quedan preservados, sin reintento global añadido ni timeout ampliado.
- Primer build falló por permisos de directorios generados vacíos. Se construyó
  con usuario root y luego se corrigió ownership de build/install/log de este
  worktree para usar las herramientas normales; no se tocó el checkout canónico.
- La revisión detectó colisión de nombres entre umbrales de velocidad; se corrigió
  antes de la cola de entrega y tiene regresión. Los bundles `heading-final-*`
  y `heading-after-change-*` son exploratorios, no la versión de entrega.

Estado: `ported`, no `parity_passed` ni `hardware_validated`. Draft PR; revisión
sensible y mejora del yaw global pendientes. Detener antes de merge/despliegue.

## Investigación posterior de la causa

La [investigación de innovaciones](heading-innovation-investigation-2026-10-14.md) identificó una contribución dominante de posición GPS sin compensación del montaje de antena en el adaptador simulado. Controles simultáneos en ambos sentidos bajan el error de curva de ≈10° a ≈2° al compensar la geometría. Es diagnóstico, no cambio productivo ni validación física; el gate de heading por sí solo sigue sin acreditar la mejora.
