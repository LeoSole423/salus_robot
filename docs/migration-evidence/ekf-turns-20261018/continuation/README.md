# Continuación: navegación automática, curvas completas

Trabajo en curso. Esto no sustituye los resultados de la primera campaña.
El objetivo continúa siendo ampliar radio/velocidad útil, no cerrar el estudio
al rechazar una variante del EKF.

## Herramientas implementadas

Runner con varias metas vía servicio SetNavGoalLL existente y guard vigente;
conversión pura de mapa al datum configurable; sin cmd_vel ni TF propios.
Captura de geometría, adquisición antes de curva, velocidades reales por fase,
error local/global físico durante la curva y radio físico observado.
Overlay Nav2 experimental con hash; copia del escenario efectivo por trial.
Corrección de ventana final: telemetría terminal repetida ya no reinicia el
segundo de observación posterior al resultado. NEEDS_OPERATOR queda registrado
como fallo inmediatamente, sin esperar innecesariamente el timeout total.

## Pilotos ejecutados

- Trayectoria con adquisición20 m, entrada28 m, punto45°, fin90°, salida16 m;
  límite1.6 m/s, ambos lados, perfil limpio y seed6400.
- Planner R4/base64 bins:2/2 planes con rulos, guard bloquea sin moverse.
- Planner R4/128 bins:2/2 vuelven a bloquear; resolución angular no resuelve.
- Misma trayectoria con planner R3: izquierda bloquea; derecha completa curva,
  adquisición GNSS compensado previa y salida. Velocidad mediana1.562 m/s,
  p051.281, ninguna muestra de curva inferior1.2; giro observado95.94°.
  Global en curva máximo2.20°, RMS1.12°, posición RMS0.054 m. Radio geométrico
  mediano4.71 m (incluye transición), p053.65 m: no equivale a giro físicoR3.
- Piloto sin punto45°: en ejecución para aislar dependencia de los puntos de paso.

El piloto R3 derecha dio terminal success y todos los gates funcionales pass,
pero la versión anterior del collector prolongó la captura por reiniciar la
ventana final con cada telemetría. Se conserva el status del proceso y el bundle;
no se reescribe como instrumento perfecto. Se añadió regresión del bug.

Los pilotos no son todavía una campaña de confirmación ni un mapa completo de
velocidad/radio. No se han validado2.0 m/s ni menor radio Nav2 en esta continuación.
No hay cambios de defaults EKF/control/planner ni acceso a hardware.

## Referencias nominales completadas

- R4 sparse, tres velocidades y seeds6500–6502:18 intentos,0 curvas completadas.
  Diecisiete terminan en guard NEEDS_OPERATOR:PLANNER_LOOP; uno timeout.
- R3 planner sobre la misma geometría sparseR4:18 intentos,17 curvas físicas
  completadas y16 gates funcionales pass. Nueve cumplen además adquisición,
  mapper previo válido, mínimo nominal y captura efectiva; no confundirlos.
- GeometríaR3 con plannerR3/auto18 a1.2:4 intentos,1 curva completada. Esto
  motiva comparación con planner2.5 antes de culpar al EKF o aumentar dirección.

`reference-r4/` y `reference-r3/` conservan summary y bundle.tar.gz con manifiesto
SHA256 por archivo. Extraer con `mkdir -p artifacts/replay && tar -xzf
<bundle.tar.gz> -C artifacts/replay`; ejecutar summarize_heading_routes.py sobre
esa raíz reproduce métricas. Los caches runtime se excluyen; se conservan CSV,
readbacks YAML, planes, debug GNSS, launch logs y resultados de setup fallidos.
Generated-configs contiene las matrices/escenarios fuera del árbol instalado;
pueden regenerarse con generate_heading_route_matrix.py y los argumentos
registrados. Ningún archivo histórico se sobrescribe.

## Cola de siguientes etapas

`stages.json` define la cola secuencial ejecutada por
`tools/run_heading_route_stages.py`, siempre nav_eval matrix --jobs2: candidatos
analytic20, radios menores, comparación de techo18/30°, control causal sin
compensación, lookahead, GNSS300 ms/EKF temporal, S y piloto2.0. La progresión
hacia2/1.65 exige curvas previas completadas en ambos lados sin comandos
nominales bajo1.2. El piloto2.0 exige el par prespecificado de1.8 de seed6501
válido en ambos lados; se conserva el resto de fallos y no se declara robustez
general por ese piloto.

La nueva hipótesis analytic20 está documentada antes de ejecutarse: valor
efectivo3 m no escala con Rmin3/4; la documentación upstream Humble recomienda
4–5×Rmin. Se prueba como cambio aislado con guard intacto y no se atribuye al EKF.
