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
