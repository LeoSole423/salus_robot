# ADR 0002: Robustecimiento Nav2 sin plugins BT legacy prematuros

- Estado: aceptada
- Fecha: 2026-08-13

## Contexto

El stack legacy registraba dos plugins propios: `IsPathClearanceValid` y
`TraceReplan`. El primero consulta un validador externo sobre el costmap global;
el segundo solo emite eventos alrededor de `NavigateThroughPoses`.

El corte actual opera exclusivamente `NavigateToPose`. Nav2 ya invalida planes
contra el costmap, el filtro keepout y los obstáculos, mientras que
`collision_monitor` detiene el movimiento local. No hay una fixture reproducible
que demuestre que el validador adicional evita un fallo que esas capas no cubran.

## Decisión

- No migrar todavía ninguno de los dos plugins a `salus_navigation_bt`.
- Añadir `nav_observer` fuera del lifecycle de Nav2 para publicar eventos de
  transición lifecycle, cambio material de plan y bloqueo/desbloqueo local.
- Reabrir la decisión solo si una prueba de pasillo, despeje o replan demuestra
  una mejora cuantificable. La futura capa de rutas podrá evaluar `TraceReplan`
  junto a `NavigateThroughPoses`.

## Consecuencias

La navegación conserva una sola fuente de control y no duplica validadores ni
servicios aún no caracterizados. `salus_navigation_bt` pasa a `characterized`,
no a `ported`; los eventos nuevos son observabilidad, no una API de control.
