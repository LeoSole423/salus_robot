# Intención: snapshot de navegación en el runtime real (#224)

## Hechos

- `nav_snapshot_server` y `navigation_snapshot.yaml` ya existen y están
  cubiertos por el renderer, el adaptador y `smoke_navigation_snapshot.sh`.
- El snapshot consume costmaps, TF timestamped y capas opcionales según ADR
  0004; no posee TF, comandos ni lifecycle de Nav2.
- La composición real única es `real_mvp.launch.py`, que incluye
  `navigation_real.launch.py`.

## Decisión

Se añade `navigation_snapshot_real.launch.py` como adaptador de launch mínimo y
se incluye una sola vez en `navigation_real.launch.py`. Fija
`use_sim_time=false` y reutiliza exactamente `navigation_snapshot.yaml`.
El snapshot permanece fuera de `navigation_core_real.launch.py`, del
`nav2_startup_coordinator` y de los gates de readiness; una captura no es una
precondición para activar navegación o control.

## Evidencia PC y límite

Los tests estructurales verifican una única instancia, ausencia de publicación
de TF/comandos y separación de lifecycle/readiness. El runtime sintético real
espera primero readiness, lifecycle y costmaps, luego consulta el servicio y
valida una respuesta PNG con las capas obligatorias disponibles. El smoke
existente de snapshot continúa cubriendo la composición de simulación.

No se usó Jetson, hardware ni movimiento; la disponibilidad del servicio en el
runtime físico queda como validación read-only posterior.
