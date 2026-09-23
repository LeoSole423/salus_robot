# Issue #63 — tolerancia global de llegada de Nav2

## Decisión

`PositionGoalChecker.xy_goal_tolerance` pasa de `1.2 m` a `2.5 m` en los
perfiles real y de simulación. El evaluador adopta el mismo valor como gate
funcional para que sus artifacts no contradigan al runtime.

## Evidencia

En la repetición georreferenciada de `PatrullaSencillaPolo`, un plan de
aproximación con geometría excesiva apareció cuando el robot ya se encontraba
cerca del checkpoint. Con el override simulado de `2.5 m`, Nav2 completó el
goal sin ejecutar esa aproximación y la transición al siguiente tramo fue
fluida. El plan excesivo se conserva como diagnóstico de #244; esta decisión
no afirma que lo haya corregido.

## Alcance y límites

- Se cambia sólo el umbral XY global; no se modifica yaw, Smac, RPP, safety,
  control ni autoridad de comandos.
- `PositionGoalChecker` vigente no exige yaw para declarar éxito. La calidad
  geométrica de yaw/curvas sigue bajo #244 y #282.
- HOME, acciones y yaws explícitos también usan `2.5 m` por decisión operativa
  del operador. Si luego requieren precisión distinta, se implementará una
  política por clase de endpoint en un corte separado.

## Validación requerida

Las tres configuraciones Nav2 y el launch del evaluador deben declarar `2.5`.
Las pruebas de navegación siguen verificando resultado terminal, safe-zero y
ausencia de autoridad adicional; una prueba física futura necesita evidencia
separada.
