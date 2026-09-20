# Issue #282 — gate funcional PC/sim de `legacy_pair`

## Alcance y provenance

Esta es la validación final del corte opt-in de #282 sobre la rama
`agent/issue-282-luna-eval`. El modo `legacy_pair` se selecciona sólo desde
los smokes; el default productivo continúa siendo `single_checkpoint`. No se
usó Jetson, hardware ni movimiento físico.

La preparación usa `salus_robot@d14d056a211248b71b6bf0ef5b65f60d9e488b63`
como base del corte de evaluación y el fix de frescura de pose de #287 como
dependencia. Los smoke artifacts se conservan bajo
`artifacts/smokes/`.

## Gates ejecutados

Primero se confirmó el aislamiento existente mediante `nav_eval`; las
simulaciones de este corte se ejecutaron serialmente, cada una con su propio
dominio ROS/Gazebo y artifact.

| Gate | Resultado | Artifact |
|---|---|---|
| ruta abierta, `legacy_pair` | PASS | `artifacts/smokes/routes-free-world-20260920T210746-1/` |
| loop + entrada a segunda vuelta | PASS | `artifacts/smokes/routes-free-world-20260920T210813-1/` |
| action checkpoint | PASS | `artifacts/smokes/routes-free-world-20260920T211203-1/` |
| manual takeover | PASS, misión queda `PAUSED` | `artifacts/smokes/routes-free-world-20260920T211540-1/` |
| cancelación durante pair | PASS, misión queda `CANCELLED` | `artifacts/smokes/routes-free-world-20260920T211651-1/` |
| patrol/HOME + batería | PASS | `artifacts/smokes/patrol-battery-return-20260920T211734-1/` |

La ruta abierta conservó requests pair `[0,1]`, `[2]`; el loop observó una
segunda iteración causal con `[0,1]` después de completar la primera vuelta.
No hubo sintéticos ni auto-intersecciones en las corridas. Los artifacts
incluyen dispatches, progreso, odometría, planes, comandos finales y eventos.

La corrida de acción demostró `ROUTE_WAYPOINT_ACTION_STARTED` y
`ROUTE_WAYPOINT_ACTION_FINISHED` para el índice esperado. Takeover y cancel
fueron verificados por su estado terminal correspondiente; el smoke omite
intencionalmente el perfil de navegación después de takeover porque una
misión pausada debe rechazar ese cambio.

Patrol/HOME completó la secuencia funcional:

```text
JOIN_LOOP -> PATROL -> EXIT_LOOP -> RETURN_HOME -> AT_HOME
```

La evidencia de geometría se conserva como diagnóstico. No se usa como
sustituto de la trayectoria ejecutada ni como autorización para cambiar el
default real.

## Recovery y límites

No se declaró PASS para fallos transitorios de planner ni de controller:
el harness existente no ofrece una inyección causal pública y segura para
provocar esas fallas dentro del smoke sin crear infraestructura artificial.
La protección disponible queda cubierta por los tests unitarios de retry,
cancelación y fail-closed del dominio; la recuperación transitoria integrada
requiere un corte posterior con un mecanismo de inyección aprobado.

También hubo intentos iniciales fallidos por setup/transporte (domain ID fuera
del rango de Fast DDS y arranques incompletos de Nav2); no se contaron como
fallos funcionales y se repitieron con dominios válidos. El script de rutas
usa ahora `FASTDDS_BUILTIN_TRANSPORTS=DEFAULT` por defecto.

## Validación local

- tests enfocados de smoke/protocolo: `57 passed`;
- `./tools/build.sh`: PASS, 14 paquetes;
- `./tools/test.sh`: PASS, `1090 tests, 0 errors, 0 failures, 2 skipped`;
- `git diff --check`, `py_compile` y `bash -n`: PASS.

Clasificación de este corte: `LEGACY_PAIR_PC_SIM_PASS` para las funciones
ensayadas, con recovery transitorio `EVIDENCE_INCOMPLETE`. El default
`single_checkpoint` no se cambia.
