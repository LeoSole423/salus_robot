# Procedimiento de desarrollo para agentes de IA

Este documento define cómo investigar, diseñar, implementar, probar y entregar
cambios en `salus_robot`. Se aplica tanto a agentes de IA como a colaboradores
humanos. La meta no es trasladar archivos de `ROS2_SALUS`, sino conservar la
intención funcional en una arquitectura más fácil de leer, probar y depurar.

## 1. Fuentes de verdad y precedencia

Antes de editar, leer `AGENTS.md` y resolver la tarea usando esta precedencia:

1. Contratos compilados en `src/salus_interfaces`.
2. Composición vigente en `src/salus_bringup/launch`.
3. Estado y evidencia en `docs/migration-status.yaml`.
4. Ownership en `docs/package-map.yaml`.
5. ADR en `docs/decisions/`.
6. Fichas de intención en `docs/migration-evidence/intent/`.
7. Tests y código del nuevo repositorio.
8. Tests, commits y documentación de `ROS2_SALUS` como evidencia histórica.

El código legacy no es automáticamente correcto ni vigente. Ante una
contradicción, documentar qué fuente se eligió y por qué.

## 2. Preflight obligatorio

Confirmar ubicación, rama y limpieza antes de trabajar:

```bash
cd <ruta-al-repositorio>/salus_robot
git status --short
git branch --show-current
git pull --ff-only origin main
```

No sobrescribir cambios del usuario. No modificar `ROS2_SALUS`, firmware o
Cockpit desde un PR de este repositorio. Cockpit nuevo se prueba únicamente en
su rama `migration/salus-robot-cockpit`; su `main` conserva compatibilidad con
el sistema anterior.

## 3. Caracterizar antes de implementar

Toda migración o corrección semántica comienza reconstruyendo la intención.
Buscar:

- nodo y launch que eran operativos en `sim_global_v2_wifi` o
  `real_global_v2_wifi`;
- interfaces, parámetros, frames, QoS, productores y consumidores;
- tests que acompañaron la funcionalidad;
- commits que introdujeron y luego corrigieron el comportamiento;
- comentarios o documentación que expliquen fallos reales;
- decisiones descartadas, variantes experimentales y código legacy.

Comandos útiles en el repositorio histórico:

```bash
git log --oneline --all -- <ruta>
git log -S '<símbolo o tópico>' --oneline --all
git log -G '<patrón>' --oneline --all
git show <commit> --stat
git show <commit> -- <ruta>
git blame -L <inicio>,<fin> <archivo>
```

No basta con leer el último archivo. Un commit de corrección suele expresar
mejor la intención que la primera implementación.

Registrar el resultado usando
[`templates/intent-evidence-template.md`](templates/intent-evidence-template.md)
en `docs/migration-evidence/intent/`. La ficha debe separar hechos,
inferencias y aspectos no validados en hardware.

## 4. Diseñar para lectura, pruebas y diagnóstico

La forma preferida es:

```text
modelos inmutables
    -> validadores y políticas puras
    -> máquina de estados pura
    -> adaptador ROS delgado
    -> launch parcial
    -> bringup
```

### Dominio puro

Una política o máquina de estados no debe leer parámetros ROS, llamar
servicios, publicar tópicos ni consultar relojes globales. Debe recibir:

- estado anterior;
- entradas tipadas;
- timestamp o duración explícita;
- configuración validada.

Debe devolver una decisión tipada, causa y métricas observables. Esto permite
probar seguridad, histéresis, cooldown, timeouts y recuperación sin levantar
ROS o Gazebo.

### Adaptador ROS

El nodo ROS administra I/O, QoS, futures, timers y traducción de mensajes. No
debe ocultar política dentro de callbacks extensos. Cada operación asíncrona
necesita timeout, resultado terminal e idempotencia cuando corresponda.

### Contratos y ownership

- Interfaces compartidas: sólo `salus_interfaces`.
- Launch completo: sólo `salus_bringup`.
- Launch parcial aislable: paquete propietario.
- Driver/backend real: `salus_hardware`.
- Algoritmo independiente del hardware: paquete de dominio correspondiente.
- Dependencia externa: fijada en `dependencies.repos`, nunca vendorizada.

Todo tópico, servicio o acción nuevo debe documentar tipo, productor,
consumidor, unidad, QoS, autoridad y degradación. Todo parámetro debe indicar
tipo, default, unidad, rango y significado operacional.

## 5. Compatibilidad y seguridad

Preservar inicialmente nombres, campos, unidades y semántica pública. Una
simplificación incompatible requiere ADR y, cuando haya consumidores activos,
adaptador temporal.

Invariantes actuales:

```text
map -> odom -> base_footprint
/cmd_vel -> /cmd_vel_safe -> /cmd_vel_final
/scan_3d_raw -> /scan_3d -> /obstacles_cloud -> /scan_clean
```

No inventar disponibilidad ante datos ausentes o vencidos. No suavizar
políticas de seguridad para hacer pasar un test. Ninguna prueba automática
puede mover hardware real.

## 6. División de trabajo SOL y Terra

Los nombres representan dos perfiles de trabajo, no dos fuentes de verdad.

### SOL: especificación y revisión sensible

Usar SOL para:

- investigar historial y reconstruir intención;
- resolver contradicciones o semántica ambigua;
- definir contratos, estados e invariantes;
- diseñar seguridad, concurrencia, recovery y fallos;
- diagnosticar CI intermitente o defectos transversales;
- revisar la implementación completa y decidir el cierre.

SOL debe entregar una especificación acotada con archivos propietarios,
entradas/salidas, comportamiento ante fallos, tests y punto exacto de parada.

### Terra: implementación delimitada

Usar Terra cuando las decisiones ya están tomadas, para:

- crear contratos y modelos especificados;
- implementar políticas puras y adaptadores claramente definidos;
- añadir parámetros, launches parciales y fixtures;
- portar tests de caracterización;
- actualizar documentación mecánica;
- abrir un PR draft.

Terra se detiene y devuelve el trabajo a SOL ante ambigüedad histórica,
cambio de seguridad, concurrencia no especificada, autoridad TF/comando dudosa
o un fallo de CI cuya causa no sea evidente.

### Handoff mínimo

Cada cambio de modelo debe indicar:

- rama, PR y SHA;
- cambios ya realizados;
- tests ejecutados y resultados;
- decisiones todavía abiertas;
- archivos modificados;
- punto exacto donde continuar o detenerse.

## 7. Rama, commits y PR

Crear una rama desde `main` actualizado:

```bash
git switch main
git pull --ff-only origin main
git switch -c agent/<corte-acotado>
```

Prefijos habituales: `migrate-*`, `fix-*`, `test-*` y `finalize-*`. Mantener un
solo corte funcional por PR. Commits recomendados:

```text
docs(subsystem): characterize historical intent
feat(subsystem): add pure policy and ROS adapter
test(subsystem): add replay and boundary smoke
fix(subsystem): preserve documented invariant
```

Antes de publicar:

```bash
git diff --check
git status --short
./tools/build.sh
./tools/test.sh
```

Ejecutar además los smokes de la frontera cambiada. Terra abre normalmente un
PR draft; SOL lo marca listo después de revisar. Usar la plantilla
[`templates/pr-body-template.md`](templates/pr-body-template.md).

## 8. Estrategia de pruebas

Orden requerido:

1. Tests de caracterización de intención legacy.
2. Unitarios de modelos, validadores, política y máquina de estados.
3. Tests del adaptador ROS: traducción, timeout, concurrencia y error.
4. Smoke aislado de la frontera inmediata.
5. Smoke de composición, sin duplicar escenarios funcionales.
6. Replay/bag o banco cuando la evidencia dependa de hardware.

Los smokes son causales: avanzan por discovery, lifecycle, TF, mensajes válidos
y estados semánticos, no por `sleep` fijo. Consultar `docs/smoke-testing.md`.

Un componente sólo pasa a:

- `ported`: implementación y pruebas nuevas disponibles;
- `parity_passed`: comparación reproducible con el stack operativo legacy;
- `hardware_validated`: evidencia registrada de bag, banco o robot.

## 9. CI y diagnóstico de fallos

Cada PR ejecuta:

- `build-unit`;
- `simulation-core`;
- `navigation-missions`.

No fusionar con jobs rojos. Ante un fallo, clasificarlo antes de editar:

1. defecto funcional del producto;
2. defecto del harness;
3. readiness incorrecto;
4. interferencia o runtime persistido;
5. cleanup o presupuesto externo;
6. flakiness reproducible;
7. fallo no relacionado con el PR.

Revisar logs y artefactos JSON antes de aumentar timeouts. No añadir reintentos
globales para ocultar intermitencias.

```bash
gh pr checks <PR>
gh run view <RUN_ID> --log-failed
gh run download <RUN_ID>
```

El nightly detecta confiabilidad acumulada. Puede ejecutarse en segundo plano
mientras continúa otro corte; un fallo se clasifica por escenario y no detiene
automáticamente toda la migración.

## 10. Cierre del PR

Un PR está terminado cuando:

- la intención y límites están documentados;
- contratos, parámetros y ownership coinciden;
- lógica sensible tiene tests puros;
- fallos y datos stale degradan de manera explícita;
- build, tests y smokes relevantes pasan;
- los tres jobs requeridos están verdes;
- README, inventario y `migration-status.yaml` están actualizados;
- no se modificó ningún repositorio fuera del alcance;
- la evidencia pendiente de hardware queda declarada.

Fusionar mediante squash y actualizar `main`:

```bash
gh pr ready <PR>
gh pr merge <PR> --squash --delete-branch
git switch main
git pull --ff-only origin main
git status --short
git log -1 --oneline
```

## 11. Prohibiciones

- No copiar nodos monolíticos sin caracterización.
- No mezclar tipos ROS propios legacy y nuevos en un mismo stack.
- No añadir rutas absolutas de una máquina a código o documentación portable.
- No guardar secretos como parámetros ROS, logs o artefactos.
- No crear múltiples autoridades para TF o `/cmd_vel_final`.
- No declarar paridad por una prueba visual aislada.
- No modificar Cockpit `main` para facilitar la migración.
- No activar hardware real sin solicitud explícita, checklist y E-stop.
