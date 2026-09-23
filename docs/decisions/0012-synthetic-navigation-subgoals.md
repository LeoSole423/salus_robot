# ADR 0012: Sintéticos como subobjetivos de navegación en piernas largas

- Estado: aceptada tras smoke PC/simulación; robot pendiente
- Fecha: 2026-09-23
- Relación: #244, #309, ADR 0011

## Contexto

La patrulla georreferenciada `PatrullaSencillaPolo` despachó un chunk
`43 → sintéticos → 44` de aproximadamente 149 m. El costmap global móvil es
de 300×300 m; la meta 44 estaba unos 15 m fuera de su límite. Nav2 abortó con
`Goal pose is out of costmap!`. Los sintéticos intermedios no ayudaron porque
`ComputePathThroughPoses` tenía que planificar el pedido completo hasta 44.
El límite de densidad de 60 m se aplica sólo a la extensión del par inicial,
no a esa pierna.

## Decisión

El executor limita cada pedido a un horizonte radial de 120 m desde la pose
de dispatch. Si una pose del chunk queda fuera, corta en la última pose
anterior dentro del horizonte: puede ser un checkpoint o un sintético
provisional. Tras éxito, avanza al siguiente punto
expandido y vuelve a construir la ventana pendiente. Un sintético terminal
no acredita `ROUTE_CHECKPOINT_REACHED`, no ejecuta acciones y no cambia el
checkpoint real pendiente. Los retries conservan el sufijo del subpedido
actual y no saltan checkpoints sin evidencia causal.

`nav_goal_horizon_m` es un parámetro `float`, por defecto 120 m, válido para
valores finitos >0. Representa el radio máximo de las poses de un pedido
Nav2 respecto de la posición al despachar. Con el costmap global actual de
300 m por lado, deja 30 m de margen respecto de la semianchura. Si la primera
pose pendiente ya está fuera del horizonte, se
pausa con `ROUTE_GOAL_HORIZON_UNREACHABLE` en lugar de despachar una meta que
se sabe fuera. La ruta lógica, el costmap y las tolerancias Nav2 no cambian.

## Alternativas descartadas

- Aumentar el costmap global para ocultar un pedido excesivamente largo:
  incrementa memoria y planificación sin acotar futuros chunks.
- Tratar un sintético como checkpoint de misión: duplicaría progreso y
  arriesgaría acciones/Patrol/HOME.
- Reducir el espaciamiento global a ciegas: crea más metas y no garantiza que
  la última pose de un pedido esté dentro del costmap.

## Validación

Tests puros cubren la pierna larga, el corte en sintético, la continuidad al
checkpoint real, el fallo cerrado sin sintético y el retry con sólo sintéticos
pendientes. El smoke `SMOKE_ROUTE_SCENARIO=horizon` usó un horizonte reducido
de 30 m para reproducir la frontera en un trayecto corto: despachó primero
`[0,0,0]` con terminal sintético, luego `[0,1]`, completó ambos checkpoints
una vez, produjo planes sin autointersección ni desvío y no frenó por safety
entre pedidos. El smoke de Patrol/HOME y los gates de CI deben quedar verdes
antes de fusionar. Sin evidencia de robot/Jetson en este corte.
