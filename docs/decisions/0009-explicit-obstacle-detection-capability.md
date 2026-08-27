# ADR 0009: perfil explícito sin detección local de obstáculos

## Estado

Aceptado para simulación. Validación en hardware pendiente.

## Contexto

SALUS debe admitir robots donde el LiDAR no está instalado o la evasión local
no forma parte del alcance. La composición anterior exigía `/scan_clean` en el
arranque, costmaps, collision monitor y arbitraje. Omitir sólo un componente
dejaba la autonomía bloqueada o fingía una protección inexistente.

## Decisión

El bringup acepta exclusivamente `obstacle_detection` (default) o
`no_obstacle_detection`. La selección ocurre al iniciar y no cambia ante fallos.

En el perfil degradado:

- no se inicia la tubería LiDAR ni se publica un scan vacío;
- las obstacle layers quedan deshabilitadas;
- un relay explícito conserva el único productor de `/cmd_vel_safe`, sin
  atribuirse detección;
- el gate de scan del startup y del árbitro se desactiva sólo por configuración;
- keepout, PathHealth, watchdog manual, arbitraje, freno y E-stop permanecen;
- `SystemCapabilities` publica `DISABLED_BY_PROFILE` para detección y LiDAR.

`ENABLED_BY_PROFILE` expresa configuración, no salud dinámica. `READY` requiere
posterior evidencia del dispositivo.

## Consecuencias

La autonomía puede operar sin detectar obstáculos físicos locales. Esta
degradación debe mostrarse permanentemente al operador y nunca habilitarse como
recuperación automática. El perfil no autoriza uso en hardware real.
