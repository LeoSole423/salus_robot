# Intención: packaging del runtime real MVP de #168

## Alcance

Este corte PC-first añade el preflight host-side y el packaging mínimo para
arrancar `real_mvp.launch.py` como servicio, sin ejecutar Jetson, hardware ni
movimiento. No recrea el top-level real ni duplica el runtime persistente.

## Hechos

- `origin/main` actualizado y limpio en `c165184522919ba7e2e6e2d527e4a92a284fddec`.
- `src/salus_bringup/launch/real_mvp.launch.py` ya es la composición final.
- `tools/prepare_real_runtime.sh` y `tools/real_runtime_exec.sh` son el camino
  existente para provenance, mounts, red y devices.
- La evidencia física de #210 confirmó `/dev/ttyACM0` para Pixhawk,
  `/dev/ttyUSB0` para UART, y una única autoridad de `/cmd_vel_final`, TF y
  `/odometry/local` durante la ventana validada.
- `salus-real-global-v2-wifi.service` sigue siendo el rollback legacy y no debe
  ser reemplazado ni deshabilitado.

## Decisiones

- El checker será un módulo pequeño, con probes inyectables y salida no
  destructiva. Un estado legacy activo o cualquier owner inesperado o
  duplicado produce fallo cerrado.
- El servicio ejecutará el checker versionado como `ExecStartPre` antes de
  invocar el wrapper que reutiliza `real_runtime_exec.sh`, con `/dev/ttyACM0` y
  `/dev/ttyUSB0` explícitos. Se ejecutará con el usuario operativo validado
  `admin` y su `HOME`, UID/GID y cache existentes.
- La configuración NTRIP será una ruta externa/ignorada referenciada por el
  `EnvironmentFile` del servicio; ningún secreto se versiona ni se imprime.
- El runbook documentará explícitamente el orden: preparar, detener legacy,
  checker, iniciar, readiness, detener, comprobar huérfanos y rollback.
- Los tests serán unitarios/estructurales y no arrancarán systemd ni hardware.

## No validado en este corte

- Instalación y ejecución real del unit de systemd en Jetson.
- Disponibilidad efectiva de los devices, credenciales NTRIP y estado del
  vehículo.
- Pruebas de UART, actuadores, E-stop, watchdog, movimiento o goal físico.

## Revalidación física y corrección de startup (2026-09-04)

### Hechos observados

- La aceptación física sobre `main@0970ab26c915ade8a8db2ca840535841285ed545`
  pasó F0--F2: el runtime quedó preparado, el baseline legacy mantuvo
  `/scan_3d` cercano a 10 Hz y el checker devolvió
  `REAL_MVP_AUTHORITY_PASS` una vez detenido legacy.
- F3 falló antes de cualquier actuación. El unit era `Type=simple` y ejecutaba
  `check_real_mvp_readiness.sh` como `ExecStartPost`; systemd inició ese post
  start apenas bifurcado el wrapper, cuando
  `/navigation_startup/diagnostics` todavía no tenía tipo/publicador.
- Al fallar el post-start, systemd terminó el launch. No se observaron defectos
  de authority ni de actuadores; legacy fue restaurado y `/scan_3d` volvió a
  estar fresco.

### Decisión de corrección

- Se conserva el orden y el ownership del unit: authority como `ExecStartPre`,
  launch como `ExecStart` y readiness como `ExecStartPost`.
- El readiness deja de requerir que el tópico ya exista al momento de iniciar
  el post-start. Dentro de una sola ejecución del runtime preparado crea un
  subscriber `rclpy` y espera causalmente el mensaje exacto
  `ACTIVE: ALL_NAV2_NODES_ACTIVE`, acotado por el timeout ya configurado.
- Si el launch no llega a publicar la transición activa, el readiness sigue
  fallando cerrado; no se agregan sleeps fijos, retries de servicio ni se
  relajan otros gates.

### Límites

- Esta corrección se valida PC-first con contratos de deployment y shell; la
  siguiente ventana física vuelve a empezar en F1/F2. No autoriza actuación,
  movimiento ni cambios ad-hoc en Jetson.

## Criterio de salida

El Draft PR queda limitado a checker, assets de deployment/runbook y tests
PC-first, con `build`, `test`, `show-args`, `git diff --check` y CI verde según
el workflow del repositorio.
