# Intención: activación causal de Nav2

## Problema observado

El nightly del 20 de agosto de 2026 mostró ejecuciones donde los procesos de
Nav2 existían, pero `bt_navigator` permanecía `inactive`. Los logs indicaban
que los costmaps intentaban activarse antes de disponer de la cadena
`map -> odom -> base_footprint`. Una ejecución adicional perdió la máscara
keepout durante el mismo periodo de arranque.

## Invariantes

- La existencia de un nodo o servicio no demuestra que sus entradas estén listas.
- Nav2 se activa una sola vez, después de observar datos progresivos y válidos.
- La ausencia de reloj, odometría, TF, scan o máscara mantiene el sistema esperando.
- Un rechazo lifecycle posterior a readiness es un fallo terminal y observable.
- Los umbrales operativos de localización, percepción y seguridad no cambian.

## Diseño migrado

`StartupPolicy` contiene únicamente estados y causas. El adaptador ROS observa
las entradas, solicita `STARTUP` al lifecycle manager y publica diagnóstico.
El map server keepout conserva su lifecycle independiente; `zones_manager`
comienza inmediatamente y espera su servicio `LoadMap`, sin un retraso fijo de
launch. Esta separación permite probar cada condición sin ejecutar Gazebo.

## Evidencia

- `test_startup_readiness.py`: gates, estados y rechazo terminal.
- `smoke_integration_sim.sh`: composición, TF y lifecycle.
- `smoke_navigation_core_sim.sh`: navegación activa.
- `smoke_navigation_zones_sim.sh`: máscara disponible.
- `smoke_route_executor_sim.sh`: misión sobre el stack activado.
