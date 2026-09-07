# Intención: rutas y patrol/HOME en el runtime real (#223)

## Hechos

- `route_executor` y `patrol_mission_coordinator` ya son nodos productivos de
  `salus_navigation`, cubiertos por sus pruebas unitarias y los smokes de
  rutas/patrulla en simulación.
- La composición productiva única es `salus_bringup/real_mvp.launch.py`, que
  incluye `salus_navigation/navigation_real.launch.py`.
- El ejecutor entrega metas al contrato de `nav_command_server`; no publica
  velocidad. Patrol delega sus tramos al ejecutor y tampoco publica velocidad
  ni invoca Nav2 directamente.
- Patrol persiste de forma atómica `patrol_mission.json` bajo el parámetro
  `runtime_dir`; la guardia `/battery_mission_guard` precede al fallback
  `/battery_state`.

## Decisión

Se añaden launches reales mínimos para esos dos ejecutables y se incluyen una
sola vez en `navigation_real.launch.py`. Ambos fijan `use_sim_time=false`.
`real_mvp.launch.py` expone el mismo directorio de patrol y el wrapper del
runtime real lo asigna por defecto a `/ros2_ws/log/runtime/patrol`, que está
montado de forma persistente. No se agregan nodos, transportes, APIs ni otra
autoridad de comandos.

## Evidencia PC y límite

La cobertura estructural verifica composición, APIs, batería, persistencia y
la única autoridad `/cmd_vel_final`. El runtime PC sintético descubre ambas
APIs en estado idle y comprueba que el arranque no emite una velocidad positiva.
El gate posterior es una validación corta en la Jetson: presencia de ambos
nodos/APIs y safe-zero sin misión. Esta ficha no afirma una misión física ni
paridad de movimiento.
