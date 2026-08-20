# Intención: acciones por waypoint y perfiles

Fuentes históricas: `1900342`, `b22bf77`, `a8cd6bc`, `05f03eb`, `b3cdc58` y
`600d425` de `ROS2_SALUS`.

## Problemas que resolvían

- ejecutar operaciones únicamente al alcanzar un waypoint real;
- sostener el freno durante toda la pausa solicitada;
- alternar conjuntamente percepción y navegación entre entornos urbanos y
  rurales;
- impedir un estado mixto cuando un componente rechaza el perfil.

## Invariantes migradas

- sólo se aceptan `brake_hold` y `set_navigation_profile`;
- los puntos sintéticos describen geometría, pero nunca ejecutan acciones;
- cada acción tiene estado explícito y puede cancelarse;
- `urban` es el perfil inicial y seguro;
- el perfil sólo cambia después de confirmar filtro de suelo, inflation local,
  inflation global y controlador; un fallo revierte los cambios confirmados;
- los cambios del operador se rechazan durante una misión activa o pausada.

La migración separa parser, estado de acciones y transacción de perfiles de los
clientes ROS. Los valores heredados son fixtures centralizados, no constantes
distribuidas por el ejecutor.
