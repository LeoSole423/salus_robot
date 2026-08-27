# Ficha de intención: comparación del comando vehicular shadow

## Alcance

Comparar continuamente la traducción esperada de `/cmd_vel_final` con
`/vehicle/command_shadow` y publicar diagnóstico reproducible. El observador no
publica comandos, no bloquea la ruta legacy y no controla ningún backend.

## Correlación

Los dos tópicos se reciben de forma asíncrona y `CmdVelFinal` no tiene header ni
secuencia. El observador mantiene una cola FIFO acotada por tópico y empareja en
orden de publicación. Esto tolera que cualquiera de los callbacks llegue
primero. Una muestra sin pareja vence según tiempo monotónico; nunca se
reutiliza indefinidamente ni se compara con una muestra posterior arbitraria.

Los desbordes y timeouts incrementan contadores independientes. Esta estrategia
es válida mientras el adaptador shadow mantenga la relación uno-a-uno y el orden
del productor legado, propiedades verificadas por el smoke de este corte. Un
contrato futuro con identificador común podría reemplazar la correlación FIFO.

## Comparación y diagnóstico

Se comparan fuente, habilitación, E-stop, freno normalizado, velocidad firmada,
ángulo Ackermann, vigencia, frame y validez del timestamp. Las tolerancias de
velocidad, dirección, freno y vigencia son parámetros finitos no negativos.

`/vehicle/command_shadow/diagnostics` publica `DiagnosticArray`:

- `WARN` hasta observar el primer par comparable;
- `OK` cuando no se registraron fallos;
- `ERROR` si hubo divergencia, timeout o desborde.

Los contadores y el último motivo permanecen latched durante la vida del nodo.
El diagnóstico declara `authoritative=false`: no cambia el comando ni implica
que el nuevo contrato esté validado en hardware.

## Evidencia

- pruebas puras de tolerancias y divergencias acumuladas;
- pruebas ROS de orden inverso, timeout monotónico y diagnóstico latched;
- smoke causal que inyecta un comando, observa la traducción y exige diagnóstico
  `OK` con al menos un par comparado;
- build y suite completa del repositorio.

## Próximo corte

Implementar un consumidor de `VehicleCommand` con validación y watchdog propio,
conectado primero a un backend de simulación seleccionable. La ruta legacy debe
seguir siendo el default y no habrá conexión a hardware en ese corte.
