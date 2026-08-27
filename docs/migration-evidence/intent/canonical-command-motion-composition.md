# Ficha de intención: movimiento compuesto con comando canónico

## Alcance

Propagar la selección exclusiva de entrada de control por los bringups de
simulación y demostrar movimiento Ackermann real dentro de Gazebo mediante la
ruta canónica. `legacy_cmd_vel` permanece como valor predeterminado.

## Composición

```text
CmdVelFinal de prueba (sin navegación)
  -> legacy_vehicle_command_adapter
  -> VehicleCommand shadow
  -> consumidor canónico + watchdog
  -> único backend sim_gazebo
  -> /cmd_vel_gazebo
  -> plugin Ackermann
  -> /odom_raw + /joint_states
```

`integration_sim.launch.py` y `sim_operational.launch.py` aceptan
`command_input_mode=legacy_cmd_vel|canonical_vehicle_command` y lo reenvían sin
reinterpretarlo. El backend real continúa inaccesible desde estos launches.

## Autoridad y límites de la evidencia

El smoke de movimiento lanza simulación y control sin navegación, por lo que su
probe puede ser el único productor de `/cmd_vel_final`. Verifica desplazamiento,
yaw positivo, parada por ausencia de comandos, reanudación con una muestra
fresca y freno.

No se publica directamente `/cmd_vel_final` cuando `nav_command_server` está
presente. La selección quedó disponible en las composiciones completas, pero
el movimiento operacional canónico mediante una meta Nav2 pertenece al próximo
corte. No hay evidencia de UART, Jetson ni hardware real.

## Aceptación

- defaults legacy preservados en ambos bringups;
- forwarding explícito probado estáticamente;
- movimiento y giro canónicos observados en odometría Gazebo;
- watchdog y freno producen salida cero;
- suite completa y smokes de CI sin regresiones.

