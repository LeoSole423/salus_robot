# Ficha de intención: navegación con actuación canónica

## Alcance

Demostrar que una meta Nav2 completa mueve el robot simulado usando la entrada
canónica seleccionable del controlador, sin introducir una segunda autoridad de
movimiento y sin cambiar el perfil legacy predeterminado.

## Autoridad y flujo

```text
/goal_pose -> Nav2 -> /cmd_vel -> collision_monitor -> /cmd_vel_safe
  -> nav_command_server -> /cmd_vel_final
  -> legacy_vehicle_command_adapter -> /vehicle/command_shadow
  -> controller_server (canonical_vehicle_command) -> Gazebo
```

`nav_command_server` continúa siendo el único productor de
`/cmd_vel_final`. El probe sólo publica la meta de alto nivel y observa plan,
comandos, estado del controlador y odometría.

## Evidencia y límites

El smoke canónico reutiliza las aserciones de navegación legacy y agrega:

- `VehicleCommand` AUTO habilitado, sin E-stop, sin freno, con velocidad
  positiva y vigencia explícita;
- `/controller/status` con `input_mode=canonical_vehicle_command`, muestra
  fresca y velocidad efectiva positiva;
- desplazamiento físico, finalización, cancelación y takeover manual ya
  comprobados por el mismo probe.

El smoke legacy permanece separado para conservar cobertura del modo por
defecto. Esta evidencia sólo cubre Gazebo: no valida UART, Jetson ni robot real.
