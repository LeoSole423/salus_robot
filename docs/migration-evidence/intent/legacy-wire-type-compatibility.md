# Compatibilidad temporal de tipos wire legacy

## Alcance

Esta ficha documenta la corrección mínima derivada de la validación estacionaria
del `real_observation.launch.py` el 2026-09-02. No modifica `ROS2_SALUS`, no
añade un bridge de tópicos, no inicia hardware y no otorga autoridad de control.

## Hechos observados

- El deployment legacy publica
  `/controller/drive_telemetry` como `interfaces/msg/DriveTelemetry`.
- El deployment legacy publica `/cmd_vel_final` como
  `interfaces/msg/CmdVelFinal`.
- Los adaptadores iniciales suscribían tipos del paquete
  `salus_interfaces` con las mismas definiciones de campos.
- ROS 2/DDS identifica un mensaje por paquete y nombre: esas suscripciones no
  se conectan aunque la forma de los campos sea idéntica.

La fuente histórica fijada para ambas definiciones es
`ROS2_SALUS@f35834989b041f51dd325c626d2338e2232d9e53`.

## Decisión de compatibilidad

`src/interfaces` conserva únicamente los dos mensajes requeridos y mantiene
el nombre ROS exacto `interfaces`. Los únicos consumidores permitidos son:

```text
/controller/drive_telemetry [interfaces/msg/DriveTelemetry]
  -> legacy_drive_measurement_node
  -> /vehicle/measurements/{traction,steering} [salus_interfaces/...]

/cmd_vel_final [interfaces/msg/CmdVelFinal]
  -> legacy_vehicle_command_node
  -> /vehicle/command_shadow [salus_interfaces/msg/VehicleCommand]
  -> vehicle_command_comparison_node (diagnósticos solamente)
```

El paquete no es API canónica, no se permite a código nuevo y se eliminará al
retirar de forma controlada `ROS2_SALUS`. Todo tipo legacy adicional requiere
evidencia física y decisión explícita; no se copia el paquete legacy completo.

## Límites y validación pendiente

Las pruebas locales publican realmente mensajes `interfaces/msg/...` y validan
las salidas canónicas. Sigue pendiente una revalidación estacionaria de sólo
esas dos fronteras en la Jetson. No implica validación de localización, MAVROS,
NTRIP, RS16, Nav2, UART ni control físico.
