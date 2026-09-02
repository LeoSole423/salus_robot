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

Los adapters seleccionan una única suscripción mediante `input_wire_type`:
`salus_interfaces` es el default para simulación y `interfaces` se fija
explícitamente sólo en `real_observation.launch.py`. Así no se mezclan dos
tipos DDS en el mismo tópico ni se rompe el producer canónico de simulación.

| Parámetro | Tipo | Default | Valores válidos | Significado operacional |
| --- | --- | --- | --- | --- |
| `input_wire_type` | string | `salus_interfaces` | `salus_interfaces`, `interfaces` | Selecciona al construir el nodo una única suscripción de entrada. El perfil real read-only debe fijar `interfaces`; no existe fallback automático. |

## Límites y validación

Las pruebas locales publican realmente mensajes `interfaces/msg/...` y validan
las salidas canónicas. La revalidación estacionaria en la Jetson ya se realizó
sobre `436b1ef` contra `ROS2_SALUS@f358349` y cerró las dos fronteras con
tráfico natural del legacy y sin inyectar comandos; ver
[`legacy-wire-hardware-revalidation-2026-09-02.md`](legacy-wire-hardware-revalidation-2026-09-02.md).

Límites de esa evidencia:

- cubre sólo la identidad wire de entrada y la traducción a salidas canónicas
  en régimen estacionario; no valida calibración física de tracción/dirección;
- no implica validación de localización, MAVROS, NTRIP, RS16, Nav2, UART ni
  control físico;
- registró un defecto aparte de doble `rclpy.shutdown()` al recibir SIGINT en
  los tres adapters (salida con código 1 sin afectar el cierre funcional), que
  se deja para un cambio propio y no se mezcla aquí.
