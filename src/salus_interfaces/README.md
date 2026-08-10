# salus_interfaces

- Responsabilidad: contratos ROS compartidos.
- No contiene: nodos, launches, lógica ni contratos legacy automáticos.
- Interfaces actuales: `CmdVelFinal`, `DriveTelemetry`, `BatteryMissionGuard`,
  `SetSimBatteryPreset` y `SetSimBatteryState`.
- Estado: primer corte migrado; nombres, campos y constantes conservan el
  contrato anterior bajo el namespace nuevo `salus_interfaces`.
- Prueba: `colcon test --packages-select salus_interfaces`.
- Migración: el resto de los contratos continúa pendiente y se incorporará por
  corte funcional.

