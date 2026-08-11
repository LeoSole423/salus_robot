# salus_interfaces

- Responsabilidad: contratos ROS compartidos.
- No contiene: nodos, launches, lógica ni contratos legacy automáticos.
- Interfaces actuales: control/batería y el corte de arbitraje: `NavTelemetry`,
  `NavEvent`, `BrakeNav`, `SetManualMode` y `GetNavState`.
- Estado: los contratos listados conservan nombres, campos y constantes del
  sistema anterior bajo el namespace nuevo `salus_interfaces`.
- Prueba: `colcon test --packages-select salus_interfaces`.
- Migración: el resto de los contratos continúa pendiente y se incorporará por
  corte funcional.
