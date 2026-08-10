# salus_interfaces

- Responsabilidad: contratos ROS compartidos.
- No contiene: nodos, launches, lógica ni contratos legacy automáticos.
- Interfaces actuales: ninguna; consultar `docs/contracts-inventory.md`.
- Estado: esqueleto.
- Prueba: `colcon test --packages-select salus_interfaces`.
- Migración: evaluar cada `msg`/`srv` anterior con propietario, semántica, QoS y test.

