# ADR 0001: Fundación Del Repositorio

- Estado: aceptada
- Fecha: 2026-08-10

## Contexto

El workspace anterior creció como prototipo y mezcla composición, algoritmos,
hardware, web, simulación y dependencias vendorizadas. Esto dificulta descubrir
ownership, probar cambios aislados y distinguir código vigente de legacy.

## Decisión

- Usar un monorepo ROS 2 Humble con paquetes por subsistema.
- Usar Docker como entorno reproducible principal.
- Reservar `salus_bringup` para composición completa.
- Centralizar contratos en `salus_interfaces`.
- Mantener dependencias externas fuera de `src` y fijarlas por commit.
- Exigir documentación y tests antes de declarar un componente operativo.

## Consecuencias

La migración será incremental y durante un tiempo coexistirán dos repositorios.
Habrá más paquetes que antes, pero cada cambio tendrá un lugar y una superficie
de prueba definidos. Humble reduce el riesgo inicial; una actualización de ROS
requerirá un ADR separado.

