# Intención: contrato de giro Ackermann sin `max_abs_angular_z`

## Alcance

- Fuente: issue #59 y path legacy `/cmd_vel_final`.
- Destino: `salus_control`.
- Incluido: retirar el parámetro no-op y caracterizar la autoridad de giro.
- Fuera de alcance: radio mínimo de Smac (#57), tuning Nav2 y límites físicos.

## Evidencia

| Fuente | Qué demuestra | Confianza |
| --- | --- | --- |
| `control_logic.py` previo | el argumento sólo se convertía a `float` | alta |
| `controller_server_node.py` previo | ROS declaraba y reenviaba el no-op | alta |
| `test_control_logic.py` | steering se deriva de curvatura y se satura por límite operativo/físico | alta |
| issue #66 | requested/applied steering y saturación ya quedan observables | alta |

## Decisión

Se elimina `max_abs_angular_z`. Para Ackermann, `angular.z / linear.x` es
curvatura y un máximo fijo de yaw-rate limita una curvatura distinta a cada
velocidad; no es una autoridad independiente documentada. El control conserva
el request, calcula steering con la batalla y limita el steering aplicado por
el mínimo de límite físico y operacional según fuente. Esto cubre auto/manual,
reversa y el fallback de referencia a baja velocidad sin clamps redundantes.

## Fallos y aceptación

La API y el nodo ya no contienen el parámetro. Los tests conservan los casos
de recta, reversa, baja/cero velocidad, saturación operacional/física y
manual/auto, además de comprobar la ausencia del contrato no-op. #57 decidirá
si el planner debe alinearse con ese límite; no se cambió ningún radio ni
límite en este corte.

Estado: `ported`; sin validación de hardware nueva.
