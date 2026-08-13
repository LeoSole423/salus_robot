# Intención caracterizada: path estable y despeje

## Evidencia legacy

- `f4f093b` introdujo un guard de clearance para evitar replans sin causa.
- `6634dcf` y `4a1a2b4` estabilizaron la validación y su trazabilidad.
- `3457b65` ajustó Nav2 para Ackermann sin recuperación de giro ni marcha atrás.

## Invariantes migrados

- Conservar el path activo ante errores pequeños y ruido aislado.
- Replanificar por colisión, inflación sostenida, desvío persistente o falta de progreso.
- Parar el movimiento automático y esperar ante costmap o TF no disponibles.
- Validar un candidato antes de reemplazar el path activo.

## Decisión de estructura

`PathHealthPolicy` conserva la lógica pura. `EvaluatePathHealth` declara si la
consulta corresponde al path `ACTIVE` o a un `CANDIDATE`; el BT C++ sólo
coordina estados. Esto impide que el orden o la concurrencia de llamadas ROS
modifique la histéresis del path activo.

## Pruebas

`test_path_health.py` cubre colisión, inflación, histéresis, stale data,
progreso y aislamiento de candidatos. `smoke_navigation_core_sim.sh` verifica
la cadena Nav2 integrada y la interfaz del servicio.
