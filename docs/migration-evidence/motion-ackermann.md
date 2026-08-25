# Evidencia de migración: movimiento Ackermann

Estado: `ported`; pendiente de comparación A/B contra `ROS2_SALUS`.

## Contrato del corte

`CmdVelFinal` entra por `/cmd_vel_final`; `salus_control` es el único traductor
hacia `/cmd_vel_gazebo`. El bridge de `salus_simulation` lo envía a
`/cmd_vel_steer` de Fortress y devuelve `/odom_raw` y `/joint_states`.

## Parámetros preservados

| Parámetro | Valor | Origen |
| --- | ---: | --- |
| Batalla | 0,94 m | `cuatri_real_v2.urdf` |
| Trocha | 0,75 m | `cuatri_real_v2.urdf` |
| Radio de rueda | 0,24 m | `cuatri_real_v2.urdf` |
| Límite físico de dirección | 0,5235987756 rad | `cuatri_real_v2.urdf` |

## Escenario reproducible

Ejecutar `./tools/smoke_motion_sim.sh`. El escenario espera odometría y
articulaciones, envía recta, giro y freno por `/cmd_vel_final`, y verifica
desplazamiento, que un `angular.z` positivo produzca yaw positivo y actuación
cero al frenar. La prueba original sólo comprobaba el valor absoluto del cambio
de yaw y no podía detectar una inversión izquierda/derecha.

## Convención de signo

ROS define yaw positivo como giro antihorario hacia la izquierda. El contrato
interno conserva `angular.z > 0 -> steer > 0 -> yaw > 0`. En el perfil legacy,
`invert_steer_from_cmd_vel=True` y `sim_invert_actuation_steer_sign=True`
producían dos inversiones que se cancelaban. La migración conservó sólo la
segunda, invirtiendo por error la actuación simulada respecto de Nav2.

La caracterización equivalente de `ROS2_SALUS` queda pendiente porque su stack
completo de simulación incluye localización, LiDAR y navegación fuera de este
corte. El siguiente paso de paridad será aislar su `sim_v2_base.launch.py` con
el mismo perfil de control y registrar una baseline antes de cambiar el estado
a `parity_passed`.
