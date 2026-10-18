# ADR 0017 — Contrato de movimiento simulado y referencia física observadora

- Fecha: 2026-10-18
- Estado: implementado para evaluación PC; hardware no validado
- Relación: #328, ADR0007/0016, PR325/327

## Problema caracterizado

Fortress instalado6.18.0 interpreta Twist.angular.z como yaw-rate. El backend
sim enviaba un ángulo: R4 a1.2 m/s entregaba rate≈0.230 en vez de0.300 rad/s.
Además, wheelbase no es la clave SDF wheel_base; el plugin usaba1.0 m en lugar
de0.94. kingpin_width faltante usaba0.8 en lugar de0.75 m del modelo.

La odometría Ackermann /odom_raw integra ruedas y promedio de articulaciones:
no mide la pose física del modelo. IMU simulada deriva de esa misma odometría.
Compararla con un EKF alimentado por ella subestima errores comunes. No se
interpreta la evidencia histórica de≈2° como error físico independiente.

Fuentes upstream contrastadas con fórmula y ensayo observado:
- https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/ackermann_steering/AckermannSteering.cc
- https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/odometry_publisher/OdometryPublisher.cc

## Decisión

El backend conserva el porcentaje de actuador y su límite aplicado, pero envía
omega=v*tan(delta)/L al plugin, con el wheelbase configurado. Esto mantiene signo
y cinemática también en reversa; freno/E-stop continúan publicando cero. El
modelo usa wheel_base0.94 y kingpin_width0.75, sin cambiar dimensiones físicas.
Son correcciones sim-only; UART, firmware y configuración real no cambian.

Añadir OdometryPublisher de Fortress para observar worldPose del modelo. El
bridge publica /simulation/ground_truth, nav_msgs/Odometry, 50 Hz, frame map,
child base_footprint, metros/radianes, QoS bridge reliable/volatile/depth10.
Consumidores: herramientas de evaluación exclusivamente. El TF auxiliar Gazebo
/simulation/ground_truth_tf no se puentea a /tf. No se introduce otra autoridad
TF ni se conecta este tópico a IMU, compensación, control o EKF. Es pasivo y no
requiere ciclo de vida. Ausencia/gaps invalidan la captura física: no hay fallback
silencioso a /odom_raw. La velocidad del observador es derivada/filtrada por el
plugin; la pose física, no su twist filtrado, es referencia primaria de error.

La configuración EKF sigue intacta. Primero se toma una nueva referencia con
simulador corregido; cualquier variante se compara sobre exactamente esa base.
No atribuir la mejora de tracking del contrato sim a un ajuste del filtro.
IMU/rueda siguen correlacionadas por construcción; no se promueve tuning basado
en esconder ese error común. /odom_raw conserva su contrato histórico y queda
registrado como referencia cinemática adicional, no truth independiente.

## Experimentos de radio

Producto conserva radio Nav2 R4 y dirección automática18°. Por autorización
explícita del operador, un wrapper en tools permite usar el techo manual30°
exclusivamente con integration_sim. No se instala como bringup de producto.
El techo se verifica leyendo parámetros efectivos y dirección medida. El radio
central ideal L/tan30°≈1.63 m no garantiza que juntas interiores, clamp del plugin
o dinámica lo alcancen. R3/R2/R1.65 son solicitudes; se reporta el radio observado.
Ensayos sostenidos al máximo son del estimador y no acreditan admisión Nav2.
Guard de U/rulos, safety y límites mecánicos permanecen activos.

## Verificación y límites

Regresión velocidad×signo falla16 casos antes de corregir unidades. Test de
modelo/bridge verifica claves, dimensiones, tópico observador y ausencia de
TF público. Pilotos con pose física verifican captura, frames, frescura y
parámetros; luego campaña pareada y smokes/build/suite/CI. No hay Jetson,
despliegue ni evidencia física. Resultados en ekf-turns-20261018.
