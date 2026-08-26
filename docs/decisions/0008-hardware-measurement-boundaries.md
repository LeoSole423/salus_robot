# ADR 0008: Fronteras de medición física, odometría y hardware

## Estado

Aceptado para la migración incremental; validación física pendiente.

## Contexto

El robot actual recibe GNSS, IMU y estado de vuelo mediante Pixhawk/MAVROS, y
telemetría de tracción y dirección mediante UART desde el controlador. La
arquitectura futura también debe admitir GNSS RTK USB directo a la Jetson, una
o más IMU en PCB y otros LiDAR, sin convertir a Pixhawk ni a un fabricante de
sensores en dependencias de navegación.

El contrato histórico `DriveTelemetry` mezcla estado del controlador con dos
magnitudes llamadas "medidas". En el hardware actual, los Hall observan el
motor/transmisión, no el desplazamiento del vehículo; el AS5600 observa el
mecanismo de dirección, no necesariamente el ángulo efectivo de las ruedas. La
odometría Ackermann además infiere el signo de avance desde el comando de
reversa. Esas cantidades no son equivalentes.

## Decisión

Los adaptadores terminan protocolos de fabricante y publican primero hechos
físicos normalizados. `TractionMeasurement` distingue eje de motor, rueda de
tracción y velocidad respecto del suelo. `SteeringMeasurement` distingue eje
de motor, mecanismo, rueda real y rueda central virtual del modelo bicicleta.
Cada muestra declara origen lógico, instante y estado. Para cada campo presente,
las máscaras disjuntas `measured_fields`, `calculated_fields` e
`inferred_fields` declaran su procedencia; su unión debe ser exactamente
`available_fields`. Una conversión nunca conserva la procedencia medida.

Los campos ausentes se expresan con `available_fields`; no se rellenan con cero.
Un valor sólo es consumible si su bit está presente, es finito y el estado es
`OK`. `STALE`, `INVALID` y `UNAVAILABLE` son estados explícitos. La dirección de
movimiento proviene del signo de una velocidad física; una orden de reversa no
puede presentarse como dirección medida.

Las conversiones mecánicas —pulsos a radianes, relación de transmisión, radio
de rueda y curva mecanismo-rueda— pertenecen a funciones puras configuradas y
probadas. La odometría de ruedas es una estimación derivada posterior y se
publica como `nav_msgs/Odometry`; no es un sensor ni una entrada genérica. La
localización fusionada permanece separada en `robot_localization` y conserva
`map -> odom -> base_footprint`.

IMU, GNSS, nubes, escaneos, estados articulares y comandos cinemáticos usan
mensajes ROS estándar cuando su semántica alcanza: `sensor_msgs/Imu`,
`NavSatFix`, `PointCloud2`, `LaserScan`, `JointState`, `nav_msgs/Odometry` y
`geometry_msgs/TwistStamped`. No se envuelve un mensaje estándar sólo para
renombrar hardware. Los diagnósticos de capacidad/salud serán ortogonales.

Pixhawk es un backend soportado de forma permanente, al mismo nivel que los
adaptadores directos. Ningún consumidor aguas arriba distingue si GNSS/IMU
llegaron por MAVROS, USB o una PCB. La selección de fuentes será explícita por
perfil; no habrá conmutación silenciosa entre dos fuentes con semánticas
distintas.

La ausencia de LiDAR es un perfil operativo válido sólo cuando la autonomía de
evitación de obstáculos está deshabilitada explícitamente. La pérdida de un
LiDAR requerido no transforma automáticamente el robot en ese perfil.

La salida hacia actuadores tendrá una frontera equivalente: comando cinemático
normalizado, arbitraje y límites de seguridad antes de un backend Pixhawk,
UART/ESP32 u otro controlador. Este corte no habilita movimiento real y no
reemplaza todavía `CmdVelFinal` ni el backend instalado.

## Consecuencias

- El hardware actual puede migrarse con adaptadores compatibles sin falsear la
  naturaleza de sus mediciones.
- Dos IMU, GNSS de doble antena o ruedas instrumentadas pueden coexistir como
  fuentes identificadas; la selección/fusión queda fuera del driver.
- Cambiar RS16 o RTK afecta el adaptador y la configuración, no Nav2 ni Cockpit.
- Los consumidores deben rechazar muestras viejas, inválidas o incompletas.
- `DriveTelemetry` se conserva durante la transición para estado y paridad; no
  será la interfaz canónica de odometría futura.
- La calibración real de signos, relación, radio efectivo y dirección sigue
  siendo requisito de hardware, no una constante asumida por este ADR.
