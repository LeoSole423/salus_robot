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

La frontera cinemática acepta una fuente física configurada por instancia y
produce otra medición con semántica distinta. Motor/transmisión a rueda usa un
factor lineal explícito; mecanismo a rueda central virtual usa una curva
polinómica calibrada y un límite físico. La salida no conserva procedencia
`measured`: una entrada inferida sigue siendo inferida y cualquier otra salida
es calculada. Conserva timestamp y secuencia para mantener el linaje.

Una configuración nueva comienza como no validada. En ese estado el conversor
puede exponer `UNAVAILABLE`, pero no publicar un campo consumible. Habilitarla
es una decisión explícita del perfil después de validar unidades, signo y
calibración. Cada instancia filtra un `source_id` concreto; no selecciona ni
conmuta fuentes silenciosamente en runtime.

La odometría canónica consume un par explícito `SOURCE_DRIVE_WHEEL` y
`SOURCE_VIRTUAL_CENTER_WHEEL`, ambos `OK`, con el campo requerido disponible y
procedencia válida. Los `source_id` esperados son parámetros. El sincronizador
mantiene como máximo una muestra por fuente y sólo emite cuando ambas son
nuevas respecto del último par y sus timestamps difieren como máximo el umbral
configurado. No interpola, no reutiliza indefinidamente una dirección anterior
y no mezcla fuentes con IDs distintos.

Un mensaje seleccionado stale, inválido, no disponible, mal formado o con
timestamp no positivo invalida la muestra almacenada de esa fuente. Un timestamp
repetido o regresivo no publica; una regresión invalida además la base temporal.
Un salto de integración mayor al máximo establece un baseline nuevo y publica
pose conservada/twist actual sin integrar distancia. Así un reinicio de sensor,
reloj o rosbag no crea movimiento ficticio ni tiempo de odometría no monótono.
La secuencia se conserva para diagnóstico, pero el timestamp es la autoridad
temporal porque fuentes independientes no comparten contador.

Durante la transición, el bringup de simulación selecciona una única autoridad
mediante `vehicle_io_profile=legacy|canonical`, con `legacy` como default. El
perfil canónico compone adaptador, conversión y odometría; su calibración de
simulación (velocidad identidad e inversión del signo de dirección medido) no
es evidencia de calibración física. Una comparación opcional puede ejecutar la
odometría histórica sólo sobre tópicos `/comparison/legacy/*`; nunca comparte
los tópicos principales ni alimenta al EKF. Los valores de perfil inválidos
fallan explícitamente.

El timestamp del par es el más reciente de sus dos observaciones. La primera
pareja válida establece baseline y publica pose cero con twist válido; las
siguientes integran sólo con `0 < dt <= max_dt_s`. El nodo publica
`/wheel/odometry` y `/vehicle/twist`, pero nunca TF: el EKF continúa como única
autoridad de `odom -> base_footprint`.

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

La salida hacia actuadores usa `VehicleCommand` como sobre atómico a nivel
vehículo. Reutiliza `ackermann_msgs/AckermannDrive` para velocidad firmada,
aceleración, jerk, ángulo de la rueda central virtual y velocidad de dirección,
y añade vigencia, fuente, habilitación, E-stop explícito y freno de servicio
normalizado. E-stop y freno no son equivalentes. Porcentajes de dirección, PWM,
torque y bytes pertenecen al adaptador Pixhawk, UART/ESP32 u otro controlador.
El consumidor conserva un watchdog monotónico local y limita la vigencia pedida
por el productor. Este corte no habilita movimiento real ni reemplaza todavía
`CmdVelFinal` o el backend instalado.

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
