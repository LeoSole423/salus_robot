# Intención: mediciones de hardware y odometría

## Hechos caracterizados

| Dato actual | Observación física | Limitación |
| --- | --- | --- |
| Hall de tracción por UART | giro de motor/transmisión convertido a velocidad lineal | no observa suelo, patinamiento ni sentido físico |
| AS5600 por UART | posición centrada del mecanismo de dirección | requiere curva calibrada para representar rueda efectiva |
| `reverse_requested` | intención de mando | no es una medición y no demuestra movimiento |
| `/wheel/odometry` | integración Ackermann de velocidad y dirección | es una estimación derivada, no "odometría de sensor" genérica |
| MAVROS GNSS/IMU | datos normalizados desde Pixhawk | el transporte no debe ser visible para localización/Nav2 |

La observación de la Jetson confirmó que el sistema instalado publica GNSS,
IMU, nube/scan y `DriveTelemetry`, y deriva `/wheel/odometry`. No se ordenó
movimiento ni se alteró el bringup real durante la caracterización.

## Contrato del primer corte

`MeasurementMetadata` identifica de manera estable la fuente, el instante de
observación, su secuencia y estado. Cada mensaje de medición declara por campo
si el valor fue medido, calculado o inferido mediante máscaras disjuntas y
exhaustivas. Los nombres no contienen
`/dev/tty*`, IDs USB ni detalles MAVLink: esos pertenecen a configuración del
adaptador.

`TractionMeasurement` representa una muestra de exactamente uno de estos
dominios: eje de motor, rueda de tracción o velocidad del vehículo respecto del
suelo. `SteeringMeasurement` representa eje del motor de dirección, mecanismo,
rueda real o ángulo central virtual. Múltiples sensores publican instancias con
`source_id` distintos; no se promedian dentro del driver. Una misma muestra
puede así contener posición medida y velocidad calculada sin falsear ninguna.

El primer adaptador de compatibilidad es de sólo lectura:
`legacy_drive_measurement_node` consume `/controller/drive_telemetry` y
publica por defecto `/vehicle/measurements/traction` y
`/vehicle/measurements/steering`. Sus tópicos y `source_id` son parámetros.
Representa la velocidad del Hall como `SOURCE_MOTOR_SHAFT`, conserva el
timestamp y marca la conversión de grados de enlace a radianes como calculada.
Como reconstruye ambos signos desde el estado booleano `reverse_requested`, la
velocidad lineal queda marcada como inferida tanto en avance como en reversa,
nunca medida. No incorpora launches, comandos ni acceso a
hardware. Las magnitudes inválidas no tienen bit disponible; una muestra no
fresca queda explícitamente `STALE`.

## Separación obligatoria

```text
protocolo/dispositivo
        -> medición física normalizada
        -> conversión mecánica calibrada
        -> odometría cinemática de ruedas
        -> fusión de localización
```

Cada flecha cambia de responsabilidad y debe poder probarse sin la siguiente.
GNSS, IMU y LiDAR siguen el mismo principio, reutilizando mensajes estándar.
El backend Pixhawk y los backends directos son opciones pares; un perfil de
bringup elige productores concretos y capacidades requeridas.

## Conversión cinemática calibrada

El segundo corte incorpora un nodo de conversión, todavía sin conectar
odometría. Consume por defecto `/vehicle/measurements/traction` y
`/vehicle/measurements/steering`; publica
`/vehicle/kinematic_inputs/traction` como `SOURCE_DRIVE_WHEEL` y
`/vehicle/kinematic_inputs/steering` como `SOURCE_VIRTUAL_CENTER_WHEEL`.
Todos los tópicos y los identificadores de fuente son parámetros. Una instancia
acepta exactamente un `source_id` de tracción y uno de dirección; para dos
esquemas físicos se ejecutan dos instancias o se elige una en bringup.

La tracción aplica `output = input * traction_linear_scale`. La dirección
aplica `output = c0 + c1*x + ... + cn*x^n`, donde `x` y el resultado están en
radianes, y limita el resultado a `steering_limit_rad`. La lista de
coeficientes debe tener entre 1 y 6 elementos finitos; el factor de tracción y
el límite deben ser finitos y estrictamente positivos. Estos parámetros son
calibración, no tuning de Nav2.

`calibration_validated` es `false` por defecto. Mientras sea falso, las salidas
son `UNAVAILABLE`, sin bits disponibles y con valores `NaN`. Un perfil de
simulación o hardware sólo puede activarlo explícitamente junto con sus
coeficientes. Las entradas con tipo físico incorrecto, `source_id` distinto,
campo ausente o valor no finito no producen un campo válido. `STALE`, `INVALID`
y `UNAVAILABLE` se conservan sin inventar datos.

Para una conversión válida, timestamp y secuencia se copian. Si el campo de
entrada era inferido, la salida también es inferida; si era medido o calculado,
la salida queda calculada. La salida usa `base_footprint` como frame por defecto,
configurable. QoS permanece reliable/volatile con profundidad 10 para ser
compatible con el corte `DriveTelemetry` actual.

Este corte no inicia el conversor desde un launch completo, no publica
`nav_msgs/Odometry`, no fusiona fuentes y no tiene autoridad de control. El
siguiente corte migrará `ackermann_odometry` para consumir exclusivamente estas
entradas cinemáticas y rechazar pares desincronizados.

## Evidencia pendiente

- medir pulsos/revolución, relación total y radio efectivo bajo carga;
- caracterizar signo físico sin deducirlo del comando;
- obtener la curva AS5600 -> mecanismo -> ángulo efectivo de rueda;
- registrar bags estacionarios y en movimiento con timestamps comparables;
- validar GNSS/IMU vía Pixhawk y directos como perfiles independientes;
- definir y probar watchdogs antes de conectar la salida normalizada a motores.
