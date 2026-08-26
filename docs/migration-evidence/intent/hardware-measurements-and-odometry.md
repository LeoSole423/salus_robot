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

Los tópicos canónicos se fijarán junto con los adaptadores en el siguiente
corte. Hasta entonces, incorporar los mensajes no cambia el grafo desplegado.
La compatibilidad prevista usa un adaptador desde `DriveTelemetry`, marcando
como `CALCULATED` las conversiones y como `INFERRED` cualquier signo obtenido
de una orden. No se publicará una muestra canónica `OK` si falta una magnitud
necesaria.

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

## Evidencia pendiente

- medir pulsos/revolución, relación total y radio efectivo bajo carga;
- caracterizar signo físico sin deducirlo del comando;
- obtener la curva AS5600 -> mecanismo -> ángulo efectivo de rueda;
- registrar bags estacionarios y en movimiento con timestamps comparables;
- validar GNSS/IMU vía Pixhawk y directos como perfiles independientes;
- definir y probar watchdogs antes de conectar la salida normalizada a motores.
