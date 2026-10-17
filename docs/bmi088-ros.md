# BMI088 en ROS: observación y comparación con Pixhawk

Entrega 2026-10-15. Implementación de software; **sin comparación física ni
cambio de la fuente operativa**. GNSS/RTCM por Pixhawk continúa fuera de este
cambio. La PCB actual sigue usando adquisición independiente, IMU-SYNC-001
diferido. ADR: [0015](decisions/0015-bmi088-observation-usb-timing.md).

## Qué está disponible

- Adquisición S3 con API Bosch, cuentas originales, SI y tiempos DRDY separados.
- Herramientas de calibración v1 en el repositorio `salus-esp32s3-fw`.
- Consulta USB temporizada, lector ROS exclusivo, aplicación del perfil y
  publicación de `sensor_msgs/Imu`/`Temperature`.
- Comparación de giróscopos en un marco común mediante TF, con métricas en ventana.

El observador **no escribe comandos de tracción**: solo diagnóstico opcode 03.
No inicia `controller_server`, MAVROS, EKF, selectores ni otros sensores. No
publica `/salus/imu/data` ni cambia `/hardware/imu_primary/data`. El firmware
mantiene READY=0 y actuadores sin implementar. No declarar paridad por un smoke.

## USB y marcas de tiempo

Requiere firmware S3 de esta entrega, con diagnóstico tipo 03; no usar el
snapshot tipo 02 antiguo para fechar mensajes ROS con garantías de frescura.

Solicitud de 7 bytes: `AB 10 03 token0 token1 token2 CRC8`. Token uint24 LE.
Solo opcode 03 admite token en los tres bytes reservados; opcodes 01/02 y
tramas v2 conservan exactamente sus contratos. El firmware no renueva la
frescura de control al recibir ninguna consulta de IMU.

Respuesta tipo 03, 128 bytes: layout del snapshot 02 hasta offset 110, cambiando
header a `5A 10 03 80`; luego:

| Offset | Campo |
| --- | --- |
| 111..113 | Token solicitado uint24 LE |
| 114..117 | Boot ID uint32 aleatorio al iniciar la aplicación |
| 118..125 | Tiempo MCU al completar recepción de la solicitud, uint64 µs |
| 126 | Reservado cero |
| 127 | CRC8 clásico de bytes 0..126 |

El offset 100..107 conserva el tiempo MCU de captura de respuesta. El boot ID
identifica sesiones, **no una placa física persistente**. Los IDs BMI088 tampoco
son números de serie únicos; la etiqueta `device_id` sigue siendo declarada por
el operador y debe corresponder a la placa calibrada.

El host mide envío/recepción con reloj monotónico. Se estima offset mediante
los cuatro tiempos host-envío, MCU-recepción, MCU-captura, host-recepción:
`offset=((MCU_rx-host_tx)+(MCU_tx-host_rx))/2`. Se convierte cada DRDY al reloj
ROS usando el ROS/monotónico del host al recibir la respuesta. Se informa la
incertidumbre de transporte `(RTT-tiempo_en_MCU)/2`; no es una especificación de
precisión completa ni sincronización física de los sensores.

Se rechazan tokens antiguos, CRC, perfiles/IDs inválidos, valores no finitos,
cronología imposible, RTT excesivo, datos vencidos, saturados y duplicados.
Cambio de boot/init permite reiniciar secuencias conservando monotonicidad de
publicación. Un salto ROS/monotónico mayor a 50 ms rechaza la muestra; si el
reloj retrocede no se publican stamps anteriores a los ya emitidos. No se usa
`use_sim_time` con este nodo físico.

Los tiempos DRDY incluyen el retardo de filtrado del sensor. Los parámetros de
compensación tienen default cero deliberado: medir/confirmar retardo antes de
configurarlos. El estado informa si se configuró compensación. No se interpolan
acelerómetro y giróscopo ni se presentan como simultáneos.

## Contratos ROS

Prefijo `/hardware/imu_bmi088/`; productor `salus_hardware/bmi088_observer`.
Los mensajes de sensores son volátiles, BEST_EFFORT, depth 5 (sensor_data QoS),
sin lifecycle administrado. Estado/comparación: RELIABLE, depth 10, volátiles.
No hay republicación de último valor como si fuese una medición nueva.

| Tópico | Tipo | Contenido / consumidor |
| --- | --- | --- |
| `data_raw` | `sensor_msgs/Imu` | Giro SI original XYZ; diagnóstico/bag |
| `data` | `sensor_msgs/Imu` | Giro corregido si hay perfil gyro; comparador y futura selección local |
| `accel_raw` | `sensor_msgs/Imu` | Aceleración específica original, gravedad conservada; bag |
| `accel` | `sensor_msgs/Imu` | Aceleración corregida si hay perfil accel; diagnóstico/fusión futura |
| `temperature` | `sensor_msgs/Temperature` | Temperatura de sensor, sin inventar temperatura ambiente |
| `status` | `std_msgs/String` JSON | Frescura por sensor, calibración, sesión, pérdidas, RTT/incertidumbre y razones |
| `comparison` | `std_msgs/String` JSON | Métricas giro XYZ, error medio/RMS, desfase de pares y frescura |

Todos los Imu tienen orientación no disponible (`orientation_covariance[0]=-1`).
Mensajes de giro marcan aceleración no disponible; mensajes de aceleración marcan
giro no disponible. El frame es `bmi088_link`, frame físico del sensor, sin
rotación manual o relabel al vehículo. Las correcciones intrínsecas se aplican
en ese frame. Perfil gyro-only o ausencia de perfil son admisibles para observar;
el estado indica qué se corrigió. No son autorización de cutover.

Las varianzas usan máximo entre el piso configurado y la dispersión del perfil.
Los pisos iniciales son conservadores/configurables, no mediciones físicas del
robot; hay que validarlos antes de fusionar. Las varianzas de calibración no
incluyen toda la incertidumbre de montaje, vibración o tiempo. La temperatura
se publica una vez por nueva medición MCU; no se duplica por jitter de clock.

El comparador usa por defecto `/hardware/imu_primary/data` del adaptador Pixhawk
y `/hardware/imu_bmi088/data`. Necesita TF de ambos frames a `base_footprint`.
Rota los vectores, empareja mediciones una a una por timestamp (≤30 ms), descarta
las vencidas (>200 ms) y calcula métricas de cinco segundos. No compara yaw de
orientación ni estima rumbo absoluto. Sin TF, referencia o pares frescos, el
estado dice que no puede comparar; no convierte muestras ausentes en error cero.
`hardware_parity` permanece false: los números no son un gate automático de uso.

## Parámetros del observador

Todos son parámetros ROS declarados, solo lectura al arrancar; reiniciar nodo
para cambiar perfil/configuración. `calibration_path` JSON no admite NaN ni
claves duplicadas y debe coincidir con schema v1, etiqueta y adquisición perfil 1.

| Parámetro | Tipo / default | Rango / significado |
| --- | --- | --- |
| `port` | string, vacío | Requerido; ruta explícita S3, no `auto` |
| `device_id` | string, vacío | Etiqueta requerida si se carga calibración |
| `calibration_path` | string, vacío | Vacío=observación sin corrección; path accesible en host/contenedor |
| `frame_id` | string, `bmi088_link` | Frame dedicado no vacío; no base_link/base_footprint/map/odom |
| `poll_hz` | double, 100 | 1..200 Hz, solicitudes nominales; no garantiza tasa física |
| `max_rtt_s` | double, 0.02 | 0.001..0.1 s, rechazo de intercambio lento |
| `max_sample_age_s` | double, 0.1 | 0.01..0.2 s, edad ROS máxima al publicar |
| `status_timeout_s` | double, 0.2 | 0.02..5 s, pérdida de frescura sin datos válidos |
| `gyro_filter_delay_s` | double, 0 | 0..0.05 s, compensación de retardo a verificar |
| `accel_filter_delay_s` | double, 0 | 0..0.05 s, compensación independiente a verificar |
| `gyro_variance_floor` | double, 0.0001 | 1e-12..1 (rad/s)², piso diagonal |
| `accel_variance_floor` | double, 0.01 | 1e-12..100 (m/s²)², piso diagonal |

Comparador: `candidate_topic`/`reference_topic` como arriba, `target_frame` string
`base_footprint`; intervalos double `max_pair_dt_s=0.03` (0.001..0.1),
`max_age_s=0.2` (0.01..2), `window_s=5.0` (0.1..60), todos en segundos.

## Uso posterior en banco/Jetson

No se ejecutaron estas acciones con hardware. Programar el firmware actualizado
requiere alimentación externa y coordinación del operador. El observador es el
**único lector del S3**: no abrir simultáneamente monitor, herramienta de
calibración o cliente de control clásico en ese mismo puerto. El Pixhawk usa
otro puerto y puede seguir publicando su referencia habitual.

Con Humble y el workspace construidos, publicar primero sin seleccionar la IMU:

```bash
ros2 run salus_hardware bmi088_observer --ros-args \
  -p port:=/dev/serial/by-id/IDENTIDAD_S3 \
  -p device_id:=SALUS-V3-001 \
  -p calibration_path:=/ruta/al/perfil/full-v1.json
```

Para composición de observación/comparación:

```bash
ros2 launch salus_bringup imu_comparison.launch.py \
  port:=/dev/serial/by-id/IDENTIDAD_S3 device_id:=SALUS-V3-001 \
  calibration_path:=/ruta/al/perfil/full-v1.json
```

El launch no inicia Pixhawk/MAVROS ni el adaptador primario. La referencia debe
estar publicándose; puede cambiarse mediante `reference_topic`. Tampoco publica
TF de montaje por defecto. Si ya existe TF correcta, reutilizarla. Para un
montaje nuevo, declararlo explícitamente con `mount_configured:=true` y
`mount_x/y/z` en metros, `mount_roll/pitch/yaw` en radianes. Los defaults cero
solo son válidos si ese montaje es realmente el declarado para el banco.
No duplicar una TF existente ni sustituir el montaje del Pixhawk por el del S3.

Consultar status/comparison y grabar ambas fuentes, temperatura, `/tf` y
`/tf_static` con rosbag2. La comparación de giro debe incluir quietud y, cuando
se autorice, giros de ambos signos; revisar bias, ruido, escala, signos, retardos,
pérdidas, reinicios y calidad en temperatura. No habilitar movimiento mediante
este launch. El cambio de fuente al EKF es una intervención posterior explícita.

## Qué falta para reemplazar Pixhawk

| Frontera | Estado |
| --- | --- |
| Driver y diagnósticos BMI088 | Implementados, host simulado/build; hardware pendiente |
| Herramientas de calibración y lectura del perfil ROS | Implementadas; calibración física pendiente |
| ROS y comparación sin cambiar autoridad | Implementados; smoke con USB/Pixhawk virtuales |
| Montaje/TF, sesgo, temperatura y vibración reales | Pendientes |
| Latencia, tasa y compensación de filtro reales | Pendientes |
| Cutover de la entrada de giro del EKF | Pendiente de evidencia y selección explícita |
| USB compartido por control e IMU continua | Gateway único/multiplexación de alta tasa pendiente; no usar dos lectores |
| GNSS USB directo y entrega RTCM | Pendientes, necesarios para retirar físicamente Pixhawk |
| Orientación fusionada/rumbo inmóvil independiente | Fuera de esta entrega; se conserva rumbo GNSS actual |

Esta entrega ofrece la salida ROS para **medir y comparar**, no declara que ya
está todo validado para retirar Pixhawk. La adquisición física es 400 Hz; este
backend de observación consulta hasta la tasa configurada y no recupera muestras
intermedias. El flujo continuo multiplexado de alta tasa sigue como corte aparte.

## Evidencia reproducible sin placa

Pruebas en `salus_hardware/test/test_bmi088_*.py`: codec, reloj, saturación,
calibración, duplicados, boot/reset, timestamps, orientación no disponible y
frescura. `test_bmi088_serial.py` usa pseudo-terminal/pyserial real, ROS/DDS,
referencia sintética y TF estática: verifica comparación y pérdida de respuestas,
y comprueba que todos los envíos sean diagnóstico, nunca control.

`salus-esp32s3-fw/tools/test_imu.py` acepta como argumento opcional la ruta de
`bmi088_domain.py` y contrasta el paquete tipo 03 generado por el C nativo con
este codec Python. No depende de que exista el repositorio hermano en runtime.

Resultados locales: build Humble de 16 paquetes correcto; suite ROS 1308 tests,
0 errores/0 fallos/1 omitido; 36 tests de helpers de host, 12 de calibración,
regresiones nativas y smoke de localización correctos. Once tests nuevos cubren
la frontera de observación/comparación. El verificador del checkout completo
sigue detenido por enlaces/rutas de `.agents/` preexistente y no versionado;
el árbol fuente previsto, sin esa carpeta ajena, pasó validación (14 paquetes).
No se publicaron cambios ni se ejecutaron acciones contra el robot.
