# Ficha de intención: perfil canónico de E/S vehicular en simulación

## Alcance

Integrar en una composición reproducible de simulación la cadena ya portada:

```text
DriveTelemetry
  -> LegacyDriveMeasurementNode
  -> TractionMeasurement + SteeringMeasurement
  -> KinematicConversionNode
  -> entradas cinemáticas calibradas
  -> KinematicAckermannOdometryNode
  -> /wheel/odometry + /vehicle/twist
```

Este corte no modifica el backend de control, no activa hardware real y no
declara válida ninguna calibración del robot físico.

## Hechos observados

- `control_sim.launch.py` publica `DriveTelemetry` con velocidad ya expresada
  en m/s.
- El backend simulado invierte por defecto el signo de la dirección medida para
  reproducir la convención histórica del controlador.
- `localization_sim.launch.py` compensa actualmente ese signo mediante
  `invert_measured_steer_sign=true` en `ackermann_odometry`.
- El EKF consume `/wheel/odometry` y es la única autoridad de TF
  `odom -> base_footprint`.
- El adaptador, el conversor calibrado y la odometría canónica ya existen y
  cuentan con pruebas unitarias; todavía no forman parte de un perfil completo.

## Decisión e invariantes

- `vehicle_io_profile` admite exclusivamente `legacy` o `canonical` y conserva
  `legacy` como valor por defecto durante la transición.
- En todo perfil existe exactamente un publicador principal de
  `/wheel/odometry` y `/vehicle/twist`.
- El perfil `canonical` usa una calibración exclusiva de simulación:
  escala de tracción `1.0` y polinomio de dirección `[0.0, -1.0]`. Esto valida
  la semántica del modelo simulado, no el hardware instalado.
- La comparación histórica es optativa y sólo válida junto a `canonical`.
  Publica `/comparison/legacy/wheel_odometry` y
  `/comparison/legacy/vehicle_twist`; no alimenta al EKF ni publica TF.
- Un selector desconocido, o pedir comparación bajo `legacy`, debe fallar al
  construir el launch en lugar de degradar silenciosamente.
- El perfil histórico mantiene sus parámetros y tópicos principales actuales.

## Pruebas y evidencia esperada

- Tests de estructura y validación de launch para selección y exclusión mutua.
- Build y tests focalizados de `salus_bringup` y `salus_localization`.
- Smoke de localización canónica que pruebe mensajes válidos, desplazamiento,
  giro, parada y una sola autoridad sobre `/wheel/odometry`.
- Suite y CI completos antes de fusionar.

## Pendiente fuera de este corte

- Calibrar signo, relación mecánica, radio efectivo y curva de dirección en
  banco o robot.
- Definir perfiles reales Pixhawk y sensores directos a Jetson.
- Comparación reproducible con bag o hardware antes de declarar paridad.

