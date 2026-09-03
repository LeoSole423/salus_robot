# Ficha de intención — owner MAVROS real reproducible (#170)

## Alcance

Este corte prepara sólo el owner MAVROS sensor-only del Pixhawk para el MVP.
No se ejecuta contra hardware y no incorpora NTRIP, entrega RTCM, RS16, UART,
localización, TF autoritativo, Nav2 ni control.

## Referencia caracterizada

La configuración se porta de
`ROS2_SALUS@f35834989b041f51dd325c626d2338e2232d9e53`:

- FCU configurable, por defecto `/dev/ttyACM0:921600`;
- MAVLink `v2.0`, target system/component `1/1`, GCS vacío y namespace raíz;
- allowlist: `sys_status`, `sys_time`, `imu`, `global_position`,
  `local_position`, `gps_status`, `gps_rtk`;
- remaps a `/imu/data`, `/global_position/raw/fix`,
  `/local_position/velocity_local` y `/local_position/odom`;
- TF de `global_position` y `local_position`, incluido FCU TF, deshabilitado.

La imagen instala `ros-humble-mavros`, `ros-humble-mavros-extras` y
`geographiclib-tools`; ejecuta el instalador de datasets incluido en MAVROS
como root, sin descargar un script externo.

## Ownership y seguridad

`pixhawk_real.launch.py` contiene exactamente una instancia de
`mavros/mavros_node`. Es un owner físico cutover-only: no puede coexistir con
el MAVROS legacy. La frontera lógica existente
`pixhawk_sensor_inputs.launch.py` no se modifica y se compondrá después con
este owner.

El launch no abre UART del ESP32, no crea un cliente NTRIP ni una segunda
entrega RTCM, no inicia RS16 y no publica TF propio. El perfil
`real_observation.launch.py` conserva explícitamente la ausencia de MAVROS.

## Validación y límite

Las pruebas locales verifican el recipe de imagen, YAML exacto, TF deshabilitado
y la composición de un único nodo sin abrir el FCU. La build de imagen valida
que MAVROS y GeographicLib se instalen. Quedan pendientes, con el legacy
detenido, la conexión a Pixhawk, IMU/GNSS/GPSRAW frescos y la coexistencia con
los adaptadores lógicos sin crear una segunda autoridad TF.
