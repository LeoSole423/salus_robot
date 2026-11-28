# Captura de incidentes de navegación

El grabador observa el grafo ROS y escribe segmentos SQLite3 de 30 segundos.
Conserva hasta 15 minutos previos y 2 posteriores al primer disparo. Los
disparos siguientes se agrupan durante el mismo intervalo posterior. El
manifiesto identifica el tramo disponible, las pérdidas de cola, los huecos,
los tópicos críticos ausentes y la versión y configuración capturadas.

## Simulación

```bash
ros2 launch salus_bringup sim_operational.launch.py headless:=true incident_recording:=true
```

El directorio de trabajo predeterminado es `runtime/incidents`. El lanzamiento
`integration_sim.launch.py` y el perfil `real_mvp.launch.py` aceptan
`incident_root` para elegir otro directorio y `incident_context_json` para
informar explícitamente `profile` y `datum` como `[latitud, longitud, yaw]`.
Use `SOURCE_SHA` para fijar el SHA cuando no hay checkout accesible. Los
parámetros efectivos se consultan periódicamente mediante servicios ROS de
lectura de los nodos de navegación y localización, sobre una lista permitida.
Las rutas planificadas y la misión activa se conservan como mensajes ROS.

El directorio debe estar en almacenamiento persistente local y compartirse con
el gateway. Para descargar desde Cockpit, configure el proxy privado de
grabaciones mediante `recordings_config`. El enlace caduca a los cinco minutos.
El archivo TAR incluye el manifiesto y los bags. El video se referencia mediante
la cámara y el intervalo UTC del grabador existente, sujeto a su propia retención.

| Parámetro del grabador | Valor inicial | Unidad y rango |
|---|---:|---|
| `segment_s` | 30 | segundos, entero positivo |
| `pre_s` y `post_s` | 900 y 120 | segundos, enteros positivos |
| `ring_bytes` | 2147483648 | bytes, entero positivo |
| `archive_bytes` | 10737418240 | bytes, entero positivo |
| `minimum_free_bytes` | 10737418240 | bytes, entero positivo |
| `steering_jump_per_m` | 0,18 | m⁻¹, real positivo finito |
| `costmap_max_hz` | 0,5 | Hz de mapas completos por tópico, 0 registra todos |
| `root` | `runtime/incidents` | directorio local escribible |
| `context_json` | vacío | archivo JSON opcional |

El grabador limita la cola a 2048 mensajes y 32 MiB de datos serializados, con
un máximo de 64 MiB de mensajes por segmento. En rotación se evalúan las cuotas y
el espacio libre. El descarte por cola se declara como captura parcial. Como
la entrega DDS ocurre antes de la cola, las pérdidas DDS detectables por huecos
siguen siendo relevantes al analizar la captura. La nube 3D y el video quedan
fuera de los bags.

## Reproducción y evaluación

Extraiga el TAR y ejecute lo siguiente con el entorno Humble y el workspace
cargados. La carpeta de salida debe ser nueva. El replay configura dominio 197
y transporte local. RViz muestra plan, tramo, escaneo, costmap y odometría. Los
comandos se observan en `/incident_replay/cmd_vel_final`.

```bash
ros2 run salus_incident incident_replay ruta/al/incidente ruta/al/bag --rviz
```

Sin `--rviz` se genera el bag. `--play` inicia la reproducción sin visor.
Para una prueba sin hardware, `tools/incident_sim_fixture.py` genera y verifica
una ventana de 17 minutos de tiempo ROS con el salto angular observado.

Ejecute el smoke de ruta dos veces, con el mismo escenario y semilla. Use
`SMOKE_INCIDENT_MEASURE=true` en ambas y `SMOKE_INCIDENT_RECORDING=true` en
la segunda. El comparador toma ambos archivos `incident_load.json`.

```bash
python3 tools/incident_load_probe.py --compare baseline.json con-grabador.json
```

Antes de habilitar el grabador durante una patrulla real, repetir la medición
de carga y confirmar que se conservan los tópicos y el intervalo esperados.
