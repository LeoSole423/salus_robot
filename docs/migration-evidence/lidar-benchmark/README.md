# Detector radial de suelo del RS16 en CPU y CUDA

Se comparó el filtro urbano desplegado en `a556832` con prototipos offline.
Los tres bags de septiembre de 2026 aportaron `/scan_3d` crudo y `/tf_static`.
El algoritmo conserva los límites de pendiente global y local de 10° y 13°.
La medición incluye deserialización, transformación, filtrado, transferencias
CUDA y serialización de salida. Excluye lectura de disco, búsqueda del TF
estático y publicación DDS. Se calienta cada variante por bag y se alterna el
orden de medición. La igualdad se evalúa sobre los mensajes deserializados.
Los bags originales y las dependencias binarias se conservan fuera de Git.

| Plataforma y variante | Nubes | Mediana | p95 | CPU por nube, mediana |
|---|---:|---:|---:|---:|
| RTX 4060 Laptop, Python | 2.903 | 31,73 ms | 39,69 ms | 31,74 ms |
| RTX 4060 Laptop, C++ | 2.903 | 3,74 ms | 4,62 ms | 3,73 ms |
| RTX 4060 Laptop, CUDA corregido | 2.903 | 2,62 ms | 3,13 ms | 2,61 ms |
| Orin Nano, Python | 120 | 61,75 ms | 80,60 ms | 56,57 ms |
| Orin Nano, C++ | 120 | 6,75 ms | 9,33 ms | 6,37 ms |
| Orin Nano, CUDA corregido | 120 | 13,00 ms | 16,48 ms | 7,08 ms |

El prototipo CUDA corregido transforma, ordena y clasifica en GPU. Recalcula
en CPU los sectores de los puntos limítrofes para conservar el redondeo de
NumPy. Antes de la corrección, siete nubes cambiaron de orden y seis cambiaron
sus puntos de obstáculo. Los rangos de la proyección 2D offline coincidieron
en esos seis casos. Después de la corrección, ambos candidatos coincidieron
con la referencia en cada nube de la comparación principal de cada equipo.

En la Orin se hicieron dos tandas con cuota de 0,35 núcleos y prioridad baja
(24 y 60 nubes) y una sin cuota ni pausa (120 nubes). El servicio real y el
detector de personas siguieron activos. La prueba no incluyó movimiento ni
medición de respuesta manual. C++ queda como primer candidato para reducir
CPU en la Orin. Los prototipos todavía no están integrados al nodo operativo.

## Reproducir

Dentro de un entorno ROS 2 Humble con Python 3.10 y acceso al driver CUDA,
usar un contenedor aislado de la red ROS del robot. Pasar bases SQLite3 con
`/scan_3d` y `/tf_static` de los bags de entrada. Para replicar la tanda Orin,
usar 40 nubes repartidas uniformemente de cada bag. En la RTX se usaron los
tres archivos completos. Los nombres están en los JSON adjuntos.

```bash
source /opt/ros/humble/setup.bash
repo="$PWD"
bench="$repo/tools/lidar_benchmark"
python3 -m pip install --target /tmp/salus-bench-deps -r "$bench/requirements.txt"
export PYTHONPATH="/tmp/salus-bench-deps:$repo/src/salus_perception:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/tmp/salus-bench-deps/nvidia/cuda_nvrtc/lib:/tmp/salus-bench-deps/nvidia/cuda_runtime/lib:${LD_LIBRARY_PATH:-}"
export OPENBLAS_NUM_THREADS=1
g++ -std=c++17 -O3 -ffp-contract=off -shared -fPIC -x c++ \
  "$bench/radial_probe.cu" -o /tmp/radial_probe.so
python3 "$bench/benchmark.py" bag1.db3 bag2.db3 bag3.db3 \
  --native-library /tmp/radial_probe.so --frames-per-bag 10000 \
  --methods cpu_native cuda_boundary --output /tmp/lidar-results.json
```
