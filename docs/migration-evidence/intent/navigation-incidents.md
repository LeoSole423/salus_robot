# Captura de incidentes de navegación

El 27 de noviembre de 2026, a las 21:30 hora de Córdoba, el controlador recibió
un cambio de velocidad angular de -0,028 a -0,361 rad/s con avance de 1,6 m/s.
La colisión prevista llegó unos 2,5 s después. El journal y el video permiten
ordenar ambos hechos. La trayectoria activa y la pose estimada no se archivaron.

El grabador es un observador independiente. Conserva mensajes ROS serializados,
contexto seleccionado de Nav2 y un vínculo temporal al video existente. El
historial identifica el incidente sin almacenar el bag en la base operativa.
La herramienta offline preserva las marcas de origen y remapea las órdenes.
Esto permite comparar planes, localización y control durante la reproducción.

El caso sintético conserva los dos comandos medidos y su separación de 31,87 ms.
La trayectoria, las poses y el entorno de ese caso son sintéticos. La
validación inicial corresponde a simulación y reproducción offline.

## Evidencia de simulación

- El caso sintético conserva 1020 segundos y permite leer un único bag SQLite3
  con las órdenes remapeadas. El smoke estructural y el de ruta completaron.
- El primer ensayo sin limitar los costmaps excedió el objetivo de intervalo
  entre comandos. Tras limitar los mapas completos a 0,5 Hz, el percentil 99
  pasó de 72,2 a 75,5 ms al activar la captura. El grabador usó 0,67 núcleos
  en el percentil 95 y 89,4 MiB de RSS máximo. Ambos recorridos terminaron sin
  avisos de incumplimiento del lazo de control.
- La comparación corresponde a una misión simulada de laboratorio. La carga
  sostenida y la cobertura funcionan como criterios de aceptación para una
  ventana física posterior.
