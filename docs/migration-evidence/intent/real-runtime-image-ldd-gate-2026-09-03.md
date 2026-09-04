# Intención: reproducibilidad del runtime real y gate `ldd` para RS16

## Alcance

- Problema: `rslidar_sdk_node` no cargó `libpcap.so.0.8` durante #190.
- Destino: `salus-robot:humble-real` y la validación software-only del
  workspace físico.
- Incluido: provenance de imagen, runtime loader, build del workspace y gate
  fail-closed de shared libraries.
- Fuera de alcance: RS16 físico, percepción, TF, configuración LiDAR y
  cualquier servicio o nodo legacy.

## Causa determinada

La imagen ARM64 usada en #190 era stale. Su ID era
`sha256:092701a121ff81dc631b287c8555b7d50b68d4cd627ea926051519183b9eab9c`,
creada a las `2026-09-03T13:43:10-03:00`. El `Dockerfile.real` vigente en
`1eb47c9` desde las `14:57:43-03:00` ya declaraba `libpcap-dev`, pero la
imagen inspeccionada no tenía `libpcap-dev`, `libpcap0.8`, entradas de
`ldconfig` ni archivos `libpcap.so*`.

En Jetson, `uname -m` fue `aarch64` y la imagen reportó arquitectura `arm64`
sobre Ubuntu Jammy. El `ldd` del `rslidar_sdk_node` montado dentro de esa
misma imagen mostró `libpcap.so.0.8 => not found`; el contenedor legacy sí lo
resolvía desde `/lib/aarch64-linux-gnu/libpcap.so.0.8`.

La evidencia no indica que falte una declaración en la receta actual. La
corrección es reconstruir la imagen actual y verificar el boundary del loader;
no se agrega una dependencia redundante.

## Gate nuevo

`tools/validate_real_runtime_image.sh`:

- inspecciona ID, fecha, arquitectura y OS de la imagen seleccionada;
- monta `src/` read-only y usa `tmpfs` para build/install/log;
- construye el workspace dentro de `salus-robot:humble-real`;
- encuentra `rslidar_sdk_node` mediante el índice ROS;
- ejecuta `ldd` sin modificar `LD_LIBRARY_PATH`;
- falla ante cualquier `not found`;
- exige que `libpcap.so.0.8` resuelva a un path real;
- usa `--network none` y no abre dispositivos ni inicia el nodo.

## Validación hasta el corte

- Diagnóstico Jetson: ejecutado con legacy activo; no se detuvo el servicio y
  no se inició RS16.
- Rebuild ARM64 completado en la Jetson con la receta vigente, sin cambios al
  Dockerfile. El nuevo ID es
  `sha256:a52b2fe21737523b0915e9d1e5049f88a4dca73c4131e51538211b71abaaaec4`,
  creado a las `2026-09-03T16:52:53-03:00`, `arm64/linux`.
- La inspección post-build confirmó `libpcap-dev` y `libpcap0.8`, ambos
  `1.10.1-4ubuntu1.22.04.1`; `ldconfig` resolvió
  `/lib/aarch64-linux-gnu/libpcap.so.0.8`.
- El build ARM64 completo del workspace terminó con 16 paquetes, incluido
  `rslidar_sdk`. El gate específico `--packages-up-to rslidar_sdk` también
  compiló `rslidar_msg` y `rslidar_sdk`.
- El `ldd` final quedó pendiente: la ejecución se detuvo al comprobar la
  legibilidad del symlink instalado antes de imprimirlo. El commit contiene
  el ajuste para continuar con `ldd`; debe relanzarse una vez tras este corte.
- El gate no inició nodos, no abrió dispositivos y usó `--network none`.
- Tests estáticos: no se pudieron ejecutar en el host porque el Python
  disponible no tiene `pytest`; `bash -n tools/validate_real_runtime_image.sh`
  pasó.

## Estado

- Estado propuesto: `in_progress`; el rebuild ARM64 y la presencia del runtime
  están demostrados, pero falta la salida final de `ldd` sin `not found`.
- No validado: funcionamiento físico RS16 y pipeline `/scan_clean`.
- No se modifica `rs16.yaml`, percepción, TF, legacy ni hardware.
