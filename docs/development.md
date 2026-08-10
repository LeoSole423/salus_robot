# Desarrollo Y Depuración

## Ciclo corto

```bash
./tools/up.sh
./tools/build.sh
./tools/test.sh
```

Para trabajar dentro del entorno:

```bash
./tools/shell.sh
colcon list
colcon build --packages-select <paquete> --symlink-install
colcon test --packages-select <paquete>
colcon test-result --verbose
```

## Reglas de depuración

1. Reproducir con el paquete y sus entradas mínimas.
2. Separar fallo de sensor, transporte, algoritmo, composición y UI.
3. Capturar parámetros efectivos, tipos, QoS y timestamps.
4. Agregar primero un test que reproduzca el fallo.
5. Verificar el backend simulado antes de probar hardware cuando sea posible.
6. No ajustar múltiples capas a la vez sin guardar una línea base.

## Convenciones

- Python: módulos pequeños, lógica pura separada del nodo ROS y tests `pytest`.
- C++: warnings habilitados, headers públicos mínimos y tests unitarios.
- Launch: composición parcial en el paquete; composición completa en bringup.
- Configuración: YAML por responsabilidad, sin duplicar defaults arbitrariamente.
- Interfaces: comentarios de unidad/semántica y compatibilidad registrada.

## Añadir un paquete o dependencia

- Actualizar `docs/package-map.yaml` y este documento si cambia el flujo.
- Preferir `rosdep`; usar `dependencies.repos` solo para fuentes no disponibles.
- Fijar cada repositorio externo a un commit, nunca a una rama flotante.
- Registrar decisiones que afecten más de un paquete en un ADR.

## Seguridad

Los launches skeleton no controlan hardware. Cuando aparezcan launches reales,
su ejecución requerirá autorización explícita, zona despejada y E-stop
accesible. Ningún test automático debe mover actuadores físicos.

