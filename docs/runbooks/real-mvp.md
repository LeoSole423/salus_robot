# Runtime real MVP: preparación y rollback

Este runbook instala el servicio final de `salus_robot` sin reemplazar el
servicio legacy. La aceptación física de #168 se ejecuta después del merge y
requiere operador, E-stop y robot inmovilizado; este documento no autoriza por
sí mismo movimiento ni pruebas de hardware.

## Preparación

1. Instalar un checkout limpio en `/opt/salus_robot`, propiedad del usuario
   operativo validado `admin`, y mantenerlo en `main` o en el SHA aprobado. El
   usuario debe tener acceso al grupo `docker` y a los devices mediante
   `dialout`.
2. Ejecutar como `admin` desde el checkout, conservando su `HOME` y UID/GID
   para reutilizar el cache existente, sin crear una cuenta ni un cache
   paralelo:

   ```bash
   ./tools/prepare_real_runtime.sh
   ```

   No usar `--force-*` salvo una decisión operativa explícita. El script
   conserva provenance, imagen, dependencias y workspace en la caché del mismo
   usuario.

3. Crear la configuración NTRIP privada únicamente en:

   ```text
   /opt/salus_robot/src/salus_hardware/config/rtk_sources.local.yaml
   ```

   Ese archivo está ignorado por Git y debe tener permisos `0600`. No poner
   credenciales en el unit, el env file, argumentos de shell, logs ni commits.
4. Copiar `deploy/systemd/salus-robot-real.env.example` a
   `/etc/salus/salus-robot-real.env` y ajustar sólo rutas no secretas si cambia
   la instalación. La ruta NTRIP del ejemplo es la ruta ignorada dentro del
   checkout que el runtime ya monta como `/ros2_ws/src`. El directorio de zonas
   del ejemplo queda en el `log` persistente del workspace preparado, montado
   como `/ros2_ws/log`.
5. Instalar el unit y recargar systemd:

   ```bash
   sudo install -D -m 0644 deploy/systemd/salus-robot-real.service \
     /etc/systemd/system/salus-robot-real.service
   sudo systemctl daemon-reload
   ```

   El checker del servicio exige explícitamente `/dev/ttyACM0` para Pixhawk y
   `/dev/ttyUSB0` para la UART del controlador; si falta cualquiera, falla con
   evidencia en lugar de que systemd salte silenciosamente el servicio. Usa
   devices explícitos y no `--privileged`.

## Arranque fail-closed

`ROS2_SALUS` debe detenerse primero por el procedimiento operativo. El checker
no lo detiene ni corrige owners:

```bash
sudo systemctl stop salus-real-global-v2-wifi.service
/opt/salus_robot/tools/check_real_mvp_authority.py \
  --device /dev/ttyACM0 --device /dev/ttyUSB0
sudo systemctl start salus-robot-real.service
sudo systemctl status --no-pager salus-robot-real.service
/opt/salus_robot/tools/check_real_mvp_readiness.sh
```

El checker debe imprimir `REAL_MVP_AUTHORITY_PASS`. Si detecta legacy,
procesos, publishers ROS o un device ocupado, falla sin matar nada. El unit
repite el checker como `ExecStartPre` y el runtime se ejecuta exclusivamente
por `tools/real_runtime_exec.sh` a través de `tools/start_real_runtime.sh`.
El readiness requiere `navigation_startup` en
`ACTIVE: ALL_NAV2_NODES_ACTIVE`.

## Parada y rollback

```bash
sudo systemctl stop salus-robot-real.service
sudo systemctl status --no-pager salus-robot-real.service
/opt/salus_robot/tools/check_real_mvp_authority.py \
  --device /dev/ttyACM0 --device /dev/ttyUSB0
docker ps --format '{{.Names}} {{.Image}}'
sudo systemctl start salus-real-global-v2-wifi.service
sudo systemctl status --no-pager salus-real-global-v2-wifi.service
```

Después de la parada, el checker debe confirmar que no quedan publishers ni
owners relevantes y `docker ps` debe mostrar únicamente los servicios
permitidos por el despliegue legacy. Verificar entonces los tópicos/rates
legacy y conservar el servicio `salus-robot-real.service` disponible para una
nueva ventana controlada.

Si un gate físico falla, detener el servicio, capturar evidencia mínima y
restaurar legacy. No añadir reintentos, tuning ni comandos de movimiento al
procedimiento.
