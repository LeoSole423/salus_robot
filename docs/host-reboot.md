# Reinicio de la Jetson desde Cockpit

La unidad `salus-host-reboot.socket` expone un socket Unix de mantenimiento en el host de la Jetson. El gateway de Cockpit autoriza al administrador, exige la confirmación visual y solicita el reinicio mediante ese socket. El flujo funciona aun cuando el WebSocket de ROS esté caído, siempre que el gateway siga accesible.

## Habilitación

Validar en banco el estado de los actuadores ante pérdida de UART y el arranque automático del servicio real antes de habilitar esta operación en un rover. Instalar los archivos desde un checkout revisado, con propietario root.

```sh
sudo groupadd --system salus-maintenance
sudo install -D -m 0644 deploy/systemd/salus-host-reboot.py /usr/local/libexec/salus-host-reboot.py
sudo install -m 0644 deploy/systemd/salus-host-reboot.socket /etc/systemd/system/
sudo install -m 0644 deploy/systemd/salus-host-reboot@.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now salus-host-reboot.socket
```

Si el grupo ya existe, reutilizarlo. Configurar el gateway de Cockpit para un único destino SALUS correspondiente a **esta misma Jetson**, con `COCKPIT_ROVER_CONTROL=true` y `COCKPIT_NAVIGATION_ALTERNATIVES` vacío. En el Compose del despliegue, incorporar esta configuración al servicio `auth` y definir `SALUS_MAINTENANCE_GID` con el GID numérico que informa `getent group salus-maintenance`.

```yaml
services:
  auth:
    environment:
      COCKPIT_HOST_REBOOT_SOCKET: /run/salus-maintenance/reboot.sock
    group_add:
      - "${SALUS_MAINTENANCE_GID:?Set the host salus-maintenance group ID}"
    volumes:
      - /run/salus-maintenance:/run/salus-maintenance:ro
```

El socket queda disponible solo a procesos autorizados por el grupo del host. El protocolo acepta `status` para consultar el identificador de arranque y `reboot <boot_id>` para solicitar el reinicio del arranque consultado. Ambas líneas terminan en LF. El servicio invoca una única operación de systemd con argumentos fijos.

Para probar conectividad desde el contenedor `auth`, usar solo la consulta de estado. La respuesta esperada contiene `state: ready` y el identificador de arranque.

```sh
docker compose exec auth node -e 'const s = require("node:net").connect(process.env.COCKPIT_HOST_REBOOT_SOCKET, () => s.write("status\n")); s.pipe(process.stdout); s.setTimeout(3000, () => s.destroy());'
```

Para deshabilitar la capacidad en el host, ejecutar `sudo systemctl disable --now salus-host-reboot.socket` y retirar la configuración de mantenimiento del gateway. El resultado `requested` significa que systemd aceptó la solicitud. La recuperación se verifica consultando nuevamente el identificador de arranque y la disponibilidad del servicio real.
