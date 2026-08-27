# Ficha de intención: perfil sin detección local de obstáculos

## Alcance

Añadir el primer perfil de capacidades explícito para robots que no requieren
LiDAR ni evasión local, priorizando simulación y manteniendo el perfil protegido
como default. No incluye múltiples IMU, heading externo ni hardware real.

## Contratos

`CapabilityState` diferencia `unknown`, `not_installed`,
`disabled_by_profile`, `unavailable`, `invalid`, `stale`, `failed`,
`enabled_by_profile` y `ready`. `SystemCapabilities` publica atómicamente el
perfil efectivo y sus capacidades en `/system/capabilities` con QoS reliable,
transient-local y depth 1. `salus_web` lo proyecta sin inferir salud por ausencia
de tópicos.

## Invariantes

- sólo hay selección explícita al launch;
- una pérdida de scan en el perfil normal continúa deteniendo autonomía;
- el perfil degradado no publica `/scan_clean` ni `/scan_preview` ficticios;
- existe exactamente un productor de `/cmd_vel_safe`;
- `nav_command_server` conserva la autoridad exclusiva de `/cmd_vel_final`;
- keepout, PathHealth, watchdogs, cancelación, freno y takeover manual siguen
  activos.

## Evidencia y límites

El smoke ejecuta una meta Nav2 completa y el escenario de zonas dinámicas en
Gazebo, comprueba el estado tipado, ausencia de outputs LiDAR y autoridad de comandos. No valida percepción,
seguridad física, Jetson ni robot real. Multiple IMU y orientación externa
pertenecen a cortes posteriores del mismo hito de perfiles.
