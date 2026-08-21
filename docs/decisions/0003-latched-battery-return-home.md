# ADR 0003: Retorno HOME enclavado por guardia de batería

- Estado: aceptada
- Fecha: 2026-08-21

## Contexto

El commit legacy `0208160` separó el SOC mostrado al operador de una guardia
de misión basada en tensión bajo carga, tensión recuperada, persistencia y
frescura. El commit `d235d77` agregó posteriormente una recuperación que podía
borrar el retorno y dejar la misión estacionada cuando la tensión volvía a
subir.

Cancelar automáticamente un retorno ya iniciado confunde recuperación de
tensión con recuperación de energía disponible. También permite que una
oscilación de carga alterne entre patrulla y retorno, y deja ambiguo qué debe
hacer el robot después de haber abandonado el loop.

## Decisión

- `BatteryMissionGuard` es la autoridad mientras sea válido. El SOC se usa
  sólo como fallback hasta observar la primera guardia válida.
- Una recomendación aceptada queda enclavada durante la misión. Una muestra
  posterior sin recomendación no cancela el retorno ni reanuda la patrulla.
- Si la misión se inicia con batería baja estando en HOME, se acepta y
  persiste, pero permanece en `AT_HOME` sin despachar movimiento.
- En `PATROL` se selecciona una salida del loop y se continúa hasta ella. En
  `JOIN_LOOP` o lejos de HOME durante `DEPART_HOME`, la decisión se conserva y
  se aplica al entrar al loop.
- En `EXIT_LOOP` o `RETURN_HOME`, muestras repetidas son idempotentes.
- Manual, E-stop o una pausa interrumpen el movimiento mediante las capas de
  seguridad existentes, conservan el latch y no reanudan automáticamente.
- Llegar a HOME conserva la causa para diagnóstico. Cancelar explícitamente la
  misión elimina su latch; una misión futura vuelve a evaluar la entrada
  vigente.

## Consecuencias

La política de entrada, la máquina de estados y el adaptador ROS tienen
responsabilidades separadas y pruebas independientes. Se descarta la
recuperación automática de `d235d77`; cualquier política futura para reanudar
una patrulla requerirá una orden explícita y otro ADR.
