# Intención: frescura de scans y safety ante una fuente vencida (#277)

## Alcance

Investigar offline las capturas del 24–25/09 y los antecedentes del 14/09,
separando el origen del retraso de la respuesta de safety. Propietario del
cambio: `salus_navigation`, árbitro final y prueba PC de la composición real.
No cambiar tolerancias, rates, filtros, TF, despliegue ni hardware.

## Evidencia histórica e intención

- #277 documenta dos rechazos el 14/09 sobre `f918d4b`; no hay stamps por
  frontera de esos eventos. Los originales de Jetson se recuperaron
  por SSH read-only el 14/10 y se cotejaron por SHA256. El journal confirma
  ambos eventos; el JSON no incluye scans por frontera.
- #279 (`09ca22a`) ya redujo la cola del ground filter a depth 1. Su A/B
  documentado reduce age p95 de obstacles de 0,759 a 0,328 s; ningún warning
  original se correlacionó durante esa prueba. No repetir ese cambio.
- Los bags posteriores tienen procedencias y coberturas diferentes. Informe y
  extracción reproducible: `artifacts/robot-real-2026-09-24/analysis-277/` en
  el directorio de evidencia externo al repositorio.
- En `main@2fc6973`, `_on_scan` renueva la frescura con recepción ROS y
  descarta `header.stamp`. Nav2 Humble 1.1.20 ignora una fuente vencida sin
  imponer por ese solo hecho un stop. Esto permite que ambos comportamientos
  juntos acepten comando positivo sin una observación vigente.

## Reproducción antes del cambio

Se extendió la prueba existente `test_navigation_real_pc_runtime_without_clock`
con una fase de scans recibidos continuamente pero con stamps de 2 s de edad,
TF y comandos frescos. Sobre el código original falla la aserción de salida
safety/zero (`baseline-regression.log`, 1 failed en 9,62 s; confirmación posterior
con 14 salidas de 1,0 m/s en `baseline-regression-positive.log`). Imagen separada
`salus-robot:issue277-validation`: Nav2 1.1.20, tf2 y tf2_ros 0.25.24.
No sensor, actuator, `/clock` ni red externa; `docker run --network none`.
Esto confirma una brecha actual de safety en software, no la causa de los
warnings del robot ni una parada física.

## Contrato y cambio acotado

El adaptador calcula edad usando stamp y reloj ROS; el árbitro recibe esa
edad y la recepción monotónica, proyectando la adquisición sobre su reloj
steady. No compara epoch ROS directamente con `time.monotonic()`.
Una fuente sólo puede habilitar automático si el stamp es finito, positivo,
no futuro a recepción y su edad proyectada actual está en [0, monitor_timeout_s].
Recepciones continuas con stamp vencido no renuevan su vigencia. Un stamp
inválido invalida la observación anterior. El límite existente sigue en 1 s.
Los modos manual y sin detección explícita mantienen sus contratos.

Reutilizar las pruebas del árbitro y la composición PC: control positivo con
scan fresco, rechazo continuo vencido, recuperación con scan fresco, obstáculo
y ausencia de scans. No agregar otro launch ni infraestructura de pruebas.

## Resultado y límites

12 pruebas focalizadas pasan; build completo y suite de paquetes ejecutados
pasan (1254 tests, 0 fallos, 1 skip, más 36 host), así como smoke safety y
self-test del harness. El entrypoint `tools/test.sh` tropieza con el validador
sobre `.agents/` preexistente; sus pasos restantes se ejecutaron separados.
Los resultados históricos del paquete excluido `rslidar_msg` se conservaron
y separaron en el reporte, sin tocar tests ni borrar evidencia.

Informe: [scan-freshness-277-2026-10-14.md](../scan-freshness-277-2026-10-14.md).
Se mantiene #277 abierto: no se identificó retrospectivamente la primera
frontera de los dos warnings originales ni se validó este fix en hardware.
