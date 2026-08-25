# salus_evaluation

Dominio puro para escenarios, métricas y gates reproducibles de navegación.
No posee autoridad de comando ni TF. La integración ROS, RViz y el runner se
incorporarán sobre estos contratos según ADR 0007.

Los escenarios instalados usan metros, radianes y segundos. En simulación se
compara la estimación contra `/odom_raw`; esto no valida comportamiento real.
