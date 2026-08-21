# Intención: patrulla estructurada, HOME y retorno

Fuentes históricas: `73434bc`, `492dd13`, `b7b0a57` y `9477dd1` de
`ROS2_SALUS`.

La patrulla está compuesta por HOME, salida, loop y conector de retorno. Los
puntos se mantienen junto a sus acciones e índices originales durante cada
rotación o transición. El loop nunca retrocede para incorporarse de nuevo.

El retorno selecciona el checkpoint del loop más cercano al primer punto del
conector —o HOME si el conector está vacío— y espera alcanzarlo antes de salir
del loop. Ante una recomendación de batería válida el retorno queda enclavado:
la recuperación de tensión no lo cancela. Sólo una acción explícita del
operador, takeover manual o E-stop puede interrumpirlo.

El dominio puro y la persistencia atómica se conectan mediante
`patrol_mission_coordinator`: convierte una misión completa una única vez,
persiste sólo una definición válida y delega los tramos al `route_executor`.
No duplica Nav2 ni publica velocidad. Los eventos de checkpoint del ejecutor
son la frontera explícita que permite salir del loop sin inferir índices desde
un poll tardío. La activación por `BatteryMissionGuard` pertenece al siguiente
corte; el retorno manual ya conserva la misma política de salida estructurada.
