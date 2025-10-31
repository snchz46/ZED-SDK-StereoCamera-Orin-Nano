# Navegación y Gestión de Misiones

Los componentes en `src/navigation/` amplían Nav2 para soportar misiones reactivas y monitoreo del sistema.

## Mission Manager
- `src/navigation/mission_manager/mission_manager_node.py`: carga waypoints desde `config/missions/*.yaml` y publica objetivos dinámicos.
- Soporta eventos definidos por plugins (`plugins/condition_plugins.py`) para misiones contextuales.

## Supervisión y failsafe
- `src/navigation/health_monitor/health_monitor_node.cpp`: vigila la latencia de tópicos y activa modos seguros.
- `config/health_rules.yaml`: define umbrales y acciones correctivas.

## Integración con Nav2
El launch `launch/mission_stack.launch.py` orquesta Nav2, el mission manager y la supervisión de forma cohesiva.
