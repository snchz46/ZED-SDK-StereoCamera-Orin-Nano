# Navigation and Mission Management

The components in `src/navigation/` extend Nav2 to support reactive missions and system monitoring.

## Mission Manager
- `src/navigation/mission_manager/mission_manager_node.py`: loads waypoints from `config/missions/*.yaml` and publishes dynamic goals.
- Supports events defined by plugins (`plugins/condition_plugins.py`) for contextual missions.

## Monitoring and failsafe
- `src/navigation/health_monitor/health_monitor_node.cpp`: watches topic latency and triggers safe modes.
- `config/health_rules.yaml`: defines thresholds and corrective actions.

## Nav2 integration
The launch file `launch/mission_stack.launch.py` orchestrates Nav2, the mission manager, and monitoring cohesively.
