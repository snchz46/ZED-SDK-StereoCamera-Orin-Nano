# Teleoperación y Experiencia de Usuario

Los recursos en `src/teleop/` permiten controlar el robot y visualizar datos en tiempo real.

## Paneles listos para usar
- `config/rviz/zed_nav2.rviz`: panel RViz con tópicos preconfigurados.
- `docs/dashboards/foxglove_panel.json`: layout para Foxglove Studio.

## Interfaz web
- `src/teleop/web_dashboard/`: aplicación React que consume ROSBridge (`rosbridge_suite`).
- Lanzar con `launch/web_dashboard.launch.py` para servir la interfaz y el bridge.

## Controles manuales
`src/teleop/joystick/joy_teleop_node.py` traduce comandos de joystick y expone un modo de parada de emergencia.
