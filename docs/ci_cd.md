# Infraestructura de CI/CD

El repositorio incluye automatizaciones para asegurar calidad y reproducibilidad.

## GitHub Actions
- Workflow `ci.yml` ejecuta linters (`ament_lint`, `ruff`, `mypy`) y pruebas (`colcon test`).
- El job `record-artifacts` almacena rosbags de referencia generados en simulación.

## Recomendaciones adicionales
1. Active reglas de branch protection para `main`.
2. Habilite revisiones obligatorias y escaneo de dependencias.
3. Publique releases etiquetadas con artefactos (modelos, configuraciones y dashboards).
