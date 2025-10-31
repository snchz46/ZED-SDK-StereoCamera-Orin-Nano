# CI/CD Infrastructure

The repository includes automations to ensure quality and reproducibility.

## GitHub Actions
- Workflow `ci.yml` runs linters (`ament_lint`, `ruff`, `mypy`) and tests (`colcon test`).
- The `record-artifacts` job stores reference rosbags generated in simulation.

## Additional recommendations
1. Enable branch protection rules for `main`.
2. Require reviews and dependency scanning.
3. Publish tagged releases with artifacts (models, configurations, and dashboards).
