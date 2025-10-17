# Repository Guidelines

## Project Structure & Module Organization
The workspace mirrors a ROS 2 layout: `common/` holds shared message definitions and utilities, `pioneer_ws/` packages drive the physical rover stack, `pioneer_base/` contains base-station orchestration nodes, `sim/` provides Gazebo-ready simulation packages, and `unified_controller/` bundles cross-platform control logic. Docker recipes live in `Docker/` for containerised development, while `record/` stores logs and visual assets. Check each top-level README for package-specific launch and configuration details.

## Build, Test, and Development Commands
Run `colcon build --symlink-install` from the workspace root to compile all packages and preserve editable Python modules. Use `source install/setup.bash` after every build or terminal session. Execute `colcon test` to trigger the `ament_*` linters and smoke tests; target individual packages with `colcon test --packages-select pioneer_base`. The `pioneer_ws/scripts/setup.sh` script provisions dependencies on fresh hardware, and `ros2 launch rover_launch rover.launch.py` will start the full rover stack once built.

## Coding Style & Naming Conventions
Python modules follow ROS 2 best practices: 4-space indentation, snake_case files, and explicit `setup.py` entry points. Keep package directories lowercase with underscores, mirroring existing names such as `locomotion_core` and `rf_receiver`. Run `colcon test` or the package-specific `python3 -m ament_flake8 <pkg>` when touching Python code to satisfy linting. YAML configs (for example `pioneer_ws/config/system_config.yaml`) should retain two-space indentation for nested keys.

## Testing Guidelines
Unit and lint tests rely on `ament_flake8`, `ament_pep257`, and package-level sanity checks. Place new tests under `<package>/test/` with descriptive snake_case filenames, e.g. `test_health_monitor.py`. Prefer deterministic fixtures and avoid hardware dependencies by mocking serial and sensor interfaces. For regression work, capture relevant logs into `record/` and reference them in the pull request.

## Commit & Pull Request Guidelines
Recent history prefers imperative, lowercase summaries such as `fix: prevent NoneType error`. Use the `type: description` pattern (`feat:`, `fix:`, `docs:`) when possible, and keep the scope focused. Pull requests should include: a concise overview, testing notes (`colcon build`, `colcon test` outputs), linked issues, and simulation or hardware evidence when behaviour changes. Request reviews from maintainers owning the touched package and flag breaking changes early.

## Environment & Configuration Tips
This stack targets ROS 2 Jazzy on Ubuntu; ensure system locale and timezone are synced before running the rover services. When creating new launch files, place them in `rover_launch/launch/` and expose configuration toggles through `system_config.yaml` to maintain centralized control. Prefer using the provided Docker setup for reproducible CI or demos when bare-metal dependencies are uncertain.
