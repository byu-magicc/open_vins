# Repository Guidelines

## Project Structure & Module Organization

OpenVINS is a C++14 visual-inertial estimation project organized as ROS packages, of which this is a fork. `ov_core/` contains camera models, tracking, simulation, and shared types; `ov_init/` implements estimator initialization; `ov_msckf/` contains the filter, factor-graph extension, ROS nodes, launches, and simulation executables; and `ov_eval/` provides trajectory and timing tools. Dataset-specific YAML, calibration, and masks live under `config/`. Small example datasets are in `ov_data/`, documentation sources in `docs/`, and analysis scripts in `plotters/`. Keep code inside the owning package's `src/` tree and update its CMake files when adding targets.

The goal of this fork is to replicate the performance of OpenVINS with a factor graph solver, while using the unique benefits of factor graphs. Changes should align with this goal.

## Build, Test, and Development Commands

The supported fork environment is Ubuntu 24.04 with ROS 2 Jazzy and GTSAM 4.2.2.

- `podman build -t open_vins .` builds the reproducible development/CI image.
- Use `colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release` for native ROS environments, run from the ROS2 workspace root.
- After sourcing `install/setup.zsh`, `ros2 launch ov_msckf multi_agent_mav_sim.launch.py` runs the primary integration simulation.

## Coding Style & Naming Conventions

Use two-space indentation, no tabs, attached braces, sorted includes, and a 140-column limit, as defined by `.clang-format`. Match existing conventions: `PascalCase` for classes and class-focused filenames, `snake_case` for functions and variables, and uppercase include guards. Prefer direct, readable research code over abstractions used only once. Do not add unnecessary default arguments or trailing whitespace. Remove methods and data structures made obsolete by a change. Preserve GPL copyright headers in C++ files.

