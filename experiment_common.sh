#!/usr/bin/env bash
# Shared build, recording, and plotting for the simulation and bag presets.

repository_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_directory=$(cd -- "${repository_directory}/../.." && pwd)
plotter_python="${PLOTTER_PYTHON:-${workspace_directory}/.venv-plotters/bin/python}"
if [[ ! -x "${plotter_python}" && -z "${PLOTTER_PYTHON:-}" ]]; then
  plotter_python=/usr/bin/python3
fi

run_experiments() {
  local launch_file=$1
  local holoocean=$2
  shift 2
  local -a plot_arguments=()
  if [[ "${holoocean}" == true ]]; then
    plot_arguments+=(--holoocean)
    if ! MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/openvins-plotters-matplotlib}" \
      "${plotter_python}" -c 'import rosbags.highlevel, numpy, scipy, matplotlib' 2>/dev/null; then
      echo "Error: install the HoloOcean dependencies in ReadMe.md and set PLOTTER_PYTHON." >&2
      return 1
    fi
  fi
  export PLOTTER_PYTHON="${plotter_python}"
  source /opt/ros/jazzy/setup.bash
  cd "${workspace_directory}"
  if [[ "${OPENVINS_SKIP_BUILD:-0}" != 1 ]]; then
    colcon build --base-paths "${repository_directory}" --symlink-install \
      --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
  fi
  source "${workspace_directory}/install/setup.bash"
  set -u

  local run_directory="${workspace_directory}/runs/$(date --utc +%Y%m%dT%H%M%SZ)"
  local results_directory="${run_directory}/results"
  local plots_directory="${run_directory}/plots"
  export MPLCONFIGDIR="${run_directory}/matplotlib"
  mkdir -p "${workspace_directory}/runs"
  mkdir "${run_directory}"
  mkdir -p "${results_directory}" "${plots_directory}" "${MPLCONFIGDIR}"

  local filter_type
  for filter_type in openvins factor_graph hybrid; do
    local mode_results="${results_directory}/${filter_type}"
    local mode_plots="${plots_directory}/${filter_type}"
    export ROS_LOG_DIR="${run_directory}/ros_logs/${filter_type}"
    mkdir -p "${mode_results}" "${mode_plots}" "${ROS_LOG_DIR}"
    ros2 launch ov_msckf "${launch_file}" "$@" \
      filter_type:="${filter_type}" save_results:=true results_path:="${mode_results}" \
      >"${ROS_LOG_DIR}/launch.log" 2>&1
    "${plotter_python}" "${repository_directory}/plotters/plot_results.py" \
      "${mode_results}" "${mode_plots}" "${plot_arguments[@]}"
  done

  local agent_count=0
  local agent_directory
  for agent_directory in "${results_directory}/openvins"/*; do
    [[ -d "${agent_directory}" ]] || continue
    local agent_name
    agent_name=$(basename -- "${agent_directory}")
    "${plotter_python}" "${repository_directory}/plotters/compare_estimators.py" \
      "${results_directory}" "${agent_name}" "${plots_directory}/${agent_name}_estimator_comparison.svg" "${plot_arguments[@]}"
    ((agent_count += 1))
  done
  if ((agent_count == 0)); then
    echo "Error: the experiment produced no agent results." >&2
    return 1
  fi
  echo "Experiment complete: ${run_directory}"
}
