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

  local interrupted=false
  local -a plotted_filters=()
  local filter_type
  for filter_type in "${filter_types[@]}"; do
    local mode_results="${results_directory}/${filter_type}"
    local mode_plots="${plots_directory}/${filter_type}"
    export ROS_LOG_DIR="${run_directory}/ros_logs/${filter_type}"
    mkdir -p "${mode_results}" "${mode_plots}" "${ROS_LOG_DIR}"
    local launch_status=0
    # ROS receives the terminal's SIGINT too; wait for it to flush results before plotting.
    trap 'interrupted=true' INT
    # OpenVINS errors have no severity tag; request only errors and hide launcher chatter.
    ros2 launch ov_msckf "${launch_file}" "$@" \
      filter_type:="${filter_type}" save_results:=true results_path:="${mode_results}" verbosity:=ERROR \
      2>&1 | (trap '' INT; exec tee "${ROS_LOG_DIR}/launch.log") | (trap '' INT; exec awk '
        {
          line = $0
          gsub(/\033\[[0-9;]*m/, "", line)
          if (line ~ /\[(ERROR|FATAL)\]/) { print line; fflush(); next }
          if (line ~ /\[(DEBUG|INFO|WARN|WARNING)\]/ || line ~ /Setting printing level to:/ ||
              line ~ /overriding node verbosity with value from ROS!/ || line ~ /^[[:space:]]*$/) next
          print line
          fflush()
        }
      ' >&2) || launch_status=$?
    trap - INT
    if [[ "${interrupted}" == true ]]; then
      echo "Simulation interrupted; plotting saved data and skipping remaining filters." >&2
    elif ((launch_status != 0)); then
      echo "Error: ${filter_type} failed with exit code ${launch_status}; simulation log: ${ROS_LOG_DIR}/launch.log" >&2
      return "${launch_status}"
    fi
    local plot_status=0
    "${plotter_python}" "${repository_directory}/plotters/plot_results.py" \
      "${mode_results}" "${mode_plots}" "${plot_arguments[@]}" || plot_status=$?
    if ((plot_status == 0)); then
      plotted_filters+=("${filter_type}")
    elif [[ "${interrupted}" == true ]]; then
      echo "Warning: could not plot interrupted ${filter_type} results; continuing with completed data." >&2
    else
      return "${plot_status}"
    fi
    if [[ "${interrupted}" == true ]]; then break; fi
  done

  if ((${#plotted_filters[@]} == 0)); then
    echo "Error: the experiment produced no plottable results; saved data: ${results_directory}" >&2
    return 1
  fi
  local agent_count=0
  local agent_directory
  for agent_directory in "${results_directory}/${plotted_filters[0]}"/*; do
    [[ -d "${agent_directory}" ]] || continue
    local agent_name
    agent_name=$(basename -- "${agent_directory}")
    if ((${#plotted_filters[@]} > 1)); then
      local comparison_status=0
      "${plotter_python}" "${repository_directory}/plotters/compare_estimators.py" \
        "${results_directory}" "${agent_name}" "${plots_directory}/${agent_name}_estimator_comparison.svg" \
        --modes "${plotted_filters[@]}" "${plot_arguments[@]}" || comparison_status=$?
      if ((comparison_status != 0)); then
        if [[ "${interrupted}" != true ]]; then return "${comparison_status}"; fi
        echo "Warning: could not compare saved results for ${agent_name}." >&2
      fi
    fi
    ((agent_count += 1))
  done
  if ((agent_count == 0)); then
    echo "Error: the experiment produced no agent results." >&2
    return 1
  fi
  echo "Experiment complete: ${run_directory}"
}
