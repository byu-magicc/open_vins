#!/usr/bin/env bash

set -eo pipefail

repository_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_directory=$(cd -- "${repository_directory}/../.." && pwd)
bags_directory="${workspace_directory}/data/holoocean"
bag_names=(center_2 left_2 right_2)
# Seconds from each bag's first /imu/data header; missing entries start at zero with truth initialization.
declare -A bag_start_times=([center_2]=9.0 [left_2]=12.0 [right_2]=11.0)
plotter_python="${PLOTTER_PYTHON:-${workspace_directory}/.venv-plotters/bin/python}"
if [[ ! -x "${plotter_python}" && -z "${PLOTTER_PYTHON:-}" ]]; then
  plotter_python=/usr/bin/python3
fi

usage() {
  echo "Usage: $(basename -- "$0") [--bags-directory PATH] [launch_argument:=value ...]"
  echo "Build once, replay the selected bags together in OpenVINS, factor-graph, and hybrid modes, and compare them."
  echo "Selected bags: ${bag_names[*]} (edit bag_names and bag_start_times to configure runs)."
  echo "Defaults: <workspace>/data/holoocean, processing as fast as OpenVINS allows. Results: <workspace>/runs/<timestamp>/."
  echo "Set PLOTTER_PYTHON to select the plotting interpreter; see ReadMe.md for dependency setup."
}

launch_arguments=()
while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --bags-directory)
      if (($# < 2)); then echo "Error: $1 requires a value." >&2; exit 2; fi
      bags_directory=$2
      shift 2
      ;;
    agent_names:=*|bag_paths:=*|bag_starts:=*|save_results:=*|results_path:=*|filter_type:=*)
      echo "Error: ${1%%:=*} is managed by this script." >&2
      exit 2
      ;;
    *:=*) launch_arguments+=("$1"); shift ;;
    *) echo "Error: unsupported or script-managed argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -d "${bags_directory}" ]]; then
  echo "Error: bag directory does not exist: ${bags_directory}" >&2
  exit 2
fi
bags_directory=$(cd -- "${bags_directory}" && pwd)
for bag_name in "${bag_names[@]}"; do
  if [[ ! -f "${bags_directory}/${bag_name}/metadata.yaml" ]]; then
    echo "Error: selected bag is missing metadata.yaml: ${bags_directory}/${bag_name}" >&2
    exit 2
  fi
done
if ! MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/openvins-plotters-matplotlib}" \
  "${plotter_python}" -c 'import rosbags.highlevel, numpy, scipy, matplotlib' 2>/dev/null; then
  echo "Error: install the plotting dependencies in ReadMe.md and set PLOTTER_PYTHON." >&2
  exit 1
fi

export PLOTTER_PYTHON="${plotter_python}"
bag_paths=()
bag_starts=()
for bag_name in "${bag_names[@]}"; do
  bag_paths+=("${bags_directory}/${bag_name}")
  bag_starts+=("${bag_start_times[${bag_name}]:-0.0}")
done
agent_names_argument=$(IFS=,; echo "${bag_names[*]}")
bag_paths_argument=$(IFS=,; echo "${bag_paths[*]}")
bag_starts_argument=$(IFS=,; echo "${bag_starts[*]}")

source /opt/ros/jazzy/setup.bash
set -u
cd "${workspace_directory}"
colcon build --symlink-install --cmake-args \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
set +u
source "${workspace_directory}/install/setup.bash"
set -u

run_directory="${workspace_directory}/runs/$(date --utc +%Y%m%dT%H%M%SZ)"
results_directory="${run_directory}/results"
plots_directory="${run_directory}/plots"
ros_logs_directory="${run_directory}/ros_logs"
export MPLCONFIGDIR="${run_directory}/matplotlib"
mkdir -p "${workspace_directory}/runs"
mkdir "${run_directory}"
mkdir -p "${results_directory}" "${plots_directory}" "${ros_logs_directory}" "${MPLCONFIGDIR}"

for filter_type in openvins factor_graph hybrid; do
  mode_results="${results_directory}/${filter_type}"
  mode_plots="${plots_directory}/${filter_type}"
  export ROS_LOG_DIR="${ros_logs_directory}/${filter_type}"
  mkdir -p "${mode_results}" "${mode_plots}" "${ROS_LOG_DIR}"
  echo "Processing ${bag_names[*]} together in ${filter_type} mode; logs: ${ROS_LOG_DIR}"
  ros2 launch ov_msckf multi_agent_bag.launch.py \
    "${launch_arguments[@]}" \
    agent_names:="${agent_names_argument}" bag_paths:="${bag_paths_argument}" bag_starts:="${bag_starts_argument}" \
    filter_type:="${filter_type}" save_results:=true results_path:="${mode_results}" \
    >"${ROS_LOG_DIR}/launch.log" 2>&1
  awk '/\[BAG\]: Simulated time/ { print }' "${ROS_LOG_DIR}/launch.log"
  "${plotter_python}" "${repository_directory}/plotters/plot_results.py" \
    "${mode_results}" "${mode_plots}" --holoocean
done

for bag_name in "${bag_names[@]}"; do
  "${plotter_python}" "${repository_directory}/plotters/compare_estimators.py" \
    "${results_directory}" "${bag_name}" "${plots_directory}/${bag_name}_estimator_comparison.svg" --holoocean
done

echo "Experiment complete: ${run_directory}"
