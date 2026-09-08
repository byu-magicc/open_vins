#!/usr/bin/env bash

set -eo pipefail

repository_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_directory=$(cd -- "${repository_directory}/../.." && pwd)

usage() {
  echo "Usage: $(basename -- "$0") [launch_argument:=value ...]"
  echo
  echo "Build the workspace, run the multi-agent simulation, and generate plots."
  echo "Results are saved under <workspace>/runs/<timestamp>/."
  echo
  echo "Examples:"
  echo "  $(basename -- "$0")"
  echo "  $(basename -- "$0") rviz_enable:=true"
  echo "  $(basename -- "$0") agent_names:=center,left datasets:=path/center.txt,path/left.txt"
}

launch_arguments=()
for argument in "$@"; do
  case "${argument}" in
    -h|--help)
      usage
      exit 0
      ;;
    save_results:=*|results_path:=*)
      echo "Error: ${argument%%:=*} is managed by this script." >&2
      exit 2
      ;;
    *:=*)
      launch_arguments+=("${argument}")
      ;;
    *)
      echo "Error: expected a ROS launch argument in name:=value form: ${argument}" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source /opt/ros/jazzy/setup.bash
set -u

cd "${workspace_directory}"
colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=Release
set +u
source "${workspace_directory}/install/setup.bash"
set -u

run_name="$(date --utc +%Y%m%dT%H%M%SZ)"
run_directory="${workspace_directory}/runs/${run_name}"
results_directory="${run_directory}/results"
plots_directory="${run_directory}/plots"
ros_logs_directory="${run_directory}/ros_logs"

if [[ -e "${run_directory}" ]]; then
  echo "Error: run directory already exists: ${run_directory}" >&2
  exit 1
fi
mkdir -p "${results_directory}" "${plots_directory}" "${ros_logs_directory}"
export ROS_LOG_DIR="${ros_logs_directory}"

ros2 launch ov_msckf multi_agent_mav_sim.launch.py \
  "${launch_arguments[@]}" \
  save_results:=true \
  results_path:="${results_directory}"

/usr/bin/python3 "${repository_directory}/plotters/openvins_multi_agent.py" \
  "${results_directory}" \
  "${plots_directory}"

agent_count=0
for agent_directory in "${results_directory}"/*; do
  [[ -d "${agent_directory}" ]] || continue
  agent_name=$(basename -- "${agent_directory}")
  /usr/bin/python3 "${repository_directory}/plotters/openvins_factor_graph_single_agent_comparison.py" \
    "${agent_directory}" \
    "${plots_directory}/${agent_name}_openvins_vs_factor_graph.svg"
  ((agent_count += 1))
done

if ((agent_count == 0)); then
  echo "Error: the simulation produced no agent result directories." >&2
  exit 1
fi

echo "Experiment complete: ${run_directory}"
