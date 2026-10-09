#!/usr/bin/env bash

set -eo pipefail

repository_directory=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_directory=$(cd -- "${repository_directory}/../.." && pwd)
bags_directory="${workspace_directory}/data/holoocean"
bag_names=(center_2 left_2 right_2)
# Seconds from the first /imu/data header. Remove an entry to use normal initialization.
declare -A bag_start_times=([center_2]=9.0 [left_2]=12.0 [right_2]=11.0)
filter_type=openvins
plotter_python="${PLOTTER_PYTHON:-${workspace_directory}/.venv-plotters/bin/python}"
if [[ ! -x "${plotter_python}" && -z "${PLOTTER_PYTHON:-}" ]]; then
  plotter_python=/usr/bin/python3
fi

usage() {
  echo "Usage: $(basename -- "$0") [--bags-directory PATH] [verbosity:=LEVEL] [rviz_enable:=BOOL] [filter_type:=MODE]"
  echo "Build once, run single-agent OpenVINS on the selected HoloOcean bags, and plot each separately."
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
    filter_type:=*) filter_type=${1#filter_type:=}; shift ;;
    verbosity:=*|rviz_enable:=*) launch_arguments+=("$1"); shift ;;
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
  "${plotter_python}" -c 'import rosbags.highlevel, numpy, scipy, matplotlib, yaml' 2>/dev/null; then
  echo "Error: install the plotting dependencies in ReadMe.md and set PLOTTER_PYTHON." >&2
  exit 1
fi

export PLOTTER_PYTHON="${plotter_python}"
# Read the same camera-to-IMU calibration used by the fixed HoloOcean launch config.
camera_imu_offset=$("${plotter_python}" - "${repository_directory}/config/holoocean_fixedwing/kalibr_imucam_chain.yaml" <<'PYTHON'
import sys
import yaml
with open(sys.argv[1]) as source:
    calibration = yaml.safe_load(source.read().removeprefix('%YAML:1.0'))
print(calibration['cam0']['timeshift_cam_imu'])
PYTHON
)

source /opt/ros/jazzy/setup.bash
set -u
cd "${workspace_directory}"
colcon build --symlink-install --cmake-args \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
set +u
source "${workspace_directory}/install/setup.bash"
set -u

run_directory="${workspace_directory}/runs/$(date --utc +%Y%m%dT%H%M%SZ)"
mkdir -p "${workspace_directory}/runs"
mkdir "${run_directory}"
export MPLCONFIGDIR="${run_directory}/matplotlib"

for bag_name in "${bag_names[@]}"; do
  bag_directory="${bags_directory}/${bag_name}"
  results_directory="${run_directory}/results/${bag_name}"
  plots_directory="${run_directory}/plots/${bag_name}"
  export ROS_LOG_DIR="${run_directory}/ros_logs/${bag_name}"
  mkdir -p "${results_directory}" "${plots_directory}" "${ROS_LOG_DIR}"
  truth_arguments=()
  if [[ -n "${bag_start_times[${bag_name}]:-}" ]]; then
    truth_arguments+=("start_time:=${bag_start_times[${bag_name}]}")
  fi
  echo "Processing ${bag_name}; start: ${bag_start_times[${bag_name}]:-normal initialization}; logs: ${ROS_LOG_DIR}"
  ros2 launch ov_msckf single_agent_holoocean.launch.py \
    "${launch_arguments[@]}" "${truth_arguments[@]}" save_results:=true filter_type:="${filter_type}" \
    results_path:="${results_directory}" bag_path:="${bag_directory}" \
    >"${ROS_LOG_DIR}/launch.log" 2>&1
  awk -v bag="${bag_name}" '/\[BAG\]: Simulated time/ { sub(/^.*\[BAG\]: /, ""); print bag ": " $0 }' \
    "${ROS_LOG_DIR}/launch.log"
  "${plotter_python}" "${repository_directory}/plotters/plot_results.py" \
    "${results_directory}" "${plots_directory}" --truth-bag "${bag_directory}" --camera-imu-offset "${camera_imu_offset}"
done

echo "Experiment complete: ${run_directory}"
