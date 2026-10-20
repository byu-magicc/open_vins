#!/usr/bin/env bash

set -eo pipefail

source "$(dirname -- "${BASH_SOURCE[0]}")/experiment_common.sh"
# Select one or more of: openvins factor_graph hybrid.
filter_types=(openvins factor_graph hybrid)
bags_directory="${workspace_directory}/data/holoocean"
bag_names=(center_2 left_2 right_2)
# Seconds from each bag's first /imu/data header; missing entries start at zero with truth initialization.
declare -A bag_start_times=([center_2]=9.0 [left_2]=12.0 [right_2]=11.0)
usage() {
  echo "Usage: $(basename -- "$0") [--bags-directory PATH] [launch_argument:=value ...]"
  echo "Build once, replay the selected bags together for each selected filter, and generate plots."
  echo "Ctrl+C stops replay, skips remaining filters, and plots saved data."
  echo "Selected filters: ${filter_types[*]} (edit filter_types to configure runs)."
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
bag_paths=()
bag_starts=()
for bag_name in "${bag_names[@]}"; do
  bag_paths+=("${bags_directory}/${bag_name}")
  bag_starts+=("${bag_start_times[${bag_name}]:-0.0}")
done
agent_names_argument=$(IFS=,; echo "${bag_names[*]}")
bag_paths_argument=$(IFS=,; echo "${bag_paths[*]}")
bag_starts_argument=$(IFS=,; echo "${bag_starts[*]}")

run_experiments multi_agent_holoocean.launch.py true "${launch_arguments[@]}" \
  agent_names:="${agent_names_argument}" bag_paths:="${bag_paths_argument}" bag_starts:="${bag_starts_argument}"
