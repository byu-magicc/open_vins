#!/usr/bin/env bash

set -eo pipefail

source "$(dirname -- "${BASH_SOURCE[0]}")/experiment_common.sh"

# Select one or more of: openvins factor_graph hybrid.
filter_types=(openvins factor_graph hybrid)

usage() {
  echo "Usage: $(basename -- "$0") [launch_argument:=value ...]"
  echo
  echo "Build the workspace, run the multi-agent simulation, and generate plots."
  echo "Ctrl+C stops the simulation, skips remaining filters, and plots saved data."
  echo "Selected filters: ${filter_types[*]} (edit filter_types to configure runs)."
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
    save_results:=*|results_path:=*|filter_type:=*)
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

run_experiments multi_agent_mav_sim.launch.py false "${launch_arguments[@]}"
