#!/usr/bin/env python3
"""Plot one recorded OpenVINS/factor-graph comparison as a 5x3 SVG.

Usage:
    python3 plotters/openvins_factor_graph_single_agent_comparison.py \
        results/<ros-namespace> plots/comparison.svg

The input directory must contain openvins.csv, factor_graph.csv, and
groundtruth.csv as written by OpenVINS with ``save_results:=true``. All three
files contain timestamp, quaternion (JPL xyzw), position, velocity, gyro bias,
and accelerometer bias columns. The estimator files additionally contain the
full 15x15 error covariance ordered as attitude, position, velocity, gyro bias,
and accelerometer bias. The factor-graph file also contains a validity flag and
optimizer diagnostics. The three files must come from the same recorded run.
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


STATE_COLUMNS = (
    "q_x",
    "q_y",
    "q_z",
    "q_w",
    "p_x",
    "p_y",
    "p_z",
    "v_x",
    "v_y",
    "v_z",
    "bg_x",
    "bg_y",
    "bg_z",
    "ba_x",
    "ba_y",
    "ba_z",
)


def load_csv(path, covariance=False, factor_graph=False):
    if not path.is_file():
        raise ValueError(f"Missing input file: {path}")
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True, dtype=float))
    available = set(data.dtype.names or ())
    required = {"timestamp", *STATE_COLUMNS}
    if covariance:
        required.update(f"cov_{index}_{index}" for index in range(15))
    if factor_graph:
        required.add("valid")
    missing = sorted(required - available)
    if missing:
        raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
    if data.size == 0:
        raise ValueError(f"{path} contains no result rows")
    timestamps = np.asarray(data["timestamp"], dtype=float)
    if not np.all(np.isfinite(timestamps)) or np.unique(timestamps).size != timestamps.size:
        raise ValueError(f"{path} has invalid or duplicate timestamps")
    return data


def state_values(data):
    values = np.column_stack([data[column] for column in STATE_COLUMNS])
    if not np.all(np.isfinite(values)):
        raise ValueError("State data contains non-finite values")
    quaternion_norms = np.linalg.norm(values[:, :4], axis=1)
    if np.any(quaternion_norms < 1e-12):
        raise ValueError("State data contains a zero quaternion")
    values[:, :4] /= quaternion_norms[:, None]
    return values


def covariance_standard_deviation(data):
    diagonal = np.column_stack([data[f"cov_{index}_{index}"] for index in range(15)])
    if not np.all(np.isfinite(diagonal)):
        raise ValueError("Covariance diagonal contains non-finite values")
    if np.any(diagonal < -1e-12):
        raise ValueError("Covariance diagonal contains a negative variance")
    return np.sqrt(np.maximum(diagonal, 0.0))


def aligned_rows(openvins, factor_graph, groundtruth):
    common = np.intersect1d(openvins["timestamp"], factor_graph["timestamp"])
    common = np.intersect1d(common, groundtruth["timestamp"])
    if common.size < 2:
        raise ValueError("The result files have fewer than two matching timestamps")

    def select(data):
        indices = {timestamp: index for index, timestamp in enumerate(data["timestamp"])}
        return data[[indices[timestamp] for timestamp in common]]

    selected_openvins = select(openvins)
    selected_factor_graph = select(factor_graph)
    selected_groundtruth = select(groundtruth)
    valid = selected_factor_graph["valid"].astype(bool)
    if np.count_nonzero(valid) < 2:
        raise ValueError("The factor graph has fewer than two valid matching results")

    dropped = openvins.size + factor_graph.size + groundtruth.size - 3 * common.size
    invalid = common.size - np.count_nonzero(valid)
    if dropped or invalid:
        print(f"Dropped {dropped} unmatched rows and {invalid} invalid factor-graph rows")
    return common[valid], selected_openvins[valid], selected_factor_graph[valid], selected_groundtruth[valid]


def attitude_error(estimate_quaternion, truth_quaternion):
    estimate = Rotation.from_quat(estimate_quaternion)
    truth = Rotation.from_quat(truth_quaternion)
    return (estimate.inv() * truth).as_rotvec()


def main():
    parser = argparse.ArgumentParser(description="Plot recorded OpenVINS and factor-graph state errors with 2-sigma bounds.")
    parser.add_argument("results_directory", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    if args.output.suffix.lower() != ".svg":
        parser.error("output must have an .svg extension")

    openvins_data = load_csv(args.results_directory / "openvins.csv", covariance=True)
    factor_graph_data = load_csv(args.results_directory / "factor_graph.csv", covariance=True, factor_graph=True)
    groundtruth_data = load_csv(args.results_directory / "groundtruth.csv")
    timestamps, openvins_data, factor_graph_data, groundtruth_data = aligned_rows(
        openvins_data, factor_graph_data, groundtruth_data
    )

    openvins_state = state_values(openvins_data)
    factor_graph_state = state_values(factor_graph_data)
    truth_state = state_values(groundtruth_data)
    openvins_std = covariance_standard_deviation(openvins_data)
    factor_graph_std = covariance_standard_deviation(factor_graph_data)

    openvins_errors = [
        truth_state[:, 4:7] - openvins_state[:, 4:7],
        np.rad2deg(attitude_error(openvins_state[:, :4], truth_state[:, :4])),
        truth_state[:, 7:10] - openvins_state[:, 7:10],
        truth_state[:, 10:13] - openvins_state[:, 10:13],
        truth_state[:, 13:16] - openvins_state[:, 13:16],
    ]
    factor_graph_errors = [
        truth_state[:, 4:7] - factor_graph_state[:, 4:7],
        np.rad2deg(attitude_error(factor_graph_state[:, :4], truth_state[:, :4])),
        truth_state[:, 7:10] - factor_graph_state[:, 7:10],
        truth_state[:, 10:13] - factor_graph_state[:, 10:13],
        truth_state[:, 13:16] - factor_graph_state[:, 13:16],
    ]
    openvins_bounds = [
        2 * openvins_std[:, 3:6],
        np.rad2deg(2 * openvins_std[:, 0:3]),
        2 * openvins_std[:, 6:9],
        2 * openvins_std[:, 9:12],
        2 * openvins_std[:, 12:15],
    ]
    factor_graph_bounds = [
        2 * factor_graph_std[:, 3:6],
        np.rad2deg(2 * factor_graph_std[:, 0:3]),
        2 * factor_graph_std[:, 6:9],
        2 * factor_graph_std[:, 9:12],
        2 * factor_graph_std[:, 12:15],
    ]

    row_labels = (
        "Position error (m)",
        "Attitude error (deg)",
        "Velocity error (m/s)",
        "Gyro bias error (rad/s)",
        "Accel bias error (m/s²)",
    )
    colors = {"OpenVINS": "tab:blue", "Factor graph": "tab:orange"}
    time = timestamps - timestamps[0]
    figure, axes = plt.subplots(5, 3, figsize=(15, 16), sharex=True)
    for row in range(5):
        for axis in range(3):
            plot = axes[row, axis]
            for name, errors, bounds in (
                ("OpenVINS", openvins_errors, openvins_bounds),
                ("Factor graph", factor_graph_errors, factor_graph_bounds),
            ):
                color = colors[name]
                plot.plot(time, errors[row][:, axis], color=color, linewidth=1.1, label=f"{name} error")
                plot.plot(time, bounds[row][:, axis], color=color, linestyle="--", linewidth=0.9, label=f"{name} ±2σ")
                plot.plot(time, -bounds[row][:, axis], color=color, linestyle="--", linewidth=0.9)
            plot.axhline(0, color="black", linewidth=0.5, alpha=0.5)
            plot.grid(True, alpha=0.3)
            if row == 0:
                plot.set_title(("x", "y", "z")[axis])
            if axis == 0:
                plot.set_ylabel(row_labels[row])
            if row == 4:
                plot.set_xlabel("Time (s)")

    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.995), ncol=4)
    figure.suptitle(f"OpenVINS Factor Graph Single-Agent Comparison — {args.results_directory.name}", y=0.965)
    figure.tight_layout(rect=(0, 0, 1, 0.94))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, format="svg")
    plt.close(figure)
    print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
