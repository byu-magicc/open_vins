#!/usr/bin/env python3
"""Compare OpenVINS, factor-graph, and hybrid experiment results for one agent."""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

STATE_COLUMNS = ("q_x", "q_y", "q_z", "q_w", "p_x", "p_y", "p_z", "v_x", "v_y", "v_z",
                 "bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z")
MODES = ("openvins", "factor_graph", "hybrid")


def load(path, covariance=True):
    if not path.is_file():
        raise ValueError(f"Missing input file: {path}")
    data = np.atleast_1d(np.genfromtxt(path, delimiter=",", names=True, dtype=float))
    required = {"timestamp", *STATE_COLUMNS}
    if covariance:
        required.update(f"cov_{index}_{index}" for index in range(15))
        required.add("valid")
    missing = sorted(required - set(data.dtype.names or ()))
    if missing or data.size == 0:
        raise ValueError(f"Invalid input {path}; missing: {', '.join(missing)}")
    return data


def select(data, timestamps):
    indices = {timestamp: index for index, timestamp in enumerate(data["timestamp"])}
    return data[[indices[timestamp] for timestamp in timestamps]]


def states(data):
    result = np.column_stack([data[column] for column in STATE_COLUMNS])
    result[:, :4] /= np.linalg.norm(result[:, :4], axis=1)[:, None]
    return result


def attitude_error(estimate, truth):
    return np.rad2deg((Rotation.from_quat(estimate).inv() * Rotation.from_quat(truth)).as_rotvec())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_directory", type=Path)
    parser.add_argument("agent")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.suffix.lower() != ".svg":
        parser.error("output must have an .svg extension")

    estimates = {mode: load(args.results_directory / mode / args.agent / "estimate.csv") for mode in MODES}
    truths = {mode: load(args.results_directory / mode / args.agent / "groundtruth.csv", False) for mode in MODES}
    timestamps = estimates[MODES[0]]["timestamp"]
    for mode in MODES:
        timestamps = np.intersect1d(timestamps, estimates[mode]["timestamp"])
        timestamps = np.intersect1d(timestamps, truths[mode]["timestamp"])
    if timestamps.size < 2:
        raise ValueError("Fewer than two valid timestamps are common to all modes")
    aligned_estimates = {mode: select(estimates[mode], timestamps) for mode in MODES}
    valid = np.logical_and.reduce([aligned_estimates[mode]["valid"].astype(bool) for mode in MODES])
    timestamps = timestamps[valid]
    aligned_estimates = {mode: data[valid] for mode, data in aligned_estimates.items()}
    aligned_truth = {mode: select(truths[mode], timestamps) for mode in MODES}
    reference_truth = states(aligned_truth[MODES[0]])
    for mode in MODES[1:]:
        if not np.allclose(states(aligned_truth[mode]), reference_truth, atol=1e-10, rtol=1e-10):
            raise ValueError(f"Ground truth differs in {mode}")

    range_times = []
    ranges_path = args.results_directory / "hybrid" / "ranges.csv"
    if ranges_path.is_file():
        with ranges_path.open(newline="") as stream:
            for event in csv.DictReader(stream):
                if event["owner"] == args.agent:
                    range_times.append(float(event["owner_timestamp"]) - timestamps[0])
                elif event["neighbor"] == args.agent:
                    range_times.append(float(event["neighbor_timestamp"]) - timestamps[0])

    time = timestamps - timestamps[0]
    estimates_state = {mode: states(data) for mode, data in aligned_estimates.items()}
    errors = {}
    bounds = {}
    for mode in MODES:
        estimate = estimates_state[mode]
        std = np.sqrt(np.maximum(0.0, np.column_stack([aligned_estimates[mode][f"cov_{i}_{i}"] for i in range(15)])))
        errors[mode] = (reference_truth[:, 4:7] - estimate[:, 4:7], attitude_error(estimate[:, :4], reference_truth[:, :4]),
                        reference_truth[:, 7:10] - estimate[:, 7:10], reference_truth[:, 10:13] - estimate[:, 10:13],
                        reference_truth[:, 13:16] - estimate[:, 13:16])
        bounds[mode] = (2 * std[:, 3:6], np.rad2deg(2 * std[:, 0:3]), 2 * std[:, 6:9], 2 * std[:, 9:12], 2 * std[:, 12:15])

    labels = ("Position error (m)", "Attitude error (deg)", "Velocity error (m/s)", "Gyro bias error (rad/s)",
              "Accel bias error (m/s²)")
    colors = {"openvins": "tab:blue", "factor_graph": "tab:orange", "hybrid": "tab:green"}
    figure, axes = plt.subplots(5, 3, figsize=(15, 16), sharex=True)
    for row in range(5):
        for axis in range(3):
            plot = axes[row, axis]
            for mode in MODES:
                plot.plot(time, errors[mode][row][:, axis], color=colors[mode], linewidth=1, label=mode.replace("_", " "))
                plot.plot(time, bounds[mode][row][:, axis], color=colors[mode], linestyle="--", linewidth=.7)
                plot.plot(time, -bounds[mode][row][:, axis], color=colors[mode], linestyle="--", linewidth=.7)
            for event in range_times:
                plot.axvline(event, color="grey", linestyle=":", linewidth=.7)
            plot.axhline(0, color="black", linewidth=.4)
            if row == 0:
                plot.set_title(("x", "y", "z")[axis])
            if axis == 0:
                plot.set_ylabel(labels[row])
            if row == 4:
                plot.set_xlabel("Time (s)")
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, legend_labels, loc="upper center", ncol=3)
    figure.suptitle(f"Estimator comparison — {args.agent}")
    figure.tight_layout(rect=(0, 0, 1, .96))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output)


if __name__ == "__main__":
    main()
