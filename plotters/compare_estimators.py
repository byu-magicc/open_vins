#!/usr/bin/env python3
"""Compare selected estimators at common valid timestamps for one agent.

Inputs are results_directory/{mode}/{agent}/estimate.csv and groundtruth.csv.
Use --modes to select two estimators, or --holoocean for exported HoloOcean CSVs.
Dashed curves show +/-2 standard deviations; --metrics saves error norms.
"""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from result_io import STATE_COLUMNS, load_csv, state_matrix as states

MODES = ("openvins", "factor_graph", "hybrid")


def load(path, covariance):
    required = ['timestamp', *STATE_COLUMNS]
    if covariance:
        required += ['valid', *(f'cov_{index}_{index}' for index in range(15))]
    return load_csv(path, required, False)


def select(data, timestamps):
    indices = {timestamp: index for index, timestamp in enumerate(data["timestamp"])}
    return data[[indices[timestamp] for timestamp in timestamps]]


def attitude_error(estimate, truth):
    return np.rad2deg((Rotation.from_quat(estimate).inv() * Rotation.from_quat(truth)).as_rotvec())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_directory", type=Path)
    parser.add_argument("agent")
    parser.add_argument("output", type=Path)
    parser.add_argument("--modes", nargs="+", choices=MODES, default=MODES)
    parser.add_argument("--holoocean", action="store_true", help="Compare exported HoloOcean CSVs without assumed zero bias truth")
    parser.add_argument("--metrics", type=Path, help="Write RMSE and final error norms to CSV")
    args = parser.parse_args()
    if args.output.suffix.lower() != ".svg":
        parser.error("output must have an .svg extension")
    modes = args.modes
    if len(modes) < 2 or len(set(modes)) != len(modes):
        parser.error("select at least two distinct modes")

    estimates = {mode: load(args.results_directory / mode / args.agent / "estimate.csv", True) for mode in modes}
    truths = {mode: load(args.results_directory / mode / args.agent / "groundtruth.csv", False) for mode in modes}
    columns = STATE_COLUMNS[:10] if args.holoocean else STATE_COLUMNS
    timestamps = estimates[modes[0]]["timestamp"]
    for mode in modes:
        timestamps = np.intersect1d(timestamps, estimates[mode]["timestamp"])
        timestamps = np.intersect1d(timestamps, truths[mode]["timestamp"])
    if timestamps.size < 2:
        raise ValueError("Fewer than two valid timestamps are common to all modes")
    aligned_estimates = {mode: select(estimates[mode], timestamps) for mode in modes}
    valid = np.logical_and.reduce([aligned_estimates[mode]["valid"] == 1 for mode in modes])
    timestamps = timestamps[valid]
    if timestamps.size < 2:
        raise ValueError("Fewer than two valid timestamps are common to all modes")
    aligned_estimates = {mode: data[valid] for mode, data in aligned_estimates.items()}
    aligned_truth = {mode: select(truths[mode], timestamps) for mode in modes}
    reference_truth = states(aligned_truth[modes[0]], columns)
    for mode in modes[1:]:
        if not np.allclose(states(aligned_truth[mode], columns), reference_truth, atol=1e-10, rtol=1e-10):
            raise ValueError(f"Ground truth differs in {mode}")

    range_times = []
    ranges_path = args.results_directory / "hybrid" / "ranges.csv"
    if "hybrid" in modes and ranges_path.is_file():
        with ranges_path.open(newline="") as stream:
            for event in csv.DictReader(stream):
                if event["owner"] == args.agent:
                    range_times.append(float(event["owner_timestamp"]) - timestamps[0])
                elif event["neighbor"] == args.agent:
                    range_times.append(float(event["neighbor_timestamp"]) - timestamps[0])

    time = timestamps - timestamps[0]
    estimates_state = {mode: states(data, columns) for mode, data in aligned_estimates.items()}
    errors = {}
    bounds = {}
    for mode in modes:
        estimate = estimates_state[mode]
        std = np.sqrt(np.maximum(0.0, np.column_stack([aligned_estimates[mode][f"cov_{i}_{i}"] for i in range(15)])))
        errors[mode] = (reference_truth[:, 4:7] - estimate[:, 4:7], attitude_error(estimate[:, :4], reference_truth[:, :4]),
                        reference_truth[:, 7:10] - estimate[:, 7:10])
        if not args.holoocean:
            errors[mode] += (reference_truth[:, 10:13] - estimate[:, 10:13], reference_truth[:, 13:16] - estimate[:, 13:16])
        bounds[mode] = (2 * std[:, 3:6], np.rad2deg(2 * std[:, 0:3]), 2 * std[:, 6:9], 2 * std[:, 9:12], 2 * std[:, 12:15])

    labels = ("Position error (m)", "Attitude error (deg)", "Velocity error (m/s)", "Gyro bias error (rad/s)",
              "Accel bias error (m/s²)")
    colors = {"openvins": "tab:blue", "factor_graph": "tab:orange", "hybrid": "tab:green"}
    rows = len(errors[modes[0]])
    figure, axes = plt.subplots(rows, 3, figsize=(15, 3.2 * rows), sharex=True)
    for row in range(rows):
        for axis in range(3):
            plot = axes[row, axis]
            for mode in modes:
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
            if row == rows - 1:
                plot.set_xlabel("Time (s)")
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, legend_labels, loc="upper center", ncol=len(modes))
    frame = " (ENU, fixed global frame)" if args.holoocean else ""
    figure.suptitle(f"Estimator comparison — {args.agent}{frame}")
    figure.tight_layout(rect=(0, 0, 1, .96))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output)
    plt.close(figure)
    print(f"Compared {timestamps.size} common valid samples over {time[-1]:.3f} seconds")
    if args.metrics is not None:
        args.metrics.parent.mkdir(parents=True, exist_ok=True)
        with args.metrics.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("agent", "mode", "quantity", "samples", "duration_s", "rmse", "final_error"))
            for mode in modes:
                for label, error in zip(labels, errors[mode]):
                    rmse = np.sqrt(np.mean(np.sum(error ** 2, axis=1)))
                    writer.writerow((args.agent, mode, label, timestamps.size, time[-1], rmse, np.linalg.norm(error[-1])))


if __name__ == "__main__":
    main()
