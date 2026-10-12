"""CSV and state handling shared by the standalone research plotters."""
import numpy as np

STATE_COLUMNS = ("q_x", "q_y", "q_z", "q_w", "p_x", "p_y", "p_z", "v_x", "v_y", "v_z",
                 "bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z")

def load_csv(path, required_columns, valid_only):
    if not path.is_file():
        raise ValueError(f"Missing input file: {path}")
    data = np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True, dtype=float))
    missing = sorted(set(required_columns) - set(data.dtype.names or ()))
    if missing:
        raise ValueError(f"{path} is missing columns: {', '.join(missing)}")
    if valid_only and 'valid' in (data.dtype.names or ()):
        data = data[data['valid'] == 1]
    if data.size == 0:
        raise ValueError(f"{path} contains no valid result rows")
    valid = data[data['valid'] == 1] if 'valid' in (data.dtype.names or ()) else data
    if not np.all(np.isfinite(np.column_stack([valid[column] for column in required_columns]))):
        raise ValueError(f'{path} contains non-finite state or covariance values')
    timestamps = np.asarray(data['timestamp'], dtype=float)
    if not np.all(np.isfinite(timestamps)) or np.unique(timestamps).size != timestamps.size:
        raise ValueError(f"{path} has invalid or duplicate timestamps")
    return data


def state_matrix(data, columns):
    state = np.column_stack([data[column] for column in columns])
    norms = np.linalg.norm(state[:, :4], axis=1)
    if np.any(norms == 0):
        raise ValueError("State quaternion has zero norm")
    state[:, :4] /= norms[:, None]
    return state
