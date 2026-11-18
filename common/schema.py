"""CSV log format shared by the experiment runner and the analysis scripts.

The first nine columns are the schema agreed by the team. `segment` is appended
(never reordering the agreed columns) so curve metrics can be computed on the arc only.
"""
import csv
from pathlib import Path

import pandas as pd

COLUMNS: list[str] = [
    "t",              # simulation time [s]
    "x", "y", "yaw",  # robot pose: axle midpoint [m], heading [rad]
    "e_true",         # signed ground-truth lateral error from pose + track geometry [m]
    "e_sensed",       # signed error the controller used (held value when line lost) [m]
    "u",              # commanded yaw rate [rad/s]
    "controller",     # "P" | "PI" | "PID"
    "line_detected",  # 1 if any sensor saw the line this step, else 0
    "segment",        # "straight" | "arc"
]


def write_run(path, rows: list[dict]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="raise")
        writer.writeheader()
        for row in rows:
            missing = set(COLUMNS) - set(row)
            if missing:
                raise ValueError(f"row is missing columns: {sorted(missing)}")
            writer.writerow(row)


def read_run(path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if list(df.columns) != COLUMNS:
        raise ValueError(f"{path}: columns {list(df.columns)} do not match schema {COLUMNS}")
    return df
