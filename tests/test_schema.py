import numpy as np
import pytest

from common.config import DEFAULT, GAINS
from common.schema import COLUMNS, read_run, write_run


def _row(t):
    return {
        "t": t, "x": 0.1 * t, "y": 0.05, "yaw": 0.0, "e_true": 0.05, "e_sensed": 0.049,
        "u": -1.0, "controller": "P", "line_detected": 1, "segment": "straight",
    }


def test_columns_exact_order():
    assert COLUMNS == ["t", "x", "y", "yaw", "e_true", "e_sensed", "u",
                       "controller", "line_detected", "segment"]


def test_roundtrip(tmp_path):
    path = tmp_path / "run.csv"
    rows = [_row(i * 0.1) for i in range(3)]
    write_run(path, rows)
    df = read_run(path)
    assert list(df.columns) == COLUMNS
    assert len(df) == 3
    assert df["e_sensed"].iloc[2] == pytest.approx(0.049)
    assert df["controller"].iloc[0] == "P"


def test_write_rejects_missing_column(tmp_path):
    row = _row(0.0)
    del row["segment"]
    with pytest.raises(ValueError):
        write_run(tmp_path / "bad.csv", [row])


def test_read_rejects_bad_header(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("t,y,x\n0,0,0\n")
    with pytest.raises(ValueError):
        read_run(path)


def test_sensor_centres():
    centres = DEFAULT.sensor_centres()
    assert np.allclose(centres, [-0.06, -0.03, 0.0, 0.03, 0.06])
    # the line (half width included) must stay under the bar at the 5 cm start offset
    half_coverage = centres.max() + DEFAULT.sensor_spacing / 2
    assert half_coverage > DEFAULT.initial_offset + DEFAULT.line_width / 2


def test_gains_cover_three_modes():
    assert set(GAINS) == {"P", "PI", "PID"}
    assert GAINS["P"][1] == 0 and GAINS["P"][2] == 0
    assert GAINS["PI"][2] == 0
