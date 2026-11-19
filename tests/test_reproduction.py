"""Tests for experiments/verify_reproduction.py, using small temporary run files."""
import shutil

import pytest

from common.schema import COLUMNS, write_run
from experiments import verify_reproduction as vr
from experiments.verify_reproduction import EXPECTED_FILES, main


def make_rows(kind, mode, n=6):
    rows = []
    for i in range(n):
        segment = "arc" if (kind == "curve" and i >= n // 2) else "straight"
        rows.append({"t": i * 0.01, "x": i * 0.003, "y": 0.05 - i * 0.001, "yaw": -0.01 * i,
                     "e_true": 0.05 - i * 0.002, "e_sensed": 0.049 - i * 0.002, "u": -1.0 + 0.1 * i,
                     "controller": mode, "line_detected": 1, "segment": segment})
    return rows


@pytest.fixture
def dirs(tmp_path):
    """A baseline folder with the six valid files and an identical candidate folder."""
    baseline, candidate = tmp_path / "baseline", tmp_path / "candidate"
    for name in EXPECTED_FILES:
        kind, mode = name[:-4].split("_")
        write_run(baseline / name, make_rows(kind, mode))
    shutil.copytree(baseline, candidate)
    return baseline, candidate


def run(dirs, capsys):
    code = main([str(dirs[0]), str(dirs[1])])
    return code, capsys.readouterr().out


def edit(path, old, new, count=1):
    path.write_text(path.read_text().replace(old, new, count))


def test_identical_sets_pass(dirs, capsys):
    code, out = run(dirs, capsys)
    assert code == 0
    assert out.count("PASS") == 6 and "FAIL" not in out
    assert "6 of 6 files reproduce the baseline" in out


def test_crlf_line_endings_are_treated_as_equal(dirs, capsys):
    path = dirs[1] / "curve_PID.csv"
    path.write_bytes(path.read_bytes().replace(b"\r\n", b"\n").replace(b"\n", b"\r\n"))
    assert b"\r\n" in path.read_bytes()
    assert run(dirs, capsys)[0] == 0


def test_missing_final_newline_is_ignored(dirs, capsys):
    path = dirs[1] / "straight_P.csv"
    path.write_text(path.read_text().rstrip("\r\n"))
    assert run(dirs, capsys)[0] == 0


def test_missing_candidate_file_fails_and_names_it(dirs, capsys):
    (dirs[1] / "curve_PI.csv").unlink()
    code, out = run(dirs, capsys)
    assert code == 1
    assert "FAIL  curve_PI.csv" in out and "file is missing" in out
    assert out.count("PASS") == 5


def test_modified_value_reports_row_and_column(dirs, capsys):
    path = dirs[1] / "straight_PID.csv"
    lines = path.read_text().splitlines()
    fields = lines[3].split(",")
    fields[COLUMNS.index("e_true")] = "0.123"
    lines[3] = ",".join(fields)
    path.write_text("\n".join(lines) + "\n")
    code, out = run(dirs, capsys)
    assert code == 1
    assert "first difference at row 3, column e_true" in out


def test_only_the_first_difference_is_reported(dirs, capsys):
    path = dirs[1] / "straight_P.csv"
    lines = path.read_text().splitlines()
    for i in (2, 4, 5):
        fields = lines[i].split(",")
        fields[1] = "9.9"
        lines[i] = ",".join(fields)
    path.write_text("\n".join(lines) + "\n")
    out = run(dirs, capsys)[1]
    assert "row 2, column x" in out and "row 4" not in out


def test_row_count_difference_fails_with_both_counts(dirs, capsys):
    path = dirs[1] / "curve_P.csv"
    lines = path.read_text().splitlines()
    path.write_text("\n".join(lines[:-2]) + "\n")  # drop the last two data rows
    code, out = run(dirs, capsys)
    assert code == 1
    assert "row count differs, baseline 6 vs candidate 4" in out


def test_wrong_header_order_fails(dirs, capsys):
    path = dirs[1] / "straight_P.csv"
    edit(path, "t,x,y,yaw", "x,t,y,yaw")
    code, out = run(dirs, capsys)
    assert code == 1 and "header" in out


def test_empty_file_fails(dirs, capsys):
    (dirs[1] / "straight_PI.csv").write_text("")
    code, out = run(dirs, capsys)
    assert code == 1 and "file is empty" in out


def test_header_only_file_fails(dirs, capsys):
    path = dirs[1] / "straight_PI.csv"
    path.write_text(path.read_text().splitlines()[0] + "\n")
    assert "no data rows" in run(dirs, capsys)[1]


def test_short_row_fails(dirs, capsys):
    path = dirs[1] / "curve_PID.csv"
    lines = path.read_text().splitlines()
    lines[2] = ",".join(lines[2].split(",")[:-1])
    path.write_text("\n".join(lines) + "\n")
    out = run(dirs, capsys)[1]
    assert "row 2 has 9 values, expected 10" in out


@pytest.mark.parametrize("bad", ["nan", "inf", "-inf"])
def test_non_finite_value_fails(dirs, capsys, bad):
    path = dirs[1] / "straight_P.csv"
    lines = path.read_text().splitlines()
    fields = lines[2].split(",")
    fields[COLUMNS.index("u")] = bad
    lines[2] = ",".join(fields)
    path.write_text("\n".join(lines) + "\n")
    code, out = run(dirs, capsys)
    assert code == 1 and "column u is not finite" in out


def test_text_in_numeric_column_fails(dirs, capsys):
    path = dirs[1] / "straight_P.csv"
    lines = path.read_text().splitlines()
    fields = lines[2].split(",")
    fields[COLUMNS.index("yaw")] = "abc"
    lines[2] = ",".join(fields)
    path.write_text("\n".join(lines) + "\n")
    assert "column yaw is not a number" in run(dirs, capsys)[1]


def test_wrong_controller_label_fails(dirs, capsys):
    path = dirs[1] / "straight_PI.csv"
    edit(path, ",PI,", ",PID,", count=1)
    assert "controller is 'PID', expected 'PI'" in run(dirs, capsys)[1]


def test_invalid_line_detected_fails(dirs, capsys):
    path = dirs[1] / "straight_P.csv"
    edit(path, ",1,straight", ",2,straight", count=1)
    assert "line_detected is '2'" in run(dirs, capsys)[1]


def test_unknown_segment_fails(dirs, capsys):
    path = dirs[1] / "curve_P.csv"
    edit(path, ",arc", ",zigzag", count=1)
    assert "segment is 'zigzag'" in run(dirs, capsys)[1]


def test_arc_row_in_straight_file_fails(dirs, capsys):
    path = dirs[1] / "straight_P.csv"
    edit(path, ",straight\n", ",arc\n", count=1)
    path_text = path.read_text()
    assert ",arc" in path_text
    assert "on the arc in a straight-track file" in run(dirs, capsys)[1]


def test_a_broken_baseline_is_reported_as_baseline(dirs, capsys):
    (dirs[0] / "curve_P.csv").write_text("")
    out = run(dirs, capsys)[1]
    assert "baseline curve_P.csv: file is empty" in out


def test_many_problems_in_one_file_are_capped(dirs, capsys):
    path = dirs[1] / "straight_P.csv"
    text = path.read_text().replace(",P,", ",PID,")
    path.write_text(text)
    out = run(dirs, capsys)[1]
    assert "further problems not listed" in out


def test_missing_directory_exits_with_status_2(tmp_path, capsys):
    code = main([str(tmp_path / "nope"), str(tmp_path)])
    assert code == 2
    assert "is not a directory" in capsys.readouterr().err


def test_first_difference_helper_directly():
    header = ",".join(COLUMNS)
    same = [header, "0,0,0,0,0,0,0,P,1,straight"]
    assert vr.first_difference(same, list(same)) is None
    other = [header, "0,0,0,0,0,0,5,P,1,straight"]
    assert "row 1, column u" in vr.first_difference(same, other)
    assert "header line differs" in vr.first_difference(same, ["x"] + same[1:])
