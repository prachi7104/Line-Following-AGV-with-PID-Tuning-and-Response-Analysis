"""Check that a set of experiment results reproduces a baseline set.

    python -m experiments.verify_reproduction BASELINE_DIR CANDIDATE_DIR

Both directories must hold the six standard run files
(straight_P/PI/PID.csv and curve_P/PI/PID.csv). For each file the checker verifies:

    * the file exists and is not empty
    * the header is exactly the project's column list, in order
    * every row has one value per column
    * the numeric columns are finite numbers
    * the controller label equals the mode in the file name
    * line_detected is 0 or 1 and segment is "straight" or "arc"
      (a straight-track file contains no "arc" rows)
    * the candidate has the same number of rows as the baseline
    * the candidate's text equals the baseline's after normalising line endings
      (CRLF and LF are treated alike; a missing final newline is ignored)

The first difference is reported with its row number and column name. The check is
deliberately strict: it says whether the files are the same, not whether a small
numerical difference is acceptable.

Exit status: 0 all checks passed, 1 at least one check failed, 2 a directory is missing.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

from common.schema import COLUMNS

MODES = ("P", "PI", "PID")
KINDS = ("straight", "curve")
EXPECTED_FILES = [f"{kind}_{mode}.csv" for kind in KINDS for mode in MODES]
NUMERIC_COLUMNS = ["t", "x", "y", "yaw", "e_true", "e_sensed", "u"]
VALID_SEGMENTS = {"straight", "arc"}


def _read_lines(path: Path) -> list[str]:
    """Lines of a text file with LF/CRLF endings and the final newline normalised away."""
    return path.read_text(encoding="utf-8").splitlines()


def check_structure(path: Path, label: str) -> list[str]:
    """Problems found in one run file on its own (no comparison with another file)."""
    problems: list[str] = []
    kind, _, mode = path.stem.partition("_")
    if not path.exists():
        return [f"{label}: file is missing"]
    lines = _read_lines(path)
    if not lines:
        return [f"{label}: file is empty"]

    header = next(csv.reader([lines[0]]))
    if header != COLUMNS:
        return [f"{label}: header {header} does not match {COLUMNS}"]
    if len(lines) == 1:
        return [f"{label}: has a header but no data rows"]

    for row_number, fields in enumerate(csv.reader(lines[1:]), start=1):
        if len(fields) != len(COLUMNS):
            problems.append(f"{label}: row {row_number} has {len(fields)} values, expected {len(COLUMNS)}")
            continue
        row = dict(zip(COLUMNS, fields))
        for column in NUMERIC_COLUMNS:
            try:
                finite = math.isfinite(float(row[column]))
            except ValueError:
                problems.append(f"{label}: row {row_number} column {column} is not a number ({row[column]!r})")
                continue
            if not finite:
                problems.append(f"{label}: row {row_number} column {column} is not finite ({row[column]!r})")
        if row["controller"] != mode:
            problems.append(f"{label}: row {row_number} controller is {row['controller']!r}, expected {mode!r}")
        if row["line_detected"] not in ("0", "1"):
            problems.append(f"{label}: row {row_number} line_detected is {row['line_detected']!r}, expected 0 or 1")
        if row["segment"] not in VALID_SEGMENTS:
            problems.append(f"{label}: row {row_number} segment is {row['segment']!r}")
        elif kind == "straight" and row["segment"] != "straight":
            problems.append(f"{label}: row {row_number} is on the arc in a straight-track file")
        if len(problems) >= 5:  # a badly broken file would otherwise flood the report
            problems.append(f"{label}: further problems not listed")
            break
    return problems


def first_difference(baseline: list[str], candidate: list[str]) -> str | None:
    """Describe the first differing data row, or return None if the lines are equal."""
    for index, (b, c) in enumerate(zip(baseline, candidate)):
        if b != c:
            if index == 0:
                return "header line differs"
            b_fields, c_fields = next(csv.reader([b])), next(csv.reader([c]))
            for column, b_value, c_value in zip(COLUMNS, b_fields, c_fields):
                if b_value != c_value:
                    return f"first difference at row {index}, column {column}: baseline {b_value} vs candidate {c_value}"
            return f"first difference at row {index}: baseline {b} vs candidate {c}"
    if len(baseline) != len(candidate):
        return f"identical for the first {min(len(baseline), len(candidate)) - 1} rows, then one file ends"
    return None


def verify(baseline_dir: Path, candidate_dir: Path) -> dict[str, list[str]]:
    """Run all checks. Returns {file name: [problems]}; an empty list means the file passed."""
    results: dict[str, list[str]] = {}
    for name in EXPECTED_FILES:
        base_path, cand_path = Path(baseline_dir) / name, Path(candidate_dir) / name
        problems = check_structure(base_path, f"baseline {name}")
        problems += check_structure(cand_path, f"candidate {name}")
        if not problems:
            base_lines, cand_lines = _read_lines(base_path), _read_lines(cand_path)
            if len(base_lines) != len(cand_lines):
                problems.append(f"{name}: row count differs, baseline {len(base_lines) - 1} vs candidate {len(cand_lines) - 1}")
            difference = first_difference(base_lines, cand_lines)
            if difference:
                problems.append(f"{name}: {difference}")
        results[name] = problems
    return results


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("baseline", type=Path, help="folder with the reference run CSVs")
    parser.add_argument("candidate", type=Path, help="folder with the run CSVs to check")
    args = parser.parse_args(argv)

    for folder in (args.baseline, args.candidate):
        if not folder.is_dir():
            print(f"error: {folder} is not a directory", file=sys.stderr)
            return 2

    results = verify(args.baseline, args.candidate)
    for name, problems in results.items():
        print(f"{'PASS' if not problems else 'FAIL'}  {name}")
        for problem in problems:
            print(f"      {problem}")
    failed = sum(1 for problems in results.values() if problems)
    print(f"\n{len(results) - failed} of {len(results)} files reproduce the baseline"
          + ("" if failed == 0 else f"; {failed} do not"))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
