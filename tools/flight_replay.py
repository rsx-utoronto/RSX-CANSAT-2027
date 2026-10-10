#!/usr/bin/env python3
"""Offline CSV replay / explicit parameter comparison using the production C++ detector.

Python owns file parsing and metrics only. It never reimplements flight detection.
"""
import argparse
import collections
import csv
import hashlib
import io
import itertools
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile

PARAMETERS = (
    "max_sample_age_ms", "max_sample_gap_ms", "max_abs_vertical_speed_mps",
    "launch_altitude_m", "launch_min_climb_mps", "launch_confirm_ms",
    "apogee_min_drop_m", "apogee_min_descent_mps", "apogee_confirm_ms",
    "landing_altitude_min_m", "landing_altitude_max_m", "landing_max_abs_speed_mps",
    "landing_max_span_m", "landing_confirm_ms", "min_confirm_samples",
)
UINT_PARAMETERS = {key for key in PARAMETERS if key.endswith("_ms")} | {"min_confirm_samples"}
EVENTS = ("launch", "apogee", "landed")
PHASES = {"waiting_for_launch", "ascending", "descending", "landed"}
STATUSES = {"invalid_configuration", "clock_regression", "missing", "nonfinite", "future",
            "stale", "out_of_order", "conflicting_duplicate", "duplicate", "seeded",
            "gap_reseeded", "implausible_rate", "accepted"}
BACKEND_FIELDS = ["row", "phase", "status", "event", "qualified_altitude_m", "vertical_speed_mps", "peak_altitude_m"]
REQUIRED_COLUMNS = ["now_ms", "sampled_at_ms", "altitude_m", "armed"]
MAX_ROWS, MAX_RUNS, MAX_TOTAL_ROWS = 100000, 64, 250000
U64_MAX = (1 << 64) - 1


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read_bytes(path, limit):
    with path.open("rb") as file:
        data = file.read(limit + 1)
    if len(data) > limit:
        raise ValueError(f"{path.name}: file exceeds {limit} byte limit")
    return data


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def parse_json(data):
    def invalid_constant(value):
        raise ValueError(f"non-standard JSON constant: {value}")
    return json.loads(data.decode("utf-8-sig"), object_pairs_hook=unique_object, parse_constant=invalid_constant)


def uint(value, label):
    if not re.fullmatch(r"[0-9]+", value) or len(value) > 20 or int(value) > U64_MAX:
        raise ValueError(f"{label}: expected uint64 decimal")
    return int(value)


def numeric_token(value):
    if isinstance(value, int):
        return str(value)
    return format(value, ".17g")


def parameter(key, value):
    if key in UINT_PARAMETERS:
        if type(value) is not int or not 0 <= value <= U64_MAX:
            raise ValueError(f"{key}: expected uint64 JSON integer")
    elif type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{key}: expected finite JSON number")
    return value


def load_config(data):
    config = parse_json(data)
    if not isinstance(config, dict) or set(config) != {"schema_version", "source", "start", "parameters"}:
        raise ValueError("config requires schema_version, source, start and parameters only")
    if type(config["schema_version"]) is not int or config["schema_version"] != 1:
        raise ValueError("unsupported config schema_version")
    if not isinstance(config["source"], str) or not config["source"].strip() or len(config["source"]) > 2000:
        raise ValueError("config source must describe provenance (max 2000 characters)")
    if config["start"] not in ("on_pad", "descending_after_release"):
        raise ValueError("invalid detector start")
    if not isinstance(config["parameters"], dict) or set(config["parameters"]) != set(PARAMETERS):
        raise ValueError("config must supply every detector parameter, with no unknown keys")
    for key, value in config["parameters"].items():
        parameter(key, value)
    return config


def load_trace(data):
    reader = csv.reader(io.StringIO(data.decode("utf-8-sig"), newline=""), strict=True)
    header = next(reader, [])
    if len(set(header)) != len(header) or set(header) not in (
            set(REQUIRED_COLUMNS), set(REQUIRED_COLUMNS + ["expected_event"])):
        raise ValueError("CSV requires now_ms,sampled_at_ms,altitude_m,armed and optional expected_event; no other columns")
    rows, labels, labeled_rows = [], {}, 0
    for line_number, fields in enumerate(reader, start=2):
        if len(rows) >= MAX_ROWS or len(fields) != len(header):
            raise ValueError(f"CSV row {line_number}: incorrect column count or row limit exceeded")
        record = {key: value.strip() for key, value in zip(header, fields)}
        now = uint(record["now_ms"], f"row {line_number} now_ms")
        sampled, height = record["sampled_at_ms"], record["altitude_m"]
        if bool(sampled) != bool(height):
            raise ValueError(f"CSV row {line_number}: missing sample requires both timestamp and altitude blank")
        if sampled:
            sampled = uint(sampled, f"row {line_number} sampled_at_ms")
            # Decimal/scientific numbers and explicit nonfinite fault values only.
            if not re.fullmatch(r"[+-]?(?:(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?|nan|inf|infinity)", height, re.IGNORECASE):
                raise ValueError(f"CSV row {line_number}: invalid altitude number")
            parsed_height = float(height)
            if not math.isfinite(parsed_height) and not re.fullmatch(r"[+-]?(nan|inf|infinity)", height, re.IGNORECASE):
                raise ValueError(f"CSV row {line_number}: altitude overflow; use explicit inf for fault injection")
            height = numeric_token(parsed_height)
        else:
            sampled = None
        if record["armed"] not in ("0", "1"):
            raise ValueError(f"CSV row {line_number}: armed must be 0 or 1")
        label = record.get("expected_event", "")
        if label:
            labeled_rows += 1
            if label not in ("none",) + EVENTS:
                raise ValueError(f"CSV row {line_number}: invalid expected_event")
            if label != "none":
                if label in labels:
                    raise ValueError(f"CSV row {line_number}: expected event {label} repeated; one flight per trace")
                labels[label] = now
        rows.append({"now_ms": now, "sampled_at_ms": sampled, "altitude_m": height,
                     "armed": record["armed"], "expected_event": label})
    if not rows:
        raise ValueError("CSV has no samples")
    if labeled_rows not in (0, len(rows)):
        raise ValueError("labels must cover every row (use none), or all be absent; partial labels are not ground truth")
    return rows, labels if labeled_rows else None


def configurations(base, grid_data):
    configs = [dict(base)]
    if grid_data is None:
        return configs
    grid = parse_json(grid_data)
    if not isinstance(grid, dict) or set(grid) != {"schema_version", "parameters"} or type(grid["schema_version"]) is not int or grid["schema_version"] != 1:
        raise ValueError("grid requires schema_version=1 and parameters")
    values = grid["parameters"]
    if not isinstance(values, dict) or not values or not set(values) <= set(PARAMETERS):
        raise ValueError("grid must vary known detector parameters")
    total = 1
    for key, items in values.items():
        if not isinstance(items, list) or not items:
            raise ValueError(f"grid {key}: nonempty list required")
        for value in items:
            parameter(key, value)
        if len(set(items)) != len(items):
            raise ValueError(f"grid {key}: duplicate values")
        total *= len(items)
    if total > MAX_RUNS:
        raise ValueError("grid exceeds 64 combinations")
    keys = sorted(values)
    seen = {tuple(base[key] for key in PARAMETERS)}
    for combination in itertools.product(*(values[key] for key in keys)):
        candidate = dict(base, **dict(zip(keys, combination)))
        identity = tuple(candidate[key] for key in PARAMETERS)
        if identity not in seen:
            seen.add(identity)
            configs.append(candidate)
    if len(configs) > MAX_RUNS:
        raise ValueError("baseline plus grid exceeds 64 unique runs")
    return configs


def run_detector(backend, start, parameters, rows):
    lines = ["CONFIG " + start + " " + " ".join(numeric_token(parameters[key]) for key in PARAMETERS)]
    for row in rows:
        sample = "- -" if row["sampled_at_ms"] is None else f"{row['sampled_at_ms']} {row['altitude_m']}"
        lines.append(f"S {row['now_ms']} {sample} {row['armed']}")
    result = subprocess.run([str(backend), "--protocol-v1"], input="\n".join(lines) + "\n",
                            capture_output=True, text=True, timeout=30, check=False)
    if result.returncode != 0:
        raise ValueError(f"backend exit {result.returncode}: {result.stderr.strip()[:1000]}")
    if result.stderr:
        raise ValueError(f"unexpected backend stderr: {result.stderr[:1000]}")
    reader = csv.DictReader(io.StringIO(result.stdout))
    if reader.fieldnames != BACKEND_FIELDS:
        raise ValueError("incompatible backend output header")
    decisions = list(reader)
    if len(decisions) != len(rows):
        raise ValueError("backend output row count mismatch")
    events, counts = {}, collections.Counter()
    for index, decision in enumerate(decisions, start=1):
        if set(decision) != set(BACKEND_FIELDS) or None in decision.values() or decision["row"] != str(index):
            raise ValueError("malformed backend output row")
        if decision["phase"] not in PHASES or decision["status"] not in STATUSES or decision["event"] not in ("none",) + EVENTS:
            raise ValueError("unknown backend state")
        for key in BACKEND_FIELDS[4:]:
            if decision[key] and not math.isfinite(float(decision[key])):
                raise ValueError("nonfinite qualified backend output")
        counts[decision["status"]] += 1
        kind = decision["event"]
        if kind != "none":
            if kind in events:
                raise ValueError("backend repeated a one-shot event")
            events[kind] = {"now_ms": rows[index - 1]["now_ms"], "sampled_at_ms": rows[index - 1]["sampled_at_ms"], "row": index}
    return decisions, events, dict(sorted(counts.items()))


def evaluate(events, labels, window_ms):
    if labels is None:
        return None
    metrics = {"matched": 0, "missed": 0, "false_events": 0, "events": {}}
    for kind in EVENTS:
        expected = labels.get(kind)
        observed = events[kind]["now_ms"] if kind in events else None
        latency = None if expected is None or observed is None else observed - expected
        matched = latency is not None and abs(latency) <= window_ms
        metrics["matched"] += int(matched)
        metrics["missed"] += int(expected is not None and not matched)
        metrics["false_events"] += int(observed is not None and not matched)
        metrics["events"][kind] = {"expected_at_ms": expected, "detected_at_ms": observed,
                                   "latency_ms": latency, "matched": matched}
    return metrics


def write_csv(path, columns, records):
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(records)


def execute(args):
    trace, config_path, backend, out = (Path(value).expanduser().resolve() for value in
                                        (args.trace, args.config, args.backend, args.output_dir))
    if out.exists():
        raise ValueError("output directory must be new; existing results are never overwritten")
    trace_data, config_data = read_bytes(trace, 20 * 1024 * 1024), read_bytes(config_path, 65536)
    config = load_config(config_data)
    rows, labels = load_trace(trace_data)
    if labels is not None and args.match_window_ms is None:
        raise ValueError("labeled traces require explicit --match-window-ms")
    grid_path = Path(args.grid).expanduser().resolve() if args.command == "tune" else None
    grid_data = read_bytes(grid_path, 65536) if grid_path else None
    candidates = configurations(config["parameters"], grid_data)
    if len(rows) * len(candidates) > MAX_TOTAL_ROWS:
        raise ValueError("comparison exceeds 250000 total sample evaluations")
    backend_sha = digest(backend.read_bytes())
    inputs = {"trace": {"path": str(trace), "sha256": digest(trace_data)},
              "config": {"path": str(config_path), "sha256": digest(config_data)},
              "backend": {"path": str(backend), "sha256": backend_sha},
              "driver": {"path": str(Path(__file__).resolve()), "sha256": digest(Path(__file__).read_bytes())}}
    if grid_path:
        inputs["grid"] = {"path": str(grid_path), "sha256": digest(grid_data)}
    summaries, comparison = [], []
    out.parent.mkdir(parents=True, exist_ok=True)
    # Backend/validation failures leave no published output directory.
    with tempfile.TemporaryDirectory(prefix=".flight-replay-", dir=out.parent) as temp:
        staging = Path(temp)
        for index, params in enumerate(candidates):
            run_id = "baseline" if index == 0 else f"candidate_{index:03d}"
            decisions, events, counts = run_detector(backend, config["start"], params, rows)
            metrics = evaluate(events, labels, args.match_window_ms)
            merged = [dict(row, **decision) for row, decision in zip(rows, decisions)]
            columns = ["row"] + REQUIRED_COLUMNS + ["expected_event"] + BACKEND_FIELDS[1:]
            filename = run_id + "_samples.csv"
            write_csv(staging / filename, columns, merged)
            summary = {"run_id": run_id, "parameters": params, "events": events,
                       "status_counts": counts, "metrics": metrics, "samples_file": filename}
            summaries.append(summary)
            record = {"run_id": run_id, **params}
            for kind in EVENTS:
                record[kind + "_at_ms"] = events[kind]["now_ms"] if kind in events else None
                record[kind + "_latency_ms"] = metrics["events"][kind]["latency_ms"] if metrics else None
            record.update({key: metrics[key] if metrics else None for key in ("matched", "missed", "false_events")})
            comparison.append(record)
        write_csv(staging / "comparison.csv", list(comparison[0]), comparison)
        if digest(backend.read_bytes()) != backend_sha:
            raise ValueError("backend changed during replay")
        files = {file.name: {"sha256": digest(file.read_bytes()), "bytes": file.stat().st_size}
                 for file in sorted(staging.iterdir())}
        report = {"schema_version": 1, "command": args.command, "source": config["source"],
                  "start": config["start"], "row_count": len(rows), "inputs": inputs,
                  "labeled": labels is not None, "match_window_ms": args.match_window_ms,
                  "timing_basis": "expected_event labels and emitted decisions use row now_ms; signed latency=detected-expected",
                  "selection": "comparison only; no automatic best config or flight approval",
                  "runs": summaries, "files": files}
        (staging / "summary.json").write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        out.mkdir(exist_ok=False)
        # A disk/write failure here may leave a partial folder; never delete user files.
        # summary.json is moved last and serves as the completion manifest.
        names = sorted(file.name for file in staging.iterdir() if file.name != "summary.json") + ["summary.json"]
        for name in names:
            (staging / name).rename(out / name)
    return {"output_dir": str(out), "summary": str(out / "summary.json"), "runs": len(summaries), "labeled": labels is not None}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("replay", "tune"):
        command = sub.add_parser(name)
        for option in ("trace", "config", "backend", "output-dir"):
            command.add_argument("--" + option, required=True)
        command.add_argument("--match-window-ms", type=lambda value: uint(value, "match window"))
        if name == "tune":
            command.add_argument("--grid", required=True)
    args = parser.parse_args(argv)
    try:
        result = execute(args)
    except (ValueError, OverflowError, OSError, csv.Error, subprocess.SubprocessError) as error:
        print(f"flight replay: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
