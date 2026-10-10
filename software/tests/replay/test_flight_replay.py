import csv
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
DRIVER = ROOT / "tools/flight_replay.py"
FIXTURES = Path(__file__).parent / "fixtures"
BACKEND = Path(sys.argv.pop(1)).resolve()
spec = importlib.util.spec_from_file_location("flight_replay", DRIVER)
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="cansat-replay-test-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.count = 0

    def csv(self, text):
        self.count += 1
        path = self.root / f"trace{self.count}.csv"
        path.write_bytes(text.encode("utf-8"))
        return path

    def config(self, mutate=None):
        config = json.loads((FIXTURES / "synthetic_config.json").read_text())
        if mutate:
            mutate(config)
        self.count += 1
        path = self.root / f"config{self.count}.json"
        path.write_text(json.dumps(config))
        return path

    def cli(self, command="replay", trace=None, config=None, grid=None, window=5000, backend=BACKEND, output=None):
        self.count += 1
        out = output or self.root / f"output{self.count}"
        args = [sys.executable, "-B", str(DRIVER), command, "--trace", str(trace or FIXTURES / "synthetic_flight.csv"),
                "--config", str(config or FIXTURES / "synthetic_config.json"), "--backend", str(backend), "--output-dir", str(out)]
        if command == "tune":
            args += ["--grid", str(grid or FIXTURES / "synthetic_grid.json")]
        if window is not None:
            args += ["--match-window-ms", str(window)]
        result = subprocess.run(args, capture_output=True, text=True, timeout=40)
        return result, out

    def success(self, **kwargs):
        result, out = self.cli(**kwargs)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, "")
        response = json.loads(result.stdout)
        self.assertEqual(response["summary"], str(out / "summary.json"))
        return json.loads((out / "summary.json").read_text()), out

    def failure(self, **kwargs):
        result, out = self.cli(**kwargs)
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)
        self.assertIn("flight replay:", result.stderr)
        self.assertEqual(result.stdout, "")
        self.assertFalse(out.exists())
        return result

    def test_native_parity_and_provenance(self):
        report, out = self.success()
        self.assertEqual(report["row_count"], 18)
        run = report["runs"][0]
        self.assertEqual({k: v["now_ms"] for k, v in run["events"].items()}, {"launch": 4000, "apogee": 10000, "landed": 17000})
        self.assertEqual(run["status_counts"], {"accepted": 17, "seeded": 1})
        self.assertEqual(run["metrics"]["matched"], 3)
        self.assertEqual(run["metrics"]["events"]["apogee"]["latency_ms"], 4000)
        for info in report["inputs"].values():
            self.assertEqual(hashlib.sha256(Path(info["path"]).read_bytes()).hexdigest(), info["sha256"])
        for name, info in report["files"].items():
            data = (out / name).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), info["sha256"])
            self.assertEqual(len(data), info["bytes"])
        decisions = list(csv.DictReader(io.StringIO((out / "baseline_samples.csv").read_text())))
        self.assertEqual(len(decisions), 18)
        self.assertEqual(decisions[0]["qualified_altitude_m"], "")
        self.assertEqual(decisions[10]["event"], "apogee")

    def test_grid_comparison_no_automatic_selection(self):
        report, out = self.success(command="tune")
        self.assertEqual([run["parameters"]["apogee_min_drop_m"] for run in report["runs"]], [5, 1, 10])
        self.assertEqual([run["events"]["apogee"]["now_ms"] for run in report["runs"]], [10000, 9000, 11000])
        self.assertIn("no automatic best", report["selection"])
        self.assertEqual(len(list(csv.DictReader(io.StringIO((out / "comparison.csv").read_text())))), 3)

    def test_inclusive_window_and_late_event_counts(self):
        report, _ = self.success(command="tune", window=3000)
        baseline, earlier, later = [run["metrics"] for run in report["runs"]]
        self.assertEqual((baseline["matched"], baseline["missed"], baseline["false_events"]), (2, 1, 1))
        self.assertEqual((earlier["matched"], earlier["missed"], earlier["false_events"]), (3, 0, 0))
        self.assertEqual(later["events"]["apogee"]["latency_ms"], 5000)

    def test_unlabeled_metrics_are_null_not_zero(self):
        original = (FIXTURES / "synthetic_flight.csv").read_text().splitlines()
        trace = self.csv("\n".join(line.rsplit(",", 1)[0] for line in original) + "\n")
        report, out = self.success(trace=trace, window=None)
        self.assertFalse(report["labeled"])
        self.assertIsNone(report["runs"][0]["metrics"])
        row = next(csv.DictReader(io.StringIO((out / "comparison.csv").read_text())))
        self.assertEqual(row["missed"], "")
        self.assertEqual(row["apogee_latency_ms"], "")

    def test_labels_require_explicit_tolerance(self):
        self.assertIn("--match-window-ms", self.failure(window=None).stderr)

    def test_partial_or_repeated_labels_rejected(self):
        for labels in [("launch", ""), ("launch", "launch"), ("none", "invalid")]:
            with self.subTest(labels=labels):
                self.failure(trace=self.csv(f"now_ms,sampled_at_ms,altitude_m,armed,expected_event\n0,0,0,1,{labels[0]}\n1,1,1,1,{labels[1]}\n"))

    def test_csv_bom_quotes_crlf_and_column_order(self):
        trace = self.csv('\ufeffarmed,altitude_m,now_ms,sampled_at_ms\r\n"1","0","0","0"\r\n1,"10",1000,1000\r\n')
        report, _ = self.success(trace=trace, window=None)
        self.assertEqual(report["runs"][0]["status_counts"], {"accepted": 1, "seeded": 1})

    def test_bad_csv_is_not_silently_repaired(self):
        header = "now_ms,sampled_at_ms,altitude_m,armed\n"
        invalid = ["", header, "now_ms,altitude_m,armed\n0,0,1\n",
                   "now_ms,sampled_at_ms,altitude_m,armed,armed\n0,0,0,1,1\n",
                   header + "0,0,0,1,extra\n", header + "\n", header + "-1,0,0,1\n",
                   header + "18446744073709551616,0,0,1\n", header + "0,0,0,true\n",
                   header + "0,,0,1\n", header + "0,0,,1\n", header + "0,0,1e9999,1\n",
                   header + '0,0,"=SUM(A1)",1\n', header + '0,0,"unclosed,1\n']
        for text in invalid:
            with self.subTest(text=text):
                self.failure(trace=self.csv(text), window=None)

    def test_fault_samples_reach_cpp_without_sorting_or_interpolation(self):
        text = "now_ms,sampled_at_ms,altitude_m,armed\n0,0,0,1\n1000,1000,10,1\n1100,1000,10,1\n1200,1000,11,1\n1300,,,1\n1400,1400,nan,1\n1500,1500,inf,1\n1600,1700,10,1\n2101,1000,10,1\n2200,2200,10,1\n2300,2100,10,1\n2400,2400,10,1\n2300,2300,10,1\n2500,2500,10,1\n2600,2600,10000,1\n"
        report, out = self.success(trace=self.csv(text), window=None)
        decisions = list(csv.DictReader(io.StringIO((out / "baseline_samples.csv").read_text())))
        self.assertEqual([row["status"] for row in decisions], ["seeded", "accepted", "duplicate", "conflicting_duplicate", "missing", "nonfinite", "nonfinite", "future", "stale", "seeded", "out_of_order", "seeded", "clock_regression", "seeded", "implausible_rate"])
        self.assertEqual(report["runs"][0]["events"], {})

    def test_bad_configuration_rejected_before_publication(self):
        mutations = [lambda c: c["parameters"].pop("launch_altitude_m"),
                     lambda c: c["parameters"].update(unknown=1),
                     lambda c: c["parameters"].update(launch_confirm_ms=True),
                     lambda c: c["parameters"].update(launch_confirm_ms=2000.0),
                     lambda c: c["parameters"].update(launch_confirm_ms=0),
                     lambda c: c["parameters"].update(landing_altitude_max_m=20),
                     lambda c: c["parameters"].update(apogee_min_drop_m=float("nan")),
                     lambda c: c.update(schema_version=True), lambda c: c.update(start="guess"),
                     lambda c: c.update(source="")]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                self.failure(config=self.config(mutate))

    def test_duplicate_json_keys_rejected(self):
        config = self.config()
        config.write_text(config.read_text().replace('"schema_version": 1', '"schema_version": 1, "schema_version": 1'))
        self.assertIn("duplicate JSON", self.failure(config=config).stderr)

    def test_grid_validation_and_no_partial_output(self):
        grid = self.root / "grid.json"
        for values in [{}, {"wrong": [1]}, {"apogee_min_drop_m": []}, {"apogee_min_drop_m": [1, 1]},
                       {"apogee_min_drop_m": list(range(1, 66))}, {"apogee_min_drop_m": [1, 0]}]:
            with self.subTest(values=values):
                grid.write_text(json.dumps({"schema_version": 1, "parameters": values}))
                self.failure(command="tune", grid=grid)

    def test_grid_deduplicates_base(self):
        grid = self.root / "grid.json"
        grid.write_text(json.dumps({"schema_version": 1, "parameters": {"apogee_min_drop_m": [5, 1]}}))
        report, _ = self.success(command="tune", grid=grid)
        self.assertEqual(len(report["runs"]), 2)

    def test_existing_output_and_inputs_untouched(self):
        out = self.root / "existing"
        out.mkdir(); marker = out / "keep.txt"; marker.write_text("user contents")
        trace = self.csv((FIXTURES / "synthetic_flight.csv").read_text())
        before = trace.read_bytes()
        result, _ = self.cli(trace=trace, output=out)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(marker.read_text(), "user contents")
        self.assertEqual(trace.read_bytes(), before)
        self.assertEqual(list(out.iterdir()), [marker])

    def test_repeat_outputs_are_byte_identical(self):
        _, a = self.success(command="tune")
        _, b = self.success(command="tune")
        self.assertEqual({p.name: p.read_bytes() for p in a.iterdir()}, {p.name: p.read_bytes() for p in b.iterdir()})

    def test_descending_start_and_uint64_time(self):
        end = (1 << 64) - 1
        text = "now_ms,sampled_at_ms,altitude_m,armed\n" + "".join(f"{t},{t},0,0\n" for t in range(end - 4000, end + 1, 1000))
        report, _ = self.success(trace=self.csv(text), config=self.config(lambda c: c.update(start="descending_after_release")), window=None)
        self.assertEqual(report["runs"][0]["events"]["landed"]["now_ms"], end)
        self.assertNotIn("launch", report["runs"][0]["events"])

    def test_backend_error_and_bad_output_are_not_results(self):
        self.failure(backend=self.root / "absent")
        self.failure(backend=Path(sys.executable))
        fake = self.root / "fake_backend"
        fake.write_text("#!/usr/bin/env python3\nprint('wrong,header')\n")
        fake.chmod(0o755)
        self.assertIn("output header", self.failure(backend=fake).stderr)

    def test_native_protocol_rejects_incomplete_or_malformed_input(self):
        for payload in ["", "CONFIG on_pad 1\n", "S 0 0 0 1\n"]:
            result = subprocess.run([str(BACKEND), "--protocol-v1"], input=payload, capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("flight replay backend:", result.stderr)

    def test_limits_are_enforced(self):
        data = b"now_ms,sampled_at_ms,altitude_m,armed\n0,0,0,1\n1,1,0,1\n"
        with patch.object(replay, "MAX_ROWS", 1), self.assertRaisesRegex(ValueError, "row limit"):
            replay.load_trace(data)
        trace = self.csv("now_ms,sampled_at_ms,altitude_m,armed\n" + "".join(f"{t},{t},0,1\n" for t in range(4000)))
        grid = self.root / "grid.json"
        grid.write_text(json.dumps({"schema_version": 1, "parameters": {"apogee_min_drop_m": list(range(1, 65))}}))
        self.assertIn("250000", self.failure(command="tune", trace=trace, grid=grid, window=None).stderr)

    def test_metric_sign_absent_and_outside_window(self):
        events = {"launch": {"now_ms": 90}, "apogee": {"now_ms": 200}}
        metrics = replay.evaluate(events, {"launch": 100, "landed": 300}, 10)
        self.assertEqual(metrics["events"]["launch"]["latency_ms"], -10)
        self.assertEqual((metrics["matched"], metrics["missed"], metrics["false_events"]), (1, 1, 1))
        self.assertIsNone(replay.evaluate(events, None, None))


if __name__ == "__main__":
    unittest.main(verbosity=2)
