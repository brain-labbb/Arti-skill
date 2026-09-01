#!/usr/bin/env python3
"""Synthetic positive/negative self-test for the Table 1 common evaluator."""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
EXP_ROOT = REPO_ROOT / "exp"
OUT = EXP_ROOT / "runtime/table1_reliability/evaluator_self_test"
MANIFEST = EXP_ROOT / "reference/table1_reliability_common_authoring_v1.json"
PROTOCOL = EXP_ROOT / "reference/table1_reliability_protocol_v1.json"
HIDDEN = EXP_ROOT / "reference/table1_reliability_hidden_specs_v1.json"
EVALUATOR = EXP_ROOT / "scripts/evaluate_table1_authoring_common.py"
PACKAGE_SCHEMA = EXP_ROOT / "reference/table1_authoring_package_schema_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")


def dump(path: Path, value: Any) -> None:
    write(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def fixture(name: str, axis: str) -> Path:
    run_root = OUT / name
    source = run_root / "source.py"
    urdf = run_root / "model.urdf"
    mesh = run_root / "assets/unit.obj"
    write(source, "print('synthetic execution probe')\n")
    write(
        mesh,
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
        "f 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n",
    )
    write(
        urdf,
        f"""<robot name="synthetic_nightstand">
  <link name="body"><visual><geometry><mesh filename="assets/unit.obj"/></geometry></visual></link>
  <link name="door"><visual><geometry><mesh filename="assets/unit.obj"/></geometry></visual></link>
  <joint name="body_to_door" type="revolute">
    <parent link="body"/><child link="door"/><axis xyz="{axis}"/>
    <limit lower="0" upper="1.92" effort="1" velocity="1"/>
  </joint>
</robot>
""",
    )
    protocol = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    now = datetime.now(timezone.utc).isoformat()
    package = {
        "schema_version": "table1_authoring_package_v1",
        "run_id": name,
        "method_id": "pva",
        "task_id": "FUR-L1-01",
        "repeat_id": "r0",
        "attempt_index": 0,
        "run_root": str(run_root),
        "bindings": {
            "protocol_sha256": sha256(PROTOCOL),
            "manifest_sha256": sha256(MANIFEST),
            "hidden_specs_sha256": sha256(HIDDEN),
            "common_evaluator_sha256": sha256(EVALUATOR),
            "package_schema_sha256": sha256(PACKAGE_SCHEMA),
        },
        "artifacts": {
            "source": {"path": "source.py", "sha256": sha256(source)},
            "urdf": {"path": "model.urdf", "sha256": sha256(urdf)},
        },
        "execution_probe": {
            "started_at_utc": now,
            "finished_at_utc": now,
            "wall_time_s": 0.01,
            "exit_code": 0,
            "timed_out": False,
            "source_sha256": sha256(source),
            "stdout_sha256": hashlib.sha256(b"").hexdigest(),
            "stderr_sha256": hashlib.sha256(b"").hexdigest(),
        },
    }
    assert protocol["common_evaluator"]["sha256"] == package["bindings"]["common_evaluator_sha256"]
    package_path = run_root / "package.json"
    dump(package_path, package)
    return package_path


def evaluate(package: Path) -> dict[str, Any]:
    report = package.parent / "report.json"
    completed = subprocess.run(
        [
            "python",
            str(EVALUATOR),
            "--package-manifest",
            str(package),
            "--output",
            str(report),
        ],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0 or not report.is_file():
        raise RuntimeError(
            f"evaluator failed: exit={completed.returncode} stderr={completed.stderr[-2000:]}"
        )
    return json.loads(report.read_text(encoding="utf-8"))


def main() -> int:
    if OUT.exists():
        shutil.rmtree(OUT)
    positive = evaluate(fixture("positive_z_axis", "0 0 1"))
    negative = evaluate(fixture("negative_x_axis", "1 0 0"))
    positive_verdicts = positive["verdicts"]
    negative_verdicts = negative["verdicts"]
    checks = {
        "positive_all_table1_gates_pass": all(positive_verdicts.values()),
        "negative_remains_executable": negative_verdicts["executable"] is True,
        "negative_retains_artifact": negative_verdicts["artifact_saved"] is True,
        "negative_joint_spec_fails": negative_verdicts["joint_spec_pass"] is False,
        "negative_common_qc_fails": negative_verdicts["common_qc_pass"] is False,
        "negative_feedback_withholds_expected_values": not any(
            token in json.dumps(negative["feedback"]).lower()
            for token in ("body_to_door", "door", "axis error", "1.92")
        ),
    }
    summary = {
        "schema_version": 1,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "positive_report_sha256": sha256(OUT / "positive_z_axis/report.json"),
        "negative_report_sha256": sha256(OUT / "negative_x_axis/report.json"),
    }
    dump(OUT / "self_check.json", summary)
    print(json.dumps(summary, sort_keys=True))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
