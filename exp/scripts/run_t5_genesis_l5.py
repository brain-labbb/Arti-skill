#!/usr/bin/env python3
"""Run Genesis L1-L5 articulation checks with collision disabled and disclose that scope."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
DEFAULT_MANIFEST = EXP_ROOT / "runtime/t5_formal_v1/simulation_ready/simulation_input_manifest.json"
DEFAULT_OUT = EXP_ROOT / "runtime/t5_formal_v1/simulators/genesis"
PASSIVE_STEPS = 120
TARGET_STEPS = 10


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def targets(urdf: Path) -> dict[str, list[float]]:
    root = ET.parse(urdf).getroot()
    rows: dict[str, list[float]] = {}
    for joint in root.findall("joint"):
        kind = joint.attrib.get("type")
        if kind == "fixed":
            continue
        limit = joint.find("limit")
        if kind in {"revolute", "prismatic"} and limit is not None:
            lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
            rows[joint.attrib["name"]] = [
                lower,
                lower + 0.25 * (upper - lower),
                (lower + upper) / 2.0,
                lower + 0.75 * (upper - lower),
                upper,
            ]
        else:
            rows[joint.attrib["name"]] = [-math.pi, -math.pi / 2, 0.0, math.pi / 2, math.pi]
    return rows


def base(package: Path) -> dict[str, Any]:
    return {
        "asset_id": package.name,
        "simulator": "genesis",
        "collision_enabled": False,
        "l1_parse": False,
        "l2_instantiate": False,
        "l3_first_step": False,
        "l4_passive_stable": False,
        "l5_full_articulation_kinematics": False,
        "strict_collision_enabled_l5": False,
        "rest_stable": False,
        "worst_state_stable": False,
        "joint_count": 0,
        "target_state_count": 0,
        "target_state_pass_count": 0,
        "elapsed_s": None,
        "error": None,
    }


def worker(package: Path, result_path: Path) -> int:
    import genesis as gs
    import numpy as np

    record = base(package)
    started = time.monotonic()
    try:
        expected = targets(package / "model.urdf")
        record["l1_parse"] = True
        gs.init(backend=gs.gpu, logging_level="warning")
        scene = gs.Scene(
            show_viewer=False,
            rigid_options=gs.options.RigidOptions(
                enable_collision=False,
                enable_self_collision=False,
            ),
        )
        entity = scene.add_entity(
            gs.morphs.URDF(
                file=str(package / "model.urdf"),
                fixed=True,
                collision=False,
                visualization=False,
                requires_jac_and_IK=False,
            )
        )
        scene.build()
        record["l2_instantiate"] = True
        joint_by_name = {
            joint.name: int(joint.dofs_idx_local[0])
            for joint in entity.joints
            if joint.n_dofs == 1
        }
        record["joint_count"] = len(joint_by_name)
        if set(joint_by_name) != set(expected):
            raise RuntimeError(
                f"joint-name mismatch: missing={sorted(set(expected)-set(joint_by_name))}, "
                f"extra={sorted(set(joint_by_name)-set(expected))}"
            )
        scene.step()
        record["l3_first_step"] = True
        for _ in range(PASSIVE_STEPS):
            scene.step()
        q = entity.get_dofs_position().detach().cpu().numpy()
        qd = entity.get_dofs_velocity().detach().cpu().numpy()
        stable = bool(np.isfinite(q).all() and np.isfinite(qd).all())
        record["l4_passive_stable"] = stable
        record["rest_stable"] = stable
        passed = 0
        total = 0
        all_stable = True
        for name, values in expected.items():
            index = joint_by_name[name]
            for value in values:
                total += 1
                entity.set_dofs_position([value], dofs_idx_local=[index], zero_velocity=True)
                observed = float(entity.get_dofs_position(dofs_idx_local=[index]).detach().cpu().numpy()[0])
                reached = math.isfinite(observed) and abs(observed - value) <= 1e-5
                for _ in range(TARGET_STEPS):
                    scene.step()
                q = entity.get_dofs_position().detach().cpu().numpy()
                qd = entity.get_dofs_velocity().detach().cpu().numpy()
                state_ok = bool(np.isfinite(q).all() and np.isfinite(qd).all())
                all_stable = all_stable and state_ok
                passed += int(reached and state_ok)
        record["target_state_count"] = total
        record["target_state_pass_count"] = passed
        record["worst_state_stable"] = all_stable
        record["l5_full_articulation_kinematics"] = passed == total and all_stable
    except BaseException as exc:  # noqa: BLE001
        record["error"] = f"{type(exc).__name__}: {exc}"
    record["elapsed_s"] = time.monotonic() - started
    dump_json(result_path, record)
    return 0 if record["l5_full_articulation_kinematics"] else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("worker_args", nargs="*")
    args = parser.parse_args()
    if args.worker:
        package, result_path = args.worker_args
        return worker(Path(package), Path(result_path))
    output = args.out.resolve()
    output.relative_to(EXP_ROOT.resolve())
    rows = json.loads(args.manifest.read_text(encoding="utf-8"))
    records: list[dict[str, Any]] = []
    for row in rows:
        package = Path(row["copied_package"])
        result_path = output / "assets" / f"{row['asset_id']}.json"
        if result_path.is_file():
            records.append(json.loads(result_path.read_text(encoding="utf-8")))
            continue
        try:
            completed = subprocess.run(
                [sys.executable, str(Path(__file__).resolve()), "--worker", str(package), str(result_path)],
                cwd=PROJECT_ROOT,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=args.timeout,
                env={**os.environ, "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"},
            )
            if result_path.is_file():
                record = json.loads(result_path.read_text(encoding="utf-8"))
                record["worker_exit_code"] = completed.returncode
                record["worker_stderr_tail"] = completed.stderr[-4000:]
            else:
                record = {
                    **base(package),
                    "error": f"worker_exit_without_result({completed.returncode})",
                    "worker_stderr_tail": completed.stderr[-4000:],
                }
        except subprocess.TimeoutExpired as exc:
            record = {
                **base(package),
                "error": f"worker_timeout({args.timeout:.0f}s)",
                "worker_stderr_tail": (exc.stderr or "")[-4000:] if isinstance(exc.stderr, str) else "",
            }
        records.append(record)
        print(row["asset_id"], "L5-kin" if record["l5_full_articulation_kinematics"] else "FAIL", flush=True)
    dump_json(output / "asset_records.json", records)
    summary = {
        "schema_version": 1,
        "protocol": "t5_genesis_l1_l5_kinematics_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "asset_count": len(records),
        "collision_enabled": False,
        "l1_parse": sum(row["l1_parse"] for row in records),
        "l2_instantiate": sum(row["l2_instantiate"] for row in records),
        "l3_first_step": sum(row["l3_first_step"] for row in records),
        "l4_passive_stable": sum(row["l4_passive_stable"] for row in records),
        "l5_full_articulation_kinematics": sum(row["l5_full_articulation_kinematics"] for row in records),
        "strict_collision_enabled_l5": 0,
        "rest_stable": sum(row["rest_stable"] for row in records),
        "worst_state_stable": sum(row["worst_state_stable"] for row in records),
        "joint_count": sum(row["joint_count"] for row in records),
        "target_state_count": sum(row["target_state_count"] for row in records),
        "target_state_pass_count": sum(row["target_state_pass_count"] for row in records),
        "scope_note": (
            "Genesis collision geometry is disabled because its first multicontact kernel did not compile within 30 minutes. "
            "This is a real Genesis parser/dynamics/articulation run but does not qualify for strict collision-enabled L5."
        ),
        "manifest_sha256": sha256(args.manifest),
    }
    dump_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
