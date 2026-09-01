#!/usr/bin/env python3
"""Run explicit L1-L5 readiness checks in PyBullet or MuJoCo on the T5 cohort."""

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
DEFAULT_OUT = EXP_ROOT / "runtime/t5_formal_v1/simulators"
PASSIVE_STEPS = 120
WORST_STATE_STEPS = 10


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


def finite(values: Any) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def xml_joint_targets(urdf: Path) -> dict[str, list[float]]:
    root = ET.parse(urdf).getroot()
    targets: dict[str, list[float]] = {}
    for joint in root.findall("joint"):
        joint_type = joint.attrib.get("type")
        if joint_type == "fixed":
            continue
        limit = joint.find("limit")
        if joint_type in {"revolute", "prismatic"} and limit is not None:
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
            targets[joint.attrib["name"]] = [
                lower,
                lower + 0.25 * (upper - lower),
                (lower + upper) / 2.0,
                lower + 0.75 * (upper - lower),
                upper,
            ]
        else:
            targets[joint.attrib["name"]] = [
                -math.pi,
                -math.pi / 2.0,
                0.0,
                math.pi / 2.0,
                math.pi,
            ]
    return targets


def base_record(simulator: str, package: Path) -> dict[str, Any]:
    return {
        "asset_id": package.name,
        "simulator": simulator,
        "package": str(package),
        "urdf_sha256": sha256(package / "model.urdf"),
        "l1_parse": False,
        "l2_instantiate": False,
        "l3_first_step": False,
        "l4_passive_stable": False,
        "l5_full_articulation": False,
        "rest_stable": False,
        "worst_state_stable": False,
        "joint_count": 0,
        "target_state_count": 0,
        "target_state_pass_count": 0,
        "max_post_step_target_deviation": None,
        "elapsed_s": None,
        "error": None,
    }


def pybullet_worker(package: Path) -> dict[str, Any]:
    import pybullet as pb

    record = base_record("pybullet", package)
    started = time.monotonic()
    connection = None
    try:
        targets = xml_joint_targets(package / "model.urdf")
        record["l1_parse"] = True
        connection = pb.connect(pb.DIRECT)
        pb.setGravity(0.0, 0.0, -9.81)
        pb.setTimeStep(1.0 / 240.0)
        body = pb.loadURDF(
            str(package / "model.urdf"),
            useFixedBase=True,
            flags=pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )
        record["l2_instantiate"] = True
        joint_by_name = {
            pb.getJointInfo(body, index)[1].decode("utf-8"): index
            for index in range(pb.getNumJoints(body))
            if pb.getJointInfo(body, index)[2] != pb.JOINT_FIXED
        }
        record["joint_count"] = len(joint_by_name)
        if set(joint_by_name) != set(targets):
            raise RuntimeError(
                f"joint-name mismatch: missing={sorted(set(targets)-set(joint_by_name))}, "
                f"extra={sorted(set(joint_by_name)-set(targets))}"
            )
        pb.stepSimulation()
        record["l3_first_step"] = True
        for index in joint_by_name.values():
            pb.setJointMotorControl2(body, index, pb.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0)
        for _ in range(PASSIVE_STEPS):
            pb.stepSimulation()
        states = [pb.getJointState(body, index) for index in joint_by_name.values()]
        passive_finite = all(finite((state[0], state[1])) for state in states)
        record["l4_passive_stable"] = passive_finite
        record["rest_stable"] = passive_finite

        deviations: list[float] = []
        all_stable = True
        passed = 0
        total = 0
        for name, values in targets.items():
            index = joint_by_name[name]
            for target in values:
                total += 1
                pb.resetJointState(body, index, target, targetVelocity=0.0)
                observed = float(pb.getJointState(body, index)[0])
                reached = math.isfinite(observed) and abs(observed - target) <= 1e-6
                for _ in range(WORST_STATE_STEPS):
                    pb.stepSimulation()
                final_state = pb.getJointState(body, index)
                stable = finite((final_state[0], final_state[1]))
                all_stable = all_stable and stable
                deviations.append(abs(observed - target))
                passed += int(reached and stable)
        record["target_state_count"] = total
        record["target_state_pass_count"] = passed
        record["max_post_step_target_deviation"] = max(deviations, default=0.0)
        record["worst_state_stable"] = all_stable
        record["l5_full_articulation"] = passed == total and all_stable
    except BaseException as exc:  # noqa: BLE001
        record["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if connection is not None:
            pb.disconnect(connection)
        record["elapsed_s"] = time.monotonic() - started
    return record


def mujoco_worker(package: Path) -> dict[str, Any]:
    import mujoco
    import numpy as np

    record = base_record("mujoco", package)
    started = time.monotonic()
    try:
        targets = xml_joint_targets(package / "model.urdf")
        record["l1_parse"] = True
        model = mujoco.MjModel.from_xml_path(str(package / "model.urdf"))
        data = mujoco.MjData(model)
        record["l2_instantiate"] = True
        joint_by_name: dict[str, tuple[int, int]] = {}
        for joint_id in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            joint_by_name[str(name)] = (joint_id, int(model.jnt_qposadr[joint_id]))
        record["joint_count"] = len(joint_by_name)
        if set(joint_by_name) != set(targets):
            raise RuntimeError(
                f"joint-name mismatch: missing={sorted(set(targets)-set(joint_by_name))}, "
                f"extra={sorted(set(joint_by_name)-set(targets))}"
            )
        mujoco.mj_step(model, data)
        record["l3_first_step"] = finite(data.qpos) and finite(data.qvel)
        for _ in range(PASSIVE_STEPS):
            mujoco.mj_step(model, data)
        passive_finite = finite(data.qpos) and finite(data.qvel)
        record["l4_passive_stable"] = passive_finite
        record["rest_stable"] = passive_finite

        deviations: list[float] = []
        all_stable = True
        passed = 0
        total = 0
        for name, values in targets.items():
            _, address = joint_by_name[name]
            for target in values:
                total += 1
                data.qpos[address] = target
                data.qvel[:] = 0.0
                mujoco.mj_forward(model, data)
                observed = float(data.qpos[address])
                reached = math.isfinite(observed) and abs(observed - target) <= 1e-6
                for _ in range(WORST_STATE_STEPS):
                    mujoco.mj_step(model, data)
                stable = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
                all_stable = all_stable and stable
                deviations.append(abs(observed - target))
                passed += int(reached and stable)
        record["target_state_count"] = total
        record["target_state_pass_count"] = passed
        record["max_post_step_target_deviation"] = max(deviations, default=0.0)
        record["worst_state_stable"] = all_stable
        record["l5_full_articulation"] = passed == total and all_stable
    except BaseException as exc:  # noqa: BLE001
        record["error"] = f"{type(exc).__name__}: {exc}"
    record["elapsed_s"] = time.monotonic() - started
    return record


def worker(simulator: str, package: Path, result_path: Path) -> int:
    result = pybullet_worker(package) if simulator == "pybullet" else mujoco_worker(package)
    dump_json(result_path, result)
    return 0 if result["l5_full_articulation"] else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulator", choices=("pybullet", "mujoco"))
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("worker_args", nargs="*")
    args = parser.parse_args()
    if args.worker:
        simulator, package, result_path = args.worker_args
        return worker(simulator, Path(package), Path(result_path))
    if args.simulator is None:
        parser.error("--simulator is required")
    output = (args.out / args.simulator).resolve()
    output.relative_to(EXP_ROOT.resolve())
    rows = json.loads(args.manifest.read_text(encoding="utf-8"))
    records: list[dict[str, Any]] = []
    for row in rows:
        package = Path(row["copied_package"])
        result_path = output / "assets" / f"{row['asset_id']}.json"
        if result_path.is_file():
            records.append(json.loads(result_path.read_text(encoding="utf-8")))
            continue
        command = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--worker",
            simulator := args.simulator,
            str(package),
            str(result_path),
        ]
        try:
            completed = subprocess.run(
                command,
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
                    **base_record(simulator, package),
                    "error": f"worker_exit_without_result({completed.returncode})",
                    "worker_stderr_tail": completed.stderr[-4000:],
                }
        except subprocess.TimeoutExpired as exc:
            record = {
                **base_record(args.simulator, package),
                "error": f"worker_timeout({args.timeout:.0f}s)",
                "worker_stderr_tail": (exc.stderr or "")[-4000:] if isinstance(exc.stderr, str) else "",
            }
        records.append(record)
        print(row["asset_id"], "L5" if record["l5_full_articulation"] else "FAIL", flush=True)
    dump_json(output / "asset_records.json", records)
    summary = {
        "schema_version": 1,
        "protocol": "t5_simulator_l1_l5_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "simulator": args.simulator,
        "asset_count": len(records),
        "l1_parse": sum(row["l1_parse"] for row in records),
        "l2_instantiate": sum(row["l2_instantiate"] for row in records),
        "l3_first_step": sum(row["l3_first_step"] for row in records),
        "l4_passive_stable": sum(row["l4_passive_stable"] for row in records),
        "l5_full_articulation": sum(row["l5_full_articulation"] for row in records),
        "rest_stable": sum(row["rest_stable"] for row in records),
        "worst_state_stable": sum(row["worst_state_stable"] for row in records),
        "joint_count": sum(row["joint_count"] for row in records),
        "target_state_count": sum(row["target_state_count"] for row in records),
        "target_state_pass_count": sum(row["target_state_pass_count"] for row in records),
        "manifest": str(args.manifest.resolve()),
        "manifest_sha256": sha256(args.manifest),
        "l5_definition": (
            "For every movable joint, set five full-range targets through the simulator state API, "
            "run forward dynamics for ten steps at each target, and require finite state throughout."
        ),
    }
    dump_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
