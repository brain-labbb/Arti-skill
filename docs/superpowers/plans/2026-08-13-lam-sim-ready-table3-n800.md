# LAM Sim-Ready Table 3 N=800 Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: Use `subagent-driven-development` to execute this plan task by task. Use `test-driven-development` within every implementation task and `verification-before-completion` before reporting the formal run complete.

**Goal:** Implement the frozen Sim-Ready Table 3 evaluator, deterministically select 800 viable LAM releases, and produce a receipt-gated formal result without replacing failed assets or changing the denominator.

**Architecture:** A single importable runner owns deterministic freezing, a stdlib XML plus NumPy analytic FK implementation, one isolated PyBullet DIRECT subprocess per asset, aggregation, and verification. A frozen JSON protocol is the sole source of evaluation constants. The controller writes immutable inputs first, launches four CPU workers with explicit timeouts, records terminal failures in the denominator, then creates aggregate outputs and a hash seal only after every selected asset has exactly one terminal record.

**Tech Stack:** Python 3.12, stdlib (`argparse`, `csv`, `concurrent.futures`, `dataclasses`, `gzip`, `hashlib`, `json`, `pathlib`, `subprocess`, `xml.etree.ElementTree`), NumPy 2.4, trimesh 4.12, PyBullet API 202010061, pytest.

**Authoritative design:** `docs/superpowers/specs/2026-08-13-lam-sim-ready-table3-n800-design.md`

**Execution environment:** `/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python`

---

## Task 1: Freeze the protocol and deterministic cohort

**Files:**

- Create: `exp/reference/urdf_sim_ready_table3_lam_viable_n800_v1.json`
- Create: `exp/scripts/run_urdf_sim_ready_table3_lam.py`
- Create: `exp/tests/test_urdf_sim_ready_table3_lam.py`
- Test: `exp/tests/test_urdf_sim_ready_table3_lam.py`

### Step 1: Write failing protocol and selection tests

Add import scaffolding and these tests. The full-repository test intentionally reads the real LAM manifest but does not write runtime output.

```python
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest


REPO = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO / "exp/scripts/run_urdf_sim_ready_table3_lam.py"
PROTOCOL_PATH = REPO / "exp/reference/urdf_sim_ready_table3_lam_viable_n800_v1.json"
LAM_ROOT = REPO / "exp/Articulated-Object-Code"


def load_runner():
    spec = importlib.util.spec_from_file_location("sim_ready_table3", RUNNER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_protocol_constants_are_frozen():
    protocol = json.loads(PROTOCOL_PATH.read_text())
    assert protocol["protocol_id"] == "urdf-sim-ready-table3-lam-viable-n800-v1"
    assert protocol["selection"]["namespace"] == "urdf-sim-ready-table3-lam-viable-n800-v1"
    assert protocol["selection"]["sample_size"] == 800
    assert protocol["sampling"]["states_per_joint"] == 21
    assert protocol["execution"]["pybullet_load_flags"] == 0
    assert protocol["execution"]["workers"] == 4
    assert protocol["execution"]["asset_timeout_seconds"] == 180
    assert protocol["thresholds"]["rotation_radians"] == pytest.approx(1e-5)


def test_selection_hash_uses_namespace_nul_rel_path():
    runner = load_runner()
    namespace = "urdf-sim-ready-table3-lam-viable-n800-v1"
    rel_path = "objects/laptop/laptop_194"
    expected = hashlib.sha256(
        namespace.encode("utf-8") + b"\0" + rel_path.encode("utf-8")
    ).hexdigest()
    assert runner.selection_hash(namespace, rel_path) == expected


def test_real_lam_selection_has_frozen_receipt():
    runner = load_runner()
    protocol = runner.load_protocol(PROTOCOL_PATH)
    frozen = runner.build_frozen_inputs(protocol, LAM_ROOT)
    assert frozen["release_count"] == 2533
    assert len(frozen["assets"]) == 800
    assert len({row["rel_path"] for row in frozen["assets"]}) == 800
    assert frozen["category_count"] == 280
    assert frozen["evaluated_joint_count"] == 2243
    assert frozen["joint_type_counts"] == {
        "continuous": 335,
        "prismatic": 768,
        "revolute": 1140,
    }
    assert [row["rel_path"] for row in frozen["assets"][:5]] == [
        "objects/ferris_wheel/ferris_wheel_026",
        "objects/swing_set_comprises_a/swing_set_comprises_a_000",
        "objects/folding_chair_typically_has/folding_chair_typically_has_010",
        "objects/clothespin_spring_type/clothespin_spring_type_013",
        "objects/laptop/laptop_194",
    ]
```

### Step 2: Run the tests to verify they fail

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py \
  -k 'protocol_constants or selection_hash or real_lam_selection'
```

Expected: FAIL because the protocol and runner do not exist.

### Step 3: Add the frozen protocol

Create a JSON document with no environment-derived defaults. It must include these values verbatim:

```json
{
  "protocol_id": "urdf-sim-ready-table3-lam-viable-n800-v1",
  "protocol_version": 1,
  "dataset": {
    "name": "Articulated-Object-Code",
    "alias": "LAM",
    "huggingface_repo": "YipengGao/Articulated-Object-Code",
    "revision": "28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0",
    "manifest_rel_path": "manifest.csv",
    "manifest_sha256": "70216593ec02b71d596e456498ff9863ad0f8e519d5d27d2cf4f58792d412412",
    "inventory_rel_path": "../dataset_inventory.json",
    "inventory_sha256": "e281119f870bb6bae9599c3edc02de0a42a257e0d433335361d4a774592c1b5a",
    "release_root_rel_path": "released_outputs",
    "tier": "viable",
    "archives": [
      {
        "name": "viable.tar.gz",
        "bytes": 1185271461,
        "sha256": "a582ef0aa0f3073749adcc73d289a12200e500c1a5762a4ee1530eefc2c4920d"
      },
      {
        "name": "loads_only.tar.gz",
        "bytes": 194746559,
        "sha256": "e616dc455450ca0f8ea1c76955929f340b75b9fa51b974327e160890620f9a9e"
      },
      {
        "name": "broken.tar.gz",
        "bytes": 199627935,
        "sha256": "ef0f6e8506e0432febd5ccc4159c6cbf66a9c5cdadd3bdbcd521478e4c0fda3a"
      }
    ]
  },
  "selection": {
    "namespace": "urdf-sim-ready-table3-lam-viable-n800-v1",
    "sample_size": 800,
    "primary_key": "rel_path",
    "hash": "sha256(namespace_utf8 || NUL || rel_path_utf8)",
    "sort": ["selection_hash", "rel_path"]
  },
  "sampling": {
    "states_per_joint": 21,
    "bounded": "linspace(lower, upper, 21, endpoints=true)",
    "continuous_lower": -3.141592653589793,
    "continuous_upper": 3.141592653589793,
    "neutral_bounded": "clamp(0, lower, upper)",
    "neutral_continuous": 0.0
  },
  "geometry": {
    "bbox_source": "visual_then_collision_fallback",
    "minimum_diagonal_m": 1e-9,
    "trimesh_process": false
  },
  "thresholds": {
    "translation_absolute_floor_m": 1e-8,
    "translation_relative_to_bbox": 1e-6,
    "rotation_radians": 1e-5,
    "readback_absolute": 1e-9,
    "readback_relative": 1e-12,
    "nondegenerate_translation_relative_to_bbox": 1e-6,
    "nondegenerate_rotation_radians": 1e-5
  },
  "execution": {
    "engine": "pybullet",
    "engine_api_version": 202010061,
    "connection": "DIRECT",
    "use_fixed_base": true,
    "pybullet_load_flags": 0,
    "disable_default_motors": true,
    "workers": 4,
    "asset_timeout_seconds": 180,
    "retry_count": 0,
    "replace_failed_assets": false
  },
  "smoke": {
    "count": 8,
    "workers": 2,
    "asset_timeout_seconds": 180,
    "status": "SMOKE_NOT_A_PAPER_RESULT"
  },
  "supported_joint_types": ["revolute", "prismatic", "continuous"],
  "unsupported_joint_policy": "fail_closed",
  "asset_pass_policy": "all_evaluated_joints_pass",
  "failure_denominator_policy": "retain_every_selected_asset_and_joint",
  "expected_receipt": {
    "release_count": 2533,
    "selected_asset_count": 800,
    "category_count": 280,
    "evaluated_joint_count": 2243,
    "selected_link_count": 4585,
    "joint_type_counts": {
      "continuous": 335,
      "prismatic": 768,
      "revolute": 1140
    }
  }
}
```

### Step 4: Implement protocol loading and frozen-input construction

In the runner, define dataclasses for joint and asset rows and implement:

```python
def canonical_json_bytes(value: object) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n").encode()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def selection_hash(namespace: str, rel_path: str) -> str:
    return hashlib.sha256(namespace.encode() + b"\0" + rel_path.encode()).hexdigest()


def load_protocol(path: Path) -> dict:
    protocol = json.loads(path.read_text())
    if protocol["protocol_version"] != 1:
        raise ValueError("unsupported protocol_version")
    return protocol
```

`build_frozen_inputs()` must use `csv.DictReader`, filter exactly `tier == "viable"`, reject duplicate `rel_path`, verify every selected `released_outputs/<rel_path>/generated.urdf`, sort by `(selection_hash, rel_path.encode("utf-8"))`, take exactly 800, and parse joint declarations without importing Articraft. Its XML declaration scanner must freeze ordered joint names, types, raw axes, declared bounds, neutral values, link names, and referenced geometry URIs; Task 2 will reuse those declarations for the full graph/FK model. Return ordered asset/joint rows plus release/category/count receipts and abort unless every `expected_receipt` field matches. Never use `object_release_id` as a unique key.

Before selecting, verify the manifest hash, inventory hash, inventory revision/repo fields, and all three archive byte-size/hash fields against the protocol. This establishes the local release provenance receipt; the selected URDF and geometry hashes establish the exact evaluated byte closure.

Add a `freeze` CLI that creates an empty output root only after validating it does not already contain files, then atomically writes:

- `protocol.json`
- `manifest.jsonl`
- `joint_manifest.jsonl`
- `resource_manifest.jsonl`
- `input_receipt.json`
- `environment.json`
- `command.txt`

Every record must carry stable keys, relative source paths, SHA-256 values, and schema version. `resource_manifest.jsonl` must include the selected URDF plus geometry resources resolved from each selected URDF; missing resources are recorded and make the later asset result fail, never disappear from the cohort.

Use one frozen `bindings` object everywhere with at least `protocol_sha256`, `manifest_sha256`, `joint_manifest_sha256`, `resource_manifest_sha256`, `input_receipt_sha256`, and `runner_sha256`. Each manifest/resource row stores both `source_abs_path` and the portable `source_rel_path`; verification rejects absolute paths outside the frozen dataset root.

Each resource row has `{asset_key, ordinal, kind, uri, source_abs_path, source_rel_path, byte_size, sha256, exists}`. Each asset manifest row stores the SHA-256 of its canonical ordered resource-row list. Freeze resolves these lists before scoring; workers validate and consume the frozen list instead of silently discovering a different closure. Every parsed `GeometryElement` stores its XML element ordinal and frozen resource ordinal. Repeated references to one URI retain separate element ordinals while the resource row may be shared; scene instances additionally retain their stable scene-node paths. Bounding-box code reads only the bytes bound by those ordinals and hashes.

`environment.json` records the resolved Python executable and SHA-256, Python/platform versions, and imported NumPy, trimesh, and PyBullet versions. Freeze calls `pybullet.getAPIVersion()` and aborts unless it equals protocol value `202010061`; every case spec/result/seal repeats the environment receipt hash.

### Step 5: Run focused tests and freeze preflight

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py \
  -k 'protocol_constants or selection_hash or real_lam_selection'
```

Expected: PASS, with the real cohort receipt exactly 2533 release candidates, 800 selected assets, 280 categories, and 2243 evaluated joints.

### Step 6: Commit

```bash
git add exp/reference/urdf_sim_ready_table3_lam_viable_n800_v1.json \
  exp/scripts/run_urdf_sim_ready_table3_lam.py \
  exp/tests/test_urdf_sim_ready_table3_lam.py
git commit -m "feat: freeze LAM table3 cohort"
```

---

## Task 2: Implement independent URDF parsing, analytic FK, and geometry scale

**Files:**

- Modify: `exp/scripts/run_urdf_sim_ready_table3_lam.py`
- Modify: `exp/tests/test_urdf_sim_ready_table3_lam.py`
- Test: `exp/tests/test_urdf_sim_ready_table3_lam.py`

### Step 1: Add failing synthetic FK tests

Build URDF strings in `tmp_path` rather than depending on an SDK. Cover all frozen semantics:

```python
def test_rpy_origin_and_revolute_fk(tmp_path):
    runner = load_runner()
    urdf = write_two_link_urdf(
        tmp_path,
        joint_type="revolute",
        origin_xyz="1 2 3",
        origin_rpy="0 0 1.5707963267948966",
        axis="1 0 0",
        limit='lower="-1" upper="2" effort="1" velocity="1"',
    )
    model = runner.parse_urdf(urdf)
    poses = runner.analytic_fk(model, {"joint": 0.5})
    np.testing.assert_allclose(poses["base"], runner.identity_pose(), atol=1e-12)
    np.testing.assert_allclose(poses["child"][:3, 3], [1.0, 2.0, 3.0], atol=1e-12)
    expected = runner.rpy_matrix(0.0, 0.0, 1.5707963267948966) @ runner.axis_angle_matrix(
        [1.0, 0.0, 0.0], 0.5
    )
    np.testing.assert_allclose(poses["child"][:3, :3], expected, atol=1e-12)


@pytest.mark.parametrize(
    ("joint_type", "expected_first", "expected_last"),
    [
        ("revolute", -2.0, 2.0),
        ("prismatic", -2.0, 2.0),
        ("continuous", -3.141592653589793, 3.141592653589793),
    ],
)
def test_sample_values_include_exact_frozen_endpoints(
    tmp_path, joint_type, expected_first, expected_last
):
    runner = load_runner()
    joint = make_parsed_joint(tmp_path, joint_type, lower=-2.0, upper=2.0)
    samples = runner.sample_joint_values(joint, load_protocol(PROTOCOL_PATH))
    assert len(samples) == 21
    assert samples[0] == pytest.approx(expected_first)
    assert samples[-1] == pytest.approx(expected_last)


def test_missing_origin_is_identity_and_missing_axis_is_urdf_x(tmp_path):
    runner = load_runner()
    model = runner.parse_urdf(write_minimal_joint_urdf(tmp_path, omit_origin=True, omit_axis=True))
    np.testing.assert_allclose(model.joints["joint"].origin, runner.identity_pose(), atol=1e-12)
    np.testing.assert_allclose(model.joints["joint"].axis, [1.0, 0.0, 0.0], atol=1e-12)


def test_planar_joint_fails_closed(tmp_path):
    runner = load_runner()
    model = runner.parse_urdf(write_minimal_joint_urdf(tmp_path, joint_type="planar"))
    with pytest.raises(runner.NotEvaluableError, match="unsupported joint type: planar"):
        runner.validate_evaluable_model(model)


def test_visual_bbox_applies_mesh_scale_and_element_origin(tmp_path):
    runner = load_runner()
    urdf = write_scaled_box_mesh_urdf(tmp_path, scale="2 3 4", origin_xyz="5 0 0")
    model = runner.parse_urdf(urdf)
    bounds = runner.neutral_geometry_bounds(model, urdf.parent)
    assert bounds.minimum.tolist() == pytest.approx([4.0, -1.5, -2.0])
    assert bounds.maximum.tolist() == pytest.approx([6.0, 1.5, 2.0])
    assert bounds.source == "visual"
```

Also add negative tests for non-finite numbers, zero axes, duplicate links/joints, missing parent/child, multiple roots, cycles, multi-parent graphs, missing bounded limits, reversed/zero-width limits, mimic, unreadable meshes, and `d <= 1e-9`.

### Step 2: Verify the new tests fail

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py -k 'fk or sample_values or bbox or fails_closed'
```

Expected: FAIL because the parser, FK, and geometry functions are absent.

### Step 3: Implement fail-closed model parsing and FK

Implement dataclasses `UrdfModel`, `Link`, `Joint`, `GeometryElement`, and `Bounds`. Parse with `xml.etree.ElementTree`; all floats must be finite. Enforce a single rooted tree and unique names. Use:

```python
def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # URDF fixed-axis roll, pitch, yaw: Rz(yaw) @ Ry(pitch) @ Rx(roll).
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)


def joint_motion(joint: Joint, q: float) -> np.ndarray:
    transform = np.eye(4)
    if joint.kind in {"revolute", "continuous"}:
        transform[:3, :3] = axis_angle_matrix(joint.axis, q)
    elif joint.kind == "prismatic":
        transform[:3, 3] = joint.axis * q
    elif joint.kind != "fixed":
        raise NotEvaluableError(f"unsupported joint type: {joint.kind}")
    return transform


def analytic_fk(model: UrdfModel, configuration: dict[str, float]) -> dict[str, np.ndarray]:
    poses = {model.root_link: np.eye(4)}
    for joint in model.topological_joints:
        q = configuration.get(joint.name, 0.0)
        poses[joint.child] = poses[joint.parent] @ joint.origin @ joint_motion(joint, q)
    return poses
```

Normalize finite nonzero axes and record the raw norm. Bounded revolute/prismatic joints require finite `lower < upper`; continuous joints use the protocol envelope. Fixed joints are part of the tree but not the evaluated-joint denominator. Mimic, planar, floating, unknown joints, malformed graphs, and invalid numeric fields make the selected asset terminally not evaluable.

### Step 4: Implement visual-first neutral-pose bounds

For every visual geometry, apply primitive dimensions or mesh vertices, mesh scale, element origin, and neutral world link pose. Use collision geometry only when an entire asset has no visual geometry. Load meshes with `trimesh.load(..., process=False)` and never repair them. With homogeneous column vectors, freeze the mesh/scene order as `V_world = T_world_link(q0) @ T_visual_element @ S_urdf_mesh @ T_scene_instance @ V_geometry`; URDF mesh scale therefore applies to the whole instantiated mesh asset, including scene-node translations. For a `trimesh.Scene`, enumerate every graph instance using its stable node path, apply that exact transform to a copy of the referenced vertices, then include those transformed vertices in the union; never concatenate raw geometry while dropping scene transforms. Resolve `package://`, `file://`, and relative URIs only within the selected release directory and reject path escapes. If the selected visual source contains any missing or unreadable resource, fail the asset rather than ignore that element or switch to collision. Compute the union AABB and diagonal; do not substitute a default scale. Add a bbox regression with two instances of one mesh, scene-node translation and rotation, and nonuniform URDF mesh scale that distinguishes this order from `T_scene_instance @ S_urdf_mesh`.

### Step 5: Run focused and full tests

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py
```

Expected: PASS.

### Step 6: Commit

```bash
git add exp/scripts/run_urdf_sim_ready_table3_lam.py \
  exp/tests/test_urdf_sim_ready_table3_lam.py
git commit -m "feat: add independent table3 kinematics"
```

---

## Task 3: Add isolated PyBullet execution and per-state scoring

**Files:**

- Modify: `exp/scripts/run_urdf_sim_ready_table3_lam.py`
- Modify: `exp/tests/test_urdf_sim_ready_table3_lam.py`
- Test: `exp/tests/test_urdf_sim_ready_table3_lam.py`

### Step 1: Add failing engine and metric tests

Use synthetic branched URDFs with inertial, visual, and collision elements. Tests must run PyBullet through the same child-process CLI used by the formal evaluator.

```python
def test_pose_error_uses_bbox_normalized_translation_and_geodesic_rotation():
    runner = load_runner()
    a = runner.identity_pose()
    b = runner.identity_pose()
    b[:3, 3] = [2e-6, 0.0, 0.0]
    b[:3, :3] = runner.axis_angle_matrix([0, 0, 1], 2e-5)
    error = runner.pose_error(a, b, bbox_diagonal=2.0)
    assert error.translation_m == pytest.approx(2e-6)
    assert error.translation_normalized == pytest.approx(1e-6)
    assert error.rotation_radians == pytest.approx(2e-5)


def test_worker_matches_analytic_fk_and_preserves_non_subtree(tmp_path):
    runner = load_runner()
    case = write_branched_revolute_case(tmp_path)
    completed = run_worker_subprocess(case)
    assert completed.returncode == 0, completed.stderr
    record = json.loads(case.result_path.read_text())
    assert record["terminal_status"] == "completed"
    assert record["asset_pass"] is True
    joint = record["joints"][0]
    assert joint["sweep_success"] is True
    assert joint["engine_agreement"] is True
    assert joint["subtree_invariance"] is True
    assert joint["roundtrip"] is True
    assert len(joint["states"]) == 21


def test_worker_loads_laptop_without_maintain_link_order_crash(tmp_path):
    case = build_real_asset_case(
        tmp_path,
        LAM_ROOT / "released_outputs/objects/laptop/laptop_095/generated.urdf",
    )
    completed = run_worker_subprocess(case, timeout=180)
    assert completed.returncode == 0, completed.stderr
    assert json.loads(case.result_path.read_text())["terminal_status"] == "completed"


def test_worker_reconstructs_link_frames_with_nonidentity_inertials(tmp_path):
    case = write_nonidentity_base_and_child_inertial_case(tmp_path)
    completed = run_worker_subprocess(case)
    assert completed.returncode == 0, completed.stderr
    record = json.loads(case.result_path.read_text())
    for state in record["joints"][0]["states"]:
        assert state["engine_frame_receipt"]["base_local_inertial_xyz"] == pytest.approx(
            [0.13, -0.21, 0.34]
        )
        assert [pose["link_name"] for pose in state["link_poses"]] == ["base", "child"]
        for pose in state["link_poses"]:
            np.testing.assert_allclose(
                runner.pose_dict_to_matrix(pose["pybullet"]),
                runner.pose_dict_to_matrix(pose["analytic"]),
                atol=1e-8,
            )


def test_inertial_regression_rejects_com_frame_or_missing_inverse(tmp_path):
    runner = load_runner()
    receipt = run_nonidentity_inertial_fixture(tmp_path)
    assert runner.pose_error(
        receipt["analytic_child"], receipt["pybullet_com_frame"], receipt["bbox_diagonal"]
    ).translation_pass is False
    assert runner.pose_error(
        receipt["analytic_root"],
        receipt["root_without_inertial_inverse"],
        receipt["bbox_diagonal"],
    ).translation_pass is False
```

Add tests that intentionally alter an expected transform to cross the translation and rotation thresholds, that verify all non-driven joints remain at neutral, that test `continuous` endpoints despite `-pi` and `pi` representing the same orientation, and that a malformed URDF yields a structured terminal failure rather than crashing the controller.

### Step 2: Verify engine tests fail

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py -k 'pose_error or worker'
```

Expected: FAIL because worker execution and scoring are absent.

### Step 3: Implement the worker CLI and exact name binding

Add an internal `worker --case-spec PATH` subcommand. In the child process:

1. Parse and validate the URDF analytically before importing PyBullet.
2. Connect with `pybullet.DIRECT`.
3. Call `loadURDF(path, useFixedBase=True, flags=0)` exactly; never OR in `URDF_MAINTAIN_LINK_ORDER`.
4. Build maps from `getJointInfo()` joint/link UTF-8 names and reject missing, extra, or duplicate mappings.
5. Disable default motors on all movable joints with zero-force velocity control.
6. For each evaluated joint and 21 samples, reset that driver to the sample and every other movable joint to neutral.
7. Call `performCollisionDetection()` only as an update barrier; do not report collision as Table 3 evidence.
8. Read back every movable joint. For child links use `getLinkState(..., computeForwardKinematics=1)[4:6]`, the PyBullet world link-frame fields, never `[0:2]` COM fields. Unconditionally read the engine's root local inertial frame from `getDynamicsInfo(body, -1)[3:5]`, even when the URDF omits `<inertial>`, and reconstruct the root link frame as `T_world_base_inertial @ inverse(T_root_link_to_inertial)`. Compute `T_align = inverse(T_world_root_link)` and left-multiply every PyBullet link frame so the URDF root is identity before analytic comparison. Store base pose, engine local inertial pose, and `T_align` in every state receipt.
9. Reset the full configuration to neutral after every sample and verify the round trip.
10. Disconnect in `finally` and atomically write one structured `result.json` even for Python exceptions.

PyBullet native crashes are handled by the parent process, not swallowed by the worker.

### Step 4: Implement the five frozen joint gates

Every case/result/state record has a positive `schema_version`. For each state, persist one row keyed by `(asset_key, joint_key, state_index)`: requested value and native unit; ordered `(joint_name, readback)` pairs for all movable joints; the exact frozen ordered all-link name list; an ordered per-link list with analytic/PyBullet translations and quaternions, finite flags, descendant flags, agreement errors, and non-subtree errors; `d` and all applied thresholds; the engine-frame receipt; and an ordered neutral-return per-link list with round-trip errors. Never serialize JSON `NaN` or infinity: any non-finite engine value becomes JSON `null` with a stable `error_code` and reason. Canonical JSON uses `allow_nan=False`. Verification requires `state_index` exactly `0..20`, exact link/joint order and identity, nullable-value reason closure, and complete post-return coverage, not just row counts. These rows must suffice to recompute every joint gate without rerunning FK. Compute all limits from the bound protocol rather than literals:

```python
thresholds = protocol["thresholds"]
translation_limit = max(
    thresholds["translation_absolute_floor_m"],
    thresholds["translation_relative_to_bbox"] * bbox_diagonal,
)
readback_limit = max(
    thresholds["readback_absolute"],
    thresholds["readback_relative"] * abs(intended_q),
)
rotation_limit = thresholds["rotation_radians"]
```

A joint passes only when all five booleans are true:

- `valid_range`
- `sweep_success`
- `nondegenerate_motion`
- `subtree_invariance`
- `roundtrip`

`sweep_success` includes finite transforms, every readback, and analytic/PyBullet agreement at all 21 states. Determine descendants from the parsed joint tree. A non-descendant must remain within the same translation/rotation tolerances in both implementations. Compute nondegenerate motion separately in analytic and PyBullet poses as the maximum sampled descendant displacement from neutral, using normalized translation OR geodesic rotation, and require both engines to pass. Taking the maximum over all 21 states ensures continuous `-pi` and `pi` do not create a false failure merely because the endpoints coincide.

### Step 5: Run tests

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py
```

Expected: PASS. The laptop regression subprocess must exit normally.

### Step 6: Commit

```bash
git add exp/scripts/run_urdf_sim_ready_table3_lam.py \
  exp/tests/test_urdf_sim_ready_table3_lam.py
git commit -m "feat: evaluate table3 states in pybullet"
```

---

## Task 4: Add the resumable controller and terminal failure accounting

**Files:**

- Modify: `exp/scripts/run_urdf_sim_ready_table3_lam.py`
- Modify: `exp/tests/test_urdf_sim_ready_table3_lam.py`
- Test: `exp/tests/test_urdf_sim_ready_table3_lam.py`

### Step 1: Add failing controller tests

Create a three-asset frozen fixture: one valid, one malformed, and one command that exceeds a short timeout through the generic isolation helper. Verify fixed denominators and resumability.

```python
def test_isolated_command_reports_signal_timeout_and_success(tmp_path):
    runner = load_runner()
    ok = runner.run_isolated([sys.executable, "-c", "print('ok')"], timeout_seconds=5)
    timeout = runner.run_isolated(
        [sys.executable, "-c", "import time; time.sleep(2)"], timeout_seconds=0.05
    )
    crash = runner.run_isolated(
        [sys.executable, "-c", "import os, signal; os.kill(os.getpid(), signal.SIGABRT)"],
        timeout_seconds=5,
    )
    assert ok.status == "completed"
    assert timeout.status == "timeout"
    assert crash.status == "signal" and crash.signal == signal.SIGABRT


def test_controller_retains_failures_and_resumes_only_sealed_cases(tmp_path):
    runner = load_runner()
    output = make_frozen_three_asset_output(tmp_path)
    first = runner.run_controller(output)
    assert first.selected_assets == 3
    assert first.terminal_assets == 3
    assert first.failed_assets == 1
    sealed_before = read_case_seals(output)
    second = runner.run_controller(output)
    assert read_case_seals(output) == sealed_before
    assert second.skipped_verified_assets == 3
```

Also test that an unsealed/invalid/tampered result is rerun, that protocol/manifest hash mismatch aborts before launch, and that a worker timeout cannot leave a false completed result.

### Step 2: Verify controller tests fail

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py -k 'isolated_command or controller'
```

Expected: FAIL because isolation and resume logic are absent.

### Step 3: Implement isolated execution and four-worker scheduling

Add a `run --output-root PATH` subcommand. A formal run does not accept worker-count or timeout overrides; it reads exactly four workers and 180 seconds from the bound protocol. It must:

- Recompute and match protocol, manifest, joint-manifest, resource-manifest, and receipt hashes before doing work.
- Use `concurrent.futures.ThreadPoolExecutor(max_workers=4)` only to supervise subprocesses; every asset still gets a fresh Python process.
- Set `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`, `MKL_NUM_THREADS=1`, and `NUMEXPR_NUM_THREADS=1` in child environments.
- Give each asset an explicit case spec, result path, and seal path under `cases/<selection_rank>-<short_hash>/`, with stdout/stderr at the exact contract paths `logs/<selection_rank>-<short_hash>.stdout.log` and `logs/<selection_rank>-<short_hash>.stderr.log`.
- Kill the process group on timeout, record elapsed time, exit code or signal, and create a terminal controller-owned failure result.
- Perform zero retries and zero replacements.
- Skip a case only when one canonical seal `bindings` object matches protocol, manifest, joint-manifest, resource-manifest, input-receipt, runner, case-spec, URDF, ordered resource-list, result, stdout, and stderr hashes. Controller-owned terminal failures use the same seal schema as worker-owned results.
- Write progress using atomic replace so interruption cannot masquerade as completion.

Use stable error codes including `invalid_input`, `not_evaluable`, `python_exception`, `native_signal`, `timeout`, `missing_result`, and `result_schema_invalid`.

### Step 4: Run controller tests and full unit suite

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py
```

Expected: PASS.

### Step 5: Commit

```bash
git add exp/scripts/run_urdf_sim_ready_table3_lam.py \
  exp/tests/test_urdf_sim_ready_table3_lam.py
git commit -m "feat: isolate and resume table3 assets"
```

---

## Task 5: Aggregate, report, seal, and independently verify outputs

**Files:**

- Modify: `exp/scripts/run_urdf_sim_ready_table3_lam.py`
- Modify: `exp/tests/test_urdf_sim_ready_table3_lam.py`
- Test: `exp/tests/test_urdf_sim_ready_table3_lam.py`

### Step 1: Add failing aggregation and tamper tests

```python
def test_aggregate_uses_fixed_asset_and_joint_denominators(tmp_path):
    runner = load_runner()
    output = make_terminal_fixture(
        tmp_path,
        assets=[
            asset_result("a", category="cat1", joint_passes=[True]),
            asset_result("b", category="cat1", joint_passes=[True, False]),
            failed_asset_result("c", category="cat2", expected_joint_count=1),
        ],
    )
    summary = runner.aggregate_output(output)
    assert summary["denominators"] == {
        "selected_assets": 3,
        "evaluated_joints": 4,
        "categories": 2,
    }
    assert summary["micro"]["joint_pass_count"] == 2
    assert summary["micro"]["joint_pass_rate"] == pytest.approx(0.5)
    assert summary["micro"]["strict_asset_pass_count"] == 1
    assert summary["micro"]["strict_asset_pass_rate"] == pytest.approx(1 / 3)


def test_verify_detects_result_tampering(tmp_path):
    runner = load_runner()
    output = make_completed_output(tmp_path)
    runner.aggregate_output(output)
    runner.verify_output(output)
    result = next((output / "cases").glob("*/result.json"))
    result.write_text(result.read_text() + " ")
    with pytest.raises(runner.VerificationError, match="hash mismatch"):
        runner.verify_output(output)
```

Add tests for macro category averaging, terminal count mismatch, duplicate/missing asset keys, duplicate/missing joint keys, malformed state/link coverage, missing logs, stale seals, gzip record counts, exact report rendering, formal/smoke count dispatch, and read-only verification.

### Step 2: Verify aggregation tests fail

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py -k 'aggregate or verify_detects'
```

Expected: FAIL because aggregation and verification are absent.

### Step 3: Implement deterministic aggregation

Add `aggregate --output-root PATH`. It must refuse partial runs. Read case records in manifest order and produce atomically:

- `state_records.jsonl.gz`
- `joint_records.jsonl`
- `asset_records.jsonl`
- `summary.json`
- `report.md`
- `self_check.json`
- `hashes.sha256`

The joint denominator comes from `joint_manifest.jsonl`, including every expected joint under failed assets as failed records. In `run_kind=formal`, the asset denominator must be exactly 800 and the joint denominator exactly 2243. In `run_kind=smoke`, denominators come from the immutable copied first-eight manifests, may never be reported as formal metrics, and status is always `SMOKE_NOT_A_PAPER_RESULT`. Report both micro joint metrics and category-macro metrics; define the macro unit and preserve categories with terminal failures. Strict asset pass is true only when all expected evaluated joints pass and the asset has no terminal failure.

Compute `joint_category_macro_rate` by dividing each category's passing joints by its frozen planned-joint denominator, then taking the unweighted mean over all represented categories. Compute `strict_asset_category_macro_rate` independently from each category's passing assets over its frozen selected-asset denominator. Categories containing only terminal failures remain in both macro calculations.

For normalized translation and rotation round-trip errors separately, emit mean, median, P90, maximum, `contributing_joint_count`, and the planned joint denominator. Unavailable numeric values are excluded only from the distribution and disclosed by coverage; their joints remain failures in pass-rate denominators. Verification recomputes every statistic from joint/state records.

`report.md` must lead with identity and completion gates, then the Table 3 metrics, failure taxonomy, environment, and limitations. It must call the cross-engine result `analytic/PyBullet FK agreement`, not semantic correctness, collision validity, physical plausibility, or dynamics validity. Implement one deterministic `render_report(summary)` function; verification rerenders and requires a byte-for-byte match so every Markdown table value and two-decimal percentage comes from `summary.json`.

### Step 4: Implement independent verification

Add `verify --output-root PATH`. Recompute every hash and invariant without trusting `summary.json`:

- Protocol ID and SHA match all records.
- Source manifest SHA and pinned dataset revision match receipts.
- For `run_kind=formal`, exactly 800 unique manifest keys and 2243 unique joint keys; for `run_kind=smoke`, exactly the bound first-eight manifest and derived joint-manifest keys. Every asset has one terminal sealed case.
- Every completed joint has exactly 21 unique state identities and complete ordered link coverage. Every uncompleted joint has a manifest-derived failed joint row with `state_coverage_reason` bound to its terminal asset seal; unavailable numeric errors remain unavailable and are not imputed as zero.
- No retries or replacements appear.
- All case seals and resource hashes match.
- Aggregated JSONL/gzip rows reproduce summary counts and rates.
- `hashes.sha256` covers every immutable input, case result/seal/log, and aggregate artifact except itself.
- A freshly rendered `report.md` matches the sealed report byte-for-byte.
- The Python executable, runner, dependency versions, and PyBullet API value reproduce the bound environment receipt.

`aggregate` computes and atomically writes the deterministic `self_check.json`, then writes the final `hashes.sha256`. After that, `verify` is strictly read-only: it recomputes and compares hashes, records, summary, self-check, and report in memory, exits nonzero on any mismatch, and never mutates the sealed root.

Add `smoke --formal-output-root PATH --output-root PATH`. It must copy and bind exactly the first eight formal manifest rows and their joint/resource records into a new exclusive root, write immutable `run_kind=smoke` and parent formal hashes in its receipt, set status `SMOKE_NOT_A_PAPER_RESULT`, and run the same worker/controller/aggregate/verify implementations with the frozen smoke concurrency. Count rules dispatch on the immutable run kind. Smoke results never modify formal manifests or case directories.

### Step 5: Run all tests

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py
```

Expected: PASS.

### Step 6: Commit

```bash
git add exp/scripts/run_urdf_sim_ready_table3_lam.py \
  exp/tests/test_urdf_sim_ready_table3_lam.py
git commit -m "feat: seal and verify table3 results"
```

---

## Task 6: Freeze inputs, run smoke, execute the formal N=800 evaluation, and verify

**Files:**

- Create: `exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1/`
- Create: `exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1_smoke_first8/`
- Verify: all formal runtime artifacts

### Step 1: Recheck runtime ownership and capacity

Run read-only checks immediately before launch:

```bash
nvidia-smi
df -h /mnt/zsn/lyb/arti-skill /dev/shm
ps -eo pid,ppid,etime,cmd --sort=-etime | rg 'run_urdf_sim_ready_table3_lam|pybullet' || true
```

This evaluator is CPU-only. Do not stop or reuse any unrelated GPU or CPU process.

### Step 2: Run the complete test suite and compile check

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m py_compile \
  exp/scripts/run_urdf_sim_ready_table3_lam.py
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python -m pytest -q \
  exp/tests/test_urdf_sim_ready_table3_lam.py
```

Expected: PASS.

### Step 3: Freeze the formal cohort exactly once

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  exp/scripts/run_urdf_sim_ready_table3_lam.py freeze \
  --protocol exp/reference/urdf_sim_ready_table3_lam_viable_n800_v1.json \
  --dataset-root exp/Articulated-Object-Code \
  --output-root exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1
```

Expected: the receipt reports `N_release=2533`, `N_eval=800`, 280 categories, and 2243 evaluated joints. Record and inspect the printed protocol, manifest, URDF, resource, and runner hashes before any worker starts.

### Step 4: Run an explicitly non-formal first-eight smoke cohort

Run:

```bash
cd /mnt/zsn/lyb/arti-skill
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  exp/scripts/run_urdf_sim_ready_table3_lam.py smoke \
  --formal-output-root exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1 \
  --output-root exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1_smoke_first8_attempt01
```

Expected: eight terminal asset records and a smoke report explicitly labeled non-formal. Inspect all failure records and logs. Fix implementation defects only; never alter protocol constants, cohort ordering, or data to improve results. After a defect fix, rerun tests and create a new exclusive sibling root such as `..._smoke_first8_attempt02`; never rewrite a completed smoke root.

### Step 5: Launch and finish the formal evaluation detached

Run:

```bash
cd /mnt/zsn/lyb/arti-skill
nohup setsid /mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  exp/scripts/run_urdf_sim_ready_table3_lam.py run \
  --output-root exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1 \
  >> exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1/controller.log 2>&1 < /dev/null &
```

The controller writes its own PID atomically to `controller.pid` immediately after start. Verify that PID and its exact command line, then monitor `controller.log`, terminal case-seal growth, and failure-class counts. If interrupted, rerun the identical command. Resume only from valid case seals. Do not delete or rewrite terminal failures, and do not replace assets.

Expected: 800/800 terminal cases, with zero active child processes after the controller exits.

### Step 6: Aggregate and verify the formal output

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  exp/scripts/run_urdf_sim_ready_table3_lam.py aggregate \
  --output-root exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  exp/scripts/run_urdf_sim_ready_table3_lam.py verify \
  --output-root exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1
```

Expected: both commands exit 0; `self_check.json` contains no failed checks; all 800 selected assets and 2243 expected joints remain in the denominators.

### Step 7: Independently inspect the result and live process state

Run:

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python - <<'PY'
import gzip
import json
from pathlib import Path

root = Path("exp/runtime/urdf_sim_ready_table3_lam_viable_n800_v1")
summary = json.loads((root / "summary.json").read_text())
self_check = json.loads((root / "self_check.json").read_text())
assets = sum(1 for _ in (root / "asset_records.jsonl").open())
joints = sum(1 for _ in (root / "joint_records.jsonl").open())
with gzip.open(root / "state_records.jsonl.gz", "rt") as handle:
    states = sum(1 for _ in handle)
print(json.dumps({
    "assets": assets,
    "joints": joints,
    "states": states,
    "summary": summary,
    "self_check": self_check,
}, indent=2, sort_keys=True))
PY
ps -eo pid,ppid,etime,cmd | rg 'run_urdf_sim_ready_table3_lam|pybullet' || true
```

Expected: asset and joint counts match the frozen manifests; state counts match completed joints times 21; no evaluator worker remains alive.

### Step 8: Commit code and immutable receipts, not bulky per-state runtime data

Before committing, inspect `git status --short` and add only evaluator-owned source, tests, protocol, design, plan, and concise receipts/report if repository policy permits. Do not stage unrelated user files or large generated case/state artifacts.

```bash
git status --short
git add exp/reference/urdf_sim_ready_table3_lam_viable_n800_v1.json \
  exp/scripts/run_urdf_sim_ready_table3_lam.py \
  exp/tests/test_urdf_sim_ready_table3_lam.py
git commit -m "feat: complete LAM sim-ready table3 evaluator"
```

Only claim completion after the verifier exits 0 and the independent counts agree. Report formal metrics directly from `summary.json`, with exact denominators and failure counts, and link `report.md`, `summary.json`, `self_check.json`, and the frozen manifest.
