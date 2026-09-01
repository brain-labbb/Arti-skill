from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_table2_supplementary_artiverse.py"
VERIFIER = REPO / "exp/scripts/verify_table2_supplementary_v1.py"

CUBE_OBJ = """\
v -0.5 -0.5 -0.5
v 0.5 -0.5 -0.5
v 0.5 0.5 -0.5
v -0.5 0.5 -0.5
v -0.5 -0.5 0.5
v 0.5 -0.5 0.5
v 0.5 0.5 0.5
v -0.5 0.5 0.5
f 1 2 3 4
f 5 6 7 8
f 1 2 6 5
f 2 3 7 6
f 3 4 8 7
f 4 1 5 8
"""

GOOD_URDF = """\
<?xml version="1.0"?>
<robot name="model_a">
  <link name="base">
    <visual><geometry><mesh filename="cube.obj"/></geometry></visual>
    <collision><geometry><mesh filename="cube.obj"/></geometry></collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <link name="lid">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="lid"/>
    <limit lower="0" upper="1" effort="10" velocity="2"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>
</robot>
"""

BROKEN_URDF = """\
<?xml version="1.0"?>
<robot name="model_b">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><mesh filename="missing.obj"/></geometry></collision>
  </link>
  <link name="lid">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="lid"/>
    <limit lower="0" upper="1" effort="10" velocity="2"/>
  </joint>
</robot>
"""


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


class SyntheticRunTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="t2s_run_")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.runner = load_module(RUNNER, "t2s_runner_under_test")

    def _build_dataset(self):
        artiverse_root = self.root / "artiverse"
        packages = {}
        for model, urdf, with_obj in (
            ("model_a", GOOD_URDF, True),
            ("model_b", BROKEN_URDF, False),
        ):
            package = artiverse_root / "data" / "cat" / "src" / model / "urdf_w_collider"
            package.mkdir(parents=True)
            (package / f"{model}.urdf").write_text(urdf, encoding="utf-8")
            if with_obj:
                (package / "cube.obj").write_text(CUBE_OBJ, encoding="utf-8")
            packages[model] = package

        # Frozen Table 2-style content manifests.
        bindings = {}
        binding_files = {}
        for model, package in packages.items():
            entries = []
            for full in sorted(package.rglob("*")):
                if full.is_file():
                    entries.append({
                        "bytes": full.stat().st_size,
                        "path": full.relative_to(package).as_posix(),
                        "sha256": sha256_bytes(full.read_bytes()),
                    })
            entries.sort(key=lambda entry: entry["path"])
            bindings[model] = self.runner.canonical_sha256(entries)
            binding_files[model] = entries
        return artiverse_root, packages, bindings, binding_files

    def _write_inputs(self, artiverse_root, packages, bindings, binding_files):
        manifest_roots = [f"data/cat/src/{model}" for model in ("model_a", "model_b")]
        table1 = {
            "dataset": "Artiverse",
            "seed": "20260813",
            "assets": [
                {"manifest_root": root, "model_id": root.rsplit("/", 1)[-1], "raw_category": "cat", "source": "src"}
                for root in manifest_roots
            ],
        }
        table1_path = self.root / "table1_manifest.json"
        table1_path.write_text(json.dumps(table1, indent=2), encoding="utf-8")

        records = []
        for index, root in enumerate(manifest_roots):
            model = root.rsplit("/", 1)[-1]
            package = packages[model]
            urdf_bytes = (package / f"{model}.urdf").read_bytes()
            records.append({
                "asset_id": root,
                "manifest_root": root,
                "model_id": model,
                "raw_category": "cat",
                "source": "src",
                "chunk_archive": "synthetic.tar.gz",
                "selection_rank": index + 1,
                "selection_hash": f"hash_{model}",
                "package": package.as_posix(),
                "primary_urdf_relative_path": f"{model}.urdf",
                "model_urdf_sha256": sha256_bytes(urdf_bytes),
                "package_binding": {
                    "content_manifest_sha256": bindings[model],
                    "file_count": len(binding_files[model]),
                    "total_bytes": 0,
                    "files": binding_files[model],
                },
            })
        table2 = {"records": records}
        body_hash = self.runner.canonical_sha256(table2)
        table2["manifest_content_sha256"] = body_hash
        table2_path = self.root / "table2_manifest.json"
        table2_path.write_text(json.dumps(table2), encoding="utf-8")

        table3_path = self.root / "table3_records.jsonl"
        table3_path.write_text(
            "\n".join(
                json.dumps({"asset_id": root, "declared_joint_count": 1})
                for root in manifest_roots
            ) + "\n",
            encoding="utf-8",
        )
        return table1_path, table2_path, table3_path, body_hash

    def test_synthetic_cohort_end_to_end(self):
        artiverse_root, packages, bindings, binding_files = self._build_dataset()
        table1_path, table2_path, table3_path, body_hash = self._write_inputs(
            artiverse_root, packages, bindings, binding_files
        )

        runner = self.runner
        runner.ARTIVERSE_ROOT = artiverse_root
        runner.TABLE1_MANIFEST = table1_path
        runner.TABLE2_MANIFEST = table2_path
        runner.TABLE3_RECORDS = table3_path
        runner.EXPECTED_TABLE1_MANIFEST_SHA256 = sha256_bytes(table1_path.read_bytes())
        runner.EXPECTED_TABLE2_MANIFEST_SELF_SHA256 = body_hash
        runner.EXPECTED_COHORT_SIZE = 2
        runner.EXPECTED_J_EVAL = 2

        output_root = self.root / "output"
        exit_code = runner.main([
            "--output", str(output_root),
            "--workers", "2",
        ])
        self.assertEqual(exit_code, 0)

        summary = json.loads((output_root / "summary.json").read_text(encoding="utf-8"))
        metrics = summary["metrics"]
        # model_a passes visual-bearing coverage; model_b fails (missing mesh).
        self.assertEqual(metrics["visual_bearing_collision_coverage"]["asset_level"]["numerator"], 1)
        self.assertEqual(metrics["visual_bearing_collision_coverage"]["asset_level"]["denominator"], 2)
        # link micro: model_a covers 2/2 visual links; model_b declares 2, covers 1 (lid box).
        self.assertEqual(metrics["visual_bearing_collision_coverage"]["link_micro"]["numerator"], 3)
        self.assertEqual(metrics["visual_bearing_collision_coverage"]["link_micro"]["denominator"], 4)
        # Both joints portable; dynamics covered only for model_a.
        self.assertEqual(metrics["joint_limit_portability"]["joint_level"]["numerator"], 2)
        self.assertEqual(metrics["joint_limit_portability"]["joint_level"]["denominator"], 2)
        self.assertEqual(metrics["joint_dynamics_coverage"]["joint_level"]["numerator"], 1)
        self.assertEqual(metrics["joint_dynamics_coverage"]["joint_level"]["denominator"], 2)
        # Placeholder registry frozen empty -> N/E with coverage 1 complete / 4 dynamic links.
        placeholder = metrics["placeholder_mass_incidence"]
        self.assertEqual(placeholder["status"], "N/E")
        self.assertEqual(placeholder["complete_inertial_coverage"]["numerator"], 1)
        self.assertEqual(placeholder["complete_inertial_coverage"]["denominator"], 4)

        verification = json.loads((output_root / "verification.json").read_text(encoding="utf-8"))
        self.assertEqual(verification["status"], "PASS", verification)

        # Independent verifier agrees when invoked directly.
        verifier = load_module(VERIFIER, "t2s_verifier_under_test")
        direct = verifier.verify_run(
            output_root,
            table1_manifest=table1_path,
            expected_table1_sha256=sha256_bytes(table1_path.read_bytes()),
        )
        self.assertEqual(direct["status"], "PASS")

    def test_binding_drift_is_fail_closed(self):
        artiverse_root, packages, bindings, binding_files = self._build_dataset()
        table1_path, table2_path, table3_path, body_hash = self._write_inputs(
            artiverse_root, packages, bindings, binding_files
        )
        # Drift the bytes of model_a after the binding was frozen.
        drifted = packages["model_a"] / "cube.obj"
        drifted.write_text(CUBE_OBJ + "# drift\n", encoding="utf-8")

        runner = self.runner
        runner.ARTIVERSE_ROOT = artiverse_root
        runner.TABLE1_MANIFEST = table1_path
        runner.TABLE2_MANIFEST = table2_path
        runner.TABLE3_RECORDS = table3_path
        runner.EXPECTED_TABLE1_MANIFEST_SHA256 = sha256_bytes(table1_path.read_bytes())
        runner.EXPECTED_TABLE2_MANIFEST_SELF_SHA256 = body_hash
        runner.EXPECTED_COHORT_SIZE = 2
        runner.EXPECTED_J_EVAL = 2

        output_root = self.root / "output_drift"
        exit_code = runner.main(["--output", str(output_root), "--workers", "2"])
        self.assertEqual(exit_code, 0)
        summary = json.loads((output_root / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["status_counts"].get("binding_failed"), 1)
        # Drifted asset keeps denominators but fails every metric.
        self.assertEqual(summary["metrics"]["visual_bearing_collision_coverage"]["asset_level"]["numerator"], 0)
        self.assertEqual(summary["metrics"]["joint_limit_portability"]["joint_level"]["denominator"], 2)
        verification = json.loads((output_root / "verification.json").read_text(encoding="utf-8"))
        self.assertEqual(verification["status"], "PASS", verification)


if __name__ == "__main__":
    unittest.main()
