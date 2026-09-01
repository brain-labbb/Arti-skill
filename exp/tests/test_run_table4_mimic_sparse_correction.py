from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


EXP = Path(__file__).parents[1]
SCRIPT = EXP / "scripts" / "run_table4_mimic_sparse_correction.py"
SPEC = importlib.util.spec_from_file_location("table4_sparse_correction_test", SCRIPT)
assert SPEC and SPEC.loader
correction = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(correction)
runner = correction.table4
common = correction.roster_common


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def urdf(*, mimic: bool = False, shifted_range: bool = False) -> str:
    lower, upper = ("1", "2") if shifted_range else ("-1", "1")
    follower = ""
    if mimic:
        follower = '''
  <link name="follower_link"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="follower" type="revolute">
    <parent link="driver_link"/><child link="follower_link"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
    <mimic joint="driver" multiplier="0.5" offset="0"/>
  </joint>'''
    return f'''<robot name="fixture">
  <link name="base"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <link name="driver_link"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="driver" type="revolute">
    <parent link="base"/><child link="driver_link"/>
    <axis xyz="0 0 1"/><limit lower="{lower}" upper="{upper}" effort="1" velocity="1"/>
  </joint>{follower}
</robot>
'''


def package_row(
    root: Path,
    asset_id: str,
    *,
    mimic: bool = False,
    shifted_range: bool = False,
    invalid: bool = False,
) -> dict[str, object]:
    package = root / asset_id
    package.mkdir(parents=True)
    model = package / "model.urdf"
    model.write_text(
        "SOURCE_COMPILE_FAILED\n" if invalid else urdf(mimic=mimic, shifted_range=shifted_range),
        encoding="utf-8",
    )
    joints = [{"name": "driver", "type": "revolute"}]
    if mimic:
        joints.append({"name": "follower", "type": "revolute"})
    return {
        "asset_id": asset_id,
        "category": "fixture",
        "source_path": str(package),
        "source_relative_path": asset_id,
        "primary_urdf_path": str(model),
        "primary_urdf_relative_path": "model.urdf",
        "primary_urdf_sha256": sha(model),
        "non_fixed_joints": joints,
        "joint_count": len(joints),
    }


@pytest.fixture()
def full_evidence(tmp_path: Path) -> dict[str, Path]:
    packages = tmp_path / "packages"
    packages.mkdir()
    rows = [
        package_row(packages, "a_stable"),
        package_row(packages, "b_mimic", mimic=True),
        package_row(packages, "c_shifted", shifted_range=True),
        package_row(packages, "d_invalid", invalid=True),
    ]
    roster_dir = tmp_path / "full_roster"
    roster_dir.mkdir()
    roster_path = roster_dir / "full_release_manifest.json"
    common.freeze_roster(
        rows,
        roster_path,
        dataset="fixture",
        source_bindings=[{"name": "fixture", "path": str(packages)}],
    )
    parent = tmp_path / "parent_v1"
    runner.run_dataset(
        roster_path,
        parent,
        dataset="articraft",
        sampling_protocol=runner.SAMPLING_PROTOCOL_V1,
        workers=1,
        timeout_seconds=30,
    )
    return {"roster": roster_path, "parent": parent}


def run_prepared_sparse(full_evidence: dict[str, Path], tmp_path: Path) -> dict[str, Path]:
    prepared = tmp_path / "prepared"
    result = correction.prepare(full_evidence["roster"], prepared, "articraft")
    assert result["selected_asset_count"] == 3
    assert result["reason_counts"] == {
        "mimic": 1,
        "range_semantic_change": 1,
        "sampling_plan_error": 1,
    }
    subset = prepared / "full_release_manifest.json"
    subset_value = common.load_roster(subset)
    assert [row["asset_id"] for row in subset_value["rows"]] == [
        "b_mimic",
        "c_shifted",
        "d_invalid",
    ]
    assert [row["sparse_parent_ordinal"] for row in subset_value["rows"]] == [1, 2, 3]
    sparse = tmp_path / "sparse_v2"
    runner.run_dataset(
        subset,
        sparse,
        dataset="articraft",
        sampling_protocol=runner.SAMPLING_PROTOCOL_V2,
        workers=1,
        timeout_seconds=30,
    )
    return {"prepared": prepared, "subset": subset, "sparse": sparse}


def test_prepare_selects_every_protocol_sensitive_asset(
    full_evidence: dict[str, Path], tmp_path: Path
) -> None:
    paths = run_prepared_sparse(full_evidence, tmp_path)
    receipt = json.loads((paths["prepared"] / "prepare_receipt.json").read_text())
    assert receipt["selection"]["selected_asset_count"] == 3
    assert receipt["selection"]["unselected_asset_count"] == 1
    assert receipt["subset_roster"]["path"] == "full_release_manifest.json"


def test_merge_and_verify_preserve_unselected_semantics(
    full_evidence: dict[str, Path], tmp_path: Path
) -> None:
    paths = run_prepared_sparse(full_evidence, tmp_path)
    output = tmp_path / "corrected"
    result = correction.merge(
        full_roster=full_evidence["roster"],
        subset_roster=paths["subset"],
        parent_output=full_evidence["parent"],
        sparse_output=paths["sparse"],
        output=output,
        dataset_slug="articraft",
    )
    assert result["status"] == "COMPLETE"
    report = correction.verify(output)
    assert report["all_pass"] is True
    records = [json.loads(line) for line in (output / "records.jsonl").read_text().splitlines()]
    assert [record["dataset_id"] for record in records] == [
        "a_stable",
        "b_mimic",
        "c_shifted",
        "d_invalid",
    ]
    assert (records[0].get("sampling_protocol") or correction.V1) == correction.V1
    assert all(record.get("sampling_protocol") == correction.V2 for record in records[1:])
    summary = json.loads((output / "summary.json").read_text())
    assert summary["sampling_protocol"] == correction.EFFECTIVE_PROTOCOL
    assert summary["physical_record_protocol_counts"] == {
        correction.V1: 1,
        correction.V2: 3,
    }
    assert summary["state_records_count"] == report["state_records"]


def test_merge_rejects_sparse_record_gap(
    full_evidence: dict[str, Path], tmp_path: Path
) -> None:
    paths = run_prepared_sparse(full_evidence, tmp_path)
    records = (paths["sparse"] / "asset_records.jsonl").read_text().splitlines()
    (paths["sparse"] / "asset_records.jsonl").write_text("\n".join(records[:-1]) + "\n")
    with pytest.raises(correction.CorrectionError, match="record count mismatch"):
        correction.merge(
            full_roster=full_evidence["roster"],
            subset_roster=paths["subset"],
            parent_output=full_evidence["parent"],
            sparse_output=paths["sparse"],
            output=tmp_path / "must_not_publish",
            dataset_slug="articraft",
        )
    assert not (tmp_path / "must_not_publish").exists()
