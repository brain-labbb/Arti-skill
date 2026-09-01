from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).parents[1]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


oracle_module = _load(
    "table4_zero_margin_oracle_test",
    ROOT / "scripts" / "table4_zero_margin_oracle.py",
)
runner = _load(
    "table4_zero_margin_runner_test",
    ROOT / "scripts" / "run_table4_full_release.py",
)


THIN_BOX_OBJ = """\
o thin_box
v -0.010 -0.005 -0.0005
v  0.010 -0.005 -0.0005
v  0.010  0.005 -0.0005
v -0.010  0.005 -0.0005
v -0.010 -0.005  0.0005
v  0.010 -0.005  0.0005
v  0.010  0.005  0.0005
v -0.010  0.005  0.0005
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
"""


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _geometry_pair_urdf(geometry: str, separation: float) -> str:
    def collision(z: float) -> str:
        return f"""
    <collision>
      <origin xyz="0.003 -0.004 {z:.12g}" rpy="0 0 0.37"/>
      <geometry>{geometry}</geometry>
    </collision>"""

    return f"""<?xml version="1.0"?>
<robot name="margin_pair">
  <link name="root"/>
  <link name="lower">{collision(0.0)}</link>
  <link name="upper">{collision(separation)}</link>
  <joint name="root_to_lower" type="fixed">
    <parent link="root"/><child link="lower"/>
  </joint>
  <joint name="root_to_upper" type="fixed">
    <parent link="root"/><child link="upper"/>
  </joint>
</robot>
"""


def _write_pair(
    tmp_path: Path,
    geometry: str,
    separation: float,
    *,
    write_mesh: bool = False,
) -> Path:
    package = tmp_path / f"pair_{str(separation).replace('.', '_')}"
    package.mkdir()
    if write_mesh:
        (package / "thin_box.obj").write_text(THIN_BOX_OBJ, encoding="ascii")
    urdf = package / "model.urdf"
    urdf.write_text(
        _geometry_pair_urdf(geometry, separation), encoding="ascii"
    )
    return urdf


def _load_body(bullet, urdf: Path, *, implicit_cylinder: bool = True):
    client = bullet.connect(bullet.DIRECT)
    flags = int(
        bullet.URDF_USE_SELF_COLLISION
        | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
        | bullet.URDF_IGNORE_VISUAL_SHAPES
    )
    if implicit_cylinder:
        flags |= int(bullet.URDF_USE_IMPLICIT_CYLINDER)
    body = bullet.loadURDF(
        str(urdf),
        useFixedBase=True,
        flags=flags,
        physicsClientId=client,
    )
    bullet.performCollisionDetection(physicsClientId=client)
    return client, body


def _build_oracle(bullet, client: int, body: int, urdf: Path):
    result = oracle_module.ZeroMarginProxyOracle.build(
        bullet,
        body,
        client,
        urdf,
        urdf.parent,
        lambda _package, source, filename: (source.parent / filename).resolve(),
    )
    bullet.performCollisionDetection(physicsClientId=client)
    return result


def _proxy_ids(margin_oracle) -> set[int]:
    return {
        proxy
        for proxies in margin_oracle.proxies.values()
        for proxy in proxies
    }


@pytest.mark.parametrize(
    ("geometry", "separation"),
    (
        ('<box size="0.02 0.01 0.01"/>', 0.005),
        ('<cylinder radius="0.01" length="0.01"/>', 0.005),
    ),
)
def test_fixed_child_proxy_preserves_true_primitive_overlap(
    tmp_path: Path, geometry: str, separation: float
) -> None:
    bullet = pytest.importorskip("pybullet")
    urdf = _write_pair(tmp_path, geometry, separation)
    client, body = _load_body(bullet, urdf)
    margin_oracle = None
    try:
        margin_oracle = _build_oracle(bullet, client, body, urdf)
        result = margin_oracle.observe(set())

        assert result["non_adjacent_max_penetration_m"] == pytest.approx(
            0.005, abs=2e-5
        )
        assert result["non_adjacent_illegal_penetration_count"] > 0
        assert result["zero_margin_rechecked_link_pair_count"] == 1
        assert result["zero_margin_detected_link_pair_count"] == 1
        receipt = margin_oracle.receipt()
        assert receipt["proxy_pair_coverage"] == (
            "all_source_link_pairs_across_all_proxy_body_chunks"
        )
        assert receipt["source_collision_filter_group"] == (
            oracle_module.SOURCE_COLLISION_FILTER_GROUP
        )
        assert receipt["proxy_collision_filter_group"] == (
            oracle_module.PROXY_COLLISION_FILTER_GROUP
        )
        assert receipt["source_proxy_isolation_policy"] == (
            "disjoint_collision_filter_group_and_mask"
        )
        for record in receipt["margin_calibration_records"]:
            assert record["proxy_shape_link_index"] == 0
            assert 0.0 <= record["proxy_margin_readback_m"] <= (
                oracle_module.NUMERICAL_ZERO_READBACK_MAX_M
            )
    finally:
        if margin_oracle is not None:
            margin_oracle.close()
        bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


@pytest.mark.parametrize(
    "geometry",
    (
        '<box size="0.02 0.01 0.01"/>',
        '<cylinder radius="0.01" length="0.01"/>',
    ),
)
def test_fixed_child_proxy_preserves_known_primitive_gap(
    tmp_path: Path, geometry: str
) -> None:
    bullet = pytest.importorskip("pybullet")
    urdf = _write_pair(tmp_path, geometry, 0.01024)
    client, body = _load_body(bullet, urdf)
    margin_oracle = None
    try:
        margin_oracle = _build_oracle(bullet, client, body, urdf)
        result = margin_oracle.observe(set())
        assert result["non_adjacent_contact_count"] == 0
        assert result["non_adjacent_max_penetration_m"] == 0.0
    finally:
        if margin_oracle is not None:
            margin_oracle.close()
        bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def test_compass_scale_mesh_gap_drops_only_default_margin_contact(
    tmp_path: Path,
) -> None:
    bullet = pytest.importorskip("pybullet")
    urdf = _write_pair(
        tmp_path,
        '<mesh filename="thin_box.obj"/>',
        0.00124,
        write_mesh=True,
    )
    client, body = _load_body(bullet, urdf)
    margin_oracle = None
    try:
        raw_contacts = bullet.getContactPoints(
            bodyA=body, bodyB=body, physicsClientId=client
        )
        raw_depth = max([-float(row[8]) for row in raw_contacts] or [0.0])
        assert raw_depth == pytest.approx(0.00176, abs=3e-5)

        margin_oracle = _build_oracle(bullet, client, body, urdf)
        result = margin_oracle.observe(set())
        assert result["raw_non_adjacent_max_penetration_m"] == pytest.approx(
            raw_depth, abs=1e-12
        )
        assert result["non_adjacent_contact_count"] == 0
        assert result["non_adjacent_max_penetration_m"] == 0.0
        assert margin_oracle.object_bbox_diagonal() > 0.02

        proxy_ids = _proxy_ids(margin_oracle)
        for contact in bullet.getContactPoints(physicsClientId=client):
            proxy_a = int(contact[1]) in proxy_ids
            proxy_b = int(contact[2]) in proxy_ids
            assert proxy_a == proxy_b
    finally:
        if margin_oracle is not None:
            margin_oracle.close()
        bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def test_mesh_proxy_reports_true_submillimeter_overlap(tmp_path: Path) -> None:
    bullet = pytest.importorskip("pybullet")
    urdf = _write_pair(
        tmp_path,
        '<mesh filename="thin_box.obj"/>',
        0.0005,
        write_mesh=True,
    )
    client, body = _load_body(bullet, urdf)
    margin_oracle = None
    try:
        margin_oracle = _build_oracle(bullet, client, body, urdf)
        result = margin_oracle.observe(set())
        assert result["non_adjacent_max_penetration_m"] == pytest.approx(
            0.0005, abs=3e-5
        )
        assert result["non_adjacent_illegal_penetration_count"] > 0
    finally:
        if margin_oracle is not None:
            margin_oracle.close()
        bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def test_watch_scale_implicit_cylinders_remove_axial_margin_contact(
    tmp_path: Path,
) -> None:
    bullet = pytest.importorskip("pybullet")
    urdf = _write_pair(
        tmp_path,
        '<cylinder radius="0.0011" length="0.00056"/>',
        0.0008,
    )

    default_client, default_body = _load_body(bullet, urdf, implicit_cylinder=False)
    margin_oracle = None
    try:
        default_contacts = bullet.getContactPoints(
            bodyA=default_body,
            bodyB=default_body,
            physicsClientId=default_client,
        )
        default_depth = max(
            [-float(row[8]) for row in default_contacts] or [0.0]
        )
        assert default_depth == pytest.approx(0.00176, abs=4e-5)
        margin_oracle = _build_oracle(
            bullet, default_client, default_body, urdf
        )
        calibrated = margin_oracle.observe(set())
        assert calibrated["raw_non_adjacent_max_penetration_m"] == pytest.approx(
            default_depth, abs=1e-12
        )
        assert calibrated["non_adjacent_contact_count"] == 0
    finally:
        if margin_oracle is not None:
            margin_oracle.close()
        bullet.removeBody(default_body, physicsClientId=default_client)
        bullet.disconnect(default_client)

    implicit_client, implicit_body = _load_body(
        bullet, urdf, implicit_cylinder=True
    )
    try:
        assert not bullet.getContactPoints(
            bodyA=implicit_body,
            bodyB=implicit_body,
            physicsClientId=implicit_client,
        )
    finally:
        bullet.removeBody(implicit_body, physicsClientId=implicit_client)
        bullet.disconnect(implicit_client)


def test_shape_support_is_explicit_and_unsupported_geometry_fails_closed(
    tmp_path: Path,
) -> None:
    bullet = pytest.importorskip("pybullet")
    mesh = tmp_path / "thin_box.obj"
    mesh.write_text(THIN_BOX_OBJ, encoding="ascii")
    supported = {
        "box": '<box size="1 1 1"/>',
        "sphere": '<sphere radius="1"/>',
        "cylinder": '<cylinder radius="1" length="1"/>',
        "capsule": '<capsule radius="1" length="1"/>',
        "mesh": '<mesh filename="thin_box.obj"/>',
    }
    for kind, geometry in supported.items():
        collision = ET.fromstring(
            f'<collision><origin xyz="1 2 3"/><geometry>{geometry}</geometry></collision>'
        )
        observed_kind, kwargs, position, _orientation = oracle_module._shape_kwargs(
            bullet,
            collision,
            package=tmp_path,
            urdf=tmp_path / "model.urdf",
            resolve_mesh=lambda _package, source, name: source.parent / name,
        )
        assert observed_kind == kind
        assert position == (1.0, 2.0, 3.0)
        assert "collisionFramePosition" not in kwargs
        assert "collisionFrameOrientation" not in kwargs

    unsupported = ET.fromstring(
        '<collision><geometry><ellipsoid size="1 1 1"/></geometry></collision>'
    )
    with pytest.raises(ValueError, match="does not support geometry"):
        oracle_module._shape_kwargs(
            bullet,
            unsupported,
            package=tmp_path,
            urdf=tmp_path / "model.urdf",
            resolve_mesh=lambda _package, source, name: source.parent / name,
        )


def test_compound_proxy_builds_all_supported_solid_geometry_types(
    tmp_path: Path,
) -> None:
    bullet = pytest.importorskip("pybullet")
    (tmp_path / "thin_box.obj").write_text(THIN_BOX_OBJ, encoding="ascii")
    collisions = """
    <collision><origin xyz="0 0 0"/><geometry><box size="0.01 0.01 0.01"/></geometry></collision>
    <collision><origin xyz="0.03 0 0"/><geometry><sphere radius="0.005"/></geometry></collision>
    <collision><origin xyz="0.06 0 0"/><geometry><cylinder radius="0.005" length="0.01"/></geometry></collision>
    <collision><origin xyz="0.09 0 0"/><geometry><capsule radius="0.005" length="0.01"/></geometry></collision>
    <collision><origin xyz="0.12 0 0"/><geometry><mesh filename="thin_box.obj"/></geometry></collision>
    """
    urdf = tmp_path / "model.urdf"
    urdf.write_text(
        f'<robot name="all_shapes"><link name="root">{collisions}</link></robot>',
        encoding="ascii",
    )
    client, body = _load_body(bullet, urdf, implicit_cylinder=False)
    margin_oracle = None
    try:
        margin_oracle = _build_oracle(bullet, client, body, urdf)
        receipt = margin_oracle.receipt()
        records = receipt["margin_calibration_records"]
        assert {record["geometry_kind"] for record in records} == {
            "box",
            "sphere",
            "cylinder",
            "capsule",
            "mesh",
        }
        assert receipt["algorithmic_margin_adjusted_element_count"] == 3
        assert receipt["intrinsic_geometry_margin_preserved_element_count"] == 2
        assert bullet.getNumJoints(
            next(iter(margin_oracle.proxies.values()))[0],
            physicsClientId=client,
        ) == 5
        for record in records:
            assert record["post_compose_shape_margin_readback_m"] == pytest.approx(
                record["margin_after_m"], abs=1e-15
            )
    finally:
        if margin_oracle is not None:
            margin_oracle.close()
        bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def _chunked_proxy_urdf(collision_count: int) -> str:
    collisions: list[str] = []
    for index in range(collision_count):
        # Elements 63 and 64 deliberately overlap across the first chunk
        # boundary.  That is same-source geometry and must not become an
        # inter-link collision merely because the implementation uses chunks.
        position_index = 63 if index == 64 else index
        collisions.append(
            "<collision>"
            f'<origin xyz="{position_index * 0.02:.12g} 0 0"/>'
            '<geometry><box size="0.01 0.01 0.01"/></geometry>'
            "</collision>"
        )
    peer_position = (collision_count - 1) * 0.02
    return (
        '<robot name="chunked_proxy">\n'
        f'<link name="root">{"\n".join(collisions)}</link>\n'
        '<link name="peer"><collision>\n'
        f'<origin xyz="{peer_position:.12g} 0 0"/>'
        '<geometry><box size="0.01 0.01 0.01"/></geometry>\n'
        "</collision></link>\n"
        '<joint name="root_to_peer" type="fixed">\n'
        '<parent link="root"/><child link="peer"/>'
        "</joint>\n</robot>"
    )


@pytest.mark.parametrize("collision_count", (128, 255, 705))
def test_large_source_link_is_chunked_without_dropping_last_element(
    tmp_path: Path, collision_count: int
) -> None:
    bullet = pytest.importorskip("pybullet")
    urdf = tmp_path / f"chunked_{collision_count}.urdf"
    urdf.write_text(_chunked_proxy_urdf(collision_count), encoding="ascii")
    client, body = _load_body(bullet, urdf, implicit_cylinder=False)
    margin_oracle = None
    try:
        margin_oracle = _build_oracle(bullet, client, body, urdf)
        limit = oracle_module.MAX_PROXY_COLLISION_ELEMENTS_PER_BODY
        root_proxies = margin_oracle.proxies[-1]
        expected_root_proxy_count = (collision_count + limit - 1) // limit
        assert len(root_proxies) == expected_root_proxy_count
        root_proxy_joint_counts = [
            bullet.getNumJoints(proxy, physicsClientId=client)
            for proxy in root_proxies
        ]
        assert sum(root_proxy_joint_counts) == collision_count
        assert max(root_proxy_joint_counts) <= limit

        receipt = margin_oracle.receipt()
        assert receipt["proxy_source_link_count"] == 2
        assert receipt["proxy_body_count"] == expected_root_proxy_count + 1
        assert receipt["proxy_collision_element_count"] == collision_count + 1
        assert receipt["proxy_chunk_collision_element_limit"] == limit
        assert receipt["proxy_chunk_policy"] == oracle_module.PROXY_CHUNK_POLICY
        assert receipt["source_proxy_isolation_policy"] == (
            "disjoint_collision_filter_group_and_mask"
        )
        assert receipt["chunked_source_link_count"] == 1
        assert receipt["max_proxy_bodies_per_source_link"] == (
            expected_root_proxy_count
        )
        assert receipt["max_collision_elements_per_proxy_body"] == limit
        assert receipt["same_source_chunk_body_pair_count"] == (
            expected_root_proxy_count * (expected_root_proxy_count - 1) // 2
        )
        expected_pair_candidates = sum(
            left_count * right_count
            for left_position, left_count in enumerate(root_proxy_joint_counts)
            for right_count in root_proxy_joint_counts[left_position + 1 :]
        )
        assert receipt["same_source_proxy_link_pair_candidate_count"] == (
            expected_pair_candidates
        )
        assert receipt["same_source_proxy_link_pair_filter_count"] == 1
        assert receipt["same_source_chunk_contact_policy"] == (
            "source_local_aabb_sweep_filter_with_observe_fail_safe"
        )

        last_record = next(
            record
            for record in receipt["margin_calibration_records"]
            if record["source_link_index"] == -1
            and record["collision_element_index"] == collision_count - 1
        )
        assert last_record["proxy_body_chunk_index"] == (
            collision_count - 1
        ) // limit
        assert last_record["proxy_shape_link_index"] == (
            collision_count - 1
        ) % limit

        source_by_proxy = {
            proxy: source_link
            for source_link, proxies in margin_oracle.proxies.items()
            for proxy in proxies
        }
        bullet.performCollisionDetection(physicsClientId=client)
        contacts = bullet.getContactPoints(physicsClientId=client)
        proxy_ids = set(source_by_proxy)
        assert all(
            not (
                (int(contact[1]) == body and int(contact[2]) in proxy_ids)
                or (int(contact[2]) == body and int(contact[1]) in proxy_ids)
            )
            for contact in contacts
        )
        assert not any(
            int(contact[1]) in source_by_proxy
            and int(contact[2]) in source_by_proxy
            and int(contact[1]) != int(contact[2])
            and source_by_proxy[int(contact[1])]
            == source_by_proxy[int(contact[2])]
            == -1
            for contact in contacts
        )

        result = margin_oracle.observe(set())
        assert result["zero_margin_rechecked_link_pair_count"] == 1
        assert result["zero_margin_detected_link_pair_count"] == 1
        assert (
            result["zero_margin_ignored_same_source_chunk_contact_count"] == 0
        )
        assert result["non_adjacent_illegal_penetration_count"] > 0
        assert result["non_adjacent_max_penetration_m"] == pytest.approx(
            0.01, abs=2e-5
        )
        assert margin_oracle.object_bbox_diagonal() > (
            collision_count - 1
        ) * 0.02
    finally:
        if margin_oracle is not None:
            margin_oracle.close()
        bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def _manifest_for_pair(urdf: Path) -> dict:
    row = {
        "asset_id": "fixture/margin-pair",
        "category": "fixture",
        "joint_count": 0,
        "non_fixed_joints": [],
        "source_path": str(urdf.parent),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": urdf.name,
        "primary_urdf_sha256": _sha(urdf),
    }
    manifest = {
        "schema_version": runner.ROSTER_SCHEMA,
        "dataset": "fixture",
        "N_eval": 1,
        "J_eval": 0,
        "rows": [row],
    }
    manifest["manifest_content_sha256"] = runner.canonical_sha256(manifest)
    return manifest


def test_runner_keeps_legacy_shape_and_binds_explicit_v3_oracle(
    tmp_path: Path,
) -> None:
    pytest.importorskip("pybullet")
    urdf = _write_pair(
        tmp_path,
        '<mesh filename="thin_box.obj"/>',
        0.00124,
        write_mesh=True,
    )
    manifest = _manifest_for_pair(urdf)

    legacy_job = runner.build_jobs(manifest, "fixture")[0]
    assert "collision_oracle" not in legacy_job
    assert "zero_margin_oracle_sha256" not in legacy_job
    legacy = runner.evaluate_job(legacy_job)
    assert legacy["schema_version"] == runner.SCHEMA_VERSION
    assert "collision_oracle" not in legacy
    assert "collision_load_flags" not in legacy
    assert "zero_margin_oracle_sha256" not in legacy
    assert legacy["state_records"][0]["schema_version"] == "table4_state_v1"
    assert legacy["max_penetration_m"] == pytest.approx(0.00176, abs=3e-5)

    v3_job = runner.build_jobs(
        manifest,
        "fixture",
        collision_oracle=runner.COLLISION_ORACLE_ZERO_MARGIN,
    )[0]
    assert v3_job["protocol_id"].endswith("_v3")
    assert v3_job["zero_margin_oracle_sha256"] == _sha(
        runner.ZERO_MARGIN_ORACLE_SCRIPT
    )
    assert v3_job["input_identity_sha256"] != legacy_job["input_identity_sha256"]
    v3 = runner.evaluate_job(v3_job)
    assert v3["status"] == "completed"
    assert v3["schema_version"] == runner.SCHEMA_VERSION_V3
    assert v3["collision_oracle"] == runner.COLLISION_ORACLE_ZERO_MARGIN
    assert v3["zero_margin_oracle_sha256"] == v3_job[
        "zero_margin_oracle_sha256"
    ]
    assert not (
        v3["collision_load_flags"]
        & pytest.importorskip("pybullet").URDF_USE_IMPLICIT_CYLINDER
    )
    assert v3["normalization_configuration"] == "expanded_rest"
    assert v3["normalization_collision_geometry"].startswith(
        "per_collision_element_numerical_zero_margin_proxy"
    )
    assert v3["state_records"][0]["schema_version"] == "table4_state_v3"
    assert v3["state_records"][0]["zero_margin_oracle_sha256"] == v3_job[
        "zero_margin_oracle_sha256"
    ]
    assert v3["state_records"][0]["raw_all_pair_max_penetration_m"] == pytest.approx(
        legacy["max_penetration_m"], abs=1e-12
    )
    assert v3["max_penetration_m"] == 0.0

    drifted = dict(v3_job)
    drifted["zero_margin_oracle_sha256"] = "0" * 64
    failed = runner.evaluate_job(drifted)
    assert failed["status"] == "error"
    assert failed["state_records"] == []
    assert "missing or drifted" in failed["issues"][0]

    runner._validate_result_binding(v3, v3_job)
    tampered_cases = {
        "schema": ("schema_version", runner.SCHEMA_VERSION_V2),
        "protocol": ("protocol_id", "stale_v2_protocol"),
        "sampling": ("sampling_protocol", runner.SAMPLING_PROTOCOL_V2),
        "identity": ("input_identity_sha256", "0" * 64),
        "oracle": ("collision_oracle", runner.COLLISION_ORACLE_LEGACY),
        "oracle_sha": ("zero_margin_oracle_sha256", "0" * 64),
        "urdf": ("urdf_path", "/stale/model.urdf"),
        "urdf_sha": ("expected_primary_urdf_sha256", "0" * 64),
        "plan": ("joint_sampling_plan_sha256", "0" * 64),
        "state_hash": ("state_records_sha256", "0" * 64),
    }
    for label, (field, value) in tampered_cases.items():
        tampered = json.loads(json.dumps(v3))
        tampered[field] = value
        with pytest.raises(ValueError, match="mismatch"):
            runner._validate_result_binding(tampered, v3_job)

    stale_state = json.loads(json.dumps(v3))
    stale_state["state_records"][0]["schema_version"] = "table4_state_v2"
    stale_state["state_records_sha256"] = runner.canonical_sha256(
        stale_state["state_records"]
    )
    with pytest.raises(ValueError, match="state 0 binding mismatch"):
        runner._validate_result_binding(stale_state, v3_job)


def test_v3_uses_one_collision_detection_snapshot_per_state(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    bullet = pytest.importorskip("pybullet")
    urdf = _write_pair(
        tmp_path,
        '<mesh filename="thin_box.obj"/>',
        0.00124,
        write_mesh=True,
    )
    job = runner.build_jobs(
        _manifest_for_pair(urdf),
        "fixture",
        collision_oracle=runner.COLLISION_ORACLE_ZERO_MARGIN,
    )[0]
    original = bullet.performCollisionDetection
    calls = 0

    def counted(*args: object, **kwargs: object) -> object:
        nonlocal calls
        calls += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(bullet, "performCollisionDetection", counted)
    result = runner.evaluate_job(job)

    assert result["status"] == "completed"
    assert result["state_records_count"] == 1
    assert calls == result["state_records_count"]
    state = result["state_records"][0]
    assert state["execution_source_hashes_sha256"] == job[
        "execution_source_hashes_sha256"
    ]
    assert "execution_source_hashes" not in state


@pytest.mark.parametrize("drift_read", (2, 3), ids=("start_only", "end_only"))
def test_v3_source_drift_is_a_fatal_nonsealable_result(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, drift_read: int
) -> None:
    pytest.importorskip("pybullet")
    urdf = _write_pair(
        tmp_path,
        '<mesh filename="thin_box.obj"/>',
        0.00124,
        write_mesh=True,
    )
    job = runner.build_jobs(
        _manifest_for_pair(urdf),
        "fixture",
        collision_oracle=runner.COLLISION_ORACLE_ZERO_MARGIN,
    )[0]
    original = runner.sha256_file
    runner_reads = 0

    def drift_once(path: Path) -> str:
        nonlocal runner_reads
        if Path(path) == runner.SCRIPT:
            runner_reads += 1
            # Read 1 is runtime_identity; reads 2 and 3 are the source checks
            # at the start and end of the child, respectively.
            if runner_reads == drift_read:
                return "0" * 64
        return original(path)

    monkeypatch.setattr(runner, "sha256_file", drift_once)
    result = runner.evaluate_job(job)

    assert result["execution_source_integrity"] == "failed"
    assert runner.EXECUTION_SOURCE_INTEGRITY_FATAL in result["issues"][0]
    with pytest.raises(
        ValueError, match=runner.EXECUTION_SOURCE_INTEGRITY_FATAL
    ):
        runner._validate_result_binding(result, job)


def test_build_jobs_reads_zero_margin_oracle_hash_once(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    urdf = _write_pair(
        tmp_path,
        '<mesh filename="thin_box.obj"/>',
        0.00124,
        write_mesh=True,
    )
    manifest = _manifest_for_pair(urdf)
    manifest["rows"] = [
        {**manifest["rows"][0], "asset_id": f"fixture/margin-pair-{index}"}
        for index in range(3)
    ]
    manifest["N_eval"] = 3
    manifest["manifest_content_sha256"] = runner.canonical_sha256(
        {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    )
    real_sha256_file = runner.sha256_file
    reads = 0

    def counted(path: Path) -> str:
        nonlocal reads
        if Path(path) == runner.ZERO_MARGIN_ORACLE_SCRIPT:
            reads += 1
        return real_sha256_file(path)

    monkeypatch.setattr(runner, "sha256_file", counted)
    jobs = runner.build_jobs(
        manifest,
        "fixture",
        collision_oracle=runner.COLLISION_ORACLE_ZERO_MARGIN,
    )
    assert reads == 1
    assert len({job["zero_margin_oracle_sha256"] for job in jobs}) == 1


def test_combined_v3_receipt_uses_frozen_summary_oracle_hash(
    tmp_path: Path,
) -> None:
    frozen = "a" * 64

    def write_summary(path: Path, helper_hash: str) -> None:
        path.mkdir()
        summary = {
            "schema_version": runner.SUMMARY_SCHEMA_V3,
            "dataset": path.name,
            "n_eval": 1,
            "j_eval": 0,
            "status": "COMPLETE",
            "sampling_protocol": runner.SAMPLING_PROTOCOL_V2,
            "collision_oracle": runner.COLLISION_ORACLE_ZERO_MARGIN,
            "zero_margin_oracle_sha256": helper_hash,
            "metrics": {},
            "source_bindings": [],
        }
        summary["summary_content_sha256"] = runner._self_hash(
            summary, "summary_content_sha256"
        )
        (path / "summary.json").write_text(
            json.dumps(summary), encoding="ascii"
        )

    first = tmp_path / "first"
    second = tmp_path / "second"
    write_summary(first, frozen)
    write_summary(second, frozen)
    receipt = runner.build_combined_receipt(
        {"first": first, "second": second}, tmp_path
    )
    assert receipt["protocol"]["zero_margin_oracle_sha256"] == frozen
    assert all(
        method["zero_margin_oracle_sha256"] == frozen
        for method in receipt["methods"]
    )

    inconsistent = tmp_path / "inconsistent"
    write_summary(inconsistent, "b" * 64)
    with pytest.raises(ValueError, match="homogeneous zero-margin oracle"):
        runner.build_combined_receipt(
            {"first": first, "inconsistent": inconsistent}, tmp_path
        )


def test_v3_run_manifest_child_summary_and_resume_share_frozen_oracle_sha(
    tmp_path: Path,
) -> None:
    pytest.importorskip("pybullet")
    urdf = _write_pair(
        tmp_path,
        '<mesh filename="thin_box.obj"/>',
        0.00124,
        write_mesh=True,
    )
    roster = tmp_path / "roster.json"
    roster.write_text(
        json.dumps(_manifest_for_pair(urdf)), encoding="ascii"
    )
    output = tmp_path / "run"
    runner.run_dataset(
        roster,
        output,
        workers=1,
        timeout_seconds=30,
        dataset="fixture",
        collision_oracle=runner.COLLISION_ORACLE_ZERO_MARGIN,
    )

    manifest = json.loads((output / "manifest.json").read_text())
    job = json.loads((output / "child_jobs/000000.json").read_text())
    record = json.loads((output / "records.jsonl").read_text())
    summary = json.loads((output / "summary.json").read_text())
    frozen = manifest["zero_margin_oracle_sha256"]
    assert frozen == _sha(runner.ZERO_MARGIN_ORACLE_SCRIPT)
    assert job["zero_margin_oracle_sha256"] == frozen
    assert record["zero_margin_oracle_sha256"] == frozen
    assert summary["zero_margin_oracle_sha256"] == frozen
    assert manifest["schema_version"] == runner.SCHEMA_VERSION_V3
    assert record["schema_version"] == runner.SCHEMA_VERSION_V3
    assert summary["schema_version"] == runner.SUMMARY_SCHEMA_V3

    # A stale compact row cannot be mixed into v3 on resume.  The durable v3
    # state stream and stripped child receipt are insufficient to bless it, so
    # the frozen job is rerun and rewrites a coherent v3 record.
    record["schema_version"] = runner.SCHEMA_VERSION_V2
    (output / "records.jsonl").write_text(
        json.dumps(record) + "\n", encoding="ascii"
    )
    runner.run_dataset(
        roster,
        output,
        workers=1,
        timeout_seconds=30,
        resume=True,
        dataset="fixture",
        collision_oracle=runner.COLLISION_ORACLE_ZERO_MARGIN,
    )
    resumed = json.loads((output / "records.jsonl").read_text())
    assert resumed["schema_version"] == runner.SCHEMA_VERSION_V3
    assert resumed["zero_margin_oracle_sha256"] == frozen


def test_manifest_json_round_trip_helper(tmp_path: Path) -> None:
    """Guard the focused fixture against non-JSON identity values."""

    urdf = _write_pair(
        tmp_path,
        '<mesh filename="thin_box.obj"/>',
        0.00124,
        write_mesh=True,
    )
    assert json.loads(json.dumps(_manifest_for_pair(urdf)))["N_eval"] == 1
