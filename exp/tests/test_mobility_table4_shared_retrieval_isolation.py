from __future__ import annotations

import ast
from pathlib import Path


REPO = Path(__file__).resolve().parents[2]
SCRIPTS = REPO / "exp/scripts"
SELECTOR = SCRIPTS / "run_partnet_mobility_clip_retrieval_selection.py"
LAUNCHER = SCRIPTS / "launch_mobility_table4_clip_retrieval_selection.py"
LOCKER = SCRIPTS / "lock_partnet_mobility_table4_render_snapshot.py"
WORKER = SCRIPTS / "render_partnet_mobility_table4_opaque.py"

CONTRACT_FIELDS = {
    "schema_version",
    "contract_id",
    "expected_assets",
    "views",
    "batch_assets",
    "task_count",
    "source_prompt_manifest_sha256",
    "prompt_only_manifest_sha256",
    "protocol_sha256",
    "model_id",
    "model_revision",
    "implementation",
    "selector_runtime",
    "gpu",
    "thread_environment",
}


def source(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def assigned_set(path: Path, variable: str) -> set[str]:
    tree = ast.parse(source(path))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == variable for target in node.targets):
            return set(ast.literal_eval(node.value))
    raise AssertionError(f"{variable} not found in {path}")


def test_selector_has_no_semantic_source_inputs() -> None:
    text = source(SELECTOR)
    forbidden = ("prompts.jsonl", "protocol.json", "amendment", "source_asset", "category", "difficulty", "spec_sha256")
    assert all(token not in text for token in forbidden)
    assert 'snapshot / "prompt_only.jsonl"' in text
    assert 'set(row) != {"task_id", "prompt"}' in text


def test_launcher_and_selector_share_exact_nonsemantic_contract_schema() -> None:
    assert assigned_set(SELECTOR, "exact_fields") == CONTRACT_FIELDS
    assert assigned_set(LAUNCHER, "CONTRACT_FIELDS") == CONTRACT_FIELDS
    assert "arm_key" not in CONTRACT_FIELDS
    assert "method" not in CONTRACT_FIELDS
    for path in (SELECTOR, LAUNCHER):
        text = source(path)
        assert 'set(contract["implementation"]) != {"selection_launcher_sha256", "selector_sha256"}' in text


def test_selector_replay_and_atomic_publication_are_fail_closed() -> None:
    text = source(SELECTOR)
    assert text.count("CLIPModel.from_pretrained") == 1
    assert 'prefix=output.name + ".staging."' in text
    assert "dir=str(output.parent.parent)" in text
    assert "shutil.rmtree(staging, ignore_errors=True)" in text
    lock_write = text.index('write_json(staging / "selection.lock.json"')
    directory_publish = text.index("staging.replace(output)")
    assert lock_write < directory_publish
    assert "selection output exists; refusing overwrite" in text
    assert "tempfile.mkdtemp" in text
    assert '"PYTHONHASHSEED": "0"' in text


def test_builder_revalidates_private_scene_and_blob_exact_closure_before_render() -> None:
    text = source(SCRIPTS / "build_partnet_mobility_table4_render_snapshot.py")
    call = "validate_private_source_closure(private_source, plan, source_lock)"
    assert call in text
    assert text.index(call) < text.index("output.mkdir(parents=True, exist_ok=False)")
    assert "geometry scene exact closure mismatch" in text
    assert "geometry blob exact closure mismatch" in text


def test_snapshot_contract_and_prompt_only_closure_match_selector() -> None:
    selector_text = source(SELECTOR)
    locker_text = source(LOCKER)
    closure = '["candidate_inventory.jsonl", "model", "prompt_only.jsonl", "renders", "selection.execution_contract.json", "snapshot.lock.json"]'
    assert closure in selector_text
    assert closure in locker_text
    assert '"selection_execution_contract_sha256"' in locker_text
    assert '"prompt_only_manifest_sha256"' in locker_text
    for text in (selector_text, locker_text):
        assert "len(key) != 64" in text
        assert 'any(char not in "0123456789abcdef" for char in key)' in text
        assert "duplicate opaque candidate key" in text
        assert "render directory exact closure" in text


def test_renderer_is_geometry_rooted_nonblank_and_camera_only() -> None:
    text = source(WORKER)
    assert 'private_root / "geometry_scenes"' in text
    assert "path.parent != geometry_scene_root" in text
    assert "doubleSided=True" in text
    assert "foreground_pixels < 32" in text
    assert "source_pose = source_camera_pose" in text
    assert "mesh.apply_transform(camera_frame" not in text
    assert "assert_paired_upright_camera_invariant()" in text
