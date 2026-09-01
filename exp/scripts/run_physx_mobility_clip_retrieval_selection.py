#!/usr/bin/env python3
"""Select global top-1 PhysX-Mobility assets from a locked render-only snapshot."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any

os.environ["HF_HUB_OFFLINE"] = "1"
os.environ["TRANSFORMERS_OFFLINE"] = "1"
os.environ["TOKENIZERS_PARALLELISM"] = "false"
os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"

import numpy as np
from PIL import Image
import torch
import transformers
from transformers import CLIPModel, CLIPProcessor


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = Path("/mnt/zsn/lyb").resolve()
DEFAULT_SNAPSHOT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1_snapshot"
DEFAULT_REPLAY_SNAPSHOT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1_snapshot_replay"
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/physx_mobility_clip_retrieval_v1"
DEFAULT_AMENDMENT = REPO / "exp/reference/table4_constraints_v2/amendment_physx_mobility_clip_retrieval_v1.json"
DEFAULT_PROMPTS = REPO / "exp/reference/table4_constraints_v2/prompts.jsonl"
MODEL_REVISION = "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
VIEW_COUNT = 8
TASK_COUNT = 18
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_MODEL_FILES = {
    "config.json": "b575ef3c36f2a057fa19e221650105052d61cc9c1a972ec15019c6261ec98770",
    "merges.txt": "f526393189112391ce6f9795d4695f704121ce452c3aad1f5335cc41337eba85",
    "preprocessor_config.json": "910e70b3956ac9879ebc90b22fb3bc8a75b6a0677814500101a4c072bd7857bd",
    "pytorch_model.bin": "a63082132ba4f97a80bea76823f544493bffa8082296d62d71581a4feff1576f",
    "special_tokens_map.json": "f8c0d6c39aee3f8431078ef6646567b0aba7f2246e9c54b8b99d55c22b707cbf",
    "tokenizer.json": "b556ac8c99757ffb677208af34bc8c6721572114111a6e0aaf5fa69ff0b8d842",
    "tokenizer_config.json": "34b7336e4bee12e0a9730eaf5189f582ef3c3eea5027f65730e5717256755aad",
    "vocab.json": "5047b556ce86ccaf6aa22b3ffccfc52d391ea4accdab9c2f2407da5b742d4363",
}
EXPECTED_RUNTIME = {
    "python": "3.13.2",
    "numpy": "2.4.4",
    "torch": "2.6.0+cu124",
    "transformers": "5.12.0",
    "pillow": "12.3.0",
}
EXPECTED_THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
EXPECTED_DEVICE = "cuda:0"
EXPECTED_BATCH_ASSETS = 8
EXPECTED_CUDA_VISIBLE_DEVICES = "1"
EXPECTED_GPU = {
    "physical_index": "1",
    "uuid": "GPU-7390aec1-d177-6672-4136-d998c85f489d",
    "name": "NVIDIA L20X",
    "driver_version": "570.172.08",
}


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"selection path outside isolated workspace: {resolved}")
    return resolved


def regular_file(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink file: {path}")
    return resolved


def regular_dir(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISDIR(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink directory: {path}")
    return resolved


def sha256_file(path: Path, block_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with regular_file(path).open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(regular_file(path).read_text())


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in regular_file(path).read_text().splitlines() if line.strip()]


def write_json(path: Path, value: Any) -> None:
    destination = safe(path, must_exist=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n")
    temporary.replace(destination)


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    destination = safe(path, must_exist=False)
    payload = "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(payload)
    temporary.replace(destination)


def normalize(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm <= 0:
        raise RuntimeError("invalid zero/nonfinite CLIP feature")
    return vector / norm


def image_projection(model: CLIPModel, pixel_values: torch.Tensor) -> torch.Tensor:
    return model.visual_projection(model.vision_model(pixel_values=pixel_values).pooler_output)


def text_projection(model: CLIPModel, input_ids: torch.Tensor, attention_mask: torch.Tensor) -> torch.Tensor:
    return model.text_projection(model.text_model(input_ids=input_ids, attention_mask=attention_mask).pooler_output)


def processor_assertions(processor: CLIPProcessor, model: CLIPModel) -> dict[str, Any]:
    image_processor = processor.image_processor
    tokenizer = processor.tokenizer
    observed = {
        "resize_shortest_edge": image_processor.size["shortest_edge"],
        "crop_height": image_processor.crop_size["height"],
        "crop_width": image_processor.crop_size["width"],
        "resample": int(image_processor.resample),
        "image_mean": list(image_processor.image_mean),
        "image_std": list(image_processor.image_std),
        "tokenizer_model_max_length": tokenizer.model_max_length,
        "tokenizer_padding_side": tokenizer.padding_side,
        "tokenizer_truncation_side": tokenizer.truncation_side,
        "projection_dim": model.config.projection_dim,
        "vision_image_size": model.config.vision_config.image_size,
        "vision_patch_size": model.config.vision_config.patch_size,
        "text_max_position_embeddings": model.config.text_config.max_position_embeddings,
    }
    expected = {
        "resize_shortest_edge": 224,
        "crop_height": 224,
        "crop_width": 224,
        "resample": 3,
        "image_mean": [0.48145466, 0.4578275, 0.40821073],
        "image_std": [0.26862954, 0.26130258, 0.27577711],
        "tokenizer_model_max_length": 77,
        "tokenizer_padding_side": "right",
        "tokenizer_truncation_side": "right",
        "projection_dim": 512,
        "vision_image_size": 224,
        "vision_patch_size": 32,
        "text_max_position_embeddings": 77,
    }
    if observed != expected:
        raise RuntimeError(f"CLIP processor/model configuration drifted: {observed!r}")
    return observed


def runtime_gate() -> dict[str, Any]:
    import PIL
    observed = {
        "python": sys.version.split()[0],
        "numpy": np.__version__,
        "torch": torch.__version__,
        "transformers": transformers.__version__,
        "pillow": PIL.__version__,
    }
    if observed != EXPECTED_RUNTIME:
        raise RuntimeError(f"selection runtime drifted: {observed!r}")
    observed_threads = {key: os.environ.get(key) for key in EXPECTED_THREAD_ENV}
    if observed_threads != EXPECTED_THREAD_ENV:
        raise RuntimeError(f"selection thread environment drifted: {observed_threads!r}")
    if os.environ.get("CUDA_VISIBLE_DEVICES") != EXPECTED_CUDA_VISIBLE_DEVICES:
        raise RuntimeError(f"formal selection requires CUDA_VISIBLE_DEVICES={EXPECTED_CUDA_VISIBLE_DEVICES}")
    query = subprocess.run(
        ["nvidia-smi", "--query-gpu=index,uuid,name,driver_version", "--format=csv,noheader,nounits", "-i", EXPECTED_GPU["physical_index"]],
        text=True, capture_output=True, check=True,
    ).stdout.strip().split(", ")
    physical = dict(zip(("physical_index", "uuid", "name", "driver_version"), query))
    if physical != EXPECTED_GPU:
        raise RuntimeError(f"formal GPU identity drifted: {physical!r}")
    properties = torch.cuda.get_device_properties(0)
    cuda = {
        "cuda_visible_devices": os.environ["CUDA_VISIBLE_DEVICES"],
        "logical_device": 0,
        "visible_device_count": torch.cuda.device_count(),
        "torch_cuda": torch.version.cuda,
        "cudnn": torch.backends.cudnn.version(),
        "name": properties.name,
        "total_memory_bytes": properties.total_memory,
        "capability": [properties.major, properties.minor],
        "physical": physical,
    }
    expected_cuda = {
        "cuda_visible_devices": "1",
        "logical_device": 0,
        "visible_device_count": 1,
        "torch_cuda": "12.4",
        "cudnn": 90100,
        "name": "NVIDIA L20X",
        "total_memory_bytes": 150121021440,
        "capability": [9, 0],
        "physical": EXPECTED_GPU,
    }
    if cuda != expected_cuda:
        raise RuntimeError(f"formal CUDA runtime drifted: {cuda!r}")
    return {"libraries": observed, "threads": observed_threads, "cuda": cuda}


def validate_snapshot(snapshot: Path, amendment: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    snapshot = regular_dir(snapshot)
    expected_root = {"candidate_inventory.jsonl", "candidate_summary.json", "ineligible.jsonl", "model", "renders", "snapshot.lock.json"}
    if {row.name for row in snapshot.iterdir()} != expected_root:
        raise RuntimeError("snapshot root file closure drifted")
    amendment_data = read_json(amendment)
    if amendment_data.get("status") != "pre_result_frozen":
        raise RuntimeError("amendment is not frozen")
    if amendment_data["implementation"]["selector_sha256"] != sha256_file(SCRIPT):
        raise RuntimeError("selector hash differs from amendment")
    lock = read_json(snapshot / "snapshot.lock.json")
    if lock.get("status") != "PASS" or lock.get("snapshot_type") != "render_only_identity_snapshot":
        raise RuntimeError("snapshot lock is not PASS")
    if lock.get("amendment_sha256") != sha256_file(amendment):
        raise RuntimeError("snapshot is not bound to this amendment")
    if lock.get("snapshot_builder_sha256") != amendment_data["implementation"]["snapshot_builder_sha256"]:
        raise RuntimeError("snapshot builder hash differs from amendment")
    inventory_path = snapshot / "candidate_inventory.jsonl"
    if lock.get("candidate_inventory_sha256") != sha256_file(inventory_path):
        raise RuntimeError("candidate inventory hash drifted")
    candidates = read_jsonl(inventory_path)
    if len(candidates) != lock.get("eligible_assets") or not candidates:
        raise RuntimeError("candidate inventory cardinality drifted")
    expected_candidate_fields = {"status", "identity", "identity_sha256", "views"}
    expected_view_fields = {"name", "bytes", "sha256"}
    for candidate in candidates:
        if set(candidate) != expected_candidate_fields or candidate["status"] != "eligible":
            raise RuntimeError(f"candidate schema drifted: {candidate.get('identity')}")
        if len(candidate["views"]) != VIEW_COUNT or any(set(view) != expected_view_fields for view in candidate["views"]):
            raise RuntimeError(f"candidate view schema drifted: {candidate['identity']}")
    render_root = regular_dir(snapshot / "renders")
    if {row.name for row in render_root.iterdir() if row.is_dir() and not row.is_symlink()} != {row["identity"] for row in candidates}:
        raise RuntimeError("render identity directory closure drifted")
    model_dir = regular_dir(snapshot / "model")
    observed_model = {row.name: sha256_file(row) for row in model_dir.iterdir() if row.is_file() and not row.is_symlink()}
    if observed_model != EXPECTED_MODEL_FILES or lock.get("model_files_sha256") != EXPECTED_MODEL_FILES:
        raise RuntimeError("CLIP model closure drifted")
    return lock, candidates


def validate(
    snapshot: Path,
    replay_snapshot: Path,
    output: Path,
    amendment: Path,
    prompts_path: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    lock, candidates = validate_snapshot(snapshot, amendment)
    replay_lock, replay_candidates = validate_snapshot(replay_snapshot, amendment)
    if candidates != replay_candidates:
        raise RuntimeError("fresh replay snapshot candidate render manifests differ")
    if lock.get("runtime_fingerprint") != replay_lock.get("runtime_fingerprint"):
        raise RuntimeError("fresh replay render runtime fingerprints differ")
    prompts_path = regular_file(prompts_path)
    if sha256_file(prompts_path) != EXPECTED_PROMPTS_SHA256:
        raise RuntimeError("frozen prompt manifest hash drifted")
    prompts = read_jsonl(prompts_path)
    if len(prompts) != TASK_COUNT:
        raise RuntimeError("prompt count is not 18")
    output = safe(output, must_exist=False)
    if not output.is_dir() or output.is_symlink():
        raise RuntimeError("formal output must be pre-created by independent auditor")
    allowed = {"protocol_audit_pre_result.json", "report.md"}
    if {row.name for row in output.iterdir()} != allowed:
        raise RuntimeError("formal output contains data beyond independent pre-result audit")
    audit = read_json(output / "protocol_audit_pre_result.json")
    if str(audit.get("verdict", "")).upper() != "PASS" or audit.get("protocol_ready") is not True:
        raise RuntimeError("independent audit did not PASS")
    if audit.get("amendment_sha256") != sha256_file(amendment):
        raise RuntimeError("independent audit amendment binding mismatch")
    return lock, candidates, replay_lock, replay_candidates, prompts


def one_embedding_run(
    snapshot: Path,
    candidates: list[dict[str, Any]],
    prompts: list[dict[str, Any]],
    device: torch.device,
    batch_assets: int,
) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    model_dir = regular_dir(snapshot / "model")
    processor = CLIPProcessor.from_pretrained(model_dir, local_files_only=True)
    model = CLIPModel.from_pretrained(model_dir, local_files_only=True).to(device=device, dtype=torch.float32).eval()
    processor_config = processor_assertions(processor, model)
    asset_features = []
    for start in range(0, len(candidates), batch_assets):
        batch = candidates[start : start + batch_assets]
        images = []
        try:
            for row in batch:
                view_names = [item["name"] for item in row["views"]]
                if view_names != [f"{index:03d}.png" for index in range(VIEW_COUNT)]:
                    raise RuntimeError(f"view closure drifted for {row['identity']}")
                render_dir = regular_dir(snapshot / "renders" / row["identity"])
                if {item.name for item in render_dir.iterdir()} != set(view_names):
                    raise RuntimeError(f"render file closure drifted for {row['identity']}")
                for item in row["views"]:
                    path = regular_file(render_dir / item["name"])
                    if path.stat().st_size != item["bytes"] or sha256_file(path) != item["sha256"]:
                        raise RuntimeError(f"render hash drifted: {row['identity']}/{item['name']}")
                    with Image.open(path) as image:
                        rgb = image.convert("RGB")
                        rgb.load()
                    images.append(rgb)
            inputs = processor(images=images, return_tensors="pt")
            with torch.inference_mode():
                features = image_projection(model, inputs["pixel_values"].to(device, dtype=torch.float32))
            features = features.detach().cpu().numpy().astype(np.float64).reshape(len(batch), VIEW_COUNT, -1)
        finally:
            for image in images:
                image.close()
        for view_features in features:
            normalized_views = np.stack([normalize(vector) for vector in view_features])
            asset_features.append(normalize(normalized_views.mean(axis=0)))
        print(f"[embed] {min(start + len(batch), len(candidates))}/{len(candidates)}", flush=True)
    asset_matrix = np.stack(asset_features)
    exact_prompts = [str(row["prompt"]) for row in prompts]
    text_inputs = processor.tokenizer(exact_prompts, padding=True, truncation=True, max_length=77, return_tensors="pt")
    with torch.inference_mode():
        text_features = text_projection(
            model,
            text_inputs["input_ids"].to(device),
            text_inputs["attention_mask"].to(device),
        )
    text_matrix = np.stack([normalize(vector) for vector in text_features.detach().cpu().numpy().astype(np.float64)])
    token_rows = [
        {
            "task_id": row["task_id"],
            "input_ids": text_inputs["input_ids"][index].tolist(),
            "attention_mask": text_inputs["attention_mask"][index].tolist(),
        }
        for index, row in enumerate(prompts)
    ]
    del model, processor
    torch.cuda.empty_cache()
    index_rows = [
        {"row": index, "identity": row["identity"], "identity_sha256": row["identity_sha256"]}
        for index, row in enumerate(candidates)
    ]
    return asset_matrix, text_matrix, index_rows, token_rows, processor_config


def select(prompts: list[dict[str, Any]], candidates: list[dict[str, Any]], asset_matrix: np.ndarray, text_matrix: np.ndarray) -> list[dict[str, Any]]:
    scores = text_matrix @ asset_matrix.T
    if scores.shape != (TASK_COUNT, len(candidates)) or not np.isfinite(scores).all():
        raise RuntimeError("invalid CLIP similarity matrix")
    selection = []
    for prompt_index, prompt in enumerate(prompts):
        ranked = sorted(
            range(len(candidates)),
            key=lambda index: (-float(scores[prompt_index, index]), candidates[index]["identity_sha256"], candidates[index]["identity"]),
        )
        winner = candidates[ranked[0]]
        selection.append({
            "task_id": prompt["task_id"],
            "identity": winner["identity"],
            "identity_sha256": winner["identity_sha256"],
            "clip_score_float64": float(scores[prompt_index, ranked[0]]),
            "rank": 1,
            "fallback_allowed": False,
        })
    return selection


def save_npy(path: Path, value: np.ndarray) -> None:
    destination = safe(path, must_exist=False)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    with temporary.open("wb") as stream:
        np.save(stream, value, allow_pickle=False)
    temporary.replace(destination)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--replay-snapshot", type=Path, default=DEFAULT_REPLAY_SNAPSHOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--amendment", type=Path, default=DEFAULT_AMENDMENT)
    parser.add_argument("--prompts", type=Path, default=DEFAULT_PROMPTS)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-assets", type=int, default=8)
    args = parser.parse_args()
    lock, candidates, replay_lock, replay_candidates, prompts = validate(
        args.snapshot, args.replay_snapshot, args.output, args.amendment, args.prompts
    )
    if args.batch_assets != EXPECTED_BATCH_ASSETS or args.device != EXPECTED_DEVICE:
        raise RuntimeError(f"formal selection requires --batch-assets {EXPECTED_BATCH_ASSETS} --device {EXPECTED_DEVICE}")
    random.seed(0)
    np.random.seed(0)
    torch.manual_seed(0)
    torch.cuda.manual_seed_all(0)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.use_deterministic_algorithms(True)
    device = torch.device(args.device)
    if device.type != "cuda" or not torch.cuda.is_available():
        raise RuntimeError("formal selection requires CUDA")
    runtime = runtime_gate()
    run1 = one_embedding_run(args.snapshot, candidates, prompts, device, args.batch_assets)
    run2 = one_embedding_run(args.replay_snapshot, replay_candidates, prompts, device, args.batch_assets)
    asset_matrix, text_matrix, index_rows, token_rows, processor_config = run1
    asset_matrix2, text_matrix2, index_rows2, token_rows2, processor_config2 = run2
    selection = select(prompts, candidates, asset_matrix, text_matrix)
    selection2 = select(prompts, candidates, asset_matrix2, text_matrix2)
    if not np.array_equal(asset_matrix, asset_matrix2) or not np.array_equal(text_matrix, text_matrix2):
        raise RuntimeError("double embedding replay is not bitwise identical")
    if index_rows != index_rows2 or token_rows != token_rows2 or processor_config != processor_config2 or selection != selection2:
        raise RuntimeError("double selection replay is not byte-equivalent")
    output = safe(args.output)
    staging = safe(output / ".selection_staging", must_exist=False)
    if staging.exists():
        raise RuntimeError("selection staging already exists; never resume or overwrite")
    staging.mkdir()
    write_jsonl(staging / "selection.jsonl", selection)
    write_jsonl(staging / "selection_replay.jsonl", selection2)
    save_npy(staging / "asset_embeddings_run1.npy", asset_matrix)
    save_npy(staging / "asset_embeddings_run2.npy", asset_matrix2)
    save_npy(staging / "prompt_embeddings_run1.npy", text_matrix)
    save_npy(staging / "prompt_embeddings_run2.npy", text_matrix2)
    write_jsonl(staging / "embedding_index.jsonl", index_rows)
    write_jsonl(staging / "prompt_tokenization.jsonl", token_rows)
    selection_lock = {
        "status": "PASS",
        "method": "physx_mobility_geometry_clip_retrieval_v1",
        "is_generation_method": False,
        "dataset_reference_only": True,
        "selection_rows": len(selection),
        "candidate_assets": len(candidates),
        "model_id": "openai/clip-vit-base-patch32",
        "model_revision": MODEL_REVISION,
        "device": str(device),
        "batch_assets": args.batch_assets,
        "view_count": VIEW_COUNT,
        "view_aggregation": "normalize each view, arithmetic mean, normalize mean",
        "dtype": "float32_model_float64_scoring",
        "global_top1": True,
        "category_filter": False,
        "tie_rule": "descending exact float64 score, then ascending identity_sha256, then ascending identity",
        "geometry_or_metadata_read_during_selection": False,
        "is_os_sandbox": False,
        "runtime": runtime,
        "processor_config": processor_config,
        "double_embedding_replay_bitwise_identical": True,
        "double_selection_replay_identical": True,
        "snapshot_lock_sha256": sha256_file(args.snapshot / "snapshot.lock.json"),
        "replay_snapshot_lock_sha256": sha256_file(args.replay_snapshot / "snapshot.lock.json"),
        "prompt_manifest_sha256": sha256_file(args.prompts),
        "selection_sha256": sha256_file(staging / "selection.jsonl"),
        "selection_replay_sha256": sha256_file(staging / "selection_replay.jsonl"),
        "asset_embeddings_run1_sha256": sha256_file(staging / "asset_embeddings_run1.npy"),
        "asset_embeddings_run2_sha256": sha256_file(staging / "asset_embeddings_run2.npy"),
        "prompt_embeddings_run1_sha256": sha256_file(staging / "prompt_embeddings_run1.npy"),
        "prompt_embeddings_run2_sha256": sha256_file(staging / "prompt_embeddings_run2.npy"),
        "embedding_index_sha256": sha256_file(staging / "embedding_index.jsonl"),
        "prompt_tokenization_sha256": sha256_file(staging / "prompt_tokenization.jsonl"),
        "selector_sha256": sha256_file(SCRIPT),
        "amendment_sha256": sha256_file(args.amendment),
    }
    write_json(staging / "selection.lock.json", selection_lock)
    publish = sorted(path for path in staging.iterdir() if path.name != "selection.lock.json")
    for path in publish:
        path.replace(output / path.name)
    (staging / "selection.lock.json").replace(output / "selection.lock.json")
    staging.rmdir()
    print(json.dumps({"status": "PASS", "selected": len(selection), "candidates": len(candidates)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
