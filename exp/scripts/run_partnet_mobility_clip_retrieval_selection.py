#!/usr/bin/env python3
"""Run the shared audited CLIP top-1 selector on an opaque mobility snapshot."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import shutil
import stat
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

os.environ["HF_HUB_OFFLINE"] = "1"
os.environ["TRANSFORMERS_OFFLINE"] = "1"
os.environ["TOKENIZERS_PARALLELISM"] = "false"
os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
if os.environ.get("PYTHONHASHSEED") != "0":
    raise RuntimeError("formal selector requires PYTHONHASHSEED=0 before process start")

import numpy as np
import PIL
from PIL import Image
import torch
import transformers
from transformers import CLIPModel, CLIPProcessor


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
DEFAULT_SNAPSHOT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1_snapshot"
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/partnet_mobility_clip_retrieval_v1/selection_bundle"
MODEL_REVISION = "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_PROTOCOL_SHA256 = "6857194072ccb0ba3943d14a62f29c2364920ec01381a5475311535c1831031f"
VIEWS = 8
DIMENSION = 512
BATCH_ASSETS = 8
CONTRACT_NAME = "selection.execution_contract.json"
SNAPSHOT_CLOSURE = ["candidate_inventory.jsonl", "model", "prompt_only.jsonl", "renders", "selection.execution_contract.json", "snapshot.lock.json"]
EXPECTED_RUNTIME = {
    "python": "3.13.2",
    "numpy": "2.4.4",
    "torch": "2.6.0+cu124",
    "transformers": "5.12.0",
    "pillow": "12.3.0",
}
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
EXPECTED_GPU = {
    "physical_device": 1,
    "logical_device": 0,
    "name": "NVIDIA L20X",
    "uuid": "GPU-7390aec1-d177-6672-4136-d998c85f489d",
    "driver_version": "570.172.08",
    "total_memory_bytes": 150121021440,
    "compute_capability": [9, 0],
}


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"outside authorized workspace: {resolved}")
    return resolved


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in safe(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def atomic_text(path: Path, value: str) -> None:
    destination = safe(path, must_exist=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def write_json(path: Path, value: Any) -> None:
    atomic_text(path, json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n")


def jsonl_bytes(rows: list[dict[str, Any]]) -> bytes:
    return "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows).encode("utf-8")


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    atomic_text(path, jsonl_bytes(rows).decode("utf-8"))


def require_regular_file(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink file: {path}")
    return resolved


def require_regular_dir(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISDIR(path.lstat().st_mode):
        raise RuntimeError(f"not a regular non-symlink directory: {path}")
    return resolved


def validate_contract(snapshot: Path) -> tuple[dict[str, Any], str]:
    contract_path = require_regular_file(snapshot / CONTRACT_NAME)
    contract_sha256 = sha256_file(contract_path)
    contract = read_json(contract_path)
    exact_fields = {
        "schema_version", "contract_id", "expected_assets", "views", "batch_assets", "task_count",
        "source_prompt_manifest_sha256", "prompt_only_manifest_sha256", "protocol_sha256", "model_id", "model_revision",
        "implementation", "selector_runtime", "gpu", "thread_environment",
    }
    if set(contract) != exact_fields or contract.get("schema_version") != 1:
        raise RuntimeError("selection execution contract schema drifted")
    if contract.get("views") != VIEWS or contract.get("batch_assets") != BATCH_ASSETS or contract.get("task_count") != 18:
        raise RuntimeError("selection execution contract cardinality drifted")
    if not isinstance(contract.get("contract_id"), str) or len(contract["contract_id"]) != 64 or any(char not in "0123456789abcdef" for char in contract["contract_id"]):
        raise RuntimeError("selection execution contract identity drifted")
    if not isinstance(contract.get("expected_assets"), int) or contract["expected_assets"] <= 0:
        raise RuntimeError("selection execution contract candidate cardinality drifted")
    if contract.get("source_prompt_manifest_sha256") != EXPECTED_PROMPTS_SHA256 or contract.get("protocol_sha256") != EXPECTED_PROTOCOL_SHA256:
        raise RuntimeError("selection execution contract benchmark hashes drifted")
    if contract.get("model_id") != "openai/clip-vit-base-patch32" or contract.get("model_revision") != MODEL_REVISION:
        raise RuntimeError("selection execution contract model identity drifted")
    if contract.get("selector_runtime") != EXPECTED_RUNTIME or contract.get("gpu") != EXPECTED_GPU:
        raise RuntimeError("selection execution contract runtime/GPU drifted")
    if set(contract["implementation"]) != {"selection_launcher_sha256", "selector_sha256"} or contract["implementation"].get("selector_sha256") != sha256_file(SCRIPT):
        raise RuntimeError("selection execution contract selector hash drifted")
    expected_threads = {
        "HF_HUB_OFFLINE": "1", "TRANSFORMERS_OFFLINE": "1", "TOKENIZERS_PARALLELISM": "false",
        "CUBLAS_WORKSPACE_CONFIG": ":4096:8", "CUDA_DEVICE_ORDER": "PCI_BUS_ID",
        "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1",
        "PYTHONHASHSEED": "0",
    }
    if contract.get("thread_environment") != expected_threads or any(os.environ.get(key) != value for key, value in expected_threads.items()):
        raise RuntimeError("selection execution contract thread environment drifted")
    return contract, contract_sha256


def validate_snapshot(snapshot: Path, contract: dict[str, Any], contract_sha256: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    snapshot = require_regular_dir(snapshot)
    if {entry.name for entry in os.scandir(snapshot)} != set(SNAPSHOT_CLOSURE):
        raise RuntimeError("selector-visible snapshot exact closure drifted")
    lock = read_json(require_regular_file(snapshot / "snapshot.lock.json"))
    expected_assets = contract["expected_assets"]
    if lock.get("status") != "PASS" or lock.get("candidate_assets") != expected_assets or lock.get("eligible_assets") != expected_assets:
        raise RuntimeError("opaque snapshot is not full PASS")
    if lock.get("selector_visible_closure") != SNAPSHOT_CLOSURE:
        raise RuntimeError("selector-visible closure declaration drifted")
    if lock.get("selection_execution_contract_sha256") != contract_sha256:
        raise RuntimeError("opaque snapshot execution-contract binding drifted")
    if lock.get("prompt_only_manifest_sha256") != contract.get("prompt_only_manifest_sha256") or sha256_file(snapshot / "prompt_only.jsonl") != contract.get("prompt_only_manifest_sha256"):
        raise RuntimeError("opaque snapshot prompt-only binding drifted")
    inventory_path = require_regular_file(snapshot / "candidate_inventory.jsonl")
    if sha256_file(inventory_path) != lock.get("candidate_inventory_sha256"):
        raise RuntimeError("candidate inventory hash drifted")
    inventory = read_jsonl(inventory_path)
    allowed = {"candidate_key", "identity_sha256", "snapshot_render_dir", "eligible", "views", "render_sha256"}
    if len(inventory) != expected_assets or inventory != sorted(inventory, key=lambda row: row["candidate_key"]):
        raise RuntimeError("candidate inventory count/order drifted")
    renders = require_regular_dir(snapshot / "renders")
    keys: set[str] = set()
    for row in inventory:
        if set(row) != allowed or row["candidate_key"] != row["identity_sha256"] or row["views"] != VIEWS or row["eligible"] is not True:
            raise RuntimeError("candidate inventory schema/identity drifted")
        key = row["candidate_key"]
        if not isinstance(key, str) or len(key) != 64 or any(char not in "0123456789abcdef" for char in key):
            raise RuntimeError("invalid opaque candidate key")
        if key in keys:
            raise RuntimeError(f"duplicate opaque candidate key: {key}")
        keys.add(key)
        expected_relative = f"renders/{row['candidate_key']}"
        expected_views = {f"{index:03d}.png" for index in range(VIEWS)}
        if row["snapshot_render_dir"] != expected_relative or set(row["render_sha256"]) != expected_views:
            raise RuntimeError("candidate render path/hash-key closure drifted")
        directory = require_regular_dir(snapshot / row["snapshot_render_dir"])
        names = sorted(item.name for item in directory.iterdir())
        if names != [f"{index:03d}.png" for index in range(VIEWS)]:
            raise RuntimeError(f"candidate render closure drifted: {row['candidate_key']}")
        for name, expected in row["render_sha256"].items():
            if sha256_file(require_regular_file(directory / name)) != expected:
                raise RuntimeError(f"candidate render hash drifted: {row['candidate_key']}/{name}")
    observed_render_dirs: set[str] = set()
    for entry in os.scandir(renders):
        path = Path(entry.path)
        if entry.is_symlink() or not entry.is_dir(follow_symlinks=False):
            raise RuntimeError(f"render root contains non-directory entry: {path}")
        observed_render_dirs.add(entry.name)
    if observed_render_dirs != keys:
        raise RuntimeError(f"render directory exact closure mismatch missing={sorted(keys - observed_render_dirs)} extra={sorted(observed_render_dirs - keys)}")
    model = require_regular_dir(snapshot / "model")
    if {entry.name for entry in os.scandir(model)} != set(EXPECTED_MODEL_FILES):
        raise RuntimeError("CLIP model snapshot exact closure drifted")
    for name, expected in EXPECTED_MODEL_FILES.items():
        if sha256_file(require_regular_file(model / name)) != expected or lock.get("model_file_sha256", {}).get(name) != expected:
            raise RuntimeError(f"CLIP model file drifted: {name}")
    return lock, inventory


def configure_determinism() -> None:
    random.seed(0)
    np.random.seed(0)
    torch.manual_seed(0)
    torch.cuda.manual_seed_all(0)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.use_deterministic_algorithms(True)


def normalized(values: np.ndarray, axis: int) -> np.ndarray:
    norms = np.linalg.norm(values, axis=axis, keepdims=True)
    if not np.isfinite(norms).all() or np.any(norms <= 0):
        raise RuntimeError("invalid embedding norm")
    return values / norms


def load_model(snapshot: Path) -> tuple[CLIPModel, CLIPProcessor]:
    configure_determinism()
    model_path = str(require_regular_dir(snapshot / "model"))
    processor = CLIPProcessor.from_pretrained(model_path, local_files_only=True)
    model = CLIPModel.from_pretrained(model_path, local_files_only=True).eval().to("cuda:0", dtype=torch.float32)
    return model, processor


def image_projection(model: CLIPModel, pixels: torch.Tensor) -> torch.Tensor:
    output = model.vision_model(pixel_values=pixels)
    return model.visual_projection(output.pooler_output)


def text_projection(model: CLIPModel, input_ids: torch.Tensor, attention_mask: torch.Tensor) -> torch.Tensor:
    output = model.text_model(input_ids=input_ids, attention_mask=attention_mask)
    return model.text_projection(output.pooler_output)


def embed_assets(snapshot: Path, inventory: list[dict[str, Any]], model: CLIPModel, processor: CLIPProcessor) -> tuple[np.ndarray, list[dict[str, Any]]]:
    matrix = np.empty((len(inventory), DIMENSION), dtype=np.float64)
    index_rows = []
    for start in range(0, len(inventory), BATCH_ASSETS):
        chunk = inventory[start:start + BATCH_ASSETS]
        images = []
        try:
            for row in chunk:
                directory = snapshot / row["snapshot_render_dir"]
                for view in range(VIEWS):
                    with Image.open(require_regular_file(directory / f"{view:03d}.png")) as source:
                        image = source.convert("RGB")
                        image.load()
                    images.append(image)
            pixels = processor(images=images, return_tensors="pt")["pixel_values"].to("cuda:0", dtype=torch.float32)
            with torch.inference_mode():
                raw = image_projection(model, pixels).detach().cpu().numpy().astype(np.float64, copy=False)
        finally:
            for image in images:
                image.close()
        aggregate = normalized(normalized(raw.reshape(len(chunk), VIEWS, DIMENSION), axis=2).mean(axis=1), axis=1)
        matrix[start:start + len(chunk)] = aggregate
        for offset, row in enumerate(chunk):
            index_rows.append({
                "row": start + offset,
                "candidate_key": row["candidate_key"],
                "identity_sha256": row["identity_sha256"],
                "snapshot_render_dir": row["snapshot_render_dir"],
                "views": VIEWS,
            })
        print(f"[clip-assets] {start + len(chunk)}/{len(inventory)}", flush=True)
    return matrix, index_rows


def embed_prompts(prompts: list[dict[str, Any]], model: CLIPModel, processor: CLIPProcessor) -> tuple[np.ndarray, list[dict[str, Any]]]:
    tokens = processor.tokenizer([row["prompt"] for row in prompts], padding=True, truncation=True, max_length=77, return_tensors="pt")
    with torch.inference_mode():
        raw = text_projection(model, tokens["input_ids"].to("cuda:0"), tokens["attention_mask"].to("cuda:0"))
    matrix = normalized(raw.detach().cpu().numpy().astype(np.float64, copy=False), axis=1)
    records = []
    for index, row in enumerate(prompts):
        records.append({
            "task_id": row["task_id"],
            "prompt_sha256": sha256_text(row["prompt"]),
            "token_count_with_special_tokens": int(tokens["attention_mask"][index].sum()),
            "token_ids_sha256": sha256_text(json.dumps(tokens["input_ids"][index].tolist(), separators=(",", ":"))),
            "max_length": 77,
            "truncation": True,
        })
    return matrix, records


def one_run(snapshot: Path, inventory: list[dict[str, Any]], prompts: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]], list[dict[str, Any]]]:
    model, processor = load_model(snapshot)
    assets, index = embed_assets(snapshot, inventory, model, processor)
    text, tokens = embed_prompts(prompts, model, processor)
    del model, processor
    torch.cuda.empty_cache()
    return assets, text, index, tokens


def select(prompts: list[dict[str, Any]], text: np.ndarray, assets: np.ndarray, index: list[dict[str, Any]], expected_assets: int) -> list[dict[str, Any]]:
    similarities = text @ assets.T
    if similarities.shape != (18, expected_assets) or not np.isfinite(similarities).all():
        raise RuntimeError("invalid similarity matrix")
    rows = []
    for prompt_offset, prompt in enumerate(prompts):
        selected = min(range(len(index)), key=lambda asset_offset: (-float(similarities[prompt_offset, asset_offset]), index[asset_offset]["candidate_key"]))
        rows.append({
            "task_id": prompt["task_id"],
            "prompt_sha256": sha256_text(prompt["prompt"]),
            "selected_embedding_row": selected,
            "selected_candidate_key": index[selected]["candidate_key"],
            "clip_cosine_similarity": float(similarities[prompt_offset, selected]),
            "tie_break": "descending exact float64 score, ascending opaque candidate key",
            "fallback_allowed": False,
        })
    return rows


def save_npy(path: Path, value: np.ndarray) -> None:
    with safe(path, must_exist=False).open("wb") as stream:
        np.save(stream, value, allow_pickle=False)


def run(snapshot: Path, output: Path) -> None:
    snapshot = safe(snapshot)
    output = safe(output, must_exist=False)
    if output.exists():
        raise RuntimeError("selection output exists; refusing overwrite")
    output.parent.mkdir(parents=True, exist_ok=True)
    contract, contract_sha256 = validate_contract(snapshot)
    snapshot_lock, inventory = validate_snapshot(snapshot, contract, contract_sha256)
    expected_assets = contract["expected_assets"]
    prompts = read_jsonl(require_regular_file(snapshot / "prompt_only.jsonl"))
    if len(prompts) != 18 or prompts != sorted(prompts, key=lambda row: row["task_id"]):
        raise RuntimeError("prompt task closure/order drifted")
    if any(set(row) != {"task_id", "prompt"} for row in prompts):
        raise RuntimeError("prompt-only selector schema drifted")
    runtime = {"python": sys.version.split()[0], "numpy": np.__version__, "torch": torch.__version__, "transformers": transformers.__version__, "pillow": PIL.__version__}
    if runtime != EXPECTED_RUNTIME:
        raise RuntimeError(f"selection runtime drifted: {runtime}")
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "1" or not torch.cuda.is_available() or torch.cuda.device_count() != 1:
        raise RuntimeError("formal selection requires only physical GPU 1 visible")
    properties = torch.cuda.get_device_properties(0)
    gpu_query = subprocess.run(
        ["nvidia-smi", "-i", "1", "--query-gpu=uuid,driver_version", "--format=csv,noheader,nounits"],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if gpu_query.returncode != 0:
        raise RuntimeError(f"cannot query physical GPU identity: {gpu_query.stderr}")
    uuid, driver = [token.strip() for token in gpu_query.stdout.strip().split(",")]
    gpu = {
        "physical_device": 1,
        "logical_device": 0,
        "name": properties.name,
        "uuid": uuid,
        "driver_version": driver,
        "total_memory_bytes": properties.total_memory,
        "compute_capability": [properties.major, properties.minor],
    }
    if gpu != EXPECTED_GPU:
        raise RuntimeError(f"formal GPU identity drifted: {gpu}")
    staging = Path(tempfile.mkdtemp(prefix=output.name + ".staging.", dir=str(output.parent.parent)))
    try:
        os.chdir(snapshot)
        print("[selection] fresh full embedding run 1/2", flush=True)
        assets1, text1, index1, tokens1 = one_run(snapshot, inventory, prompts)
        selection1 = select(prompts, text1, assets1, index1, expected_assets)
        print("[selection] fresh full embedding run 2/2", flush=True)
        assets2, text2, index2, tokens2 = one_run(snapshot, inventory, prompts)
        selection2 = select(prompts, text2, assets2, index2, expected_assets)
        if not np.array_equal(assets1, assets2) or not np.array_equal(text1, text2):
            raise RuntimeError("fresh full embedding replay differs")
        if index1 != index2 or tokens1 != tokens2 or jsonl_bytes(selection1) != jsonl_bytes(selection2):
            raise RuntimeError("fresh full selection metadata replay differs")
        save_npy(staging / "asset_embeddings_run1.npy", assets1)
        save_npy(staging / "asset_embeddings_run2.npy", assets2)
        save_npy(staging / "prompt_embeddings_run1.npy", text1)
        save_npy(staging / "prompt_embeddings_run2.npy", text2)
        write_jsonl(staging / "embedding_index.jsonl", index1)
        write_jsonl(staging / "prompt_tokenization.jsonl", tokens1)
        write_jsonl(staging / "selection.jsonl", selection1)
        write_jsonl(staging / "selection_replay.jsonl", selection2)
        preflight = {
        "schema_version": 1,
        "status": "PASS",
        "snapshot_lock_sha256": sha256_file(snapshot / "snapshot.lock.json"),
        "prompt_manifest_sha256": EXPECTED_PROMPTS_SHA256,
        "prompt_only_manifest_sha256": contract["prompt_only_manifest_sha256"],
        "protocol_sha256": EXPECTED_PROTOCOL_SHA256,
        "contract_id": contract["contract_id"],
        "selection_execution_contract_sha256": contract_sha256,
        "candidate_assets": expected_assets,
        "eligible_assets": expected_assets,
        "prompt_count": 18,
        "model_revision": MODEL_REVISION,
        "batch_assets": BATCH_ASSETS,
        "full_embedding_replays": 2,
        "full_embedding_replay_byte_identical": True,
        "runtime": runtime,
        "gpu": gpu,
        "repair_attempts": 0,
        "rank_fallbacks": 0,
        "implementation": contract["implementation"],
    }
        write_json(staging / "selection_preflight.json", preflight)
        locked_files = ["asset_embeddings_run1.npy", "asset_embeddings_run2.npy", "prompt_embeddings_run1.npy", "prompt_embeddings_run2.npy", "embedding_index.jsonl", "prompt_tokenization.jsonl", "selection.jsonl", "selection_replay.jsonl", "selection_preflight.json"]
        selection_lock = {
        "schema_version": 1,
        "status": "PASS",
        "contract_id": contract["contract_id"],
        "phase": "selection_locked_before_source_binding_or_geometry_access",
        "snapshot_lock_sha256": preflight["snapshot_lock_sha256"],
        "private_source_audit_lock_sha256": snapshot_lock["private_source_audit_lock_sha256"],
        "source_binding_sha256": snapshot_lock["source_binding_sha256"],
        "prompt_manifest_sha256": EXPECTED_PROMPTS_SHA256,
        "prompt_only_manifest_sha256": contract["prompt_only_manifest_sha256"],
        "selection_execution_contract_sha256": contract_sha256,
        "candidate_assets": expected_assets,
        "eligible_assets": expected_assets,
        "task_count": 18,
        "full_embedding_replays": 2,
        "full_embedding_replay_byte_identical": True,
        "repair_attempts": 0,
        "rank_fallbacks": 0,
        "geometry_access_during_selection": False,
        "implementation": contract["implementation"],
        "locked_file_sha256": {name: sha256_file(staging / name) for name in locked_files},
    }
        write_json(staging / "selection.lock.json", selection_lock)
        staging.replace(output)
        print(json.dumps({"status": "SELECTION_LOCKED", "tasks": 18, "lock_sha256": sha256_file(output / "selection.lock.json")}, indent=2), flush=True)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-root", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    run(args.snapshot_root, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
