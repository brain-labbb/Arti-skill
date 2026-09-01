#!/usr/bin/env python3
"""Run audited prompt-only CLIP retrieval on a render-only Artiverse snapshot."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import stat
import sys
from pathlib import Path
from typing import Any

os.environ["HF_HUB_OFFLINE"] = "1"
os.environ["TRANSFORMERS_OFFLINE"] = "1"
os.environ["HF_HUB_DISABLE_PROGRESS_BARS"] = "1"
os.environ["TOKENIZERS_PARALLELISM"] = "false"
os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import numpy as np
import PIL
from PIL import Image
import torch
import transformers
from transformers import CLIPModel, CLIPProcessor


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
WORKSPACE = REPO.parent.resolve()
MATERIALIZER = REPO / "exp/scripts/run_artiverse_clip_retrieval_materialize.py"
SNAPSHOT_BUILDER = REPO / "exp/scripts/prepare_artiverse_clip_retrieval_snapshot.py"
DEFAULT_SNAPSHOT = REPO / "exp/runtime/table4_constraints_v2/artiverse_clip_retrieval_v1_snapshot"
DEFAULT_OUTPUT = REPO / "exp/runtime/table4_constraints_v2/artiverse_clip_retrieval_v1"
METHOD = "artiverse_clip_retrieval_v1"
MODEL_REVISION = "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268"
FORMAL_BATCH_ASSETS = 8
EXPECTED_PROMPTS_SHA256 = "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e"
EXPECTED_DATASET_MANIFEST_SHA256 = "8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586"
EXPECTED_ORIGINAL_AMENDMENT_SHA256 = "ded01ce6b663559ea64955d5a69100894d7baacc304579131692accb02042fce"
EXPECTED_THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
EXPECTED_RUNTIME_VERSIONS = {
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
REQUIRED_VIEWS = tuple(f"{index:03d}.png" for index in range(16))
ALLOWED_PREEXISTING_OUTPUTS = {"protocol_audit_pre_result.json", "report.md"}


def safe(path: Path, *, must_exist: bool = True) -> Path:
    resolved = path.resolve(strict=must_exist)
    if resolved != WORKSPACE and WORKSPACE not in resolved.parents:
        raise RuntimeError(f"path outside authorized workspace: {resolved}")
    return resolved


def sha256_file(path: Path, block_size: int = 8 * 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with safe(path).open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def read_json(path: Path) -> Any:
    return json.loads(safe(path).read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    for line_number, line in enumerate(safe(path).read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        row = json.loads(line)
        if not isinstance(row, dict):
            raise RuntimeError(f"JSONL line {line_number} is not an object: {path}")
        rows.append(row)
    return rows


def atomic_text(path: Path, value: str) -> None:
    destination = safe(path, must_exist=False)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = safe(destination.with_suffix(destination.suffix + ".tmp"), must_exist=False)
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(destination)


def write_json(path: Path, value: Any) -> None:
    atomic_text(path, json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n")


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    atomic_text(
        path,
        "".join(json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows),
    )


def jsonl_bytes(rows: list[dict[str, Any]]) -> bytes:
    return "".join(
        json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n" for row in rows
    ).encode("utf-8")


def require_regular_file(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
        raise RuntimeError(f"required regular non-symlink file missing: {path}")
    return resolved


def require_regular_dir(path: Path) -> Path:
    resolved = safe(path)
    if path.is_symlink() or not stat.S_ISDIR(path.lstat().st_mode):
        raise RuntimeError(f"required regular non-symlink directory missing: {path}")
    return resolved


def validate_output_audit(output: Path, snapshot_lock: dict[str, Any]) -> dict[str, Any]:
    output = safe(output, must_exist=False)
    if not output.exists():
        raise RuntimeError("formal output must first be created by the independent pre-result auditor")
    require_regular_dir(output)
    present = {entry.name for entry in os.scandir(output)}
    if present != ALLOWED_PREEXISTING_OUTPUTS:
        raise RuntimeError(
            f"formal output must initially contain exactly {sorted(ALLOWED_PREEXISTING_OUTPUTS)}; "
            f"observed {sorted(present)}"
        )
    audit_path = require_regular_file(output / "protocol_audit_pre_result.json")
    report_path = require_regular_file(output / "report.md")
    audit = read_json(audit_path)
    verdict = str(audit.get("verdict") or audit.get("status") or "").upper()
    if verdict != "PASS":
        raise RuntimeError(f"pre-result protocol audit verdict is not PASS: {verdict!r}")
    required = {
        "original_amendment_sha256": EXPECTED_ORIGINAL_AMENDMENT_SHA256,
        "addendum_sha256": snapshot_lock["implementation"]["addendum_sha256"],
        "snapshot_preflight_script_sha256": sha256_file(SNAPSHOT_BUILDER),
        "selector_sha256": sha256_file(SCRIPT),
        "materializer_sha256": sha256_file(MATERIALIZER),
        "formal_batch_assets": FORMAL_BATCH_ASSETS,
        "report_sha256": sha256_file(report_path),
    }
    for field, expected in required.items():
        if audit.get(field) != expected:
            raise RuntimeError(
                f"pre-result audit binding mismatch for {field}: "
                f"{audit.get(field)!r} != {expected!r}"
            )
    if audit.get("protocol_ready") is not True:
        raise RuntimeError("pre-result audit did not mark protocol_ready=true")
    return {
        "path": str(audit_path),
        "sha256": sha256_file(audit_path),
        "report_path": str(report_path),
        "report_sha256": sha256_file(report_path),
        "verdict": verdict,
        "bindings": required,
    }


def validate_snapshot(snapshot: Path) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    snapshot = require_regular_dir(snapshot)
    root_entries = {entry.name for entry in os.scandir(snapshot)}
    expected_root_entries = {
        "candidate_inventory.jsonl",
        "candidate_summary.json",
        "model",
        "prompts.jsonl",
        "renders",
        "snapshot.lock.json",
    }
    if root_entries != expected_root_entries:
        raise RuntimeError(
            f"snapshot root closure drifted: {sorted(root_entries)} != {sorted(expected_root_entries)}"
        )
    lock = read_json(require_regular_file(snapshot / "snapshot.lock.json"))
    if lock.get("status") != "PASS" or lock.get("snapshot_type") != "render_only_identity_snapshot":
        raise RuntimeError("render-only snapshot lock is not a PASS")
    if lock.get("is_os_sandbox") is not False:
        raise RuntimeError("snapshot must not overclaim OS sandbox isolation")
    if lock.get("prompt_manifest_sha256") != EXPECTED_PROMPTS_SHA256:
        raise RuntimeError("snapshot prompt manifest hash mismatch")
    if lock.get("dataset_manifest_sha256") != EXPECTED_DATASET_MANIFEST_SHA256:
        raise RuntimeError("snapshot dataset manifest hash mismatch")
    if lock.get("candidate_assets") != 3544:
        raise RuntimeError("snapshot candidate count is not 3544")
    gate = lock.get("full_source_gate", {})
    if not gate.get("passed") or gate.get("actual_model_roots") != 3544:
        raise RuntimeError("snapshot full-source gate is not a 3544-root PASS")
    if gate.get("actual_file_count") != 531937 or gate.get("actual_input_bytes") != 86992752890:
        raise RuntimeError("snapshot full-source file/byte gate drifted")
    if gate.get("symlink_count") != 0:
        raise RuntimeError("snapshot source data contained symlinks")
    archives = lock.get("archives")
    if not isinstance(archives, list) or len(archives) != 2 or not all(row.get("passed") for row in archives):
        raise RuntimeError("snapshot archive gate is incomplete or failed")
    locked_files = lock.get("snapshot_files_sha256", {})
    expected_locked = {"prompts.jsonl", "candidate_inventory.jsonl", "candidate_summary.json"}
    if set(locked_files) != expected_locked:
        raise RuntimeError("snapshot locked-file set drifted")
    for name, expected in locked_files.items():
        if sha256_file(require_regular_file(snapshot / name)) != expected:
            raise RuntimeError(f"snapshot locked-file hash mismatch: {name}")
    if locked_files["prompts.jsonl"] != EXPECTED_PROMPTS_SHA256:
        raise RuntimeError("snapshot prompt copy is not the frozen prompt manifest")
    model_hashes = lock.get("model_files_sha256", {})
    if model_hashes != EXPECTED_MODEL_FILES:
        raise RuntimeError("snapshot model file set or hashes drifted")
    model_root = require_regular_dir(snapshot / "model")
    model_entries = {entry.name for entry in os.scandir(model_root)}
    if model_entries != set(EXPECTED_MODEL_FILES):
        raise RuntimeError("snapshot model directory contains missing or extra files")
    for name, expected in model_hashes.items():
        if sha256_file(require_regular_file(model_root / name)) != expected:
            raise RuntimeError(f"snapshot model hash mismatch: {name}")
    inventory = read_jsonl(snapshot / "candidate_inventory.jsonl")
    inventory_by_identity = {row.get("identity"): row for row in inventory}
    if len(inventory) != 3544 or len(inventory_by_identity) != 3544:
        raise RuntimeError("snapshot candidate inventory is incomplete or duplicated")
    eligible = [row for row in inventory if row.get("eligible") is True]
    if len(eligible) != lock.get("eligible_assets"):
        raise RuntimeError("snapshot eligible count disagrees with candidate inventory")
    if lock.get("eligible_render_files") != len(eligible) * 16:
        raise RuntimeError("snapshot eligible render count drifted")
    renders_root = require_regular_dir(snapshot / "renders")
    expected_render_dirs = {row["identity_sha256"] for row in eligible}
    observed_render_entries = {entry.name for entry in os.scandir(renders_root)}
    if observed_render_entries != expected_render_dirs:
        raise RuntimeError("snapshot renders directory identity closure drifted")
    if any(not entry.is_dir(follow_symlinks=False) for entry in os.scandir(renders_root)):
        raise RuntimeError("snapshot renders root contains a non-directory or symlink entry")
    for row in eligible:
        identity_hash = row.get("identity_sha256")
        if identity_hash != sha256_text(str(row.get("identity"))):
            raise RuntimeError("snapshot identity hash mismatch")
        render_dir = require_regular_dir(snapshot / str(row.get("snapshot_render_dir")))
        png_names = sorted(entry.name for entry in os.scandir(render_dir))
        if png_names != list(REQUIRED_VIEWS):
            raise RuntimeError(f"snapshot render set drifted: {row.get('identity')}")
        for name in REQUIRED_VIEWS:
            expected = row["views"][name]["sha256"]
            if sha256_file(require_regular_file(render_dir / name)) != expected:
                raise RuntimeError(f"snapshot render hash mismatch: {row.get('identity')}/{name}")
    prompts = read_jsonl(snapshot / "prompts.jsonl")
    expected_tasks = {f"T4C{number:03d}" for number in range(1, 19)}
    if len(prompts) != 18 or {row.get("task_id") for row in prompts} != expected_tasks:
        raise RuntimeError("snapshot prompt task set drifted")
    implementation = lock.get("implementation", {})
    expected_code = {
        "snapshot_preflight_script_sha256": sha256_file(SNAPSHOT_BUILDER),
        "selector_sha256": sha256_file(SCRIPT),
        "materializer_sha256": sha256_file(MATERIALIZER),
    }
    for field, expected in expected_code.items():
        if implementation.get(field) != expected:
            raise RuntimeError(f"snapshot implementation binding drifted: {field}")
    if implementation.get("original_amendment_sha256") != EXPECTED_ORIGINAL_AMENDMENT_SHA256:
        raise RuntimeError("snapshot original amendment binding drifted")
    return lock, eligible, sorted(prompts, key=lambda row: row["task_id"])


def configure_determinism() -> None:
    random.seed(0)
    np.random.seed(0)
    torch.manual_seed(0)
    torch.cuda.manual_seed_all(0)
    torch.use_deterministic_algorithms(True)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True


def normalized(value: np.ndarray, *, axis: int = -1) -> np.ndarray:
    norm = np.linalg.norm(value, axis=axis, keepdims=True)
    if not np.isfinite(norm).all() or (norm <= 0).any():
        raise RuntimeError("non-finite or zero CLIP feature norm")
    return value / norm


def processor_assertions(processor: CLIPProcessor, model: CLIPModel) -> dict[str, Any]:
    image = processor.image_processor
    tokenizer = processor.tokenizer
    observed = {
        "resize_shortest_edge": image.size.shortest_edge,
        "crop_height": image.crop_size.height,
        "crop_width": image.crop_size.width,
        "resample": int(image.resample),
        "image_mean": list(image.image_mean),
        "image_std": list(image.image_std),
        "do_resize": image.do_resize,
        "do_center_crop": image.do_center_crop,
        "do_normalize": image.do_normalize,
        "do_convert_rgb": image.do_convert_rgb,
        "tokenizer_model_max_length": tokenizer.model_max_length,
        "tokenizer_padding_side": tokenizer.padding_side,
        "tokenizer_truncation_side": tokenizer.truncation_side,
        "tokenizer_bos_token_id": tokenizer.bos_token_id,
        "tokenizer_eos_token_id": tokenizer.eos_token_id,
        "tokenizer_pad_token_id": tokenizer.pad_token_id,
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
        "do_resize": True,
        "do_center_crop": True,
        "do_normalize": True,
        "do_convert_rgb": True,
        "tokenizer_model_max_length": 77,
        "tokenizer_padding_side": "right",
        "tokenizer_truncation_side": "right",
        "tokenizer_bos_token_id": 49406,
        "tokenizer_eos_token_id": 49407,
        "tokenizer_pad_token_id": 49407,
        "projection_dim": 512,
        "vision_image_size": 224,
        "vision_patch_size": 32,
        "text_max_position_embeddings": 77,
    }
    if observed != expected:
        raise RuntimeError(f"loaded CLIP runtime configuration drifted: {observed}")
    return observed


def image_projection(model: CLIPModel, pixel_values: torch.Tensor) -> torch.Tensor:
    outputs = model.vision_model(pixel_values=pixel_values)
    return model.visual_projection(outputs.pooler_output)


def text_projection(
    model: CLIPModel,
    input_ids: torch.Tensor,
    attention_mask: torch.Tensor,
) -> torch.Tensor:
    outputs = model.text_model(input_ids=input_ids, attention_mask=attention_mask)
    return model.text_projection(outputs.pooler_output)


def load_model(snapshot: Path) -> tuple[CLIPModel, CLIPProcessor, dict[str, Any]]:
    configure_determinism()
    model_root = str(require_regular_dir(snapshot / "model"))
    processor = CLIPProcessor.from_pretrained(model_root, local_files_only=True)
    model = CLIPModel.from_pretrained(model_root, local_files_only=True).eval().to(
        "cuda:0", dtype=torch.float32
    )
    assertions = processor_assertions(processor, model)
    return model, processor, assertions


def embed_assets(
    snapshot: Path,
    eligible: list[dict[str, Any]],
    model: CLIPModel,
    processor: CLIPProcessor,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    embeddings = np.empty((len(eligible), 512), dtype=np.float64)
    index_records = []
    for start in range(0, len(eligible), FORMAL_BATCH_ASSETS):
        chunk = eligible[start : start + FORMAL_BATCH_ASSETS]
        images = []
        try:
            for record in chunk:
                render_dir = snapshot / record["snapshot_render_dir"]
                for name in REQUIRED_VIEWS:
                    with Image.open(require_regular_file(render_dir / name)) as source:
                        rgb = source.convert("RGB")
                        rgb.load()
                    images.append(rgb)
            inputs = processor(images=images, return_tensors="pt")
            pixels = inputs["pixel_values"].to("cuda:0", dtype=torch.float32)
            with torch.inference_mode():
                features = image_projection(model, pixels)
            raw = features.detach().cpu().numpy().astype(np.float64, copy=False)
        finally:
            for image in images:
                image.close()
        raw = raw.reshape(len(chunk), 16, 512)
        aggregate = normalized(normalized(raw, axis=2).mean(axis=1), axis=1)
        embeddings[start : start + len(chunk)] = aggregate
        for offset, record in enumerate(chunk):
            index_records.append({
                "row": start + offset,
                "identity": record["identity"],
                "identity_sha256": record["identity_sha256"],
                "snapshot_render_dir": record["snapshot_render_dir"],
                "views": 16,
            })
        print(f"[embedding] {start + len(chunk)}/{len(eligible)} assets", flush=True)
    return embeddings, index_records


def embed_prompts(
    prompts: list[dict[str, Any]],
    model: CLIPModel,
    processor: CLIPProcessor,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    texts = [str(row["prompt"]) for row in prompts]
    tokens = processor.tokenizer(
        texts,
        padding=True,
        truncation=True,
        max_length=77,
        return_tensors="pt",
    )
    with torch.inference_mode():
        features = text_projection(
            model,
            tokens["input_ids"].to("cuda:0"),
            tokens["attention_mask"].to("cuda:0"),
        )
    embeddings = normalized(features.detach().cpu().numpy().astype(np.float64, copy=False), axis=1)
    records = []
    for index, row in enumerate(prompts):
        token_ids = tokens["input_ids"][index].tolist()
        mask = tokens["attention_mask"][index].tolist()
        records.append({
            "task_id": row["task_id"],
            "prompt_sha256": sha256_text(str(row["prompt"])),
            "token_count_with_special_tokens": int(sum(mask)),
            "token_ids_sha256": sha256_text(json.dumps(token_ids, separators=(",", ":"))),
            "max_length": 77,
            "truncation": True,
        })
    return embeddings, records


def select_top1(
    prompts: list[dict[str, Any]],
    prompt_embeddings: np.ndarray,
    asset_embeddings: np.ndarray,
    index_records: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    scores = prompt_embeddings @ asset_embeddings.T
    if scores.shape != (18, len(index_records)) or not np.isfinite(scores).all():
        raise RuntimeError("invalid CLIP similarity matrix")
    selections = []
    for prompt_index, prompt in enumerate(prompts):
        order = sorted(
            range(len(index_records)),
            key=lambda index: (
                -float(scores[prompt_index, index]),
                index_records[index]["identity_sha256"],
                index_records[index]["identity"],
            ),
        )
        selected_index = order[0]
        selected = index_records[selected_index]
        selections.append({
            "task_id": prompt["task_id"],
            "prompt_sha256": sha256_text(str(prompt["prompt"])),
            "selected_embedding_row": selected_index,
            "selected_identity": selected["identity"],
            "selected_identity_sha256": selected["identity_sha256"],
            "clip_cosine_similarity": float(scores[prompt_index, selected_index]),
            "tie_break": "descending exact float64 score, ascending identity SHA-256, ascending identity",
            "fallback_allowed": False,
        })
    return selections


def save_npy(path: Path, value: np.ndarray) -> None:
    destination = safe(path, must_exist=False)
    temporary = safe(destination.with_suffix(destination.suffix + ".tmp"), must_exist=False)
    with temporary.open("wb") as stream:
        np.save(stream, value, allow_pickle=False)
    temporary.replace(destination)


def gpu_record() -> dict[str, Any]:
    if os.environ.get("CUDA_DEVICE_ORDER") != "PCI_BUS_ID":
        raise RuntimeError("CUDA_DEVICE_ORDER must be PCI_BUS_ID")
    if os.environ.get("CUDA_VISIBLE_DEVICES") != "0":
        raise RuntimeError("formal run requires CUDA_VISIBLE_DEVICES=0")
    if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
        raise RuntimeError("pinned physical GPU 0 is not exclusively visible")
    properties = torch.cuda.get_device_properties(0)
    return {
        "cuda_device_order": os.environ["CUDA_DEVICE_ORDER"],
        "cuda_visible_devices": os.environ["CUDA_VISIBLE_DEVICES"],
        "logical_device": 0,
        "pinned_physical_device": 0,
        "name": properties.name,
        "total_memory_bytes": properties.total_memory,
        "compute_capability": [properties.major, properties.minor],
        "device_count_visible": torch.cuda.device_count(),
    }


def runtime_version_gate() -> dict[str, str]:
    observed = {
        "python": sys.version.split()[0],
        "numpy": np.__version__,
        "torch": torch.__version__,
        "transformers": transformers.__version__,
        "pillow": PIL.__version__,
    }
    if observed != EXPECTED_RUNTIME_VERSIONS:
        raise RuntimeError(f"formal runtime versions drifted: {observed}")
    return observed


def thread_environment_gate() -> dict[str, str]:
    observed = {name: os.environ.get(name, "") for name in EXPECTED_THREAD_ENV}
    if observed != EXPECTED_THREAD_ENV:
        raise RuntimeError(f"formal numerical thread environment drifted: {observed}")
    return observed


def one_full_embedding_run(
    snapshot: Path,
    eligible: list[dict[str, Any]],
    prompts: list[dict[str, Any]],
) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    model, processor, runtime = load_model(snapshot)
    assets, index = embed_assets(snapshot, eligible, model, processor)
    prompt_embeddings, tokenization = embed_prompts(prompts, model, processor)
    del model, processor
    torch.cuda.empty_cache()
    return assets, prompt_embeddings, index, tokenization, runtime


def run_selection(snapshot: Path, output: Path) -> None:
    snapshot = safe(snapshot)
    lock, eligible, prompts = validate_snapshot(snapshot)
    audit = validate_output_audit(output, lock)
    versions = runtime_version_gate()
    thread_environment = thread_environment_gate()
    device = gpu_record()
    os.chdir(snapshot)
    print("[selection] full embedding run 1/2", flush=True)
    assets1, prompts1, index1, tokens1, runtime1 = one_full_embedding_run(snapshot, eligible, prompts)
    selections1 = select_top1(prompts, prompts1, assets1, index1)
    print("[selection] full embedding run 2/2 with a fresh model instance", flush=True)
    assets2, prompts2, index2, tokens2, runtime2 = one_full_embedding_run(snapshot, eligible, prompts)
    selections2 = select_top1(prompts, prompts2, assets2, index2)
    if not np.array_equal(assets1, assets2):
        raise RuntimeError("independent full asset embedding replay is not byte-identical")
    if not np.array_equal(prompts1, prompts2):
        raise RuntimeError("independent full prompt embedding replay is not byte-identical")
    if index1 != index2 or tokens1 != tokens2 or runtime1 != runtime2:
        raise RuntimeError("independent full embedding replay metadata differs")
    if jsonl_bytes(selections1) != jsonl_bytes(selections2):
        raise RuntimeError("independent full top-1 replay is not byte-identical")
    save_npy(output / "asset_embeddings_run1.npy", assets1)
    save_npy(output / "asset_embeddings_run2.npy", assets2)
    save_npy(output / "prompt_embeddings_run1.npy", prompts1)
    save_npy(output / "prompt_embeddings_run2.npy", prompts2)
    write_jsonl(output / "embedding_index.jsonl", index1)
    write_jsonl(output / "prompt_tokenization.jsonl", tokens1)
    write_jsonl(output / "selection.jsonl", selections1)
    write_jsonl(output / "selection_replay.jsonl", selections2)
    preflight = {
        "schema_version": 1,
        "status": "PASS",
        "method": METHOD,
        "snapshot": str(snapshot),
        "snapshot_lock_sha256": sha256_file(snapshot / "snapshot.lock.json"),
        "snapshot_isolation_claim": lock["isolation_claim"],
        "is_os_sandbox": False,
        "pre_result_audit": audit,
        "formal_batch_assets": FORMAL_BATCH_ASSETS,
        "candidate_assets": lock["candidate_assets"],
        "eligible_assets": lock["eligible_assets"],
        "prompt_count": len(prompts),
        "model_revision": MODEL_REVISION,
        "processor_runtime_assertions": runtime1,
        "gpu": device,
        "runtime_versions": versions,
        "numerical_thread_environment": thread_environment,
        "inference_dtype": "float32_without_amp",
        "aggregation_dtype": "float64",
        "full_embedding_replays": 2,
        "full_embedding_replay_byte_identical": True,
        "selection_script_sha256": sha256_file(SCRIPT),
        "materializer_sha256": sha256_file(MATERIALIZER),
        "snapshot_preflight_script_sha256": sha256_file(SNAPSHOT_BUILDER),
        "addendum_sha256": lock["implementation"]["addendum_sha256"],
        "original_amendment_sha256": EXPECTED_ORIGINAL_AMENDMENT_SHA256,
        "repair_attempts": 0,
    }
    write_json(output / "selection_preflight.json", preflight)
    locked_names = [
        "protocol_audit_pre_result.json",
        "report.md",
        "selection_preflight.json",
        "asset_embeddings_run1.npy",
        "asset_embeddings_run2.npy",
        "prompt_embeddings_run1.npy",
        "prompt_embeddings_run2.npy",
        "embedding_index.jsonl",
        "prompt_tokenization.jsonl",
        "selection.jsonl",
        "selection_replay.jsonl",
    ]
    selection_lock = {
        "schema_version": 1,
        "method": METHOD,
        "phase": "selection_locked_before_geometry_access",
        "snapshot_lock_sha256": preflight["snapshot_lock_sha256"],
        "dataset_manifest_sha256": EXPECTED_DATASET_MANIFEST_SHA256,
        "prompt_manifest_sha256": EXPECTED_PROMPTS_SHA256,
        "original_amendment_sha256": EXPECTED_ORIGINAL_AMENDMENT_SHA256,
        "addendum_sha256": preflight["addendum_sha256"],
        "selection_script_sha256": preflight["selection_script_sha256"],
        "materializer_sha256": preflight["materializer_sha256"],
        "snapshot_preflight_script_sha256": preflight["snapshot_preflight_script_sha256"],
        "formal_batch_assets": FORMAL_BATCH_ASSETS,
        "candidate_assets": lock["candidate_assets"],
        "eligible_assets": lock["eligible_assets"],
        "embedding_index_rows": len(index1),
        "task_count": len(selections1),
        "full_embedding_replays": 2,
        "full_embedding_replay_byte_identical": True,
        "repair_attempts": 0,
        "geometry_access_during_selection": False,
        "isolation_claim": lock["isolation_claim"],
        "is_os_sandbox": False,
        "locked_file_sha256": {name: sha256_file(output / name) for name in locked_names},
    }
    write_json(output / "selection.lock.json", selection_lock)
    print(json.dumps({
        "status": "SELECTION_LOCKED",
        "candidates": selection_lock["candidate_assets"],
        "eligible": selection_lock["eligible_assets"],
        "tasks": selection_lock["task_count"],
        "full_embedding_replays": 2,
        "selection_lock_sha256": sha256_file(output / "selection.lock.json"),
    }, indent=2), flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot-root", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_selection(args.snapshot_root, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
