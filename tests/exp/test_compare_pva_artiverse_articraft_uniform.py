from __future__ import annotations

import hashlib
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SUBJECT = ROOT / "exp/scripts/compare_pva_artiverse_articraft_uniform.py"


def load_subject() -> Any:
    name = "compare_pva_artiverse_articraft_policy_test_subject"
    spec = importlib.util.spec_from_file_location(name, SUBJECT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_relative_render_path_is_resolved_under_render_root(tmp_path: Path) -> None:
    subject = load_subject()
    image = tmp_path / "category" / "asset" / "imgs" / "000.png"
    image.parent.mkdir(parents=True)
    image.write_bytes(b"placeholder")

    resolved = subject._row_image_path(
        tmp_path,
        {"output_path": "category/asset/imgs/000.png"},
    )

    assert resolved == image.resolve()


def test_articraft_feature_cache_keys_requested_amp_and_helper_sha(
    tmp_path: Path,
) -> None:
    subject = load_subject()
    helper_path = tmp_path / "feature_helper.py"
    helper_path.write_text("VERSION = 1\n", encoding="utf-8")
    image = tmp_path / "000.png"
    image.write_bytes(b"image")
    model_dirs = {encoder: tmp_path / f"model-{encoder}" for encoder in subject.ENCODERS}
    for model_dir in model_dirs.values():
        model_dir.mkdir()

    calls: list[tuple[str, bool]] = []

    class Helper:
        def extract_image_features(
            self,
            image_paths: Any,
            *,
            model_path: Path,
            batch_size: int,
            device: str,
            num_workers: int,
            use_amp: bool,
        ) -> tuple[np.ndarray, dict[str, Any]]:
            encoder = model_path.name.removeprefix("model-")
            dimension = 3 if encoder == "dinov2" else 2
            calls.append((encoder, use_amp))
            matrix = np.arange(1, dimension + 1, dtype=np.float32)[None, :]
            return matrix, {
                "model_type": encoder,
                "feature_dim": dimension,
                "feature_source": "test",
                "image_count": len(image_paths),
                "device": "cpu",
                "amp": False,
                "batch_size": batch_size,
                "num_workers": num_workers,
            }

    def normalize(matrix: np.ndarray) -> np.ndarray:
        matrix = np.asarray(matrix, dtype=np.float32)
        return matrix / np.linalg.norm(matrix, axis=1, keepdims=True)

    fake_base = SimpleNamespace(
        PVA_HELPER_SCRIPT=helper_path,
        _pva_helper=lambda: Helper(),
        l2_normalize=normalize,
    )
    subject._BASE = fake_base
    bundle = SimpleNamespace(
        records=(
            SimpleNamespace(
                category="category",
                asset_id="asset",
                image_bytes=image.stat().st_size,
                image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
                image_path=image,
            ),
        ),
        receipts={"render_manifest": {"sha256": "a" * 64}},
    )
    pva = SimpleNamespace(
        feature_manifests={
            encoder: {
                "model_path": str(model_dirs[encoder]),
                "model_fingerprint": f"fingerprint-{encoder}",
            }
            for encoder in subject.ENCODERS
        },
        features={
            "dinov2": np.ones((1, 3), dtype=np.float32),
            "clip": np.ones((1, 2), dtype=np.float32),
        },
    )

    for use_amp in (False, True, True):
        subject._extract_articraft(
            bundle,
            pva=pva,
            output_dir=tmp_path / "output",
            batch_size=1,
            device="cpu",
            num_workers=1,
            use_amp=use_amp,
            force_extract=False,
        )

    assert calls == [
        ("dinov2", False),
        ("clip", False),
        ("dinov2", True),
        ("clip", True),
    ]

    helper_path.write_text("VERSION = 2\n", encoding="utf-8")
    subject._extract_articraft(
        bundle,
        pva=pva,
        output_dir=tmp_path / "output",
        batch_size=1,
        device="cpu",
        num_workers=1,
        use_amp=True,
        force_extract=False,
    )
    assert calls[-2:] == [("dinov2", True), ("clip", True)]
    assert len(calls) == 6
