#!/usr/bin/env python3
"""Create a blinded four-view packet for independent T3 semantic/hierarchy review."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


EXP_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RECORDS = EXP_ROOT / "runtime/nano3d_glb_n33/output/records.json"
DEFAULT_OUT = EXP_ROOT / "runtime/t3_formal_v1/semantic_review"
VIEWS = ((22, 38), (22, 128), (22, 218), (65, 308))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def render_contact_sheet(glb: Path, output: Path) -> None:
    loaded = trimesh.load(glb, force="scene", process=False)
    mesh = loaded.dump(concatenate=True)
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    normals = np.asarray(mesh.face_normals)
    if len(faces) > 18000:
        indices = np.linspace(0, len(faces) - 1, 18000, dtype=int)
        faces = faces[indices]
        normals = normals[indices]
    face_vertices = vertices[faces]
    light = np.clip(normals @ np.asarray([0.55, -0.35, 0.76]), -1.0, 1.0)
    shade = 0.34 + 0.58 * ((light + 1.0) / 2.0)
    colors = np.column_stack(
        [0.45 * shade, 0.58 * shade, 0.76 * shade, np.ones(len(faces))]
    )
    bounds = np.asarray(mesh.bounds)
    centre = bounds.mean(axis=0)
    radius = max(float((bounds[1] - bounds[0]).max()) / 2.0, 1e-6)
    fig = plt.figure(figsize=(10, 10), dpi=120)
    for index, (elevation, azimuth) in enumerate(VIEWS, start=1):
        axis = fig.add_subplot(2, 2, index, projection="3d")
        axis.add_collection3d(
            Poly3DCollection(
                face_vertices,
                facecolors=colors,
                linewidths=0.0,
                antialiased=False,
            )
        )
        axis.set_xlim(centre[0] - radius, centre[0] + radius)
        axis.set_ylim(centre[1] - radius, centre[1] + radius)
        axis.set_zlim(centre[2] - radius, centre[2] + radius)
        axis.view_init(elev=elevation, azim=azimuth)
        axis.set_axis_off()
    fig.tight_layout(pad=0.15)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, facecolor="white")
    plt.close(fig)


def category_from_asset(asset_id: str) -> str:
    return asset_id.rsplit("__seed_", 1)[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--records", type=Path, default=DEFAULT_RECORDS)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    output = args.out.resolve()
    output.relative_to(EXP_ROOT.resolve())
    rows = json.loads(args.records.read_text(encoding="utf-8"))
    panels: list[dict[str, Any]] = []
    for index, row in enumerate(sorted(rows, key=lambda item: item["asset_id"]), start=1):
        glb = Path(row["output_glb"])
        render = output / "renders" / f"S{index:03d}.png"
        if not render.exists():
            render_contact_sheet(glb, render)
        edges = [edge for edge in row["urdf_expected_edges"]]
        panels.append(
            {
                "panel_id": f"S{index:03d}",
                "asset_id": row["asset_id"],
                "category": category_from_asset(row["asset_id"]).replace("_", " "),
                "render": str(render),
                "render_sha256": sha256(render),
                "emitted_parts": row["urdf_link_names"],
                "emitted_parent_child_edges": edges,
                "review_fields": {
                    "parts": [
                        {"name": name, "semantically_valid": None, "role": None, "notes": ""}
                        for name in row["urdf_link_names"]
                    ],
                    "expected_visible_or_functional_roles": [],
                    "edges": [
                        {"parent": parent, "child": child, "semantically_correct": None, "notes": ""}
                        for parent, child in edges
                    ],
                    "hierarchy_exact_match": None,
                    "instance_discriminability": None,
                    "notes": "",
                },
            }
        )
    packet = {
        "schema_version": 1,
        "protocol": "t3_independent_model_semantic_review_v1",
        "blinding": "Reviewers receive category, four-view render, emitted link names and emitted edges; no source template, tests, prior score, or other reviewer labels.",
        "instructions": [
            "Inspect every four-view render before labeling the panel.",
            "A semantically valid emitted part must denote a coherent rigid functional or structural unit; decorative naming alone is insufficient.",
            "List expected roles conservatively: only externally visible or functionally necessary rigid roles supported by category and render. Do not invent hidden internals.",
            "For each expected role, provide min_instances and matched emitted part names. Precision uses valid emitted parts; recall uses matched expected role instances.",
            "Judge each parent-child edge as a mechanically plausible ownership/articulation relation. hierarchy_exact_match requires no missing, extra, or incorrectly parented required role.",
            "instance_discriminability is true only when repeated instances have distinct, stable names.",
            "Return the complete packet JSON with every null replaced; do not edit any other file.",
        ],
        "panel_count": len(panels),
        "panels": panels,
    }
    dump_json(output / "review_packet.json", packet)
    print(json.dumps({"panels": len(panels), "packet": str(output / "review_packet.json")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
