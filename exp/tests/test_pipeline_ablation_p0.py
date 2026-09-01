from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


EXP_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXP_ROOT.parent
SCRIPT_PATH = EXP_ROOT / "scripts/pipeline_ablation_p0.py"


def load_script():
    spec = importlib.util.spec_from_file_location("pipeline_ablation_p0", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


p0 = load_script()


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def make_record(records_root: Path, record_id: str, subcategory: str) -> Path:
    root = records_root / record_id
    write_json(root / "picture.json", {"category": "T2Fresh", "subcategory": subcategory})
    write_json(
        root / "record.json",
        {"active_revision_id": "rev_000001", "collections": ["workbench"]},
    )
    write_json(root / "collections/workbench.json", {"archived": False})
    model = root / "revisions/rev_000001/model.py"
    model.parent.mkdir(parents=True, exist_ok=True)
    model.write_text(
        "from __future__ import annotations\n"
        "\n"
        "def build():\n"
        "    width = 1.0\n"
        "    return {'width': width}\n",
        encoding="utf-8",
    )
    return model


def make_source_map(path: Path, slug: str, record_id: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"""# {slug} - SourceMap

source_map_schema: 1
export_category: {slug}
picture_category: T2Fresh
picture_subcategory: {slug}
category_scope: A synthetic articulated widget used only by isolated contract tests.

sync_records:
  - {record_id}

## Source review

| Record/Revision | Review status | Decision | Note |
|---|---|---|---|
| {record_id}/rev_000001 | reviewed | used | Supplies both test components and their mounting interface. |

## Candidate map

| Slot | Candidate | Component type | Record/Revision | Exact model.py:Lx-Ly | Distinction | Evidence |
|---|---|---|---|---|---|---|
| host | base | host body | {record_id}/rev_000001 | model.py:L3-L5 | structure | The source defines the host body and its mounting width. |
| module | knob | moving module | {record_id}/rev_000001 | model.py:L3-L5 | structure+motion | The source defines a separately mounted moving module. |
""",
        encoding="utf-8",
    )


def make_design(path: Path, source_map: Path, slug: str, record_id: str) -> dict:
    source_hash = hashlib.sha256(source_map.read_bytes()).hexdigest()
    payload = {
        "schema_version": 1,
        "slug": slug,
        "source_map_path": str(source_map.resolve()),
        "source_map_sha256": source_hash,
        "slots": [
            {
                "name": "host",
                "component_type": "host body",
                "required": True,
                "candidates": [
                    {
                        "name": "base",
                        "record_id": record_id,
                        "revision": "rev_000001",
                        "source_spans": ["model.py:L3-L5"],
                        "evidence": "The source defines the host body and its mounting width.",
                        "implementation_function": "_host_base_build",
                        "parameters": [
                            {
                                "name": "width",
                                "mode": "independent",
                                "value_type": "float",
                                "unit": "m",
                                "minimum": 0.2,
                                "maximum": 0.5,
                            },
                            {
                                "name": "inner_width",
                                "mode": "derived",
                                "value_type": "float",
                                "unit": "m",
                                "expression": "width - 0.02",
                                "depends_on": ["width"],
                            },
                        ],
                        "interfaces": [
                            {
                                "name": "mount",
                                "kind": "plane",
                                "role": "provider",
                                "part": "host",
                                "dimensions": {"extent": "inner_width"},
                            }
                        ],
                        "notes": "Keep the host parameterization independent of the mounted module.",
                    }
                ],
            },
            {
                "name": "module",
                "component_type": "moving module",
                "required": True,
                "candidates": [
                    {
                        "name": "knob",
                        "record_id": record_id,
                        "revision": "rev_000001",
                        "source_spans": ["model.py:L3-L5"],
                        "evidence": "The source defines a separately mounted moving module.",
                        "implementation_function": "_module_knob_build",
                        "parameters": [],
                        "interfaces": [
                            {
                                "name": "mount",
                                "kind": "plane",
                                "role": "consumer",
                                "part": "knob",
                                "dimensions": {"required": 0.1},
                            }
                        ],
                        "notes": "Mate the module locally to the host plane.",
                    }
                ],
            },
        ],
        "multiplicities": [],
        "bindings": [
            {
                "binding_id": "mount_module",
                "provider": "host.mount",
                "consumer": "module.mount",
                "joint_type": "fixed",
                "derived": [
                    {
                        "name": "adapter_width",
                        "mode": "derived",
                        "value_type": "float",
                        "unit": "m",
                        "expression": "min(provider.extent, consumer.required)",
                        "depends_on": ["provider.extent", "consumer.required"],
                    }
                ],
            }
        ],
        "category_anchors": [],
        "assembly_notes": "Preserve a local plane mate and the moving-module clearance.",
    }
    write_json(path, payload)
    return payload


def common_packet(slug: str, model_source: str = "def build():\n    return {}\n") -> dict:
    return {
        "schema_version": 1,
        "packet_kind": "pipeline_ablation_common_authoring_packet",
        "task": {"slug": slug, "prompt": "Author one reusable articulated widget generator."},
        "raw_source_pool": [
            {
                "record_id": f"rec_{slug}_source",
                "revision": "rev_000001",
                "model_source": model_source,
            }
        ],
        "sdk_contract": {
            "authoring_instructions": "Return one self-contained template using only the allowed API.",
            "allowed_api": ["build_object", "save_artifact"],
        },
    }


class PipelineAblationFreshnessTest(unittest.TestCase):
    def test_raw_records_do_not_downgrade_but_source_map_does(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            runtime = root / "exp/runtime"
            manifest = root / "candidates.json"
            write_json(
                manifest,
                {
                    "tasks": [
                        {"slug": "clean_widget", "records": [{"record_id": "rec_clean_widget_source"}]},
                        {"slug": "dirty_widget", "records": [{"record_id": "rec_dirty_widget_source"}]},
                    ]
                },
            )
            raw = runtime / "preparation/records/rec_clean_widget_source/revisions/rev_000001/model.py"
            raw.parent.mkdir(parents=True)
            raw.write_text("# clean_widget raw source only\n", encoding="utf-8")
            source_map = runtime / "preparation/source_maps/dirty_widget.md"
            source_map.parent.mkdir(parents=True)
            source_map.write_text("export_category: dirty_widget\n", encoding="utf-8")
            candidates, manifests = p0.load_candidates([manifest])
            first = p0.scan_freshness(
                project_root=root,
                candidates=candidates,
                manifest_paths=manifests,
                scopes=[("runtime", runtime)],
            )
            second = p0.scan_freshness(
                project_root=root,
                candidates=candidates,
                manifest_paths=manifests,
                scopes=[("runtime", runtime)],
            )
            self.assertEqual(first, second)
            statuses = {row["slug"]: row["status"] for row in first["tasks"]}
            self.assertEqual(statuses["clean_widget"], "no_local_conflict_found")
            self.assertEqual(statuses["dirty_widget"], "development_only")
            observed_statuses = {row["status"] for row in first["tasks"]}
            self.assertNotIn("fresh", observed_statuses)
            self.assertLessEqual(observed_statuses, set(p0.FRESHNESS_STATUSES))
            self.assertFalse(first["selection_authorized"])

    def test_formal_twelve_are_development_only(self):
        manifest = PROJECT_ROOT / "exp/runtime/t2_formal_v1/preparation/formal_source_manifest.json"
        if not manifest.is_file():
            self.skipTest("local t2 formal preparation is unavailable")
        candidates, manifests = p0.load_candidates([manifest])
        preparation = manifest.parent
        report = p0.scan_freshness(
            project_root=PROJECT_ROOT,
            candidates=candidates,
            manifest_paths=manifests,
            scopes=[("t2_formal_preparation", preparation)],
            max_evidence_per_candidate=20,
        )
        self.assertEqual(len(report["tasks"]), 12)
        self.assertEqual(report["status_counts"]["development_only"], 12)
        self.assertTrue(all(row["status"] == "development_only" for row in report["tasks"]))


class PipelineAblationFactorTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.slug = "widget"
        self.record_id = "rec_widget_source"
        self.records_root = self.root / "records"
        self.model = make_record(self.records_root, self.record_id, self.slug)
        self.source_map = self.root / "source_maps/widget.md"
        make_source_map(self.source_map, self.slug, self.record_id)
        self.design = self.root / "designs/widget.json"
        self.raw_design = make_design(self.design, self.source_map, self.slug, self.record_id)
        self.common = common_packet(self.slug, self.model.read_text(encoding="utf-8"))

    def tearDown(self):
        self.tmp.cleanup()

    def build(self, *, common: dict | None = None):
        return p0.prepare_factor_bundle(
            project_root=PROJECT_ROOT,
            source_map=self.source_map,
            design_path=self.design,
            records_root=self.records_root,
            common_authoring_packet=self.common if common is None else common,
        )

    def test_factor_projection_reconstructs_and_composes_four_arms(self):
        bundle = self.build()
        audit = bundle["private_audit"]
        self.assertEqual(audit["verdict"], "pass", audit["problems"])
        self.assertTrue(audit["normalization_reconstruction"]["byte_identical"])
        self.assertTrue(audit["common_authoring_packet_byte_identical_across_arms"])
        packets = bundle["author_packets"]
        self.assertEqual(set(packets), set(p0.ARM_FACTORS))
        self.assertEqual(packets["A00"]["factors"], {})
        self.assertEqual(list(packets["A10"]["factors"]), ["S_factor"])
        self.assertEqual(list(packets["A01"]["factors"]), ["D_factor"])
        self.assertEqual(list(packets["A11"]["factors"]), ["S_factor", "D_factor"])
        common_bytes = {p0.canonical_bytes(row["common_authoring_packet"]) for row in packets.values()}
        self.assertEqual(len(common_bytes), 1)
        self.assertEqual(packets["A00"]["common_authoring_packet"]["raw_source_pool"], self.common["raw_source_pool"])
        s_factor = packets["A10"]["factors"]["S_factor"]
        owners = s_factor["source_map"]["source_to_owner_functions"]
        self.assertEqual(
            {row["implementation_function"] for row in owners},
            {"_host_base_build", "_module_knob_build"},
        )
        serialized_d = json.dumps(packets["A01"]["factors"]["D_factor"], sort_keys=True)
        for forbidden in (self.record_id, "rev_000001", "model.py:L3-L5", "implementation_function"):
            self.assertNotIn(forbidden, serialized_d)
        for packet in packets.values():
            self.assertEqual(p0.schema_findings("author_packet", packet), [])

    def test_common_packet_is_required_and_fail_closed(self):
        bad = copy.deepcopy(self.common)
        bad["raw_source_pool"] = []
        bundle = self.build(common=bad)
        self.assertEqual(bundle["private_audit"]["verdict"], "fail")
        self.assertEqual(bundle["author_packets"], {})
        self.assertTrue(any("raw_source_pool" in item for item in bundle["private_audit"]["problems"]))

    def test_d_source_reference_leak_is_blocking(self):
        payload = copy.deepcopy(self.raw_design)
        payload["assembly_notes"] = f"Reopen {self.record_id}/rev_000001 model.py:L3-L5."
        write_json(self.design, payload)
        bundle = self.build()
        self.assertEqual(bundle["private_audit"]["verdict"], "fail")
        self.assertEqual(bundle["author_packets"], {})
        self.assertTrue(any(item.startswith("leakage:") for item in bundle["private_audit"]["problems"]))

    def test_candidate_provenance_mismatch_is_blocking(self):
        payload = copy.deepcopy(self.raw_design)
        payload["slots"][0]["candidates"][0]["evidence"] = "Drifted evidence."
        write_json(self.design, payload)
        bundle = self.build()
        self.assertEqual(bundle["private_audit"]["verdict"], "fail")
        self.assertTrue(any("candidate_provenance_mismatch" in item for item in bundle["private_audit"]["problems"]))

    def test_binding_endpoint_must_exist_on_every_candidate(self):
        d_factor = p0.build_d_factor(self.raw_design)
        second = copy.deepcopy(d_factor["slots"][0]["candidates"][0])
        second["name"] = "base_without_mount"
        second["interfaces"] = []
        d_factor["slots"][0]["candidates"].append(second)
        findings = p0.dependency_findings(d_factor)
        self.assertIn(
            "binding_endpoint_not_total:mount_module:provider:host.mount",
            findings,
        )


class PipelineAblationDevelopmentFixtureSmokeTest(unittest.TestCase):
    def test_extension_ladder_formal_factor_fixture(self):
        preparation = PROJECT_ROOT / "exp/runtime/t2_formal_v1/preparation"
        source_map = preparation / "source_maps/extension_ladder.md"
        design = preparation / "designs/extension_ladder.json"
        records_root = preparation / "records"
        if not all(path.exists() for path in (source_map, design, records_root)):
            self.skipTest("local t2 formal fixture is unavailable")
        source_maps, _ = p0._load_authoring_modules(PROJECT_ROOT)
        resolution = source_maps.parse_source_map(source_map, records_root)
        common = {
            "schema_version": 1,
            "packet_kind": "pipeline_ablation_common_authoring_packet",
            "task": {
                "slug": "extension_ladder",
                "prompt": "Author one reusable extension-ladder generator from the complete raw source pool.",
            },
            "raw_source_pool": [
                {
                    "record_id": record.record_id,
                    "revision": record.revision,
                    "model_source": record.model_path.read_text(encoding="utf-8", errors="replace"),
                }
                for record in resolution.source_pool
            ],
            "sdk_contract": {
                "authoring_instructions": "Produce one self-contained single-file template under the shared build contract.",
                "allowed_api": ["agent", "build123d", "template_sdk"],
            },
        }
        bundle = p0.prepare_factor_bundle(
            project_root=PROJECT_ROOT,
            source_map=source_map,
            design_path=design,
            records_root=records_root,
            common_authoring_packet=common,
        )
        audit = bundle["private_audit"]
        self.assertEqual(audit["verdict"], "pass", audit["problems"])
        self.assertEqual(set(bundle["author_packets"]), set(p0.ARM_FACTORS))
        self.assertTrue(audit["normalization_reconstruction"]["byte_identical"])


if __name__ == "__main__":
    unittest.main()
