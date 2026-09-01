from __future__ import annotations

import importlib.util
import json
import random
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


EXP_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = EXP_ROOT.parent
EVALUATOR_PATH = EXP_ROOT / "scripts/evaluate_pipeline_ablation_canonical_cases.py"
SPEC_PATH = EXP_ROOT / "reference/pipeline_ablation_canonical_dev_fixture_v1.json"


def load_evaluator():
    spec = importlib.util.spec_from_file_location("pipeline_ablation_canonical_evaluator_test", EVALUATOR_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


evaluator = load_evaluator()


def template_source(mutation: str | None, rng: random.Random) -> str:
    axis = (0.0, 0.0, 1.0)
    origin = (0.0, 0.5, 0.4)
    lower = 0.0
    upper = 1.2
    include_door = True
    body_values = ("box", "rounded")
    extra_domain_slot = ""
    duplicate_joint = False
    revision = "rev_000007"
    if mutation == "axis":
        axis = rng.choice(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
    elif mutation == "malformed_axis":
        axis = (0.0, 1.0)
    elif mutation == "origin":
        origin = (round(rng.uniform(0.1, 0.3), 6), 0.5, 0.4)
    elif mutation == "malformed_origin":
        origin = (0.0, "not-a-number", 0.4)
    elif mutation == "limit":
        upper = round(rng.uniform(0.2, 0.6), 6)
    elif mutation == "axis_sign_transformed_limit":
        axis = (0.0, 0.0, -1.0)
        lower = -1.2
        upper = 0.0
    elif mutation == "axis_sign_untransformed_limit":
        axis = (0.0, 0.0, -1.0)
    elif mutation == "ambiguous_joint":
        duplicate_joint = True
    elif mutation == "missing_role":
        include_door = False
    elif mutation == "domain_shrink":
        body_values = ("box",)
    elif mutation == "domain_extra":
        extra_domain_slot = 'Slot("candidate_only", ("hidden_choice",)),'
    elif mutation == "source_revision":
        revision = f"rev_{rng.randint(1, 6):06d}"
    elif mutation is not None:
        raise ValueError(mutation)
    return textwrap.dedent(
        f"""
        import random
        from dataclasses import dataclass

        @dataclass(frozen=True)
        class Slot:
            name: str
            values: tuple

        @dataclass(frozen=True)
        class Domain:
            slots: tuple

            def sample(self, seed: int):
                rng = random.Random(seed)
                return {{slot.name: rng.choice(slot.values) for slot in self.slots}}

            def validate(self, values):
                declared = {{slot.name: slot for slot in self.slots}}
                if set(values) != set(declared):
                    raise ValueError("domain fields do not match")
                for name, value in values.items():
                    if value not in declared[name].values:
                        raise ValueError(f"undeclared value for {{name}}: {{value!r}}")

        @dataclass(frozen=True)
        class Origin:
            xyz: tuple

        @dataclass(frozen=True)
        class Limits:
            lower: float
            upper: float

        @dataclass(frozen=True)
        class Part:
            name: str

        @dataclass(frozen=True)
        class Joint:
            name: str
            articulation_type: str
            parent: str
            child: str
            origin: Origin
            axis: tuple
            motion_limits: Limits

        class Model:
            def __init__(self):
                self.parts = []
                self.articulations = []

            def part(self, name):
                self.parts.append(Part(name))

            def articulation(self, name, articulation_type, parent, child, *, origin, axis, motion_limits):
                self.articulations.append(
                    Joint(name, articulation_type, parent, child, origin, axis, motion_limits)
                )

        SOURCE_PROVENANCE = {{
            "rec_dev_synthetic_hinge": {{"revision": {revision!r}}}
        }}

        @dataclass(frozen=True)
        class Config:
            body_variant: str
            panel_count: int

        TEMPLATE_DOMAIN = Domain(slots=(
            Slot("body_variant", {body_values!r}),
            Slot("panel_count", (1, 2)),
            {extra_domain_slot}
        ))

        def config_from_seed(seed: int) -> Config:
            values = TEMPLATE_DOMAIN.sample(seed)
            return Config(body_variant=values["body_variant"], panel_count=values["panel_count"])

        def resolve_config(config: Config) -> Config:
            TEMPLATE_DOMAIN.validate({{
                "body_variant": config.body_variant,
                "panel_count": config.panel_count,
            }})
            return config

        def build_synthetic_hinged_container(config: Config):
            resolve_config(config)
            model = Model()
            model.part("body")
            model.part("door_0_decoration")
            for index in range(config.panel_count):
                child = f"door_{{index}}"
                if {include_door!r}:
                    model.part(child)
                model.articulation(
                    f"body_to_door_{{index}}",
                    "revolute",
                    "body",
                    child,
                    origin=Origin(xyz={origin!r}),
                    axis={axis!r},
                    motion_limits=Limits(lower={lower!r}, upper={upper!r}),
                )
                if {duplicate_joint!r} and index == 0:
                    model.articulation(
                        "body_to_door_0_duplicate",
                        "revolute",
                        "body",
                        child,
                        origin=Origin(xyz={origin!r}),
                        axis={axis!r},
                        motion_limits=Limits(lower={lower!r}, upper={upper!r}),
                    )
            return model
        """
    )


class PipelineAblationCanonicalEvaluatorTest(unittest.TestCase):
    def setUp(self):
        self.spec = json.loads(SPEC_PATH.read_text(encoding="utf-8"))
        self.tmp = tempfile.TemporaryDirectory(dir=EXP_ROOT / "tests", prefix="canonical_eval_test_")
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def write_template(self, name: str, mutation: str | None, rng: random.Random) -> Path:
        path = self.root / f"{name}.py"
        path.write_text(template_source(mutation, rng), encoding="utf-8")
        return path

    def test_positive_fixture_passes_smoke_but_strict_remains_not_evaluable(self):
        seed = int(self.spec["mutation_test_contract"]["seed"])
        report = evaluator.evaluate(self.write_template("positive", None, random.Random(seed)), SPEC_PATH)
        self.assertEqual(
            report["case_source"],
            "evaluator_owned_json; candidate samplers/seeds/corners are not case generators",
        )
        self.assertEqual(report["development_smoke_status"], "PASS")
        self.assertEqual(report["strict_success_status"], "N/E")
        for metric in ("source_content_fidelity", "collision_geometry_validity", "continuous_collision_detection"):
            self.assertEqual(report["metrics"][metric]["status"], "N/E")
        self.assertEqual(len(report["cases"]), 2)
        first_role_records = report["cases"][0]["metrics"]["role_presence"]["evidence"]["records"]
        door_role = next(row for row in first_role_records if row["role_id"] == "door_0")
        self.assertEqual(door_role["matched_parts"], ["door_0"])
        flipped = evaluator.evaluate(
            self.write_template(
                "axis_sign_transformed_limit",
                "axis_sign_transformed_limit",
                random.Random(seed),
            ),
            SPEC_PATH,
        )
        self.assertEqual(flipped["metrics"]["joint_axis"]["status"], "PASS")
        self.assertEqual(flipped["metrics"]["joint_limit"]["status"], "PASS")
        self.assertEqual(flipped["development_smoke_status"], "PASS")

    def test_seeded_required_mutations_fail_the_expected_metric(self):
        contract = self.spec["mutation_test_contract"]
        rng = random.Random(int(contract["seed"]))
        mutation_names = list(contract["required_mutations"])
        rng.shuffle(mutation_names)
        for mutation in mutation_names:
            with self.subTest(mutation=mutation):
                path = self.write_template(mutation, mutation, rng)
                report = evaluator.evaluate(path, SPEC_PATH)
                expected_metric = contract["required_mutations"][mutation]
                self.assertEqual(report["development_smoke_status"], "FAIL")
                self.assertEqual(report["metrics"][expected_metric]["status"], "FAIL")
                self.assertNotEqual(report["strict_success_status"], "PASS")
                if mutation == "ambiguous_joint":
                    for metric in ("joint_detection", "joint_axis", "joint_origin", "joint_limit"):
                        self.assertEqual(report["metrics"][metric]["status"], "FAIL")
                    detection_records = [
                        row
                        for case in report["cases"]
                        for row in case["metrics"]["joint_detection"]["evidence"]["records"]
                    ]
                    self.assertTrue(
                        any("ambiguous joint match" in row.get("reason", "") for row in detection_records)
                    )

    def test_cli_writes_a_report_without_promoting_n_e_to_success(self):
        seed = int(self.spec["mutation_test_contract"]["seed"])
        template = self.write_template("cli_positive", None, random.Random(seed))
        output = self.root / "report.json"
        completed = subprocess.run(
            [
                sys.executable,
                str(EVALUATOR_PATH),
                "--template",
                str(template),
                "--spec",
                str(SPEC_PATH),
                "--output",
                str(output),
            ],
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        report = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(report["development_smoke_status"], "PASS")
        self.assertEqual(report["strict_success_status"], "N/E")


if __name__ == "__main__":
    unittest.main()
