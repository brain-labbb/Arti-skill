"""Landed Shenzhou style return capsule (reference: night recovery photo).

Layout (meters, Z up, capsule axis on Z, modeled in canonical upright pose):
- capsule: blunt charred heat shield disc under a bell-shaped body with
  scorched tan-bronze ablative walls, dark scorch streaks swept up the bell,
  two round portholes, four radial RCS thruster ports near the base, a dark
  open cabin recess under the top collar, a grey hatch collar with a white
  seal face, and two hinge posts beside the collar.
- hatch: round domed crew hatch lid with a central viewport, an exterior
  grab handle, and a hinge barrel with straps, on a REVOLUTE hinge at the
  -X collar edge; positive q swings the lid up and over to the side (open,
  as in the photo where recovery crew work at the open hatch).
"""

from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
BASE_R = 1.26  # heat shield rim radius (Shenzhou is ~2.5 m across)
COLLAR_TOP_Z = 2.36
HINGE_XYZ = (-0.49, 0.0, 2.40)  # hatch hinge point at the -X collar edge
LID_OFFSET_X = 0.49  # lid axis offset inside the hatch part frame
HATCH_OPEN = 2.0

RCS_AZIMUTHS = [pi / 4.0 + k * pi / 2.0 for k in range(4)]
STREAK_AZIMUTHS = [0.3, 1.2, 2.4, 3.5, 4.4, 5.5]


def _radial(r: float, ang: float, z: float) -> tuple[float, float, float]:
    return (r * cos(ang), r * sin(ang), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shenzhou_return_capsule")

    model.material("charred_bronze", color=(0.55, 0.42, 0.27))
    model.material("scorch_dark", color=(0.20, 0.16, 0.12))
    model.material("shield_char", color=(0.15, 0.12, 0.10))
    model.material("collar_grey", color=(0.62, 0.63, 0.65))
    model.material("lid_grey", color=(0.76, 0.77, 0.78))
    model.material("seal_white", color=(0.88, 0.88, 0.87))
    model.material("cabin_dark", color=(0.07, 0.07, 0.08))
    model.material("glass_dark", color=(0.05, 0.06, 0.08))

    # -------------------------------------------------------------- capsule
    capsule = model.part("capsule")

    # Blunt ablative heat shield with a charred proud rim ring.
    shield_profile = [
        (0.00, 0.00),
        (0.80, 0.05),
        (1.15, 0.14),
        (BASE_R, 0.28),
        (BASE_R, 0.38),
        (0.00, 0.38),
    ]
    capsule.visual(
        mesh_from_geometry(LatheGeometry(shield_profile, segments=48), "heat_shield"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="shield_char",
        name="heat_shield",
    )

    # Bell-shaped scorched body, widest above the shield, narrowing to the
    # flat top face that carries the hatch collar.
    bell_profile = [
        (0.00, 0.34),
        (1.24, 0.34),
        (1.22, 0.60),
        (1.10, 1.10),
        (0.92, 1.55),
        (0.72, 1.90),
        (0.56, 2.12),
        (0.50, 2.22),
        (0.00, 2.22),
    ]
    capsule.visual(
        mesh_from_geometry(LatheGeometry(bell_profile, segments=48), "bell_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="charred_bronze",
        name="bell_body",
    )

    # Raised hatch collar (hollow ring) with a white seal face on top.
    collar = LatheGeometry.from_shell_profiles(
        [(0.46, 0.0), (0.46, 0.16)],
        [(0.38, 0.0), (0.38, 0.16)],
        segments=48,
    )
    capsule.visual(
        mesh_from_geometry(collar, "hatch_collar"),
        origin=Origin(xyz=(0.0, 0.0, 2.20)),
        material="collar_grey",
        name="hatch_collar",
    )
    seal = LatheGeometry.from_shell_profiles(
        [(0.45, 0.0), (0.45, 0.015)],
        [(0.39, 0.0), (0.39, 0.015)],
        segments=48,
    )
    capsule.visual(
        mesh_from_geometry(seal, "hatch_seal_ring"),
        origin=Origin(xyz=(0.0, 0.0, 2.355)),
        material="seal_white",
        name="hatch_seal_ring",
    )
    # Dark cabin opening visible through the collar when the hatch is open.
    capsule.visual(
        Cylinder(radius=0.37, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 2.215)),
        material="cabin_dark",
        name="cabin_recess",
    )

    # Two round portholes on the bell flanks.
    for i, ang in enumerate((pi / 2.0, -pi / 2.0)):
        capsule.visual(
            Cylinder(radius=0.155, length=0.04),
            origin=Origin(xyz=_radial(0.99, ang, 1.35), rpy=(0.0, pi / 2.0 - 0.38, ang)),
            material="seal_white",
            name=f"porthole_rim_{i}",
        )
        capsule.visual(
            Cylinder(radius=0.11, length=0.05),
            origin=Origin(xyz=_radial(1.00, ang, 1.35), rpy=(0.0, pi / 2.0 - 0.38, ang)),
            material="glass_dark",
            name=f"porthole_glass_{i}",
        )

    # Four radial RCS thruster ports near the base.
    for i, ang in enumerate(RCS_AZIMUTHS):
        capsule.visual(
            Cylinder(radius=0.045, length=0.07),
            origin=Origin(xyz=_radial(1.20, ang, 0.50), rpy=(0.0, pi / 2.0 - 0.08, ang)),
            material="scorch_dark",
            name=f"rcs_port_{i}",
        )

    # Dark reentry scorch streaks swept up the bell wall.
    for i, ang in enumerate(STREAK_AZIMUTHS):
        z_c = 0.95 if i % 2 == 0 else 1.15
        r_c = 1.136 if i % 2 == 0 else 1.08
        capsule.visual(
            Box((0.03, 0.10, 0.45)),
            origin=Origin(xyz=_radial(r_c, ang, z_c), rpy=(0.0, -0.30, ang)),
            material="scorch_dark",
            name=f"scorch_streak_{i}",
        )

    # Hatch hinge posts rising from the bell top beside the collar.
    for i, sy in enumerate((-1, 1)):
        capsule.visual(
            Box((0.06, 0.05, 0.19)),
            origin=Origin(xyz=(-0.49, sy * 0.095, 2.305)),
            material="collar_grey",
            name=f"hinge_post_{i}",
        )

    # ---------------------------------------------------------------- hatch
    hatch = model.part("hatch")
    lid_profile = [
        (0.000, 0.000),
        (0.300, 0.000),
        (0.440, 0.005),
        (0.455, 0.050),
        (0.420, 0.100),
        (0.260, 0.155),
        (0.000, 0.180),
    ]
    hatch.visual(
        mesh_from_geometry(LatheGeometry(lid_profile, segments=48), "lid_shell"),
        origin=Origin(xyz=(LID_OFFSET_X, 0.0, 0.0)),
        material="lid_grey",
        name="lid_shell",
    )
    # Central viewport with rim and dark glass.
    hatch.visual(
        Cylinder(radius=0.125, length=0.02),
        origin=Origin(xyz=(LID_OFFSET_X, 0.0, 0.17)),
        material="collar_grey",
        name="viewport_rim",
    )
    hatch.visual(
        Cylinder(radius=0.095, length=0.02),
        origin=Origin(xyz=(LID_OFFSET_X, 0.0, 0.185)),
        material="glass_dark",
        name="viewport_glass",
    )
    # Exterior grab handle near the free edge (bar on two posts).
    for i, sy in enumerate((-1, 1)):
        hatch.visual(
            Cylinder(radius=0.012, length=0.08),
            origin=Origin(xyz=(LID_OFFSET_X + 0.24, sy * 0.06, 0.14)),
            material="collar_grey",
            name=f"handle_post_{i}",
        )
    hatch.visual(
        Cylinder(radius=0.015, length=0.16),
        origin=Origin(xyz=(LID_OFFSET_X + 0.24, 0.0, 0.18), rpy=(pi / 2.0, 0.0, 0.0)),
        material="collar_grey",
        name="handle_bar",
    )
    # Hinge barrel and straps tying the lid to the hinge line.
    hatch.visual(
        Cylinder(radius=0.028, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.02), rpy=(pi / 2.0, 0.0, 0.0)),
        material="collar_grey",
        name="hinge_barrel",
    )
    for i, sy in enumerate((-1, 1)):
        hatch.visual(
            Box((0.12, 0.04, 0.03)),
            origin=Origin(xyz=(0.05, sy * 0.045, 0.02)),
            material="collar_grey",
            name=f"hinge_strap_{i}",
        )

    model.articulation(
        "capsule_to_hatch",
        ArticulationType.REVOLUTE,
        parent=capsule,
        child=hatch,
        origin=Origin(xyz=HINGE_XYZ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.6, lower=0.0, upper=HATCH_OPEN),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Intentional local embed: the hinge barrel is captured between the two
    # hinge posts beside the collar.
    for i in range(2):
        ctx.allow_overlap(
            "capsule",
            "hatch",
            elem_a=f"hinge_post_{i}",
            elem_b="hinge_barrel",
            reason="hatch hinge barrel is captured between the collar-side hinge posts",
        )

    parts = {p.name for p in object_model.parts}
    ctx.check(
        "capsule and hatch parts present",
        {"capsule", "hatch"} <= parts,
        f"parts: {sorted(parts)}",
    )

    capsule = object_model.get_part("capsule")
    n_port = sum(1 for v in capsule.visuals if v.name and v.name.startswith("porthole_rim_"))
    ctx.check("2 round portholes", n_port == 2, f"found {n_port}")
    n_rcs = sum(1 for v in capsule.visuals if v.name and v.name.startswith("rcs_port_"))
    ctx.check("4 radial RCS thruster ports", n_rcs == 4, f"found {n_rcs}")
    n_streak = sum(1 for v in capsule.visuals if v.name and v.name.startswith("scorch_streak_"))
    ctx.check("6 scorch streaks on the bell", n_streak == 6, f"found {n_streak}")

    j_hatch = object_model.get_articulation("capsule_to_hatch")
    ctx.check(
        "crew hatch hinge is REVOLUTE about the collar-edge Y axis",
        j_hatch.articulation_type == ArticulationType.REVOLUTE and abs(j_hatch.axis[1]) > 0.99,
        f"{j_hatch.articulation_type}, axis {j_hatch.axis}",
    )
    ctx.check(
        "hatch closes at q=0 and opens to ~2 rad",
        j_hatch.motion_limits is not None
        and j_hatch.motion_limits.lower == 0.0
        and j_hatch.motion_limits.upper >= 1.8,
        f"limits {j_hatch.motion_limits}",
    )

    # Overall proportions: ~2.5 m across the shield, base on z=0, collar top
    # near 2.4 m.
    aabb = ctx.part_world_aabb("capsule")
    width = aabb[1][0] - aabb[0][0]
    ctx.check("capsule bell spans ~2.5 m across", 2.4 < width < 2.6, f"width {width:.3f}")
    ctx.check("heat shield sits at the base", aabb[0][2] < 0.01, f"min z {aabb[0][2]:.3f}")
    ctx.check(
        "collar and hinge posts top out near 2.4 m",
        2.30 < aabb[1][2] < 2.50,
        f"max z {aabb[1][2]:.3f}",
    )

    # Closed lid is centered over the collar and seats just above the seal.
    hatch_aabb0 = ctx.part_world_aabb("hatch")
    ctx.check(
        "closed lid is centered over the collar opening",
        hatch_aabb0[1][0] > 0.40 and abs(hatch_aabb0[1][1] + hatch_aabb0[0][1]) < 0.05,
        f"closed aabb {hatch_aabb0}",
    )
    ctx.expect_within(
        "hatch",
        "capsule",
        axes="xy",
        margin=0.05,
        name="closed hatch stays within the capsule footprint",
    )
    ctx.expect_gap(
        "hatch",
        "capsule",
        axis="z",
        positive_elem="lid_shell",
        negative_elem="hatch_seal_ring",
        min_gap=0.0,
        max_gap=0.05,
        name="closed lid seats just above the white seal face",
    )

    # Opening the hinge swings the lid up and over to the -X side.
    with ctx.pose({j_hatch: HATCH_OPEN}):
        hatch_open = ctx.part_world_aabb("hatch")
        ctx.check(
            "open lid clears the collar opening toward -X",
            hatch_open[1][0] < -0.35,
            f"open aabb {hatch_open}",
        )
        ctx.check(
            "open lid swings up above the closed height",
            hatch_open[1][2] > hatch_aabb0[1][2] + 0.4,
            f"open max z {hatch_open[1][2]:.3f}, closed {hatch_aabb0[1][2]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
