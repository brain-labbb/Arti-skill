from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


POLE_X = (-0.085, 0.085)
POLE_Y = 0.0
UPPER_SLEEVE_MOUTH_Z = 0.180

# ---------------------------------------------------------------------------
# 4-section telescoping configuration
# Three stages nest below the upper handle section (section 1).
# Each stage's tube fits inside the previous stage's tube.
# ---------------------------------------------------------------------------
STAGE_DEFS = [
    {
        "name": "mid",
        "tube_name": "mid_tube",
        "band_name": "mid_band",
        "tube_r": 0.0080,
        "tube_len": 0.300,
        "tube_z": 0.010,
        "band_r": 0.0088,
        "band_z": -0.060,
        "band_len": 0.012,
        "junction_z": -0.125,
        "slide": 0.085,
    },
    {
        "name": "inner",
        "tube_name": "inner_tube",
        "band_name": "inner_band",
        "tube_r": 0.0065,
        "tube_len": 0.280,
        "tube_z": 0.010,
        "band_r": 0.0072,
        "band_z": -0.050,
        "band_len": 0.010,
        "junction_z": -0.115,
        "slide": 0.080,
    },
    {
        "name": "lower",
        "tube_name": "lower_tube",
        "band_name": None,
        "tube_r": 0.0050,
        "tube_len": 0.260,
        "tube_z": 0.010,
        "band_r": None,
        "band_z": 0.0,
        "band_len": 0.0,
        "junction_z": 0.0,
        "slide": 0.075,
    },
]

# Twist-collar ring dimensions for each of the 3 section junctions
COLLAR_DEFS = [
    {"outer_r": 0.0130, "h": 0.032},   # upper-to-mid
    {"outer_r": 0.0110, "h": 0.028},   # mid-to-inner
    {"outer_r": 0.0090, "h": 0.026},   # inner-to-lower
]


# ---------------------------------------------------------------------------
# Geometry helpers (preserved from parent)
# ---------------------------------------------------------------------------

def _cyl_z(part, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _make_cork_handle_mesh(name: str):
    """Lathed, lightly ergonomic cork grip with top and lower flares."""
    profile = [
        (0.0130, -0.085),
        (0.0180, -0.075),
        (0.0160, -0.050),
        (0.0135, -0.020),
        (0.0165, 0.015),
        (0.0185, 0.050),
        (0.0155, 0.080),
        (0.0125, 0.088),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48, closed=True), name)


def _make_foam_grip_mesh(name: str):
    """Ribbed lower black grip sleeve under the cork handle."""
    profile = [
        (0.0110, -0.070),
        (0.0165, -0.064),
        (0.0165, -0.048),
        (0.0125, -0.043),
        (0.0125, -0.030),
        (0.0160, -0.025),
        (0.0160, -0.010),
        (0.0125, -0.005),
        (0.0125, 0.010),
        (0.0156, 0.016),
        (0.0156, 0.030),
        (0.0122, 0.036),
        (0.0122, 0.050),
        (0.0150, 0.056),
        (0.0150, 0.070),
        (0.0110, 0.074),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=40, closed=True), name)


def _make_tip_mesh(name: str):
    return mesh_from_geometry(
        ConeGeometry(0.0075, 0.050, radial_segments=32, closed=True), name
    )


def _make_basket_mesh(name: str):
    return mesh_from_geometry(
        TorusGeometry(0.030, 0.0026, radial_segments=16, tubular_segments=48), name
    )


def _make_twist_collar_mesh(outer_r: float, height: float, name: str):
    """Twist-lock collar ring with knurled grip ridges (hollow bore)."""
    inner_r = outer_r * 0.82
    bump = outer_r * 0.05
    profile = [
        (inner_r, -height * 0.50),
        (outer_r * 0.97, -height * 0.48),
        (outer_r, -height * 0.35),
        (outer_r + bump, -height * 0.18),
        (outer_r, -height * 0.02),
        (outer_r + bump, height * 0.14),
        (outer_r, height * 0.30),
        (outer_r + bump, height * 0.44),
        (outer_r * 0.97, height * 0.48),
        (inner_r, height * 0.50),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=36, closed=True), name)


# ---------------------------------------------------------------------------
# Upper assemblies (handle + sleeve — shared between both poles)
# ---------------------------------------------------------------------------

def _add_upper_pole(root, i, x, mats, handle_mesh, foam_mesh):
    cork, black, white, metal, dark = mats
    root.visual(
        handle_mesh,
        origin=Origin(xyz=(x, POLE_Y, 0.585)),
        material=cork,
        name=f"pole_{i}_cork_handle",
    )
    root.visual(
        foam_mesh,
        origin=Origin(xyz=(x, POLE_Y, 0.425)),
        material=black,
        name=f"pole_{i}_foam_grip",
    )
    _cyl_z(root, 0.0175, 0.030, (x, POLE_Y, 0.688), metal, f"pole_{i}_top_cap")
    _cyl_z(root, 0.0100, 0.300, (x, POLE_Y, 0.330), white, f"pole_{i}_upper_sleeve")
    _cyl_z(root, 0.0115, 0.016, (x, POLE_Y, 0.475), black, f"pole_{i}_upper_black_band")
    _cyl_z(root, 0.0115, 0.014, (x, POLE_Y, 0.205), black, f"pole_{i}_lower_black_band")
    _cyl_z(root, 0.0032, 0.298, (x, POLE_Y, 0.540), black, f"pole_{i}_hidden_core")
    root.visual(
        Box((0.005, 0.0024, 0.180)),
        origin=Origin(xyz=(x, POLE_Y - 0.0105, 0.335)),
        material=dark,
        name=f"pole_{i}_carbon_label",
    )
    root.visual(
        Box((0.018, 0.012, 0.088)),
        origin=Origin(xyz=(x - 0.038, POLE_Y - 0.049, 0.575), rpy=(0.0, 0.0, -0.35)),
        material=black,
        name=f"pole_{i}_strap_tag",
    )
    root.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (x - 0.006, -0.004, 0.670),
                    (x - 0.030, -0.036, 0.640),
                    (x - 0.037, -0.055, 0.585),
                    (x - 0.022, -0.040, 0.535),
                    (x - 0.004, -0.014, 0.610),
                    (x + 0.006, -0.004, 0.670),
                ],
                radius=0.0028,
                samples_per_segment=14,
                radial_segments=14,
                cap_ends=True,
            ),
            f"pole_{i}_wrist_loop_mesh",
        ),
        material=black,
        name=f"pole_{i}_wrist_loop",
    )


def _add_lower_stage_extras(part, mats, tip_mesh, basket_mesh):
    """Ferrule, trekking basket, and carbide tip on the lowest stage."""
    _, black, _, metal, _ = mats
    _cyl_z(part, 0.0062, 0.025, (0.0, 0.0, -0.120), metal, "ferrule")
    part.visual(
        basket_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=black,
        name="basket_ring",
    )
    part.visual(
        Box((0.066, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=black,
        name="basket_spoke_x",
    )
    part.visual(
        Box((0.004, 0.066, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=black,
        name="basket_spoke_y",
    )
    part.visual(
        tip_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.155), rpy=(math.pi, 0.0, 0.0)),
        material=metal,
        name="carbide_tip",
    )


# ---------------------------------------------------------------------------
# Model construction
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="trekking_pole_4section_twist_lock",
        meta={
            "run_notes": (
                "4-section twist-lock telescoping trekking pole pair. "
                "Replaces external flip levers with internal twist-collar "
                "expander joints (CONTINUOUS) at each section mouth. "
                "Four nested telescoping stages with PRISMATIC joints probe "
                "nested twist-collar / sleeve-mouth clearance across four "
                "decreasing-diameter stages while still collapsing."
            )
        },
    )

    cork = model.material("cork_like_tan", rgba=(0.72, 0.45, 0.22, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.005, 0.006, 0.006, 1.0))
    white = model.material("white_aluminum", rgba=(0.93, 0.92, 0.88, 1.0))
    metal = model.material("brushed_silver", rgba=(0.70, 0.72, 0.73, 1.0))
    dark = model.material("dark_carbon_fiber", rgba=(0.015, 0.018, 0.017, 1.0))
    mats = (cork, black, white, metal, dark)

    # --- Upper assemblies (root): handles, grips, upper sleeves for both poles ---
    upper = model.part("upper_assemblies")
    handle_mesh = _make_cork_handle_mesh("shared_cork_handle")
    foam_mesh = _make_foam_grip_mesh("shared_foam_grip")
    for i, x in enumerate(POLE_X):
        _add_upper_pole(upper, i, x, mats, handle_mesh, foam_mesh)

    # Pair tether
    upper.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (POLE_X[0] + 0.006, -0.006, 0.672),
                    (-0.030, -0.018, 0.690),
                    (0.030, -0.018, 0.690),
                    (POLE_X[1] - 0.006, -0.006, 0.672),
                ],
                radius=0.0019,
                samples_per_segment=12,
                radial_segments=12,
                cap_ends=True,
            ),
            "pair_tether_mesh",
        ),
        material=black,
        name="pair_tether",
    )

    tip_mesh = _make_tip_mesh("shared_lower_tip")
    basket_mesh = _make_basket_mesh("shared_trekking_basket")

    # Pre-create one twist-collar mesh per junction size
    collar_meshes = [
        _make_twist_collar_mesh(cd["outer_r"], cd["h"], f"twist_collar_mesh_{j}")
        for j, cd in enumerate(COLLAR_DEFS)
    ]

    # --- Telescoping stages + twist collars in a shared loop helper ---
    for i, x in enumerate(POLE_X):
        parent_part = upper
        prev_joint_origin = Origin(xyz=(x, POLE_Y, UPPER_SLEEVE_MOUTH_Z))
        prev_stage_name = "upper"

        for j, sdef in enumerate(STAGE_DEFS):
            # -- Create the telescoping stage part --
            stage_name = f"{sdef['name']}_stage_{i}"
            stage = model.part(stage_name)

            # Main shaft tube - explicit visual creation with literal name
            if sdef["tube_name"] == "mid_tube":
                stage.visual(
                    Cylinder(radius=sdef["tube_r"], length=sdef["tube_len"]),
                    origin=Origin(xyz=(0.0, 0.0, sdef["tube_z"])),
                    material=dark,
                    name="mid_tube",
                )
            elif sdef["tube_name"] == "inner_tube":
                stage.visual(
                    Cylinder(radius=sdef["tube_r"], length=sdef["tube_len"]),
                    origin=Origin(xyz=(0.0, 0.0, sdef["tube_z"])),
                    material=dark,
                    name="inner_tube",
                )
            elif sdef["tube_name"] == "lower_tube":
                stage.visual(
                    Cylinder(radius=sdef["tube_r"], length=sdef["tube_len"]),
                    origin=Origin(xyz=(0.0, 0.0, sdef["tube_z"])),
                    material=dark,
                    name="lower_tube",
                )

            # Decorative marking band - explicit visual creation with literal name
            if sdef["band_name"] == "mid_band":
                stage.visual(
                    Cylinder(radius=sdef["band_r"], length=sdef["band_len"]),
                    origin=Origin(xyz=(0.0, 0.0, sdef["band_z"])),
                    material=black,
                    name="mid_band",
                )
            elif sdef["band_name"] == "inner_band":
                stage.visual(
                    Cylinder(radius=sdef["band_r"], length=sdef["band_len"]),
                    origin=Origin(xyz=(0.0, 0.0, sdef["band_z"])),
                    material=black,
                    name="inner_band",
                )

            # Lowest stage gets ferrule, basket, tip
            if j == len(STAGE_DEFS) - 1:
                _add_lower_stage_extras(stage, mats, tip_mesh, basket_mesh)

            # PRISMATIC telescoping joint (positive q extends downward)
            joint_name = f"{prev_stage_name}_to_{sdef['name']}_{i}"
            model.articulation(
                joint_name,
                ArticulationType.PRISMATIC,
                parent=parent_part,
                child=stage,
                origin=prev_joint_origin,
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=60.0, velocity=0.20,
                    lower=0.0, upper=sdef["slide"],
                ),
            )

            # -- Twist collar at this junction (CONTINUOUS) --
            collar_name = f"twist_collar_{i}_{j}"
            collar = model.part(collar_name)
            collar.visual(
                collar_meshes[j],
                material=black,
                name="collar_ring",
            )
            model.articulation(
                f"twist_to_{sdef['name']}_{i}",
                ArticulationType.CONTINUOUS,
                parent=parent_part,
                child=collar,
                origin=prev_joint_origin,
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(effort=2.0, velocity=5.0),
            )

            # Advance loop state for next stage
            parent_part = stage
            prev_joint_origin = Origin(xyz=(0.0, 0.0, sdef["junction_z"]))
            prev_stage_name = sdef["name"]

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_assemblies")

    for i in range(2):
        stage_names = ["mid", "inner", "lower"]
        stages = [object_model.get_part(f"{n}_stage_{i}") for n in stage_names]
        mid, inner, lower = stages

        prismatic_names = [
            f"upper_to_mid_{i}",
            f"mid_to_inner_{i}",
            f"inner_to_lower_{i}",
        ]
        prismatic_joints = [
            object_model.get_articulation(n) for n in prismatic_names
        ]
        upper_to_mid, mid_to_inner, inner_to_lower = prismatic_joints

        collar_parts = [
            object_model.get_part(f"twist_collar_{i}_{j}") for j in range(3)
        ]
        collar_joints = [
            object_model.get_articulation(f"twist_to_{n}_{i}")
            for n in stage_names
        ]

        # ---- Existence checks for the changed joint/part names ----
        for j, sname in enumerate(stage_names):
            ctx.check(
                f"twist-lock CONTINUOUS joint twist_to_{sname}_{i} exists",
                object_model.get_articulation(f"twist_to_{sname}_{i}") is not None,
                details=f"4-section twist-lock collar joint at {sname} mouth",
            )
            ctx.check(
                f"4-section telescoping stage {sname}_stage_{i} exists",
                object_model.get_part(f"{sname}_stage_{i}") is not None,
                details=f"Nested telescoping stage {j + 2} of 4",
            )

        # ---- Overlap allowances: nested telescoping tubes ----
        ctx.allow_overlap(
            upper, mid,
            elem_a=f"pole_{i}_upper_sleeve", elem_b="mid_tube",
            reason="Mid tube telescopes inside the upper sleeve (4-section twist-lock).",
        )
        ctx.allow_overlap(
            upper, mid,
            elem_a=f"pole_{i}_lower_black_band", elem_b="mid_tube",
            reason="Decorative band encircles the mid tube at the upper sleeve mouth.",
        )
        ctx.allow_overlap(
            mid, inner,
            elem_a="mid_tube", elem_b="inner_tube",
            reason="Inner tube telescopes inside the mid tube (4-section twist-lock).",
        )
        ctx.allow_overlap(
            mid, inner,
            elem_a="mid_band", elem_b="inner_tube",
            reason="Mid-stage band encircles the inner tube at the mid sleeve mouth.",
        )
        ctx.allow_overlap(
            inner, lower,
            elem_a="inner_tube", elem_b="lower_tube",
            reason="Lower tube telescopes inside the inner tube (4-section twist-lock).",
        )
        ctx.allow_overlap(
            inner, lower,
            elem_a="inner_band", elem_b="lower_tube",
            reason="Inner-stage band encircles the lower tube at the inner sleeve mouth.",
        )

        # ---- Overlap allowances: non-adjacent stage nesting (4-section collapse) ----
        ctx.allow_overlap(
            inner, upper,
            elem_a="inner_tube", elem_b=f"pole_{i}_upper_sleeve",
            reason="Inner tube nests inside the upper sleeve when fully collapsed (4-section telescoping).",
        )
        ctx.allow_overlap(
            inner, upper,
            elem_a="inner_tube", elem_b=f"pole_{i}_lower_black_band",
            reason="Inner tube passes through the upper sleeve mouth band when collapsed.",
        )
        ctx.allow_overlap(
            lower, mid,
            elem_a="lower_tube", elem_b="mid_tube",
            reason="Lower tube nests inside the mid tube when fully collapsed (4-section telescoping).",
        )
        ctx.allow_overlap(
            lower, mid,
            elem_a="lower_tube", elem_b="mid_band",
            reason="Lower tube passes through the mid tube mouth band when collapsed.",
        )
        ctx.allow_overlap(
            lower, upper,
            elem_a="lower_tube", elem_b=f"pole_{i}_upper_sleeve",
            reason="Lower tube nests inside the upper sleeve when fully collapsed (4-section telescoping).",
        )
        ctx.allow_overlap(
            lower, upper,
            elem_a="lower_tube", elem_b=f"pole_{i}_lower_black_band",
            reason="Lower tube passes through the upper sleeve mouth band when collapsed.",
        )

        # ---- Overlap allowances: twist collars ----
        collar_parents = [upper, mid, inner]
        parent_sleeve_elems = [f"pole_{i}_upper_sleeve", "mid_tube", "inner_tube"]
        child_tube_elems = ["mid_tube", "inner_tube", "lower_tube"]
        for j in range(3):
            parent = collar_parents[j]
            collar = collar_parts[j]
            child_stage = stages[j]
            
            # Twist collars are separate parts that clamp around parent tube with clearance
            ctx.allow_isolated_part(
                collar,
                reason=(
                    f"Twist collar {j} pole {i} is a separate twist-lock expander ring "
                    f"that clamps around the parent section mouth with sliding clearance."
                ),
            )
            
            ctx.allow_overlap(
                parent, collar,
                elem_a=parent_sleeve_elems[j], elem_b="collar_ring",
                reason=(
                    f"Twist collar {j} wraps around the parent tube mouth "
                    f"for the twist-lock expander mechanism."
                ),
            )
            ctx.allow_overlap(
                collar, child_stage,
                elem_a="collar_ring", elem_b=child_tube_elems[j],
                reason=(
                    f"Twist collar {j} surrounds the child tube passing "
                    f"through the section mouth."
                ),
            )
            
            # Proof: collar overlaps parent tube in Z (proves it's at the mouth)
            ctx.expect_overlap(
                collar, parent, axes="z",
                elem_a="collar_ring", elem_b=parent_sleeve_elems[j],
                min_overlap=0.005,
                name=f"twist collar {j} pole {i} overlaps parent tube mouth",
            )

        # ---- Coaxial centering (XY) across all four stages ----
        ctx.expect_within(
            mid, upper, axes="xy",
            inner_elem="mid_tube", outer_elem=f"pole_{i}_upper_sleeve",
            margin=0.004,
            name=f"mid stage {i} coaxial in upper sleeve",
        )
        ctx.expect_within(
            inner, mid, axes="xy",
            inner_elem="inner_tube", outer_elem="mid_tube",
            margin=0.004,
            name=f"inner stage {i} coaxial in mid tube",
        )
        ctx.expect_within(
            lower, inner, axes="xy",
            inner_elem="lower_tube", outer_elem="inner_tube",
            margin=0.004,
            name=f"lower stage {i} coaxial in inner tube",
        )

        # ---- Z-retention at rest (all stages inserted when collapsed) ----
        ctx.expect_overlap(
            mid, upper, axes="z",
            elem_a="mid_tube", elem_b=f"pole_{i}_upper_sleeve",
            min_overlap=0.040,
            name=f"mid stage {i} retained in upper sleeve at rest",
        )
        ctx.expect_overlap(
            inner, mid, axes="z",
            elem_a="inner_tube", elem_b="mid_tube",
            min_overlap=0.040,
            name=f"inner stage {i} retained in mid tube at rest",
        )
        ctx.expect_overlap(
            lower, inner, axes="z",
            elem_a="lower_tube", elem_b="inner_tube",
            min_overlap=0.040,
            name=f"lower stage {i} retained in inner tube at rest",
        )

        # ---- Twist collar z-overlap with child tube (proves collar placement) ----
        for j in range(3):
            child_tube_name = STAGE_DEFS[j]["tube_name"]
            ctx.expect_overlap(
                collar_parts[j], stages[j], axes="z",
                elem_a="collar_ring", elem_b=child_tube_name,
                min_overlap=0.005,
                name=(
                    f"twist collar {j} pole {i} z-overlaps child tube "
                    f"at section mouth (nested clearance verified)"
                ),
            )

        # ---- Full extension pose: all stages extended ----
        rest_positions = [ctx.part_world_position(s) for s in stages]
        full_ext_pose = {
            upper_to_mid: STAGE_DEFS[0]["slide"],
            mid_to_inner: STAGE_DEFS[1]["slide"],
            inner_to_lower: STAGE_DEFS[2]["slide"],
        }
        with ctx.pose(full_ext_pose):
            ext_positions = [ctx.part_world_position(s) for s in stages]

            # Still retained at full extension
            ctx.expect_overlap(
                mid, upper, axes="z",
                elem_a="mid_tube", elem_b=f"pole_{i}_upper_sleeve",
                min_overlap=0.010,
                name=f"mid stage {i} retained when fully extended",
            )
            ctx.expect_overlap(
                inner, mid, axes="z",
                elem_a="inner_tube", elem_b="mid_tube",
                min_overlap=0.010,
                name=f"inner stage {i} retained when fully extended",
            )
            ctx.expect_overlap(
                lower, inner, axes="z",
                elem_a="lower_tube", elem_b="inner_tube",
                min_overlap=0.010,
                name=f"lower stage {i} retained when fully extended",
            )

        # ---- Telescoping motion: each stage moves downward ----
        for j, (rest, ext) in enumerate(zip(rest_positions, ext_positions)):
            ctx.check(
                f"stage {j} ({stage_names[j]}) pole {i} telescopes downward",
                rest is not None and ext is not None and ext[2] < rest[2] - 0.02,
                details=(
                    f"rest_z={rest[2] if rest else None}, "
                    f"ext_z={ext[2] if ext else None}"
                ),
            )

    return ctx.report()


object_model = build_object_model()
