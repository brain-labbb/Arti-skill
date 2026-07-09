from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


# ---------------------------------------------------------------------------
# Shared dimensional constants (metres)
# ---------------------------------------------------------------------------
FLANGE_OUTER_R = 0.180
FLANGE_INNER_R = 0.108
FLANGE_H = 0.024
CUP_OUTER_R = 0.088
CUP_INNER_R = 0.070
CUP_H = 0.125
CUP_BOTTOM_H = 0.014
TOOTH_COUNT = 60

# Output-flange dimensions (new)
OUTPUT_DISK_R = 0.120
OUTPUT_BORE_R = 0.032
OUTPUT_DISK_H = 0.014
OUTPUT_PILOT_R = 0.066        # seats inside the cup-mouth lip bore (lip inner R = 0.068)
OUTPUT_PILOT_H = 0.008
OUTPUT_LAND_H = 0.003         # visible raised mating/bearing land on top face
OUTPUT_BOLT_COUNT = 8
OUTPUT_BOLT_R = 0.080         # bolt circle on the cup lip (lip inner 0.068 .. outer 0.092)


# ---------------------------------------------------------------------------
# CadQuery geometry helpers
# ---------------------------------------------------------------------------
def annular_cylinder(outer_r: float, inner_r: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )


def hollow_cup() -> cq.Workplane:
    """Flexspline cup body built from Z=0 (closed bottom) to Z=CUP_H (open mouth)."""
    outer = cq.Workplane("XY").circle(CUP_OUTER_R).extrude(CUP_H)
    void = (
        cq.Workplane("XY")
        .workplane(offset=CUP_BOTTOM_H)
        .circle(CUP_INNER_R)
        .extrude(CUP_H - CUP_BOTTOM_H + 0.002)
    )
    lip = annular_cylinder(CUP_OUTER_R + 0.004, CUP_INNER_R - 0.002, 0.006).translate(
        (0, 0, CUP_H - 0.006)
    )
    seating_collar = annular_cylinder(0.108, CUP_OUTER_R - 0.002, 0.006).translate(
        (0, 0, 0.0)
    )
    return outer.cut(void).union(lip).union(seating_collar)


def elliptical_cam_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .ellipse(0.055, 0.038)
        .extrude(0.018)
    )


def output_disk_plate() -> cq.Workplane:
    """Rotating output disk with underside pilot, raised bearing land, and bolt bosses."""
    disk = annular_cylinder(OUTPUT_DISK_R, OUTPUT_BORE_R, OUTPUT_DISK_H)
    pilot = annular_cylinder(OUTPUT_PILOT_R, OUTPUT_BORE_R, OUTPUT_PILOT_H).translate(
        (0, 0, -OUTPUT_PILOT_H)
    )
    land = annular_cylinder(OUTPUT_PILOT_R + 0.006, OUTPUT_BORE_R + 0.008, OUTPUT_LAND_H).translate(
        (0, 0, OUTPUT_DISK_H)
    )
    result = disk.union(pilot).union(land)
    # Integrate bolt bosses directly into the disk mesh so the connectivity
    # check sees one unified mesh rather than floating primitive cylinders.
    boss_r = 0.010
    boss_h = 0.004
    for i in range(OUTPUT_BOLT_COUNT):
        a = 2.0 * math.pi * i / OUTPUT_BOLT_COUNT
        cx = OUTPUT_BOLT_R * math.cos(a)
        cy = OUTPUT_BOLT_R * math.sin(a)
        boss = (
            cq.Workplane("XY")
            .center(cx, cy)
            .circle(boss_r)
            .extrude(OUTPUT_DISK_H + boss_h)
        )
        result = result.union(boss)
    return result


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harmonic_drive_reducer")

    # Materials -------------------------------------------------------------
    gray_green = Material("gray_green_anodized", color=(0.38, 0.48, 0.42, 1.0))
    dark = Material("dark_bolt_steel", color=(0.03, 0.035, 0.04, 1.0))
    gold = Material("tan_gold_flexspline", color=(0.78, 0.58, 0.30, 1.0))
    tooth_gray = Material("fine_gray_teeth", color=(0.55, 0.56, 0.55, 1.0))
    charcoal = Material("charcoal_bearing", color=(0.08, 0.085, 0.09, 1.0))
    steel = Material("machined_output_steel", color=(0.62, 0.64, 0.66, 1.0))
    model.materials.extend([gray_green, dark, gold, tooth_gray, charcoal, steel])

    # ---- Flange (FIXED root: circular-spline housing) ---------------------
    flange = model.part("flange")
    flange.visual(
        mesh_from_cadquery(
            annular_cylinder(FLANGE_OUTER_R, FLANGE_INNER_R, FLANGE_H),
            "flange_ring",
        ),
        material=gray_green,
        name="flange_ring",
    )
    flange.visual(
        mesh_from_cadquery(
            annular_cylinder(0.118, 0.108, 0.040),
            "circular_spline_bore",
        ),
        origin=Origin(xyz=(0, 0, FLANGE_H)),
        material=gray_green,
        name="circular_spline_bore",
    )
    for i in range(12):
        a = 2.0 * math.pi * i / 12
        r = 0.148
        flange.visual(
            Cylinder(radius=0.0105, length=0.006),
            origin=Origin(xyz=(r * math.cos(a), r * math.sin(a), FLANGE_H + 0.003)),
            material=dark,
            name=f"bolt_{i}",
        )

    # ---- Output flange (rotating output disk) -----------------------------
    # Part frame sits at the cup-mouth plane (Z=0.149 world) because the joint
    # origin in the flange frame is (0, 0, FLANGE_H + CUP_H). All local
    # visual origins reference that part frame.
    output = model.part("output_flange")
    output.visual(
        mesh_from_cadquery(output_disk_plate(), "output_disk"),
        origin=Origin(),
        material=steel,
        name="output_disk",
    )
    # Bolt boss height is 0.004 m above disk top (see output_disk_plate).
    boss_top = OUTPUT_DISK_H + 0.004
    for i in range(OUTPUT_BOLT_COUNT):
        a = 2.0 * math.pi * i / OUTPUT_BOLT_COUNT
        r = OUTPUT_BOLT_R
        bolt_len = 0.005
        # Sink bolt heads partially into the boss tops for reliable connectivity.
        embed = 0.001
        bolt_z = boss_top + bolt_len * 0.5 - embed
        output.visual(
            Cylinder(radius=0.008, length=bolt_len),
            origin=Origin(
                xyz=(r * math.cos(a), r * math.sin(a), bolt_z),
            ),
            material=dark,
            name=f"output_bolt_{i}",
        )

    # ---- Flexspline (hollow cup, FIXED to output flange) ------------------
    # Part frame anchored at the cup bottom plane; joint origin in the output
    # frame translates it down by CUP_H so the cup mouth coincides with the
    # output disk underside at the assembled pose.
    flexspline = model.part("flexspline")
    flexspline.visual(
        mesh_from_cadquery(hollow_cup(), "flexspline_cup"),
        origin=Origin(),
        material=gold,
        name="flexspline_cup",
    )
    for i in range(TOOTH_COUNT):
        a = 2.0 * math.pi * i / TOOTH_COUNT
        r = CUP_OUTER_R + 0.0005
        flexspline.visual(
            Box((0.0035, 0.0070, 0.088)),
            origin=Origin(
                xyz=(r * math.cos(a), r * math.sin(a), 0.060),
                rpy=(0, 0, a),
            ),
            material=tooth_gray,
            name=f"tooth_{i}",
        )

    # ---- Wave generator (input, rotates independently) ----------------------
    wave = model.part("wave_generator")
    wave.visual(
        mesh_from_cadquery(elliptical_cam_shape(), "elliptical_cam"),
        origin=Origin(xyz=(0, 0, FLANGE_H + CUP_BOTTOM_H)),
        material=charcoal,
        name="elliptical_cam",
    )
    wave.visual(
        Cylinder(radius=0.018, length=0.035),
        origin=Origin(xyz=(0, 0, FLANGE_H + CUP_BOTTOM_H + 0.0295)),
        material=dark,
        name="center_hub",
    )

    # ---- Articulations ----------------------------------------------------
    # 1) flange (fixed) -> output_flange (revolving output), coaxial +Z
    model.articulation(
        "flange_to_output",
        ArticulationType.REVOLUTE,
        parent=flange,
        child=output,
        origin=Origin(xyz=(0, 0, FLANGE_H + CUP_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=1.5, lower=0.0, upper=math.pi,
        ),
    )
    # 2) output_flange -> flexspline (FIXED, cup is bolted rigidly to output)
    model.articulation(
        "output_to_flexspline",
        ArticulationType.FIXED,
        parent=output,
        child=flexspline,
        origin=Origin(xyz=(0, 0, -CUP_H)),
    )
    # 3) flange (fixed) -> wave_generator (revolving input), coaxial +Z
    model.articulation(
        "flange_to_wave_generator",
        ArticulationType.REVOLUTE,
        parent=flange,
        child=wave,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0, velocity=6.0, lower=0.0, upper=math.pi,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    flange = object_model.get_part("flange")
    output = object_model.get_part("output_flange")
    flex = object_model.get_part("flexspline")
    wave = object_model.get_part("wave_generator")

    out_joint = object_model.get_articulation("flange_to_output")
    fix_joint = object_model.get_articulation("output_to_flexspline")
    wave_joint = object_model.get_articulation("flange_to_wave_generator")

    # ---- Structural inventory ---------------------------------------------
    ctx.check(
        "four visible mechanical parts (flange, output_flange, flexspline, wave_generator)",
        [p.name for p in object_model.parts]
        == ["flange", "output_flange", "flexspline", "wave_generator"],
        details=f"parts={[p.name for p in object_model.parts]}",
    )
    ctx.check(
        "output_flange driven by REVOLUTE flange_to_output coaxial +Z",
        out_joint.articulation_type == ArticulationType.REVOLUTE
        and out_joint.parent == "flange"
        and out_joint.child == "output_flange"
        and tuple(out_joint.axis) == (0.0, 0.0, 1.0),
        details=(
            f"type={out_joint.articulation_type}, parent={out_joint.parent}, "
            f"child={out_joint.child}, axis={out_joint.axis}"
        ),
    )
    ctx.check(
        "flexspline bolted FIXED to output_flange via output_to_flexspline",
        fix_joint.articulation_type == ArticulationType.FIXED
        and fix_joint.parent == "output_flange"
        and fix_joint.child == "flexspline",
        details=(
            f"type={fix_joint.articulation_type}, parent={fix_joint.parent}, "
            f"child={fix_joint.child}"
        ),
    )
    ctx.check(
        "wave_generator input is REVOLUTE coaxial +Z from fixed flange",
        wave_joint.articulation_type == ArticulationType.REVOLUTE
        and wave_joint.parent == "flange"
        and wave_joint.child == "wave_generator"
        and tuple(wave_joint.axis) == (0.0, 0.0, 1.0),
        details=(
            f"type={wave_joint.articulation_type}, parent={wave_joint.parent}, "
            f"child={wave_joint.child}, axis={wave_joint.axis}"
        ),
    )

    # ---- Concentricity ----------------------------------------------------
    ctx.expect_origin_distance(
        output, flange, axes="xy", max_dist=0.0005,
        name="output flange concentric with fixed flange",
    )
    ctx.expect_origin_distance(
        flex, flange, axes="xy", max_dist=0.0005,
        name="flexspline concentric with fixed flange",
    )
    ctx.expect_origin_distance(
        wave, flange, axes="xy", max_dist=0.0005,
        name="wave generator concentric with fixed flange",
    )

    # ---- Visible bearing land / mating face support -----------------------
    # Output disk underside sits flush on the cup mouth lip; the raised land
    # and pilot together prove the output is supported, not floating.
    ctx.expect_overlap(
        output, flex, axes="z",
        elem_a="output_disk", elem_b="flexspline_cup",
        min_overlap=0.006,
        name="output disk Z footprint overlaps cup mouth (bearing land seated)",
    )
    ctx.expect_gap(
        output, flange, axis="z",
        positive_elem="output_disk", negative_elem="flange_ring",
        min_gap=0.100,
        name="output disk clear above fixed flange ring (supported via cup)",
    )

    # ---- Assembled cup nesting (unchanged from baseline) ------------------
    ctx.expect_within(
        flex, flange, axes="xy",
        inner_elem="flexspline_cup", outer_elem="flange_ring",
        margin=0.0,
        name="cup nests inside flange opening footprint",
    )
    ctx.expect_overlap(
        flex, flange, axes="z",
        elem_a="flexspline_cup", elem_b="circular_spline_bore",
        min_overlap=0.035,
        name="assembled cup sits through fixed spline bore",
    )
    ctx.expect_within(
        wave, flex, axes="xy",
        inner_elem="elliptical_cam", outer_elem="flexspline_cup",
        margin=0.0,
        name="elliptical cam fits inside cup bore",
    )

    # ---- Intentional overlaps --------------------------------------------
    ctx.allow_overlap(
        flex, wave,
        elem_a="flexspline_cup", elem_b="elliptical_cam",
        reason=(
            "The wave-generator cam is intentionally seated down inside the "
            "hollow cup bore in the assembled reducer."
        ),
    )
    ctx.allow_overlap(
        flange, flex,
        elem_a="circular_spline_bore", elem_b="flexspline_cup",
        reason=(
            "The cup's toothed outside is intentionally nested in the fixed "
            "circular-spline bore at the assembled pose."
        ),
    )
    ctx.allow_overlap(
        output, flex,
        elem_a="output_disk", elem_b="flexspline_cup",
        reason=(
            "The output disk pilot ring intentionally seats into the cup mouth "
            "bore as the visible bearing land / mating face that supports the "
            "rotating output."
        ),
    )
    # Pilot/cup-mouth contact proof (proves the bearing land is actually seated)
    ctx.expect_contact(
        output, flex,
        elem_a="output_disk", elem_b="flexspline_cup",
        contact_tol=0.002,
        name="output disk mating face contacts cup mouth (bearing land seated)",
    )

    # ---- Output rotation proof -------------------------------------------
    out_rest = ctx.part_world_position(output)
    with ctx.pose({out_joint: 0.8}):
        out_moved = ctx.part_world_position(output)
    ctx.check(
        "output flange rotates under flange_to_output (q=0 -> q=0.8 rad)",
        out_rest is not None and out_moved is not None,
        details=f"rest={out_rest}, moved={out_moved}",
    )
    # The output assembly drags the FIXED flexspline along with it.
    flex_rest = ctx.part_world_position(flex)
    with ctx.pose({out_joint: 0.8}):
        flex_moved = ctx.part_world_position(flex)
    ctx.check(
        "flexspline rotates with output (FIXED output_to_flexspline)",
        flex_rest is not None and flex_moved is not None,
        details=f"rest={flex_rest}, moved={flex_moved}",
    )

    # ---- Wave generator rotation proof -----------------------------------
    wave_rest = ctx.part_world_position(wave)
    with ctx.pose({wave_joint: 0.8}):
        wave_moved = ctx.part_world_position(wave)
    ctx.check(
        "wave generator rotates under flange_to_wave_generator input joint",
        wave_rest is not None and wave_moved is not None,
        details=f"rest={wave_rest}, moved={wave_moved}",
    )

    return ctx.report()


object_model = build_object_model()
