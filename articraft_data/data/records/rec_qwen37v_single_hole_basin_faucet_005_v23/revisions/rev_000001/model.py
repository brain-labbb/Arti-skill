from __future__ import annotations

"""Modern squared single-hole basin faucet with pull-up drain rod.

A sharply squared monobloc body (~0.20 m tall) on a square base flange.
A rectangular channel spout projects forward from the upper body with a
real hollow open outlet at the mouth. A separate circular aerator insert
sits at the spout mouth. A flat lever on top controls flow via a revolute
joint. A pull-up drain rod behind the body slides vertically via a
prismatic joint.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters). World frame: +X forward (spout direction), +Z up,
# +Y to the left when facing the spout.
# ---------------------------------------------------------------------------
FLANGE_W = 0.068
FLANGE_H = 0.012
BODY_W = 0.048  # square cross-section
BODY_H = 0.180
BODY_Z0 = FLANGE_H  # body column base

SPOUT_W = 0.036
SPOUT_H = 0.028
SPOUT_L = 0.120
SPOUT_WALL = 0.005
SPOUT_ROOT_Z = 0.155  # spout centerline height
SPOUT_EMBED = 0.005  # root embedded inside body for visual continuity

AERATOR_R = 0.0092  # press-fits into spout inner height (0.018) with slight seating embed
AERATOR_T = 0.004

STEM_R = 0.009
STEM_H = 0.016
LEVER_L = 0.110
LEVER_W = 0.016
LEVER_H = 0.006

ROD_R = 0.003
ROD_LEN = 0.095
KNOB_R = 0.008
KNOB_H = 0.010
BRACKET_R = 0.007
BRACKET_L = 0.016
GUIDE_Z = 0.125
ROD_Y = -(BODY_W / 2.0 + BRACKET_L / 2.0)  # rod passes through bracket center
ROD_Z0 = 0.065  # rod bottom at rest

LIFT_RANGE = math.radians(40.0)
ROD_TRAVEL = 0.040

BODY_TOP_Z = BODY_Z0 + BODY_H  # 0.192


def _build_spout() -> cq.Workplane:
    """Hollow rectangular tube spout, open at the mouth (+X) end.

    Built centered at the local origin. The root wall (closed end) is at
    the -X face; the mouth (open end) is at +X.
    """
    outer = cq.Workplane("XY").box(SPOUT_L, SPOUT_W, SPOUT_H)
    # Inner cutout shifted toward +X so the mouth end is fully open
    inner = (
        cq.Workplane("XY")
        .center(SPOUT_WALL / 2.0, 0.0)
        .box(
            SPOUT_L - SPOUT_WALL,
            SPOUT_W - 2.0 * SPOUT_WALL,
            SPOUT_H - 2.0 * SPOUT_WALL,
        )
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("chrome", rgba=(0.85, 0.86, 0.88, 1.0))
    model.material("dark_chrome", rgba=(0.40, 0.41, 0.43, 1.0))
    model.material("rubber_black", rgba=(0.10, 0.10, 0.10, 1.0))

    # ----------------------------------------------------------- body (root)
    body = model.part("faucet_body")

    # Square base flange
    body.visual(
        Box((FLANGE_W, FLANGE_W, FLANGE_H)),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )

    # Squared monobloc column
    body.visual(
        Box((BODY_W, BODY_W, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + BODY_H / 2.0)),
        material="brushed_steel",
        name="body_column",
    )

    # Hollow rectangular spout tube
    spout_origin_x = BODY_W / 2.0 - SPOUT_EMBED + SPOUT_L / 2.0
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_tube"),
        origin=Origin(xyz=(spout_origin_x, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_tube",
    )

    # Drain rod guide bracket (horizontal cylinder on body back face)
    bracket_center_y = -(BODY_W / 2.0 + BRACKET_L / 2.0)
    body.visual(
        Cylinder(radius=BRACKET_R, length=BRACKET_L),
        origin=Origin(
            xyz=(0.0, bracket_center_y, GUIDE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="brushed_steel",
        name="guide_bracket",
    )

    # ------------------------------------------------------- lever handle
    handle = model.part("lever_handle")

    # Pivot stem rising from body top
    handle.visual(
        Cylinder(radius=STEM_R, length=STEM_H),
        origin=Origin(xyz=(0.0, 0.0, STEM_H / 2.0)),
        material="chrome",
        name="pivot_stem",
    )

    # Flat rectangular lever bar extending forward from stem top
    handle.visual(
        Box((LEVER_L, LEVER_W, LEVER_H)),
        origin=Origin(xyz=(LEVER_L / 2.0, 0.0, STEM_H + LEVER_H / 2.0)),
        material="chrome",
        name="lever_bar",
    )

    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        # -Y so positive q lifts the forward (+X) lever tip upward
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=-LIFT_RANGE, upper=LIFT_RANGE
        ),
    )

    # --------------------------------------------------------- drain rod
    drain = model.part("drain_rod")

    # Thin vertical rod shaft
    drain.visual(
        Cylinder(radius=ROD_R, length=ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, ROD_LEN / 2.0)),
        material="chrome",
        name="rod_shaft",
    )

    # Pull knob on top of rod
    drain.visual(
        Cylinder(radius=KNOB_R, length=KNOB_H),
        origin=Origin(xyz=(0.0, 0.0, ROD_LEN + KNOB_H / 2.0)),
        material="dark_chrome",
        name="pull_knob",
    )

    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain,
        origin=Origin(xyz=(0.0, ROD_Y, ROD_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.1, lower=0.0, upper=ROD_TRAVEL
        ),
    )

    # ----------------------------------------- aerator insert (fixed)
    aerator = model.part("aerator_insert")

    # Thin circular disc at the spout mouth
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_T),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber_black",
        name="aerator_disc",
    )

    # Position aerator at the spout mouth
    mouth_x = BODY_W / 2.0 - SPOUT_EMBED + SPOUT_L - AERATOR_T / 2.0
    model.articulation(
        "aerator_mount",
        ArticulationType.FIXED,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(mouth_x, 0.0, SPOUT_ROOT_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    handle = object_model.get_part("lever_handle")
    drain = object_model.get_part("drain_rod")
    aerator = object_model.get_part("aerator_insert")

    lift = object_model.get_articulation("handle_lift")
    slide = object_model.get_articulation("drain_slide")

    spout = body.get_visual("spout_tube")
    col = body.get_visual("body_column")
    bar = handle.get_visual("lever_bar")
    rod_shaft = drain.get_visual("rod_shaft")
    aerator_disc = aerator.get_visual("aerator_disc")
    bracket = body.get_visual("guide_bracket")

    # --- intentional overlaps -------------------------------------------
    ctx.allow_overlap(
        handle,
        body,
        reason="pivot stem seats into body top (captured pivot)",
    )
    ctx.allow_overlap(
        drain,
        body,
        elem_a="rod_shaft",
        elem_b="guide_bracket",
        reason="drain rod passes through guide bracket (captured shaft)",
    )
    ctx.allow_overlap(
        aerator,
        body,
        elem_a="aerator_disc",
        elem_b="spout_tube",
        reason="aerator press-fits into spout mouth (seated compression, ~0.2 mm embed)",
    )

    # --- squared body form -----------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"base flange must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        aabb is not None and 0.190 < aabb[1][2] < 0.225,
        f"body top should be ~0.20 m, got {aabb}",
    )

    # Squared column: width and depth should match BODY_W (not circular)
    col_aabb = ctx.part_element_world_aabb(body, elem=col)
    ctx.check(
        "squared_body_column",
        col_aabb is not None
        and abs((col_aabb[1][0] - col_aabb[0][0]) - BODY_W) < 1e-3
        and abs((col_aabb[1][1] - col_aabb[0][1]) - BODY_W) < 1e-3,
        f"body column should be squared {BODY_W}x{BODY_W}, got {col_aabb}",
    )

    # --- spout projects forward ------------------------------------------
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > BODY_W / 2.0 + 0.100,
        f"spout should project >0.10 m forward from body, got {spout_aabb}",
    )

    # --- hollow outlet: aerator is separate circular insert ---------------
    aer_aabb = ctx.part_element_world_aabb(aerator, elem=aerator_disc)
    ctx.check(
        "aerator_at_spout_mouth",
        aer_aabb is not None and aer_aabb[1][0] > BODY_W / 2.0 + 0.100,
        f"aerator should be at spout mouth (far forward), got {aer_aabb}",
    )
    # Aerator disc is circular (Y and Z extents should be ~2*AERATOR_R)
    ctx.check(
        "aerator_is_circular",
        aer_aabb is not None
        and abs((aer_aabb[1][1] - aer_aabb[0][1]) - 2.0 * AERATOR_R) < 1e-3
        and abs((aer_aabb[1][2] - aer_aabb[0][2]) - 2.0 * AERATOR_R) < 1e-3,
        f"aerator should be circular with diameter {2*AERATOR_R}, got {aer_aabb}",
    )
    # Aerator within spout opening in YZ
    ctx.expect_within(
        aerator,
        body,
        axes="yz",
        inner_elem=aerator_disc,
        outer_elem=spout,
        margin=0.005,
        name="aerator_within_spout_mouth",
    )
    # Aerator is seated (contacting the inner walls)
    ctx.expect_contact(
        aerator,
        body,
        elem_a=aerator_disc,
        elem_b=spout,
        contact_tol=0.001,
        name="aerator_seated_in_spout",
    )

    # --- drain rod behind body -------------------------------------------
    rod_aabb = ctx.part_element_world_aabb(drain, elem=rod_shaft)
    ctx.check(
        "drain_rod_behind_body",
        rod_aabb is not None
        and (rod_aabb[0][1] + rod_aabb[1][1]) / 2.0 < -BODY_W / 2.0 + 0.010,
        f"drain rod center should be behind body (-Y), got {rod_aabb}",
    )
    # Guide bracket contacts the rod
    ctx.expect_contact(
        body,
        drain,
        elem_a=bracket,
        elem_b=rod_shaft,
        contact_tol=0.003,
        name="bracket_supports_rod",
    )

    # --- joint plan: revolute for handle ---------------------------------
    ctx.check(
        "lift_is_revolute",
        lift.articulation_type == ArticulationType.REVOLUTE,
        "handle_lift must be REVOLUTE",
    )
    ctx.check(
        "lift_axis_sideways",
        abs(lift.axis[1]) == 1.0 and lift.axis[0] == 0.0 and lift.axis[2] == 0.0,
        f"lift axis should be ±Y, got {lift.axis}",
    )
    ctx.check(
        "lift_range_pm40",
        lift.motion_limits is not None
        and abs(lift.motion_limits.lower + LIFT_RANGE) < 1e-6
        and abs(lift.motion_limits.upper - LIFT_RANGE) < 1e-6,
        "lift range must be ±40°",
    )

    # --- joint plan: prismatic for drain rod ----------------------------
    ctx.check(
        "slide_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        "drain_slide must be PRISMATIC",
    )
    ctx.check(
        "slide_axis_vertical",
        abs(slide.axis[2]) == 1.0 and slide.axis[0] == 0.0 and slide.axis[1] == 0.0,
        f"slide axis should be ±Z, got {slide.axis}",
    )
    ctx.check(
        "slide_range_0_to_40mm",
        slide.motion_limits is not None
        and abs(slide.motion_limits.lower) < 1e-6
        and abs(slide.motion_limits.upper - ROD_TRAVEL) < 1e-6,
        "slide range must be 0..0.040 m",
    )

    # --- motion proofs --------------------------------------------------
    # Handle lift: positive q raises lever tip
    with ctx.pose({lift: LIFT_RANGE}):
        up_aabb = ctx.part_element_world_aabb(handle, elem=bar)
        ctx.check(
            "lift_up_raises_lever",
            up_aabb is not None and up_aabb[1][2] > BODY_TOP_Z + 0.035,
            f"at +40° lever tip should rise above body top, got {up_aabb}",
        )

    # Drain rod: positive q slides rod upward
    rest_rod = ctx.part_element_world_aabb(drain, elem=rod_shaft)
    with ctx.pose({slide: ROD_TRAVEL}):
        raised_rod = ctx.part_element_world_aabb(drain, elem=rod_shaft)
        ctx.check(
            "drain_rod_slides_up",
            rest_rod is not None
            and raised_rod is not None
            and raised_rod[0][2] > rest_rod[0][2] + 0.030,
            f"rod should move up ~0.04 m: rest={rest_rod}, raised={raised_rod}",
        )

    # Handle seats on body top
    ctx.expect_contact(body, handle, name="handle_seats_on_body")

    return ctx.report()


object_model = build_object_model()
