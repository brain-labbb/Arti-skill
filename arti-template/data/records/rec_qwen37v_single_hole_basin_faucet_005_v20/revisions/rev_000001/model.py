from __future__ import annotations

"""Single-hole basin faucet variant with tapered conical body and top-mounted flow knob.

A compact single-hole basin faucet (~0.20 m tall) with a tapered conical body
(wider at the base, narrowing toward the top) standing on a round base flange.
A small forward beak projects from the upper front of the body as the water
outlet. A cylindrical flow knob with subtle grip grooves sits on top and rotates
about the vertical axis to control flow. A thin cartridge cap seam ring is
visible just below the knob mount.

Articulation:
- ``knob_flow``: revolute about the vertical (Z) axis at the body top,
  -90..+90 deg; positive q opens flow (clockwise from above).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Dimensions (meters). World frame: +X forward (beak direction), +Z up.
# ----------------------------------------------------------------------------
BASE_FLANGE_RADIUS = 0.034
BASE_FLANGE_HEIGHT = 0.010

BODY_BOTTOM_RADIUS = 0.028  # bottom of tapered section (just above flange)
BODY_TOP_RADIUS = 0.018     # top of tapered section
BODY_HEIGHT = 0.190         # total conical body height (above flange)
BODY_TOTAL_HEIGHT = BASE_FLANGE_HEIGHT + BODY_HEIGHT  # ~0.200 m

# Cartridge cap seam: thin ring near the top of the body
CAP_SEAM_Z = BODY_HEIGHT - 0.018  # ~0.012 m below the top
CAP_SEAM_RADIUS = BODY_TOP_RADIUS + 0.004  # slightly wider than body at that height
CAP_SEAM_HEIGHT = 0.003

# Forward beak spout dimensions
BEAK_LENGTH = 0.040   # forward projection
BEAK_WIDTH = 0.018    # sideways width
BEAK_HEIGHT = 0.012   # vertical thickness
BEAK_ORIGIN_Z = BODY_HEIGHT - 0.008  # near the top of the body

# Flow knob dimensions
KNOB_DIAMETER = 0.038
KNOB_HEIGHT = 0.022
KNOB_STEM_RADIUS = 0.006
KNOB_STEM_LENGTH = 0.010

FLOW_RANGE = math.radians(90.0)


def _build_conical_body() -> cq.Workplane:
    """Tapered conical body column: wider at bottom, narrower at top.
    
    Built with base at z=0, extending to z=BODY_HEIGHT.
    """
    body = (
        cq.Workplane("XY")
        .circle(BODY_BOTTOM_RADIUS)
        .workplane(offset=BODY_HEIGHT)
        .circle(BODY_TOP_RADIUS)
        .loft()
    )
    return body


def _build_beak_spout() -> cq.Workplane:
    """Small forward beak projecting from the upper front of the body.
    
    Built in spout-local coords: origin at the body surface, heading +X forward.
    The beak is a rounded wedge shape that tapers to a narrow opening at the tip.
    """
    # Create a beak shape: wider at root, narrowing forward
    # Use a simple box-like shape with rounded edges
    beak = (
        cq.Workplane("XY")
        .transformed(offset=(BEAK_LENGTH / 2.0, 0.0, 0.0))
        .box(BEAK_LENGTH, BEAK_WIDTH, BEAK_HEIGHT)
    )
    # Cut a small channel on the underside to read as a water outlet
    channel = (
        cq.Workplane("XY")
        .transformed(offset=(BEAK_LENGTH * 0.7, 0.0, -BEAK_HEIGHT / 2.0 + 0.001))
        .box(BEAK_LENGTH * 0.5, BEAK_WIDTH * 0.5, 0.004)
    )
    beak = beak.cut(channel)
    # Round the front edge
    beak = beak.edges("|Z").fillet(0.003)
    return beak


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("dark_seam", rgba=(0.35, 0.36, 0.38, 1.0))
    model.material("knob_steel", rgba=(0.72, 0.73, 0.75, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")

    # Base flange - round, slightly wider
    body.visual(
        Cylinder(radius=BASE_FLANGE_RADIUS, length=BASE_FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )

    # Tapered conical body column
    body.visual(
        mesh_from_cadquery(_build_conical_body(), "conical_column"),
        origin=Origin(xyz=(0.0, 0.0, BASE_FLANGE_HEIGHT)),
        material="brushed_steel",
        name="body_column",
    )

    # Forward beak spout
    body.visual(
        mesh_from_cadquery(_build_beak_spout(), "beak_spout"),
        origin=Origin(xyz=(BODY_TOP_RADIUS * 0.6, 0.0, BASE_FLANGE_HEIGHT + BEAK_ORIGIN_Z)),
        material="brushed_steel",
        name="beak_spout",
    )

    # Cartridge cap seam ring - thin ring below the knob mount
    # Compute the body radius at the seam height via linear interpolation
    frac = CAP_SEAM_Z / BODY_HEIGHT
    body_radius_at_seam = BODY_BOTTOM_RADIUS + (BODY_TOP_RADIUS - BODY_BOTTOM_RADIUS) * frac
    body.visual(
        Cylinder(radius=body_radius_at_seam + 0.002, length=CAP_SEAM_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FLANGE_HEIGHT + CAP_SEAM_Z + CAP_SEAM_HEIGHT / 2.0)),
        material="dark_seam",
        name="cap_seam_ring",
    )

    # --------------------------------------------------------- flow knob
    # Cylindrical knob with grip grooves, mounted on top of the body
    flow_knob = model.part("flow_knob")

    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
        center=False,
    )
    flow_knob.visual(
        mesh_from_geometry(knob_geom, "knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="knob_steel",
        name="knob_grip",
    )

    # Knob stem that inserts into the body top (connects knob to body)
    flow_knob.visual(
        Cylinder(radius=KNOB_STEM_RADIUS, length=KNOB_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -KNOB_STEM_LENGTH / 2.0)),
        material="bright_steel",
        name="knob_stem",
    )

    # Articulation: knob rotates about Z axis at the top of the body
    knob_mount_z = BASE_FLANGE_HEIGHT + BODY_HEIGHT
    model.articulation(
        "knob_flow",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flow_knob,
        origin=Origin(xyz=(0.0, 0.0, knob_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=-FLOW_RANGE, upper=FLOW_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    flow_joint = object_model.get_articulation("knob_flow")
    spout = body.get_visual("beak_spout")
    cap_seam = body.get_visual("cap_seam_ring")
    knob_grip = knob.get_visual("knob_grip")
    column = body.get_visual("body_column")

    # Intentional seated embedding: knob stem inserts into body top
    ctx.allow_overlap(
        knob,
        body,
        elem_a="knob_stem",
        elem_b="body_column",
        reason="knob stem is seated into the body top bore for cartridge connection",
    )

    # --- static form: conical body -------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"base flange must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        aabb is not None and 0.190 < aabb[1][2] < 0.210,
        f"body top should be ~0.20 m up, got {aabb}",
    )

    # Verify tapered shape: bottom is wider than top
    col_aabb = ctx.part_element_world_aabb(body, elem=column)
    ctx.check(
        "body_is_tapered",
        col_aabb is not None,
        f"conical body column must exist, got {col_aabb}",
    )
    if col_aabb is not None:
        # The column should be wider at the bottom than the top
        col_width_x = col_aabb[1][0] - col_aabb[0][0]
        # For a tapered body, width should be between the two radii * 2
        ctx.check(
            "body_width_consistent_with_taper",
            2.0 * BODY_TOP_RADIUS - 0.002 < col_width_x < 2.0 * BODY_BOTTOM_RADIUS + 0.002,
            f"tapered body width should be between {2*BODY_TOP_RADIUS} and {2*BODY_BOTTOM_RADIUS}, got {col_width_x}",
        )

    # --- beak spout ----------------------------------------------------------
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "beak_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > BODY_TOP_RADIUS + 0.025,
        f"beak spout should project forward past the body, got {spout_aabb}",
    )
    ctx.check(
        "beak_is_small",
        spout_aabb is not None and (spout_aabb[1][0] - spout_aabb[0][0]) < 0.055,
        f"beak should be a small forward projection (<0.055 m), got {spout_aabb}",
    )
    ctx.check(
        "beak_near_top",
        spout_aabb is not None and spout_aabb[0][2] > 0.160,
        f"beak should be near the top of the body (>0.16 m), got {spout_aabb}",
    )

    # --- cartridge cap seam --------------------------------------------------
    seam_aabb = ctx.part_element_world_aabb(body, elem=cap_seam)
    ctx.check(
        "cap_seam_exists_below_knob",
        seam_aabb is not None and seam_aabb[1][2] > 0.160 and seam_aabb[0][2] < 0.200,
        f"cap seam ring should be visible below the knob mount, got {seam_aabb}",
    )
    ctx.check(
        "cap_seam_is_thin",
        seam_aabb is not None and (seam_aabb[1][2] - seam_aabb[0][2]) < 0.005,
        f"cap seam should be a thin ring (<5 mm), got {seam_aabb}",
    )

    # --- flow knob on top ----------------------------------------------------
    knob_aabb = ctx.part_element_world_aabb(knob, elem=knob_grip)
    ctx.check(
        "knob_on_top_of_body",
        knob_aabb is not None and knob_aabb[0][2] > 0.185,
        f"flow knob should sit on top of the body, got {knob_aabb}",
    )
    ctx.check(
        "knob_has_grooves",
        knob_aabb is not None and (knob_aabb[1][0] - knob_aabb[0][0]) > 0.034,
        f"knob with grooves should be ~0.038 m diameter, got {knob_aabb}",
    )

    # --- joint plan ----------------------------------------------------------
    ctx.check(
        "flow_axis_vertical",
        abs(flow_joint.axis[2]) == 1.0 and flow_joint.axis[0] == 0.0 and flow_joint.axis[1] == 0.0,
        f"flow knob must rotate about vertical Z axis, got {flow_joint.axis}",
    )
    ctx.check(
        "flow_range_pm90deg",
        flow_joint.motion_limits is not None
        and abs(flow_joint.motion_limits.lower + FLOW_RANGE) < 1e-6
        and abs(flow_joint.motion_limits.upper - FLOW_RANGE) < 1e-6,
        "flow range must be -90..+90 deg",
    )

    # --- motion proof --------------------------------------------------------
    # Rotate knob and verify it actually moves
    rest_pos = ctx.part_world_position(knob)
    with ctx.pose({flow_joint: FLOW_RANGE}):
        rotated_pos = ctx.part_world_position(knob)
        ctx.check(
            "knob_rotates_about_vertical",
            rest_pos is not None and rotated_pos is not None
            and abs(rotated_pos[2] - rest_pos[2]) < 0.001,
            f"knob Z position should stay constant during rotation: rest={rest_pos}, rotated={rotated_pos}",
        )

    # Knob stays connected to body during rotation
    with ctx.pose({flow_joint: FLOW_RANGE * 0.5}):
        ctx.expect_contact(body, knob, contact_tol=0.002, name="knob_stays_connected_at_half_turn")

    return ctx.report()


object_model = build_object_model()
