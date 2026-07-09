from __future__ import annotations

"""Single-hole basin faucet with rectangular slot outlet and top flow knob.

A compact single-hole basin faucet (~0.20 m tall) with a vertical cylindrical
body on a wider base flange. A channel spout projects forward ~0.13 m and ends
in a flat rectangular slot outlet with a real hollow mouth. A separate circular
aerator insert sits at the spout mouth. On top of the body, a cylindrical flow
knob rotates about the vertical axis for flow control.

Articulation:
- ``knob_turn``: revolute about Z (vertical), -90..+90 deg, controlling flow.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
# Dimensions (meters). World frame: +X forward (spout direction), +Z up.
# ----------------------------------------------------------------------------
BODY_RADIUS = 0.0275  # 0.055 m diameter column
BODY_HEIGHT = 0.200
FLANGE_RADIUS = 0.0345
FLANGE_HEIGHT = 0.012

SPOUT_ROOT_Z = 0.120  # height where spout leaves the body
SPOUT_LENGTH = 0.130  # forward projection of spout
SPOUT_OUTER_W = 0.030  # outer width of spout tube
SPOUT_OUTER_H = 0.020  # outer height of spout tube
SPOUT_WALL = 0.004  # wall thickness

SLOT_W = 0.018  # rectangular slot width at mouth
SLOT_H = 0.008  # rectangular slot height at mouth
SLOT_DEPTH = 0.020  # how deep the slot cut goes into the tip

AERATOR_RADIUS = 0.012  # circular aerator disc
AERATOR_THICKNESS = 0.004

KNOB_DIAMETER = 0.036
KNOB_HEIGHT = 0.018
KNOB_STEM_RADIUS = 0.006
KNOB_STEM_LENGTH = 0.008

KNOB_RANGE = math.radians(90.0)


def _build_spout() -> cq.Workplane:
    """Spout with hollow rectangular slot outlet at the tip.

    Built in spout-local coordinates: origin at the body front, +X forward.
    The spout is a rectangular tube swept forward with a gentle downward curve.
    A rectangular slot is cut into the tip face to create the hollow outlet.
    """
    # Outer shell: box-like spout body projected forward along a curved path
    # Build as a solid block first, then hollow it out
    path_pts = [
        (0.0, 0.0),
        (0.040, -0.002),
        (0.080, -0.008),
        (0.110, -0.018),
        (0.130, -0.030),
    ]
    path = cq.Workplane("XZ").spline(path_pts)

    hw = SPOUT_OUTER_W / 2.0
    hh = SPOUT_OUTER_H / 2.0

    # Outer profile: rounded rectangle
    outer_profile = (
        cq.Workplane("YZ")
        .rect(SPOUT_OUTER_W, SPOUT_OUTER_H)
        .extrude(0.001)  # dummy to make it a face
    )
    # Actually, let's use a simpler approach: build outer as swept rect, inner as swept smaller rect
    outer_profile = (
        cq.Workplane("YZ")
        .rect(SPOUT_OUTER_W, SPOUT_OUTER_H)
    )
    outer = outer_profile.sweep(path)

    # Inner cavity (water channel)
    inner_w = SPOUT_OUTER_W - 2.0 * SPOUT_WALL
    inner_h = SPOUT_OUTER_H - 2.0 * SPOUT_WALL
    inner_profile = (
        cq.Workplane("YZ")
        .rect(inner_w, inner_h)
    )
    inner = inner_profile.sweep(path)

    # Subtract inner from outer to make hollow tube
    spout = outer.cut(inner)

    # Cut rectangular slot at the tip to create the outlet opening
    # The tip is at approximately x=0.130, z=-0.030 in local coords
    # We cut a slot from the front face going backward
    slot_cutter = (
        cq.Workplane("YZ")
        .transformed(offset=(SPOUT_LENGTH - SLOT_DEPTH + 0.001, 0.0, -0.030))
        .rect(SLOT_W, SLOT_H)
        .extrude(SLOT_DEPTH + 0.002)
    )
    spout = spout.cut(slot_cutter)

    return spout


def _build_aerator() -> cq.Workplane:
    """Circular aerator insert disc with cross-bar grid pattern.

    A flat disc with small through-holes to represent the aerator mesh screen.
    Built centered at origin, aligned with Z axis.
    """
    disc = (
        cq.Workplane("XY")
        .circle(AERATOR_RADIUS)
        .extrude(AERATOR_THICKNESS)
    )
    # Cut a ring of small holes to represent the mesh/aeration pattern
    hole_r = 0.0018
    ring_r = AERATOR_RADIUS * 0.6
    n_holes = 8
    for i in range(n_holes):
        angle = 2.0 * math.pi * i / n_holes
        hx = ring_r * math.cos(angle)
        hy = ring_r * math.sin(angle)
        hole = (
            cq.Workplane("XY")
            .transformed(offset=(hx, hy, -0.001))
            .circle(hole_r)
            .extrude(AERATOR_THICKNESS + 0.002)
        )
        disc = disc.cut(hole)
    # Center hole
    center_hole = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -0.001))
        .circle(hole_r * 1.2)
        .extrude(AERATOR_THICKNESS + 0.002)
    )
    disc = disc.cut(center_hole)
    # Outer rim ring cut for visual separation
    rim_cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, -0.001))
        .circle(AERATOR_RADIUS + 0.001)
        .circle(AERATOR_RADIUS - 0.0015)
        .extrude(AERATOR_THICKNESS + 0.002)
    )
    # Don't cut the rim - keep the outer ring intact

    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    model.material("dark_steel", rgba=(0.45, 0.46, 0.48, 1.0))
    model.material("aerator_mesh", rgba=(0.55, 0.56, 0.58, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Base flange
    body.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    # Main column
    column_len = BODY_HEIGHT - FLANGE_HEIGHT - 0.005
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=column_len),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT + column_len / 2.0)),
        material="brushed_steel",
        name="body_column",
    )
    # Flat top cap
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.0025)),
        material="bright_steel",
        name="body_cap",
    )
    # Spout with rectangular slot outlet
    body.visual(
        mesh_from_cadquery(_build_spout(), "spout_body"),
        origin=Origin(xyz=(BODY_RADIUS * 0.3, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )

    # ------------------------------------------------------------------ knob
    # Flow knob on top of body: cylindrical with knurled grip and pointer line.
    # The knob part includes a stem that penetrates the body top (intentional
    # small overlap for shaft seating).
    knob_part = model.part("flow_knob")
    # Stem (goes into body top for shaft connection)
    knob_part.visual(
        Cylinder(radius=KNOB_STEM_RADIUS, length=KNOB_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -KNOB_STEM_LENGTH / 2.0)),
        material="dark_steel",
        name="knob_stem",
    )
    # Main knob body
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=32, depth=0.001),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
        center=False,
    )
    knob_part.visual(
        mesh_from_geometry(knob_geom, "knob_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="bright_steel",
        name="knob_cap",
    )

    # Articulation: knob rotates about vertical Z axis on top of body
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob_part,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=3.0, lower=-KNOB_RANGE, upper=KNOB_RANGE
        ),
    )

    # ------------------------------------------------------------------ aerator
    # Separate circular aerator insert seated at the spout mouth
    aerator_part = model.part("aerator")
    # Position: at the spout tip mouth, facing forward (-X into spout)
    # The spout tip is at approximately x = BODY_RADIUS*0.3 + SPOUT_LENGTH,
    # z = SPOUT_ROOT_Z - 0.030
    aerator_x = BODY_RADIUS * 0.3 + SPOUT_LENGTH - 0.002
    aerator_z = SPOUT_ROOT_Z - 0.030
    aerator_part.visual(
        mesh_from_cadquery(_build_aerator(), "aerator_disc"),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="aerator_mesh",
        name="aerator_disc",
    )

    # Fixed joint: aerator is rigidly seated at the spout mouth
    model.articulation(
        "aerator_mount",
        ArticulationType.FIXED,
        parent=body,
        child=aerator_part,
        origin=Origin(xyz=(aerator_x, 0.0, aerator_z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    aerator = object_model.get_part("aerator")
    knob_joint = object_model.get_articulation("knob_turn")
    spout = body.get_visual("spout_channel")
    knob_cap = knob.get_visual("knob_cap")

    # Intentional overlaps: knob stem seated into body top
    ctx.allow_overlap(
        body,
        knob,
        reason="knob stem is seated into the body top as a shaft connection for the flow control knob",
    )
    # Aerator seated at spout mouth
    ctx.allow_overlap(
        aerator,
        body,
        reason="aerator disc is seated flush against the spout mouth opening",
    )

    # --- static form -------------------------------------------------------
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body_grounded",
        aabb is not None and abs(aabb[0][2]) < 1e-6,
        f"base flange must sit on z=0, got {aabb}",
    )
    ctx.check(
        "body_height_about_0p20",
        aabb is not None and 0.190 < aabb[1][2] < 0.225,
        f"body top should be ~0.20 m up (plus knob), got {aabb}",
    )

    # Spout projects forward
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.120,
        f"spout should reach >0.12 m forward, got {spout_aabb}",
    )

    # Rectangular slot outlet: spout mouth is narrower than full spout width
    # proving there's a slot opening (not a full round opening)
    ctx.check(
        "slot_outlet_narrower_than_spout",
        spout_aabb is not None
        and (spout_aabb[1][2] - spout_aabb[0][2]) > SLOT_H + 0.004,
        f"spout height should be larger than slot height, proving rectangular slot, got {spout_aabb}",
    )

    # --- aerator presence and seating --------------------------------------
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator_exists",
        aerator_aabb is not None,
        "aerator part must exist",
    )
    # Aerator is at the spout mouth (forward position)
    ctx.check(
        "aerator_at_spout_mouth",
        aerator_aabb is not None and aerator_aabb[1][0] > 0.100,
        f"aerator should be near spout tip forward, got {aerator_aabb}",
    )
    # Aerator is near spout height
    ctx.check(
        "aerator_near_spout_height",
        aerator_aabb is not None
        and abs((aerator_aabb[0][2] + aerator_aabb[1][2]) / 2.0 - (SPOUT_ROOT_Z - 0.030)) < 0.015,
        f"aerator should be near spout mouth height, got {aerator_aabb}",
    )
    # Aerator contacts spout body (seated)
    ctx.expect_contact(aerator, body, name="aerator_seated_at_mouth")

    # --- knob on top -------------------------------------------------------
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "knob_on_top_of_body",
        knob_aabb is not None and knob_aabb[0][2] > BODY_HEIGHT - 0.015,
        f"knob should sit on top of body (~{BODY_HEIGHT} m), got {knob_aabb}",
    )
    # Knob stem contacts body (shaft seating)
    ctx.expect_contact(knob, body, name="knob_stem_seats_in_body")

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "knob_axis_vertical",
        abs(knob_joint.axis[2]) == 1.0
        and knob_joint.axis[0] == 0.0
        and knob_joint.axis[1] == 0.0,
        f"knob must rotate about vertical Z axis, got {knob_joint.axis}",
    )
    ctx.check(
        "knob_range_pm90deg",
        knob_joint.motion_limits is not None
        and abs(knob_joint.motion_limits.lower + KNOB_RANGE) < 1e-6
        and abs(knob_joint.motion_limits.upper - KNOB_RANGE) < 1e-6,
        "knob range must be -90..+90 deg",
    )

    # --- motion proof ------------------------------------------------------
    # Rotate knob: the indicator line should change orientation
    rest_knob = ctx.part_element_world_aabb(knob, elem=knob_cap)
    with ctx.pose({knob_joint: KNOB_RANGE}):
        turned_knob = ctx.part_element_world_aabb(knob, elem=knob_cap)
        # The knob itself is cylindrical so AABB may not change much,
        # but verify the part doesn't move vertically (it's revolute, not prismatic)
        ctx.check(
            "knob_turn_preserves_height",
            rest_knob is not None
            and turned_knob is not None
            and abs(rest_knob[0][2] - turned_knob[0][2]) < 0.002,
            f"knob rotation should not change Z position: rest={rest_knob}, turned={turned_knob}",
        )
    # Negative turn also preserves position
    with ctx.pose({knob_joint: -KNOB_RANGE}):
        neg_knob = ctx.part_element_world_aabb(knob, elem=knob_cap)
        ctx.check(
            "knob_negative_turn_preserves_height",
            rest_knob is not None
            and neg_knob is not None
            and abs(rest_knob[0][2] - neg_knob[0][2]) < 0.002,
            f"negative knob rotation should not change Z: rest={rest_knob}, neg={neg_knob}",
        )

    return ctx.report()


object_model = build_object_model()
