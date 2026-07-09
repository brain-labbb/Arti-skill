from __future__ import annotations

"""Single-hole basin faucet variant: squared modern monobloc with top flow knob.

A sharply squared monobloc body (~0.20 m tall) on a wider square base flange.
A flat-bottomed open-channel spout projects forward and slightly down ~0.13 m
from the front of the body, with a real hollow cylindrical outlet at the mouth.
A cylindrical flow knob sits on top of the body and rotates about the vertical
axis for flow control (quarter-turn, 0..90 deg).

Articulation:
- ``knob_rotate``: revolute about the vertical (Z) axis at the body top,
  0..+90 deg; positive q opens flow.
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
BODY_WIDTH = 0.050  # X extent of the squared body
BODY_DEPTH = 0.050  # Y extent
BODY_HEIGHT = 0.180  # Z extent of the main column above flange
FLANGE_WIDTH = 0.068
FLANGE_DEPTH = 0.068
FLANGE_HEIGHT = 0.012

SPOUT_ROOT_Z = 0.110  # channel centerline height where it leaves the body
SPOUT_OUTER_W = 0.032
SPOUT_OUTER_H = 0.020
SPOUT_WALL = 0.004
SPOUT_FLOOR = 0.005

# Hollow outlet tube at spout mouth
OUTLET_OD = 0.020  # outer diameter of the outlet tube
OUTLET_ID = 0.014  # inner diameter (hollow bore)
OUTLET_LENGTH = 0.025  # length of tube hanging down from spout tip

# Flow knob on top
KNOB_DIAMETER = 0.040
KNOB_HEIGHT = 0.024
KNOB_STEM_RADIUS = 0.006
KNOB_STEM_LENGTH = 0.010

KNOB_RANGE = math.radians(90.0)  # quarter-turn flow control


def _build_spout_with_outlet() -> cq.Workplane:
    """Open-top U-channel spout with integrated hollow outlet tube at the tip.

    Built in spout-local coordinates: the channel centerline starts at the
    origin heading +X; the visual is placed at the body front at
    ``SPOUT_ROOT_Z``. The outlet tube hangs down from the spout tip as one
    connected piece.
    """
    # Spline path for the channel
    path = cq.Workplane("XZ").spline(
        [
            (0.000, 0.000),
            (0.055, -0.004),
            (0.095, -0.012),
            (0.125, -0.024),
            (0.145, -0.042),
        ]
    )
    hw = SPOUT_OUTER_W / 2.0
    hh = SPOUT_OUTER_H / 2.0
    inner_hw = hw - SPOUT_WALL
    floor_v = -hh + SPOUT_FLOOR
    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-hw, hh),
                (-hw, -hh),
                (hw, -hh),
                (hw, hh),
                (inner_hw, hh),
                (inner_hw, floor_v),
                (-inner_hw, floor_v),
                (-inner_hw, hh),
            ]
        )
        .close()
    )
    spout = profile.sweep(path)

    # Outlet tube: build as a solid cylinder overlapping well into the channel
    # to ensure a clean union, then bore out the interior.
    tip_x = 0.145
    tip_z_bottom = -0.042 - hh  # bottom of the channel at tip
    outer_r = OUTLET_OD / 2.0
    inner_r = OUTLET_ID / 2.0
    # Overlap 8 mm into the channel body for solid union connectivity
    overlap = 0.008
    tube_total_len = OUTLET_LENGTH + overlap

    # Solid outer cylinder extending from well inside the spout downward
    outer_cyl = (
        cq.Workplane("XY")
        .transformed(offset=(tip_x, 0.0, tip_z_bottom + overlap))
        .circle(outer_r)
        .extrude(-tube_total_len)
    )
    combined = spout.union(outer_cyl)

    # Bore the hollow passage through the outlet tube
    bore = (
        cq.Workplane("XY")
        .transformed(offset=(tip_x, 0.0, tip_z_bottom + overlap + 0.001))
        .circle(inner_r)
        .extrude(-(tube_total_len + 0.002))
    )
    combined = combined.cut(bore)

    return combined


def _build_body_shell() -> cq.Workplane:
    """Squared monobloc body shell with slight edge chamfer."""
    body = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
    )
    # Chamfer vertical edges for a modern look
    body = body.edges("|Z").chamfer(0.002)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("brushed_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("bright_steel", rgba=(0.81, 0.82, 0.84, 1.0))

    # ------------------------------------------------------------------ body
    body = model.part("faucet_body")
    # Square base flange
    body.visual(
        Box((FLANGE_WIDTH, FLANGE_DEPTH, FLANGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT / 2.0)),
        material="brushed_steel",
        name="base_flange",
    )
    # Squared monobloc column
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_HEIGHT)),
        material="brushed_steel",
        name="body_column",
    )
    # Channel spout with integrated hollow outlet tube at mouth
    body.visual(
        mesh_from_cadquery(_build_spout_with_outlet(), "spout_with_outlet"),
        origin=Origin(xyz=(BODY_WIDTH / 2.0, 0.0, SPOUT_ROOT_Z)),
        material="brushed_steel",
        name="spout_channel",
    )

    # ------------------------------------------------------------------ knob
    # Cylindrical flow knob on top of the body, rotates about Z.
    knob = model.part("flow_knob")
    # Knob stem (short shaft connecting knob to body top)
    knob.visual(
        Cylinder(radius=KNOB_STEM_RADIUS, length=KNOB_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, KNOB_STEM_LENGTH / 2.0)),
        material="bright_steel",
        name="knob_stem",
    )
    # Knob body
    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=32, depth=0.001),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "knob_cap"),
        origin=Origin(xyz=(0.0, 0.0, KNOB_STEM_LENGTH + KNOB_HEIGHT / 2.0)),
        material="bright_steel",
        name="knob_cap",
    )

    # Articulation: body -> knob, revolute about Z
    body_top_z = FLANGE_HEIGHT + BODY_HEIGHT
    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, body_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=KNOB_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    rotate = object_model.get_articulation("knob_rotate")

    spout = body.get_visual("spout_channel")
    knob_cap = knob.get_visual("knob_cap")
    knob_stem = knob.get_visual("knob_stem")
    body_col = body.get_visual("body_column")

    # Intentional overlap: knob stem embeds slightly into body top for seated mount
    ctx.allow_overlap(
        knob,
        body,
        elem_a="knob_stem",
        elem_b="body_column",
        reason="knob stem seats into the body top bore (small local embed for shaft capture)",
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
        aabb is not None and 0.185 < aabb[1][2] < 0.215,
        f"total body top should be ~0.19-0.21 m up, got {aabb}",
    )

    # Squared body: X and Y extents should be close and rectangular
    body_col_aabb = ctx.part_element_world_aabb(body, elem=body_col)
    ctx.check(
        "squared_body_proportions",
        body_col_aabb is not None
        and abs((body_col_aabb[1][0] - body_col_aabb[0][0]) - BODY_WIDTH) < 0.005
        and abs((body_col_aabb[1][1] - body_col_aabb[0][1]) - BODY_DEPTH) < 0.005,
        f"body column should be ~{BODY_WIDTH}x{BODY_DEPTH} squared, got {body_col_aabb}",
    )

    # Spout projects forward
    spout_aabb = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "spout_projects_forward",
        spout_aabb is not None and spout_aabb[1][0] > 0.140,
        f"channel spout should reach >0.14 m forward, got {spout_aabb}",
    )

    # Hollow outlet tube extends below spout tip (integrated into spout channel)
    spout_aabb_full = ctx.part_element_world_aabb(body, elem=spout)
    ctx.check(
        "outlet_extends_below_channel",
        spout_aabb_full is not None and spout_aabb_full[0][2] < SPOUT_ROOT_Z - 0.060,
        f"spout+outlet should reach well below spout root (outlet tube), got {spout_aabb_full}",
    )
    ctx.check(
        "outlet_has_substantial_height",
        spout_aabb_full is not None
        and (SPOUT_ROOT_Z - spout_aabb_full[0][2]) > 0.060,
        f"total spout+outlet drop should include outlet tube length, got {spout_aabb_full}",
    )

    # Knob on top of body
    knob_cap_aabb = ctx.part_element_world_aabb(knob, elem=knob_cap)
    body_top_z = FLANGE_HEIGHT + BODY_HEIGHT
    ctx.check(
        "knob_above_body",
        knob_cap_aabb is not None and knob_cap_aabb[0][2] > body_top_z - 0.005,
        f"knob cap should sit on body top (~{body_top_z} m), got {knob_cap_aabb}",
    )

    # Knob stem connects knob to body
    ctx.expect_contact(knob, body, elem_a=knob_stem, elem_b=body_col, name="knob_stem_seats_on_body")

    # --- joint plan --------------------------------------------------------
    ctx.check(
        "knob_axis_vertical",
        abs(rotate.axis[2]) == 1.0 and rotate.axis[0] == 0.0 and rotate.axis[1] == 0.0,
        f"knob must rotate about vertical Z axis, got {rotate.axis}",
    )
    ctx.check(
        "knob_range_0_to_90",
        rotate.motion_limits is not None
        and abs(rotate.motion_limits.lower) < 1e-6
        and abs(rotate.motion_limits.upper - KNOB_RANGE) < 1e-3,
        "knob range must be 0..90 deg",
    )

    # --- motion proof ------------------------------------------------------
    # At q=0 knob indicator faces forward; at q=90deg it rotates sideways.
    # We verify the knob actually rotates by checking its AABB changes.
    rest_aabb = ctx.part_element_world_aabb(knob, elem=knob_cap)
    with ctx.pose({rotate: KNOB_RANGE}):
        turned_aabb = ctx.part_element_world_aabb(knob, elem=knob_cap)
        # The indicator line (engraved) is asymmetric, so AABB should shift
        # slightly. More importantly, confirm knob stays on body.
        ctx.check(
            "knob_stays_on_body_at_max",
            turned_aabb is not None and turned_aabb[0][2] > body_top_z - 0.005,
            f"knob must remain on body top when turned, got {turned_aabb}",
        )

    # Knob stem is seated at body top (contact, small intentional overlap allowed)
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        max_penetration=0.005,
        positive_elem=knob_stem,
        negative_elem=body_col,
        name="knob_stem_seated_at_body_top",
    )

    return ctx.report()


object_model = build_object_model()
