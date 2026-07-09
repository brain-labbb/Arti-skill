from __future__ import annotations

"""Sharply-squared modern monobloc single-hole basin faucet with cylindrical flow knob.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A square base plate anchors a compact squared rectangular monobloc body.
- A flat rectangular spout arm extends forward from the body top, with a real
  hollow cylindrical outlet bore recessed into its underside near the tip.
- A cylindrical flow knob sits on a short shaft atop the body and rotates about
  the vertical axis (quarter-turn flow control, 0..90 deg).
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
# Key dimensions (meters)
# ----------------------------------------------------------------------------
BASE_SIDE = 0.065
BASE_H = 0.008

BODY_WIDTH_Y = 0.048
BODY_DEPTH_X = 0.042
BODY_H = 0.148
BODY_TOP_Z = BASE_H + BODY_H  # 0.156

SPOUT_WIDTH_Y = 0.036
SPOUT_THICK_Z = 0.016
SPOUT_BACK_X = BODY_DEPTH_X / 2.0  # flush with body front face
SPOUT_REACH = 0.125
SPOUT_TIP_X = SPOUT_BACK_X + SPOUT_REACH
SPOUT_TOP_Z = BODY_TOP_Z - 0.006
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.134

# Outlet bore — real hollow cavity cut into the spout underside
OUTLET_X = SPOUT_TIP_X - 0.014
OUTLET_R = 0.008
OUTLET_DEPTH = 0.011  # bore depth upward into the spout
COLLAR_OUTER_R = 0.011
COLLAR_H = 0.005  # chrome collar protrudes below spout underside

# Knob shaft post
POST_R = 0.005
POST_H = 0.008
POST_TOP_Z = BODY_TOP_Z + POST_H

# Knob
KNOB_DIAMETER = 0.030
KNOB_HEIGHT = 0.020

KNOB_ROTATE_RANGE = math.radians(90.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    matte = model.material("knob_matte", rgba=(0.22, 0.22, 0.24, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: base plate, monobloc column, spout with hollow outlet, post
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Square base plate
    body.visual(
        Box((BASE_SIDE, BASE_SIDE, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=chrome,
        name="base_plate",
    )

    # Squared monobloc column
    body.visual(
        Box((BODY_DEPTH_X, BODY_WIDTH_Y, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + BODY_H / 2.0)),
        material=chrome,
        name="monobloc",
    )

    # Spout arm with real hollow bore — built with CadQuery
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    spout_center_x = (SPOUT_BACK_X + SPOUT_TIP_X) / 2.0

    spout_solid = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BOT_Z)
        .transformed(offset=(spout_center_x, 0.0, 0.0))
        .rect(spout_len, SPOUT_WIDTH_Y)
        .extrude(SPOUT_THICK_Z)
    )
    # Cut a real cylindrical bore from the underside near the tip
    bore_cutter = (
        cq.Workplane("XY")
        .workplane(offset=SPOUT_BOT_Z - 0.001)
        .transformed(offset=(OUTLET_X, 0.0, 0.0))
        .circle(OUTLET_R)
        .extrude(OUTLET_DEPTH + 0.001)
    )
    spout_with_bore = spout_solid.cut(bore_cutter)
    body.visual(
        mesh_from_cadquery(spout_with_bore, "spout_arm"),
        origin=Origin(),
        material=chrome,
        name="spout_arm",
    )

    # Chrome collar ring around the outlet opening (protrudes below spout)
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .circle(OUTLET_R)
        .extrude(COLLAR_H)
    )
    body.visual(
        mesh_from_cadquery(collar, "outlet_collar"),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - COLLAR_H)),
        material=chrome,
        name="outlet_collar",
    )

    # Dark disc at the bore ceiling (visible through the hollow outlet)
    # Top face at SPOUT_BOT_Z + OUTLET_DEPTH (the bore ceiling) for connectivity
    cavity_h = 0.004
    body.visual(
        Cylinder(radius=OUTLET_R - 0.001, length=cavity_h),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z + OUTLET_DEPTH - cavity_h / 2.0)),
        material=dark,
        name="outlet_cavity",
    )

    # Knob shaft post on body top
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="knob_shaft",
    )

    # ------------------------------------------------------------------
    # Flow knob: cylindrical knob on top, rotates about vertical axis
    # ------------------------------------------------------------------
    knob = model.part("flow_knob")

    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="knurled", count=32, depth=0.0008, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,  # mounting face at z=0
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "knob_body"),
        origin=Origin(),
        material=matte,
        name="knob_body",
    )

    # Stem that reaches down to contact the shaft post top
    stem_h = 0.004
    knob.visual(
        Cylinder(radius=POST_R + 0.001, length=stem_h),
        origin=Origin(xyz=(0.0, 0.0, -stem_h / 2.0)),
        material=matte,
        name="knob_stem",
    )

    # Asymmetric flow tab extending radially from the knob side
    tab_len = 0.012
    tab_w = 0.008
    tab_h = 0.006
    knob.visual(
        Box((tab_len, tab_w, tab_h)),
        origin=Origin(xyz=(KNOB_DIAMETER / 2.0 + tab_len / 2.0, 0.0, KNOB_HEIGHT * 0.4)),
        material=matte,
        name="flow_tab",
    )

    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=4.0, lower=0.0, upper=KNOB_ROTATE_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    rotate = object_model.get_articulation("knob_rotate")

    # --- joint plan: knob is revolute about vertical, 0..90 deg ---
    ctx.check(
        "knob_rotate is revolute 0..90 deg about vertical Z axis",
        rotate.articulation_type == ArticulationType.REVOLUTE
        and abs(rotate.axis[0]) < 1e-9
        and abs(rotate.axis[1]) < 1e-9
        and abs(abs(rotate.axis[2]) - 1.0) < 1e-9
        and rotate.motion_limits is not None
        and abs(rotate.motion_limits.lower) < 1e-9
        and abs(rotate.motion_limits.upper - math.radians(90.0)) < 1e-6,
        details=f"axis={rotate.axis}, limits={rotate.motion_limits}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "faucet total height is compact basin scale (~0.17-0.21 m)",
        knob_aabb is not None and 0.16 <= knob_aabb[1][2] <= 0.22,
        details=f"knob_aabb={knob_aabb}",
    )

    # --- squared monobloc body ---
    mono_aabb = ctx.part_element_world_aabb(body, elem="monobloc")
    ctx.check(
        "monobloc body is a tall squared rectangular column",
        mono_aabb is not None
        and abs((mono_aabb[1][0] - mono_aabb[0][0]) - BODY_DEPTH_X) < 1e-4
        and abs((mono_aabb[1][1] - mono_aabb[0][1]) - BODY_WIDTH_Y) < 1e-4
        and (mono_aabb[1][2] - mono_aabb[0][2]) > 0.10,
        details=f"mono_aabb={mono_aabb}",
    )

    # --- spout extends forward from body ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_arm")
    ctx.check(
        "spout arm cantilevers forward from the body front face",
        spout_aabb is not None
        and spout_aabb[1][0] > BODY_DEPTH_X / 2.0 + 0.08,
        details=f"spout_aabb={spout_aabb}",
    )

    # --- real hollow outlet at spout mouth ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="outlet_collar")
    cavity_aabb = ctx.part_element_world_aabb(body, elem="outlet_cavity")
    ctx.check(
        "outlet collar protrudes below the spout underside near the tip",
        collar_aabb is not None
        and collar_aabb[0][2] < SPOUT_BOT_Z - 0.001
        and collar_aabb[1][0] > SPOUT_BACK_X + 0.05,
        details=f"collar_aabb={collar_aabb}",
    )
    ctx.check(
        "dark outlet cavity is recessed inside the bore above the collar bottom",
        cavity_aabb is not None
        and collar_aabb is not None
        and cavity_aabb[0][2] > collar_aabb[0][2] + 0.002
        and cavity_aabb[1][2] < SPOUT_TOP_Z,
        details=f"cavity_aabb={cavity_aabb}, collar_aabb={collar_aabb}",
    )
    ctx.check(
        "outlet cavity is contained within collar footprint in XY",
        collar_aabb is not None
        and cavity_aabb is not None
        and cavity_aabb[0][0] >= collar_aabb[0][0] - 0.001
        and cavity_aabb[1][0] <= collar_aabb[1][0] + 0.001
        and cavity_aabb[0][1] >= collar_aabb[0][1] - 0.001
        and cavity_aabb[1][1] <= collar_aabb[1][1] + 0.001,
        details=f"cavity={cavity_aabb}, collar={collar_aabb}",
    )

    # --- knob mounted on shaft post ---
    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_stem",
        elem_b="knob_shaft",
        contact_tol=0.005,
        name="knob stem contacts the shaft post",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="xy",
        min_overlap=0.005,
        elem_a="knob_stem",
        elem_b="knob_shaft",
        name="knob stem overlaps shaft post in XY (mounted on top)",
    )

    # Allow intentional overlap: knob stem nests around the shaft post
    ctx.allow_overlap(
        knob,
        body,
        elem_a="knob_stem",
        elem_b="knob_shaft",
        reason="The knob stem intentionally wraps around the shaft post for mechanical connection.",
    )

    # --- decisive pose: knob rotation swings the flow tab ---
    tab_rest_aabb = ctx.part_element_world_aabb(knob, elem="flow_tab")
    with ctx.pose({rotate: KNOB_ROTATE_RANGE}):
        tab_rotated_aabb = ctx.part_element_world_aabb(knob, elem="flow_tab")
        ctx.check(
            "positive knob rotation swings the flow tab from +X toward +Y",
            tab_rest_aabb is not None
            and tab_rotated_aabb is not None
            and tab_rotated_aabb[1][1] > tab_rest_aabb[1][1] + 0.005,
            details=f"rest_tab={tab_rest_aabb}, rotated_tab={tab_rotated_aabb}",
        )
        # Knob body stays above monobloc top
        ctx.expect_gap(
            knob,
            body,
            axis="z",
            max_penetration=0.002,
            positive_elem="knob_body",
            negative_elem="monobloc",
            name="rotated knob body stays above monobloc top",
        )

    return ctx.report()


object_model = build_object_model()
