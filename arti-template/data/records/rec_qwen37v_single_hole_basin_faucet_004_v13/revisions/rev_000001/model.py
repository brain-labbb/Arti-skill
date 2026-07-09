from __future__ import annotations

"""Sharply squared modern monobloc single-hole basin faucet with side lever.

Layout (meters, +Z up, ground at z=0, spout extends along +X):
- An oval rubber gasket sits on the counter at z=0.
- A sharply squared monobloc column rises from the gasket.
- A flat rectangular spout arm extends forward from the upper body, with a real
  hollow cylindrical outlet cavity recessed into its underside near the tip.
- A short horizontal axle on the right side of the body carries the side lever,
  which lifts upward (revolute, 0..30 deg) for flow control.
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
# Key dimensions (meters)
# ---------------------------------------------------------------------------
# Oval base gasket (ellipse ring)
GASKET_A = 0.033  # semi-major (X)
GASKET_B = 0.028  # semi-minor (Y)
GASKET_HOLE_R = 0.016  # center mounting hole
GASKET_THICK = 0.003
GASKET_TOP_Z = GASKET_THICK  # 0.003

# Monobloc body column
BODY_DX = 0.050  # width (X)
BODY_DY = 0.045  # depth (Y)
BODY_TOP_Z = 0.135
BODY_H = BODY_TOP_Z - GASKET_TOP_Z  # 0.132

# Spout arm (extends forward along +X from body front face)
SPOUT_ROOT_X = BODY_DX / 2.0  # 0.025 — flush with body front
SPOUT_TIP_X = 0.130  # total forward reach ~0.105 m past body front
SPOUT_W = 0.038  # spout width (Y)
SPOUT_THICK = 0.022  # spout thickness (Z)
SPOUT_TOP_Z = BODY_TOP_Z - 0.005  # spout top slightly below column top: 0.130
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK  # 0.108
SPOUT_LEN = SPOUT_TIP_X - SPOUT_ROOT_X  # 0.105

# Hollow outlet cavity (cylindrical pocket in spout underside near tip)
OUTLET_X = SPOUT_TIP_X - 0.018  # 0.112 — near the tip
OUTLET_R = 0.010  # cavity radius
CAVITY_DEPTH = 0.012  # pocket depth upward from spout bottom

# Side-lever mount (on right +Y face of body)
LEVER_MOUNT_Z = BODY_TOP_Z - 0.030  # 0.105 — near the top
LEVER_MOUNT_Y = BODY_DY / 2.0  # 0.0225 — body right face
AXLE_R = 0.005  # visible axle boss radius
AXLE_LEN = 0.012  # axle protrusion from body face

# Side lever
LEVER_LEN = 0.095  # lever length (extends in +Y from mount)
LEVER_W = 0.018  # lever width (X direction, front-back)
LEVER_THICK = 0.012  # lever thickness (Z direction)
HUB_R = 0.008  # hub radius around axle
HUB_LEN = 0.014  # hub length along axle axis

LEVER_RANGE = math.radians(30.0)  # flow: 0..30 degrees


def _build_spout_with_cavity():
    """Build the spout arm as a CadQuery solid with a real hollow outlet cavity."""
    # Spout box centered at local origin
    spout = cq.Workplane("XY").box(SPOUT_LEN, SPOUT_W, SPOUT_THICK)
    # Cut cylindrical pocket from the underside near the tip
    # Pocket opens from z = -SPOUT_THICK/2 upward by CAVITY_DEPTH
    outlet_local_x = (OUTLET_X - (SPOUT_ROOT_X + SPOUT_TIP_X) / 2.0)
    cutter = (
        cq.Workplane("XY")
        .transformed(offset=(outlet_local_x, 0.0, -SPOUT_THICK / 2.0 - 0.0005))
        .circle(OUTLET_R)
        .extrude(CAVITY_DEPTH + 0.001)
    )
    return spout.cut(cutter)


def _build_oval_gasket():
    """Build an oval (elliptical) ring gasket using CadQuery."""
    outer = (
        cq.Workplane("XY")
        .ellipse(GASKET_A, GASKET_B)
        .extrude(GASKET_THICK)
    )
    hole = (
        cq.Workplane("XY")
        .circle(GASKET_HOLE_R)
        .extrude(GASKET_THICK)
    )
    return outer.cut(hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squared_monobloc_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark_chrome = model.material("dark_chrome", rgba=(0.25, 0.27, 0.30, 1.0))
    rubber = model.material("rubber_gasket", rgba=(0.12, 0.12, 0.13, 1.0))
    outlet_dark = model.material("outlet_cavity", rgba=(0.05, 0.05, 0.06, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: gasket, monobloc column, spout arm with hollow outlet,
    # axle boss
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Oval base gasket at z=0
    gasket_cq = _build_oval_gasket()
    body.visual(
        mesh_from_cadquery(gasket_cq, "base_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rubber,
        name="base_gasket",
    )

    # Squared monobloc column
    body.visual(
        Box((BODY_DX, BODY_DY, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, GASKET_TOP_Z + BODY_H / 2.0)),
        material=chrome,
        name="body_column",
    )

    # Spout arm with hollow outlet cavity (CadQuery)
    spout_cq = _build_spout_with_cavity()
    spout_center_x = (SPOUT_ROOT_X + SPOUT_TIP_X) / 2.0
    spout_center_z = (SPOUT_TOP_Z + SPOUT_BOT_Z) / 2.0
    body.visual(
        mesh_from_cadquery(spout_cq, "spout_arm"),
        origin=Origin(xyz=(spout_center_x, 0.0, spout_center_z)),
        material=chrome,
        name="spout_arm",
    )

    # Dark outlet cavity floor (visible inside the hollow pocket).
    # Extends slightly above the cavity ceiling into the solid spout arm
    # to ensure visual connectivity within the body part.
    body.visual(
        Cylinder(radius=OUTLET_R - 0.001, length=0.004),
        origin=Origin(
            xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z + CAVITY_DEPTH + 0.001)
        ),
        material=outlet_dark,
        name="outlet_floor",
    )

    # Chrome outlet rim ring (visible lip around the cavity opening)
    rim = (
        cq.Workplane("XY")
        .circle(OUTLET_R + 0.002)
        .circle(OUTLET_R)
        .extrude(0.003)
    )
    body.visual(
        mesh_from_cadquery(rim, "outlet_rim"),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.001)),
        material=dark_chrome,
        name="outlet_rim",
    )

    # Axle boss on the body right face (+Y side)
    body.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(
            xyz=(0.0, LEVER_MOUNT_Y + AXLE_LEN / 2.0, LEVER_MOUNT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="axle_boss",
    )

    # ------------------------------------------------------------------
    # Side lever: rotates on the horizontal axle for flow control
    # Part frame origin at the axle center on the body right face.
    # Lever extends in local +Y from the mount point.
    # ------------------------------------------------------------------
    lever = model.part("side_lever")

    # Lever hub around the axle
    lever.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(
            xyz=(0.0, HUB_LEN / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lever_hub",
    )

    # Lever arm (flat rectangular bar extending in +Y)
    lever_arm_start_y = HUB_LEN / 2.0 + 0.002  # small gap past the hub
    lever.visual(
        Box((LEVER_W, LEVER_LEN, LEVER_THICK)),
        origin=Origin(
            xyz=(0.0, lever_arm_start_y + LEVER_LEN / 2.0, 0.0)
        ),
        material=chrome,
        name="lever_arm",
    )

    # Small grip cap at the lever tip (visual end detail)
    lever.visual(
        Box((LEVER_W + 0.004, 0.008, LEVER_THICK + 0.003)),
        origin=Origin(
            xyz=(0.0, lever_arm_start_y + LEVER_LEN - 0.004, 0.0)
        ),
        material=dark_chrome,
        name="lever_grip_cap",
    )

    # Articulation: revolute about horizontal front-back axis (X)
    # At q=0 the lever is horizontal; positive q lifts the tip upward.
    # axis=(1,0,0): right-hand rule rotates +Y toward +Z.
    model.articulation(
        "lever_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, LEVER_MOUNT_Y, LEVER_MOUNT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=LEVER_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("side_lever")
    lever_joint = object_model.get_articulation("lever_rotate")

    # --- joint plan: revolute side lever on horizontal axle ---
    ctx.check(
        "lever joint is revolute 0..30 deg about horizontal front-back axis",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(lever_joint.axis[0] - 1.0) < 1e-9
        and abs(lever_joint.axis[1]) < 1e-9
        and abs(lever_joint.axis[2]) < 1e-9
        and lever_joint.motion_limits is not None
        and abs(lever_joint.motion_limits.lower) < 1e-9
        and abs(lever_joint.motion_limits.upper - math.radians(30.0)) < 1e-6,
        details=f"axis={lever_joint.axis}, limits={lever_joint.motion_limits}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "base gasket is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-4,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "faucet total height is compact basin scale (~0.14 to 0.20 m)",
        body_aabb is not None and 0.13 <= body_aabb[1][2] <= 0.20,
        details=f"body top z={body_aabb[1][2] if body_aabb else None}",
    )

    # --- squared monobloc body ---
    column_aabb = ctx.part_element_world_aabb(body, elem="body_column")
    ctx.check(
        "body column is a sharply squared rectangular block",
        column_aabb is not None
        and abs((column_aabb[1][0] - column_aabb[0][0]) - BODY_DX) < 0.002
        and abs((column_aabb[1][1] - column_aabb[0][1]) - BODY_DY) < 0.002,
        details=f"column_aabb={column_aabb}",
    )

    # --- oval base gasket ---
    gasket_aabb = ctx.part_element_world_aabb(body, elem="base_gasket")
    ctx.check(
        "oval base gasket present at the base",
        gasket_aabb is not None
        and abs(gasket_aabb[0][2]) < 1e-4
        and abs(gasket_aabb[1][2] - GASKET_THICK) < 0.002,
        details=f"gasket_aabb={gasket_aabb}",
    )
    ctx.check(
        "gasket is wider in X than Y (oval, not circular)",
        gasket_aabb is not None
        and (gasket_aabb[1][0] - gasket_aabb[0][0])
        > (gasket_aabb[1][1] - gasket_aabb[0][1]) + 0.002,
        details=f"gasket_aabb={gasket_aabb}",
    )

    # --- real hollow outlet at the spout mouth ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_arm")
    rim_aabb = ctx.part_element_world_aabb(body, elem="outlet_rim")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_floor")
    ctx.check(
        "spout arm extends forward from the body",
        spout_aabb is not None and spout_aabb[1][0] > BODY_DX / 2.0 + 0.08,
        details=f"spout_aabb={spout_aabb}",
    )
    ctx.check(
        "outlet rim sits at the spout underside near the tip",
        rim_aabb is not None
        and rim_aabb[0][2] < SPOUT_BOT_Z + 0.003
        and rim_aabb[0][0] > SPOUT_ROOT_X + 0.05,
        details=f"rim_aabb={rim_aabb}",
    )
    ctx.check(
        "dark outlet floor is recessed above the rim opening (hollow cavity)",
        outlet_aabb is not None
        and rim_aabb is not None
        and outlet_aabb[0][2] > rim_aabb[0][2] + 0.001
        and outlet_aabb[0][2] > SPOUT_BOT_Z,
        details=f"outlet={outlet_aabb}, rim={rim_aabb}",
    )

    # --- side lever mounting ---
    hub_aabb = ctx.part_element_world_aabb(lever, elem="lever_hub")
    ctx.check(
        "lever hub wraps the axle on the body right face",
        hub_aabb is not None
        and hub_aabb[0][1] > BODY_DY / 2.0 - 0.005,
        details=f"hub_aabb={hub_aabb}",
    )

    # The lever hub intentionally wraps the short axle boss (captured shaft).
    ctx.allow_overlap(
        body,
        lever,
        elem_a="axle_boss",
        elem_b="lever_hub",
        reason="The lever hub is intentionally sleeved over the short horizontal axle boss as a captured shaft mount.",
    )
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_hub",
        elem_b="axle_boss",
        contact_tol=0.002,
        name="lever hub is in contact with the axle boss (captured shaft)",
    )

    # The lever arm itself must clear the body column at rest.
    ctx.expect_gap(
        lever,
        body,
        axis="y",
        min_gap=-0.001,
        positive_elem="lever_arm",
        negative_elem="body_column",
        name="lever arm clears the body column at rest",
    )

    # --- decisive pose: positive lever angle lifts the tip upward ---
    rest_aabb = lever_aabb
    rest_top_z = rest_aabb[1][2] if rest_aabb is not None else None
    with ctx.pose({lever_joint: LEVER_RANGE}):
        lifted_aabb = ctx.part_world_aabb(lever)
        ctx.check(
            "positive lever angle raises the grip tip upward",
            rest_top_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_top_z + 0.02,
            details=f"rest_top={rest_top_z}, lifted_aabb={lifted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
