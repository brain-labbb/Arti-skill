from __future__ import annotations

"""Single-hole basin faucet: tall straight tower with short forward spout and side lever.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A square stepped base plate carries a tall slim rectangular tower.
- A short rectangular spout blade cantilevers forward from the tower top,
  with a real hollow cylindrical outlet tube at its mouth underside.
- A small boss block protrudes from the tower right side (+Y face) near the top.
  A horizontal axle pin along X goes through the boss.
- A flat rectangular side lever extends outward along +Y from the boss,
  rotating about the horizontal X axis (flow control, 0..30 deg).
  Positive rotation lifts the lever tip upward.
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

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
BASE_LOWER_SIDE = 0.080
BASE_LOWER_H = 0.005
BASE_UPPER_SIDE = 0.062
BASE_UPPER_H = 0.010
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.015

TOWER_DEPTH_X = 0.035
TOWER_WIDTH_Y = 0.040
TOWER_TOP_Z = 0.340  # tall tower

SPOUT_WIDTH_Y = 0.040
SPOUT_THICK_Z = 0.018
SPOUT_BACK_X = TOWER_DEPTH_X / 2.0  # flush with tower front face
SPOUT_TIP_X = SPOUT_BACK_X + 0.080  # short forward reach ~0.08 m
SPOUT_TOP_Z = TOWER_TOP_Z  # blade top flush with tower top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z

# Hollow outlet at spout mouth: real CadQuery tube
OUTLET_X = SPOUT_TIP_X - 0.012  # near spout tip
OUTLET_OUTER_R = 0.010
OUTLET_INNER_R = 0.007
OUTLET_H = 0.012  # tube height; protrudes below spout underside

# Boss block on tower right side (+Y face)
BOSS_SIZE_X = 0.024  # front-to-back
BOSS_SIZE_Y = 0.014  # protrusion from tower face
BOSS_SIZE_Z = 0.028  # height
BOSS_CENTER_Z = TOWER_TOP_Z - 0.035  # near top of tower
BOSS_FACE_Y = TOWER_WIDTH_Y / 2.0  # tower right face
BOSS_CENTER_Y = BOSS_FACE_Y + BOSS_SIZE_Y / 2.0  # boss center

# Axle pin along X through the boss
AXLE_R = 0.005
AXLE_LEN = BOSS_SIZE_X + 0.008  # slightly wider than boss

# Lever paddle extends along +Y from past the boss
LEVER_LEN = 0.090  # lever paddle length
LEVER_WIDTH_Z = 0.024  # lever height
LEVER_THICK_X = 0.010  # lever front-to-back thickness
LEVER_START_Y = BOSS_FACE_Y + BOSS_SIZE_Y  # starts at boss outer face

LEVER_RANGE = math.radians(30.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_tall_tower")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base, tower, spout, outlet tube, boss
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Base plate - lower step
    body.visual(
        Box((BASE_LOWER_SIDE, BASE_LOWER_SIDE, BASE_LOWER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_plate_lower",
    )
    # Base plate - upper step
    body.visual(
        Box((BASE_UPPER_SIDE, BASE_UPPER_SIDE, BASE_UPPER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_plate_upper",
    )

    # Tall straight tower
    tower_h = TOWER_TOP_Z - BASE_TOP_Z
    body.visual(
        Box((TOWER_DEPTH_X, TOWER_WIDTH_Y, tower_h)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + tower_h / 2.0)),
        material=chrome,
        name="tower",
    )

    # Short forward spout blade
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    body.visual(
        Box((spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)
        ),
        material=chrome,
        name="spout_blade",
    )

    # Real hollow outlet tube at spout mouth underside (CadQuery annulus)
    outlet_tube = (
        cq.Workplane("XY")
        .circle(OUTLET_OUTER_R)
        .circle(OUTLET_INNER_R)
        .extrude(OUTLET_H)
    )
    body.visual(
        mesh_from_cadquery(outlet_tube, "outlet_tube"),
        # Tube hangs below spout: top of tube at spout underside
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - OUTLET_H)),
        material=chrome,
        name="outlet_tube",
    )
    # Dark outlet face inside the tube (recessed disc)
    body.visual(
        Cylinder(radius=OUTLET_INNER_R, length=0.004),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - OUTLET_H + 0.004)),
        material=dark,
        name="outlet_face",
    )

    # Boss block on tower right side — provides the bearing mount for the axle
    body.visual(
        Box((BOSS_SIZE_X, BOSS_SIZE_Y, BOSS_SIZE_Z)),
        origin=Origin(xyz=(0.0, BOSS_CENTER_Y, BOSS_CENTER_Z)),
        material=chrome,
        name="lever_boss",
    )
    # Axle pin through the boss (cylinder along X, centered on boss)
    body.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(
            xyz=(0.0, BOSS_CENTER_Y, BOSS_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="axle_pin",
    )

    # ------------------------------------------------------------------
    # Side lever: rotates about horizontal X axis through the boss center
    # Child frame at the boss center (axle axis intersection).
    # Lever extends along +Y from the boss outer face.
    # Positive rotation about +X lifts the lever tip upward (+Z).
    # ------------------------------------------------------------------
    lever = model.part("side_lever")

    # Lever paddle — starts at the boss outer face and extends outward along +Y
    # In local frame (joint origin at boss center): boss outer face at y = BOSS_SIZE_Y/2
    lever.visual(
        Box((LEVER_THICK_X, LEVER_LEN, LEVER_WIDTH_Z)),
        origin=Origin(
            xyz=(0.0, BOSS_SIZE_Y / 2.0 + LEVER_LEN / 2.0, 0.0),
        ),
        material=chrome,
        name="lever_paddle",
    )
    # Small grip rib at the lever tip for visual detail
    lever.visual(
        Box((LEVER_THICK_X + 0.004, 0.008, LEVER_WIDTH_Z + 0.004)),
        origin=Origin(
            xyz=(0.0, BOSS_SIZE_Y / 2.0 + LEVER_LEN - 0.004, 0.0),
        ),
        material=chrome,
        name="lever_grip",
    )

    model.articulation(
        "lever_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        # Joint origin at the boss outer face center (axle axis intersection)
        origin=Origin(xyz=(0.0, BOSS_CENTER_Y, BOSS_CENTER_Z)),
        # Axis along +X: positive q lifts the +Y lever tip toward +Z
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LEVER_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("side_lever")
    lever_joint = object_model.get_articulation("lever_rotate")

    # --- joint type, axis, and range ---
    ctx.check(
        "lever joint is revolute 0..30 deg about horizontal X axis",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(lever_joint.axis[0]) - 1.0) < 1e-9
        and abs(lever_joint.axis[1]) < 1e-9
        and abs(lever_joint.axis[2]) < 1e-9
        and lever_joint.motion_limits is not None
        and abs(lever_joint.motion_limits.lower - 0.0) < 1e-9
        and abs(lever_joint.motion_limits.upper - math.radians(30.0)) < 1e-6,
        details=f"axis={lever_joint.axis}, limits={lever_joint.motion_limits}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "tower height is tall (~0.34 m to tower top)",
        body_aabb is not None and 0.32 <= body_aabb[1][2] <= 0.36,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "spout has short forward reach (~0.06..0.10 m past tower front face)",
        body_aabb is not None
        and 0.06 <= body_aabb[1][0] - TOWER_DEPTH_X / 2.0 <= 0.10,
        details=f"body max x={None if body_aabb is None else body_aabb[1][0]}",
    )

    # --- hollow outlet at spout mouth ---
    tube_aabb = ctx.part_element_world_aabb(body, elem="outlet_tube")
    face_aabb = ctx.part_element_world_aabb(body, elem="outlet_face")
    ctx.check(
        "outlet tube protrudes below the spout underside",
        tube_aabb is not None
        and tube_aabb[0][2] < SPOUT_BOT_Z - 0.002,
        details=f"tube_aabb={tube_aabb}",
    )
    ctx.check(
        "dark outlet face is recessed inside the hollow outlet tube",
        tube_aabb is not None
        and face_aabb is not None
        and face_aabb[0][2] > tube_aabb[0][2] + 0.001
        and face_aabb[0][0] > tube_aabb[0][0]
        and face_aabb[1][0] < tube_aabb[1][0],
        details=f"face_aabb={face_aabb}, tube_aabb={tube_aabb}",
    )

    # --- lever mounts on the tower side via boss ---
    boss_aabb = ctx.part_element_world_aabb(body, elem="lever_boss")
    ctx.check(
        "boss block protrudes from tower right face (positive Y)",
        boss_aabb is not None
        and boss_aabb[1][1] > TOWER_WIDTH_Y / 2.0 + 0.005,
        details=f"boss_aabb={boss_aabb}",
    )
    ctx.check(
        "lever assembly extends past the boss along +Y",
        lever_aabb is not None
        and lever_aabb[1][1] > BOSS_CENTER_Y + BOSS_SIZE_Y / 2.0 + 0.02,
        details=f"lever_aabb={lever_aabb}",
    )

    # --- mounting: lever paddle inner face contacts the boss outer face ---
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_paddle",
        elem_b="lever_boss",
        contact_tol=0.002,
        name="lever paddle inner face contacts the boss outer face",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="xz",
        min_overlap=0.005,
        elem_a="lever_paddle",
        elem_b="lever_boss",
        name="lever paddle overlaps the boss footprint in the XZ plane",
    )

    # --- decisive pose check: positive rotation lifts lever tip upward ---
    rest_tip_z = lever_aabb[1][2] if lever_aabb is not None else None
    rest_tip_y = lever_aabb[1][1] if lever_aabb is not None else None
    with ctx.pose({lever_joint: LEVER_RANGE}):
        rotated_aabb = ctx.part_world_aabb(lever)
        ctx.check(
            "positive lever rotation raises the paddle tip upward",
            rest_tip_z is not None
            and rotated_aabb is not None
            and rotated_aabb[1][2] > rest_tip_z + 0.02,
            details=f"rest_top={rest_tip_z}, rotated_aabb={rotated_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
