from __future__ import annotations

"""Polished-chrome single-hole basin faucet — squared modern monobloc variant.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A square escutcheon base plate carries a sharply squared monobloc body column.
- A short squared spout extends forward from the upper body, with a real hollow
  cylindrical outlet bored through its front face and a separate circular
  aerator insert seated at the mouth.
- A flat rectangular lever handle sits on top of the body on a revolute joint
  (flow control, 0..25 deg lift).
- A thin pull-up drain rod slides vertically behind the body on a prismatic
  joint (0..0.035 m travel) for pop-up drain operation.
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
BASE_SIDE = 0.055
BASE_H = 0.008
BASE_TOP_Z = BASE_H  # 0.008

BODY_DEPTH_X = 0.042
BODY_WIDTH_Y = 0.038
BODY_H = 0.160
BODY_TOP_Z = BASE_TOP_Z + BODY_H  # 0.168

# Spout emerges from the upper portion of the body
SPOUT_ROOT_Z = BODY_TOP_Z - 0.040  # 0.128 — spout root near body top
SPOUT_WIDTH_Y = 0.030
SPOUT_THICK_Z = 0.025
SPOUT_REACH_X = 0.095  # forward reach from body front face
SPOUT_FRONT_X = BODY_DEPTH_X / 2.0 + SPOUT_REACH_X  # tip x
SPOUT_BACK_X = BODY_DEPTH_X / 2.0  # root at body front face

# Hollow outlet bore at spout mouth
OUTLET_R = 0.010  # bore radius
OUTLET_DEPTH = 0.018  # bore depth into spout body
OUTLET_CENTER_Z = SPOUT_ROOT_Z + SPOUT_THICK_Z / 2.0  # centered in spout thickness

# Aerator insert — separate circular disc seated at the mouth
AERATOR_R = OUTLET_R - 0.001  # slightly smaller to sit inside bore
AERATOR_THICK = 0.003
# Aerator sits at the front face of the spout, recessed slightly
AERATOR_X = SPOUT_FRONT_X - AERATOR_THICK / 2.0 + 0.001

# Handle dimensions
HANDLE_LEN_X = 0.100
HANDLE_WIDTH_Y = 0.022
HANDLE_THICK_Z = 0.012
HANDLE_PIVOT_X = 0.0  # centered on body axis
HANDLE_MOUNT_Z = BODY_TOP_Z  # handle sits on body top

# Pivot boss — small chrome block connecting handle to body top
BOSS_DEPTH_X = 0.028
BOSS_WIDTH_Y = 0.026
BOSS_H = 0.010
BOSS_TOP_Z = BODY_TOP_Z + BOSS_H  # 0.178

# Drain rod behind body
DRAIN_ROD_R = 0.003
DRAIN_ROD_LEN = 0.060  # total rod length
DRAIN_KNOB_R = 0.007  # pull knob at top
DRAIN_KNOB_H = 0.008
DRAIN_ROD_X = -(BODY_DEPTH_X / 2.0 + DRAIN_ROD_R)  # rod surface touches body rear face
DRAIN_ROD_BASE_Z = BODY_TOP_Z - 0.010  # enters body near top rear
DRAIN_GUIDE_H = 0.012  # guide bracket height
DRAIN_GUIDE_OUTER_R = DRAIN_ROD_R + 0.004  # chrome guide ring around rod

# Joint ranges
LIFT_RANGE = math.radians(25.0)
DRAIN_TRAVEL = 0.035  # meters


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    aerator_mat = model.material("aerator_mesh", rgba=(0.55, 0.57, 0.60, 1.0))
    rod_chrome = model.material("rod_chrome", rgba=(0.78, 0.80, 0.84, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: base plate, monobloc column, squared spout with hollow
    # outlet, and aerator insert
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Square escutcheon base plate
    body.visual(
        Box((BASE_SIDE, BASE_SIDE, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=chrome,
        name="base_plate",
    )

    # Sharply squared monobloc column
    body.visual(
        Box((BODY_DEPTH_X, BODY_WIDTH_Y, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + BODY_H / 2.0)),
        material=chrome,
        name="monobloc_column",
    )

    # Spout body — squared rectangular blade with hollow cylindrical outlet
    # bore cut through the front face.
    spout_len = SPOUT_FRONT_X - SPOUT_BACK_X
    spout_cx = (SPOUT_BACK_X + SPOUT_FRONT_X) / 2.0
    spout_cz = SPOUT_ROOT_Z + SPOUT_THICK_Z / 2.0

    spout_solid = (
        cq.Workplane("XY")
        .transformed(offset=(spout_cx, 0.0, spout_cz))
        .box(spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)
    )
    # Cut cylindrical bore from the front face inward (hollow outlet)
    bore_cutter = (
        cq.Workplane("YZ")
        .transformed(offset=(0.0, spout_cz, SPOUT_FRONT_X))
        .circle(OUTLET_R)
        .extrude(-OUTLET_DEPTH)
    )
    spout_hollow = spout_solid.cut(bore_cutter)

    body.visual(
        mesh_from_cadquery(spout_hollow, "spout_body"),
        material=chrome,
        name="spout_body",
    )

    # Dark outlet cavity — fills the bore to contact bore walls (connected)
    outlet_cavity = (
        cq.Workplane("YZ")
        .transformed(offset=(0.0, spout_cz, SPOUT_FRONT_X - OUTLET_DEPTH))
        .circle(OUTLET_R)
        .extrude(OUTLET_DEPTH)
    )
    body.visual(
        mesh_from_cadquery(outlet_cavity, "outlet_cavity"),
        material=dark,
        name="outlet_cavity",
    )

    # Separate circular aerator insert disc seated into the spout mouth
    # Starts slightly inside spout front face to ensure contact with bore rim
    aerator_insert_depth = 0.004  # how far back into the bore it seats
    aerator_disc = (
        cq.Workplane("YZ")
        .transformed(offset=(0.0, spout_cz, SPOUT_FRONT_X - aerator_insert_depth))
        .circle(AERATOR_R)
        .extrude(aerator_insert_depth + AERATOR_THICK)
    )
    # Cut small holes in the aerator to make it look like a mesh screen
    for dy in [-0.004, 0.0, 0.004]:
        for dz in [-0.004, 0.0, 0.004]:
            if dy * dy + dz * dz <= (AERATOR_R - 0.002) ** 2:
                hole = (
                    cq.Workplane("YZ")
                    .transformed(offset=(0.0, spout_cz + dy, SPOUT_FRONT_X - aerator_insert_depth - 0.001))
                    .circle(0.0012)
                    .extrude(aerator_insert_depth + AERATOR_THICK + 0.002)
                )
                aerator_disc = aerator_disc.cut(hole)

    body.visual(
        mesh_from_cadquery(aerator_disc, "aerator_insert"),
        material=aerator_mat,
        name="aerator_insert",
    )

    # Small boss/pivot mount on body top for handle
    body.visual(
        Box((BOSS_DEPTH_X, BOSS_WIDTH_Y, BOSS_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + BOSS_H / 2.0)),
        material=chrome,
        name="handle_boss",
    )

    # Drain rod guide ring on body rear face — annular bracket that the
    # rod slides through, providing physical support contact.
    guide_ring = (
        cq.Workplane("XY")
        .circle(DRAIN_GUIDE_OUTER_R)
        .circle(DRAIN_ROD_R)
        .extrude(DRAIN_GUIDE_H)
    )
    body.visual(
        mesh_from_cadquery(guide_ring, "drain_guide_ring"),
        # Orient vertically: ring axis along X, seated on body rear face
        origin=Origin(
            xyz=(DRAIN_ROD_X, 0.0, DRAIN_ROD_BASE_Z + 0.005),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="drain_guide_ring",
    )

    # ------------------------------------------------------------------
    # Lever handle — revolute joint for flow control
    # Joint at rear of handle on the boss top, axis along Y (left-right)
    # so positive q lifts the forward grip end upward.
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")
    # Handle blade extends forward from pivot; bottom sits on boss top
    handle.visual(
        Box((HANDLE_LEN_X, HANDLE_WIDTH_Y, HANDLE_THICK_Z)),
        origin=Origin(xyz=(HANDLE_LEN_X / 2.0, 0.0, HANDLE_THICK_Z / 2.0)),
        material=chrome,
        name="handle_blade",
    )
    # Small pivot collar wrapping around the boss-to-handle connection
    handle.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.008, 0.0, 0.003)),
        material=chrome,
        name="pivot_collar",
    )

    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, BOSS_TOP_Z)),
        # Blade extends along +X; -Y axis makes positive q lift the tip up.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=0.0, upper=LIFT_RANGE
        ),
    )

    # ------------------------------------------------------------------
    # Drain rod — prismatic joint for pop-up drain
    # Rod slides vertically (Z axis) behind the body.
    # ------------------------------------------------------------------
    drain = model.part("drain_rod")
    # Main rod shaft
    drain.visual(
        Cylinder(radius=DRAIN_ROD_R, length=DRAIN_ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LEN / 2.0)),
        material=rod_chrome,
        name="rod_shaft",
    )
    # Pull knob at top of rod
    drain.visual(
        Cylinder(radius=DRAIN_KNOB_R, length=DRAIN_KNOB_H),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_ROD_LEN + DRAIN_KNOB_H / 2.0)),
        material=chrome,
        name="drain_knob",
    )

    model.articulation(
        "drain_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain,
        # Rod origin: behind body, partially inserted into body
        origin=Origin(xyz=(DRAIN_ROD_X, 0.0, DRAIN_ROD_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=0.5, lower=0.0, upper=DRAIN_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    handle = object_model.get_part("lever_handle")
    drain = object_model.get_part("drain_rod")
    lift = object_model.get_articulation("handle_lift")
    drain_joint = object_model.get_articulation("drain_lift")

    # --- joint plan checks ---
    ctx.check(
        "handle_lift is revolute 0..25 deg about horizontal Y axis",
        lift.articulation_type == ArticulationType.REVOLUTE
        and abs(lift.axis[0]) < 1e-9
        and abs(abs(lift.axis[1]) - 1.0) < 1e-9
        and abs(lift.axis[2]) < 1e-9
        and lift.motion_limits is not None
        and abs(lift.motion_limits.lower) < 1e-9
        and abs(lift.motion_limits.upper - math.radians(25.0)) < 1e-6,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )
    ctx.check(
        "drain_lift is prismatic along Z, 0..0.035 m",
        drain_joint.articulation_type == ArticulationType.PRISMATIC
        and abs(drain_joint.axis[0]) < 1e-9
        and abs(drain_joint.axis[1]) < 1e-9
        and abs(abs(drain_joint.axis[2]) - 1.0) < 1e-9
        and drain_joint.motion_limits is not None
        and abs(drain_joint.motion_limits.lower) < 1e-9
        and abs(drain_joint.motion_limits.upper - DRAIN_TRAVEL) < 1e-6,
        details=f"axis={drain_joint.axis}, limits={drain_joint.motion_limits}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "faucet body height is compact basin scale ~0.17..0.22 m",
        body_aabb is not None and 0.15 <= body_aabb[1][2] <= 0.24,
        details=f"body top z={body_aabb[1][2] if body_aabb else None}",
    )

    # --- squared monobloc body ---
    column_aabb = ctx.part_element_world_aabb(body, elem="monobloc_column")
    ctx.check(
        "monobloc column is sharply squared (rectangular cross-section)",
        column_aabb is not None
        and (column_aabb[1][0] - column_aabb[0][0]) > 0.030
        and (column_aabb[1][1] - column_aabb[0][1]) > 0.025,
        details=f"column_aabb={column_aabb}",
    )

    # --- hollow outlet at spout mouth ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_body")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_cavity")
    ctx.check(
        "hollow outlet cavity is recessed inside the spout mouth",
        spout_aabb is not None
        and outlet_aabb is not None
        and outlet_aabb[1][0] <= spout_aabb[1][0] + 0.001
        and outlet_aabb[0][0] > spout_aabb[0][0],
        details=f"spout={spout_aabb}, outlet={outlet_aabb}",
    )

    # --- separate aerator insert ---
    aerator_aabb = ctx.part_element_world_aabb(body, elem="aerator_insert")
    ctx.check(
        "aerator insert is a separate circular disc at the spout mouth",
        aerator_aabb is not None
        and spout_aabb is not None
        and aerator_aabb[1][0] >= spout_aabb[1][0] - 0.010
        and (aerator_aabb[1][1] - aerator_aabb[0][1]) < 0.025
        and (aerator_aabb[1][2] - aerator_aabb[0][2]) < 0.025,
        details=f"aerator={aerator_aabb}",
    )

    # --- drain rod behind body ---
    drain_aabb = ctx.part_world_aabb(drain)
    ctx.check(
        "drain rod is positioned behind the body (negative X of body center)",
        drain_aabb is not None
        and (drain_aabb[0][0] + drain_aabb[1][0]) / 2.0 < -0.010,
        details=f"drain_aabb={drain_aabb}",
    )

    # --- handle seating ---
    ctx.expect_contact(
        handle,
        body,
        elem_a="pivot_collar",
        elem_b="handle_boss",
        contact_tol=1e-4,
        name="handle pivot collar seats on the body boss",
    )

    # --- decisive pose: handle lift ---
    handle_aabb_rest = ctx.part_world_aabb(handle)
    rest_tip_z = handle_aabb_rest[1][2] if handle_aabb_rest else None
    with ctx.pose({lift: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive handle lift raises the grip tip upward",
            rest_tip_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_tip_z + 0.015,
            details=f"rest_top={rest_tip_z}, lifted={lifted_aabb}",
        )

    # --- decisive pose: drain rod pull-up ---
    drain_rest_z = drain_aabb[1][2] if drain_aabb else None
    with ctx.pose({drain_joint: DRAIN_TRAVEL}):
        drain_raised_aabb = ctx.part_world_aabb(drain)
        ctx.check(
            "drain rod pulls upward when actuated (prismatic Z motion)",
            drain_rest_z is not None
            and drain_raised_aabb is not None
            and drain_raised_aabb[1][2] > drain_rest_z + DRAIN_TRAVEL - 0.002,
            details=f"rest_top={drain_rest_z}, raised={drain_raised_aabb}",
        )

    # Allow drain rod shaft to overlap with body (rod slides through body bore)
    ctx.allow_overlap(
        body,
        drain,
        elem_a="monobloc_column",
        elem_b="rod_shaft",
        reason="Drain rod slides through a bore in the rear of the monobloc body.",
    )

    return ctx.report()


object_model = build_object_model()
