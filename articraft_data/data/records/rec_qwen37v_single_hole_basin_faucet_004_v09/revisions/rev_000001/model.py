from __future__ import annotations

"""Single-hole basin faucet variant: polished-chrome single-lever with detachable-look
spout collar, pull-up drain rod, and real hollow outlet at the spout mouth.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A square stepped base plate carries a slim rectangular column.
- A flat rectangular spout blade cantilevers forward from the column top via a
  visible collar seam that makes the spout look detachable.
- A real hollow outlet (through-hole) is cut in the spout underside near the tip.
- Above the spout root, a short chrome post carries the lever pivot block, which
  swivels about a vertical axis (temperature, -45..+45 deg).
- The flat rectangular lever handle lifts on a horizontal left-right axis through
  the pivot block (flow, 0..25 deg).
- Behind the column, a thin pull-up drain rod slides vertically (prismatic, 0..0.04 m).
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
BASE_LOWER_SIDE = 0.090
BASE_LOWER_H = 0.006
BASE_UPPER_SIDE = 0.068
BASE_UPPER_H = 0.012
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.018

COLUMN_DEPTH_X = 0.035
COLUMN_WIDTH_Y = 0.045
COLUMN_TOP_Z = 0.235

SPOUT_WIDTH_Y = 0.050
SPOUT_THICK_Z = 0.020
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear face
SPOUT_TIP_X = 0.1825  # ~0.17 m forward reach past the column front face
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.215

# --- Collar seam at spout root (detachable-look) ---
COLLAR_LEN_X = 0.012  # short collar sleeve along X at spout root
COLLAR_WIDTH_Y = SPOUT_WIDTH_Y + 0.006  # slightly wider than spout
COLLAR_THICK_Z = SPOUT_THICK_Z + 0.006  # slightly taller than spout
COLLAR_CENTER_X = SPOUT_BACK_X + COLLAR_LEN_X / 2.0
GROOVE_WIDTH_Y = COLLAR_WIDTH_Y + 0.002
GROOVE_THICK_Z = 0.002  # thin dark groove line

# --- Hollow outlet at spout mouth ---
OUTLET_X = 0.162  # outlet center, near the spout tip
OUTLET_R = 0.009  # through-hole radius
OUTLET_RECESS_DEPTH = 0.004  # counterbore recess from underside

POST_R = 0.013
POST_H = 0.013
POST_TOP_Z = COLUMN_TOP_Z + POST_H  # 0.248

BLOCK_DEPTH_X = 0.045
BLOCK_WIDTH_Y = 0.044
BLOCK_H = 0.0365
BLOCK_TOP_REL = BLOCK_H  # in swivel-child frame (origin at post top)

HANDLE_LEN_X = 0.170
HANDLE_WIDTH_Y = 0.050
HANDLE_THICK_Z = 0.013
HANDLE_FLOAT = 0.0015  # blade floats just above the block top
HANDLE_REAR_REL_X = -BLOCK_DEPTH_X / 2.0  # blade rear flush with block rear

# --- Drain rod (pull-up behind column) ---
ROD_R = 0.004
ROD_H = 0.100  # visible length above body top
ROD_CAP_R = 0.008
ROD_CAP_H = 0.010
ROD_X = -COLUMN_DEPTH_X / 2.0 - 0.007  # behind column; boss overlaps column rear face
ROD_REST_BOTTOM_Z = COLUMN_TOP_Z - 0.020  # rod shaft starts slightly below column top
DRAIN_TRAVEL = 0.040  # max upward slide

LIFT_RANGE = math.radians(25.0)
SWIVEL_RANGE = math.radians(45.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    groove = model.material("seam_groove", rgba=(0.15, 0.15, 0.16, 1.0))
    red = model.material("hot_red", rgba=(0.80, 0.08, 0.08, 1.0))
    blue = model.material("cold_blue", rgba=(0.10, 0.30, 0.78, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base plate, column, spout blade, collar, outlet, post
    # ------------------------------------------------------------------
    body = model.part("faucet_body")
    body.visual(
        Box((BASE_LOWER_SIDE, BASE_LOWER_SIDE, BASE_LOWER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_plate_lower",
    )
    body.visual(
        Box((BASE_UPPER_SIDE, BASE_UPPER_SIDE, BASE_UPPER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_plate_upper",
    )
    column_h = COLUMN_TOP_Z - BASE_TOP_Z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # --- Spout blade with real hollow outlet (CadQuery through-hole) ---
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    spout_solid = (
        cq.Workplane("XY")
        .box(spout_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z, centered=(True, True, False))
    )
    # Cut a through-hole near the tip for the real hollow outlet
    outlet_local_x = OUTLET_X - (SPOUT_BACK_X + spout_len / 2.0)
    spout_with_hole = (
        spout_solid
        .faces(">Z").workplane()
        .center(outlet_local_x, 0.0)
        .circle(OUTLET_R)
        .cutThruAll()
    )
    body.visual(
        mesh_from_cadquery(spout_with_hole, "spout_blade"),
        origin=Origin(
            xyz=((SPOUT_BACK_X + SPOUT_TIP_X) / 2.0, 0.0, SPOUT_BOT_Z)
        ),
        material=chrome,
        name="spout_blade",
    )

    # Dark outlet bore visible inside the through-hole — same radius as the
    # hole so it contacts the inner cylindrical wall, making it connected.
    # Slightly shorter than the spout so it reads as recessed at both faces.
    bore_len = SPOUT_THICK_Z - 0.002
    body.visual(
        Cylinder(radius=OUTLET_R, length=bore_len),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0)),
        material=dark,
        name="outlet_bore",
    )

    # --- Collar seam at spout root (detachable-look) ---
    # Outer chrome collar sleeve
    collar_shape = (
        cq.Workplane("XY")
        .box(COLLAR_LEN_X, COLLAR_WIDTH_Y, COLLAR_THICK_Z, centered=(True, True, False))
    )
    # Cut through the spout-profile hole so it wraps around
    collar_hollow = (
        cq.Workplane("XY")
        .box(COLLAR_LEN_X + 0.002, SPOUT_WIDTH_Y - 0.002, SPOUT_THICK_Z - 0.002, centered=(True, True, False))
    )
    collar_ring = collar_shape.cut(collar_hollow)
    body.visual(
        mesh_from_cadquery(collar_ring, "spout_collar"),
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, SPOUT_BOT_Z - 0.003)),
        material=chrome,
        name="spout_collar",
    )
    # Thin dark groove line at the collar rear edge (seam indicator)
    body.visual(
        Box((0.002, GROOVE_WIDTH_Y, COLLAR_THICK_Z + 0.002)),
        origin=Origin(xyz=(SPOUT_BACK_X + 0.001, 0.0, SPOUT_BOT_Z - 0.003 + COLLAR_THICK_Z / 2.0)),
        material=groove,
        name="collar_groove",
    )

    # Mounting post for handle swivel
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )

    # --- Drain rod guide boss on column rear ---
    body.visual(
        Cylinder(radius=ROD_R + 0.003, length=0.015),
        origin=Origin(xyz=(ROD_X, 0.0, COLUMN_TOP_Z - 0.010)),
        material=chrome,
        name="rod_guide_boss",
    )

    # ------------------------------------------------------------------
    # Swivel stage: lever pivot block on the mounting post (temperature)
    # ------------------------------------------------------------------
    block = model.part("lever_pivot_block")
    block.visual(
        Box((BLOCK_DEPTH_X, BLOCK_WIDTH_Y, BLOCK_H)),
        origin=Origin(xyz=(0.0, 0.0, BLOCK_H / 2.0)),
        material=chrome,
        name="pivot_block",
    )
    # Tiny hot/cold temperature dots on the block front face
    dot_x = BLOCK_DEPTH_X / 2.0
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, 0.007, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=red,
        name="hot_dot",
    )
    block.visual(
        Cylinder(radius=0.0025, length=0.003),
        origin=Origin(xyz=(dot_x, -0.007, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="cold_dot",
    )

    model.articulation(
        "handle_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=block,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=3.0, lower=-SWIVEL_RANGE, upper=SWIVEL_RANGE
        ),
    )

    # ------------------------------------------------------------------
    # Lift stage: flat rectangular lever handle (flow)
    # ------------------------------------------------------------------
    handle = model.part("lever_handle")
    handle.visual(
        Box((HANDLE_LEN_X, HANDLE_WIDTH_Y, HANDLE_THICK_Z)),
        origin=Origin(xyz=(HANDLE_LEN_X / 2.0, 0.0, HANDLE_THICK_Z / 2.0)),
        material=chrome,
        name="handle_blade",
    )
    heel_h = HANDLE_FLOAT + 0.004
    handle.visual(
        Box((0.018, 0.030, heel_h)),
        origin=Origin(xyz=(0.009, 0.0, -HANDLE_FLOAT + heel_h / 2.0)),
        material=chrome,
        name="pivot_heel",
    )

    model.articulation(
        "handle_lift",
        ArticulationType.REVOLUTE,
        parent=block,
        child=handle,
        origin=Origin(xyz=(HANDLE_REAR_REL_X, 0.0, BLOCK_TOP_REL + HANDLE_FLOAT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=LIFT_RANGE),
    )

    # ------------------------------------------------------------------
    # Drain rod: pull-up pop-up drain stopper rod behind column
    # Slides vertically via prismatic joint
    # ------------------------------------------------------------------
    drain = model.part("drain_rod")
    # Rod shaft
    drain.visual(
        Cylinder(radius=ROD_R, length=ROD_H),
        origin=Origin(xyz=(0.0, 0.0, ROD_H / 2.0)),
        material=chrome,
        name="rod_shaft",
    )
    # Rod cap/knob at top
    drain.visual(
        Cylinder(radius=ROD_CAP_R, length=ROD_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, ROD_H + ROD_CAP_H / 2.0)),
        material=chrome,
        name="rod_cap",
    )

    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain,
        origin=Origin(xyz=(ROD_X, 0.0, ROD_REST_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0, velocity=0.5, lower=0.0, upper=DRAIN_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    block = object_model.get_part("lever_pivot_block")
    handle = object_model.get_part("lever_handle")
    drain = object_model.get_part("drain_rod")
    swivel = object_model.get_articulation("handle_swivel")
    lift = object_model.get_articulation("handle_lift")
    drain_joint = object_model.get_articulation("drain_slide")

    # --- joint plan: types, axes, ranges ---
    ctx.check(
        "lift joint is revolute 0..25 deg about horizontal left-right axis",
        lift.articulation_type == ArticulationType.REVOLUTE
        and abs(lift.axis[0]) < 1e-9
        and abs(abs(lift.axis[1]) - 1.0) < 1e-9
        and abs(lift.axis[2]) < 1e-9
        and lift.motion_limits is not None
        and abs(lift.motion_limits.lower - 0.0) < 1e-9
        and abs(lift.motion_limits.upper - math.radians(25.0)) < 1e-6,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )
    ctx.check(
        "swivel joint is revolute -45..+45 deg about vertical axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and abs(swivel.axis[0]) < 1e-9
        and abs(swivel.axis[1]) < 1e-9
        and abs(abs(swivel.axis[2]) - 1.0) < 1e-9
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + math.radians(45.0)) < 1e-6
        and abs(swivel.motion_limits.upper - math.radians(45.0)) < 1e-6,
        details=f"axis={swivel.axis}, limits={swivel.motion_limits}",
    )
    ctx.check(
        "drain_slide is prismatic along Z with 0..0.04 m travel",
        drain_joint.articulation_type == ArticulationType.PRISMATIC
        and abs(drain_joint.axis[0]) < 1e-9
        and abs(drain_joint.axis[1]) < 1e-9
        and abs(abs(drain_joint.axis[2]) - 1.0) < 1e-9
        and drain_joint.motion_limits is not None
        and abs(drain_joint.motion_limits.lower - 0.0) < 1e-9
        and abs(drain_joint.motion_limits.upper - DRAIN_TRAVEL) < 1e-6,
        details=f"axis={drain_joint.axis}, limits={drain_joint.motion_limits}",
    )
    ctx.check(
        "swivel parents the lift joint (serial chain on the handle)",
        swivel.child == block.name and lift.parent == block.name and lift.child == handle.name,
        details=f"swivel.child={swivel.child}, lift.parent={lift.parent}, lift.child={lift.child}",
    )
    ctx.check(
        "drain rod is parented to the faucet body",
        drain_joint.parent == body.name and drain_joint.child == drain.name,
        details=f"parent={drain_joint.parent}, child={drain_joint.child}",
    )

    # --- grounding and true scale ---
    body_aabb = ctx.part_world_aabb(body)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.30 m (handle is the topmost part)",
        handle_aabb is not None and 0.28 <= handle_aabb[1][2] <= 0.32,
        details=f"handle_aabb={handle_aabb}",
    )
    ctx.check(
        "spout blade cantilevers ~0.17 m forward of the column front face",
        body_aabb is not None and 0.16 <= body_aabb[1][0] - COLUMN_DEPTH_X / 2.0 <= 0.19,
        details=f"body max x={None if body_aabb is None else body_aabb[1][0]}",
    )

    # --- collar seam at spout root (detachable look) ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="spout_collar")
    groove_aabb = ctx.part_element_world_aabb(body, elem="collar_groove")
    ctx.check(
        "spout collar wraps the spout root with visible seam",
        collar_aabb is not None
        and groove_aabb is not None
        and collar_aabb[0][0] < SPOUT_BACK_X + COLLAR_LEN_X + 0.005
        and abs(collar_aabb[0][2] - (SPOUT_BOT_Z - 0.003)) < 0.005,
        details=f"collar_aabb={collar_aabb}, groove_aabb={groove_aabb}",
    )
    ctx.check(
        "collar is slightly wider than spout blade (visible ring)",
        collar_aabb is not None
        and (collar_aabb[1][1] - collar_aabb[0][1]) > SPOUT_WIDTH_Y + 0.003,
        details=f"collar width={collar_aabb}",
    )

    # --- hollow outlet at spout mouth ---
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_bore")
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_blade")
    ctx.check(
        "outlet bore sits inside the spout blade near the tip",
        outlet_aabb is not None
        and spout_aabb is not None
        and outlet_aabb[0][0] > spout_aabb[0][0]
        and outlet_aabb[1][0] < spout_aabb[1][0]
        and outlet_aabb[0][2] >= spout_aabb[0][2] - 0.001,
        details=f"outlet_aabb={outlet_aabb}, spout_aabb={spout_aabb}",
    )
    ctx.check(
        "outlet bore is recessed below spout top (hollow through-hole)",
        outlet_aabb is not None
        and spout_aabb is not None
        and outlet_aabb[1][2] < spout_aabb[1][2],
        details=f"outlet_top={outlet_aabb[1][2] if outlet_aabb else None}, spout_top={spout_aabb[1][2] if spout_aabb else None}",
    )

    # --- drain rod behind column ---
    drain_aabb = ctx.part_world_aabb(drain)
    ctx.check(
        "drain rod sits behind the column (negative X from column center)",
        drain_aabb is not None
        and drain_aabb[0][0] < -COLUMN_DEPTH_X / 2.0,
        details=f"drain_aabb={drain_aabb}",
    )
    ctx.check(
        "drain rod cap is above column top at rest",
        drain_aabb is not None
        and drain_aabb[1][2] > COLUMN_TOP_Z,
        details=f"drain_aabb={drain_aabb}",
    )

    # --- temperature dots ---
    hot_aabb = ctx.part_element_world_aabb(block, elem="hot_dot")
    cold_aabb = ctx.part_element_world_aabb(block, elem="cold_dot")
    ctx.check(
        "red/blue temperature dots sit proud on the pivot block face",
        hot_aabb is not None
        and cold_aabb is not None
        and hot_aabb[1][0] > BLOCK_DEPTH_X / 2.0
        and hot_aabb[0][1] > cold_aabb[1][1],
        details=f"hot={hot_aabb}, cold={cold_aabb}",
    )

    # --- mounting: block seats on the post, handle floats just above block ---
    ctx.expect_contact(
        block,
        body,
        elem_a="pivot_block",
        elem_b="mounting_post",
        contact_tol=1e-5,
        name="pivot block seats on the chrome mounting post",
    )
    ctx.expect_contact(
        handle,
        block,
        elem_a="pivot_heel",
        elem_b="pivot_block",
        contact_tol=1e-5,
        name="handle pivot heel seats on the pivot block top",
    )
    ctx.expect_gap(
        handle,
        block,
        axis="z",
        min_gap=0.0005,
        max_gap=0.004,
        positive_elem="handle_blade",
        negative_elem="pivot_block",
        name="handle blade floats slightly above the pivot block",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        min_gap=0.03,
        name="handle assembly stays clear above the fixed spout blade",
    )
    ctx.expect_overlap(
        handle,
        block,
        axes="xy",
        min_overlap=0.02,
        name="handle blade root covers the pivot block footprint",
    )

    # --- drain rod passes through guide boss (intentional nesting) ---
    ctx.allow_overlap(
        drain,
        body,
        elem_a="rod_shaft",
        elem_b="rod_guide_boss",
        reason="The drain rod shaft slides through the guide boss on the column rear.",
    )
    ctx.expect_overlap(
        drain,
        body,
        axes="y",
        min_overlap=0.002,
        elem_a="rod_shaft",
        elem_b="rod_guide_boss",
        name="drain rod shaft aligns with the guide boss on Y",
    )
    ctx.expect_overlap(
        drain,
        body,
        axes="x",
        min_overlap=0.002,
        elem_a="rod_shaft",
        elem_b="rod_guide_boss",
        name="drain rod shaft aligns with the guide boss on X",
    )

    # --- decisive pose checks ---
    rest_tip_z = handle_aabb[1][2] if handle_aabb is not None else None
    with ctx.pose({lift: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive lift raises the handle grip tip upward",
            rest_tip_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_tip_z + 0.04,
            details=f"rest_top={rest_tip_z}, lifted_aabb={lifted_aabb}",
        )
        ctx.expect_gap(
            handle,
            block,
            axis="z",
            max_penetration=0.0,
            name="lifted handle does not dig into the pivot block",
        )

    rest_handle_aabb = handle_aabb
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swung_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "positive swivel slews the handle sideways about the vertical post axis",
            rest_handle_aabb is not None
            and swung_aabb is not None
            and swung_aabb[1][1] > rest_handle_aabb[1][1] + 0.05,
            details=f"rest={rest_handle_aabb}, swung={swung_aabb}",
        )
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            min_gap=0.03,
            name="swiveled handle still clears the fixed spout",
        )

    # --- drain rod slides upward ---
    rest_drain_top = drain_aabb[1][2] if drain_aabb is not None else None
    with ctx.pose({drain_joint: DRAIN_TRAVEL}):
        raised_drain_aabb = ctx.part_world_aabb(drain)
        ctx.check(
            "positive drain_slide raises the rod upward by the travel distance",
            rest_drain_top is not None
            and raised_drain_aabb is not None
            and raised_drain_aabb[1][2] > rest_drain_top + DRAIN_TRAVEL - 0.005,
            details=f"rest_top={rest_drain_top}, raised={raised_drain_aabb}",
        )
        ctx.expect_overlap(
            drain,
            body,
            axes="y",
            min_overlap=0.002,
            elem_a="rod_shaft",
            elem_b="rod_guide_boss",
            name="raised drain rod still aligns with guide boss",
        )

    return ctx.report()


object_model = build_object_model()
