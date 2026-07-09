from __future__ import annotations

# Realistic articulated desk pencil sharpener with a FOLDING hand crank.
# The primary mechanism is the side hand crank (CONTINUOUS rotary about the
# horizontal axle). The crank arm has a second REVOLUTE fold joint at its tip:
# the grip segment folds from a deployed perpendicular position (for cranking)
# to a stowed position flat against the arm.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Coordinate convention (housing/part frames, meters)
#   +X : toward the FRONT face (pencil port faces +X)
#   +Y : toward the RIGHT side where the crank lives
#   +Z : up
# ---------------------------------------------------------------------------

# Housing outer extents.
BODY_W = 0.090  # depth along X
BODY_D = 0.082  # width along Y
BODY_H = 0.078  # tall lower body
BODY_FILLET = 0.012

# Upper shoulder block that the crank axle and clamp posts attach to.
SHOULDER_H = 0.026
SHOULDER_W = 0.084
SHOULDER_D = 0.078

# Pencil port (front face) geometry.
PORT_CENTER_Z = 0.040
PORT_OUTER_R = 0.018
PORT_THROAT_R = 0.0075
PORT_DEPTH = 0.022

# Crank geometry.
AXLE_R = 0.0055
AXLE_LEN = 0.018  # exposed stub length outboard of the housing
CRANK_ARM_LEN = 0.034  # radial arm length (axle center to knuckle center)
CRANK_ARM_W = 0.0095
CRANK_ARM_T = 0.0055
HANDLE_R = 0.006
HANDLE_LEN = 0.022

# Fold knuckle location: at the arm tip, midway through the arm plate thickness.
KNUCKLE_Y = AXLE_LEN + 0.001 + CRANK_ARM_T / 2.0  # ≈ 0.02175
KNUCKLE_Z = CRANK_ARM_LEN  # at the arm tip
KNUCKLE_BARREL_R = 0.004
KNUCKLE_BARREL_LEN = 0.012  # along X

# Right wall of the housing in part-local X-Y-Z; crank sits just outboard of it.
RIGHT_WALL_Y = BODY_D / 2.0
AXLE_Z = BODY_H + 0.006  # axle height (on the shoulder block, mid)


def _rounded_box(width: float, depth: float, height: float, fillet: float) -> cq.Workplane:
    """Solid rounded box centered on XY, base at z=0, vertical edges filleted."""
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(fillet)
    )


def _build_housing() -> cq.Workplane:
    """Hollowed charcoal housing with a front pencil port and an upper shoulder."""
    body = _rounded_box(BODY_W, BODY_D, BODY_H, BODY_FILLET)

    # Upper shoulder block (slightly inset) that carries the crank + clamp posts.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .box(SHOULDER_W, SHOULDER_D, SHOULDER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.006)
    )
    body = body.union(shoulder)

    # Front pencil port: a conical insertion funnel cut into the +X face.
    port_axis = cq.Workplane("YZ").workplane(offset=BODY_W / 2.0 + 0.001)
    # Outer recess (shallow dish where the pencil rests).
    recess = (
        port_axis.center(0.0, PORT_CENTER_Z)
        .circle(PORT_OUTER_R)
        .extrude(-0.006)
    )
    body = body.cut(recess)
    # Tapered throat leading toward the cutter (funnel).
    throat = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_W / 2.0 - 0.005)
        .center(0.0, PORT_CENTER_Z)
        .circle(PORT_OUTER_R - 0.004)
        .workplane(offset=-PORT_DEPTH)
        .circle(PORT_THROAT_R)
        .loft(combine=False)
    )
    body = body.cut(throat)

    # Axle bore through the right shoulder wall so the crank shaft reads as
    # entering the housing.
    axle_bore = (
        cq.Workplane("XZ")
        .workplane(offset=-(RIGHT_WALL_Y + 0.002))
        .center(0.0, AXLE_Z)
        .circle(AXLE_R + 0.0009)
        .extrude(0.020)
    )
    body = body.cut(axle_bore)

    return body


def _build_cutter() -> cq.Workplane:
    """Helical milling cutter visible inside the pencil port (along X)."""
    cutter_front_x = BODY_W / 2.0 - 0.004
    cutter_back_x = BODY_W / 2.0 - PORT_DEPTH - 0.012
    core = (
        cq.Workplane("YZ")
        .workplane(offset=cutter_back_x)
        .center(0.0, PORT_CENTER_Z)
        .circle(PORT_THROAT_R - 0.0008)
        .extrude(cutter_front_x - cutter_back_x)
    )
    flutes = None
    for k in range(4):
        ang = k * math.pi / 2.0
        rib = (
            cq.Workplane("YZ")
            .workplane(offset=BODY_W / 2.0 - PORT_DEPTH - 0.001)
            .center(
                (PORT_THROAT_R - 0.0015) * math.cos(ang),
                PORT_CENTER_Z + (PORT_THROAT_R - 0.0015) * math.sin(ang),
            )
            .circle(0.0011)
            .extrude(0.013)
        )
        flutes = rib if flutes is None else flutes.union(rib)
    return core.union(flutes)


def _build_front_plate() -> cq.Workplane:
    """Thin badge plate on the lower front face."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=BODY_W / 2.0)
        .center(0.0, 0.016)
        .rect(0.044, 0.010)
        .extrude(0.0010)
        .edges("|X")
        .fillet(0.0015)
    )


def _build_clamp_post() -> cq.Workplane:
    """One cylindrical clamp/cover post that rises from the top deck."""
    deck_z = BODY_H + SHOULDER_H
    post = (
        cq.Workplane("XY")
        .workplane(offset=deck_z)
        .circle(0.0065)
        .extrude(0.014)
        .edges(">Z")
        .fillet(0.0035)
    )
    groove = (
        cq.Workplane("XY")
        .workplane(offset=deck_z + 0.0095)
        .circle(0.0066)
        .extrude(0.0012)
    )
    return post.cut(groove.intersect(post)).union(post)


def _build_crank_arm() -> cq.Workplane:
    """Crank arm: axle stub + flange + hub + radial arm + knuckle barrel.

    Authored in a local frame whose ORIGIN is the axle center, with the axle
    running along local +Y. Uses the XZ-workplane convention (normal=-Y, so
    extrude goes in -Y) and mirrors across XZ to point outboard (+Y).
    """
    # Axle stub: starts slightly inboard and runs outboard past the wall face.
    axle = (
        cq.Workplane("XZ")
        .workplane(offset=-0.004)
        .circle(AXLE_R)
        .extrude(AXLE_LEN + 0.004)
    )
    # Bearing flange that seats flat against the outboard housing wall face.
    flange = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .circle(0.0105)
        .extrude(0.0022)
    )
    # Hub collar where the arm meets the axle.
    hub = (
        cq.Workplane("XZ")
        .workplane(offset=AXLE_LEN - 0.004)
        .circle(0.0085)
        .extrude(0.006)
    )
    # Radial arm: flat bar extending in +Z from the axle outboard plane.
    arm_y = AXLE_LEN + 0.001
    arm = (
        cq.Workplane("XZ")
        .workplane(offset=arm_y)
        .center(0.0, CRANK_ARM_LEN / 2.0)
        .rect(CRANK_ARM_W, CRANK_ARM_LEN)
        .extrude(CRANK_ARM_T)
        .edges("|Y")
        .fillet(0.0025)
    )
    # Knuckle barrel: a small hinge cylinder along X at the arm tip. Built at
    # negative Y before mirror so it lands at positive Y (matching the arm)
    # after the XZ mirror. The YZ workplane has normal +X; center(-KNUCKLE_Y,
    # KNUCKLE_Z) places the cylinder at world Y=-KNUCKLE_Y before mirror →
    # +KNUCKLE_Y after mirror.
    knuckle = (
        cq.Workplane("YZ")
        .workplane(offset=-KNUCKLE_BARREL_LEN / 2.0)
        .center(-KNUCKLE_Y, KNUCKLE_Z)
        .circle(KNUCKLE_BARREL_R)
        .extrude(KNUCKLE_BARREL_LEN)
    )
    crank_arm = axle.union(flange).union(hub).union(arm).union(knuckle)
    # Mirror across XZ plane so the assembly points outboard (+Y).
    return crank_arm.mirror(mirrorPlane="XZ")


def _build_crank_grip() -> cq.Workplane:
    """Folding grip segment: handle + end cap + knuckle sleeve.

    Authored in the grip's own local frame with ORIGIN at the fold knuckle
    center. The handle cylinder extends along local +Y from the origin
    (perpendicular to the arm when deployed, q=0 on the fold joint).

    The XZ workplane has normal=-Y, so to get a cylinder spanning y∈[0, L]
    we place the workplane at y=+L via offset=-L and extrude by L in -Y.
    """
    # Knuckle sleeve: barrel along X at the origin, wraps around the arm's
    # knuckle barrel to form a visible barrel-hinge. Uses YZ workplane (normal=+X).
    sleeve = (
        cq.Workplane("YZ")
        .workplane(offset=-KNUCKLE_BARREL_LEN / 2.0 + 0.001)
        .center(0.0, 0.0)
        .circle(KNUCKLE_BARREL_R + 0.0012)
        .extrude(KNUCKLE_BARREL_LEN - 0.002)
    )
    # Grip handle: cylinder along +Y from y=0 to y=HANDLE_LEN.
    # Place the XZ workplane at y=+HANDLE_LEN (offset=-HANDLE_LEN) and extrude
    # back toward y=0 in the -Y normal direction.
    handle = (
        cq.Workplane("XZ")
        .workplane(offset=-HANDLE_LEN)
        .center(0.0, 0.0)
        .circle(HANDLE_R)
        .extrude(HANDLE_LEN)
    )
    # End cap (mushroom head) at y=HANDLE_LEN to y=HANDLE_LEN+0.003.
    handle_cap = (
        cq.Workplane("XZ")
        .workplane(offset=-(HANDLE_LEN + 0.003))
        .center(0.0, 0.0)
        .circle(HANDLE_R + 0.0015)
        .extrude(0.003)
        .edges(">Y")
        .fillet(0.002)
    )
    # Connecting collar between the knuckle sleeve and the handle base so the
    # grip reads as one rigid piece bridging the hinge to the shaft.
    collar = (
        cq.Workplane("XZ")
        .workplane(offset=-0.004)
        .center(0.0, 0.0)
        .circle(HANDLE_R + 0.002)
        .extrude(0.008)
    )
    grip = sleeve.union(handle).union(handle_cap).union(collar)
    return grip


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pencil_sharpener")

    charcoal = model.material("charcoal_plastic", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.14, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    badge = model.material("badge_silver", rgba=(0.78, 0.79, 0.80, 1.0))
    grip_mat = model.material("grip_rubber", rgba=(0.18, 0.18, 0.20, 1.0))

    # --- Housing (root) ---------------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing(), "housing"),
        material=charcoal,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_front_plate(), "front_plate"),
        material=badge,
        name="front_plate",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H + SHOULDER_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + SHOULDER_H) / 2.0)),
    )

    # --- Cutter (fixed inside the port) -----------------------------------
    cutter = model.part("cutter")
    cutter.visual(
        mesh_from_cadquery(_build_cutter(), "cutter"),
        material=steel,
        name="cutter_body",
    )
    cutter.inertial = Inertial.from_geometry(
        Cylinder(radius=PORT_THROAT_R, length=0.024),
        mass=0.02,
        origin=Origin(xyz=(BODY_W / 2.0 - PORT_DEPTH, 0.0, PORT_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "housing_to_cutter",
        ArticulationType.FIXED,
        parent=housing,
        child=cutter,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Two top clamp posts (fixed) --------------------------------------
    deck_z = BODY_H + SHOULDER_H
    for idx, x_off in enumerate((-0.020, 0.020)):
        post = model.part(f"clamp_post_{idx + 1}")
        post.visual(
            mesh_from_cadquery(_build_clamp_post(), f"clamp_post_{idx + 1}"),
            material=dark_metal,
            name=f"clamp_post_shell_{idx + 1}",
        )
        post.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0065, length=0.014),
            mass=0.01,
            origin=Origin(xyz=(x_off, 0.012, deck_z + 0.007)),
        )
        model.articulation(
            f"housing_to_clamp_post_{idx + 1}",
            ArticulationType.FIXED,
            parent=housing,
            child=post,
            origin=Origin(xyz=(x_off, 0.012, 0.0)),
        )

    # --- Crank arm (CONTINUOUS rotary about the axle) ---------------------
    crank_arm = model.part("crank_arm")
    crank_arm.visual(
        mesh_from_cadquery(_build_crank_arm(), "crank_arm"),
        material=dark_metal,
        name="crank_arm_body",
    )
    crank_arm.inertial = Inertial.from_geometry(
        Box((CRANK_ARM_W, AXLE_LEN + CRANK_ARM_T, CRANK_ARM_LEN + 2 * KNUCKLE_BARREL_R)),
        mass=0.025,
        origin=Origin(xyz=(0.0, (AXLE_LEN + CRANK_ARM_T) / 2.0, CRANK_ARM_LEN / 2.0)),
    )

    # The crank arm is authored with its axle center at local origin and the
    # axle running along +Y. Place the articulation frame at the right housing
    # wall, at the axle height. Rotation about +Y spins the arm.
    model.articulation(
        "housing_to_crank_arm",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank_arm,
        origin=Origin(xyz=(0.0, RIGHT_WALL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    # --- Crank grip (REVOLUTE fold joint at the arm tip) ------------------
    crank_grip = model.part("crank_grip")
    crank_grip.visual(
        mesh_from_cadquery(_build_crank_grip(), "crank_grip"),
        material=grip_mat,
        name="crank_grip_body",
    )
    crank_grip.inertial = Inertial.from_geometry(
        Cylinder(radius=HANDLE_R + 0.002, length=HANDLE_LEN + 0.006),
        mass=0.012,
        origin=Origin(xyz=(0.0, (HANDLE_LEN + 0.003) / 2.0, 0.0)),
    )

    # The grip is authored with its origin at the fold knuckle center and the
    # handle extending in local +Y. The fold joint origin is placed at the arm
    # tip in the crank_arm frame. Axis along -X so positive q folds the grip
    # from +Y (deployed, perpendicular to arm) toward -Z (stowed, flat along
    # the arm back toward the axle).
    model.articulation(
        "crank_arm_to_grip",
        ArticulationType.REVOLUTE,
        parent=crank_arm,
        child=crank_grip,
        origin=Origin(xyz=(0.0, KNUCKLE_Y, KNUCKLE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    crank_arm = object_model.get_part("crank_arm")
    crank_grip = object_model.get_part("crank_grip")
    cutter = object_model.get_part("cutter")
    post1 = object_model.get_part("clamp_post_1")
    post2 = object_model.get_part("clamp_post_2")
    axle_joint = object_model.get_articulation("housing_to_crank_arm")
    fold_joint = object_model.get_articulation("crank_arm_to_grip")

    # --- Axle joint: continuous rotary ------------------------------------
    ctx.check(
        "axle joint is continuous rotary",
        str(axle_joint.joint_type).lower().endswith("continuous"),
        details=f"joint_type={axle_joint.joint_type}",
    )
    axle_axis = tuple(axle_joint.axis)
    ctx.check(
        "axle joint axis is horizontal along Y",
        abs(axle_axis[1]) > 0.99 and abs(axle_axis[0]) < 0.01 and abs(axle_axis[2]) < 0.01,
        details=f"axis={axle_axis}",
    )

    # --- Fold joint: revolute at the arm tip ------------------------------
    ctx.check(
        "fold joint is revolute",
        str(fold_joint.joint_type).lower().endswith("revolute"),
        details=f"joint_type={fold_joint.joint_type}",
    )
    fold_axis = tuple(fold_joint.axis)
    ctx.check(
        "fold joint axis is along X (perpendicular to arm and grip)",
        abs(fold_axis[0]) > 0.99 and abs(fold_axis[1]) < 0.01 and abs(fold_axis[2]) < 0.01,
        details=f"axis={fold_axis}",
    )
    ctx.check(
        "fold joint has realistic motion limits (0 to ~90 degrees)",
        fold_joint.motion_limits is not None
        and fold_joint.motion_limits.lower is not None
        and fold_joint.motion_limits.upper is not None
        and abs(fold_joint.motion_limits.lower) < 0.1
        and 1.0 < fold_joint.motion_limits.upper < 2.0,
        details=f"limits={fold_joint.motion_limits}",
    )

    # --- Crank arm lives on the +Y (right) side, outboard of housing ------
    h_aabb = ctx.part_world_aabb(housing)
    arm_aabb = ctx.part_world_aabb(crank_arm)
    ctx.check(
        "crank arm sits on the +Y side of the housing",
        arm_aabb is not None and h_aabb is not None and arm_aabb[1][1] > h_aabb[1][1],
        details=f"arm_max_y={arm_aabb[1][1] if arm_aabb else None}, housing_max_y={h_aabb[1][1] if h_aabb else None}",
    )

    # --- Actuating the axle joint rotates the whole crank assembly --------
    rest_arm_aabb = ctx.part_world_aabb(crank_arm)
    with ctx.pose({axle_joint: math.pi / 2.0}):
        quarter_arm_aabb = ctx.part_world_aabb(crank_arm)
    ctx.check(
        "axle rotation moves the crank arm (quarter turn swings in X)",
        rest_arm_aabb is not None and quarter_arm_aabb is not None
        and abs(quarter_arm_aabb[1][0] - rest_arm_aabb[1][0]) > 0.01,
        details=f"rest_max_x={rest_arm_aabb[1][0] if rest_arm_aabb else None}, quarter_max_x={quarter_arm_aabb[1][0] if quarter_arm_aabb else None}",
    )

    # --- Fold joint: deployed vs stowed poses -----------------------------
    grip_rest_aabb = ctx.part_world_aabb(crank_grip)
    with ctx.pose({fold_joint: math.pi / 2.0}):
        grip_folded_aabb = ctx.part_world_aabb(crank_grip)

    # At deployed (q=0) the grip extends outboard in +Y from the arm tip.
    # The grip max Y should be well beyond the arm's max Y.
    ctx.check(
        "deployed grip extends outboard beyond the arm (in +Y)",
        grip_rest_aabb is not None and arm_aabb is not None
        and grip_rest_aabb[1][1] > arm_aabb[1][1] + 0.005,
        details=f"grip_max_y={grip_rest_aabb[1][1] if grip_rest_aabb else None}, arm_max_y={arm_aabb[1][1] if arm_aabb else None}",
    )

    # When folded (q=π/2), the grip's Y extent should shrink (it folds along Z)
    # and its Z extent should change as it swings parallel to the arm.
    ctx.check(
        "folding the grip reduces its Y extent (grip folds along arm)",
        grip_rest_aabb is not None and grip_folded_aabb is not None
        and (grip_folded_aabb[1][1] - grip_folded_aabb[0][1])
        < (grip_rest_aabb[1][1] - grip_rest_aabb[0][1]) - 0.005,
        details=f"deployed_dy={grip_rest_aabb[1][1] - grip_rest_aabb[0][1] if grip_rest_aabb else None}, folded_dy={grip_folded_aabb[1][1] - grip_folded_aabb[0][1] if grip_folded_aabb else None}",
    )

    # When folded, the grip Z extent should change noticeably (it swings from
    # near the arm tip to extending back along the arm).
    ctx.check(
        "folding the grip changes its Z extent",
        grip_rest_aabb is not None and grip_folded_aabb is not None
        and abs(
            (grip_folded_aabb[1][2] - grip_folded_aabb[0][2])
            - (grip_rest_aabb[1][2] - grip_rest_aabb[0][2])
        ) > 0.005,
        details=f"deployed_dz={grip_rest_aabb[1][2] - grip_rest_aabb[0][2] if grip_rest_aabb else None}, folded_dz={grip_folded_aabb[1][2] - grip_folded_aabb[0][2] if grip_folded_aabb else None}",
    )

    # --- Grip is connected to the arm (fold joint origin at arm tip) ------
    ctx.expect_overlap(
        crank_grip,
        crank_arm,
        axes="xy",
        min_overlap=0.002,
        name="grip overlaps arm in XY at the knuckle",
    )

    # --- Front pencil port + cutter present and located -------------------
    ctx.allow_overlap(
        cutter,
        housing,
        elem_a="cutter_body",
        elem_b="housing_shell",
        reason="The milling cutter is intentionally seated/captured inside the front pencil port throat of the housing.",
    )
    ctx.allow_overlap(
        crank_arm,
        housing,
        elem_a="crank_arm_body",
        elem_b="housing_shell",
        reason="The crank arm axle stub is intentionally captured in the housing axle bore so the crank reads as mounted, not floating.",
    )
    # The grip knuckle sleeve wraps around the arm's knuckle barrel (barrel hinge).
    ctx.allow_overlap(
        crank_grip,
        crank_arm,
        elem_a="crank_grip_body",
        elem_b="crank_arm_body",
        reason="The grip knuckle sleeve wraps around the arm knuckle barrel to form a visible barrel-hinge at the fold joint.",
    )

    cut_aabb = ctx.part_world_aabb(cutter)
    ctx.check(
        "cutter sits inside the front port region (near port center height)",
        cut_aabb is not None
        and (cut_aabb[0][2] < PORT_CENTER_Z < cut_aabb[1][2]),
        details=f"cutter_aabb={cut_aabb}",
    )
    front_face_x = h_aabb[1][0]
    ctx.check(
        "cutter front end is recessed behind the front housing face",
        cut_aabb is not None and cut_aabb[1][0] < front_face_x - 0.002,
        details=f"cutter_max_x={cut_aabb[1][0] if cut_aabb else None}, front_face_x={front_face_x}",
    )

    # --- Two clamp posts rise above the deck ------------------------------
    deck_z = BODY_H + SHOULDER_H
    for post, pname in ((post1, "clamp_post_1"), (post2, "clamp_post_2")):
        p_aabb = ctx.part_world_aabb(post)
        ctx.check(
            f"{pname} rises above the top deck",
            p_aabb is not None and p_aabb[1][2] > deck_z + 0.008,
            details=f"{pname}_max_z={p_aabb[1][2] if p_aabb else None}",
        )
    ctx.expect_origin_distance(post1, post2, axes="x", min_dist=0.03)

    # --- Connectivity: crank axle reaches the housing wall ----------------
    ctx.expect_overlap(
        crank_arm,
        housing,
        axes="xz",
        min_overlap=0.004,
        name="crank arm axle aligns with the housing axle boss",
    )

    return ctx.report()


object_model = build_object_model()
