from __future__ import annotations

# Variant: manual pencil sharpener with a flat steel wedge blade seated across
# the conical pencil port (user twists the pencil by hand against the fixed
# blade) and a revolute hinged shavings lid on the rear face. The crank-driven
# helical cutter has been removed entirely. The primary user-facing mechanism
# is now the shavings lid that swings open to empty the housing.

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
#   +Y : toward the RIGHT side
#   +Z : up
# ---------------------------------------------------------------------------

# Housing outer extents.
BODY_W = 0.090  # depth along X
BODY_D = 0.082  # width along Y
BODY_H = 0.078  # tall lower body
BODY_FILLET = 0.012

# Upper shoulder block that the clamp posts attach to.
SHOULDER_H = 0.026
SHOULDER_W = 0.084
SHOULDER_D = 0.078

# Pencil port (front face) geometry.
PORT_CENTER_Z = 0.040
PORT_OUTER_R = 0.018
PORT_THROAT_R = 0.0075
PORT_DEPTH = 0.022

# Shavings compartment access opening on the rear (-X) face.
SHAVINGS_W = 0.055
SHAVINGS_H = 0.038
SHAVINGS_BOTTOM_Z = 0.014

# Shavings lid (hinged cover).
LID_W = 0.060
LID_H = 0.042
LID_T = 0.003
HINGE_R = 0.003
HINGE_LEN = 0.052

# Flat wedge blade.
BLADE_W = 0.022
BLADE_H = 0.016
BLADE_T = 0.0008

# Derived positions.
REAR_X = -BODY_W / 2.0
HINGE_Z = SHAVINGS_BOTTOM_Z + SHAVINGS_H  # top of shavings opening


def _rounded_box(width: float, depth: float, height: float, fillet: float) -> cq.Workplane:
    """Solid rounded box centered on XY, base at z=0, vertical edges filleted."""
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(fillet)
    )


def _build_housing() -> cq.Workplane:
    """Hollowed charcoal housing with front pencil port, upper shoulder,
    and a rectangular shavings access opening on the rear face."""
    body = _rounded_box(BODY_W, BODY_D, BODY_H, BODY_FILLET)

    # Upper shoulder block (slightly inset) that carries the clamp posts.
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
    # Tapered throat leading toward the blade (funnel).
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

    # Shavings access opening on the rear face (-X).
    opening = (
        cq.Workplane("YZ")
        .workplane(offset=REAR_X - 0.001)
        .center(0.0, SHAVINGS_BOTTOM_Z + SHAVINGS_H / 2.0)
        .rect(SHAVINGS_W, SHAVINGS_H)
        .extrude(0.012)
    )
    body = body.cut(opening)

    # Hinge barrel recess: a small cylindrical pocket at the top of the
    # shavings opening where the lid hinge barrel seats.
    hinge_recess = (
        cq.Workplane("XZ")
        .workplane(offset=-(HINGE_LEN / 2.0 + 0.001))
        .center(REAR_X + 0.003, HINGE_Z)
        .circle(HINGE_R + 0.0008)
        .extrude(HINGE_LEN + 0.002)
    )
    body = body.cut(hinge_recess)

    return body


def _build_blade() -> cq.Workplane:
    """Flat steel wedge blade: a thin plate with a beveled cutting edge."""
    # Build the blade as a thin rectangular plate.
    blade = (
        cq.Workplane("XY")
        .box(BLADE_T, BLADE_W, BLADE_H, centered=(True, True, True))
    )
    # Add a small bevel on one long edge to represent the cutting edge.
    # The bevel is a thin triangular wedge along the bottom edge.
    bevel = (
        cq.Workplane("XY")
        .workplane(offset=-BLADE_H / 2.0)
        .center(0.0, 0.0)
        .rect(BLADE_T + 0.0006, BLADE_W)
        .workplane(offset=0.002)
        .rect(BLADE_T, BLADE_W)
        .loft(combine=False)
    )
    blade = blade.union(bevel)
    return blade


def _build_front_plate() -> cq.Workplane:
    """Thin badge plate on the lower front face (the logo strip)."""
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
    # Knurled cap groove (small recessed ring near the top).
    groove = (
        cq.Workplane("XY")
        .workplane(offset=deck_z + 0.0095)
        .circle(0.0066)
        .extrude(0.0012)
    )
    return post.cut(groove.intersect(post)).union(post)


def _build_shavings_lid() -> cq.Workplane:
    """Hinged shavings cover: panel hanging down from a hinge barrel.

    Authored with origin at the hinge pin center. Panel extends in -Z
    (downward when closed) with thickness in +X (toward housing interior).
    The hinge barrel runs along Y at the origin.
    """
    # Lid panel: from z=-LID_H to z=0, x from 0 to LID_T, y centered.
    panel = (
        cq.Workplane("YZ")
        .workplane(offset=0.0)
        .center(0.0, -LID_H / 2.0)
        .rect(LID_W, LID_H)
        .extrude(LID_T)
    )
    # Round the bottom corners of the panel slightly.
    panel = panel.edges("|X and <Z").fillet(0.003)

    # Small finger tab/lip at the bottom center for opening.
    tab = (
        cq.Workplane("YZ")
        .workplane(offset=-0.001)
        .center(0.0, -LID_H + 0.004)
        .rect(0.016, 0.008)
        .extrude(0.001)
        .edges("|X and <Z")
        .fillet(0.002)
    )
    panel = panel.union(tab)

    # Hinge barrel: cylinder along Y at the top (origin).
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=-HINGE_LEN / 2.0)
        .circle(HINGE_R)
        .extrude(HINGE_LEN)
    )
    # Small end caps on the barrel for visual detail.
    cap_l = (
        cq.Workplane("XZ")
        .workplane(offset=-HINGE_LEN / 2.0 - 0.001)
        .circle(HINGE_R + 0.0008)
        .extrude(0.002)
    )
    cap_r = (
        cq.Workplane("XZ")
        .workplane(offset=HINGE_LEN / 2.0 - 0.001)
        .circle(HINGE_R + 0.0008)
        .extrude(0.002)
    )

    return panel.union(barrel).union(cap_l).union(cap_r)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pencil_sharpener")

    charcoal = model.material("charcoal_plastic", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.14, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    badge = model.material("badge_silver", rgba=(0.78, 0.79, 0.80, 1.0))
    lid_finish = model.material("lid_plastic", rgba=(0.25, 0.26, 0.28, 1.0))

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
    # Flat steel blade seated across the conical port throat (fixed, inlined).
    housing.visual(
        mesh_from_cadquery(_build_blade(), "blade"),
        material=steel,
        origin=Origin(
            xyz=(BODY_W / 2.0 - PORT_DEPTH * 0.45, 0.0, PORT_CENTER_Z),
            rpy=(0.0, 0.44, 0.0),  # ~25° tilt about Y
        ),
        name="blade",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H + SHOULDER_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + SHOULDER_H) / 2.0)),
    )

    # --- Two top clamp posts (fixed, repeated via loop) -------------------
    deck_z = BODY_H + SHOULDER_H
    for idx, x_off in enumerate((-0.020, 0.020)):
        post = model.part(f"clamp_post_{idx}")
        post.visual(
            mesh_from_cadquery(_build_clamp_post(), f"clamp_post_{idx}"),
            material=dark_metal,
            name=f"clamp_post_shell_{idx}",
        )
        post.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0065, length=0.014),
            mass=0.01,
            origin=Origin(xyz=(x_off, 0.012, deck_z + 0.007)),
        )
        model.articulation(
            f"housing_to_clamp_post_{idx}",
            ArticulationType.FIXED,
            parent=housing,
            child=post,
            origin=Origin(xyz=(x_off, 0.012, 0.0)),
        )

    # --- Shavings lid (PRIMARY mechanism: REVOLUTE hinge) -----------------
    lid = model.part("shavings_lid")
    lid.visual(
        mesh_from_cadquery(_build_shavings_lid(), "shavings_lid"),
        material=lid_finish,
        name="lid_panel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_T, LID_W, LID_H)),
        mass=0.02,
        origin=Origin(xyz=(LID_T / 2.0, 0.0, -LID_H / 2.0)),
    )

    # The lid part frame origin is at the hinge pin center. The articulation
    # frame is placed at the top of the shavings opening on the rear face.
    # Axis (0, 1, 0): positive q swings the lid bottom outward (-X) and up.
    model.articulation(
        "housing_to_lid",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(REAR_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    lid = object_model.get_part("shavings_lid")
    post0 = object_model.get_part("clamp_post_0")
    post1 = object_model.get_part("clamp_post_1")
    lid_joint = object_model.get_articulation("housing_to_lid")

    # --- Mechanism: lid joint type + axis ---------------------------------
    ctx.check(
        "lid is a revolute joint",
        str(lid_joint.joint_type).lower().endswith("revolute"),
        details=f"joint_type={lid_joint.joint_type}",
    )
    axis = tuple(lid_joint.axis)
    ctx.check(
        "lid hinge axis is horizontal along Y",
        abs(axis[1]) > 0.99 and abs(axis[0]) < 0.01 and abs(axis[2]) < 0.01,
        details=f"axis={axis}",
    )

    # --- Lid motion limits are bounded ------------------------------------
    limits = lid_joint.motion_limits
    ctx.check(
        "lid has bounded motion limits",
        limits is not None and limits.lower is not None and limits.upper is not None,
        details=f"limits={limits}",
    )
    if limits and limits.lower is not None and limits.upper is not None:
        ctx.check(
            "lid upper limit allows meaningful opening (>1 rad)",
            limits.upper > 1.0,
            details=f"upper={limits.upper}",
        )

    # --- Lid lives on the rear (-X) side of the housing -------------------
    h_aabb = ctx.part_world_aabb(housing)
    l_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid sits on the -X (rear) side of the housing",
        l_aabb is not None and h_aabb is not None and l_aabb[0][0] < h_aabb[0][0] + 0.005,
        details=f"lid_min_x={l_aabb[0][0] if l_aabb else None}, housing_min_x={h_aabb[0][0] if h_aabb else None}",
    )

    # --- Opening the lid moves it outward ---------------------------------
    rest_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.2}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "opening the lid moves the bottom edge outward in -X",
        rest_aabb is not None and open_aabb is not None
        and open_aabb[0][0] < rest_aabb[0][0] - 0.01,
        details=f"rest_min_x={rest_aabb[0][0] if rest_aabb else None}, open_min_x={open_aabb[0][0] if open_aabb else None}",
    )

    # --- Blade is present inside the front port region --------------------
    blade_visual = housing.get_visual("blade")
    ctx.check(
        "blade visual exists on the housing",
        blade_visual is not None,
        details="blade visual not found",
    )

    # The blade should be located near the port center height.
    # Use element-level AABB to verify.
    blade_aabb = ctx.part_element_world_aabb(housing, elem="blade")
    ctx.check(
        "blade sits near the port center height",
        blade_aabb is not None
        and blade_aabb[0][2] < PORT_CENTER_Z < blade_aabb[1][2],
        details=f"blade_aabb={blade_aabb}",
    )
    # Blade should be in the front half of the housing (near the port).
    ctx.check(
        "blade is located in the front portion of the housing",
        blade_aabb is not None and blade_aabb[0][0] > 0.0,
        details=f"blade_min_x={blade_aabb[0][0] if blade_aabb else None}",
    )

    # --- Front pencil port exists (housing has the port geometry) ----------
    # The housing front face should extend to approximately BODY_W/2.
    ctx.check(
        "housing front face reaches the port region",
        h_aabb is not None and h_aabb[1][0] > BODY_W / 2.0 - 0.005,
        details=f"housing_max_x={h_aabb[1][0] if h_aabb else None}",
    )

    # --- Two clamp posts rise above the deck ------------------------------
    deck_z = BODY_H + SHOULDER_H
    for post, name in ((post0, "clamp_post_0"), (post1, "clamp_post_1")):
        p_aabb = ctx.part_world_aabb(post)
        ctx.check(
            f"{name} rises above the top deck",
            p_aabb is not None and p_aabb[1][2] > deck_z + 0.008,
            details=f"{name}_max_z={p_aabb[1][2] if p_aabb else None}",
        )
    # The two posts are distinct in X.
    ctx.expect_origin_distance(post0, post1, axes="x", min_dist=0.03)

    # --- Lid hinge barrel connects to the housing at the hinge point ------
    ctx.allow_overlap(
        lid,
        housing,
        elem_a="lid_panel",
        elem_b="housing_shell",
        reason="The lid hinge barrel is intentionally seated in the housing hinge recess for a captured hinge fit.",
    )
    ctx.expect_contact(
        lid,
        housing,
        elem_a="lid_panel",
        elem_b="housing_shell",
        contact_tol=0.005,
        name="lid hinge barrel contacts the housing at the hinge point",
    )

    # --- Shavings opening exists on the rear face -------------------------
    # The lid should roughly cover the opening area when closed.
    ctx.expect_overlap(
        lid,
        housing,
        axes="yz",
        min_overlap=0.020,
        name="lid covers the shavings opening area in YZ projection",
    )

    return ctx.report()


object_model = build_object_model()
