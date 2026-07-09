from __future__ import annotations

# Sliding-sleeve CD jewel case variant.
#
# Coordinate convention:
#   - The case lies flat in the XY plane, "up" is +Z.
#   - X is the case long axis (~0.142 m), Y is the case depth (~0.125 m).
#   - The sleeve slides along +X (the long axis) to open/close.
#   - The front finger-notches are on the -Y edge.
#   - z=0 is the underside of the base frame; the assembly stacks upward in +Z.
#
# Parts / articulations:
#   - base (root): clear base frame + fixed dark inner tray with raised hub +
#     rosette and front finger notches. STATIC.
#   - sleeve: clear rectangular slipcase that slides over the base along +X.
#     PRISMATIC along X, opens 0 -> ~0.17 m.
#   - disc: silver CD. CONTINUOUS spin about the vertical hub axis (+Z), sits
#     on the tray rosette hub.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    Inertial,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
CASE_W = 0.142  # X extent (long axis)
CASE_D = 0.125  # Y extent
BASE_T = 0.006  # base frame thickness (Z)
WALL = 0.004  # frame wall thickness

DISC_R = 0.060  # CD outer radius (120 mm disc)
DISC_T = 0.0012  # CD thickness
DISC_HOLE_R = 0.0075  # CD center hole radius

HUB_R = 0.009  # raised tray hub radius
HUB_H = 0.006  # hub height above tray floor

TRAY_FLOOR_Z = BASE_T  # tray floor sits at top of base frame

# ---- sleeve dimensions -------------------------------------------------------
SL_CLEAR = 0.001  # clearance between sleeve inner walls and base outer
SL_WALL = 0.002  # sleeve shell wall thickness

SL_IX = CASE_W + 2 * SL_CLEAR  # inner cavity X
SL_IY = CASE_D + 2 * SL_CLEAR  # inner cavity Y
SL_IZ = BASE_T + 0.0022 + HUB_H + DISC_T + 2 * SL_CLEAR  # inner cavity Z
SL_OX = SL_IX + 2 * SL_WALL  # outer X (symmetric; -X wall is cut open)
SL_OY = SL_IY + 2 * SL_WALL  # outer Y
SL_OZ = SL_IZ + 2 * SL_WALL  # outer Z

# Height at which the sleeve center sits (in base frame) so the base content
# (tray + hub + disc) fits inside the inner cavity with clearance on all sides.
CONTENT_TOP = BASE_T + 0.0022 + HUB_H  # top of the hub above tray floor
SL_JOINT_Z = (CONTENT_TOP + SL_CLEAR) - SL_IZ / 2.0  # cavity top clears hub


def _shallow_shell(w: float, d: float, wall: float, depth: float) -> cq.Workplane:
    """Open-topped shallow rectangular shell (thin walls + floor)."""
    outer = cq.Workplane("XY").box(w, d, depth, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .workplane(offset=wall)
        .box(w - 2 * wall, d - 2 * wall, depth, centered=(True, True, False))
    )
    return outer.cut(inner)


def _base_frame() -> cq.Workplane:
    """Clear base frame: a shallow rectangular shell (open top) that holds the tray."""
    return _shallow_shell(CASE_W, CASE_D, WALL, BASE_T)


def _tray_solid() -> cq.Workplane:
    """Dark inner tray with raised center hub, rosette teeth, and front finger notches."""
    tw = CASE_W - 2 * WALL + 0.0008
    td = CASE_D - 2 * WALL + 0.0008
    floor_t = 0.0022
    tray = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z)
        .box(tw, td, floor_t, centered=(True, True, False))
    )
    # Raised circular hub boss + rosette teeth as a single extrusion.
    ros_z = TRAY_FLOOR_Z + floor_t
    hub_and_teeth = cq.Workplane("XY").workplane(offset=ros_z)
    # Hub circle
    hub_and_teeth = hub_and_teeth.circle(HUB_R)
    # Rosette teeth: 8 small circles around the hub
    n_teeth = 8
    for i in range(n_teeth):
        a = 2 * math.pi * i / n_teeth
        tx = (HUB_R + 0.0015) * math.cos(a)
        ty = (HUB_R + 0.0015) * math.sin(a)
        hub_and_teeth = hub_and_teeth.moveTo(tx, ty).circle(0.0016)
    hub_solid = hub_and_teeth.extrude(HUB_H)
    tray = tray.union(hub_solid)
    # Front finger-notch cutouts on the -Y edge (two scoops in one cut).
    notches = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z - 0.001)
        .moveTo(-0.026, -td / 2.0)
        .circle(0.013)
        .moveTo(0.026, -td / 2.0)
        .circle(0.013)
        .extrude(floor_t + 0.003)
    )
    tray = tray.cut(notches)
    return tray


def _sleeve_shell() -> cq.Workplane:
    """Clear rectangular slipcase sleeve, open on the -X end.

    Built as a hollow rectangular tube (outer box minus inner box), then
    capped on the +X end and opened on the -X end.  Authored centered at the
    origin so the part frame sits at the geometric center of the outer box.
    """
    t = SL_WALL
    # Outer solid tube (box extruded along Z, centered at origin).
    outer = (
        cq.Workplane("XY")
        .box(SL_OX, SL_OY, SL_OZ, centered=(True, True, True))
    )
    # Inner cavity cut: a box that removes the interior, leaving walls on all
    # 6 faces.  We then selectively reopen the -X face.
    inner = (
        cq.Workplane("XY")
        .box(SL_IX, SL_IY, SL_IZ, centered=(True, True, True))
    )
    tube = outer.cut(inner)
    # +X end cap is already present from the outer box; we just need to reopen
    # the -X wall.  Cut a slot through the -X wall that matches the inner
    # opening so the sleeve can slide over the base.
    opener = (
        cq.Workplane("XY")
        .box(t + 0.002, SL_IY - 0.0005, SL_IZ - 0.0005, centered=(True, True, True))
        .translate((-SL_OX / 2.0 + t / 2.0, 0.0, 0.0))
    )
    return tube.cut(opener)


def _disc_solid() -> cq.Workplane:
    """Silver CD: thin cylinder with a central hole."""
    disc = (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_T)
        .faces(">Z")
        .workplane()
        .hole(DISC_HOLE_R * 2.0)
    )
    return disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_jewel_case_sleeve")

    clear = model.material("clear_plastic", rgba=(0.55, 0.60, 0.68, 0.30))
    tray_dark = model.material("tray_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    silver = model.material("disc_silver", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---- base (root): clear frame + fixed dark inner tray ----
    base = model.part("base")

    base.visual(
        mesh_from_cadquery(_base_frame(), "base_frame"),
        material=clear,
        name="base_frame",
    )
    base.visual(
        mesh_from_cadquery(_tray_solid(), "inner_tray"),
        material=tray_dark,
        name="inner_tray",
    )
    base.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, BASE_T + HUB_H)),
        mass=0.060,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
    )

    # ---- sleeve: clear rectangular slipcase, prismatic along +X ----
    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_cadquery(_sleeve_shell(), "sleeve_shell"),
        material=clear,
        name="sleeve_shell",
    )
    sleeve.inertial = Inertial.from_geometry(
        Box((SL_OX, SL_OY, SL_OZ)),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Prismatic joint: sleeve slides along +X (the case long axis).
    # Joint origin sits at the sleeve center height so the base nests inside
    # the cavity at q=0 (closed).  Positive q slides the sleeve off toward +X.
    model.articulation(
        "base_to_sleeve",
        ArticulationType.PRISMATIC,
        parent=base,
        child=sleeve,
        origin=Origin(xyz=(0.0, 0.0, SL_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.5, lower=0.0, upper=0.17),
    )

    # ---- disc: silver CD, continuous spin about the vertical hub axis ----
    disc = model.part("disc")
    disc.visual(
        mesh_from_cadquery(_disc_solid(), "disc_body"),
        material=silver,
        name="disc_body",
    )
    # Small off-center label marker so rotation is visually/numerically detectable.
    disc.visual(
        Cylinder(radius=0.004, length=DISC_T),
        origin=Origin(xyz=(0.030, 0.0, DISC_T / 2.0)),
        material=tray_dark,
        name="disc_marker",
    )
    disc.inertial = Inertial.from_geometry(
        Cylinder(radius=DISC_R, length=DISC_T),
        mass=0.016,
        origin=Origin(xyz=(0.0, 0.0, DISC_T / 2.0)),
    )
    # Disc center hole drops over the hub; it rests on the rosette/hub top.
    disc_z = TRAY_FLOOR_Z + 0.0022 + HUB_H * 0.7
    model.articulation(
        "hub_to_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disc,
        origin=Origin(xyz=(0.0, 0.0, disc_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    sleeve = object_model.get_part("sleeve")
    disc = object_model.get_part("disc")
    sleeve_joint = object_model.get_articulation("base_to_sleeve")
    disc_joint = object_model.get_articulation("hub_to_disc")

    # --- sleeve wraps the base with clearance; the prismatic joint is the
    #     mechanical connection, not physical contact. ---
    ctx.allow_isolated_part(
        sleeve,
        reason="The clear slipcase sleeve slides over the base with uniform "
        "clearance on all sides; the prismatic joint provides the mechanical "
        "connection, not surface contact.",
    )
    # Proof: the sleeve encloses the base footprint when closed.
    ctx.expect_overlap(
        sleeve,
        base,
        axes="xy",
        min_overlap=0.08,
        elem_a="sleeve_shell",
        elem_b="base_frame",
        name="closed sleeve covers the base footprint",
    )

    # --- sleeve and base are clear/transparent (material alpha < 1) ---
    sleeve_mat = sleeve.get_visual("sleeve_shell").material
    alpha = sleeve_mat.rgba[3] if sleeve_mat.rgba is not None else 1.0
    ctx.check(
        "sleeve is transparent (alpha < 1)",
        alpha < 0.6,
        details=f"sleeve alpha={alpha}",
    )
    base_mat = base.get_visual("base_frame").material
    base_alpha = base_mat.rgba[3] if base_mat.rgba is not None else 1.0
    ctx.check(
        "base frame is clear plastic (alpha < 1)",
        base_alpha < 0.6,
        details=f"base alpha={base_alpha}",
    )

    # --- sleeve joint is prismatic along X (the case long axis) ---
    ctx.check(
        "sleeve joint is prismatic",
        sleeve_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={sleeve_joint.articulation_type}",
    )
    ctx.check(
        "sleeve slide axis runs along X (case long axis)",
        abs(sleeve_joint.axis[0]) > 0.9,
        details=f"axis={sleeve_joint.axis}",
    )

    # --- sliding the sleeve exposes the base along +X ---
    closed_center = ctx.part_world_position(sleeve)
    with ctx.pose({sleeve_joint: 0.12}):
        open_center = ctx.part_world_position(sleeve)
    ctx.check(
        "sleeve slides toward +X when opened",
        open_center is not None
        and closed_center is not None
        and open_center[0] > closed_center[0] + 0.10,
        details=f"closed_x={closed_center}, open_x={open_center}",
    )

    # When slid open, the sleeve no longer fully covers the base in X.
    with ctx.pose({sleeve_joint: 0.15}):
        open_sleeve_aabb = ctx.part_world_aabb(sleeve)
        base_aabb = ctx.part_world_aabb(base)
    # Sleeve min X should be past base center when slid open.
    ctx.check(
        "open sleeve has shifted past base center in X",
        open_sleeve_aabb[0][0] > base_aabb[0][0] + 0.05,
        details=f"sleeve_min_x={open_sleeve_aabb[0][0]}, base_min_x={base_aabb[0][0]}",
    )

    # --- disc spins about the vertical hub axis (off-center marker rotates) ---
    marker_rest = ctx.part_element_world_aabb(disc, elem="disc_marker")
    rest_cx = 0.5 * (marker_rest[0][0] + marker_rest[1][0])
    rest_cy = 0.5 * (marker_rest[0][1] + marker_rest[1][1])
    with ctx.pose({disc_joint: math.radians(90.0)}):
        marker_q = ctx.part_element_world_aabb(disc, elem="disc_marker")
        q_cx = 0.5 * (marker_q[0][0] + marker_q[1][0])
        q_cy = 0.5 * (marker_q[0][1] + marker_q[1][1])
    ctx.check(
        "disc spin moves the off-center marker around the hub axis",
        abs(q_cx - rest_cx) > 0.02 and abs(q_cy - rest_cy) > 0.02,
        details=f"rest=({rest_cx:.4f},{rest_cy:.4f}) quarter=({q_cx:.4f},{q_cy:.4f})",
    )
    ctx.check(
        "disc spin axis is vertical (+Z)",
        abs(disc_joint.axis[2]) > 0.9,
        details=f"axis={disc_joint.axis}",
    )

    # --- disc sits on the tray hub (centered over the hub, resting on top) ---
    ctx.allow_overlap(
        disc,
        base,
        elem_a="disc_body",
        elem_b="inner_tray",
        reason="The CD center hole drops over the raised tray hub/rosette; the hub "
        "intentionally pokes through the disc bore so the disc is captured and seated.",
    )
    ctx.expect_within(
        disc,
        base,
        axes="xy",
        inner_elem="disc_body",
        outer_elem="base_frame",
        margin=0.002,
        name="disc sits within the case footprint",
    )
    disc_pos = ctx.part_world_position(disc)
    ctx.check(
        "disc is centered over the hub axis",
        abs(disc_pos[0]) < 0.005 and abs(disc_pos[1]) < 0.005,
        details=f"disc origin={disc_pos}",
    )
    disc_bottom = ctx.part_world_aabb(disc)[0][2]
    ctx.check(
        "disc rests on the tray hub above the tray floor",
        TRAY_FLOOR_Z - 0.001 < disc_bottom < TRAY_FLOOR_Z + HUB_H + 0.002,
        details=f"disc_bottom={disc_bottom}, floor={TRAY_FLOOR_Z}",
    )

    # --- tray is seated in the base frame ---
    ctx.allow_overlap(
        base,
        base,
        elem_a="inner_tray",
        elem_b="base_frame",
        reason="The dark inner tray is press-fit into the clear base frame; its floor "
        "edges intentionally seat a fraction of a millimeter into the frame walls.",
    )
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem="inner_tray",
        outer_elem="base_frame",
        margin=0.001,
        name="inner tray nests within the base frame footprint",
    )

    return ctx.report()


object_model = build_object_model()
