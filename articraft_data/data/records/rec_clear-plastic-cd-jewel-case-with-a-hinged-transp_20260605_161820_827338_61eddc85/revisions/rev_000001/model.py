from __future__ import annotations

# Standard clear-plastic CD jewel case (lying open).
#
# Coordinate convention:
#   - The case lies flat in the XY plane, "up" is +Z.
#   - X is the case width (~0.142 m), Y is the case depth (~0.125 m).
#   - The rear hinge runs left-right along the back +Y edge; the lid swings up
#     about that hinge (axis parallel to X).
#   - The front finger-notches are on the -Y edge.
#   - z=0 is the underside of the base frame; the assembly stacks upward in +Z.
#
# Parts / articulations:
#   - base (root): clear base frame + fixed dark inner tray with raised hub +
#     rosette and front finger notches. STATIC.
#   - lid: clear hinged transparent cover. REVOLUTE about the rear hinge
#     (axis ~ +X), opens 0 -> ~70 deg.
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
CASE_W = 0.142  # X extent
CASE_D = 0.125  # Y extent
BASE_T = 0.006  # base frame thickness (Z)
WALL = 0.004  # frame wall thickness
LID_T = 0.005  # lid shell thickness (Z)
LID_WALL = 0.0016  # lid shell wall thickness

HINGE_Y = CASE_D / 2.0  # rear hinge line at +Y edge
HINGE_Z = BASE_T  # hinge axis height (top of base frame)

DISC_R = 0.060  # CD outer radius (120 mm disc)
DISC_T = 0.0012  # CD thickness
DISC_HOLE_R = 0.0075  # CD center hole radius

HUB_R = 0.009  # raised tray hub radius
HUB_H = 0.006  # hub height above tray floor

TRAY_FLOOR_Z = BASE_T  # tray floor sits at top of base frame


def _shallow_shell(w: float, d: float, wall: float, depth: float) -> cq.Workplane:
    # A shallow open tray/shell: an open-topped box with thin walls and floor.
    # Outer box from z=0..depth, hollowed from the top leaving `wall` walls/floor.
    outer = cq.Workplane("XY").box(w, d, depth, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .workplane(offset=wall)
        .box(w - 2 * wall, d - 2 * wall, depth, centered=(True, True, False))
    )
    return outer.cut(inner)


def _base_frame() -> cq.Workplane:
    # Clear base frame: a shallow rectangular shell (open top) that holds the tray.
    return _shallow_shell(CASE_W, CASE_D, WALL, BASE_T)


def _tray_solid() -> cq.Workplane:
    # Dark inner tray that nests inside the base frame. A thin rectangular floor
    # slightly smaller than the base interior, with a raised center hub + rosette
    # and two front finger-notch cutouts on the -Y edge.
    # Tray floor spans the full base interior so it seats against the frame walls
    # (a tiny overlap into the walls keeps the assembly geometrically connected).
    tw = CASE_W - 2 * WALL + 0.0008
    td = CASE_D - 2 * WALL + 0.0008
    floor_t = 0.0022
    tray = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z)
        .box(tw, td, floor_t, centered=(True, True, False))
    )
    # Raised circular hub boss at the center.
    hub = (
        cq.Workplane("XY")
        .workplane(offset=TRAY_FLOOR_Z + floor_t)
        .circle(HUB_R)
        .extrude(HUB_H)
    )
    tray = tray.union(hub)
    # Rosette: small radial teeth ring around the hub (the disc-retaining flexure).
    ros_z = TRAY_FLOOR_Z + floor_t
    teeth = cq.Workplane("XY").workplane(offset=ros_z)
    n_teeth = 8
    for i in range(n_teeth):
        a = 2 * math.pi * i / n_teeth
        tx = (HUB_R + 0.0015) * math.cos(a)
        ty = (HUB_R + 0.0015) * math.sin(a)
        teeth = (
            teeth.moveTo(tx, ty)
            .circle(0.0016)
            .extrude(HUB_H * 0.7)
        )
    tray = tray.union(teeth)
    # Front finger-notch cutouts: two scoops on the -Y front edge so a finger can
    # lift the disc. Cut shallow half-cylinders out of the front floor edge.
    notch = cq.Workplane("XY").workplane(offset=TRAY_FLOOR_Z - 0.001)
    for nx in (-0.026, 0.026):
        notch = (
            notch.moveTo(nx, -td / 2.0)
            .circle(0.013)
            .extrude(floor_t + 0.003)
        )
    tray = tray.cut(notch)
    return tray


def _lid_shell() -> cq.Workplane:
    # Clear hinged cover: a shallow shell that mirrors the base, opening downward
    # in its local frame so that at q=0 (closed) it caps the case. Authored in the
    # lid-local frame with its hinge edge at local y=0 and the body extending to -Y.
    # Local: open side faces -Z (toward the case when closed).
    w = CASE_W
    d = CASE_D
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-LID_T)
        .box(w, d, LID_T, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-LID_T)
        .box(w - 2 * LID_WALL, d - 2 * LID_WALL, LID_T - LID_WALL, centered=(True, True, False))
    )
    lid = outer.cut(inner)
    return lid


def _disc_solid() -> cq.Workplane:
    # Silver CD: thin cylinder with a central hole (boolean cut).
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
    model = ArticulatedObject(name="cd_jewel_case")

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

    # ---- lid: clear hinged transparent cover, revolute about the rear hinge ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell"),
        material=clear,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, LID_T)),
        mass=0.035,
        origin=Origin(xyz=(0.0, -CASE_D / 2.0, -LID_T / 2.0)),
    )
    # Hinge frame at the rear +Y edge, at the top of the base frame. The lid
    # local origin sits on the hinge line; its body extends to local -Y. Axis +X
    # so positive q lifts the front (-Y) edge up in +Z.
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(70.0)),
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


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    disc = object_model.get_part("disc")
    lid_joint = object_model.get_articulation("base_to_lid")
    disc_joint = object_model.get_articulation("hub_to_disc")

    # --- lid is clear/transparent (material alpha < 1) ---
    lid_mat = lid.get_visual("lid_shell").material
    alpha = lid_mat.rgba[3] if lid_mat.rgba is not None else 1.0
    ctx.check(
        "lid is transparent (alpha < 1)",
        alpha < 0.6,
        details=f"lid alpha={alpha}",
    )
    base_mat = base.get_visual("base_frame").material
    base_alpha = base_mat.rgba[3] if base_mat.rgba is not None else 1.0
    ctx.check(
        "base frame is clear plastic (alpha < 1)",
        base_alpha < 0.6,
        details=f"base alpha={base_alpha}",
    )

    # --- lid hinges about the rear edge: opening lifts the front edge up ---
    closed = _ext(ctx.part_world_aabb(lid))
    closed_top = ctx.part_world_aabb(lid)[1][2]
    closed_front_y = ctx.part_world_aabb(lid)[0][1]
    with ctx.pose({lid_joint: math.radians(60.0)}):
        open_aabb = ctx.part_world_aabb(lid)
        open_top = open_aabb[1][2]
        open_front_y = open_aabb[0][1]
    ctx.check(
        "lid swings open and lifts upward",
        open_top > closed_top + 0.03,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )
    ctx.check(
        "opening swings the free edge toward the hinge (front y moves rearward)",
        open_front_y > closed_front_y + 0.02,
        details=f"closed_front_y={closed_front_y}, open_front_y={open_front_y}",
    )

    # Hinge sits at the rear edge, near the top of the base frame.
    ctx.check(
        "hinge axis runs along X (left-right)",
        abs(lid_joint.axis[0]) > 0.9,
        details=f"axis={lid_joint.axis}",
    )

    # --- disc spins about the vertical hub axis (off-center marker rotates) ---
    marker_rest = ctx.part_element_world_aabb(disc, elem="disc_marker")
    rest_cx = 0.5 * (marker_rest[0][0] + marker_rest[1][0])
    rest_cy = 0.5 * (marker_rest[0][1] + marker_rest[1][1])
    with ctx.pose({disc_joint: math.radians(90.0)}):
        marker_q = ctx.part_element_world_aabb(disc, elem="disc_marker")
        q_cx = 0.5 * (marker_q[0][0] + marker_q[1][0])
        q_cy = 0.5 * (marker_q[0][1] + marker_q[1][1])
    # Quarter turn about +Z: marker at (+0.03, 0) -> (0, +0.03).
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
    # Disc rests at/above the tray floor (on the hub), not floating high.
    disc_bottom = ctx.part_world_aabb(disc)[0][2]
    ctx.check(
        "disc rests on the tray hub above the tray floor",
        TRAY_FLOOR_Z - 0.001 < disc_bottom < TRAY_FLOOR_Z + HUB_H + 0.002,
        details=f"disc_bottom={disc_bottom}, floor={TRAY_FLOOR_Z}",
    )

    # --- tray is seated in the base frame (the dark tray nests in the clear base) ---
    # The tray floor seats against the frame interior walls with a tiny intentional
    # embed so the two bodies read as one connected base assembly.
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
