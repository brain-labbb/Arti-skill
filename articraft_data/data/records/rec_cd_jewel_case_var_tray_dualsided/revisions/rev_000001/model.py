from __future__ import annotations

# Clear plastic CD jewel case with a dual-sided flip tray.
#
# Fork of the standard jewel case: the fixed inner tray is replaced by a
# thin panel that flips about a center pivot axis (parallel to X) so it
# presents a disc-holding hub on *both* faces.  A silver CD rides on the
# upper hub.
#
# Coordinate convention:
#   - Case lies flat in XY; "up" is +Z.
#   - X = case width (~0.142 m), Y = case depth (~0.125 m).
#   - Rear lid hinge runs along the +Y edge; the lid swings up about X.
#   - Flip-tray pivot axis is at the case center (y=0, z=PIVOT_Z).
#   - z=0 is the underside of the base frame.
#
# Parts / articulations:
#   base (root)     – clear frame only.
#   flip_tray       – thin panel, hubs+rosette on both faces, pivot pins.
#   lid             – clear hinged cover.
#   disc            – silver CD on the upper hub.
#
#   base_to_flip_tray  REVOLUTE  center X axis, 0 → π
#   base_to_lid        REVOLUTE  rear hinge X axis, 0 → 70°
#   tray_to_disc       CONTINUOUS vertical spin on upper hub

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

# ---- key dimensions (meters) -------------------------------------------------
CASE_W = 0.142
CASE_D = 0.125
BASE_T = 0.006
WALL = 0.004
LID_T = 0.005
LID_WALL = 0.0016

HINGE_Y = CASE_D / 2.0
HINGE_Z = BASE_T

DISC_R = 0.060
DISC_T = 0.0012
DISC_HOLE_R = 0.0075

HUB_R = 0.009
HUB_H = 0.006
N_TEETH = 8
TOOTH_OFFSET = 0.0015
TOOTH_R = 0.0016

TRAY_PANEL_T = 0.002
TRAY_W = CASE_W - 2 * WALL - 0.002
TRAY_D = CASE_D - 2 * WALL - 0.006
PIVOT_Z = 0.016
PIVOT_PIN_R = 0.0015
PIVOT_PIN_L = 0.003

# Pivot bearing post dimensions (extend from base frame top to pivot height)
POST_W = 0.006       # post width (X direction)
POST_D = 0.008       # post depth (Y direction)
POST_TOP = PIVOT_Z + PIVOT_PIN_R + 0.002  # post extends above pivot pin center


# ---- geometry helpers --------------------------------------------------------

def _shallow_shell(w: float, d: float, wall: float, depth: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(w, d, depth, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .workplane(offset=wall)
        .box(w - 2 * wall, d - 2 * wall, depth, centered=(True, True, False))
    )
    return outer.cut(inner)


def _base_frame() -> cq.Workplane:
    return _shallow_shell(CASE_W, CASE_D, WALL, BASE_T)


def _pivot_post(side_sign: int) -> cq.Workplane:
    """Bearing post on one side wall of the base frame.

    *side_sign* = +1 for the +X (right) side, -1 for the -X (left) side.
    The post extends from the base frame top (z=BASE_T) up to POST_TOP.
    """
    post_cx = side_sign * (CASE_W / 2.0 - WALL - POST_W / 2.0)
    post = (
        cq.Workplane("XY")
        .workplane(offset=BASE_T)
        .center(post_cx, 0.0)
        .box(POST_W, POST_D, POST_TOP - BASE_T, centered=(True, True, False))
    )
    return post


def _hub_face(face_sign: int) -> cq.Workplane:
    """Hub boss + rosette teeth on one face of the flip tray.

    *face_sign* = +1 for the top (+Z) face, -1 for the bottom (-Z) face.
    The tray local origin is the pivot axis; the panel surfaces sit at
    z = ±TRAY_PANEL_T / 2.
    """
    face_z = face_sign * TRAY_PANEL_T / 2.0

    if face_sign > 0:
        hub = (
            cq.Workplane("XY")
            .workplane(offset=face_z)
            .circle(HUB_R)
            .extrude(HUB_H)
        )
        teeth_z0 = face_z
    else:
        hub = (
            cq.Workplane("XY")
            .workplane(offset=face_z - HUB_H)
            .circle(HUB_R)
            .extrude(HUB_H)
        )
        teeth_z0 = face_z - HUB_H * 0.7

    teeth_h = HUB_H * 0.7
    teeth = cq.Workplane("XY").workplane(offset=teeth_z0)
    for i in range(N_TEETH):
        a = 2.0 * math.pi * i / N_TEETH
        tx = (HUB_R + TOOTH_OFFSET) * math.cos(a)
        ty = (HUB_R + TOOTH_OFFSET) * math.sin(a)
        teeth = teeth.moveTo(tx, ty).circle(TOOTH_R).extrude(teeth_h)

    return hub.union(teeth)


def _tray_panel() -> cq.Workplane:
    """Thin rectangular panel with full-depth finger-notch scoops at -Y edge,
    side notches for the pivot bearing posts, and integral pivot pin stubs."""
    panel = (
        cq.Workplane("XY")
        .workplane(offset=-TRAY_PANEL_T / 2.0)
        .box(TRAY_W, TRAY_D, TRAY_PANEL_T, centered=(True, True, False))
    )
    # Through-cut finger notches at the front (-Y) edge, accessible from both faces.
    notch = cq.Workplane("XY").workplane(offset=-TRAY_PANEL_T / 2.0 - 0.001)
    for nx in (-0.026, 0.026):
        notch = (
            notch.moveTo(nx, -TRAY_D / 2.0)
            .circle(0.013)
            .extrude(TRAY_PANEL_T + 0.002)
        )
    panel = panel.cut(notch)

    # Side notches for pivot posts: rectangular cutouts at ±X edges where the
    # bearing posts pass through.
    post_notch = cq.Workplane("XY").workplane(offset=-TRAY_PANEL_T / 2.0 - 0.001)
    notch_w = POST_W + 0.002  # clearance around post
    notch_d = POST_D + 0.002
    for side in (1, -1):
        nx = side * (TRAY_W / 2.0 - notch_w / 2.0 + 0.001)
        post_notch = (
            post_notch.center(nx, 0.0)
            .rect(notch_w, notch_d)
            .extrude(TRAY_PANEL_T + 0.002)
        )
    panel = panel.cut(post_notch)

    # Integral pivot pin stubs: cylinders extending from deep inside the panel
    # outward through the side notches and into the bearing posts. Starting
    # inside the panel ensures connectivity despite the side notches.
    for side in (1, -1):
        pin_inner_x = side * 0.055  # well inside the panel, before the notch
        pin_outer_x = side * (TRAY_W / 2.0 + PIVOT_PIN_L)
        pin_start_x = min(pin_inner_x, pin_outer_x)
        pin_length = abs(pin_outer_x - pin_inner_x)
        pin = (
            cq.Workplane("YZ")
            .workplane(offset=pin_start_x)
            .circle(PIVOT_PIN_R)
            .extrude(pin_length)
        )
        panel = panel.union(pin)

    return panel


def _lid_shell() -> cq.Workplane:
    w, d = CASE_W, CASE_D
    outer = (
        cq.Workplane("XY")
        .workplane(offset=-LID_T)
        .box(w, d, LID_T, centered=(True, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-LID_T)
        .box(w - 2 * LID_WALL, d - 2 * LID_WALL, LID_T - LID_WALL,
             centered=(True, True, False))
    )
    return outer.cut(inner)


def _disc_solid() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(DISC_R)
        .extrude(DISC_T)
        .faces(">Z")
        .workplane()
        .hole(DISC_HOLE_R * 2.0)
    )


# ---- model -------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cd_jewel_case_flip_tray")

    clear = model.material("clear_plastic", rgba=(0.55, 0.60, 0.68, 0.30))
    tray_dark = model.material("tray_dark", rgba=(0.10, 0.10, 0.12, 1.0))
    tray_grey = model.material("tray_grey", rgba=(0.22, 0.22, 0.24, 1.0))
    silver = model.material("disc_silver", rgba=(0.80, 0.82, 0.85, 1.0))

    # ---- base (root): clear frame only ---------------------------------------
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_frame(), "base_frame"),
        material=clear,
        name="base_frame",
    )
    # Pivot bearing posts: right (i=0, +X) and left (i=1, -X) — via loop
    post_sides = [1, -1]
    for i, side in enumerate(post_sides):
        base.visual(
            mesh_from_cadquery(_pivot_post(side), f"pivot_post_{i}"),
            material=clear,
            name=f"pivot_post_{i}",
        )
    base.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, BASE_T)),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
    )

    # ---- flip_tray: dual-sided panel with hubs on both faces -----------------
    flip_tray = model.part("flip_tray")

    # Main panel (with finger notches cut through)
    flip_tray.visual(
        mesh_from_cadquery(_tray_panel(), "tray_panel"),
        material=tray_grey,
        name="tray_panel",
    )

    # Hub faces: top (i=0, +Z) and bottom (i=1, -Z) — repeated via loop
    face_signs = [1, -1]
    for i, sign in enumerate(face_signs):
        flip_tray.visual(
            mesh_from_cadquery(_hub_face(sign), f"hub_face_{i}"),
            material=tray_dark,
            name=f"hub_face_{i}",
        )

    flip_tray.inertial = Inertial.from_geometry(
        Box((TRAY_W + 2 * PIVOT_PIN_L, TRAY_D, TRAY_PANEL_T + 2 * HUB_H)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Flip articulation: revolute about the center X axis at the pivot height.
    # Positive q flips the front edge downward (right-hand rule about +X).
    model.articulation(
        "base_to_flip_tray",
        ArticulationType.REVOLUTE,
        parent=base,
        child=flip_tray,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=math.pi,
        ),
    )

    # ---- lid: clear hinged cover (unchanged from parent) ---------------------
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
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=3.0, lower=0.0, upper=math.radians(70.0),
        ),
    )

    # ---- disc: silver CD on the upper hub face -------------------------------
    disc = model.part("disc")
    disc.visual(
        mesh_from_cadquery(_disc_solid(), "disc_body"),
        material=silver,
        name="disc_body",
    )
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
    # Disc sits on the upper hub face of the flip tray.
    disc_hub_z = TRAY_PANEL_T / 2.0 + HUB_H
    model.articulation(
        "tray_to_disc",
        ArticulationType.CONTINUOUS,
        parent=flip_tray,
        child=disc,
        origin=Origin(xyz=(0.0, 0.0, disc_hub_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


# ---- tests -------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    flip_tray = object_model.get_part("flip_tray")
    lid = object_model.get_part("lid")
    disc = object_model.get_part("disc")

    flip_joint = object_model.get_articulation("base_to_flip_tray")
    lid_joint = object_model.get_articulation("base_to_lid")
    disc_joint = object_model.get_articulation("tray_to_disc")

    # --- transparency ---------------------------------------------------------
    lid_alpha = lid.get_visual("lid_shell").material.rgba[3]
    ctx.check("lid is transparent", lid_alpha < 0.6, details=f"alpha={lid_alpha}")
    base_alpha = base.get_visual("base_frame").material.rgba[3]
    ctx.check("base frame is clear plastic", base_alpha < 0.6, details=f"alpha={base_alpha}")

    # --- dual hubs exist on the flip tray ------------------------------------
    ctx.check(
        "flip tray has top hub face (hub_face_0)",
        flip_tray.get_visual("hub_face_0") is not None,
    )
    ctx.check(
        "flip tray has bottom hub face (hub_face_1)",
        flip_tray.get_visual("hub_face_1") is not None,
    )

    # Top hub sits above the tray panel center, bottom hub below.
    top_hub_aabb = ctx.part_element_world_aabb(flip_tray, elem="hub_face_0")
    bot_hub_aabb = ctx.part_element_world_aabb(flip_tray, elem="hub_face_1")
    panel_aabb = ctx.part_element_world_aabb(flip_tray, elem="tray_panel")
    top_hub_z = 0.5 * (top_hub_aabb[0][2] + top_hub_aabb[1][2])
    bot_hub_z = 0.5 * (bot_hub_aabb[0][2] + bot_hub_aabb[1][2])
    tray_panel_z = 0.5 * (panel_aabb[0][2] + panel_aabb[1][2])
    ctx.check(
        "top hub is above tray panel center",
        top_hub_z > tray_panel_z + 0.001,
        details=f"hub_z={top_hub_z:.4f} panel_z={tray_panel_z:.4f}",
    )
    ctx.check(
        "bottom hub is below tray panel center",
        bot_hub_z < tray_panel_z - 0.001,
        details=f"hub_z={bot_hub_z:.4f} panel_z={tray_panel_z:.4f}",
    )

    # --- flip articulation ----------------------------------------------------
    ctx.check(
        "flip axis runs along X (left-right)",
        abs(flip_joint.axis[0]) > 0.9,
        details=f"axis={flip_joint.axis}",
    )

    # The flip joint upper limit should allow a full 180° flip.
    flip_upper = flip_joint.motion_limits.upper
    ctx.check(
        "flip joint allows full 180° rotation",
        flip_upper is not None and flip_upper >= math.pi - 0.01,
        details=f"upper={flip_upper}",
    )

    # At rest (q=0), the tray panel should be approximately horizontal.
    rest_aabb = ctx.part_world_aabb(flip_tray)
    rest_span_z = rest_aabb[1][2] - rest_aabb[0][2]
    ctx.check(
        "tray is compact at rest (panel + hubs, not excessively tall)",
        rest_span_z < 0.020,
        details=f"z_span={rest_span_z:.4f}",
    )

    # After flipping 180°, the tray bounding box should be nearly identical
    # (center-pivot design keeps the tray in place, just inverted).
    with ctx.pose({flip_joint: math.pi}):
        flip_aabb = ctx.part_world_aabb(flip_tray)
    ctx.check(
        "flipped tray stays within the case footprint",
        abs(flip_aabb[1][2] - rest_aabb[1][2]) < 0.005
        and abs(flip_aabb[0][2] - rest_aabb[0][2]) < 0.005,
        details=f"rest_z=({rest_aabb[0][2]:.4f},{rest_aabb[1][2]:.4f}) "
                f"flip_z=({flip_aabb[0][2]:.4f},{flip_aabb[1][2]:.4f})",
    )

    # --- disc spins on upper hub ---------------------------------------------
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
        details=f"rest=({rest_cx:.4f},{rest_cy:.4f}) q90=({q_cx:.4f},{q_cy:.4f})",
    )
    ctx.check(
        "disc spin axis is vertical (+Z)",
        abs(disc_joint.axis[2]) > 0.9,
        details=f"axis={disc_joint.axis}",
    )

    # Disc parent is the flip tray (not the base).
    disc_parent_name = disc_joint.parent if isinstance(disc_joint.parent, str) else disc_joint.parent.name
    ctx.check(
        "disc is mounted on the flip tray (not the base)",
        disc_parent_name == "flip_tray",
        details=f"parent={disc_parent_name}",
    )

    # --- disc/hub overlap allowance ------------------------------------------
    ctx.allow_overlap(
        disc,
        flip_tray,
        elem_a="disc_body",
        elem_b="hub_face_0",
        reason="The CD center hole drops over the upper hub; the hub intentionally "
               "pokes through the disc bore so the disc is captured and seated.",
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
        details=f"disc_pos={disc_pos}",
    )

    # --- pivot pin / bearing post overlap allowances --------------------------
    # Pivot pins are now integral to tray_panel; allow overlap with posts
    post_names = ["pivot_post_0", "pivot_post_1"]
    for post_name in post_names:
        ctx.allow_overlap(
            flip_tray,
            base,
            elem_a="tray_panel",
            elem_b=post_name,
            reason=f"Tray panel pivot pin stub seats into bearing post {post_name} as "
                   "the flip-axis bearing surface.",
        )
        ctx.expect_contact(
            flip_tray,
            base,
            elem_a="tray_panel",
            elem_b=post_name,
            contact_tol=0.005,
            name=f"tray_panel contacts {post_name}",
        )

    ctx.expect_within(
        flip_tray,
        base,
        axes="xy",
        inner_elem="tray_panel",
        outer_elem="base_frame",
        margin=0.002,
        name="tray panel nests within the base frame footprint",
    )

    # --- lid hinge still works -----------------------------------------------
    closed_top = ctx.part_world_aabb(lid)[1][2]
    with ctx.pose({lid_joint: math.radians(60.0)}):
        open_top = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "lid swings open and lifts upward",
        open_top > closed_top + 0.03,
        details=f"closed_top={closed_top:.4f}, open_top={open_top:.4f}",
    )
    ctx.check(
        "lid hinge axis runs along X",
        abs(lid_joint.axis[0]) > 0.9,
        details=f"axis={lid_joint.axis}",
    )

    # --- non-fixed joints exist -----------------------------------------------
    ctx.check(
        "at least one non-fixed joint exists",
        flip_joint.articulation_type != ArticulationType.FIXED,
        details=f"flip_joint type={flip_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
