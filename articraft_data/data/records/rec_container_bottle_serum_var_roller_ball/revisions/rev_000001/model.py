from __future__ import annotations

# Small amber-glass serum bottle with a roller-ball applicator closure.
# Fork of the dropper-cap serum bottle: same round glass body, hollow shell,
# neck, and white label band — closure replaced by a roller-ball applicator.
#
# Frame: bottle axis along +Z, base at z=0.
#
# Construction:
#   - body (root): amber glass body + shoulder + neck (hollow shell),
#     white label band, white roller-ball housing with socket cup,
#     and three retainer tabs around the socket rim.
#   - roller_ball: stainless-steel ball captured in the housing socket,
#     free to roll on a revolute joint.
#   - overcap: removable gray protective cap that covers the ball and
#     housing, pulled straight up on a prismatic joint.
#
# Articulations:
#   - body_to_ball:     REVOLUTE, horizontal X axis through ball centre.
#   - body_to_overcap:  PRISMATIC, +Z pull-off travel.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- bottle body dimensions (identical to parent) ----
BODY_R = 0.015            # body outer radius (~30 mm diameter)
BODY_Z0 = 0.0             # base
BODY_TOP = 0.060          # top of straight cylinder wall
SHOULDER_TOP = 0.070      # top of rounded shoulder
NECK_R = 0.0080           # neck outer radius
NECK_TOP = 0.082          # top of neck (bottle mouth)
WALL = 0.0018             # glass wall thickness
BORE_R = NECK_R - WALL    # inner bore at neck

# ---- label band (identical to parent) ----
LABEL_Z0 = 0.012
LABEL_Z1 = 0.044
LABEL_R = BODY_R + 0.0006

# ---- roller-ball closure ----
HOUSING_R = 0.011              # housing outer radius
HOUSING_BORE_R = NECK_R        # bore press-fits onto the neck (zero clearance)
HOUSING_Z0 = 0.076             # housing bottom (wraps upper neck only)
PLATFORM_TOP = 0.088           # top of the socket platform
BALL_R = 0.004                 # roller-ball radius (8 mm diameter)
BALL_CZ = 0.085                # ball centre Z (50 % above platform)
CHANNEL_R = 0.003              # serum channel through the platform
SOCKET_R = BALL_R - 0.0001    # socket cup radius (slight capture overlap)
RETAINER_COUNT = 3             # retainer tabs around socket rim
TAB_R_OFFSET = BALL_R + 0.003  # radial placement of retainer tabs

# ---- overcap ----
OVERCAP_R = 0.013              # overcap outer radius
OVERCAP_WALL = 0.002           # wall thickness
OVERCAP_TOP_THICK = 0.002      # closed-top wall
OVERCAP_HEIGHT = 0.026         # total cap height
OVERCAP_Z0 = 0.068             # cap bottom when seated
OVERCAP_TRAVEL = 0.055         # prismatic pull-off distance
RIDGE_Z = 0.077                # retention ridge height inside cap
RIDGE_INNER = HOUSING_R - 0.0001  # protrudes into housing (capture clip)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _body_glass_mesh():
    """Hollow amber-glass shell: straight body + lofted shoulder + neck,
    with a tapered internal bore so the bottle reads as an open container."""
    outer = (
        cq.Workplane("XY")
        .circle(BODY_R)
        .extrude(BODY_TOP)
    )
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP)
        .circle(BODY_R)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(NECK_R + 0.0015)
        .loft(ruled=False)
    )
    neck = (
        cq.Workplane("XY")
        .workplane(offset=SHOULDER_TOP)
        .circle(NECK_R)
        .extrude(NECK_TOP - SHOULDER_TOP)
    )
    solid = outer.union(shoulder).union(neck)

    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .circle(BODY_R - WALL)
        .workplane(offset=(BODY_TOP - WALL))
        .circle(BODY_R - WALL)
        .workplane(offset=(SHOULDER_TOP - BODY_TOP))
        .circle(BORE_R)
        .workplane(offset=(NECK_TOP - SHOULDER_TOP + 0.001))
        .circle(BORE_R)
        .loft(ruled=False)
    )
    solid = solid.cut(cavity)
    return mesh_from_cadquery(solid, "body_glass")


def _housing_mesh():
    """Roller-ball socket housing: cylindrical sleeve that slips over the
    neck, with a platform on top containing a concave spherical cup for
    the ball and a serum channel through the centre."""
    # Outer shell
    outer = (
        cq.Workplane("XY")
        .workplane(offset=HOUSING_Z0)
        .circle(HOUSING_R)
        .extrude(PLATFORM_TOP - HOUSING_Z0)
    )
    # Neck bore (press-fit onto the neck, zero clearance for contact)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=HOUSING_Z0 - 0.001)
        .circle(HOUSING_BORE_R)
        .extrude(NECK_TOP - HOUSING_Z0 + 0.003)
    )
    housing = outer.cut(bore)

    # Serum channel through the platform
    channel = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP - 0.001)
        .circle(CHANNEL_R)
        .extrude(PLATFORM_TOP - NECK_TOP + 0.002)
    )
    housing = housing.cut(channel)

    # Ball socket — spherical cup cut from the top (slightly smaller than
    # ball so the ball is captured in the socket with contact)
    socket = (
        cq.Workplane("XY")
        .workplane(offset=BALL_CZ)
        .sphere(SOCKET_R)
    )
    housing = housing.cut(socket)
    return mesh_from_cadquery(housing, "housing")


def _retainer_tab_mesh():
    """Small retaining tab on the socket rim that captures the ball.
    Shared geometry helper used by the for-loop below."""
    tab = (
        cq.Workplane("XY")
        .box(0.003, 0.002, 0.002, centered=(True, True, False))
    )
    return mesh_from_cadquery(tab, "retainer_tab")


def _overcap_mesh():
    """Removable protective overcap: hollow cylinder with closed top,
    open bottom, and a retention ridge inside that clips onto the housing."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=OVERCAP_Z0)
        .circle(OVERCAP_R)
        .extrude(OVERCAP_HEIGHT)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=OVERCAP_Z0 - 0.001)
        .circle(OVERCAP_R - OVERCAP_WALL)
        .extrude(OVERCAP_HEIGHT - OVERCAP_TOP_THICK + 0.001)
    )
    cap = outer.cut(inner)

    # Retention ridge: thin inward-protruding ring near the cap bottom
    # that snaps over the housing exterior for a clip-fit.
    ridge = (
        cq.Workplane("XY")
        .workplane(offset=RIDGE_Z)
        .circle(OVERCAP_R - OVERCAP_WALL + 0.0001)
        .circle(RIDGE_INNER)
        .extrude(0.002)
    )
    cap = cap.union(ridge)
    return mesh_from_cadquery(cap, "overcap_shell")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serum_bottle_roller")

    # Materials
    amber = model.material("amber_glass", rgba=(0.45, 0.24, 0.06, 0.5))
    white = model.material("white_plastic", rgba=(0.95, 0.95, 0.94, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.97, 0.96, 1.0))
    steel = model.material("steel_ball", rgba=(0.72, 0.73, 0.76, 1.0))
    cap_gray = model.material("cap_gray", rgba=(0.88, 0.88, 0.90, 1.0))

    # ---- body (root) ----
    body = model.part("body")
    body.visual(_body_glass_mesh(), material=amber, name="body_glass")

    # Label band (identical to parent — wraps the body middle)
    label = (
        cq.Workplane("XY")
        .workplane(offset=LABEL_Z0)
        .circle(LABEL_R)
        .circle(BODY_R - 0.001)
        .extrude(LABEL_Z1 - LABEL_Z0)
    )
    body.visual(
        mesh_from_cadquery(label, "label_band"),
        material=label_white,
        name="label_band",
    )

    # Roller-ball housing (fixed visual on body, sits on the neck)
    body.visual(_housing_mesh(), material=white, name="housing")

    # Retainer tabs around the socket rim (repeated sub-parts via for-loop)
    for i in range(RETAINER_COUNT):
        angle = i * (2.0 * math.pi / RETAINER_COUNT)
        tab_x = TAB_R_OFFSET * math.cos(angle)
        tab_y = TAB_R_OFFSET * math.sin(angle)
        body.visual(
            _retainer_tab_mesh(),
            origin=Origin(xyz=(tab_x, tab_y, PLATFORM_TOP)),
            material=white,
            name=f"retainer_tab_{i}",
        )

    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BODY_TOP),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    # ---- roller ball ----
    # The ball part frame is placed at the ball centre by the joint origin.
    # All visuals and inertials are therefore relative to (0, 0, 0) in the
    # child frame, which maps to world (0, 0, BALL_CZ) at rest.
    roller_ball = model.part("roller_ball")
    roller_ball.visual(
        Sphere(BALL_R),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="ball",
    )
    roller_ball.inertial = Inertial.from_geometry(
        Sphere(BALL_R),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- overcap ----
    overcap = model.part("overcap")
    overcap.visual(_overcap_mesh(), material=cap_gray, name="overcap_shell")
    overcap.inertial = Inertial.from_geometry(
        Cylinder(OVERCAP_R, OVERCAP_HEIGHT),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, OVERCAP_Z0 + OVERCAP_HEIGHT / 2.0)),
    )

    # ---- articulations ----

    # Ball rolls in the socket on a horizontal axis.
    # Joint origin at the ball centre — this is the actual contact surface
    # (the socket cup) where the ball seats.
    model.articulation(
        "body_to_ball",
        ArticulationType.REVOLUTE,
        parent=body,
        child=roller_ball,
        origin=Origin(xyz=(0.0, 0.0, BALL_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=1.0,
            velocity=5.0,
        ),
    )

    # Overcap pulls straight up off the bottle.
    # Joint origin at (0,0,0) so the cap visuals (in world coords) stay
    # correct; the prismatic axis moves them along +Z.
    model.articulation(
        "body_to_overcap",
        ArticulationType.PRISMATIC,
        parent=body,
        child=overcap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OVERCAP_TRAVEL,
            effort=5.0,
            velocity=0.1,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    roller_ball = object_model.get_part("roller_ball")
    overcap = object_model.get_part("overcap")
    ball_joint = object_model.get_articulation("body_to_ball")
    cap_joint = object_model.get_articulation("body_to_overcap")

    # --- body is the same amber glass bottle (identical to parent) ---
    glass_aabb = ctx.part_element_world_aabb(body, elem="body_glass")
    gmn, gmx = glass_aabb
    glass_h = gmx[2] - gmn[2]
    glass_dia = max(gmx[0] - gmn[0], gmx[1] - gmn[1])
    ctx.check(
        "glass body is short (squat) — height under 0.09 m",
        glass_h < 0.090,
        details=f"glass height={glass_h:.4f}",
    )
    ctx.check(
        "glass body is narrow (~0.03 m dia)",
        0.020 < glass_dia < 0.040,
        details=f"glass dia={glass_dia:.4f}",
    )
    body_glass = body.get_visual("body_glass")
    rgba = getattr(body_glass.material, "rgba", None)
    ctx.check(
        "body glass is amber-tinted and translucent",
        rgba is not None and rgba[0] > rgba[2] and rgba[3] < 0.95,
        details=f"amber_glass rgba={rgba}",
    )

    # --- label band wraps the body ---
    label_aabb = ctx.part_element_world_aabb(body, elem="label_band")
    lmn, lmx = label_aabb
    ctx.check(
        "label band is on the body wall, around the middle",
        lmn[2] > 0.005 and lmx[2] < SHOULDER_TOP,
        details=f"label z=[{lmn[2]:.4f},{lmx[2]:.4f}]",
    )

    # --- housing at the bottle mouth ---
    housing_aabb = ctx.part_element_world_aabb(body, elem="housing")
    hmn, hmx = housing_aabb
    ctx.check(
        "housing sits at the bottle neck/mouth area",
        hmn[2] >= SHOULDER_TOP - 0.005 and hmx[2] >= NECK_TOP,
        details=f"housing z=[{hmn[2]:.4f},{hmx[2]:.4f}]",
    )

    # --- retainer tabs exist around the socket ---
    for i in range(RETAINER_COUNT):
        ctx.check(
            f"retainer_tab_{i} exists on the housing rim",
            body.get_visual(f"retainer_tab_{i}") is not None,
            details=f"retainer_tab_{i} not found on body",
        )

    # --- roller ball captured in the socket ---
    ball_aabb = ctx.part_element_world_aabb(roller_ball, elem="ball")
    ball_mn, ball_mx = ball_aabb
    ctx.check(
        "roller ball sits at the bottle mouth, above the neck",
        ball_mn[2] >= NECK_TOP - BALL_R - 0.002,
        details=f"ball z=[{ball_mn[2]:.4f},{ball_mx[2]:.4f}]",
    )
    ctx.check(
        "roller ball is ~8 mm diameter",
        abs((ball_mx[2] - ball_mn[2]) - 2 * BALL_R) < 0.002,
        details=f"ball dia={ball_mx[2] - ball_mn[2]:.4f}",
    )
    ctx.check(
        "ball protrudes above the housing platform (accessible for application)",
        ball_mx[2] > PLATFORM_TOP,
        details=f"ball top={ball_mx[2]:.4f}, platform={PLATFORM_TOP:.4f}",
    )
    # Ball XY centred on the bottle axis
    ball_cx = (ball_mn[0] + ball_mx[0]) / 2.0
    ball_cy = (ball_mn[1] + ball_mx[1]) / 2.0
    ctx.check(
        "ball is centred on the bottle axis",
        abs(ball_cx) < 0.002 and abs(ball_cy) < 0.002,
        details=f"ball centre=({ball_cx:.4f},{ball_cy:.4f})",
    )

    # Ball joint is revolute
    ctx.check(
        "ball joint is REVOLUTE",
        ball_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={ball_joint.articulation_type}",
    )

    # Ball rotation at a non-zero pose keeps the ball centred in place
    with ctx.pose({ball_joint: 1.0}):
        ball_aabb_rot = ctx.part_element_world_aabb(roller_ball, elem="ball")
    ctx.check(
        "ball stays centred in the socket after rotation (sphere invariant)",
        abs(ball_aabb_rot[0][2] - ball_mn[2]) < 0.002
        and abs(ball_aabb_rot[1][2] - ball_mx[2]) < 0.002,
        details=f"rotated z=[{ball_aabb_rot[0][2]:.4f},{ball_aabb_rot[1][2]:.4f}], "
                f"rest z=[{ball_mn[2]:.4f},{ball_mx[2]:.4f}]",
    )

    # Ball-housing intentional overlap: the ball is captured in the socket cup
    ctx.allow_overlap(
        roller_ball, body,
        elem_a="ball", elem_b="housing",
        reason="The roller ball is intentionally captured in the housing socket cup "
               "(slight overlap represents the seated ball-in-socket fit).",
    )
    ctx.expect_contact(
        roller_ball, body,
        elem_a="ball", elem_b="housing",
        contact_tol=0.002,
        name="ball contacts the housing socket",
    )

    # --- overcap covers ball when seated ---
    cap_aabb_rest = ctx.part_world_aabb(overcap)
    cap_mn_rest, cap_mx_rest = cap_aabb_rest
    ctx.check(
        "overcap covers the ball when seated (Z containment)",
        cap_mx_rest[2] > ball_mx[2] and cap_mn_rest[2] < ball_mn[2],
        details=f"cap z=[{cap_mn_rest[2]:.4f},{cap_mx_rest[2]:.4f}], "
                f"ball z=[{ball_mn[2]:.4f},{ball_mx[2]:.4f}]",
    )
    ctx.expect_within(
        roller_ball, overcap,
        axes="z",
        margin=0.001,
        inner_elem="ball",
        name="ball is contained within overcap Z extent when seated",
    )

    # Overcap retention ridge overlaps the housing (clip-fit)
    ctx.allow_overlap(
        overcap, body,
        elem_a="overcap_shell", elem_b="housing",
        reason="The overcap retention ridge intentionally clips over the housing "
               "exterior (snap-fit capture ring).",
    )
    ctx.expect_contact(
        overcap, body,
        elem_a="overcap_shell", elem_b="housing",
        contact_tol=0.002,
        name="overcap retention ridge contacts the housing",
    )

    # Overcap is prismatic
    ctx.check(
        "overcap joint is PRISMATIC",
        cap_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={cap_joint.articulation_type}",
    )

    # Overcap pulls straight up and clears the ball
    rest_pos = ctx.part_world_position(overcap)
    with ctx.pose({cap_joint: OVERCAP_TRAVEL}):
        off_pos = ctx.part_world_position(overcap)
        cap_mn_off = ctx.part_world_aabb(overcap)[0]
    ctx.check(
        "overcap translates straight up when removed",
        off_pos[2] > rest_pos[2] + 0.04
        and abs(off_pos[0] - rest_pos[0]) < 1e-4
        and abs(off_pos[1] - rest_pos[1]) < 1e-4,
        details=f"rest={rest_pos}, off={off_pos}",
    )
    ctx.check(
        "overcap clears the ball at full removal",
        cap_mn_off[2] > ball_mx[2],
        details=f"cap bottom(off)={cap_mn_off[2]:.4f}, ball top={ball_mx[2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
