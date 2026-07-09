from __future__ import annotations

# Aerosol SPRAY PAINT CAN with a lift-off dust cap and a WIDE FAN-CAP actuator.
# Fork of the simple press-button parent: replaces the small nozzle button with
# a broad oval fan cap that BOTH presses down (PRISMATIC) AND twists about Z
# (REVOLUTE) to switch the spray pattern between horizontal and vertical fan.
#
# Frame: can axis along +Z. Base rim at z=0, body rises in +Z, crimped dome
# and fan-cap actuator at the top. Cylinder ~0.20 m tall, ~0.065 m diameter.
#
# Articulations:
#   - dust cap: PRISMATIC lift UP (+Z) ~0.09 m (identical to parent).
#   - fan_cap_stem: PRISMATIC press DOWN (-Z) ~0.004 m (replaces nozzle_press).
#   - fan_cap: REVOLUTE twist about Z (0 to pi/2) on top of the stem.
#
# The fan cap is a CHILD of the stem (which is a CHILD of the body), so
# pressing the stem carries the cap down, and twisting the cap is independent.

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
CAN_R = 0.0325            # body radius (~0.065 m diameter)
BODY_BOTTOM = 0.0         # base plane
BODY_TOP = 0.150          # where the straight wall ends and the dome begins
DOME_TOP = 0.176          # top of the crimped dome (mounting cup rim)
VALVE_TOP = 0.182         # top of the central valve stem the actuator seats on
STEM_SEAT_Z = 0.176       # z where the fan-cap stem base sits (same as parent nozzle)
STEM_HEIGHT = 0.010       # stem local height (sleeve + collar)
HEAD_HEIGHT = 0.014       # fan-cap head local height
CAP_BOTTOM = 0.150        # dust cap lower rim (sleeves down over the body shoulder)
CAP_TOP = 0.210           # dust cap top


def _can_body_solid() -> cq.Workplane:
    """Tall cylindrical can body with crimped dome and valve stem (identical to parent)."""
    # Bottom rim (slightly larger ring at the base for the rolled seam).
    body = (
        cq.Workplane("XY")
        .circle(CAN_R + 0.0012)
        .extrude(0.006)
    )
    # Main straight wall.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .circle(CAN_R)
        .extrude(BODY_TOP - 0.005)
    )
    # Crimped top dome: loft from the wall up and inward to the mounting cup rim.
    dome = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(CAN_R)
        .workplane(offset=0.006)
        .circle(CAN_R - 0.001)
        .workplane(offset=0.004)
        .circle(CAN_R - 0.006)
        .workplane(offset=0.017)
        .circle(0.014)
        .loft(ruled=False)
    )
    body = body.union(dome)
    # Central valve stem boss the actuator seats over.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=DOME_TOP - 0.001)
        .circle(0.006)
        .extrude(VALVE_TOP - DOME_TOP)
    )
    return body


def _fan_cap_stem_solid() -> cq.Workplane:
    """Short cylindrical sleeve with a top collar — the prismatic press member."""
    # Main sleeve: fits over the valve stem.
    stem = (
        cq.Workplane("XY")
        .circle(0.008)
        .extrude(0.007)
    )
    # Bore to accept the valve stem (valve radius 0.006).
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(0.0062)
        .extrude(0.006)
    )
    stem = stem.cut(bore)
    # Top collar/flange: wider platform the fan-cap head seats on.
    collar = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .circle(0.012)
        .extrude(0.004)
    )
    stem = stem.union(collar)
    return stem


def _fan_cap_head_solid() -> cq.Workplane:
    """Broad oval fan cap disc with an elongated spray slot on the top face."""
    # Oval disc body: wider in X (spray-slot direction), narrower in Y.
    cap = (
        cq.Workplane("XY")
        .ellipse(0.020, 0.013)
        .extrude(HEAD_HEIGHT)
    )
    # Round the top edges for a finished look.
    cap = cap.edges(">Z").fillet(0.003)
    # Internal spray channel: bore from the bottom face upward.
    channel = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(0.011)
    )
    cap = cap.cut(channel)
    # Spray slot on the top face: elongated along X (horizontal fan at q=0).
    # Cuts through the top surface into the internal channel.
    slot = (
        cq.Workplane("XY")
        .workplane(offset=HEAD_HEIGHT + 0.001)
        .rect(0.028, 0.003)
        .extrude(-0.006)
    )
    cap = cap.cut(slot)
    return cap


def _dust_cap_solid() -> cq.Workplane:
    """Dark grey dust cap shell (identical to parent)."""
    h = CAP_TOP - CAP_BOTTOM
    outer = (
        cq.Workplane("XY")
        .circle(CAN_R + 0.0035)
        .extrude(h - 0.010)
    )
    # Rounded closed top.
    top = (
        cq.Workplane("XY")
        .workplane(offset=h - 0.011)
        .circle(CAN_R + 0.0035)
        .workplane(offset=0.007)
        .circle(CAN_R - 0.004)
        .workplane(offset=0.004)
        .circle(0.010)
        .loft(ruled=False)
    )
    cap = outer.union(top)
    # Hollow the inside so it slips over the can top.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(CAN_R - 0.0006)
        .extrude(h - 0.014)
    )
    cap = cap.cut(bore)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spray_paint_can_fan_cap")

    label = model.material("label_splatter", rgba=(0.90, 0.90, 0.92, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_cap = model.material("dark_grey_cap", rgba=(0.20, 0.21, 0.20, 1.0))
    dark_stem = model.material("dark_stem", rgba=(0.16, 0.16, 0.17, 1.0))
    fan_cap_color = model.material("fan_cap_red", rgba=(0.72, 0.18, 0.12, 1.0))

    # ---- can body (root) ---- identical to parent
    body = model.part("can_body")
    body.visual(
        mesh_from_cadquery(_can_body_solid(), "can_body"),
        material=metal,
        name="can_body",
    )
    # Splatter-graphic label band wrapped on the straight wall.
    label_band = CylinderGeometry(CAN_R + 0.0004, 0.105, radial_segments=64)
    label_band.translate(0.0, 0.0, 0.012 + 0.105 / 2.0)
    body.visual(mesh_from_geometry(label_band, "label_band"), material=label, name="label_band")
    body.inertial = Inertial.from_geometry(
        Cylinder(CAN_R, 0.176),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    # ---- fan-cap stem: child of body, presses straight DOWN ----
    stem = model.part("fan_cap_stem")
    stem.visual(
        mesh_from_cadquery(_fan_cap_stem_solid(), "fan_cap_stem"),
        material=dark_stem,
        name="fan_cap_stem",
    )
    stem.inertial = Inertial.from_geometry(
        Cylinder(0.012, STEM_HEIGHT),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, STEM_HEIGHT / 2.0)),
    )
    model.articulation(
        "fan_cap_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, STEM_SEAT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.05, lower=0.0, upper=0.004),
    )

    # ---- fan-cap head: child of stem, twists about Z ----
    head = model.part("fan_cap")
    head.visual(
        mesh_from_cadquery(_fan_cap_head_solid(), "fan_cap"),
        material=fan_cap_color,
        name="fan_cap",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.040, 0.026, HEAD_HEIGHT)),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, HEAD_HEIGHT / 2.0)),
    )
    model.articulation(
        "fan_cap_twist",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=head,
        # Joint origin at the top of the stem collar (contact surface).
        origin=Origin(xyz=(0.0, 0.0, STEM_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.5708),
    )

    # ---- dust cap: child of body, lifts straight UP (identical to parent) ----
    cap = model.part("dust_cap")
    cap.visual(
        mesh_from_cadquery(_dust_cap_solid(), "dust_cap"),
        material=dark_cap,
        name="dust_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAN_R + 0.0035, CAP_TOP - CAP_BOTTOM),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP - CAP_BOTTOM) / 2.0)),
    )
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.2, lower=0.0, upper=0.090),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    stem = object_model.get_part("fan_cap_stem")
    head = object_model.get_part("fan_cap")
    cap = object_model.get_part("dust_cap")
    cap_lift = object_model.get_articulation("cap_lift")
    fan_press = object_model.get_articulation("fan_cap_press")
    fan_twist = object_model.get_articulation("fan_cap_twist")

    # --- can body is a tall cylinder (identical check to parent) ---
    bmn, bmx = ctx.part_world_aabb(body)
    height = bmx[2] - bmn[2]
    dia_x = bmx[0] - bmn[0]
    dia_y = bmx[1] - bmn[1]
    ctx.check(
        "can body is a tall cylinder (~0.18 m, ~0.065 m dia)",
        height > 0.16 and 0.05 < dia_x < 0.08 and 0.05 < dia_y < 0.08 and height > dia_x * 2.0,
        details=f"height={height:.3f}, dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )

    # --- fan-cap stem is mounted on the can top, near central axis ---
    stem_pos = ctx.part_world_position(stem)
    ctx.check(
        "fan-cap stem is mounted on the can top",
        stem_pos is not None and stem_pos[2] > 0.16 and abs(stem_pos[0]) < 0.01 and abs(stem_pos[1]) < 0.01,
        details=f"stem origin={stem_pos}",
    )

    # --- stem seats over the valve stem (intentional local overlap) ---
    ctx.allow_overlap(
        stem,
        body,
        elem_a="fan_cap_stem",
        elem_b="can_body",
        reason="The fan-cap stem sleeve is intentionally seated over the central valve stem.",
    )
    ctx.expect_contact(stem, body, name="fan-cap stem seated on valve stem")

    # --- fan-cap head sits on top of the stem ---
    head_pos = ctx.part_world_position(head)
    ctx.check(
        "fan-cap head is mounted above the stem",
        head_pos is not None and head_pos[2] > stem_pos[2] + 0.005,
        details=f"head_z={head_pos[2]:.4f}, stem_z={stem_pos[2]:.4f}",
    )

    # --- head seats on the stem collar (contact at the interface) ---
    ctx.expect_contact(head, stem, name="fan-cap head seated on stem collar")

    # --- fan-cap head is a broad oval shape (wider than the simple button) ---
    hmn, hmx = ctx.part_world_aabb(head)
    head_dx = hmx[0] - hmn[0]
    head_dy = hmx[1] - hmn[1]
    ctx.check(
        "fan-cap head is broad oval (wider than 0.030 m in slot direction)",
        head_dx > 0.030 and head_dy > 0.018,
        details=f"head_dx={head_dx:.4f}, head_dy={head_dy:.4f}",
    )

    # --- fan-cap presses straight DOWN (prismatic) ---
    stem_rest_z = ctx.part_world_position(stem)[2]
    head_rest_z = ctx.part_world_position(head)[2]
    with ctx.pose({fan_press: 0.004}):
        stem_pressed_z = ctx.part_world_position(stem)[2]
        head_pressed_z = ctx.part_world_position(head)[2]
    ctx.check(
        "fan-cap stem presses straight down (~0.004 m)",
        stem_rest_z - stem_pressed_z > 0.003,
        details=f"stem rest={stem_rest_z:.4f}, pressed={stem_pressed_z:.4f}",
    )
    ctx.check(
        "fan-cap head moves down with the stem when pressed",
        head_rest_z - head_pressed_z > 0.003,
        details=f"head rest={head_rest_z:.4f}, pressed={head_pressed_z:.4f}",
    )

    # --- fan-cap twists about Z (revolute): slot orientation changes ---
    # At rest (q=0): the oval is wider in X (slot along X = horizontal fan).
    mn0, mx0 = ctx.part_world_aabb(head)
    dx_at_rest = mx0[0] - mn0[0]
    dy_at_rest = mx0[1] - mn0[1]
    ctx.check(
        "at q=0 the fan cap is wider in X (horizontal slot / horizontal fan)",
        dx_at_rest > dy_at_rest,
        details=f"dx={dx_at_rest:.4f}, dy={dy_at_rest:.4f}",
    )

    # At q=pi/2 (90 deg): the oval rotates, now wider in Y.
    with ctx.pose({fan_twist: 1.5708}):
        mn90, mx90 = ctx.part_world_aabb(head)
        dx_at_90 = mx90[0] - mn90[0]
        dy_at_90 = mx90[1] - mn90[1]
    ctx.check(
        "at q=pi/2 the fan cap is wider in Y (slot rotated to vertical fan)",
        dy_at_90 > dx_at_90,
        details=f"dx={dx_at_90:.4f}, dy={dy_at_90:.4f}",
    )

    # Twist does not change Z position (pure rotation about Z).
    with ctx.pose({fan_twist: 1.5708}):
        head_twisted_z = ctx.part_world_position(head)[2]
    ctx.check(
        "fan-cap twist does not change height",
        abs(head_twisted_z - head_rest_z) < 0.001,
        details=f"rest_z={head_rest_z:.4f}, twisted_z={head_twisted_z:.4f}",
    )

    # --- dust cap sleeves over the can top (identical to parent) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="dust_cap",
        elem_b="can_body",
        reason="Dust cap intentionally sleeves down over the can's top shoulder when seated.",
    )
    # Dust cap also encloses the fan-cap assembly when seated.
    ctx.allow_overlap(
        cap,
        head,
        elem_a="dust_cap",
        elem_b="fan_cap",
        reason="Dust cap dome encloses the fan-cap head when both are at rest.",
    )
    ctx.allow_overlap(
        cap,
        stem,
        elem_a="dust_cap",
        elem_b="fan_cap_stem",
        reason="Dust cap bore surrounds the fan-cap stem when seated.",
    )

    cap_rest_mn, cap_rest_mx = ctx.part_world_aabb(cap)
    ctx.check(
        "dust cap is seated over the can top at rest",
        cap_rest_mn[2] < 0.16 and cap_rest_mx[2] > 0.20,
        details=f"cap z range at rest=({cap_rest_mn[2]:.3f}, {cap_rest_mx[2]:.3f})",
    )

    # --- dust cap lifts straight UP a LARGE amount, clear of the fan cap ---
    hmn_head, hmx_head = ctx.part_world_aabb(head)
    head_top = hmx_head[2]
    cap_rest_pos_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_lift: 0.090}):
        cap_up_mn, cap_up_mx = ctx.part_world_aabb(cap)
        cap_up_pos = ctx.part_world_position(cap)
    ctx.check(
        "dust cap lifts straight up by a large amount (~0.09 m)",
        cap_up_pos[2] - cap_rest_pos_z > 0.085,
        details=f"cap rose from z={cap_rest_pos_z:.3f} to z={cap_up_pos[2]:.3f}",
    )
    ctx.check(
        "lifted cap bottom clears the fan-cap head top (fan cap revealed)",
        cap_up_mn[2] > head_top + 0.002,
        details=f"lifted cap bottom={cap_up_mn[2]:.3f}, head top={head_top:.3f}",
    )
    ctx.check(
        "cap lift is purely vertical (no lateral drift)",
        abs(cap_up_pos[0]) < 0.005 and abs(cap_up_pos[1]) < 0.005,
        details=f"lifted cap xy=({cap_up_pos[0]:.4f}, {cap_up_pos[1]:.4f})",
    )

    return ctx.report()
