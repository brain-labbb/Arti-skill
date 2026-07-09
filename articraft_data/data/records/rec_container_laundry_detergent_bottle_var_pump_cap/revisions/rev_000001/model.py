from __future__ import annotations

# Tide detergent bottle — pump dispenser fork variant.
# Same body, integrated side-loop handle, threaded neck, and labels as the
# parent jug.  The measuring-cup screw cap is replaced by a screw-collar pump
# dispenser with a dip tube and a press-down actuator head.
#
# Frame: bottle axis along +Z (vertical).  Bottom at z=0, total height ~0.27 m,
# ~0.15 m wide.  Broad faces face ±Y; the integrated side loop handle is on the
# +X side.  The "Tide" bullseye label and "he" badge sit on the −Y face.
#
# Construction:
#   - body (root): glossy orange HDPE jug (contoured shoulder loft), integrated
#     side loop handle (slab with oval cut-out), short threaded neck, pour
#     spout, bullseye label + he badge.  Identical to parent.
#   - pump_base: screw-collar pump housing (hollow collar ring over the neck +
#     pump cylinder on top), grip ridges (repeated), dip tube extending down
#     into the bottle, collar marker tab.
#   - pump_actuator: press-down actuator disc with a horizontal nozzle spout
#     and a stem that slides into the pump cylinder.
#
# Articulations (share the neck +Z axis):
#   - collar_screw: CONTINUOUS spin of pump_base about +Z at the neck seat.
#   - pump_press: PRISMATIC press-down of pump_actuator along −Z (dispense).

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

# ---- key heights / radii (meters) — unchanged from parent ----
BODY_BOTTOM_Z = 0.0
SHOULDER_Z = 0.175
NECK_BOTTOM_Z = 0.198
NECK_TOP_Z = 0.214
NECK_R = 0.0235
SPOUT_TOP_Z = 0.210

# ---- pump dispenser dimensions ----
COLLAR_SEAT_Z = 0.196          # pump base seats here (same as parent cap seat)
COLLAR_HEIGHT = 0.024
COLLAR_OUTER_R = 0.029
COLLAR_INNER_R = 0.0245

PUMP_CYL_HEIGHT = 0.042
PUMP_CYL_R = 0.017
PUMP_TOP_LOCAL = COLLAR_HEIGHT + PUMP_CYL_HEIGHT   # 0.066

STEM_R = 0.007
STEM_LENGTH = 0.048

HEAD_R = 0.022
HEAD_HEIGHT = 0.011

NOZZLE_LENGTH = 0.032
NOZZLE_R = 0.004

DIP_TUBE_R = 0.003
DIP_TUBE_VISIBLE_LENGTH = 0.165  # visible portion below collar
DIP_TUBE_EMBED = 0.025           # portion embedded into housing for connectivity
DIP_TUBE_TOTAL = DIP_TUBE_VISIBLE_LENGTH + DIP_TUBE_EMBED

STROKE = 0.015                   # pump press travel (m)

GRIP_COUNT = 6
GRIP_DX = 0.004                  # radial extent of each grip ridge
GRIP_DY = 0.002                  # tangential thickness


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _body_shell() -> cq.Workplane:
    """Jug body: vertical loft of rounded-rect sections with contoured shoulder,
    threaded neck, and bored pour mouth.  Identical to parent."""
    sections = [
        (0.004, 0.070, 0.044),
        (0.020, 0.075, 0.048),
        (0.090, 0.075, 0.048),
        (0.150, 0.073, 0.046),
        (0.175, 0.064, 0.040),
        (0.190, 0.040, 0.030),
        (0.198, 0.028, 0.028),
    ]
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, hx, hy) in enumerate(sections):
        off = z if i == 0 else z - prev_z
        wp = wp.workplane(offset=off)
        wp = wp.rect(2.0 * hx, 2.0 * hy)
        prev_z = z
    body = wp.loft(ruled=False)

    try:
        body = body.edges("|Z").fillet(0.012)
    except Exception:
        pass

    neck = (
        cq.Workplane("XY")
        .workplane(offset=NECK_BOTTOM_Z)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - NECK_BOTTOM_Z + 0.004)
    )
    body = body.union(neck)

    mouth = (
        cq.Workplane("XY")
        .workplane(offset=0.120)
        .circle(0.013)
        .extrude(0.115)
    )
    body = body.cut(mouth)
    return body


def _handle_solid() -> cq.Workplane:
    """Integrated side loop handle on +X.  Identical to parent."""
    slab = (
        cq.Workplane("XZ")
        .transformed(offset=(0.072, 0.110, 0.0))
        .rect(0.066, 0.150)
        .extrude(0.022, both=True)
    )
    try:
        slab = slab.edges("|Y").fillet(0.014)
    except Exception:
        pass
    hole = (
        cq.Workplane("XZ")
        .transformed(offset=(0.074, 0.110, 0.0))
        .ellipse(0.020, 0.050)
        .extrude(0.06, both=True)
    )
    slab = slab.cut(hole)
    return slab


def _spout_mesh():
    """Raised pour spout (unchanged from parent; now enclosed under pump collar)."""
    o = cq.Workplane("XY").workplane(offset=0.206).circle(0.018).extrude(0.026)
    i = cq.Workplane("XY").workplane(offset=0.204).circle(0.013).extrude(0.032)
    spout = o.cut(i)
    return mesh_from_cadquery(spout, "pour_spout")


def _pump_housing_mesh():
    """Screw collar + pump cylinder as one CadQuery solid.

    The collar is a hollow ring that fits over the threaded neck.  A transition
    disc at the collar top connects the collar annulus to the narrower pump
    cylinder rising above it."""
    # Solid collar disc (will be bored out below)
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_R)
        .extrude(COLLAR_HEIGHT)
    )

    # Transition disc at collar top — bridges collar annulus to pump cylinder
    transition = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_HEIGHT - 0.001)
        .circle(COLLAR_OUTER_R - 0.001)
        .extrude(0.003)
    )

    # Pump cylinder body above collar
    pump_cyl = (
        cq.Workplane("XY")
        .workplane(offset=COLLAR_HEIGHT)
        .circle(PUMP_CYL_R)
        .extrude(PUMP_CYL_HEIGHT)
    )

    housing = collar.union(transition).union(pump_cyl)

    # Bore out the collar center for the neck passage (stop below transition disc)
    bore = (
        cq.Workplane("XY")
        .circle(COLLAR_INNER_R)
        .extrude(COLLAR_HEIGHT - 0.001)
    )
    housing = housing.cut(bore)

    # Small fillet on collar bottom edge
    try:
        housing = housing.edges("<Z").fillet(0.002)
    except Exception:
        pass

    return mesh_from_cadquery(housing, "pump_housing")


def _grip_ridge_geom():
    """Shared geometry for one collar grip ridge."""
    return Box((GRIP_DX, GRIP_DY, COLLAR_HEIGHT * 0.70))


def _actuator_head_mesh():
    """Press-down actuator disc with filleted top edge."""
    head = (
        cq.Workplane("XY")
        .circle(HEAD_R)
        .extrude(HEAD_HEIGHT)
    )
    try:
        head = head.edges(">Z").fillet(0.003)
    except Exception:
        pass
    return mesh_from_cadquery(head, "actuator_head")


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tide_pump_bottle")

    # ---- materials ----
    orange = model.material("tide_orange", rgba=(0.95, 0.46, 0.07, 1.0))
    label_yellow = model.material("label_yellow", rgba=(0.98, 0.83, 0.10, 1.0))
    label_blue = model.material("label_blue", rgba=(0.10, 0.22, 0.62, 1.0))
    he_silver = model.material("he_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    pump_white = model.material("pump_white", rgba=(0.92, 0.92, 0.93, 1.0))
    pump_dark = model.material("pump_dark", rgba=(0.25, 0.25, 0.28, 1.0))
    accent_teal = model.material("accent_teal", rgba=(0.10, 0.65, 0.68, 1.0))

    # ==================== body (root) — identical to parent ====================
    body = model.part("body")

    shell = _body_shell().union(_handle_solid())
    body.visual(mesh_from_cadquery(shell, "jug_shell"), material=orange, name="jug_shell")
    body.visual(_spout_mesh(), material=orange, name="pour_spout")

    body.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(-0.010, -0.0475, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=label_yellow,
        name="bullseye_label",
    )
    body.visual(
        Box((0.060, 0.004, 0.018)),
        origin=Origin(xyz=(-0.010, -0.0475, 0.060)),
        material=label_blue,
        name="label_band",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(-0.012, -0.047, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=he_silver,
        name="he_badge",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.15, 0.095, 0.21)), mass=1.1, origin=Origin(xyz=(0.0, 0.0, 0.095))
    )

    # ==================== pump_base (screw collar + pump housing) ====================
    pump_base = model.part("pump_base")

    pump_base.visual(_pump_housing_mesh(), material=pump_white, name="pump_housing")

    # Repeated grip ridges on the collar (for-i-in-range pattern)
    for i in range(GRIP_COUNT):
        angle = i * (2.0 * math.pi / GRIP_COUNT)
        r = COLLAR_OUTER_R + 0.001   # center slightly proud of collar surface
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        pump_base.visual(
            _grip_ridge_geom(),
            origin=Origin(
                xyz=(x, y, COLLAR_HEIGHT * 0.35),
                rpy=(0.0, 0.0, angle),
            ),
            material=pump_dark,
            name=f"grip_ridge_{i}",
        )

    # Dip tube — extends from inside the pump housing down into the bottle
    # through the neck bore.  Top portion embeds into the housing transition disc
    # for connectivity; visible portion hangs below the collar.
    dip_center_z = -(DIP_TUBE_VISIBLE_LENGTH - DIP_TUBE_EMBED) / 2.0
    pump_base.visual(
        Cylinder(radius=DIP_TUBE_R, length=DIP_TUBE_TOTAL),
        origin=Origin(xyz=(0.0, 0.0, dip_center_z)),
        material=pump_white,
        name="dip_tube",
    )

    # Off-axis collar marker tab for rotation visibility
    pump_base.visual(
        Box((0.005, 0.005, 0.008)),
        origin=Origin(xyz=(COLLAR_OUTER_R + 0.002, 0.0, COLLAR_HEIGHT * 0.50)),
        material=accent_teal,
        name="collar_marker",
    )

    pump_base.inertial = Inertial.from_geometry(
        Cylinder(radius=COLLAR_OUTER_R, length=PUMP_TOP_LOCAL),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, PUMP_TOP_LOCAL / 2.0)),
    )

    # ==================== pump_actuator (press-down head) ====================
    pump_actuator = model.part("pump_actuator")

    # Actuator head disc (top surface you press)
    pump_actuator.visual(_actuator_head_mesh(), material=accent_teal, name="actuator_head")

    # Nozzle spout extending in −Y from the head
    nozzle_cy = -(HEAD_R * 0.40 + NOZZLE_LENGTH / 2.0)
    nozzle_cz = HEAD_HEIGHT * 0.55
    pump_actuator.visual(
        Cylinder(radius=NOZZLE_R, length=NOZZLE_LENGTH),
        origin=Origin(
            xyz=(0.0, nozzle_cy, nozzle_cz),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=pump_white,
        name="nozzle_spout",
    )

    # Stem — extends down from the actuator into the pump cylinder
    pump_actuator.visual(
        Cylinder(radius=STEM_R, length=STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -STEM_LENGTH / 2.0)),
        material=pump_dark,
        name="actuator_stem",
    )

    pump_actuator.inertial = Inertial.from_geometry(
        Cylinder(radius=HEAD_R, length=HEAD_HEIGHT + STEM_LENGTH),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, -(STEM_LENGTH - HEAD_HEIGHT) / 2.0)),
    )

    # ---- collar_screw: CONTINUOUS rotation of pump_base about +Z ----
    model.articulation(
        "collar_screw",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=pump_base,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0),
    )

    # ---- pump_press: PRISMATIC press-down of actuator along −Z ----
    # Origin at the top face of the pump cylinder (actual contact surface where
    # the actuator head rests).  Positive q presses the head DOWN.
    model.articulation(
        "pump_press",
        ArticulationType.PRISMATIC,
        parent=pump_base,
        child=pump_actuator,
        origin=Origin(xyz=(0.0, 0.0, PUMP_TOP_LOCAL)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=STROKE, effort=5.0, velocity=0.5),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pump_base = object_model.get_part("pump_base")
    pump_actuator = object_model.get_part("pump_actuator")
    collar_screw = object_model.get_articulation("collar_screw")
    pump_press = object_model.get_articulation("pump_press")

    # ---- intentional-overlap allowances ----

    # Screw collar fits over the threaded neck.
    ctx.allow_overlap(
        pump_base, body,
        elem_a="pump_housing", elem_b="jug_shell",
        reason="The screw collar fits over the threaded neck (skirt surrounds neck).",
    )
    # Pump collar encloses the pour spout area.
    ctx.allow_overlap(
        pump_base, body,
        elem_a="pump_housing", elem_b="pour_spout",
        reason="The pump housing encloses the pour spout area under the collar.",
    )
    # Dip tube passes through the neck bore into the bottle interior.
    ctx.allow_overlap(
        pump_base, body,
        elem_a="dip_tube", elem_b="jug_shell",
        reason="The dip tube passes through the neck bore into the bottle.",
    )
    # Actuator stem slides inside the pump cylinder.
    ctx.allow_overlap(
        pump_actuator, pump_base,
        elem_a="actuator_stem", elem_b="pump_housing",
        reason="The actuator stem slides inside the pump cylinder bore.",
    )

    # ---- proof checks paired with allowances ----

    # Stem stays centered in the pump cylinder on non-motion axes.
    ctx.expect_within(
        pump_actuator, pump_base,
        axes="xy",
        inner_elem="actuator_stem", outer_elem="pump_housing",
        margin=0.002,
        name="actuator stem stays centered in pump cylinder (rest)",
    )

    # Dip tube is within the neck bore in XY.
    ctx.expect_within(
        pump_base, body,
        axes="xy",
        inner_elem="dip_tube", outer_elem="jug_shell",
        margin=0.005,
        name="dip tube passes through the neck bore",
    )

    # ---- pump actuator sits on top of the bottle ----
    act_aabb = ctx.part_world_aabb(pump_actuator)
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "pump actuator is the highest element on the bottle",
        act_aabb is not None and body_aabb is not None
        and act_aabb[1][2] >= body_aabb[1][2] - 1e-6,
        details=f"actuator z-max={act_aabb[1][2]}, body z-max={body_aabb[1][2]}",
    )

    # ---- pump press moves actuator downward ----
    rest_z = ctx.part_world_position(pump_actuator)[2]
    with ctx.pose({pump_press: STROKE}):
        pressed_z = ctx.part_world_position(pump_actuator)[2]
    ctx.check(
        "pump press moves actuator downward by the full stroke",
        pressed_z < rest_z - 0.010,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )

    # Stem remains centered in pump cylinder when pressed.
    with ctx.pose({pump_press: STROKE}):
        ctx.expect_within(
            pump_actuator, pump_base,
            axes="xy",
            inner_elem="actuator_stem", outer_elem="pump_housing",
            margin=0.002,
            name="actuator stem stays centered in pump cylinder (pressed)",
        )

    # ---- collar rotation swings the nozzle around +Z ----
    nozzle_rest = ctx.part_element_world_aabb(pump_actuator, elem="nozzle_spout")
    with ctx.pose({collar_screw: math.pi / 2.0}):
        nozzle_rot = ctx.part_element_world_aabb(pump_actuator, elem="nozzle_spout")
    rest_cx = (nozzle_rest[0][0] + nozzle_rest[1][0]) / 2.0
    rest_cy = (nozzle_rest[0][1] + nozzle_rest[1][1]) / 2.0
    rot_cx = (nozzle_rot[0][0] + nozzle_rot[1][0]) / 2.0
    rot_cy = (nozzle_rot[0][1] + nozzle_rot[1][1]) / 2.0
    swing = math.hypot(rot_cx - rest_cx, rot_cy - rest_cy)
    ctx.check(
        "collar rotation swings the nozzle around +Z",
        swing > 0.015,
        details=f"rest=({rest_cx:.4f},{rest_cy:.4f}), "
                f"rot90=({rot_cx:.4f},{rot_cy:.4f}), swing={swing:.4f}",
    )

    # ---- nozzle spout is a visible horizontal feature ----
    nozzle_y_extent = nozzle_rest[1][1] - nozzle_rest[0][1]
    ctx.check(
        "pump actuator has a visible nozzle spout",
        nozzle_y_extent > 0.020,
        details=f"nozzle Y-extent={nozzle_y_extent:.4f}",
    )

    # ---- grip ridges present on collar ----
    grip_names = [v.name for v in pump_base.visuals if v.name.startswith("grip_ridge_")]
    ctx.check(
        f"collar has {GRIP_COUNT} grip ridges",
        len(grip_names) == GRIP_COUNT,
        details=f"found {len(grip_names)}: {grip_names}",
    )

    # ---- dip tube extends below the neck into the bottle ----
    dip_aabb = ctx.part_element_world_aabb(pump_base, elem="dip_tube")
    ctx.check(
        "dip tube extends deep into the bottle interior",
        dip_aabb is not None and dip_aabb[0][2] < 0.05,
        details=f"dip_tube z-min={None if dip_aabb is None else dip_aabb[0][2]}",
    )

    # ---- body features still present (fork identity) ----
    handle_aabb = ctx.part_element_world_aabb(body, elem="jug_shell")
    ctx.check(
        "jug extends out to the side loop handle on +X",
        handle_aabb is not None and handle_aabb[1][0] > 0.085,
        details=f"jug_shell x-max={None if handle_aabb is None else handle_aabb[1][0]}",
    )
    ctx.expect_contact(
        body, body, elem_a="bullseye_label", elem_b="jug_shell",
        name="bullseye label is on the jug body",
    )

    return ctx.report()


object_model = build_object_model()
