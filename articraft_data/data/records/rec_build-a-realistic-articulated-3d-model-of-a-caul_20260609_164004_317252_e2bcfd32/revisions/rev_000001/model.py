from __future__ import annotations

# Hot glue gun (branded "CRV TOOLS") modeled faithfully from the reference image.
# The reference shows a pistol-grip electric glue gun: a purple plastic body with a
# horizontal barrel up top, a chrome conical metal nozzle at the front, an orange
# squeeze trigger that advances the glue stick, a red/black rocker power switch at
# the lower rear of the handle, and a white glue stick fed into the rear of the
# barrel.
#
# Coordinate frame:
#   +X = forward, toward the nozzle tip
#   +Z = up
#   +Y = gun width (left / right)
# Origin sits at the rear of the barrel, on the barrel centerline.
#
# Articulations (physically reasoned from the image):
#   - trigger:  REVOLUTE. The orange trigger is a finger lever pivoting about a pin
#               just under the barrel/front of the handle. Squeezing rotates its
#               lower (free) end rearward toward the grip. Primary mechanism.
#   - power_switch: REVOLUTE. The red rocker switch pivots about its mid-line.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BARREL_LEN = 0.110          # horizontal body section length (X)
BARREL_W = 0.044            # body width (Y)
BARREL_H = 0.052            # body height at the barrel (Z)
BARREL_Z = 0.000            # barrel centerline z

GLUE_BORE_R = 0.0115        # internal glue-stick bore radius
NOZZLE_BASE_R = 0.0150      # nozzle collar radius at the body
NOZZLE_TIP_R = 0.0040       # nozzle tip radius
NOZZLE_LEN = 0.052          # cone length forward of the body
NOZZLE_TIP_BORE_R = 0.0016  # nozzle outlet hole radius

HANDLE_W = 0.030            # handle grip width (Y)
HANDLE_LEN = 0.044          # handle cross-section depth (X)
HANDLE_DROP = 0.115         # how far the handle drops below the barrel (Z)
HANDLE_REAR_X = -0.030      # x of the handle (sits at rear of barrel, raked back)

TRIGGER_W = 0.016
TRIGGER_THK = 0.011
TRIGGER_LEN = 0.040         # finger-lever length below the pivot
TRIGGER_PIVOT_X = 0.012     # pivot x, just under the front of the barrel/handle
TRIGGER_PIVOT_Z = -0.030    # pivot z, below the barrel

SWITCH_W = 0.018            # rocker width (Y)
SWITCH_H = 0.024            # rocker height along the handle (its long dimension)
SWITCH_THK = 0.006          # rocker protrusion thickness

PURPLE = (0.32, 0.16, 0.55, 1.0)
ORANGE = (0.95, 0.60, 0.10, 1.0)
CHROME = (0.78, 0.80, 0.83, 1.0)
WHITE = (0.92, 0.92, 0.90, 1.0)
SWITCH_BLACK = (0.10, 0.10, 0.11, 1.0)
SWITCH_RED = (0.85, 0.12, 0.10, 1.0)


def _body_shape() -> cq.Workplane:
    """Pistol-grip body: horizontal barrel blended into a raked-back handle.

    Built in meters. The glue-stick bore is cut straight through along X, and the
    front face is bored to receive the nozzle collar.
    """
    barrel_cx = BARREL_LEN / 2.0 - 0.030  # rear of barrel near x=-0.030

    # Barrel: rounded box that reads as the upper body / glue chamber.
    barrel = (
        cq.Workplane("XY")
        .box(BARREL_LEN, BARREL_W, BARREL_H)
        .edges("|X")
        .fillet(0.014)
        .translate((barrel_cx, 0.0, BARREL_Z))
    )

    # Handle: a downward, slightly rear-raked grip. Built as a tall rounded box and
    # rotated about Y so the bottom trails toward the rear (matches the image rake).
    handle = (
        cq.Workplane("XY")
        .box(HANDLE_LEN, HANDLE_W, HANDLE_DROP)
        .edges("|Y")
        .fillet(0.010)
        .rotate((0, 0, 0), (0, 1, 0), -16.0)
        .translate((HANDLE_REAR_X + 0.006, 0.0, -HANDLE_DROP / 2.0 - 0.006))
    )

    body = barrel.union(handle)

    # Blend barrel-to-handle junction.
    body = body.edges("|Y").fillet(0.006)

    # Glue-stick bore straight through the barrel along X.
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=barrel_cx - BARREL_LEN / 2.0 - 0.005)
        .circle(GLUE_BORE_R)
        .extrude(BARREL_LEN + 0.02)
        .translate((0, 0, BARREL_Z))
    )
    body = body.cut(bore)

    return body


def _nozzle_shape() -> cq.Workplane:
    """Chrome conical metal nozzle, hollow with a small outlet at the tip.

    Authored in its own local frame with the collar base at local x=0 and the cone
    extending toward +X. Attached at the front face of the body.
    """
    # Solid cone via loft between a base circle and a tip circle. The YZ workplane
    # normal is +X, so successive `workplane(offset=...)` steps move along +X.
    cone = (
        cq.Workplane("YZ")
        .circle(NOZZLE_BASE_R)
        .workplane(offset=NOZZLE_LEN)
        .circle(NOZZLE_TIP_R)
        .loft(combine=True)
    )

    # A short cylindrical collar at the base that plugs into the body bore.
    collar = (
        cq.Workplane("YZ")
        .workplane(offset=-0.010)
        .circle(GLUE_BORE_R - 0.0005)
        .extrude(0.012)
    )
    nozzle = cone.union(collar)

    # Hollow it: subtract the inner glue channel that tapers to the tip outlet.
    channel = (
        cq.Workplane("YZ")
        .workplane(offset=-0.010)
        .circle(GLUE_BORE_R - 0.0025)
        .workplane(offset=NOZZLE_LEN + 0.012)
        .circle(NOZZLE_TIP_BORE_R)
        .loft(combine=True)
    )
    nozzle = nozzle.cut(channel)

    return nozzle


def _trigger_shape() -> cq.Workplane:
    """Orange squeeze trigger.

    Authored in the trigger-local frame with the pivot at local origin. The lever
    blade hangs down toward -Z (closed/rest position), curving slightly forward at
    the bottom to match the finger contour in the image.
    """
    # Pivot boss the lever wraps around the pin.
    boss = (
        cq.Workplane("XY")
        .cylinder(TRIGGER_W + 0.002, 0.007)
        .rotate((0, 0, 0), (1, 0, 0), 90.0)
    )

    # Curved blade: a swept rounded rectangle following a gentle forward curve.
    pts = [
        (0.0, 0.0),
        (0.004, -TRIGGER_LEN * 0.45),
        (0.010, -TRIGGER_LEN * 0.85),
        (0.013, -TRIGGER_LEN),
    ]
    blade = (
        cq.Workplane("XZ")
        .moveTo(*pts[0])
        .lineTo(*pts[1])
        .lineTo(*pts[2])
        .lineTo(*pts[3])
        .lineTo(pts[3][0] - TRIGGER_THK, pts[3][1])
        .lineTo(pts[2][0] - TRIGGER_THK, pts[2][1])
        .lineTo(pts[1][0] - TRIGGER_THK, pts[1][1])
        .lineTo(-TRIGGER_THK, 0.0)
        .close()
        .extrude(TRIGGER_W, both=True)
    )
    # Round the bottom finger edge.
    blade = blade.edges("|Y").fillet(0.0025)

    trigger = boss.union(blade)
    return trigger


def _switch_shape() -> cq.Workplane:
    """Red rocker for the power switch, authored about its pivot at local origin.

    The rocker face lies in the handle's rear surface. Its long axis is +Z (up the
    handle) and it protrudes toward -X (out the rear of the handle).
    """
    rocker = (
        cq.Workplane("XY")
        .box(SWITCH_THK, SWITCH_W, SWITCH_H)
        .edges("|X")
        .fillet(0.0015)
    )
    return rocker


def _switch_bezel_shape() -> cq.Workplane:
    """Black bezel/surround that frames the rocker on the handle (part of body)."""
    bezel = (
        cq.Workplane("XY")
        .box(0.004, SWITCH_W + 0.006, SWITCH_H + 0.006)
        .edges("|X")
        .fillet(0.0015)
    )
    return bezel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hot_glue_gun")

    purple = model.material("purple_plastic", rgba=PURPLE)
    orange = model.material("orange_plastic", rgba=ORANGE)
    chrome = model.material("chrome_nozzle", rgba=CHROME)
    white = model.material("glue_stick", rgba=WHITE)
    blk = model.material("switch_bezel", rgba=SWITCH_BLACK)
    red = model.material("switch_red", rgba=SWITCH_RED)

    # ----------------------------------------------------------------- body
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=purple,
        name="body_shell",
    )

    # Switch bezel sits on the rear face of the handle, framing the rocker.
    # Handle rear face (raked back) is roughly at x ~ -0.052, lower-mid handle.
    switch_origin_x = -0.050
    switch_origin_z = -0.072
    body.visual(
        mesh_from_cadquery(_switch_bezel_shape(), "switch_bezel"),
        origin=Origin(xyz=(switch_origin_x, 0.0, switch_origin_z)),
        material=blk,
        name="switch_bezel",
    )

    # ----------------------------------------------------------------- nozzle (fixed)
    nozzle = model.part("nozzle")
    nozzle.visual(
        mesh_from_cadquery(_nozzle_shape(), "nozzle_cone"),
        material=chrome,
        name="nozzle_cone",
    )
    # Front face of the barrel is near x = BARREL_LEN/2 - 0.030 + BARREL_LEN/2.
    barrel_front_x = (BARREL_LEN / 2.0 - 0.030) + BARREL_LEN / 2.0
    model.articulation(
        "body_to_nozzle",
        ArticulationType.FIXED,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(barrel_front_x - 0.006, 0.0, BARREL_Z)),
    )

    # ----------------------------------------------------------------- glue stick (fixed)
    glue = model.part("glue_stick")
    stick = (
        cq.Workplane("YZ")
        .circle(GLUE_BORE_R - 0.0015)
        .extrude(0.075)
    )
    glue.visual(
        mesh_from_cadquery(stick, "glue_stick_rod"),
        material=white,
        name="glue_stick_rod",
    )
    barrel_rear_x = (BARREL_LEN / 2.0 - 0.030) - BARREL_LEN / 2.0
    model.articulation(
        "body_to_glue_stick",
        ArticulationType.FIXED,
        parent=body,
        child=glue,
        # Stick protrudes out the rear and runs into the barrel bore.
        origin=Origin(xyz=(barrel_rear_x - 0.040, 0.0, BARREL_Z)),
    )

    # ----------------------------------------------------------------- trigger (REVOLUTE)
    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_cadquery(_trigger_shape(), "trigger_blade"),
        material=orange,
        name="trigger_blade",
    )
    # Pivot pin just below the front of the barrel / top of the handle.
    model.articulation(
        "trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(TRIGGER_PIVOT_X, 0.0, TRIGGER_PIVOT_Z)),
        # Blade hangs toward -Z and curves slightly +X. Squeezing pulls the free
        # bottom end rearward (-X) toward the grip. Right-hand rule about +Y
        # rotates -Z toward -X, swinging the bottom toward the handle.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.45),
    )

    # ----------------------------------------------------------------- power switch (REVOLUTE rocker)
    switch = model.part("power_switch")
    switch.visual(
        mesh_from_cadquery(_switch_shape(), "switch_rocker"),
        material=red,
        name="switch_rocker",
    )
    model.articulation(
        "power_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=switch,
        # Sit the rocker just proud of the bezel on the handle rear face.
        origin=Origin(xyz=(switch_origin_x - 0.003, 0.0, switch_origin_z)),
        # Rocker pivots about its horizontal mid-line (Y axis): top tips out / in.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.25, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    nozzle = object_model.get_part("nozzle")
    glue = object_model.get_part("glue_stick")
    trigger = object_model.get_part("trigger")
    switch = object_model.get_part("power_switch")

    trigger_joint = object_model.get_articulation("trigger")
    switch_joint = object_model.get_articulation("power_switch")
    nozzle_joint = object_model.get_articulation("body_to_nozzle")

    # ---- Intentional seated/captured fits (local, hidden, mechanical) ----
    # The glue stick is fed into the barrel bore; the nozzle collar plugs into the
    # front of the bore; the trigger pivot boss is captured inside the body at its
    # pin. These are real seating/capture fits, not stray collisions.
    ctx.allow_overlap(
        body,
        glue,
        elem_a="body_shell",
        elem_b="glue_stick_rod",
        reason="The white glue stick is fed into and retained inside the barrel bore.",
    )
    ctx.allow_overlap(
        body,
        nozzle,
        elem_a="body_shell",
        elem_b="nozzle_cone",
        reason="The nozzle collar plugs into the front of the barrel bore to seat the spout.",
    )
    ctx.allow_overlap(
        body,
        trigger,
        elem_a="body_shell",
        elem_b="trigger_blade",
        reason="The trigger pivot boss is captured inside the body around its pin.",
    )

    # ---- Joint types & axes are what we claim ----
    ctx.check(
        "trigger is revolute",
        trigger_joint.joint_type == "revolute",
        details=f"got {trigger_joint.joint_type}",
    )
    ctx.check(
        "trigger axis is Y",
        abs(abs(trigger_joint.axis[1]) - 1.0) < 1e-6
        and abs(trigger_joint.axis[0]) < 1e-6
        and abs(trigger_joint.axis[2]) < 1e-6,
        details=f"got {trigger_joint.axis}",
    )
    ctx.check(
        "switch is revolute",
        switch_joint.joint_type == "revolute",
        details=f"got {switch_joint.joint_type}",
    )
    ctx.check(
        "nozzle is fixed to body",
        nozzle_joint.joint_type == "fixed",
        details=f"got {nozzle_joint.joint_type}",
    )

    # ---- Hero parts exist and are placed correctly relative to the body ----
    # Nozzle is forward of the body's front and points along +X.
    body_aabb = ctx.part_world_aabb(body)
    nozzle_aabb = ctx.part_world_aabb(nozzle)
    assert body_aabb is not None and nozzle_aabb is not None
    ctx.check(
        "nozzle protrudes forward of body",
        nozzle_aabb[1][0] > body_aabb[1][0] + 0.02,
        details=f"nozzle max x={nozzle_aabb[1][0]:.4f} body max x={body_aabb[1][0]:.4f}",
    )
    # Nozzle tip is narrow (chrome cone tapers to a point).
    ctx.check(
        "nozzle tapers to a narrow tip",
        (nozzle_aabb[1][1] - nozzle_aabb[0][1]) <= 2.0 * NOZZLE_BASE_R + 0.001,
        details=f"nozzle y-extent={(nozzle_aabb[1][1] - nozzle_aabb[0][1]):.4f}",
    )

    # Glue stick protrudes out the rear (behind the body).
    glue_aabb = ctx.part_world_aabb(glue)
    assert glue_aabb is not None
    ctx.check(
        "glue stick protrudes out the rear",
        glue_aabb[0][0] < body_aabb[0][0] + 0.005,
        details=f"glue min x={glue_aabb[0][0]:.4f} body min x={body_aabb[0][0]:.4f}",
    )

    # Body is taller than wide (pistol grip drops well below the barrel).
    ctx.check(
        "body reads as a tall pistol grip",
        (body_aabb[1][2] - body_aabb[0][2]) > (body_aabb[1][1] - body_aabb[0][1]),
        details=f"z-extent={(body_aabb[1][2] - body_aabb[0][2]):.4f}, "
        f"y-extent={(body_aabb[1][1] - body_aabb[0][1]):.4f}",
    )

    # ---- Nozzle collar is retained inside the barrel bore (proves seating) ----
    ctx.expect_overlap(
        nozzle,
        body,
        axes="x",
        elem_a="nozzle_cone",
        elem_b="body_shell",
        min_overlap=0.010,
        name="nozzle collar stays inserted in the body",
    )
    ctx.expect_contact(
        nozzle,
        body,
        elem_a="nozzle_cone",
        elem_b="body_shell",
        name="nozzle touches the body",
    )

    # ---- Trigger sits below the barrel and is captured at the pivot ----
    trigger_aabb = ctx.part_world_aabb(trigger)
    assert trigger_aabb is not None
    ctx.check(
        "trigger hangs below the barrel",
        trigger_aabb[0][2] < BARREL_Z - BARREL_H / 2.0,
        details=f"trigger min z={trigger_aabb[0][2]:.4f}",
    )

    # ---- Glue stick is retained inside the barrel bore (proves the feed) ----
    ctx.expect_overlap(
        glue,
        body,
        axes="x",
        elem_a="glue_stick_rod",
        elem_b="body_shell",
        min_overlap=0.020,
        name="glue stick stays inserted in the bore",
    )

    # ---- Actuate the trigger: squeezing swings the free bottom end rearward (-X) ----
    rest = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_joint: 0.42}):
        squeezed = ctx.part_world_aabb(trigger)
    assert rest is not None and squeezed is not None
    # The free bottom edge of the blade hangs below the pivot; squeezing rotates it
    # rearward toward the grip, so the blade's rearmost reach (min x) moves toward -X.
    rest_rear_x = rest[0][0]
    sq_rear_x = squeezed[0][0]
    ctx.check(
        "squeezing trigger pulls free end rearward",
        sq_rear_x < rest_rear_x - 0.005,
        details=f"rest min x={rest_rear_x:.4f}, squeezed min x={sq_rear_x:.4f}",
    )

    # ---- Actuate the rocker: top of switch should swing in X when toggled ----
    rest_sw = ctx.part_world_aabb(switch)
    with ctx.pose({switch_joint: 0.22}):
        tipped_sw = ctx.part_world_aabb(switch)
    assert rest_sw is not None and tipped_sw is not None
    ctx.check(
        "power rocker tilts when toggled",
        abs(tipped_sw[0][0] - rest_sw[0][0]) > 0.001
        or abs(tipped_sw[1][2] - rest_sw[1][2]) > 0.001,
        details=f"rest={rest_sw}, tipped={tipped_sw}",
    )

    return ctx.report()


object_model = build_object_model()
