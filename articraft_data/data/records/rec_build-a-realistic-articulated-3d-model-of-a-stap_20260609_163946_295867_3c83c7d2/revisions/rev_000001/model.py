from __future__ import annotations

# Realistic desktop half-strip stapler, modeled from picture/Handtools/Stapler/001.png.
#
# Real object: a charcoal/dark-gray office stapler. The big rounded "hump" on
# top is the magazine-cover / handle arm. It is hinged at the rear and swings up
# about a transverse pivot pin so staples can be loaded and so the head presses
# down to drive a staple. Underneath the cover is a metal staple carrier rail.
# The lower body is a long tray with a rounded toe; near the front sits a bright
# metal anvil plate carrying the two staple-clinching grooves.
#
# Axes (model frame):
#   +X = length (front of jaw at +X, rear hinge at -X)
#   +Y = width
#   +Z = up
#
# Primary articulation: base_to_arm, REVOLUTE about the rear hinge (axis +Y).
# Positive q lifts the front of the top arm upward (open the stapler).

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
# Dimensions (meters)
# ---------------------------------------------------------------------------
TOTAL_LEN = 0.158  # nose-to-tail length of the closed body
WIDTH = 0.040  # body width
WALL = 0.0015  # plastic shell wall thickness

# Lower body (base tray). The deck under the cover is kept low so the cover
# clears it; only the rear heel rises to carry the hinge.
BASE_LEN = 0.158
DECK_FRONT_Z = 0.010  # thin deck at the front jaw
DECK_MID_Z = 0.012  # deck under the cover
BASE_REAR_H = 0.019  # rear heel that carries the hinge mount
BASE_FOOT_Z = 0.0  # base sits on z=0

# Top arm (magazine cover / handle) - the big curved hump
ARM_LEN = 0.150
ARM_MAX_H = 0.036  # crown height of the hump above its underside

# Rear hinge. Pivot sits above the rear heel so the cover underside clears the
# base deck in the closed pose.
HINGE_X = -TOTAL_LEN / 2.0 + 0.009  # pivot near the rear
HINGE_Z = 0.0205  # pivot height above z=0
HINGE_PIN_R = 0.0024
HINGE_PIN_LEN = WIDTH + 0.004
HINGE_KNUCKLE_R = 0.0042
HINGE_KNUCKLE_LEN = 0.010

# Metal anvil plate (front, on the base top deck) carrying clinch grooves
ANVIL_LEN = 0.030
ANVIL_W = 0.026
ANVIL_TH = 0.0016
ANVIL_X = TOTAL_LEN / 2.0 - 0.030
ANVIL_SEAT_Z = DECK_FRONT_Z + 0.0005  # plate rests just on the front deck

# Staple carrier rail under the arm (the metal track)
RAIL_LEN = 0.110
RAIL_W = 0.012
RAIL_H = 0.010

# Colors
COL_BODY = (0.20, 0.21, 0.22, 1.0)  # charcoal plastic
COL_ARM = (0.235, 0.245, 0.255, 1.0)  # slightly lighter charcoal hump
COL_METAL = (0.78, 0.79, 0.80, 1.0)  # bright steel anvil / rail
COL_DARK_METAL = (0.42, 0.43, 0.45, 1.0)  # darker pin / carrier


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters)
# ---------------------------------------------------------------------------
def _base_body() -> cq.Workplane:
    """Lower tray: long body with a rounded toe, a low deck under the cover, and
    a raised rear heel that carries the hinge. Built as a lofted side profile so
    the silhouette tapers like the real base."""
    x0 = -BASE_LEN / 2.0
    x1 = BASE_LEN / 2.0
    # Side profile in XZ: rear heel taller (hinge mount), then a long low deck
    # that drops to a thin, rounded front toe.
    pts = [
        (x0, BASE_FOOT_Z),
        (x0, BASE_REAR_H),
        (x0 + 0.014, BASE_REAR_H),
        (HINGE_X + 0.008, 0.016),  # shoulder just ahead of the hinge
        (-0.020, DECK_MID_Z),
        (0.040, DECK_MID_Z),
        (x1 - 0.030, DECK_FRONT_Z),
        (x1 - 0.012, DECK_FRONT_Z - 0.002),
        (x1, 0.004),  # rounded toe tip
        (x1, BASE_FOOT_Z),
    ]
    prof = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
    )
    body = prof.extrude(WIDTH / 2.0, both=True)
    # Round the long top edges so the tray reads as soft plastic. Keep the
    # radius small so the thin front toe stays a valid fillet.
    body = body.edges("|Y and >Z").fillet(0.0022)
    return body


def _base_heel_pads() -> cq.Workplane:
    """Two rubber-style feet pads under the rear heel so the body reads as
    grounded; kept as part of the base shell visual region."""
    pad = (
        cq.Workplane("XY")
        .box(0.014, WIDTH - 0.008, 0.0025, centered=(True, True, False))
        .translate((-BASE_LEN / 2.0 + 0.012, 0.0, -0.0005))
    )
    return pad


def _anvil_plate() -> cq.Workplane:
    """Bright metal anvil plate seated on the front deck with two clinch
    grooves (the slots that bend staple legs)."""
    plate = (
        cq.Workplane("XY")
        .box(ANVIL_LEN, ANVIL_W, ANVIL_TH, centered=(True, True, False))
        .edges("|Z").fillet(0.002)
    )
    # Two clinch grooves running across the plate width.
    groove = cq.Workplane("XY").box(0.0026, 0.012, 0.0014, centered=(True, True, False))
    plate = plate.cut(groove.translate((-0.004, 0.0, ANVIL_TH - 0.0013)))
    plate = plate.cut(groove.translate((0.004, 0.0, ANVIL_TH - 0.0013)))
    return plate.translate((ANVIL_X, 0.0, ANVIL_SEAT_Z))


def _arm_shell() -> cq.Workplane:
    """The top magazine cover / handle: the big rounded hump. Authored in the
    ARM (child) local frame where the hinge pivot sits at local origin and the
    cover extends along +X. Built from a curved side silhouette extruded across
    the width, hollowed underneath, with two rear cheeks that reach down to the
    hinge pin so the whole arm is one connected solid."""
    L = ARM_LEN
    crown = ARM_MAX_H
    # Side silhouette of the cover in XZ. The cover bottom is a few mm above the
    # pivot (arm-z=0) so it clears the base deck; the rear cheeks bridge down.
    bottom = 0.006
    pts = [
        (0.004, bottom),
        (0.004, crown * 0.62),
        (0.016, crown * 0.92),
        (0.044, crown),
        (L - 0.045, crown),
        (L - 0.014, crown * 0.86),
        (L, crown * 0.42),  # rounded front nose
        (L, bottom),
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    shell = prof.extrude(WIDTH / 2.0, both=True)
    # Round the long crown edges and the nose to get the soft pebble shape.
    shell = shell.edges("|Y and >Z").fillet(0.010)
    shell = shell.edges("|Y and >X").fillet(0.006)
    shell = shell.edges("|X and >Z").fillet(0.008)
    # Hollow the underside so the cover is a real shell, not a solid block.
    cavity = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.014, bottom + 0.0015),
                (0.014, crown * 0.70),
                (0.044, crown * 0.78),
                (L - 0.050, crown * 0.78),
                (L - 0.024, crown * 0.55),
                (L - 0.018, bottom + 0.0015),
            ]
        )
        .close()
        .extrude((WIDTH / 2.0) - WALL, both=True)
    )
    shell = shell.cut(cavity)
    # Two rear cheeks descend from the cover bottom to the hinge pin, forming the
    # arm-side hinge ears that the pin runs through. They sit inboard of the base
    # knuckles (which are near the outer edges) so they interleave on the shared
    # pin instead of colliding with the knuckle rings.
    cheek = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.005, 0.0),
                (0.012, 0.0),
                (0.012, bottom + 0.002),
                (-0.005, bottom + 0.002),
            ]
        )
        .close()
        .extrude(0.005)
    )
    left_cheek = cheek.translate((0.0, 0.006, 0.0))
    right_cheek = cheek.translate((0.0, -0.006 - 0.005, 0.0))
    shell = shell.union(left_cheek).union(right_cheek)
    return shell


def _hinge_knuckles() -> cq.Workplane:
    """Two visible hinge knuckles on the rear heel of the base that capture the
    pin, authored in the BASE frame around the pivot at (HINGE_X, *, HINGE_Z).
    Each knuckle is bored for the pin and skirted down to the heel so it reads
    as mounted, not floating."""
    # The knuckle ring axis runs along +Y (the pivot axis); XZ workplane normal
    # is +Y, so extruding it builds a Y-axis cylinder.
    kn = cq.Workplane("XZ").circle(HINGE_KNUCKLE_R).extrude(HINGE_KNUCKLE_LEN / 2.0, both=True)
    # Skirt connecting the knuckle ring down into the rear heel deck.
    skirt = cq.Workplane("XY").box(
        2 * HINGE_KNUCKLE_R, HINGE_KNUCKLE_LEN, HINGE_Z, centered=(True, True, False)
    ).translate((0.0, 0.0, -HINGE_Z / 2.0))
    knuckle = kn.union(skirt)
    # Bore the pin hole through the ring along +Y.
    bore = cq.Workplane("XZ").circle(HINGE_PIN_R + 0.0006).extrude(0.05, both=True)
    knuckle = knuckle.cut(bore)
    left = knuckle.translate((HINGE_X, WIDTH / 2.0 - 0.003, HINGE_Z))
    right = knuckle.translate((HINGE_X, -(WIDTH / 2.0 - 0.003), HINGE_Z))
    return left.union(right)


def _hinge_pin() -> cq.Workplane:
    """Transverse hinge pin spanning the rear along +Y, authored in the ARM
    frame at the arm local origin (the pivot). XZ workplane normal is +Y, so
    extruding it builds a Y-axis cylinder (the pivot axis)."""
    pin = (
        cq.Workplane("XZ")
        .circle(HINGE_PIN_R)
        .extrude(HINGE_PIN_LEN / 2.0, both=True)
    )
    return pin


def _carrier_rail() -> cq.Workplane:
    """Metal staple carrier rail under the arm; rides with the arm. Authored in
    the ARM frame, hanging below the cover crown toward the deck."""
    rail = (
        cq.Workplane("XY")
        .box(RAIL_LEN, RAIL_W, RAIL_H, centered=(True, True, False))
        .edges("|X").fillet(0.0015)
    )
    # An end pusher block (spring follower face) at the rear.
    pusher = cq.Workplane("XY").box(0.006, RAIL_W - 0.002, RAIL_H + 0.004, centered=(True, True, False))
    rail = rail.union(pusher.translate((-RAIL_LEN / 2.0 + 0.003, 0.0, 0.0)))
    # Place along the arm underside: front of rail near the nose, just below crown.
    rail = rail.translate((0.030 + RAIL_LEN / 2.0, 0.0, 0.003))
    return rail


def _driver_blade() -> cq.Workplane:
    """The driver/anvil head at the front of the arm: a thin metal blade tongue
    that pushes the staple down. Authored in the ARM frame, at the nose."""
    blade = (
        cq.Workplane("XY")
        .box(0.006, 0.020, 0.016, centered=(True, True, False))
        .edges("|Z").fillet(0.001)
    )
    return blade.translate((ARM_LEN - 0.018, 0.0, 0.001))


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    body_mat = model.material("body_plastic", rgba=COL_BODY)
    arm_mat = model.material("arm_plastic", rgba=COL_ARM)
    metal_mat = model.material("bright_steel", rgba=COL_METAL)
    dark_metal_mat = model.material("dark_steel", rgba=COL_DARK_METAL)

    # ---- Base (root) -----------------------------------------------------
    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_body(), "base_body"), material=body_mat, name="base_body")
    base.visual(mesh_from_cadquery(_base_heel_pads(), "base_feet"), material=dark_metal_mat, name="base_feet")
    base.visual(mesh_from_cadquery(_anvil_plate(), "anvil_plate"), material=metal_mat, name="anvil_plate")
    base.visual(mesh_from_cadquery(_hinge_knuckles(), "hinge_knuckles"), material=dark_metal_mat, name="hinge_knuckles")

    # ---- Top arm (magazine cover / handle) -------------------------------
    arm = model.part("top_arm")
    arm.visual(mesh_from_cadquery(_arm_shell(), "arm_shell"), material=arm_mat, name="arm_shell")
    arm.visual(mesh_from_cadquery(_carrier_rail(), "carrier_rail"), material=metal_mat, name="carrier_rail")
    arm.visual(mesh_from_cadquery(_driver_blade(), "driver_blade"), material=metal_mat, name="driver_blade")
    arm.visual(mesh_from_cadquery(_hinge_pin(), "hinge_pin"), material=dark_metal_mat, name="hinge_pin")

    # ---- Rear hinge ------------------------------------------------------
    # The arm is authored with its pivot at the arm local origin and the cover
    # extending along +X. At q=0 the arm origin coincides with the hinge frame
    # placed on the base at (HINGE_X, 0, HINGE_Z). The closed cover extends
    # along +X above the deck. The cover extends +X from the pivot, so axis=-Y
    # makes positive q lift the front nose upward (right-hand rule).
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.52),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("top_arm")
    hinge = object_model.get_articulation("base_to_arm")

    # --- Joint contract ---------------------------------------------------
    ctx.check(
        "hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis is transverse (along -Y so positive q opens upward)",
        tuple(round(a, 6) for a in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "hinge opens upward from closed (lower=0, upper>0)",
        lim is not None and lim.lower == 0.0 and lim.upper is not None and lim.upper > 0.3,
        details=f"limits=({lim.lower if lim else None},{lim.upper if lim else None})",
    )

    # --- Hero parts exist -------------------------------------------------
    base_body = base.get_visual("base_body")
    anvil = base.get_visual("anvil_plate")
    knuckles = base.get_visual("hinge_knuckles")
    arm_shell = arm.get_visual("arm_shell")
    rail = arm.get_visual("carrier_rail")
    blade = arm.get_visual("driver_blade")
    pin = arm.get_visual("hinge_pin")
    for label, vis in [
        ("base_body", base_body),
        ("anvil_plate", anvil),
        ("hinge_knuckles", knuckles),
        ("arm_shell", arm_shell),
        ("carrier_rail", rail),
        ("driver_blade", blade),
        ("hinge_pin", pin),
    ]:
        ctx.check(f"{label} present", vis is not None, details=f"{label} missing")

    # --- Closed pose: the top cover sits above and overlaps the base tray ----
    with ctx.pose({hinge: 0.0}):
        # The cover crown is clearly above the base deck (true clamshell stack):
        # the cover underside (arm_shell min-z) clears the front/mid deck.
        shell_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
        body_aabb = ctx.part_element_world_aabb(base, elem="base_body")
        ctx.check(
            "cover crown rises well above the base body",
            shell_aabb is not None
            and body_aabb is not None
            and shell_aabb[1][2] > body_aabb[1][2] + 0.025,
            details=f"shell_top={shell_aabb[1][2]:.4f} body_top={body_aabb[1][2]:.4f}",
        )

        # Footprint of the cover overlaps the base in plan view (it covers the
        # tray). Width is only ~40 mm, so require a modest y-overlap.
        ctx.expect_overlap(
            arm, base, axes="xy", min_overlap=0.03,
            name="closed cover covers the base tray footprint",
        )

        # The metal anvil plate rests just on top of the front deck, not floating
        # high above it nor sunk below it.
        anvil_aabb = ctx.part_element_world_aabb(base, elem="anvil_plate")
        ctx.check(
            "anvil plate seated low on the front deck",
            anvil_aabb is not None and 0.008 < anvil_aabb[0][2] < 0.014,
            details=f"anvil_min_z={anvil_aabb[0][2] if anvil_aabb else None}",
        )

        # The driver blade hangs above the anvil plate (jaw closed but clearing),
        # proving the head registers over the clinch plate.
        blade_aabb = ctx.part_element_world_aabb(arm, elem="driver_blade")
        ctx.check(
            "driver blade registers above the anvil plate",
            blade_aabb is not None
            and anvil_aabb is not None
            and blade_aabb[0][2] >= anvil_aabb[1][2] - 0.001
            and abs(blade_aabb[0][0] - anvil_aabb[0][0]) < 0.04,
            details=f"blade_min_z={blade_aabb[0][2]:.4f} anvil_max_z={anvil_aabb[1][2]:.4f}",
        )

        # Hinge pin (arm) is captured by the base knuckles at the rear pivot.
        ctx.expect_contact(
            arm, base, elem_a=pin, elem_b=knuckles, contact_tol=0.0015,
            name="hinge pin captured by base knuckles",
        )
        closed_nose = ctx.part_element_world_aabb(arm, elem="driver_blade")

    # --- Open pose: lifting the cover raises the front nose / driver blade ----
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_nose = ctx.part_element_world_aabb(arm, elem="driver_blade")

    ctx.check(
        "opening hinge lifts the front driver blade upward",
        closed_nose is not None
        and open_nose is not None
        and open_nose[0][2] > closed_nose[0][2] + 0.03,
        details=f"closed_blade_min_z={closed_nose[0][2]:.4f}, open_blade_min_z={open_nose[0][2]:.4f}",
    )

    # The hinge pin is intentionally captured inside the base knuckle bores, and
    # the arm-side cheeks interleave with the base knuckles on the shared pin.
    # Allow those local hinge overlaps so the pivot reads as a real knuckle joint.
    ctx.allow_overlap(
        arm,
        base,
        elem_a=pin,
        elem_b=knuckles,
        reason="The transverse hinge pin is intentionally captured inside the base hinge-knuckle bores.",
    )
    ctx.allow_overlap(
        arm,
        base,
        elem_a=arm_shell,
        elem_b=knuckles,
        reason="The arm-side hinge cheeks interleave with the base knuckles on the shared pivot pin.",
    )

    return ctx.report()


object_model = build_object_model()
