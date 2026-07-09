from __future__ import annotations

# Compact mini-stapler fork: a short, stubby desktop stapler with a blunt
# rounded toe and proportionally taller base walls. Same rear-hinged swing-up
# top arm, anvil plate, and carrier rail as the full-size parent stapler.
#
# Key structural change vs parent: the lower body is ~60 % the parent length
# (92 mm vs 158 mm) with walls roughly twice as tall relative to body length,
# and a blunt rounded front toe instead of a long tapered nose.
#
# Axes (model frame):
#   +X = length (front jaw at +X, rear hinge at -X)
#   +Y = width
#   +Z = up
#
# Primary articulation: base_to_arm, REVOLUTE about the rear hinge (axis -Y).
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
# Dimensions (meters) — compact mini-stapler variant
# ---------------------------------------------------------------------------
TOTAL_LEN = 0.092          # short nose-to-tail (parent was 0.158)
WIDTH = 0.034              # slightly narrower body
WALL = 0.0015              # plastic shell wall thickness

# Lower body: shorter tray, blunt rounded toe, proportionally taller walls
BASE_LEN = TOTAL_LEN
DECK_FRONT_Z = 0.013       # tall front deck (blunt stubby toe)
DECK_MID_Z = 0.016         # taller mid deck
BASE_REAR_H = 0.025        # taller rear heel (wall / length ratio ~0.27)
BASE_FOOT_Z = 0.0

# Top arm (magazine cover / handle hump)
ARM_LEN = 0.084
ARM_MAX_H = 0.032

# Rear hinge pivot
HINGE_X = -TOTAL_LEN / 2.0 + 0.008
HINGE_Z = 0.026
HINGE_PIN_R = 0.002
HINGE_PIN_LEN = WIDTH + 0.004
HINGE_KNUCKLE_R = 0.0036
HINGE_KNUCKLE_LEN = 0.007

# Metal anvil plate
ANVIL_LEN = 0.022
ANVIL_W = 0.022
ANVIL_TH = 0.0015
ANVIL_X = TOTAL_LEN / 2.0 - 0.020
ANVIL_SEAT_Z = DECK_FRONT_Z + 0.0005

# Staple carrier rail
RAIL_LEN = 0.055
RAIL_W = 0.010
RAIL_H = 0.009

# Colors / materials
COL_BODY = (0.20, 0.21, 0.22, 1.0)        # charcoal plastic body
COL_ARM = (0.235, 0.245, 0.255, 1.0)      # slightly lighter charcoal arm
COL_METAL = (0.78, 0.79, 0.80, 1.0)       # bright steel
COL_DARK_METAL = (0.42, 0.43, 0.45, 1.0)  # darker pin / hardware


# ---------------------------------------------------------------------------
# Shared geometry helpers (used by for-i-in-range loops in build)
# ---------------------------------------------------------------------------
def _foot_pad_shape() -> cq.Workplane:
    """Single rubber foot pad, built at origin (centered XY, bottom at Z=0)."""
    return (
        cq.Workplane("XY")
        .box(0.011, WIDTH - 0.010, 0.0025, centered=(True, True, False))
        .edges("|Z").fillet(0.001)
    )


def _knuckle_shape() -> cq.Workplane:
    """Single hinge knuckle at origin: bored ring + support skirt that ties
    down to the base heel.  Ring axis is +Y (the pivot axis)."""
    kn = (
        cq.Workplane("XZ")
        .circle(HINGE_KNUCKLE_R)
        .extrude(HINGE_KNUCKLE_LEN / 2.0, both=True)
    )
    skirt = (
        cq.Workplane("XY")
        .box(
            2 * HINGE_KNUCKLE_R,
            HINGE_KNUCKLE_LEN,
            HINGE_Z,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -HINGE_Z / 2.0))
    )
    knuckle = kn.union(skirt)
    bore = (
        cq.Workplane("XZ")
        .circle(HINGE_PIN_R + 0.0005)
        .extrude(0.04, both=True)
    )
    knuckle = knuckle.cut(bore)
    return knuckle


# ---------------------------------------------------------------------------
# Main geometry builders (CadQuery, authored in meters)
# ---------------------------------------------------------------------------
def _base_body() -> cq.Workplane:
    """Short stubby lower tray: blunt rounded toe, tall walls, rear heel."""
    x0 = -BASE_LEN / 2.0
    x1 = BASE_LEN / 2.0
    pts = [
        (x0, BASE_FOOT_Z),                       # rear bottom
        (x0, BASE_REAR_H),                       # rear top (heel)
        (x0 + 0.010, BASE_REAR_H),               # heel flat
        (HINGE_X + 0.006, 0.021),                # shoulder ahead of hinge
        (-0.010, DECK_MID_Z),                    # mid deck
        (0.010, DECK_MID_Z),                     # mid deck forward
        (x1 - 0.016, DECK_FRONT_Z),              # approach to front
        (x1 - 0.004, DECK_FRONT_Z - 0.001),      # near toe
        (x1, 0.006),                             # blunt rounded toe tip
        (x1, BASE_FOOT_Z),                       # front bottom
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    body = prof.extrude(WIDTH / 2.0, both=True)
    body = body.edges("|Y and >Z").fillet(0.002)
    return body


def _anvil_plate() -> cq.Workplane:
    """Bright metal anvil plate with two clinch grooves, on the front deck."""
    plate = (
        cq.Workplane("XY")
        .box(ANVIL_LEN, ANVIL_W, ANVIL_TH, centered=(True, True, False))
        .edges("|Z").fillet(0.002)
    )
    groove = (
        cq.Workplane("XY")
        .box(0.0022, 0.010, 0.0013, centered=(True, True, False))
    )
    plate = plate.cut(groove.translate((-0.003, 0.0, ANVIL_TH - 0.0012)))
    plate = plate.cut(groove.translate((0.003, 0.0, ANVIL_TH - 0.0012)))
    return plate.translate((ANVIL_X, 0.0, ANVIL_SEAT_Z))


def _arm_shell() -> cq.Workplane:
    """Top magazine cover / handle: rounded hump.  Authored in the arm child
    frame where the pivot sits at local origin and the cover extends +X.
    Hollowed underside, two rear hinge cheeks reaching down to the pin."""
    L = ARM_LEN
    crown = ARM_MAX_H
    bottom = 0.005
    pts = [
        (0.003, bottom),
        (0.003, crown * 0.62),
        (0.010, crown * 0.92),
        (0.024, crown),
        (L - 0.028, crown),
        (L - 0.008, crown * 0.86),
        (L, crown * 0.42),
        (L, bottom),
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    shell = prof.extrude(WIDTH / 2.0, both=True)
    shell = shell.edges("|Y and >Z").fillet(0.005)
    shell = shell.edges("|Y and >X").fillet(0.003)
    shell = shell.edges("|X and >Z").fillet(0.004)
    # Hollow the underside
    cavity = (
        cq.Workplane("XZ")
        .polyline([
            (0.010, bottom + 0.0015),
            (0.010, crown * 0.70),
            (0.024, crown * 0.78),
            (L - 0.032, crown * 0.78),
            (L - 0.016, crown * 0.55),
            (L - 0.010, bottom + 0.0015),
        ])
        .close()
        .extrude((WIDTH / 2.0) - WALL, both=True)
    )
    shell = shell.cut(cavity)
    # Rear hinge cheeks descend from cover bottom to the pivot pin
    cheek = (
        cq.Workplane("XZ")
        .polyline([
            (-0.004, 0.0),
            (0.008, 0.0),
            (0.008, bottom + 0.002),
            (-0.004, bottom + 0.002),
        ])
        .close()
        .extrude(0.004)
    )
    left_cheek = cheek.translate((0.0, 0.006, 0.0))
    right_cheek = cheek.translate((0.0, -0.006 - 0.004, 0.0))
    shell = shell.union(left_cheek).union(right_cheek)
    return shell


def _hinge_pin() -> cq.Workplane:
    """Transverse hinge pin along +Y, authored in arm frame at pivot."""
    return (
        cq.Workplane("XZ")
        .circle(HINGE_PIN_R)
        .extrude(HINGE_PIN_LEN / 2.0, both=True)
    )


def _carrier_rail() -> cq.Workplane:
    """Metal staple carrier rail under the arm (rides with the arm)."""
    rail = (
        cq.Workplane("XY")
        .box(RAIL_LEN, RAIL_W, RAIL_H, centered=(True, True, False))
        .edges("|X").fillet(0.0012)
    )
    pusher = cq.Workplane("XY").box(
        0.005, RAIL_W - 0.002, RAIL_H + 0.003,
        centered=(True, True, False),
    )
    rail = rail.union(pusher.translate((-RAIL_LEN / 2.0 + 0.003, 0.0, 0.0)))
    rail = rail.translate((0.020 + RAIL_LEN / 2.0, 0.0, 0.003))
    return rail


def _driver_blade() -> cq.Workplane:
    """Driver / anvil head blade at the arm nose."""
    blade = (
        cq.Workplane("XY")
        .box(0.005, 0.018, 0.014, centered=(True, True, False))
        .edges("|Z").fillet(0.001)
    )
    return blade.translate((ARM_LEN - 0.014, 0.0, 0.001))


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_stapler")

    body_mat = model.material("body_plastic", rgba=COL_BODY)
    arm_mat = model.material("arm_plastic", rgba=COL_ARM)
    metal_mat = model.material("bright_steel", rgba=COL_METAL)
    dark_mat = model.material("dark_steel", rgba=COL_DARK_METAL)

    # ---- Base (root) --------------------------------------------------
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "base_body"),
        material=body_mat, name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_anvil_plate(), "anvil_plate"),
        material=metal_mat, name="anvil_plate",
    )

    # Foot pads — repeated sub-parts via for-i-in-range loop
    FOOT_PAD_X = [-BASE_LEN / 2.0 + 0.010, BASE_LEN / 2.0 - 0.012]
    for i in range(len(FOOT_PAD_X)):
        pad = _foot_pad_shape().translate((FOOT_PAD_X[i], 0.0, -0.0005))
        base.visual(
            mesh_from_cadquery(pad, f"foot_pad_{i}"),
            material=dark_mat, name=f"foot_pad_{i}",
        )

    # Hinge knuckles — repeated sub-parts via for-i-in-range loop
    KNUCKLE_Y = [WIDTH / 2.0 - 0.003, -(WIDTH / 2.0 - 0.003)]
    for i in range(len(KNUCKLE_Y)):
        kn = _knuckle_shape().translate((HINGE_X, KNUCKLE_Y[i], HINGE_Z))
        base.visual(
            mesh_from_cadquery(kn, f"hinge_knuckle_{i}"),
            material=dark_mat, name=f"hinge_knuckle_{i}",
        )

    # ---- Top arm (magazine cover / handle) ----------------------------
    arm = model.part("top_arm")
    arm.visual(
        mesh_from_cadquery(_arm_shell(), "arm_shell"),
        material=arm_mat, name="arm_shell",
    )
    arm.visual(
        mesh_from_cadquery(_carrier_rail(), "carrier_rail"),
        material=metal_mat, name="carrier_rail",
    )
    arm.visual(
        mesh_from_cadquery(_driver_blade(), "driver_blade"),
        material=metal_mat, name="driver_blade",
    )
    arm.visual(
        mesh_from_cadquery(_hinge_pin(), "hinge_pin"),
        material=dark_mat, name="hinge_pin",
    )

    # ---- Rear hinge articulation --------------------------------------
    # Arm child frame has pivot at local origin, cover extends +X.
    # At q=0 the arm origin coincides with the hinge frame on the base.
    # axis=-Y makes positive q lift the front nose upward.
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=3.0, lower=0.0, upper=0.55,
        ),
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

    # --- Compact mini-stapler proportions (the targeted change) ---------
    body_aabb = ctx.part_element_world_aabb(base, elem="base_body")
    if body_aabb is not None:
        body_dx = body_aabb[1][0] - body_aabb[0][0]
        body_dz = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "compact body length (< 0.11 m)",
            body_dx < 0.11,
            details=f"body_length={body_dx:.4f}",
        )
        ctx.check(
            "stubby wall-to-length ratio (> 0.24)",
            body_dz / body_dx > 0.24,
            details=f"ratio={body_dz / body_dx:.3f}",
        )

    # Blunt rounded toe: front deck is tall (not thin/tapered)
    ctx.check(
        "blunt stubby toe (front deck >= 0.011 m)",
        DECK_FRONT_Z >= 0.011,
        details=f"deck_front_z={DECK_FRONT_Z}",
    )

    # --- Joint contract ------------------------------------------------
    ctx.check(
        "hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis is transverse (-Y, positive q opens upward)",
        tuple(round(a, 6) for a in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "hinge opens upward from closed (lower=0, upper>0.3)",
        lim is not None and lim.lower == 0.0 and lim.upper > 0.3,
        details=f"limits=({lim.lower},{lim.upper})",
    )

    # --- Hero parts exist ----------------------------------------------
    arm_shell = arm.get_visual("arm_shell")
    rail = arm.get_visual("carrier_rail")
    blade = arm.get_visual("driver_blade")
    pin = arm.get_visual("hinge_pin")
    anvil = base.get_visual("anvil_plate")
    kn0 = base.get_visual("hinge_knuckle_0")
    kn1 = base.get_visual("hinge_knuckle_1")
    fp0 = base.get_visual("foot_pad_0")
    fp1 = base.get_visual("foot_pad_1")
    for label, vis in [
        ("base_body", base.get_visual("base_body")),
        ("anvil_plate", anvil),
        ("arm_shell", arm_shell),
        ("carrier_rail", rail),
        ("driver_blade", blade),
        ("hinge_pin", pin),
        ("hinge_knuckle_0", kn0),
        ("hinge_knuckle_1", kn1),
        ("foot_pad_0", fp0),
        ("foot_pad_1", fp1),
    ]:
        ctx.check(f"{label} present", vis is not None, details=f"{label} missing")

    # --- Closed pose: cover sits above base tray -----------------------
    with ctx.pose({hinge: 0.0}):
        shell_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
        body_aabb2 = ctx.part_element_world_aabb(base, elem="base_body")
        ctx.check(
            "cover crown rises above base body",
            shell_aabb is not None
            and body_aabb2 is not None
            and shell_aabb[1][2] > body_aabb2[1][2] + 0.020,
            details=(
                f"shell_top={shell_aabb[1][2]:.4f} "
                f"body_top={body_aabb2[1][2]:.4f}"
            ),
        )

        ctx.expect_overlap(
            arm, base, axes="xy", min_overlap=0.02,
            name="closed cover covers base tray footprint",
        )

        anvil_aabb = ctx.part_element_world_aabb(base, elem="anvil_plate")
        ctx.check(
            "anvil plate seated on front deck",
            anvil_aabb is not None and 0.010 < anvil_aabb[0][2] < 0.018,
            details=f"anvil_min_z={anvil_aabb[0][2] if anvil_aabb else None}",
        )

        blade_aabb = ctx.part_element_world_aabb(arm, elem="driver_blade")
        ctx.check(
            "driver blade registers above anvil",
            blade_aabb is not None
            and anvil_aabb is not None
            and blade_aabb[0][2] >= anvil_aabb[1][2] - 0.002
            and abs(blade_aabb[0][0] - anvil_aabb[0][0]) < 0.04,
            details=(
                f"blade_z={blade_aabb[0][2]:.4f} "
                f"anvil_z={anvil_aabb[1][2]:.4f}"
            ),
        )

        # Hinge pin captured by knuckle bore
        ctx.expect_contact(
            arm, base, elem_a=pin, elem_b=kn0, contact_tol=0.002,
            name="hinge pin captured by knuckle_0",
        )
        closed_nose = ctx.part_element_world_aabb(arm, elem="driver_blade")

    # --- Open pose: lifting the arm raises the driver blade ------------
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_nose = ctx.part_element_world_aabb(arm, elem="driver_blade")

    ctx.check(
        "opening hinge lifts driver blade upward",
        closed_nose is not None
        and open_nose is not None
        and open_nose[0][2] > closed_nose[0][2] + 0.020,
        details=(
            f"closed_z={closed_nose[0][2]:.4f} "
            f"open_z={open_nose[0][2]:.4f}"
        ),
    )

    # --- Intentional hinge overlaps (scoped) ---------------------------
    for i in range(2):
        ctx.allow_overlap(
            arm, base,
            elem_a="hinge_pin", elem_b=f"hinge_knuckle_{i}",
            reason=f"Hinge pin captured inside knuckle_{i} bore.",
        )
        ctx.allow_overlap(
            arm, base,
            elem_a="arm_shell", elem_b=f"hinge_knuckle_{i}",
            reason=f"Arm hinge cheek interleaves with knuckle_{i} on shared pivot.",
        )

    return ctx.report()


object_model = build_object_model()
