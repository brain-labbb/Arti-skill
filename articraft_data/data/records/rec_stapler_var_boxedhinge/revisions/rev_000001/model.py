from __future__ import annotations

# Realistic desktop half-strip stapler — enclosed-hinge housing variant.
#
# Fork of the parent stapler: the exposed transverse pin and knuckles at the
# rear are replaced by a tall boxed plastic housing that fully encloses the
# pivot mechanism. The top arm still swings up about the same revolute pivot,
# but the hardware is hidden inside the housing shell.
#
# Axes (model frame):
#   +X = length (front of jaw at +X, rear hinge at -X)
#   +Y = width
#   +Z = up
#
# Primary articulation: base_to_arm, REVOLUTE about the rear pivot (axis -Y).
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

# Lower body (base tray)
BASE_LEN = 0.158
DECK_FRONT_Z = 0.010
DECK_MID_Z = 0.012
BASE_REAR_H = 0.019
BASE_FOOT_Z = 0.0

# Top arm (magazine cover / handle)
ARM_LEN = 0.150
ARM_MAX_H = 0.036

# Rear pivot location (unchanged from parent)
HINGE_X = -TOTAL_LEN / 2.0 + 0.009
HINGE_Z = 0.0205

# Rear housing (boxed enclosure hiding the pivot)
HOUSING_FRONT_X = HINGE_X + 0.014  # front face of the housing
HOUSING_W = 0.046  # wider than the arm shell so the arm straddles the housing
HOUSING_H = 0.042  # tall enough to clearly enclose the pivot
HOUSING_WALL = 0.0025

# Metal anvil plate
ANVIL_LEN = 0.030
ANVIL_W = 0.026
ANVIL_TH = 0.0016
ANVIL_X = TOTAL_LEN / 2.0 - 0.030
ANVIL_SEAT_Z = DECK_FRONT_Z + 0.0005

# Staple carrier rail
RAIL_LEN = 0.110
RAIL_W = 0.012
RAIL_H = 0.010

# Colors
COL_BODY = (0.20, 0.21, 0.22, 1.0)  # charcoal plastic base
COL_ARM = (0.235, 0.245, 0.255, 1.0)  # slightly lighter charcoal hump
COL_METAL = (0.78, 0.79, 0.80, 1.0)  # bright steel anvil / rail
COL_DARK_METAL = (0.42, 0.43, 0.45, 1.0)  # darker steel accents
COL_HOUSING = (0.155, 0.165, 0.185, 1.0)  # darker blue-charcoal housing


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters)
# ---------------------------------------------------------------------------
def _base_body() -> cq.Workplane:
    """Lower tray: long body with a rounded toe, a low deck under the cover, and
    a raised rear heel that meets the housing. Built as a lofted side profile so
    the silhouette tapers like the real base."""
    x0 = -BASE_LEN / 2.0
    x1 = BASE_LEN / 2.0
    pts = [
        (x0, BASE_FOOT_Z),
        (x0, BASE_REAR_H),
        (x0 + 0.014, BASE_REAR_H),
        (HINGE_X + 0.008, 0.016),
        (-0.020, DECK_MID_Z),
        (0.040, DECK_MID_Z),
        (x1 - 0.030, DECK_FRONT_Z),
        (x1 - 0.012, DECK_FRONT_Z - 0.002),
        (x1, 0.004),
        (x1, BASE_FOOT_Z),
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    body = prof.extrude(WIDTH / 2.0, both=True)
    body = body.edges("|Y and >Z").fillet(0.0022)
    return body


def _foot_pad(x_pos: float) -> cq.Workplane:
    """Shared geometry helper: a rubber-style foot pad centred at *x_pos*."""
    return (
        cq.Workplane("XY")
        .box(0.014, WIDTH - 0.008, 0.0025, centered=(True, True, False))
        .edges("|Z").fillet(0.001)
        .translate((x_pos, 0.0, -0.0005))
    )


# Foot-pad X positions: rear heel and front jaw.
_FOOT_X_POSITIONS = [
    -BASE_LEN / 2.0 + 0.012,
    BASE_LEN / 2.0 - 0.022,
]


def _anvil_plate() -> cq.Workplane:
    """Bright metal anvil plate seated on the front deck with two clinch
    grooves (the slots that bend staple legs)."""
    plate = (
        cq.Workplane("XY")
        .box(ANVIL_LEN, ANVIL_W, ANVIL_TH, centered=(True, True, False))
        .edges("|Z").fillet(0.002)
    )
    groove = cq.Workplane("XY").box(
        0.0026, 0.012, 0.0014, centered=(True, True, False)
    )
    plate = plate.cut(groove.translate((-0.004, 0.0, ANVIL_TH - 0.0013)))
    plate = plate.cut(groove.translate((0.004, 0.0, ANVIL_TH - 0.0013)))
    return plate.translate((ANVIL_X, 0.0, ANVIL_SEAT_Z))


def _rear_housing() -> cq.Workplane:
    """Tall boxed plastic housing at the rear that encloses the pivot mechanism.

    Built as a U-channel open at the top-front: two tall side walls, a back
    wall, and a floor plate.  The arm shell straddles the housing (the housing
    is wider than the arm shell) and the arm's rear tongue extends down inside
    the cavity to the pivot.  The interior is hollow — the pivot pin and
    knuckles are represented as hidden inside the enclosure."""
    hx0 = -BASE_LEN / 2.0
    hx1 = HOUSING_FRONT_X
    h_len = hx1 - hx0
    h_w = HOUSING_W
    h_h = HOUSING_H
    wall = HOUSING_WALL

    # Outer solid box — fillet the outer edges *before* cutting the cavity so
    # that the fillet radius is not constrained by the thin wall thickness.
    outer = (
        cq.Workplane("XY")
        .box(h_len, h_w, h_h, centered=(False, True, False))
        .translate((hx0, 0.0, 0.0))
    )
    outer = outer.edges(">Z").fillet(0.002)
    outer = outer.edges("|Z").fillet(0.0015)

    # Interior cavity cut from the top and front, leaving the back wall, side
    # walls, and floor plate.
    cavity_l = h_len - wall  # leave a back wall
    cavity_w = h_w - 2.0 * wall
    cavity_h = h_h + 0.001  # extend above the top so the top is fully open
    cavity = (
        cq.Workplane("XY")
        .box(cavity_l, cavity_w, cavity_h, centered=(False, True, False))
        .translate((hx0 + wall, 0.0, wall))
    )
    housing = outer.cut(cavity)
    return housing


def _arm_shell() -> cq.Workplane:
    """The top magazine cover / handle: the big rounded hump.

    Authored in the ARM (child) local frame where the hinge pivot sits at the
    local origin and the cover extends along +X.  A single central rear tongue
    descends from the shell into the housing to the pivot (replacing the
    parent's two side cheeks that interleaved with exposed knuckles)."""
    L = ARM_LEN
    crown = ARM_MAX_H
    bottom = 0.006  # shell underside above the pivot (clears the base deck)

    # Side silhouette of the cover in XZ.
    pts = [
        (0.004, bottom),
        (0.004, crown * 0.62),
        (0.016, crown * 0.92),
        (0.044, crown),
        (L - 0.045, crown),
        (L - 0.014, crown * 0.86),
        (L, crown * 0.42),
        (L, bottom),
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    shell = prof.extrude(WIDTH / 2.0, both=True)

    # Round the crown edges and nose for the soft pebble shape.
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

    # Central rear tongue descending from the shell into the housing cavity.
    # This replaces the parent's two interleaving hinge cheeks.  The tongue
    # reaches down to the pivot (arm-local z≈0) so the whole arm is one
    # connected solid that pivots inside the housing.
    tongue_w = 0.020  # narrower than the housing cavity (~0.041)
    tongue = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.004, -0.002),
                (0.012, -0.002),
                (0.012, bottom + 0.003),
                (-0.004, bottom + 0.003),
            ]
        )
        .close()
        .extrude(tongue_w / 2.0, both=True)
    )
    # Round the lower tongue edges so the hidden pivot end reads as a moulded
    # bushing seat rather than a sharp block.
    tongue = tongue.edges("<Z").fillet(0.0015)
    shell = shell.union(tongue)
    return shell


def _carrier_rail() -> cq.Workplane:
    """Metal staple carrier rail under the arm; rides with the arm."""
    rail = (
        cq.Workplane("XY")
        .box(RAIL_LEN, RAIL_W, RAIL_H, centered=(True, True, False))
        .edges("|X").fillet(0.0015)
    )
    pusher = cq.Workplane("XY").box(
        0.006, RAIL_W - 0.002, RAIL_H + 0.004, centered=(True, True, False)
    )
    rail = rail.union(pusher.translate((-RAIL_LEN / 2.0 + 0.003, 0.0, 0.0)))
    rail = rail.translate((0.030 + RAIL_LEN / 2.0, 0.0, 0.003))
    return rail


def _driver_blade() -> cq.Workplane:
    """The driver/anvil head at the front of the arm: a thin metal blade tongue
    that pushes the staple down."""
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
    model = ArticulatedObject(name="desktop_stapler_enclosed_hinge")

    body_mat = model.material("body_plastic", rgba=COL_BODY)
    arm_mat = model.material("arm_plastic", rgba=COL_ARM)
    metal_mat = model.material("bright_steel", rgba=COL_METAL)
    dark_metal_mat = model.material("dark_steel", rgba=COL_DARK_METAL)
    housing_mat = model.material("housing_plastic", rgba=COL_HOUSING)

    # ---- Base (root) -----------------------------------------------------
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "base_body"),
        material=body_mat,
        name="base_body",
    )

    # Repeated foot pads emitted via a loop with shared geometry helper.
    for i, x_pos in enumerate(_FOOT_X_POSITIONS):
        base.visual(
            mesh_from_cadquery(_foot_pad(x_pos), f"foot_pad_{i}"),
            material=dark_metal_mat,
            name=f"foot_pad_{i}",
        )

    base.visual(
        mesh_from_cadquery(_anvil_plate(), "anvil_plate"),
        material=metal_mat,
        name="anvil_plate",
    )

    # Tall boxed rear housing — replaces the exposed knuckles from the parent.
    base.visual(
        mesh_from_cadquery(_rear_housing(), "rear_housing"),
        material=housing_mat,
        name="rear_housing",
    )

    # ---- Top arm (magazine cover / handle) -------------------------------
    arm = model.part("top_arm")
    arm.visual(
        mesh_from_cadquery(_arm_shell(), "arm_shell"),
        material=arm_mat,
        name="arm_shell",
    )
    arm.visual(
        mesh_from_cadquery(_carrier_rail(), "carrier_rail"),
        material=metal_mat,
        name="carrier_rail",
    )
    arm.visual(
        mesh_from_cadquery(_driver_blade(), "driver_blade"),
        material=metal_mat,
        name="driver_blade",
    )

    # ---- Rear pivot (unchanged articulation) -----------------------------
    # The arm is authored with its pivot at the arm local origin and the cover
    # extending along +X.  At q=0 the arm origin coincides with the pivot
    # frame placed on the base at (HINGE_X, 0, HINGE_Z).  The cover extends
    # +X from the pivot, so axis=-Y makes positive q lift the front nose
    # upward (right-hand rule).
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=3.0, lower=0.0, upper=0.52
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

    # --- Joint contract ---------------------------------------------------
    ctx.check(
        "hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis is transverse (-Y so positive q opens upward)",
        tuple(round(a, 6) for a in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "hinge opens upward from closed (lower=0, upper>0.3)",
        lim is not None and lim.lower == 0.0 and lim.upper is not None and lim.upper > 0.3,
        details=f"limits=({lim.lower if lim else None},{lim.upper if lim else None})",
    )

    # --- Hero parts exist -------------------------------------------------
    base_body = base.get_visual("base_body")
    anvil = base.get_visual("anvil_plate")
    housing = base.get_visual("rear_housing")
    arm_shell = arm.get_visual("arm_shell")
    rail = arm.get_visual("carrier_rail")
    blade = arm.get_visual("driver_blade")
    for label, vis in [
        ("base_body", base_body),
        ("anvil_plate", anvil),
        ("rear_housing", housing),
        ("arm_shell", arm_shell),
        ("carrier_rail", rail),
        ("driver_blade", blade),
    ]:
        ctx.check(f"{label} present", vis is not None, details=f"{label} missing")

    # Repeated foot pads exist (loop-emitted).
    for i in range(len(_FOOT_X_POSITIONS)):
        name = f"foot_pad_{i}"
        ctx.check(
            f"{name} present",
            base.get_visual(name) is not None,
            details=f"{name} missing",
        )

    # --- No exposed knuckles or pin (enclosed-hinge variant contract) -----
    def _has_visual(part, name: str) -> bool:
        try:
            part.get_visual(name)
            return True
        except Exception:
            return False

    ctx.check(
        "no exposed hinge knuckles",
        not _has_visual(base, "hinge_knuckles"),
        details="hinge_knuckles should not exist on the enclosed-hinge variant",
    )
    ctx.check(
        "no exposed hinge pin",
        not _has_visual(arm, "hinge_pin"),
        details="hinge_pin should not exist on the enclosed-hinge variant",
    )

    # --- Housing encloses the pivot ---------------------------------------
    housing_aabb = ctx.part_element_world_aabb(base, elem="rear_housing")
    ctx.check(
        "housing top rises well above the pivot",
        housing_aabb is not None and housing_aabb[1][2] > HINGE_Z + 0.015,
        details=(
            f"housing_top={housing_aabb[1][2]:.4f}" if housing_aabb else "no aabb"
        ),
    )
    ctx.check(
        "housing floor reaches the base foot",
        housing_aabb is not None and housing_aabb[0][2] < 0.005,
        details=(
            f"housing_bottom={housing_aabb[0][2]:.4f}" if housing_aabb else "no aabb"
        ),
    )
    # The pivot sits inside the housing X and Z extents.
    ctx.check(
        "pivot is inside the housing envelope",
        housing_aabb is not None
        and housing_aabb[0][0] <= HINGE_X <= housing_aabb[1][0]
        and housing_aabb[0][2] <= HINGE_Z <= housing_aabb[1][2],
        details=f"housing_x=({housing_aabb[0][0]:.4f},{housing_aabb[1][0]:.4f}) "
        f"housing_z=({housing_aabb[0][2]:.4f},{housing_aabb[1][2]:.4f})"
        if housing_aabb
        else "no aabb",
    )

    # --- Closed pose checks -----------------------------------------------
    with ctx.pose({hinge: 0.0}):
        shell_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
        body_aabb = ctx.part_element_world_aabb(base, elem="base_body")
        ctx.check(
            "cover crown rises well above the base body",
            shell_aabb is not None
            and body_aabb is not None
            and shell_aabb[1][2] > body_aabb[1][2] + 0.025,
            details=f"shell_top={shell_aabb[1][2]:.4f} body_top={body_aabb[1][2]:.4f}",
        )

        ctx.expect_overlap(
            arm, base, axes="xy", min_overlap=0.03,
            name="closed cover covers the base tray footprint",
        )

        anvil_aabb = ctx.part_element_world_aabb(base, elem="anvil_plate")
        ctx.check(
            "anvil plate seated low on the front deck",
            anvil_aabb is not None and 0.008 < anvil_aabb[0][2] < 0.014,
            details=f"anvil_min_z={anvil_aabb[0][2] if anvil_aabb else None}",
        )

        blade_aabb = ctx.part_element_world_aabb(arm, elem="driver_blade")
        ctx.check(
            "driver blade registers above the anvil plate",
            blade_aabb is not None
            and anvil_aabb is not None
            and blade_aabb[0][2] >= anvil_aabb[1][2] - 0.001
            and abs(blade_aabb[0][0] - anvil_aabb[0][0]) < 0.04,
            details=f"blade_min_z={blade_aabb[0][2]:.4f} anvil_max_z={anvil_aabb[1][2]:.4f}",
        )

        # The arm's rear tongue is inside the housing cavity at rest — this is
        # the enclosed-pivot design intent.
        ctx.expect_overlap(
            arm, base, axes="xz", elem_a="arm_shell", elem_b="rear_housing",
            min_overlap=0.004,
            name="arm rear tongue overlaps housing footprint (enclosed pivot)",
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
        details=(
            f"closed_blade_min_z={closed_nose[0][2]:.4f}, "
            f"open_blade_min_z={open_nose[0][2]:.4f}"
        ),
    )

    # --- Overlap allowance: arm shell inside housing cavity ---------------
    # The arm's rear tongue is intentionally represented as pivoting inside the
    # housing cavity.  This is the core design intent of the enclosed-hinge
    # variant: the pivot hardware is hidden within the boxed housing.
    ctx.allow_overlap(
        arm,
        base,
        elem_a="arm_shell",
        elem_b="rear_housing",
        reason=(
            "The arm rear tongue pivots inside the boxed housing cavity; "
            "the pivot mechanism is intentionally enclosed within the housing."
        ),
    )

    return ctx.report()


object_model = build_object_model()
