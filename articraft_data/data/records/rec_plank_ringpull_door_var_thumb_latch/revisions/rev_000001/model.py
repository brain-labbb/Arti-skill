from __future__ import annotations

# Weathered vertical-plank wooden door, shown closed, fitted with a Suffolk
# thumb-latch (vertical iron grip handle with a pivoting thumb lever bar above
# a latch bar), set in an old stone doorway above a stone step. A single leaf
# is hinged on one jamb and swings on a vertical (Z) axis. The stone surround
# and step are fixed.
#
# Frame convention (world up = +Z):
#   - X is the door's facing direction (front of the leaf points to +X).
#   - Y is the door width direction (hinge jamb on the -Y side).
#   - Z is height, built from the ground plane (z = 0) upward.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute real-world dimensions (meters)
# ---------------------------------------------------------------------------
DOOR_W = 0.85          # leaf width (along Y)
DOOR_H = 2.00          # leaf height (along Z)
DOOR_T = 0.050         # leaf thickness (along X)

PLANK_COUNT = 5
PLANK_GAP = 0.004      # narrow grooved gap between planks
PLANK_T = 0.044        # plank board thickness (front face proud of backing)
BACK_T = 0.012         # thin backing board behind planks (carries the planks)

LEDGE_T = 0.022        # iron ledge/brace board thickness (front of planks)
LEDGE_W = 0.075        # ledge board width (along Z for horizontal ledges)

# Stone surround
JAMB_W = 0.16          # stone jamb width (along Y) on each side
JAMB_D = 0.30          # stone jamb depth (along X)
HEADER_H = 0.22        # stone header/lintel height (along Z)
STEP_H = 0.12          # stoop top height the leaf sits on (door bottom clears it)
STEP_RUN = 0.34        # stone step depth (along X, projecting in front)
LOWER_STEP_H = 0.06    # lower descending step in front of the stoop (below door bottom)
WALL_T = 0.28          # overall stone wall thickness (along X)
REVEAL = 0.03          # door set back from the front face of the wall opening

# Suffolk thumb-latch hardware (replaces ring pull)
GRIP_W = 0.040         # grip handle width (Y)
GRIP_H = 0.200         # grip handle height (Z)
GRIP_T = 0.016         # grip handle thickness (X, proud of plank face)
GRIP_CORNER_R = 0.006  # grip handle corner radius

THUMB_L = 0.120        # thumb lever total length (extends outward from door)
THUMB_W = 0.024        # thumb lever width (Z, vertical extent of the bar)
THUMB_T = 0.007        # thumb lever thickness (Y)

PIVOT_PIN_R = 0.006    # pivot pin radius
PIVOT_PIN_LEN = 0.050  # pivot pin length (through the door thickness)

LATCH_BAR_L = 0.160    # latch bar length (horizontal, extends toward free edge)
LATCH_BAR_W = 0.028    # latch bar width (Z, vertical extent)
LATCH_BAR_T = 0.008    # latch bar thickness (X, proud of plank face)

LOCK_W = 0.085         # lock plate width (Y)
LOCK_H = 0.150         # lock plate height (Z)
LOCK_T = 0.008         # lock plate thickness (X, proud of leaf face)

# Thumb-latch mount location in the door_leaf frame (front face, free side, at
# handle height). Shared by the fixed hardware, the lever part, and the joint.
DOOR_LEAF_Z0 = STEP_H + 0.01   # leaf bottom sits just above the stone stoop top
PLATE_FRONT_X = -DOOR_T / 2.0 + BACK_T + PLANK_T  # plank front face X
HW_CY = DOOR_W - 0.20          # handle center across the leaf (toward +Y edge)
HW_CZ = DOOR_LEAF_Z0 + 0.98    # handle center height (~1.0 m)

# Pivot point: at the top of the grip handle area, on the plank face.
# The lever pivots here; the grip handle is below.
PIVOT_Z = HW_CZ + GRIP_H / 2.0 - 0.018  # near top of grip handle
PIVOT_X = PLATE_FRONT_X + GRIP_T         # pivot pin at outer face of grip

# Vertical hinge position: the hinge jamb is on the -Y side.
OPENING_W = DOOR_W + 0.02
OPENING_H = DOOR_LEAF_Z0 + DOOR_H + 0.05  # header clears the raised leaf top

# Half thickness centers used repeatedly.
LEAF_FRONT_X = DOOR_T / 2.0  # +X face of the leaf in its local frame


def _wood(model: ArticulatedObject) -> None:
    model.material("weathered_oak", rgba=(0.470, 0.405, 0.320, 1.0))
    model.material("dark_oak_groove", rgba=(0.300, 0.255, 0.195, 1.0))
    model.material("aged_iron", rgba=(0.205, 0.180, 0.165, 1.0))
    model.material("old_stone", rgba=(0.560, 0.545, 0.520, 1.0))
    model.material("old_stone_dark", rgba=(0.470, 0.460, 0.435, 1.0))
    model.material("worn_stone_step", rgba=(0.500, 0.480, 0.450, 1.0))
    model.material("brass_sign", rgba=(0.235, 0.210, 0.190, 1.0))
    # Darker iron for latch hardware detail
    model.material("dark_iron", rgba=(0.160, 0.145, 0.135, 1.0))


def _build_stone_surround(model: ArticulatedObject) -> None:
    """Fixed stone doorway: two jambs, a header lintel, and a stone step."""
    surround = model.part("stone_surround")

    half_open = OPENING_W / 2.0
    jamb_center_y = half_open + JAMB_W / 2.0
    wall_top = OPENING_H + HEADER_H

    for sign in (-1.0, 1.0):
        side = "pos" if sign > 0 else "neg"
        surround.visual(
            Box((JAMB_D, JAMB_W, wall_top)),
            origin=Origin(xyz=(0.0, sign * jamb_center_y, wall_top / 2.0)),
            material="old_stone",
            name=f"jamb_{side}",
        )

        course_x = JAMB_D / 2.0 - 0.012
        n_courses = 7
        course_h = (wall_top - 0.10) / n_courses
        for c in range(n_courses):
            cz = 0.10 + course_h / 2.0 + c * course_h
            jitter = 0.012 if (c + (0 if sign > 0 else 1)) % 2 == 0 else -0.010
            blk_h = course_h - 0.012
            blk_w = JAMB_W - 0.030 + jitter
            mat = "old_stone_dark" if (c + (1 if sign > 0 else 0)) % 2 == 0 else "old_stone"
            surround.visual(
                Box((0.030, blk_w, blk_h)),
                origin=Origin(xyz=(course_x, sign * jamb_center_y, cz)),
                material=mat,
                name=f"course_{side}_{c}",
            )

    header_span_y = OPENING_W + 2.0 * JAMB_W
    surround.visual(
        Box((JAMB_D, header_span_y, HEADER_H)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_H + HEADER_H / 2.0)),
        material="old_stone",
        name="header_lintel",
    )

    head_course_x = JAMB_D / 2.0 - 0.012
    for k, dy in enumerate((-0.30, 0.0, 0.30)):
        mat = "old_stone_dark" if k % 2 == 1 else "old_stone"
        surround.visual(
            Box((0.030, 0.26, HEADER_H - 0.04)),
            origin=Origin(
                xyz=(head_course_x, dy, OPENING_H + HEADER_H / 2.0)
            ),
            material=mat,
            name=f"head_course_{k}",
        )

    arch_r = header_span_y / 2.0
    arch = (
        cq.Workplane("XY")
        .moveTo(-arch_r, 0.0)
        .threePointArc((0.0, arch_r), (arch_r, 0.0))
        .lineTo(-arch_r, 0.0)
        .close()
        .extrude(0.060)
    )
    surround.visual(
        mesh_from_cadquery(arch, "arched_head"),
        origin=Origin(
            xyz=(JAMB_D / 2.0 - 0.030, 0.0, OPENING_H + HEADER_H),
            rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
        ),
        material="old_stone",
        name="arched_head",
    )

    keystone = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.060, -0.105),
                (0.060, -0.105),
                (0.085, 0.105),
                (-0.085, 0.105),
            ]
        )
        .close()
        .extrude(0.045)
    )
    surround.visual(
        mesh_from_cadquery(keystone, "keystone"),
        origin=Origin(
            xyz=(JAMB_D / 2.0 - 0.040, 0.0, OPENING_H + HEADER_H + 0.11),
        ),
        material="old_stone_dark",
        name="keystone_block",
    )

    step_span_y = OPENING_W + 2.0 * JAMB_W

    landing_run = STEP_RUN * 0.60
    surround.visual(
        Box((JAMB_D + landing_run, step_span_y, STEP_H)),
        origin=Origin(xyz=(landing_run / 2.0, 0.0, STEP_H / 2.0)),
        material="worn_stone_step",
        name="stone_step",
    )

    lower_front_x = JAMB_D / 2.0 + landing_run
    surround.visual(
        Box((STEP_RUN, step_span_y - 0.06, LOWER_STEP_H)),
        origin=Origin(
            xyz=(lower_front_x + STEP_RUN / 2.0, 0.0, LOWER_STEP_H / 2.0),
        ),
        material="worn_stone_step",
        name="stone_step_lower",
    )

    # Iron pintle pins driven into the hinge jamb.
    jamb_inner = -half_open
    pin_len = 0.034
    pin_y = jamb_inner - 0.020
    z0_leaf = DOOR_LEAF_Z0
    leaf_h = DOOR_H
    barrel_local_x = -DOOR_T / 2.0 + BACK_T + PLANK_T + LEDGE_T / 2.0
    pin_x = WALL_T / 2.0 - REVEAL - DOOR_T + barrel_local_x
    for tag, cz in (("bottom", z0_leaf + 0.20), ("top", z0_leaf + leaf_h - 0.20)):
        surround.visual(
            Cylinder(radius=0.008, length=pin_len),
            origin=Origin(xyz=(pin_x, pin_y, cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="aged_iron",
            name=f"jamb_pintle_{tag}",
        )


def _build_door_leaf(model: ArticulatedObject) -> None:
    """Vertical-plank wooden leaf with iron ledges, Suffolk thumb-latch fixed
    hardware (grip handle, pivot pin, latch bar), and lock plate.

    The part frame sits on the vertical hinge line (the -Y jamb edge) at the
    door bottom (z = 0). Local +Y runs across the leaf toward the free (latch)
    edge.
    """
    leaf = model.part("door_leaf")

    z0 = DOOR_LEAF_Z0
    leaf_h = DOOR_H
    back_cx = DOOR_W / 2.0
    back_cz = z0 + leaf_h / 2.0
    back_cx_x = -DOOR_T / 2.0 + BACK_T / 2.0
    leaf.visual(
        Box((BACK_T, DOOR_W, leaf_h)),
        origin=Origin(xyz=(back_cx_x, back_cx, back_cz)),
        material="dark_oak_groove",
        name="backing_board",
    )

    # Vertical planks across the leaf width.
    usable_w = DOOR_W - (PLANK_COUNT - 1) * PLANK_GAP
    plank_w = usable_w / PLANK_COUNT
    plank_cx_x = -DOOR_T / 2.0 + BACK_T + PLANK_T / 2.0
    for i in range(PLANK_COUNT):
        cy = plank_w / 2.0 + i * (plank_w + PLANK_GAP)
        leaf.visual(
            Box((PLANK_T, plank_w, leaf_h)),
            origin=Origin(xyz=(plank_cx_x, cy, back_cz)),
            material="weathered_oak",
            name=f"plank_{i}",
        )

    # Iron ledge/strap braces with hinge barrels.
    ledge_cx_x = -DOOR_T / 2.0 + BACK_T + PLANK_T + LEDGE_T / 2.0
    barrel_r = 0.018
    barrel_len = 0.10
    strap_y0 = 0.05
    strap_w = DOOR_W - strap_y0
    strap_cy = strap_y0 + strap_w / 2.0
    for tag, cz in (("bottom", z0 + 0.20), ("top", z0 + leaf_h - 0.20)):
        leaf.visual(
            Box((LEDGE_T, strap_w, LEDGE_W)),
            origin=Origin(xyz=(ledge_cx_x, strap_cy, cz)),
            material="aged_iron",
            name=f"ledge_{tag}",
        )
        leaf.visual(
            Cylinder(radius=barrel_r, length=barrel_len),
            origin=Origin(xyz=(ledge_cx_x, -0.006, cz)),
            material="aged_iron",
            name=f"hinge_barrel_{tag}",
        )
        stub_w = 0.075
        leaf.visual(
            Box((LEDGE_T, stub_w, LEDGE_W * 0.6)),
            origin=Origin(xyz=(ledge_cx_x, -0.005 + stub_w / 2.0, cz)),
            material="aged_iron",
            name=f"hinge_stub_{tag}",
        )

    # Plank front surface X.
    plate_x = -DOOR_T / 2.0 + BACK_T + PLANK_T

    # Small brass/iron sign plate near the top of the leaf.
    sign_t = 0.006
    sign_cz = z0 + leaf_h - 0.42
    leaf.visual(
        Box((sign_t, 0.16, 0.10)),
        origin=Origin(xyz=(plate_x + sign_t / 2.0, back_cx, sign_cz)),
        material="brass_sign",
        name="sign_plate",
    )

    # ---- Suffolk thumb-latch fixed hardware on the plank face ----

    # Vertical grip handle: a rounded-rectangle iron bar mounted proud of the
    # plank face. Built with CadQuery for proper rounded corners that read as
    # a forged iron handle grip.
    hw_cy = HW_CY
    hw_cz = HW_CZ

    grip_profile = (
        cq.Workplane("YZ")
        .rect(GRIP_W, GRIP_H)
        .extrude(GRIP_T)
    )
    # Fillet the vertical edges (along extrusion axis X) to round the grip.
    grip_profile = grip_profile.edges("|X").fillet(GRIP_CORNER_R)
    leaf.visual(
        mesh_from_cadquery(grip_profile, "grip_handle"),
        origin=Origin(xyz=(plate_x, hw_cy, hw_cz)),
        material="aged_iron",
        name="grip_handle",
    )

    # Mounting studs (two bolts through the grip into the plank). Visible as
    # small iron discs on the grip face.
    for dz_tag, dz in (("upper", GRIP_H / 2.0 - 0.025), ("lower", -(GRIP_H / 2.0 - 0.025))):
        leaf.visual(
            Cylinder(radius=0.005, length=0.005),
            origin=Origin(
                xyz=(plate_x + GRIP_T + 0.0025, hw_cy, hw_cz + dz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="dark_iron",
            name=f"grip_stud_{dz_tag}",
        )

    # Pivot pin: a horizontal iron pin going through the door at the pivot
    # height. This is the pin the thumb lever rocks on. Cylinder long axis is
    # +Z by default; rotate so it points along X (through the door).
    leaf.visual(
        Cylinder(radius=PIVOT_PIN_R, length=PIVOT_PIN_LEN),
        origin=Origin(
            xyz=(plate_x + GRIP_T - PIVOT_PIN_LEN / 2.0 + 0.010, hw_cy, PIVOT_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="dark_iron",
        name="pivot_pin",
    )

    # Latch bar: a horizontal iron bar mounted on the plank face, extending from
    # the handle area toward the free (+Y) edge. This is the bar that catches
    # the door frame keep. Positioned at the pivot height (or slightly below).
    latch_cz = PIVOT_Z - THUMB_W / 2.0 - 0.008  # just below the lever
    latch_y0 = hw_cy + GRIP_W / 2.0 + 0.010     # starts just past the grip
    latch_cy = latch_y0 + LATCH_BAR_L / 2.0
    leaf.visual(
        Box((LATCH_BAR_T, LATCH_BAR_L, LATCH_BAR_W)),
        origin=Origin(xyz=(plate_x + LATCH_BAR_T / 2.0, latch_cy, latch_cz)),
        material="aged_iron",
        name="latch_bar",
    )

    # Latch bar guide bracket: a small iron loop/staple holding the bar against
    # the door face so it doesn't float.
    bracket_y = latch_y0 + LATCH_BAR_L * 0.6
    leaf.visual(
        Box((0.006, 0.020, LATCH_BAR_W + 0.012)),
        origin=Origin(
            xyz=(plate_x + LATCH_BAR_T + 0.003, bracket_y, latch_cz),
        ),
        material="dark_iron",
        name="latch_bracket",
    )

    # ---- Iron lock plate (below the latch mechanism) ----
    lock_cz = z0 + 0.86
    leaf.visual(
        Box((LOCK_T, LOCK_W, LOCK_H)),
        origin=Origin(xyz=(plate_x + LOCK_T / 2.0, hw_cy + 0.13, lock_cz)),
        material="aged_iron",
        name="lock_plate",
    )
    leaf.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(
            xyz=(plate_x + LOCK_T + 0.010, hw_cy + 0.13, lock_cz - 0.02),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="aged_iron",
        name="lock_knob",
    )


def _build_thumb_lever(model: ArticulatedObject) -> None:
    """The pivoting thumb lever bar as its own rotatable part.

    The part's local frame origin coincides with the revolute joint origin at
    the pivot pin center on the door face. The lever extends outward (+X from
    door face) at rest. Pressing the outer end down (-Z) actuates the latch.
    """
    lever_part = model.part("thumb_lever")

    # Build the lever bar: a flat iron bar extending outward from the pivot.
    # At rest (q=0) it extends in +X from the origin. The bar is thin in Y and
    # taller in Z (like a paddle you press with your thumb).
    # Use CadQuery for a proper rounded-end bar shape.
    lever = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(THUMB_L - THUMB_W / 2.0, 0.0)
        .threePointArc(
            (THUMB_L, THUMB_W / 2.0), (THUMB_L - THUMB_W / 2.0, THUMB_W)
        )
        .lineTo(0.0, THUMB_W)
        .threePointArc(
            (-THUMB_W / 2.0, THUMB_W / 2.0), (0.0, 0.0)
        )
        .close()
        .extrude(THUMB_T)
    )
    # The lever is built in XZ plane extruded along Y. Center it on the pivot:
    # shift so the pivot end is near origin and the bar extends in +X. The Z
    # extent goes from 0 to THUMB_W; shift down so it's centered on z=0 at rest.
    lever_part.visual(
        mesh_from_cadquery(lever, "lever_bar"),
        origin=Origin(
            xyz=(0.0, -THUMB_T / 2.0, -THUMB_W / 2.0),
        ),
        material="aged_iron",
        name="lever_bar",
    )

    # Pivot boss: a small iron disc/collar wrapping the pivot pin at the lever's
    # inner end. This makes the pivot connection visible.
    lever_part.visual(
        Cylinder(radius=PIVOT_PIN_R + 0.004, length=THUMB_T + 0.004),
        origin=Origin(
            xyz=(0.0, -(THUMB_T + 0.004) / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="dark_iron",
        name="lever_boss",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plank_door_suffolk_latch")
    _wood(model)

    _build_stone_surround(model)
    _build_door_leaf(model)
    _build_thumb_lever(model)

    # Single revolute leaf on a VERTICAL (Z) hinge axis at the -Y jamb.
    half_open = OPENING_W / 2.0
    hinge_y = -half_open + 0.01
    hinge_x = WALL_T / 2.0 - REVEAL - DOOR_T

    model.articulation(
        "surround_to_leaf",
        ArticulationType.REVOLUTE,
        parent="stone_surround",
        child="door_leaf",
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    # Thumb lever rocks on a horizontal Y-axis at the pivot pin.
    # The lever extends outward (+X from door face). With axis (0, 1, 0),
    # positive q rotates +X toward -Z (outer end presses down), which is the
    # correct thumb-press motion. Rest is q=0 (horizontal).
    model.articulation(
        "leaf_to_thumb_lever",
        ArticulationType.REVOLUTE,
        parent="door_leaf",
        child="thumb_lever",
        origin=Origin(xyz=(PIVOT_X, HW_CY, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=0.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    surround = object_model.get_part("stone_surround")
    leaf = object_model.get_part("door_leaf")
    lever_part = object_model.get_part("thumb_lever")
    hinge = object_model.get_articulation("surround_to_leaf")
    lever_joint = object_model.get_articulation("leaf_to_thumb_lever")

    # --- Articulation contract: vertical hinge, realistic swing range ---
    ax = hinge.axis
    ctx.check(
        "hinge axis is vertical (Z)",
        abs(ax[2]) > 0.99 and abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6,
        details=f"axis={ax}",
    )
    lims = hinge.motion_limits
    ctx.check(
        "hinge has a realistic opening range",
        lims is not None
        and lims.lower is not None
        and lims.upper is not None
        and abs(lims.lower) < 1e-6
        and 1.2 <= lims.upper <= 3.2,
        details=f"limits=({lims.lower if lims else None},{lims.upper if lims else None})",
    )

    # --- Hero geometry present and legible ---
    lever_bar = lever_part.get_visual("lever_bar")
    grip = leaf.get_visual("grip_handle")
    latch = leaf.get_visual("latch_bar")
    lock = leaf.get_visual("lock_plate")
    ctx.check(
        "leaf has all five vertical planks",
        all(leaf.get_visual(f"plank_{i}") is not None for i in range(PLANK_COUNT)),
        details="planks 0..4 must exist",
    )
    ctx.check(
        "Suffolk latch hardware present: grip handle, thumb lever, latch bar, lock plate",
        lever_bar is not None
        and grip is not None
        and latch is not None
        and lock is not None,
        details="grip_handle, lever_bar, latch_bar, and lock_plate are hero features",
    )

    # --- Thumb lever joint axis is horizontal and parallel to door face (Y) ---
    lax = lever_joint.axis
    ctx.check(
        "thumb lever joint axis is horizontal and parallel to door face (Y)",
        abs(lax[1]) > 0.99 and abs(lax[0]) < 1e-6 and abs(lax[2]) < 1e-6,
        details=f"lever axis={lax}",
    )
    llims = lever_joint.motion_limits
    ctx.check(
        "thumb lever has a sensible rock range",
        llims is not None
        and llims.lower is not None
        and llims.upper is not None
        and abs(llims.lower) < 1e-6
        and 0.3 <= llims.upper <= 1.2,
        details=f"lever limits=({llims.lower if llims else None},{llims.upper if llims else None})",
    )

    # --- Standing on the ground, not sinking/floating ---
    surround_aabb = ctx.part_world_aabb(surround)
    leaf_aabb = ctx.part_world_aabb(leaf)
    ctx.check(
        "stone surround sits on the ground plane",
        surround_aabb is not None and abs(surround_aabb[0][2]) < 1e-6,
        details=f"surround z-min={surround_aabb[0][2] if surround_aabb else None}",
    )
    ctx.check(
        "leaf is upright and tall (~2 m), above the ground",
        leaf_aabb is not None
        and leaf_aabb[0][2] > 0.0
        and (leaf_aabb[1][2] - leaf_aabb[0][2]) > 1.8,
        details=f"leaf z=({leaf_aabb[0][2] if leaf_aabb else None},{leaf_aabb[1][2] if leaf_aabb else None})",
    )

    # --- Outward swing clears the stone stoop ---
    step_aabb = ctx.part_element_world_aabb(surround, elem="stone_step")
    ctx.check(
        "door bottom clears the stone stoop so it never clips the step when swung",
        leaf_aabb is not None
        and step_aabb is not None
        and leaf_aabb[0][2] > step_aabb[1][2],
        details=f"leaf z-min={leaf_aabb[0][2] if leaf_aabb else None}, "
        f"stoop z-max={step_aabb[1][2] if step_aabb else None}",
    )

    # --- Closed leaf seats within the stone opening ---
    ctx.expect_within(
        leaf,
        surround,
        axes="y",
        inner_elem="backing_board",
        margin=0.02,
        name="closed leaf body fits between the jambs",
    )

    # --- Hinge connection: barrels wrap the jamb pintles (captured pins) ---
    ctx.allow_overlap(
        leaf,
        surround,
        elem_a="hinge_barrel_bottom",
        elem_b="jamb_pintle_bottom",
        reason="Bottom hinge knuckle wraps the iron jamb pintle (captured pin).",
    )
    ctx.allow_overlap(
        leaf,
        surround,
        elem_a="hinge_barrel_top",
        elem_b="jamb_pintle_top",
        reason="Top hinge knuckle wraps the iron jamb pintle (captured pin).",
    )
    ctx.allow_overlap(
        leaf,
        surround,
        elem_a="hinge_barrel_bottom",
        elem_b="jamb_neg",
        reason="Bottom hinge knuckle is seated against/into the hinge jamb stone.",
    )
    ctx.allow_overlap(
        leaf,
        surround,
        elem_a="hinge_barrel_top",
        elem_b="jamb_neg",
        reason="Top hinge knuckle is seated against/into the hinge jamb stone.",
    )

    # --- Thumb lever captured on its pivot pin: the lever boss wraps the pin ---
    ctx.allow_overlap(
        lever_part,
        leaf,
        elem_a="lever_boss",
        elem_b="pivot_pin",
        reason="Thumb lever boss wraps the pivot pin (captured pin); small overlap "
        "keeps the lever mounted on its pivot.",
    )

    # --- Lever boss sits at top of grip handle where the pivot is mounted ---
    ctx.allow_overlap(
        lever_part,
        leaf,
        elem_a="lever_boss",
        elem_b="grip_handle",
        reason="The pivot boss is seated at the top of the grip handle where the "
        "pivot pin passes through; small local overlap represents the pivot mount.",
    )

    # --- Lever bar inner end nests at the top of the grip handle at the pivot ---
    ctx.allow_overlap(
        lever_part,
        leaf,
        elem_a="lever_bar",
        elem_b="grip_handle",
        reason="The lever bar originates from the pivot at the top of the grip handle; "
        "the inner end of the bar is seated against the grip top (Suffolk latch design).",
    )

    # Prove the captured-pin hinge connections contact.
    ctx.expect_contact(
        leaf,
        surround,
        elem_a="hinge_barrel_bottom",
        elem_b="jamb_pintle_bottom",
        name="bottom hinge barrel contacts its jamb pintle",
    )
    ctx.expect_contact(
        leaf,
        surround,
        elem_a="hinge_barrel_top",
        elem_b="jamb_pintle_top",
        name="top hinge barrel contacts its jamb pintle",
    )

    # --- Thumb lever stands proud of the plank face ---
    lever_aabb = ctx.part_element_world_aabb(lever_part, elem="lever_bar")
    plank_aabb = ctx.part_element_world_aabb(leaf, elem="plank_0")
    ctx.check(
        "thumb lever stands proud of the plank face",
        lever_aabb is not None
        and plank_aabb is not None
        and lever_aabb[1][0] > plank_aabb[1][0],
        details=f"lever xmax={lever_aabb[1][0] if lever_aabb else None}, "
        f"plank xmax={plank_aabb[1][0] if plank_aabb else None}",
    )

    # Prove the lever bar is seated at/against the grip handle top.
    ctx.expect_contact(
        lever_part,
        leaf,
        elem_a="lever_bar",
        elem_b="grip_handle",
        name="lever bar inner end contacts grip handle at pivot",
    )

    # --- Grip handle is above the latch bar (thumb lever is above latch) ---
    grip_aabb = ctx.part_element_world_aabb(leaf, elem="grip_handle")
    latch_aabb = ctx.part_element_world_aabb(leaf, elem="latch_bar")
    ctx.check(
        "thumb lever pivot is above the latch bar",
        grip_aabb is not None
        and latch_aabb is not None
        and grip_aabb[1][2] > latch_aabb[0][2],
        details=f"grip zmax={grip_aabb[1][2] if grip_aabb else None}, "
        f"latch zmin={latch_aabb[0][2] if latch_aabb else None}",
    )

    # --- Posing the thumb lever rocks it: outer end moves downward ---
    closed_lever = ctx.part_element_world_aabb(lever_part, elem="lever_bar")
    with ctx.pose({lever_joint: 0.5}):
        pressed_lever = ctx.part_element_world_aabb(lever_part, elem="lever_bar")
        ctx.check(
            "pressing the thumb lever moves the outer end downward (-Z)",
            closed_lever is not None
            and pressed_lever is not None
            # The outer end (xmax region) should drop in Z.
            and pressed_lever[0][2] < closed_lever[0][2] - 0.005,
            details=f"closed zmin={closed_lever[0][2] if closed_lever else None}, "
            f"pressed zmin={pressed_lever[0][2] if pressed_lever else None}",
        )

    # --- Decisive open pose: free edge actually swings forward (+X) ---
    closed_free = ctx.part_element_world_aabb(leaf, elem=f"plank_{PLANK_COUNT - 1}")
    with ctx.pose({hinge: 1.4}):
        open_free = ctx.part_element_world_aabb(leaf, elem=f"plank_{PLANK_COUNT - 1}")
        ctx.check(
            "open pose swings the free edge forward (+X)",
            closed_free is not None
            and open_free is not None
            and open_free[1][0] > closed_free[1][0] + 0.3,
            details=f"closed free xmax={closed_free[1][0] if closed_free else None}, "
            f"open free xmax={open_free[1][0] if open_free else None}",
        )

    return ctx.report()


object_model = build_object_model()
