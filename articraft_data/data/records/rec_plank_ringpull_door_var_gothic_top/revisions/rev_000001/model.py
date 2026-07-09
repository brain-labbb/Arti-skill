from __future__ import annotations

# Weathered vertical-plank wooden door with a pointed lancet gothic arch top,
# shown closed, with a round iron ring-pull handle and an iron lock plate, set
# in an old stone doorway above a stone step. The stone surround and step are
# the fixed root on the ground; a single leaf is hinged on one jamb and swings
# on a vertical (Z) axis.
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
DOOR_H = 2.00          # leaf height to arch peak (along Z)
DOOR_T = 0.050         # leaf thickness (along X)

PLANK_COUNT = 5
PLANK_GAP = 0.004      # narrow grooved gap between planks
PLANK_T = 0.044        # plank board thickness (front face proud of backing)
BACK_T = 0.012         # thin backing board behind planks (carries the planks)

LEDGE_T = 0.022        # iron ledge/brace board thickness (front of planks)
LEDGE_W = 0.075        # ledge board width (along Z for horizontal ledges)

# --- Lancet gothic arch parameters ---
ARCH_RISE = 0.40       # arch rise above the spring line at the center peak
# The spring line is the height where the rectangular body ends and the arch
# curve begins. At the door edges (y=0, y=DOOR_W) the planks are at the spring
# line height; at center (y=DOOR_W/2) they peak at DOOR_H.

# Stone surround
JAMB_W = 0.16          # stone jamb width (along Y) on each side
JAMB_D = 0.30          # stone jamb depth (along X)
HEADER_H = 0.22        # stone header/lintel height (along Z)
STEP_H = 0.12          # stoop top height the leaf sits on (door bottom clears it)
STEP_RUN = 0.34        # stone step depth (along X, projecting in front)
LOWER_STEP_H = 0.06    # lower descending step in front of the stoop (below door bottom)
WALL_T = 0.28          # overall stone wall thickness (along X)
REVEAL = 0.03          # door set back from the front face of the wall opening

# Ring-pull / lock hardware
RING_OUT_R = 0.058     # outer radius of the iron ring
RING_TUBE_R = 0.0095   # tube (cross-section) radius of the ring
# Square iron backplate seated flush on the door face, with a short central
# boss/stud projecting forward that the ring hangs from and pivots on.
PLATE_SIZE = 0.095     # square backplate edge length (Y and Z)
PLATE_T = 0.007        # backplate thickness (X, proud of plank face)
BOSS_R = 0.011         # forward boss/stud radius
BOSS_LEN = 0.022       # forward boss/stud length (along X)
LOCK_W = 0.085         # lock plate width (Y)
LOCK_H = 0.150         # lock plate height (Z)
LOCK_T = 0.008         # lock plate thickness (X, proud of leaf face)

# Ring-pull mount location in the door_leaf frame (front face, free side, at
# handle height). Shared by the fixed hardware, the ring part, and the joint.
DOOR_LEAF_Z0 = STEP_H + 0.01   # leaf bottom sits just above the stone stoop top
PLATE_FRONT_X = -DOOR_T / 2.0 + BACK_T + PLANK_T  # plank front face X
HW_CY = DOOR_W - 0.20          # ring-pull center across the leaf (toward +Y edge)
HW_CZ = DOOR_LEAF_Z0 + 0.98    # ring-pull center height (~1.0 m)
# The boss sits a little above the plate center so the ring hangs below it.
BOSS_Z_OFF = PLATE_SIZE / 2.0 - 0.020
# Boss pivot point (boss axis line) in the door_leaf frame: X at the boss
# center, Y at the hardware center, Z at the boss height. The ring hangs from
# and pivots about this line (axis along Y).
BOSS_PIVOT_X = PLATE_FRONT_X + PLATE_T + BOSS_LEN / 2.0
BOSS_PIVOT_Z = HW_CZ + BOSS_Z_OFF

# Vertical hinge position: the hinge jamb is on the -Y side. The leaf part
# frame sits on the hinge line at the door bottom, with the leaf extending
# toward +Y. The opening clear width equals DOOR_W plus small clearances.
OPENING_W = DOOR_W + 0.02
OPENING_H = DOOR_LEAF_Z0 + DOOR_H + 0.05  # header clears the arch peak

# Half thickness centers used repeatedly.
LEAF_FRONT_X = DOOR_T / 2.0  # +X face of the leaf in its local frame

# Spring line: height (above leaf origin / z0) where the rectangular body ends
# and the lancet arch curve begins.
SPRING_LOCAL = DOOR_H - ARCH_RISE


# ---------------------------------------------------------------------------
# Lancet arch geometry helpers
# ---------------------------------------------------------------------------

def _lancet_arch_z(y: float, W: float, spring_z: float, rise: float) -> float:
    """Return the Z height of the lancet arch at position y along the span.

    The lancet arch is formed by two circular arcs meeting at a point at the
    center (y = W/2). Each arc has its center on the spring line, offset from
    the nearest spring point by R (the arc radius).
    """
    if rise <= 1e-9 or W <= 1e-9:
        return spring_z
    # Arc center offset from the left spring point (for left arc).
    # Derived from: center at (c, spring_z), passing through (0, spring_z) and
    # (W/2, spring_z + rise). R = c, and c = W/4 + rise²/W.
    c = W / 4.0 + rise * rise / W
    R = c
    half = W / 2.0
    if y <= half:
        dy = y - c
    else:
        # Right arc center at (W - c, spring_z) by symmetry.
        dy = y - (W - c)
    val = R * R - dy * dy
    if val <= 0.0:
        return spring_z
    return spring_z + math.sqrt(val)


def _make_arch_plank_solid(
    y_left: float,
    y_right: float,
    z_bot: float,
    W: float,
    spring_z: float,
    rise: float,
    thickness: float,
) -> cq.Workplane:
    """Build an arch-topped plank solid via CadQuery.

    The profile is drawn in the YZ plane (CadQuery "YZ" workplane) with Y and
    Z in the leaf frame, then extruded in +X by ``thickness``. The solid must
    then be translated to the correct X position via the visual origin.
    
    The lancet arch is formed by TWO separate circular arcs meeting at a sharp
    point at the center. We sample the arch curve at multiple points along the
    plank span to create a smooth profile.
    """
    z_left = _lancet_arch_z(y_left, W, spring_z, rise)

    wp = cq.Workplane("YZ")
    # Start at bottom-left corner.
    wp = wp.moveTo(y_left, z_bot)
    # Left edge up to the arch start.
    wp = wp.lineTo(y_left, z_left)

    # Sample the arch curve along the plank span with multiple points for a
    # smooth profile. Use 8-10 sample points for good curvature.
    n_samples = 10
    for k in range(1, n_samples + 1):
        y = y_left + (y_right - y_left) * k / n_samples
        z = _lancet_arch_z(y, W, spring_z, rise)
        wp = wp.lineTo(y, z)

    # Right edge down to bottom.
    wp = wp.lineTo(y_right, z_bot)
    # Close the rectangular bottom.
    wp = wp.close()

    return wp.extrude(thickness)


def _make_arch_board_solid(
    y_span: float,
    z_bot: float,
    W: float,
    spring_z: float,
    rise: float,
    thickness: float,
    n_segments: int = 16,
) -> cq.Workplane:
    """Build a full-width arch-topped board (backing board) via CadQuery.

    Uses multiple arc segments across the full span for a smooth profile.
    Extruded in +X by ``thickness`` from the YZ plane at X=0.
    """
    half = W / 2.0
    wp = cq.Workplane("YZ")
    # Bottom-left.
    wp = wp.moveTo(0.0, z_bot)
    # Up to arch start at left edge.
    z_start = _lancet_arch_z(0.0, W, spring_z, rise)
    wp = wp.lineTo(0.0, z_start)

    # Arch curve across full span, broken at the center for the two arcs.
    # Left arc: from y=0 to y=half.
    seg_half = n_segments // 2
    for k in range(1, seg_half + 1):
        y = half * k / seg_half
        z = _lancet_arch_z(y, W, spring_z, rise)
        wp = wp.lineTo(y, z)
    # Right arc: from y=half to y=W.
    for k in range(1, seg_half + 1):
        y = half + half * k / seg_half
        z = _lancet_arch_z(y, W, spring_z, rise)
        wp = wp.lineTo(y, z)

    # Down to bottom-right.
    wp = wp.lineTo(W, z_bot)
    wp = wp.close()

    return wp.extrude(thickness)


def _wood(model: ArticulatedObject) -> None:
    # Reference reads as light, silvery weathered grey-brown plank wood (aged,
    # sun-bleached). Earlier near-black tone was far too dark.
    model.material("weathered_oak", rgba=(0.470, 0.405, 0.320, 1.0))
    model.material("dark_oak_groove", rgba=(0.300, 0.255, 0.195, 1.0))
    # Aged wrought iron: dark warm grey, not pure black, with a faint rust cast.
    model.material("aged_iron", rgba=(0.205, 0.180, 0.165, 1.0))
    # Weathered warm grey stone doorway surround. The photo reads as a cool-warm
    # grey carved stone, dirtier and greyer than pale cream limestone.
    model.material("old_stone", rgba=(0.560, 0.545, 0.520, 1.0))
    # Slightly darker, mossier blocks to break up the coursing and read as
    # individual weathered stones.
    model.material("old_stone_dark", rgba=(0.470, 0.460, 0.435, 1.0))
    # Worn, scuffed stone step/threshold, a touch warmer and darker (foot traffic).
    model.material("worn_stone_step", rgba=(0.500, 0.480, 0.450, 1.0))
    # Worn iron latch/lock plate (matches the muted rectangular plate in photo),
    # no bright brass plaque in the reference.
    model.material("brass_sign", rgba=(0.235, 0.210, 0.190, 1.0))


def _build_stone_surround(model: ArticulatedObject) -> None:
    """Fixed stone doorway: two jambs, a header lintel, and a stone step."""
    surround = model.part("stone_surround")

    half_open = OPENING_W / 2.0
    jamb_center_y = half_open + JAMB_W / 2.0
    # The wall block runs from z=0 to the top of the header.
    wall_top = OPENING_H + HEADER_H

    # Two side jambs (left and right of the opening). Built from z=0 upward.
    # Each jamb is the structural block; rough block coursing is laid proud of
    # its front face below to read as individual weathered stones.
    for sign in (-1.0, 1.0):
        side = "pos" if sign > 0 else "neg"
        surround.visual(
            Box((JAMB_D, JAMB_W, wall_top)),
            origin=Origin(xyz=(0.0, sign * jamb_center_y, wall_top / 2.0)),
            material="old_stone",
            name=f"jamb_{side}",
        )

        # Rough irregular block coursing proud of each jamb's front face. The
        # blocks are slightly offset in height/width on each side to break the
        # regular grid and read as hand-cut weathered stone.
        course_x = JAMB_D / 2.0 - 0.012  # block centers, proud of jamb front
        n_courses = 7
        course_h = (wall_top - 0.10) / n_courses
        for c in range(n_courses):
            cz = 0.10 + course_h / 2.0 + c * course_h
            # Alternate the block heights and a slight inset to avoid a clean grid.
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

    # Header / lintel spanning the opening above the door.
    header_span_y = OPENING_W + 2.0 * JAMB_W
    surround.visual(
        Box((JAMB_D, header_span_y, HEADER_H)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_H + HEADER_H / 2.0)),
        material="old_stone",
        name="header_lintel",
    )

    # Coursing band proud of the header front, three rough voussoir-like blocks.
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

    # A rounded/arched stone head on top of the lintel, echoing the reference's
    # softly rounded top. Built as a half-disc (semicircle) extruded in X, set
    # proud of the header front, with a flat keystone wedge at its apex.
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
        # Extruded along +Z; rotate so the disc lies in the Y-Z plane facing +X
        # and the curve arcs upward. Seat its flat base on the lintel top.
        origin=Origin(
            xyz=(JAMB_D / 2.0 - 0.030, 0.0, OPENING_H + HEADER_H),
            rpy=(math.pi / 2.0, 0.0, math.pi / 2.0),
        ),
        material="old_stone",
        name="arched_head",
    )

    # Flat trapezoidal keystone wedge seated proud of the arch front face,
    # centered low on the arch (just above the lintel) like the reference.
    # Built in the Y-Z plane (wide in Y, tall in Z) and extruded thin along +X
    # so it lies flat against the arch face rather than spiking outward.
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

    # ---- Stone stoop / threshold at the base ----
    step_span_y = OPENING_W + 2.0 * JAMB_W

    # Upper landing: the full-height stoop top, running from under the door
    # (covering the threshold) and projecting forward.
    landing_run = STEP_RUN * 0.60
    surround.visual(
        Box((JAMB_D + landing_run, step_span_y, STEP_H)),
        origin=Origin(xyz=(landing_run / 2.0, 0.0, STEP_H / 2.0)),
        material="worn_stone_step",
        name="stone_step",
    )

    # Lower step: a worn slab descending in front of the landing. Its top sits
    # well below the door bottom, clear of the leaf's swing arc.
    lower_front_x = JAMB_D / 2.0 + landing_run
    surround.visual(
        Box((STEP_RUN, step_span_y - 0.06, LOWER_STEP_H)),
        origin=Origin(
            xyz=(lower_front_x + STEP_RUN / 2.0, 0.0, LOWER_STEP_H / 2.0),
        ),
        material="worn_stone_step",
        name="stone_step_lower",
    )

    # Iron pintle pins driven into the hinge jamb. Each pin is a HORIZONTAL iron
    # post seated deep in the jamb stone and projecting toward +Y into the door's
    # hinge barrel, so the barrel wraps the pin (captured pin) and the leaf is
    # carried by the jamb.
    jamb_inner = -half_open  # jamb inner face in the parent frame
    pin_len = 0.034            # long axis Y
    pin_y = jamb_inner - 0.020  # center: spans into stone and into barrel back
    # Pin X matches the leaf hinge-barrel X in world (barrels sit at the plank
    # face). We expose the pins at the same height bands as the barrels.
    z0_leaf = DOOR_LEAF_Z0
    leaf_h = DOOR_H
    barrel_local_x = -DOOR_T / 2.0 + BACK_T + PLANK_T + LEDGE_T / 2.0
    pin_x = WALL_T / 2.0 - REVEAL - DOOR_T + barrel_local_x
    # Top barrel is now below the spring line; bottom stays the same.
    top_barrel_local = SPRING_LOCAL - LEDGE_W / 2.0 - 0.03
    for tag, cz in (
        ("bottom", z0_leaf + 0.20),
        ("top", z0_leaf + top_barrel_local),
    ):
        surround.visual(
            Cylinder(radius=0.008, length=pin_len),
            origin=Origin(xyz=(pin_x, pin_y, cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="aged_iron",
            name=f"jamb_pintle_{tag}",
        )


def _build_door_leaf(model: ArticulatedObject) -> None:
    """Vertical-plank wooden leaf with lancet gothic arch top, iron ledgers,
    ring pull, and lock plate.

    The part frame sits on the vertical hinge line (the -Y jamb edge) at the
    door bottom (z = 0). Local +Y runs across the leaf toward the free (latch)
    edge. The leaf is modeled in this hinge-edge frame so the joint origin can
    sit exactly on the real pivot.
    """
    leaf = model.part("door_leaf")

    z0 = DOOR_LEAF_Z0  # leaf bottom sits just above the stone stoop top
    spring_z = z0 + SPRING_LOCAL  # spring line in world Z (leaf frame)

    # Backing board (thin, carries all planks). Reduced to the spring line
    # height so it does not protrude above the arched planks at the edges.
    # The arch portion of the planks is self-supporting (as in real plank doors).
    back_cx_x = -DOOR_T / 2.0 + BACK_T / 2.0
    leaf.visual(
        Box((BACK_T, DOOR_W, SPRING_LOCAL)),
        origin=Origin(xyz=(back_cx_x, DOOR_W / 2.0, z0 + SPRING_LOCAL / 2.0)),
        material="dark_oak_groove",
        name="backing_board",
    )

    # Vertical planks across the leaf width, each with a lancet gothic arch
    # top profile built from real CadQuery arc faces (threePointArc).
    usable_w = DOOR_W - (PLANK_COUNT - 1) * PLANK_GAP
    plank_w = usable_w / PLANK_COUNT
    # X offset for the plank extrusion start (rear of planks on backing front).
    x_start = -DOOR_T / 2.0 + BACK_T
    for i in range(PLANK_COUNT):
        y_left = i * (plank_w + PLANK_GAP)
        y_right = y_left + plank_w

        solid = _make_arch_plank_solid(
            y_left=y_left,
            y_right=y_right,
            z_bot=z0,
            W=DOOR_W,
            spring_z=spring_z,
            rise=ARCH_RISE,
            thickness=PLANK_T,
        )
        leaf.visual(
            mesh_from_cadquery(solid, f"plank_{i}"),
            # The solid is extruded from X=0 in +X by PLANK_T; translate to
            # the correct X position in the leaf frame.
            origin=Origin(xyz=(x_start, 0.0, 0.0)),
            material="weathered_oak",
            name=f"plank_{i}",
        )

    # Iron ledge/strap braces on the front face (top and bottom horizontal
    # boards), typical batten/ledge plank-door construction. The top ledge is
    # positioned below the arch spring line so it sits on the rectangular body.
    ledge_cx_x = -DOOR_T / 2.0 + BACK_T + PLANK_T + LEDGE_T / 2.0
    barrel_r = 0.018
    barrel_len = 0.10
    strap_y0 = 0.05
    strap_w = DOOR_W - strap_y0
    strap_cy = strap_y0 + strap_w / 2.0

    # Bottom ledge at z0 + 0.20, top ledge below spring line.
    top_ledge_cz = z0 + SPRING_LOCAL - LEDGE_W / 2.0 - 0.03
    for tag, cz in (("bottom", z0 + 0.20), ("top", top_ledge_cz)):
        leaf.visual(
            Box((LEDGE_T, strap_w, LEDGE_W)),
            origin=Origin(xyz=(ledge_cx_x, strap_cy, cz)),
            material="aged_iron",
            name=f"ledge_{tag}",
        )
        # Vertical iron hinge barrel (knuckle) at the leaf hinge edge.
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

    # Plank front surface (where face-mounted hardware sits).
    plate_x = -DOOR_T / 2.0 + BACK_T + PLANK_T

    # Small brass/iron sign plate near the top of the leaf (matches the light
    # rectangular plate in the reference). Mounted flush on the plank face.
    sign_t = 0.006
    sign_cz = z0 + SPRING_LOCAL - 0.42
    leaf.visual(
        Box((sign_t, 0.16, 0.10)),
        origin=Origin(xyz=(plate_x + sign_t / 2.0, DOOR_W / 2.0, sign_cz)),
        material="brass_sign",
        name="sign_plate",
    )

    # ---- Iron ring-pull hardware: SQUARE backplate + forward boss/stud ----
    hw_cy = HW_CY
    hw_cz = HW_CZ

    # Square iron backplate seated flush on the plank face.
    leaf.visual(
        Box((PLATE_T, PLATE_SIZE, PLATE_SIZE)),
        origin=Origin(xyz=(plate_x + PLATE_T / 2.0, hw_cy, hw_cz)),
        material="aged_iron",
        name="ring_backplate",
    )

    # Short forward boss/stud projecting from the plate that the ring hangs on.
    boss_x = plate_x + PLATE_T + BOSS_LEN / 2.0
    leaf.visual(
        Cylinder(radius=BOSS_R, length=BOSS_LEN),
        origin=Origin(
            xyz=(boss_x, hw_cy, hw_cz + BOSS_Z_OFF),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="aged_iron",
        name="ring_boss",
    )

    # ---- Iron lock plate ----
    lock_cz = z0 + 0.86
    leaf.visual(
        Box((LOCK_T, LOCK_W, LOCK_H)),
        origin=Origin(xyz=(plate_x + LOCK_T / 2.0, hw_cy + 0.13, lock_cz)),
        material="aged_iron",
        name="lock_plate",
    )
    # Keyhole boss / handle stub on the lock plate.
    leaf.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(
            xyz=(plate_x + LOCK_T + 0.010, hw_cy + 0.13, lock_cz - 0.02),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="aged_iron",
        name="lock_knob",
    )


def _build_ring_pull(model: ArticulatedObject) -> None:
    """The iron pull ring as its own rotatable part.

    The part's local frame origin coincides with the revolute joint origin,
    which is placed on the boss pivot line (top of the ring). The ring hangs
    BELOW the origin so the joint origin sits at the top of the ring; rotating
    about the (horizontal, door-face-parallel) joint axis lifts the ring's
    lower edge away from the door face.
    """
    ring_part = model.part("ring_pull")

    ring_center_r = RING_OUT_R - RING_TUBE_R
    ring = (
        cq.Workplane("XY")
        .moveTo(ring_center_r, 0.0)
        .circle(RING_TUBE_R)
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    ring_part.visual(
        mesh_from_cadquery(ring, "ring"),
        origin=Origin(
            xyz=(0.0, 0.0, -ring_center_r),
            rpy=(0.0, 0.0, -math.pi / 2.0),
        ),
        material="aged_iron",
        name="ring",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plank_door_lancet_arch")
    _wood(model)

    _build_stone_surround(model)
    _build_door_leaf(model)
    _build_ring_pull(model)

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

    # Pull ring hangs from the boss and pivots so it can be lifted/pulled.
    model.articulation(
        "leaf_to_ring",
        ArticulationType.REVOLUTE,
        parent="door_leaf",
        child="ring_pull",
        origin=Origin(xyz=(BOSS_PIVOT_X, HW_CY, BOSS_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    surround = object_model.get_part("stone_surround")
    leaf = object_model.get_part("door_leaf")
    ring_part = object_model.get_part("ring_pull")
    hinge = object_model.get_articulation("surround_to_leaf")
    ring_joint = object_model.get_articulation("leaf_to_ring")

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
    ring = ring_part.get_visual("ring")
    backplate = leaf.get_visual("ring_backplate")
    boss = leaf.get_visual("ring_boss")
    lock = leaf.get_visual("lock_plate")
    ctx.check(
        "leaf has all five vertical planks",
        all(leaf.get_visual(f"plank_{i}") is not None for i in range(PLANK_COUNT)),
        details="planks 0..4 must exist",
    )
    ctx.check(
        "ring pull (ring part + backplate + boss) and lock plate present",
        ring is not None
        and backplate is not None
        and boss is not None
        and lock is not None,
        details="ring part, square backplate, boss, and lock_plate are hero features",
    )

    # --- Lancet arch: center plank taller than edge planks ---
    center_aabb = ctx.part_element_world_aabb(leaf, elem="plank_2")
    edge_aabb_0 = ctx.part_element_world_aabb(leaf, elem="plank_0")
    edge_aabb_4 = ctx.part_element_world_aabb(leaf, elem=f"plank_{PLANK_COUNT - 1}")
    ctx.check(
        "lancet arch: center plank peak is higher than edge plank tops",
        center_aabb is not None
        and edge_aabb_0 is not None
        and edge_aabb_4 is not None
        and center_aabb[1][2] > edge_aabb_0[1][2] + 0.05
        and center_aabb[1][2] > edge_aabb_4[1][2] + 0.05,
        details=(
            f"center peak z={center_aabb[1][2] if center_aabb else None}, "
            f"edge_0 top z={edge_aabb_0[1][2] if edge_aabb_0 else None}, "
            f"edge_4 top z={edge_aabb_4[1][2] if edge_aabb_4 else None}"
        ),
    )

    # --- Lancet arch: peak is below the stone head soffit ---
    header_aabb = ctx.part_element_world_aabb(surround, elem="header_lintel")
    ctx.check(
        "lancet arch peak stays below the stone header soffit",
        center_aabb is not None
        and header_aabb is not None
        and center_aabb[1][2] < header_aabb[0][2],
        details=(
            f"arch peak z={center_aabb[1][2] if center_aabb else None}, "
            f"soffit z={header_aabb[0][2] if header_aabb else None}"
        ),
    )

    # --- Arch rise is substantial (not flat-topped) ---
    # Measure the rise from the spring line (where the arch begins) to the peak.
    # The spring line is at DOOR_LEAF_Z0 + SPRING_LOCAL in world coordinates.
    spring_line_world = DOOR_LEAF_Z0 + SPRING_LOCAL
    ctx.check(
        "lancet arch has substantial rise (>0.30m from spring line to peak)",
        center_aabb is not None
        and (center_aabb[1][2] - spring_line_world) > 0.30,
        details=(
            f"peak z={center_aabb[1][2] if center_aabb else None}, "
            f"spring line z={spring_line_world}, "
            f"rise={(center_aabb[1][2] - spring_line_world) if center_aabb else None}"
        ),
    )

    # --- Ring is its own part on a revolute joint about a horizontal Y axis ---
    rax = ring_joint.axis
    ctx.check(
        "ring joint axis is horizontal and parallel to the door face (Y)",
        abs(rax[1]) > 0.99 and abs(rax[0]) < 1e-6 and abs(rax[2]) < 1e-6,
        details=f"ring axis={rax}",
    )
    rlims = ring_joint.motion_limits
    ctx.check(
        "ring joint has a sensible lift range",
        rlims is not None
        and rlims.lower is not None
        and rlims.upper is not None
        and abs(rlims.lower) < 1e-6
        and 0.6 <= rlims.upper <= 1.6,
        details=f"ring limits=({rlims.lower if rlims else None},{rlims.upper if rlims else None})",
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

    # --- Outward swing clears the stone stoop (no clipping through the step) ---
    step_aabb = ctx.part_element_world_aabb(surround, elem="stone_step")
    ctx.check(
        "door bottom clears the stone stoop so it never clips the step when swung",
        leaf_aabb is not None
        and step_aabb is not None
        and leaf_aabb[0][2] > step_aabb[1][2],
        details=f"leaf z-min={leaf_aabb[0][2] if leaf_aabb else None}, "
        f"stoop z-max={step_aabb[1][2] if step_aabb else None}",
    )

    # --- Closed leaf seats within the stone opening (between the jambs) ---
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

    # --- Ring captured on its boss (hung ring) ---
    ctx.allow_overlap(
        ring_part,
        leaf,
        elem_a="ring",
        elem_b="ring_boss",
        reason="Pull ring is hung on and captured by the iron boss stud (the ring "
        "tube wraps the boss); a small overlap keeps it from floating.",
    )

    # Prove the captured-pin connection actually contacts.
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

    # --- Ring pull and lock plate are mounted on the leaf face (proud +X) ---
    ring_aabb = ctx.part_element_world_aabb(ring_part, elem="ring")
    plank_aabb = ctx.part_element_world_aabb(leaf, elem="plank_0")
    ctx.check(
        "ring pull stands proud of the plank face",
        ring_aabb is not None and plank_aabb is not None and ring_aabb[1][0] > plank_aabb[1][0],
        details=f"ring xmax={ring_aabb[1][0] if ring_aabb else None}, plank xmax={plank_aabb[1][0] if plank_aabb else None}",
    )

    # --- Ring hangs below its boss in the rest pose ---
    boss_aabb = ctx.part_element_world_aabb(leaf, elem="ring_boss")
    ctx.check(
        "ring hangs below the boss at rest",
        ring_aabb is not None
        and boss_aabb is not None
        and ring_aabb[0][2] < boss_aabb[0][2],
        details=f"ring zmin={ring_aabb[0][2] if ring_aabb else None}, "
        f"boss zmin={boss_aabb[0][2] if boss_aabb else None}",
    )

    # --- The ring actually rotates on its boss ---
    closed_ring = ctx.part_element_world_aabb(ring_part, elem="ring")
    with ctx.pose({ring_joint: 1.0}):
        lifted_ring = ctx.part_element_world_aabb(ring_part, elem="ring")
        ctx.check(
            "posing the ring joint swings the ring's lower edge away from the door (+X)",
            closed_ring is not None
            and lifted_ring is not None
            and lifted_ring[1][0] > closed_ring[1][0] + 0.02
            and lifted_ring[0][2] > closed_ring[0][2] + 0.01,
            details=f"closed ring xmax={closed_ring[1][0] if closed_ring else None}, "
            f"lifted ring xmax={lifted_ring[1][0] if lifted_ring else None}, "
            f"closed zmin={closed_ring[0][2] if closed_ring else None}, "
            f"lifted zmin={lifted_ring[0][2] if lifted_ring else None}",
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
            details=f"closed free xmax={closed_free[1][0] if closed_free else None}, open free xmax={open_free[1][0] if open_free else None}",
        )

    return ctx.report()


object_model = build_object_model()
