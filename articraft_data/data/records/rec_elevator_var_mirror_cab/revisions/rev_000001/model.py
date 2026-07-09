from __future__ import annotations

# Lobby elevator: polished brass center-opening doors in a cream marble
# surround, with a lit floor-position indicator above and a brass hall
# call-button plate beside the doors. When the two door leaves slide apart
# they reveal the cab interior: a large mirrored rear wall flanked by slim
# polished trim, side panels, and a light ceiling.
#
# Convention (real meters, Z-up):
#   X  -> doorway width / door slide axis
#   +Y -> into the cab (depth); the lobby viewer looks toward +Y
#   +Z -> up; lobby/cab floor at z = 0
# Marble wall front face at y = 0; the cab is recessed behind it.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions
# ---------------------------------------------------------------------------

# Doorway opening (clear interior of the marble frame).
OPENING_W = 1.10          # doorway clear width (X)
OPENING_H = 2.10          # doorway clear height (Z)

# Marble wall surround.
WALL_W = 2.30             # overall marble facade width (X)
WALL_H = 2.95             # overall marble facade height (Z)
WALL_T = 0.16             # marble depth (Y), front face at y = 0
JAMB = (WALL_W - OPENING_W) / 2.0   # marble on each side of the opening

# Header box (brass) above the doors holding the position indicator.
HEADER_W = OPENING_W + 0.18
HEADER_H = 0.26
HEADER_T = 0.10
HEADER_Z = OPENING_H + 0.04 + HEADER_H / 2.0   # sits just above the opening

# Floor-position indicator (lit amber segments inset in the header).
IND_SEG_W = 0.045
IND_SEG_H = 0.10
IND_SEG_GAP = 0.018
IND_N = 5

# Hall call-button brass plate beside the doors (to the right).
CALL_W = 0.10
CALL_H = 0.22
CALL_T = 0.012
CALL_X = OPENING_W / 2.0 + JAMB / 2.0   # centered in the right jamb
CALL_Z = 1.10

# Door leaves (polished brass, center-opening).
DOOR_W = OPENING_W / 2.0 + 0.012   # slight overlap at center seam when closed
DOOR_H = OPENING_H + 0.02
DOOR_T = 0.040
# Doors run on a track just BEHIND the marble back face (y = WALL_T = 0.16),
# so the polished leaves are framed by the marble portal and never intersect
# the marble jambs/lintel. Door body Y in [0.165, 0.205].
DOOR_Y = WALL_T + 0.005 + DOOR_T / 2.0
DOOR_OPEN = 0.52                    # each leaf travel when fully open

# Cab interior (recessed box behind the doorway).
CAB_W = 1.46                        # interior clear width (X)
CAB_DEPTH = 1.50                    # interior clear depth (Y)
CAB_H = 2.30                        # interior clear height (Z)
CAB_FRONT_Y = WALL_T               # cab opening flush with rear of marble
CAB_BACK_Y = CAB_FRONT_Y + CAB_DEPTH
CAB_SHELL = 0.06                    # structural shell / panel reveal thickness
# Cab floor / interior begins just behind the door track so the leaves never
# touch the floor shelf or the cab walls (door pocket gap).
FLOOR_FRONT_Y = WALL_T + 0.084   # ~0.244, just behind the door track sill

# Mirror panel on the cab back wall (single large reflective surface).
MIRROR_W = CAB_W - 0.14             # mirror clear width (X)
MIRROR_H = CAB_H - 0.16             # mirror clear height (Z)
MIRROR_T = 0.010                    # mirror glass thickness (Y)
TRIM_W = 0.030                      # polished trim strip width (X)
TRIM_T = MIRROR_T + 0.004           # trim slightly thicker than mirror

# Tessellation tolerances for crisp panel reveals.
TOL = 0.0015
ATOL = 0.2


# ---------------------------------------------------------------------------
# Geometry builders
# ---------------------------------------------------------------------------


def _marble_surround_shape() -> cq.Workplane:
    """Cream marble facade slab with the doorway cut through it.

    Authored in meters. Slab spans X = [-WALL_W/2, WALL_W/2],
    Y = [0, WALL_T], Z = [0, WALL_H]. The doorway is a real boolean
    through-cut from z = 0 up to OPENING_H, centered in X.
    """
    # centered=(True, False, False): X centered, Y in [0, WALL_T],
    # Z in [0, WALL_H] (floor at z=0). No extra Z translate.
    slab = (
        cq.Workplane("XY")
        .box(WALL_W, WALL_T, WALL_H, centered=(True, False, False))
    )
    # Doorway cutter: a fully centered box positioned so it spans X in
    # [-OPENING_W/2, OPENING_W/2], Z in [0, OPENING_H], over-deep in Y so the
    # boolean cleanly clears the front and back faces of the marble slab.
    cutter = (
        cq.Workplane("XY")
        .box(OPENING_W, WALL_T + 0.10, OPENING_H, centered=(True, True, True))
        .translate((0.0, WALL_T / 2.0, OPENING_H / 2.0))
    )
    slab = slab.cut(cutter)

    # Recessed reveal: a shallow inner step around the opening (front only) so
    # the marble reads as a framed portal rather than a bare punched hole.
    reveal_w = OPENING_W + 0.10
    reveal_h = OPENING_H + 0.05
    reveal = (
        cq.Workplane("XY")
        .box(reveal_w, 0.06, reveal_h, centered=(True, True, True))
        .translate((0.0, 0.03, reveal_h / 2.0))
    )
    slab = slab.cut(reveal)

    # A horizontal base wainscot ledge (the dark marble dado line in the
    # reference) cut as a shallow groove across the facade front face.
    groove = (
        cq.Workplane("XY")
        .box(WALL_W + 0.02, 0.024, 0.03, centered=(True, True, True))
        .translate((0.0, 0.0, 0.90))
    )
    slab = slab.cut(groove)
    return slab


def _header_box_shape() -> cq.Workplane:
    """Polished brass header box above the doors. A shallow rectangular frame
    relief around the display area reads as the indicator surround; the lit
    indicator strip is mounted proud on the front face (no deep cavity, so the
    overlap engine sees a clean convex box)."""
    body = (
        cq.Workplane("XY")
        .box(HEADER_W, HEADER_T, HEADER_H, centered=(True, True, True))
        .edges("|Y").fillet(0.012)
    )
    return body.translate((0.0, -HEADER_T / 2.0, HEADER_Z))


def _door_track_shape() -> cq.Workplane:
    """Brass header track rail + threshold sill on which the leaves run.

    Both span the doorway width at the door Y plane, just behind the marble,
    so the sliding leaves physically contact the grounded structure (the door
    leaves hang from the header track and ride the sill). Authored in world
    coordinates (this is a static base feature).
    """
    track_w = OPENING_W + 0.16
    track_depth = 0.08
    # Front face embeds a hair (0.004 m, below the 0.005 overlap tolerance) into
    # the marble back so the track is solidly connected to the grounded marble
    # while the band still overlaps the door Y range so the leaves contact it.
    track_yc = WALL_T - 0.004 + track_depth / 2.0
    parts = cq.Workplane("XY")

    # Header track rail just above the opening, at the door Y band.
    rail = (
        cq.Workplane("XY")
        .box(track_w, track_depth, 0.06, centered=(True, True, True))
        .translate((0.0, track_yc, OPENING_H + 0.02))
    )
    parts = parts.union(rail)

    # Threshold sill at the floor, at the door Y band.
    sill = (
        cq.Workplane("XY")
        .box(track_w, track_depth, 0.02, centered=(True, True, False))
        .translate((0.0, track_yc, 0.0))
    )
    parts = parts.union(sill)
    return parts


def _call_plate_shape() -> cq.Workplane:
    """Brass hall call-button plate with two recessed round call buttons."""
    plate = (
        cq.Workplane("XY")
        .box(CALL_W, CALL_T, CALL_H, centered=(True, True, True))
        .edges("|Y").fillet(0.012)
    )
    # Two recessed circular call-button pockets (up / down).
    for dz in (0.05, -0.05):
        pocket = (
            cq.Workplane("XZ")
            .workplane(offset=-CALL_T / 2.0 - 0.001)
            .center(0.0, dz)
            .circle(0.022)
            .extrude(0.012)
        )
        plate = plate.cut(pocket)
    # Embed the back of the plate slightly into the marble face (y=0) so it
    # reads as mounted flush to the wall rather than floating.
    return plate.translate((CALL_X, CALL_T / 2.0 - 0.006, CALL_Z))


def _door_leaf_shape() -> cq.Workplane:
    """A polished brass door leaf with three subtle vertical reveal grooves.

    Authored centered at the local origin in X so it can be positioned by the
    visual origin. Thickness along Y, height along Z.
    """
    leaf = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H, centered=(True, True, True))
        .edges("|Y").fillet(0.004)
    )
    # Vertical reveal grooves on the front (lobby, -Y) face.
    for gx in (-DOOR_W / 4.0, 0.0, DOOR_W / 4.0):
        groove = (
            cq.Workplane("XY")
            .box(0.006, 0.006, DOOR_H - 0.06, centered=(True, True, True))
            .translate((gx, -DOOR_T / 2.0 + 0.003, 0.0))
        )
        leaf = leaf.cut(groove)
    return leaf


def _cab_shell_shape() -> cq.Workplane:
    """Recessed cab box: back wall + two side walls + ceiling slab, built as
    explicit unioned slabs so the front (toward the lobby) is fully OPEN. The
    walls reach forward to the marble back face (CAB_FRONT_Y) so the cab reads
    as one connected structure attached behind the doorway.

    The interior clear volume is X in [-CAB_W/2, CAB_W/2], Y in
    [CAB_FRONT_Y, CAB_BACK_Y], Z in [0, CAB_H].
    """
    outer_w = CAB_W + 2 * CAB_SHELL
    shell = cq.Workplane("XY")

    # Back wall (full width, full height) at the rear.
    back = (
        cq.Workplane("XY")
        .box(outer_w, CAB_SHELL, CAB_H + CAB_SHELL, centered=(True, True, False))
        .translate((0.0, CAB_BACK_Y + CAB_SHELL / 2.0, 0.0))
    )
    shell = shell.union(back)

    # Two side walls running from the marble back to the cab back.
    for sx in (-(CAB_W / 2.0) - CAB_SHELL / 2.0, (CAB_W / 2.0) + CAB_SHELL / 2.0):
        side = (
            cq.Workplane("XY")
            .box(CAB_SHELL, CAB_DEPTH, CAB_H + CAB_SHELL, centered=(True, True, False))
            .translate((sx, (CAB_FRONT_Y + CAB_BACK_Y) / 2.0, 0.0))
        )
        shell = shell.union(side)

    # Ceiling slab capping the cab. Spans Y in [CAB_FRONT_Y, CAB_BACK_Y+CAB_SHELL]
    # so its front edge stays flush with the marble back (it does not poke into
    # the marble lintel).
    ceil_depth = (CAB_BACK_Y + CAB_SHELL) - CAB_FRONT_Y
    ceil = (
        cq.Workplane("XY")
        .box(outer_w, ceil_depth, CAB_SHELL, centered=(True, True, True))
        .translate((0.0, CAB_FRONT_Y + ceil_depth / 2.0, CAB_H + CAB_SHELL / 2.0))
    )
    shell = shell.union(ceil)
    return shell


def _mirror_panel_shape() -> cq.Workplane:
    """Single large reflective mirror panel filling the cab rear wall.

    A thin near-white glass sheet mounted flat against the cab back wall,
    spanning most of the width and height. The front face (toward -Y, the
    lobby viewer) is the reflective surface.
    """
    # Mirror sits flush against the inner face of the back wall.
    mirror_y_front = CAB_BACK_Y - MIRROR_T
    return (
        cq.Workplane("XY")
        .box(MIRROR_W, MIRROR_T, MIRROR_H, centered=(True, True, False))
        .translate((0.0, mirror_y_front + MIRROR_T / 2.0, 0.08))
    )


def _mirror_trim_shape() -> cq.Workplane:
    """Slim polished trim strips flanking the mirror panel (left and right).

    Two vertical polished metal strips, one on each side of the mirror,
    reading as a decorative frame. Each strip is thin in Y and tall in Z,
    sitting just proud of the mirror surface.
    """
    trim = cq.Workplane("XY")
    mirror_y_front = CAB_BACK_Y - MIRROR_T
    trim_y_front = mirror_y_front - 0.002  # slightly proud of mirror face

    for sx in (-(MIRROR_W / 2.0 + TRIM_W / 2.0), (MIRROR_W / 2.0 + TRIM_W / 2.0)):
        strip = (
            cq.Workplane("XY")
            .box(TRIM_W, TRIM_T, MIRROR_H + 0.02, centered=(True, True, False))
            .translate((sx, trim_y_front + TRIM_T / 2.0, 0.07))
        )
        trim = trim.union(strip)
    return trim


def _cab_side_panels_shape() -> cq.Workplane:
    """Side wall cladding skins (thin panels on left and right cab walls)."""
    skin = 0.018
    panels = cq.Workplane("XY")
    for sx in (-(CAB_W / 2.0) + skin / 2.0, (CAB_W / 2.0) - skin / 2.0):
        side = (
            cq.Workplane("XY")
            .box(skin, CAB_DEPTH - 0.04, CAB_H - 0.02, centered=(True, True, False))
            .translate((sx, CAB_FRONT_Y + CAB_DEPTH / 2.0, 0.01))
        )
        panels = panels.union(side)
    return panels


def _ceiling_shape() -> cq.Workplane:
    """Light (near-white) recessed ceiling panel inside the cab. Its top edge
    runs up to the shell ceiling slab (z = CAB_H) so it reads as mounted to the
    ceiling rather than as a floating slab."""
    return (
        cq.Workplane("XY")
        .box(CAB_W - 0.08, CAB_DEPTH - 0.08, 0.05, centered=(True, True, True))
        .translate((0.0, CAB_FRONT_Y + CAB_DEPTH / 2.0, CAB_H - 0.02))
    )


def _floor_shape() -> cq.Workplane:
    """Cab floor pad sitting at z = 0, starting just behind the door track so
    the sliding leaves never touch it (door pocket gap)."""
    depth = CAB_BACK_Y - 0.01 - FLOOR_FRONT_Y
    return (
        cq.Workplane("XY")
        .box(CAB_W - 0.02, depth, 0.03, centered=(True, True, False))
        .translate((0.0, (FLOOR_FRONT_Y + CAB_BACK_Y - 0.01) / 2.0, 0.0))
    )


def _indicator_shape() -> cq.Workplane:
    """Lit amber position-indicator strip: a row of small emissive segments on
    a single thin backing bar, mounted ON the header front face (-Y). The bar
    back seats ~0.004 m into the header front face (below the 0.005 overlap
    tolerance) for a solid connection; the lit segments stand proud toward the
    lobby. This avoids embedding the strip inside the concave display slot."""
    total = IND_N * IND_SEG_W + (IND_N - 1) * IND_SEG_GAP
    back_t = 0.010
    # Backing bar, with its front face at local y = 0.
    strip = (
        cq.Workplane("XY")
        .box(total + 0.012, back_t, IND_SEG_H + 0.012, centered=(True, False, True))
        .translate((0.0, -back_t, 0.0))
    )
    x0 = -total / 2.0 + IND_SEG_W / 2.0
    for i in range(IND_N):
        # Lit segments stand proud of the backing bar toward -Y (the lobby).
        seg = (
            cq.Workplane("XY")
            .box(IND_SEG_W, 0.012, IND_SEG_H, centered=(True, False, True))
            .translate((x0 + i * (IND_SEG_W + IND_SEG_GAP), -0.012, 0.0))
        )
        strip = strip.union(seg)
    # Header front face is at y = -HEADER_T. Place the bar so its back face
    # seats 0.004 m into that face; segments protrude toward the lobby.
    return strip.translate((0.0, -HEADER_T + 0.004, HEADER_Z + 0.01))


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lobby_elevator")

    model.material("marble_cream", rgba=(0.91, 0.88, 0.80, 1.0))
    model.material("brass", rgba=(0.84, 0.66, 0.26, 1.0))
    model.material("teal_panel", rgba=(0.16, 0.33, 0.40, 1.0))
    model.material("mirror_glass", rgba=(0.92, 0.93, 0.94, 1.0))
    model.material("polished_trim", rgba=(0.82, 0.82, 0.84, 1.0))
    model.material("ceiling_white", rgba=(0.90, 0.90, 0.86, 1.0))
    model.material("amber_lit", rgba=(0.98, 0.70, 0.18, 1.0))
    model.material("dark_cab_floor", rgba=(0.20, 0.20, 0.22, 1.0))

    # --- BASE: marble surround + brass header + call plate + cab structure ---
    base = model.part("marble_surround")
    base.visual(
        mesh_from_cadquery(_marble_surround_shape(), "marble_surround",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="marble_cream",
        name="marble_facade",
    )
    base.visual(
        mesh_from_cadquery(_header_box_shape(), "header_box",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="brass",
        name="header_box",
    )
    base.visual(
        mesh_from_cadquery(_call_plate_shape(), "call_plate",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="brass",
        name="call_plate",
    )
    base.visual(
        mesh_from_cadquery(_door_track_shape(), "door_track",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="brass",
        name="door_track",
    )

    # --- CAB INTERIOR (part of the static base assembly) ---
    cab = model.part("cab_interior")
    cab.visual(
        mesh_from_cadquery(_cab_shell_shape(), "cab_shell",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="teal_panel",
        name="cab_shell",
    )
    cab.visual(
        mesh_from_cadquery(_mirror_panel_shape(), "mirror_panel",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="mirror_glass",
        name="mirror_panel",
    )
    cab.visual(
        mesh_from_cadquery(_mirror_trim_shape(), "mirror_trim",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="polished_trim",
        name="mirror_trim",
    )
    cab.visual(
        mesh_from_cadquery(_cab_side_panels_shape(), "side_panels",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="teal_panel",
        name="side_panels",
    )
    cab.visual(
        mesh_from_cadquery(_ceiling_shape(), "cab_ceiling",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="ceiling_white",
        name="cab_ceiling",
    )
    cab.visual(
        mesh_from_cadquery(_floor_shape(), "cab_floor",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="dark_cab_floor",
        name="cab_floor",
    )
    # Cab is rigidly fixed to the marble opening (one continuous structure).
    model.articulation(
        "surround_to_cab",
        ArticulationType.FIXED,
        parent=base,
        child=cab,
        origin=Origin(),
    )

    # --- INDICATOR (lit segments) on the header ---
    indicator = model.part("indicator")
    indicator.visual(
        mesh_from_cadquery(_indicator_shape(), "indicator",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="amber_lit",
        name="indicator_segments",
    )
    model.articulation(
        "header_to_indicator",
        ArticulationType.FIXED,
        parent=base,
        child=indicator,
        origin=Origin(),
    )

    # --- DOOR LEAVES (brass, center-opening, prismatic) ---
    # Closed: leaves meet at center. The leaf mesh is centered at local origin,
    # so the visual origin places each leaf's center. q=0 = closed.
    left_door = model.part("left_door")
    left_door.visual(
        mesh_from_cadquery(_door_leaf_shape(), "left_door",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="brass",
        name="left_leaf",
    )
    right_door = model.part("right_door")
    right_door.visual(
        mesh_from_cadquery(_door_leaf_shape(), "right_door",
                           tolerance=TOL, angular_tolerance=ATOL),
        material="brass",
        name="right_leaf",
    )

    # Closed-pose centers: each leaf covers half the opening.
    left_center_x = -DOOR_W / 2.0 + 0.006
    right_center_x = DOOR_W / 2.0 - 0.006
    door_z = OPENING_H / 2.0

    # Left leaf slides toward -X to open.
    model.articulation(
        "surround_to_left_door",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_door,
        origin=Origin(xyz=(left_center_x, DOOR_Y, door_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DOOR_OPEN, effort=200.0, velocity=0.4),
    )
    # Right leaf slides toward +X to open.
    model.articulation(
        "surround_to_right_door",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_door,
        origin=Origin(xyz=(right_center_x, DOOR_Y, door_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DOOR_OPEN, effort=200.0, velocity=0.4),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("marble_surround")
    cab = object_model.get_part("cab_interior")
    indicator = object_model.get_part("indicator")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_joint = object_model.get_articulation("surround_to_left_door")
    right_joint = object_model.get_articulation("surround_to_right_door")

    # --- Intentional mechanical overlaps -----------------------------------
    # Center-opening leaves meet at the center with a small astragal overlap
    # when closed (the meeting stiles lap each other). The leaves also ride the
    # brass header track and threshold sill (hanger/guide engagement). These are
    # local, mostly-hidden, mechanically explanatory overlaps.
    ctx.allow_overlap(
        left_door, right_door, elem_a="left_leaf", elem_b="right_leaf",
        reason="Center-opening leaves lap at the meeting stiles when closed.",
    )
    ctx.allow_overlap(
        left_door, base, elem_a="left_leaf", elem_b="door_track",
        reason="Left leaf hanger/guide rides in the header track and sill.",
    )
    ctx.allow_overlap(
        right_door, base, elem_a="right_leaf", elem_b="door_track",
        reason="Right leaf hanger/guide rides in the header track and sill.",
    )

    # --- Scale / grounding ---
    base_aabb = ctx.part_world_aabb(base)
    cab_aabb = ctx.part_world_aabb(cab)
    ctx.check(
        "facade and cab reach the floor",
        base_aabb is not None and abs(base_aabb[0][2]) < 0.02
        and cab_aabb is not None and abs(cab_aabb[0][2]) < 0.06,
        details=f"base_zmin={base_aabb[0][2] if base_aabb else None}, "
                f"cab_zmin={cab_aabb[0][2] if cab_aabb else None}",
    )
    ctx.check(
        "facade is full lobby height (>=2.5 m)",
        base_aabb is not None and base_aabb[1][2] >= 2.5,
        details=f"base_zmax={base_aabb[1][2] if base_aabb else None}",
    )

    # --- Cab is recessed BEHIND the doorway (further +Y than the doors) ---
    door_aabb = ctx.part_world_aabb(left_door)
    ctx.check(
        "cab body sits behind the door plane",
        cab_aabb is not None and door_aabb is not None
        and cab_aabb[1][1] > door_aabb[1][1],
        details=f"cab_ymax={cab_aabb[1][1]:.3f}, door_ymax={door_aabb[1][1]:.3f}"
        if (cab_aabb and door_aabb) else "no aabb",
    )

    # --- Doors hang from the brass header track / ride the sill (connected) ---
    ctx.expect_overlap(
        left_door, base, axes="x", min_overlap=0.20,
        elem_a="left_leaf", elem_b="door_track",
        name="left leaf engages the door track",
    )
    ctx.expect_overlap(
        right_door, base, axes="x", min_overlap=0.20,
        elem_a="right_leaf", elem_b="door_track",
        name="right leaf engages the door track",
    )

    # --- CLOSED: door leaves cover the opening and meet at the center seam ---
    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        # The two leaves together span at least the doorway width.
        ld = ctx.part_world_aabb(left_door)
        rd = ctx.part_world_aabb(right_door)
        span = (rd[1][0] - ld[0][0]) if (ld and rd) else 0.0
        ctx.check(
            "closed leaves cover the doorway width",
            span >= OPENING_W - 0.01,
            details=f"covered_span={span:.3f} vs opening={OPENING_W}",
        )
        # Leaves are tall enough to cover the opening height.
        ctx.check(
            "closed leaves cover the doorway height",
            ld is not None and ld[1][2] >= OPENING_H - 0.02 and ld[0][2] <= 0.02,
            details=f"left_z=[{ld[0][2]:.3f},{ld[1][2]:.3f}]" if ld else "no aabb",
        )
        # Leaves meet near the center (small overlap at the seam, no gap).
        ctx.expect_overlap(
            left_door, right_door, axes="x", min_overlap=0.0,
            name="closed leaves meet at center seam",
        )

    # --- OPEN: leaves retract clear of the doorway, revealing the cab ---
    with ctx.pose({left_joint: DOOR_OPEN, right_joint: DOOR_OPEN}):
        ld_o = ctx.part_world_aabb(left_door)
        rd_o = ctx.part_world_aabb(right_door)
        # Left leaf inner edge has retracted to -X of the opening edge.
        ctx.check(
            "open left leaf clears the doorway center",
            ld_o is not None and ld_o[1][0] <= -OPENING_W / 2.0 + 0.10,
            details=f"left_xmax={ld_o[1][0]:.3f}" if ld_o else "no aabb",
        )
        ctx.check(
            "open right leaf clears the doorway center",
            rd_o is not None and rd_o[0][0] >= OPENING_W / 2.0 - 0.10,
            details=f"right_xmin={rd_o[0][0]:.3f}" if rd_o else "no aabb",
        )

    # --- Door travel direction is correct (leaves move apart when opening) ---
    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        l_closed = ctx.part_world_position(left_door)
        r_closed = ctx.part_world_position(right_door)
    with ctx.pose({left_joint: DOOR_OPEN, right_joint: DOOR_OPEN}):
        l_open = ctx.part_world_position(left_door)
        r_open = ctx.part_world_position(right_door)
    ctx.check(
        "leaves separate (center-opening) when opened",
        l_open[0] < l_closed[0] - 0.1 and r_open[0] > r_closed[0] + 0.1,
        details=f"L {l_closed[0]:.3f}->{l_open[0]:.3f}, R {r_closed[0]:.3f}->{r_open[0]:.3f}",
    )

    # --- Cab interior features exist and are placed correctly ---
    ctx.check(
        "cab is wide and deep enough to read as a cab",
        cab_aabb is not None
        and (cab_aabb[1][0] - cab_aabb[0][0]) >= 1.3
        and (cab_aabb[1][1] - cab_aabb[0][1]) >= 1.3,
        details=f"cab_dx={cab_aabb[1][0]-cab_aabb[0][0]:.3f}, "
                f"cab_dy={cab_aabb[1][1]-cab_aabb[0][1]:.3f}" if cab_aabb else "no aabb",
    )

    # Mirror panel on the cab back wall: large reflective surface filling the
    # rear wall, mounted near the back of the cab interior.
    mirror_aabb = ctx.part_element_world_aabb(cab, elem="mirror_panel")
    ctx.check(
        "mirror panel fills most of the cab back wall height",
        mirror_aabb is not None
        and (mirror_aabb[1][2] - mirror_aabb[0][2]) >= CAB_H * 0.75
        and mirror_aabb[1][2] >= CAB_H - 0.30,
        details=f"mirror_z=[{mirror_aabb[0][2]:.3f},{mirror_aabb[1][2]:.3f}]" if mirror_aabb else "no aabb",
    )
    ctx.check(
        "mirror panel fills most of the cab back wall width",
        mirror_aabb is not None
        and (mirror_aabb[1][0] - mirror_aabb[0][0]) >= CAB_W * 0.70,
        details=f"mirror_dx={mirror_aabb[1][0]-mirror_aabb[0][0]:.3f}" if mirror_aabb else "no aabb",
    )
    # Mirror is at the rear of the cab (high Y value, near CAB_BACK_Y).
    ctx.check(
        "mirror panel is at the cab rear wall",
        mirror_aabb is not None and mirror_aabb[1][1] >= CAB_BACK_Y - 0.10,
        details=f"mirror_ymax={mirror_aabb[1][1]:.3f}, cab_back_y={CAB_BACK_Y:.3f}" if mirror_aabb else "no aabb",
    )

    # Polished trim strips flank the mirror (left and right).
    trim_aabb = ctx.part_element_world_aabb(cab, elem="mirror_trim")
    ctx.check(
        "mirror trim flanks the mirror panel vertically",
        trim_aabb is not None
        and (trim_aabb[1][2] - trim_aabb[0][2]) >= MIRROR_H * 0.90,
        details=f"trim_dz={trim_aabb[1][2]-trim_aabb[0][2]:.3f}" if trim_aabb else "no aabb",
    )
    ctx.check(
        "mirror trim extends wider than mirror (flanking strips)",
        trim_aabb is not None and mirror_aabb is not None
        and trim_aabb[1][0] > mirror_aabb[1][0]
        and trim_aabb[0][0] < mirror_aabb[0][0],
        details=f"trim_x=[{trim_aabb[0][0]:.3f},{trim_aabb[1][0]:.3f}], "
                f"mirror_x=[{mirror_aabb[0][0]:.3f},{mirror_aabb[1][0]:.3f}]"
        if (trim_aabb and mirror_aabb) else "no aabb",
    )

    # Ceiling near the top of the cab.
    ceil_aabb = ctx.part_element_world_aabb(cab, elem="cab_ceiling")
    ctx.check(
        "ceiling sits high in the cab",
        ceil_aabb is not None and ceil_aabb[0][2] >= CAB_H - 0.20,
        details=f"ceil_zmin={ceil_aabb[0][2]:.3f}" if ceil_aabb else "no aabb",
    )

    # Indicator lit segments sit above the doorway on the header.
    ind_aabb = ctx.part_world_aabb(indicator)
    ctx.check(
        "lit indicator sits above the doorway",
        ind_aabb is not None and ind_aabb[0][2] >= OPENING_H,
        details=f"indicator_zmin={ind_aabb[0][2]:.3f}" if ind_aabb else "no aabb",
    )

    return ctx.report()


object_model = build_object_model()
