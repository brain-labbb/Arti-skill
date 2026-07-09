from __future__ import annotations

# Passenger elevator landing entrance.
#
# Coordinate convention (Z-up, meters):
#   +X = wall width        (doors slide along X)
#   +Y = wall thickness / depth, going back into the shaft
#   +Z = height            (ground/floor at z = 0)
#
# Y layout (front -> back):
#   buttons/digit proud    y < 0
#   door leaves            y in [-0.040, 0.000]  (shallow pocket in front of wall)
#   granite wall front     y = 0
#   granite wall body      y in [0.000, 0.150]
#   shaft recess           y in [~0, 0.350]      (inside the cut doorway tunnel)
#
# The dark granite wall surround is the fixed root. Two brushed-stainless
# center-opening leaves slide apart along X. Indicator, call panel, and sill are
# semantically separate parts, each fixed to and physically touching the wall.
#
# CadQuery convention used below (verified): on a "XZ" workplane,
#   .rect(w, h).extrude(+d)  spans local y in [-d, 0]
#   .rect(w, h).extrude(-d)  spans local y in [ 0, +d]
# so an XZ slab extruded by +WALL_D with no Y translate already gives y in
# [-WALL_D, 0]; we shift it to put the front face at y=0.

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
# Dimensions
# ---------------------------------------------------------------------------

# Granite wall slab: front face at y=0, body into +Y.
WALL_W = 2.60
WALL_H = 2.70
WALL_D = 0.15

# Doorway opening cut through the wall.
OPEN_W = 1.15
OPEN_H = 2.15
OPEN_HALF = OPEN_W / 2.0  # 0.575

# Door leaves (brushed stainless). Each covers half the opening when closed.
LEAF_W = OPEN_HALF  # 0.575
LEAF_BOTTOM = 0.018  # rests at the sill-track top
LEAF_TOP = OPEN_H  # reaches the opening head
LEAF_H = LEAF_TOP - LEAF_BOTTOM
LEAF_T = 0.040
DOOR_FRONT_Y = -0.040  # front face of the leaves
DOOR_BACK_Y = 0.0  # back face flush with the wall front plane
DOOR_CY = (DOOR_FRONT_Y + DOOR_BACK_Y) / 2.0  # -0.020

# Jamb / steel frame lining the opening (front trim band, into +Y a bit).
JAMB_T = 0.045  # how far the jamb reaches inward over the opening edge
JAMB_D = 0.12  # jamb depth along +Y from the wall front

# Prismatic travel: each leaf retracts a bit more than its width into its pocket.
DOOR_TRAVEL = 0.52

# Hall lantern cluster above the doorway: two metal housings stacked vertically,
# each carrying an emissive arrow lens (up-triangle and down-triangle).
LANTERN_W = 0.11          # housing width
LANTERN_H = 0.11          # housing height
LANTERN_D = 0.050         # housing depth (embedded into granite from y=0)
LANTERN_GAP = 0.025       # vertical gap between the two housings
LANTERN_CX = 0.0          # centered above the doorway
# Down-lantern center Z (just above the opening head).
LANTERN_CZ_DOWN = OPEN_H + 0.16
# Up-lantern center Z (above the down lantern).
LANTERN_CZ_UP = LANTERN_CZ_DOWN + LANTERN_H + LANTERN_GAP
# Arrow lens (emissive triangle) inside each housing.
LENS_INSET = 0.008        # how far the lens sits behind the housing front face
LENS_SIZE = 0.058         # bounding size of the arrow triangle
LENS_DEPTH = 0.006        # thickness of the emissive lens plate

# Hall call panel (right of the opening, ~1.1 m height).  Larger than the
# parent's tiny up/down plate: carries one bigger single call button.
PLATE_W = 0.13
PLATE_H = 0.24
PLATE_EMBED = 0.012       # embedded into granite front from y=0 to y=PLATE_EMBED
PLATE_CX = OPEN_HALF + 0.20   # beside the opening
PLATE_CZ = 1.10
BUTTON_R = 0.025          # single large call button radius
BUTTON_H = 0.016          # button protrusion height

# Threshold sill at the floor (grooved door track), butting into the wall front.
SILL_W = OPEN_W + 0.10
SILL_FRONT_Y = -0.14
SILL_BACK_Y = 0.03  # overlaps the wall front by 3 cm for a real connection
SILL_TOP = LEAF_BOTTOM  # 0.018; grooved top is the door track plane

# Shaft recess behind the doors (very dark), inside the cut doorway tunnel.
SHAFT_W = OPEN_W
SHAFT_H = OPEN_H
SHAFT_FRONT_Y = 0.0
SHAFT_BACK_Y = 0.35

# ---------------------------------------------------------------------------
# Colors / materials
# ---------------------------------------------------------------------------
GRANITE = (0.17, 0.17, 0.20, 1.0)
STEEL = (0.72, 0.73, 0.75, 1.0)
STEEL_DARK = (0.55, 0.56, 0.58, 1.0)
LANTERN_HOUSING = (0.22, 0.22, 0.24, 1.0)    # dark anodized metal housing
LANTERN_AMBER = (0.95, 0.72, 0.18, 1.0)      # warm amber emissive lens
LANTERN_LENS_BG = (0.12, 0.12, 0.13, 1.0)    # dark lens background
SHAFT_DARK = (0.05, 0.05, 0.06, 1.0)
CALL_PLATE_STEEL = (0.78, 0.79, 0.81, 1.0)   # brushed stainless call panel
BUTTON_STEEL = (0.82, 0.83, 0.85, 1.0)       # polished button cap


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters)
# ---------------------------------------------------------------------------
def _box_xyz(xc: float, xs: float, yc: float, ys: float, zc: float, zs: float) -> cq.Workplane:
    """Axis-aligned box from explicit center+size on each axis."""
    return cq.Workplane("XY").box(xs, ys, zs).translate((xc, yc, zc))


def _granite_wall_shape() -> cq.Workplane:
    """Dark granite slab with a real rectangular doorway cut through it.

    Front face at y=0, back face at y=WALL_D, floor at z=0, top at z=WALL_H,
    centered on X.
    """
    slab = _box_xyz(0.0, WALL_W, WALL_D / 2.0, WALL_D, WALL_H / 2.0, WALL_H)
    # Cut the doorway: a through-hole from the floor up to OPEN_H.
    opening = _box_xyz(
        0.0, OPEN_W, WALL_D / 2.0, WALL_D + 0.04, OPEN_H / 2.0, OPEN_H
    )
    return slab.cut(opening)


def _jamb_shape() -> cq.Workplane:
    """Brushed-steel U-jamb lining the doorway opening (two sides + head).

    A steel band wrapping the two vertical reveals and the head of the opening,
    flush at the wall front (y=0) and reaching JAMB_D into +Y. It reaches JAMB_T
    inward over each opening edge so it reads as a finished frame.
    """
    outer_w = OPEN_W + 2.0 * JAMB_T
    outer_top = OPEN_H + JAMB_T
    # Solid front band centered on the opening, from floor up over the head.
    frame = _box_xyz(0.0, outer_w, JAMB_D / 2.0, JAMB_D, outer_top / 2.0, outer_top)
    # Clear the opening through the band.
    clear = _box_xyz(
        0.0, OPEN_W, JAMB_D / 2.0, JAMB_D + 0.04, OPEN_H / 2.0 + 0.05, OPEN_H + 0.10
    )
    band = frame.cut(clear)
    # Trim below the floor (keep z >= 0).
    below = _box_xyz(0.0, outer_w + 0.05, JAMB_D / 2.0, JAMB_D + 0.05, -0.05, 0.10)
    return band.cut(below)


def _leaf_shape(sign: float) -> cq.Workplane:
    """A single brushed-stainless door leaf with a subtle recessed panel.

    `sign` = -1 (left) or +1 (right). The inner edge sits at x=0 (the center
    seam). Leaf spans z in [LEAF_BOTTOM, LEAF_TOP] and y in [DOOR_FRONT_Y, 0].
    """
    cx = sign * LEAF_W / 2.0
    cz = (LEAF_BOTTOM + LEAF_TOP) / 2.0
    body = _box_xyz(cx, LEAF_W, DOOR_CY, LEAF_T, cz, LEAF_H)
    # Subtle recessed panel on the front face (shallow pocket).
    pocket_w = LEAF_W - 0.12
    pocket_h = LEAF_H - 0.20
    pocket = _box_xyz(
        cx, pocket_w, DOOR_FRONT_Y + 0.004, 0.010, cz, pocket_h
    )
    leaf = body.cut(pocket)
    leaf = leaf.edges("|Z").fillet(0.004)
    return leaf


def _shaft_recess_shape() -> cq.Workplane:
    """Hollow elevator-car interior visible through the doorway.

    Five thin panels form a box open at the front (y = SHAFT_FRONT_Y):
    back wall, left wall, right wall, ceiling, and floor.  The panels sit
    just inside the opening edges so they are visible at any viewing angle
    and give real depth, unlike a solid box.
    """
    depth = SHAFT_BACK_Y - SHAFT_FRONT_Y   # 0.350
    t = 0.018                               # panel thickness
    w = SHAFT_W
    h = SHAFT_H

    # Back wall – full width, at the far end of the shaft
    back = _box_xyz(0.0, w, depth - t / 2.0, t, h / 2.0, h)

    # Left interior wall (inside opening, at x = −w/2 … −w/2+t)
    left = _box_xyz(-w / 2.0 + t / 2.0, t, depth / 2.0, depth, h / 2.0, h)

    # Right interior wall
    right = _box_xyz(w / 2.0 - t / 2.0, t, depth / 2.0, depth, h / 2.0, h)

    # Ceiling panel
    ceil_ = _box_xyz(0.0, w, depth / 2.0, depth, h - t / 2.0, t)

    # Floor panel (inside the shaft, behind the threshold sill)
    floor_ = _box_xyz(0.0, w, depth / 2.0, depth, t / 2.0, t)

    return back.union(left).union(right).union(ceil_).union(floor_)


def _sill_shape() -> cq.Workplane:
    """Stainless threshold sill at the floor with longitudinal door-track grooves."""
    yc = (SILL_FRONT_Y + SILL_BACK_Y) / 2.0
    yd = SILL_BACK_Y - SILL_FRONT_Y
    base = _box_xyz(0.0, SILL_W, yc, yd, SILL_TOP / 2.0, SILL_TOP)
    # Longitudinal grooves running along X (the slide direction), in the top face.
    n_grooves = 5
    groove_w = 0.010
    groove_depth = 0.008
    spacing = 0.022
    for i in range(n_grooves):
        gy = DOOR_CY + (i - (n_grooves - 1) / 2.0) * spacing
        groove = _box_xyz(
            0.0,
            SILL_W + 0.02,
            gy,
            groove_w,
            SILL_TOP - groove_depth / 2.0 + 0.0005,
            groove_depth,
        )
        base = base.cut(groove)
    return base


def _lantern_housing_shape(cx: float, cz: float) -> cq.Workplane:
    """One hall-lantern metal housing: a small rounded box recessed into granite.

    The housing sits with its front face slightly proud of the wall (y = -0.003)
    and its body embedded into the granite (y up to LANTERN_D). A shallow
    recess on the front face holds the arrow lens.
    """
    y_front = -0.003
    y_back = LANTERN_D
    yc = (y_front + y_back) / 2.0
    yd = y_back - y_front
    body = _box_xyz(cx, LANTERN_W, yc, yd, cz, LANTERN_H)
    body = body.edges("|Y").fillet(0.008)
    # Shallow front pocket for the lens (cut from the front face inward).
    pocket_w = LANTERN_W - 0.020
    pocket_h = LANTERN_H - 0.020
    pocket_d = LENS_INSET + 0.004
    pocket = _box_xyz(cx, pocket_w, y_front + pocket_d / 2.0, pocket_d, cz, pocket_h)
    return body.cut(pocket)


def _lantern_backplate_shape(cx: float) -> cq.Workplane:
    """Shared mounting backplate connecting the two lantern housings.

    A thin vertical plate behind both housings, embedded in the granite, that
    ties the two separate housings into one connected assembly.
    """
    total_h = (LANTERN_CZ_UP - LANTERN_CZ_DOWN) + LANTERN_H
    cz_center = (LANTERN_CZ_UP + LANTERN_CZ_DOWN) / 2.0
    plate_w = LANTERN_W - 0.010  # slightly narrower than housing
    plate_d = 0.012
    y_front = LANTERN_D - 0.002  # behind the housings, still in granite
    yc = y_front + plate_d / 2.0
    return _box_xyz(cx, plate_w, yc, plate_d, cz_center, total_h)


def _arrow_lens_shape(cx: float, cz: float, direction: float) -> cq.Workplane:
    """Emissive arrow-triangle lens sitting in a lantern housing recess.

    `direction` = +1 for UP arrow (triangle pointing +Z), -1 for DOWN arrow
    (triangle pointing -Z). The lens is a thin extruded triangle in the XZ
    plane, extruded along Y. The back face extends to the housing pocket floor
    so the lens physically seats against the housing (not floating in the recess).
    """
    half = LENS_SIZE / 2.0
    # Triangle vertices in XZ: equilateral-ish arrow pointing in `direction`.
    if direction > 0:
        # Up arrow: tip at top, base at bottom.
        pts = [(0.0, half), (-half * 0.85, -half * 0.6), (half * 0.85, -half * 0.6)]
    else:
        # Down arrow: tip at bottom, base at top.
        pts = [(0.0, -half), (-half * 0.85, half * 0.6), (half * 0.85, half * 0.6)]
    # Lens extends from slightly behind the housing front face to the pocket
    # floor, so it physically seats against the housing pocket back wall.
    y_front_lens = -0.001
    pocket_d = LENS_INSET + 0.004
    y_back_lens = -0.003 + pocket_d  # touches pocket back wall at y=0.009
    lens_depth = y_back_lens - y_front_lens
    lens = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(lens_depth)
        .translate((cx, y_back_lens, cz))
    )
    return lens


def _call_plate_shape() -> cq.Workplane:
    """Larger stainless hall call-panel plate, embedded flush in the wall front."""
    y_front = -0.003
    y_back = PLATE_EMBED
    yc = (y_front + y_back) / 2.0
    yd = y_back - y_front
    plate = _box_xyz(PLATE_CX, PLATE_W, yc, yd, PLATE_CZ, PLATE_H)
    plate = plate.edges("|Y").fillet(0.008)
    return plate


def _call_button_shape() -> cq.Workplane:
    """One large round call button proud of the call plate, toward -Y."""
    # Button spans y in [-0.014, +0.004]: proud of the plate face (y=-0.003) but
    # rooted ~7 mm into the plate body so it seats (not floating) on the plate.
    y_back = 0.004
    button = (
        cq.Workplane("XZ")
        .circle(BUTTON_R)
        .extrude(BUTTON_H)
        .translate((PLATE_CX, y_back, PLATE_CZ))
    )
    return button


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="elevator_landing_entrance")

    model.material("granite", rgba=GRANITE)
    model.material("steel", rgba=STEEL)
    model.material("steel_dark", rgba=STEEL_DARK)
    model.material("lantern_housing", rgba=LANTERN_HOUSING)
    model.material("lantern_amber", rgba=LANTERN_AMBER)
    model.material("lens_bg", rgba=LANTERN_LENS_BG)
    model.material("shaft_dark", rgba=SHAFT_DARK)
    model.material("call_plate", rgba=CALL_PLATE_STEEL)
    model.material("button_steel", rgba=BUTTON_STEEL)

    # --- Fixed root: the granite wall surround, its jamb, and the shaft recess. ---
    wall = model.part("wall_surround")
    wall.visual(
        mesh_from_cadquery(_granite_wall_shape(), "granite_slab"),
        material="granite",
    )
    wall.visual(
        mesh_from_cadquery(_jamb_shape(), "door_jamb"),
        material="steel_dark",
    )
    wall.visual(
        mesh_from_cadquery(_shaft_recess_shape(), "shaft_recess"),
        material="shaft_dark",
    )

    # Indicator: hall-lantern cluster (two arrow lanterns above the doorway).
    indicator = model.part("indicator")
    # Build a combined housing: both lantern housings + shared backplate,
    # unioned into one connected mesh.
    combined_housing = (
        _lantern_housing_shape(LANTERN_CX, LANTERN_CZ_UP)
        .union(_lantern_housing_shape(LANTERN_CX, LANTERN_CZ_DOWN))
        .union(_lantern_backplate_shape(LANTERN_CX))
    )
    indicator.visual(
        mesh_from_cadquery(combined_housing, "lantern_housing"),
        material="lantern_housing",
    )
    # Arrow lenses as separate emissive elements, each seated against the pocket wall.
    lantern_configs = [
        (LANTERN_CZ_UP, +1.0, "up"),     # up arrow lens (top)
        (LANTERN_CZ_DOWN, -1.0, "down"),  # down arrow lens (bottom)
    ]
    for i, (cz, direction, name_i) in enumerate(lantern_configs):
        indicator.visual(
            mesh_from_cadquery(
                _arrow_lens_shape(LANTERN_CX, cz, direction), f"arrow_lens_{name_i}"
            ),
            material="lantern_amber",
        )

    # Hall call panel: larger plate with one big call button, beside the opening.
    call_panel = model.part("call_panel")
    call_panel.visual(
        mesh_from_cadquery(_call_plate_shape(), "call_plate"),
        material="call_plate",
    )
    call_panel.visual(
        mesh_from_cadquery(_call_button_shape(), "call_button"),
        material="button_steel",
    )

    # Threshold sill: distinct part, butting into the wall front at the floor.
    sill = model.part("sill")
    sill.visual(
        mesh_from_cadquery(_sill_shape(), "sill_track"),
        material="steel",
    )

    # --- Door leaves (brushed stainless). ---
    left_door = model.part("left_door")
    left_door.visual(
        mesh_from_cadquery(_leaf_shape(-1.0), "left_leaf"),
        material="steel",
    )

    right_door = model.part("right_door")
    right_door.visual(
        mesh_from_cadquery(_leaf_shape(1.0), "right_leaf"),
        material="steel",
    )

    # --- Fixed mounts of the surround sub-parts to the wall root. ---
    model.articulation(
        "wall_to_indicator",
        ArticulationType.FIXED,
        parent=wall,
        child=indicator,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "wall_to_call_panel",
        ArticulationType.FIXED,
        parent=wall,
        child=call_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "wall_to_sill",
        ArticulationType.FIXED,
        parent=wall,
        child=sill,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Prismatic door joints: lower = CLOSED, upper = OPEN. ---
    # Left leaf retracts toward -X; right leaf retracts toward +X.
    model.articulation(
        "wall_to_left_door",
        ArticulationType.PRISMATIC,
        parent=wall,
        child=left_door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=DOOR_TRAVEL, effort=400.0, velocity=0.5
        ),
    )
    model.articulation(
        "wall_to_right_door",
        ArticulationType.PRISMATIC,
        parent=wall,
        child=right_door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=DOOR_TRAVEL, effort=400.0, velocity=0.5
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_surround")
    indicator = object_model.get_part("indicator")
    call_panel = object_model.get_part("call_panel")
    sill = object_model.get_part("sill")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    j_left = object_model.get_articulation("wall_to_left_door")
    j_right = object_model.get_articulation("wall_to_right_door")

    # Intentional, mechanically-real overlaps: each surround sub-part is fixed to
    # the wall by being embedded into / butting against the granite slab. These
    # are local seated-mount overlaps, each proven below with an exact check.
    ctx.allow_overlap(
        sill,
        wall,
        reason="The threshold sill butts into the wall front so it is physically attached at the floor.",
    )
    ctx.allow_overlap(
        indicator,
        wall,
        reason="The hall-lantern housings are recessed into the granite above the doorway.",
    )
    ctx.allow_overlap(
        call_panel,
        wall,
        reason="The call plate is flush-mounted (embedded) into the granite beside the doorway.",
    )
    # Prove each embed actually contacts the wall (not floating).
    ctx.expect_contact(indicator, wall, name="hall lanterns are seated in the wall")
    ctx.expect_contact(call_panel, wall, name="call panel is seated in the wall")
    ctx.expect_contact(sill, wall, name="sill is seated against the wall")

    # --- Hall lantern cluster checks. ---
    ind_aabb = ctx.part_world_aabb(indicator)
    ctx.check(
        "hall lanterns are above the doorway opening",
        ind_aabb[0][2] > OPEN_H,
        details=f"indicator_zmin={ind_aabb[0][2]:.4f} vs opening_h={OPEN_H}",
    )
    # The lantern cluster should be centered above the doorway (x ~ 0).
    ind_cx = (ind_aabb[0][0] + ind_aabb[1][0]) / 2.0
    ctx.check(
        "hall lanterns are centered above the doorway",
        abs(ind_cx) < 0.05,
        details=f"indicator_cx={ind_cx:.4f}",
    )
    # The cluster should span two lanterns vertically (at least LANTERN_H * 2 + gap).
    ind_h = ind_aabb[1][2] - ind_aabb[0][2]
    ctx.check(
        "hall lantern cluster spans both up and down lanterns",
        ind_h >= 2.0 * LANTERN_H + LANTERN_GAP - 0.01,
        details=f"indicator_height={ind_h:.4f} vs expected>={2.0 * LANTERN_H + LANTERN_GAP - 0.01:.4f}",
    )

    # --- Call panel checks. ---
    plate_aabb = ctx.part_world_aabb(call_panel)
    plate_cx = (plate_aabb[0][0] + plate_aabb[1][0]) / 2.0
    plate_cz = (plate_aabb[0][2] + plate_aabb[1][2]) / 2.0
    plate_w = plate_aabb[1][0] - plate_aabb[0][0]
    plate_h = plate_aabb[1][2] - plate_aabb[0][2]
    ctx.check(
        "call panel is beside the opening at hand height",
        plate_cx > OPEN_HALF and 0.9 < plate_cz < 1.3,
        details=f"plate_cx={plate_cx:.3f}, plate_cz={plate_cz:.3f}",
    )
    ctx.check(
        "call panel is larger than the parent's tiny plate",
        plate_w >= 0.12 and plate_h >= 0.22,
        details=f"plate_w={plate_w:.4f}, plate_h={plate_h:.4f}",
    )

    # --- Closed pose (q=0): leaves meet at the center seam and cover opening. ---
    with ctx.pose({j_left: 0.0, j_right: 0.0}):
        # Center seam gap ~0: right leaf min-x meets left leaf max-x at x=0.
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=-0.002,
            max_gap=0.004,
            name="closed leaves meet at center seam",
        )
        la = ctx.part_world_aabb(left_door)
        ra = ctx.part_world_aabb(right_door)
        lcx = (la[0][0] + la[1][0]) / 2.0
        rcx = (ra[0][0] + ra[1][0]) / 2.0
        # Leaves symmetric about x=0 when closed.
        ctx.check(
            "closed leaves are symmetric about center",
            abs(lcx + rcx) < 0.01 and abs(lcx) > 0.2,
            details=f"left_cx={lcx:.4f}, right_cx={rcx:.4f}",
        )
        # Same height span and Y placement (mirror geometry).
        ctx.check(
            "leaves share z-span",
            abs(la[0][2] - ra[0][2]) < 0.002 and abs(la[1][2] - ra[1][2]) < 0.002,
            details=f"L_z=({la[0][2]:.3f},{la[1][2]:.3f}) R_z=({ra[0][2]:.3f},{ra[1][2]:.3f})",
        )
        ctx.check(
            "leaves share y-span",
            abs(la[0][1] - ra[0][1]) < 0.002 and abs(la[1][1] - ra[1][1]) < 0.002,
            details=f"L_y=({la[0][1]:.3f},{la[1][1]:.3f}) R_y=({ra[0][1]:.3f},{ra[1][1]:.3f})",
        )
        # Closed leaf pair spans the full opening width (no peek-through at sides).
        total_w = ra[1][0] - la[0][0]
        ctx.check(
            "closed leaves cover the opening width",
            total_w >= OPEN_W - 0.01,
            details=f"covered_width={total_w:.4f} vs opening={OPEN_W}",
        )
        # Leaves reach (about) the sill and the opening head.
        ctx.check(
            "closed leaves span the opening height",
            la[0][2] < 0.03 and la[1][2] > OPEN_H - 0.02,
            details=f"L_z=({la[0][2]:.3f},{la[1][2]:.3f})",
        )

    # --- Open pose (upper): leaves retract clear of the opening center. ---
    with ctx.pose({j_left: DOOR_TRAVEL, j_right: DOOR_TRAVEL}):
        la_o = ctx.part_world_aabb(left_door)
        ra_o = ctx.part_world_aabb(right_door)
        center_gap = ra_o[0][0] - la_o[1][0]
        ctx.check(
            "open leaves clear the opening center",
            center_gap > 0.9 * OPEN_W,
            details=f"center_gap={center_gap:.4f} vs opening={OPEN_W}",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.9 * OPEN_W,
            name="open leaves separated by ~opening width",
        )

    # Decisive directional check: each leaf moves outward when opened.
    with ctx.pose({j_left: 0.0, j_right: 0.0}):
        lc = ctx.part_world_position(left_door)
        rc = ctx.part_world_position(right_door)
    with ctx.pose({j_left: DOOR_TRAVEL, j_right: DOOR_TRAVEL}):
        lo = ctx.part_world_position(left_door)
        ro = ctx.part_world_position(right_door)
    ctx.check(
        "left leaf opens toward -X, right toward +X",
        lo[0] < lc[0] - 0.3 and ro[0] > rc[0] + 0.3,
        details=f"left {lc[0]:.3f}->{lo[0]:.3f}, right {rc[0]:.3f}->{ro[0]:.3f}",
    )

    # --- Surround features in the right place, grounded, not floating. ---
    sill_aabb = ctx.part_world_aabb(sill)
    ctx.check(
        "sill sits at the floor (z ~ 0)",
        sill_aabb[0][2] < 0.005 and sill_aabb[1][2] < 0.05,
        details=f"sill_z=({sill_aabb[0][2]:.4f},{sill_aabb[1][2]:.4f})",
    )

    wall_aabb = ctx.part_world_aabb(wall)
    ctx.check(
        "wall surround is large and grounded",
        wall_aabb[0][2] < 0.01
        and (wall_aabb[1][0] - wall_aabb[0][0]) > OPEN_W + 0.5
        and (wall_aabb[1][2] - wall_aabb[0][2]) > OPEN_H,
        details=f"wall_x=({wall_aabb[0][0]:.2f},{wall_aabb[1][0]:.2f}) wall_z=({wall_aabb[0][2]:.2f},{wall_aabb[1][2]:.2f})",
    )

    return ctx.report()


object_model = build_object_model()
