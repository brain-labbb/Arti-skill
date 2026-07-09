from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Articraft brief
# ---------------------------------------------------------------------------
# Object: a residential telescoping sectional garage door with PERFORATED
#   SECURITY-GRILLE leaves, ~2.4 m wide x ~2.2 m tall, standing on the
#   ground (z = 0 upward). A black steel surround frame holds a stack of
#   grey grille leaves; a small latch handle sits on the bottom leaf.
# Root/support: the black steel frame (two jambs, a front header fascia, a
#   floor sill, and two deep side guide channels) is the fixed root.
# Parts: frame (root) + 6 grille leaves. slat_0 is the TOP leaf and is
#   FIXED to the frame. Each lower leaf hangs off the leaf above it.
# Articulations: slat_0 is attached to the frame with a FIXED joint. Each
#   remaining leaf is a PRISMATIC child of the leaf above it, axis +Z,
#   travelling one slat pitch upward RELATIVE TO ITS PARENT. The leaves are
#   depth-staggered: each lower leaf sits one panel thickness behind its
#   parent, so when raised every leaf telescopes up and nests behind the
#   leaf above it, the whole stack collecting behind the fixed top leaf
#   inside the frame (never above the header).
# Visible geometry: each leaf is a thin perforated metal grille — a flat
#   plate with a regular grid of square through-cut holes and a solid border
#   frame — reading as an open metal security grille. A dark latch handle
#   is mounted on the bottom leaf.
# Support/fit: each leaf sits in the frame opening with a small side
#   clearance; the side guide channels run the full stack depth.
# Intentional overlaps: none expected with thin grille leaves.
# Tests: door reads as full-height stack of 6 grille leaves, frame
#   surrounds the opening, leaves are depth-staggered, closed laps leave no
#   vertical gap, the open pose telescopes correctly, the latch handle is
#   mounted on the bottom leaf, each grille is a thin perforated sheet.
# Assumptions: 6 equal leaves; a black powder-coated steel frame;
#   galvanized steel grille leaves with ~65 mm square holes at ~90 mm pitch.
# ---------------------------------------------------------------------------

# --- Overall dimensions (meters) -------------------------------------------
DOOR_W = 2.40  # overall frame outer width
JAMB_W = 0.06  # slim side jamb width
JAMB_D = 0.12  # frame depth (front-to-back, along Y)
HEADER_H = 0.06  # slim top header height

OPENING_W = DOOR_W - 2.0 * JAMB_W  # clear opening width = 2.28
N_SLATS = 6  # ~6 horizontal grille leaves
SLAT_PITCH = 0.355  # vertical band height owned by each leaf
SLAT_LAP = 0.020  # vertical lap: a leaf's top edge tucks behind the leaf above
SLAT_D = 0.04  # leaf depth stagger (mounting offset between leaves)
SLAT_W = OPENING_W - 0.012  # leaf panel width, small side clearance

THRESHOLD_H = 0.05  # slim floor sill / bottom surround rail
PANEL_BASE_Z = THRESHOLD_H  # bottom of the lowest leaf sits on the sill

OPENING_H = N_SLATS * SLAT_PITCH  # 2.13 m clear panel stack
FRAME_OUTER_H = PANEL_BASE_Z + OPENING_H + HEADER_H  # overall frame height

FRAME_FRONT_Y = -JAMB_D / 2.0
PANEL_FRONT_Y = FRAME_FRONT_Y + 0.02  # top leaf front face recessed 0.02 m
STACK_BACK_Y = PANEL_FRONT_Y + N_SLATS * SLAT_D  # back face of the deepest leaf

# --- Grille parameters -----------------------------------------------------
GRILLE_THICKNESS = 0.005  # 5 mm perforated sheet thickness
GRILLE_HOLE_SIZE = 0.065  # 65 mm square cutouts
GRILLE_PITCH = 0.090  # 90 mm center-to-center → 25 mm bars
GRILLE_FRAME_W = 0.018  # 18 mm solid border around the grille field


def _band_center_z(i: int) -> float:
    """World z of the center of leaf i's vertical band (i = 0 is the TOP leaf)."""
    return PANEL_BASE_Z + OPENING_H - SLAT_PITCH * (i + 0.5)


def _leaf_center_y(i: int) -> float:
    """World y of the center of leaf i (each leaf one thickness behind its parent)."""
    return PANEL_FRONT_Y + SLAT_D * i + SLAT_D / 2.0


def _build_grille_mesh(field_h: float):
    """Build a perforated security-grille mesh for a leaf of given height.

    Creates a flat plate in local XY with a regular grid of square
    through-cut holes and a solid border frame, then rotates it into the
    leaf's XZ plane (visible face toward -Y, thickness along Y).
    """
    half_w = SLAT_W / 2.0
    half_h = field_h / 2.0

    # Outer rectangular profile (CCW winding).
    outer = [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]

    # Compute the regular grid of square holes inside the border frame.
    inner_w = SLAT_W - 2.0 * GRILLE_FRAME_W
    inner_h = field_h - 2.0 * GRILLE_FRAME_W
    n_cols = max(1, int(inner_w / GRILLE_PITCH))
    n_rows = max(1, int(inner_h / GRILLE_PITCH))
    x_spacing = inner_w / n_cols
    y_spacing = inner_h / n_rows
    hs = GRILLE_HOLE_SIZE / 2.0

    holes = []
    for row in range(n_rows):
        for col in range(n_cols):
            cx = -inner_w / 2.0 + x_spacing / 2.0 + col * x_spacing
            cy = -inner_h / 2.0 + y_spacing / 2.0 + row * y_spacing
            # Square hole profile (CW winding for interior loop).
            holes.append([
                (cx - hs, cy - hs),
                (cx - hs, cy + hs),
                (cx + hs, cy + hs),
                (cx + hs, cy - hs),
            ])

    geom = ExtrudeWithHolesGeometry(
        outer, holes, GRILLE_THICKNESS, cap=True, center=True, closed=True,
    )
    # Rotate from XY plane (geometry convention) into XZ plane (leaf frame):
    # geometry Y -> world Z (height), geometry Z -> world -Y (thickness).
    geom.rotate_x(math.pi / 2)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sectional_garage_door")

    # Flat matte palette: a slim near-black surround, galvanized steel grille,
    # and dark handle metal.
    steel_black = model.material("powdercoat_black_steel", rgba=(0.09, 0.09, 0.095, 1.0))
    grille_metal = model.material("grille_galvanized_steel", rgba=(0.48, 0.49, 0.50, 1.0))
    track_grey = model.material("galvanized_track", rgba=(0.40, 0.41, 0.42, 1.0))
    handle_metal = model.material("dark_handle_metal", rgba=(0.13, 0.13, 0.14, 1.0))

    # ---------------------------------------------------------------- frame
    # The frame is the fixed root: two side jambs, a front header fascia, a
    # floor sill, and two deep side guide channels the leaf edges ride in.
    frame = model.part("frame")

    jamb_full_h = OPENING_H + HEADER_H + THRESHOLD_H
    frame.visual(
        Box((JAMB_W, JAMB_D, jamb_full_h)),
        origin=Origin(xyz=(-(OPENING_W + JAMB_W) / 2.0, 0.0, PANEL_BASE_Z + OPENING_H / 2.0 + (HEADER_H - THRESHOLD_H) / 2.0)),
        material=steel_black,
        name="jamb_0",
    )
    frame.visual(
        Box((JAMB_W, JAMB_D, jamb_full_h)),
        origin=Origin(xyz=((OPENING_W + JAMB_W) / 2.0, 0.0, PANEL_BASE_Z + OPENING_H / 2.0 + (HEADER_H - THRESHOLD_H) / 2.0)),
        material=steel_black,
        name="jamb_1",
    )
    # Front header fascia spanning between the jambs.
    header_d = 0.055
    frame.visual(
        Box((DOOR_W, header_d, HEADER_H)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + header_d / 2.0, PANEL_BASE_Z + OPENING_H + HEADER_H / 2.0)),
        material=steel_black,
        name="header",
    )
    # Floor sill the bottom leaf seats onto.
    sill_d = (STACK_BACK_Y + 0.01) - FRAME_FRONT_Y
    frame.visual(
        Box((DOOR_W, sill_d, THRESHOLD_H)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + sill_d / 2.0, THRESHOLD_H / 2.0)),
        material=steel_black,
        name="threshold",
    )
    # Deep side guide channels just outside the leaf edges.
    channel_d = (STACK_BACK_Y + 0.01) - FRAME_FRONT_Y
    channel_y = FRAME_FRONT_Y + channel_d / 2.0
    channel_z = PANEL_BASE_Z + OPENING_H / 2.0
    frame.visual(
        Box((0.04, channel_d, OPENING_H)),
        origin=Origin(xyz=(-(OPENING_W / 2.0 + 0.02), channel_y, channel_z)),
        material=track_grey,
        name="guide_track_0",
    )
    frame.visual(
        Box((0.04, channel_d, OPENING_H)),
        origin=Origin(xyz=(OPENING_W / 2.0 + 0.02, channel_y, channel_z)),
        material=track_grey,
        name="guide_track_1",
    )

    # ----------------------------------------------------------- grille leaves
    # Each leaf is a thin perforated security grille with a regular grid of
    # square through-cut holes, flanked by two full-depth structural edge
    # rails that slide in the guide tracks and provide face-to-face contact
    # between adjacent leaves (as in a real telescoping security grille).
    # slat_0 (TOP) has field_h = SLAT_PITCH; lower leaves are one lap taller
    # with the extra strip extending above the band.
    rail_w = 0.028  # edge rail width (28 mm structural C-channel)
    rail_d = SLAT_D  # edge rail depth = full depth stagger
    for i in range(N_SLATS):
        slat = model.part(f"slat_{i}")
        field_h = SLAT_PITCH if i == 0 else SLAT_PITCH + SLAT_LAP
        field_cz = 0.0 if i == 0 else SLAT_LAP / 2.0

        grille_geom = _build_grille_mesh(field_h)
        # Shift the grille vertically so the lap strip (for non-top leaves)
        # extends above the band center.
        grille_geom.translate(0.0, 0.0, field_cz)
        grille_mesh = mesh_from_geometry(grille_geom, f"grille_{i}")

        slat.visual(
            grille_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=grille_metal,
            name="grille_panel",
        )
        # Full-depth structural edge rails at both sides. These provide the
        # sliding interface for the guide tracks and face-to-face contact
        # between adjacent leaves when telescoped.
        rail_x = SLAT_W / 2.0 - rail_w / 2.0
        slat.visual(
            Box((rail_w, rail_d, field_h)),
            origin=Origin(xyz=(-rail_x, 0.0, field_cz)),
            material=track_grey,
            name="edge_rail_0",
        )
        slat.visual(
            Box((rail_w, rail_d, field_h)),
            origin=Origin(xyz=(rail_x, 0.0, field_cz)),
            material=track_grey,
            name="edge_rail_1",
        )

    # Latch handle on the bottom leaf (slat_5), near its lower edge.
    bottom = model.get_part(f"slat_{N_SLATS - 1}")
    handle_y = -GRILLE_THICKNESS / 2.0 - 0.010  # stands proud of grille front face
    bottom.visual(
        Box((0.18, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, handle_y, -SLAT_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="latch_handle",
    )
    # Small mounting bosses anchoring the handle to the grille face.
    boss_y = -GRILLE_THICKNESS / 2.0 - 0.004
    bottom.visual(
        Box((0.02, 0.016, 0.024)),
        origin=Origin(xyz=(-0.07, boss_y, -SLAT_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="handle_boss_0",
    )
    bottom.visual(
        Box((0.02, 0.016, 0.024)),
        origin=Origin(xyz=(0.07, boss_y, -SLAT_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="handle_boss_1",
    )

    # ----------------------------------------------------------- articulation
    # slat_0 (TOP leaf) is FIXED to the frame at the top of the opening. Every
    # lower leaf is a PRISMATIC child of the leaf ABOVE it: joint frame one
    # pitch below and one thickness behind the parent, axis +Z, travel of one
    # pitch.
    model.articulation(
        "frame_to_slat_0",
        ArticulationType.FIXED,
        parent=frame,
        child="slat_0",
        origin=Origin(xyz=(0.0, _leaf_center_y(0), _band_center_z(0))),
    )
    lift_limits = MotionLimits(effort=400.0, velocity=0.30, lower=0.0, upper=SLAT_PITCH)
    for i in range(1, N_SLATS):
        model.articulation(
            f"slat_{i - 1}_to_slat_{i}",
            ArticulationType.PRISMATIC,
            parent=f"slat_{i - 1}",
            child=f"slat_{i}",
            origin=Origin(xyz=(0.0, SLAT_D, -SLAT_PITCH)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=lift_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    slats = [object_model.get_part(f"slat_{i}") for i in range(N_SLATS)]
    lift_joints = [
        object_model.get_articulation(f"slat_{i - 1}_to_slat_{i}") for i in range(1, N_SLATS)
    ]

    # --- Hero feature: 6 grille leaves fill the full opening height ---------
    top_aabb = ctx.part_element_world_aabb(slats[0], elem="grille_panel")
    bottom_aabb = ctx.part_element_world_aabb(slats[-1], elem="grille_panel")
    ctx.check(
        "grille leaf stack spans the full opening height",
        bottom_aabb is not None
        and top_aabb is not None
        and bottom_aabb[0][2] <= PANEL_BASE_Z + 0.01
        and top_aabb[1][2] >= PANEL_BASE_Z + OPENING_H - 0.01,
        details=f"bottom={bottom_aabb}, top={top_aabb}",
    )

    # --- Each grille is a thin perforated sheet, not a solid panel ----------
    for i, slat in enumerate(slats):
        grille_aabb = ctx.part_element_world_aabb(slat, elem="grille_panel")
        ctx.check(
            f"slat_{i} grille is a thin perforated sheet (not solid)",
            grille_aabb is not None
            and (grille_aabb[1][1] - grille_aabb[0][1]) < 0.012,
            details=f"y_extent={grille_aabb}",
        )

    # --- Frame surrounds the opening ----------------------------------------
    for i, slat in enumerate(slats):
        ctx.expect_within(
            slat,
            frame,
            axes="x",
            margin=0.0,
            inner_elem="grille_panel",
            name=f"slat_{i} grille fits within the frame opening width",
        )

    # --- Depth stagger: edge rails provide face-to-face contact chain --------
    # The full-depth structural edge rails on adjacent leaves touch face-to-face
    # (zero gap in Y), maintaining the telescoping support chain.
    for i in range(1, N_SLATS):
        ctx.expect_gap(
            slats[i],
            slats[i - 1],
            axis="y",
            positive_elem="edge_rail_0",
            negative_elem="edge_rail_0",
            min_gap=-0.001,
            max_gap=0.002,
            name=f"slat_{i} edge rail contacts slat_{i - 1} edge rail (face-to-face)",
        )

    # --- Closed laps: no see-through vertical gap between leaves ------------
    for i in range(1, N_SLATS):
        upper = ctx.part_element_world_aabb(slats[i - 1], elem="grille_panel")
        lower = ctx.part_element_world_aabb(slats[i], elem="grille_panel")
        ctx.check(
            f"slat_{i} lap tucks behind slat_{i - 1} with no vertical gap",
            upper is not None
            and lower is not None
            and lower[1][2] >= upper[0][2] + SLAT_LAP - 0.002,
            details=f"upper={upper}, lower={lower}",
        )

    # --- Bottom leaf seats on the sill --------------------------------------
    ctx.expect_gap(
        slats[-1],
        frame,
        axis="z",
        positive_elem="grille_panel",
        negative_elem="threshold",
        min_gap=-0.001,
        max_gap=0.01,
        name="bottom grille leaf seats on the floor sill",
    )

    # --- Latch handle is mounted on the bottom leaf -------------------------
    ctx.expect_contact(
        slats[-1],
        slats[-1],
        elem_a="latch_handle",
        elem_b="handle_boss_0",
        name="latch handle is anchored by its mounting boss",
    )

    # --- Articulation: telescoping lift -------------------------------------
    rest_positions = [ctx.part_world_position(s) for s in slats]
    open_pose = {j: SLAT_PITCH for j in lift_joints}
    with ctx.pose(open_pose):
        open_positions = [ctx.part_world_position(s) for s in slats]
        open_aabbs = [ctx.part_element_world_aabb(s, elem="grille_panel") for s in slats]

    # The fixed top leaf must not move; leaf i rises i pitches (chain sum).
    chain_ok = True
    for i in range(N_SLATS):
        if rest_positions[i] is None or open_positions[i] is None:
            chain_ok = False
            break
        rise = open_positions[i][2] - rest_positions[i][2]
        if abs(rise - SLAT_PITCH * i) > 1e-3:
            chain_ok = False
            break
    ctx.check(
        "top leaf stays fixed and each lower leaf rises one pitch per chain link",
        chain_ok,
        details=f"rest={rest_positions}, open={open_positions}",
    )
    # When fully open every leaf nests in the TOP band.
    stack_top = max(a[1][2] for a in open_aabbs if a is not None)
    stack_bottom = min(a[0][2] for a in open_aabbs if a is not None)
    ctx.check(
        "open stack stays inside the frame (no leaf above the header line)",
        len([a for a in open_aabbs if a is not None]) == N_SLATS
        and stack_top <= PANEL_BASE_Z + OPENING_H + SLAT_LAP + 0.002,
        details=f"stack_top={stack_top}, opening_top={PANEL_BASE_Z + OPENING_H}",
    )
    ctx.check(
        "open stack clears the opening below the top band",
        stack_bottom >= PANEL_BASE_Z + OPENING_H - SLAT_PITCH - 0.002,
        details=f"stack_bottom={stack_bottom}",
    )

    return ctx.report()


object_model = build_object_model()
