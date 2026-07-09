from __future__ import annotations

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
# Object: a residential telescoping sectional garage door, ~2.46 m wide x
#   ~2.24 m tall, standing on the ground (z = 0 upward). Bold 90 mm
#   square-tube black steel guide rails stand proud on each side of the
#   opening, connected by a header beam across the top and a deep sill at the
#   bottom. Six grey embossed horizontal panel leaves stack inside the
#   opening; a small latch handle sits on the bottom leaf.
# Root/support: the black steel frame (two hollow square-tube guide rails, a
#   header beam, a floor sill, and two visible track channels on the rail
#   inner faces) is the fixed root.
# Parts: frame (root) + 6 grey panel leaves. slat_0 is the TOP leaf and is
#   FIXED to the frame. Each lower leaf hangs off the leaf above it.
# Articulations: slat_0 is attached to the frame with a FIXED joint. Each
#   remaining leaf is a PRISMATIC child of the leaf above it, axis +Z,
#   travelling one slat pitch upward RELATIVE TO ITS PARENT. The leaves are
#   not coplanar: each lower leaf sits one panel thickness behind its parent
#   with their faces touching, so when raised every leaf telescopes up and
#   nests flat behind the leaf above it, the whole stack collecting behind the
#   fixed top leaf inside the frame (never above the header).
# Visible geometry: depth-staggered grey leaves inside bold square-tube guide
#   rails, raised embossed pillows, a seam groove per leaf, visible track
#   channels on the rail inner faces, and a small dark latch handle on the
#   bottom leaf.
# Support/fit: each leaf sits in the frame opening with a small side
#   clearance; the track channels on the rail inner faces guide the leaf
#   edges. Closed leaves overlap by a small lap: each leaf's top edge tucks
#   behind the bottom edge of the leaf above.
# Intentional overlaps: adjacent leaves nest face-to-face when telescoped
#   (pillows/handle press against the neighbouring leaf), scoped per adjacent
#   pair with allow_overlap.
# Tests: bold guide rails flank the opening and stand proud of the panel
#   front face, track channels are visible on inner rail faces, door reads as
#   full-height stack of 6 leaves, leaves are depth-staggered face-to-face,
#   closed laps leave no vertical gap, the open pose telescopes the lower
#   leaves up behind the fixed top leaf, the latch handle is mounted on the
#   bottom leaf.
# Assumptions: 6 equal leaves; 90 mm square-tube black powder-coated steel
#   guide rails; mid-grey embossed steel panels.
# ---------------------------------------------------------------------------

# --- Overall dimensions (meters) -------------------------------------------
OPENING_W = 2.28          # clear opening width between the guide rails
RAIL_SIZE = 0.09          # bold 90 mm square-tube guide rail cross section
RAIL_WALL = 0.005         # 5 mm tube wall thickness (visible at cut ends)
DOOR_W = OPENING_W + 2.0 * RAIL_SIZE  # overall width including rails = 2.46

N_SLATS = 6               # ~6 horizontal panels, matching the reference
SLAT_PITCH = 0.355        # vertical band height owned by each leaf
SLAT_LAP = 0.020          # vertical lap: leaf top tucks behind leaf above
SLAT_D = 0.04             # leaf thickness (along Y) = per-leaf depth stagger
SLAT_W = OPENING_W - 0.012  # leaf panel width, small side clearance

HEADER_H = 0.06           # header beam height
THRESHOLD_H = 0.05        # floor sill height
PANEL_BASE_Z = THRESHOLD_H  # bottom of lowest leaf sits on the sill

OPENING_H = N_SLATS * SLAT_PITCH  # 2.13 m clear panel stack
FRAME_OUTER_H = PANEL_BASE_Z + OPENING_H + HEADER_H  # overall frame height

# Y layout: rails stand 20 mm proud of the panel front face
PANEL_FRONT_Y = -0.04                          # front-most panel face
RAIL_FRONT_Y = PANEL_FRONT_Y - 0.020           # rails project forward
RAIL_CENTER_Y = RAIL_FRONT_Y + RAIL_SIZE / 2.0 # rail depth center
STACK_BACK_Y = PANEL_FRONT_Y + N_SLATS * SLAT_D  # back of deepest leaf

# Rail height and Z center (spans sill to header top)
RAIL_HEIGHT = OPENING_H + HEADER_H + THRESHOLD_H
RAIL_CENTER_Z = PANEL_BASE_Z + OPENING_H / 2.0 + (HEADER_H - THRESHOLD_H) / 2.0


def _band_center_z(i: int) -> float:
    """World z of the center of leaf i's vertical band (i = 0 is the TOP leaf)."""
    return PANEL_BASE_Z + OPENING_H - SLAT_PITCH * (i + 0.5)


def _leaf_center_y(i: int) -> float:
    """World y of the center of leaf i (each leaf one thickness behind its parent)."""
    return PANEL_FRONT_Y + SLAT_D * i + SLAT_D / 2.0


def _build_hollow_rail(height: float) -> ExtrudeWithHolesGeometry:
    """Hollow square-tube cross section extruded vertically along Z."""
    hs = RAIL_SIZE / 2.0
    outer = [(-hs, -hs), (hs, -hs), (hs, hs), (-hs, hs)]
    hi = (RAIL_SIZE - 2.0 * RAIL_WALL) / 2.0
    inner = [(-hi, -hi), (hi, -hi), (hi, hi), (-hi, hi)]
    return ExtrudeWithHolesGeometry(outer, [inner], height, center=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sectional_garage_door")

    # Flat matte palette: black steel frame, mid-grey panels, darker grooves,
    # galvanized track channels, dark handle metal.
    steel_black = model.material("powdercoat_black_steel", rgba=(0.09, 0.09, 0.095, 1.0))
    panel_grey = model.material("embossed_grey_steel", rgba=(0.50, 0.505, 0.51, 1.0))
    panel_grey_pillow = model.material("panel_pillow_grey", rgba=(0.54, 0.545, 0.55, 1.0))
    groove_grey = model.material("panel_groove_grey", rgba=(0.30, 0.305, 0.31, 1.0))
    track_steel = model.material("galvanized_track_steel", rgba=(0.35, 0.36, 0.38, 1.0))
    handle_metal = model.material("dark_handle_metal", rgba=(0.13, 0.13, 0.14, 1.0))

    # ---------------------------------------------------------------- frame
    # The frame is the fixed root: two bold hollow square-tube guide rails,
    # a header beam across the top, a deep floor sill, and two visible track
    # channels on the inner faces of the rails.
    frame = model.part("frame")

    # Bold hollow square-tube guide rails (one on each side)
    for i in range(2):
        rail_geom = _build_hollow_rail(RAIL_HEIGHT)
        rail_mesh = mesh_from_geometry(rail_geom, f"guide_rail_{i}")
        sign = -1 if i == 0 else 1
        rail_x = sign * (OPENING_W / 2.0 + RAIL_SIZE / 2.0)
        frame.visual(
            rail_mesh,
            origin=Origin(xyz=(rail_x, RAIL_CENTER_Y, RAIL_CENTER_Z)),
            material=steel_black,
            name=f"guide_rail_{i}",
        )

    # Track channels: visible galvanized steel strips on the inner face of
    # each rail where the panel edges ride. Each strip sits flush with the
    # rail inner face, protruding slightly into the opening.
    track_slot_d = 0.006   # strip thickness (X direction)
    track_slot_w = 0.050   # strip width on the rail face (Y direction)
    for i in range(2):
        sign = -1 if i == 0 else 1
        # Place the strip so its rail-side face is flush with the rail inner
        # face and it protrudes track_slot_d into the opening.
        track_x = sign * (OPENING_W / 2.0 - track_slot_d / 2.0)
        frame.visual(
            Box((track_slot_d, track_slot_w, OPENING_H)),
            origin=Origin(xyz=(track_x, RAIL_CENTER_Y, PANEL_BASE_Z + OPENING_H / 2.0)),
            material=track_steel,
            name=f"track_channel_{i}",
        )

    # Header beam spanning the full door width across the rail tops.
    header_d = RAIL_SIZE
    frame.visual(
        Box((DOOR_W, header_d, HEADER_H)),
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, PANEL_BASE_Z + OPENING_H + HEADER_H / 2.0)),
        material=steel_black,
        name="header",
    )

    # Floor sill spanning the full door width, extending from the rail front
    # face rearward past the deepest leaf so every leaf lands on steel.
    sill_d = (STACK_BACK_Y + 0.01) - RAIL_FRONT_Y
    frame.visual(
        Box((DOOR_W, sill_d, THRESHOLD_H)),
        origin=Origin(xyz=(0.0, RAIL_FRONT_Y + sill_d / 2.0, THRESHOLD_H / 2.0)),
        material=steel_black,
        name="threshold",
    )

    # ---------------------------------------------------------------- leaves
    # Each leaf is authored with its part origin at the center of its
    # closed-pose vertical BAND (and at its own depth plane). Lower leaves are
    # one lap taller than their band: the extra strip extends above the band,
    # tucked behind the bottom edge of the leaf above, so the closed door has
    # no see-through gap while the leaves stay in separate depth planes.
    pillow_inset_x = 0.04   # raised field stops short of panel side edges
    pillow_inset_z = 0.045  # ...and short of the top/bottom band edges
    groove_h = 0.018        # height of the recessed seam groove
    for i in range(N_SLATS):
        slat = model.part(f"slat_{i}")
        field_h = SLAT_PITCH if i == 0 else SLAT_PITCH + SLAT_LAP
        field_cz = 0.0 if i == 0 else SLAT_LAP / 2.0
        # Main flat panel field (the full pressed-steel leaf).
        slat.visual(
            Box((SLAT_W, SLAT_D, field_h)),
            origin=Origin(xyz=(0.0, 0.0, field_cz)),
            material=panel_grey,
            name="panel_field",
        )
        # Subtly raised central pillow spanning most of the visible band.
        pillow_y = -SLAT_D / 2.0 - 0.004  # proud of the front face by ~4 mm
        slat.visual(
            Box((SLAT_W - 2.0 * pillow_inset_x, 0.014, SLAT_PITCH - 2.0 * pillow_inset_z)),
            origin=Origin(xyz=(0.0, pillow_y, 0.0)),
            material=panel_grey_pillow,
            name="panel_pillow",
        )
        # Recessed dark groove along the bottom edge forming the seam line.
        slat.visual(
            Box((SLAT_W, 0.018, groove_h)),
            origin=Origin(xyz=(0.0, -SLAT_D / 2.0 + 0.018 / 2.0 - 0.002, -SLAT_PITCH / 2.0 + groove_h / 2.0)),
            material=groove_grey,
            name="seam_groove",
        )

    # Latch handle on the bottom leaf (slat_5), near its lower edge.
    bottom = model.get_part(f"slat_{N_SLATS - 1}")
    handle_y = -SLAT_D / 2.0 - 0.018  # stands proud of the leaf front face
    bottom.visual(
        Box((0.18, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, handle_y, -SLAT_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="latch_handle",
    )
    # Small mounting bosses anchoring the handle to the panel face (no float).
    bottom.visual(
        Box((0.02, 0.02, 0.024)),
        origin=Origin(xyz=(-0.07, -SLAT_D / 2.0 - 0.008, -SLAT_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="handle_boss_0",
    )
    bottom.visual(
        Box((0.02, 0.02, 0.024)),
        origin=Origin(xyz=(0.07, -SLAT_D / 2.0 - 0.008, -SLAT_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="handle_boss_1",
    )

    # ----------------------------------------------------------- articulation
    # slat_0 (TOP leaf) is FIXED to the frame at the top of the opening. Every
    # lower leaf is a PRISMATIC child of the leaf ABOVE it: joint frame one
    # pitch below and one thickness behind the parent, axis +Z, travel of one
    # pitch. Driving a joint to its upper limit slides that leaf up RELATIVE TO
    # ITS PARENT until it nests flat against the parent's back face; with the
    # whole chain raised the leaves telescope into a tight depth-wise stack
    # behind the fixed top leaf, never rising above the frame header.
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

    # --- Hero feature: bold square-tube guide rails -------------------------
    rail_0_aabb = ctx.part_element_world_aabb(frame, elem="guide_rail_0")
    rail_1_aabb = ctx.part_element_world_aabb(frame, elem="guide_rail_1")

    # Rails are chunky box sections (at least 80 mm cross section width)
    ctx.check(
        "bold square-tube guide rails are chunky (>=80mm cross section)",
        rail_0_aabb is not None
        and rail_1_aabb is not None
        and (rail_0_aabb[1][0] - rail_0_aabb[0][0]) >= 0.080
        and (rail_1_aabb[1][0] - rail_1_aabb[0][0]) >= 0.080,
        details=f"rail_0_x={rail_0_aabb}, rail_1_x={rail_1_aabb}",
    )

    # Rails flank the opening (left rail at negative X, right rail at positive X)
    ctx.check(
        "guide rails flank the opening on each side",
        rail_0_aabb is not None
        and rail_1_aabb is not None
        and rail_0_aabb[1][0] < 0.0
        and rail_1_aabb[0][0] > 0.0,
        details=f"rail_0={rail_0_aabb}, rail_1={rail_1_aabb}",
    )

    # Rails stand proud of the panel front face (rail front Y < panel front Y)
    panel_front_aabb = ctx.part_element_world_aabb(slats[0], elem="panel_field")
    ctx.check(
        "guide rails stand proud of the panel front face",
        rail_0_aabb is not None
        and panel_front_aabb is not None
        and rail_0_aabb[0][1] < panel_front_aabb[0][1] - 0.010,
        details=(
            f"rail_front_y={rail_0_aabb[0][1] if rail_0_aabb else None}, "
            f"panel_front_y={panel_front_aabb[0][1] if panel_front_aabb else None}"
        ),
    )

    # Rails are also deep (square cross section: depth ~= width)
    ctx.check(
        "guide rails have square-tube cross section (depth ~= width)",
        rail_0_aabb is not None
        and (rail_0_aabb[1][1] - rail_0_aabb[0][1]) >= 0.080,
        details=f"rail_0_y_extent={rail_0_aabb}",
    )

    # --- Track channels visible on inner rail faces ------------------------
    track_0_aabb = ctx.part_element_world_aabb(frame, elem="track_channel_0")
    track_1_aabb = ctx.part_element_world_aabb(frame, elem="track_channel_1")
    ctx.check(
        "track channels visible on guide rail inner faces",
        track_0_aabb is not None and track_1_aabb is not None,
        details=f"track_0={track_0_aabb}, track_1={track_1_aabb}",
    )
    # Track sits on the rail inner face, protruding slightly into the opening
    if track_0_aabb is not None and rail_0_aabb is not None:
        ctx.check(
            "left track channel is on the inner face of the left rail",
            abs(track_0_aabb[0][0] - rail_0_aabb[1][0]) < 0.002
            and track_0_aabb[1][0] > rail_0_aabb[1][0],
            details=f"track_0={track_0_aabb}, rail_0={rail_0_aabb}",
        )

    # --- 6 leaves fill the full opening height ------------------------------
    top_aabb = ctx.part_element_world_aabb(slats[0], elem="panel_field")
    bottom_aabb = ctx.part_element_world_aabb(slats[-1], elem="panel_field")
    ctx.check(
        "leaf stack spans the full opening height",
        bottom_aabb is not None
        and top_aabb is not None
        and bottom_aabb[0][2] <= PANEL_BASE_Z + 0.01
        and top_aabb[1][2] >= PANEL_BASE_Z + OPENING_H - 0.01,
        details=f"bottom={bottom_aabb}, top={top_aabb}",
    )

    # --- Each leaf fits within the frame opening width ----------------------
    for i, slat in enumerate(slats):
        ctx.expect_within(
            slat,
            frame,
            axes="x",
            margin=0.0,
            inner_elem="panel_field",
            name=f"slat_{i} panel fits within the frame opening width",
        )

    # --- Depth stagger: each lower leaf sits flat against its parent's back --
    for i in range(1, N_SLATS):
        ctx.expect_gap(
            slats[i],
            slats[i - 1],
            axis="y",
            positive_elem="panel_field",
            negative_elem="panel_field",
            min_gap=-0.001,
            max_gap=0.002,
            name=f"slat_{i} rides face-to-face one thickness behind slat_{i - 1}",
        )

    # --- Closed laps: no see-through vertical gap between leaves ------------
    for i in range(1, N_SLATS):
        upper = ctx.part_element_world_aabb(slats[i - 1], elem="panel_field")
        lower = ctx.part_element_world_aabb(slats[i], elem="panel_field")
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
        positive_elem="panel_field",
        negative_elem="threshold",
        min_gap=-0.001,
        max_gap=0.01,
        name="bottom leaf seats on the floor sill",
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
        open_aabbs = [ctx.part_element_world_aabb(s, elem="panel_field") for s in slats]

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
    # When fully open every leaf nests in the TOP band: the stack never rises
    # above the frame, and the rest of the opening below is cleared.
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

    # --- Intended overlaps between adjacent nesting leaves ------------------
    for i in range(1, N_SLATS):
        ctx.allow_overlap(
            slats[i - 1],
            slats[i],
            reason=(
                "Telescoping garage door: each leaf nests flat against the back of the "
                "leaf above it when raised; pillows/handle press into that contact plane."
            ),
        )

    return ctx.report()


object_model = build_object_model()
