from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------------------------------------------------------------------------
# Articraft brief
# ---------------------------------------------------------------------------
# Object: a residential telescoping sectional garage door, ~2.4 m wide x
#   ~2.2 m tall, standing on the ground (z = 0 upward). A black steel surround
#   frame holds a stack of grey embossed horizontal panel leaves; a small latch
#   handle sits on the bottom leaf.
# Root/support: the black steel frame (two jambs, a front header fascia, a
#   floor sill, and two deep side guide channels) is the fixed root.
# Parts: frame (root) + 6 grey panel leaves. slat_0 is the TOP leaf and is
#   FIXED to the frame. Each lower leaf hangs off the leaf above it.
# Articulations: slat_0 is attached to the frame with a FIXED joint. Each
#   remaining leaf is a PRISMATIC child of the leaf above it, axis +Z,
#   travelling one slat pitch upward RELATIVE TO ITS PARENT. The leaves are
#   not coplanar: each lower leaf sits one panel thickness behind its parent
#   with their faces touching, so when raised every leaf telescopes up and
#   nests flat behind the leaf above it, the whole stack collecting behind the
#   fixed top leaf inside the frame (never above the header).
# Visible geometry: depth-staggered grey leaves inside a black frame, each leaf
#   featuring a horizontally ribbed corrugated profile (several thin parallel
#   horizontal rib bars proud of the front face), a seam groove per leaf, and a
#   small dark latch handle on the bottom leaf.
# Support/fit: each leaf sits in the frame opening with a small side
#   clearance; the side guide channels run the full stack depth. Closed leaves
#   overlap by a small lap: each leaf's top edge tucks behind the bottom edge
#   of the leaf above, so the closed door reads tight with no see-through gap.
# Intentional overlaps: adjacent leaves nest face-to-face when telescoped
#   (pillows/handle press against the neighbouring leaf), scoped per adjacent
#   pair with allow_overlap.
# Tests: door reads as full-height stack of 6 leaves, frame surrounds the
#   opening, leaves are depth-staggered face-to-face, closed laps leave no
#   vertical gap, the open pose telescopes the lower leaves up behind the
#   fixed top leaf without rising above the frame, the latch handle is mounted
#   on the bottom leaf.
# Assumptions: 6 equal leaves; a black powder-coated steel frame; mid-grey
#   embossed steel panels.
# ---------------------------------------------------------------------------

# --- Overall dimensions (meters) -------------------------------------------
# Reference is a near-square mid-grey sectional door with a SLIM black surround
# and ~6 flat embossed panels separated by clean horizontal grooves.
DOOR_W = 2.40  # overall frame outer width
JAMB_W = 0.06  # slim side jamb width (slim black surround)
JAMB_D = 0.12  # frame depth (front-to-back, along Y)
HEADER_H = 0.06  # slim top header height

OPENING_W = DOOR_W - 2.0 * JAMB_W  # clear opening width = 2.28
N_SLATS = 6  # ~6 horizontal panels, matching the reference photo
SLAT_PITCH = 0.355  # vertical band height owned by each leaf
SLAT_LAP = 0.020  # vertical lap: a leaf's top edge tucks behind the leaf above
SLAT_D = 0.04  # leaf thickness (along Y) = the per-leaf depth stagger
SLAT_W = OPENING_W - 0.012  # leaf panel width, small side clearance

THRESHOLD_H = 0.05  # slim floor sill / bottom surround rail
PANEL_BASE_Z = THRESHOLD_H  # bottom of the lowest leaf sits on the sill

OPENING_H = N_SLATS * SLAT_PITCH  # 2.13 m clear panel stack
FRAME_OUTER_H = PANEL_BASE_Z + OPENING_H + HEADER_H  # overall frame height

# Leaves sit slightly recessed behind the front face of the frame. slat_0 (the
# fixed TOP leaf) is the front-most; each lower leaf sits one leaf thickness
# further back, faces touching, so the stack telescopes cleanly.
FRAME_FRONT_Y = -JAMB_D / 2.0
PANEL_FRONT_Y = FRAME_FRONT_Y + 0.02  # top leaf front face recessed 0.02 m
STACK_BACK_Y = PANEL_FRONT_Y + N_SLATS * SLAT_D  # back face of the deepest leaf


def _band_center_z(i: int) -> float:
    """World z of the center of leaf i's vertical band (i = 0 is the TOP leaf)."""
    return PANEL_BASE_Z + OPENING_H - SLAT_PITCH * (i + 0.5)


def _leaf_center_y(i: int) -> float:
    """World y of the center of leaf i (each leaf one thickness behind its parent)."""
    return PANEL_FRONT_Y + SLAT_D * i + SLAT_D / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sectional_garage_door")

    # Flat matte palette: a slim near-black surround, flat mid-grey panels, and
    # a slightly darker grey for the recessed grooves between the panels.
    steel_black = model.material("powdercoat_black_steel", rgba=(0.09, 0.09, 0.095, 1.0))
    panel_grey = model.material("embossed_grey_steel", rgba=(0.50, 0.505, 0.51, 1.0))
    panel_rib = model.material("panel_rib_grey", rgba=(0.55, 0.555, 0.56, 1.0))
    groove_grey = model.material("panel_groove_grey", rgba=(0.30, 0.305, 0.31, 1.0))
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
    # Front header fascia spanning between the jambs. It sits at the FRONT of
    # the frame depth only, leaving clearance behind it so the telescoped
    # leaves (whose laps rise slightly past the opening top) never hit it.
    header_d = 0.055
    frame.visual(
        Box((DOOR_W, header_d, HEADER_H)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + header_d / 2.0, PANEL_BASE_Z + OPENING_H + HEADER_H / 2.0)),
        material=steel_black,
        name="header",
    )
    # Floor sill the bottom leaf seats onto. It extends rearward under the
    # whole depth-staggered stack so the deepest leaf still lands on steel.
    sill_d = (STACK_BACK_Y + 0.01) - FRAME_FRONT_Y
    frame.visual(
        Box((DOOR_W, sill_d, THRESHOLD_H)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + sill_d / 2.0, THRESHOLD_H / 2.0)),
        material=steel_black,
        name="threshold",
    )
    # Deep side guide channels just outside the leaf edges (inside the jamb
    # line). They run the full opening height and the full stack depth, so
    # every depth-staggered leaf edge rides in a channel both closed and open.
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

    # ---------------------------------------------------------------- leaves
    # Each leaf is authored with its part origin at the center of its
    # closed-pose vertical BAND (and at its own depth plane). Lower leaves are
    # one lap taller than their band: the extra strip extends above the band,
    # tucked behind the bottom edge of the leaf above, so the closed door has
    # no see-through gap while the leaves stay in separate depth planes.
    rib_inset_x = 0.04  # ribs stop short of the panel side edges
    rib_inset_z = 0.045  # ...and short of the top/bottom band edges
    groove_h = 0.018  # height of the recessed seam groove
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
        # Horizontally ribbed corrugated profile: several thin parallel
        # horizontal rib bars proud of the front face, replacing the single
        # flat embossed pillow. Ribs are evenly spaced within the visible band
        # and stay inside the insets so they never reach the lap strip hidden
        # behind the leaf above.
        n_ribs = 5
        rib_h = 0.016  # height of each horizontal rib bar
        rib_proud = 0.007  # how far each rib stands proud of the front face
        rib_w = SLAT_W - 2.0 * rib_inset_x  # rib spans the inset field width
        rib_band = SLAT_PITCH - 2.0 * rib_inset_z  # usable band for ribs
        rib_spacing = (rib_band - n_ribs * rib_h) / max(n_ribs - 1, 1)
        rib_base_z = -rib_band / 2.0 + rib_h / 2.0  # center z of the lowest rib
        rib_y = -SLAT_D / 2.0 - rib_proud / 2.0  # rib sits proud of front face
        for r in range(n_ribs):
            rib_cz = rib_base_z + r * (rib_h + rib_spacing)
            slat.visual(
                Box((rib_w, rib_proud, rib_h)),
                origin=Origin(xyz=(0.0, rib_y, rib_cz)),
                material=panel_rib,
                name=f"rib_{r}",
            )
        # Recessed dark groove along the bottom edge of the leaf forming the
        # clean horizontal seam line above the leaf below.
        slat.visual(
            Box((SLAT_W, 0.018, groove_h)),
            origin=Origin(xyz=(0.0, -SLAT_D / 2.0 + 0.018 / 2.0 - 0.002, -SLAT_PITCH / 2.0 + groove_h / 2.0)),
            material=groove_grey,
            name="seam_groove",
        )

    # Latch handle on the bottom leaf (slat_5, the deepest), near its lower edge.
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

    # --- Hero feature: 6 leaves fill the full opening height ----------------
    # slat_0 is the fixed TOP leaf; slat_5 is the bottom leaf on the sill.
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

    # --- Frame surrounds the opening ---------------------------------------
    # Each leaf sits horizontally within the clear opening (inside the jambs).
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
    # Each lower leaf's top edge (its lap strip) reaches up past the bottom
    # edge of the leaf above, in the depth plane behind it.
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

    # --- Corrugation ribs: each leaf has multiple horizontal rib bars --------
    # Verify ribs exist on every leaf and stand proud of the panel front face.
    for i, slat in enumerate(slats):
        field_aabb = ctx.part_element_world_aabb(slat, elem="panel_field")
        rib_aabb = ctx.part_element_world_aabb(slat, elem="rib_0")
        ctx.check(
            f"slat_{i} has corrugation ribs proud of the panel front face",
            field_aabb is not None
            and rib_aabb is not None
            and rib_aabb[0][1] < field_aabb[0][1] - 0.002,
            details=f"field_front_y={field_aabb[0][1] if field_aabb else None}, "
            f"rib_front_y={rib_aabb[0][1] if rib_aabb else None}",
        )
        # Verify the topmost rib exists too (confirms multiple ribs per leaf).
        top_rib_aabb = ctx.part_element_world_aabb(slat, elem="rib_4")
        ctx.check(
            f"slat_{i} has multiple horizontal ribs (rib_0 through rib_4)",
            rib_aabb is not None
            and top_rib_aabb is not None
            and top_rib_aabb[1][2] > rib_aabb[1][2] + 0.02,
            details=f"rib_0_top={rib_aabb[1][2] if rib_aabb else None}, "
            f"rib_4_top={top_rib_aabb[1][2] if top_rib_aabb else None}",
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
    # above the frame (only the hidden lap strips pass the opening top), and
    # the rest of the opening below is cleared.
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
    # Adjacent leaves deliberately press face-to-face: the corrugation ribs and
    # the latch handle of a leaf seat against the back of the leaf in front of
    # it when the door telescopes open.
    for i in range(1, N_SLATS):
        ctx.allow_overlap(
            slats[i - 1],
            slats[i],
            reason=(
                "Telescoping garage door: each leaf nests flat against the back of the "
                "leaf above it when raised; corrugation ribs/handle press into that contact plane."
            ),
        )

    return ctx.report()


object_model = build_object_model()
