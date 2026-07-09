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
# Object: a residential sectional garage door, ~2.4 m wide x ~2.2 m tall,
#   standing on the ground (z = 0 upward). A black steel surround frame holds
#   three tall grey raised-panel section leaves; a small latch handle sits on
#   the bottom section.
# Root/support: the black steel frame (two jambs, a front header fascia, a
#   floor sill, and two deep side guide channels) is the fixed root.
# Parts: frame (root) + 3 raised-panel section leaves. section_0 is the TOP
#   leaf and is FIXED to the frame. Each lower leaf hangs off the leaf above.
# Articulations: section_0 is FIXED to the frame. Each remaining section is a
#   PRISMATIC child of the section above it, axis +Z, travelling one section
#   pitch upward RELATIVE TO ITS PARENT. The sections are depth-staggered:
#   each lower section sits one panel thickness behind its parent with faces
#   touching, so when raised every section telescopes up and nests flat behind
#   the section above it, the whole stack collecting behind the fixed top
#   section inside the frame.
# Visible geometry: depth-staggered grey sections inside a black frame, each
#   section has a RECESSED BORDER with a RAISED central field (raised-panel
#   embossing) instead of the parent's flat pillow, plus a seam groove per
#   section and a small dark latch handle on the bottom section.
# Support/fit: each section sits in the frame opening with a small side
#   clearance; the side guide channels run the full stack depth.
# Intentional overlaps: adjacent sections nest face-to-face when telescoped
#   (raised fields press against the neighbouring section), scoped per
#   adjacent pair with allow_overlap.
# Tests: 3 sections fill the full opening, frame surrounds the opening,
#   sections are depth-staggered, closed laps leave no vertical gap, open pose
#   telescopes lower sections up behind fixed top section without rising above
#   the frame, raised-field geometry is present on every section, latch handle
#   is mounted on the bottom section.
# Assumptions: 3 equal sections; black powder-coated steel frame; mid-grey
#   raised-panel steel sections.
# ---------------------------------------------------------------------------

# --- Overall dimensions (meters) -------------------------------------------
DOOR_W = 2.40  # overall frame outer width
JAMB_W = 0.06  # slim side jamb width
JAMB_D = 0.12  # frame depth (front-to-back, along Y)
HEADER_H = 0.06  # slim top header height

OPENING_W = DOOR_W - 2.0 * JAMB_W  # clear opening width = 2.28
N_SECTIONS = 3  # three tall raised-panel sections
SECTION_PITCH = 0.71  # vertical band height per section (3 * 0.71 = 2.13 m)
SECTION_LAP = 0.025  # vertical lap: a section's top edge tucks behind the one above
SECTION_D = 0.04  # section thickness (along Y) = per-section depth stagger
SECTION_W = OPENING_W - 0.012  # section panel width, small side clearance

THRESHOLD_H = 0.05  # slim floor sill / bottom surround rail
PANEL_BASE_Z = THRESHOLD_H  # bottom of the lowest section sits on the sill

OPENING_H = N_SECTIONS * SECTION_PITCH  # 2.13 m clear panel stack
FRAME_OUTER_H = PANEL_BASE_Z + OPENING_H + HEADER_H  # overall frame height

# Sections sit slightly recessed behind the front face of the frame. section_0
# (the fixed TOP section) is the front-most; each lower section sits one
# thickness further back, faces touching, so the stack telescopes cleanly.
FRAME_FRONT_Y = -JAMB_D / 2.0
PANEL_FRONT_Y = FRAME_FRONT_Y + 0.02  # top section front face recessed 0.02 m
STACK_BACK_Y = PANEL_FRONT_Y + N_SECTIONS * SECTION_D  # back of deepest section


def _section_center_z(i: int) -> float:
    """World z of section i center (i = 0 is the TOP/fixed section)."""
    return PANEL_BASE_Z + OPENING_H - SECTION_PITCH * (i + 0.5)


def _section_center_y(i: int) -> float:
    """World y of section i (each one thickness behind its parent)."""
    return PANEL_FRONT_Y + SECTION_D * i + SECTION_D / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sectional_garage_door")

    # --- Materials ---------------------------------------------------------
    steel_black = model.material("powdercoat_black_steel", rgba=(0.09, 0.09, 0.095, 1.0))
    panel_grey = model.material("section_panel_grey", rgba=(0.50, 0.505, 0.51, 1.0))
    raised_field_grey = model.material("raised_field_grey", rgba=(0.56, 0.565, 0.57, 1.0))
    groove_grey = model.material("panel_groove_grey", rgba=(0.30, 0.305, 0.31, 1.0))
    track_grey = model.material("galvanized_track", rgba=(0.40, 0.41, 0.42, 1.0))
    handle_metal = model.material("dark_handle_metal", rgba=(0.13, 0.13, 0.14, 1.0))

    # ---------------------------------------------------------------- frame
    # The frame is the fixed root: two side jambs, a front header fascia, a
    # floor sill, and two deep side guide channels the section edges ride in.
    frame = model.part("frame")

    jamb_full_h = OPENING_H + HEADER_H + THRESHOLD_H
    frame.visual(
        Box((JAMB_W, JAMB_D, jamb_full_h)),
        origin=Origin(xyz=(-(OPENING_W + JAMB_W) / 2.0, 0.0,
                           PANEL_BASE_Z + OPENING_H / 2.0 + (HEADER_H - THRESHOLD_H) / 2.0)),
        material=steel_black,
        name="jamb_0",
    )
    frame.visual(
        Box((JAMB_W, JAMB_D, jamb_full_h)),
        origin=Origin(xyz=((OPENING_W + JAMB_W) / 2.0, 0.0,
                           PANEL_BASE_Z + OPENING_H / 2.0 + (HEADER_H - THRESHOLD_H) / 2.0)),
        material=steel_black,
        name="jamb_1",
    )
    # Front header fascia spanning between the jambs.
    header_d = 0.055
    frame.visual(
        Box((DOOR_W, header_d, HEADER_H)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + header_d / 2.0,
                           PANEL_BASE_Z + OPENING_H + HEADER_H / 2.0)),
        material=steel_black,
        name="header",
    )
    # Floor sill the bottom section seats onto.
    sill_d = (STACK_BACK_Y + 0.01) - FRAME_FRONT_Y
    frame.visual(
        Box((DOOR_W, sill_d, THRESHOLD_H)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + sill_d / 2.0, THRESHOLD_H / 2.0)),
        material=steel_black,
        name="threshold",
    )
    # Deep side guide channels just outside the section edges.
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

    # -------------------------------------------------------------- sections
    # Each section is a tall raised-panel leaf. The panel_body represents the
    # recessed border surface; the raised_field sits proud of the front face,
    # creating the classic raised-panel embossing. The visible border width
    # is the exposed panel_body surface surrounding the raised field.
    border_inset_x = 0.065  # raised field inset from side edges (stile width)
    border_inset_z = 0.060  # raised field inset from top/bottom band edges (rail width)
    raised_proud = 0.009  # raised field protrudes 9 mm past the border surface
    groove_h = 0.018  # height of the recessed seam groove

    for i in range(N_SECTIONS):
        section = model.part(f"section_{i}")

        # Full section panel body (the recessed border-level surface).
        # Lower sections are one lap taller: the extra strip extends above
        # the band, tucked behind the bottom edge of the section above.
        field_h = SECTION_PITCH if i == 0 else SECTION_PITCH + SECTION_LAP
        field_cz = 0.0 if i == 0 else SECTION_LAP / 2.0
        section.visual(
            Box((SECTION_W, SECTION_D, field_h)),
            origin=Origin(xyz=(0.0, 0.0, field_cz)),
            material=panel_grey,
            name="panel_body",
        )

        # Raised central field proud of the front face. This creates the
        # raised-panel look: the border (exposed panel_body) is recessed
        # relative to this raised center field.
        raised_w = SECTION_W - 2.0 * border_inset_x
        raised_h = SECTION_PITCH - 2.0 * border_inset_z
        raised_y = -SECTION_D / 2.0 - raised_proud / 2.0
        section.visual(
            Box((raised_w, raised_proud, raised_h)),
            origin=Origin(xyz=(0.0, raised_y, 0.0)),
            material=raised_field_grey,
            name="raised_field",
        )

        # Recessed dark groove along the bottom edge forming the horizontal
        # seam line above the section below.
        section.visual(
            Box((SECTION_W, 0.018, groove_h)),
            origin=Origin(xyz=(0.0, -SECTION_D / 2.0 + 0.018 / 2.0 - 0.002,
                               -SECTION_PITCH / 2.0 + groove_h / 2.0)),
            material=groove_grey,
            name="seam_groove",
        )

    # Latch handle on the bottom section (section_2), near its lower edge.
    bottom = model.get_part(f"section_{N_SECTIONS - 1}")
    handle_y = -SECTION_D / 2.0 - 0.018  # stands proud of the section front face
    bottom.visual(
        Box((0.18, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, handle_y, -SECTION_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="latch_handle",
    )
    # Small mounting bosses anchoring the handle to the panel face.
    bottom.visual(
        Box((0.02, 0.02, 0.024)),
        origin=Origin(xyz=(-0.07, -SECTION_D / 2.0 - 0.008, -SECTION_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="handle_boss_0",
    )
    bottom.visual(
        Box((0.02, 0.02, 0.024)),
        origin=Origin(xyz=(0.07, -SECTION_D / 2.0 - 0.008, -SECTION_PITCH / 2.0 + 0.06)),
        material=handle_metal,
        name="handle_boss_1",
    )

    # ----------------------------------------------------------- articulation
    # section_0 (TOP leaf) is FIXED to the frame. Every lower section is a
    # PRISMATIC child of the section ABOVE it: joint frame one pitch below and
    # one thickness behind the parent, axis +Z, travel of one pitch.
    model.articulation(
        "frame_to_section_0",
        ArticulationType.FIXED,
        parent=frame,
        child="section_0",
        origin=Origin(xyz=(0.0, _section_center_y(0), _section_center_z(0))),
    )
    lift_limits = MotionLimits(effort=400.0, velocity=0.30, lower=0.0, upper=SECTION_PITCH)
    for i in range(1, N_SECTIONS):
        model.articulation(
            f"section_{i - 1}_to_section_{i}",
            ArticulationType.PRISMATIC,
            parent=f"section_{i - 1}",
            child=f"section_{i}",
            origin=Origin(xyz=(0.0, SECTION_D, -SECTION_PITCH)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=lift_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sections = [object_model.get_part(f"section_{i}") for i in range(N_SECTIONS)]
    lift_joints = [
        object_model.get_articulation(f"section_{i - 1}_to_section_{i}")
        for i in range(1, N_SECTIONS)
    ]

    # --- Hero feature: 3 tall sections fill the full opening height --------
    top_aabb = ctx.part_element_world_aabb(sections[0], elem="panel_body")
    bottom_aabb = ctx.part_element_world_aabb(sections[-1], elem="panel_body")
    ctx.check(
        "section stack spans the full opening height",
        bottom_aabb is not None
        and top_aabb is not None
        and bottom_aabb[0][2] <= PANEL_BASE_Z + 0.01
        and top_aabb[1][2] >= PANEL_BASE_Z + OPENING_H - 0.01,
        details=f"bottom={bottom_aabb}, top={top_aabb}",
    )

    # --- Each section is tall (about 0.71 m each, ~3x parent slat) -------
    for i, section in enumerate(sections):
        section_aabb = ctx.part_element_world_aabb(section, elem="panel_body")
        ctx.check(
            f"section_{i} is a tall panel (>= 0.65 m band height)",
            section_aabb is not None
            and (section_aabb[1][2] - section_aabb[0][2]) >= 0.65,
            details=f"aabb={section_aabb}",
        )

    # --- Frame surrounds the opening ---------------------------------------
    for i, section in enumerate(sections):
        ctx.expect_within(
            section,
            frame,
            axes="x",
            margin=0.0,
            inner_elem="panel_body",
            name=f"section_{i} panel fits within the frame opening width",
        )

    # --- Depth stagger: each lower section sits behind its parent ----------
    for i in range(1, N_SECTIONS):
        ctx.expect_gap(
            sections[i],
            sections[i - 1],
            axis="y",
            positive_elem="panel_body",
            negative_elem="panel_body",
            min_gap=-0.001,
            max_gap=0.002,
            name=f"section_{i} rides face-to-face one thickness behind section_{i - 1}",
        )

    # --- Raised-panel embossing: every section has a raised field proud ----
    for i, section in enumerate(sections):
        body_aabb = ctx.part_element_world_aabb(section, elem="panel_body")
        field_aabb = ctx.part_element_world_aabb(section, elem="raised_field")
        ctx.check(
            f"section_{i} has a raised field proud of the recessed border",
            body_aabb is not None
            and field_aabb is not None
            and field_aabb[0][1] < body_aabb[0][1] - 0.004,
            details=f"body_min_y={body_aabb[0][1]}, field_min_y={field_aabb[0][1]}",
        )

    # --- Raised field is inset from panel edges (visible border) -----------
    for i, section in enumerate(sections):
        body_aabb = ctx.part_element_world_aabb(section, elem="panel_body")
        field_aabb = ctx.part_element_world_aabb(section, elem="raised_field")
        ctx.check(
            f"section_{i} raised field is inset from panel edges (visible border)",
            body_aabb is not None
            and field_aabb is not None
            and field_aabb[0][0] > body_aabb[0][0] + 0.03  # left inset
            and field_aabb[1][0] < body_aabb[1][0] - 0.03  # right inset
            and field_aabb[0][2] > body_aabb[0][2] + 0.03  # bottom inset
            and field_aabb[1][2] < body_aabb[1][2] - 0.03,  # top inset
            details=f"body={body_aabb}, field={field_aabb}",
        )

    # --- Closed laps: no see-through vertical gap between sections ---------
    for i in range(1, N_SECTIONS):
        upper = ctx.part_element_world_aabb(sections[i - 1], elem="panel_body")
        lower = ctx.part_element_world_aabb(sections[i], elem="panel_body")
        ctx.check(
            f"section_{i} lap tucks behind section_{i - 1} with no vertical gap",
            upper is not None
            and lower is not None
            and lower[1][2] >= upper[0][2] + SECTION_LAP - 0.002,
            details=f"upper={upper}, lower={lower}",
        )

    # --- Bottom section seats on the sill ----------------------------------
    ctx.expect_gap(
        sections[-1],
        frame,
        axis="z",
        positive_elem="panel_body",
        negative_elem="threshold",
        min_gap=-0.001,
        max_gap=0.01,
        name="bottom section seats on the floor sill",
    )

    # --- Latch handle is mounted on the bottom section ---------------------
    ctx.expect_contact(
        sections[-1],
        sections[-1],
        elem_a="latch_handle",
        elem_b="handle_boss_0",
        name="latch handle is anchored by its mounting boss",
    )

    # --- Articulation: telescoping lift ------------------------------------
    rest_positions = [ctx.part_world_position(s) for s in sections]
    open_pose = {j: SECTION_PITCH for j in lift_joints}
    with ctx.pose(open_pose):
        open_positions = [ctx.part_world_position(s) for s in sections]
        open_aabbs = [ctx.part_element_world_aabb(s, elem="panel_body") for s in sections]

    # The fixed top section must not move; section i rises i pitches (chain sum).
    chain_ok = True
    for i in range(N_SECTIONS):
        if rest_positions[i] is None or open_positions[i] is None:
            chain_ok = False
            break
        rise = open_positions[i][2] - rest_positions[i][2]
        if abs(rise - SECTION_PITCH * i) > 1e-3:
            chain_ok = False
            break
    ctx.check(
        "top section stays fixed and each lower section rises one pitch per chain link",
        chain_ok,
        details=f"rest={rest_positions}, open={open_positions}",
    )
    # When fully open every section nests in the TOP band.
    stack_top = max(a[1][2] for a in open_aabbs if a is not None)
    stack_bottom = min(a[0][2] for a in open_aabbs if a is not None)
    ctx.check(
        "open stack stays inside the frame (no section above the header line)",
        len([a for a in open_aabbs if a is not None]) == N_SECTIONS
        and stack_top <= PANEL_BASE_Z + OPENING_H + SECTION_LAP + 0.002,
        details=f"stack_top={stack_top}, opening_top={PANEL_BASE_Z + OPENING_H}",
    )
    ctx.check(
        "open stack clears the opening below the top band",
        stack_bottom >= PANEL_BASE_Z + OPENING_H - SECTION_PITCH - 0.002,
        details=f"stack_bottom={stack_bottom}",
    )

    # --- Intended overlaps between adjacent nesting sections ---------------
    for i in range(1, N_SECTIONS):
        ctx.allow_overlap(
            sections[i - 1],
            sections[i],
            reason=(
                "Sectional garage door: each section nests flat against the back of the "
                "section above it when raised; raised fields press into that contact plane."
            ),
        )

    return ctx.report()


object_model = build_object_model()
