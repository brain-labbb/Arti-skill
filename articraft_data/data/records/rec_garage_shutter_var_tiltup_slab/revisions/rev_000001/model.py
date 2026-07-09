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
# Object: a residential single-piece tilt-up canopy garage door, ~2.4 m wide x
#   ~2.13 m tall, standing on the ground (z = 0 upward). A black steel surround
#   frame holds one full-height grey embossed panel leaf; a small latch handle
#   sits near the bottom of the leaf.
# Root/support: the black steel frame (two jambs, a front header fascia, a
#   floor sill, and two deep side guide channels) is the fixed root.
# Parts: frame (root) + door_leaf (single full-height tilt-up panel).
# Articulations: frame_to_leaf, REVOLUTE, hinge at the top edge of the leaf
#   at the header line, axis along -X so positive q swings the bottom outward
#   (toward -Y, the front) and upward. Limits 0 to ~1.3 rad (~75°).
# Visible geometry: single full-height grey panel with 6 rows of raised
#   embossed pillows and seam grooves matching the parent sectional pattern,
#   plus a small dark latch handle near the bottom edge.
# Support/fit: the leaf sits in the frame opening with small side clearance;
#   the side guide channels run the full opening height.
# Intentional overlaps: none (single solid leaf inside a surrounding frame).
# Tests: door leaf fills the full opening height, frame surrounds the opening,
#   revolute joint swings the door outward/upward, closed pose seats leaf on
#   sill, latch handle mounted on leaf, embossing rows present.
# Assumptions: same frame dimensions and proportions as the parent sectional
#   door; 6 embossing rows on one monolithic panel; canopy pivot at header.
# ---------------------------------------------------------------------------

# --- Overall dimensions (meters) -------------------------------------------
# Reference matches the parent near-square mid-grey sectional door with a slim
# black surround; only the closure mechanism changes.
DOOR_W = 2.40  # overall frame outer width
JAMB_W = 0.06  # slim side jamb width
JAMB_D = 0.12  # frame depth (front-to-back, along Y)
HEADER_H = 0.06  # slim top header height

OPENING_W = DOOR_W - 2.0 * JAMB_W  # clear opening width = 2.28
N_ROWS = 6  # embossing rows matching parent sectional pattern
ROW_PITCH = 0.355  # vertical band height per embossing row
SLAT_D = 0.04  # panel thickness (along Y)
SLAT_W = OPENING_W - 0.012  # panel width, small side clearance

THRESHOLD_H = 0.05  # floor sill height
PANEL_BASE_Z = THRESHOLD_H  # bottom of door leaf sits on the sill

OPENING_H = N_ROWS * ROW_PITCH  # 2.13 m clear opening height
FRAME_OUTER_H = PANEL_BASE_Z + OPENING_H + HEADER_H  # overall frame height

FRAME_FRONT_Y = -JAMB_D / 2.0  # front face of the frame
PANEL_FRONT_Y = FRAME_FRONT_Y + 0.02  # panel front recessed 20 mm behind frame front
LEAF_CENTER_Y = PANEL_FRONT_Y + SLAT_D / 2.0  # panel thickness center
STACK_BACK_Y = PANEL_FRONT_Y + N_ROWS * SLAT_D  # kept for identical frame sill depth

# Hinge line: top edge of the door leaf at the header.
HINGE_Z = PANEL_BASE_Z + OPENING_H

# Embossing parameters (identical to parent sectional door).
PILLOW_INSET_X = 0.04  # raised field stops short of panel side edges
PILLOW_INSET_Z = 0.045  # ...and short of row top/bottom band edges
GROOVE_H = 0.018  # height of the recessed seam groove


def _row_band_center_z(i: int) -> float:
    """World z of the center of embossing row i (i = 0 is the TOP row)."""
    return PANEL_BASE_Z + OPENING_H - ROW_PITCH * (i + 0.5)


def _local_row_center_z(i: int) -> float:
    """Local z of row i center relative to the hinge (part origin at top edge)."""
    return _row_band_center_z(i) - HINGE_Z


def _add_embossing_row(leaf, i: int, pillow_mat, groove_mat) -> None:
    """Add one pillow + seam groove visual pair for embossing row i."""
    row_cz = _local_row_center_z(i)
    # Raised central pillow spanning most of the visible band.
    pillow_y = -SLAT_D / 2.0 - 0.004  # proud of the front face by ~4 mm
    leaf.visual(
        Box((SLAT_W - 2.0 * PILLOW_INSET_X, 0.014, ROW_PITCH - 2.0 * PILLOW_INSET_Z)),
        origin=Origin(xyz=(0.0, pillow_y, row_cz)),
        material=pillow_mat,
        name=f"panel_pillow_{i}",
    )
    # Recessed dark groove along the bottom edge of the row band.
    leaf.visual(
        Box((SLAT_W, 0.018, GROOVE_H)),
        origin=Origin(
            xyz=(0.0, -SLAT_D / 2.0 + 0.018 / 2.0 - 0.002, row_cz - ROW_PITCH / 2.0 + GROOVE_H / 2.0)
        ),
        material=groove_mat,
        name=f"seam_groove_{i}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_up_garage_door")

    # Flat matte palette: a slim near-black surround, flat mid-grey panels, and
    # a slightly darker grey for the recessed grooves between the embossing rows.
    steel_black = model.material("powdercoat_black_steel", rgba=(0.09, 0.09, 0.095, 1.0))
    panel_grey = model.material("embossed_grey_steel", rgba=(0.50, 0.505, 0.51, 1.0))
    panel_grey_pillow = model.material("panel_pillow_grey", rgba=(0.54, 0.545, 0.55, 1.0))
    groove_grey = model.material("panel_groove_grey", rgba=(0.30, 0.305, 0.31, 1.0))
    track_grey = model.material("galvanized_track", rgba=(0.40, 0.41, 0.42, 1.0))
    handle_metal = model.material("dark_handle_metal", rgba=(0.13, 0.13, 0.14, 1.0))

    # ---------------------------------------------------------------- frame
    # The frame is the fixed root: two side jambs, a front header fascia, a
    # floor sill, and two deep side guide channels. Kept identical to the
    # parent sectional door frame.
    frame = model.part("frame")

    jamb_full_h = OPENING_H + HEADER_H + THRESHOLD_H
    jamb_cz = PANEL_BASE_Z + OPENING_H / 2.0 + (HEADER_H - THRESHOLD_H) / 2.0
    frame.visual(
        Box((JAMB_W, JAMB_D, jamb_full_h)),
        origin=Origin(xyz=(-(OPENING_W + JAMB_W) / 2.0, 0.0, jamb_cz)),
        material=steel_black,
        name="jamb_0",
    )
    frame.visual(
        Box((JAMB_W, JAMB_D, jamb_full_h)),
        origin=Origin(xyz=((OPENING_W + JAMB_W) / 2.0, 0.0, jamb_cz)),
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
    # Floor sill the bottom of the door leaf seats onto.
    sill_d = (STACK_BACK_Y + 0.01) - FRAME_FRONT_Y
    frame.visual(
        Box((DOOR_W, sill_d, THRESHOLD_H)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + sill_d / 2.0, THRESHOLD_H / 2.0)),
        material=steel_black,
        name="threshold",
    )
    # Deep side guide channels just outside the leaf edges (inside the jamb
    # line). They run the full opening height and the full frame depth.
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

    # -------------------------------------------------------- door leaf
    # Single full-height tilt-up panel. Part origin at the hinge point (top
    # edge center of the panel at the header line). In local frame the panel
    # extends downward from z = 0 to z = -OPENING_H.
    leaf = model.part("door_leaf")

    # Main flat panel field.
    leaf.visual(
        Box((SLAT_W, SLAT_D, OPENING_H)),
        origin=Origin(xyz=(0.0, 0.0, -OPENING_H / 2.0)),
        material=panel_grey,
        name="panel_field",
    )

    # Embossing rows: 6 pillow + groove pairs matching the parent pattern.
    for i in range(N_ROWS):
        _add_embossing_row(leaf, i, panel_grey_pillow, groove_grey)

    # Latch handle near the bottom of the panel, standing proud of the front
    # face. Identical to the parent handle placement.
    handle_z = -OPENING_H + 0.06
    handle_y = -SLAT_D / 2.0 - 0.018
    leaf.visual(
        Box((0.18, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, handle_y, handle_z)),
        material=handle_metal,
        name="latch_handle",
    )
    # Small mounting bosses anchoring the handle to the panel face.
    leaf.visual(
        Box((0.02, 0.02, 0.024)),
        origin=Origin(xyz=(-0.07, -SLAT_D / 2.0 - 0.008, handle_z)),
        material=handle_metal,
        name="handle_boss_0",
    )
    leaf.visual(
        Box((0.02, 0.02, 0.024)),
        origin=Origin(xyz=(0.07, -SLAT_D / 2.0 - 0.008, handle_z)),
        material=handle_metal,
        name="handle_boss_1",
    )

    # ----------------------------------------------------------- articulation
    # Single revolute joint at the top header hinge line.
    # axis = (-1, 0, 0) so positive q swings the door bottom outward (-Y) and
    # upward, like a real canopy tilt-up garage door.
    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, LEAF_CENTER_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.5, lower=0.0, upper=1.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    leaf = object_model.get_part("door_leaf")
    hinge = object_model.get_articulation("frame_to_leaf")

    # --- Hero feature: single leaf fills the full opening height -----------
    leaf_aabb = ctx.part_element_world_aabb(leaf, elem="panel_field")
    ctx.check(
        "door leaf spans the full opening height",
        leaf_aabb is not None
        and leaf_aabb[0][2] <= PANEL_BASE_Z + 0.01
        and leaf_aabb[1][2] >= PANEL_BASE_Z + OPENING_H - 0.01,
        details=f"leaf_aabb={leaf_aabb}",
    )

    # --- Leaf fits within the frame opening width -------------------------
    ctx.expect_within(
        leaf,
        frame,
        axes="x",
        margin=0.0,
        inner_elem="panel_field",
        name="door leaf fits within the frame opening width",
    )

    # --- Closed leaf seats on the sill ------------------------------------
    ctx.expect_gap(
        leaf,
        frame,
        axis="z",
        positive_elem="panel_field",
        negative_elem="threshold",
        min_gap=-0.001,
        max_gap=0.01,
        name="closed door leaf seats on the floor sill",
    )

    # --- Latch handle is mounted on the door leaf -------------------------
    ctx.expect_contact(
        leaf,
        leaf,
        elem_a="latch_handle",
        elem_b="handle_boss_0",
        name="latch handle is anchored by its mounting boss",
    )

    # --- Embossing rows present (6 pillow + groove pairs) -----------------
    for i in range(N_ROWS):
        pillow_aabb = ctx.part_element_world_aabb(leaf, elem=f"panel_pillow_{i}")
        ctx.check(
            f"embossing row {i} pillow present on the single leaf",
            pillow_aabb is not None,
            details=f"panel_pillow_{i}_aabb={pillow_aabb}",
        )
        groove_aabb = ctx.part_element_world_aabb(leaf, elem=f"seam_groove_{i}")
        ctx.check(
            f"embossing row {i} seam groove present on the single leaf",
            groove_aabb is not None,
            details=f"seam_groove_{i}_aabb={groove_aabb}",
        )

    # --- Articulation: tilt-up canopy swing --------------------------------
    rest_aabb = ctx.part_element_world_aabb(leaf, elem="panel_field")
    rest_handle = ctx.part_element_world_aabb(leaf, elem="latch_handle")

    open_q = 1.2  # ~69 degrees, well within the 1.3 rad upper limit
    with ctx.pose({hinge: open_q}):
        open_aabb = ctx.part_element_world_aabb(leaf, elem="panel_field")
        open_handle = ctx.part_element_world_aabb(leaf, elem="latch_handle")

    # The door bottom must swing outward (toward -Y, the front) when opened.
    ctx.check(
        "open pose swings door bottom outward past the frame front",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < rest_aabb[0][1] - 0.3,
        details=f"rest_min_y={rest_aabb[0][1] if rest_aabb else None}, "
        f"open_min_y={open_aabb[0][1] if open_aabb else None}",
    )

    # The door bottom must rise upward when opened.
    ctx.check(
        "open pose raises door bottom well above the sill",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] > rest_aabb[0][2] + 0.3,
        details=f"rest_min_z={rest_aabb[0][2] if rest_aabb else None}, "
        f"open_min_z={open_aabb[0][2] if open_aabb else None}",
    )

    # The latch handle (at the door bottom) must move outward and upward,
    # confirming the tilt-up canopy mechanism.
    ctx.check(
        "latch handle swings outward and upward in open pose",
        rest_handle is not None
        and open_handle is not None
        and open_handle[0][1] < rest_handle[0][1] - 0.3
        and open_handle[0][2] > rest_handle[0][2] + 0.3,
        details=f"rest_handle={rest_handle}, open_handle={open_handle}",
    )

    # At rest, the leaf top must be near the hinge (header line).
    ctx.check(
        "closed leaf top meets the header hinge line",
        rest_aabb is not None
        and abs(rest_aabb[1][2] - HINGE_Z) < 0.01,
        details=f"leaf_top_z={rest_aabb[1][2] if rest_aabb else None}, hinge_z={HINGE_Z}",
    )

    return ctx.report()


object_model = build_object_model()
