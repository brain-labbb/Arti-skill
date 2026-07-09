from __future__ import annotations

# Folding document folder (classic manila-style file folder, green).
#
# Real object: a letter-size file folder made of a single sheet of stiff card
# scored and folded at the bottom into a back cover and a front cover. The front
# cover has three evenly spaced cut tabs projecting up from its top edge
# (left, center, right indexing tabs). The folder holds a small stack of
# papers; one sheet is lifting out.
#
# Coordinate convention:
#   +X : folder width (left -> right), the FOLD/SPINE line runs along X.
#   +Y : depth / thickness direction. Closed, both covers stack near Y~0.
#   +Z : up. The fold/spine is at the bottom (z=0); covers rise in +Z.
#
# Articulation (the real mechanism):
#   - back_cover is the root (it stands up at the back).
#   - front_cover is a REVOLUTE child that swings open about the bottom spine
#     line (hinge along X at z=0). Positive q opens the front cover forward (+Y).
#   - loose_sheet is a REVOLUTE child of the back_cover (the sheet lifting out),
#     pivoting up/out about the same bottom spine line. Positive q lifts it out.
#   - The inner paper stack is fixed inside, captured between the covers.

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
# Absolute dimensions (meters) -- US letter file folder proportions
# ---------------------------------------------------------------------------

COVER_W = 0.300            # folder width along X
COVER_H = 0.240            # cover height (z above the spine fold)
COVER_T = 0.0016          # card stock thickness (Y)
CORNER_R = 0.006          # rounded outer corners of the covers

# Tabs cut into the top edge of the FRONT cover (labeling tabs).
# Three equally sized tabs evenly spaced across the top edge (left, center, right).
N_TABS = 3               # number of indexing tabs along the top edge
TAB_W = 0.070            # each tab width along X
TAB_H = 0.022            # how far each tab projects above the cover top edge

# Bottom spine fold: a small rounded gusset so the two covers read as joined
# by a continuous folded edge rather than meeting at a hard seam.
SPINE_R = 0.004           # spine fold radius (rounded bottom edge)

# Inner paper stack (captured pages) and the loose lifting sheet.
PAGE_W = 0.282            # page width (slightly narrower than the cover)
PAGE_H = 0.225           # page height
PAGE_T = 0.0010          # single sheet thickness
STACK_N = 3              # number of captured pages in the inner stack
STACK_GAP = 0.0016       # spacing between captured pages

# Standing geometry: closed, the covers lean very slightly apart so the stack
# fits; the front cover sits a hair in +Y of the back cover.
COVER_GAP = 0.010        # nominal interior gap between covers when "closed/ajar"

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

GREEN_RGBA = (0.36, 0.70, 0.20, 1.0)        # bright green folder card
GREEN_DARK_RGBA = (0.30, 0.60, 0.16, 1.0)   # spine fold, slightly shaded
PAGE_RGBA = (0.93, 0.94, 0.95, 1.0)         # off-white paper
LINE_RGBA = (0.20, 0.22, 0.26, 1.0)         # ruled lines on the sheet (dark)


# ---------------------------------------------------------------------------
# Cover geometry (CadQuery)
# ---------------------------------------------------------------------------

def _tab_shape(x_left: float) -> cq.Workplane:
    """Single labeling tab projecting above the cover top edge.

    Shared geometry helper used by the for-i-in-range(N_TABS) loop.
    x_left is the tab's left-edge offset from the cover left edge (along X).
    """
    h = COVER_H
    r = CORNER_R
    tab = (
        cq.Workplane("XZ")
        .transformed(offset=(x_left + TAB_W / 2.0, h + TAB_H / 2.0, 0.0))
        .rect(TAB_W, TAB_H + 2 * r)  # overlap into the body so it unions cleanly
        .extrude(COVER_T)
    )
    tab = tab.edges("|Y").fillet(min(r, TAB_H / 2.5))
    return tab


def _cover_profile(with_tabs: bool) -> cq.Workplane:
    """2D outline of a cover in the XZ plane (local x=0..COVER_W, z=0..COVER_H).

    Built on the XZ workplane so the resulting extrude (thin) runs along Y.
    The front cover variant adds three evenly spaced projecting top tabs.
    """
    w = COVER_W
    h = COVER_H
    r = CORNER_R

    # Main rounded rectangle body via a 2D sketch on XZ.
    body = (
        cq.Workplane("XZ")
        .transformed(offset=(w / 2.0, h / 2.0, 0.0))
        .rect(w, h)
        .extrude(COVER_T)
    )
    # Round the outer corners (the four vertical-ish edges of the thin slab that
    # run along Y). Fillet the |Y edges.
    body = body.edges("|Y").fillet(r)

    if with_tabs:
        # Three equally sized tabs evenly spaced across the top edge.
        gap = (COVER_W - N_TABS * TAB_W) / (N_TABS + 1)
        for i in range(N_TABS):
            x_left = gap + i * (TAB_W + gap)
            body = body.union(_tab_shape(x_left))

    return body


def _spine_shape() -> cq.Workplane:
    """Rounded bottom fold/gusset joining the two covers.

    A thin quarter-rounded bar running the full width along X at z~0, occupying
    the small wedge between the two covers so the bottom reads as a single folded
    edge. Authored in the back-cover-local frame (x=0..COVER_W).
    """
    w = COVER_W
    depth = COVER_GAP + 2 * COVER_T  # spans across the interior gap in Y
    # A short bar centered in the gap, rounded along its bottom long edges.
    bar = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, depth / 2.0, SPINE_R))
        .box(w, depth, 2 * SPINE_R)
        .edges("|X").fillet(SPINE_R * 0.9)
    )
    return bar


def _page_shape(width: float, height: float, thickness: float) -> cq.Workplane:
    """A single sheet of paper in the XZ plane, extruded thin along Y."""
    return (
        cq.Workplane("XZ")
        .transformed(offset=(width / 2.0, height / 2.0, 0.0))
        .rect(width, height)
        .extrude(thickness)
    )


def _ruled_lines_shape(width: float, height: float, thickness: float) -> cq.Workplane:
    """Thin ruled lines lying on the face of the loose sheet (decorative)."""
    lines = None
    n = 16
    margin = 0.020
    usable = height - 2 * margin
    for i in range(n):
        z = margin + (i + 0.5) * usable / n
        bar = (
            cq.Workplane("XZ")
            .transformed(offset=(width / 2.0, z, 0.0))
            .rect(width - 2 * margin, 0.0009)
            .extrude(thickness)
        )
        lines = bar if lines is None else lines.union(bar)
    return lines


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_document_folder")

    model.material("folder_green", rgba=GREEN_RGBA)
    model.material("folder_green_dark", rgba=GREEN_DARK_RGBA)
    model.material("paper", rgba=PAGE_RGBA)
    model.material("ruled_line", rgba=LINE_RGBA)

    # -- Back cover (root). It stands up at the back, tilted slightly back so the
    #    folder stands open. Authored in its local frame: hinge edge (bottom fold)
    #    at local z=0, the cover rising along +Z, slab thickness along +Y. --
    back = model.part("back_cover")
    back.visual(
        mesh_from_cadquery(_cover_profile(with_tabs=False), "back_cover"),
        material="folder_green",
        name="back_panel",
    )
    # Spine fold gusset, attached to the back cover at the bottom, bridging the
    # interior gap toward the front cover.
    back.visual(
        mesh_from_cadquery(_spine_shape(), "spine_fold"),
        material="folder_green_dark",
        name="spine_fold",
    )

    # -- Captured inner page stack, fixed to the back cover (sits just in +Y of
    #    the back panel, rising from near the spine). These read as the papers
    #    held inside the folder. --
    for i in range(STACK_N):
        y = COVER_T + 0.0008 + i * STACK_GAP
        x_off = (COVER_W - PAGE_W) / 2.0
        page = _page_shape(PAGE_W, PAGE_H, PAGE_T)
        back.visual(
            mesh_from_cadquery(page, f"stack_page_{i}"),
            origin=Origin(xyz=(x_off, y, 0.004)),
            material="paper",
            name=f"stack_page_{i}",
        )

    # -- Front cover: REVOLUTE child swinging open about the bottom spine. --
    front = model.part("front_cover")
    front.visual(
        mesh_from_cadquery(_cover_profile(with_tabs=True), "front_cover"),
        material="folder_green",
        name="front_panel",
    )

    # -- Loose sheet lifting out of the folder: REVOLUTE child of the back cover,
    #    pivoting up/out about the same bottom spine line. --
    loose = model.part("loose_sheet")
    loose.visual(
        mesh_from_cadquery(_page_shape(PAGE_W, PAGE_H, PAGE_T), "loose_sheet"),
        material="paper",
        name="loose_panel",
    )
    loose.visual(
        mesh_from_cadquery(_ruled_lines_shape(PAGE_W, PAGE_H, PAGE_T * 1.4), "loose_lines"),
        origin=Origin(xyz=(0.0, PAGE_T * 0.5, 0.0)),
        material="ruled_line",
        name="loose_lines",
    )

    # ----- Articulations -----
    # The whole folder stands with the fold at the bottom (world z=0). We pose
    # the back cover leaning slightly backward and the front cover swinging open
    # forward, both about the bottom spine line (hinge axis along X at z=0).
    #
    # Each cover is authored with its hinge edge at local z=0 and the cover
    # rising along +Z. Positive revolute q about the chosen axis must OPEN the
    # cover (tip it away in +Y) rather than close it through the stack.

    # back_cover -> front_cover : hinge along the bottom spine.
    # The front cover rises along local +Z from z=0; rotating about +X axis tips
    # the free (top) edge toward +Y, i.e. the cover opens forward. Place the
    # joint frame just in +Y of the back cover bottom so the front cover sits in
    # front of the stack.
    model.articulation(
        "spine_hinge_front",
        ArticulationType.REVOLUTE,
        parent="back_cover",
        child="front_cover",
        origin=Origin(xyz=(0.0, COVER_GAP + COVER_T, 0.0)),
        # Cover rises along local +Z; about -X axis a positive q swings the top
        # edge toward +Y (forward), i.e. the front cover opens forward.
        axis=(-1.0, 0.0, 0.0),
        # 0 ~ closed (front cover nearly upright against the stack); positive q
        # swings the top edge forward into +Y (the folder opens).
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.4),
    )

    # back_cover -> loose_sheet : the sheet lifting out, hinged at the spine line
    # too but rising from inside the stack. Positive q lifts the free edge out.
    model.articulation(
        "spine_hinge_sheet",
        ArticulationType.REVOLUTE,
        parent="back_cover",
        child="loose_sheet",
        origin=Origin(xyz=((COVER_W - PAGE_W) / 2.0, COVER_T + 0.0008 + STACK_N * STACK_GAP, 0.0)),
        # About -X axis, positive q swings the sheet's free (top) edge toward +Y,
        # lifting it forward out of the stack.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    back = object_model.get_part("back_cover")
    front = object_model.get_part("front_cover")
    loose = object_model.get_part("loose_sheet")

    j_front = object_model.get_articulation("spine_hinge_front")
    j_sheet = object_model.get_articulation("spine_hinge_sheet")

    # --- Joint contract: both hinges are revolute about the X (spine) axis. ---
    ctx.check(
        "front hinge is revolute",
        j_front.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={j_front.articulation_type}",
    )
    ctx.check(
        "front hinge axis is X (spine line)",
        abs(abs(j_front.axis[0]) - 1.0) < 1e-6
        and abs(j_front.axis[1]) < 1e-6
        and abs(j_front.axis[2]) < 1e-6,
        details=f"axis={j_front.axis}",
    )
    ctx.check(
        "sheet hinge is revolute about X",
        j_sheet.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(j_sheet.axis[0]) - 1.0) < 1e-6,
        details=f"type={j_sheet.articulation_type}, axis={j_sheet.axis}",
    )

    # --- HERO geometry: front cover carries three projecting labeling tabs. ---
    front_present = front.get_visual("front_panel") is not None
    ctx.check("front cover panel present", front_present, details="front_panel missing")
    front_aabb = ctx.part_world_aabb(front)
    back_aabb = ctx.part_world_aabb(back)
    # The front cover (with tabs) should rise higher than the back cover, because
    # the tabs project above the top edge.
    ctx.check(
        "front cover tabs project above back cover top",
        front_aabb[1][2] > back_aabb[1][2] + (TAB_H * 0.5),
        details=f"front_top={front_aabb[1][2]:.3f}, back_top={back_aabb[1][2]:.3f}",
    )
    # The tab projection above the cover body accounts for the union overshoot
    # (each tab rect is TAB_H + 2*r tall, centered so r extends below the cover
    # top for a clean union; the visible projection is approximately TAB_H + r).
    tab_projection = front_aabb[1][2] - (COVER_H)
    expected_projection = TAB_H + CORNER_R
    ctx.check(
        "tab projection height matches TAB_H + union overshoot",
        abs(tab_projection - expected_projection) < 0.005,
        details=f"projection={tab_projection:.4f}, expected={expected_projection:.4f}",
    )
    # Three tabs: verify via the expected tab count constant.
    ctx.check(
        "three indexing tabs configured",
        N_TABS == 3,
        details=f"N_TABS={N_TABS}",
    )

    # --- Covers read as a folder: similar width, spanning the full COVER_W. ---
    front_w = front_aabb[1][0] - front_aabb[0][0]
    ctx.check(
        "front cover spans the folder width",
        front_w > COVER_W - 0.02,
        details=f"front_w={front_w:.3f} vs COVER_W={COVER_W:.3f}",
    )

    # --- The covers are joined at the bottom (spine) -- their bottoms align
    #     near z=0 (no floating cover). ---
    ctx.check(
        "front and back covers share the bottom spine line",
        abs(front_aabb[0][2] - back_aabb[0][2]) < 0.01
        and front_aabb[0][2] < 0.01,
        details=f"front_bottom={front_aabb[0][2]:.3f}, back_bottom={back_aabb[0][2]:.3f}",
    )

    # --- Captured page stack lives between the covers, near the bottom. ---
    page0 = back.get_visual("stack_page_0")
    ctx.check("inner page stack present", page0 is not None, details="stack_page_0 missing")
    stack_aabb = ctx.part_element_world_aabb(back, elem="stack_page_0")
    ctx.check(
        "inner page stack sits inside the folder footprint",
        stack_aabb is not None
        and stack_aabb[0][0] > back_aabb[0][0] - 1e-3
        and stack_aabb[1][0] < back_aabb[1][0] + 1e-3,
        details=f"stack_x=({stack_aabb[0][0]:.3f},{stack_aabb[1][0]:.3f})",
    )

    # --- HERO motion: closing/opening the front cover about the spine. ---
    # At rest (q=0) the front cover top edge sits at some +Y; opening it pushes
    # the top edge farther forward (+Y) and lowers its top Z (it tips over).
    rest_front = ctx.part_world_aabb(front)
    rest_top_y = rest_front[1][1]
    rest_top_z = rest_front[1][2]
    with ctx.pose({j_front: 1.2}):
        open_front = ctx.part_world_aabb(front)
        ctx.check(
            "opening front cover swings its top edge forward (+Y)",
            open_front[1][1] > rest_top_y + 0.05,
            details=f"rest_top_y={rest_top_y:.3f}, open_top_y={open_front[1][1]:.3f}",
        )
        ctx.check(
            "opening front cover lowers its top edge (it tips over)",
            open_front[1][2] < rest_top_z - 0.02,
            details=f"rest_top_z={rest_top_z:.3f}, open_top_z={open_front[1][2]:.3f}",
        )

    # --- HERO motion: lifting the loose sheet out about the spine. ---
    rest_sheet = ctx.part_world_aabb(loose)
    rest_sheet_top_y = rest_sheet[1][1]
    with ctx.pose({j_sheet: 1.0}):
        lifted = ctx.part_world_aabb(loose)
        ctx.check(
            "lifting the loose sheet swings it out (+Y) of the stack",
            lifted[1][1] > rest_sheet_top_y + 0.05,
            details=f"rest_y={rest_sheet_top_y:.3f}, lifted_y={lifted[1][1]:.3f}",
        )

    # --- Ruled lines are present on the loose sheet (visible detail). ---
    ctx.check(
        "loose sheet has ruled lines",
        loose.get_visual("loose_lines") is not None,
        details="loose_lines missing",
    )

    return ctx.report()


object_model = build_object_model()
