from __future__ import annotations

# Folding document folder (classic manila-style file folder, green).
#
# Real object: a letter-size file folder made of a single sheet of stiff card
# scored and folded at the bottom into a back cover and a front cover. The front
# cover has a cut tab projecting up from its top edge (the labeling tab). The
# folder holds a small stack of papers; one sheet is lifting out.
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

# Tab cut into the top edge of the FRONT cover (the labeling tab).
TAB_W = 0.110             # tab width along X
TAB_H = 0.022            # how far the tab projects above the cover top edge
TAB_X0 = 0.030           # tab left edge offset from cover left edge

# Bottom accordion-pleated expanding gusset along the spine line.
# The gusset is a series of regular zig-zag pleat facets running the full
# cover width, allowing the folder to expand.
GUSSET_H = 0.012          # zigzag height above the spine line (Z)
GUSSET_DEPTH = 0.016      # total Y depth of the gusset section
N_PLEATS = 6              # number of pleat facets in the accordion

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

def _cover_profile(with_tab: bool) -> cq.Workplane:
    """2D outline of a cover in the XZ plane (local x=0..COVER_W, z=0..COVER_H).

    Built on the XZ workplane so the resulting extrude (thin) runs along Y.
    The front cover variant adds the projecting top tab.
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

    if with_tab:
        # Tab block projecting above the top edge.
        tab = (
            cq.Workplane("XZ")
            .transformed(offset=(TAB_X0 + TAB_W / 2.0, h + TAB_H / 2.0, 0.0))
            .rect(TAB_W, TAB_H + 2 * r)  # overlap into the body so it unions cleanly
            .extrude(COVER_T)
        )
        tab = tab.edges("|Y").fillet(min(r, TAB_H / 2.5))
        body = body.union(tab)

    return body


def _pleat_facet(
    width: float,
    length: float,
    thickness: float,
    y_center: float,
    z_center: float,
    tilt_deg: float,
) -> cq.Workplane:
    """Shared pleat geometry helper: one thin flat panel, positioned and tilted.

    Creates a thin rectangular panel (card-stock pleat facet) with:
      - ``width`` along X (spanning the folder width, centered at x = width/2)
      - ``length`` along the tilted diagonal (the pleat segment length)
      - ``thickness`` (card stock)

    The panel is centered at (width/2, y_center, z_center) and tilted by
    ``tilt_deg`` around the X axis to form one segment of the accordion zigzag.
    """
    return (
        cq.Workplane("XY")
        .transformed(
            offset=(width / 2.0, y_center, z_center),
            rotate=(tilt_deg, 0.0, 0.0),
        )
        .box(width, length, thickness)
    )


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
        mesh_from_cadquery(_cover_profile(with_tab=False), "back_cover"),
        material="folder_green",
        name="back_panel",
    )
    # Accordion-pleated expanding gusset along the bottom spine line.
    # Each pleat is a thin flat panel tilted alternately to form a zigzag,
    # running the full cover width. The loop emits N_PLEATS facets using the
    # shared _pleat_facet helper with regular placement.
    _dy = GUSSET_DEPTH / N_PLEATS
    _panel_angle_rad = math.atan2(GUSSET_H, _dy)
    _panel_angle_deg = math.degrees(_panel_angle_rad)
    _panel_length = math.sqrt(_dy ** 2 + GUSSET_H ** 2)
    # Slight extension so adjacent pleats overlap at fold lines (mesh connectivity).
    _pleat_extension = 0.002
    _effective_length = _panel_length + _pleat_extension

    for i in range(N_PLEATS):
        _y_start = i * _dy
        _z_start = 0.0 if i % 2 == 0 else GUSSET_H
        _y_end = (i + 1) * _dy
        _z_end = GUSSET_H if i % 2 == 0 else 0.0
        _y_mid = (_y_start + _y_end) / 2.0
        _z_mid = (_z_start + _z_end) / 2.0
        _rot_x = _panel_angle_deg if i % 2 == 0 else -_panel_angle_deg

        pleat = _pleat_facet(
            COVER_W, _effective_length, COVER_T,
            _y_mid, _z_mid, _rot_x,
        )
        back.visual(
            mesh_from_cadquery(pleat, f"pleat_{i}"),
            material="folder_green_dark",
            name=f"pleat_{i}",
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
        mesh_from_cadquery(_cover_profile(with_tab=True), "front_cover"),
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

    # --- HERO geometry: front cover carries the projecting tab. ---
    tab_present = front.get_visual("front_panel") is not None
    ctx.check("front cover panel present", tab_present, details="front_panel missing")
    front_aabb = ctx.part_world_aabb(front)
    back_aabb = ctx.part_world_aabb(back)
    # The front cover (with tab) should rise higher than the back cover, because
    # the tab projects above the top edge.
    ctx.check(
        "front cover tab projects above back cover top",
        front_aabb[1][2] > back_aabb[1][2] + (TAB_H * 0.5),
        details=f"front_top={front_aabb[1][2]:.3f}, back_top={back_aabb[1][2]:.3f}",
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

    # --- Accordion gusset: pleat visuals exist and span the full width. ---
    pleat_names = [f"pleat_{i}" for i in range(N_PLEATS)]
    for pname in pleat_names:
        ctx.check(
            f"gusset {pname} present",
            back.get_visual(pname) is not None,
            details=f"{pname} missing from back_cover",
        )

    # The first and last pleats should span the full folder width.
    pleat0_aabb = ctx.part_element_world_aabb(back, elem="pleat_0")
    ctx.check(
        "gusset pleat_0 spans the folder width",
        pleat0_aabb is not None
        and (pleat0_aabb[1][0] - pleat0_aabb[0][0]) > COVER_W - 0.02,
        details=f"pleat_0 width={pleat0_aabb[1][0] - pleat0_aabb[0][0]:.3f}" if pleat0_aabb else "pleat_0 missing",
    )

    # Zigzag: even pleats peak high (z ~ GUSSET_H) and odd pleats dip low.
    # Check that the top of an even pleat is significantly above the bottom of
    # an odd pleat, confirming the zigzag alternation.
    pleat0_top_z = pleat0_aabb[1][2] if pleat0_aabb else 0.0
    pleat1_aabb = ctx.part_element_world_aabb(back, elem="pleat_1")
    pleat1_bot_z = pleat1_aabb[0][2] if pleat1_aabb else 0.0
    ctx.check(
        "gusset shows zigzag alternation (even pleat peaks above odd pleat valley)",
        pleat0_top_z > pleat1_bot_z + 0.003,
        details=f"pleat_0_top_z={pleat0_top_z:.4f}, pleat_1_bot_z={pleat1_bot_z:.4f}",
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
