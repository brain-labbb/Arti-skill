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

# Elastic corner straps (closure mechanism).
# Two diagonal elastic bands on the front cover outer face, one crossing the
# top-right corner and one crossing the bottom-right corner. Each strap is
# hinged at the right cover edge and can lift off to release the corner.
STRAP_LENGTH = 0.075       # diagonal strap length (m)
STRAP_WIDTH = 0.010        # strap width in cover plane, perpendicular to length
STRAP_THICK = 0.0015       # strap thickness (protrusion from cover face)
STRAP_OFFSET = 0.045       # hinge offset from corner along right edge
STRAP_N = 2                # number of corner straps (top-right, bottom-right)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

GREEN_RGBA = (0.36, 0.70, 0.20, 1.0)        # bright green folder card
GREEN_DARK_RGBA = (0.30, 0.60, 0.16, 1.0)   # spine fold, slightly shaded
PAGE_RGBA = (0.93, 0.94, 0.95, 1.0)         # off-white paper
LINE_RGBA = (0.20, 0.22, 0.26, 1.0)         # ruled lines on the sheet (dark)
STRAP_RGBA = (0.22, 0.15, 0.10, 1.0)        # dark brown elastic strap


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


def _strap_shape(length: float, width: float, thickness: float, z_dir: int) -> cq.Workplane:
    """Slender elastic strap band from origin.

    Authored in local frame: hinge at origin (0, 0, 0), strap extends
    diagonally in the (-X, z_dir * +Z) direction at 45° in the XZ plane.
    The band width lies in the XZ plane (perpendicular to the diagonal),
    and the band thickness is along Y (protrusion from the cover face).

    z_dir: +1 for top strap (diagonal goes up-left), -1 for bottom (down-left).
    """
    # Build a box along +X centered at (length/2, 0, 0).
    # X = length (will become the diagonal direction),
    # Y = thickness (protrusion from cover face),
    # Z = width (in-plane, perpendicular to the diagonal).
    band = (
        cq.Workplane("XY")
        .transformed(offset=(length / 2.0, 0.0, 0.0))
        .box(length, thickness, width)
    )
    # Rotate about Y axis to point diagonally:
    #   z_dir=+1 → -135° maps +X to (-cos45, 0, +sin45) = up-left
    #   z_dir=-1 → +135° maps +X to (-cos45, 0, -sin45) = down-left
    angle = -135.0 * z_dir
    band = band.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle)
    return band


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_document_folder")

    model.material("folder_green", rgba=GREEN_RGBA)
    model.material("folder_green_dark", rgba=GREEN_DARK_RGBA)
    model.material("paper", rgba=PAGE_RGBA)
    model.material("ruled_line", rgba=LINE_RGBA)
    model.material("elastic_strap", rgba=STRAP_RGBA)

    # -- Back cover (root). It stands up at the back, tilted slightly back so the
    #    folder stands open. Authored in its local frame: hinge edge (bottom fold)
    #    at local z=0, the cover rising along +Z, slab thickness along +Y. --
    back = model.part("back_cover")
    back.visual(
        mesh_from_cadquery(_cover_profile(with_tab=False), "back_cover"),
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
        mesh_from_cadquery(_cover_profile(with_tab=True), "front_cover"),
        material="folder_green",
        name="front_panel",
    )

    # -- Elastic corner straps on the front cover outer face. --
    # Each strap is a slender diagonal band hinged at the right edge of the front
    # cover. At q=0 the strap lies flat across the corner holding the folder
    # shut; positive q lifts the free end off the cover to release.
    # strap_0 = top-right corner, strap_1 = bottom-right corner.
    strap_z_dirs = [+1, -1]
    strap_z_hinges = [
        COVER_H - STRAP_OFFSET,   # top strap: hinge below top-right corner
        STRAP_OFFSET,              # bottom strap: hinge above bottom-right corner
    ]
    for i in range(STRAP_N):
        strap = model.part(f"strap_{i}")
        strap.visual(
            mesh_from_cadquery(
                _strap_shape(STRAP_LENGTH, STRAP_WIDTH, STRAP_THICK, strap_z_dirs[i]),
                f"strap_{i}",
            ),
            material="elastic_strap",
            name=f"strap_band_{i}",
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

    # front_cover -> strap_i : elastic corner strap release joints.
    # Each strap is hinged at the right edge of the front cover on the outer
    # face. Axis along -Z so positive q lifts the free end (+Y) off the corner.
    for i in range(STRAP_N):
        model.articulation(
            f"strap_release_{i}",
            ArticulationType.REVOLUTE,
            parent="front_cover",
            child=f"strap_{i}",
            origin=Origin(xyz=(COVER_W, 0.0, strap_z_hinges[i])),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=0.0, upper=0.7,
            ),
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

    # --- Elastic corner straps (closure mechanism). ---
    # Allow the small intentional overlap of each strap seated flat on the
    # outer face of the front cover (half-thickness embedding).
    for i in range(STRAP_N):
        ctx.allow_overlap(
            "front_cover", f"strap_{i}",
            elem_a="front_panel",
            elem_b=f"strap_band_{i}",
            reason=f"Elastic strap_{i} is seated flat on the outer face of the front cover.",
        )

    for i in range(STRAP_N):
        strap_i = object_model.get_part(f"strap_{i}")
        j_strap_i = object_model.get_articulation(f"strap_release_{i}")

        ctx.check(
            f"strap_{i} part exists",
            strap_i is not None,
            details=f"strap_{i} missing",
        )
        ctx.check(
            f"strap_{i} has band visual",
            strap_i.get_visual(f"strap_band_{i}") is not None,
            details=f"strap_band_{i} missing",
        )
        ctx.check(
            f"strap_release_{i} is revolute",
            j_strap_i.articulation_type == ArticulationType.REVOLUTE,
            details=f"type={j_strap_i.articulation_type}",
        )
        ctx.check(
            f"strap_release_{i} axis is along Z",
            abs(abs(j_strap_i.axis[2]) - 1.0) < 1e-6
            and abs(j_strap_i.axis[0]) < 1e-6
            and abs(j_strap_i.axis[1]) < 1e-6,
            details=f"axis={j_strap_i.axis}",
        )

        # Strap sits on the front cover outer face — prove contact.
        ctx.expect_contact(
            strap_i, front,
            contact_tol=0.003,
            elem_a=f"strap_band_{i}",
            elem_b="front_panel",
            name=f"strap_{i} contacts the front cover face",
        )

        # Positive joint position lifts the free end away from the cover (+Y).
        rest_strap = ctx.part_world_aabb(strap_i)
        rest_max_y = rest_strap[1][1]
        with ctx.pose({j_strap_i: 0.5}):
            lifted_strap = ctx.part_world_aabb(strap_i)
            lifted_max_y = lifted_strap[1][1]
            ctx.check(
                f"strap_release_{i} positive q lifts strap off cover (+Y)",
                lifted_max_y > rest_max_y + 0.005,
                details=f"rest_max_y={rest_max_y:.4f}, lifted_max_y={lifted_max_y:.4f}",
            )

    # Straps are vertically separated (one near top corner, one near bottom).
    strap_0_aabb = ctx.part_world_aabb(object_model.get_part("strap_0"))
    strap_1_aabb = ctx.part_world_aabb(object_model.get_part("strap_1"))
    strap_0_center_z = (strap_0_aabb[0][2] + strap_0_aabb[1][2]) / 2.0
    strap_1_center_z = (strap_1_aabb[0][2] + strap_1_aabb[1][2]) / 2.0
    ctx.check(
        "straps are vertically separated (top and bottom corners)",
        abs(strap_0_center_z - strap_1_center_z) > 0.05,
        details=f"strap_0_z={strap_0_center_z:.3f}, strap_1_z={strap_1_center_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
