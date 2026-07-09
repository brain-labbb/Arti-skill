from __future__ import annotations

# Side-hung wooden casement window, swung open, with visible brass butt hinges
# and a brass casement stay arm. Reference image 007: a deep-brown stained-wood
# window with a thin static outer frame and a single operable sash, dark tinted
# glass, gold-coloured (brass) butt hinges down one jamb and a small brass stay
# bar holding the sash open.
#
# Coordinate convention (Z up):
#   - Height runs along +Z (sill at z~0, head near the top).
#   - Width runs along X.
#   - Frame depth / glazing thickness / swing-normal runs along Y. The glass
#     plane is the X-Z plane.
#   - The sash is side-hung on the LEFT jamb (hinge line is the left vertical
#     edge). At q=0 the sash is shut, coplanar with the frame. Positive q swings
#     the free (right) edge outward toward +Y, reproducing the open photo at
#     ~0.75 rad.
#
# Structure:
#   - frame (static root): a real hollow wood profile (head, sill, two jambs)
#     built as a CadQuery slab with the glass opening cut out.
#   - sash (moving): its own wood frame ring + a thin tinted-glass pane, authored
#     in a hinge-local frame so the joint sits on the visible hinge line.
#   - two brass butt hinges (BarrelHingeGeometry) mounted on the hinge edge,
#     straddling the sash jamb and the frame jamb.
#   - a slim brass stay bar linking the sash to the frame head, holding it open.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BarrelHingeGeometry,
    Box,
    Cylinder,
    HingeHolePattern,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

WIN_W = 0.90          # overall window width (X)
WIN_H = 1.20          # overall window height (Z)
FRAME_FACE = 0.060    # outer frame member face width (in-plane, X/Z)
FRAME_DEPTH = 0.070   # outer frame depth (Y)

SILL_Z = 0.0          # bottom of the outer frame sits at z=0

# Sash: sits in the opening, slightly inset from the outer frame.
SASH_GAP = 0.006      # running clearance between sash outer edge and frame reveal
SASH_FACE = 0.055     # sash member face width
SASH_DEPTH = 0.050    # sash member depth (Y)
GLASS_T = 0.008       # tinted glass thickness (Y)
GLASS_REBATE = 0.006  # how far glass tucks under the sash lip per edge

# Opening (clear span inside the outer frame).
OPEN_X0 = -WIN_W / 2.0 + FRAME_FACE
OPEN_X1 = WIN_W / 2.0 - FRAME_FACE
OPEN_Z0 = SILL_Z + FRAME_FACE
OPEN_Z1 = WIN_H - FRAME_FACE
OPENING_W = OPEN_X1 - OPEN_X0
OPENING_H = OPEN_Z1 - OPEN_Z0

# Sash overall size (fills the opening minus the running clearance).
SASH_W = OPENING_W - 2 * SASH_GAP
SASH_H = OPENING_H - 2 * SASH_GAP

# Hinge edge world X (left reveal) and world Z range for the sash body.
HINGE_X = OPEN_X0 + SASH_GAP        # left edge of the sash (hinge line)
SASH_BOTTOM_Z = OPEN_Z0 + SASH_GAP  # world z of the sash local z=0
SASH_TOP_Z = SASH_BOTTOM_Z + SASH_H

# Brass butt hinge geometry.
HINGE_LEN = 0.110
HINGE_LEAF = 0.022
HINGE_LEAF_T = 0.0028
HINGE_PIN_D = 0.006
HINGE_KNUCKLE_D = 0.0125

# Stay bar.
STAY_W = 0.018
STAY_T = 0.006
STAY_LEN = 0.230

# Divided-light muntin grid (3×3 colonial arrangement → 9 panes).
GRID_COLS = 3
GRID_ROWS = 3
MUNTIN_FACE = 0.012    # muntin bar visible face width (in-plane, m)
MUNTIN_DEPTH = 0.020   # muntin bar depth (Y); sits within the sash rebate

# Inner glazing opening inside the sash frame (where glass + muntins live).
INNER_X0 = SASH_FACE
INNER_X1 = SASH_W - SASH_FACE
INNER_Z0 = SASH_FACE
INNER_Z1 = SASH_H - SASH_FACE
INNER_W = INNER_X1 - INNER_X0
INNER_H = INNER_Z1 - INNER_Z0

# Cell dimensions: each pane's clear opening between muntins / frame edges.
CELL_W = (INNER_W - (GRID_COLS - 1) * MUNTIN_FACE) / GRID_COLS
CELL_H = (INNER_H - (GRID_ROWS - 1) * MUNTIN_FACE) / GRID_ROWS

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

WOOD_RGBA = (0.255, 0.145, 0.075, 1.0)    # deep brown stained wood
GLASS_RGBA = (0.16, 0.17, 0.22, 0.38)     # dark tinted glass, semi-transparent
BRASS_RGBA = (0.80, 0.62, 0.22, 1.0)      # warm gold brass hardware


# ---------------------------------------------------------------------------
# Static outer frame geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_outer_frame_shape() -> cq.Workplane:
    """Static wood outer frame as a real hollow profile.

    World frame: opening centered on X=0, Z from 0 (sill) to WIN_H (head).
    A solid slab with the glass opening cut through, leaving head, sill, and
    two jambs as a continuous perimeter ring.
    """
    slab = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, WIN_H / 2.0))
        .box(WIN_W, FRAME_DEPTH, WIN_H)
    )
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(
            (OPEN_X0 + OPEN_X1) / 2.0,
            0.0,
            (OPEN_Z0 + OPEN_Z1) / 2.0,
        ))
        .box(OPENING_W, FRAME_DEPTH + 0.02, OPENING_H)
    )
    return slab.cut(opening)


# ---------------------------------------------------------------------------
# Sash geometry (CadQuery), authored in the hinge-local frame.
#   local x: 0 at hinge edge .. SASH_W at free edge
#   local z: 0 at sash bottom .. SASH_H at sash top
#   local y: sash thickness centered on y=0
# ---------------------------------------------------------------------------

def _build_sash_frame_shape() -> cq.Workplane:
    """Sash wood frame ring (its own perimeter), glass opening hollow."""
    w = SASH_W
    h = SASH_H
    t = SASH_FACE
    d = SASH_DEPTH

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, 0.0, h / 2.0))
        .box(w, d, h)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, 0.0, h / 2.0))
        .box(w - 2 * t, d + 0.02, h - 2 * t)
    )
    return outer.cut(inner)


def _muntin_bar(span: float, orientation: str = "vertical") -> Box:
    """Shared muntin bar geometry helper for the divided-light grid.

    orientation='vertical'  : bar spans ``span`` along Z, face width along X.
    orientation='horizontal': bar spans ``span`` along X, face width along Z.
    Depth (Y) is always MUNTIN_DEPTH.
    """
    if orientation == "vertical":
        return Box((MUNTIN_FACE, MUNTIN_DEPTH, span))
    return Box((span, MUNTIN_DEPTH, MUNTIN_FACE))


def _glass_pane(w: float, h: float) -> Box:
    """Shared glass pane geometry helper for one grid cell."""
    return Box((w, GLASS_T, h))


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_hung_wood_casement")

    model.material("wood", rgba=WOOD_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("brass", rgba=BRASS_RGBA)

    # --- Static outer frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_outer_frame_shape(), "frame"),
        material="wood",
        name="frame_shell",
    )

    # --- Operable sash (own frame ring + tinted glass) ---
    sash = model.part("sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sash_frame"),
        material="wood",
        name="sash_frame",
    )
    # --- Divided-light glass panes (3×3 colonial grid) ---
    # Nine equal panes fill the cells formed by the muntin grid.
    for j in range(GRID_COLS):
        for k in range(GRID_ROWS):
            cell_x0 = INNER_X0 + j * (CELL_W + MUNTIN_FACE)
            cell_z0 = INNER_Z0 + k * (CELL_H + MUNTIN_FACE)
            gx = cell_x0 + CELL_W / 2.0
            gz = cell_z0 + CELL_H / 2.0
            sash.visual(
                _glass_pane(CELL_W, CELL_H),
                origin=Origin(xyz=(gx, 0.0, gz)),
                material="glass",
                name=f"pane_{j}_{k}",
            )

    # --- Interior muntin bars (shared geometry helper, for-i-in-range) ---
    # Two interior vertical muntins divide the sash into three columns.
    for i in range(GRID_COLS - 1):
        mx = INNER_X0 + (i + 1) * CELL_W + (i + 0.5) * MUNTIN_FACE
        sash.visual(
            _muntin_bar(INNER_H, "vertical"),
            origin=Origin(xyz=(mx, 0.0, (INNER_Z0 + INNER_Z1) / 2.0)),
            material="wood",
            name=f"vmuntin_{i}",
        )

    # Two interior horizontal muntins divide the sash into three rows.
    for i in range(GRID_ROWS - 1):
        mz = INNER_Z0 + (i + 1) * CELL_H + (i + 0.5) * MUNTIN_FACE
        sash.visual(
            _muntin_bar(INNER_W, "horizontal"),
            origin=Origin(xyz=((INNER_X0 + INNER_X1) / 2.0, 0.0, mz)),
            material="wood",
            name=f"hmuntin_{i}",
        )

    # --- Brass butt hinges on the hinge (left) edge, two down the jamb ---
    # BarrelHingeGeometry builds a two-leaf hinge around a local Z pin axis.
    # We mount it on the sash so its pin runs vertically along the hinge line
    # (sash local x=0). Leaf A folds onto the sash jamb (+x), leaf B onto the
    # frame jamb. The hinge mesh is authored centered; rotate so its Z pin axis
    # becomes the world/sash Z axis (it already is), and offset to the hinge edge.
    hinge_geom = BarrelHingeGeometry(
        HINGE_LEN,
        leaf_width_a=HINGE_LEAF,
        leaf_width_b=HINGE_LEAF,
        leaf_thickness=HINGE_LEAF_T,
        pin_diameter=HINGE_PIN_D,
        knuckle_outer_diameter=HINGE_KNUCKLE_D,
        knuckle_count=5,
        open_angle_deg=90.0,
        holes_a=HingeHolePattern(style="round", count=3, diameter=0.003, edge_margin=0.006),
        holes_b=HingeHolePattern(style="round", count=3, diameter=0.003, edge_margin=0.006),
    )
    # Hinge pin sits just proud of the glass face (front, -Y) along the hinge
    # edge. With open_angle_deg=90, one leaf lies along the sash face and the
    # other wraps to the frame, straddling the hinge edge as a real butt hinge.
    hinge_y = -(SASH_DEPTH / 2.0)
    for i, frac in enumerate((0.80, 0.20)):
        z = SASH_FACE / 2.0 + frac * (SASH_H - SASH_FACE)
        sash.visual(
            mesh_from_geometry(hinge_geom, f"hinge_{i}"),
            origin=Origin(xyz=(0.0, hinge_y, z)),
            material="brass",
            name=f"hinge_{i}",
        )

    # --- Brass casement stay bar (slim flat bar) ---
    # A thin brass bar linking the sash near its top rail back to the frame
    # head, holding the sash open. Authored in the sash-local frame, mounted on
    # the sash near the hinge side and angled across toward the head; it reads as
    # the small stay bar at the top of the photo.
    stay = Box((STAY_LEN, STAY_T, STAY_W))
    stay_anchor_x = SASH_W * 0.30
    stay_z = SASH_H - SASH_FACE / 2.0
    stay_y = -(SASH_DEPTH / 2.0 + STAY_T / 2.0 + 0.004)
    sash.visual(
        stay,
        origin=Origin(xyz=(stay_anchor_x, stay_y, stay_z), rpy=(0.0, 0.0, 0.0)),
        material="brass",
        name="stay_bar",
    )
    # A small brass pivot stud at the stay's frame end (reads as the keep/pin).
    sash.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(
            xyz=(stay_anchor_x - STAY_LEN / 2.0, stay_y, stay_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="brass",
        name="stay_pivot",
    )

    # ----- Articulation -----
    # Side-hung casement: hinge line is the left vertical jamb (sash local x=0).
    # Joint origin on the hinge line at world (HINGE_X, 0, SASH_BOTTOM_Z) so the
    # sash local frame coincides there at q=0 (sash shut, coplanar with frame).
    # The sash body extends along local +X. Axis (0,0,1): positive q rotates the
    # free edge toward +Y (outward), opening the casement like the photo.
    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash",
        origin=Origin(xyz=(HINGE_X, 0.0, SASH_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.5),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

OPEN_POSE = 0.78  # radians, the photo's swung-open pose


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    hinge = object_model.get_articulation("frame_to_sash")

    # --- Intentional overlaps ---
    # Glass panes are rebated under the sash frame lip (captured glass).
    for j in range(GRID_COLS):
        for k in range(GRID_ROWS):
            ctx.allow_overlap(
                sash, sash,
                elem_a=f"pane_{j}_{k}",
                elem_b="sash_frame",
                reason=f"Tinted glass pane {j},{k} is rebated under the sash wood frame lip so it reads as captured, not floating.",
            )
    # Muntin bars span the inner glazing opening; their ends seat into the sash
    # frame rebate (intentional local embed at the frame contact).
    for i in range(GRID_COLS - 1):
        ctx.allow_overlap(
            sash, sash,
            elem_a=f"vmuntin_{i}",
            elem_b="sash_frame",
            reason=f"Vertical muntin {i} seats into the sash frame rebate at both ends.",
        )
    for i in range(GRID_ROWS - 1):
        ctx.allow_overlap(
            sash, sash,
            elem_a=f"hmuntin_{i}",
            elem_b="sash_frame",
            reason=f"Horizontal muntin {i} seats into the sash frame rebate at both ends.",
        )
    # Muntin bars are deeper than glass panes; at crossings the muntin
    # stands proud and intentionally overlaps the glass cells it separates.
    for i in range(GRID_COLS - 1):
        for j in range(GRID_COLS):
            for k in range(GRID_ROWS):
                ctx.allow_overlap(
                    sash, sash,
                    elem_a=f"vmuntin_{i}",
                    elem_b=f"pane_{j}_{k}",
                    reason="Muntin bar sits proud of the glass plane and overlaps adjacent panes at the grid crossing.",
                )
    for i in range(GRID_ROWS - 1):
        for j in range(GRID_COLS):
            for k in range(GRID_ROWS):
                ctx.allow_overlap(
                    sash, sash,
                    elem_a=f"hmuntin_{i}",
                    elem_b=f"pane_{j}_{k}",
                    reason="Muntin bar sits proud of the glass plane and overlaps adjacent panes at the grid crossing.",
                )
    # Muntin bars cross each other at the grid intersections.
    for vi in range(GRID_COLS - 1):
        for hi in range(GRID_ROWS - 1):
            ctx.allow_overlap(
                sash, sash,
                elem_a=f"vmuntin_{vi}",
                elem_b=f"hmuntin_{hi}",
                reason="Vertical and horizontal muntins cross at grid intersections.",
            )

    # Brass butt hinges straddle the hinge edge: one leaf overlaps the sash jamb,
    # the other reaches to the frame jamb. Allow sash-frame and hinge-into-sash.
    ctx.allow_overlap(
        frame, sash,
        reason="Side-hung sash hinges at the left jamb; the brass butt hinges and sash jamb intentionally meet the frame jamb at the hinge line.",
    )
    for i in range(2):
        ctx.allow_overlap(
            sash, sash,
            elem_a=f"hinge_{i}",
            elem_b="sash_frame",
            reason="Butt hinge leaf is screwed onto the sash jamb, so it embeds slightly into the sash frame.",
        )
    # Stay bar / pivot mount onto the sash frame.
    ctx.allow_overlap(
        sash, sash,
        elem_a="stay_bar",
        elem_b="sash_frame",
        reason="Brass stay bar is mounted onto the sash top rail.",
    )
    ctx.allow_overlap(
        sash, sash,
        elem_a="stay_pivot",
        elem_b="sash_frame",
        reason="Stay pivot stud seats into the sash top rail.",
    )

    # --- Frame is the static root, wider/taller than the sash, sash inside it ---
    frame_aabb = ctx.part_world_aabb(frame)
    with ctx.pose({hinge: 0.0}):
        sash_aabb = ctx.part_world_aabb(sash)
        fw = frame_aabb[1][0] - frame_aabb[0][0]
        fh = frame_aabb[1][2] - frame_aabb[0][2]
        sw = sash_aabb[1][0] - sash_aabb[0][0]
        sh = sash_aabb[1][2] - sash_aabb[0][2]
        ctx.check(
            "frame is wider than the sash",
            fw > sw + 0.04,
            details=f"frame_w={fw:.3f}, sash_w={sw:.3f}",
        )
        ctx.check(
            "frame is taller than the sash",
            fh > sh + 0.04,
            details=f"frame_h={fh:.3f}, sash_h={sh:.3f}",
        )
        # Window stands vertically with its sill at/near the floor.
        ctx.check(
            "frame sits on the floor (z>=0)",
            frame_aabb[0][2] >= -1e-4 and frame_aabb[0][2] < 0.02,
            details=f"frame zmin={frame_aabb[0][2]:.4f}",
        )
        ctx.check(
            "window is taller than wide (stands upright)",
            fh > fw,
            details=f"frame_h={fh:.3f}, frame_w={fw:.3f}",
        )
        # Closed pose: sash seated in the frame plane (small Y spread).
        sash_dy = sash_aabb[1][1] - sash_aabb[0][1]
        ctx.check(
            "closed sash is shallow in Y (seated in frame plane)",
            sash_dy < 0.12,
            details=f"sash Y depth={sash_dy:.3f}",
        )
        # Closed sash overlaps the frame opening footprint (it reads shut).
        ctx.expect_within(
            sash, frame,
            axes="x",
            inner_elem="sash_frame",
            margin=0.02,
            name="closed sash stays within the frame width",
        )
        closed_free_y = sash_aabb[1][1]  # max Y of sash when shut
        closed_free_x = sash_aabb[1][0]  # free (right) edge X when shut

    # --- HERO: driving the joint swings the free edge out toward +Y, hinge
    #     edge stays put on the left jamb ---
    with ctx.pose({hinge: OPEN_POSE}):
        open_aabb = ctx.part_world_aabb(sash)
        open_free_y = open_aabb[1][1]
        open_hinge_x_min = open_aabb[0][0]
        # Free edge leaves the closed plane substantially in +Y.
        ctx.check(
            "open sash free edge swings out toward +Y",
            open_free_y > closed_free_y + 0.20,
            details=f"closed max_y={closed_free_y:.3f}, open max_y={open_free_y:.3f}",
        )
        # The hinge (left) edge stays anchored near the left jamb (x ~ HINGE_X).
        ctx.check(
            "hinge edge stays anchored at the left jamb",
            abs(open_hinge_x_min - HINGE_X) < 0.06,
            details=f"open hinge xmin={open_hinge_x_min:.3f}, hinge_x={HINGE_X:.3f}",
        )
        # The free edge pulls back in X as it swings (rotating about the hinge).
        ctx.check(
            "open sash free edge rotates inward in X",
            open_aabb[1][0] < closed_free_x - 0.05,
            details=f"closed free x={closed_free_x:.3f}, open free x={open_aabb[1][0]:.3f}",
        )

    # --- Brass hinges are on the hinge (left) edge of the sash ---
    h0 = ctx.part_element_world_aabb(sash, elem="hinge_0")
    if h0 is not None:
        with ctx.pose({hinge: 0.0}):
            h0c = ctx.part_element_world_aabb(sash, elem="hinge_0")
            h0_x = (h0c[0][0] + h0c[1][0]) / 2.0
            ctx.check(
                "brass hinge sits on the left (hinge) edge",
                abs(h0_x - HINGE_X) < 0.05,
                details=f"hinge_0 x center={h0_x:.3f}, hinge_x={HINGE_X:.3f}",
            )

    # --- Stay bar stands proud of the glass plane (front, -Y) near the top ---
    stay_aabb = ctx.part_element_world_aabb(sash, elem="stay_bar")
    if stay_aabb is not None:
        with ctx.pose({hinge: 0.0}):
            sa = ctx.part_element_world_aabb(sash, elem="stay_bar")
            stay_y = (sa[0][1] + sa[1][1]) / 2.0
            stay_z = (sa[0][2] + sa[1][2]) / 2.0
            ctx.check(
                "stay bar stands off the glass (front, -Y)",
                stay_y < -SASH_DEPTH / 2.0,
                details=f"stay Y center={stay_y:.3f}",
            )
            ctx.check(
                "stay bar is near the top of the sash",
                stay_z > SASH_BOTTOM_Z + 0.6 * SASH_H,
                details=f"stay Z center={stay_z:.3f}",
            )

    # --- 3×3 colonial divided-light grid checks ---
    # Verify all nine panes exist and are named consistently.
    pane_count = 0
    for j in range(GRID_COLS):
        for k in range(GRID_ROWS):
            p_aabb = ctx.part_element_world_aabb(sash, elem=f"pane_{j}_{k}")
            ctx.check(
                f"pane {j},{k} exists",
                p_aabb is not None,
                details=f"pane_{j}_{k} AABB is None",
            )
            if p_aabb is not None:
                pane_count += 1
    ctx.check(
        "sash has exactly 9 glass panes (3×3 grid)",
        pane_count == 9,
        details=f"found {pane_count} panes",
    )

    # Verify 2 interior vertical + 2 interior horizontal muntins exist.
    for i in range(GRID_COLS - 1):
        va = ctx.part_element_world_aabb(sash, elem=f"vmuntin_{i}")
        ctx.check(
            f"vertical muntin {i} exists",
            va is not None,
            details=f"vmuntin_{i} AABB is None",
        )
    for i in range(GRID_ROWS - 1):
        ha = ctx.part_element_world_aabb(sash, elem=f"hmuntin_{i}")
        ctx.check(
            f"horizontal muntin {i} exists",
            ha is not None,
            details=f"hmuntin_{i} AABB is None",
        )

    # Vertical muntins divide the sash into columns: pane column-0 is to the
    # left of vmuntin_0, column-1 is between the two muntins, column-2 is to
    # the right of vmuntin_1.
    with ctx.pose({hinge: 0.0}):
        p00 = ctx.part_element_world_aabb(sash, elem="pane_0_0")
        p20 = ctx.part_element_world_aabb(sash, elem="pane_2_0")
        v0 = ctx.part_element_world_aabb(sash, elem="vmuntin_0")
        v1 = ctx.part_element_world_aabb(sash, elem="vmuntin_1")
        if p00 is not None and v0 is not None:
            ctx.check(
                "pane column 0 is left of vmuntin_0",
                p00[1][0] <= v0[0][0] + 0.002,
                details=f"pane_0_0 xmax={p00[1][0]:.4f}, vmuntin_0 xmin={v0[0][0]:.4f}",
            )
        if p20 is not None and v1 is not None:
            ctx.check(
                "pane column 2 is right of vmuntin_1",
                p20[0][0] >= v1[1][0] - 0.002,
                details=f"pane_2_0 xmin={p20[0][0]:.4f}, vmuntin_1 xmax={v1[1][0]:.4f}",
            )
        if v0 is not None and v1 is not None:
            ctx.check(
                "vmuntin_0 is left of vmuntin_1",
                v0[1][0] < v1[0][0],
                details=f"v0 xmax={v0[1][0]:.4f}, v1 xmin={v1[0][0]:.4f}",
            )

        # Horizontal muntins divide the sash into rows: row-0 at the bottom,
        # row-2 at the top.
        p02 = ctx.part_element_world_aabb(sash, elem="pane_0_2")
        h0 = ctx.part_element_world_aabb(sash, elem="hmuntin_0")
        h1 = ctx.part_element_world_aabb(sash, elem="hmuntin_1")
        if p00 is not None and h0 is not None:
            ctx.check(
                "pane row 0 is below hmuntin_0",
                p00[1][2] <= h0[0][2] + 0.002,
                details=f"pane_0_0 zmax={p00[1][2]:.4f}, hmuntin_0 zmin={h0[0][2]:.4f}",
            )
        if p02 is not None and h1 is not None:
            ctx.check(
                "pane row 2 is above hmuntin_1",
                p02[0][2] >= h1[1][2] - 0.002,
                details=f"pane_0_2 zmin={p02[0][2]:.4f}, hmuntin_1 zmax={h1[1][2]:.4f}",
            )
        if h0 is not None and h1 is not None:
            ctx.check(
                "hmuntin_0 is below hmuntin_1",
                h0[1][2] < h1[0][2],
                details=f"h0 zmax={h0[1][2]:.4f}, h1 zmin={h1[0][2]:.4f}",
            )

    return ctx.report()


object_model = build_object_model()
