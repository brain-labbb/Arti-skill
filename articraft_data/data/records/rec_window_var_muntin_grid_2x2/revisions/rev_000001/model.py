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

# Divided-light muntin grid (colonial 2x2 = 4 panes).
GRID_COLS = 2           # panes across (horizontal)
GRID_ROWS = 2           # panes down (vertical)
MUNTIN_W = 0.022        # muntin bar face width (in-plane)
MUNTIN_D = SASH_DEPTH   # muntin bar depth (flush with sash)

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


def _muntin_bar(length_x: float, length_z: float) -> Box:
    """Shared muntin-bar geometry helper.

    Returns a Box sized for one muntin segment.  The caller decides whether
    the long axis runs along X (horizontal muntin) or Z (vertical muntin) by
    passing the appropriate (length_x, length_z) pair.  Depth is always the
    full sash depth so the bar reads flush with the sash rails/stiles.
    """
    return Box((length_x, MUNTIN_D, length_z))


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

    # --- Operable sash (own frame ring + divided-light muntin grid + panes) ---
    sash = model.part("sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(), "sash_frame"),
        material="wood",
        name="sash_frame",
    )

    # Glass opening bounds in sash-local frame.
    gx0 = SASH_FACE - GLASS_REBATE
    gx1 = SASH_W - SASH_FACE + GLASS_REBATE
    gz0 = SASH_FACE - GLASS_REBATE
    gz1 = SASH_H - SASH_FACE + GLASS_REBATE
    glass_area_w = gx1 - gx0
    glass_area_h = gz1 - gz0
    cell_w = glass_area_w / GRID_COLS
    cell_h = glass_area_h / GRID_ROWS

    # Vertical muntin bars (GRID_COLS - 1 bars splitting panes horizontally).
    for i in range(GRID_COLS - 1):
        xc = gx0 + (i + 1) * cell_w
        zc = (gz0 + gz1) / 2.0
        sash.visual(
            _muntin_bar(MUNTIN_W, glass_area_h),
            origin=Origin(xyz=(xc, 0.0, zc)),
            material="wood",
            name=f"vertical_muntin_{i}",
        )

    # Horizontal muntin bars (GRID_ROWS - 1 bars splitting panes vertically).
    for i in range(GRID_ROWS - 1):
        xc = (gx0 + gx1) / 2.0
        zc = gz0 + (i + 1) * cell_h
        sash.visual(
            _muntin_bar(glass_area_w, MUNTIN_W),
            origin=Origin(xyz=(xc, 0.0, zc)),
            material="wood",
            name=f"horizontal_muntin_{i}",
        )

    # Glass panes in the cells formed by the muntin grid (true divided light).
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            cx0 = gx0 + col * cell_w + (MUNTIN_W / 2.0 if col > 0 else 0.0)
            cx1 = gx0 + (col + 1) * cell_w - (MUNTIN_W / 2.0 if col < GRID_COLS - 1 else 0.0)
            cz0 = gz0 + row * cell_h + (MUNTIN_W / 2.0 if row > 0 else 0.0)
            cz1 = gz0 + (row + 1) * cell_h - (MUNTIN_W / 2.0 if row < GRID_ROWS - 1 else 0.0)
            pane_w = cx1 - cx0
            pane_h = cz1 - cz0
            sash.visual(
                Box((pane_w, GLASS_T, pane_h)),
                origin=Origin(xyz=((cx0 + cx1) / 2.0, 0.0, (cz0 + cz1) / 2.0)),
                material="glass",
                name=f"pane_{row}_{col}",
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
    # Each divided-light glass pane is rebated under the sash frame lip.
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            ctx.allow_overlap(
                sash, sash,
                elem_a=f"pane_{row}_{col}",
                elem_b="sash_frame",
                reason="Divided-light glass pane is rebated under the sash wood frame lip so it reads as captured.",
            )
    # Muntin bars sit flush with the sash frame depth and embed into the frame
    # rebate at their ends.
    for i in range(GRID_COLS - 1):
        ctx.allow_overlap(
            sash, sash,
            elem_a=f"vertical_muntin_{i}",
            elem_b="sash_frame",
            reason="Vertical muntin bar embeds into the sash frame rebate at top and bottom rails.",
        )
    for i in range(GRID_ROWS - 1):
        ctx.allow_overlap(
            sash, sash,
            elem_a=f"horizontal_muntin_{i}",
            elem_b="sash_frame",
            reason="Horizontal muntin bar embeds into the sash frame rebate at left and right stiles.",
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

    # --- Divided-light muntin grid: 2x2 colonial arrangement ---
    with ctx.pose({hinge: 0.0}):
        # Verify all 4 glass panes exist and are distinct.
        pane_aabbs = []
        for row in range(GRID_ROWS):
            for col in range(GRID_COLS):
                pane_name = f"pane_{row}_{col}"
                p_aabb = ctx.part_element_world_aabb(sash, elem=pane_name)
                ctx.check(
                    f"{pane_name} exists",
                    p_aabb is not None,
                    details=f"pane {row},{col} not found",
                )
                if p_aabb is not None:
                    pane_aabbs.append((row, col, p_aabb))

        # Verify the 2x2 grid arrangement: panes are arranged in rows and columns.
        if len(pane_aabbs) == 4:
            # Group by row (Z) and column (X).
            row_0_panes = [p for r, c, p in pane_aabbs if r == 0]
            row_1_panes = [p for r, c, p in pane_aabbs if r == 1]
            col_0_panes = [p for r, c, p in pane_aabbs if c == 0]
            col_1_panes = [p for r, c, p in pane_aabbs if c == 1]

            # Row 0 should be below row 1 (lower Z).
            if row_0_panes and row_1_panes:
                row_0_z_max = max(p[1][2] for p in row_0_panes)
                row_1_z_min = min(p[0][2] for p in row_1_panes)
                ctx.check(
                    "pane row 0 is below row 1",
                    row_0_z_max < row_1_z_min,
                    details=f"row0 zmax={row_0_z_max:.3f}, row1 zmin={row_1_z_min:.3f}",
                )

            # Column 0 should be left of column 1 (lower X).
            if col_0_panes and col_1_panes:
                col_0_x_max = max(p[1][0] for p in col_0_panes)
                col_1_x_min = min(p[0][0] for p in col_1_panes)
                ctx.check(
                    "pane col 0 is left of col 1",
                    col_0_x_max < col_1_x_min,
                    details=f"col0 xmax={col_0_x_max:.3f}, col1 xmin={col_1_x_min:.3f}",
                )

        # Verify vertical muntin exists and runs vertically (tall in Z, narrow in X).
        vm_aabb = ctx.part_element_world_aabb(sash, elem="vertical_muntin_0")
        ctx.check(
            "vertical_muntin_0 exists",
            vm_aabb is not None,
            details="vertical muntin not found",
        )
        if vm_aabb is not None:
            vm_dx = vm_aabb[1][0] - vm_aabb[0][0]
            vm_dz = vm_aabb[1][2] - vm_aabb[0][2]
            ctx.check(
                "vertical muntin is tall and narrow (vertical orientation)",
                vm_dz > 3.0 * vm_dx,
                details=f"vm dz={vm_dz:.3f}, vm dx={vm_dx:.3f}",
            )
            # Vertical muntin should be between the two columns of panes.
            if len(pane_aabbs) == 4:
                vm_x_center = (vm_aabb[0][0] + vm_aabb[1][0]) / 2.0
                left_col_x_max = max(p[1][0] for r, c, p in pane_aabbs if c == 0)
                right_col_x_min = min(p[0][0] for r, c, p in pane_aabbs if c == 1)
                ctx.check(
                    "vertical muntin sits between left and right pane columns",
                    left_col_x_max < vm_x_center < right_col_x_min,
                    details=f"left xmax={left_col_x_max:.3f}, vm x={vm_x_center:.3f}, right xmin={right_col_x_min:.3f}",
                )

        # Verify horizontal muntin exists and runs horizontally (wide in X, narrow in Z).
        hm_aabb = ctx.part_element_world_aabb(sash, elem="horizontal_muntin_0")
        ctx.check(
            "horizontal_muntin_0 exists",
            hm_aabb is not None,
            details="horizontal muntin not found",
        )
        if hm_aabb is not None:
            hm_dx = hm_aabb[1][0] - hm_aabb[0][0]
            hm_dz = hm_aabb[1][2] - hm_aabb[0][2]
            ctx.check(
                "horizontal muntin is wide and short (horizontal orientation)",
                hm_dx > 3.0 * hm_dz,
                details=f"hm dx={hm_dx:.3f}, hm dz={hm_dz:.3f}",
            )
            # Horizontal muntin should be between the two rows of panes.
            if len(pane_aabbs) == 4:
                hm_z_center = (hm_aabb[0][2] + hm_aabb[1][2]) / 2.0
                bottom_row_z_max = max(p[1][2] for r, c, p in pane_aabbs if r == 0)
                top_row_z_min = min(p[0][2] for r, c, p in pane_aabbs if r == 1)
                ctx.check(
                    "horizontal muntin sits between bottom and top pane rows",
                    bottom_row_z_max < hm_z_center < top_row_z_min,
                    details=f"bottom zmax={bottom_row_z_max:.3f}, hm z={hm_z_center:.3f}, top zmin={top_row_z_min:.3f}",
                )

    return ctx.report()


object_model = build_object_model()
