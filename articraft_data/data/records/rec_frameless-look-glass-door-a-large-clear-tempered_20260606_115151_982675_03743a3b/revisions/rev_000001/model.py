from __future__ import annotations

# Frameless-look glass door: a large clear tempered-glass swinging leaf beside a
# fixed clear-glass side panel, each with a tall tubular stainless bar handle and
# thin dark edge framing.
#
# Coordinate convention (object-intrinsic):
#   +X = horizontal, across the two panels (side panel + leaf side by side)
#   +Y = depth, glass thickness direction (front faces +Y)
#   +Z = vertical, up  (base at z=0, top at z≈PANEL_H)
#
# Root geometry (side panel + surround) is authored in WORLD coords: bottom z=0.
# Leaf geometry is authored in LEAF-LOCAL coords: hinge joint origin is at world
# (HINGE_X, 0, Z_OFF), so local z=0 maps to world z=Z_OFF (panel mid-height).
# Leaf geometry is centered at local z=0 so it spans local ±PANEL_H/2 → world 0..PANEL_H.

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

# --- Real-world dimensions (meters) -----------------------------------------

PANEL_W = 0.86
PANEL_H = 2.02
GLASS_T = 0.012
EDGE_FRAME = 0.018  # thin dark edge framing depth (Y) beyond glass
EDGE_W = 0.022  # visible width of the dark edge frame band

GAP = 0.010  # reveal gap between leaf and side panel

# X layout: side panel on -X, leaf on +X.
PANEL_CX = PANEL_W / 2.0 + GAP / 2.0  # leaf center (+X side)
SIDE_CX = -(PANEL_W / 2.0 + GAP / 2.0)  # side panel center (-X side)

# Leaf hinge on its outer (+X) edge.
LEAF_X_HI = PANEL_CX + PANEL_W / 2.0
HINGE_X = LEAF_X_HI + 0.006

# Bar handle.
HANDLE_LEN = 1.20
HANDLE_R = 0.0150
STANDOFF_R = 0.0140
STANDOFF_OUT = 0.055  # standoff projection beyond glass face
STANDOFF_INSET = 0.14  # from each handle end
HANDLE_OFFSET_X = 0.13  # handle inset from the panel's handle-side edge

# Z offset: world coords. Root geometry is authored with z center at Z_OFF so
# the assembly base sits at z=0 (bottom = Z_OFF - PANEL_H/2 = 0).
Z_OFF = PANEL_H / 2.0  # = 1.01 m

# --- Materials ---------------------------------------------------------------

GLASS_RGBA = (0.72, 0.80, 0.84, 0.28)  # clear tempered glass, translucent
EDGE_RGBA = (0.10, 0.10, 0.11, 1.0)  # thin dark edge framing
STEEL_RGBA = (0.80, 0.81, 0.83, 1.0)  # brushed stainless


# --- CadQuery geometry helpers ----------------------------------------------


def _glass_pane(local_cx: float = 0.0, z_center: float = Z_OFF) -> cq.Workplane:
    """Large clear glass pane.

    z_center: Z center in the caller's coord system.
      - For root (world): pass Z_OFF so bottom is at z=0.
      - For leaf-local: pass 0.0 (joint origin at world Z_OFF handles the lift).
    """
    return (
        cq.Workplane("XZ")
        .center(local_cx, z_center)
        .box(PANEL_W - 2.0 * EDGE_W, PANEL_H - 2.0 * EDGE_W, GLASS_T, centered=(True, True, True))
    )


def _edge_frame(local_cx: float = 0.0, z_center: float = Z_OFF) -> cq.Workplane:
    """Thin dark edge framing (a rectangular band) around the pane."""
    outer = (
        cq.Workplane("XZ")
        .center(local_cx, z_center)
        .box(PANEL_W, PANEL_H, GLASS_T + 2.0 * EDGE_FRAME, centered=(True, True, True))
    )
    inner = (
        cq.Workplane("XZ")
        .center(local_cx, z_center)
        .box(
            PANEL_W - 2.0 * EDGE_W,
            PANEL_H - 2.0 * EDGE_W,
            GLASS_T + 2.0 * EDGE_FRAME + 0.01,
            centered=(True, True, True),
        )
    )
    return outer.cut(inner)


def _root_surround() -> cq.Workplane:
    """Thin dark surround on the root: a hinge post at the leaf hinge line plus
    top header and bottom sill rails bridging it to the side panel.

    Built in WORLD coords (root part), bottom at z=0.
    """
    post_w = 0.030
    depth = GLASS_T + 2.0 * EDGE_FRAME
    rail_h = 0.040

    # Hinge post at the leaf hinge line.
    post = (
        cq.Workplane("XZ")
        .center(HINGE_X, Z_OFF)
        .box(post_w, PANEL_H, depth, centered=(True, True, True))
    )

    # Top header and bottom sill span from the side panel's outer (-X) edge to
    # the hinge post.
    rail_x_lo = SIDE_CX - PANEL_W / 2.0
    rail_x_hi = HINGE_X + post_w / 2.0
    rail_cx = (rail_x_lo + rail_x_hi) / 2.0
    rail_len = rail_x_hi - rail_x_lo
    rail_z = PANEL_H / 2.0 + rail_h / 2.0 - 0.006  # offset from panel center

    surround = post
    for dz in (rail_z, -rail_z):
        rail = (
            cq.Workplane("XZ")
            .center(rail_cx, Z_OFF + dz)
            .box(rail_len, rail_h, depth, centered=(True, True, True))
        )
        surround = surround.union(rail)
    return surround


def _bar_handle(local_cx: float, z_center: float = Z_OFF) -> cq.Workplane:
    """Back-to-back tubular bar pull (one bar each face) on through-standoffs.

    Bars run vertically (Z). z_center controls where the handle is centered in Z:
      - For root (world): pass Z_OFF (handle centered at mid-height in world).
      - For leaf-local: pass 0.0 (joint origin provides the world lift).
    """
    hx = local_cx
    face_y = GLASS_T / 2.0
    bar_y = face_y + STANDOFF_OUT + HANDLE_R
    handle_z0 = z_center - HANDLE_LEN / 2.0

    bars = None
    for side in (+1.0, -1.0):
        bar = cq.Solid.makeCylinder(
            HANDLE_R,
            HANDLE_LEN,
            cq.Vector(hx, side * bar_y, handle_z0),
            cq.Vector(0, 0, 1),
        )
        b = cq.Workplane("XY").add(bar)
        bars = b if bars is None else bars.union(b)

    # Through-standoffs spanning from the +Y bar to the -Y bar (through glass).
    asm = bars
    for sz in (handle_z0 + STANDOFF_INSET, handle_z0 + HANDLE_LEN - STANDOFF_INSET):
        so = cq.Solid.makeCylinder(
            STANDOFF_R,
            2.0 * bar_y,
            cq.Vector(hx, -bar_y, sz),
            cq.Vector(0, 1, 0),
        )
        asm = asm.union(cq.Workplane("XY").add(so))
    return asm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="frameless_glass_door")

    glass_mat = model.material("clear_glass", rgba=GLASS_RGBA)
    edge_mat = model.material("dark_edge", rgba=EDGE_RGBA)
    steel_mat = model.material("brushed_steel", rgba=STEEL_RGBA)

    # --- Root: fixed side panel (glass + edge frame + handle) ---------------
    # Authored in WORLD coords (z_center=Z_OFF so base at z=0).
    side = model.part("side_panel")
    side.visual(mesh_from_cadquery(_glass_pane(SIDE_CX, z_center=Z_OFF), "side_glass"), material=glass_mat, name="side_glass")
    side.visual(mesh_from_cadquery(_edge_frame(SIDE_CX, z_center=Z_OFF), "side_edge"), material=edge_mat, name="side_edge")
    side.visual(mesh_from_cadquery(_root_surround(), "root_surround"), material=edge_mat, name="root_surround")
    # Side-panel handle near the inner (toward-leaf, +X) edge.
    side.visual(
        mesh_from_cadquery(_bar_handle(SIDE_CX + PANEL_W / 2.0 - HANDLE_OFFSET_X, z_center=Z_OFF), "side_handle"),
        material=steel_mat,
        name="side_handle",
    )

    # --- Operable leaf (glass + edge frame + handle) ------------------------
    # Authored in LEAF-LOCAL coords (z_center=0.0).
    # Joint origin at world (HINGE_X, 0, Z_OFF): local z=0 → world z=Z_OFF.
    # Leaf body extends along local -X from hinge (local X=0).
    leaf = model.part("glass_leaf")
    leaf_cx = -PANEL_W / 2.0
    leaf.visual(mesh_from_cadquery(_glass_pane(leaf_cx, z_center=0.0), "leaf_glass"), material=glass_mat, name="leaf_glass")
    leaf.visual(mesh_from_cadquery(_edge_frame(leaf_cx, z_center=0.0), "leaf_edge"), material=edge_mat, name="leaf_edge")
    # Leaf handle near its free (-X, handle-side) edge.
    leaf.visual(
        mesh_from_cadquery(_bar_handle(-PANEL_W + HANDLE_OFFSET_X, z_center=0.0), "leaf_handle"),
        material=steel_mat,
        name="leaf_handle",
    )

    # --- Articulation: leaf swings about vertical hinge on its +X edge -------
    # Joint origin at world (HINGE_X, 0, Z_OFF) so the leaf hangs correctly
    # above z=0. Axis -Z makes positive q swing the free edge toward +Y (outward).
    model.articulation(
        "side_panel_to_leaf",
        ArticulationType.REVOLUTE,
        parent=side,
        child=leaf,
        origin=Origin(xyz=(HINGE_X, 0.0, Z_OFF)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side = object_model.get_part("side_panel")
    leaf = object_model.get_part("glass_leaf")
    hinge = object_model.get_articulation("side_panel_to_leaf")

    # Intentional seating overlaps: glass sits inside the edge frame band, and
    # handle standoffs pass through the glass.
    for part, g, e, h in (
        (side, "side_glass", "side_edge", "side_handle"),
        (leaf, "leaf_glass", "leaf_edge", "leaf_handle"),
    ):
        ctx.allow_overlap(
            part, part, elem_a=g, elem_b=e,
            reason="Glass pane edge seats inside the thin dark edge framing.",
        )
        ctx.allow_overlap(
            part, part, elem_a=h, elem_b=g,
            reason="Through-standoffs pass through the glass to carry the back-to-back handle.",
        )

    # Root surround overlaps the side edge frame (one connected fixed member).
    ctx.allow_overlap(
        side, side, elem_a="root_surround", elem_b="side_edge",
        reason="Hinge post / rails tie into the side panel edge frame as one fixed assembly.",
    )
    # Leaf hinge edge seats against the hinge post.
    ctx.allow_overlap(
        leaf, side, elem_a="leaf_edge", elem_b="root_surround",
        reason="Leaf hinge edge seats against the grounded hinge post it pivots on.",
    )
    # Leaf glass hinge-side edge is physically inside the pivot-post channel.
    ctx.allow_overlap(
        leaf, side, elem_a="leaf_glass", elem_b="root_surround",
        reason="Hinge-side glass edge is seated inside the pivot-post channel of the hinge post.",
    )

    # Assembly sits on the ground plane: zmin ≈ 0, total height ≈ PANEL_H.
    side_aabb = ctx.part_world_aabb(side)
    ctx.check(
        "assembly base at z=0 (side panel zmin)",
        side_aabb is not None and abs(side_aabb[0][2]) < 0.05,
        details=f"side panel aabb={side_aabb}",
    )
    ctx.check(
        "assembly height ≈ PANEL_H",
        side_aabb is not None and abs((side_aabb[1][2] - side_aabb[0][2]) - PANEL_H) < 0.15,
        details=f"dz={side_aabb[1][2] - side_aabb[0][2] if side_aabb else None}",
    )

    # Both glass panes present and large.
    side_g = ctx.part_element_world_aabb(side, elem="side_glass")
    leaf_g = ctx.part_element_world_aabb(leaf, elem="leaf_glass")
    ctx.check(
        "side glass pane present and large",
        side_g is not None
        and (side_g[1][2] - side_g[0][2]) > 1.7
        and (side_g[1][0] - side_g[0][0]) > 0.6,
        details=f"side glass aabb={side_g}",
    )
    ctx.check(
        "leaf glass pane present and large",
        leaf_g is not None
        and (leaf_g[1][2] - leaf_g[0][2]) > 1.7
        and (leaf_g[1][0] - leaf_g[0][0]) > 0.6,
        details=f"leaf glass aabb={leaf_g}",
    )

    # Both bar handles present, vertical, and mounted (touch their glass).
    for part, h, g, label in (
        (side, "side_handle", "side_glass", "side"),
        (leaf, "leaf_handle", "leaf_glass", "leaf"),
    ):
        h_aabb = ctx.part_element_world_aabb(part, elem=h)
        ctx.check(
            f"{label} bar handle present and vertical",
            h_aabb is not None and (h_aabb[1][2] - h_aabb[0][2]) > 0.9,
            details=f"{label} handle aabb={h_aabb}",
        )
        ctx.expect_contact(
            part, part, elem_a=h, elem_b=g, contact_tol=1e-3,
            name=f"{label} handle mounted to glass",
        )

    # Side panel is fixed: it is the root (not a child of any articulation).
    ctx.check(
        "side panel is fixed root",
        hinge.parent == "side_panel" and hinge.child == "glass_leaf",
        details=f"parent={hinge.parent}, child={hinge.child}",
    )

    # Closed pose: leaf sits on the +X side of the side panel.
    with ctx.pose({hinge: 0.0}):
        closed_pos = ctx.part_world_position(leaf)
        ctx.expect_origin_gap(
            leaf, side, axis="x", min_gap=0.0,
            name="leaf sits on +X side of the side panel",
        )
        ctx.expect_contact(
            leaf, side, elem_a="leaf_edge", elem_b="root_surround",
            contact_tol=1e-3,
            name="leaf hinge edge seats on hinge post",
        )

    # Open pose: leaf swings clear (free edge -> +Y) while the hinge edge stays
    # connected to the side panel.
    with ctx.pose({hinge: 1.4}):
        open_pos = ctx.part_world_position(leaf)
        open_aabb = ctx.part_world_aabb(leaf)
        ctx.check(
            "leaf swings outward when opened",
            open_aabb is not None and open_aabb[1][1] > 0.6,
            details=f"open leaf aabb={open_aabb}",
        )
        ctx.check(
            "hinge edge stays connected",
            closed_pos is not None
            and open_pos is not None
            and abs(open_pos[0] - closed_pos[0]) < 0.02
            and abs(open_pos[1] - closed_pos[1]) < 0.02,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    return ctx.report()


object_model = build_object_model()
