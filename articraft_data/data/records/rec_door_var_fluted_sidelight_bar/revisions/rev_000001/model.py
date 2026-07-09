from __future__ import annotations

# Modern dark entry door with a fixed fluted-glass sidelight.
#
# Coordinate convention (object-intrinsic):
#   +X = horizontal, across the frame (door + sidelight side by side)
#   +Y = depth, door/glass thickness direction (front of door faces +Y)
#   +Z = vertical, up  (base at z=0, top at z≈OUTER_H)
#
# The fixed dark frame is the root and carries the fluted-glass sidelight panel
# on the -X side. The operable leaf sits on the +X side and swings about a
# vertical hinge on its outer (+X) edge. The leaf carries a long vertical
# brushed-steel tubular bar pull handle on standoffs and a narrow vertical
# glass strip near its free (-X) edge.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Real-world dimensions (meters) -----------------------------------------

LEAF_W = 0.92
LEAF_H = 2.04
LEAF_T = 0.045

SIDELIGHT_W = 0.42  # visible fluted-glass aperture width
SIDELIGHT_GLASS_T = 0.020

FRAME_DEPTH = 0.12  # jamb depth (Y)
FRAME_FACE = 0.060  # visible frame face width on each member
FRAME_T = 0.070  # frame member thickness through wall (Y), <= FRAME_DEPTH

# Opening that the frame surrounds: door bay + mullion + sidelight bay.
MULLION_W = 0.055  # dark divider between door and sidelight
GAP = 0.004  # reveal gap between leaf and frame

OPENING_W = LEAF_W + MULLION_W + SIDELIGHT_W + 2.0 * GAP
OPENING_H = LEAF_H + 2.0 * GAP

OUTER_W = OPENING_W + 2.0 * FRAME_FACE
OUTER_H = OPENING_H + 2.0 * FRAME_FACE

# Z offset so base of assembly sits at z=0.
# The frame geometry is authored centered at z=0, so shift up by OUTER_H/2.
Z_OFF = OUTER_H / 2.0

# X positions. Door bay on +X, sidelight bay on -X.
DOOR_X_HI = OPENING_W / 2.0 - GAP  # door outer (hinge) edge near +X
DOOR_X_LO = DOOR_X_HI - LEAF_W  # door free edge (handle side)

SIDELIGHT_X_HI = DOOR_X_LO - GAP - MULLION_W
SIDELIGHT_X_LO = SIDELIGHT_X_HI - SIDELIGHT_W
SIDELIGHT_CX = (SIDELIGHT_X_HI + SIDELIGHT_X_LO) / 2.0

# Hinge sits just outside the door's +X edge, on the frame.
HINGE_X = DOOR_X_HI + GAP

# Door glass strip (narrow vertical strip near the free edge).
STRIP_W = 0.090
STRIP_H = 1.55
STRIP_T = 0.018
STRIP_LOCAL_X = -LEAF_W + 0.16  # measured from hinge (local +X at hinge -> leaf body at -X)

# Bar pull handle.
HANDLE_LEN = 1.30
HANDLE_R = 0.0140
STANDOFF_R = 0.0130
STANDOFF_LEN = 0.060
STANDOFF_INSET = 0.16  # from each handle end

# Leaf fluting (vertical half-round ribs on front face, matching sidelight pitch).
FLUTE_R = 0.012
FLUTE_PITCH = 0.030
FLUTE_BASE_T = LEAF_T - FLUTE_R  # thin base plate behind ribs

# --- Materials ---------------------------------------------------------------

FRAME_RGBA = (0.13, 0.13, 0.145, 1.0)  # dark anodized / graphite
DOOR_RGBA = (0.16, 0.16, 0.175, 1.0)  # dark brushed leaf face
GLASS_RGBA = (0.62, 0.70, 0.74, 0.34)  # translucent tinted glass
STEEL_RGBA = (0.78, 0.79, 0.81, 1.0)  # brushed stainless


# --- CadQuery geometry helpers ----------------------------------------------


def _frame_with_sidelight() -> cq.Workplane:
    """Fixed outer frame (perimeter + mullion) in world XZ, shifted to z=0 base.

    Width->X, height->Z, thickness extruded in Y (via XZ workplane).
    """
    # Outer slab centered at (0, Z_OFF) so bottom at z=0.
    wp = (
        cq.Workplane("XZ")
        .center(0.0, Z_OFF)
        .box(OUTER_W, OUTER_H, FRAME_T, centered=(True, True, True))
    )
    # Door bay opening cut.
    door_cx = (DOOR_X_HI + DOOR_X_LO) / 2.0 + GAP / 2.0
    door_open_w = LEAF_W + 2.0 * GAP
    wp = wp.cut(
        cq.Workplane("XZ")
        .center(door_cx, Z_OFF)
        .box(door_open_w, OPENING_H, FRAME_T + 0.02, centered=(True, True, True))
    )
    # Sidelight bay opening cut.
    wp = wp.cut(
        cq.Workplane("XZ")
        .center(SIDELIGHT_CX, Z_OFF)
        .box(SIDELIGHT_W + 2.0 * GAP, OPENING_H, FRAME_T + 0.02, centered=(True, True, True))
    )
    return wp


def _sidelight_glass() -> cq.Workplane:
    """Fluted-glass sidelight pane with vertical flutes, in world XZ frame.

    Sized slightly larger than the aperture so its edges seat into the frame
    rebate (small intentional overlap, allowed in tests). Shifted to z=0 base.
    """
    pane = (
        cq.Workplane("XZ")
        .center(SIDELIGHT_CX, Z_OFF)
        .box(SIDELIGHT_W + 0.010, OPENING_H + 0.006, SIDELIGHT_GLASS_T, centered=(True, True, True))
    )
    # Vertical flutes: subtract a row of shallow half-round grooves on +Y face.
    flute_r = 0.012
    pitch = 0.030
    n = int(SIDELIGHT_W // pitch)
    start = SIDELIGHT_CX - (n - 1) * pitch / 2.0
    front_y = SIDELIGHT_GLASS_T / 2.0
    cutters = None
    for i in range(n):
        x = start + i * pitch
        groove = (
            cq.Workplane("XZ")
            .center(x, Z_OFF)
            .workplane(offset=front_y - flute_r * 0.55)
            .circle(flute_r)
            .extrude(flute_r)
        )
        cutters = groove if cutters is None else cutters.union(groove)
    if cutters is not None:
        pane = pane.cut(cutters)
    return pane


def _mullion() -> cq.Workplane:
    """Dark vertical mullion divider between door bay and sidelight bay.

    Shifted to z=0 base.
    """
    cx = (DOOR_X_LO - GAP + SIDELIGHT_X_HI + GAP) / 2.0
    return (
        cq.Workplane("XZ")
        .center(cx, Z_OFF)
        .box(MULLION_W, OPENING_H, FRAME_T, centered=(True, True, True))
    )


def _flute_rib_center(flute_i: int, x_start: float, pitch: float, face_y: float):
    """Shared rib geometry helper: (x, y) center for flute rib at index flute_i.

    Each rib is a vertical half-round cylinder protruding from the leaf front
    face (+Y). Centers sit at regular horizontal pitch, matching the sidelight
    fluting rhythm.
    """
    return (x_start + flute_i * pitch, face_y)


def _leaf_panel() -> cq.Workplane:
    """Fluted operable leaf in its LOCAL frame: hinge edge at local X=0.

    The leaf body extends along local -X (free/handle edge at X=-LEAF_W).
    A thin base plate carries a row of evenly spaced vertical half-round flute
    ribs on the front (+Y) face, matching the sidelight fluting pitch.

    LOCAL frame: the joint origin is at world (HINGE_X, 0, Z_OFF), so local
    z=0 maps to world z=Z_OFF (mid-height). Geometry is centered at local z=0
    so the leaf spans local [-LEAF_H/2, +LEAF_H/2] in Z, which maps to world
    [0, LEAF_H] correctly.

    A narrow vertical glass aperture is cut near the free edge so the glass
    strip reads as set into the leaf.
    """
    # Extend the hinge edge a few mm past local X=0 so it seats against the
    # frame jamb at the hinge (small intentional overlap, allowed in tests).
    hinge_overlap = 0.008
    body_w = LEAF_W + hinge_overlap
    base_t = FLUTE_BASE_T
    leaf_cx = (-LEAF_W + hinge_overlap) / 2.0

    # Thin structural base plate.
    plate = (
        cq.Workplane("XZ")
        .center(leaf_cx, 0.0)
        .box(body_w, LEAF_H, base_t, centered=(True, True, True))
    )

    # --- Vertical flute ribs on front (+Y) face ---
    front_y = base_t / 2.0
    n_flutes = int(body_w // FLUTE_PITCH)
    x_start = leaf_cx - (n_flutes - 1) * FLUTE_PITCH / 2.0

    # Compute rib centers via shared helper in a for-i-in-range loop.
    rib_points = []
    for flute_i in range(n_flutes):
        rib_points.append(_flute_rib_center(flute_i, x_start, FLUTE_PITCH, front_y))

    # Extrude all flute cylinders at once (full circles, full height).
    all_cyls = (
        cq.Workplane("XY")
        .workplane(offset=-LEAF_H / 2.0)
        .pushPoints(rib_points)
        .circle(FLUTE_R)
        .extrude(LEAF_H)
    )

    # Trim back halves with a single cutter so only the front half-round
    # of each rib protrudes from the base plate face. Leave a small embed
    # depth so ribs overlap into the base plate for connected geometry.
    rib_embed = 0.003  # ribs sink 3 mm into base plate for solid union
    total_span = (n_flutes - 1) * FLUTE_PITCH + FLUTE_R * 4.0
    cutter_top_y = front_y - rib_embed
    back_cutter = (
        cq.Workplane("XY")
        .center(leaf_cx, cutter_top_y - 0.5)
        .box(total_span, 1.0, LEAF_H * 1.1, centered=(True, True, True))
    )
    trimmed_ribs = all_cyls.cut(back_cutter)

    # Union ribs with base plate into one continuous leaf solid.
    leaf = plate.union(trimmed_ribs)

    # Cut the strip aperture through the base plate and any ribs in the way.
    leaf = leaf.cut(
        cq.Workplane("XZ")
        .center(STRIP_LOCAL_X, 0.0)
        .box(STRIP_W, STRIP_H, LEAF_T + 0.02, centered=(True, True, True))
    )
    return leaf


def _leaf_glass_strip() -> cq.Workplane:
    """Narrow vertical glass strip seated in the leaf aperture (local frame).

    Slightly oversized so its edges seat into the aperture rebate (small
    intentional overlap, allowed in tests). Centered at local z=0 (maps to
    world z=Z_OFF at mid-height).
    """
    return (
        cq.Workplane("XZ")
        .center(STRIP_LOCAL_X, 0.0)
        .box(STRIP_W + 0.006, STRIP_H + 0.006, STRIP_T, centered=(True, True, True))
    )


def _build_handle() -> cq.Workplane:
    """Tubular bar pull + two standoffs, single unioned solid (leaf-local).

    Bar axis = Z (vertical). Standoff axis = Y (depth), reaching from inside
    the leaf front face out to the bar. Positioned near the free (handle) edge.
    Bar centered at local z=0 (maps to world z=Z_OFF at mid-height of door).
    """
    handle_x = -LEAF_W + 0.105
    face_y = FLUTE_BASE_T / 2.0  # mount between ribs on the base plate face
    bar_y = face_y + STANDOFF_LEN + HANDLE_R
    handle_z0 = -HANDLE_LEN / 2.0  # local: centered at z=0

    # Vertical bar.
    bar = cq.Solid.makeCylinder(
        HANDLE_R, HANDLE_LEN, cq.Vector(handle_x, bar_y, handle_z0), cq.Vector(0, 0, 1)
    )
    asm = cq.Workplane("XY").add(bar)

    y0 = face_y - 0.006  # start just inside the leaf face for solid contact
    length = bar_y - y0
    for sz in (handle_z0 + STANDOFF_INSET, handle_z0 + HANDLE_LEN - STANDOFF_INSET):
        so = cq.Solid.makeCylinder(
            STANDOFF_R, length, cq.Vector(handle_x, y0, sz), cq.Vector(0, 1, 0)
        )
        asm = asm.union(cq.Workplane("XY").add(so))
    return asm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="entry_door_with_sidelight")

    frame_mat = model.material("frame_graphite", rgba=FRAME_RGBA)
    door_mat = model.material("leaf_dark", rgba=DOOR_RGBA)
    glass_mat = model.material("glass_tinted", rgba=GLASS_RGBA)
    steel_mat = model.material("brushed_steel", rgba=STEEL_RGBA)

    # --- Root: fixed frame + mullion + fluted sidelight glass ---------------
    frame = model.part("door_frame")
    frame.visual(mesh_from_cadquery(_frame_with_sidelight(), "frame"), material=frame_mat, name="frame_perimeter")
    frame.visual(mesh_from_cadquery(_mullion(), "mullion"), material=frame_mat, name="mullion")
    frame.visual(
        mesh_from_cadquery(_sidelight_glass(), "sidelight_glass"),
        material=glass_mat,
        name="sidelight_glass",
    )

    # --- Operable leaf ------------------------------------------------------
    leaf = model.part("door_leaf")
    leaf.visual(mesh_from_cadquery(_leaf_panel(), "leaf_panel"), material=door_mat, name="leaf_panel")
    leaf.visual(
        mesh_from_cadquery(_leaf_glass_strip(), "leaf_glass_strip"),
        material=glass_mat,
        name="leaf_glass_strip",
    )
    leaf.visual(
        mesh_from_cadquery(_build_handle(), "bar_handle"),
        material=steel_mat,
        name="bar_handle",
    )

    # --- Articulation: leaf swings about vertical hinge on its +X edge -------
    # Hinge origin placed at world-space hinge column at mid-height (Z_OFF).
    # Leaf body extends along local -X from hinge at local X=0. Axis -Z makes
    # positive q swing the free edge toward +Y (outward, toward the viewer).
    model.articulation(
        "frame_to_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(HINGE_X, 0.0, Z_OFF)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("door_frame")
    leaf = object_model.get_part("door_leaf")
    hinge = object_model.get_articulation("frame_to_leaf")

    # Intentional small seating overlaps.
    ctx.allow_overlap(
        frame,
        frame,
        elem_a="sidelight_glass",
        elem_b="frame_perimeter",
        reason="Sidelight pane seats into the frame rebate around its aperture.",
    )
    ctx.allow_overlap(
        leaf,
        leaf,
        elem_a="leaf_glass_strip",
        elem_b="leaf_panel",
        reason="Narrow glass strip seats into the leaf aperture rebate.",
    )
    ctx.allow_overlap(
        leaf,
        leaf,
        elem_a="bar_handle",
        elem_b="leaf_panel",
        reason="Handle standoffs are anchored a few mm into the leaf face.",
    )
    ctx.allow_overlap(
        leaf,
        frame,
        elem_a="leaf_panel",
        elem_b="frame_perimeter",
        reason="Leaf hinge edge seats against the frame jamb at the hinge.",
    )

    # Assembly sits on the ground plane: zmin ≈ 0, height ≈ OUTER_H.
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "assembly base at z=0 (frame zmin)",
        frame_aabb is not None and abs(frame_aabb[0][2]) < 0.05,
        details=f"frame aabb={frame_aabb}",
    )
    ctx.check(
        "assembly height ≈ OUTER_H",
        frame_aabb is not None and abs((frame_aabb[1][2] - frame_aabb[0][2]) - OUTER_H) < 0.15,
        details=f"dz={frame_aabb[1][2] - frame_aabb[0][2] if frame_aabb else None}",
    )

    # Sidelight glass present and reasonably sized.
    sl_aabb = ctx.part_element_world_aabb(frame, elem="sidelight_glass")
    ctx.check(
        "sidelight glass present and tall",
        sl_aabb is not None and (sl_aabb[1][2] - sl_aabb[0][2]) > 1.6,
        details=f"sidelight aabb={sl_aabb}",
    )
    # Sidelight is fixed: it belongs to the root frame, not the leaf.
    ctx.check(
        "sidelight is part of fixed frame",
        any(v.name == "sidelight_glass" for v in frame.visuals),
        details="sidelight_glass must be a visual on door_frame",
    )

    # Leaf glass strip present and sized.
    strip_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_glass_strip")
    ctx.check(
        "leaf glass strip present and tall",
        strip_aabb is not None and (strip_aabb[1][2] - strip_aabb[0][2]) > 1.4,
        details=f"strip aabb={strip_aabb}",
    )

    # Leaf panel has fluted ribs: Y-extent must exceed the base plate alone,
    # proving the half-round ribs protrude from the front face.
    panel_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_panel")
    if panel_aabb is not None:
        panel_dy = panel_aabb[1][1] - panel_aabb[0][1]
        ctx.check(
            "leaf panel has flute rib protrusion",
            panel_dy > FLUTE_BASE_T + FLUTE_R * 0.5,
            details=f"panel Y extent={panel_dy:.4f}, expected > {FLUTE_BASE_T + FLUTE_R * 0.5:.4f}",
        )
        ctx.check(
            "leaf panel flute envelope within expected range",
            panel_dy < FLUTE_BASE_T + FLUTE_R * 1.5,
            details=f"panel Y extent={panel_dy:.4f}, expected < {FLUTE_BASE_T + FLUTE_R * 1.5:.4f}",
        )

    # Bar handle present, vertical, and mounted to the leaf (contacts leaf panel).
    handle_aabb = ctx.part_element_world_aabb(leaf, elem="bar_handle")
    ctx.check(
        "bar handle present and vertical",
        handle_aabb is not None and (handle_aabb[1][2] - handle_aabb[0][2]) > 1.0,
        details=f"handle aabb={handle_aabb}",
    )
    ctx.expect_contact(
        leaf,
        leaf,
        elem_a="bar_handle",
        elem_b="leaf_panel",
        contact_tol=1e-3,
        name="handle standoffs touch leaf face",
    )

    # Closed pose: hinge edge seats against the frame jamb, and the leaf spans
    # the frame height.
    with ctx.pose({hinge: 0.0}):
        closed_pos = ctx.part_world_position(leaf)
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="leaf_panel",
            elem_b="frame_perimeter",
            contact_tol=1e-3,
            name="leaf hinge edge seats on frame jamb",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="z",
            elem_a="leaf_panel",
            elem_b="frame_perimeter",
            min_overlap=1.5,
            name="closed leaf spans frame height",
        )

    # Open pose: leaf swings clear (free edge moves to +Y) while hinge edge
    # stays connected to the frame.
    with ctx.pose({hinge: 1.4}):
        open_pos = ctx.part_world_position(leaf)
        open_aabb = ctx.part_world_aabb(leaf)
        ctx.check(
            "leaf swings outward when opened",
            open_aabb is not None and open_aabb[1][1] > 0.6,
            details=f"open leaf aabb={open_aabb}",
        )
        # Hinge edge stays put: leaf origin (at hinge) barely moves.
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
