from __future__ import annotations

# Classic double entry door – single-active-leaf variant.
#
# Articraft brief:
# - Object: classic double entry door, opening ~1.70 m wide x 2.10 m tall,
#   each leaf ~0.84 m wide x 0.045 m thick x 2.08 m tall, in a walnut frame.
#   One active leaf swings on a revolute hinge; the other is fixed to the
#   frame with a vertical astragal meeting bead.
# - Root/support: fixed wood frame (two side jambs, head jamb, carved base
#   threshold, doorstop beads) carries the active leaf on its hinge and
#   incorporates the inactive leaf, its hardware, and the astragal as inline
#   visuals.
# - Parts: frame (root), door_0 (active leaf), door_0_lever (active lever).
#   The inactive leaf, its handle, its lever bar, and the astragal are inline
#   visuals on the frame – no separate part or FIXED joint.
# - Articulations: frame_to_door_0 REVOLUTE, vertical (+Z) hinge axis on the
#   left jamb outer edge, opening outward. door_0_to_lever REVOLUTE,
#   horizontal spindle axis through the rose, positive q presses the lever
#   tip downward.
# - Visible geometry: dark walnut frame, walnut leaves with crisp raised
#   panels and stiles/rails, brass escutcheons and lever hardware on both
#   leaves, a vertical astragal strip with bead at the inactive-leaf meeting
#   edge projecting past center to overlap the active leaf when closed.
# - Support/fit: active leaf hinge edge meets the left jamb; inactive leaf
#   is permanently fixed to the frame at the right side; the astragal covers
#   the center reveal.
# - Intentional overlaps: the astragal strip is seated 1 mm into the inactive
#   leaf face (same part) and projects ~16 mm past the meeting edge, creating
#   a small local overlap with the active leaf front face at the meeting stile
#   when closed (scoped allowance + proof check).
# - Tests: both leaves present and sized, astragal present with correct
#   dimensions at the meeting edge, active leaf swings on its hinge, inactive
#   leaf stays fixed during active-leaf poses, lever articulates, astragal
#   seated overlap is minimal.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Dimensions (meters)
# ----------------------------------------------------------------------------
OPENING_W = 1.70           # clear double-door opening width
OPENING_H = 2.10           # clear opening height
LEAF_GAP = 0.010           # reveal at the central meeting stiles (per leaf side)
CENTER_REVEAL = 0.006      # gap between the two leaves at the center
JAMB_REVEAL = 0.006        # gap between each leaf hinge edge and its jamb face

LEAF_W = (OPENING_W - CENTER_REVEAL - 2 * JAMB_REVEAL) / 2.0  # ~0.841 m
LEAF_H = OPENING_H - 0.02  # leaf slightly shorter than the opening
LEAF_T = 0.045             # leaf thickness

JAMB_W = 0.12              # frame jamb face width
JAMB_D = 0.18              # frame depth (front to back)
HEAD_H = 0.14              # head jamb height
BASE_H = 0.16              # carved base threshold height

FLOOR_Z = 0.0

# Frame outer footprint
FRAME_OUTER_W = OPENING_W + 2 * JAMB_W
FRAME_OUTER_H = OPENING_H + HEAD_H + BASE_H

# Hinge X positions (world / frame coordinates)
HALF_OPEN = OPENING_W / 2.0
LEFT_HINGE_X = -(HALF_OPEN - JAMB_REVEAL)
RIGHT_HINGE_X = HALF_OPEN - JAMB_REVEAL

WALNUT = (0.28, 0.16, 0.09, 1.0)
WALNUT_DARK = (0.20, 0.11, 0.06, 1.0)
BRASS = (0.78, 0.60, 0.22, 1.0)

HANDLE_Z = LEAF_H * 0.45        # escutcheon center height
SPINDLE_Z = HANDLE_Z + 0.03     # lever spindle (rose center) height


# ----------------------------------------------------------------------------
# CadQuery builders
# ----------------------------------------------------------------------------
def _build_frame_members() -> dict[str, cq.Workplane]:
    """Fixed walnut frame as separate members so exact collisions do not bridge
    the opening. Returns {name: shape}.

    Members: two jambs, head jamb, carved base rail, and three doorstop beads.
    """
    half_open = OPENING_W / 2.0
    members: dict[str, cq.Workplane] = {}

    # Left/right jambs (vertical posts beside the opening)
    members["jamb_left"] = (
        cq.Workplane("XY")
        .box(JAMB_W, JAMB_D, OPENING_H)
        .translate((-(half_open + JAMB_W / 2.0), 0.0, OPENING_H / 2.0))
    )
    members["jamb_right"] = (
        cq.Workplane("XY")
        .box(JAMB_W, JAMB_D, OPENING_H)
        .translate((half_open + JAMB_W / 2.0, 0.0, OPENING_H / 2.0))
    )
    # Head jamb across the top
    members["head_jamb"] = (
        cq.Workplane("XY")
        .box(FRAME_OUTER_W, JAMB_D, HEAD_H)
        .translate((0.0, 0.0, OPENING_H + HEAD_H / 2.0))
    )
    # Carved base rail with two decorative grooves on the front face
    base = (
        cq.Workplane("XY")
        .box(FRAME_OUTER_W, JAMB_D, BASE_H)
        .translate((0.0, 0.0, -BASE_H / 2.0))
    )
    groove = (
        cq.Workplane("XY")
        .box(FRAME_OUTER_W - 0.04, 0.02, 0.018)
        .translate((0.0, JAMB_D / 2.0 - 0.01, -BASE_H / 2.0 + 0.04))
    )
    base = base.cut(groove).cut(groove.translate((0.0, 0.0, 0.05)))
    members["base_rail"] = base

    # Doorstop beads behind the closed leaves (negative Y), one per side + head.
    stop_t = 0.014
    stop_proud = 0.02
    stop_y = -LEAF_T / 2.0 - stop_t / 2.0 - 0.001
    members["stop_left"] = (
        cq.Workplane("XY")
        .box(stop_proud, stop_t, OPENING_H - 0.02)
        .translate((-half_open + stop_proud / 2.0, stop_y, OPENING_H / 2.0))
    )
    members["stop_right"] = (
        cq.Workplane("XY")
        .box(stop_proud, stop_t, OPENING_H - 0.02)
        .translate((half_open - stop_proud / 2.0, stop_y, OPENING_H / 2.0))
    )
    members["stop_head"] = (
        cq.Workplane("XY")
        .box(OPENING_W, stop_t, stop_proud)
        .translate((0.0, stop_y, OPENING_H - stop_proud / 2.0))
    )
    return members


def _build_leaf(sign: float) -> cq.Workplane:
    """One walnut leaf with three raised panels and stiles/rails.

    Local frame:
      - hinge edge at local X=0
      - leaf body extends toward `sign` (+1 => +X, -1 => -X)
      - thickness along Y, centered; front face = +Y for BOTH leaves
      - height along Z in [0, LEAF_H]

    The meeting (inner) edge is always the end nearest the center, i.e. at
    X = sign * LEAF_W. `sign` mirrors the second leaf while keeping a common
    world-facing front so both brass handles sit on the same side.
    """
    cx = sign * LEAF_W / 2.0
    leaf = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((cx, 0.0, 0.0))
    )

    # Raised-panel layout: stiles (vertical) and rails (horizontal) frame the
    # three stacked panels. Each panel reads as a classic RAISED/fielded panel:
    #   1. a shallow recessed field is cut into the front face, leaving the
    #      stiles and rails proud as a flat border;
    #   2. a proud molding ring (a thin frame just inside the recess) lines the
    #      field like applied bolection molding;
    #   3. a raised fielded pad sits in the middle, stepping back up so the
    #      panel center is proud of the recess floor but below the stile face.
    stile = 0.11           # side/center stile width
    rail = 0.13            # top/bottom/mid rail height
    field_inset_y = 0.014  # recess depth into the leaf front face
    molding_proud = 0.006  # how proud the molding ring sits above recess floor
    pad_proud = 0.010      # how proud the fielded pad center sits

    face_y = LEAF_T / 2.0  # front face of the leaf
    floor_y = face_y - field_inset_y  # recessed field floor (front of pad base)

    # Vertical extent split into 3 panels with rails between.
    n_panels = 3
    usable_h = LEAF_H - 2 * rail - (n_panels - 1) * rail
    panel_h = usable_h / n_panels
    panel_w = LEAF_W - 2 * stile

    panel_centers_z = []
    z = rail + panel_h / 2.0
    for _ in range(n_panels):
        panel_centers_z.append(z)
        z += panel_h + rail

    panel_center_x = cx

    for cz in panel_centers_z:
        # 1. Recessed field cut into the front face.
        field = (
            cq.Workplane("XY")
            .box(panel_w, field_inset_y * 2.0, panel_h)
            .translate((panel_center_x, face_y, cz))
        )
        leaf = leaf.cut(field)

        # 2. Proud molding ring lining the field (outer frame minus inner hole),
        #    standing proud of the recess floor like applied bolection molding.
        mold_outer = (
            cq.Workplane("XY")
            .box(panel_w, molding_proud * 2.0, panel_h)
            .translate((panel_center_x, floor_y + molding_proud, cz))
        )
        mold_inner = (
            cq.Workplane("XY")
            .box(panel_w - 0.05, molding_proud * 4.0, panel_h - 0.05)
            .translate((panel_center_x, floor_y + molding_proud, cz))
        )
        molding = mold_outer.cut(mold_inner)
        leaf = leaf.union(molding)

        # 3. Raised fielded pad in the center, proud of the recess floor.
        pad = (
            cq.Workplane("XY")
            .box(panel_w - 0.08, (pad_proud + (face_y - floor_y)) , panel_h - 0.08)
            .translate(
                (panel_center_x, floor_y + (pad_proud + (face_y - floor_y)) / 2.0, cz)
            )
        )
        leaf = leaf.union(pad)

    # Soften the visible outer vertical edges of the leaf.
    try:
        leaf = leaf.edges("|Z").fillet(0.004)
    except Exception:
        pass

    return leaf


def _plate_x(sign: float) -> float:
    """Leaf-local X of the escutcheon/rose center, on the meeting stile."""
    return sign * (LEAF_W - 0.055)


def _build_handle_fixed(sign: float) -> cq.Workplane:
    """Brass escutcheon plate + rose, fixed to the leaf, in leaf-local coords.

    Placed near the meeting edge (X = sign*LEAF_W), on the front (+Y) face,
    centered at the handle height. The moving lever bar is a separate part.
    """
    plate_x = _plate_x(sign)
    face_y = LEAF_T / 2.0

    # Escutcheon: tall narrow brass plate on the meeting stile
    escutcheon = (
        cq.Workplane("XY")
        .box(0.04, 0.010, 0.18)
        .translate((plate_x, face_y + 0.003, HANDLE_Z))
    )
    # Keyhole detail cut into escutcheon
    keyhole = (
        cq.Workplane("XY")
        .cylinder(0.020, 0.004)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((plate_x, face_y + 0.006, HANDLE_Z - 0.04))
    )
    escutcheon = escutcheon.cut(keyhole)

    # Rose: fixed collar the lever spindle passes through
    rose = (
        cq.Workplane("XY")
        .cylinder(0.012, 0.030)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((plate_x, face_y + 0.006, SPINDLE_Z))
    )
    return escutcheon.union(rose)


def _build_lever(sign: float) -> cq.Workplane:
    """Brass lever bar + neck, in lever-local coords.

    Local origin sits ON the spindle axis at the rose center; the spindle axis
    is local Y (out of the leaf face), so a revolute joint about Y swings the
    bar tip up/down. The bar projects toward the leaf body (away from the
    meeting edge), mirrored by `sign` like the leaves.
    """
    face_y = LEAF_T / 2.0
    neck = (
        cq.Workplane("XY")
        .cylinder(0.018, 0.014)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, face_y + 0.016, 0.0))
    )
    lever = (
        cq.Workplane("XY")
        .box(0.085, 0.016, 0.016)
        .translate((-sign * 0.05, face_y + 0.022, 0.0))
    )
    return neck.union(lever)


def _build_astragal() -> cq.Workplane:
    """Vertical astragal meeting bead in world/frame coordinates.

    A narrow walnut strip with a half-round bead on the exposed edge, mounted
    on the front face of the inactive (right) leaf at its meeting edge. It
    projects past the meeting edge toward the active leaf to cover the center
    reveal when the active leaf is closed.
    """
    astragal_w = 0.038        # total strip width
    astragal_t = 0.012        # strip thickness (proud of face)
    astragal_h = LEAF_H - 0.04  # nearly full leaf height
    overlap = 0.016           # projection past meeting edge toward active leaf

    face_y = LEAF_T / 2.0
    meeting_x = RIGHT_HINGE_X - LEAF_W  # inactive leaf meeting edge in world

    # Strip positioned so it is seated 1 mm into the inactive leaf face
    # (connected within the same frame part) and projects past the meeting edge.
    inner_x = meeting_x - overlap           # edge nearest the active leaf
    cx = inner_x + astragal_w / 2.0
    cy = face_y + astragal_t / 2.0 - 0.001  # 1 mm seated embed
    cz = astragal_h / 2.0 + 0.02            # vertically centered with offset

    strip = (
        cq.Workplane("XY")
        .box(astragal_w, astragal_t, astragal_h)
        .translate((cx, cy, cz))
    )

    # Decorative half-round bead on the exposed inner edge
    bead_r = 0.005
    bead = (
        cq.Workplane("XY")
        .cylinder(astragal_h, bead_r)
        .translate((inner_x, cy, cz))
    )

    return strip.union(bead)


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_double_entry_door")

    model.material("walnut", rgba=WALNUT)
    model.material("walnut_dark", rgba=WALNUT_DARK)
    model.material("brass", rgba=BRASS)

    # --- Frame (root) ---
    frame = model.part("frame")
    for mname, mshape in _build_frame_members().items():
        frame.visual(
            mesh_from_cadquery(mshape, f"frame_{mname}"),
            material="walnut_dark",
            name=f"frame_{mname}",
        )

    # --- Inactive leaf (fixed to frame, inline visuals) ---
    # The inactive leaf is on the right side (sign=-1). It does not swing;
    # its leaf body, hardware, and the astragal are all frame visuals.
    inactive_sign = -1.0

    frame.visual(
        mesh_from_cadquery(
            _build_leaf(inactive_sign).translate((RIGHT_HINGE_X, 0.0, 0.0)),
            "frame_inactive_leaf",
        ),
        material="walnut",
        name="frame_inactive_leaf",
    )
    frame.visual(
        mesh_from_cadquery(
            _build_handle_fixed(inactive_sign).translate((RIGHT_HINGE_X, 0.0, 0.0)),
            "frame_inactive_handle",
        ),
        material="brass",
        name="frame_inactive_handle",
    )
    # Fixed lever bar on inactive leaf (in world coordinates)
    inactive_spindle_x = RIGHT_HINGE_X + _plate_x(inactive_sign)
    frame.visual(
        mesh_from_cadquery(
            _build_lever(inactive_sign).translate(
                (inactive_spindle_x, 0.0, SPINDLE_Z)
            ),
            "frame_inactive_lever",
        ),
        material="brass",
        name="frame_inactive_lever",
    )
    # Astragal meeting bead
    frame.visual(
        mesh_from_cadquery(_build_astragal(), "frame_astragal"),
        material="walnut",
        name="frame_astragal",
    )

    # --- Active leaf (door_0) ---
    active_sign = 1.0

    door_0 = model.part("door_0")
    door_0.visual(
        mesh_from_cadquery(_build_leaf(active_sign), "door_0_leaf"),
        material="walnut",
        name="door_0_leaf",
    )
    door_0.visual(
        mesh_from_cadquery(_build_handle_fixed(active_sign), "door_0_handle"),
        material="brass",
        name="door_0_handle",
    )

    # Rotating lever handle on active leaf: its own part, hinged on the
    # spindle through the rose.
    lever_0 = model.part("door_0_lever")
    lever_0.visual(
        mesh_from_cadquery(_build_lever(active_sign), "door_0_lever_bar"),
        material="brass",
        name="door_0_lever_bar",
    )
    model.articulation(
        "door_0_to_lever",
        ArticulationType.REVOLUTE,
        parent=door_0,
        child=lever_0,
        origin=Origin(xyz=(_plate_x(active_sign), 0.0, SPINDLE_Z)),
        axis=(0.0, -active_sign, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=0.7),
    )

    # Hinge: active leaf on left jamb, leaf extends +X; +Z axis swings free
    # edge +Y (outward).
    model.articulation(
        "frame_to_door_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door_0,
        origin=Origin(xyz=(LEFT_HINGE_X, 0.0, FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.92),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    lever_0 = object_model.get_part("door_0_lever")
    hinge_0 = object_model.get_articulation("frame_to_door_0")
    lever_j0 = object_model.get_articulation("door_0_to_lever")

    leaf0 = door_0.get_visual("door_0_leaf")
    handle0 = door_0.get_visual("door_0_handle")
    inactive_leaf = frame.get_visual("frame_inactive_leaf")
    inactive_handle = frame.get_visual("frame_inactive_handle")
    astragal = frame.get_visual("frame_astragal")

    # --- Structural change: one active leaf, one fixed inactive leaf ---
    ctx.check(
        "inactive leaf visual present on frame",
        inactive_leaf is not None,
    )
    ctx.check(
        "inactive handle visual present on frame",
        inactive_handle is not None,
    )
    ctx.check(
        "astragal visual present on frame",
        astragal is not None,
    )

    # --- Active leaf sizing (raised-panel leaf present and sized) ---
    aabb0 = ctx.part_element_world_aabb(door_0, elem=leaf0)
    assert aabb0 is not None
    lo0, hi0 = aabb0
    w0 = hi0[0] - lo0[0]
    h0 = hi0[2] - lo0[2]
    t0 = hi0[1] - lo0[1]
    ctx.check(
        "door_0 leaf width plausible",
        0.75 <= w0 <= 0.95,
        details=f"width={w0:.3f}",
    )
    ctx.check(
        "door_0 leaf height plausible",
        1.9 <= h0 <= 2.15,
        details=f"height={h0:.3f}",
    )
    ctx.check(
        "door_0 leaf thickness plausible (raised panels add depth)",
        0.04 <= t0 <= 0.09,
        details=f"thickness={t0:.3f}",
    )

    # --- Inactive leaf sizing ---
    aabb_i = ctx.part_element_world_aabb(frame, elem=inactive_leaf)
    assert aabb_i is not None
    lo_i, hi_i = aabb_i
    w_i = hi_i[0] - lo_i[0]
    h_i = hi_i[2] - lo_i[2]
    ctx.check(
        "inactive leaf width plausible",
        0.75 <= w_i <= 0.95,
        details=f"width={w_i:.3f}",
    )
    ctx.check(
        "inactive leaf height plausible",
        1.9 <= h_i <= 2.15,
        details=f"height={h_i:.3f}",
    )

    # --- Both leaves at matching heights and same front face ---
    c0_z = (lo0[2] + hi0[2]) / 2.0
    ci_z = (lo_i[2] + hi_i[2]) / 2.0
    ctx.check(
        "both leaves at same height center",
        abs(c0_z - ci_z) < 0.02,
        details=f"active_cz={c0_z:.3f} inactive_cz={ci_z:.3f}",
    )
    c0_y = (lo0[1] + hi0[1]) / 2.0
    ci_y = (lo_i[1] + hi_i[1]) / 2.0
    ctx.check(
        "both leaves on same front face (matching Y center)",
        abs(c0_y - ci_y) < 0.02,
        details=f"active_cy={c0_y:.3f} inactive_cy={ci_y:.3f}",
    )

    # --- Active leaf on left, inactive on right ---
    c0_x = (lo0[0] + hi0[0]) / 2.0
    ci_x = (lo_i[0] + hi_i[0]) / 2.0
    ctx.check(
        "active leaf on left side (negative X)",
        c0_x < -0.2,
        details=f"active_cx={c0_x:.3f}",
    )
    ctx.check(
        "inactive leaf on right side (positive X)",
        ci_x > 0.2,
        details=f"inactive_cx={ci_x:.3f}",
    )

    # --- Astragal dimensions and position ---
    ast_aabb = ctx.part_element_world_aabb(frame, elem=astragal)
    assert ast_aabb is not None
    ast_lo, ast_hi = ast_aabb
    ast_w = ast_hi[0] - ast_lo[0]
    ast_h = ast_hi[2] - ast_lo[2]
    ast_cx = (ast_lo[0] + ast_hi[0]) / 2.0
    ctx.check(
        "astragal has full vertical extent",
        ast_h > 1.5,
        details=f"height={ast_h:.3f}",
    )
    ctx.check(
        "astragal has narrow width",
        0.02 <= ast_w <= 0.06,
        details=f"width={ast_w:.3f}",
    )
    ctx.check(
        "astragal positioned at center meeting edge",
        abs(ast_cx) < 0.05,
        details=f"cx={ast_cx:.3f}",
    )
    # Astragal projects past the inactive leaf meeting edge toward the active leaf
    ctx.check(
        "astragal inner edge extends past center toward active leaf",
        ast_lo[0] < 0.0,
        details=f"astragal_min_x={ast_lo[0]:.4f}",
    )

    # --- Active leaf brass hardware ---
    h_aabb = ctx.part_element_world_aabb(door_0, elem=handle0)
    assert h_aabb is not None
    hlo, hhi = h_aabb
    ctx.check(
        "door_0 brass hardware has escutcheon height",
        (hhi[2] - hlo[2]) >= 0.12,
        details=f"hardware_z_extent={hhi[2]-hlo[2]:.3f}",
    )
    ctx.check(
        "door_0 escutcheon/rose sit proud of front face",
        (hhi[1] - hlo[1]) >= 0.005,
        details=f"hardware_y_extent={hhi[1]-hlo[1]:.3f}",
    )

    # --- Lever on active leaf articulates ---
    with ctx.pose({hinge_0: 0.0, lever_j0: 0.0}):
        rest = ctx.part_world_aabb(lever_0)
    assert rest is not None
    ctx.check(
        "door_0 lever bar projects from front face",
        (rest[1][1] - rest[0][1]) >= 0.02,
        details=f"lever_y_extent={rest[1][1]-rest[0][1]:.3f}",
    )
    ctx.check(
        "door_0 lever bar has horizontal reach at rest",
        (rest[1][0] - rest[0][0]) >= 0.07,
        details=f"lever_x_extent={rest[1][0]-rest[0][0]:.3f}",
    )
    with ctx.pose({hinge_0: 0.0, lever_j0: 0.7}):
        pressed = ctx.part_world_aabb(lever_0)
    assert pressed is not None
    ctx.check(
        "door_0 lever tip swings down when pressed",
        pressed[0][2] < rest[0][2] - 0.02,
        details=f"rest_minZ={rest[0][2]:.3f} pressed_minZ={pressed[0][2]:.3f}",
    )

    # --- Closed pose: active leaf meets frame, inactive stays fixed ---
    with ctx.pose({hinge_0: 0.0}):
        ctx.expect_contact(
            door_0, frame, contact_tol=0.02,
            name="door_0 hinge edge meets frame jamb when closed",
        )
        # Astragal projects past center toward the active leaf meeting edge
        ctx.expect_overlap(
            frame, door_0,
            axes="x",
            elem_a="frame_astragal",
            elem_b="door_0_leaf",
            min_overlap=0.010,
            name="astragal overlaps active leaf meeting edge in X when closed",
        )
        # Active leaf meeting edge is behind the astragal in X
        ctx.expect_overlap(
            door_0, frame,
            axes="z",
            elem_a="door_0_leaf",
            elem_b="frame_astragal",
            min_overlap=1.5,
            name="active leaf and astragal overlap vertically when closed",
        )

    # --- Open pose: only active leaf swings, inactive stays fixed ---
    open_q = 1.4
    with ctx.pose({hinge_0: 0.0}):
        closed_pos = ctx.part_world_aabb(door_0)
        closed_inactive_aabb = ctx.part_element_world_aabb(frame, elem=inactive_leaf)

    with ctx.pose({hinge_0: open_q}):
        open_pos = ctx.part_world_aabb(door_0)
        open_inactive_aabb = ctx.part_element_world_aabb(frame, elem=inactive_leaf)

        assert closed_pos is not None and open_pos is not None
        assert closed_inactive_aabb is not None and open_inactive_aabb is not None

        ctx.check(
            "door_0 swings outward when opened",
            open_pos[1][1] > closed_pos[1][1] + 0.3,
            details=f"closed_maxY={closed_pos[1][1]:.3f} open_maxY={open_pos[1][1]:.3f}",
        )
        # Inactive leaf does not move when active leaf opens
        ctx.check(
            "inactive leaf stays fixed during active leaf swing",
            abs(closed_inactive_aabb[0][0] - open_inactive_aabb[0][0]) < 0.001
            and abs(closed_inactive_aabb[1][1] - open_inactive_aabb[1][1]) < 0.001,
            details=(
                f"closed_minX={closed_inactive_aabb[0][0]:.4f} "
                f"open_minX={open_inactive_aabb[0][0]:.4f}"
            ),
        )
        ctx.expect_contact(
            door_0, frame, contact_tol=0.05,
            name="door_0 hinge edge stays at jamb when open",
        )

    # --- Intentional overlap allowances ---

    # Lever neck wraps around the rose spindle (mechanical nesting, same as parent).
    ctx.allow_overlap(
        door_0, lever_0,
        elem_a="door_0_handle",
        elem_b="door_0_lever_bar",
        reason=(
            "The lever neck cylinder wraps around the rose/escutcheon spindle "
            "like a real door handle mechanism; this is intentional mechanical "
            "nesting with minimal overlap volume."
        ),
    )
    ctx.expect_contact(
        door_0, lever_0,
        elem_a="door_0_handle",
        elem_b="door_0_lever_bar",
        contact_tol=0.001,
        name="lever neck seats on rose spindle",
    )

    # Astragal meeting bead seated into inactive leaf face and projecting past
    # the meeting edge, creating a small local overlap with the active leaf at
    # the meeting stile when closed.
    ctx.allow_overlap(
        frame, door_0,
        elem_a="frame_astragal",
        elem_b="door_0_leaf",
        reason=(
            "The astragal meeting bead is seated against the inactive leaf face "
            "and projects past the meeting edge, creating a small intentional "
            "overlap with the active leaf front face at the meeting stile when "
            "closed, matching real astragal installation."
        ),
    )
    ctx.expect_contact(
        frame, door_0,
        elem_a="frame_astragal",
        elem_b="door_0_leaf",
        contact_tol=0.005,
        name="astragal contacts active leaf at meeting stile",
    )

    return ctx.report()


object_model = build_object_model()
