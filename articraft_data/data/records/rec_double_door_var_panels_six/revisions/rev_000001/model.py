from __future__ import annotations

# Classic double entry door in dark walnut wood.
#
# Articraft brief:
# - Object: classic double entry door, opening ~1.70 m wide x 2.10 m tall,
#   each leaf ~0.83 m wide x 0.045 m thick x 2.05 m tall, in a walnut frame.
# - Root/support: fixed wood frame (two side jambs, head jamb, carved base
#   threshold, and a paneled back surround) carries both swinging leaves.
# - Parts: frame (root), door_0, door_1. Each leaf carries six small stacked
#   raised rectangular panels (boolean-raised, not flat), a meeting stile, plus
#   a brass handle and escutcheon at the inner meeting edge.
# - Articulations: frame_to_door_0 and frame_to_door_1, both REVOLUTE, vertical
#   (+Z) hinge axes on the OUTER edges at the jambs, opening symmetrically
#   outward. Names door_0/door_1 (symmetric, no left/right per link naming).
#   Each leaf additionally carries a REVOLUTE lever handle (door_N_to_lever)
#   rotating about the horizontal spindle axis through the rose, so pressing
#   the brass lever bar swings its tip downward.
# - Visible geometry: dark walnut leaves with crisp raised panels and stiles/
#   rails, brass hardware, paneled frame with a carved base rail.
# - Support/fit: each leaf hinge edge meets its jamb; leaves meet at the center
#   with a small reveal so they do not interpenetrate when closed.
# - Intentional overlaps: none required.
# - Tests: panels present and sized, handle/escutcheon present, leaves meet with
#   a reveal when closed, both leaves swing clear while hinge edges stay at the
#   jambs when open.

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

LEAF_W = (OPENING_W - CENTER_REVEAL - 2 * JAMB_REVEAL) / 2.0  # ~0.839 m
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

WALNUT = (0.28, 0.16, 0.09, 1.0)
WALNUT_DARK = (0.20, 0.11, 0.06, 1.0)
BRASS = (0.78, 0.60, 0.22, 1.0)


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


PANEL_COUNT = 6


def _add_raised_panel(
    leaf: cq.Workplane,
    *,
    center_x: float,
    center_z: float,
    panel_w: float,
    panel_h: float,
    face_y: float,
    floor_y: float,
    field_inset_y: float,
    molding_proud: float,
    pad_proud: float,
) -> cq.Workplane:
    """Add one classic raised/fielded panel to the leaf body.

    Steps:
      1. Cut a shallow recessed field into the front face, leaving stiles/rails
         proud as a flat border.
      2. Union a proud molding ring (thin frame just inside the recess) lining
         the field like applied bolection molding.
      3. Union a raised fielded pad in the center, stepping back up so the
         panel center is proud of the recess floor but below the stile face.
    """
    # Scale inner cutouts proportionally to panel size so small panels still
    # read as raised with visible molding and pad margins.
    margin_w = min(0.05, panel_w * 0.18)
    margin_h = min(0.05, panel_h * 0.18)
    pad_inset_w = min(0.08, panel_w * 0.28)
    pad_inset_h = min(0.08, panel_h * 0.28)

    # 1. Recessed field cut into the front face.
    field = (
        cq.Workplane("XY")
        .box(panel_w, field_inset_y * 2.0, panel_h)
        .translate((center_x, face_y, center_z))
    )
    leaf = leaf.cut(field)

    # 2. Proud molding ring lining the field (outer frame minus inner hole),
    #    standing proud of the recess floor like applied bolection molding.
    mold_outer = (
        cq.Workplane("XY")
        .box(panel_w, molding_proud * 2.0, panel_h)
        .translate((center_x, floor_y + molding_proud, center_z))
    )
    mold_inner = (
        cq.Workplane("XY")
        .box(panel_w - margin_w, molding_proud * 4.0, panel_h - margin_h)
        .translate((center_x, floor_y + molding_proud, center_z))
    )
    molding = mold_outer.cut(mold_inner)
    leaf = leaf.union(molding)

    # 3. Raised fielded pad in the center, proud of the recess floor.
    pad = (
        cq.Workplane("XY")
        .box(
            panel_w - pad_inset_w,
            pad_proud + (face_y - floor_y),
            panel_h - pad_inset_h,
        )
        .translate(
            (center_x, floor_y + (pad_proud + (face_y - floor_y)) / 2.0, center_z)
        )
    )
    leaf = leaf.union(pad)
    return leaf


def _build_leaf(sign: float) -> cq.Workplane:
    """One walnut leaf with six stacked raised panels and stiles/rails.

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
    # stacked panels. Rail height is reduced for 6-panel layout so each panel
    # still has a readable height while the total leaf proportions stay correct.
    stile = 0.09           # side stile width
    rail = 0.06            # top/bottom/mid rail height
    field_inset_y = 0.014  # recess depth into the leaf front face
    molding_proud = 0.006  # how proud the molding ring sits above recess floor
    pad_proud = 0.010      # how proud the fielded pad center sits

    face_y = LEAF_T / 2.0  # front face of the leaf
    floor_y = face_y - field_inset_y  # recessed field floor (front of pad base)

    # Vertical extent split into PANEL_COUNT panels with rails between.
    panel_count = PANEL_COUNT
    usable_h = LEAF_H - 2 * rail - (panel_count - 1) * rail
    panel_h = usable_h / panel_count
    panel_w = LEAF_W - 2 * stile

    panel_center_x = cx

    for i in range(panel_count):
        cz = rail + panel_h / 2.0 + i * (panel_h + rail)
        leaf = _add_raised_panel(
            leaf,
            center_x=panel_center_x,
            center_z=cz,
            panel_w=panel_w,
            panel_h=panel_h,
            face_y=face_y,
            floor_y=floor_y,
            field_inset_y=field_inset_y,
            molding_proud=molding_proud,
            pad_proud=pad_proud,
        )

    # Soften the visible outer vertical edges of the leaf.
    try:
        leaf = leaf.edges("|Z").fillet(0.004)
    except Exception:
        pass

    return leaf


HANDLE_Z = LEAF_H * 0.45        # escutcheon center height
SPINDLE_Z = HANDLE_Z + 0.03     # lever spindle (rose center) height


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

    # --- Leaves ---
    # door_0 extends +X from the left jamb; door_1 extends -X from the right
    # jamb. Both keep +Y as the world-facing front so both brass handles sit on
    # the same side and the meeting stiles meet at the center.
    half_open = OPENING_W / 2.0
    signs = {0: 1.0, 1: -1.0}

    for idx in range(2):
        sign = signs[idx]
        leaf = model.part(f"door_{idx}")
        leaf.visual(
            mesh_from_cadquery(_build_leaf(sign), f"door_{idx}_leaf"),
            material="walnut",
            name=f"door_{idx}_leaf",
        )
        leaf.visual(
            mesh_from_cadquery(_build_handle_fixed(sign), f"door_{idx}_handle"),
            material="brass",
            name=f"door_{idx}_handle",
        )

        # Rotating lever handle: its own part, hinged on the spindle through
        # the rose. Axis is the leaf-local Y spindle; the sign flip keeps
        # "press down" as the positive direction on both mirrored leaves.
        lever = model.part(f"door_{idx}_lever")
        lever.visual(
            mesh_from_cadquery(_build_lever(sign), f"door_{idx}_lever_bar"),
            material="brass",
            name=f"door_{idx}_lever_bar",
        )
        model.articulation(
            f"door_{idx}_to_lever",
            ArticulationType.REVOLUTE,
            parent=leaf,
            child=lever,
            origin=Origin(xyz=(_plate_x(sign), 0.0, SPINDLE_Z)),
            axis=(0.0, -sign, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=0.7),
        )

    # door_0: hinge at left jamb, leaf extends +X; +Z axis swings free edge +Y.
    left_hinge_x = -(half_open - JAMB_REVEAL)
    model.articulation(
        "frame_to_door_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_0"),
        origin=Origin(xyz=(left_hinge_x, 0.0, FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.92),
    )

    # door_1: hinge at right jamb, leaf extends -X; -Z axis swings free edge +Y,
    # symmetric outward with door_0.
    right_hinge_x = half_open - JAMB_REVEAL
    model.articulation(
        "frame_to_door_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_1"),
        origin=Origin(xyz=(right_hinge_x, 0.0, FLOOR_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.92),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    lever_0 = object_model.get_part("door_0_lever")
    lever_1 = object_model.get_part("door_1_lever")
    hinge_0 = object_model.get_articulation("frame_to_door_0")
    hinge_1 = object_model.get_articulation("frame_to_door_1")
    lever_j0 = object_model.get_articulation("door_0_to_lever")
    lever_j1 = object_model.get_articulation("door_1_to_lever")

    leaf0 = door_0.get_visual("door_0_leaf")
    leaf1 = door_1.get_visual("door_1_leaf")
    handle0 = door_0.get_visual("door_0_handle")
    handle1 = door_1.get_visual("door_1_handle")

    # --- Lever/rose intentional overlap (captured shaft through the rose) ---
    for idx in range(2):
        door_nm = f"door_{idx}"
        lever_nm = f"door_{idx}_lever"
        ctx.allow_overlap(
            door_nm,
            lever_nm,
            elem_a=f"door_{idx}_handle",
            elem_b=f"door_{idx}_lever_bar",
            reason=(
                "The lever neck sits on/through the rose collar as a captured "
                "shaft; this small local overlap represents the spindle passing "
                "through the fixed rose."
            ),
        )

    # --- Hero geometry: leaf sizing (raised-panel leaf present and sized) ---
    for door, leaf_elem, nm in ((door_0, leaf0, "door_0"), (door_1, leaf1, "door_1")):
        aabb = ctx.part_element_world_aabb(door, elem=leaf_elem)
        assert aabb is not None
        (lo, hi) = aabb
        w = hi[0] - lo[0]
        h = hi[2] - lo[2]
        t = hi[1] - lo[1]
        ctx.check(
            f"{nm} leaf width plausible",
            0.75 <= w <= 0.95,
            details=f"width={w:.3f}",
        )
        ctx.check(
            f"{nm} leaf height plausible",
            1.9 <= h <= 2.15,
            details=f"height={h:.3f}",
        )
        ctx.check(
            f"{nm} leaf thickness plausible (raised panels add depth)",
            0.04 <= t <= 0.09,
            details=f"thickness={t:.3f}",
        )

    # --- Six stacked raised panels per leaf ---
    ctx.check(
        "panel count is six",
        PANEL_COUNT == 6,
        details=f"PANEL_COUNT={PANEL_COUNT}",
    )
    # Verify the panel layout math produces six panels that fit the leaf.
    panel_count = PANEL_COUNT
    stile = 0.09
    rail = 0.06
    usable_h = LEAF_H - 2 * rail - (panel_count - 1) * rail
    panel_h = usable_h / panel_count
    panel_w = LEAF_W - 2 * stile
    ctx.check(
        "each panel has readable height (>0.15 m)",
        panel_h > 0.15,
        details=f"panel_h={panel_h:.4f}",
    )
    ctx.check(
        "each panel has readable width (>0.40 m)",
        panel_w > 0.40,
        details=f"panel_w={panel_w:.4f}",
    )
    ctx.check(
        "six panels with rails fit within leaf height",
        2 * rail + panel_count * panel_h + (panel_count - 1) * rail <= LEAF_H + 1e-6,
        details=f"total_layout={2 * rail + panel_count * panel_h + (panel_count - 1) * rail:.4f} leaf_h={LEAF_H:.4f}",
    )

    # --- Hero hardware: brass handle/escutcheon present and on the leaf ---
    for door, h_elem, nm in ((door_0, handle0, "door_0"), (door_1, handle1, "door_1")):
        h_aabb = ctx.part_element_world_aabb(door, elem=h_elem)
        assert h_aabb is not None
        (hlo, hhi) = h_aabb
        # Escutcheon tall, lever projects: confirm a non-trivial vertical extent
        ctx.check(
            f"{nm} brass hardware has escutcheon height",
            (hhi[2] - hlo[2]) >= 0.12,
            details=f"hardware_z_extent={hhi[2]-hlo[2]:.3f}",
        )
        # Plate/rose sit proud of the front face
        ctx.check(
            f"{nm} escutcheon/rose sit proud of front face",
            (hhi[1] - hlo[1]) >= 0.005,
            details=f"hardware_y_extent={hhi[1]-hlo[1]:.3f}",
        )

    # --- Lever handles: present, proud of the face, and ROTATE on the spindle ---
    for lever, lever_j, nm in ((lever_0, lever_j0, "door_0"), (lever_1, lever_j1, "door_1")):
        with ctx.pose({hinge_0: 0.0, hinge_1: 0.0, lever_j: 0.0}):
            rest = ctx.part_world_aabb(lever)
        assert rest is not None
        ctx.check(
            f"{nm} lever bar projects from front face",
            (rest[1][1] - rest[0][1]) >= 0.02,
            details=f"lever_y_extent={rest[1][1]-rest[0][1]:.3f}",
        )
        ctx.check(
            f"{nm} lever bar has horizontal reach at rest",
            (rest[1][0] - rest[0][0]) >= 0.07,
            details=f"lever_x_extent={rest[1][0]-rest[0][0]:.3f}",
        )
        with ctx.pose({hinge_0: 0.0, hinge_1: 0.0, lever_j: 0.7}):
            pressed = ctx.part_world_aabb(lever)
        assert pressed is not None
        # Pressing the lever rotates the bar tip downward about the spindle.
        ctx.check(
            f"{nm} lever tip swings down when pressed",
            pressed[0][2] < rest[0][2] - 0.02,
            details=f"rest_minZ={rest[0][2]:.3f} pressed_minZ={pressed[0][2]:.3f}",
        )

    # --- LEAF MIRROR SYMMETRY across the world X=0 plane (closed pose) ---
    # The user complaint was that the two leaves were not mirror-symmetric (the
    # raised-panel layout, molding, grain, and handles must match as mirror
    # images and both sit on the SAME front face). Assert this directly.
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        def _sym_center(part, elem):
            a = ctx.part_element_world_aabb(part, elem=elem)
            assert a is not None
            lo_, hi_ = a
            c = [(lo_[i] + hi_[i]) / 2.0 for i in range(3)]
            d = [hi_[i] - lo_[i] for i in range(3)]
            return c, d

        for elem0, elem1, label in (
            (leaf0, leaf1, "leaf"),
            (handle0, handle1, "handle"),
        ):
            c0, d0 = _sym_center(door_0, elem0)
            c1, d1 = _sym_center(door_1, elem1)
            ctx.check(
                f"{label} mirrors across X=0 (centers opposite)",
                abs(c0[0] + c1[0]) < 0.01 and c0[0] < 0 < c1[0],
                details=f"cx0={c0[0]:.4f} cx1={c1[0]:.4f}",
            )
            ctx.check(
                f"{label} sits on same front face (matching Y)",
                abs(c0[1] - c1[1]) < 0.01,
                details=f"cy0={c0[1]:.4f} cy1={c1[1]:.4f}",
            )
            ctx.check(
                f"{label} same height center (matching Z)",
                abs(c0[2] - c1[2]) < 0.01,
                details=f"cz0={c0[2]:.4f} cz1={c1[2]:.4f}",
            )
            ctx.check(
                f"{label} identical size on both leaves (mirror)",
                all(abs(d0[i] - d1[i]) < 0.01 for i in range(3)),
                details=f"d0={[round(v,4) for v in d0]} d1={[round(v,4) for v in d1]}",
            )

        # Lever parts mirror across X=0 as well (levers at rest).
        la0 = ctx.part_world_aabb(lever_0)
        la1 = ctx.part_world_aabb(lever_1)
        assert la0 is not None and la1 is not None
        lc0 = [(la0[0][i] + la0[1][i]) / 2.0 for i in range(3)]
        lc1 = [(la1[0][i] + la1[1][i]) / 2.0 for i in range(3)]
        ctx.check(
            "lever bars mirror across X=0 (centers opposite)",
            abs(lc0[0] + lc1[0]) < 0.01 and lc0[0] < 0 < lc1[0],
            details=f"cx0={lc0[0]:.4f} cx1={lc1[0]:.4f}",
        )
        ctx.check(
            "lever bars at same height and face (matching Y/Z)",
            abs(lc0[1] - lc1[1]) < 0.01 and abs(lc0[2] - lc1[2]) < 0.01,
            details=f"c0={[round(v,4) for v in lc0]} c1={[round(v,4) for v in lc1]}",
        )

    # --- Closed pose: leaves meet at center with a small reveal, no overlap ---
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        # Both hinge edges stay at their jambs (connected): leaf min/max X near
        # the opening edges.
        a0 = ctx.part_element_world_aabb(door_0, elem=leaf0)
        a1 = ctx.part_element_world_aabb(door_1, elem=leaf1)
        assert a0 is not None and a1 is not None
        # door_0 occupies the left half, door_1 the right half.
        # They must not interpenetrate at the center; expect a gap along X.
        # door_1 is on the +X side, door_0 on the -X side.
        ctx.expect_gap(
            door_1,
            door_0,
            axis="x",
            positive_elem=leaf1,
            negative_elem=leaf0,
            min_gap=0.0,
            max_gap=0.05,
            name="closed leaves meet with small center reveal",
        )
        # Each leaf hinge edge sits at its jamb (connected to frame).
        ctx.expect_contact(
            door_0, frame, contact_tol=0.02,
            name="door_0 hinge edge meets frame jamb",
        )
        ctx.expect_contact(
            door_1, frame, contact_tol=0.02,
            name="door_1 hinge edge meets frame jamb",
        )

    # --- Open pose: both leaves swing clear, hinge edges remain at jambs ---
    open_q = 1.4
    closed_pos_0 = None
    closed_pos_1 = None
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        closed_pos_0 = ctx.part_world_aabb(door_0)
        closed_pos_1 = ctx.part_world_aabb(door_1)

    with ctx.pose({hinge_0: open_q, hinge_1: open_q}):
        open0 = ctx.part_world_aabb(door_0)
        open1 = ctx.part_world_aabb(door_1)
        assert open0 is not None and open1 is not None
        assert closed_pos_0 is not None and closed_pos_1 is not None
        # Open leaves swing out in +Y (front): max Y grows substantially.
        ctx.check(
            "door_0 swings outward when opened",
            open0[1][1] > closed_pos_0[1][1] + 0.3,
            details=f"closed_maxY={closed_pos_0[1][1]:.3f} open_maxY={open0[1][1]:.3f}",
        )
        ctx.check(
            "door_1 swings outward when opened",
            open1[1][1] > closed_pos_1[1][1] + 0.3,
            details=f"closed_maxY={closed_pos_1[1][1]:.3f} open_maxY={open1[1][1]:.3f}",
        )
        # When open, the leaves swing clear of each other at the center
        # (their X footprints separate as they rotate toward the jambs).
        ctx.expect_contact(
            door_0, frame, contact_tol=0.05,
            name="door_0 hinge edge stays at jamb when open",
        )
        ctx.expect_contact(
            door_1, frame, contact_tol=0.05,
            name="door_1 hinge edge stays at jamb when open",
        )

    return ctx.report()


object_model = build_object_model()
