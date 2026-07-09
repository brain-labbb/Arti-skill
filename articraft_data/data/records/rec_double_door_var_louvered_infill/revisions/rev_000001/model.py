from __future__ import annotations

# Classic double entry door in dark walnut wood — louvered shutter variant.
#
# Articraft brief:
# - Object: classic double entry door, opening ~1.70 m wide x 2.10 m tall,
#   each leaf ~0.83 m wide x 0.045 m thick x 2.05 m tall, in a walnut frame.
# - Root/support: fixed wood frame (two side jambs, head jamb, carved base
#   threshold, and a paneled back surround) carries both swinging leaves.
# - Parts: frame (root), door_0, door_1. Each leaf has a frame (stiles + rails)
#   with a full-height louver opening filled by N horizontal angled slats,
#   plus a brass handle and escutcheon at the inner meeting edge.
# - Articulations: frame_to_door_0 and frame_to_door_1, both REVOLUTE, vertical
#   (+Z) hinge axes on the OUTER edges at the jambs, opening symmetrically
#   outward. Names door_0/door_1 (symmetric, no left/right per link naming).
#   Each leaf additionally carries a REVOLUTE lever handle (door_N_to_lever)
#   rotating about the horizontal spindle axis through the rose, so pressing
#   the brass lever bar swings its tip downward.
# - Visible geometry: dark walnut leaves with louvered shutter infill (horizontal
#   angled slats in a for loop), stiles/rails framing the opening, brass hardware,
#   paneled frame with a carved base rail.
# - Support/fit: each leaf hinge edge meets its jamb; leaves meet at the center
#   with a small reveal so they do not interpenetrate when closed.
# - Intentional overlaps: slat ends embed slightly into stiles for connectivity.
# - Tests: slats present and angled, frame sized, handle/escutcheon present,
#   leaves meet with a reveal when closed, both leaves swing clear while hinge
#   edges stay at the jambs when open.

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


def _build_leaf_frame(sign: float) -> cq.Workplane:
    """One walnut leaf frame with a full-height louver opening cut through.

    Local frame:
      - hinge edge at local X=0
      - leaf body extends toward `sign` (+1 => +X, -1 => -X)
      - thickness along Y, centered; front face = +Y for BOTH leaves
      - height along Z in [0, LEAF_H]

    The frame consists of two vertical stiles and two horizontal rails
    surrounding a rectangular opening where louver slats are mounted.
    """
    stile = 0.11           # side stile width
    rail = 0.13            # top/bottom rail height

    cx = sign * LEAF_W / 2.0
    leaf = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((cx, 0.0, 0.0))
    )

    # Cut the full-height louver opening through the leaf thickness,
    # leaving stiles on the sides and rails at top/bottom.
    louver_w = LEAF_W - 2 * stile
    louver_h = LEAF_H - 2 * rail
    louver_center_z = rail + louver_h / 2.0
    opening = (
        cq.Workplane("XY")
        .box(louver_w, LEAF_T * 1.2, louver_h)
        .translate((cx, 0.0, louver_center_z))
    )
    leaf = leaf.cut(opening)

    # Soften the visible outer vertical edges of the leaf frame.
    try:
        leaf = leaf.edges("|Z").fillet(0.004)
    except Exception:
        pass

    return leaf


def _build_louver_slat(center_x: float, center_z: float, width: float,
                       thickness: float, face_h: float, angle_deg: float) -> cq.Workplane:
    """One angled horizontal louver slat (shared geometry helper).

    The slat is a thin board tilted about the X axis so its front edge sits
    higher than its rear edge, like a real plantation shutter louver.
    """
    return (
        cq.Workplane("XY")
        .box(width, thickness, face_h)
        .rotate((0, 0, 0), (1, 0, 0), angle_deg)
        .translate((center_x, 0.0, center_z))
    )


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

    # Louver slat parameters (shared across both leaves)
    stile_w = 0.11
    rail_h = 0.13
    slat_count = 18
    slat_angle = 35.0        # degrees tilt from horizontal
    slat_thick = 0.009       # board thickness
    slat_face_h = 0.040      # face height before tilting
    louver_w = LEAF_W - 2 * stile_w + 0.006  # slight embed into stiles for connectivity
    louver_bottom_z = rail_h
    louver_h = LEAF_H - 2 * rail_h
    slat_spacing = louver_h / slat_count

    for idx in range(2):
        sign = signs[idx]
        leaf = model.part(f"door_{idx}")
        cx = sign * LEAF_W / 2.0

        # Leaf frame (stiles + rails with louver opening)
        leaf.visual(
            mesh_from_cadquery(_build_leaf_frame(sign), f"door_{idx}_frame"),
            material="walnut",
            name=f"door_{idx}_frame",
        )

        # Louver slats: for-loop over slat_count with shared helper
        for i in range(slat_count):
            slat_z = louver_bottom_z + slat_spacing * (i + 0.5)
            slat_shape = _build_louver_slat(
                cx, slat_z, louver_w, slat_thick, slat_face_h, slat_angle
            )
            leaf.visual(
                mesh_from_cadquery(slat_shape, f"door_{idx}_slat_{i}"),
                material="walnut",
                name=f"door_{idx}_slat_{i}",
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

    frame0 = door_0.get_visual("door_0_frame")
    frame1 = door_1.get_visual("door_1_frame")
    handle0 = door_0.get_visual("door_0_handle")
    handle1 = door_1.get_visual("door_1_handle")
    slat_0_0 = door_0.get_visual("door_0_slat_0")
    slat_1_0 = door_1.get_visual("door_1_slat_0")

    # --- Intentional overlaps: lever neck embeds through rose collar ---
    for d, lev, h_elem, lev_elem in (
        (door_0, lever_0, "door_0_handle", "door_0_lever_bar"),
        (door_1, lever_1, "door_1_handle", "door_1_lever_bar"),
    ):
        ctx.allow_overlap(
            d, lev,
            elem_a=h_elem, elem_b=lev_elem,
            reason="Lever neck passes through the rose collar as a real spindle connection.",
        )
        # Proof: the lever still articulates correctly (rotation about spindle axis).
        with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
            ctx.expect_contact(
                d, lev, contact_tol=0.02,
                elem_a=h_elem, elem_b=lev_elem,
                name=f"{h_elem} and {lev_elem} remain in contact at spindle",
            )

    # --- Hero geometry: leaf frame sizing (louvered leaf present and sized) ---
    for door, frame_elem, nm in ((door_0, frame0, "door_0"), (door_1, frame1, "door_1")):
        aabb = ctx.part_element_world_aabb(door, elem=frame_elem)
        assert aabb is not None
        (lo, hi) = aabb
        w = hi[0] - lo[0]
        h = hi[2] - lo[2]
        t = hi[1] - lo[1]
        ctx.check(
            f"{nm} leaf frame width plausible",
            0.75 <= w <= 0.95,
            details=f"width={w:.3f}",
        )
        ctx.check(
            f"{nm} leaf frame height plausible",
            1.9 <= h <= 2.15,
            details=f"height={h:.3f}",
        )
        ctx.check(
            f"{nm} leaf frame thickness plausible",
            0.03 <= t <= 0.08,
            details=f"thickness={t:.3f}",
        )

    # --- Louver slats: present, angled, and spanning the opening ---
    slat_count = 18
    for door_idx, door_part, nm in ((0, door_0, "door_0"), (1, door_1, "door_1")):
        # Verify first and last slat exist
        first_slat = door_part.get_visual(f"door_{door_idx}_slat_0")
        last_slat = door_part.get_visual(f"door_{door_idx}_slat_{slat_count - 1}")
        # Slat is tilted: Y extent should be greater than just the thickness
        # (tilted at 35°, so Y extent ≈ thick*cos + face_h*sin ≈ 0.030m)
        s_aabb = ctx.part_element_world_aabb(door_part, elem=first_slat)
        assert s_aabb is not None
        slo, shi = s_aabb
        y_extent = shi[1] - slo[1]
        ctx.check(
            f"{nm} louver slat is angled (Y extent > board thickness)",
            y_extent > 0.015,
            details=f"slat_y_extent={y_extent:.4f}",
        )
        # Slat spans the opening width (between stiles)
        x_extent = shi[0] - slo[0]
        ctx.check(
            f"{nm} louver slat spans most of the leaf width",
            x_extent > 0.50,
            details=f"slat_x_extent={x_extent:.3f}",
        )
        # Last slat is near the top rail
        last_aabb = ctx.part_element_world_aabb(door_part, elem=last_slat)
        assert last_aabb is not None
        llo, lhi = last_aabb
        ctx.check(
            f"{nm} last louver slat is near top of leaf",
            lhi[2] > 1.80,
            details=f"last_slat_max_z={lhi[2]:.3f}",
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
    # The two leaves must be mirror-symmetric: louver frame, slats, and handles
    # match as mirror images and both sit on the SAME front face.
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        def _sym_center(part, elem):
            a = ctx.part_element_world_aabb(part, elem=elem)
            assert a is not None
            lo_, hi_ = a
            c = [(lo_[i] + hi_[i]) / 2.0 for i in range(3)]
            d = [hi_[i] - lo_[i] for i in range(3)]
            return c, d

        for elem0, elem1, label in (
            (frame0, frame1, "leaf_frame"),
            (handle0, handle1, "handle"),
            (slat_0_0, slat_1_0, "louver_slat_0"),
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
        a0 = ctx.part_element_world_aabb(door_0, elem=frame0)
        a1 = ctx.part_element_world_aabb(door_1, elem=frame1)
        assert a0 is not None and a1 is not None
        # door_0 occupies the left half, door_1 the right half.
        # They must not interpenetrate at the center; expect a gap along X.
        # door_1 is on the +X side, door_0 on the -X side.
        ctx.expect_gap(
            door_1,
            door_0,
            axis="x",
            positive_elem=frame1,
            negative_elem=frame0,
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
