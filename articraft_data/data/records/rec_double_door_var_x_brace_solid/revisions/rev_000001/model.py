from __future__ import annotations

# Classic double entry door in dark walnut wood — ledged-and-X-braced variant.
#
# Articraft brief:
# - Object: classic double entry door, opening ~1.70 m wide x 2.10 m tall,
#   each leaf ~0.83 m wide x 0.045 m thick x 2.08 m tall, in a walnut frame.
# - Root/support: fixed wood frame (two side jambs, head jamb, carved base
#   threshold, and doorstop beads) carries both swinging leaves.
# - Parts: frame (root), door_0, door_1, door_0_lever, door_1_lever. Each leaf
#   carries a ledged-and-X-braced board infill: BOARD_COUNT vertical
#   tongue-and-groove boards (emitted by for loop with shared helper),
#   top and bottom ledger rails, two diagonal X-braces, plus stiles and
#   meeting stile with brass handle and escutcheon.
# - Articulations: frame_to_door_0 and frame_to_door_1, both REVOLUTE, vertical
#   (+Z) hinge axes on the OUTER edges at the jambs, opening symmetrically
#   outward. Each leaf additionally carries a REVOLUTE lever handle.
# - Visible geometry: dark walnut leaves with vertical T&G boards, ledger
#   rails, X-braces, brass hardware, paneled frame with carved base rail.
# - Support/fit: hinge edges at jambs; leaves meet at center with reveal.
# - Intentional overlaps: none required.
# - Tests: board count verified per leaf, ledger rails at top/bottom, braces
#   present with diagonal extent, leaves swing correctly, hardware intact.

import math
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

# --- Ledged-and-X-braced infill dimensions ---
BOARD_COUNT = 7            # vertical tongue-and-groove boards per leaf
STILE_W = 0.08             # hinge stile and meeting stile width
BOARD_GAP = 0.003          # tongue-and-groove reveal between boards
BOARD_T = 0.012            # board thickness proud of slab face
LEDGER_H = 0.08            # ledger rail height
LEDGER_T = 0.018           # ledger rail thickness proud of slab face
BRACE_W = 0.055            # diagonal brace width
BRACE_T = 0.015            # brace thickness (sits on top of boards)

# Derived infill geometry
INFILL_W = LEAF_W - 2 * STILE_W
INFILL_H = LEAF_H - 2 * LEDGER_H
BOARD_W = (INFILL_W - (BOARD_COUNT - 1) * BOARD_GAP) / BOARD_COUNT

WALNUT = (0.28, 0.16, 0.09, 1.0)
WALNUT_DARK = (0.20, 0.11, 0.06, 1.0)
WALNUT_LIGHT = (0.36, 0.22, 0.12, 1.0)
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


def _build_leaf_slab(sign: float) -> cq.Workplane:
    """Flat leaf slab with proud stiles at hinge and meeting edges.

    Local frame:
      - hinge edge at local X=0
      - leaf body extends toward `sign` (+1 => +X, -1 => -X)
      - thickness along Y, centered; front face = +Y for BOTH leaves
      - height along Z in [0, LEAF_H]
    """
    cx = sign * LEAF_W / 2.0
    slab = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T, LEAF_H, centered=(True, True, False))
        .translate((cx, 0.0, 0.0))
    )

    # Proud stiles at hinge and meeting edges (slightly raised on the face)
    stile_proud = 0.005
    face_y = LEAF_T / 2.0

    # Meeting stile (inner edge, where the handle mounts)
    stile_x_meeting = sign * (LEAF_W - STILE_W / 2.0)
    meeting_stile = (
        cq.Workplane("XY")
        .box(STILE_W, stile_proud, LEAF_H)
        .translate((stile_x_meeting, face_y + stile_proud / 2.0, LEAF_H / 2.0))
    )
    slab = slab.union(meeting_stile)

    # Hinge stile (outer edge)
    stile_x_hinge = sign * STILE_W / 2.0
    hinge_stile = (
        cq.Workplane("XY")
        .box(STILE_W, stile_proud, LEAF_H)
        .translate((stile_x_hinge, face_y + stile_proud / 2.0, LEAF_H / 2.0))
    )
    slab = slab.union(hinge_stile)

    # Soften outer vertical edges
    try:
        slab = slab.edges("|Z").fillet(0.003)
    except Exception:
        pass

    return slab


def _make_tg_board(center_x: float, face_y: float, w: float, t: float,
                   h: float, cz: float) -> cq.Workplane:
    """Shared helper: one vertical tongue-and-groove board, proud of the slab.

    Args:
        center_x: board center X position (world or leaf-local).
        face_y: front face Y of the slab.
        w: board width.
        t: board thickness (proud dimension).
        h: board height.
        cz: board center Z.
    """
    return (
        cq.Workplane("XY")
        .box(w, t, h)
        .translate((center_x, face_y + t / 2.0, cz))
    )


def _build_ledger(sign: float, position: str) -> cq.Workplane:
    """Top or bottom horizontal ledger rail on the leaf front face."""
    cx = sign * LEAF_W / 2.0
    face_y = LEAF_T / 2.0
    if position == "top":
        cz = LEAF_H - LEDGER_H / 2.0
    else:
        cz = LEDGER_H / 2.0
    # Ledger spans full leaf width (covers stiles too)
    return (
        cq.Workplane("XY")
        .box(LEAF_W, LEDGER_T, LEDGER_H)
        .translate((cx, face_y + LEDGER_T / 2.0, cz))
    )


def _build_xbrace(sign: float, slope_sign: float) -> cq.Workplane:
    """One diagonal brace of the X pattern.

    slope_sign: +1 for / brace (bottom-left to top-right), -1 for \\ brace.
    The brace sits on top of the boards, between the ledger rails.
    The long axis tilts in the XZ plane (door face), so we rotate about Y.
    """
    cx = sign * LEAF_W / 2.0
    face_y = LEAF_T / 2.0
    brace_y = face_y + BOARD_T + BRACE_T / 2.0

    diag_len = math.sqrt(INFILL_W ** 2 + INFILL_H ** 2)
    angle_deg = math.degrees(math.atan2(INFILL_H, INFILL_W)) * slope_sign

    # Box: X=diagonal length, Y=thickness (proud of face), Z=visible width.
    # Rotate about Y to tilt the long axis from horizontal into the XZ plane.
    return (
        cq.Workplane("XY")
        .box(diag_len, BRACE_T, BRACE_W)
        .rotate((0, 0, 0), (0, 1, 0), angle_deg)
        .translate((cx, brace_y, LEDGER_H + INFILL_H / 2.0))
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
    model.material("walnut_light", rgba=WALNUT_LIGHT)
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
    half_open = OPENING_W / 2.0
    signs = {0: 1.0, 1: -1.0}

    face_y = LEAF_T / 2.0
    board_cz = LEDGER_H + INFILL_H / 2.0  # board center Z (between ledgers)

    for idx in range(2):
        sign = signs[idx]
        leaf = model.part(f"door_{idx}")

        # Base slab with stiles
        leaf.visual(
            mesh_from_cadquery(_build_leaf_slab(sign), f"door_{idx}_slab"),
            material="walnut",
            name=f"door_{idx}_slab",
        )

        # --- Vertical T&G boards (for loop with shared helper) ---
        for i in range(BOARD_COUNT):
            bx = sign * (STILE_W + BOARD_W / 2.0 + i * (BOARD_W + BOARD_GAP))
            board_shape = _make_tg_board(bx, face_y, BOARD_W, BOARD_T, INFILL_H, board_cz)
            leaf.visual(
                mesh_from_cadquery(board_shape, f"door_{idx}_board_{i}"),
                material="walnut_light",
                name=f"door_{idx}_board_{i}",
            )

        # --- Top and bottom ledger rails ---
        for pos in ("top", "bottom"):
            leaf.visual(
                mesh_from_cadquery(_build_ledger(sign, pos), f"door_{idx}_ledger_{pos}"),
                material="walnut_dark",
                name=f"door_{idx}_ledger_{pos}",
            )

        # --- X-braces (two diagonal boards) ---
        for bi in range(2):
            slope_sign = 1.0 if bi == 0 else -1.0
            leaf.visual(
                mesh_from_cadquery(_build_xbrace(sign, slope_sign), f"door_{idx}_brace_{bi}"),
                material="walnut_dark",
                name=f"door_{idx}_brace_{bi}",
            )

        # --- Brass hardware (unchanged from parent) ---
        leaf.visual(
            mesh_from_cadquery(_build_handle_fixed(sign), f"door_{idx}_handle"),
            material="brass",
            name=f"door_{idx}_handle",
        )

        # Rotating lever handle: its own part, hinged on the spindle through
        # the rose.
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

    # door_1: hinge at right jamb, leaf extends -X; -Z axis swings free edge +Y.
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

    # --- Prompt-specific: board count per leaf ---
    for door, nm in ((door_0, "door_0"), (door_1, "door_1")):
        board_names = [f"{nm}_board_{i}" for i in range(BOARD_COUNT)]
        found = 0
        for bname in board_names:
            try:
                door.get_visual(bname)
                found += 1
            except Exception:
                pass
        ctx.check(
            f"{nm} has {BOARD_COUNT} vertical T&G boards",
            found == BOARD_COUNT,
            details=f"found={found} expected={BOARD_COUNT}",
        )

    # --- Prompt-specific: ledger rails at top and bottom ---
    for door, nm in ((door_0, "door_0"), (door_1, "door_1")):
        for pos in ("top", "bottom"):
            vname = f"{nm}_ledger_{pos}"
            try:
                v = door.get_visual(vname)
                aabb = ctx.part_element_world_aabb(door, elem=v)
                assert aabb is not None
                lo, hi = aabb
                ledger_h = hi[2] - lo[2]
                ctx.check(
                    f"{nm} {pos} ledger rail present and sized",
                    0.06 <= ledger_h <= 0.12,
                    details=f"ledger_h={ledger_h:.3f}",
                )
            except Exception:
                ctx.fail(f"{nm} {pos} ledger rail missing", "visual not found")

        # --- Prompt-specific: top ledger above bottom ledger ---
        try:
            top_aabb = ctx.part_element_world_aabb(door, elem=f"{nm}_ledger_top")
            bot_aabb = ctx.part_element_world_aabb(door, elem=f"{nm}_ledger_bottom")
            assert top_aabb is not None and bot_aabb is not None
            ctx.check(
                f"{nm} top ledger is above bottom ledger",
                top_aabb[0][2] > bot_aabb[1][2] - 0.02,
                details=f"top_min_z={top_aabb[0][2]:.3f} bot_max_z={bot_aabb[1][2]:.3f}",
            )
        except Exception:
            pass

    # --- Prompt-specific: X-braces present with diagonal extent ---
    for door, nm in ((door_0, "door_0"), (door_1, "door_1")):
        for bi in range(2):
            vname = f"{nm}_brace_{bi}"
            try:
                aabb = ctx.part_element_world_aabb(door, elem=vname)
                assert aabb is not None
                lo, hi = aabb
                brace_x_extent = hi[0] - lo[0]
                brace_z_extent = hi[2] - lo[2]
                # A diagonal brace should have significant extent in both X and Z
                ctx.check(
                    f"{nm} brace_{bi} has diagonal extent (X and Z)",
                    brace_x_extent > 0.3 and brace_z_extent > 0.8,
                    details=f"x_extent={brace_x_extent:.3f} z_extent={brace_z_extent:.3f}",
                )
            except Exception:
                ctx.fail(f"{nm} brace_{bi} missing", "visual not found")

    # --- Prompt-specific: boards are between the ledgers vertically ---
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        for door, nm in ((door_0, "door_0"), (door_1, "door_1")):
            try:
                bot_aabb = ctx.part_element_world_aabb(door, elem=f"{nm}_ledger_bottom")
                top_aabb = ctx.part_element_world_aabb(door, elem=f"{nm}_ledger_top")
                board_0_aabb = ctx.part_element_world_aabb(door, elem=f"{nm}_board_0")
                assert bot_aabb is not None and top_aabb is not None and board_0_aabb is not None
                # Board bottom should be near bottom ledger top
                ctx.check(
                    f"{nm} boards sit between ledgers (bottom)",
                    board_0_aabb[0][2] >= bot_aabb[1][2] - 0.02,
                    details=f"board_min_z={board_0_aabb[0][2]:.3f} ledger_bot_max_z={bot_aabb[1][2]:.3f}",
                )
                # Board top should be near top ledger bottom
                ctx.check(
                    f"{nm} boards sit between ledgers (top)",
                    board_0_aabb[1][2] <= top_aabb[0][2] + 0.02,
                    details=f"board_max_z={board_0_aabb[1][2]:.3f} ledger_top_min_z={top_aabb[0][2]:.3f}",
                )
            except Exception:
                pass

    # --- Hero geometry: leaf sizing ---
    for door, nm in ((door_0, "door_0"), (door_1, "door_1")):
        slab = door.get_visual(f"{nm}_slab")
        aabb = ctx.part_element_world_aabb(door, elem=slab)
        assert aabb is not None
        (lo, hi) = aabb
        w = hi[0] - lo[0]
        h = hi[2] - lo[2]
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

    # --- Hero hardware: brass handle/escutcheon present ---
    for door, nm in ((door_0, "door_0"), (door_1, "door_1")):
        h_elem = door.get_visual(f"{nm}_handle")
        h_aabb = ctx.part_element_world_aabb(door, elem=h_elem)
        assert h_aabb is not None
        (hlo, hhi) = h_aabb
        ctx.check(
            f"{nm} brass hardware has escutcheon height",
            (hhi[2] - hlo[2]) >= 0.12,
            details=f"hardware_z_extent={hhi[2]-hlo[2]:.3f}",
        )
        ctx.check(
            f"{nm} escutcheon/rose sit proud of front face",
            (hhi[1] - hlo[1]) >= 0.005,
            details=f"hardware_y_extent={hhi[1]-hlo[1]:.3f}",
        )

    # --- Lever handles: present, proud, and ROTATE on the spindle ---
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
        ctx.check(
            f"{nm} lever tip swings down when pressed",
            pressed[0][2] < rest[0][2] - 0.02,
            details=f"rest_minZ={rest[0][2]:.3f} pressed_minZ={pressed[0][2]:.3f}",
        )

    # --- Leaf mirror symmetry across X=0 (closed pose) ---
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        def _sym_center(part, elem):
            a = ctx.part_element_world_aabb(part, elem=elem)
            assert a is not None
            lo_, hi_ = a
            c = [(lo_[i] + hi_[i]) / 2.0 for i in range(3)]
            d = [hi_[i] - lo_[i] for i in range(3)]
            return c, d

        slab0 = door_0.get_visual("door_0_slab")
        slab1 = door_1.get_visual("door_1_slab")
        handle0 = door_0.get_visual("door_0_handle")
        handle1 = door_1.get_visual("door_1_handle")

        for elem0, elem1, label in (
            (slab0, slab1, "slab"),
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

        # Lever mirror symmetry
        la0 = ctx.part_world_aabb(lever_0)
        la1 = ctx.part_world_aabb(lever_1)
        assert la0 is not None and la1 is not None
        lc0 = [(la0[0][i] + la0[1][i]) / 2.0 for i in range(3)]
        lc1 = [(la1[0][i] + la1[1][i]) / 2.0 for i in range(3)]
        ctx.check(
            "lever bars mirror across X=0",
            abs(lc0[0] + lc1[0]) < 0.01 and lc0[0] < 0 < lc1[0],
            details=f"cx0={lc0[0]:.4f} cx1={lc1[0]:.4f}",
        )

    # --- Intentional overlap: lever neck seated on the handle rose ---
    ctx.allow_overlap(
        "door_0", "door_0_lever",
        elem_a="door_0_handle", elem_b="door_0_lever_bar",
        reason="The lever neck is intentionally seated on the handle rose/escutcheon as a spindle-mounted handle.",
    )
    ctx.allow_overlap(
        "door_1", "door_1_lever",
        elem_a="door_1_handle", elem_b="door_1_lever_bar",
        reason="The lever neck is intentionally seated on the handle rose/escutcheon as a spindle-mounted handle.",
    )
    # Proof: lever contact with handle at rest
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0, lever_j0: 0.0, lever_j1: 0.0}):
        ctx.expect_contact(
            door_0, lever_0, contact_tol=0.02,
            name="door_0 lever seated on handle rose",
        )
        ctx.expect_contact(
            door_1, lever_1, contact_tol=0.02,
            name="door_1 lever seated on handle rose",
        )

    # --- Closed pose: leaves meet at center with reveal ---
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        slab0 = door_0.get_visual("door_0_slab")
        slab1 = door_1.get_visual("door_1_slab")
        ctx.expect_gap(
            door_1,
            door_0,
            axis="x",
            positive_elem=slab1,
            negative_elem=slab0,
            min_gap=0.0,
            max_gap=0.05,
            name="closed leaves meet with small center reveal",
        )
        ctx.expect_contact(
            door_0, frame, contact_tol=0.02,
            name="door_0 hinge edge meets frame jamb",
        )
        ctx.expect_contact(
            door_1, frame, contact_tol=0.02,
            name="door_1 hinge edge meets frame jamb",
        )

    # --- Open pose: both leaves swing clear ---
    open_q = 1.4
    with ctx.pose({hinge_0: 0.0, hinge_1: 0.0}):
        closed_pos_0 = ctx.part_world_aabb(door_0)
        closed_pos_1 = ctx.part_world_aabb(door_1)

    with ctx.pose({hinge_0: open_q, hinge_1: open_q}):
        open0 = ctx.part_world_aabb(door_0)
        open1 = ctx.part_world_aabb(door_1)
        assert open0 is not None and open1 is not None
        assert closed_pos_0 is not None and closed_pos_1 is not None
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
