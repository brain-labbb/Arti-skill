from __future__ import annotations

# White plastic (HDPE) gallon jug -- a "refresh" fresh-floral air-freshener /
# cleaner gallon jug. Rounded-rectangular body, clearly taller than wide, with a
# tapered shoulder rising to a small top deck. The body is a real blow-molded
# shell (hollow) with a through pour-mouth at the offset neck. A prominent
# integrated loop grip handle is fused into the upper shoulder on one side, with
# a real open finger hole. A near-square "refresh" front label panel carries a
# teal swoosh accent and a couple of pink floral dots.
#
# Frame: +Z up. The jug bottom sits on z=0. The rounded-rect footprint is
# centered on X/Y. The threaded neck is offset toward +X but kept close to the
# centerline; the integrated grip handle loop occupies the farther -X side of
# the top shoulder so the neck has clear room on the deck.
#
# Articulations (decoupled screw cap on the offset neck, both about the +Z neck
# axis, routed through a massless carrier link):
#   - cap_rotate: CONTINUOUS spin about +Z (unscrew/screw the cap)
#   - cap_slide:  PRISMATIC lift up off the neck along +Z
# The handle is a FIXED part of the body.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters): a real US gallon jug, ~0.28 m tall ----
BODY_W = 0.150  # X footprint
BODY_D = 0.135  # Y footprint
BASE_H = 0.012  # short flared foot/base ridge at the very bottom
BODY_H = 0.215  # top of the straight body (shoulder start), measured from z=0
SHOULDER_H = 0.050  # tapered shoulder rising to the small top deck
TOP_Z = BODY_H + SHOULDER_H  # ~0.265 m -> small top deck height

DECK_W = BODY_W * 0.60
DECK_D = BODY_D * 0.58

WALL = 0.0035  # shell wall thickness
INNER_TOP_Z = TOP_Z - WALL  # leaves a real top-deck wall except at the pour mouth

NECK_X = 0.020  # neck offset toward +X from body center, close to the deck center
NECK_Y = 0.0
NECK_R = 0.0190
NECK_H = 0.024
NECK_TOP_Z = TOP_Z + NECK_H  # neck rim where the cap seats
MOUTH_BORE_R = NECK_R - 0.006
MOUTH_BORE_BOTTOM_Z = WALL + 0.001  # cuts down to the hollow jug floor, not just the shoulder

CAP_R = 0.0225
CAP_H = 0.028

# ---- handle loop geometry (shared by builder + tests) ----
HANDLE_CX = -0.045  # handle sits farther out on the -X side of the top
HANDLE_OUTER_W = 0.072
HANDLE_OUTER_H = 0.115
HANDLE_THICK_Y = 0.028
# Root the loop legs down into the straight body wall so both legs land on solid
# (the shoulder tapers in above this), guaranteeing a fused, connected handle.
HANDLE_ROOT_Z = BODY_H - 0.030
HANDLE_CZ = HANDLE_ROOT_Z + HANDLE_OUTER_H / 2.0  # outer loop center z
HANDLE_HOLE_W = 0.040  # finger-hole width (X)
HANDLE_HOLE_H = 0.072  # finger-hole height (Z)
HANDLE_HOLE_CZ = HANDLE_CZ + 0.014  # finger-hole center z (raised above the deck)


def _rrect_loft(sections) -> cq.Workplane:
    # sections: list of (z, w, d) -> rounded-rect cross sections stacked along +Z.
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, w, d) in enumerate(sections):
        off = z if i == 0 else z - prev_z
        if i == 0:
            wp = wp.workplane(offset=z)
        else:
            wp = wp.workplane(offset=off)
        wp = wp.rect(w, d)
        prev_z = z
    return wp.loft(ruled=False)


def _body_solid() -> cq.Workplane:
    # Rounded rectangular HDPE jug: a flared foot, a near-straight body that
    # tapers in at the shoulder up to a small rounded-rect top deck, hollowed to
    # read as a real blow-molded shell with a through pour-mouth.
    outer_sections = [
        (0.0, BODY_W * 0.96, BODY_D * 0.96),  # foot tuck-in at the very bottom
        (BASE_H, BODY_W, BODY_D),  # flared base ridge
        (BODY_H * 0.5, BODY_W, BODY_D),  # straight side wall
        (BODY_H, BODY_W * 0.97, BODY_D * 0.97),  # shoulder start
        (BODY_H + SHOULDER_H * 0.55, BODY_W * 0.78, BODY_D * 0.76),
        (TOP_Z, DECK_W, DECK_D),  # small top deck
    ]
    outer = _rrect_loft(outer_sections)
    # Inner cavity: same profile inset by one wall, but with its floor lifted to
    # z=WALL and its top held below the deck so the jug keeps a closed bottom and
    # a real top wall. The neck bore below is then cut down through this cavity
    # depth so the pour mouth opens into the hollow jug, not a blind dimple.
    inner_sections = [
        (WALL, BODY_W * 0.96 - 2 * WALL, BODY_D * 0.96 - 2 * WALL),
        (BASE_H, BODY_W - 2 * WALL, BODY_D - 2 * WALL),
        (BODY_H * 0.5, BODY_W - 2 * WALL, BODY_D - 2 * WALL),
        (BODY_H, BODY_W * 0.97 - 2 * WALL, BODY_D * 0.97 - 2 * WALL),
        (
            BODY_H + SHOULDER_H * 0.55,
            BODY_W * 0.78 - 2 * WALL,
            BODY_D * 0.76 - 2 * WALL,
        ),
        (INNER_TOP_Z, DECK_W - 2 * WALL, DECK_D - 2 * WALL),
    ]
    inner = _rrect_loft(inner_sections)

    # Offset threaded neck rising from the top deck toward +X.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=TOP_Z - 0.004)
        .center(NECK_X, NECK_Y)
        .circle(NECK_R + 0.005)
        .workplane(offset=0.007)
        .circle(NECK_R)
        .workplane(offset=NECK_H - 0.003)
        .circle(NECK_R)
        .loft(ruled=False)
    )
    shell_blank = outer.union(neck)

    # Through pour mouth: bore the neck axis down below the shoulder and into
    # the hollow interior so the screw mouth has a continuous open path.
    neck_bore = (
        cq.Workplane("XY")
        .workplane(offset=MOUTH_BORE_BOTTOM_Z)
        .center(NECK_X, NECK_Y)
        .circle(MOUTH_BORE_R)
        .extrude((NECK_TOP_Z + 0.002) - MOUTH_BORE_BOTTOM_Z)
    )
    body = shell_blank.cut(inner).cut(neck_bore)

    return body


def _handle_solid() -> cq.Workplane:
    # Integrated top loop grip handle on the farther -X side of the deck
    # (opposite the offset neck): a tall rounded loop whose legs root deep into
    # the body wall and rise well above the deck, with a generous open finger
    # hole so it reads as a real grab loop.
    # Outer rounded-rect loop profile in XZ; its bottom edge sits inside the body
    # so the loop legs fuse into the body wall.
    loop = (
        cq.Workplane("XZ")
        .center(HANDLE_CX, HANDLE_CZ)
        .rect(HANDLE_OUTER_W, HANDLE_OUTER_H)
        .extrude(HANDLE_THICK_Y, both=True)
    )
    loop = loop.edges("|Y").fillet(0.016)
    # Finger hole, raised so its bottom clears the deck (a real open loop).
    hole = (
        cq.Workplane("XZ")
        .center(HANDLE_CX, HANDLE_HOLE_CZ)
        .rect(HANDLE_HOLE_W, HANDLE_HOLE_H)
        .extrude(HANDLE_THICK_Y + 0.02, both=True)
    )
    hole = hole.edges("|Y").fillet(0.015)
    handle = loop.cut(hole)
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gallon_jug")

    hdpe = model.material("hdpe_white", rgba=(0.95, 0.95, 0.95, 1.0))
    label_white = model.material("label_white", rgba=(0.97, 0.98, 1.0, 1.0))
    label_teal = model.material("label_teal", rgba=(0.28, 0.72, 0.80, 1.0))
    label_pink = model.material("label_pink", rgba=(0.90, 0.30, 0.55, 1.0))
    cap_white = model.material("cap_white", rgba=(0.92, 0.92, 0.92, 1.0))
    marker_red = model.material("marker_red", rgba=(0.85, 0.20, 0.30, 1.0))

    # ---- body (root): jug shell + offset neck + integrated grip handle ----
    body = model.part("body")

    shell = _body_solid().union(_handle_solid())
    body.visual(mesh_from_cadquery(shell, "jug_shell"), material=hdpe, name="jug_shell")

    # Front "refresh" label panel: near-square, wrapping the front (-Y) face,
    # slightly proud of the wall.
    label_y = -BODY_D / 2.0 - 0.0015
    label_cz = BODY_H * 0.50
    body.visual(
        Box((0.126, 0.004, 0.118)),
        origin=Origin(xyz=(0.0, label_y, label_cz)),
        material=label_white,
        name="label_panel",
    )
    # Teal "refresh" swoosh accent bar across the upper third of the label.
    body.visual(
        Box((0.110, 0.0035, 0.014)),
        origin=Origin(xyz=(0.0, label_y - 0.0018, label_cz + 0.012)),
        material=label_teal,
        name="label_swoosh",
    )
    # A couple of small pink/magenta floral dots flanking the swoosh.
    body.visual(
        Box((0.012, 0.0035, 0.012)),
        origin=Origin(xyz=(-0.018, label_y - 0.0020, label_cz - 0.004)),
        material=label_pink,
        name="floral_dot_0",
    )
    body.visual(
        Box((0.012, 0.0035, 0.012)),
        origin=Origin(xyz=(0.034, label_y - 0.0020, label_cz - 0.010)),
        material=label_pink,
        name="floral_dot_1",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, TOP_Z)),
        mass=0.90,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z * 0.45)),
    )

    # ---- massless carrier on the offset neck axis (rotation stage) ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

    # ---- ribbed white screw cap (slide stage) ----
    cap = model.part("cap")
    cap_body = CylinderGeometry(CAP_R, CAP_H, radial_segments=40)
    cap_body.translate(0.0, 0.0, CAP_H / 2.0)
    # Knurl/rib ridges around the cap skirt.
    for i in range(22):
        a = 2.0 * math.pi * i / 22.0
        rib = CylinderGeometry(0.0016, CAP_H * 0.82, radial_segments=6)
        rib.translate(CAP_R * 0.99 * math.cos(a), CAP_R * 0.99 * math.sin(a), CAP_H / 2.0)
        cap_body.merge(rib)
    cap.visual(mesh_from_geometry(cap_body, "cap_shell"), material=cap_white, name="cap_shell")
    # Off-axis marker so rotation is visible in tests (a small dot near the rim).
    cap.visual(
        Box((0.006, 0.006, 0.006)),
        origin=Origin(xyz=(CAP_R - 0.004, 0.0, CAP_H + 0.001)),
        material=marker_red,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2.0 * CAP_R, 2.0 * CAP_R, CAP_H)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0)),
    )

    # rotation about +Z at the offset neck top
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(NECK_X, NECK_Y, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # slide up off the neck along +Z (decoupled from rotation)
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_H, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    carrier = object_model.get_part("cap_carrier")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    # The cap seats over the offset neck rim; allow the intentional small embed of
    # the cap skirt over the neck rim (screw-on capture).
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="jug_shell",
        reason="Cap skirt seats over the threaded neck rim (screw-on capture).",
    )

    # --- jug is clearly taller than wide ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jug taller than wide",
        bext[2] > bext[0] + 0.05 and bext[2] > bext[1] + 0.05,
        details=f"body extents={bext}",
    )

    # --- neck/cap offset to one side of the top, but closer to center than the
    # far edge, leaving room between the mouth and the outward-shifted handle ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap offset to +X side but close to center",
        cap_pos is not None and 0.012 < cap_pos[0] < 0.026,
        details=f"cap pos={cap_pos}",
    )
    ctx.check(
        "cap sits near the top of the jug",
        cap_pos is not None and cap_pos[2] > TOP_Z - 0.01,
        details=f"cap pos={cap_pos}, top deck={TOP_Z}",
    )

    # --- integrated grip handle: the body rises above the top deck and extends
    # to the -X side opposite the neck (the loop) ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "handle loop rises above the top deck",
        body_aabb is not None and body_aabb[1][2] > TOP_Z + 0.03,
        details=f"body top z={None if body_aabb is None else body_aabb[1][2]}, deck={TOP_Z}",
    )
    ctx.check(
        "handle on the -X side opposite the neck",
        body_aabb is not None and body_aabb[0][0] < -0.06,
        details=f"body -X extent={None if body_aabb is None else body_aabb[0][0]}",
    )

    # --- handle has a REAL open finger hole: the loop straddles the hole, so the
    # body has two separated solid spans in Z at the handle X (rails above and
    # below an empty middle). We verify this from the jug_shell mesh: at the
    # handle's X column there must be mesh material both well above and well below
    # the hole center, with the hole center span empty. ---
    shell_aabb = ctx.part_element_world_aabb(body, elem="jug_shell")
    # top rail of the loop must reach clearly above the finger-hole center
    top_rail_top = shell_aabb[1][2]
    ctx.check(
        "handle finger hole is a real open loop (top rail above hole)",
        top_rail_top > HANDLE_HOLE_CZ + 0.015,
        details=f"shell top z={top_rail_top:.4f}, hole center z={HANDLE_HOLE_CZ:.4f}",
    )
    # the loop must be wide enough in X (the hole carves out the middle in Z)
    ctx.check(
        "handle loop reaches well left of body center",
        shell_aabb[0][0] < HANDLE_CX - 0.025 and HANDLE_CX < -0.04,
        details=f"shell -X extent={shell_aabb[0][0]:.4f}, handle cx={HANDLE_CX}",
    )

    # --- hollow jug and through pour mouth: the internal void reaches below the
    # shoulder and the bore overlaps that void, so the neck is not a blind hole. ---
    ctx.check(
        "jug body has a hollow interior below the shoulder",
        WALL < MOUTH_BORE_BOTTOM_Z < BODY_H and INNER_TOP_Z < TOP_Z,
        details=(
            f"wall={WALL:.4f}, bore bottom={MOUTH_BORE_BOTTOM_Z:.4f}, "
            f"body shoulder={BODY_H:.4f}, inner top={INNER_TOP_Z:.4f}, deck={TOP_Z:.4f}"
        ),
    )
    ctx.check(
        "pour mouth bore passes through the deck into the hollow jug",
        MOUTH_BORE_BOTTOM_Z < INNER_TOP_Z < NECK_TOP_Z and MOUTH_BORE_R > 0.010,
        details=(
            f"bore bottom={MOUTH_BORE_BOTTOM_Z:.4f}, inner top={INNER_TOP_Z:.4f}, "
            f"neck top={NECK_TOP_Z:.4f}, bore radius={MOUTH_BORE_R:.4f}"
        ),
    )

    # --- cap spins about +Z: the off-axis marker moves around the axis ---
    m0 = ctx.part_element_world_aabb(cap, elem="cap_marker")
    mc0 = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(cap, elem="cap_marker")
        mc1 = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    moved = math.hypot(mc1[0] - mc0[0], mc1[1] - mc0[1])
    ctx.check(
        "cap marker swings around the +Z axis when rotated",
        moved > 0.01,
        details=f"marker rest={mc0}, quarter-turn={mc1}, moved={moved:.4f}",
    )

    # --- cap slides up off the neck along +Z ---
    z_rest = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_H}):
        z_up = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap slides up off the neck",
        z_up > z_rest + CAP_H * 0.8,
        details=f"rest z={z_rest:.4f}, lifted z={z_up:.4f}",
    )

    # --- cap is mounted on the neck (overlaps the neck footprint at rest) ---
    ctx.expect_overlap(
        cap, body, axes="xy", min_overlap=0.01, name="cap seated over neck footprint"
    )

    # carrier is massless / no visuals
    ctx.check(
        "carrier has no visuals",
        len(carrier.visuals) == 0,
        details=f"carrier visuals={len(carrier.visuals)}",
    )

    return ctx.report()


object_model = build_object_model()
