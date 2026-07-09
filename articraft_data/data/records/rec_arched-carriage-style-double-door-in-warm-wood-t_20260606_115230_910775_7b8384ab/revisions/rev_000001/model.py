from __future__ import annotations

# Arched carriage-style double door in warm wood.
# Two leaves: upper arched glass panes divided by muntins, lower X-braced board
# panels, ornate black strap hinges and ring/lever hardware, set within an
# arched stone surround with pilasters and a keystone.
#
# Coordinate frame: X = width (right +), Y = depth/thickness (toward viewer +),
# Z = height (up). The stone surround (pilasters, entablature, arch ring,
# keystone) is the FIXED root. Two wood leaves meet at center and swing on
# vertical (Z) hinge axes at their OUTER edges.
#
# Mirror symmetry: ONE leaf is authored in a leaf-local frame (hinge edge at
# local x=0, body extending to +x, FRONT face at +Y). The right leaf is the
# exact MIRROR of the left across the center plane: every solid has its local
# X negated (mirror=True), so the body extends to -x while the front face stays
# at +Y. Both leaves are placed with NO yaw, so all hardware (straps, ring,
# X-brace) reads on the same +Y front face and the grain/pattern is a true
# left-right mirror -- never rotational symmetry that would flip the second
# leaf's face to the back.
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

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
OPENING_W = 1.50  # clear doorway width (pair)
LEAF_W = OPENING_W / 2.0  # one leaf = 0.75 m
SPRING_Z = 1.95  # height where the semicircular arch springs
ARCH_R = OPENING_W / 2.0  # arch radius -> full semicircle across the pair
ARCH_TOP = SPRING_Z + ARCH_R  # apex height ~2.7 m
LEAF_T = 0.05  # leaf thickness

GLASS_SPLIT_Z = 1.30  # rail dividing upper glass from lower X-braced boards
STONE_W = 0.26  # pilaster / jamb stone width
STONE_D = 0.22  # surround depth

# Floor datum: the stone threshold/base course rests ON the floor (z=0) and is
# BASE_Z tall. Pilasters, arch, keystone and the door leaves all start at the
# top of the threshold (z = BASE_Z), so the whole structure sits at zmin=0.
BASE_Z = 0.10

DOOR_HALF = OPENING_W / 2.0

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
WOOD = Material(name="warm_wood", rgba=(0.80, 0.48, 0.24, 1.0))
WOOD_DARK = Material(name="board_shadow", rgba=(0.60, 0.34, 0.16, 1.0))
STONE = Material(name="light_stone", rgba=(0.82, 0.80, 0.74, 1.0))
GLASS = Material(name="door_glass", rgba=(0.30, 0.42, 0.46, 0.45))
IRON = Material(name="wrought_iron", rgba=(0.08, 0.08, 0.09, 1.0))


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _mirror_x(wp: cq.Workplane, mirror: bool) -> cq.Workplane:
    """Return ``wp`` mirrored across the YZ plane (X -> -X) when ``mirror`` is
    True, otherwise unchanged. Used to turn the authored LEFT leaf into the
    RIGHT leaf so the two are exact mirror images (front face stays at +Y)."""
    if not mirror:
        return wp
    return wp.mirror(mirrorPlane="YZ")


def _leaf_profile_local() -> cq.Workplane:
    """The doorway profile for ONE leaf in the leaf-local frame: a rectangle
    from z=0..SPRING_Z spanning x=0..LEAF_W, plus the arched top (the half of
    the semicircle that lies over this leaf). The local hinge edge is at x=0;
    the meeting edge is at x=LEAF_W where the arch center sits."""
    rect = cq.Workplane("XY").box(LEAF_W, LEAF_T, SPRING_Z, centered=(False, True, False))
    # Arch cylinder centered on the leaf (y=0), extruded symmetrically through
    # the full leaf thickness via both=True.
    arch_cyl = (
        cq.Workplane("XZ", origin=(LEAF_W, 0.0, SPRING_Z))
        .circle(ARCH_R)
        .extrude(LEAF_T / 2.0, both=True)
    )
    clip = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T + 0.02, ARCH_R + 0.02, centered=(False, True, False))
        .translate((0.0, 0.0, SPRING_Z))
    )
    arch = arch_cyl.intersect(clip)
    return rect.union(arch)


def _leaf_frame_and_panels(mirror: bool = False) -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    """Returns (wood_frame, glass, x_brace) for one leaf in the leaf-local
    frame. The wood frame has the upper arched glass opening cut out, leaving a
    stile/rail border and a central vertical muntin. The window is divided by a
    grid of muntins (one vertical + two horizontal bars). When ``mirror`` is
    True every solid is mirrored across X so the right leaf is the exact mirror
    of the left (the X-braces meet to form a continuous V/diamond across the
    closed pair)."""
    half_t = LEAF_T / 2.0
    body = _leaf_profile_local()

    stile = 0.10
    front_y = half_t

    win_z0 = GLASS_SPLIT_Z + 0.06
    upper_clip = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T + 0.1, (ARCH_TOP - win_z0) + 0.2, centered=(False, True, False))
        .translate((0.0, 0.0, win_z0))
    )
    inset_rect = (
        cq.Workplane("XY")
        .box(LEAF_W - 2 * stile, LEAF_T + 0.1, SPRING_Z, centered=(False, True, False))
        .translate((stile, 0.0, 0.0))
    )
    inset_arch_cyl = (
        cq.Workplane("XZ", origin=(LEAF_W, 0.0, SPRING_Z))
        .circle(ARCH_R - stile)
        .extrude((LEAF_T + 0.1) / 2.0, both=True)
    )
    inset_arch_clip = (
        cq.Workplane("XY")
        .box(LEAF_W, LEAF_T + 0.2, ARCH_R, centered=(False, True, False))
        .translate((0.0, 0.0, SPRING_Z))
    )
    inset_profile = inset_rect.union(inset_arch_cyl.intersect(inset_arch_clip))
    window_solid = inset_profile.intersect(upper_clip)

    # Muntin grid dividing the upper glass: one vertical bar down the middle
    # plus two horizontal bars, so each leaf reads as multiple divided panes.
    muntin_w = 0.030
    muntin = (
        cq.Workplane("XY")
        .box(muntin_w, LEAF_T + 0.1, ARCH_TOP, centered=(True, True, False))
        .translate((LEAF_W * 0.5, 0.0, win_z0))
    )
    win_top = ARCH_TOP
    for hz in (win_z0 + (win_top - win_z0) * 0.36, win_z0 + (win_top - win_z0) * 0.70):
        bar = (
            cq.Workplane("XY")
            .box(LEAF_W, LEAF_T + 0.1, muntin_w, centered=(False, True, True))
            .translate((0.0, 0.0, hz))
        )
        muntin = muntin.union(bar)
    window_cut = window_solid.cut(muntin)

    frame = body.cut(window_cut)

    # Glass pane filling the window opening, thin and centered in Y.
    glass = cq.Workplane("XY").box(LEAF_W, 0.012, ARCH_TOP, centered=(False, True, False))
    glass = glass.intersect(window_cut)

    # Lower X-brace: two diagonal boards across the lower board panel.
    lower_top = GLASS_SPLIT_Z
    bx0, bx1 = stile, LEAF_W - stile
    bz0, bz1 = 0.12, lower_top - 0.04
    board_t = 0.06
    brace_y = front_y - 0.006
    diag_len = math.hypot(bx1 - bx0, bz1 - bz0)
    ang = math.degrees(math.atan2(bz1 - bz0, bx1 - bx0))
    cxm = (bx0 + bx1) / 2.0
    czm = (bz0 + bz1) / 2.0
    diag1 = (
        cq.Workplane("XZ", origin=(cxm, brace_y, czm))
        .box(diag_len, board_t, 0.018)
        .rotate((cxm, brace_y, czm), (cxm, brace_y - 1, czm), ang)
    )
    diag2 = (
        cq.Workplane("XZ", origin=(cxm, brace_y, czm))
        .box(diag_len, board_t, 0.018)
        .rotate((cxm, brace_y, czm), (cxm, brace_y - 1, czm), -ang)
    )
    xbrace = diag1.union(diag2)
    # Horizontal ledger boards top and bottom of the X-brace, as on a real
    # ledged-and-braced carriage door.
    for lz in (bz0, bz1):
        ledger = (
            cq.Workplane("XY", origin=(0.0, brace_y, lz))
            .box(LEAF_W - 2 * stile, board_t, 0.05, centered=(False, True, True))
            .translate((stile, 0.0, 0.0))
        )
        xbrace = xbrace.union(ledger)

    return (
        _mirror_x(frame, mirror),
        _mirror_x(glass, mirror),
        _mirror_x(xbrace, mirror),
    )


def _strap_hinges(mirror: bool = False) -> list[tuple[str, cq.Workplane]]:
    """Three black wrought-iron strap hinges along the hinge (outer) edge of a
    leaf, in the leaf-local frame. Each strap = a horizontal band on the front
    face anchored at the hinge edge, with a barrel pintle at x=0. Each hinge is
    returned as its own connected solid/visual. When ``mirror`` is True each
    strap is mirrored across X so the right leaf's straps land on its (mirrored)
    outer edge -- the hardware mirrors with the leaf."""
    half_t = LEAF_T / 2.0
    front_y = half_t
    strap_len = LEAF_W * 0.55
    strap_w = 0.05
    strap_t = 0.012
    out: list[tuple[str, cq.Workplane]] = []
    for i, z in enumerate((0.35, GLASS_SPLIT_Z * 0.5 + 0.2, GLASS_SPLIT_Z + 0.45)):
        strap = cq.Workplane("XY", origin=(0.0, front_y + strap_t / 2 - 0.002, z)).box(
            strap_len, strap_t, strap_w, centered=(False, True, True)
        )
        # Pintle barrel just inboard of the hinge edge so it reads as wrapped
        # around the edge without crossing into the stone jamb.
        barrel = cq.Workplane("XY", origin=(0.019, front_y - 0.005, z)).cylinder(
            strap_w * 1.4, 0.018, direct=(0, 0, 1)
        )
        out.append((f"strap_{i}", _mirror_x(strap.union(barrel, clean=True), mirror)))
    return out


def _ring_pull(mirror: bool = False) -> cq.Workplane:
    """Iron ring/lever pull near the meeting edge, in the leaf-local frame.
    Mirrored across X with the leaf so the two pulls sit symmetrically at the
    center meeting stiles."""
    half_t = LEAF_T / 2.0
    x = LEAF_W - 0.08
    z = 1.05
    y = half_t
    # Back plate flush on the face, projecting out to y in [face, face+0.02].
    plate = cq.Workplane("XY", origin=(x, y + 0.010, z)).box(
        0.05, 0.024, 0.16, centered=(True, True, True)
    )
    # Ring annulus standing proud of the plate; extrude so its base overlaps the
    # plate front (plate front at y+0.022).
    ring = (
        cq.Workplane("XZ", origin=(x, y + 0.014, z - 0.07))
        .circle(0.045)
        .circle(0.033)
        .extrude(0.018, both=False)
    )
    # Lever bar bridging into the plate.
    lever = cq.Workplane("XY", origin=(x, y + 0.018, z + 0.04)).box(
        0.10, 0.020, 0.016, centered=(True, True, True)
    )
    return _mirror_x(plate.union(ring, clean=True).union(lever, clean=True), mirror)


def _stone_surround() -> list[tuple[str, cq.Workplane]]:
    """Light-stone arched surround built from convex members: two pilasters with
    capitals, a base course, an arched stone ring over the doorway, a keystone,
    and side cornice blocks."""
    members: list[tuple[str, cq.Workplane]] = []
    d = STONE_D

    def member(name, wp):
        # Lift every surround member to the floor datum so the threshold/base
        # course rests at z=0 and nothing dips below the floor.
        members.append((name, wp.translate((0.0, 0.0, BASE_Z))))

    pil_h = SPRING_Z + 0.10
    for sgn, nm in ((-1.0, "pilaster_left"), (1.0, "pilaster_right")):
        cx = sgn * (DOOR_HALF + STONE_W / 2.0)
        member(
            nm,
            cq.Workplane("XY")
            .box(STONE_W, d, pil_h, centered=(True, True, False))
            .translate((cx, 0.0, 0.0)),
        )
        member(
            f"{nm}_capital",
            cq.Workplane("XY")
            .box(STONE_W - 0.02, d + 0.03, 0.10, centered=(True, True, False))
            .translate((cx, 0.0, pil_h)),
        )

    half_w_outer = DOOR_HALF + STONE_W
    # Base course / threshold: authored at z in [-BASE_Z, 0] so after the
    # member() lift it rests on the floor at z in [0, BASE_Z].
    member(
        "base_course",
        cq.Workplane("XY")
        .box(2 * half_w_outer, d, BASE_Z, centered=(True, True, False))
        .translate((0.0, 0.0, -BASE_Z)),
    )

    spring = SPRING_Z + 0.10
    outer = (
        cq.Workplane("XZ", origin=(0.0, 0.0, spring))
        .circle(ARCH_R + STONE_W)
        .circle(ARCH_R)
        .extrude(d / 2.0, both=True)
    )
    upper_clip = (
        cq.Workplane("XY")
        .box(
            2 * (ARCH_R + STONE_W) + 0.1,
            d + 0.1,
            (ARCH_R + STONE_W) + 0.2,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, spring))
    )
    arch_ring = outer.intersect(upper_clip)
    member("arch_ring", arch_ring)

    member(
        "keystone",
        cq.Workplane("XY")
        .box(0.16, d + 0.04, 0.26, centered=(True, True, False))
        .translate((0.0, 0.0, spring + ARCH_R - 0.05)),
    )

    # Side cornice blocks above each capital. Sized and centered over the
    # pilaster footprint so they stay clear of the door arch shoulders.
    cornice_w = STONE_W - 0.02
    for sgn, nm in ((-1.0, "cornice_left"), (1.0, "cornice_right")):
        cx = sgn * (DOOR_HALF + STONE_W / 2.0)
        member(
            nm,
            cq.Workplane("XY")
            .box(cornice_w, d + 0.05, 0.08, centered=(True, True, False))
            .translate((cx, 0.0, pil_h + 0.10)),
        )

    return members


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arched_carriage_double_door")
    for mat in (WOOD, WOOD_DARK, STONE, GLASS, IRON):
        model.material(mat.name, rgba=mat.rgba)

    # --- Fixed root: stone surround ---
    surround = model.part("stone_surround")
    for name, member in _stone_surround():
        surround.visual(
            mesh_from_cadquery(member, f"stone_{name}"),
            material=STONE,
            name=f"stone_{name}",
        )

    # --- Two operable wood leaves (door_1 is the exact mirror of door_0) ---
    for idx in (0, 1):
        mirror = idx == 1
        leaf = model.part(f"door_{idx}")
        frame, glass, xbrace = _leaf_frame_and_panels(mirror=mirror)
        leaf.visual(
            mesh_from_cadquery(frame, f"door_frame_{idx}"),
            material=WOOD,
            name=f"door_frame_{idx}",
        )
        leaf.visual(
            mesh_from_cadquery(glass, f"door_glass_{idx}"),
            material=GLASS,
            name=f"door_glass_{idx}",
        )
        leaf.visual(
            mesh_from_cadquery(xbrace, f"door_xbrace_{idx}"),
            material=WOOD_DARK,
            name=f"door_xbrace_{idx}",
        )
        for sname, strap in _strap_hinges(mirror=mirror):
            leaf.visual(
                mesh_from_cadquery(strap, f"door_{sname}_{idx}"),
                material=IRON,
                name=f"door_{sname}_{idx}",
            )
        leaf.visual(
            mesh_from_cadquery(_ring_pull(mirror=mirror), f"door_ring_{idx}"),
            material=IRON,
            name=f"door_ring_{idx}",
        )

    # --- Articulations: vertical hinge axes at the OUTER edges ---
    # door_0 hinges at world x=-DOOR_HALF, body extends +x to meet center.
    # door_1 is the MIRRORED leaf: hinge at world x=+DOOR_HALF, body extends -x
    # to meet center. NO yaw on door_1 -- the mirror is baked into geometry, so
    # both front faces stay at +Y and the pair reads mirror-symmetric. Opening
    # outward = each leaf swinging toward +Y, so door_0 opens with positive
    # rotation and door_1 with negative rotation.
    model.articulation(
        "surround_to_door_0",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=model.get_part("door_0"),
        origin=Origin(xyz=(-DOOR_HALF, 0.0, BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=1.9),
    )
    model.articulation(
        "surround_to_door_1",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=model.get_part("door_1"),
        origin=Origin(xyz=(DOOR_HALF, 0.0, BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-1.9, upper=0.0),
    )
    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    surround = object_model.get_part("stone_surround")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    j0 = object_model.get_articulation("surround_to_door_0")
    j1 = object_model.get_articulation("surround_to_door_1")

    # --- Leaf scale: real carriage-door leaf, arched top reaching ~2.7 m ---
    for leaf, frame_elem in ((door_0, "door_frame_0"), (door_1, "door_frame_1")):
        aabb = ctx.part_element_world_aabb(leaf, elem=frame_elem)
        assert aabb is not None
        w = aabb[1][0] - aabb[0][0]
        top = aabb[1][2]
        ctx.check(
            f"{leaf.name} leaf width is realistic",
            0.65 <= w <= 0.90,
            details=f"width={w:.3f}",
        )
        ctx.check(
            f"{leaf.name} reaches the arch apex",
            top >= ARCH_TOP - 0.05,
            details=f"top={top:.3f} apex={ARCH_TOP:.3f}",
        )

    # --- Hero features present on each leaf ---
    for idx, leaf in ((0, door_0), (1, door_1)):
        glass = ctx.part_element_world_aabb(leaf, elem=f"door_glass_{idx}")
        xb = ctx.part_element_world_aabb(leaf, elem=f"door_xbrace_{idx}")
        # Aggregate the three strap-hinge visuals into one bounding extent.
        strap_aabbs = [
            ctx.part_element_world_aabb(leaf, elem=f"door_strap_{si}_{idx}") for si in range(3)
        ]
        assert all(a is not None for a in strap_aabbs)
        s_minx = min(a[0][0] for a in strap_aabbs)
        s_maxx = max(a[1][0] for a in strap_aabbs)
        s_minz = min(a[0][2] for a in strap_aabbs)
        s_maxz = max(a[1][2] for a in strap_aabbs)
        ring = ctx.part_element_world_aabb(leaf, elem=f"door_ring_{idx}")
        assert glass and xb and ring
        ctx.check(
            f"{leaf.name} upper arched glass present",
            glass[1][2] >= SPRING_Z and (glass[1][2] - glass[0][2]) >= 0.5,
            details=f"glass z=[{glass[0][2]:.2f},{glass[1][2]:.2f}]",
        )
        xb_w = xb[1][0] - xb[0][0]
        xb_h = xb[1][2] - xb[0][2]
        ctx.check(
            f"{leaf.name} lower X-brace spans the board panel",
            xb_w >= 0.4 and xb_h >= 0.6 and xb[1][2] <= BASE_Z + GLASS_SPLIT_Z + 0.05,
            details=f"xb w={xb_w:.2f} h={xb_h:.2f} top={xb[1][2]:.2f}",
        )
        straps_w = s_maxx - s_minx
        straps_span_z = s_maxz - s_minz
        ctx.check(
            f"{leaf.name} strap hinges present and span inward from the edge",
            straps_w >= 0.30 and straps_span_z >= 1.0,
            details=f"straps width={straps_w:.2f} z_span={straps_span_z:.2f}",
        )
        ctx.check(
            f"{leaf.name} ring/lever pull present",
            (ring[1][2] - ring[0][2]) >= 0.10,
            details=f"ring h={ring[1][2] - ring[0][2]:.2f}",
        )

    # --- Stone surround is the fixed root and frames the arch ---
    s_aabb = ctx.part_world_aabb(surround)
    assert s_aabb is not None
    s_w = s_aabb[1][0] - s_aabb[0][0]
    s_top = s_aabb[1][2]
    ctx.check(
        "stone surround is wider than the doorway (pilasters present)",
        s_w >= OPENING_W + 1.5 * STONE_W,
        details=f"surround width={s_w:.2f}",
    )
    ctx.check(
        "stone surround rises over the arch with a keystone",
        s_top >= ARCH_TOP,
        details=f"surround top={s_top:.2f} apex={ARCH_TOP:.2f}",
    )
    key = ctx.part_element_world_aabb(surround, elem="stone_keystone")
    arch = ctx.part_element_world_aabb(surround, elem="stone_arch_ring")
    assert key and arch
    ctx.check(
        "keystone sits at the arch apex",
        key[1][2] >= ARCH_TOP - 0.02 and abs((key[0][0] + key[1][0]) / 2.0) < 0.10,
        details=f"key top={key[1][2]:.2f}",
    )
    ctx.check(
        "arch ring spans above the spring line",
        arch[0][2] >= SPRING_Z - 0.05 and (arch[1][0] - arch[0][0]) >= OPENING_W,
        details=f"arch minz={arch[0][2]:.2f} w={arch[1][0] - arch[0][0]:.2f}",
    )

    # --- Mirror symmetry: door_1 is the exact left-right mirror of door_0 ---
    # In the closed pose every matching element of door_1 must sit at the
    # negated X of door_0's element and at the SAME Y (same +Y front face) and
    # SAME Z. This is the user's key requirement: the grain/pattern/hardware is
    # mirror-symmetric, NOT rotationally symmetric (which would flip the second
    # leaf's hardware to the back face).
    def _centroid(leaf, elem):
        a = ctx.part_element_world_aabb(leaf, elem=elem)
        assert a is not None, f"missing element {elem}"
        return (
            0.5 * (a[0][0] + a[1][0]),
            0.5 * (a[0][1] + a[1][1]),
            0.5 * (a[0][2] + a[1][2]),
        )

    with ctx.pose({j0: 0.0, j1: 0.0}):
        mirror_pairs = [
            ("door_frame_0", "door_frame_1"),
            ("door_glass_0", "door_glass_1"),
            ("door_xbrace_0", "door_xbrace_1"),
            ("door_ring_0", "door_ring_1"),
            ("door_strap_0_0", "door_strap_0_1"),
            ("door_strap_1_0", "door_strap_1_1"),
            ("door_strap_2_0", "door_strap_2_1"),
        ]
        for e0, e1 in mirror_pairs:
            c0 = _centroid(door_0, e0)
            c1 = _centroid(door_1, e1)
            ctx.check(
                f"{e1} mirrors {e0} across center (X negated)",
                abs(c1[0] + c0[0]) < 0.01,
                details=f"x0={c0[0]:.3f} x1={c1[0]:.3f}",
            )
            ctx.check(
                f"{e1} shares the front face / height of {e0} (Y,Z equal)",
                abs(c1[1] - c0[1]) < 0.01 and abs(c1[2] - c0[2]) < 0.01,
                details=f"y0={c0[1]:.3f} y1={c1[1]:.3f} z0={c0[2]:.3f} z1={c1[2]:.3f}",
            )
        # Hardware must be on the +Y front face of BOTH leaves (not flipped).
        ctx.check(
            "both ring pulls on the +Y front face",
            _centroid(door_0, "door_ring_0")[1] > 0.0 and _centroid(door_1, "door_ring_1")[1] > 0.0,
            details="ring pulls must share the front face",
        )

    # --- Closed pose: leaves meet at center, do not interpenetrate ---
    with ctx.pose({j0: 0.0, j1: 0.0}):
        ctx.expect_gap(door_1, door_0, axis="x", min_gap=-0.01, max_gap=0.03)

    # --- Open pose: both leaves swing outward, hinges stay attached ---
    def _frame_y(leaf, elem):
        a = ctx.part_element_world_aabb(leaf, elem=elem)
        return None if a is None else 0.5 * (a[0][1] + a[1][1])

    cy0 = _frame_y(door_0, "door_frame_0")
    cy1 = _frame_y(door_1, "door_frame_1")
    with ctx.pose({j0: 1.6, j1: -1.6}):
        oy0 = _frame_y(door_0, "door_frame_0")
        oy1 = _frame_y(door_1, "door_frame_1")
        ctx.expect_contact(door_0, surround, name="door_0 hinge stays attached")
        ctx.expect_contact(door_1, surround, name="door_1 hinge stays attached")
    assert None not in (cy0, cy1, oy0, oy1)
    ctx.check(
        "door_0 swings outward when opened",
        oy0 > cy0 + 0.20,
        details=f"closed_y={cy0:.3f} open_y={oy0:.3f}",
    )
    ctx.check(
        "door_1 swings outward when opened",
        oy1 > cy1 + 0.20,
        details=f"closed_y={cy1:.3f} open_y={oy1:.3f}",
    )

    return ctx.report()
