from __future__ import annotations

# Ornate wrought-iron double-leaf entrance gate in a plain rectangular surround.
#
# Articraft brief
# - Object: ornate wrought-iron double-leaf entrance gate set in a plain
#   rectangular stone / plaster surround with a flat horizontal iron top rail
#   across the head of the opening. No arch and no fanlight grille.
#   Overall opening ~2.2 m wide x ~2.24 m tall; each leaf ~1.06 m x ~2.18 m.
# - Frame: world up is +Z. The assembly stands on the ground plane (z>=0).
#   Gate width runs along X (left<->right), leaf thickness along Y
#   (front<->back), height along Z. The two leaves swing outward toward -Y.
# - Root/support: `stone_surround` = the two masonry jamb pillars, the flat
#   stone lintel spanning the opening head, and the threshold sill.
# - Parts: stone_surround (root), door_0 and door_1 (mirrored iron leaves:
#   perimeter frame, vertical bars, scroll panel, gold accents, latch stile).
# - Articulations: surround_to_door_0 / surround_to_door_1, both REVOLUTE on
#   the VERTICAL (Z) axis at the outer jambs. Each leaf is authored extending
#   inward from its hinge toward the center. door_1's axis is negated and the
#   leaf is mimic-coupled to door_0 so positive q swings BOTH leaves outward.
# - Visible geometry: hollow barred iron leaves (gaps between vertical bars),
#   gold scrollwork rosettes, plain rectangular stone surround with a flat
#   lintel header, flat horizontal iron top rail spanning the opening head,
#   cream plaster reveal (sides + head), dark threshold.
# - Intentional overlaps: hinge knuckles embed slightly into the jamb pillars
#   (captured-pin pattern); scoped allowance only.
# - Tests: both leaves present, closed leaves meet in the middle and seat
#   against their jambs, an open pose swings each free edge outward (-Y),
#   rectangular surround has a flat lintel (no arch), top rail spans the
#   opening head, no floating parts.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------

LEAF_W = 1.06          # one leaf width (X)
LEAF_H = 2.18          # one leaf height (Z)
LEAF_T = 0.055         # leaf frame depth (Y)
JAMB_REVEAL = 0.03     # reveal gap between the leaf hinge edge and the jamb
OPENING_W = 2.0 * (LEAF_W + JAMB_REVEAL) + 0.04   # clear opening width
SILL_Z = 0.04          # threshold sill thickness; leaves start just above it
LEAF_Z0 = SILL_Z       # bottom of each leaf
LEAF_ZC = LEAF_Z0 + LEAF_H / 2.0  # leaf vertical center

PILLAR_W = 0.30        # jamb pillar width (X)
PILLAR_D = 0.34        # jamb pillar / wall depth (Y)

CENTER_GAP = 0.02      # gap where the two leaves meet
FRAME_W = 0.075        # leaf stile / rail bar width
BAR_W = 0.020          # vertical picket bar (square cross-section)
N_BARS = 7             # vertical pickets per leaf
SCROLL_BAR = 0.016     # iron scroll-bar cross-section width

# Hinge geometry
KNUCKLE_R = 0.030
KNUCKLE_LEN = 0.10
PIN_R = 0.012

# Top rail dimensions
TOP_RAIL_H = 0.06      # rail height (Z)
TOP_RAIL_D = 0.050     # rail depth (Y)
HEAD_CLEAR = 0.015     # clearance gap between leaf top and top rail bottom

# Opening head: the pillar height must clear the leaf + top rail + gap.
PILLAR_H = LEAF_Z0 + LEAF_H + HEAD_CLEAR + TOP_RAIL_H  # lintel underside
LINTEL_H = 0.25        # flat stone lintel height above the opening head
SURROUND_H = PILLAR_H + LINTEL_H  # total surround height from ground


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------


def _materials(model: ArticulatedObject) -> None:
    model.material("stone", rgba=(0.86, 0.83, 0.76, 1.0))
    model.material("plaster", rgba=(0.92, 0.90, 0.84, 1.0))
    model.material("threshold", rgba=(0.32, 0.30, 0.28, 1.0))
    model.material("iron", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("gold", rgba=(0.74, 0.58, 0.20, 1.0))


# ---------------------------------------------------------------------------
# Rectangular stone surround (root) — plain pillars + flat lintel, no arch
# ---------------------------------------------------------------------------


def _rectangular_surround() -> cq.Workplane:
    """Two jamb pillars + a flat stone lintel spanning the opening head,
    forming a plain rectangular surround with no arch.

    The facade lies in the XZ plane; the wall depth runs along Y (thickness
    PILLAR_D, centered on Y=0). The opening is a clean rectangle from the
    sill to the lintel underside. Built from three separate boxes (left pillar,
    right pillar, lintel) unioned together so there is no boolean-cut ambiguity.
    """
    half_open = OPENING_W / 2.0

    # Left pillar: from ground to lintel underside, left of the opening.
    left_x_center = -(half_open + PILLAR_W / 2.0)
    left_pillar = (
        cq.Workplane("XY")
        .box(PILLAR_W, PILLAR_D, PILLAR_H, centered=(True, True, False))
        .translate((left_x_center, 0.0, 0.0))
    )

    # Right pillar: mirror of left.
    right_x_center = half_open + PILLAR_W / 2.0
    right_pillar = (
        cq.Workplane("XY")
        .box(PILLAR_W, PILLAR_D, PILLAR_H, centered=(True, True, False))
        .translate((right_x_center, 0.0, 0.0))
    )

    # Lintel: spans the full width above the opening head.
    total_w = OPENING_W + 2 * PILLAR_W
    lintel = (
        cq.Workplane("XY")
        .box(total_w, PILLAR_D, LINTEL_H, centered=(True, True, False))
        .translate((0.0, 0.0, PILLAR_H))
    )

    return left_pillar.union(right_pillar).union(lintel)


def _top_rail() -> cq.Workplane:
    """Flat horizontal iron rail across the head of the gate opening, sitting
    just below the stone lintel. Spans the full clear opening width."""
    z_center = PILLAR_H - TOP_RAIL_H / 2.0
    return (
        cq.Workplane("XZ")
        .center(0.0, z_center)
        .rect(OPENING_W, TOP_RAIL_H)
        .extrude(TOP_RAIL_D, both=True)
    )


def _threshold() -> cq.Workplane:
    """Dark stone threshold sill the leaves stand on (depth along Y)."""
    return (
        cq.Workplane("XZ")
        .box(OPENING_W + 0.04, SILL_Z, PILLAR_D, centered=(True, False, True))
    )


def _plaster_reveal() -> cq.Workplane:
    """Cream plaster inner reveal band lining the rectangular doorway sides
    and head."""
    lintel_bottom = PILLAR_H
    t = 0.04
    band = 0.05
    half = OPENING_W / 2.0
    rev = cq.Workplane("XY")
    # Left + right vertical reveals.
    for sx in (-1.0, 1.0):
        col = (
            cq.Workplane("XY")
            .box(band, t, lintel_bottom - SILL_Z, centered=(True, True, False))
            .translate((sx * (half - band / 2.0), -PILLAR_D / 2.0 + t / 2.0, SILL_Z))
        )
        rev = rev.union(col)
    # Top head reveal spanning the opening width.
    head = (
        cq.Workplane("XY")
        .box(OPENING_W - 2 * band, t, band, centered=(True, True, False))
        .translate((0.0, -PILLAR_D / 2.0 + t / 2.0, lintel_bottom - band))
    )
    rev = rev.union(head)
    return rev


# ---------------------------------------------------------------------------
# Scrollwork primitives (curled wrought-iron C-scrolls / volutes)
# ---------------------------------------------------------------------------


def _c_arc(cx: float, cz: float, r_out: float, bar: float, depth: float,
           a0_deg: float, a1_deg: float) -> cq.Workplane:
    """A curved iron bar: an angular slice of a thin annulus lying in the XZ
    facade plane, centered at (cx, cz). Used as the building block for the
    C-scrolls and volutes that fill the gate panels.

    `bar` is the iron stock thickness, `depth` the proud face depth (Y), and
    [a0_deg, a1_deg] the swept sector (angles measured CCW from +X)."""
    ro = r_out
    ri = max(r_out - bar, 0.001)
    ring = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .circle(ro)
        .circle(ri)
        .extrude(depth, both=True)
    )
    # Wedge that selects the desired angular sector (a fan of triangles).
    a0 = math.radians(a0_deg)
    a1 = math.radians(a1_deg)
    big = ro * 4.0
    pts = [(0.0, 0.0)]
    steps = max(2, int(abs(a1_deg - a0_deg) / 18) + 1)
    for k in range(steps + 1):
        a = a0 + (a1 - a0) * k / steps
        pts.append((big * math.cos(a), big * math.sin(a)))
    wedge = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .polyline(pts)
        .close()
        .extrude(depth + 0.01, both=True)
    )
    return ring.intersect(wedge)


def _volute(cx: float, cz: float, sign_in: float, scale: float, bar: float,
            depth: float) -> cq.Workplane:
    """A spiral volute: two nested C-arcs of decreasing radius plus a solid
    eye, forming a tight curl. `sign_in` (+1/-1) selects which way it curls."""
    s = sign_in
    # Outer ~3/4 turn of the spiral.
    body = _c_arc(cx, cz, 0.085 * scale, bar, depth, 0, 250)
    # Inner ~3/4 turn, offset toward the eye for the spiral nesting.
    ecx = cx - s * 0.028 * scale
    inner = _c_arc(ecx, cz, 0.050 * scale, bar, depth, 60, 320)
    # Solid eye boss at the spiral center.
    eye = (
        cq.Workplane("XZ")
        .center(ecx, cz)
        .circle(0.018 * scale)
        .extrude(depth, both=True)
    )
    # Short connector bridging the outer body, inner arc, and eye so the volute
    # is always a single connected solid (no hairline islands).
    bridge = (
        cq.Workplane("XZ")
        .center(0.5 * (cx + ecx), cz)
        .rect(abs(cx - ecx) + 0.085 * scale, bar)
        .extrude(depth, both=True)
    )
    return body.union(inner).union(eye).union(bridge)


# ---------------------------------------------------------------------------
# Iron gate leaf
# ---------------------------------------------------------------------------


def _leaf_iron(sign: float) -> cq.Workplane:
    """Black iron leaf authored in its own local frame.

    The hinge edge is at local X=0. The leaf body extends toward the center
    along +X for door_0 (sign=+1) and along -X for door_1 (sign=-1).
    Vertical extent is z in [LEAF_Z0, LEAF_Z0+LEAF_H]; thickness centered on Y=0.
    """
    inner = LEAF_W - CENTER_GAP / 2.0  # usable leaf width from hinge to meeting edge
    z0 = LEAF_Z0
    z1 = LEAF_Z0 + LEAF_H
    t = LEAF_T

    def x(u: float) -> float:
        # u in [0, inner] measured from the hinge edge toward the center.
        return sign * u

    leaf = cq.Workplane("XY")

    # Perimeter frame: outer rectangular iron border (hollow center).
    outer = (
        cq.Workplane("XY")
        .box(inner, t, LEAF_H, centered=(False, True, False))
        .translate((0 if sign > 0 else -inner, 0, z0))
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(inner - 2 * FRAME_W, t + 0.02, LEAF_H - 2 * FRAME_W, centered=(False, True, False))
        .translate((FRAME_W if sign > 0 else -inner + FRAME_W, 0, z0 + FRAME_W))
    )
    frame = outer.cut(inner_cut)
    leaf = leaf.union(frame)

    usable = inner - 2 * FRAME_W
    rail_w = FRAME_W * 0.75

    # Three horizontal rails carve the leaf into (bottom->top):
    #   * a short lower scroll band,
    #   * a tall picket panel filled with vertical bars + overlaid scrolls,
    #   * an upper scroll panel under the head rail.
    rail_lo_z = z0 + 0.46     # top of the lower scroll band
    rail_mid_z = z1 - 0.66    # top of the picket panel / bottom of upper panel
    for rz in (rail_lo_z, rail_mid_z):
        rail = (
            cq.Workplane("XY")
            .box(inner, t, rail_w, centered=(False, True, True))
            .translate((0 if sign > 0 else -inner, 0, rz))
        )
        leaf = leaf.union(rail)

    # Tall picket panel: closely spaced vertical bars with real air gaps.
    pk_z0 = rail_lo_z + rail_w / 2.0
    pk_z1 = rail_mid_z - rail_w / 2.0
    bar_zc = (pk_z0 + pk_z1) / 2.0
    bar_h = pk_z1 - pk_z0
    for i in range(N_BARS):
        u = FRAME_W + usable * (i + 0.5) / N_BARS
        bar = (
            cq.Workplane("XY")
            .box(BAR_W, t * 0.7, bar_h, centered=(True, True, True))
            .translate((x(u), 0, bar_zc))
        )
        leaf = leaf.union(bar)

    # Iron scrollwork is fused into the leaf so nothing floats; gold accents
    # (separate visual) sit on top of these same curls.
    leaf = leaf.union(_leaf_scroll_iron(sign, rail_lo_z, rail_mid_z, z0, z1))

    return leaf


def _scroll_panel_iron(cx: float, cz: float, half_w: float, half_h: float,
                       bar: float, depth: float, reach_h: float | None = None) -> cq.Workplane:
    """A symmetric cluster of mirrored volutes filling a rectangular panel
    centered at (cx, cz): four corner volutes curling toward the middle, a
    central vertical spine, a horizontal cross-tie, and a central oval boss.

    A horizontal cross-tie and a tall spine guarantee every volute is fused to
    the rest of the cluster, and the spine reaches `reach_h` (default the panel
    edges) so the cluster bonds to the surrounding iron rails / frame and never
    floats."""
    work = cq.Workplane("XY")
    sv = half_w * 0.55
    sh = half_h * 0.55
    if reach_h is None:
        reach_h = half_h * 2.05  # span to the rails bounding the panel
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            v = _volute(cx + sx * sv, cz + sz * sh, -sx, half_h / 0.16, bar, depth)
            work = work.union(v)
    # Vertical central spine reaching the panel-bounding rails/frame.
    spine = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .rect(bar, reach_h)
        .extrude(depth, both=True)
    )
    work = work.union(spine)
    # Horizontal cross-tie spanning the corner volutes so they are all bonded.
    tie = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .rect(2.0 * sv + 0.04, bar)
        .extrude(depth, both=True)
    )
    work = work.union(tie)
    # Two more cross-ties at the upper / lower volute rows for full bonding.
    for sz in (-1.0, 1.0):
        row_tie = (
            cq.Workplane("XZ")
            .center(cx, cz + sz * sh)
            .rect(2.0 * sv + 0.04, bar)
            .extrude(depth, both=True)
        )
        work = work.union(row_tie)
    # Central oval boss.
    boss = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .ellipse(half_w * 0.16, half_h * 0.30)
        .extrude(depth, both=True)
    )
    work = work.union(boss)
    return work


def _leaf_scroll_iron(sign: float, rail_lo_z: float, rail_mid_z: float,
                      z0: float, z1: float) -> cq.Workplane:
    """Dense iron C-scrolls / volutes that fill the upper and lower panels of
    the leaf (the picket panel keeps its bars and gets light scroll overlay)."""
    inner = LEAF_W - CENTER_GAP / 2.0
    t = LEAF_T
    usable = inner - 2 * FRAME_W
    depth = t * 0.55
    cx = (FRAME_W + usable * 0.5) if sign > 0 else -(FRAME_W + usable * 0.5)

    work = cq.Workplane("XY")

    # --- Upper scroll panel (under the head rail) ---
    up_top = z1 - FRAME_W
    up_zc = 0.5 * (rail_mid_z + up_top)
    up_hh = 0.5 * (up_top - rail_mid_z) * 0.9
    # spine reaches a touch past both bounding rails so the cluster bonds.
    up_reach = (up_top - rail_mid_z) + 0.06
    work = work.union(
        _scroll_panel_iron(cx, up_zc, usable * 0.5 * 0.92, up_hh, SCROLL_BAR, depth,
                           reach_h=up_reach)
    )

    # --- Lower scroll band ---
    lo_bot = z0 + FRAME_W
    lo_zc = 0.5 * (lo_bot + rail_lo_z)
    lo_hh = 0.5 * (rail_lo_z - lo_bot) * 0.92
    lo_reach = (rail_lo_z - lo_bot) + 0.06
    work = work.union(
        _scroll_panel_iron(cx, lo_zc, usable * 0.5 * 0.92, lo_hh, SCROLL_BAR, depth,
                           reach_h=lo_reach)
    )

    # --- Light scroll overlay tying the picket panel to the rails ---
    pk_zc = 0.5 * (rail_lo_z + rail_mid_z)
    for sz in (-1.0, 1.0):
        row_z = pk_zc + sz * (rail_mid_z - rail_lo_z) * 0.34
        # A thin horizontal tie crosses all pickets at this row, bonding the
        # overlay volutes to the picket panel.
        tie = (
            cq.Workplane("XZ")
            .center(cx, row_z)
            .rect(usable * 0.92, SCROLL_BAR * 0.8)
            .extrude(depth, both=True)
        )
        work = work.union(tie)
        for sx in (-1.0, 1.0):
            v = _volute(cx + sx * usable * 0.30, row_z, -sx, 0.85, SCROLL_BAR, depth)
            work = work.union(v)
    return work


def _gold_tip(cx: float, cz: float, r: float, proud: float) -> cq.Workplane:
    """A small gold disc/cap that sits proud of an iron volute eye."""
    return (
        cq.Workplane("XZ")
        .center(cx, cz)
        .circle(r)
        .extrude(proud, both=True)
    )


def _panel_gold(cx: float, cz: float, half_w: float, half_h: float,
                proud: float) -> cq.Workplane:
    """Gold accents for one scroll panel: a gold tip on each of the four
    volute eyes plus a gold boss at the panel center. Mirrors the geometry of
    `_scroll_panel_iron` so every gold piece lands on iron beneath it."""
    sv = half_w * 0.55
    sh = half_h * 0.55
    scale = half_h / 0.16
    work = _gold_tip(cx, cz, half_h * 0.20, proud)  # central boss
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            # eye of the volute (see _volute: eye at cx - s*0.028*scale)
            ex = (cx + sx * sv) - (-sx) * 0.028 * scale
            ez = cz + sz * sh
            work = work.union(_gold_tip(ex, ez, 0.018 * scale, proud))
    return work


def _leaf_scrolls(sign: float) -> cq.Workplane:
    """Gold tips/accents distributed across the leaf scrollwork: a gold cap on
    every volute eye and panel boss, plus gold caps on the picket heads.
    Returned as a separate (gold) visual fused onto the iron beneath."""
    inner = LEAF_W - CENTER_GAP / 2.0
    z0 = LEAF_Z0
    z1 = LEAF_Z0 + LEAF_H
    t = LEAF_T
    usable = inner - 2 * FRAME_W
    proud = t * 0.55  # match iron scroll proud depth so gold reads on the iron
    cx = (FRAME_W + usable * 0.5) if sign > 0 else -(FRAME_W + usable * 0.5)

    # Recompute the same rail layout the leaf uses.
    rail_lo_z = z0 + 0.46
    rail_mid_z = z1 - 0.66

    def x(u: float) -> float:
        return sign * u

    # Upper scroll panel gold.
    up_zc = 0.5 * (rail_mid_z + (z1 - FRAME_W))
    up_hh = 0.5 * ((z1 - FRAME_W) - rail_mid_z) * 0.9
    scrolls = _panel_gold(cx, up_zc, usable * 0.5 * 0.92, up_hh, proud)

    # Lower scroll band gold.
    lo_zc = 0.5 * ((z0 + FRAME_W) + rail_lo_z)
    lo_hh = 0.5 * (rail_lo_z - (z0 + FRAME_W)) * 0.92
    scrolls = scrolls.union(
        _panel_gold(cx, lo_zc, usable * 0.5 * 0.92, lo_hh, proud)
    )

    # Picket-overlay volute eyes (gold tips), matching _leaf_scroll_iron.
    pk_zc = 0.5 * (rail_lo_z + rail_mid_z)
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            vcx = cx + sx * usable * 0.30
            vcz = pk_zc + sz * (rail_mid_z - rail_lo_z) * 0.34
            ex = vcx - (-(-sx)) * 0.028 * 0.85  # eye of overlay volute
            scrolls = scrolls.union(_gold_tip(ex, vcz, 0.018 * 0.85, proud))

    # Gold cap rings on the picket heads at the mid rail.
    for i in range(N_BARS):
        u = FRAME_W + usable * (i + 0.5) / N_BARS
        cap = (
            cq.Workplane("XZ")
            .center(x(u), rail_mid_z)
            .circle(0.014)
            .extrude(proud, both=True)
        )
        scrolls = scrolls.union(cap)

    return scrolls


def _leaf_hinges(sign: float) -> cq.Workplane:
    """Three hinge knuckles (barrels) on the leaf hinge edge, around the
    vertical pin axis at local X=0. Authored as part of the leaf so they
    rotate with it; nudged toward the jamb so they read as mounted into the
    masonry pillar (captured-pin)."""
    knuckles = cq.Workplane("XY")
    # toward the jamb means away from the leaf body: -X for door_0 (sign=+1).
    out = -sign * 0.022
    zs = (LEAF_Z0 + 0.16, LEAF_Z0 + LEAF_H / 2.0, LEAF_Z0 + LEAF_H - 0.16)
    for hz in zs:
        # barrel
        kn = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(KNUCKLE_LEN)
            .translate((out, 0.0, hz - KNUCKLE_LEN / 2.0))
        )
        knuckles = knuckles.union(kn)
        # short connecting stub tying the barrel back to the leaf stile
        stub = (
            cq.Workplane("XY")
            .box(abs(out) + KNUCKLE_R + 0.02, KNUCKLE_R * 1.2, KNUCKLE_LEN * 0.6,
                 centered=(False, True, True))
            .translate((min(out, 0.0) if sign > 0 else 0.0, 0.0, hz))
        )
        # ensure stub reaches from barrel center to local x=0 (the frame edge)
        stub = (
            cq.Workplane("XY")
            .box(KNUCKLE_R + abs(out) + 0.03, KNUCKLE_R, KNUCKLE_LEN * 0.55,
                 centered=(True, True, True))
            .translate((out / 2.0, 0.0, hz))
        )
        knuckles = knuckles.union(stub)
    # vertical pin rod tying the three barrels together along the axis
    pin = (
        cq.Workplane("XY")
        .circle(PIN_R)
        .extrude(LEAF_H - 0.10)
        .translate((out, 0.0, LEAF_Z0 + 0.05))
    )
    knuckles = knuckles.union(pin)
    return knuckles


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wrought_iron_double_gate")
    _materials(model)

    # --- Root: plain rectangular stone surround + top rail ---
    surround = model.part("stone_surround")
    surround.visual(
        mesh_from_cadquery(_rectangular_surround(), "masonry"),
        material="stone",
        name="masonry",
    )
    surround.visual(
        mesh_from_cadquery(_plaster_reveal(), "plaster_reveal"),
        material="plaster",
        name="plaster_reveal",
    )
    surround.visual(
        mesh_from_cadquery(_threshold(), "threshold"),
        material="threshold",
        name="threshold",
    )
    surround.visual(
        mesh_from_cadquery(_top_rail(), "top_rail"),
        material="iron",
        name="top_rail",
    )

    # --- door_0: hinged at the LEFT jamb (-X side), opens toward -Y ---
    door_0 = model.part("door_0")
    door_0.visual(mesh_from_cadquery(_leaf_iron(+1.0), "door_0_iron"), material="iron", name="door_0_iron")
    door_0.visual(
        mesh_from_cadquery(_leaf_scrolls(+1.0), "door_0_scrolls"), material="gold", name="door_0_scrolls"
    )
    door_0.visual(
        mesh_from_cadquery(_leaf_hinges(+1.0), "door_0_knuckles"), material="iron", name="door_0_knuckles"
    )

    # --- door_1: hinged at the RIGHT jamb (+X side), mirrored ---
    door_1 = model.part("door_1")
    door_1.visual(mesh_from_cadquery(_leaf_iron(-1.0), "door_1_iron"), material="iron", name="door_1_iron")
    door_1.visual(
        mesh_from_cadquery(_leaf_scrolls(-1.0), "door_1_scrolls"), material="gold", name="door_1_scrolls"
    )
    door_1.visual(
        mesh_from_cadquery(_leaf_hinges(-1.0), "door_1_knuckles"), material="iron", name="door_1_knuckles"
    )

    # Hinge X positions: set in from the jamb inner faces by JAMB_REVEAL so the
    # closed leaf clears the masonry (a real reveal gap, no flush contact).
    hinge_x = OPENING_W / 2.0 - JAMB_REVEAL

    # door_0 leaf is authored extending +X from its local origin. To place its
    # hinge edge at the LEFT jamb (-X) and have it span toward the center,
    # the joint origin sits at x = -hinge_x. Positive q about +Z swings the
    # free (center) edge toward -Y (outward, opening).
    model.articulation(
        "surround_to_door_0",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=door_0,
        origin=Origin(xyz=(-hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.92),
    )

    # door_1 leaf is authored extending -X from its local origin; its hinge
    # edge sits at the RIGHT jamb (+X). Negate the axis so the SAME positive q
    # swings its free edge outward toward -Y as well. Mimic-couple to door_0.
    model.articulation(
        "surround_to_door_1",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=door_1,
        origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.92),
        mimic=Mimic(joint="surround_to_door_0", multiplier=1.0, offset=0.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    surround = object_model.get_part("stone_surround")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    j0 = object_model.get_articulation("surround_to_door_0")
    j1 = object_model.get_articulation("surround_to_door_1")

    # Hinge knuckles capture the jamb pillar edges; allow that small embed.
    ctx.allow_overlap(
        door_0,
        surround,
        elem_a="door_0_knuckles",
        elem_b="masonry",
        reason="Hinge knuckles intentionally embed into the jamb pillar (captured-pin mount).",
    )
    ctx.allow_overlap(
        door_1,
        surround,
        elem_a="door_1_knuckles",
        elem_b="masonry",
        reason="Hinge knuckles intentionally embed into the jamb pillar (captured-pin mount).",
    )

    # --- Closed pose: both leaves present, upright, span the opening ---
    aabb0 = ctx.part_world_aabb(door_0)
    aabb1 = ctx.part_world_aabb(door_1)
    if not ctx.check(
        "both leaves resolve to a world AABB",
        aabb0 is not None and aabb1 is not None,
    ):
        return ctx.report()

    # Each leaf is a tall upright panel (height >> thickness).
    h0 = aabb0[1][2] - aabb0[0][2]
    h1 = aabb1[1][2] - aabb1[0][2]
    ctx.check("door_0 is a tall upright leaf", h0 > 2.0, details=f"height={h0:.3f}")
    ctx.check("door_1 is a tall upright leaf", h1 > 2.0, details=f"height={h1:.3f}")

    # Leaves stand on the threshold, not below the floor / not floating.
    ctx.check(
        "door_0 stands on the ground plane",
        abs(aabb0[0][2] - LEAF_Z0) < 0.02,
        details=f"z_min={aabb0[0][2]:.3f}",
    )
    ctx.check(
        "door_1 stands on the ground plane",
        abs(aabb1[0][2] - LEAF_Z0) < 0.02,
        details=f"z_min={aabb1[0][2]:.3f}",
    )

    # Closed: the leaves are mirror images about the center plane (x ~ 0).
    cx0 = 0.5 * (aabb0[0][0] + aabb0[1][0])
    cx1 = 0.5 * (aabb1[0][0] + aabb1[1][0])
    ctx.check(
        "closed leaves are symmetric about the center plane",
        abs(cx0 + cx1) < 0.03 and cx0 < 0.0 < cx1,
        details=f"center_x door_0={cx0:.3f}, door_1={cx1:.3f}",
    )
    # door_0 occupies the -X half, door_1 the +X half when closed.
    ctx.check(
        "door_0 spans the left half",
        aabb0[0][0] < -0.5 and aabb0[1][0] > -0.06,
        details=f"x=[{aabb0[0][0]:.3f},{aabb0[1][0]:.3f}]",
    )
    ctx.check(
        "door_1 spans the right half",
        aabb1[1][0] > 0.5 and aabb1[0][0] < 0.06,
        details=f"x=[{aabb1[0][0]:.3f},{aabb1[1][0]:.3f}]",
    )
    # The two closed leaves meet near the center (small reveal gap, no big gap).
    ctx.expect_gap(
        door_1, door_0, axis="x", min_gap=-0.01, max_gap=0.09,
        name="closed leaves meet at the center with a small reveal",
    )

    # --- Rectangular surround: flat lintel, no arch ---
    masonry_aabb = ctx.part_element_world_aabb(surround, elem="masonry")
    if ctx.check("masonry resolves to an AABB", masonry_aabb is not None):
        masonry_h = masonry_aabb[1][2] - masonry_aabb[0][2]
        # The surround extends above the leaf tops (lintel header present).
        ctx.check(
            "surround has a lintel above the leaves",
            masonry_aabb[1][2] > LEAF_Z0 + LEAF_H + 0.10,
            details=f"masonry z_max={masonry_aabb[1][2]:.3f}, leaf top={LEAF_Z0 + LEAF_H:.3f}",
        )
        # The surround is a rectangular block: its top is flat (no curved arch
        # protrusion). The top surface should be close to SURROUND_H.
        ctx.check(
            "surround top is a flat lintel (no arch crown)",
            abs(masonry_aabb[1][2] - SURROUND_H) < 0.02,
            details=f"masonry z_max={masonry_aabb[1][2]:.3f}, expected={SURROUND_H:.3f}",
        )
        # The surround spans the full opening plus both pillars.
        masonry_w = masonry_aabb[1][0] - masonry_aabb[0][0]
        expected_w = OPENING_W + 2 * PILLAR_W
        ctx.check(
            "surround spans opening plus both pillars",
            abs(masonry_w - expected_w) < 0.05,
            details=f"masonry width={masonry_w:.3f}, expected={expected_w:.3f}",
        )

    # --- Top rail: flat horizontal iron rail across the opening head ---
    rail_aabb = ctx.part_element_world_aabb(surround, elem="top_rail")
    if ctx.check("top rail resolves to an AABB", rail_aabb is not None):
        # The top rail sits at the head of the opening, below the lintel.
        rail_z_max = rail_aabb[1][2]
        rail_z_min = rail_aabb[0][2]
        ctx.check(
            "top rail is at the opening head (below lintel, above leaves)",
            rail_z_max > LEAF_Z0 + LEAF_H - 0.10
            and rail_z_min < PILLAR_H + 0.01,
            details=f"rail z=[{rail_z_min:.3f},{rail_z_max:.3f}], leaf top={LEAF_Z0 + LEAF_H:.3f}",
        )
        # Top rail spans the full opening width.
        rail_w = rail_aabb[1][0] - rail_aabb[0][0]
        ctx.check(
            "top rail spans the opening width",
            abs(rail_w - OPENING_W) < 0.05,
            details=f"rail width={rail_w:.3f}, opening={OPENING_W:.3f}",
        )
        # Top rail is centered on the opening.
        rail_cx = 0.5 * (rail_aabb[0][0] + rail_aabb[1][0])
        ctx.check(
            "top rail is centered on the opening",
            abs(rail_cx) < 0.02,
            details=f"rail center_x={rail_cx:.3f}",
        )
        # Top rail contacts the masonry surround (mounted into the pillars).
        ctx.expect_contact(
            surround, surround, elem_a="top_rail", elem_b="masonry",
            contact_tol=0.02, name="top rail meets the stone surround",
        )

    # --- Open pose: positive q swings both free (center) edges outward (-Y) ---
    closed_y0 = aabb0[0][1]  # min Y when closed
    closed_y1 = aabb1[0][1]
    with ctx.pose({j0: 1.5}):  # j1 follows via mimic
        open0 = ctx.part_world_aabb(door_0)
        open1 = ctx.part_world_aabb(door_1)
        if ctx.check(
            "open-pose AABBs resolve",
            open0 is not None and open1 is not None,
        ):
            ctx.check(
                "door_0 swings outward (-Y) when opened",
                open0[0][1] < closed_y0 - 0.30,
                details=f"closed_minY={closed_y0:.3f}, open_minY={open0[0][1]:.3f}",
            )
            ctx.check(
                "door_1 swings outward (-Y) when opened",
                open1[0][1] < closed_y1 - 0.30,
                details=f"closed_minY={closed_y1:.3f}, open_minY={open1[0][1]:.3f}",
            )
            # Opened leaves clear the center passage.
            ctx.check(
                "leaves open the central passage",
                open0[1][0] < 0.35 and open1[0][0] > -0.35,
                details=f"d0 maxX={open0[1][0]:.3f}, d1 minX={open1[0][0]:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
