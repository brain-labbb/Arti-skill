from __future__ import annotations

# Ornate wrought-iron single-swing entrance gate in an arched stone surround.
#
# Articraft brief
# - Object: ornate wrought-iron single-swing entrance gate set in an arched
#   stone / plaster surround with a fixed decorative arched fanlight above.
#   Overall opening ~2.2 m wide x ~3.4 m tall; one wide leaf ~2.14 m x ~2.18 m.
# - Frame: world up is +Z. The assembly stands on the ground plane (z>=0).
#   Gate width runs along X (left<->right), leaf thickness along Y
#   (front<->back), height along Z. The leaf swings outward toward -Y.
# - Root/support: `stone_surround` = the two masonry jamb pillars, the
#   semicircular arch, the threshold sill, and the FIXED fanlight ironwork.
# - Parts: stone_surround (root), gate_leaf (single wide iron leaf:
#   perimeter frame, vertical bars, scroll panels, gold accents, latch stile).
# - Articulations: surround_to_leaf, REVOLUTE on the VERTICAL (Z) axis at the
#   left jamb. The leaf extends +X from its hinge edge toward the right jamb.
#   Positive q swings the free (latch) edge outward toward -Y.
# - Visible geometry: hollow barred iron leaf (gaps between vertical bars),
#   gold scrollwork rosettes, latch stile with handle at the free edge,
#   stone arch with a hollow fanlight filled by an ornate iron transom.
# - Intentional overlaps: hinge knuckles embed slightly into the jamb pillar
#   (captured-pin pattern); scoped allowance only.
# - Tests: single leaf present, closed leaf spans the full opening and seats
#   against the latch jamb, open pose swings the free edge outward (-Y),
#   fanlight grille is mounted, latch stile reaches the opposite jamb.

import math

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

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------

OPENING_W = 2.22         # clear opening width between jamb inner faces
LEAF_W = 2.14            # single leaf width (X): spans opening with clearance
LEAF_H = 2.18            # leaf height (Z)
LEAF_T = 0.055           # leaf frame depth (Y)
JAMB_REVEAL = 0.03       # reveal gap between the leaf hinge edge and the jamb
LATCH_CLEARANCE = 0.02   # gap between latch stile and opposite jamb
SILL_Z = 0.04            # threshold sill thickness; leaf starts just above it
LEAF_Z0 = SILL_Z         # bottom of leaf
LEAF_ZC = LEAF_Z0 + LEAF_H / 2.0  # leaf vertical center

PILLAR_W = 0.30          # jamb pillar width (X)
PILLAR_D = 0.34          # jamb pillar / wall depth (Y)
PILLAR_H = LEAF_Z0 + LEAF_H + 0.02  # pillar height up to the spring line

FRAME_W = 0.075          # leaf stile / rail bar width
BAR_W = 0.020            # vertical picket bar (square cross-section)
N_BARS = 14              # vertical pickets across the full leaf
SCROLL_BAR = 0.016       # iron scroll-bar cross-section width
LATCH_STILE_W = 0.060    # latch stile width (wider than a normal stile)

# Hinge geometry
KNUCKLE_R = 0.030
KNUCKLE_LEN = 0.10
PIN_R = 0.012


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
# Stone surround + fixed fanlight (root)
# ---------------------------------------------------------------------------


def _stone_masonry() -> cq.Workplane:
    """Two jamb pillars + the solid masonry arch ring, with the arched
    opening (door + fanlight) cut out as a real hollow.

    The facade lies in the XZ plane; the wall depth runs along Y (thickness
    PILLAR_D, centered on Y=0). The arch is a true semicircular ring.
    """
    half_open = OPENING_W / 2.0
    arch_inner_r = half_open
    arch_outer_r = half_open + PILLAR_W
    spring_z = PILLAR_H

    # Full facade slab from ground to the top of the arch, depth along Y.
    slab_h = spring_z + arch_outer_r
    wall = (
        cq.Workplane("XZ")
        .box(OPENING_W + 2 * PILLAR_W, slab_h, PILLAR_D, centered=(True, False, True))
    )

    # Cut the rectangular doorway/fanlight opening (below spring line).
    door_cut = (
        cq.Workplane("XZ")
        .workplane()
        .center(0.0, spring_z / 2.0)
        .box(OPENING_W, spring_z, PILLAR_D + 0.02, centered=(True, True, True))
    )
    wall = wall.cut(door_cut)

    # Cut the semicircular arched opening above the spring line.
    arch_cut = (
        cq.Workplane("XZ")
        .workplane()
        .center(0.0, spring_z)
        .circle(arch_inner_r)
        .extrude(PILLAR_D + 0.02, both=True)
    )
    wall = wall.cut(arch_cut)

    # Trim any masonry that grew above the outer arch crown to a clean ring.
    crown_cut = (
        cq.Workplane("XZ")
        .workplane()
        .center(0.0, spring_z)
        .circle(arch_outer_r)
        .extrude(PILLAR_D + 0.04, both=True)
    )
    top_band = (
        cq.Workplane("XZ")
        .workplane()
        .center(0.0, spring_z + arch_outer_r / 2.0)
        .box(OPENING_W + 2 * PILLAR_W + 0.1, arch_outer_r, PILLAR_D + 0.1, centered=(True, True, True))
    )
    arched_top = top_band.intersect(crown_cut)
    lower_part = (
        cq.Workplane("XZ")
        .workplane()
        .center(0.0, spring_z / 2.0)
        .box(OPENING_W + 2 * PILLAR_W + 0.1, spring_z, PILLAR_D + 0.1, centered=(True, True, True))
    )
    wall = wall.intersect(lower_part.union(arched_top))

    return wall


# Fanlight transom layout (shared by the iron grille and its gold accents).
FAN_BAR = 0.022
FAN_DEPTH = 0.045
FAN_BAND_IN = 0.86
FAN_OVAL_C = 0.45
FAN_OVAL_RX = 0.135
FAN_OVAL_RZ = 0.265
FAN_PED_Z = 0.66
FAN_BOWL_X = 0.20
FAN_VOL_BIG = (0.45, 0.42, 2.2)
FAN_VOL_MID = (0.72, 0.18, 1.6)
FAN_VOL_CRN = (0.88, 0.10, 1.0)
FAN_ELL_POS = (0.38, 0.16)


def _fanlight_grille() -> cq.Workplane:
    """Fixed decorative ironwork filling the semicircular fanlight above the
    spring line."""
    spring_z = PILLAR_H
    r = OPENING_W / 2.0 + 0.008
    bar = FAN_BAR
    depth = FAN_DEPTH
    cy = spring_z

    def ring(ro: float, ri: float, h: float) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .workplane()
            .center(0.0, cy)
            .circle(ro)
            .circle(ri)
            .extrude(h, both=True)
        )

    rim = ring(r, r - bar, depth / 2.0)
    upper = (
        cq.Workplane("XZ")
        .workplane()
        .center(0.0, cy + r / 2.0)
        .box(2 * r + 0.2, r, depth + 0.1, centered=(True, True, True))
    )
    grille = rim.intersect(upper)

    tie = (
        cq.Workplane("XZ")
        .workplane()
        .center(0.0, cy + bar / 2.0)
        .box(2 * r, bar, depth, centered=(True, True, True))
    )
    grille = grille.union(tie)

    band_in = r * FAN_BAND_IN
    inner_rail = ring(band_in + bar * 0.8, band_in, depth / 2.0).intersect(upper)
    grille = grille.union(inner_rail)

    n_bal = 15
    bal_len = (r - bar * 0.5) - band_in
    for i in range(n_bal):
        theta = -math.pi / 2.0 + math.pi * (i + 0.5) / n_bal
        baluster = (
            cq.Workplane("XY")
            .box(bar * 0.7, depth, bal_len, centered=(True, True, False))
            .translate((0, 0, band_in))
            .rotate((0, 0, 0), (0, 1, 0), math.degrees(theta))
            .translate((0, 0, cy))
        )
        grille = grille.union(baluster)

    r_mid = (band_in + r - bar) / 2.0
    ring_ro = (r - bar - band_in) / 2.0 + 0.004
    ring_ri = ring_ro - bar * 0.8
    for i in range(n_bal - 1):
        ang = math.pi * (i + 1) / n_bal
        circ = (
            cq.Workplane("XZ")
            .center(r_mid * math.cos(ang), cy + r_mid * math.sin(ang))
            .circle(ring_ro)
            .circle(ring_ri)
            .extrude(depth / 2.0, both=True)
        )
        grille = grille.union(circ)

    orn = cq.Workplane("XY")

    spine = (
        cq.Workplane("XZ")
        .center(0.0, cy + (band_in + 0.02) / 2.0)
        .rect(bar, band_in + 0.02)
        .extrude(depth / 2.0, both=True)
    )
    orn = orn.union(spine)

    oval = (
        cq.Workplane("XZ")
        .center(0.0, cy + FAN_OVAL_C)
        .ellipse(FAN_OVAL_RX, FAN_OVAL_RZ)
        .extrude(depth / 2.0, both=True)
        .cut(
            cq.Workplane("XZ")
            .center(0.0, cy + FAN_OVAL_C)
            .ellipse(FAN_OVAL_RX - bar, FAN_OVAL_RZ - bar)
            .extrude(depth, both=True)
        )
    )
    orn = orn.union(oval)

    ped = (
        cq.Workplane("XZ")
        .center(0.0, cy + FAN_PED_Z)
        .rect(0.64, bar)
        .extrude(depth / 2.0, both=True)
    )
    orn = orn.union(ped)
    for sx in (-1.0, 1.0):
        post = (
            cq.Workplane("XZ")
            .center(sx * 0.30, cy + (0.50 + FAN_PED_Z) / 2.0)
            .rect(bar, FAN_PED_Z - 0.50 + bar)
            .extrude(depth / 2.0, both=True)
        )
        orn = orn.union(post)

    for sx in (-1.0, 1.0):
        for vx, vz, scale in (FAN_VOL_BIG, FAN_VOL_MID, FAN_VOL_CRN):
            orn = orn.union(
                _volute(sx * vx, cy + vz, -sx, scale, bar * 0.9, depth / 2.0)
            )
        ell = (
            cq.Workplane("XZ")
            .center(sx * FAN_ELL_POS[0], cy + FAN_ELL_POS[1])
            .ellipse(0.14, 0.068)
            .extrude(depth / 2.0, both=True)
        )
        orn = orn.union(ell)

    tie_hi = (
        cq.Workplane("XZ")
        .center(0.0, cy + FAN_VOL_BIG[1])
        .rect(2 * (FAN_VOL_BIG[0] + 0.19), bar)
        .extrude(depth / 2.0, both=True)
    )
    tie_lo = (
        cq.Workplane("XZ")
        .center(0.0, cy + FAN_ELL_POS[1])
        .rect(2 * 0.86, bar)
        .extrude(depth / 2.0, both=True)
    )
    orn = orn.union(tie_hi).union(tie_lo)

    orn = orn.intersect(upper).intersect(
        cq.Workplane("XZ")
        .workplane()
        .center(0.0, cy)
        .circle(band_in + 0.012)
        .extrude(depth, both=True)
    )
    grille = grille.union(orn)

    return grille


def _fanlight_gold() -> cq.Workplane:
    """Gold accents of the fanlight, sitting proud of the iron grille."""
    cy = PILLAR_H
    proud = FAN_DEPTH / 2.0 + 0.008

    def solid(cx: float, cz: float, rx: float, rz: float) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .center(cx, cy + cz)
            .ellipse(rx, rz)
            .extrude(proud, both=True)
        )

    gold = solid(0.0, FAN_OVAL_C, FAN_OVAL_RX - 0.018, FAN_OVAL_RZ - 0.018)

    for sx in (-1.0, 1.0):
        gold = gold.union(solid(sx * FAN_BOWL_X, FAN_PED_Z + 0.045, 0.075, 0.050))
        gold = gold.union(
            solid(sx * FAN_ELL_POS[0], FAN_ELL_POS[1], 0.115, 0.052)
        )
        for vx, vz, scale in (FAN_VOL_BIG, FAN_VOL_MID, FAN_VOL_CRN):
            ex = sx * vx + sx * 0.028 * scale
            gold = gold.union(solid(ex, vz, 0.018 * scale, 0.018 * scale))

    return gold


def _threshold() -> cq.Workplane:
    """Dark stone threshold sill the leaf stands on."""
    return (
        cq.Workplane("XZ")
        .box(OPENING_W + 0.04, SILL_Z, PILLAR_D, centered=(True, False, True))
    )


def _plaster_reveal() -> cq.Workplane:
    """Cream plaster inner reveal band lining the rectangular doorway sides."""
    spring_z = PILLAR_H
    t = 0.04
    band = 0.05
    half = OPENING_W / 2.0
    rev = cq.Workplane("XY")
    for sx in (-1.0, 1.0):
        col = (
            cq.Workplane("XY")
            .box(band, t, spring_z - SILL_Z, centered=(True, True, False))
            .translate((sx * (half - band / 2.0), -PILLAR_D / 2.0 + t / 2.0, SILL_Z))
        )
        rev = rev.union(col)
    return rev


# ---------------------------------------------------------------------------
# Scrollwork primitives
# ---------------------------------------------------------------------------


def _c_arc(cx: float, cz: float, r_out: float, bar: float, depth: float,
           a0_deg: float, a1_deg: float) -> cq.Workplane:
    """A curved iron bar: an angular slice of a thin annulus in the XZ plane."""
    ro = r_out
    ri = max(r_out - bar, 0.001)
    ring = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .circle(ro)
        .circle(ri)
        .extrude(depth, both=True)
    )
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
    """A spiral volute: two nested C-arcs plus a solid eye."""
    s = sign_in
    body = _c_arc(cx, cz, 0.085 * scale, bar, depth, 0, 250)
    ecx = cx - s * 0.028 * scale
    inner = _c_arc(ecx, cz, 0.050 * scale, bar, depth, 60, 320)
    eye = (
        cq.Workplane("XZ")
        .center(ecx, cz)
        .circle(0.018 * scale)
        .extrude(depth, both=True)
    )
    bridge = (
        cq.Workplane("XZ")
        .center(0.5 * (cx + ecx), cz)
        .rect(abs(cx - ecx) + 0.085 * scale, bar)
        .extrude(depth, both=True)
    )
    return body.union(inner).union(eye).union(bridge)


# ---------------------------------------------------------------------------
# Iron gate leaf (single wide leaf, hinged at the left jamb)
# ---------------------------------------------------------------------------


def _leaf_iron() -> cq.Workplane:
    """Black iron leaf authored in its own local frame.

    The hinge edge is at local X=0. The leaf body extends toward +X across
    the full opening. Vertical extent is z in [LEAF_Z0, LEAF_Z0+LEAF_H];
    thickness centered on Y=0.
    """
    w = LEAF_W
    z0 = LEAF_Z0
    z1 = LEAF_Z0 + LEAF_H
    t = LEAF_T

    leaf = cq.Workplane("XY")

    # Perimeter frame: outer rectangular iron border (hollow center).
    outer = (
        cq.Workplane("XY")
        .box(w, t, LEAF_H, centered=(False, True, False))
        .translate((0, 0, z0))
    )
    inner_cut = (
        cq.Workplane("XY")
        .box(w - 2 * FRAME_W, t + 0.02, LEAF_H - 2 * FRAME_W,
             centered=(False, True, False))
        .translate((FRAME_W, 0, z0 + FRAME_W))
    )
    frame = outer.cut(inner_cut)
    leaf = leaf.union(frame)

    usable = w - 2 * FRAME_W - LATCH_STILE_W  # usable width minus latch stile
    rail_w = FRAME_W * 0.75

    # Three horizontal rails carve the leaf into panels.
    rail_lo_z = z0 + 0.46
    rail_mid_z = z1 - 0.66
    for rz in (rail_lo_z, rail_mid_z):
        rail = (
            cq.Workplane("XY")
            .box(w, t, rail_w, centered=(False, True, True))
            .translate((0, 0, rz))
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
            .translate((u, 0, bar_zc))
        )
        leaf = leaf.union(bar)

    # Iron scrollwork fused into the leaf.
    leaf = leaf.union(_leaf_scroll_iron(rail_lo_z, rail_mid_z, z0, z1))

    # Latch stile: a solid vertical bar at the free edge (latch side).
    latch_stile = (
        cq.Workplane("XY")
        .box(LATCH_STILE_W, t, LEAF_H, centered=(False, True, False))
        .translate((w - LATCH_STILE_W, 0, z0))
    )
    leaf = leaf.union(latch_stile)

    return leaf


def _scroll_panel_iron(cx: float, cz: float, half_w: float, half_h: float,
                       bar: float, depth: float, reach_h: float | None = None) -> cq.Workplane:
    """A symmetric cluster of mirrored volutes filling a rectangular panel."""
    work = cq.Workplane("XY")
    sv = half_w * 0.55
    sh = half_h * 0.55
    if reach_h is None:
        reach_h = half_h * 2.05
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            v = _volute(cx + sx * sv, cz + sz * sh, -sx, half_h / 0.16, bar, depth)
            work = work.union(v)
    spine = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .rect(bar, reach_h)
        .extrude(depth, both=True)
    )
    work = work.union(spine)
    tie = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .rect(2.0 * sv + 0.04, bar)
        .extrude(depth, both=True)
    )
    work = work.union(tie)
    for sz in (-1.0, 1.0):
        row_tie = (
            cq.Workplane("XZ")
            .center(cx, cz + sz * sh)
            .rect(2.0 * sv + 0.04, bar)
            .extrude(depth, both=True)
        )
        work = work.union(row_tie)
    boss = (
        cq.Workplane("XZ")
        .center(cx, cz)
        .ellipse(half_w * 0.16, half_h * 0.30)
        .extrude(depth, both=True)
    )
    work = work.union(boss)
    return work


def _leaf_scroll_iron(rail_lo_z: float, rail_mid_z: float,
                      z0: float, z1: float) -> cq.Workplane:
    """Dense iron C-scrolls / volutes that fill the upper and lower panels."""
    t = LEAF_T
    usable = LEAF_W - 2 * FRAME_W - LATCH_STILE_W
    depth = t * 0.55
    cx = FRAME_W + usable * 0.5

    work = cq.Workplane("XY")

    # Upper scroll panel
    up_top = z1 - FRAME_W
    up_zc = 0.5 * (rail_mid_z + up_top)
    up_hh = 0.5 * (up_top - rail_mid_z) * 0.9
    up_reach = (up_top - rail_mid_z) + 0.06
    work = work.union(
        _scroll_panel_iron(cx, up_zc, usable * 0.5 * 0.92, up_hh, SCROLL_BAR, depth,
                           reach_h=up_reach)
    )

    # Lower scroll band
    lo_bot = z0 + FRAME_W
    lo_zc = 0.5 * (lo_bot + rail_lo_z)
    lo_hh = 0.5 * (rail_lo_z - lo_bot) * 0.92
    lo_reach = (rail_lo_z - lo_bot) + 0.06
    work = work.union(
        _scroll_panel_iron(cx, lo_zc, usable * 0.5 * 0.92, lo_hh, SCROLL_BAR, depth,
                           reach_h=lo_reach)
    )

    # Light scroll overlay tying the picket panel to the rails
    pk_zc = 0.5 * (rail_lo_z + rail_mid_z)
    for sz in (-1.0, 1.0):
        row_z = pk_zc + sz * (rail_mid_z - rail_lo_z) * 0.34
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
    """Gold accents for one scroll panel."""
    sv = half_w * 0.55
    sh = half_h * 0.55
    scale = half_h / 0.16
    work = _gold_tip(cx, cz, half_h * 0.20, proud)
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            ex = (cx + sx * sv) - (-sx) * 0.028 * scale
            ez = cz + sz * sh
            work = work.union(_gold_tip(ex, ez, 0.018 * scale, proud))
    return work


def _leaf_scrolls() -> cq.Workplane:
    """Gold tips/accents distributed across the leaf scrollwork."""
    z0 = LEAF_Z0
    z1 = LEAF_Z0 + LEAF_H
    t = LEAF_T
    usable = LEAF_W - 2 * FRAME_W - LATCH_STILE_W
    proud = t * 0.55
    cx = FRAME_W + usable * 0.5

    rail_lo_z = z0 + 0.46
    rail_mid_z = z1 - 0.66

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

    # Picket-overlay volute eyes (gold tips).
    pk_zc = 0.5 * (rail_lo_z + rail_mid_z)
    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            vcx = cx + sx * usable * 0.30
            vcz = pk_zc + sz * (rail_mid_z - rail_lo_z) * 0.34
            ex = vcx - (-(-sx)) * 0.028 * 0.85
            scrolls = scrolls.union(_gold_tip(ex, vcz, 0.018 * 0.85, proud))

    # Gold cap rings on the picket heads at the mid rail.
    for i in range(N_BARS):
        u = FRAME_W + usable * (i + 0.5) / N_BARS
        cap = (
            cq.Workplane("XZ")
            .center(u, rail_mid_z)
            .circle(0.014)
            .extrude(proud, both=True)
        )
        scrolls = scrolls.union(cap)

    # Gold accent on the latch handle.
    latch_handle_gold = (
        cq.Workplane("XZ")
        .center(LEAF_W - LATCH_STILE_W / 2.0, LEAF_ZC)
        .circle(0.022)
        .extrude(proud, both=True)
    )
    scrolls = scrolls.union(latch_handle_gold)

    return scrolls


def _leaf_hinges() -> cq.Workplane:
    """Three hinge knuckles (barrels) on the leaf hinge edge, around the
    vertical pin axis at local X=0."""
    knuckles = cq.Workplane("XY")
    out = -0.022  # toward the jamb (negative X direction)
    zs = (LEAF_Z0 + 0.16, LEAF_Z0 + LEAF_H / 2.0, LEAF_Z0 + LEAF_H - 0.16)
    for hz in zs:
        kn = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(KNUCKLE_LEN)
            .translate((out, 0.0, hz - KNUCKLE_LEN / 2.0))
        )
        knuckles = knuckles.union(kn)
        stub = (
            cq.Workplane("XY")
            .box(KNUCKLE_R + abs(out) + 0.03, KNUCKLE_R, KNUCKLE_LEN * 0.55,
                 centered=(True, True, True))
            .translate((out / 2.0, 0.0, hz))
        )
        knuckles = knuckles.union(stub)
    pin = (
        cq.Workplane("XY")
        .circle(PIN_R)
        .extrude(LEAF_H - 0.10)
        .translate((out, 0.0, LEAF_Z0 + 0.05))
    )
    knuckles = knuckles.union(pin)
    return knuckles


def _latch_handle() -> cq.Workplane:
    """A latch handle on the free (latch) edge of the leaf: a horizontal
    iron bar with a gold-tipped end, mounted on the latch stile."""
    t = LEAF_T
    handle_x = LEAF_W - LATCH_STILE_W / 2.0
    handle_z = LEAF_ZC
    proud = t / 2.0 + 0.02

    # Horizontal bar extending from the latch stile toward -Y (outward).
    bar = (
        cq.Workplane("XY")
        .box(0.035, 0.14, 0.035, centered=(True, False, True))
        .translate((handle_x, -proud, handle_z))
    )
    # Mounting plate on the stile face.
    plate = (
        cq.Workplane("XY")
        .box(0.06, 0.015, 0.08, centered=(True, True, True))
        .translate((handle_x, t / 2.0 + 0.007, handle_z))
    )
    handle = bar.union(plate)

    # Connecting stub from plate through the stile to ensure connectivity.
    stub = (
        cq.Workplane("XY")
        .box(0.04, t + 0.03, 0.04, centered=(True, True, True))
        .translate((handle_x, 0, handle_z))
    )
    handle = handle.union(stub)

    return handle


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wrought_iron_single_gate")
    _materials(model)

    # --- Root: stone surround + fixed fanlight ---
    surround = model.part("stone_surround")
    surround.visual(mesh_from_cadquery(_stone_masonry(), "masonry"), material="stone", name="masonry")
    surround.visual(
        mesh_from_cadquery(_plaster_reveal(), "plaster_reveal"),
        material="plaster",
        name="plaster_reveal",
    )
    surround.visual(
        mesh_from_cadquery(_threshold(), "threshold"), material="threshold", name="threshold"
    )
    surround.visual(
        mesh_from_cadquery(_fanlight_grille(), "fanlight_grille"),
        material="iron",
        name="fanlight_grille",
    )
    surround.visual(
        mesh_from_cadquery(_fanlight_gold(), "fanlight_gold"),
        material="gold",
        name="fanlight_gold",
    )

    # --- gate_leaf: single wide leaf hinged at the LEFT jamb (-X side) ---
    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        mesh_from_cadquery(_leaf_iron(), "leaf_iron"),
        material="iron",
        name="leaf_iron",
    )
    gate_leaf.visual(
        mesh_from_cadquery(_leaf_scrolls(), "leaf_scrolls"),
        material="gold",
        name="leaf_scrolls",
    )
    gate_leaf.visual(
        mesh_from_cadquery(_leaf_hinges(), "leaf_knuckles"),
        material="iron",
        name="leaf_knuckles",
    )
    gate_leaf.visual(
        mesh_from_cadquery(_latch_handle(), "latch_handle"),
        material="iron",
        name="latch_handle",
    )

    # Hinge position: set in from the left jamb inner face by JAMB_REVEAL.
    hinge_x = OPENING_W / 2.0 - JAMB_REVEAL

    # The leaf is authored extending +X from its local origin (hinge edge).
    # Joint origin sits at x = -hinge_x (left jamb). Positive q about -Z
    # swings the free (latch) edge toward -Y (outward, opening).
    model.articulation(
        "surround_to_leaf",
        ArticulationType.REVOLUTE,
        parent=surround,
        child=gate_leaf,
        origin=Origin(xyz=(-hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=0.0, upper=1.92),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    surround = object_model.get_part("stone_surround")
    gate_leaf = object_model.get_part("gate_leaf")
    hinge = object_model.get_articulation("surround_to_leaf")

    # Hinge knuckles capture the jamb pillar edge; allow that small embed.
    ctx.allow_overlap(
        gate_leaf,
        surround,
        elem_a="leaf_knuckles",
        elem_b="masonry",
        reason="Hinge knuckles intentionally embed into the jamb pillar (captured-pin mount).",
    )

    # --- Closed pose: leaf spans the full opening ---
    aabb = ctx.part_world_aabb(gate_leaf)
    if not ctx.check(
        "gate leaf resolves to a world AABB",
        aabb is not None,
    ):
        return ctx.report()

    # The leaf is a tall upright panel (height >> thickness).
    leaf_h = aabb[1][2] - aabb[0][2]
    ctx.check("gate leaf is a tall upright panel", leaf_h > 2.0, details=f"height={leaf_h:.3f}")

    # Leaf stands on the threshold.
    ctx.check(
        "gate leaf stands on the ground plane",
        abs(aabb[0][2] - LEAF_Z0) < 0.02,
        details=f"z_min={aabb[0][2]:.3f}",
    )

    # Single leaf spans the full opening width (from left jamb area to right jamb area).
    leaf_width = aabb[1][0] - aabb[0][0]
    ctx.check(
        "gate leaf spans the full opening width",
        leaf_width > OPENING_W * 0.85,
        details=f"leaf_width={leaf_width:.3f}, opening={OPENING_W:.3f}",
    )

    # Closed leaf: hinge edge near the left jamb, latch edge near the right jamb.
    ctx.check(
        "closed leaf hinge edge is near the left jamb",
        aabb[0][0] < -OPENING_W / 2.0 + 0.10,
        details=f"x_min={aabb[0][0]:.3f}",
    )
    ctx.check(
        "closed leaf latch edge reaches the right jamb",
        aabb[1][0] > OPENING_W / 2.0 - 0.15,
        details=f"x_max={aabb[1][0]:.3f}",
    )

    # The latch stile meets the opposite jamb with a small clearance gap.
    latch_jamb_x = OPENING_W / 2.0
    ctx.check(
        "latch stile is within reach of the right jamb",
        aabb[1][0] > latch_jamb_x - 0.10,
        details=f"leaf_x_max={aabb[1][0]:.3f}, jamb_x={latch_jamb_x:.3f}",
    )

    # Verify the latch handle visual exists on the leaf.
    latch_aabb = ctx.part_element_world_aabb(gate_leaf, elem="latch_handle")
    ctx.check(
        "latch handle visual exists on the gate leaf",
        latch_aabb is not None,
    )
    if latch_aabb is not None:
        # Latch handle is at the free (right) edge of the leaf.
        ctx.check(
            "latch handle is at the free edge of the leaf",
            latch_aabb[0][0] > aabb[0][0] + LEAF_W * 0.7,
            details=f"handle_x_min={latch_aabb[0][0]:.3f}, leaf_x_min={aabb[0][0]:.3f}",
        )

    # The decorative fanlight grille is fixed and sits above the leaf.
    fan_aabb = ctx.part_element_world_aabb(surround, elem="fanlight_grille")
    if not ctx.check("fanlight grille resolves to an AABB", fan_aabb is not None):
        return ctx.report()
    ctx.check(
        "fanlight grille sits above the leaf (fixed fanlight)",
        fan_aabb[0][2] > LEAF_Z0 + LEAF_H - 0.10,
        details=f"grille z_min={fan_aabb[0][2]:.3f}, leaf top={LEAF_Z0 + LEAF_H:.3f}",
    )
    ctx.expect_contact(
        surround, surround, elem_a="fanlight_grille", elem_b="masonry",
        contact_tol=0.02, name="fanlight grille meets the stone arch",
    )

    # Gold transom accents exist and are centered.
    gold_aabb = ctx.part_element_world_aabb(surround, elem="fanlight_gold")
    if ctx.check("fanlight gold accents resolve to an AABB", gold_aabb is not None):
        gcx = 0.5 * (gold_aabb[0][0] + gold_aabb[1][0])
        ctx.check(
            "fanlight gold sits above the spring line",
            gold_aabb[0][2] > PILLAR_H - 0.01,
            details=f"gold z_min={gold_aabb[0][2]:.3f}, spring={PILLAR_H:.3f}",
        )
        ctx.check(
            "fanlight gold composition is centered",
            abs(gcx) < 0.02,
            details=f"gold center_x={gcx:.3f}",
        )

    # --- Open pose: positive q swings the free (latch) edge outward (-Y) ---
    closed_y_min = aabb[0][1]
    closed_x_max = aabb[1][0]
    with ctx.pose({hinge: 1.5}):
        open_aabb = ctx.part_world_aabb(gate_leaf)
        if ctx.check(
            "open-pose AABB resolves",
            open_aabb is not None,
        ):
            ctx.check(
                "gate leaf swings outward (-Y) when opened",
                open_aabb[0][1] < closed_y_min - 0.30,
                details=f"closed_minY={closed_y_min:.3f}, open_minY={open_aabb[0][1]:.3f}",
            )
            # Opened leaf clears the center passage.
            ctx.check(
                "opened leaf clears the central passage",
                open_aabb[0][0] > -0.35 or open_aabb[1][0] < 0.35,
                details=f"open x=[{open_aabb[0][0]:.3f},{open_aabb[1][0]:.3f}]",
            )
            # The latch edge (formerly at +X) swings toward -Y.
            ctx.check(
                "latch edge moves outward when opening",
                open_aabb[0][1] < closed_y_min - 0.20,
                details=f"closed_minY={closed_y_min:.3f}, open_minY={open_aabb[0][1]:.3f}",
            )

    # Verify only one non-fixed articulation exists (single-swing gate).
    joints = [j for j in object_model.articulations]
    ctx.check(
        "exactly one articulation (single-swing gate)",
        len(joints) == 1,
        details=f"found {len(joints)} articulations",
    )

    return ctx.report()


object_model = build_object_model()
