from __future__ import annotations

# Arched-top wooden casement window with a radial fanlight transom and two
# divided-lite casement sashes, set in a white stone surround with a bracketed
# sill.
#
# Reference (002.png): a round-arched window. A white stone surround frames it:
# a semicircular arch moulding with a keystone at the apex, straight stone jambs
# down the sides, and a projecting stone sill carried on two stepped corbels at
# the bottom. Inside the stone sits a stained-brown WOOD frame: an arched head,
# two jambs, and a horizontal spring-line transom. Above the transom is a fixed
# semicircular FANLIGHT with radial muntin bars (a sunburst) plus a concentric
# arc bar. Below the transom are TWO casement sashes that meet at the center,
# each divided into a grid of small lites (~3 columns x 5 rows) by real muntins.
#
# Coordinate convention (Z up):
#   - height along +Z, width along X, frame depth / swing-normal along Y.
#   - the glass plane is the X-Z plane; sashes swing into +/-Y.
#   - the stone sill bottom sits at z=0.
#
# Structure:
#   - Static root part "surround": stone arch moulding + jambs + keystone +
#     sill + two corbels + the wood arched frame (head, jambs, spring transom)
#     + the fixed fanlight (radial + arc ribs). All fused/attached to the root.
#   - Two casement sash parts (sash_left, sash_right): each a wood ring + a real
#     muntin grid (3x5 lites) + thin tinted glass. Built as one parametric sash,
#     instantiated twice and mirrored in X.
#   - Side-hung REVOLUTE joints on the outer jambs; positive q swings each free
#     (center-meeting) edge outward toward the room (-Y).

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
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

# Wood-frame clear opening: a rectangle topped by a semicircle.
OPENING_HALF_W = 0.46           # half clear width (X) of the wood opening
ARCH_R = OPENING_HALF_W         # semicircle radius == half opening width

# The projecting sill is carried on two corbels that hang below it. The
# corbels are the lowest geometry, so the whole assembly is lifted by GROUND_LIFT
# to put the corbel bottoms at z=0 (the assembly stands on the ground at z=0).
CORBEL_DROP = 0.090             # how far corbels hang below the sill
# Corbels = upper step (0.7*drop) + lower step (0.4*drop) = 1.1*drop below sill.
GROUND_LIFT = CORBEL_DROP * 1.1   # lifts corbel bottoms to z=0

SILL_BOTTOM_Z = GROUND_LIFT     # bottom of the stone sill block
SILL_TOP_Z = SILL_BOTTOM_Z + 0.150   # top of the stone sill (sashes start here)
SPRING_Z = SILL_BOTTOM_Z + 1.18      # spring line: arch bottom / top of sashes
APEX_Z = SPRING_Z + ARCH_R           # top of the wood arch opening

# Wood frame profile widths.
WOOD_FRAME_W = 0.060            # wood frame face width (jambs / arch ring / head)
WOOD_DEPTH = 0.080              # wood frame depth (Y)
TRANSOM_H = 0.055               # spring-line transom bar height (Z)

# Center mullion (slim wood mullion behind the meeting stiles).
MULLION_W = 0.045

# Stone surround.
STONE_W = 0.110                 # stone moulding face width (the visible band)
STONE_DEPTH = 0.090             # stone depth (Y) - sits slightly proud
STONE_GAP = 0.010               # tiny reveal between wood frame outer and stone inner

# Keystone (wedge block at the arch apex).
KEYSTONE_W_BOT = 0.110
KEYSTONE_W_TOP = 0.150
KEYSTONE_H = 0.160

# Stone sill + corbels.
SILL_OVERHANG_X = 0.075         # sill projects past stone jambs each side
SILL_FRONT_Y = 0.085            # sill projects forward (+Y)
CORBEL_W = 0.075
CORBEL_INSET_X = 0.18           # corbel center offset from window center

# Fanlight (above the spring line, the fixed semicircle).
FANLIGHT_RIB_W = 0.024          # radial / arc rib thickness
FAN_RADIAL_COUNT = 7            # radial sunburst bars
FAN_ARC_FRACS = (0.55,)         # concentric arc bar at this fraction of radius

# Sash divided lites.
SASH_COLS = 3
SASH_ROWS = 5
MUNTIN_W = 0.022                # muntin bar width
SASH_STILE_W = 0.050            # sash perimeter frame face width
SASH_DEPTH = 0.055              # sash ring depth (Y)
GLASS_T = 0.006

# Sash plan: the two sashes together fill the rectangular lower opening
# (z from SILL_TOP_Z to SPRING_Z, x from -OPENING_HALF_W to +OPENING_HALF_W).
# The rebate (lip) is applied only on the HINGE (jamb) side and the top/bottom,
# never on the center-meeting edge, so the two sashes leave a clean center gap
# instead of overlapping at the meeting stiles.
SASH_REBATE = 0.012             # sash laps the frame jamb/sill/head lip
SASH_CLEAR_MULLION = 0.014      # gap between the two center-meeting stiles

SASH_OPENING_W = OPENING_HALF_W - SASH_CLEAR_MULLION / 2.0
SASH_OPENING_H = SPRING_Z - SILL_TOP_Z

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

WOOD_RGBA = (0.36, 0.20, 0.10, 1.0)     # stained brown wood
STONE_RGBA = (0.86, 0.85, 0.82, 1.0)    # white/cream stone
GLASS_RGBA = (0.50, 0.56, 0.60, 0.30)   # cool grey, semi-transparent


# ===========================================================================
# Helpers: profiles are drawn in the XZ plane and extruded symmetrically
# about Y=0 so every authored solid is centered on the glass plane.
# ===========================================================================

def _extrude_xz(wp: cq.Workplane, depth: float) -> cq.Workplane:
    """Extrude an XZ-plane closed wire symmetrically about Y=0 by `depth` total."""
    return wp.extrude(depth / 2.0, both=True)


def _build_wood_frame_shape() -> cq.Workplane:
    """Stained-wood arched frame: arched head ring, two jambs, spring transom,
    plus a slim center mullion behind the meeting stiles.
    """
    half_w = OPENING_HALF_W
    fw = WOOD_FRAME_W
    bottom = SILL_TOP_Z

    # OUTER outline (outer edge of the wood frame): rectangle + semicircle.
    outer_half = half_w + fw
    outer = (
        cq.Workplane("XZ")
        .moveTo(-outer_half, bottom)
        .lineTo(outer_half, bottom)
        .lineTo(outer_half, SPRING_Z)
        .threePointArc((0.0, SPRING_Z + outer_half), (-outer_half, SPRING_Z))
        .close()
    )
    outer_solid = _extrude_xz(outer, WOOD_DEPTH)

    # INNER opening (clear glass area): rectangle + semicircle radius half_w.
    inner = (
        cq.Workplane("XZ")
        .moveTo(-half_w, bottom - 0.02)
        .lineTo(half_w, bottom - 0.02)
        .lineTo(half_w, SPRING_Z)
        .threePointArc((0.0, SPRING_Z + half_w), (-half_w, SPRING_Z))
        .close()
    )
    inner_solid = _extrude_xz(inner, WOOD_DEPTH + 0.04)

    frame = outer_solid.cut(inner_solid)

    # Spring-line transom bar across the full inner width at z=SPRING_Z.
    transom = (
        cq.Workplane("XZ")
        .moveTo(0.0, SPRING_Z)
        .rect(2 * half_w, TRANSOM_H)
    )
    frame = frame.union(_extrude_xz(transom, WOOD_DEPTH))

    # Center mullion from sill to transom.
    mull_h = SPRING_Z - SILL_TOP_Z
    mullion = (
        cq.Workplane("XZ")
        .moveTo(0.0, SILL_TOP_Z + mull_h / 2.0)
        .rect(MULLION_W, mull_h)
    )
    frame = frame.union(_extrude_xz(mullion, WOOD_DEPTH * 0.7))

    return frame


def _build_fanlight_shape() -> cq.Workplane:
    """Fixed radial fanlight ribs above the spring line (sunburst + arc bar).

    The sunburst center is on the spring line at the window center; radial ribs
    fan out to just inside the wood arch, with one concentric arc bar.
    """
    cz = SPRING_Z
    r_out = ARCH_R - 0.008   # ribs reach just inside the wood arch ring
    rib_d = WOOD_DEPTH * 0.6
    shape = None

    # Radial ribs: evenly spaced strictly between the two horizontals.
    for i in range(FAN_RADIAL_COUNT):
        ang = math.pi * (i + 1) / (FAN_RADIAL_COUNT + 1)
        length = r_out
        mx = math.cos(ang) * length / 2.0
        mz = cz + math.sin(ang) * length / 2.0
        rib = (
            cq.Workplane("XZ")
            .moveTo(mx, mz)
            .rect(length, FANLIGHT_RIB_W)
        )
        rib_solid = _extrude_xz(rib, rib_d)
        # rotate the rib in the XZ plane (about a Y-parallel axis at its center).
        rib_solid = rib_solid.rotate((mx, 0.0, mz), (mx, 1.0, mz), math.degrees(ang))
        shape = rib_solid if shape is None else shape.union(rib_solid)

    # Concentric arc bar(s): a thin annular band segment at each fraction radius.
    for frac in FAN_ARC_FRACS:
        r = r_out * frac
        ri = r - FANLIGHT_RIB_W
        band = (
            cq.Workplane("XZ")
            .moveTo(r, cz)
            .threePointArc((0.0, cz + r), (-r, cz))
            .lineTo(-ri, cz)
            .threePointArc((0.0, cz + ri), (ri, cz))
            .close()
        )
        band_solid = _extrude_xz(band, rib_d)
        shape = band_solid if shape is None else shape.union(band_solid)

    return shape


def _build_stone_surround_shape() -> cq.Workplane:
    """White stone arched surround band wrapping the wood arch (jambs + arch)."""
    in_half = OPENING_HALF_W + WOOD_FRAME_W + STONE_GAP
    out_half = in_half + STONE_W
    bottom = SILL_TOP_Z

    outer = (
        cq.Workplane("XZ")
        .moveTo(-out_half, bottom)
        .lineTo(out_half, bottom)
        .lineTo(out_half, SPRING_Z)
        .threePointArc((0.0, SPRING_Z + out_half), (-out_half, SPRING_Z))
        .close()
    )
    outer_solid = _extrude_xz(outer, STONE_DEPTH)

    inner = (
        cq.Workplane("XZ")
        .moveTo(-in_half, bottom - 0.02)
        .lineTo(in_half, bottom - 0.02)
        .lineTo(in_half, SPRING_Z)
        .threePointArc((0.0, SPRING_Z + in_half), (-in_half, SPRING_Z))
        .close()
    )
    inner_solid = _extrude_xz(inner, STONE_DEPTH + 0.04)

    return outer_solid.cut(inner_solid)


def _build_keystone_shape() -> cq.Workplane:
    """Trapezoidal keystone wedge straddling the arch apex."""
    out_half = OPENING_HALF_W + WOOD_FRAME_W + STONE_GAP + STONE_W
    arch_apex_z = SPRING_Z + out_half  # top of the outer stone arc
    z_bot = arch_apex_z - KEYSTONE_H * 0.60
    z_top = arch_apex_z + KEYSTONE_H * 0.40
    half_bot = KEYSTONE_W_BOT / 2.0
    half_top = KEYSTONE_W_TOP / 2.0
    key = (
        cq.Workplane("XZ")
        .moveTo(-half_bot, z_bot)
        .lineTo(half_bot, z_bot)
        .lineTo(half_top, z_top)
        .lineTo(-half_top, z_top)
        .close()
    )
    return _extrude_xz(key, STONE_DEPTH + 0.020)


def _build_sill_and_corbels_shape() -> cq.Workplane:
    """Projecting stone sill slab plus two stepped corbels below it.

    Sill block spans z in [SILL_BOTTOM_Z, SILL_TOP_Z]; corbels hang below the
    sill down toward z=0 (the assembly ground reference).
    """
    out_half = OPENING_HALF_W + WOOD_FRAME_W + STONE_GAP + STONE_W
    sill_half_w = out_half + SILL_OVERHANG_X
    back_y = -STONE_DEPTH / 2.0
    front_y = STONE_DEPTH / 2.0 + SILL_FRONT_Y
    cy = (back_y + front_y) / 2.0
    depth = front_y - back_y

    sill_h = SILL_TOP_Z - SILL_BOTTOM_Z
    sill = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, cy, SILL_BOTTOM_Z + sill_h / 2.0))
        .box(2 * sill_half_w, depth, sill_h)
    )
    shape = sill

    # Two stepped corbels hanging below the sill near each end (down toward z=0).
    upper_h = CORBEL_DROP * 0.7
    lower_h = CORBEL_DROP * 0.4
    for sx in (-1.0, 1.0):
        cxp = sx * CORBEL_INSET_X
        c1 = (
            cq.Workplane("XY")
            .transformed(offset=(cxp, cy, SILL_BOTTOM_Z - upper_h / 2.0))
            .box(CORBEL_W, depth * 0.85, upper_h)
        )
        c2 = (
            cq.Workplane("XY")
            .transformed(offset=(cxp, cy + 0.01, SILL_BOTTOM_Z - upper_h - lower_h / 2.0))
            .box(CORBEL_W * 0.6, depth * 0.6, lower_h)
        )
        shape = shape.union(c1).union(c2)

    return shape


# ===========================================================================
# Sash geometry (parametric, mirrored in X by `sign`).
#   local X: 0 at the hinge (outer jamb) edge; body extends along sign*X toward
#            the center-meeting edge.
#   local Z: 0 at the sash bottom (world SILL_TOP_Z - SASH_REBATE).
#   local Y: thickness centered at y=0.
# The rebate lip is on the hinge side + top/bottom only; the center-meeting edge
# stops at the opening center minus the gap so the two sashes never overlap.
# ===========================================================================

SASH_W_FULL = SASH_OPENING_W + SASH_REBATE       # rebate on the hinge side only
SASH_H_FULL = SASH_OPENING_H + 2 * SASH_REBATE   # rebate on head + sill


def _build_sash_frame_shape(sign: float) -> cq.Workplane:
    """Sash wood ring + a 3x5 grid of muntin bars (real divided lites)."""
    w = SASH_W_FULL
    h = SASH_H_FULL
    t = SASH_STILE_W
    d = SASH_DEPTH
    cx = sign * w / 2.0

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(cx, 0.0, h / 2.0))
        .box(w, d, h)
    )
    opening = (
        cq.Workplane("XY")
        .transformed(offset=(cx, 0.0, h / 2.0))
        .box(w - 2 * t, d + 0.02, h - 2 * t)
    )
    grid = outer.cut(opening)

    # Inner opening bounds in absolute local X.
    if sign > 0:
        ix0, ix1 = t, w - t
    else:
        ix0, ix1 = -(w - t), -t
    iz0, iz1 = t, h - t
    inner_w = ix1 - ix0
    inner_h = iz1 - iz0
    bar_d = d * 0.7

    # vertical muntins
    for c in range(1, SASH_COLS):
        x = ix0 + inner_w * c / SASH_COLS
        bar = (
            cq.Workplane("XY")
            .transformed(offset=(x, 0.0, (iz0 + iz1) / 2.0))
            .box(MUNTIN_W, bar_d, inner_h)
        )
        grid = grid.union(bar)
    # horizontal muntins
    for r in range(1, SASH_ROWS):
        z = iz0 + inner_h * r / SASH_ROWS
        bar = (
            cq.Workplane("XY")
            .transformed(offset=((ix0 + ix1) / 2.0, 0.0, z))
            .box(inner_w, bar_d, MUNTIN_W)
        )
        grid = grid.union(bar)

    return grid


def _build_sash_glass_shape(sign: float) -> cq.Workplane:
    """Thin tinted glass pane filling the sash opening, rebated under the lip."""
    w = SASH_W_FULL
    h = SASH_H_FULL
    t = SASH_STILE_W
    rebate = 0.006
    gw = (w - 2 * t) + 2 * rebate
    gh = (h - 2 * t) + 2 * rebate
    cx = sign * w / 2.0
    glass = (
        cq.Workplane("XY")
        .transformed(offset=(cx, 0.0, h / 2.0))
        .box(gw, GLASS_T, gh)
    )
    return glass


def _add_sash(model: ArticulatedObject, name: str, *, sign: float) -> None:
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(sign), f"{name}_frame"),
        material="wood",
        name=f"{name}_frame",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(sign), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )


# ===========================================================================
# Model
# ===========================================================================

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arched_fanlight_casement")

    model.material("wood", rgba=WOOD_RGBA)
    model.material("stone", rgba=STONE_RGBA)
    model.material("glass", rgba=GLASS_RGBA)

    # --- Static root: stone surround + wood arch frame + fixed fanlight ---
    surround = model.part("surround")
    surround.visual(
        mesh_from_cadquery(_build_stone_surround_shape(), "stone_arch"),
        material="stone",
        name="stone_arch",
    )
    surround.visual(
        mesh_from_cadquery(_build_keystone_shape(), "keystone"),
        material="stone",
        name="keystone",
    )
    surround.visual(
        mesh_from_cadquery(_build_sill_and_corbels_shape(), "sill_corbels"),
        material="stone",
        name="sill_corbels",
    )
    surround.visual(
        mesh_from_cadquery(_build_wood_frame_shape(), "wood_frame"),
        material="wood",
        name="wood_frame",
    )
    surround.visual(
        mesh_from_cadquery(_build_fanlight_shape(), "fanlight"),
        material="wood",
        name="fanlight",
    )

    # --- Two casement sashes (true X-mirror, no yaw flip) ---
    _add_sash(model, "sash_left", sign=+1.0)
    _add_sash(model, "sash_right", sign=-1.0)

    # ----- Articulations -----
    # Sash local z=0 maps to world SILL_TOP_Z - SASH_REBATE (sash laps the sill).
    sash_z0 = SILL_TOP_Z - SASH_REBATE

    # LEFT sash: hinge at LEFT jamb (world x = -OPENING_HALF_W - REBATE), body +X
    # toward the center. Free (center) edge at +X. axis -Z swings it out to -Y.
    left_hinge_x = -OPENING_HALF_W - SASH_REBATE
    model.articulation(
        "left_jamb_to_sash",
        ArticulationType.REVOLUTE,
        parent="surround",
        child="sash_left",
        origin=Origin(xyz=(left_hinge_x, 0.0, sash_z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.5),
    )

    # RIGHT sash: hinge at RIGHT jamb (world x = +OPENING_HALF_W + REBATE), body
    # -X toward the center. Free (center) edge at -X. axis +Z swings it out to -Y.
    right_hinge_x = OPENING_HALF_W + SASH_REBATE
    model.articulation(
        "right_jamb_to_sash",
        ArticulationType.REVOLUTE,
        parent="surround",
        child="sash_right",
        origin=Origin(xyz=(right_hinge_x, 0.0, sash_z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=0.0, upper=1.5),
    )

    return model


# ===========================================================================
# Tests
# ===========================================================================

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    surround = object_model.get_part("surround")
    sash_left = object_model.get_part("sash_left")
    sash_right = object_model.get_part("sash_right")
    j_left = object_model.get_articulation("left_jamb_to_sash")
    j_right = object_model.get_articulation("right_jamb_to_sash")

    # --- Intentional overlaps (genuine capture) ---
    for s in ("sash_left", "sash_right"):
        ctx.allow_overlap(
            s, s,
            elem_a=f"{s}_glass", elem_b=f"{s}_frame",
            reason="Glass pane is rebated under the sash wood lip and muntins so it reads as captured, not floating.",
        )
    ctx.allow_overlap(
        "surround", "sash_left",
        reason="Left sash rebate laps the wood frame jamb/sill/mullion lip so the closed casement reads as seated.",
    )
    ctx.allow_overlap(
        "surround", "sash_right",
        reason="Right sash rebate laps the wood frame jamb/sill/mullion lip so the closed casement reads as seated.",
    )

    # --- Closed pose: both sashes seated in the frame plane ---
    with ctx.pose({j_left: 0.0, j_right: 0.0}):
        la = ctx.part_world_aabb(sash_left)
        ra = ctx.part_world_aabb(sash_right)
        la_yc = (la[0][1] + la[1][1]) / 2.0
        ra_yc = (ra[0][1] + ra[1][1]) / 2.0
        ctx.check(
            "sashes coplanar at closed pose",
            abs(la_yc - ra_yc) < 0.02,
            details=f"left_yc={la_yc:.3f} right_yc={ra_yc:.3f}",
        )
        la_xc = (la[0][0] + la[1][0]) / 2.0
        ra_xc = (ra[0][0] + ra[1][0]) / 2.0
        ctx.check(
            "left sash left of center, right sash right of center",
            la_xc < -0.05 and ra_xc > 0.05,
            details=f"left_xc={la_xc:.3f} right_xc={ra_xc:.3f}",
        )
        la_w = la[1][0] - la[0][0]
        ra_w = ra[1][0] - ra[0][0]
        ctx.check(
            "mirrored sashes have matching width and symmetric X",
            abs(la_w - ra_w) < 0.01 and abs(la_xc + ra_xc) < 0.02,
            details=f"left_w={la_w:.3f} right_w={ra_w:.3f} xc_sum={la_xc + ra_xc:.3f}",
        )
        # Sashes occupy the LOWER opening, below the spring line.
        ctx.check(
            "sashes sit below the spring line (fanlight is above)",
            la[1][2] <= SPRING_Z + 0.03 and ra[1][2] <= SPRING_Z + 0.03,
            details=f"left top z={la[1][2]:.3f} right top z={ra[1][2]:.3f} spring={SPRING_Z}",
        )

    # --- Static surround: root, spans the full arched opening, sits on z=0 ---
    sa = ctx.part_world_aabb(surround)
    la = ctx.part_world_aabb(sash_left)
    ctx.check(
        "surround wider than a single sash",
        (sa[1][0] - sa[0][0]) > (la[1][0] - la[0][0]) + 0.4,
        details=f"surround_w={sa[1][0]-sa[0][0]:.3f} sash_w={la[1][0]-la[0][0]:.3f}",
    )
    ctx.check(
        "assembly sits on the sill at z=0",
        abs(sa[0][2]) < 0.02,
        details=f"surround zmin={sa[0][2]:.4f}",
    )
    ctx.check(
        "arch reaches above the spring line",
        sa[1][2] > SPRING_Z + ARCH_R - 0.05,
        details=f"surround top z={sa[1][2]:.3f} expected >~{SPRING_Z + ARCH_R:.3f}",
    )
    sh = sa[1][2] - sa[0][2]
    sd = sa[1][1] - sa[0][1]
    ctx.check(
        "window stands vertically and is tall",
        sh > 1.6 and sh > sd * 3.0,
        details=f"height={sh:.3f} depth={sd:.3f}",
    )

    # --- Fanlight + keystone + sill present on the static surround ---
    fan = ctx.part_element_world_aabb(surround, elem="fanlight")
    ctx.check(
        "fanlight present above the spring line",
        fan is not None and fan[1][2] > SPRING_Z + 0.05 and (fan[1][0] - fan[0][0]) > 0.4,
        details=f"fanlight aabb={fan}",
    )
    key = ctx.part_element_world_aabb(surround, elem="keystone")
    ctx.check(
        "keystone present at the arch apex",
        key is not None and key[1][2] > SPRING_Z + ARCH_R and abs((key[0][0] + key[1][0]) / 2.0) < 0.05,
        details=f"keystone aabb={key}",
    )
    sill = ctx.part_element_world_aabb(surround, elem="sill_corbels")
    # Sill+corbels are the lowest geometry (corbel bottoms ~ z=0) and the sill
    # projects forward in +Y past the stone band.
    ctx.check(
        "projecting sill with corbels is the lowest element and projects in +Y",
        sill is not None
        and abs(sill[0][2]) < 0.02
        and sill[0][2] <= sa[0][2] + 1e-4
        and sill[1][1] > STONE_DEPTH / 2.0 + 0.03,
        details=f"sill aabb={sill}",
    )

    # --- Sash divided-lite muntins exist (frame element wide & tall, gridded) ---
    for s, part in (("sash_left", sash_left), ("sash_right", sash_right)):
        fr = ctx.part_element_world_aabb(part, elem=f"{s}_frame")
        ctx.check(
            f"{s} muntin grid spans the sash opening",
            fr is not None and (fr[1][0] - fr[0][0]) > 0.25 and (fr[1][2] - fr[0][2]) > 0.7,
            details=f"frame aabb={fr}",
        )

    # --- HERO: left sash opens (free/center edge swings to -Y, hinge stays) ---
    la0 = ctx.part_world_aabb(sash_left)
    l_hinge_x_rest = la0[0][0]
    l_free_y_rest = la0[1][1]
    with ctx.pose({j_left: 1.3}):
        lo = ctx.part_world_aabb(sash_left)
        ctx.check(
            "left sash free edge swings out toward -Y",
            lo[0][1] < l_free_y_rest - 0.18,
            details=f"open ymin={lo[0][1]:.3f} rest front y={l_free_y_rest:.3f}",
        )
        ctx.check(
            "left sash hinge edge stays at the jamb",
            abs(lo[0][0] - l_hinge_x_rest) < 0.07,
            details=f"open xmin={lo[0][0]:.3f} rest xmin={l_hinge_x_rest:.3f}",
        )

    # --- HERO mirror: right sash opens toward -Y, hinge stays at right jamb ---
    ra0 = ctx.part_world_aabb(sash_right)
    r_hinge_x_rest = ra0[1][0]
    r_free_y_rest = ra0[1][1]
    with ctx.pose({j_right: 1.3}):
        ro = ctx.part_world_aabb(sash_right)
        ctx.check(
            "right sash free edge swings out toward -Y",
            ro[0][1] < r_free_y_rest - 0.18,
            details=f"open ymin={ro[0][1]:.3f} rest front y={r_free_y_rest:.3f}",
        )
        ctx.check(
            "right sash hinge edge stays at the jamb",
            abs(ro[1][0] - r_hinge_x_rest) < 0.07,
            details=f"open xmax={ro[1][0]:.3f} rest xmax={r_hinge_x_rest:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
