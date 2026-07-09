from __future__ import annotations

# Twin-sash casement window with a dark anthracite aluminium frame, integral
# venetian blinds between the glass, mounted on a projecting white sill.
#
# Reference (001.png): two tall narrow sashes side by side in a near-black
# aluminium frame; the whole frame sits on a chunky projecting white sill.
# Behind the glass there is a stack of thin horizontal venetian slats (between-
# glass blinds). The sashes are side-hung casements (each hinges on its outer
# jamb, meeting at a center mullion).
#
# Coordinate convention (Z up):
#   - height along +Z, width along X, frame depth / swing-normal along Y.
#   - the glass plane is the X-Z plane; sashes swing into +/-Y.
#   - the assembly sits with the white sill bottom at z=0.
#
# Structure:
#   - Root (static): white projecting sill + dark anthracite outer frame
#     (head, sill rail, two jambs, center mullion), one CadQuery solid each.
#   - Two casement sash parts, each = anthracite sash ring + thin tinted glass +
#     a stack of horizontal venetian slats sitting just inside the sash behind
#     the glass + a slim lever handle on the center-meeting stile.
#   - Side-hung REVOLUTE joints: left sash hinges on the left jamb, right sash on
#     the right jamb. One parametric sash, instantiated twice, mirrored in X.
#     Positive q swings the free (center) edge outward toward -Y. The blinds ride
#     with their sash.

import math

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

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

# Overall opening (dark frame outer extents). Tall narrow twin sashes.
FRAME_W = 1.10               # outer frame width along X
FRAME_H = 1.45               # outer frame height along Z (dark frame only)
FRAME_DEPTH = 0.075          # outer frame depth along Y

OUTER_PROFILE = 0.055        # dark frame face width (head/jambs/sill rail)
MULLION_W = 0.050            # center mullion width (X)

# White projecting sill below the dark frame.
SILL_H = 0.075               # sill height (Z)
SILL_OVERHANG_Y = 0.060      # how far the sill projects past the frame in +Y
SILL_OVERHANG_X = 0.045      # how far the sill projects past the frame on each side
SILL_DEPTH = FRAME_DEPTH + SILL_OVERHANG_Y + 0.030   # sill total depth in Y

# The dark frame sits on top of the sill.
FRAME_Z0 = SILL_H            # bottom of dark frame
FRAME_Z1 = FRAME_Z0 + FRAME_H

# Sash geometry. Each clear opening = (FRAME_W - 2*OUTER_PROFILE - MULLION_W)/2.
OPENING_W = FRAME_W - 2 * OUTER_PROFILE - MULLION_W   # combined clear width (both)
SASH_OPENING_W = OPENING_W / 2.0                      # one sash clear opening width
SASH_OPENING_H = FRAME_H - 2 * OUTER_PROFILE

SASH_PROFILE = 0.042         # sash ring face width (X/Z)
SASH_DEPTH = 0.050           # sash ring depth (Y)
SASH_CLEAR = 0.004           # running clearance between sash edge and frame opening

# A sash slightly overlaps the frame opening lip so it reads as seated (rebate).
SASH_W = SASH_OPENING_W + 2 * (OUTER_PROFILE * 0.0)   # placeholder; computed below
GLASS_T = 0.008              # glazing thickness (Y)

# Venetian slats (between-glass blinds) inside each sash.
SLAT_COUNT = 11
SLAT_T = 0.0035              # slat thickness (Z)
SLAT_DEPTH = 0.010           # slat depth (Y) - thin horizontal members

# Lever handle on the center-meeting stile.
HANDLE_LEN = 0.110
HANDLE_R = 0.0085
HANDLE_STANDOFF = 0.028

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

ANTHRACITE_RGBA = (0.13, 0.135, 0.145, 1.0)   # near-black aluminium
WHITE_SILL_RGBA = (0.92, 0.92, 0.93, 1.0)     # white sill
GLASS_RGBA = (0.55, 0.62, 0.68, 0.28)         # cool grey, semi-transparent
SLAT_RGBA = (0.85, 0.86, 0.87, 1.0)           # pale aluminium venetian slats
HANDLE_RGBA = (0.20, 0.21, 0.23, 1.0)         # dark satin lever handle


# ---------------------------------------------------------------------------
# Static root geometry (CadQuery)
# ---------------------------------------------------------------------------

def _build_dark_frame_shape() -> cq.Workplane:
    """Dark anthracite outer frame: head, sill rail, two jambs, center mullion.

    World frame: opening centered on X=0; dark frame spans Z = FRAME_Z0..FRAME_Z1;
    depth centered on Y=0. Built as a solid slab with the two sash openings cut
    out, leaving a real perimeter ring plus the center mullion.
    """
    cx = 0.0
    cz = (FRAME_Z0 + FRAME_Z1) / 2.0

    outer = (
        cq.Workplane("XY")
        .transformed(offset=(cx, 0.0, cz))
        .box(FRAME_W, FRAME_DEPTH, FRAME_H)
    )

    # Two clear openings (left + right), separated by the center mullion.
    # Left opening spans X from frame inner-left to mullion-left.
    inner_left = -FRAME_W / 2.0 + OUTER_PROFILE
    inner_right = FRAME_W / 2.0 - OUTER_PROFILE
    mull_left = -MULLION_W / 2.0
    mull_right = MULLION_W / 2.0
    op_z0 = FRAME_Z0 + OUTER_PROFILE
    op_z1 = FRAME_Z1 - OUTER_PROFILE

    left_cut = (
        cq.Workplane("XY")
        .transformed(offset=((inner_left + mull_left) / 2.0, 0.0, (op_z0 + op_z1) / 2.0))
        .box(mull_left - inner_left, FRAME_DEPTH + 0.02, op_z1 - op_z0)
    )
    right_cut = (
        cq.Workplane("XY")
        .transformed(offset=((mull_right + inner_right) / 2.0, 0.0, (op_z0 + op_z1) / 2.0))
        .box(inner_right - mull_right, FRAME_DEPTH + 0.02, op_z1 - op_z0)
    )

    return outer.cut(left_cut).cut(right_cut)


def _build_sill_shape() -> cq.Workplane:
    """White projecting sill slab under the dark frame.

    Wider than the frame in X, projecting forward in +Y, with a slight forward
    slope read implied by overhang. Top at z=SILL_H, bottom at z=0.
    """
    sill_w = FRAME_W + 2 * SILL_OVERHANG_X
    # Sill is pushed forward in +Y so its front face projects past the frame.
    # Frame depth centered on Y=0 spans -FRAME_DEPTH/2 .. +FRAME_DEPTH/2.
    # Sill spans from the frame back face forward by SILL_DEPTH.
    sill_back_y = -FRAME_DEPTH / 2.0
    sill_front_y = sill_back_y + SILL_DEPTH
    cy = (sill_back_y + sill_front_y) / 2.0
    sill = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, cy, SILL_H / 2.0))
        .box(sill_w, SILL_DEPTH, SILL_H)
    )
    return sill


# ---------------------------------------------------------------------------
# Sash geometry (CadQuery) - authored in the sash-local HINGE frame.
#   local X: 0 at the hinge (outer jamb) edge; body extends along sign*X toward
#            the free (center) edge. sign=+1 for the left sash, -1 for the right
#            sash (a true X-mirror, keeping the +Y/-Y faces identical for both).
#   local Z: 0 at the sash bottom, +Z up.
#   local Y: sash thickness centered at y=0. Blinds always on the +Y side.
# ---------------------------------------------------------------------------

# Sash overall width: spans the clear opening plus a small rebate lip over the
# frame on the outer/hinge side and the mullion side so it reads as seated.
REBATE = 0.012
SASH_W_FULL = SASH_OPENING_W + 2 * REBATE
SASH_H_FULL = SASH_OPENING_H + 2 * REBATE


def _build_sash_frame_shape(sign: float) -> cq.Workplane:
    """Anthracite sash ring (one rebated rectangle, glass opening cut out)."""
    w = SASH_W_FULL
    h = SASH_H_FULL
    t = SASH_PROFILE
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
    return outer.cut(opening)


def _build_sash_glass_shape(sign: float) -> cq.Workplane:
    """Thin tinted glass pane filling the sash opening, rebated under the lip."""
    w = SASH_W_FULL
    h = SASH_H_FULL
    t = SASH_PROFILE
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


def _build_blind_slats_shape(sign: float) -> cq.Workplane:
    """A stack of thin horizontal venetian slats sitting just inside the sash,
    behind the glass (between-glass blinds). Mounted to the sash ring at the
    sides so they are not floating: each slat spans the full inner opening width
    and tucks under the sash side stiles.
    """
    w = SASH_W_FULL
    h = SASH_H_FULL
    t = SASH_PROFILE
    inner_w = w - 2 * t
    # Slats span slightly past the inner opening into the side stiles so they
    # read as mounted within the sash (captured by side rails).
    slat_w = inner_w + 0.012
    cx = sign * w / 2.0
    # Slats sit just behind the glass plane (glass at y=0). Put blinds on the
    # +Y / inner cavity side so they read "between the glass / inside" for BOTH
    # sashes (sign only mirrors X, never Y).
    slat_y = +0.012
    z0 = t + 0.030
    z1 = h - t - 0.030
    span = z1 - z0
    shape = None
    for i in range(SLAT_COUNT):
        frac = i / (SLAT_COUNT - 1)
        z = z0 + frac * span
        slat = (
            cq.Workplane("XY")
            .transformed(offset=(cx, slat_y, z))
            .box(slat_w, SLAT_DEPTH, SLAT_T)
        )
        shape = slat if shape is None else shape.union(slat)
    return shape


# ---------------------------------------------------------------------------
# Sash part builder
# ---------------------------------------------------------------------------

def _add_sash(model: ArticulatedObject, name: str, *, sign: float) -> None:
    """Build one casement sash in its local hinge frame (hinge edge at local
    x=0, free edge at local x=sign*SASH_W_FULL). sign=+1 left, -1 right.
    """
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_build_sash_frame_shape(sign), f"{name}_frame"),
        material="anthracite",
        name=f"{name}_frame",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(sign), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )
    sash.visual(
        mesh_from_cadquery(_build_blind_slats_shape(sign), f"{name}_blinds"),
        material="slat",
        name=f"{name}_blinds",
    )

    # Lever handle on the free (center-meeting) stile, mounted on a standoff
    # facing -Y (the room side). Place near mid-height.
    handle_x = sign * (SASH_W_FULL - SASH_PROFILE / 2.0)
    handle_z = SASH_H_FULL * 0.5
    handle_y = -(SASH_DEPTH / 2.0 + HANDLE_STANDOFF)
    # Standoff base (short cylinder into the sash stile).
    sash.visual(
        Cylinder(radius=0.010, length=HANDLE_STANDOFF + SASH_DEPTH / 2.0),
        origin=Origin(xyz=(handle_x, handle_y / 2.0, handle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="handle",
        name=f"{name}_handle_base",
    )
    # Lever bar pointing downward-ish (vertical bar) along Z.
    sash.visual(
        Cylinder(radius=HANDLE_R, length=HANDLE_LEN),
        origin=Origin(xyz=(handle_x, handle_y, handle_z - HANDLE_LEN / 2.0 + 0.015)),
        material="handle",
        name=f"{name}_handle_lever",
    )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_sash_casement_venetian")

    model.material("anthracite", rgba=ANTHRACITE_RGBA)
    model.material("white_sill", rgba=WHITE_SILL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("slat", rgba=SLAT_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)

    # --- Static root: sill + dark frame fused as the carrier ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_sill_shape(), "sill"),
        material="white_sill",
        name="sill",
    )
    frame.visual(
        mesh_from_cadquery(_build_dark_frame_shape(), "dark_frame"),
        material="anthracite",
        name="dark_frame",
    )

    # --- Two casement sashes (true X-mirror, NO yaw flip) ---
    _add_sash(model, "sash_left", sign=+1.0)
    _add_sash(model, "sash_right", sign=-1.0)

    # ----- Articulations -----
    # Both sashes hinge at local x=0 (their outer jamb). Local frame == world
    # orientation for both (no yaw flip), so glass stays at y=0 and the venetian
    # blinds stay on the +Y cavity side for both. Swing side is the room (-Y).
    # Sash seated (closed) z-bottom in world = FRAME_Z0 + OUTER_PROFILE - REBATE.
    sash_z0 = FRAME_Z0 + OUTER_PROFILE - REBATE

    # LEFT sash: hinge on the LEFT JAMB, body extends toward the center (+X).
    # Free (center) edge at +X. Right-hand rule about +Z rotates +X toward +Y;
    # we want the free edge to swing OUT toward the room (-Y), so axis = -Z.
    left_hinge_x = -FRAME_W / 2.0 + OUTER_PROFILE - REBATE
    model.articulation(
        "left_jamb_to_sash",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash_left",
        origin=Origin(xyz=(left_hinge_x, 0.0, sash_z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.6),
    )

    # RIGHT sash: hinge on the RIGHT JAMB, body extends toward the center (-X,
    # the X-mirrored sash). Free (center) edge at -X. Right-hand rule about +Z
    # rotates -X toward -Y, which is OUT toward the room, so axis = +Z.
    right_hinge_x = FRAME_W / 2.0 - OUTER_PROFILE + REBATE
    model.articulation(
        "right_jamb_to_sash",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash_right",
        origin=Origin(xyz=(right_hinge_x, 0.0, sash_z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=1.6),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash_left = object_model.get_part("sash_left")
    sash_right = object_model.get_part("sash_right")
    j_left = object_model.get_articulation("left_jamb_to_sash")
    j_right = object_model.get_articulation("right_jamb_to_sash")

    # --- Intentional overlaps (genuine capture) ---
    for s in ("sash_left", "sash_right"):
        ctx.allow_overlap(
            s, s,
            elem_a=f"{s}_glass", elem_b=f"{s}_frame",
            reason="Glass pane is rebated under the sash frame lip so it reads as captured, not floating.",
        )
        ctx.allow_overlap(
            s, s,
            elem_a=f"{s}_blinds", elem_b=f"{s}_frame",
            reason="Venetian slats span past the inner opening into the side stiles so they read as mounted within the sash.",
        )
        ctx.allow_overlap(
            s, s,
            elem_a=f"{s}_handle_base", elem_b=f"{s}_frame",
            reason="Handle standoff is seated into the sash stile to mount the lever.",
        )
    # Sashes rebate over the frame opening lip (seated, side-hung).
    ctx.allow_overlap(
        "frame", "sash_left",
        reason="Left sash rebate laps the frame jamb/mullion lip so the closed casement reads as seated in the opening.",
    )
    ctx.allow_overlap(
        "frame", "sash_right",
        reason="Right sash rebate laps the frame jamb/mullion lip so the closed casement reads as seated in the opening.",
    )

    # --- Closed pose: both sashes seated in the frame plane (small Y spread) ---
    with ctx.pose({j_left: 0.0, j_right: 0.0}):
        la = ctx.part_world_aabb(sash_left)
        ra = ctx.part_world_aabb(sash_right)
        # Sashes are coplanar in Y at closed pose.
        la_yc = (la[0][1] + la[1][1]) / 2.0
        ra_yc = (ra[0][1] + ra[1][1]) / 2.0
        ctx.check(
            "sashes coplanar at closed pose",
            abs(la_yc - ra_yc) < 0.03,
            details=f"left_yc={la_yc:.3f} right_yc={ra_yc:.3f}",
        )
        # Left sash sits left of center, right sash sits right of center.
        la_xc = (la[0][0] + la[1][0]) / 2.0
        ra_xc = (ra[0][0] + ra[1][0]) / 2.0
        ctx.check(
            "left sash left of center, right sash right of center",
            la_xc < -0.05 and ra_xc > 0.05,
            details=f"left_xc={la_xc:.3f} right_xc={ra_xc:.3f}",
        )
        # Mirror symmetry: left/right sash X extents match (mirrored).
        la_w = la[1][0] - la[0][0]
        ra_w = ra[1][0] - ra[0][0]
        ctx.check(
            "mirrored sashes have matching width",
            abs(la_w - ra_w) < 0.01 and abs(la_xc + ra_xc) < 0.02,
            details=f"left_w={la_w:.3f} right_w={ra_w:.3f}, xc_sum={la_xc + ra_xc:.3f}",
        )

    # --- Frame is static root, spans wider/taller than a single sash ---
    fa = ctx.part_world_aabb(frame)
    la = ctx.part_world_aabb(sash_left)
    frame_w = fa[1][0] - fa[0][0]
    sash_w = la[1][0] - la[0][0]
    ctx.check(
        "frame wider than a single sash",
        frame_w > sash_w + 0.4,
        details=f"frame_w={frame_w:.3f} sash_w={sash_w:.3f}",
    )
    # Bottom of the whole assembly (white sill) sits at/near z=0.
    ctx.check(
        "assembly sits on the sill at z=0",
        abs(fa[0][2]) < 0.01,
        details=f"frame zmin={fa[0][2]:.4f}",
    )
    # The assembly stands vertically (taller than deep).
    frame_h = fa[1][2] - fa[0][2]
    frame_d = fa[1][1] - fa[0][1]
    ctx.check(
        "window stands vertically",
        frame_h > 1.0 and frame_h > frame_d * 3.0,
        details=f"frame_h={frame_h:.3f} frame_d={frame_d:.3f}",
    )

    # --- Blinds present, sit inside the sash opening, behind the glass ---
    for s, part in (("sash_left", sash_left), ("sash_right", sash_right)):
        ba = ctx.part_element_world_aabb(part, elem=f"{s}_blinds")
        ga = ctx.part_element_world_aabb(part, elem=f"{s}_glass")
        ctx.check(
            f"{s} blinds present and span the opening",
            ba is not None and (ba[1][0] - ba[0][0]) > 0.2 and (ba[1][2] - ba[0][2]) > 0.5,
            details=f"blinds aabb={ba}",
        )
        ctx.check(
            f"{s} blinds sit behind the glass (+Y side)",
            ba is not None and ga is not None and ((ba[0][1] + ba[1][1]) / 2.0) > ((ga[0][1] + ga[1][1]) / 2.0),
            details=f"blinds_yc vs glass_yc",
        )

    # --- HERO: left sash opens (free/center edge swings out toward -Y, hinge
    # edge at the left jamb stays put) ---
    la0 = ctx.part_world_aabb(sash_left)
    # hinge edge (min X of closed left sash) is the left jamb edge.
    left_hinge_x_rest = la0[0][0]
    left_free_x_rest = la0[1][0]
    left_free_y_rest = la0[1][1]  # front face
    with ctx.pose({j_left: 1.4}):
        lo = ctx.part_world_aabb(sash_left)
        # The free edge (formerly max X) should now swing out: Y extent grows in -Y.
        ctx.check(
            "left sash free edge swings out toward -Y",
            lo[0][1] < left_free_y_rest - 0.20,
            details=f"open ymin={lo[0][1]:.3f} rest front y={left_free_y_rest:.3f}",
        )
        # The hinge edge (left jamb) stays roughly in place in X.
        ctx.check(
            "left sash hinge edge stays at the jamb",
            abs(lo[0][0] - left_hinge_x_rest) < 0.06,
            details=f"open xmin={lo[0][0]:.3f} rest xmin={left_hinge_x_rest:.3f}",
        )
        # The sash no longer fully covers its closed X span (it rotated open).
        ctx.check(
            "left sash rotated out of the closed plane",
            lo[1][0] < left_free_x_rest - 0.10,
            details=f"open xmax={lo[1][0]:.3f} rest xmax={left_free_x_rest:.3f}",
        )

    # --- HERO mirror: right sash opens toward -Y, hinge at right jamb stays put ---
    ra0 = ctx.part_world_aabb(sash_right)
    right_hinge_x_rest = ra0[1][0]
    right_free_x_rest = ra0[0][0]
    right_free_y_rest = ra0[1][1]
    with ctx.pose({j_right: 1.4}):
        ro = ctx.part_world_aabb(sash_right)
        ctx.check(
            "right sash free edge swings out toward -Y",
            ro[0][1] < right_free_y_rest - 0.20,
            details=f"open ymin={ro[0][1]:.3f} rest front y={right_free_y_rest:.3f}",
        )
        ctx.check(
            "right sash hinge edge stays at the jamb",
            abs(ro[1][0] - right_hinge_x_rest) < 0.06,
            details=f"open xmax={ro[1][0]:.3f} rest xmax={right_hinge_x_rest:.3f}",
        )
        ctx.check(
            "right sash rotated out of the closed plane",
            ro[0][0] > right_free_x_rest + 0.10,
            details=f"open xmin={ro[0][0]:.3f} rest xmin={right_free_x_rest:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
