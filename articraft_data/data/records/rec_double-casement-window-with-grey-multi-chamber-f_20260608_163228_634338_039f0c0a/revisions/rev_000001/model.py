from __future__ import annotations

# Double casement (French) window: a light-grey outer frame with a thick
# multi-chamber profile (visible stepped/chambered layers on the jambs), and
# TWO tall side-hung sashes that meet at the center. Each sash has a stepped
# (beveled) inner profile, dark glass, and a lever handle at its center meeting
# stile -> twin lever handles side by side at the center. Closed in the photo.
#
# Per the brief: build ONE parametric sash and instantiate it twice MIRRORED in
# X (do NOT yaw 180 degrees, which would flip the decorated room face to the
# back). The X-mirror is done on the GEOMETRY: the left sash is built in its
# native local frame (hinge stile at local x=0, free/center stile at local
# x=SASH_W) and the right sash is the same geometry reflected across the YZ
# plane (hinge stile at local x=0 still, but the body now extends along local
# -X). Reflection keeps the stepped lip + handle on the same room-facing (+Y)
# side for both leaves -- a true mirror, not a yaw.
#
# Each sash is a side-hung REVOLUTE casement hinged on its OUTER jamb; free
# edges meet at the center. Signs are chosen so positive q swings each free edge
# OUTWARD toward -Y (the exterior, matching a casement opening away from the
# room in this exterior view).
#
# Coordinate convention (per brief):
#   +Z up. Width along X, frame depth / glazing thickness / swing-normal along Y.
#   Glass plane is the X-Z plane. The frame bottom sits at z=0.
#
# Structure:
#   - frame (static root): CadQuery multi-chamber outer frame (deep slab with the
#     sash opening cut out, plus stepped chamber ribs on the outboard face).
#   - sash_left, sash_right: two leaves from one parametric builder; the right
#     leaf is the X-reflected copy. Each carries a stepped grey ring + dark glass
#     + a lever handle on its free (center) stile + barrel hinges on the outer
#     jamb stile.

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

SASH_CLEAR_W = 0.520        # clear glass width per sash (X)
SASH_CLEAR_H = 1.180        # clear glass height per sash (Z)

FRAME_FACE = 0.075          # outer frame face width (in-plane), thick profile
FRAME_DEPTH = 0.090         # outer frame depth (Y), multi-chamber profile depth
CHAMBER_STEP = 0.018        # projection of each chamber step rib (Y)
CHAMBER_RIBS = 3            # number of stepped chamber ribs on the jamb edge

SILL_Z = 0.0

SASH_FACE = 0.062           # sash ring face width
SASH_DEPTH = 0.070          # sash profile depth (Y)
SASH_STEP_FACE = 0.020      # width of the inner stepped/beveled lip
SASH_STEP_DEPTH = 0.018     # projection of the inner step toward the room (+Y)
GLASS_T = 0.008
SASH_CLEARANCE = 0.004      # running gap between sash ring and frame reveal
MEETING_GAP = 0.006         # half-gap at the center meeting stiles

HANDLE_BASE_W = 0.034
HANDLE_BASE_H = 0.090
HANDLE_BASE_T = 0.016       # standoff off the sash face (Y)
HANDLE_LEVER_LEN = 0.120
HANDLE_LEVER_R = 0.009

HINGE_R = 0.012
HINGE_LEN = 0.060
HINGE_COUNT = 2

# Derived overall sizes
SASH_W = SASH_CLEAR_W + 2 * SASH_FACE          # full sash ring width
SASH_H = SASH_CLEAR_H + 2 * SASH_FACE          # full sash ring height
OPENING_W = 2 * SASH_W + 2 * SASH_CLEARANCE + 2 * MEETING_GAP
OPENING_H = SASH_H + 2 * SASH_CLEARANCE
TOTAL_W = OPENING_W + 2 * FRAME_FACE
TOTAL_H = OPENING_H + 2 * FRAME_FACE

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

FRAME_RGBA = (0.66, 0.67, 0.68, 1.0)    # light-grey multi-chamber frame
SASH_RGBA = (0.70, 0.71, 0.72, 1.0)     # slightly lighter grey sash
GLASS_RGBA = (0.10, 0.12, 0.14, 0.55)   # dark glass, semi-transparent
HANDLE_RGBA = (0.78, 0.79, 0.80, 1.0)   # brushed light-grey lever handle
HINGE_RGBA = (0.45, 0.46, 0.47, 1.0)


# ---------------------------------------------------------------------------
# Static frame geometry (CadQuery): multi-chamber outer frame
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Light-grey outer frame with a layered multi-chamber profile.

    World coords: opening centered on X=0; Z from 0 (bottom) to TOTAL_H. A deep
    perimeter slab has the single sash opening cut out, then stepped chamber ribs
    are added on the outboard (-Y) face so the visible jamb edge reads as a
    multi-chamber profile (image 004's left jamb).
    """
    slab = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, SILL_Z + TOTAL_H / 2.0))
        .box(TOTAL_W, FRAME_DEPTH, TOTAL_H)
    )
    op_x = OPENING_W
    op_z = OPENING_H
    cut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, SILL_Z + FRAME_FACE + op_z / 2.0))
        .box(op_x, FRAME_DEPTH + 0.02, op_z)
    )
    frame = slab.cut(cut)

    # Each chamber rib is a perimeter ring inset from the frame outer edge by an
    # increasing amount and projecting further out (-Y), so the visible jamb edge
    # reads as a stack of stepped chambers. The ring inner edge stays aligned with
    # the main opening so each rib remains a true frame ring (not a sliver).
    out_y = -FRAME_DEPTH / 2.0
    for i in range(CHAMBER_RIBS):
        inset = (i + 1) * (FRAME_FACE / (CHAMBER_RIBS + 2))
        rib_w = TOTAL_W - 2 * inset
        rib_h = TOTAL_H - 2 * inset
        proj = (i + 1) * CHAMBER_STEP
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, out_y - proj / 2.0, SILL_Z + TOTAL_H / 2.0))
            .box(rib_w, proj, rib_h)
        )
        rib_cut = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, out_y - proj / 2.0, SILL_Z + FRAME_FACE + op_z / 2.0))
            .box(op_x, proj + 0.02, op_z)
        )
        rib = rib.cut(rib_cut)
        frame = frame.union(rib)
    return frame


# ---------------------------------------------------------------------------
# Parametric sash geometry (CadQuery): stepped ring + glass, authored ONCE.
# ---------------------------------------------------------------------------
# Native local sash frame: x runs 0 .. SASH_W (hinge stile at x=0, free/center
# stile at x=SASH_W), z runs 0 .. SASH_H, y is depth centered at 0. The room
# face is +Y (stepped lip + handle live there).

def _build_sash_ring_shape() -> cq.Workplane:
    """Stepped/beveled sash ring: outer perimeter ring + an inner stepped lip
    projecting toward the room (+Y), matching the photo's beveled sash edge."""
    w, h = SASH_W, SASH_H
    f = SASH_FACE
    d = SASH_DEPTH

    outer = cq.Workplane("XY").box(w, d, h, centered=(False, True, False))
    inner_cut = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, 0.0, h / 2.0))
        .box(w - 2 * f, d + 0.02, h - 2 * f)
    )
    ring = outer.cut(inner_cut)

    sf = SASH_STEP_FACE
    lip_outer = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, d / 2.0 + SASH_STEP_DEPTH / 2.0, h / 2.0))
        .box(w - 2 * f + 2 * sf, SASH_STEP_DEPTH, h - 2 * f + 2 * sf)
    )
    lip_cut = (
        cq.Workplane("XY")
        .transformed(offset=(w / 2.0, d / 2.0 + SASH_STEP_DEPTH / 2.0, h / 2.0))
        .box(w - 2 * f, SASH_STEP_DEPTH + 0.02, h - 2 * f)
    )
    lip = lip_outer.cut(lip_cut)
    return ring.union(lip)


def _build_sash_glass_shape() -> cq.Workplane:
    """Dark glass pane rebated under the sash ring lip."""
    w, h = SASH_W, SASH_H
    f = SASH_FACE
    rebate = 0.006
    gx0, gx1 = f - rebate, w - f + rebate
    gz0, gz1 = f - rebate, h - f + rebate
    return (
        cq.Workplane("XY")
        .transformed(offset=((gx0 + gx1) / 2.0, 0.0, (gz0 + gz1) / 2.0))
        .box(gx1 - gx0, GLASS_T, gz1 - gz0)
    )


def _maybe_mirror(shape: cq.Workplane, mirror_x: bool) -> cq.Workplane:
    """Reflect the shape across the YZ plane (X -> -X) for the mirrored leaf.

    This is a true X-mirror, not a yaw: the +Y room face stays +Y, so the
    stepped lip and handle remain on the room side for both leaves.
    """
    if not mirror_x:
        return shape
    return shape.mirror(mirrorPlane="YZ", basePointVector=(0.0, 0.0, 0.0))


def _add_sash(model: ArticulatedObject, name: str, *, mirror_x: bool) -> None:
    """Build one parametric sash. For the native (left) leaf the hinge stile is
    at local x=0 and the body extends +X. For the mirrored (right) leaf the same
    geometry is reflected across YZ, so the hinge stile is at local x=0 and the
    body extends -X, while the room (+Y) decorated face is preserved.
    """
    sash = model.part(name)
    sash.visual(
        mesh_from_cadquery(_maybe_mirror(_build_sash_ring_shape(), mirror_x), f"{name}_ring"),
        material="sash",
        name=f"{name}_ring",
    )
    sash.visual(
        mesh_from_cadquery(_maybe_mirror(_build_sash_glass_shape(), mirror_x), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )

    sign = -1.0 if mirror_x else 1.0  # mirror flips local X positions

    # Lever handle on the free (center-meeting) stile, ~45% up.
    hx = sign * (SASH_W - SASH_FACE / 2.0)
    hz = SASH_H * 0.45
    hy_base = SASH_DEPTH / 2.0 + SASH_STEP_DEPTH + HANDLE_BASE_T / 2.0
    sash.visual(
        Box((HANDLE_BASE_W, HANDLE_BASE_T, HANDLE_BASE_H)),
        origin=Origin(xyz=(hx, hy_base, hz)),
        material="handle",
        name=f"{name}_handle_base",
    )
    # Lever bar: angled down, extending toward the hinge side (local -X for the
    # native leaf, mirrored for the right leaf), standing off in +Y.
    hy_lever = SASH_DEPTH / 2.0 + SASH_STEP_DEPTH + HANDLE_BASE_T + HANDLE_LEVER_R
    lever_cx = hx - sign * (HANDLE_LEVER_LEN / 2.0 - HANDLE_BASE_W / 2.0)
    sash.visual(
        Cylinder(radius=HANDLE_LEVER_R, length=HANDLE_LEVER_LEN),
        origin=Origin(
            xyz=(lever_cx, hy_lever, hz - 0.03),
            rpy=(0.0, math.pi / 2.0 + 0.35, 0.0),
        ),
        material="handle",
        name=f"{name}_handle_lever",
    )

    # Visible barrel hinges on the hinge (outer-jamb) stile (local x=0 either way).
    z0 = SASH_FACE + HINGE_LEN / 2.0
    z_span = SASH_H - 2 * SASH_FACE - HINGE_LEN
    for i in range(HINGE_COUNT):
        frac = 0.0 if HINGE_COUNT == 1 else i / (HINGE_COUNT - 1)
        z = z0 + (0.12 + 0.76 * frac) * z_span
        sash.visual(
            Cylinder(radius=HINGE_R, length=HINGE_LEN),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="hinge",
            name=f"{name}_hinge_{i}",
        )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_casement_window")

    model.material("frame", rgba=FRAME_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)
    model.material("hinge", rgba=HINGE_RGBA)

    # --- Static multi-chamber frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="frame",
        name="frame_profile",
    )

    # --- Two mirrored parametric sashes ---
    _add_sash(model, "sash_left", mirror_x=False)
    _add_sash(model, "sash_right", mirror_x=True)

    # ----- Articulations (mirrored in X, NOT yawed) -----
    inner_x0 = -OPENING_W / 2.0    # left reveal
    inner_x1 = OPENING_W / 2.0     # right reveal
    seat_z = SILL_Z + FRAME_FACE + SASH_CLEARANCE  # sash local z=0 maps here

    # LEFT sash: hinged on the LEFT (outer) jamb. Body extends +X toward center.
    # Free (center) edge should swing OUT toward -Y. Body along +X about hinge
    # axis +Z sends the free edge to +Y (right-hand rule), so use axis (0,0,-1).
    hinge_x_left = inner_x0 + SASH_CLEARANCE
    model.articulation(
        "frame_to_sash_left",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash_left",
        origin=Origin(xyz=(hinge_x_left, 0.0, seat_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.6),
    )

    # RIGHT sash (geometry mirrored): hinge stile at local x=0, body extends -X.
    # Mount at the RIGHT reveal so the body fills the right half (toward center).
    # Body along -X about hinge axis +Z sends the free edge to -Y (right-hand
    # rule), which is OUTWARD as desired, so use axis (0,0,1).
    hinge_x_right = inner_x1 - SASH_CLEARANCE
    model.articulation(
        "frame_to_sash_right",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash_right",
        origin=Origin(xyz=(hinge_x_right, 0.0, seat_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.6),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash_l = object_model.get_part("sash_left")
    sash_r = object_model.get_part("sash_right")
    j_l = object_model.get_articulation("frame_to_sash_left")
    j_r = object_model.get_articulation("frame_to_sash_right")

    # --- Intentional overlaps ---
    for nm in ("sash_left", "sash_right"):
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_glass",
            elem_b=f"{nm}_ring",
            reason="Sash glass is rebated under the sash ring lip (captured glazing).",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_handle_base",
            elem_b=f"{nm}_ring",
            reason="Lever handle base rose is seated onto the sash face to mount the handle.",
        )
        ctx.allow_overlap(
            nm, nm,
            elem_a=f"{nm}_handle_lever",
            elem_b=f"{nm}_handle_base",
            reason="Lever bar attaches into its mounting base rose.",
        )
        for i in range(HINGE_COUNT):
            ctx.allow_overlap(
                nm, nm,
                elem_a=f"{nm}_hinge_{i}",
                elem_b=f"{nm}_ring",
                reason="Hinge knuckle is mounted on the sash hinge stile.",
            )
        ctx.allow_overlap(
            "frame", nm,
            reason="Sash hinges on the frame outer jamb; its hinge stile and knuckles intentionally meet the frame at the hinge line.",
        )
    # The two sashes meet at the center: their meeting stiles intentionally abut.
    ctx.allow_overlap(
        "sash_left", "sash_right",
        reason="Double-casement leaves meet at the center; their free meeting stiles intentionally abut at the closed pose.",
    )

    # --- Frame is the static root, stands vertically on the ground ---
    fa = ctx.part_world_aabb(frame)
    fw = fa[1][0] - fa[0][0]
    fh = fa[1][2] - fa[0][2]
    fd = fa[1][1] - fa[0][1]
    ctx.check(
        "frame sits on the ground at z~0",
        abs(fa[0][2] - SILL_Z) < 0.01,
        details=f"frame zmin={fa[0][2]:.3f}",
    )
    ctx.check(
        "window stands vertical (height >= width, height >> depth)",
        fh > fw * 0.9 and fh > 5.0 * fd,
        details=f"w={fw:.3f}, h={fh:.3f}, d={fd:.3f}",
    )
    ctx.check(
        "frame is realistic double-casement size",
        1.0 < fw < 1.8 and 1.1 < fh < 1.8,
        details=f"w={fw:.3f}, h={fh:.3f}",
    )

    # --- LEFT/RIGHT SYMMETRY: equal size, mirror-placed about X=0 ---
    al = ctx.part_world_aabb(sash_l)
    ar = ctx.part_world_aabb(sash_r)
    wl = al[1][0] - al[0][0]
    wr = ar[1][0] - ar[0][0]
    hl = al[1][2] - al[0][2]
    hr = ar[1][2] - ar[0][2]
    ctx.check(
        "sashes have equal width",
        abs(wl - wr) < 0.01,
        details=f"left width={wl:.3f}, right width={wr:.3f}",
    )
    ctx.check(
        "sashes have equal height",
        abs(hl - hr) < 0.01,
        details=f"left height={hl:.3f}, right height={hr:.3f}",
    )
    cxl = (al[0][0] + al[1][0]) / 2.0
    cxr = (ar[0][0] + ar[1][0]) / 2.0
    ctx.check(
        "sashes are mirror-placed about the center X=0",
        cxl < 0.0 < cxr and abs(cxl + cxr) < 0.02,
        details=f"left X center={cxl:.3f}, right X center={cxr:.3f}",
    )
    ctx.check(
        "sashes share the same vertical extent",
        abs(al[0][2] - ar[0][2]) < 0.01 and abs(al[1][2] - ar[1][2]) < 0.01,
        details=f"left z=({al[0][2]:.3f},{al[1][2]:.3f}), right z=({ar[0][2]:.3f},{ar[1][2]:.3f})",
    )

    # --- CLOSED pose: both sashes seated coplanar, meeting at the center ---
    with ctx.pose({j_l: 0.0, j_r: 0.0}):
        rl = ctx.part_element_world_aabb(sash_l, elem="sash_left_ring")
        rr = ctx.part_element_world_aabb(sash_r, elem="sash_right_ring")
        cy_l = (rl[0][1] + rl[1][1]) / 2.0
        cy_r = (rr[0][1] + rr[1][1]) / 2.0
        ctx.check(
            "closed: sash rings seated in the frame plane",
            abs(cy_l) < 0.05 and abs(cy_r) < 0.05,
            details=f"left ring Y={cy_l:.3f}, right ring Y={cy_r:.3f}",
        )
        gap = rr[0][0] - rl[1][0]  # right ring min X minus left ring max X
        ctx.check(
            "closed: sashes meet at the center with a small gap",
            -0.01 < gap < 0.06,
            details=f"center meeting gap={gap:.3f}",
        )
        hbl = ctx.part_element_world_aabb(sash_l, elem="sash_left_handle_base")
        hbr = ctx.part_element_world_aabb(sash_r, elem="sash_right_handle_base")
        hx_l = (hbl[0][0] + hbl[1][0]) / 2.0
        hx_r = (hbr[0][0] + hbr[1][0]) / 2.0
        ctx.check(
            "twin handles sit at the center meeting stiles",
            hx_l < 0.0 < hx_r and abs(hx_l) < 0.12 and abs(hx_r) < 0.12,
            details=f"left handle X={hx_l:.3f}, right handle X={hx_r:.3f}",
        )
        hy_l = (hbl[0][1] + hbl[1][1]) / 2.0
        ctx.check(
            "handles stand off the room-side face",
            hy_l > SASH_DEPTH / 2.0,
            details=f"left handle Y={hy_l:.3f}",
        )

    # --- OPEN pose: each free edge swings OUTWARD toward -Y, hinge edges fixed ---
    with ctx.pose({j_l: 0.0, j_r: 0.0}):
        al0 = ctx.part_world_aabb(sash_l)
        ar0 = ctx.part_world_aabb(sash_r)
        free_l_y0 = (al0[0][1] + al0[1][1]) / 2.0
        free_r_y0 = (ar0[0][1] + ar0[1][1]) / 2.0
        hinge_l_x0 = al0[0][0]   # left sash hinge edge = its min X (left jamb)
        hinge_r_x0 = ar0[1][0]   # right sash hinge edge = its max X (right jamb)
    with ctx.pose({j_l: 1.2, j_r: 1.2}):
        al1 = ctx.part_world_aabb(sash_l)
        ar1 = ctx.part_world_aabb(sash_r)
        free_l_y1 = (al1[0][1] + al1[1][1]) / 2.0
        free_r_y1 = (ar1[0][1] + ar1[1][1]) / 2.0
        ctx.check(
            "open: left sash free edge swings OUT toward -Y",
            free_l_y1 < free_l_y0 - 0.15,
            details=f"closed Y={free_l_y0:.3f}, open Y={free_l_y1:.3f}",
        )
        ctx.check(
            "open: right sash free edge swings OUT toward -Y",
            free_r_y1 < free_r_y0 - 0.15,
            details=f"closed Y={free_r_y0:.3f}, open Y={free_r_y1:.3f}",
        )
        ctx.check(
            "open: left sash hinge edge stays on the left jamb",
            abs(al1[0][0] - hinge_l_x0) < 0.10,
            details=f"closed hinge X={hinge_l_x0:.3f}, open min X={al1[0][0]:.3f}",
        )
        ctx.check(
            "open: right sash hinge edge stays on the right jamb",
            abs(ar1[1][0] - hinge_r_x0) < 0.10,
            details=f"closed hinge X={hinge_r_x0:.3f}, open max X={ar1[1][0]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
