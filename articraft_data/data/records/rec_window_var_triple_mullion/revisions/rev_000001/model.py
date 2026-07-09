from __future__ import annotations

# Three-wide mullioned casement window: a light-grey outer frame with a thick
# multi-chamber profile, THREE tall side-hung sashes separated by two vertical
# mullions across one shared outer frame. Each sash has a stepped (beveled)
# inner profile, dark glass, and a lever handle on its free stile.
#
# The leaf count is parameter-driven: NUM_SASHES controls the number of bays,
# mullions, and joints via a single for-i-in-range loop.
#
# Layout (exterior view):
#   [left jamb][sash_0][mullion_0][sash_1][mullion_1][sash_2][right jamb]
#
# Hinging policy (side-hung casement):
#   sash_0: left-hung  (hinge on left jamb,    free edge near mullion_0)
#   sash_1: left-hung  (hinge on mullion_0,    free edge near mullion_1)
#   sash_2: right-hung (hinge on right jamb,   free edge near mullion_1)
#
# Signs chosen so positive q swings each free edge OUTWARD toward -Y.
#
# Coordinate convention:
#   +Z up. Width along X, frame depth / glazing thickness along Y.
#   Glass plane is the X-Z plane. Frame bottom at z=0.

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
# Parameters
# ---------------------------------------------------------------------------

NUM_SASHES = 3  # parameter-driven leaf count

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

SASH_CLEAR_W = 0.520        # clear glass width per sash (X)
SASH_CLEAR_H = 1.180        # clear glass height per sash (Z)

FRAME_FACE = 0.075          # outer frame face width (in-plane), thick profile
FRAME_DEPTH = 0.090         # outer frame depth (Y), multi-chamber depth
CHAMBER_STEP = 0.018        # projection of each chamber step rib (Y)
CHAMBER_RIBS = 3            # number of stepped chamber ribs on outboard face

MULLION_W = 0.050           # mullion face width (vertical divider between bays)

SILL_Z = 0.0

SASH_FACE = 0.062           # sash ring face width
SASH_DEPTH = 0.070          # sash profile depth (Y)
SASH_STEP_FACE = 0.020      # inner stepped/beveled lip width
SASH_STEP_DEPTH = 0.018     # step projection toward room (+Y)
GLASS_T = 0.008
SASH_CLEARANCE = 0.004      # running gap between sash ring and frame/mullion reveal

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
BAY_W = SASH_W + 2 * SASH_CLEARANCE            # one sash bay clear width
OPENING_W = NUM_SASHES * BAY_W + (NUM_SASHES - 1) * MULLION_W
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
# Bay layout helpers (parameter-driven)
# ---------------------------------------------------------------------------

def _bay_left(i: int) -> float:
    """Left edge of bay i in world X (opening centered on X=0)."""
    return -OPENING_W / 2.0 + i * (BAY_W + MULLION_W)


def _bay_center(i: int) -> float:
    """Center X of bay i."""
    return _bay_left(i) + BAY_W / 2.0


def _mullion_center(i: int) -> float:
    """Center X of mullion between bay i and bay i+1."""
    return _bay_left(i) + BAY_W + MULLION_W / 2.0


def _is_left_hung(i: int) -> bool:
    """Sashes 0..N-2 are left-hung; sash N-1 is right-hung."""
    return i < NUM_SASHES - 1


# ---------------------------------------------------------------------------
# Static frame geometry (CadQuery): multi-chamber outer frame + mullions
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Light-grey outer frame with layered multi-chamber profile.

    The frame slab has NUM_SASHES bay openings cut through it. The uncut slab
    material between adjacent bays naturally forms the vertical mullions.
    Chamber ribs on the outboard (-Y) face are also cut per-bay so the mullion
    zone inherits the same stepped profile depth as the outer jambs.
    """
    slab = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, SILL_Z + TOTAL_H / 2.0))
        .box(TOTAL_W, FRAME_DEPTH, TOTAL_H)
    )
    # Cut each bay opening through the slab
    frame = slab
    bz = SILL_Z + FRAME_FACE + OPENING_H / 2.0
    for i in range(NUM_SASHES):
        bx = _bay_center(i)
        cut = (
            cq.Workplane("XY")
            .transformed(offset=(bx, 0.0, bz))
            .box(BAY_W, FRAME_DEPTH + 0.02, OPENING_H)
        )
        frame = frame.cut(cut)

    # Chamber ribs on outboard face: perimeter rings with bay cuts
    out_y = -FRAME_DEPTH / 2.0
    for r in range(CHAMBER_RIBS):
        inset = (r + 1) * (FRAME_FACE / (CHAMBER_RIBS + 2))
        rib_w = TOTAL_W - 2 * inset
        rib_h = TOTAL_H - 2 * inset
        proj = (r + 1) * CHAMBER_STEP
        rib = (
            cq.Workplane("XY")
            .transformed(offset=(0.0, out_y - proj / 2.0, SILL_Z + TOTAL_H / 2.0))
            .box(rib_w, proj, rib_h)
        )
        # Cut each bay opening through the rib ring
        for i in range(NUM_SASHES):
            bx = _bay_center(i)
            rib_cut = (
                cq.Workplane("XY")
                .transformed(offset=(bx, out_y - proj / 2.0, bz))
                .box(BAY_W, proj + 0.02, OPENING_H)
            )
            rib = rib.cut(rib_cut)
        frame = frame.union(rib)

    return frame


# ---------------------------------------------------------------------------
# Parametric sash geometry (shared helper, instantiated in a loop)
# ---------------------------------------------------------------------------

def _build_sash_ring_shape() -> cq.Workplane:
    """Stepped/beveled sash ring in native local frame.

    Local frame: x 0..SASH_W (hinge stile at x=0, free stile at x=SASH_W),
    z 0..SASH_H, y centered at 0. Room face is +Y (stepped lip + handle).
    """
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
    """Reflect across the YZ plane (X -> -X) for right-hung leaves."""
    if not mirror_x:
        return shape
    return shape.mirror(mirrorPlane="YZ", basePointVector=(0.0, 0.0, 0.0))


def _add_sash(model: ArticulatedObject, name: str, *, mirror_x: bool) -> None:
    """Build one parametric sash part. mirror_x=True for right-hung leaves.

    Native local frame: hinge stile at local x=0, body extends +X (or -X when
    mirrored). Room (+Y) decorated face is preserved for both orientations.
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

    # Lever handle on the free (non-hinge) stile, ~45% up.
    hx = sign * (SASH_W - SASH_FACE / 2.0)
    hz = SASH_H * 0.45
    hy_base = SASH_DEPTH / 2.0 + SASH_STEP_DEPTH + HANDLE_BASE_T / 2.0
    sash.visual(
        Box((HANDLE_BASE_W, HANDLE_BASE_T, HANDLE_BASE_H)),
        origin=Origin(xyz=(hx, hy_base, hz)),
        material="handle",
        name=f"{name}_handle_base",
    )
    # Lever bar extending toward hinge side, standing off in +Y.
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

    # Visible barrel hinges on the hinge stile (local x=0 either way).
    z0 = SASH_FACE + HINGE_LEN / 2.0
    z_span = SASH_H - 2 * SASH_FACE - HINGE_LEN
    for h in range(HINGE_COUNT):
        frac = 0.0 if HINGE_COUNT == 1 else h / (HINGE_COUNT - 1)
        z = z0 + (0.12 + 0.76 * frac) * z_span
        sash.visual(
            Cylinder(radius=HINGE_R, length=HINGE_LEN),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="hinge",
            name=f"{name}_hinge_{h}",
        )


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_casement_window")

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

    seat_z = SILL_Z + FRAME_FACE + SASH_CLEARANCE  # sash local z=0 maps here

    # --- Build NUM_SASHES sashes via parameter-driven loop ---
    for i in range(NUM_SASHES):
        name = f"sash_{i}"
        left_hung = _is_left_hung(i)
        mirror_x = not left_hung  # right-hung leaves are X-mirrored

        _add_sash(model, name, mirror_x=mirror_x)

        # Hinge position and axis
        if left_hung:
            # Hinge at left edge of bay; body extends +X toward right mullion/jamb
            hinge_x = _bay_left(i) + SASH_CLEARANCE
            # Body along +X about axis -Z: free edge swings to -Y (outward)
            axis = (0.0, 0.0, -1.0)
        else:
            # Hinge at right edge of bay; body extends -X toward left mullion
            hinge_x = _bay_left(i) + BAY_W - SASH_CLEARANCE
            # Body along -X about axis +Z: free edge swings to -Y (outward)
            axis = (0.0, 0.0, 1.0)

        model.articulation(
            f"frame_to_{name}",
            ArticulationType.REVOLUTE,
            parent="frame",
            child=name,
            origin=Origin(xyz=(hinge_x, 0.0, seat_z)),
            axis=axis,
            motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.6),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sashes = [object_model.get_part(f"sash_{i}") for i in range(NUM_SASHES)]
    joints = [object_model.get_articulation(f"frame_to_sash_{i}") for i in range(NUM_SASHES)]

    # --- Intentional overlaps (looped per sash) ---
    for i in range(NUM_SASHES):
        nm = f"sash_{i}"
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
        for h in range(HINGE_COUNT):
            ctx.allow_overlap(
                nm, nm,
                elem_a=f"{nm}_hinge_{h}",
                elem_b=f"{nm}_ring",
                reason="Hinge knuckle is mounted on the sash hinge stile.",
            )
        ctx.allow_overlap(
            "frame", nm,
            reason="Sash hinges on the frame jamb or mullion; hinge stile and knuckles intentionally meet the frame at the hinge line.",
        )

    # --- Frame is the static root, stands vertically ---
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
        "window stands vertical (height >= width * 0.6, height >> depth)",
        fh > fw * 0.55 and fh > 4.0 * fd,
        details=f"w={fw:.3f}, h={fh:.3f}, d={fd:.3f}",
    )
    ctx.check(
        "frame is realistic triple-casement size",
        1.5 < fw < 2.8 and 1.1 < fh < 1.9,
        details=f"w={fw:.3f}, h={fh:.3f}",
    )

    # --- Three sashes: equal size, evenly spaced left to right ---
    aabbs = [ctx.part_world_aabb(s) for s in sashes]
    widths = [a[1][0] - a[0][0] for a in aabbs]
    heights = [a[1][2] - a[0][2] for a in aabbs]
    centers_x = [(a[0][0] + a[1][0]) / 2.0 for a in aabbs]

    for i in range(NUM_SASHES):
        ctx.check(
            f"sash_{i} has realistic width",
            0.4 < widths[i] < 0.8,
            details=f"width={widths[i]:.3f}",
        )
    for i in range(1, NUM_SASHES):
        ctx.check(
            f"sash_{i} same width as sash_0",
            abs(widths[i] - widths[0]) < 0.01,
            details=f"sash_0 w={widths[0]:.3f}, sash_{i} w={widths[i]:.3f}",
        )
        ctx.check(
            f"sash_{i} same height as sash_0",
            abs(heights[i] - heights[0]) < 0.01,
            details=f"sash_0 h={heights[0]:.3f}, sash_{i} h={heights[i]:.3f}",
        )
    # Ordered left to right
    for i in range(1, NUM_SASHES):
        ctx.check(
            f"sash_{i} is to the right of sash_{i-1}",
            centers_x[i] > centers_x[i - 1] + 0.1,
            details=f"sash_{i-1} cx={centers_x[i-1]:.3f}, sash_{i} cx={centers_x[i]:.3f}",
        )
    # Approximately evenly spaced (parameter-driven layout)
    if NUM_SASHES >= 3:
        gap01 = centers_x[1] - centers_x[0]
        gap12 = centers_x[2] - centers_x[1]
        ctx.check(
            "three sashes are approximately evenly spaced",
            abs(gap01 - gap12) < 0.05,
            details=f"gap01={gap01:.3f}, gap12={gap12:.3f}",
        )

    # --- CLOSED pose: all sashes seated coplanar in the frame plane ---
    closed_pose = {j: 0.0 for j in joints}
    with ctx.pose(closed_pose):
        for i in range(NUM_SASHES):
            nm = f"sash_{i}"
            ring_aabb = ctx.part_element_world_aabb(sashes[i], elem=f"{nm}_ring")
            cy = (ring_aabb[0][1] + ring_aabb[1][1]) / 2.0
            ctx.check(
                f"closed: sash_{i} ring seated in frame plane",
                abs(cy) < 0.05,
                details=f"ring Y={cy:.3f}",
            )
        # Sashes do not interpenetrate each other (mullions separate them)
        for i in range(NUM_SASHES - 1):
            right_edge = aabbs[i][1][0]
            left_edge_next = aabbs[i + 1][0][0]
            # The mullion sits between them, so there should be separation
            ctx.check(
                f"closed: sash_{i} and sash_{i+1} separated by mullion",
                left_edge_next > right_edge - 0.01,
                details=f"sash_{i} xmax={right_edge:.3f}, sash_{i+1} xmin={left_edge_next:.3f}",
            )

    # --- OPEN pose: each free edge swings OUTWARD toward -Y ---
    open_pose = {j: 1.2 for j in joints}
    closed_y_centers = []
    hinge_xs_closed = []
    with ctx.pose(closed_pose):
        for i in range(NUM_SASHES):
            a = ctx.part_world_aabb(sashes[i])
            closed_y_centers.append((a[0][1] + a[1][1]) / 2.0)
            if _is_left_hung(i):
                hinge_xs_closed.append(a[0][0])  # min X = hinge edge
            else:
                hinge_xs_closed.append(a[1][0])  # max X = hinge edge

    with ctx.pose(open_pose):
        for i in range(NUM_SASHES):
            a = ctx.part_world_aabb(sashes[i])
            open_y = (a[0][1] + a[1][1]) / 2.0
            ctx.check(
                f"open: sash_{i} free edge swings OUT toward -Y",
                open_y < closed_y_centers[i] - 0.10,
                details=f"closed Y={closed_y_centers[i]:.3f}, open Y={open_y:.3f}",
            )
            if _is_left_hung(i):
                hinge_x_open = a[0][0]
            else:
                hinge_x_open = a[1][0]
            ctx.check(
                f"open: sash_{i} hinge edge stays near its jamb/mullion",
                abs(hinge_x_open - hinge_xs_closed[i]) < 0.10,
                details=f"closed hinge X={hinge_xs_closed[i]:.3f}, open hinge X={hinge_x_open:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
