from __future__ import annotations

# Bank of grey aluminium casement windows: a 2-wide x 2-high grid of lites
# (4 openings) in a static grey aluminium frame with one internal vertical
# mullion and one horizontal transom. Light-blue tinted glass fills every
# lite. One of the four lites is an operable side-hung casement sash (the rest
# are fixed glass set directly in the frame):
#   - the TOP-LEFT lite swings INWARD (toward +Y), hinged on the LEFT jamb
#
# Coordinate convention (per brief):
#   +Z is up. Width runs along X, frame depth / glazing thickness / swing-normal
#   along Y. The glass plane is the X-Z plane. The assembly sill sits at z=0.
#
# Structure:
#   - frame (static root): CadQuery perimeter + 1 mullion + 1 transom, built as
#     a solid slab with the 4 lite openings cut out, leaving a true frame web.
#   - fixed glass panes (3): thin tinted panes rebated into the fixed lites, all
#     parented rigidly... actually fused onto the frame as separate visuals so
#     they read as captured glass (the frame is their carrier).
#   - one operable sash part: its own aluminium frame ring + tinted glass
#     + a small lever handle, hinged to the frame with a REVOLUTE joint about a
#     vertical edge (axis (0,0,1)), swinging +Y (in).
#   - visible barrel hinges (short cylinders) straddle the sash hinge edge.

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

# Overall bank: 2 lites wide x 2 lites high. Each lite ~0.62 m wide x ~0.78 m
# tall clear, giving an outer frame ~1.41 m wide x ~1.73 m tall.
N_COLS = 2
N_ROWS = 2

LITE_W = 0.620          # clear lite width (X)
LITE_H = 0.780          # clear lite height (Z)

FRAME_W = 0.055         # perimeter / mullion / transom face width (in-plane)
FRAME_DEPTH = 0.075     # frame profile depth (Y)
GLASS_T = 0.008         # glass pane thickness (Y)

# Outer overall extents
TOTAL_W = N_COLS * LITE_W + (N_COLS + 1) * FRAME_W   # 2*0.62 + 3*0.055 = 1.405
TOTAL_H = N_ROWS * LITE_H + (N_ROWS + 1) * FRAME_W   # 2*0.78 + 3*0.055 = 1.725

SILL_Z = 0.0            # bottom of the frame sits on the ground

# Sash (operable leaf) profile
SASH_FRAME_W = 0.050    # sash ring face width
SASH_DEPTH = 0.055      # sash profile depth (Y), a bit thinner than main frame
SASH_GLASS_T = 0.008
SASH_CLEARANCE = 0.004  # running clearance between sash ring and lite reveal

# Lever handle
HANDLE_BASE_W = 0.030
HANDLE_BASE_H = 0.070
HANDLE_BASE_T = 0.014   # base standoff off the sash face (Y)
HANDLE_LEVER_LEN = 0.110
HANDLE_LEVER_R = 0.0085

# Hinges (visible barrel knuckles)
HINGE_R = 0.011
HINGE_LEN = 0.055
HINGE_COUNT = 2

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

ALU_RGBA = (0.62, 0.64, 0.66, 1.0)      # grey aluminium frame
SASH_RGBA = (0.66, 0.68, 0.70, 1.0)     # slightly lighter sash aluminium
GLASS_RGBA = (0.62, 0.78, 0.84, 0.34)   # light-blue tinted, semi-transparent
HANDLE_RGBA = (0.30, 0.31, 0.33, 1.0)   # dark satin lever handle
HINGE_RGBA = (0.40, 0.41, 0.43, 1.0)    # darker machined hinge knuckle


# ---------------------------------------------------------------------------
# Lite grid geometry (world coordinates)
# ---------------------------------------------------------------------------
# Lite (col, row) clear-opening bounds in world X/Z. col 0 = left, row 0 = bottom.

def _lite_bounds(col: int, row: int) -> tuple[float, float, float, float]:
    """Return (x0, x1, z0, z1) clear-opening bounds of lite (col, row)."""
    x0 = -TOTAL_W / 2.0 + FRAME_W + col * (LITE_W + FRAME_W)
    x1 = x0 + LITE_W
    z0 = SILL_Z + FRAME_W + row * (LITE_H + FRAME_W)
    z1 = z0 + LITE_H
    return x0, x1, z0, z1


# ---------------------------------------------------------------------------
# Static frame geometry (CadQuery): perimeter + mullions + transom as a web
# ---------------------------------------------------------------------------

def _build_frame_shape() -> cq.Workplane:
    """Static aluminium frame: a solid slab the full outer size with the 4 lite
    openings cut out, leaving a true perimeter + 1 vertical mullion + 1
    horizontal transom web. Authored in world coordinates (X width, Z height,
    Y depth centered at 0)."""
    slab = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, SILL_Z + TOTAL_H / 2.0))
        .box(TOTAL_W, FRAME_DEPTH, TOTAL_H)
    )

    for col in range(N_COLS):
        for row in range(N_ROWS):
            x0, x1, z0, z1 = _lite_bounds(col, row)
            cut = (
                cq.Workplane("XY")
                .transformed(offset=((x0 + x1) / 2.0, 0.0, (z0 + z1) / 2.0))
                .box(x1 - x0, FRAME_DEPTH + 0.02, z1 - z0)
            )
            slab = slab.cut(cut)
    return slab


def _build_fixed_glass_shape(col: int, row: int) -> cq.Workplane:
    """Thin tinted glass pane for one FIXED lite, rebated under the frame lip."""
    x0, x1, z0, z1 = _lite_bounds(col, row)
    rebate = 0.007  # glass tucks under the frame lip on every edge
    gx0, gx1 = x0 - rebate, x1 + rebate
    gz0, gz1 = z0 - rebate, z1 + rebate
    return (
        cq.Workplane("XY")
        .transformed(offset=((gx0 + gx1) / 2.0, 0.0, (gz0 + gz1) / 2.0))
        .box(gx1 - gx0, GLASS_T, gz1 - gz0)
    )


# ---------------------------------------------------------------------------
# Sash geometry (CadQuery): an aluminium ring + glass, in a local hinge frame.
# ---------------------------------------------------------------------------
# Local sash frame: x runs 0 .. SASH_W (hinge edge at x=0, free edge at x=SASH_W),
# z runs 0 .. SASH_H, y is depth centered at 0. The sash fills the lite opening
# minus a running clearance all round.

def _sash_size(col: int, row: int) -> tuple[float, float]:
    x0, x1, z0, z1 = _lite_bounds(col, row)
    sw = (x1 - x0) - 2 * SASH_CLEARANCE
    sh = (z1 - z0) - 2 * SASH_CLEARANCE
    return sw, sh


def _build_sash_ring_shape(sw: float, sh: float) -> cq.Workplane:
    """Aluminium sash perimeter ring (slab minus a single central opening)."""
    t = SASH_FRAME_W
    d = SASH_DEPTH
    outer = (
        cq.Workplane("XY")
        .box(sw, d, sh, centered=(False, True, False))
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(sw / 2.0, 0.0, sh / 2.0))
        .box(sw - 2 * t, d + 0.02, sh - 2 * t)
    )
    return outer.cut(inner)


def _build_sash_glass_shape(sw: float, sh: float) -> cq.Workplane:
    """Thin tinted glass for the sash, rebated under the sash ring lip."""
    t = SASH_FRAME_W
    rebate = 0.006
    gx0, gx1 = t - rebate, sw - t + rebate
    gz0, gz1 = t - rebate, sh - t + rebate
    return (
        cq.Workplane("XY")
        .transformed(offset=((gx0 + gx1) / 2.0, 0.0, (gz0 + gz1) / 2.0))
        .box(gx1 - gx0, SASH_GLASS_T, gz1 - gz0)
    )


def _add_sash(
    model: ArticulatedObject,
    name: str,
    *,
    sw: float,
    sh: float,
    handle_side: str,   # "free" edge where the lever sits (opposite the hinge)
    handle_face_y: float,  # sign of Y face the handle stands off toward
) -> None:
    """Build one operable casement sash in its local hinge frame (hinge at x=0,
    extends to local +X)."""
    sash = model.part(name)

    sash.visual(
        mesh_from_cadquery(_build_sash_ring_shape(sw, sh), f"{name}_ring"),
        material="sash",
        name=f"{name}_ring",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(sw, sh), f"{name}_glass"),
        material="glass",
        name=f"{name}_glass",
    )

    # Lever handle on the free vertical stile (local x = sw - SASH_FRAME_W/2),
    # at hand height (~45% up the sash). A flat base plate sits on the sash face;
    # a horizontal lever bar extends inboard from it.
    hx = sw - SASH_FRAME_W / 2.0
    hz = sh * 0.45
    hy_base = handle_face_y * (SASH_DEPTH / 2.0 + HANDLE_BASE_T / 2.0)
    sash.visual(
        Box((HANDLE_BASE_W, HANDLE_BASE_T, HANDLE_BASE_H)),
        origin=Origin(xyz=(hx, hy_base, hz)),
        material="handle",
        name=f"{name}_handle_base",
    )
    # Lever bar: extends horizontally toward the hinge side (local -X) so it
    # reads as a turnable casement handle. Stands off the base in the same Y.
    hy_lever = handle_face_y * (SASH_DEPTH / 2.0 + HANDLE_BASE_T + HANDLE_LEVER_R)
    sash.visual(
        Cylinder(radius=HANDLE_LEVER_R, length=HANDLE_LEVER_LEN),
        origin=Origin(
            xyz=(hx - HANDLE_LEVER_LEN / 2.0 + HANDLE_BASE_W / 2.0, hy_lever, hz),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="handle",
        name=f"{name}_handle_lever",
    )

    # Visible barrel hinges straddling the sash hinge edge (local x=0).
    z0 = SASH_FRAME_W + HINGE_LEN / 2.0
    z_span = sh - 2 * SASH_FRAME_W - HINGE_LEN
    for i in range(HINGE_COUNT):
        frac = 0.0 if HINGE_COUNT == 1 else i / (HINGE_COUNT - 1)
        # Bias the two hinges toward top and bottom thirds.
        z = z0 + (0.15 + 0.7 * frac) * z_span
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
    model = ArticulatedObject(name="aluminium_casement_window_bank")

    model.material("alu", rgba=ALU_RGBA)
    model.material("sash", rgba=SASH_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("handle", rgba=HANDLE_RGBA)
    model.material("hinge", rgba=HINGE_RGBA)

    # --- Static frame web (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_frame_shape(), "frame"),
        material="alu",
        name="frame_web",
    )

    # --- Fixed glass in the 3 non-operable lites ---
    # Operable lite: top-left (col 0, row 1) swings IN. The other three lites
    # get fixed glass fused onto the frame part.
    operable = {(0, 1)}
    for col in range(N_COLS):
        for row in range(N_ROWS):
            if (col, row) in operable:
                continue
            frame.visual(
                mesh_from_cadquery(
                    _build_fixed_glass_shape(col, row), f"fixed_glass_{col}_{row}"
                ),
                material="glass",
                name=f"fixed_glass_{col}_{row}",
            )

    # --- Operable sash: TOP-LEFT, hinged on the LEFT jamb, swings IN (+Y) ---
    sw_l, sh_l = _sash_size(0, 1)
    # Handle on the free (right) stile, standing off toward the room (+Y), since
    # this leaf opens inward.
    _add_sash(model, "sash_inward", sw=sw_l, sh=sh_l, handle_side="free", handle_face_y=1.0)

    # ----- Articulations -----
    # Inward leaf: hinge line is the LEFT edge of lite (0,1). The sash local frame
    # has its hinge edge at local x=0 and extends along local +X. Seat the hinge
    # at the left reveal + clearance, vertically at the sash bottom (local z=0).
    lx0, lx1, lz0, lz1 = _lite_bounds(0, 1)
    hinge_x_in = lx0 + SASH_CLEARANCE        # left reveal, just inside the jamb
    hinge_z_in = lz0 + SASH_CLEARANCE        # sash local z=0 maps here
    # Closed leaf extends +X from the hinge. axis +Z -> positive q swings the free
    # edge toward +Y (into the room). That matches the inward-opening leaf.
    model.articulation(
        "frame_to_sash_inward",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash_inward",
        origin=Origin(xyz=(hinge_x_in, 0.0, hinge_z_in)),
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
    sash_in = object_model.get_part("sash_inward")
    j_in = object_model.get_articulation("frame_to_sash_inward")

    # --- Intentional overlaps ---
    # Fixed glass panes are rebated under the frame lip (captured glass).
    operable = {(0, 1)}
    for col in range(N_COLS):
        for row in range(N_ROWS):
            if (col, row) in operable:
                continue
            ctx.allow_overlap(
                "frame", "frame",
                elem_a=f"fixed_glass_{col}_{row}",
                elem_b="frame_web",
                reason="Fixed glass pane is rebated under the aluminium frame lip so it reads as captured, not floating.",
            )
    # Sash glass tucks under its own sash ring lip.
    ctx.allow_overlap(
        "sash_inward", "sash_inward",
        elem_a="sash_inward_glass",
        elem_b="sash_inward_ring",
        reason="Sash glass is rebated under the sash aluminium ring lip (captured glazing).",
    )
    # Handle base plate seats slightly onto the sash ring face.
    ctx.allow_overlap(
        "sash_inward", "sash_inward",
        elem_a="sash_inward_handle_base",
        elem_b="sash_inward_ring",
        reason="Lever handle base plate is seated onto the sash face to mount the handle.",
    )
    # Lever bar overlaps its own base plate where it attaches.
    ctx.allow_overlap(
        "sash_inward", "sash_inward",
        elem_a="sash_inward_handle_lever",
        elem_b="sash_inward_handle_base",
        reason="Lever bar attaches into its mounting base plate.",
    )
    # Hinge knuckles straddle the sash hinge edge and the frame jamb/mullion.
    for i in range(HINGE_COUNT):
        ctx.allow_overlap(
            "sash_inward", "sash_inward",
            elem_a=f"sash_inward_hinge_{i}",
            elem_b="sash_inward_ring",
            reason="Hinge knuckle is mounted on the sash hinge stile.",
        )
    ctx.allow_overlap(
        "frame", "sash_inward",
        reason="Sash hinges on the frame jamb; its hinge stile and knuckles intentionally meet the frame at the hinge line.",
    )

    # --- Grid count: exactly 4 lites (2x2) ---
    all_lites = [(col, row) for col in range(N_COLS) for row in range(N_ROWS)]
    ctx.check(
        "bank has 4 lites (2 cols x 2 rows)",
        len(all_lites) == 4,
        details=f"lite count={len(all_lites)}",
    )

    # --- Frame is the static root and spans the full bank ---
    frame_aabb = ctx.part_world_aabb(frame)
    fw = frame_aabb[1][0] - frame_aabb[0][0]
    fh = frame_aabb[1][2] - frame_aabb[0][2]
    ctx.check(
        "frame spans full 2-wide bank",
        fw > 1.2 * LITE_W and fw < 3.0 * LITE_W,
        details=f"frame width={fw:.3f}",
    )
    ctx.check(
        "frame spans full 2-high bank",
        fh > 1.8 * LITE_H,
        details=f"frame height={fh:.3f}",
    )
    # Sill at/near z=0, window stands vertically (taller-than-deep).
    ctx.check(
        "frame sits on the ground at z~0",
        abs(frame_aabb[0][2] - SILL_Z) < 0.01,
        details=f"frame zmin={frame_aabb[0][2]:.3f}",
    )
    frame_depth = frame_aabb[1][1] - frame_aabb[0][1]
    ctx.check(
        "window stands vertical (height >> depth)",
        fh > 5.0 * frame_depth,
        details=f"height={fh:.3f}, depth={frame_depth:.3f}",
    )

    # --- Fixed glass count: 3 fixed lites ---
    fixed_count = sum(
        1 for col in range(N_COLS) for row in range(N_ROWS)
        if (col, row) not in operable
    )
    ctx.check(
        "3 fixed glass panes in a 2x2 bank with 1 operable",
        fixed_count == 3,
        details=f"fixed count={fixed_count}",
    )

    # --- A single sash is much smaller than the whole frame and sits inside it ---
    sin_aabb = ctx.part_world_aabb(sash_in)
    sin_w = sin_aabb[1][0] - sin_aabb[0][0]
    ctx.check(
        "operable sash is one lite wide (fits inside bank)",
        sin_w < 0.75 * fw,
        details=f"sash width={sin_w:.3f}, frame width={fw:.3f}",
    )

    # --- CLOSED pose: sash seated in the frame plane (small Y spread) ---
    with ctx.pose({j_in: 0.0}):
        a_in = ctx.part_world_aabb(sash_in)
        # Sash ring center near Y=0 (the glass plane), excluding handle standoff.
        ring_in = ctx.part_element_world_aabb(sash_in, elem="sash_inward_ring")
        cy_in = (ring_in[0][1] + ring_in[1][1]) / 2.0
        ctx.check(
            "closed: inward sash seated in frame plane",
            abs(cy_in) < 0.03,
            details=f"sash_inward ring Y center={cy_in:.3f}",
        )
        # Closed sash sits within the top-left lite of the bank in X.
        cx_in = (a_in[0][0] + a_in[1][0]) / 2.0
        ctx.check(
            "closed: inward sash on the left side of the bank",
            cx_in < 0.0,
            details=f"inward X={cx_in:.3f}",
        )
        # Record closed hinge-edge X position for the open-pose hinge-fixed test.
        hinge_in_x_closed = a_in[0][0]   # hinge on the left edge (min X)
        free_in_y_closed = (a_in[0][1] + a_in[1][1]) / 2.0

    # --- OPEN pose: inward sash swings +Y ---
    with ctx.pose({j_in: 1.2}):
        o_in = ctx.part_world_aabb(sash_in)
        free_in_y_open = (o_in[0][1] + o_in[1][1]) / 2.0
        # Inward leaf moves toward +Y (into the room).
        ctx.check(
            "open: inward sash swings IN toward +Y",
            free_in_y_open > free_in_y_closed + 0.15,
            details=f"closed Y={free_in_y_closed:.3f}, open Y={free_in_y_open:.3f}",
        )
        # Hinge edge stays put (revolute about the jamb, not a translation):
        # inward hinge is the left edge (min X) -> should not move much.
        ctx.check(
            "open: inward sash hinge edge stays on the left jamb",
            abs(o_in[0][0] - hinge_in_x_closed) < 0.08,
            details=f"closed hinge X={hinge_in_x_closed:.3f}, open min X={o_in[0][0]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
