from __future__ import annotations

# Variant 19: Two-panel vertical sliding window (double-hung style), white vinyl
# frame with deep track grooves, rubber gasket strips, and insect screen.
#
# Structure:
#   frame (root): thick vinyl frame with deep track grooves cut into head/sill
#   upper_sash: FIXED in rear track, vinyl ring + glass + rubber gaskets
#   lower_sash: PRISMATIC vertical (+Z), slides UP to open; carries latch
#   screen: FIXED in outermost track; aluminum frame + perforated mesh panel
#
# Coordinate convention:
#   +Z up, window stands vertically
#   width -> X, height -> Z (sill near z=0), depth -> Y
#   +Y room side, -Y exterior
#   q=0 CLOSED, positive q slides lower sash UP (+Z)

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
# Dimensions (meters)
# ---------------------------------------------------------------------------

TOTAL_W = 1.52
TOTAL_H = 1.72

FRAME_FACE = 0.085
FRAME_DEPTH = 0.140

SASH_FACE = 0.065
SASH_DEPTH = 0.050
GLASS_T = 0.008
REBATE = 0.005
MEETING_OVERLAP = 0.035

# Track groove channels (cut into head/sill)
GROOVE_W = 0.022
GROOVE_DEPTH = 0.022

# Rubber gasket strips
GASKET_FACE = 0.006
GASKET_T = 0.004

# Insect screen
SCREEN_FRAME_FACE = 0.025
SCREEN_FRAME_DEPTH = 0.018

# Latch hardware
LATCH_W = 0.040
LATCH_H = 0.025
LATCH_T = 0.010
LEVER_LEN = 0.035
LEVER_R = 0.005

# Y layout: exterior (-Y) to interior (+Y)
SCREEN_Y = -0.055
UPPER_SASH_Y = -0.015
LOWER_SASH_Y = 0.035

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

VINYL_RGBA = (0.94, 0.95, 0.96, 1.0)
GLASS_RGBA = (0.52, 0.60, 0.66, 0.30)
RUBBER_RGBA = (0.12, 0.12, 0.12, 1.0)
SCREEN_RGBA = (0.35, 0.38, 0.40, 0.55)
ALUMINUM_RGBA = (0.72, 0.74, 0.76, 1.0)
METAL_RGBA = (0.74, 0.76, 0.79, 1.0)
GROOVE_RGBA = (0.80, 0.82, 0.84, 1.0)

# ---------------------------------------------------------------------------
# Derived layout
# ---------------------------------------------------------------------------

HALF_W = TOTAL_W / 2.0
INNER_X0 = -HALF_W + FRAME_FACE
INNER_X1 = HALF_W - FRAME_FACE
INNER_Z0 = FRAME_FACE
INNER_Z1 = TOTAL_H - FRAME_FACE
INNER_W = INNER_X1 - INNER_X0
INNER_H = INNER_Z1 - INNER_Z0

# Each sash: full inner width, half inner height + meeting overlap
SASH_OUTER_W = INNER_W
SASH_H = (INNER_H + MEETING_OVERLAP) / 2.0
GLASS_W = SASH_OUTER_W - 2 * SASH_FACE
GLASS_H = SASH_H - 2 * SASH_FACE

UPPER_CZ = INNER_Z1 - SASH_H / 2.0
LOWER_CZ = INNER_Z0 + SASH_H / 2.0

# Screen (slightly proud of opening to ensure frame contact)
SCREEN_OUTER_W = INNER_W + 0.002
SCREEN_OUTER_H = INNER_H + 0.002
SCREEN_MESH_W = SCREEN_OUTER_W - 2 * SCREEN_FRAME_FACE
SCREEN_MESH_H = SCREEN_OUTER_H - 2 * SCREEN_FRAME_FACE
SCREEN_CZ = (INNER_Z0 + INNER_Z1) / 2.0

SLIDE_TRAVEL = SASH_H * 0.85


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _slab(x0, x1, z0, z1, yc, d):
    """Axis-aligned box in X-Z plane, centered on yc with depth d along Y."""
    return (
        cq.Workplane("XY")
        .transformed(offset=((x0 + x1) / 2.0, yc, (z0 + z1) / 2.0))
        .box(x1 - x0, d, z1 - z0)
    )


def _frame_shape():
    """Outer frame with hollow opening and deep track grooves on head/sill."""
    outer = _slab(-HALF_W, HALF_W, 0.0, TOTAL_H, 0.0, FRAME_DEPTH)
    opening = _slab(INNER_X0, INNER_X1, INNER_Z0, INNER_Z1, 0.0, FRAME_DEPTH + 0.02)
    f = outer.cut(opening)
    # Wide track groove spanning both sash Y tracks, cut into sill
    y_lo = min(UPPER_SASH_Y, LOWER_SASH_Y) - GROOVE_W / 2.0
    y_hi = max(UPPER_SASH_Y, LOWER_SASH_Y) + GROOVE_W / 2.0
    y_mid = (y_lo + y_hi) / 2.0
    y_span = y_hi - y_lo
    sill_g = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, y_mid, INNER_Z0 - GROOVE_DEPTH / 2.0))
        .box(INNER_W, y_span, GROOVE_DEPTH)
    )
    head_g = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, y_mid, INNER_Z1 + GROOVE_DEPTH / 2.0))
        .box(INNER_W, y_span, GROOVE_DEPTH)
    )
    f = f.cut(sill_g).cut(head_g)
    return f


def _sash_shape():
    """Sash ring (hollow frame) in local frame centered at origin."""
    ow, oh = SASH_OUTER_W, SASH_H
    gw, gh = GLASS_W, GLASS_H
    outer = _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, SASH_DEPTH)
    hole = _slab(-gw / 2, gw / 2, -gh / 2, gh / 2, 0.0, SASH_DEPTH + 0.02)
    return outer.cut(hole)


def _glass_shape():
    """Glass pane slightly larger than opening (rebated under sash lip)."""
    w = GLASS_W + 2 * REBATE
    h = GLASS_H + 2 * REBATE
    return _slab(-w / 2, w / 2, -h / 2, h / 2, 0.0, GLASS_T)


def _gasket_shape():
    """Thin rubber frame ring around glass perimeter."""
    ow = GLASS_W + 2 * GASKET_FACE
    oh = GLASS_H + 2 * GASKET_FACE
    iw = GLASS_W
    ih = GLASS_H
    outer = _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, GASKET_T)
    inner = _slab(-iw / 2, iw / 2, -ih / 2, ih / 2, 0.0, GASKET_T + 0.002)
    return outer.cut(inner)


def _screen_frame_shape():
    """Thin hollow aluminum screen frame in local frame."""
    ow, oh = SCREEN_OUTER_W, SCREEN_OUTER_H
    iw = ow - 2 * SCREEN_FRAME_FACE
    ih = oh - 2 * SCREEN_FRAME_FACE
    outer = _slab(-ow / 2, ow / 2, -oh / 2, oh / 2, 0.0, SCREEN_FRAME_DEPTH)
    inner = _slab(-iw / 2, iw / 2, -ih / 2, ih / 2, 0.0, SCREEN_FRAME_DEPTH + 0.002)
    return outer.cut(inner)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_sliding_window")
    model.material("vinyl", rgba=VINYL_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("rubber", rgba=RUBBER_RGBA)
    model.material("screen_mesh", rgba=SCREEN_RGBA)
    model.material("aluminum", rgba=ALUMINUM_RGBA)
    model.material("metal", rgba=METAL_RGBA)
    model.material("groove", rgba=GROOVE_RGBA)

    # --- Frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame_shell"),
        material="vinyl", name="frame_shell",
    )
    # Track groove markers (visible channel fills in the deep head/sill grooves)
    # Slightly proud of groove channel to ensure contact with frame_shell
    groove_fill_t = GROOVE_DEPTH + 0.004
    groove_fill_w = GROOVE_W + 0.004
    groove_fill_x = INNER_W * 0.92
    for sy, tag in [(UPPER_SASH_Y, "rear"), (LOWER_SASH_Y, "front")]:
        frame.visual(
            Box((groove_fill_x, groove_fill_w, groove_fill_t)),
            origin=Origin(xyz=(0.0, sy, INNER_Z0 - GROOVE_DEPTH / 2.0)),
            material="groove", name=f"sill_track_{tag}",
        )
        frame.visual(
            Box((groove_fill_x, groove_fill_w, groove_fill_t)),
            origin=Origin(xyz=(0.0, sy, INNER_Z1 + GROOVE_DEPTH / 2.0)),
            material="groove", name=f"head_track_{tag}",
        )

    # --- Upper sash (FIXED, rear track) ---
    upper = model.part("upper_sash")
    upper.visual(
        mesh_from_cadquery(_sash_shape(), "upper_vinyl"),
        material="vinyl", name="upper_vinyl",
    )
    upper.visual(
        mesh_from_cadquery(_glass_shape(), "upper_glass"),
        material="glass", name="upper_glass",
    )
    # Rubber gaskets front and rear of glass
    gasket_dy = GLASS_T / 2.0
    for side, dy in [("front", gasket_dy), ("rear", -gasket_dy)]:
        upper.visual(
            mesh_from_cadquery(_gasket_shape(), f"upper_gasket_{side}"),
            material="rubber", origin=Origin(xyz=(0.0, dy, 0.0)),
            name=f"upper_gasket_{side}",
        )

    # --- Lower sash (PRISMATIC, front track, slides up) ---
    lower = model.part("lower_sash")
    lower.visual(
        mesh_from_cadquery(_sash_shape(), "lower_vinyl"),
        material="vinyl", name="lower_vinyl",
    )
    lower.visual(
        mesh_from_cadquery(_glass_shape(), "lower_glass"),
        material="glass", name="lower_glass",
    )
    for side, dy in [("front", gasket_dy), ("rear", -gasket_dy)]:
        lower.visual(
            mesh_from_cadquery(_gasket_shape(), f"lower_gasket_{side}"),
            material="rubber", origin=Origin(xyz=(0.0, dy, 0.0)),
            name=f"lower_gasket_{side}",
        )

    # Latch on lower sash meeting rail (top rail)
    rail_z = SASH_H / 2.0 - SASH_FACE / 2.0
    face_y = SASH_DEPTH / 2.0
    lower.visual(
        Box((LATCH_W, LATCH_T, LATCH_H)),
        origin=Origin(xyz=(0.0, face_y + LATCH_T / 2.0 - 0.001, rail_z)),
        material="metal", name="lower_latch_plate",
    )
    lower.visual(
        Cylinder(radius=LEVER_R, length=LEVER_LEN),
        origin=Origin(xyz=(0.0, face_y + LATCH_T + LEVER_LEN / 2.0 - 0.001, rail_z),
                      rpy=(1.5708, 0.0, 0.0)),
        material="metal", name="lower_latch_lever",
    )

    # --- Screen panel (FIXED, outermost track) ---
    screen = model.part("screen")
    screen.visual(
        mesh_from_cadquery(_screen_frame_shape(), "screen_frame"),
        material="aluminum", name="screen_frame",
    )
    # Screen mesh panel: thin translucent panel reads as insect screen
    # Sized to fill the screen frame inner opening (contact with frame edges)
    screen.visual(
        Box((SCREEN_MESH_W + 0.002, 0.002, SCREEN_MESH_H + 0.002)),
        material="screen_mesh",
        name="screen_mesh",
    )

    # --- Articulations ---
    model.articulation(
        "frame_to_upper_sash", ArticulationType.FIXED,
        parent="frame", child="upper_sash",
        origin=Origin(xyz=(0.0, UPPER_SASH_Y, UPPER_CZ)),
    )
    model.articulation(
        "frame_to_lower_sash", ArticulationType.PRISMATIC,
        parent="frame", child="lower_sash",
        origin=Origin(xyz=(0.0, LOWER_SASH_Y, LOWER_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5,
                                   lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "frame_to_screen", ArticulationType.FIXED,
        parent="frame", child="screen",
        origin=Origin(xyz=(0.0, SCREEN_Y, SCREEN_CZ)),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    upper = object_model.get_part("upper_sash")
    lower = object_model.get_part("lower_sash")
    screen = object_model.get_part("screen")
    slide = object_model.get_articulation("frame_to_lower_sash")

    # --- Overlap allowances ---
    for prefix, nm in [("upper", "upper_sash"), ("lower", "lower_sash")]:
        # Glass rebated under sash lip
        ctx.allow_overlap(
            nm, nm, elem_a=f"{prefix}_glass", elem_b=f"{prefix}_vinyl",
            reason="Glass pane rebated under sash lip for captured fit.",
        )
        for side in ("front", "rear"):
            # Gasket wraps glass edge
            ctx.allow_overlap(
                nm, nm,
                elem_a=f"{prefix}_gasket_{side}", elem_b=f"{prefix}_glass",
                reason=f"Rubber gasket wraps the {side} glass edge.",
            )
            # Gasket seated on sash frame
            ctx.allow_overlap(
                nm, nm,
                elem_a=f"{prefix}_gasket_{side}", elem_b=f"{prefix}_vinyl",
                reason=f"Rubber gasket seated against {side} sash frame.",
            )
        # Sash seated in frame track
        ctx.allow_overlap(
            "frame", nm, elem_a="frame_shell", elem_b=f"{prefix}_vinyl",
            reason=f"{nm} ring seated in frame track groove.",
        )
        ctx.allow_overlap(
            "frame", nm, elem_a="frame_shell", elem_b=f"{prefix}_glass",
            reason=f"{nm} glass within frame opening rebate.",
        )
        for side in ("front", "rear"):
            ctx.allow_overlap(
                "frame", nm, elem_a="frame_shell",
                elem_b=f"{prefix}_gasket_{side}",
                reason=f"{nm} gasket within frame opening.",
            )

    # Groove markers inside frame groove channels
    for tag in ("rear", "front"):
        for loc in ("sill", "head"):
            ctx.allow_overlap(
                "frame", "frame",
                elem_a="frame_shell", elem_b=f"{loc}_track_{tag}",
                reason=f"Track groove marker fills the {loc} groove channel.",
            )

    # Latch seated on lower sash
    ctx.allow_overlap(
        "lower_sash", "lower_sash",
        elem_a="lower_latch_plate", elem_b="lower_vinyl",
        reason="Latch keeper plate seated on meeting rail face.",
    )

    # Screen in outer track
    ctx.allow_overlap(
        "frame", "screen", elem_a="frame_shell", elem_b="screen_frame",
        reason="Screen frame seated in outer frame track.",
    )
    ctx.allow_overlap(
        "frame", "screen", elem_a="frame_shell", elem_b="screen_mesh",
        reason="Screen mesh within frame opening.",
    )
    # Screen mesh fills screen frame inner opening
    ctx.allow_overlap(
        "screen", "screen", elem_a="screen_mesh", elem_b="screen_frame",
        reason="Screen mesh panel seated inside the screen frame opening.",
    )

    # --- Closed pose (q=0) ---
    with ctx.pose({slide: 0.0}):
        frame_aabb = ctx.part_world_aabb(frame)
        upper_aabb = ctx.part_world_aabb(upper)
        lower_aabb = ctx.part_world_aabb(lower)
        screen_aabb = ctx.part_world_aabb(screen)

        # Frame spans full window
        ctx.check(
            "frame spans full width",
            (frame_aabb[1][0] - frame_aabb[0][0]) > TOTAL_W - 0.02,
            details=f"frame_w={frame_aabb[1][0] - frame_aabb[0][0]:.3f}",
        )
        ctx.check(
            "sill near z=0",
            abs(frame_aabb[0][2]) < 0.02,
            details=f"zmin={frame_aabb[0][2]:.4f}",
        )

        # Upper sash above lower sash
        upper_cz = (upper_aabb[0][2] + upper_aabb[1][2]) / 2.0
        lower_cz = (lower_aabb[0][2] + lower_aabb[1][2]) / 2.0
        ctx.check(
            "upper sash above lower sash",
            upper_cz > lower_cz + 0.10,
            details=f"upper_z={upper_cz:.3f}, lower_z={lower_cz:.3f}",
        )

        # Both sashes within frame height
        for nm, ab in [("upper", upper_aabb), ("lower", lower_aabb)]:
            ctx.check(
                f"{nm} sash within frame height",
                ab[0][2] > frame_aabb[0][2] - 1e-4 and ab[1][2] < frame_aabb[1][2] + 1e-4,
                details=f"{nm} z=[{ab[0][2]:.3f},{ab[1][2]:.3f}]",
            )

        # Sashes overlap in XZ projection (seated in frame)
        ctx.expect_overlap(upper, frame, axes="xz", min_overlap=0.03,
                           name="upper sash seated in frame opening")
        ctx.expect_overlap(lower, frame, axes="xz", min_overlap=0.03,
                           name="lower sash seated in frame opening")

        # Screen is in outermost Y track (more negative than both sashes)
        screen_cy = (screen_aabb[0][1] + screen_aabb[1][1]) / 2.0
        upper_cy = (upper_aabb[0][1] + upper_aabb[1][1]) / 2.0
        lower_cy = (lower_aabb[0][1] + lower_aabb[1][1]) / 2.0
        ctx.check(
            "screen in outer track (exterior of both sashes)",
            screen_cy < upper_cy - 0.01 and screen_cy < lower_cy - 0.01,
            details=f"screen_y={screen_cy:.3f}, upper_y={upper_cy:.3f}, lower_y={lower_cy:.3f}",
        )

        # Screen within frame bounds
        ctx.expect_within(screen, frame, axes="xz", margin=0.01,
                          name="screen within frame opening")

        # Track grooves exist on sill and head
        for tag in ("rear", "front"):
            sill_z = ctx.part_element_world_aabb(frame, elem=f"sill_track_{tag}")
            head_z = ctx.part_element_world_aabb(frame, elem=f"head_track_{tag}")
            ctx.check(
                f"sill track {tag} below inner opening",
                sill_z[1][2] < INNER_Z0 + 0.005,
                details=f"sill_track_{tag} zmax={sill_z[1][2]:.4f}",
            )
            ctx.check(
                f"head track {tag} above inner opening",
                head_z[0][2] > INNER_Z1 - 0.005,
                details=f"head_track_{tag} zmin={head_z[0][2]:.4f}",
            )

        # Gaskets present around glass on each sash
        for prefix, sash_part in [("upper", upper), ("lower", lower)]:
            for side in ("front", "rear"):
                g_aabb = ctx.part_element_world_aabb(sash_part, elem=f"{prefix}_gasket_{side}")
                gl_aabb = ctx.part_element_world_aabb(sash_part, elem=f"{prefix}_glass")
                ctx.check(
                    f"{prefix} gasket {side} overlaps glass in XZ",
                    g_aabb[0][0] < gl_aabb[1][0] and g_aabb[1][0] > gl_aabb[0][0]
                    and g_aabb[0][2] < gl_aabb[1][2] and g_aabb[1][2] > gl_aabb[0][2],
                    details=f"gasket xz=[{g_aabb[0][0]:.3f}..{g_aabb[1][0]:.3f}, {g_aabb[0][2]:.3f}..{g_aabb[1][2]:.3f}]",
                )

        rest_cz = lower_cz

    # --- Open pose: lower sash slides UP ---
    travel = slide.motion_limits.upper
    with ctx.pose({slide: travel}):
        open_aabb = ctx.part_world_aabb(lower)
        open_cz = (open_aabb[0][2] + open_aabb[1][2]) / 2.0

        # Lower sash moved upward
        ctx.check(
            "lower sash slides upward when opened",
            open_cz > rest_cz + 0.10,
            details=f"rest_z={rest_cz:.3f}, open_z={open_cz:.3f}, travel={travel:.3f}",
        )

        # Vertical movement (no X drift)
        rest_cx = (lower_aabb[0][0] + lower_aabb[1][0]) / 2.0
        open_cx = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
        ctx.check(
            "slide is purely vertical (no X drift)",
            abs(open_cx - rest_cx) < 0.01,
            details=f"rest_x={rest_cx:.3f}, open_x={open_cx:.3f}",
        )

        # Sash retained within frame at full travel
        f_aabb = ctx.part_world_aabb(frame)
        ctx.check(
            "sash retained within frame Z span at full travel",
            open_aabb[0][2] > f_aabb[0][2] - 1e-4
            and open_aabb[1][2] < f_aabb[1][2] + 1e-4,
            details=f"sash z=[{open_aabb[0][2]:.3f},{open_aabb[1][2]:.3f}] "
                    f"frame z=[{f_aabb[0][2]:.3f},{f_aabb[1][2]:.3f}]",
        )

        # Sash still overlaps frame in X (still in track)
        ctx.expect_overlap(lower, frame, axes="x", min_overlap=0.20,
                           name="sash retains horizontal engagement at full travel")

    # --- Joint structure checks ---
    ctx.check(
        "lower sash joint is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "lower sash joint axis is vertical (+Z)",
        abs(slide.axis[2] - 1.0) < 0.01 and abs(slide.axis[0]) < 0.01 and abs(slide.axis[1]) < 0.01,
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "lower sash has positive travel",
        slide.motion_limits.upper is not None and slide.motion_limits.upper > 0.2,
        details=f"upper={slide.motion_limits.upper}",
    )

    return ctx.report()


object_model = build_object_model()
