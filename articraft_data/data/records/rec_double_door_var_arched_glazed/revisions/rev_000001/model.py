from __future__ import annotations

# Aluminum storefront double door — arched-top variant
# Two narrow-stile leaves with semicircular arched tops that together complete
# one round-arched head. Each leaf holds a large single glass pane extending
# into the arch, with a diagonal tubular push-bar handle.
#
# Articraft brief:
# - Object: arched aluminum storefront double door, ~1.70 m wide, peak ~2.35 m.
# - Root/support: light anodized-aluminum frame (two side jambs + arched header).
# - Parts: door_frame (root), door_leaf_0, door_leaf_1.
#   Each leaf: arched profile (rectangle + quarter-circle top), large glass pane
#   following the arch, diagonal push-bar in the rectangular portion.
# - Articulations: frame_to_door_leaf_0 / frame_to_door_leaf_1, REVOLUTE,
#   vertical hinge axes at the outer jamb faces, leaves swing outward (+Y).
# - The two leaf arches share one circle centered at the opening midpoint,
#   so the leaf tops together complete the full semicircular arch head.
# - World axes: X=width, Y=depth/swing, Z=height. Base at z=0. UP is +Z.

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Dimensions (all in meters)
# ---------------------------------------------------------------------------
OPENING_W = 1.70
FRAME_FACE = 0.06       # visible frame width (side jambs in X, header band)
FRAME_DEPTH = 0.10      # frame depth along Y

CLEAR_W = OPENING_W - 2.0 * FRAME_FACE   # 1.58 m clear between jambs
ARCH_R = CLEAR_W / 2.0                   # 0.79 m semicircular arch radius
SPRING_H = 1.50                          # spring-line height (rect → arch)

CENTER_GAP = 0.010       # gap between meeting stiles when closed
LEAF_W = (CLEAR_W - CENTER_GAP) / 2.0   # ~0.785 m each leaf
LEAF_T = 0.045           # leaf thickness along Y

STILE_W = 0.05           # narrow-stile vertical rails
BOT_RAIL = 0.16          # taller kick rail

GLASS_T = 0.010          # glass pane thickness

BAR_R = 0.014            # push-bar tube radius
STANDOFF_LEN = 0.05      # bar proud of face surface

# Total heights
LEAF_PEAK_H = SPRING_H + ARCH_R           # 2.29 m leaf peak
FRAME_PEAK_H = SPRING_H + ARCH_R + FRAME_FACE  # 2.35 m frame peak

ALU_RGBA = (0.82, 0.83, 0.84, 1.0)
GLASS_RGBA = (0.62, 0.72, 0.78, 0.45)


# ---------------------------------------------------------------------------
# Parametric arched-leaf geometry helpers (CadQuery)
# ---------------------------------------------------------------------------

def _arch_profile_midpoint(center_x, center_y, radius, start_angle, end_angle):
    """Return the midpoint of an arc for CadQuery threePointArc."""
    mid_angle = (start_angle + end_angle) / 2.0
    return (center_x + radius * math.cos(mid_angle),
            center_y + radius * math.sin(mid_angle))


def _arched_leaf_frame(sign: float) -> cq.Workplane:
    """Build the arched leaf frame solid (stiles + bottom rail + arch top,
    with glass opening cut through).

    Profile drawn in CadQuery XY plane (X=width, Y=height), extruded along Z,
    then rotated so that profile-Y → world-Z (height) and extrusion-Z → world-Y
    (depth/thickness).

    In local frame (sign=+1): hinge edge at X=0, meeting edge near X=ARCH_R.
    The arch semicircle center is at (sign*ARCH_R, SPRING_H) with radius ARCH_R.
    When the part is placed by the articulation, this center lands at world X=0
    (the opening midpoint), matching the frame inner arch.
    """
    arch_cx = sign * ARCH_R

    # --- Outer profile ---
    # Bottom-left (hinge), bottom-right (meeting), peak, arch back to spring.
    p_bl = (0.0, 0.0)
    p_br = (sign * ARCH_R, 0.0)
    p_peak = (sign * ARCH_R, SPRING_H + ARCH_R)
    p_spring = (0.0, SPRING_H)

    # Outer arc midpoint (quarter circle from peak to spring point).
    # sign=+1: arc from 90° to 180°, mid at 135°
    # sign=-1: arc from 90° to 0°, mid at 45°
    if sign > 0:
        outer_mid = _arch_profile_midpoint(arch_cx, SPRING_H, ARCH_R,
                                           math.pi / 2, math.pi)
    else:
        outer_mid = _arch_profile_midpoint(arch_cx, SPRING_H, ARCH_R,
                                           math.pi / 2, 0.0)

    outer_wp = (
        cq.Workplane("XY")
        .moveTo(*p_bl)
        .lineTo(*p_br)
        .lineTo(*p_peak)
        .threePointArc(outer_mid, p_spring)
        .close()
    )
    outer_solid = outer_wp.extrude(LEAF_T)

    # --- Glass opening (inset profile) ---
    inner_r = ARCH_R - STILE_W
    gi_x = sign * STILE_W
    go_x = sign * (ARCH_R - STILE_W)
    g_bot = BOT_RAIL

    # Meeting side glass top: on inner arch at x = go_x.
    # (go_x - arch_cx)^2 + (z - SPRING_H)^2 = inner_r^2
    # STILE_W^2 + (z - SPRING_H)^2 = inner_r^2
    g_meeting_top = SPRING_H + math.sqrt(max(0.0, inner_r ** 2 - STILE_W ** 2))

    # Hinge side glass top: on inner arch at x = gi_x.
    # (gi_x - arch_cx)^2 = (sign*(STILE_W - ARCH_R))^2 = inner_r^2
    # So z = SPRING_H exactly.
    g_hinge_top = SPRING_H

    g_bl = (gi_x, g_bot)
    g_br = (go_x, g_bot)
    g_tr = (go_x, g_meeting_top)
    g_spring = (gi_x, g_hinge_top)

    # Inner arc midpoint.
    # sign=+1: arc from ~94° to 180° (meeting top to hinge spring)
    # sign=-1: arc from ~86° to 0°
    if sign > 0:
        inner_start_angle = math.acos(max(-1.0, min(1.0, -STILE_W / inner_r)))
        inner_mid = _arch_profile_midpoint(arch_cx, SPRING_H, inner_r,
                                           inner_start_angle, math.pi)
    else:
        inner_start_angle = math.acos(max(-1.0, min(1.0, STILE_W / inner_r)))
        inner_mid = _arch_profile_midpoint(arch_cx, SPRING_H, inner_r,
                                           inner_start_angle, 0.0)

    glass_cut = (
        cq.Workplane("XY")
        .moveTo(*g_bl)
        .lineTo(*g_br)
        .lineTo(*g_tr)
        .threePointArc(inner_mid, g_spring)
        .close()
        .extrude(LEAF_T + 0.02)  # oversized for clean boolean cut
    )

    # Subtract glass opening from outer frame.
    frame_solid = outer_solid.cut(glass_cut)

    # Rotate: profile-Y → world-Z, extrusion-Z → world-Y.
    # rotate 90° about X: (x,y,z) → (x, -z, y)
    # Extrusion Z ∈ [0, LEAF_T] → Y ∈ [0, -LEAF_T]; translate to center.
    frame_solid = (
        frame_solid
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0, LEAF_T / 2.0, 0))
    )
    return frame_solid


def _arched_glass_pane(sign: float) -> cq.Workplane:
    """Build the arched glass pane solid (follows the inner arch profile)."""
    arch_cx = sign * ARCH_R
    inner_r = ARCH_R - STILE_W

    gi_x = sign * STILE_W
    go_x = sign * (ARCH_R - STILE_W)
    g_bot = BOT_RAIL
    g_meeting_top = SPRING_H + math.sqrt(max(0.0, inner_r ** 2 - STILE_W ** 2))
    g_hinge_top = SPRING_H

    g_bl = (gi_x, g_bot)
    g_br = (go_x, g_bot)
    g_tr = (go_x, g_meeting_top)
    g_spring = (gi_x, g_hinge_top)

    if sign > 0:
        inner_start_angle = math.acos(max(-1.0, min(1.0, -STILE_W / inner_r)))
        inner_mid = _arch_profile_midpoint(arch_cx, SPRING_H, inner_r,
                                           inner_start_angle, math.pi)
    else:
        inner_start_angle = math.acos(max(-1.0, min(1.0, STILE_W / inner_r)))
        inner_mid = _arch_profile_midpoint(arch_cx, SPRING_H, inner_r,
                                           inner_start_angle, 0.0)

    glass = (
        cq.Workplane("XY")
        .moveTo(*g_bl)
        .lineTo(*g_br)
        .lineTo(*g_tr)
        .threePointArc(inner_mid, g_spring)
        .close()
        .extrude(GLASS_T)
    )

    # Same rotation as frame.
    glass = (
        glass
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0, GLASS_T / 2.0, 0))
    )
    return glass


def _arched_frame_header() -> cq.Workplane:
    """Build the arched frame header: a semicircular ring band.

    Uses boolean subtraction: outer half-cylinder minus inner half-cylinder.
    The profile is in CadQuery XY (X=width, Y=height above spring line),
    extruded along Z (depth), then rotated to world orientation and positioned
    at the spring line.
    """
    r_outer = ARCH_R + FRAME_FACE
    r_inner = ARCH_R

    # Outer full cylinder (extruded circle).
    outer = cq.Workplane("XY").circle(r_outer).extrude(FRAME_DEPTH)

    # Cut inner cylinder to create annular ring.
    outer = outer.faces(">Z").workplane().circle(r_inner).cutThruAll()

    # Cut away the bottom half (y < 0 in profile space) to leave only
    # the upper semicircular band.
    bottom_cutter = (
        cq.Workplane("XY")
        .box(2.0 * r_outer + 0.1, 2.0 * r_outer, FRAME_DEPTH + 0.1)
        .translate((0.0, -r_outer, FRAME_DEPTH / 2.0))
    )
    header = outer.cut(bottom_cutter)

    # Rotate: profile-Y → world-Z, extrusion-Z → world-Y, position at spring line.
    header = (
        header
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0, FRAME_DEPTH / 2.0, SPRING_H))
    )
    return header


def _push_bar_mesh(sign: float, part_name: str) -> object:
    """Diagonal push-bar handle using tube_from_spline_points.

    The bar runs diagonally from lower-outer to upper-inner within the
    rectangular portion of the leaf (below the spring line). Standoff tips
    embed into the leaf body for spatial connectivity.
    """
    front_y = -(LEAF_T / 2.0 + STANDOFF_LEN)
    embed_y = 0.0  # center of leaf thickness

    low_x = sign * (STILE_W + 0.06)
    low_z = BOT_RAIL + 0.08
    high_x = sign * (ARCH_R - STILE_W - 0.04)
    rect_glass_h = SPRING_H - BOT_RAIL
    high_z = BOT_RAIL + rect_glass_h * 0.62

    pts = [
        (low_x, embed_y, low_z),
        (low_x, front_y, low_z),
        (low_x + (high_x - low_x) * 0.33, front_y, low_z + (high_z - low_z) * 0.33),
        (low_x + (high_x - low_x) * 0.67, front_y, low_z + (high_z - low_z) * 0.67),
        (high_x, front_y, high_z),
        (high_x, embed_y, high_z),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            pts,
            radius=BAR_R,
            samples_per_segment=10,
            radial_segments=14,
            cap_ends=True,
        ),
        f"{part_name}_push_bar",
    )


def _add_leaf_visuals(
    model: ArticulatedObject,
    part_name: str,
    sign: float,
    mat_alu: str,
    mat_glass: str,
) -> None:
    """Build and register all visuals for one arched leaf."""
    leaf = model.part(part_name)

    leaf.visual(
        mesh_from_cadquery(_arched_leaf_frame(sign), f"{part_name}_frame"),
        material=mat_alu,
        name=f"{part_name}_frame",
    )
    leaf.visual(
        mesh_from_cadquery(_arched_glass_pane(sign), f"{part_name}_glass"),
        material=mat_glass,
        name=f"{part_name}_glass",
    )
    bar_mesh = _push_bar_mesh(sign, part_name)
    leaf.visual(bar_mesh, origin=Origin(), material=mat_alu,
                name=f"{part_name}_push_bar")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminum_storefront_doors_arched")

    mat_alu = model.material("anodized_aluminum", rgba=ALU_RGBA)
    mat_glass = model.material("storefront_glass", rgba=GLASS_RGBA)

    # --- Fixed anodized-aluminum frame (root) ---
    frame = model.part("door_frame")
    half_open = OPENING_W / 2.0

    # Side jambs: from z=0 to z=SPRING_H (arch header takes over above).
    for i, side_sign in enumerate((+1.0, -1.0)):
        jamb_name = f"jamb_{i}"
        jamb_cx = side_sign * (half_open - FRAME_FACE / 2.0)
        frame.visual(
            mesh_from_cadquery(
                cq.Workplane("XY")
                .box(FRAME_FACE, FRAME_DEPTH, SPRING_H)
                .translate((jamb_cx, 0.0, SPRING_H / 2.0)),
                jamb_name,
            ),
            material=mat_alu, name=jamb_name,
        )

    # Arched header: semicircular ring band above the spring line.
    frame.visual(
        mesh_from_cadquery(_arched_frame_header(), "arch_header"),
        material=mat_alu, name="arch_header",
    )

    # --- Two arched glass leaves ---
    _add_leaf_visuals(model, "door_leaf_0", sign=+1.0,
                      mat_alu=mat_alu, mat_glass=mat_glass)
    _add_leaf_visuals(model, "door_leaf_1", sign=-1.0,
                      mat_alu=mat_alu, mat_glass=mat_glass)

    # Hinge pivot positions: at the inner face of each side jamb.
    pivot0_x = -(half_open - FRAME_FACE)
    pivot1_x = half_open - FRAME_FACE

    model.articulation(
        "frame_to_door_leaf_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_leaf_0"),
        origin=Origin(xyz=(pivot0_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0,
                                   lower=0.0, upper=1.4),
    )
    model.articulation(
        "frame_to_door_leaf_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=model.get_part("door_leaf_1"),
        origin=Origin(xyz=(pivot1_x, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0,
                                   lower=0.0, upper=1.4),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("door_frame")
    door0 = object_model.get_part("door_leaf_0")
    door1 = object_model.get_part("door_leaf_1")
    hinge0 = object_model.get_articulation("frame_to_door_leaf_0")
    hinge1 = object_model.get_articulation("frame_to_door_leaf_1")

    # --- Intentional embeddings ---
    for d in (door0, door1):
        ctx.allow_overlap(
            d, d,
            elem_a=f"{d.name}_glass",
            elem_b=f"{d.name}_frame",
            reason="Glass pane is glazed (lapped) into the stile/rail rabbet.",
        )
        ctx.allow_overlap(
            d, d,
            elem_a=f"{d.name}_push_bar",
            elem_b=f"{d.name}_frame",
            reason="Push-bar standoffs embed into the leaf face where fastened.",
        )
        ctx.allow_overlap(
            d, d,
            elem_a=f"{d.name}_push_bar",
            elem_b=f"{d.name}_glass",
            reason="Bar standoffs touch the glass pane face at their base.",
        )

    # The arch header ring and leaf frames share the same semicircular
    # boundary (same circle). Their convex collision hulls interpenetrate
    # even though the visual geometry is correctly non-overlapping:
    # the header ring is OUTSIDE the arch circle and the leaf is INSIDE.
    for d in (door0, door1):
        ctx.allow_overlap(
            frame, d,
            elem_a="arch_header",
            elem_b=f"{d.name}_frame",
            reason=("Leaf arch top shares the same semicircle as the frame "
                    "inner arch; convex collision hulls overlap but the "
                    "ring-shaped header is outside and the leaf is inside."),
        )
    # Proof: each leaf fits within the frame opening in XY at the spring line.
    for d in (door0, door1):
        ctx.expect_within(
            d, frame, axes="x",
            margin=0.01,
            name=f"{d.name}_within_frame_opening_x",
        )

    # --- Each leaf has arched profile: peak extends above spring line ---
    for d in (door0, door1):
        fa = ctx.part_element_world_aabb(d, elem=f"{d.name}_frame")
        assert fa is not None
        mn, mx = fa
        w = float(mx[0] - mn[0])
        h = float(mx[2] - mn[2])
        peak_z = float(mx[2])
        ctx.check(f"{d.name}_width_plausible",
                  0.70 <= w <= 0.85, details=f"w={w:.3f}")
        ctx.check(f"{d.name}_arch_peak_above_spring",
                  peak_z > SPRING_H + ARCH_R * 0.9,
                  details=f"peak_z={peak_z:.3f}, expected>{SPRING_H + ARCH_R * 0.9:.3f}")
        ctx.check(f"{d.name}_height_plausible",
                  2.10 <= h <= 2.50, details=f"h={h:.3f}")
        ctx.check(f"{d.name}_base_at_z0", mn[2] <= 0.01,
                  details=f"zmin={mn[2]:.4f}")

    # --- Leaf mirror-symmetry: same frame dimensions ---
    fa0 = ctx.part_element_world_aabb(door0, elem="door_leaf_0_frame")
    fa1 = ctx.part_element_world_aabb(door1, elem="door_leaf_1_frame")
    assert fa0 is not None and fa1 is not None
    w0 = fa0[1][0] - fa0[0][0]
    w1 = fa1[1][0] - fa1[0][0]
    h0 = fa0[1][2] - fa0[0][2]
    h1 = fa1[1][2] - fa1[0][2]
    peak0 = fa0[1][2]
    peak1 = fa1[1][2]
    ctx.check("leaf_sym_width", abs(w0 - w1) < 0.005,
              details=f"w0={w0:.4f} w1={w1:.4f}")
    ctx.check("leaf_sym_height", abs(h0 - h1) < 0.005,
              details=f"h0={h0:.4f} h1={h1:.4f}")
    ctx.check("arch_peaks_match", abs(peak0 - peak1) < 0.005,
              details=f"peak0={peak0:.4f} peak1={peak1:.4f}")

    # --- Each leaf has a large glass pane extending into the arch ---
    for d in (door0, door1):
        ga = ctx.part_element_world_aabb(d, elem=f"{d.name}_glass")
        assert ga is not None
        gw = float(ga[1][0] - ga[0][0])
        gh = float(ga[1][2] - ga[0][2])
        glass_top = float(ga[1][2])
        ctx.check(
            f"{d.name}_glass_large",
            gh > 1.0 and gw > 0.50,
            details=f"glass w={gw:.3f} h={gh:.3f}",
        )
        ctx.check(
            f"{d.name}_glass_into_arch",
            glass_top > SPRING_H + 0.20,
            details=f"glass_top={glass_top:.3f}, expected>{SPRING_H + 0.20:.3f}",
        )

    # Glass symmetry.
    ga0 = ctx.part_element_world_aabb(door0, elem="door_leaf_0_glass")
    ga1 = ctx.part_element_world_aabb(door1, elem="door_leaf_1_glass")
    assert ga0 is not None and ga1 is not None
    gw0 = ga0[1][0] - ga0[0][0]
    gw1 = ga1[1][0] - ga1[0][0]
    gh0 = ga0[1][2] - ga0[0][2]
    gh1 = ga1[1][2] - ga1[0][2]
    ctx.check("glass_sym_width", abs(gw0 - gw1) < 0.005,
              details=f"gw0={gw0:.4f} gw1={gw1:.4f}")
    ctx.check("glass_sym_height", abs(gh0 - gh1) < 0.005,
              details=f"gh0={gh0:.4f} gh1={gh1:.4f}")

    # --- Each leaf has a diagonal push bar ---
    for d in (door0, door1):
        ba = ctx.part_element_world_aabb(d, elem=f"{d.name}_push_bar")
        assert ba is not None
        bdx = float(ba[1][0] - ba[0][0])
        bdz = float(ba[1][2] - ba[0][2])
        ctx.check(
            f"{d.name}_bar_diagonal",
            bdx > 0.25 and bdz > 0.25,
            details=f"bar dx={bdx:.3f} dz={bdz:.3f}",
        )
        ctx.check(
            f"{d.name}_bar_proud_of_face",
            float(ba[0][1]) < -(LEAF_T / 2.0 + 0.02),
            details=f"bar minY={float(ba[0][1]):.3f}",
        )

    # Bar symmetry.
    ba0 = ctx.part_element_world_aabb(door0, elem="door_leaf_0_push_bar")
    ba1 = ctx.part_element_world_aabb(door1, elem="door_leaf_1_push_bar")
    assert ba0 is not None and ba1 is not None
    bdx0 = ba0[1][0] - ba0[0][0]
    bdx1 = ba1[1][0] - ba1[0][0]
    bdz0 = ba0[1][2] - ba0[0][2]
    bdz1 = ba1[1][2] - ba1[0][2]
    ctx.check("bar_sym_dx", abs(bdx0 - bdx1) < 0.03,
              details=f"bdx0={bdx0:.4f} bdx1={bdx1:.4f}")
    ctx.check("bar_sym_dz", abs(bdz0 - bdz1) < 0.03,
              details=f"bdz0={bdz0:.4f} bdz1={bdz1:.4f}")

    # --- Frame has arched header above spring line ---
    ha = ctx.part_element_world_aabb(frame, elem="arch_header")
    assert ha is not None
    ctx.check("arch_header_above_spring",
              float(ha[0][2]) >= SPRING_H - 0.01,
              details=f"header_zmin={float(ha[0][2]):.3f}")
    ctx.check("arch_header_peak",
              float(ha[1][2]) > SPRING_H + ARCH_R * 0.9,
              details=f"header_zmax={float(ha[1][2]):.3f}")

    # --- Closed pose: leaves meet at center ---
    with ctx.pose({hinge0: 0.0, hinge1: 0.0}):
        ctx.expect_gap(
            door1, door0, axis="x",
            min_gap=0.0, max_gap=0.05,
            name="leaves_meet_at_center",
        )
        ctx.expect_contact(door0, frame, name="door_leaf_0_on_frame")
        ctx.expect_contact(door1, frame, name="door_leaf_1_on_frame")

        b0 = ctx.part_element_world_aabb(door0, elem="door_leaf_0_push_bar")
        b1 = ctx.part_element_world_aabb(door1, elem="door_leaf_1_push_bar")
        assert b0 is not None and b1 is not None
        ctx.check(
            "bars_form_V_near_center",
            abs(float(b0[1][0]) - float(b1[0][0])) < 0.30,
            details=f"bar0 maxX={float(b0[1][0]):.3f} bar1 minX={float(b1[0][0]):.3f}",
        )

    # --- Open pose: both leaves swing outward ---
    with ctx.pose({hinge0: 0.0, hinge1: 0.0}):
        c0 = ctx.part_world_aabb(door0)
        c1 = ctx.part_world_aabb(door1)
    with ctx.pose({hinge0: 1.2, hinge1: 1.2}):
        o0 = ctx.part_world_aabb(door0)
        o1 = ctx.part_world_aabb(door1)
        assert o0 and o1 and c0 and c1
        ctx.check("door_leaf_0_swings_outward",
                  float(o0[1][1]) > float(c0[1][1]) + 0.3,
                  details=f"closed={float(c0[1][1]):.3f} open={float(o0[1][1]):.3f}")
        ctx.check("door_leaf_1_swings_outward",
                  float(o1[1][1]) > float(c1[1][1]) + 0.3,
                  details=f"closed={float(c1[1][1]):.3f} open={float(o1[1][1]):.3f}")

    return ctx.report()


object_model = build_object_model()
