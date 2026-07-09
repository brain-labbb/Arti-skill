from __future__ import annotations

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

# ── Dimensions ──────────────────────────────────────────────────────
TRAY_RADIUS = 0.065          # 130 mm diameter
BASE_BOTTOM_H = 0.004        # 4 mm base plate
WALL_H = 0.016               # 16 mm compartment wall height
WALL_TOP_Z = BASE_BOTTOM_H + WALL_H   # 20 mm
DIVIDER_T = 0.002            # 2 mm divider walls
RIM_T = 0.003                # 3 mm outer rim
CENTER_POST_R = 0.004        # 4 mm center pivot post radius
CENTER_POST_H = 0.026        # 26 mm post (extends above lid for retention)
LID_T = 0.003                # 3 mm dial lid thickness
LID_R = TRAY_RADIUS - 0.001  # lid clearance inside rim
LID_BOTTOM_Z = WALL_TOP_Z   # lid sits on wall tops
N_SECTORS = 7
SECTOR_ANGLE = 2.0 * math.pi / N_SECTORS   # ≈ 51.43°
WELL_INNER_R = CENTER_POST_R + 0.003        # 7 mm
WELL_OUTER_R = TRAY_RADIUS - RIM_T - 0.001  # 61 mm
WELL_FLOOR_H = 0.001         # 1 mm tinted well floor
DIVIDER_GAP = 0.015          # angular gap (rad) each side of divider
WALL_LEN = TRAY_RADIUS - RIM_T - CENTER_POST_R   # radial wall length
WALL_MID_R = CENTER_POST_R + WALL_LEN / 2.0       # wall midpoint radius


# ── Geometry helpers ────────────────────────────────────────────────

def _annular_sector_solid(inner_r, outer_r, a0, a1, height, n_arc=12):
    """Build an annular-sector (ring-slice) CadQuery solid from Z=0 to Z=height."""
    pts = []
    for i in range(n_arc + 1):
        a = a0 + (a1 - a0) * i / n_arc
        pts.append((outer_r * math.cos(a), outer_r * math.sin(a)))
    for i in range(n_arc + 1):
        a = a1 - (a1 - a0) * i / n_arc
        pts.append((inner_r * math.cos(a), inner_r * math.sin(a)))
    wp = cq.Workplane("XY").moveTo(pts[0][0], pts[0][1])
    for p in pts[1:]:
        wp = wp.lineTo(p[0], p[1])
    return wp.close().extrude(height)


def _annular_sector_mesh(inner_r, outer_r, a0, a1, height, name):
    return mesh_from_cadquery(
        _annular_sector_solid(inner_r, outer_r, a0, a1, height),
        name,
        tolerance=0.0003,
        angular_tolerance=0.15,
    )


def _dial_disc_mesh(radius, win_a0, win_a1, thickness, hole_r, name):
    """Disc with one pie-sector window cutout and a center pivot hole."""
    n_arc = 20
    disc = cq.Workplane("XY").circle(radius).extrude(thickness)

    # Pie-sector window cutter (oversized for clean boolean)
    cut_r = radius + 0.003
    pts = [(0.0, 0.0)]
    for i in range(n_arc + 1):
        a = win_a0 + (win_a1 - win_a0) * i / n_arc
        pts.append((cut_r * math.cos(a), cut_r * math.sin(a)))
    cutter = cq.Workplane("XY").moveTo(pts[0][0], pts[0][1])
    for p in pts[1:]:
        cutter = cutter.lineTo(p[0], p[1])
    cutter = cutter.close().extrude(thickness + 0.006).translate((0, 0, -0.003))
    disc = disc.cut(cutter)

    # Center pivot hole
    hole = (
        cq.Workplane("XY")
        .circle(hole_r)
        .extrude(thickness + 0.006)
        .translate((0, 0, -0.003))
    )
    disc = disc.cut(hole)

    return mesh_from_cadquery(
        disc, name, tolerance=0.0003, angular_tolerance=0.12,
    )


# ── Build ────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotating_weekly_pill_dispenser")

    # Materials
    cream = model.material("warm_white_plastic", rgba=(0.94, 0.88, 0.68, 1.0))
    clear_lid = model.material("smoky_clear", rgba=(0.72, 0.78, 0.82, 0.35))
    white_print = model.material("white_print", rgba=(1.0, 1.0, 0.96, 1.0))
    grip_mat = model.material("grip_plastic", rgba=(0.85, 0.82, 0.72, 1.0))

    # Pastel tinted well floors (one tint per day)
    well_mats = [
        model.material("well_rose",     rgba=(1.00, 0.70, 0.68, 1.0)),
        model.material("well_blue",     rgba=(0.68, 0.78, 1.00, 1.0)),
        model.material("well_amber",    rgba=(1.00, 0.85, 0.60, 1.0)),
        model.material("well_lavender", rgba=(0.82, 0.75, 0.95, 1.0)),
        model.material("well_sage",     rgba=(0.72, 0.88, 0.72, 1.0)),
        model.material("well_aqua",     rgba=(0.65, 0.88, 0.88, 1.0)),
        model.material("well_butter",   rgba=(0.97, 0.92, 0.65, 1.0)),
    ]

    # ── base_tray ──────────────────────────────────────────────────
    base = model.part("base_tray")

    # Base plate (full disc, Z 0 → BASE_BOTTOM_H)
    base.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(TRAY_RADIUS).extrude(BASE_BOTTOM_H),
            "base_plate",
            tolerance=0.0003,
            angular_tolerance=0.15,
        ),
        origin=Origin(),
        material=cream,
        name="base_plate",
    )

    # Outer rim wall (annular ring, Z BASE_BOTTOM_H → WALL_TOP_Z)
    base.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .circle(TRAY_RADIUS)
            .circle(TRAY_RADIUS - RIM_T)
            .extrude(WALL_H),
            "outer_rim",
            tolerance=0.0003,
            angular_tolerance=0.15,
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM_H)),
        material=cream,
        name="outer_rim",
    )

    # Center pivot post (Z 0 → CENTER_POST_H, extends above lid)
    base.visual(
        Cylinder(radius=CENTER_POST_R, length=CENTER_POST_H),
        origin=Origin(xyz=(0.0, 0.0, CENTER_POST_H / 2.0)),
        material=cream,
        name="center_post",
    )

    # Post retention cap (wider lip above the dial lid)
    base.visual(
        Cylinder(radius=CENTER_POST_R + 0.002, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, CENTER_POST_H - 0.001)),
        material=cream,
        name="post_cap",
    )

    # 7 compartments: divider walls, tinted well floors
    for i in range(N_SECTORS):
        angle = i * SECTOR_ANGLE

        # Radial divider wall (Box rotated about Z)
        cx = WALL_MID_R * math.cos(angle)
        cy = WALL_MID_R * math.sin(angle)
        base.visual(
            Box((WALL_LEN, DIVIDER_T, WALL_H)),
            origin=Origin(
                xyz=(cx, cy, BASE_BOTTOM_H + WALL_H / 2.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=cream,
            name=f"divider_wall_{i}",
        )

        # Tinted well floor (annular sector on base plate)
        a0 = angle + DIVIDER_GAP
        a1 = (i + 1) * SECTOR_ANGLE - DIVIDER_GAP
        base.visual(
            _annular_sector_mesh(
                WELL_INNER_R, WELL_OUTER_R, a0, a1, WELL_FLOOR_H,
                f"well_floor_{i}",
            ),
            origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM_H)),
            material=well_mats[i],
            name=f"well_floor_{i}",
        )

        # Day-number dot on outer rim top (centered on each sector)
        dot_a = angle + SECTOR_ANGLE / 2.0
        dot_r = TRAY_RADIUS - RIM_T / 2.0
        base.visual(
            Cylinder(radius=0.0018, length=0.0008),
            origin=Origin(
                xyz=(dot_r * math.cos(dot_a), dot_r * math.sin(dot_a),
                     WALL_TOP_Z + 0.0004),
            ),
            material=white_print,
            name=f"day_dot_{i}",
        )

    # ── dial_lid ───────────────────────────────────────────────────
    dial_lid = model.part("dial_lid")

    # Window spans one sector (sector 0 at rest)
    win_a0 = 0.0 + DIVIDER_GAP
    win_a1 = SECTOR_ANGLE - DIVIDER_GAP

    # Disc with sector window cutout and center pivot hole
    dial_lid.visual(
        _dial_disc_mesh(
            LID_R, win_a0, win_a1, LID_T,
            CENTER_POST_R + 0.001,
            "dial_disc",
        ),
        origin=Origin(),
        material=clear_lid,
        name="dial_disc",
    )

    # Finger-grip tab on solid disc, just past the window trailing edge
    grip_a = win_a1 + 0.06
    grip_r = LID_R - 0.008
    dial_lid.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(
            xyz=(grip_r * math.cos(grip_a),
                 grip_r * math.sin(grip_a),
                 LID_T + 0.0015),
            rpy=(0.0, 0.0, grip_a),
        ),
        material=grip_mat,
        name="grip_tab",
    )

    # Small indicator dot beside the window opening
    ind_a = win_a0 - 0.03
    ind_r = LID_R - 0.012
    dial_lid.visual(
        Cylinder(radius=0.0015, length=0.0008),
        origin=Origin(
            xyz=(ind_r * math.cos(ind_a),
                 ind_r * math.sin(ind_a),
                 LID_T + 0.0004),
        ),
        material=white_print,
        name="window_indicator",
    )

    # ── Articulation ───────────────────────────────────────────────
    model.articulation(
        "base_to_dial_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial_lid,
        origin=Origin(xyz=(0.0, 0.0, LID_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=SECTOR_ANGLE * (N_SECTORS - 1),
        ),
    )

    return model


# ── Tests ────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_tray")
    dial_lid = object_model.get_part("dial_lid")
    dial_joint = object_model.get_articulation("base_to_dial_lid")

    # Structural identity checks
    ctx.check("dial_lid_part_exists", dial_lid is not None)
    ctx.check("base_to_dial_lid_exists", dial_joint is not None)
    ctx.check(
        "dial_joint_is_revolute_z",
        dial_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(dial_joint.axis[2]) > 0.99,
    )

    # 7 well floors and 7 divider walls
    ctx.check(
        "seven_well_floors",
        all(base.get_visual(f"well_floor_{i}") is not None
            for i in range(N_SECTORS)),
    )
    ctx.check(
        "seven_divider_walls",
        all(base.get_visual(f"divider_wall_{i}") is not None
            for i in range(N_SECTORS)),
    )

    # Dial disc covers most of the base footprint at rest (XY projection)
    ctx.expect_overlap(
        dial_lid, base,
        axes="xy",
        min_overlap=0.04,
        elem_a="dial_disc",
        elem_b="base_plate",
        name="dial_covers_base_at_rest",
    )

    # Dial sits on top of the base plate (Z clearance)
    ctx.expect_gap(
        dial_lid, base,
        axis="z",
        max_penetration=0.002,
        positive_elem="dial_disc",
        negative_elem="base_plate",
        name="dial_above_base_plate",
    )

    # Rotation proof: the grip tab world position shifts when dial rotates
    rest_aabb = ctx.part_element_world_aabb(dial_lid, elem="grip_tab")
    with ctx.pose({dial_joint: SECTOR_ANGLE}):
        rot_aabb = ctx.part_element_world_aabb(dial_lid, elem="grip_tab")
    ctx.check(
        "dial_rotation_shifts_window",
        rest_aabb is not None
        and rot_aabb is not None
        and (abs(rot_aabb[0][0] - rest_aabb[0][0]) > 0.005
             or abs(rot_aabb[0][1] - rest_aabb[0][1]) > 0.005),
        details=f"rest={rest_aabb}, rotated={rot_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
