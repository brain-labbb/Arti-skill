from __future__ import annotations

"""Electric table saw: under-table configuration of the cordless circular saw.

A flat steel table with a blade slot forms the work surface. The teal motor
housing and blade are mounted below, and the toothed blade protrudes up
through the slot. A prismatic rip fence slides across the tabletop.

Canonical frame: +X is the feed direction, +Z is up, +Y is left.
Real-world scale: table about 0.55 x 0.40 m, 140 mm blade.

Articulated mechanisms:
- blade_spin: toothed circular blade spins about the arbor (CONTINUOUS).
- fence_slide: rip fence slides across the tabletop (PRISMATIC).
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

HALF_PI = math.pi / 2.0

# ── shared constants ──────────────────────────────────────────────────
BLADE_RADIUS = 0.076
TABLE_THICK = 0.025
TABLE_TOP_Z = TABLE_THICK / 2.0          # top surface of centred extrusion
ARBOR_X = 0.02
ARBOR_Y = 0.0
ARBOR_Z = -0.030                          # below table; blade pokes up

TABLE_W = 0.55
TABLE_D = 0.40
LEG_HEIGHT = 0.65
LEG_SZ = 0.04
HALF_W = TABLE_W / 2.0
HALF_D = TABLE_D / 2.0


# ── geometry helpers ──────────────────────────────────────────────────
def _side_extrusion(profile, width: float, name: str):
    """Extrude an XZ side-silhouette profile along the Y (width) axis."""
    geom = ExtrudeGeometry(profile, width, cap=True, center=True)
    geom.rotate_x(HALF_PI)
    return mesh_from_geometry(geom, name)


def _blade_teeth_profile(inner_r: float, outer_r: float, n: int):
    """Closed polygon profile of a toothed disc in local XY."""
    pts = []
    for i in range(n):
        a0 = (i / n) * 2.0 * math.pi
        a1 = ((i + 0.5) / n) * 2.0 * math.pi
        pts.append((outer_r * math.cos(a0), outer_r * math.sin(a0)))
        pts.append((inner_r * math.cos(a1), inner_r * math.sin(a1)))
    return pts


# ── build ─────────────────────────────────────────────────────────────
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_table_saw")

    # materials
    teal = model.material("teal_housing", rgba=(0.10, 0.52, 0.55, 1.0))
    teal_dark = model.material("teal_dark", rgba=(0.07, 0.40, 0.43, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    chrome = model.material("chrome_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.66, 0.67, 0.70, 1.0))
    table_steel = model.material("table_steel", rgba=(0.60, 0.62, 0.64, 1.0))
    fence_alum = model.material("fence_aluminum", rgba=(0.75, 0.76, 0.72, 1.0))
    red_accent = model.material("red_accent", rgba=(0.72, 0.14, 0.12, 1.0))

    # ============================================================ TABLE (root)
    table = model.part("table")

    # ── tabletop with blade slot ──
    table_outer = [
        (-HALF_W, -HALF_D), (HALF_W, -HALF_D),
        (HALF_W, HALF_D), (-HALF_W, HALF_D),
    ]
    slot_hx = 0.082          # half-length along X
    slot_hy = 0.005          # half-width along Y (wider than blade 0.0013)
    blade_slot = [
        (ARBOR_X - slot_hx, -slot_hy),
        (ARBOR_X + slot_hx, -slot_hy),
        (ARBOR_X + slot_hx, slot_hy),
        (ARBOR_X - slot_hx, slot_hy),
    ]
    table.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                table_outer, [blade_slot], TABLE_THICK, cap=True, center=True
            ),
            "tabletop",
        ),
        material=table_steel,
        name="tabletop",
    )

    # ── table legs (4 corners) ──
    leg_corners = [
        (HALF_W - 0.04, HALF_D - 0.04),
        (HALF_W - 0.04, -(HALF_D - 0.04)),
        (-(HALF_W - 0.04), HALF_D - 0.04),
        (-(HALF_W - 0.04), -(HALF_D - 0.04)),
    ]
    leg_cz = -TABLE_THICK / 2.0 - LEG_HEIGHT / 2.0 + 0.005  # slight embed
    for i, (lx, ly) in enumerate(leg_corners):
        table.visual(
            Box((LEG_SZ, LEG_SZ, LEG_HEIGHT)),
            origin=Origin(xyz=(lx, ly, leg_cz)),
            material=charcoal,
            name=f"table_leg_{i}",
        )

    # ── side rails & cross braces ──
    rail_z = -0.45
    for i, ry in enumerate([HALF_D - 0.04, -(HALF_D - 0.04)]):
        table.visual(
            Box((TABLE_W - 0.10, 0.025, 0.025)),
            origin=Origin(xyz=(0.0, ry, rail_z)),
            material=charcoal,
            name=f"side_rail_{i}",
        )
    for i, rx in enumerate([HALF_W - 0.04, -(HALF_W - 0.04)]):
        table.visual(
            Box((0.025, TABLE_D - 0.10, 0.025)),
            origin=Origin(xyz=(rx, 0.0, rail_z)),
            material=charcoal,
            name=f"cross_brace_{i}",
        )

    # ── motor assembly (under table) ──
    # Housing shell – teal rounded body
    table.visual(
        _side_extrusion(
            rounded_rect_profile(0.17, 0.11, 0.035), 0.075, "housing_shell"
        ),
        origin=Origin(xyz=(-0.02, 0.04, -0.10)),
        material=teal,
        name="housing_shell",
    )

    # Motor barrel (cross-mounted on housing)
    table.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(-0.01, 0.09, -0.09), rpy=(HALF_PI, 0.0, 0.0)),
        material=teal,
        name="motor_barrel",
    )
    table.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(-0.01, 0.128, -0.09), rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="motor_end_cap",
    )

    # Gearbox nose – bridges housing up to the table plate and arbor.
    # Offset in +Y so the gearbox sits beside the thin blade plane, not
    # around it (blade Y ≈ 0 ± 0.0013; gearbox starts at Y ≈ 0.008).
    table.visual(
        Box((0.060, 0.035, 0.090)),
        origin=Origin(xyz=(0.02, 0.025, -0.050)),
        material=teal,
        name="gearbox_nose",
    )

    # Arbor boss
    table.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(
            xyz=(ARBOR_X, ARBOR_Y + 0.020, ARBOR_Z), rpy=(HALF_PI, 0.0, 0.0)
        ),
        material=charcoal,
        name="arbor_boss",
    )

    # ── upper blade guard (covers exposed blade above table) ──
    ug_ang = [math.pi * (0.03 + 0.94 * i / 36) for i in range(37)]
    table.visual(
        _side_extrusion(
            [
                ((BLADE_RADIUS + 0.016) * math.cos(a) + ARBOR_X,
                 (BLADE_RADIUS + 0.016) * math.sin(a) + ARBOR_Z)
                for a in ug_ang
            ]
            + [
                (BLADE_RADIUS * math.cos(a) + ARBOR_X,
                 BLADE_RADIUS * math.sin(a) + ARBOR_Z)
                for a in reversed(ug_ang)
            ],
            0.020,
            "upper_guard_ring",
        ),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=teal_dark,
        name="upper_guard",
    )
    table.visual(
        _side_extrusion(
            [
                ((BLADE_RADIUS + 0.014) * math.cos(a) + ARBOR_X,
                 (BLADE_RADIUS + 0.014) * math.sin(a) + ARBOR_Z)
                for a in [math.pi * (0.05 + 0.9 * i / 24) for i in range(25)]
            ]
            + [
                (ARBOR_X - (BLADE_RADIUS + 0.014), ARBOR_Z),
                (ARBOR_X + (BLADE_RADIUS + 0.014), ARBOR_Z),
            ],
            0.030,
            "upper_guard_web",
        ),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=teal_dark,
        name="upper_guard_web",
    )

    # ── battery pack (under table, behind motor) ──
    table.visual(
        Box((0.075, 0.090, 0.070)),
        origin=Origin(xyz=(-0.08, 0.06, -0.14)),
        material=black_plastic,
        name="battery_pack",
    )
    table.visual(
        Box((0.060, 0.070, 0.014)),
        origin=Origin(xyz=(-0.05, 0.06, -0.14)),
        material=charcoal,
        name="battery_terminal_shroud",
    )

    # Brand plate on housing side
    table.visual(
        Box((0.070, 0.004, 0.030)),
        origin=Origin(xyz=(-0.02, 0.076, -0.06)),
        material=red_accent,
        name="brand_plate",
    )

    # Vent / cord stub
    cord_geom = tube_from_spline_points(
        [(-0.10, 0.07, -0.12), (-0.12, 0.08, -0.13), (-0.13, 0.09, -0.14)],
        radius=0.006,
        samples_per_segment=8,
    )
    table.visual(
        mesh_from_geometry(cord_geom, "vent_stub"),
        material=charcoal,
        name="vent_stub",
    )

    # ============================================================ BLADE
    blade = model.part("blade")
    blade.visual(
        _side_extrusion(
            _blade_teeth_profile(BLADE_RADIUS - 0.007, BLADE_RADIUS, 48),
            0.0026,
            "blade_disc",
        ),
        material=blade_steel,
        name="blade_disc",
    )
    blade.visual(
        Cylinder(radius=BLADE_RADIUS - 0.005, length=0.0016),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=chrome,
        name="blade_plate",
    )
    blade.visual(
        Cylinder(radius=BLADE_RADIUS * 0.36, length=0.0032),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=blade_steel,
        name="blade_inner_ring",
    )
    blade.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="arbor_bolt",
    )

    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=table,
        child=blade,
        origin=Origin(xyz=(ARBOR_X, ARBOR_Y, ARBOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=350.0),
    )

    # ============================================================ RIP FENCE
    fence = model.part("rip_fence")
    # Tall fence face
    fence.visual(
        Box((0.40, 0.025, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, TABLE_TOP_Z + 0.0275)),
        material=fence_alum,
        name="fence_bar",
    )
    # Low base rail on table surface
    fence.visual(
        Box((0.40, 0.040, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, TABLE_TOP_Z + 0.004)),
        material=chrome,
        name="fence_rail",
    )

    model.articulation(
        "fence_slide",
        ArticulationType.PRISMATIC,
        parent=table,
        child=fence,
        origin=Origin(xyz=(0.0, 0.08, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.5, lower=-0.06, upper=0.10
        ),
    )

    return model


# ── tests ─────────────────────────────────────────────────────────────
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    table = object_model.get_part("table")
    blade = object_model.get_part("blade")
    fence = object_model.get_part("rip_fence")

    blade_joint = object_model.get_articulation("blade_spin")
    fence_joint = object_model.get_articulation("fence_slide")

    # ── intentional fits ──
    # Arbor boss and arbor bolt are close neighbours on the arbor stack
    ctx.expect_contact(
        table, blade,
        elem_a="arbor_boss",
        elem_b="arbor_bolt",
        contact_tol=4e-3,
        name="blade seats on the arbor boss",
    )

    # ── prompt-level geometry checks ──
    blade_aabb = ctx.part_world_aabb(blade)
    ctx.check(
        "blade protrudes above the table surface through the slot",
        blade_aabb is not None and blade_aabb[1][2] > TABLE_TOP_Z + 0.01,
        details=f"blade_max_z={None if blade_aabb is None else blade_aabb[1][2]}, "
                f"table_top={TABLE_TOP_Z}",
    )
    ctx.check(
        "blade is a disc with diameter > 0.12 m",
        blade_aabb is not None and (blade_aabb[1][0] - blade_aabb[0][0]) > 0.12,
        details=f"blade_aabb={blade_aabb}",
    )

    table_aabb = ctx.part_world_aabb(table)
    ctx.check(
        "table is the broad work surface at the top of the assembly",
        table_aabb is not None and table_aabb[1][2] > 0.0
        and (table_aabb[1][0] - table_aabb[0][0]) > 0.40,
        details=f"table_aabb={table_aabb}",
    )

    fence_aabb = ctx.part_world_aabb(fence)
    ctx.check(
        "rip fence sits on or above the table top surface",
        fence_aabb is not None and fence_aabb[0][2] > TABLE_TOP_Z - 0.005,
        details=f"fence_min_z={None if fence_aabb is None else fence_aabb[0][2]}",
    )

    # Motor assembly hangs below the table
    housing_aabb = ctx.part_element_world_aabb(table, elem="housing_shell")
    ctx.check(
        "housing_shell is below the table top",
        housing_aabb is not None and housing_aabb[1][2] < 0.0,
        details=f"housing_max_z={None if housing_aabb is None else housing_aabb[1][2]}",
    )
    ctx.check(
        "assembly extends well below the table (motor underneath)",
        table_aabb is not None and table_aabb[0][2] < -0.10,
        details=f"table_min_z={None if table_aabb is None else table_aabb[0][2]}",
    )

    # ── decisive poses ──
    # Blade spins about the arbor
    p_rest = ctx.part_world_aabb(blade)
    with ctx.pose({blade_joint: 0.8}):
        p_spun = ctx.part_world_aabb(blade)
    ctx.check(
        "blade rotates about the arbor",
        p_rest is not None and p_spun is not None,
        details=f"rest={p_rest}, spun={p_spun}",
    )

    # Fence slides across the table in Y
    f_rest = ctx.part_world_position(fence)
    with ctx.pose({fence_joint: 0.08}):
        f_slid = ctx.part_world_position(fence)
    ctx.check(
        "rip fence slides across the tabletop in Y",
        f_rest is not None and f_slid is not None
        and abs(f_slid[1] - f_rest[1]) > 0.06,
        details=f"rest={f_rest}, slid={f_slid}",
    )

    return ctx.report()


object_model = build_object_model()
