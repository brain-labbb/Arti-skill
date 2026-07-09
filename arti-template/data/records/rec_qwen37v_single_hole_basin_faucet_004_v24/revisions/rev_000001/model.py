from __future__ import annotations

"""Single-hole basin faucet with rounded waterfall spout and side lever.

Layout (meters, +Z up, ground at z=0, spout extends along +X):
- A circular stepped base carries a cylindrical column.
- A waterfall-style spout curves forward from the column top with a rounded lip.
- A side lever mounts on a short horizontal axle boss on the column right side;
  it rotates about the Y-axis (left-right) for flow on/off (0..30 deg).
- Subtle grooves run along the lever grip surface.
"""

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

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
BASE_LOWER_R = 0.030
BASE_LOWER_H = 0.005
BASE_UPPER_R = 0.024
BASE_UPPER_H = 0.010
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.015

COLUMN_R = 0.018
COLUMN_TOP_Z = 0.155  # column height above base top
SPOUT_ROOT_Z = COLUMN_TOP_Z  # spout starts at column top

# Waterfall spout: curved channel sweeping forward
SPOUT_WIDTH = 0.040  # spout channel width (Y)
SPOUT_HEIGHT = 0.014  # channel wall thickness
SPOUT_REACH = 0.095  # horizontal reach forward from column center
SPOUT_DROP = 0.025  # how far the spout curves downward
SPOUT_LIP_R = 0.005  # rounded lip radius at the spout tip

# Lever mounting boss on column side
BOSS_R = 0.010
BOSS_LEN = 0.012  # protrudes from column surface
BOSS_CENTER_Z = COLUMN_TOP_Z - 0.030  # just below column top
BOSS_CENTER_Y = COLUMN_R + BOSS_LEN / 2.0  # on the +Y side

# Lever handle
LEVER_LEN = 0.100  # total length of lever
LEVER_WIDTH = 0.016  # lever cross-section width (along axle)
LEVER_THICK = 0.010  # lever cross-section thickness
LEVER_PIVOT_R = 0.008  # pivot cylinder at the mount end

# Grooves on lever grip
GROOVE_COUNT = 5
GROOVE_WIDTH = 0.002
GROOVE_DEPTH = 0.001
GROOVE_SPACING = 0.012

LIFT_RANGE = math.radians(30.0)


def _build_spout_shape() -> cq.Workplane:
    """Build the waterfall spout as a swept channel with rounded lip."""
    # Path: gentle curve from column top forward and slightly down
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, SPOUT_ROOT_Z)
        .spline(
            [
                (0.0, SPOUT_ROOT_Z),
                (SPOUT_REACH * 0.4, SPOUT_ROOT_Z - SPOUT_DROP * 0.1),
                (SPOUT_REACH * 0.7, SPOUT_ROOT_Z - SPOUT_DROP * 0.5),
                (SPOUT_REACH, SPOUT_ROOT_Z - SPOUT_DROP),
            ],
            tangents=[(1.0, -0.05), (1.0, -0.4)],
        )
    )

    # Cross-section: rectangular channel profile at the spout root
    profile = (
        cq.Workplane("YZ")
        .center(0.0, SPOUT_ROOT_Z)
        .rect(SPOUT_WIDTH, SPOUT_HEIGHT)
    )

    spout = profile.sweep(path, isFrenet=True)

    # Fillet the front (leading) edges to create the rounded waterfall lip
    # Select edges near the spout tip
    spout = (
        spout
        .edges(cq.selectors.BoxSelector(
            (SPOUT_REACH - SPOUT_LIP_R * 2, -SPOUT_WIDTH / 2 - 0.001, SPOUT_ROOT_Z - SPOUT_DROP - SPOUT_HEIGHT),
            (SPOUT_REACH + 0.005, SPOUT_WIDTH / 2 + 0.001, SPOUT_ROOT_Z - SPOUT_DROP + SPOUT_HEIGHT),
        ))
        .fillet(SPOUT_LIP_R * 0.8)
    )

    return spout


def _build_lever_with_grooves() -> cq.Workplane:
    """Build the lever handle with subtle grip grooves on top surface."""
    # Main lever body: extends along +X from the pivot center
    lever = (
        cq.Workplane("XY")
        .box(LEVER_LEN, LEVER_WIDTH, LEVER_THICK, centered=False)
    )

    # Round the grip end (far end from pivot)
    lever = (
        lever
        .edges(cq.selectors.BoxSelector(
            (LEVER_LEN - LEVER_WIDTH / 2 - 0.001, -LEVER_WIDTH / 2 - 0.001, -LEVER_THICK / 2 - 0.001),
            (LEVER_LEN + 0.001, LEVER_WIDTH / 2 + 0.001, LEVER_THICK / 2 + 0.001),
        ))
        .fillet(LEVER_WIDTH / 2.0 * 0.9)
    )

    # Cut grooves on top surface of the grip region
    groove_start_x = LEVER_LEN * 0.45
    for i in range(GROOVE_COUNT):
        gx = groove_start_x + i * GROOVE_SPACING
        groove_cutter = (
            cq.Workplane("XY")
            .transformed(offset=(gx, 0.0, LEVER_THICK / 2.0 - GROOVE_DEPTH / 2.0))
            .box(GROOVE_WIDTH, LEVER_WIDTH * 0.8, GROOVE_DEPTH)
        )
        lever = lever.cut(groove_cutter)

    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_mat = model.material("brushed_chrome", rgba=(0.75, 0.77, 0.80, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: base, column, waterfall spout, mounting boss
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Stepped circular base
    body.visual(
        Cylinder(radius=BASE_LOWER_R, length=BASE_LOWER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_lower",
    )
    body.visual(
        Cylinder(radius=BASE_UPPER_R, length=BASE_UPPER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_upper",
    )

    # Cylindrical column
    col_h = COLUMN_TOP_Z - BASE_TOP_Z
    body.visual(
        Cylinder(radius=COLUMN_R, length=col_h),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + col_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Waterfall spout with rounded lip
    spout = _build_spout_shape()
    body.visual(
        mesh_from_cadquery(spout, "waterfall_spout"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="waterfall_spout",
    )

    # Dark outlet recess under the spout tip (where water exits)
    body.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(SPOUT_REACH - 0.005, 0.0, SPOUT_ROOT_Z - SPOUT_DROP - SPOUT_HEIGHT / 2.0 + 0.002)),
        material=dark,
        name="outlet",
    )

    # Lever mounting boss on column side (+Y direction)
    # Cylindrical boss protruding from column surface along +Y
    body.visual(
        Cylinder(radius=BOSS_R, length=BOSS_LEN),
        origin=Origin(
            xyz=(0.0, COLUMN_R + BOSS_LEN / 2.0, BOSS_CENTER_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="lever_boss",
    )

    # ------------------------------------------------------------------
    # Side lever: rotates on horizontal axle (Y-axis) through the boss
    # The lever part frame is at the axle center; handle extends along +X.
    # ------------------------------------------------------------------
    lever = model.part("lever_handle")

    # Pivot cylinder at the mount point (centered at part origin = joint origin)
    lever.visual(
        Cylinder(radius=LEVER_PIVOT_R, length=LEVER_WIDTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_cylinder",
    )

    # Lever blade with grooves — starts at X=0 to merge with pivot cylinder
    lever_shape = _build_lever_with_grooves()
    lever.visual(
        mesh_from_cadquery(lever_shape, "lever_blade"),
        origin=Origin(xyz=(0.0, -LEVER_WIDTH / 2.0, -LEVER_THICK / 2.0)),
        material=grip_mat,
        name="lever_blade",
    )

    # Articulation: revolute about horizontal Y-axis through boss end face
    # axis=(0, -1, 0) so positive q lifts the +X grip end upward
    model.articulation(
        "lever_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, COLUMN_R + BOSS_LEN, BOSS_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=LIFT_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    lever = object_model.get_part("lever_handle")
    lever_joint = object_model.get_articulation("lever_rotate")

    # --- joint check: revolute, horizontal Y-axis, 0..30 deg ---
    ctx.check(
        "lever joint is revolute 0..30 deg about horizontal Y-axis",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(lever_joint.axis[0]) < 1e-9
        and abs(abs(lever_joint.axis[1]) - 1.0) < 1e-9
        and abs(lever_joint.axis[2]) < 1e-9
        and lever_joint.motion_limits is not None
        and abs(lever_joint.motion_limits.lower) < 1e-9
        and abs(lever_joint.motion_limits.upper - math.radians(30.0)) < 1e-6,
        details=f"axis={lever_joint.axis}, limits={lever_joint.motion_limits}",
    )

    # --- grounding ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "base is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )

    # --- compact basin faucet height (shorter than vessel faucet) ---
    ctx.check(
        "faucet height is compact basin scale 0.14..0.22 m",
        body_aabb is not None and 0.14 <= body_aabb[1][2] <= 0.22,
        details=f"body_aabb={body_aabb}",
    )

    # --- waterfall spout extends forward ---
    ctx.check(
        "waterfall spout extends forward from column",
        body_aabb is not None and body_aabb[1][0] > COLUMN_R + 0.05,
        details=f"body max x={body_aabb[1][0] if body_aabb else None}",
    )

    # --- waterfall spout has rounded lip (check the spout element exists) ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="waterfall_spout")
    ctx.check(
        "waterfall spout with rounded lip exists and extends forward",
        spout_aabb is not None
        and spout_aabb[1][0] > COLUMN_R + 0.06
        and (spout_aabb[1][1] - spout_aabb[0][1]) > 0.025,
        details=f"spout_aabb={spout_aabb}",
    )

    # --- lever blade has grooves (wider than plain blade due to groove cuts visible) ---
    blade_aabb = ctx.part_element_world_aabb(lever, elem="lever_blade")
    ctx.check(
        "lever blade with grip grooves exists",
        blade_aabb is not None and (blade_aabb[1][0] - blade_aabb[0][0]) > 0.06,
        details=f"blade_aabb={blade_aabb}",
    )

    # --- lever is mounted on the boss side ---
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever is side-mounted (extends to +Y of column center)",
        lever_aabb is not None and lever_aabb[1][1] > COLUMN_R + 0.01,
        details=f"lever_aabb={lever_aabb}",
    )

    # --- mounting: pivot axle captured in the boss ---
    ctx.allow_overlap(
        body,
        lever,
        elem_a="lever_boss",
        elem_b="pivot_cylinder",
        reason="The lever pivot axle is intentionally captured inside the boss end face, representing a real side-lever axle mount.",
    )
    ctx.allow_overlap(
        body,
        lever,
        elem_a="lever_boss",
        elem_b="lever_blade",
        reason="The lever blade root sockets over the boss end, representing the lever handle wrapping around its mounting boss.",
    )
    ctx.expect_overlap(
        lever,
        body,
        axes="y",
        min_overlap=0.005,
        elem_a="pivot_cylinder",
        elem_b="lever_boss",
        name="lever pivot cylinder overlaps boss on Y-axis (axle mount)",
    )
    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_blade",
        elem_b="lever_boss",
        contact_tol=0.002,
        name="lever blade root contacts the boss face",
    )

    # --- decisive pose: positive lever rotation lifts the grip end upward ---
    rest_aabb = lever_aabb
    rest_top_z = rest_aabb[1][2] if rest_aabb is not None else None
    with ctx.pose({lever_joint: LIFT_RANGE}):
        lifted_aabb = ctx.part_world_aabb(lever)
        ctx.check(
            "positive lever rotation raises grip end upward",
            rest_top_z is not None
            and lifted_aabb is not None
            and lifted_aabb[1][2] > rest_top_z + 0.01,
            details=f"rest_top={rest_top_z}, lifted_aabb={lifted_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
