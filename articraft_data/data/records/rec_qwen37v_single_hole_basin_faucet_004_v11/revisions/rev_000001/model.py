from __future__ import annotations

"""Single-hole basin faucet with tall straight tower and short swiveling spout.

Layout (meters, +Z up, ground at z=0, spout extends along +X):
- Circular stepped base plate (single-hole mount) carries a tall rectangular tower.
- A cylindrical collar/socket sits at the tower top.
- A short rectangular spout arm extends forward from the collar, swiveling about
  a vertical axis through the tower centerline.
- A real hollow outlet hole is cut through the spout underside near the tip.
- A separate circular aerator insert (flanged ring + mesh screen) press-fits
  into the outlet from below, its flange resting on the spout underside.
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
BASE_R_LOWER = 0.028
BASE_H_LOWER = 0.006
BASE_R_UPPER = 0.022
BASE_H_UPPER = 0.008
BASE_TOP_Z = BASE_H_LOWER + BASE_H_UPPER  # 0.014

TOWER_W_Y = 0.040
TOWER_D_X = 0.035
TOWER_TOP_Z = 0.280
TOWER_H = TOWER_TOP_Z - BASE_TOP_Z  # 0.266

COLLAR_R = 0.021
COLLAR_H = 0.016

SPOUT_REACH = 0.100  # forward extent from tower center
SPOUT_W_Y = 0.030
SPOUT_H_Z = 0.020

# Outlet hole cut through spout underside near tip
OUTLET_R = 0.009
OUTLET_FROM_TIP = 0.014
OUTLET_X_LOCAL = SPOUT_REACH - OUTLET_FROM_TIP  # 0.086 in spout local frame

# Aerator: flanged insert
# Stem fits inside the outlet hole
AERATOR_STEM_R = OUTLET_R - 0.001  # 0.008 — clearance fit inside hole
AERATOR_STEM_H = 0.005
# Flange sits on spout underside, wider than hole for contact
AERATOR_FLANGE_R = 0.013
AERATOR_FLANGE_H = 0.003
# Mesh screen
AERATOR_SCREEN_R = AERATOR_STEM_R - 0.001  # 0.007

SWIVEL_RANGE = math.radians(60.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    mesh_gray = model.material("aerator_mesh", rgba=(0.35, 0.36, 0.38, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: circular base plate + tall rectangular tower + collar
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Lower base plate (wider step)
    body.visual(
        Cylinder(radius=BASE_R_LOWER, length=BASE_H_LOWER),
        origin=Origin(xyz=(0.0, 0.0, BASE_H_LOWER / 2.0)),
        material=chrome,
        name="base_lower",
    )
    # Upper base plate (narrower step)
    body.visual(
        Cylinder(radius=BASE_R_UPPER, length=BASE_H_UPPER),
        origin=Origin(xyz=(0.0, 0.0, BASE_H_LOWER + BASE_H_UPPER / 2.0)),
        material=chrome,
        name="base_upper",
    )
    # Tall rectangular tower
    body.visual(
        Box((TOWER_D_X, TOWER_W_Y, TOWER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + TOWER_H / 2.0)),
        material=chrome,
        name="tower",
    )
    # Cylindrical collar/socket at tower top (spout swivels within)
    body.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z + COLLAR_H / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # ------------------------------------------------------------------
    # Spout: short rectangular arm with hollow outlet, swivels on body
    # Part frame at swivel axis (tower top center).
    # Spout arm extends from X=0 forward, bottom at Z=0 in local frame.
    # ------------------------------------------------------------------
    spout = model.part("spout")

    # Build spout arm with outlet hole via CadQuery
    spout_solid = (
        cq.Workplane("XY")
        .transformed(offset=(SPOUT_REACH / 2.0, 0.0, SPOUT_H_Z / 2.0))
        .box(SPOUT_REACH, SPOUT_W_Y, SPOUT_H_Z)
    )
    # Cut cylindrical through-hole for the outlet near the tip
    outlet_cutter = (
        cq.Workplane("XY")
        .transformed(offset=(OUTLET_X_LOCAL, 0.0, -0.005))
        .circle(OUTLET_R)
        .extrude(SPOUT_H_Z + 0.010)
    )
    spout_shape = spout_solid.cut(outlet_cutter)

    spout.visual(
        mesh_from_cadquery(spout_shape, "spout_arm"),
        origin=Origin(),
        material=chrome,
        name="spout_arm",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, TOWER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=2.0, lower=-SWIVEL_RANGE, upper=SWIVEL_RANGE
        ),
    )

    # ------------------------------------------------------------------
    # Aerator: separate flanged circular insert, child of spout
    # The flange rests on the spout underside around the outlet hole.
    # The stem protrudes up into the hole.
    # Aerator part frame at the spout bottom face, at the outlet X.
    # ------------------------------------------------------------------
    aerator = model.part("aerator")

    # Flange: annular ring wider than the hole, sits on spout underside
    # Extends from Z=-AERATOR_FLANGE_H to Z=0 in aerator local frame
    flange_shape = (
        cq.Workplane("XY")
        .circle(AERATOR_FLANGE_R)
        .circle(AERATOR_STEM_R)
        .extrude(AERATOR_FLANGE_H)
    )
    aerator.visual(
        mesh_from_cadquery(flange_shape, "aerator_flange"),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_FLANGE_H)),
        material=chrome,
        name="aerator_flange",
    )

    # Stem: cylindrical insert that fits inside the outlet hole
    # Extends from Z=0 upward into the hole
    stem_shape = (
        cq.Workplane("XY")
        .circle(AERATOR_STEM_R)
        .circle(AERATOR_SCREEN_R)
        .extrude(AERATOR_STEM_H)
    )
    aerator.visual(
        mesh_from_cadquery(stem_shape, "aerator_stem"),
        origin=Origin(),
        material=chrome,
        name="aerator_stem",
    )

    # Mesh screen disc at the bottom of the stem (visible from below)
    aerator.visual(
        Cylinder(radius=AERATOR_SCREEN_R, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=mesh_gray,
        name="aerator_screen",
    )

    # Dark recessed outlet face inside the hole (visible from above through hole)
    # Connected to the aerator stem top
    aerator.visual(
        Cylinder(radius=OUTLET_R - 0.002, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark,
        name="outlet_recess",
    )

    model.articulation(
        "aerator_mount",
        ArticulationType.FIXED,
        parent=spout,
        child=aerator,
        # Aerator origin at spout bottom face, at the outlet X position
        origin=Origin(xyz=(OUTLET_X_LOCAL, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    swivel = object_model.get_articulation("spout_swivel")
    aerator_mount = object_model.get_articulation("aerator_mount")

    # --- Intentional overlap: spout arm emerges from within the collar socket ---
    ctx.allow_overlap(
        body,
        spout,
        elem_a="swivel_collar",
        elem_b="spout_arm",
        reason=(
            "The spout arm root is intentionally nested inside the cylindrical "
            "swivel collar socket, representing the spout emerging from the body."
        ),
    )
    ctx.expect_contact(
        spout,
        body,
        elem_a="spout_arm",
        elem_b="swivel_collar",
        contact_tol=0.005,
        name="spout arm root is supported by the swivel collar socket",
    )

    # --- joint plan ---
    ctx.check(
        "spout_swivel is revolute about vertical axis ±60 deg",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and abs(swivel.axis[2] - 1.0) < 1e-9
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_RANGE) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_RANGE) < 1e-6,
        details=f"axis={swivel.axis}, limits={swivel.motion_limits}",
    )
    ctx.check(
        "aerator is a separate part fixed-mounted to the spout",
        aerator_mount.articulation_type == ArticulationType.FIXED
        and aerator_mount.parent == spout.name
        and aerator_mount.child == aerator.name,
        details=f"parent={aerator_mount.parent}, child={aerator_mount.child}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "base plate grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "tall tower: total height ~0.30 m",
        spout_aabb is not None and 0.29 <= spout_aabb[1][2] <= 0.32,
        details=f"spout_aabb={spout_aabb}",
    )
    ctx.check(
        "short forward spout reach ~0.07-0.12 m past tower front face",
        spout_aabb is not None
        and 0.06 <= spout_aabb[1][0] - TOWER_D_X / 2.0 <= 0.13,
        details=f"spout max x={None if spout_aabb is None else spout_aabb[1][0]}",
    )

    # --- hollow outlet at spout mouth (near bottom) ---
    outlet_aabb = ctx.part_element_world_aabb(aerator, elem="outlet_recess")
    spout_arm_aabb = ctx.part_element_world_aabb(spout, elem="spout_arm")
    ctx.check(
        "outlet recess is near spout underside, close to the tip",
        outlet_aabb is not None
        and spout_arm_aabb is not None
        and outlet_aabb[0][2] < (spout_arm_aabb[0][2] + spout_arm_aabb[1][2]) / 2.0
        and outlet_aabb[1][0] > spout_arm_aabb[0][0] + 0.05,
        details=f"outlet={outlet_aabb}, arm={spout_arm_aabb}",
    )

    # --- aerator insert contacts spout underside ---
    ctx.expect_contact(
        aerator,
        spout,
        elem_a="aerator_flange",
        elem_b="spout_arm",
        contact_tol=1e-5,
        name="aerator flange contacts the spout underside around the outlet",
    )
    ctx.expect_overlap(
        aerator,
        spout,
        axes="xy",
        min_overlap=0.005,
        name="aerator XY footprint overlaps the spout outlet region",
    )

    # --- decisive pose: swivel moves spout sideways ---
    rest_spout_aabb = spout_aabb
    with ctx.pose({swivel: SWIVEL_RANGE}):
        swung_aabb = ctx.part_world_aabb(spout)
        ctx.check(
            "positive swivel rotates spout sideways about vertical axis",
            rest_spout_aabb is not None
            and swung_aabb is not None
            and abs(swung_aabb[1][1] - rest_spout_aabb[1][1]) > 0.02,
            details=f"rest={rest_spout_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
