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

# ---------------------------------------------------------------------------
# High-arc gooseneck faucet variant 26.
#
# Derived from the gloss-black monobloc kitchen mixer tap. Structural changes:
#  1. Outlet shaped as a short angled aerator at the end of the arc (angled
#     ~35 deg from vertical, toward +X).
#  2. A temperature ring rotates around the pedestal column (continuous
#     revolute about Z).
#  3. Shallow ribbing on the spray head (radial ridges around the tip).
#  4. A thin seam ring at the swivel collar (dark groove between chrome
#     collar and spout base).
#
# Layout (world frame, deck at z=0, +X is front/user-facing spout direction,
# +Z is up).
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020
COLUMN_TOP = 0.132

# Cross valve cylinder
CROSS_Z = 0.085
CROSS_R = 0.0225
CROSS_TUBE_LEN = 0.170
CAP_LEN = 0.005
CAP_R = 0.0235
CAP_Y = CROSS_TUBE_LEN / 2.0 + CAP_LEN / 2.0

# Pin levers (lever-local frame at the valve axis center)
LEVER_Y = 0.058
BOSS_R = 0.010
BOSS_LEN = 0.016
BOSS_Z = 0.026
PIN_R = 0.006
PIN_LEN = 0.100
PIN_Z0 = 0.032

# Temperature ring (around column, between cross valve and collar)
TEMP_RING_R_OUTER = 0.026  # slightly wider than column
TEMP_RING_R_INNER = 0.0199  # slight interference with column for contact
TEMP_RING_H = 0.010
TEMP_RING_Z_CENTER = 0.118  # between cross (0.085) and collar (0.140)

# Swivel collar + gooseneck
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

# Seam ring at collar (thin dark groove, sits right at collar top)
SEAM_R = 0.0220
SEAM_H = 0.0015
SEAM_Z = SWIVEL_Z + SEAM_H / 2.0  # just above collar top surface

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153
REACH_X = 2.0 * ARC_R  # 0.144 m
DROP_END = 0.124

# Angled aerator nozzle parameters
AERATOR_ANGLE_DEG = 35.0  # tilt from vertical toward +X
AERATOR_LEN = 0.025
AERATOR_R = 0.012  # outer radius of angled nozzle body
AERATOR_TIP_R = 0.009  # mesh screen tip

# Spray head ribbing
RIB_COUNT = 8
RIB_WIDTH = 0.003
RIB_THICK = 0.002  # proud of the sleeve surface
RIB_LEN = 0.022  # along the sleeve length

# Tip sleeve (slightly larger to accommodate ribbing)
SLEEVE_R = 0.0165
SLEEVE_LEN = 0.028

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R
SWIVEL_LIMIT = math.radians(110.0)


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube: straight riser, high semicircular arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def _angled_aerator_body() -> cq.Workplane:
    """Short angled aerator nozzle, centered at origin.
    Axis points in (sin(35°), 0, -cos(35°)) — downward and forward."""
    # Build cylinder centered at origin: from -LEN/2 to +LEN/2 along Z
    body = (
        cq.Workplane("XY")
        .workplane(offset=-AERATOR_LEN / 2.0)
        .circle(AERATOR_R)
        .extrude(AERATOR_LEN)
    )
    # Rotate +Z axis to (sin(35°), 0, -cos(35°)): rotate by 145° around Y
    # since cos(145°) = -cos(35°), sin(145°) = sin(35°)
    body = body.rotate((0, 0, 0), (0, 1, 0), 180.0 - AERATOR_ANGLE_DEG)
    return body


def _aerator_tip_mesh() -> cq.Workplane:
    """Small disc representing the aerator mesh screen at the nozzle tip."""
    # Build a thin disc centered at origin, then tilt same as body
    tip = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(AERATOR_TIP_R)
        .extrude(0.002)
    )
    tip = tip.rotate((0, 0, 0), (0, 1, 0), 180.0 - AERATOR_ANGLE_DEG)
    return tip


def _temperature_ring() -> cq.Workplane:
    """Annular ring that wraps around the column pedestal."""
    outer = (
        cq.Workplane("XY")
        .circle(TEMP_RING_R_OUTER)
        .circle(TEMP_RING_R_INNER)
        .extrude(TEMP_RING_H)
    )
    return outer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet_v26")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    seam_dark = model.material("seam_dark", rgba=(0.02, 0.02, 0.025, 1.0))
    ring_chrome = model.material("ring_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    rib_material = model.material("rib_matte", rgba=(0.06, 0.06, 0.065, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_TOP - 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_TOP + 0.004) / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    # Horizontal cross valve cylinder
    column.visual(
        Cylinder(radius=CROSS_R, length=CROSS_TUBE_LEN),
        origin=Origin(xyz=(0.0, 0.0, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="cross_tube",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_0",
    )
    column.visual(
        Cylinder(radius=CAP_R, length=CAP_LEN),
        origin=Origin(xyz=(0.0, -CAP_Y, CROSS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="valve_end_cap_1",
    )
    # Chrome collar ring
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )

    # ------------------------------------------------------- temperature ring
    temp_ring = model.part("temperature_ring")
    temp_ring.visual(
        mesh_from_cadquery(_temperature_ring(), "temp_ring_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=ring_chrome,
        name="ring_body",
    )
    # Small indicator notch on the ring (gives visual feedback of rotation)
    temp_ring.visual(
        Box(size=(0.004, 0.003, TEMP_RING_H * 0.6)),
        origin=Origin(xyz=(TEMP_RING_R_OUTER - 0.001, 0.0, TEMP_RING_H * 0.3)),
        material=matte_black,
        name="ring_indicator",
    )
    model.articulation(
        "ring_rotation",
        ArticulationType.REVOLUTE,
        parent=column,
        child=temp_ring,
        origin=Origin(xyz=(0.0, 0.0, TEMP_RING_Z_CENTER - TEMP_RING_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    # Chrome tip sleeve with ribbing
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="tip_sleeve",
    )
    # Shallow ribbing on the spray head (radial ridges around the sleeve)
    for i in range(RIB_COUNT):
        angle = 2.0 * math.pi * i / RIB_COUNT
        rib_x = REACH_X + (SLEEVE_R + RIB_THICK / 2.0) * math.cos(angle)
        rib_y = (SLEEVE_R + RIB_THICK / 2.0) * math.sin(angle)
        spout.visual(
            Box(size=(RIB_THICK, RIB_WIDTH, RIB_LEN)),
            origin=Origin(
                xyz=(rib_x, rib_y, DROP_END + SLEEVE_LEN / 2.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=rib_material,
            name=f"spray_rib_{i}",
        )

    # Angled aerator nozzle body at end of drop leg
    angle_rad = math.radians(AERATOR_ANGLE_DEG)
    # Aerator axis direction: (sin(35°), 0, -cos(35°)) — down and forward.
    # Top of aerator connects at drop-leg end (REACH_X, 0, DROP_END).
    # Center = top - (LEN/2) * axis_dir = top + (LEN/2) * (-axis_dir)
    aerator_center_x = REACH_X + (AERATOR_LEN / 2.0) * math.sin(angle_rad)
    aerator_center_z = DROP_END - (AERATOR_LEN / 2.0) * math.cos(angle_rad)
    spout.visual(
        mesh_from_cadquery(_angled_aerator_body(), "aerator_body"),
        origin=Origin(xyz=(aerator_center_x, 0.0, aerator_center_z)),
        material=outlet_dark,
        name="aerator_nozzle",
    )
    # Aerator mesh screen at the nozzle tip (embedded 1mm into nozzle body for connectivity)
    tip_x = REACH_X + (AERATOR_LEN - 0.001) * math.sin(angle_rad)
    tip_z = DROP_END - (AERATOR_LEN - 0.001) * math.cos(angle_rad)
    spout.visual(
        mesh_from_cadquery(_aerator_tip_mesh(), "aerator_tip"),
        origin=Origin(xyz=(tip_x, 0.0, tip_z)),
        material=seam_dark,
        name="aerator_mesh_screen",
    )

    # Thin seam ring at the swivel collar (visible groove)
    spout.visual(
        Cylinder(radius=SEAM_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_Z - SWIVEL_Z)),
        material=seam_dark,
        name="collar_seam",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.5, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # ------------------------------------------------------------- pin levers
    for idx, y_sign in ((0, 1.0), (1, -1.0)):
        lever = model.part(f"pin_lever_{idx}")
        lever.visual(
            Cylinder(radius=BOSS_R, length=BOSS_LEN),
            origin=Origin(xyz=(0.0, 0.0, BOSS_Z)),
            material=gloss_black,
            name="lever_boss",
        )
        lever.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(xyz=(0.0, 0.0, PIN_Z0 + PIN_LEN / 2.0)),
            material=gloss_black,
            name="lever_pin",
        )
        model.articulation(
            f"lever_pivot_{idx}",
            ArticulationType.REVOLUTE,
            parent=column,
            child=lever,
            origin=Origin(xyz=(0.0, y_sign * LEVER_Y, CROSS_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=2.0, lower=-math.pi / 2.0, upper=0.0
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    temp_ring = object_model.get_part("temperature_ring")
    lever_0 = object_model.get_part("pin_lever_0")
    lever_1 = object_model.get_part("pin_lever_1")

    swivel = object_model.get_articulation("spout_swivel")
    ring_rot = object_model.get_articulation("ring_rotation")
    pivot_0 = object_model.get_articulation("lever_pivot_0")
    pivot_1 = object_model.get_articulation("lever_pivot_1")

    # Intentional overlaps: lever bosses seat into the valve cylinder
    ctx.allow_overlap(
        lever_0,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )
    ctx.allow_overlap(
        lever_1,
        column,
        elem_a="lever_boss",
        elem_b="cross_tube",
        reason="Lever boss intentionally seats a few mm into the valve cylinder.",
    )
    # Temperature ring wraps around the column
    ctx.allow_overlap(
        temp_ring,
        column,
        elem_a="ring_body",
        elem_b="column_shaft",
        reason="Temperature ring is an annular sleeve that wraps around the column pedestal.",
    )

    # ----- scale and proportions
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m (high-arc silhouette)",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.388,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.130,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- angled aerator at end of arc
    aerator = ctx.part_element_world_aabb(spout, elem="aerator_nozzle")
    sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "angled aerator nozzle hangs below the tip sleeve",
        aerator is not None
        and sleeve is not None
        and aerator[0][2] < sleeve[0][2],
        details=f"aerator={aerator}, sleeve={sleeve}",
    )
    ctx.check(
        "aerator nozzle extends forward (+X) of the sleeve center (angled outlet)",
        aerator is not None
        and sleeve is not None
        and 0.5 * (aerator[0][0] + aerator[1][0]) > 0.5 * (sleeve[0][0] + sleeve[1][0]) + 0.003,
        details=f"aerator center x vs sleeve center x",
    )

    # ----- temperature ring around pedestal
    ring_aabb = ctx.part_world_aabb(temp_ring)
    cross = ctx.part_element_world_aabb(column, elem="cross_tube")
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "temperature ring sits between the cross valve and the swivel collar",
        ring_aabb is not None
        and cross is not None
        and collar is not None
        and ring_aabb[0][2] >= cross[1][2] - 0.005
        and ring_aabb[1][2] <= collar[0][2] + 0.005,
        details=f"ring={ring_aabb}, cross_top={cross[1][2] if cross else None}, collar_bot={collar[0][2] if collar else None}",
    )
    ctx.expect_within(
        temp_ring,
        column,
        axes="xy",
        margin=0.008,
        name="temperature ring wraps around the column (XY within column + margin)",
    )

    # ----- ring articulation is non-fixed (revolute about Z)
    ctx.check(
        "temperature ring has a non-fixed revolute joint about Z",
        ring_rot.articulation_type == ArticulationType.REVOLUTE
        and tuple(ring_rot.axis) == (0.0, 0.0, 1.0)
        and ring_rot.motion_limits is not None
        and abs(ring_rot.motion_limits.lower + math.pi) < 1e-6
        and abs(ring_rot.motion_limits.upper - math.pi) < 1e-6,
    )

    # ----- ring pose: rotating moves the indicator
    ring_rest = ctx.part_world_aabb(temp_ring)
    with ctx.pose({ring_rot: math.pi / 2.0}):
        ring_rotated = ctx.part_world_aabb(temp_ring)
    ctx.check(
        "temperature ring rotation moves the part (indicator sweeps)",
        ring_rest is not None
        and ring_rotated is not None
        and (
            abs(ring_rotated[0][0] - ring_rest[0][0]) > 1e-6
            or abs(ring_rotated[0][1] - ring_rest[0][1]) > 1e-6
            or abs(ring_rotated[1][0] - ring_rest[1][0]) > 1e-6
            or abs(ring_rotated[1][1] - ring_rest[1][1]) > 1e-6
        ),
        details=f"rest={ring_rest}, rotated={ring_rotated}",
    )

    # ----- shallow ribbing on spray head
    rib_0 = ctx.part_element_world_aabb(spout, elem="spray_rib_0")
    rib_4 = ctx.part_element_world_aabb(spout, elem="spray_rib_4")
    ctx.check(
        "spray head has ribbing elements around the tip sleeve",
        rib_0 is not None
        and rib_4 is not None
        and sleeve is not None
        and rib_0[0][2] >= sleeve[0][2] - 0.005
        and rib_0[1][2] <= sleeve[1][2] + 0.005,
        details=f"rib_0={rib_0}, rib_4={rib_4}, sleeve={sleeve}",
    )

    # ----- thin seam at the swivel collar
    seam = ctx.part_element_world_aabb(spout, elem="collar_seam")
    ctx.check(
        "thin seam ring exists at the swivel collar junction",
        seam is not None
        and collar is not None
        and abs(seam[0][2] - collar[1][2]) <= 0.003
        and (seam[1][2] - seam[0][2]) <= 0.003,
        details=f"seam={seam}, collar_top={collar[1][2] if collar else None}",
    )

    # ----- spout swivel joint remains functional
    ctx.check(
        "spout swivel is revolute about the vertical column axis",
        swivel.articulation_type == ArticulationType.REVOLUTE
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and abs(swivel.motion_limits.lower + SWIVEL_LIMIT) < 1e-6
        and abs(swivel.motion_limits.upper - SWIVEL_LIMIT) < 1e-6,
    )

    # ----- swivel pose
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    with ctx.pose({swivel: 1.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="tip_sleeve")
    ctx.check(
        "spout swivel carries the outlet sideways about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 1e-4
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.05,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ----- lever joints
    for pivot, name in ((pivot_0, "lever_pivot_0"), (pivot_1, "lever_pivot_1")):
        ctx.check(
            f"{name} is revolute -90..0 deg about the valve Y axis",
            pivot.articulation_type == ArticulationType.REVOLUTE
            and tuple(pivot.axis) == (0.0, -1.0, 0.0)
            and pivot.motion_limits is not None
            and abs(pivot.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(pivot.motion_limits.upper) < 1e-6,
        )

    # Lever boss seating
    for lever, name in ((lever_0, "pin_lever_0"), (lever_1, "pin_lever_1")):
        ctx.expect_overlap(
            lever,
            column,
            axes="z",
            elem_a="lever_boss",
            elem_b="cross_tube",
            min_overlap=0.003,
            name=f"{name} boss seats into the valve cylinder",
        )

    return ctx.report()


object_model = build_object_model()
