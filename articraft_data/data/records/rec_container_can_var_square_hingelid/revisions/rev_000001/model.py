from __future__ import annotations

# Square brushed-metal tin box with a hinged flip lid (fork of press-fit variant).
# Frame: box centered on Z axis, footprint in XY, ground at z=0.
#   - Rounded-square TIN BODY is root: hollow shell with smooth rim.
#   - Flat LID hinged along the rear top edge, swings up and back.
# Articulation:
#   - body_to_lid: REVOLUTE, hinge at rear rim, axis along +X,
#     positive q opens the lid upward and backward (~130°).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
HALF = 0.060           # half-width of the square footprint (0.12 m wide)
FILLET = 0.012         # corner radius of the rounded square
WALL = 0.0035          # body wall thickness

BODY_BOTTOM = 0.0      # body sits on the ground
RIM_TOP = 0.108        # top of the body rim
BODY_OUTER_HALF = HALF

LID_OUTER_HALF = HALF + 0.0015   # lid overhangs the body wall slightly
LID_THICKNESS = 0.004            # lid plate thickness

# ---- hinge dimensions ----
KNUCKLE_RADIUS = 0.003           # hinge knuckle radius
KNUCKLE_LEN = 0.014              # each knuckle segment length along X
KNUCKLE_GAP = 0.001              # gap between adjacent knuckle segments
TOTAL_KNUCKLES = 5               # 3 on body + 2 on lid, interleaved
HINGE_SPAN = TOTAL_KNUCKLES * (KNUCKLE_LEN + KNUCKLE_GAP)  # total hinge width
SEGMENT_PITCH = KNUCKLE_LEN + KNUCKLE_GAP

LID_OPEN_ANGLE = 2.27            # ~130 degrees


def _rounded_square_prism(half: float, fillet: float, z0: float, z1: float) -> cq.Workplane:
    """Rounded-square vertical prism from z0 to z1."""
    return (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .rect(2.0 * half, 2.0 * half)
        .extrude(z1 - z0)
        .edges("|Z")
        .fillet(fillet)
    )


def _knuckle_x_center(index: int) -> float:
    """X center of the i-th hinge knuckle segment (0-indexed, centered on x=0)."""
    return -HINGE_SPAN / 2.0 + SEGMENT_PITCH * index + KNUCKLE_LEN / 2.0


def _knuckle_cylinder(x_center: float, y_center: float, z_center: float) -> cq.Workplane:
    """Hinge knuckle cylinder along X, centered at (x_center, y_center, z_center)."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=x_center - KNUCKLE_LEN / 2.0)
        .circle(KNUCKLE_RADIUS)
        .extrude(KNUCKLE_LEN)
        .translate((0.0, y_center, z_center))
    )


def _body_solid() -> cq.Workplane:
    """Hollow rounded-square shell (no rabbet — hinged lid sits flush on rim)."""
    outer = _rounded_square_prism(BODY_OUTER_HALF, FILLET, BODY_BOTTOM, RIM_TOP)

    # Inner cavity: open at top, leaving a floor
    inner_half = BODY_OUTER_HALF - WALL
    inner = _rounded_square_prism(
        inner_half, max(FILLET - WALL, 0.002), WALL, RIM_TOP + 0.001
    )
    shell = outer.cut(inner)

    # Hinge knuckles on rear top rim (body gets even indices: 0, 2, 4)
    for i in range(TOTAL_KNUCKLES):
        if i % 2 == 0:
            x_c = _knuckle_x_center(i)
            knuckle = _knuckle_cylinder(x_c, -BODY_OUTER_HALF, RIM_TOP)
            shell = shell.union(knuckle)

    return shell


def _lid_solid() -> cq.Workplane:
    """Flat lid plate with hinge knuckles, modeled in lid local frame.

    Lid local frame: origin at the hinge pin center.
    +Y extends forward (toward front of box), +Z is up from plate bottom.
    """
    # Main plate: rounded rect centered at (0, HALF) in XY, from z=0 to z=LID_THICKNESS
    plate = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, HALF)
        .rect(2.0 * LID_OUTER_HALF, 2.0 * HALF)
        .extrude(LID_THICKNESS)
        .edges("|Z")
        .fillet(FILLET)
    )

    # Reinforcement rib at the hinge edge: a thickened strip connecting knuckles to plate
    rib = (
        cq.Workplane("XY")
        .workplane(offset=-KNUCKLE_RADIUS)
        .center(0.0, 0.004)
        .rect(HINGE_SPAN + KNUCKLE_LEN, 0.008)
        .extrude(KNUCKLE_RADIUS + LID_THICKNESS)
    )
    plate = plate.union(rib)

    # Hinge knuckles on lid rear edge (lid gets odd indices: 1, 3)
    for i in range(TOTAL_KNUCKLES):
        if i % 2 == 1:
            x_c = _knuckle_x_center(i)
            knuckle = _knuckle_cylinder(x_c, 0.0, 0.0)
            plate = plate.union(knuckle)

    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_metal_tin_box")

    brushed = model.material("brushed_metal", rgba=(0.74, 0.75, 0.77, 1.0))
    brushed_lid = model.material("brushed_metal_lid", rgba=(0.72, 0.73, 0.75, 1.0))

    # ---- tin body (root) ----
    body = model.part("tin_body")
    body.visual(
        mesh_from_cadquery(_body_solid(), "tin_body_shell"),
        material=brushed,
        name="tin_body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((2 * HALF, 2 * HALF, RIM_TOP)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP / 2.0)),
    )

    # ---- hinged lid ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_shell"),
        material=brushed_lid,
        name="lid_shell",
    )
    lid.inertial = Inertial.from_geometry(
        Box((2 * LID_OUTER_HALF, 2 * HALF, LID_THICKNESS + 2 * KNUCKLE_RADIUS)),
        mass=0.05,
        origin=Origin(xyz=(0.0, HALF, LID_THICKNESS / 2.0)),
    )

    # Revolute hinge at rear top edge, axis along +X
    # Positive q rotates +Y (lid front) toward +Z (up), opening the lid.
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -BODY_OUTER_HALF, RIM_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0,
            lower=0.0, upper=LID_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("tin_body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    # Hinge interface: interleaved knuckles and lid rib share the pin axis
    # with the body rim — small local intentional overlap.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_shell",
        elem_b="tin_body_shell",
        reason=(
            "Interleaved hinge knuckles and lid reinforcement rib share the "
            "pin axis at the rear top rim; small local overlap with the body shell."
        ),
    )

    # ---- body geometry checks (same footprint as parent) ----
    bmn, bmx = ctx.part_world_aabb(body)
    bx = bmx[0] - bmn[0]
    by = bmx[1] - bmn[1]
    bz = bmx[2] - bmn[2]
    ctx.check(
        "tin body has a square footprint",
        abs(bx - by) < 0.006 and bx > 0.10,
        details=f"footprint x={bx:.4f} y={by:.4f}",
    )
    ctx.check(
        "tin body height is in range",
        0.09 < bz < 0.14,
        details=f"body height z={bz:.4f}",
    )

    # ---- closed-lid checks ----
    ctx.expect_overlap(
        lid, body,
        axes="xy",
        elem_a="lid_shell", elem_b="tin_body_shell",
        min_overlap=0.08,
        name="closed lid covers the tin footprint",
    )

    lmn0, lmx0 = ctx.part_world_aabb(lid)
    ctx.check(
        "closed lid sits near rim top",
        lmn0[2] >= RIM_TOP - 0.006,
        details=f"lid bottom z={lmn0[2]:.4f}, rim top={RIM_TOP:.4f}",
    )
    ctx.check(
        "closed lid top is above rim",
        lmx0[2] > RIM_TOP,
        details=f"lid top z={lmx0[2]:.4f}, rim top={RIM_TOP:.4f}",
    )

    # ---- articulated hinge checks ----
    ctx.check(
        "lid hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )

    rest_top_z = lmx0[2]
    rest_bottom_z = lmn0[2]

    with ctx.pose({hinge: LID_OPEN_ANGLE}):
        opened_mn, opened_mx = ctx.part_world_aabb(lid)

    ctx.check(
        "opened lid rises well above closed position",
        opened_mx[2] > rest_top_z + 0.03,
        details=f"closed top z={rest_top_z:.4f}, opened max z={opened_mx[2]:.4f}",
    )
    ctx.check(
        "opened lid extends above hinge axis",
        opened_mx[2] > RIM_TOP + 0.06,
        details=f"opened max z={opened_mx[2]:.4f}, hinge z={RIM_TOP:.4f}",
    )
    ctx.check(
        "opened lid features swing below hinge axis",
        opened_mn[2] < RIM_TOP,
        details=f"opened min z={opened_mn[2]:.4f}, hinge z={RIM_TOP:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
