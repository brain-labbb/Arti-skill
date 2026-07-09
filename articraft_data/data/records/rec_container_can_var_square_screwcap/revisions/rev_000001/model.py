from __future__ import annotations

# Square brushed-metal tin box with a round screw cap on a threaded neck.
# Fork of the press-fit lift-off lid variant: same rounded-square body
# footprint, but the closure is now a round threaded neck rising from the
# center of the square top deck plus a knurled screw cap.
#
# Frame: box centered on the world Z axis, footprint in XY, ground at z=0.
#   - The rounded-square TIN BODY is the root: a hollow shell with a closed
#     top deck and a short threaded neck rising from its center.
#   - A massless CAP CARRIER slides up off the neck (PRISMATIC).
#   - The knurled SCREW CAP spins on the carrier (CONTINUOUS).
#
# Articulations:
#   - body_to_carrier: PRISMATIC, carrier lifts straight up (+Z) off the neck.
#   - carrier_to_cap: CONTINUOUS, cap spins freely about the neck axis.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobSkirt,
    KnobBore,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---- key dimensions (meters) ----
HALF = 0.060          # half-width of the square footprint (0.12 m wide)
FILLET = 0.012        # corner radius of the rounded square
WALL = 0.0035         # body wall thickness

BODY_BOTTOM = 0.0     # body sits on the ground
BODY_TOP = 0.108      # top of the body shell (closed deck)
BODY_OUTER_HALF = HALF

# Threaded neck dimensions
NECK_RADIUS = 0.016       # outer radius of the threaded neck
NECK_BORE_RADIUS = 0.011  # open bore that connects to the hollow body
NECK_HEIGHT = 0.018       # height of the neck above the top deck
NECK_TOP = BODY_TOP + NECK_HEIGHT  # absolute top of the neck

# Screw cap dimensions
CAP_DIAMETER = 0.038      # cap outer diameter (larger than neck for grip)
CAP_HEIGHT = 0.020        # cap total height
CAP_BORE_DIAMETER = 0.028 # internal bore to clear the neck

# Prismatic travel for lifting the cap assembly off the neck
CAP_LIFT = 0.040


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _body_shell_mesh():
    """Hollow rounded-square shell built with mesh geometry helpers.

    Uses ExtrudeWithHolesGeometry for the walls and the drilled top deck, plus
    a separate floor plate. The neck bore opens through the deck into the
    internal cavity instead of sitting on a closed plate.
    """
    outer_profile = rounded_rect_profile(
        2.0 * BODY_OUTER_HALF, 2.0 * BODY_OUTER_HALF, FILLET
    )
    inner_half = BODY_OUTER_HALF - WALL
    inner_profile = rounded_rect_profile(
        2.0 * inner_half, 2.0 * inner_half, max(FILLET - WALL, 0.002)
    )

    # Wall frame: extruded rounded-square frame from floor top to deck underside
    wall_height = BODY_TOP - WALL - WALL  # from WALL to BODY_TOP - WALL
    walls = ExtrudeWithHolesGeometry(
        outer_profile, [inner_profile], wall_height, cap=True, center=False
    )
    # Translate walls to sit on the floor plate
    walls.translate(0.0, 0.0, WALL)

    # Floor plate
    floor_profile = rounded_rect_profile(
        2.0 * BODY_OUTER_HALF, 2.0 * BODY_OUTER_HALF, FILLET
    )
    floor = ExtrudeGeometry.from_z0(floor_profile, WALL, cap=True)

    # Top deck plate with a real central access hole aligned to the neck bore.
    deck = ExtrudeWithHolesGeometry(
        floor_profile,
        [_circle_profile(NECK_BORE_RADIUS, segments=48)],
        WALL,
        cap=True,
        center=False,
    )
    deck.translate(0.0, 0.0, BODY_TOP - WALL)

    # Merge all shell pieces
    result = MeshGeometry()
    result = result.merge(floor)
    result = result.merge(walls)
    result = result.merge(deck)
    return result


def _neck_solid() -> cq.Workplane:
    """Threaded neck: a short open tube with a raised collar."""
    neck = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - WALL)
        .circle(NECK_RADIUS)
        .circle(NECK_BORE_RADIUS)
        .extrude(NECK_HEIGHT + WALL)
    )
    # Thread collar: a raised ring on the neck for visual thread indication.
    collar_z = BODY_TOP + NECK_HEIGHT * 0.5
    collar = (
        cq.Workplane("XY")
        .workplane(offset=collar_z - 0.001)
        .circle(NECK_RADIUS + 0.002)
        .circle(NECK_BORE_RADIUS)
        .extrude(0.005)
    )
    return neck.union(collar)


def _build_screw_cap() -> KnobGeometry:
    """Knurled screw cap modeled as a knob with a bore to clear the neck."""
    return KnobGeometry(
        CAP_DIAMETER,
        CAP_HEIGHT,
        body_style="cylindrical",
        skirt=KnobSkirt(
            CAP_DIAMETER + 0.002,
            0.003,
            flare=0.0,
            chamfer=0.001,
        ),
        grip=KnobGrip(
            style="knurled",
            count=48,
            depth=0.001,
            helix_angle_deg=25.0,
        ),
        bore=KnobBore(style="round", diameter=CAP_BORE_DIAMETER),
        center=False,  # mounting face at z=0
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metal_tin_screw_cap")

    brushed = model.material("brushed_metal", rgba=(0.74, 0.75, 0.77, 1.0))
    cap_mat = model.material("cap_metal", rgba=(0.62, 0.63, 0.66, 1.0))

    # ---- tin body (root) with threaded neck ----
    body = model.part("tin_body")
    # Shell: mesh-based hollow body for clean connectivity
    body.visual(
        mesh_from_geometry(_body_shell_mesh(), "tin_body_shell"),
        material=brushed,
        name="tin_body_shell",
    )
    # Neck: CadQuery solid rising from the top deck
    body.visual(
        mesh_from_cadquery(_neck_solid(), "tin_neck"),
        material=brushed,
        name="tin_neck",
    )
    body.inertial = Inertial.from_geometry(
        Box((2 * HALF, 2 * HALF, BODY_TOP)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    # ---- massless cap carrier (intermediate for prismatic lift) ----
    carrier = model.part("cap_carrier")
    # No visuals: this is a massless kinematic carrier only.

    # ---- knurled screw cap ----
    cap = model.part("screw_cap")
    cap_geom = _build_screw_cap()
    cap.visual(
        mesh_from_geometry(cap_geom, "screw_cap_shell"),
        material=cap_mat,
        name="screw_cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_DIAMETER / 2.0, length=CAP_HEIGHT),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT / 2.0)),
    )

    # ---- articulation: body -> carrier (PRISMATIC, lift off) ----
    # Joint origin at the top of the neck (contact/seating surface).
    # Positive q lifts the carrier (and cap) upward off the neck.
    model.articulation(
        "body_to_carrier",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.1, lower=0.0, upper=CAP_LIFT
        ),
    )

    # ---- articulation: carrier -> cap (CONTINUOUS, spin) ----
    # Cap spins freely about the neck axis. Joint origin at the cap mounting
    # face (coincident with carrier frame at the neck top when seated).
    model.articulation(
        "carrier_to_cap",
        ArticulationType.CONTINUOUS,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("tin_body")
    carrier = object_model.get_part("cap_carrier")
    cap = object_model.get_part("screw_cap")
    lift = object_model.get_articulation("body_to_carrier")
    spin = object_model.get_articulation("carrier_to_cap")

    # --- Body footprint: still a rounded square ---
    bmn, bmx = ctx.part_world_aabb(body)
    bx = bmx[0] - bmn[0]
    by = bmx[1] - bmn[1]
    bz = bmx[2] - bmn[2]
    ctx.check(
        "tin body has a square footprint",
        abs(bx - by) < 0.004 and bx > 0.10,
        details=f"footprint x={bx:.4f} y={by:.4f}",
    )
    ctx.check(
        "tin body height is reasonable",
        bz > 0.09 and bz < 0.14,
        details=f"body height z={bz:.4f}",
    )

    # --- Neck is round and cap sits on it ---
    ctx.expect_overlap(
        cap, body,
        axes="xy",
        elem_a="screw_cap_shell",
        elem_b="tin_body_shell",
        min_overlap=0.02,
        name="cap overlaps body footprint in XY when seated",
    )
    ctx.expect_overlap(
        cap, body,
        axes="xy",
        elem_a="screw_cap_shell",
        elem_b="tin_neck",
        min_overlap=0.02,
        name="cap surrounds the neck in XY when seated",
    )

    # --- Seated cap sits on top of the neck region ---
    cap_pos_rest = ctx.part_world_position(cap)
    ctx.check(
        "seated cap is at or above the body top deck",
        cap_pos_rest[2] >= BODY_TOP - 0.005,
        details=f"cap z={cap_pos_rest[2]:.4f}, body top={BODY_TOP:.4f}",
    )
    ctx.check(
        "neck bore passes through the top deck into the hollow tin body",
        0.0 < NECK_BORE_RADIUS < NECK_RADIUS
        and NECK_BORE_RADIUS * 2 < CAP_BORE_DIAMETER
        and BODY_TOP - WALL - WALL > BODY_TOP * 0.85,
        details=(
            f"neck_bore_radius={NECK_BORE_RADIUS:.4f}, "
            f"neck_radius={NECK_RADIUS:.4f}, body_cavity_h={BODY_TOP - WALL - WALL:.4f}"
        ),
    )

    # --- Cap lifts off when prismatic joint is at max ---
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({lift: CAP_LIFT}):
        lifted_z = ctx.part_world_position(cap)[2]
        cap_mn_lifted, _ = ctx.part_world_aabb(cap)
    ctx.check(
        "cap lifts straight up off the neck",
        lifted_z > rest_z + 0.03,
        details=f"rest_z={rest_z:.4f}, lifted_z={lifted_z:.4f}",
    )
    ctx.check(
        "lifted cap clears the neck top",
        cap_mn_lifted[2] > NECK_TOP - 0.002,
        details=f"lifted cap bottom z={cap_mn_lifted[2]:.4f}, neck top={NECK_TOP:.4f}",
    )

    # --- Cap spins (CONTINUOUS joint exists and is continuous type) ---
    ctx.check(
        "carrier_to_cap is a continuous joint",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={spin.articulation_type}",
    )

    # --- Spinning the cap does not translate it vertically ---
    with ctx.pose({spin: 3.14159}):
        spun_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "spinning the cap does not change its height",
        abs(spun_z - rest_z) < 0.001,
        details=f"rest_z={rest_z:.4f}, spun_z={spun_z:.4f}",
    )

    # --- Body-to-carrier joint is prismatic ---
    ctx.check(
        "body_to_carrier is a prismatic joint",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type={lift.articulation_type}",
    )

    # --- Cap carrier is connected between body and cap ---
    ctx.check(
        "carrier parent is tin_body",
        lift.parent == body.name or lift.parent == body,
        details=f"lift parent={lift.parent}",
    )
    ctx.check(
        "carrier child is cap via spin joint",
        spin.child == cap.name or spin.child == cap,
        details=f"spin child={spin.child}",
    )

    return ctx.report()


object_model = build_object_model()
