"""Large ground-station parabolic antenna dish on a pedestal mount.

Reference: photo of a warm-grey painted steel earth-station antenna seen from
behind/below — square anchor base plate, tapered skirt, cylindrical pedestal
column with bolt flanges, an elevation clevis head with a long adjustment jack
rod hanging beside the column, and a deep paraboloid reflector tilted skyward.
The dish back carries a radial truss lattice (ribs + hoop rings) and a central
rear hub can with a dark feed opening through the vertex.

Frame: +Z up, ground at z=0, dish boresight tilted toward +X and up.
Approximate real scale: reflector diameter 3.0 m, pedestal ~1.4 m tall.

Articulations:
- azimuth_slew  (REVOLUTE about Z): mount head + dish rotate on the column.
- dish_elevation (REVOLUTE about Y): dish tilts about the clevis pivot; the
  joint origin is pre-tilted so q=0 matches the photo's skyward attitude.
"""

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

# ---------------------------------------------------------------- constants

# Reflector paraboloid
DISH_R = 1.50          # reflector rim radius
FOCAL = 1.25           # focal length -> depth = R^2/(4f) = 0.45
SHELL_T = 0.025        # shell thickness (measured along z)
HOLE_R = 0.155         # vertex feed hole radius
VZ = 0.50              # vertex plane height in the dish (child) frame

# Rear hub can
HUB_RO = 0.21
HUB_RI = 0.14
HUB_Z0 = 0.16
HUB_Z1 = 0.54

# Back truss lattice
RIB_COUNT = 12
RIB_R0 = 0.19
RIB_R1 = 1.47
RIB_T = 0.035          # rib plate thickness (tangential)
RIB_D0 = 0.26          # truss depth at hub
RIB_D1 = 0.075         # truss depth at rim
RING_RADII = (0.72, 1.26)

# Pedestal
PLATE_HALF = 0.475
PLATE_H = 0.06
COL_R = 0.115
COL_TOP = 1.35
FLANGE_R = 0.155

# Mount head / pivot
AZ_Z = 1.38            # azimuth joint height (top of column flange)
PIVOT_Z = 0.32         # elevation pivot height in head-local frame
PIN_R = 0.045
TILT0 = 0.61           # built-in skyward tilt at q=0 (rad, about +Y)

ELEV_LOWER = -0.30
ELEV_UPPER = 0.55


def _z_front(r: float) -> float:
    """Concave (front) reflector surface height in the dish frame."""
    return VZ + r * r / (4.0 * FOCAL)


def _rib_depth(r: float) -> float:
    t = (r - RIB_R0) / (RIB_R1 - RIB_R0)
    return RIB_D0 + t * (RIB_D1 - RIB_D0)


# ---------------------------------------------------------------- cq shapes


def _pedestal_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .rect(2 * PLATE_HALF, 2 * PLATE_HALF)
        .extrude(PLATE_H)
    )
    skirt = (
        cq.Workplane("XY", origin=(0.0, 0.0, PLATE_H))
        .rect(0.50, 0.50)
        .workplane(offset=0.28)
        .rect(0.24, 0.24)
        .loft()
    )
    column = (
        cq.Workplane("XY", origin=(0.0, 0.0, 0.30))
        .circle(COL_R)
        .extrude(COL_TOP - 0.30)
    )
    collar = (
        cq.Workplane("XY", origin=(0.0, 0.0, 1.14))
        .circle(0.15)
        .extrude(0.04)
    )
    top_flange = (
        cq.Workplane("XY", origin=(0.0, 0.0, 1.33))
        .circle(FLANGE_R)
        .extrude(AZ_Z - 1.33)
    )
    body = plate.union(skirt).union(column).union(collar).union(top_flange)
    # anchor bolt heads on the base plate corners
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            bolt = (
                cq.Workplane("XY", origin=(sx * 0.40, sy * 0.40, PLATE_H))
                .circle(0.016)
                .extrude(0.022)
            )
            body = body.union(bolt)
    return body


def _reflector_shape() -> cq.Workplane:
    n = 16
    front = []
    for i in range(n):
        r = HOLE_R + (DISH_R - HOLE_R) * i / (n - 1)
        front.append((r, _z_front(r)))
    back = [(r, z - SHELL_T) for (r, z) in reversed(front)]
    pts = front + back
    profile = cq.Workplane("XZ").polyline(pts).close()
    return profile.revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def _rib_shape() -> cq.Workplane:
    n = 12
    top = []
    for i in range(n):
        r = RIB_R0 + (RIB_R1 - RIB_R0) * i / (n - 1)
        top.append((r, _z_front(r) - SHELL_T + 0.010))
    bottom = [(r, z - _rib_depth(r)) for (r, z) in reversed(top)]
    pts = top + bottom
    return (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(RIB_T / 2.0, both=True)
    )


def _truss_shape() -> cq.Workplane:
    rib = _rib_shape()
    truss = rib
    for i in range(1, RIB_COUNT):
        ang = 360.0 * i / RIB_COUNT
        truss = truss.union(rib.rotate((0, 0, 0), (0, 0, 1), ang))
    # hoop stiffener rings tying the ribs together
    for rr in RING_RADII:
        zb = _z_front(rr) - SHELL_T + 0.010 - _rib_depth(rr)
        ring = (
            cq.Workplane("XZ")
            .polyline(
                [
                    (rr - 0.018, zb),
                    (rr + 0.018, zb),
                    (rr + 0.018, zb + 0.060),
                    (rr - 0.018, zb + 0.060),
                ]
            )
            .close()
            .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        )
        truss = truss.union(ring)
    return truss


def _hub_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, HUB_Z0))
        .circle(HUB_RO)
        .circle(HUB_RI)
        .extrude(HUB_Z1 - HUB_Z0)
    )


# ---------------------------------------------------------------- model


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ground_station_antenna_dish")

    paint = model.material("steel_paint", rgba=(0.80, 0.78, 0.73, 1.0))
    face = model.material("dish_face", rgba=(0.89, 0.88, 0.85, 1.0))
    dark = model.material("feed_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    steel = model.material("bare_steel", rgba=(0.68, 0.67, 0.65, 1.0))

    # ---- pedestal (root): base plate + skirt + column + flanges
    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_shape(), "pedestal_body"),
        material=paint,
        name="pedestal_body",
    )

    # ---- azimuth mount head: bearing disc, riser, clevis plates, pivot pin,
    #      and the elevation jack rod hanging beside the column
    head = model.part("mount_head")
    head.visual(
        Cylinder(radius=0.155, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=paint,
        name="head_base_disc",
    )
    head.visual(
        Box((0.26, 0.30, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=paint,
        name="head_riser",
    )
    for side, sy in (("left", 1.0), ("right", -1.0)):
        head.visual(
            Box((0.26, 0.035, 0.24)),
            origin=Origin(xyz=(0.0, sy * 0.1495, 0.27)),
            material=paint,
            name=f"clevis_plate_{side}",
        )
    head.visual(
        Cylinder(radius=PIN_R, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    head.visual(
        Box((0.10, 0.06, 0.08)),
        origin=Origin(xyz=(-0.17, 0.0, 0.07)),
        material=paint,
        name="jack_bracket",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.85),
        origin=Origin(xyz=(-0.332, 0.0, -0.311), rpy=(0.0, 0.34, 0.0)),
        material=steel,
        name="jack_rod",
    )

    model.articulation(
        "azimuth_slew",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, AZ_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=200.0, velocity=0.5),
    )

    # ---- dish assembly: trunnion boss + web + hub can + reflector + truss.
    # Child frame origin sits at the elevation pivot; boresight is child +Z.
    # The joint origin rpy pre-tilts the dish skyward so q=0 matches the photo.
    dish = model.part("dish_assembly")
    dish.visual(
        Cylinder(radius=0.06, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paint,
        name="trunnion_boss",
    )
    dish.visual(
        Box((0.32, 0.05, 0.38)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=paint,
        name="hub_web",
    )
    dish.visual(
        mesh_from_cadquery(_hub_shape(), "hub_can"),
        material=paint,
        name="hub_can",
    )
    dish.visual(
        Cylinder(radius=0.146, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=dark,
        name="feed_plug",
    )
    dish.visual(
        mesh_from_cadquery(_reflector_shape(), "reflector_shell"),
        material=face,
        name="reflector_shell",
    )
    dish.visual(
        mesh_from_cadquery(_truss_shape(), "back_truss"),
        material=paint,
        name="back_truss",
    )

    model.articulation(
        "dish_elevation",
        ArticulationType.REVOLUTE,
        parent=head,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(0.0, TILT0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ELEV_LOWER, upper=ELEV_UPPER, effort=400.0, velocity=0.2
        ),
    )

    return model


# ---------------------------------------------------------------- tests


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    head = object_model.get_part("mount_head")
    dish = object_model.get_part("dish_assembly")
    azimuth = object_model.get_articulation("azimuth_slew")
    elevation = object_model.get_articulation("dish_elevation")

    # Intentional local embeddings.
    ctx.allow_overlap(
        dish,
        head,
        elem_a="trunnion_boss",
        elem_b="pivot_pin",
        reason="elevation pin is captured inside the trunnion bore",
    )
    ctx.allow_overlap(
        dish,
        head,
        elem_a="hub_web",
        elem_b="pivot_pin",
        reason="pivot pin passes through the hub support web",
    )
    ctx.allow_overlap(
        head,
        pedestal,
        elem_a="head_base_disc",
        elem_b="pedestal_body",
        reason="azimuth bearing disc seats into the column top flange",
    )

    # --- hero geometry: reflector is a full ~3 m dish held above the pedestal
    refl = ctx.part_element_world_aabb(dish, elem="reflector_shell")
    ctx.check(
        "reflector spans ~3 m across the slew axis",
        refl is not None and (refl[1][1] - refl[0][1]) > 2.9,
        details=f"reflector aabb={refl}",
    )
    ctx.check(
        "reflector rim stays above the pedestal base",
        refl is not None and refl[0][2] > 0.9,
        details=f"reflector aabb={refl}",
    )

    # feed opening: dark plug sits inside the hub can bore (dish frame stack)
    plug = ctx.part_element_world_aabb(dish, elem="feed_plug")
    hub = ctx.part_element_world_aabb(dish, elem="hub_can")
    ctx.check(
        "dark feed plug is contained inside the rear hub can",
        plug is not None
        and hub is not None
        and all(plug[0][i] > hub[0][i] - 1e-6 and plug[1][i] < hub[1][i] + 1e-6 for i in range(3)),
        details=f"plug={plug} hub={hub}",
    )

    # jack rod hangs beside the column, below the mount head
    rod = ctx.part_element_world_aabb(head, elem="jack_rod")
    ctx.check(
        "jack rod reaches down alongside the column",
        rod is not None and rod[0][2] < 0.75 and rod[1][0] < -0.10,
        details=f"rod aabb={rod}",
    )

    # dish is retained on the pivot pin (captured trunnion)
    ctx.expect_contact(
        dish,
        head,
        elem_a="trunnion_boss",
        elem_b="pivot_pin",
        name="trunnion rides on the elevation pin",
    )
    ctx.expect_contact(
        head,
        pedestal,
        elem_a="head_base_disc",
        elem_b="pedestal_body",
        name="mount head seats on the column flange",
    )

    # --- elevation articulation: positive q pitches the boresight down toward
    # the horizon, dropping the front rim; negative q tips it further skyward.
    c0 = ctx.part_element_world_aabb(dish, elem="reflector_shell")
    with ctx.pose({elevation: ELEV_UPPER}):
        c_up = ctx.part_element_world_aabb(dish, elem="reflector_shell")
        ctx.check(
            "elevation upper limit drops the front rim toward the horizon",
            c0 is not None and c_up is not None and c_up[0][2] < c0[0][2] - 0.30,
            details=f"min_z q=0: {c0[0][2] if c0 else None} -> {c_up[0][2] if c_up else None}",
        )
        ctx.check(
            "tilted dish still clears the ground and base plate",
            c_up is not None and c_up[0][2] > 0.25,
            details=f"aabb={c_up}",
        )
    with ctx.pose({elevation: ELEV_LOWER}):
        c_dn = ctx.part_element_world_aabb(dish, elem="reflector_shell")
        ctx.check(
            "elevation lower limit raises the front rim (points closer to zenith)",
            c0 is not None and c_dn is not None and c_dn[0][2] > c0[0][2] + 0.10,
            details=f"min_z q=0: {c0[0][2] if c0 else None} -> {c_dn[0][2] if c_dn else None}",
        )

    # --- azimuth articulation: slewing 90 deg swings the tilted dish centroid
    # from the +X side around to the +Y side (3D AABB-center displacement).
    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    with ctx.pose({azimuth: math.pi / 2.0}):
        c_az = ctx.part_element_world_aabb(dish, elem="reflector_shell")
        ctx.check(
            "azimuth slew swings the dish centroid sideways",
            c0 is not None
            and c_az is not None
            and math.dist(_center(c0), _center(c_az)) > 0.30
            and _center(c_az)[1] > 0.25,
            details=f"center q=0: {_center(c0) if c0 else None} -> {_center(c_az) if c_az else None}",
        )
        rod_az = ctx.part_element_world_aabb(head, elem="jack_rod")
        ctx.check(
            "jack rod slews with the mount head",
            rod_az is not None and rod_az[0][1] < -0.10,
            details=f"rod aabb={rod_az}",
        )

    return ctx.report()


object_model = build_object_model()
