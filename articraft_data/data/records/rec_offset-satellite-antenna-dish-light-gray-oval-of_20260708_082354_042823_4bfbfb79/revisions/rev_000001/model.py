from __future__ import annotations

"""Offset satellite antenna dish (reference: Astronomy/Antenna dish/002.png).

Light-gray offset reflector seen from behind in the reference photo:
- triangular pressed-steel floor base with gusset ribs and a straight pole mast
- azimuth clamp bracket sleeved onto the mast top (REVOLUTE about the mast axis)
- dish assembly (reflector shell + trapezoid rear stiffening frame + LNB feed
  arm + feedhorn) pivoting on a horizontal elevation bolt (REVOLUTE)
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- materials
DISH_GRAY = Material(name="dish_gray", rgba=(0.83, 0.84, 0.85, 1.0))
STEEL_GRAY = Material(name="steel_gray", rgba=(0.62, 0.63, 0.65, 1.0))
ZINC_GRAY = Material(name="zinc_gray", rgba=(0.72, 0.73, 0.74, 1.0))
DARK_STEEL = Material(name="dark_steel", rgba=(0.30, 0.30, 0.32, 1.0))
LNB_WHITE = Material(name="lnb_white", rgba=(0.92, 0.92, 0.90, 1.0))

# ---------------------------------------------------------------- dimensions
MAST_R = 0.030
MAST_H = 1.02
PLATE_T = 0.012

SLEEVE_R = 0.040
SLEEVE_H = 0.16
AZ_Z0 = 0.88  # azimuth joint origin height (bottom of clamp sleeve)

PIVOT_X = 0.09  # elevation pivot, in az_clamp local frame
PIVOT_Z = 0.08

# Reflector: spherical-cap shell, aperture radius A, sphere radius RR.
RR = 1.30
SHELL_T = 0.012
A = 0.50
CAP_D = RR - math.sqrt(RR * RR - A * A)  # cap depth ~0.0985

# Dish boresight tilt back angle (offset-dish look) and mesh mounting rotation.
BETA = 0.30
SB, CB = math.sin(BETA), math.cos(BETA)
PHI = -(math.pi / 2.0 + BETA)  # Origin pitch: mesh +z (cap bulge) -> back/down

# Reflector rim-center position in the dish_assembly frame (pivot at origin).
DISH_POS = (0.17, 0.0, 0.05)
FRAME_Z = CAP_D + 0.030  # rear frame plane, in dish-mesh z coordinates


def dish_pt(xm: float, ym: float, zm: float) -> tuple[float, float, float]:
    """Map a dish-mesh-frame point into the dish_assembly frame."""
    return (
        DISH_POS[0] - SB * xm - CB * zm,
        ym,
        DISH_POS[2] + CB * xm - SB * zm,
    )


def _reflector_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").sphere(RR)
    inner = cq.Workplane("XY").sphere(RR - SHELL_T)
    shell = outer.cut(inner)
    slab = (
        cq.Workplane("XY")
        .box(2.2 * A, 2.2 * A, CAP_D + SHELL_T + 0.02)
        .translate((0.0, 0.0, (RR - CAP_D) + (CAP_D + SHELL_T + 0.02) / 2.0))
    )
    # Keep only the +z cap; put the rim plane at mesh z=0 (bulge toward +z).
    return shell.intersect(slab).translate((0.0, 0.0, -(RR - CAP_D)))


def _base_plate_shape() -> cq.Workplane:
    pts = [(0.30, 0.0), (-0.20, 0.26), (-0.20, -0.26)]
    return cq.Workplane("XY").polyline(pts).close().extrude(PLATE_T)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_satellite_antenna_dish")

    # ---------------------------------------------------------- mast + base
    base = model.part("mast_base")
    base.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="base_plate",
        material=STEEL_GRAY,
    )
    base.visual(
        Cylinder(radius=0.045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        name="mast_weld_collar",
        material=STEEL_GRAY,
    )
    for i in range(3):
        ang = i * 2.0 * math.pi / 3.0
        base.visual(
            Box((0.13, 0.010, 0.045)),
            origin=Origin(
                xyz=(0.095 * math.cos(ang), 0.095 * math.sin(ang), 0.032),
                rpy=(0.0, 0.0, ang),
            ),
            name=f"base_gusset_{i}",
            material=STEEL_GRAY,
        )
    base.visual(
        Cylinder(radius=MAST_R, length=MAST_H),
        origin=Origin(xyz=(0.0, 0.0, MAST_H / 2.0)),
        name="mast_tube",
        material=ZINC_GRAY,
    )

    # ------------------------------------------------- azimuth clamp bracket
    az = model.part("az_clamp")
    az.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_H),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_H / 2.0)),
        name="clamp_sleeve",
        material=ZINC_GRAY,
    )
    for i, zb in enumerate((0.04, 0.12)):
        az.visual(
            Cylinder(radius=0.008, length=0.030),
            origin=Origin(xyz=(-0.045, 0.0, zb), rpy=(0.0, math.pi / 2.0, 0.0)),
            name=f"clamp_bolt_{i}",
            material=DARK_STEEL,
        )
    for name, sy in (("clevis_plate_left", 1.0), ("clevis_plate_right", -1.0)):
        az.visual(
            # Runs from inside the sleeve wall (x=0.005) out past the pivot,
            # welding the clevis to the clamp sleeve (clear of the mast, r=0.03).
            Box((0.11, 0.008, 0.13)),
            origin=Origin(xyz=(0.06, sy * 0.039, 0.075)),
            name=name,
            material=ZINC_GRAY,
        )
    az.visual(
        Cylinder(radius=0.010, length=0.114),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="pivot_pin",
        material=DARK_STEEL,
    )
    for name, sy in (("pivot_nut_left", 1.0), ("pivot_nut_right", -1.0)):
        az.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(
                xyz=(PIVOT_X, sy * 0.060, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)
            ),
            name=name,
            material=DARK_STEEL,
        )

    # ------------------------------------------------------- dish assembly
    dish = model.part("dish_assembly")
    dish.visual(
        mesh_from_cadquery(_reflector_shape(), "reflector_shell"),
        origin=Origin(xyz=DISH_POS, rpy=(0.0, PHI, 0.0)),
        name="reflector_shell",
        material=DISH_GRAY,
    )

    # Elevation hub straddled by the clevis, riding on the pivot pin.
    dish.visual(
        Box((0.05, 0.048, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="elev_hub",
        material=STEEL_GRAY,
    )
    # Boss joining the hub to the rear frame plane.
    dish.visual(
        Box((0.06, 0.05, 0.062)),
        origin=Origin(xyz=dish_pt(0.0, 0.0, 0.155), rpy=(0.0, PHI, 0.0)),
        name="frame_boss",
        material=STEEL_GRAY,
    )

    # Trapezoid rear stiffening frame: two rails + three cross bars + 4 pads.
    for name, sv in (("frame_rail_left", 1.0), ("frame_rail_right", -1.0)):
        dish.visual(
            Box((0.56, 0.018, 0.018)),
            origin=Origin(xyz=dish_pt(0.0, sv * 0.11, FRAME_Z), rpy=(0.0, PHI, 0.0)),
            name=name,
            material=ZINC_GRAY,
        )
    for i, u0 in enumerate((-0.26, 0.0, 0.26)):
        dish.visual(
            Box((0.018, 0.26, 0.018)),
            origin=Origin(xyz=dish_pt(u0, 0.0, FRAME_Z), rpy=(0.0, PHI, 0.0)),
            name=f"frame_cross_bar_{i}",
            material=ZINC_GRAY,
        )
    for i, (su, sv) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        u0, v0 = su * 0.26, sv * 0.11
        r = math.hypot(u0, v0)
        z_back = CAP_D - (RR - math.sqrt(RR * RR - r * r))
        z0 = z_back - 0.010  # embedded into the shell back
        z1 = FRAME_Z + 0.006  # through the frame bars
        dish.visual(
            Cylinder(radius=0.009, length=z1 - z0),
            origin=Origin(xyz=dish_pt(u0, v0, (z0 + z1) / 2.0), rpy=(0.0, PHI, 0.0)),
            name=f"frame_pad_{i}",
            material=ZINC_GRAY,
        )

    # LNB feed arm from the lower rim out to the feedhorn in front of the dish.
    p1 = dish_pt(-0.46, 0.0, 0.008)  # rooted inside the shell near the lower rim
    p2 = (0.53, 0.0, -0.21)  # arm tip at the LNB clamp
    dvec = (p2[0] - p1[0], p2[2] - p1[2])
    arm_len = math.hypot(*dvec)
    dish.visual(
        Cylinder(radius=0.012, length=arm_len),
        origin=Origin(
            xyz=((p1[0] + p2[0]) / 2.0, 0.0, (p1[2] + p2[2]) / 2.0),
            rpy=(0.0, math.atan2(dvec[0], dvec[1]), 0.0),
        ),
        name="feed_arm",
        material=DISH_GRAY,
    )
    dish.visual(
        Box((0.028, 0.030, 0.055)),
        origin=Origin(xyz=(0.53, 0.0, -0.19)),
        name="lnb_clamp",
        material=DARK_STEEL,
    )
    feed = (0.53, 0.0, -0.165)
    aim = (0.075 - feed[0], 0.03 - feed[2])  # boresight back toward dish center
    aim_n = math.hypot(*aim)
    aim_u = (aim[0] / aim_n, aim[1] / aim_n)
    psi = math.atan2(aim_u[0], aim_u[1])
    dish.visual(
        Cylinder(radius=0.021, length=0.085),
        origin=Origin(xyz=feed, rpy=(0.0, psi, 0.0)),
        name="lnb_body",
        material=LNB_WHITE,
    )
    dish.visual(
        Cylinder(radius=0.030, length=0.025),
        origin=Origin(
            xyz=(feed[0] + 0.052 * aim_u[0], 0.0, feed[2] + 0.052 * aim_u[1]),
            rpy=(0.0, psi, 0.0),
        ),
        name="feedhorn_flare",
        material=LNB_WHITE,
    )

    # ------------------------------------------------------------- joints
    model.articulation(
        "azimuth_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=az,
        origin=Origin(xyz=(0.0, 0.0, AZ_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent=az,
        child=dish,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        # -Y axis so positive q raises the dish boresight (tilts the face up).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.8, lower=-0.30, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("mast_base")
    az = object_model.get_part("az_clamp")
    dish = object_model.get_part("dish_assembly")
    yaw = object_model.get_articulation("azimuth_yaw")
    elev = object_model.get_articulation("elevation_tilt")

    mast_tube = base.get_visual("mast_tube")
    base_plate = base.get_visual("base_plate")
    sleeve = az.get_visual("clamp_sleeve")
    pin = az.get_visual("pivot_pin")
    plate_l = az.get_visual("clevis_plate_left")
    hub = dish.get_visual("elev_hub")
    boss = dish.get_visual("frame_boss")

    # Intentional embeddings: clamp sleeve hugs the mast; the elevation pin
    # passes through the dish hub/boss it carries.
    ctx.allow_overlap(
        base, az, elem_a=mast_tube, elem_b=sleeve, reason="clamp sleeve hugs the mast"
    )
    ctx.allow_overlap(
        az, dish, elem_a=pin, elem_b=hub, reason="elevation pin passes through the hub"
    )
    ctx.allow_overlap(
        az, dish, elem_a=pin, elem_b=boss, reason="elevation pin passes through the boss"
    )

    # Reflector reads as a ~1 m dish standing on the mast.
    refl = ctx.part_element_world_aabb(dish, elem="reflector_shell")
    assert refl is not None
    assert refl[1][1] - refl[0][1] > 0.90, "reflector aperture should be ~1 m wide"
    assert refl[1][2] - refl[0][2] > 0.85, "reflector should stand ~1 m tall"

    # Feedhorn hangs in FRONT of the reflector on the feed arm.
    lnb = ctx.part_element_world_aabb(dish, elem="lnb_body")
    assert lnb is not None
    assert lnb[0][0] > refl[1][0] + 0.05, "LNB must sit in front of the dish face"
    ctx.expect_overlap(
        dish,
        dish,
        axes="xz",
        elem_a=dish.get_visual("feed_arm"),
        elem_b=dish.get_visual("lnb_clamp"),
        min_overlap=0.002,
        name="feed_arm_reaches_lnb_clamp",
    )

    # Clamp seated on the mast; hub captured between the clevis plates & pin.
    ctx.expect_overlap(base, az, axes="xy", elem_a=mast_tube, elem_b=sleeve, min_overlap=0.04)
    ctx.expect_overlap(az, dish, axes="xyz", elem_a=pin, elem_b=hub, min_overlap=0.004)
    ctx.expect_gap(
        az,
        dish,
        axis="y",
        positive_elem=plate_l,
        negative_elem=hub,
        min_gap=0.002,
        max_gap=0.03,
        name="hub_clears_clevis_plate",
    )

    # Dish stays well above the triangular floor plate at rest.
    ctx.expect_gap(dish, base, axis="z", negative_elem=base_plate, min_gap=0.30)

    lnb_c0 = (
        (lnb[0][0] + lnb[1][0]) / 2,
        (lnb[0][1] + lnb[1][1]) / 2,
        (lnb[0][2] + lnb[1][2]) / 2,
    )

    # Elevation tilt really pitches the dish: the feed rises/falls with +/- q.
    with ctx.pose({elev: 0.30}):
        up = ctx.part_element_world_aabb(dish, elem="lnb_body")
        assert up is not None
        assert (up[0][2] + up[1][2]) / 2 > lnb_c0[2] + 0.08, "positive elevation must raise feed"
        ctx.expect_gap(dish, base, axis="z", negative_elem=base_plate, min_gap=0.25)
    with ctx.pose({elev: -0.30}):
        dn = ctx.part_element_world_aabb(dish, elem="lnb_body")
        assert dn is not None
        assert (dn[0][2] + dn[1][2]) / 2 < lnb_c0[2] - 0.08, "negative elevation must lower feed"
        ctx.expect_gap(dish, base, axis="z", negative_elem=base_plate, min_gap=0.25)

    # Azimuth yaw slews the whole dish head around the mast.
    with ctx.pose({yaw: 1.0}):
        sl = ctx.part_element_world_aabb(dish, elem="lnb_body")
        assert sl is not None
        assert abs((sl[0][1] + sl[1][1]) / 2 - lnb_c0[1]) > 0.30, "yaw must slew the feed sideways"

    return ctx.report()


object_model = build_object_model()
