from __future__ import annotations

"""Open-ring prong snap button fastener.

A nickel-plated press-stud snap fastener installed on two fabric swatches.
The female (socket) half features a domed cap disc on the outer face of the
upper fabric and an annular spring socket ring on the inner face.  The male
(stud) half has a five-prong ring eyelet on the outer face of the lower
fabric, with a stud post and mushroom head protruding through the fabric on
the inner face.

Pressing the two halves together along the common axis snaps the stud head
into the socket ring; pulling them apart releases the engagement.

World frame: +Z is the engagement axis (upward toward the cap).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- snap dimensions (metres) -------------------------------------------
CAP_OD = 0.0125
CAP_H = 0.0025

SOCKET_OD = 0.0090
SOCKET_ID = 0.0052
SOCKET_H = 0.0028

STUD_HEAD_OD = 0.0052
STUD_HEAD_H = 0.0018

STUD_POST_OD = 0.0035
STUD_POST_H = 0.0038

PRONG_RING_OD = 0.0090
PRONG_RING_ID = 0.0040
PRONG_RING_H = 0.0015
PRONG_W = 0.0015
PRONG_T = 0.0006
PRONG_L = 0.0040
N_PRONGS = 5

FABRIC_OD = 0.0300
FABRIC_H = 0.0015

DISENGAGE_MAX = 0.018  # metres of pull-apart travel

# ---- Z layout (engaged q=0, female frame at world origin) ---------------
#  socket_ring:   [ 0.0001,  0.0029 ]
#  top_fabric:    [ 0.0029,  0.0044 ]
#  cap_disc:      [ 0.0044,  0.0069 ]
#  stud_head:     [ 0.0003,  0.0021 ]  (inside socket cavity)
#  stud_post:     [-0.0035,  0.0003 ]
#  bottom_fabric: [-0.0050, -0.0035 ]
#  prong_ring:    [-0.0065, -0.0050 ]  (base; prongs extend to -0.001)

F_SOCKET_Z = 0.0015
F_FABRIC_Z = 0.00365
F_CAP_Z = 0.00565

M_HEAD_Z = 0.0012
M_POST_Z = -0.0016
M_FABRIC_Z = -0.00425
M_PRONG_Z = -0.00575


# ---- geometry helpers ----------------------------------------------------
def _annulus(outer_r: float, inner_r: float, height: float) -> cq.Workplane:
    """Annular cylinder centred at z = 0."""
    outer = cq.Workplane("XY").circle(outer_r).extrude(height)
    inner = cq.Workplane("XY").circle(inner_r).extrude(height)
    return outer.cut(inner).translate((0.0, 0.0, -height / 2.0))


def _cap_disc_shape() -> cq.Workplane:
    """Domed cap disc centred at z = 0."""
    r = CAP_OD / 2.0
    h = CAP_H
    disc = cq.Workplane("XY").circle(r).extrude(h)
    disc = disc.edges(">Z").fillet(min(h * 0.35, r * 0.20))
    return disc.translate((0.0, 0.0, -h / 2.0))


def _stud_head_shape() -> cq.Workplane:
    """Mushroom-dome stud head centred at z = 0."""
    r = STUD_HEAD_OD / 2.0
    h = STUD_HEAD_H
    head = cq.Workplane("XY").circle(r).extrude(h)
    head = head.edges(">Z").fillet(min(h * 0.40, r * 0.35))
    return head.translate((0.0, 0.0, -h / 2.0))


def _prong_ring_shape() -> cq.Workplane:
    """Annular base with *N_PRONGS* upward-extending prongs, base centred at z = 0."""
    base = _annulus(PRONG_RING_OD / 2.0, PRONG_RING_ID / 2.0, PRONG_RING_H)
    result = base
    r_mean = (PRONG_RING_OD / 2.0 + PRONG_RING_ID / 2.0) / 2.0
    for i in range(N_PRONGS):
        angle_deg = i * 360.0 / N_PRONGS
        prong = (
            cq.Workplane("XY")
            .box(PRONG_W, PRONG_T, PRONG_L, centered=(True, True, False))
            .translate((r_mean, 0.0, PRONG_RING_H / 2.0))
        )
        if angle_deg > 0.0:
            prong = prong.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        result = result.union(prong)
    return result


# ---- build ---------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_button_fastener")

    nickel = model.material("nickel", rgba=(0.72, 0.73, 0.75, 1.0))
    fabric_top = model.material("denim_blue", rgba=(0.25, 0.32, 0.50, 1.0))
    fabric_bot = model.material("cotton_red", rgba=(0.65, 0.18, 0.18, 1.0))

    # ---- female (socket) half ----
    socket_part = model.part("snap_socket")

    socket_part.visual(
        mesh_from_cadquery(_cap_disc_shape(), "cap_disc"),
        origin=Origin(xyz=(0.0, 0.0, F_CAP_Z)),
        material=nickel,
        name="cap_disc",
    )
    socket_part.visual(
        Cylinder(radius=FABRIC_OD / 2.0, length=FABRIC_H),
        origin=Origin(xyz=(0.0, 0.0, F_FABRIC_Z)),
        material=fabric_top,
        name="top_fabric",
    )
    socket_part.visual(
        mesh_from_cadquery(
            _annulus(SOCKET_OD / 2.0, SOCKET_ID / 2.0, SOCKET_H), "socket_ring"
        ),
        origin=Origin(xyz=(0.0, 0.0, F_SOCKET_Z)),
        material=nickel,
        name="socket_ring",
    )

    # ---- male (stud) half ----
    stud_part = model.part("snap_stud")

    stud_part.visual(
        mesh_from_cadquery(_stud_head_shape(), "stud_head"),
        origin=Origin(xyz=(0.0, 0.0, M_HEAD_Z)),
        material=nickel,
        name="stud_head",
    )
    stud_part.visual(
        Cylinder(radius=STUD_POST_OD / 2.0, length=STUD_POST_H),
        origin=Origin(xyz=(0.0, 0.0, M_POST_Z)),
        material=nickel,
        name="stud_post",
    )
    stud_part.visual(
        Cylinder(radius=FABRIC_OD / 2.0, length=FABRIC_H),
        origin=Origin(xyz=(0.0, 0.0, M_FABRIC_Z)),
        material=fabric_bot,
        name="bottom_fabric",
    )
    stud_part.visual(
        mesh_from_cadquery(_prong_ring_shape(), "prong_ring"),
        origin=Origin(xyz=(0.0, 0.0, M_PRONG_Z)),
        material=nickel,
        name="prong_ring",
    )

    # ---- articulation ----
    model.articulation(
        "snap_engage",
        ArticulationType.PRISMATIC,
        parent=socket_part,
        child=stud_part,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),  # positive q pulls male away (downward)
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.5,
            lower=0.0,
            upper=DISENGAGE_MAX,
        ),
    )

    return model


# ---- tests ---------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    socket_part = object_model.get_part("snap_socket")
    stud_part = object_model.get_part("snap_stud")
    engage = object_model.get_articulation("snap_engage")

    # -- Engaged (q=0): stud head inside socket ring cavity --
    ctx.expect_within(
        stud_part,
        socket_part,
        axes="xy",
        inner_elem="stud_head",
        outer_elem="socket_ring",
        margin=0.001,
        name="stud_head is radially contained within the socket_ring cavity",
    )

    # The two halves are coaxial on the engagement axis.
    ctx.expect_overlap(
        socket_part,
        stud_part,
        axes="xy",
        min_overlap=0.004,
        name="snap halves share the engagement axis",
    )

    # -- Disengaged pose: prismatic joint separates the halves --
    engaged_head_aabb = ctx.part_element_world_aabb(stud_part, elem="stud_head")

    with ctx.pose({engage: DISENGAGE_MAX}):
        separated_head_aabb = ctx.part_element_world_aabb(stud_part, elem="stud_head")

        ctx.check(
            "snap_engage prismatic joint pulls the stud out of the socket",
            engaged_head_aabb is not None
            and separated_head_aabb is not None
            and separated_head_aabb[1][2]
            < engaged_head_aabb[0][2] - 0.005,
            details=(
                f"engaged stud_head top="
                f"{engaged_head_aabb[1][2] if engaged_head_aabb else None}, "
                f"separated stud_head top="
                f"{separated_head_aabb[1][2] if separated_head_aabb else None}"
            ),
        )

        ctx.expect_gap(
            socket_part,
            stud_part,
            axis="z",
            positive_elem="socket_ring",
            negative_elem="stud_head",
            min_gap=0.005,
            name="fully disengaged halves have clear vertical separation",
        )

    return ctx.report()


object_model = build_object_model()
