from __future__ import annotations

"""Hand-held snap button fastener pliers.

A chrome-steel snap-setting plier tool with red vinyl handle grips. The fixed
frame arm rises from the pivot into a curved head that carries a white plastic
die holding the nickel snap socket/cap half. The moving lever arm pivots on
the rivet: squeezing the handles raises its white anvil post — which carries
the matching snap stud half — and presses the two snap button halves together.

World frame: the tool stands head-up. The pivot rivet axis is +X, the jaws
overhang toward +Y, and the handles extend downward (-Z) in a V.
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

# ---------------------------------------------------------------- dimensions
PLATE_T = 0.0045          # steel arm plate thickness (X)
FRAME_PLATE_X = -0.005    # frame plate spans [-0.005, -0.0005]
LEVER_PLATE_X = 0.0005    # lever plate spans [+0.0005, +0.005]
GRIP_T = 0.010            # vinyl grip sleeve thickness (X)

DIE_Y = 0.055             # die / anvil press axis, forward of the pivot
DIE_BOTTOM_Z = 0.0385     # bottom face of the white upper die
ANVIL_TOP_Z = 0.020       # top face of the white anvil post

SQUEEZE_MAX = 0.15        # rad; closes the 8.5 mm snap gap to ~0.5 mm


def _strip(p0: tuple[float, float], p1: tuple[float, float], width: float):
    """Rectangle corners for a straight bar from p0 to p1 in the YZ profile."""
    du, dv = p1[0] - p0[0], p1[1] - p0[1]
    norm = math.hypot(du, dv)
    nu, nv = -dv / norm * width / 2.0, du / norm * width / 2.0
    return [
        (p0[0] + nu, p0[1] + nv),
        (p1[0] + nu, p1[1] + nv),
        (p1[0] - nu, p1[1] - nv),
        (p0[0] - nu, p0[1] - nv),
    ]


def _profile_bar(p0, p1, width, thickness):
    """Extruded straight bar in the YZ plane; extrusion is along +X."""
    return cq.Workplane("YZ").polyline(_strip(p0, p1, width)).close().extrude(thickness)


def _profile_disc(center, radius, thickness):
    return cq.Workplane("YZ").pushPoints([center]).circle(radius).extrude(thickness)


def _frame_body():
    """Fixed arm: handle bar, pivot boss, riser, top overhang and head cap."""
    handle = _profile_bar((0.020, -0.130), (0.003, -0.008), 0.013, PLATE_T)
    boss = _profile_disc((0.0, 0.0), 0.010, PLATE_T)
    riser = _profile_bar((0.000, 0.005), (0.012, 0.050), 0.014, PLATE_T)
    overhang = _profile_bar((0.006, 0.052), (DIE_Y, 0.052), 0.012, PLATE_T)
    plate = handle.union(boss).union(riser).union(overhang).translate((FRAME_PLATE_X, 0, 0))
    # Head cap centered on the die axis so the die reads as mounted under it.
    head_cap = (
        cq.Workplane("XY")
        .box(0.012, 0.024, 0.010, centered=(True, True, True))
        .translate((0.0, DIE_Y, 0.055))
    )
    return plate.union(head_cap)


def _lever_body():
    """Moving arm: handle bar, pivot boss and anvil arm reaching the die axis."""
    handle = _profile_bar((-0.050, -0.125), (-0.002, -0.006), 0.013, PLATE_T)
    boss = _profile_disc((0.0, 0.0), 0.010, PLATE_T)
    anvil_arm = _profile_bar((0.004, 0.004), (DIE_Y, 0.012), 0.012, PLATE_T)
    return handle.union(boss).union(anvil_arm).translate((LEVER_PLATE_X, 0, 0))


def _grip(p0, p1, x_offset):
    """Red vinyl sleeve: wide bar with a rounded end disc at the handle tip."""
    bar = _profile_bar(p0, p1, 0.018, GRIP_T)
    tip = _profile_disc(p0, 0.009, GRIP_T)
    return bar.union(tip).translate((x_offset, 0, 0))


def _pivot_rivet():
    shank = cq.Workplane("YZ").circle(0.0055).extrude(0.015).translate((-0.0075, 0, 0))
    head_a = cq.Workplane("YZ").circle(0.0075).extrude(0.0018).translate((-0.0093, 0, 0))
    head_b = cq.Workplane("YZ").circle(0.0075).extrude(0.0018).translate((0.0075, 0, 0))
    return shank.union(head_a).union(head_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_fastener_pliers")

    steel = model.material("chrome_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    vinyl = model.material("red_vinyl", rgba=(0.78, 0.10, 0.10, 1.0))
    plastic = model.material("white_plastic", rgba=(0.94, 0.94, 0.92, 1.0))
    nickel = model.material("nickel_snap", rgba=(0.72, 0.73, 0.75, 1.0))

    # ------------------------------------------------------------ frame arm
    frame = model.part("frame_arm")
    frame.visual(mesh_from_cadquery(_frame_body(), "frame_body"),
                 material=steel, name="frame_body")
    frame.visual(mesh_from_cadquery(_pivot_rivet(), "pivot_rivet"),
                 material=steel, name="pivot_rivet")
    # White plastic upper die hanging under the head, press face down.
    frame.visual(
        Cylinder(radius=0.008, length=0.052 - DIE_BOTTOM_Z),
        origin=Origin(xyz=(0.0, DIE_Y, (0.052 + DIE_BOTTOM_Z) / 2.0)),
        material=plastic,
        name="upper_die",
    )
    frame.visual(
        mesh_from_cadquery(_grip((0.020, -0.130), (0.010, -0.060), -0.00775), "frame_grip"),
        material=vinyl,
        name="frame_grip",
    )

    # ------------------------------------------------------------ lever arm
    lever = model.part("lever_arm")
    lever.visual(mesh_from_cadquery(_lever_body(), "lever_body"),
                 material=steel, name="lever_body")
    # White plastic anvil post rising from the anvil arm, press face up.
    lever.visual(
        Cylinder(radius=0.006, length=ANVIL_TOP_Z - 0.008),
        origin=Origin(xyz=(0.0, DIE_Y, (ANVIL_TOP_Z + 0.008) / 2.0)),
        material=plastic,
        name="anvil_post",
    )
    lever.visual(
        mesh_from_cadquery(_grip((-0.050, -0.125), (-0.030, -0.062), -0.00225), "lever_grip"),
        material=vinyl,
        name="lever_grip",
    )

    # -------------------------------------------- snap socket half (in die)
    socket = model.part("snap_socket")
    socket.visual(
        Cylinder(radius=0.0062, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, -0.0007)),  # cap embeds 0.3 mm into the die
        material=nickel,
        name="cap_disc",
    )
    socket.visual(
        Cylinder(radius=0.0048, length=0.0028),
        origin=Origin(xyz=(0.0, 0.0, -0.0031)),
        material=nickel,
        name="socket_ring",
    )

    # -------------------------------------------- snap stud half (on anvil)
    stud = model.part("snap_stud")
    stud.visual(
        Cylinder(radius=0.0062, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.00045)),  # ring embeds 0.3 mm into anvil
        material=nickel,
        name="prong_ring",
    )
    stud.visual(
        Cylinder(radius=0.0026, length=0.0031),
        origin=Origin(xyz=(0.0, 0.0, 0.00275)),
        material=nickel,
        name="stud_post",
    )
    stud.visual(
        Cylinder(radius=0.0033, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0049)),
        material=nickel,
        name="stud_head",
    )

    # -------------------------------------------------------- articulations
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        # Positive q (right-hand rule about +X) swings the lever handle toward
        # the frame handle (+Y) and raises the anvil arm toward the die.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=SQUEEZE_MAX),
    )
    model.articulation(
        "die_to_socket",
        ArticulationType.FIXED,
        parent=frame,
        child=socket,
        origin=Origin(xyz=(0.0, DIE_Y, DIE_BOTTOM_Z)),
    )
    model.articulation(
        "anvil_to_stud",
        ArticulationType.FIXED,
        parent=lever,
        child=stud,
        origin=Origin(xyz=(0.0, DIE_Y, ANVIL_TOP_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame_arm")
    lever = object_model.get_part("lever_arm")
    socket = object_model.get_part("snap_socket")
    stud = object_model.get_part("snap_stud")
    pivot = object_model.get_articulation("handle_pivot")

    # Intentional local embeddings.
    ctx.allow_overlap(
        frame,
        lever,
        elem_a="pivot_rivet",
        elem_b="lever_body",
        reason="The pivot rivet is a captured pin passing through the lever plate boss.",
    )
    ctx.allow_overlap(
        socket,
        frame,
        elem_a="cap_disc",
        elem_b="upper_die",
        reason="The snap cap is seated 0.3 mm into the upper die recess.",
    )
    ctx.allow_overlap(
        stud,
        lever,
        elem_a="prong_ring",
        elem_b="anvil_post",
        reason="The prong ring is seated 0.3 mm into the anvil post recess.",
    )

    # Snap halves are seated on their dies and coaxial at rest.
    ctx.expect_gap(
        frame,
        socket,
        axis="z",
        positive_elem="upper_die",
        negative_elem="cap_disc",
        max_penetration=0.0006,
        max_gap=0.0001,
        name="snap cap seats against the upper die",
    )
    ctx.expect_gap(
        stud,
        lever,
        axis="z",
        positive_elem="prong_ring",
        negative_elem="anvil_post",
        max_penetration=0.0006,
        name="snap stud ring seats on the anvil post",
    )
    ctx.expect_overlap(
        socket,
        stud,
        axes="xy",
        min_overlap=0.005,
        name="snap halves are coaxial under the press axis at rest",
    )

    # Jaw is open at rest: clear vertical gap between the two snap halves.
    ctx.expect_gap(
        socket,
        stud,
        axis="z",
        min_gap=0.0075,
        name="snap halves are separated at rest",
    )

    # Handles form a V at rest: lever grip sits on -Y, frame grip on +Y.
    ctx.expect_origin_gap(
        frame,
        lever,
        axis="y",
        min_gap=-0.001,
        name="grip separation baseline",
    )
    frame_grip_box = ctx.part_element_world_aabb(frame, elem="frame_grip")
    lever_grip_rest = ctx.part_element_world_aabb(lever, elem="lever_grip")
    ctx.check(
        "handle grips diverge into a V at rest",
        frame_grip_box is not None
        and lever_grip_rest is not None
        and frame_grip_box[0][1] > lever_grip_rest[1][1],
        details=f"frame_grip={frame_grip_box}, lever_grip={lever_grip_rest}",
    )

    # Squeezing the handles closes the snap halves onto each other.
    with ctx.pose({pivot: SQUEEZE_MAX}):
        ctx.expect_gap(
            socket,
            stud,
            axis="z",
            min_gap=0.0,
            max_gap=0.0020,
            name="full squeeze presses the snap halves nearly together",
        )
        ctx.expect_overlap(
            socket,
            stud,
            axes="y",
            min_overlap=0.002,
            name="stud stays under the socket during the press stroke",
        )
        lever_grip_closed = ctx.part_element_world_aabb(lever, elem="lever_grip")
        ctx.check(
            "squeezing swings the lever grip toward the frame grip",
            lever_grip_rest is not None
            and lever_grip_closed is not None
            and lever_grip_closed[1][1] > lever_grip_rest[1][1] + 0.008,
            details=f"rest={lever_grip_rest}, closed={lever_grip_closed}",
        )

    return ctx.report()


object_model = build_object_model()
