from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Snap fastener (press stud): standalone ball-and-socket snap fastener
# installed on two fabric swatch layers.
#
# Frame: +Z is up, coaxial along Z.
# Root = top_half (female): domed snap_cap + brass snap_socket on top fabric.
# Child = bottom_half (male): steel die_post ball stud + cap_die base flange
#         on bottom fabric.
# PRISMATIC engage_snap joint: +Z axis, positive q seats the ball into the
# socket (snap action).  q=0 is disengaged (separated).
# ---------------------------------------------------------------------------

SNAP_TRAVEL = 0.012  # m, full engagement stroke


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snap_fastener_press_stud")

    # --- Materials ---
    gunmetal = model.material("gunmetal", rgba=(0.33, 0.35, 0.38, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.64, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.79, 0.81, 1.0))
    fabric = model.material("navy_fabric", rgba=(0.15, 0.18, 0.32, 1.0))

    # ======================================================= top_half (root)
    # Female half: domed cap + socket ring, riveted through top fabric.
    top_half = model.part("top_half")

    # Top fabric swatch disc (thin circular cloth sample).
    top_half.visual(
        Cylinder(radius=0.017, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=fabric,
        name="fabric_top",
    )

    # snap_cap: domed cap with a slight base rim, sitting atop the fabric.
    # LatheGeometry solid-of-revolution profile (radius, z).
    dome = LatheGeometry(
        [
            (0.0072, 0.0),       # outer rim edge
            (0.0072, 0.0004),    # rim top
            (0.0068, 0.0006),    # rim-to-dome transition
            (0.0058, 0.0013),
            (0.0042, 0.0023),
            (0.0024, 0.0032),
            (0.0008, 0.0038),
            (0.0, 0.0040),      # dome peak
        ],
        segments=48,
    )
    top_half.visual(
        mesh_from_geometry(dome, "snap_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=gunmetal,
        name="snap_cap",
    )

    # snap_socket: S-spring socket ring below the fabric.
    # Hollow cylindrical ring (annular shell) that the ball stud snaps into.
    socket = LatheGeometry.from_shell_profiles(
        [(0.0050, 0.0), (0.0050, 0.003)],   # outer wall
        [(0.0030, 0.0), (0.0030, 0.003)],   # inner bore
        segments=48,
    )
    top_half.visual(
        mesh_from_geometry(socket, "snap_socket"),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=brass,
        name="snap_socket",
    )

    # ================================================= bottom_half (child)
    # Male half: ball stud + base flange, riveted through bottom fabric.
    bottom_half = model.part("bottom_half")

    # Layout convention (child-local frame, at q=0 coincides with parent):
    #
    # Engaged world Z positions  (q = SNAP_TRAVEL = 0.012):
    #   die_post ball center    = -0.0015  (inside socket bore)
    #   die_post_stem           = -0.0065 to -0.0037
    #   fabric_bottom           = -0.0065 to -0.0055
    #   cap_die flange          = -0.0080 to -0.0065
    #
    # Local = World - SNAP_TRAVEL:
    #   ball center             = -0.0135
    #   stem                    = -0.0185 to -0.0157, center -0.0171
    #   fabric_bottom center    = -0.0180
    #   cap_die origin          = -0.0200

    # die_post: ball stud sphere (snaps into the socket ring).
    bottom_half.visual(
        Sphere(radius=0.0022),
        origin=Origin(xyz=(0.0, 0.0, -0.0135)),
        material=steel,
        name="die_post",
    )

    # die_post_stem: cylindrical post connecting ball to base flange.
    bottom_half.visual(
        Cylinder(radius=0.0012, length=0.0028),
        origin=Origin(xyz=(0.0, 0.0, -0.0171)),
        material=steel,
        name="die_post_stem",
    )

    # Bottom fabric swatch disc.
    bottom_half.visual(
        Cylinder(radius=0.017, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, -0.0180)),
        material=fabric,
        name="fabric_bottom",
    )

    # cap_die: base rivet flange (visible on the outside of bottom fabric).
    # Solid of revolution with a center bore and tapered outer edge.
    flange = LatheGeometry(
        [
            (0.0050, 0.0),
            (0.0050, 0.0008),
            (0.0042, 0.0015),
            (0.0012, 0.0015),
            (0.0012, 0.0),
        ],
        segments=48,
    )
    bottom_half.visual(
        mesh_from_geometry(flange, "cap_die"),
        origin=Origin(xyz=(0.0, 0.0, -0.0200)),
        material=steel,
        name="cap_die",
    )

    # ==================================================== engage_snap joint
    # PRISMATIC: child translates along +Z to seat ball into socket.
    model.articulation(
        "engage_snap",
        ArticulationType.PRISMATIC,
        parent=top_half,
        child=bottom_half,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.1,
            lower=0.0,
            upper=SNAP_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    top_half = object_model.get_part("top_half")
    bottom_half = object_model.get_part("bottom_half")
    engage = object_model.get_articulation("engage_snap")

    # The male half is intentionally separated from the female half at q=0
    # (disengaged state). The PRISMATIC engage_snap joint retains the
    # mechanical connection along the Z axis.
    ctx.allow_isolated_part(
        bottom_half,
        reason="The male snap half separates from the female half when disengaged; the PRISMATIC engage_snap joint retains the coaxial mechanical connection.",
    )

    # --- Disengaged pose (q=0): ball stud well below socket ring ---
    ctx.expect_gap(
        top_half,
        bottom_half,
        axis="z",
        positive_elem="snap_socket",
        negative_elem="die_post",
        min_gap=0.005,
        name="disengaged ball stud is separated below the socket ring",
    )

    # --- Engaged pose (q=SNAP_TRAVEL): ball seated inside socket ---
    with ctx.pose({engage: SNAP_TRAVEL}):
        ctx.expect_overlap(
            bottom_half,
            top_half,
            axes="z",
            elem_a="die_post",
            elem_b="snap_socket",
            min_overlap=0.002,
            name="engaged ball stud overlaps the socket ring along Z",
        )
        ctx.expect_within(
            bottom_half,
            top_half,
            axes="xy",
            inner_elem="die_post",
            outer_elem="snap_socket",
            margin=0.001,
            name="engaged ball stud is centered within the socket ring bore",
        )

    # --- Engagement direction proof ---
    rest_pos = ctx.part_world_position(bottom_half)
    with ctx.pose({engage: SNAP_TRAVEL}):
        engaged_pos = ctx.part_world_position(bottom_half)

    ctx.check(
        "engage_snap seats the ball stud upward into the socket",
        rest_pos is not None
        and engaged_pos is not None
        and engaged_pos[2] > rest_pos[2] + 0.010,
        details=f"rest={rest_pos}, engaged={engaged_pos}",
    )

    return ctx.report()


object_model = build_object_model()
