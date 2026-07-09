"""Glossy white astronaut helmet with flip-up amber bubble visor.

Modeled after the reference image: a spherical hard shell in glossy white,
a large dark amber tinted bubble visor filling the front opening, a wide
brushed-silver neck ring collar with rivets, side visor pivot hardware, a
small round pressure gauge on the side of the shell, and a purge valve
near the neck ring. The visor flips up on a revolute joint through the
side pivots.
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

# ---------------------------------------------------------------- dimensions
SHELL_R_OUT = 0.160  # outer radius of the spherical shell
SHELL_R_IN = 0.148  # inner radius (hollow shell)
NECK_CUT_Z = -0.108  # shell-local plane where the neck opening is cut
SHELL_CZ = 0.144  # absolute height of the shell center (ring bottom at z=0)

VISOR_R_OUT = 0.170  # visor bubble outer radius (concentric with shell)
VISOR_R_IN = 0.164  # visor bubble inner radius

RING_R_OUT = 0.128
RING_R_IN = 0.098
RING_H = 0.036
FLANGE_R_OUT = 0.134
FLANGE_H = 0.008

N_RING_RIVETS = 10


def _dir(azimuth_deg: float, elevation_deg: float) -> tuple[float, float, float]:
    """Unit vector on the shell from azimuth (from +X toward +Y) and elevation."""
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    return (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))


def _aim_rpy(n: tuple[float, float, float]) -> tuple[float, float, float]:
    """rpy that aligns a cylinder's local +Z with direction n."""
    yaw = math.atan2(n[1], n[0])
    pitch = math.acos(max(-1.0, min(1.0, n[2])))
    return (0.0, pitch, yaw)


def _shell_shape() -> cq.Workplane:
    """Hollow sphere with a rounded-rectangle front visor opening and neck cut."""
    shell = (
        cq.Workplane("XY")
        .sphere(SHELL_R_OUT)
        .cut(cq.Workplane("XY").sphere(SHELL_R_IN))
    )
    # Front visor opening: rounded rectangle punched forward along +X.
    opening = (
        cq.Workplane("YZ", origin=(0.05, 0.0, 0.005))
        .rect(0.21, 0.16)
        .extrude(0.18)
        .edges("|X")
        .fillet(0.05)
    )
    shell = shell.cut(opening)
    # Neck opening: remove everything below the neck-ring seat plane.
    neck_cutter = cq.Workplane("XY", origin=(0.0, 0.0, NECK_CUT_Z - 0.15)).box(
        0.5, 0.5, 0.3
    )
    return shell.cut(neck_cutter)


def _visor_shape() -> cq.Workplane:
    """Bubble visor: a spherical shell segment concentric with the helmet shell.

    Side pivot arms (thin equatorial bands of the same spherical shell) sweep
    back around the helmet toward the +/-Y pivot axis so the bubble is carried
    by the pivot hubs instead of floating in front of the opening.
    """
    bubble = (
        cq.Workplane("XY")
        .sphere(VISOR_R_OUT)
        .cut(cq.Workplane("XY").sphere(VISOR_R_IN))
    )
    window = (
        cq.Workplane("YZ", origin=(0.0, 0.0, 0.005))
        .rect(0.24, 0.19)
        .extrude(0.20)
        .edges("|X")
        .fillet(0.055)
    )
    # Equatorial pivot-arm band reaching the +/-Y pivot poles.
    arm_band = cq.Workplane("XY", origin=(0.09, 0.0, 0.0)).box(0.22, 0.36, 0.048)
    return bubble.intersect(window.union(arm_band))


def _neck_ring_shape() -> cq.Workplane:
    """Wide brushed-metal collar ring with a slightly wider bottom flange."""
    collar = (
        cq.Workplane("XY", origin=(0.0, 0.0, -RING_H / 2.0))
        .circle(RING_R_OUT)
        .circle(RING_R_IN)
        .extrude(RING_H)
    )
    flange = (
        cq.Workplane("XY", origin=(0.0, 0.0, -RING_H / 2.0))
        .circle(FLANGE_R_OUT)
        .circle(RING_R_IN)
        .extrude(FLANGE_H)
    )
    return collar.union(flange)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="astronaut_helmet")

    shell_white = model.material("shell_white", rgba=(0.93, 0.93, 0.95, 1.0))
    visor_amber = model.material("visor_amber", rgba=(0.24, 0.14, 0.07, 0.96))
    metal_silver = model.material("metal_silver", rgba=(0.72, 0.73, 0.76, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.38, 0.39, 0.42, 1.0))
    dial_white = model.material("dial_white", rgba=(0.96, 0.96, 0.93, 1.0))
    needle_dark = model.material("needle_dark", rgba=(0.14, 0.14, 0.16, 1.0))

    # ---------------------------------------------------------------- shell
    shell = model.part("helmet_shell")
    shell.visual(
        mesh_from_cadquery(_shell_shape(), "helmet_shell"),
        origin=Origin(xyz=(0.0, 0.0, SHELL_CZ)),
        material=shell_white,
        name="shell_dome",
    )

    # Side visor pivot hardware: silver boss discs on the shell, both sides.
    for i, side in enumerate((-1.0, 1.0)):
        shell.visual(
            Cylinder(radius=0.022, length=0.014),
            origin=Origin(
                xyz=(0.0, side * 0.157, SHELL_CZ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_silver,
            name=f"visor_pivot_boss_{i}",
        )

    # Small round pressure gauge on the side of the shell (as in the image).
    g = _dir(-60.0, 26.0)
    g_rpy = _aim_rpy(g)
    shell.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(
            xyz=(0.157 * g[0], 0.157 * g[1], SHELL_CZ + 0.157 * g[2]),
            rpy=g_rpy,
        ),
        material=metal_silver,
        name="gauge_bezel",
    )
    shell.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(
            xyz=(0.1655 * g[0], 0.1655 * g[1], SHELL_CZ + 0.1655 * g[2]),
            rpy=g_rpy,
        ),
        material=dial_white,
        name="gauge_dial",
    )
    shell.visual(
        Box((0.013, 0.0035, 0.0030)),
        origin=Origin(
            xyz=(0.1685 * g[0], 0.1685 * g[1], SHELL_CZ + 0.1685 * g[2]),
            rpy=g_rpy,
        ),
        material=needle_dark,
        name="gauge_needle",
    )

    # Purge valve with a small knob near the neck ring (lower side of shell).
    v = _dir(-75.0, -25.0)
    v_rpy = _aim_rpy(v)
    shell.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(
            xyz=(0.168 * v[0], 0.168 * v[1], SHELL_CZ + 0.168 * v[2]),
            rpy=v_rpy,
        ),
        material=metal_silver,
        name="purge_valve_stem",
    )
    shell.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(
            xyz=(0.187 * v[0], 0.187 * v[1], SHELL_CZ + 0.187 * v[2]),
            rpy=v_rpy,
        ),
        material=dial_white,
        name="purge_valve_knob",
    )

    # ------------------------------------------------------------ neck ring
    neck_ring = model.part("neck_ring")
    neck_ring.visual(
        mesh_from_cadquery(_neck_ring_shape(), "neck_ring"),
        material=metal_silver,
        name="collar_ring",
    )
    # Rivets around the collar band.
    for i in range(N_RING_RIVETS):
        a = 2.0 * math.pi * i / N_RING_RIVETS
        neck_ring.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(
                xyz=(0.127 * math.cos(a), 0.127 * math.sin(a), 0.004),
                rpy=(0.0, math.pi / 2.0, a),
            ),
            material=metal_dark,
            name=f"ring_rivet_{i}",
        )

    model.articulation(
        "shell_to_neck_ring",
        ArticulationType.FIXED,
        parent=shell,
        child=neck_ring,
        # Ring center sits so the collar top plane meets the shell neck cut.
        origin=Origin(xyz=(0.0, 0.0, SHELL_CZ + NECK_CUT_Z - RING_H / 2.0)),
    )

    # ----------------------------------------------------------------- visor
    visor = model.part("visor")
    visor.visual(
        mesh_from_cadquery(_visor_shape(), "visor"),
        material=visor_amber,
        name="visor_bubble",
    )
    # Pivot hubs on the visor arms: they seat into the shell's pivot bosses
    # (coaxial with the hinge axis, so they stay seated at every visor angle).
    for i, side in enumerate((-1.0, 1.0)):
        visor.visual(
            Cylinder(radius=0.015, length=0.014),
            origin=Origin(
                xyz=(0.0, side * 0.168, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_silver,
            name=f"visor_pivot_hub_{i}",
        )
        visor.visual(
            Cylinder(radius=0.011, length=0.007),
            origin=Origin(
                xyz=(0.0, side * 0.1775, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_dark,
            name=f"visor_pivot_cap_{i}",
        )

    model.articulation(
        "shell_to_visor",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=visor,
        # Pivot axis runs through the side pivot bosses at shell-center height,
        # so the concentric bubble sweeps clear of the dome while flipping up.
        origin=Origin(xyz=(0.0, 0.0, SHELL_CZ)),
        # Visor faces +X; -Y axis makes positive q flip the visor upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("helmet_shell")
    visor = object_model.get_part("visor")
    neck_ring = object_model.get_part("neck_ring")
    hinge = object_model.get_articulation("shell_to_visor")

    # The visor hinge is a real flip-up mechanism with a usable range.
    limits = hinge.motion_limits
    assert limits is not None and limits.upper is not None and limits.upper >= 1.0

    # Prompt-critical detail features exist on the shell.
    for feature in (
        "gauge_bezel",
        "gauge_dial",
        "gauge_needle",
        "purge_valve_stem",
        "purge_valve_knob",
        "visor_pivot_boss_0",
        "visor_pivot_boss_1",
    ):
        assert shell.get_visual(feature) is not None

    # The visor pivot hubs intentionally seat inside the shell pivot bosses:
    # a captured coaxial pivot shaft, hidden inside the boss at every angle.
    bubble = visor.get_visual("visor_bubble")
    for i in range(2):
        hub = visor.get_visual(f"visor_pivot_hub_{i}")
        boss = shell.get_visual(f"visor_pivot_boss_{i}")
        ctx.allow_overlap(
            visor,
            shell,
            elem_a=hub,
            elem_b=boss,
            reason="visor pivot hub is a captured coaxial pivot shaft seated in the shell pivot boss",
        )
        ctx.allow_overlap(
            visor,
            shell,
            elem_a=bubble,
            elem_b=boss,
            reason="visor pivot-arm end socket wraps around the coaxial shell pivot boss",
        )
        ctx.expect_contact(visor, shell, elem_a=hub, elem_b=boss, contact_tol=1e-4)

    with ctx.pose({hinge: 0.0}):
        # Closed: the bubble covers the front opening (large lateral/vertical
        # projected overlap with the shell) and clears the neck ring below.
        ctx.expect_overlap(visor, shell, axes="yz", min_overlap=0.10)
        ctx.expect_gap(visor, neck_ring, axis="z", min_gap=0.005)
        # Neck ring seats directly under the shell neck cut.
        ctx.expect_gap(shell, neck_ring, axis="z", max_gap=0.002, max_penetration=0.001)

    with ctx.pose({hinge: 1.3}):
        # Flipped up: the visor swings well above the neck ring.
        ctx.expect_gap(visor, neck_ring, axis="z", min_gap=0.05)

    return ctx.report()


object_model = build_object_model()
