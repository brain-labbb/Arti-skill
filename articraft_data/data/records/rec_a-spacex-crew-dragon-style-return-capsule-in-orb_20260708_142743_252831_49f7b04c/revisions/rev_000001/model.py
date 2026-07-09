"""Crew Dragon style return capsule in orbit (reference: docked capsule photo).

Layout (meters, Z up, capsule axis on Z, free-floating root):
- capsule: dark ablative heat shield disc under a white truncated-cone
  pressure shell, four recessed Draco thruster pods (2 dark nozzles each),
  two round portholes, a US-flag panel, a small sensor square with a red
  status light, and a forward docking tunnel with a maroon seal band and two
  hinge lugs at the deck edge.
- docking_ring: silver soft-capture ring with 3 inward-leaning capture
  petals and 3 actuator pins riding inside the tunnel wall, on a PRISMATIC
  joint that extends the ring along +Z.
- nose_cone: hollow ogive shell with hinge barrel and straps, on a REVOLUTE
  hinge at the -X deck edge; positive q swings it open away from the port
  (as in the photo, where the cone hangs open beside the docking ring).
"""

from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
BASE_R = 2.05  # heat shield rim radius (Dragon is ~4 m across)
WALL_TILT = 0.345  # sidewall lean from vertical (outward normal pitch, rad)
DECK_Z = 2.85  # forward bulkhead deck height
RING_REST_Z = 3.16  # prismatic joint origin (ring bottom face)
RING_TRAVEL = 0.15
HINGE_XYZ = (-1.10, 0.0, 2.92)  # nose cone hinge point at the -X deck edge
CONE_OFFSET_X = 1.10  # cone axis offset inside the nose_cone part frame

POD_AZIMUTHS = [pi / 4.0 + k * pi / 2.0 for k in range(4)]
POD_Z = 1.10

PORTHOLE_Z = 2.00


def _radial(r: float, ang: float, z: float) -> tuple[float, float, float]:
    return (r * cos(ang), r * sin(ang), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="crew_dragon_return_capsule")

    model.material("shell_white", color=(0.92, 0.92, 0.93))
    model.material("shell_grey", color=(0.78, 0.79, 0.81))
    model.material("pica_char", color=(0.17, 0.15, 0.14))
    model.material("nozzle_dark", color=(0.10, 0.10, 0.11))
    model.material("cavity_dark", color=(0.08, 0.08, 0.09))
    model.material("seal_maroon", color=(0.44, 0.14, 0.11))
    model.material("ring_silver", color=(0.70, 0.71, 0.74))
    model.material("flag_blue", color=(0.14, 0.20, 0.45))
    model.material("flag_red", color=(0.70, 0.14, 0.16))
    model.material("status_red", color=(0.85, 0.10, 0.10))

    # -------------------------------------------------------------- capsule
    capsule = model.part("capsule")

    # Dark ablative heat shield with a slightly convex face and proud rim.
    shield_profile = [
        (0.00, 0.00),
        (1.20, 0.03),
        (1.80, 0.10),
        (2.02, 0.20),
        (BASE_R, 0.32),
        (0.00, 0.32),
    ]
    capsule.visual(
        mesh_from_geometry(LatheGeometry(shield_profile, segments=48), "heat_shield"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="pica_char",
        name="heat_shield",
    )

    # White truncated-cone pressure shell with shoulder and forward deck.
    body_profile = [
        (0.00, 0.30),
        (2.00, 0.30),
        (2.00, 0.45),
        (1.30, 2.40),
        (1.10, 2.70),
        (1.02, DECK_Z),
        (0.00, DECK_Z),
    ]
    capsule.visual(
        mesh_from_geometry(LatheGeometry(body_profile, segments=48), "pressure_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="shell_white",
        name="pressure_shell",
    )

    # Forward docking tunnel (hollow shell, open at the top) + maroon band.
    tunnel = LatheGeometry.from_shell_profiles(
        [(0.58, 0.0), (0.58, 0.32)],
        [(0.48, 0.0), (0.48, 0.32)],
        segments=48,
    )
    capsule.visual(
        mesh_from_geometry(tunnel, "docking_tunnel"),
        origin=Origin(xyz=(0.0, 0.0, 2.83)),
        material="shell_grey",
        name="docking_tunnel",
    )
    band = LatheGeometry.from_shell_profiles(
        [(0.615, 0.0), (0.615, 0.06)],
        [(0.575, 0.0), (0.575, 0.06)],
        segments=48,
    )
    capsule.visual(
        mesh_from_geometry(band, "docking_band"),
        origin=Origin(xyz=(0.0, 0.0, 3.09)),
        material="seal_maroon",
        name="docking_band",
    )
    # Dark forward hatch face visible down the open tunnel.
    capsule.visual(
        Cylinder(radius=0.47, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 2.86)),
        material="cavity_dark",
        name="tunnel_interior",
    )

    # Recessed Draco thruster pods: light panel + 2 dark oval nozzles each.
    for i, ang in enumerate(POD_AZIMUTHS):
        capsule.visual(
            Box((0.06, 0.48, 0.40)),
            origin=Origin(xyz=_radial(1.75, ang, POD_Z), rpy=(0.0, -WALL_TILT, ang)),
            material="shell_grey",
            name=f"draco_pod_panel_{i}",
        )
        for j, side in enumerate((-1, 1)):
            base = _radial(1.79, ang, POD_Z)
            pos = (
                base[0] - side * 0.12 * sin(ang),
                base[1] + side * 0.12 * cos(ang),
                base[2],
            )
            capsule.visual(
                Cylinder(radius=0.075, length=0.04),
                origin=Origin(xyz=pos, rpy=(0.0, pi / 2.0 - WALL_TILT, ang)),
                material="nozzle_dark",
                name=f"draco_nozzle_{i}_{j}",
            )

    # Two round portholes on the upper cone.
    for i, ang in enumerate((0.55, -0.55)):
        capsule.visual(
            Cylinder(radius=0.13, length=0.04),
            origin=Origin(
                xyz=_radial(1.435, ang, PORTHOLE_Z), rpy=(0.0, pi / 2.0 - WALL_TILT, ang)
            ),
            material="shell_grey",
            name=f"porthole_rim_{i}",
        )
        capsule.visual(
            Cylinder(radius=0.09, length=0.04),
            origin=Origin(
                xyz=_radial(1.45, ang, PORTHOLE_Z), rpy=(0.0, pi / 2.0 - WALL_TILT, ang)
            ),
            material="cavity_dark",
            name=f"porthole_glass_{i}",
        )

    # US flag panel (white field, blue canton, red stripes) at azimuth 0.
    flag_c = _radial(1.42, 0.0, 2.05)
    n_vec = (cos(WALL_TILT), 0.0, sin(WALL_TILT))  # outward normal at azimuth 0
    u_vec = (-sin(WALL_TILT), 0.0, cos(WALL_TILT))  # up along the wall

    def flag_pos(t_off: float, u_off: float) -> tuple[float, float, float]:
        return (
            flag_c[0] + u_off * u_vec[0] + 0.004 * n_vec[0],
            flag_c[1] + t_off,
            flag_c[2] + u_off * u_vec[2] + 0.004 * n_vec[2],
        )

    capsule.visual(
        Box((0.024, 0.42, 0.28)),
        origin=Origin(xyz=flag_c, rpy=(0.0, -WALL_TILT, 0.0)),
        material="shell_white",
        name="flag_field",
    )
    capsule.visual(
        Box((0.026, 0.18, 0.13)),
        origin=Origin(xyz=flag_pos(-0.105, 0.0675), rpy=(0.0, -WALL_TILT, 0.0)),
        material="flag_blue",
        name="flag_canton",
    )
    for k in range(3):  # full-width stripes on the lower half
        capsule.visual(
            Box((0.026, 0.42, 0.02)),
            origin=Origin(xyz=flag_pos(0.0, -0.03 - 0.05 * k), rpy=(0.0, -WALL_TILT, 0.0)),
            material="flag_red",
            name=f"flag_stripe_full_{k}",
        )
    for k in range(2):  # short stripes beside the canton
        capsule.visual(
            Box((0.026, 0.20, 0.02)),
            origin=Origin(xyz=flag_pos(0.105, 0.045 + 0.055 * k), rpy=(0.0, -WALL_TILT, 0.0)),
            material="flag_red",
            name=f"flag_stripe_short_{k}",
        )

    # Small dark sensor square and red status light (as in the photo).
    capsule.visual(
        Box((0.024, 0.12, 0.12)),
        origin=Origin(xyz=_radial(1.40, -0.85, 2.10), rpy=(0.0, -WALL_TILT, -0.85)),
        material="cavity_dark",
        name="sensor_panel",
    )
    capsule.visual(
        Cylinder(radius=0.028, length=0.03),
        origin=Origin(xyz=_radial(1.46, -0.70, 1.95), rpy=(0.0, pi / 2.0 - WALL_TILT, -0.70)),
        material="status_red",
        name="status_light",
    )

    # Nose cone hinge lugs at the -X deck edge.
    for i, sy in enumerate((-1, 1)):
        capsule.visual(
            Box((0.12, 0.04, 0.10)),
            origin=Origin(xyz=(-1.06, sy * 0.16, 2.86)),
            material="shell_grey",
            name=f"hinge_lug_{i}",
        )

    # --------------------------------------------------------- docking ring
    ring = model.part("docking_ring")
    ring_band = LatheGeometry.from_shell_profiles(
        [(0.60, 0.0), (0.60, 0.10)],
        [(0.50, 0.0), (0.50, 0.10)],
        segments=48,
    )
    ring.visual(
        mesh_from_geometry(ring_band, "capture_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="ring_silver",
        name="capture_ring",
    )
    petal_tilt = 0.6
    for i in range(3):
        ang = 2.0 * pi * i / 3.0
        r_c = 0.53 - sin(petal_tilt) * 0.066
        ring.visual(
            Box((0.02, 0.14, 0.16)),
            origin=Origin(
                xyz=_radial(r_c, ang, 0.10 + cos(petal_tilt) * 0.066 - 0.006),
                rpy=(0.0, -petal_tilt, ang),
            ),
            material="ring_silver",
            name=f"capture_petal_{i}",
        )
    for i in range(3):
        ang = 2.0 * pi * i / 3.0 + pi / 3.0
        ring.visual(
            Cylinder(radius=0.02, length=0.30),
            origin=Origin(xyz=_radial(0.53, ang, -0.13)),
            material="ring_silver",
            name=f"capture_pin_{i}",
        )

    model.articulation(
        "capsule_to_docking_ring",
        ArticulationType.PRISMATIC,
        parent=capsule,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, RING_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.1, lower=0.0, upper=RING_TRAVEL),
    )

    # ------------------------------------------------------------ nose cone
    nose = model.part("nose_cone")
    cone_outer = [
        (1.04, 0.00),
        (1.03, 0.30),
        (0.95, 0.60),
        (0.78, 0.90),
        (0.52, 1.10),
        (0.20, 1.22),
        (0.06, 1.25),
    ]
    cone_inner = [
        (0.97, 0.00),
        (0.96, 0.30),
        (0.88, 0.58),
        (0.71, 0.86),
        (0.45, 1.04),
        (0.15, 1.14),
        (0.06, 1.16),
    ]
    cone_shell = LatheGeometry.from_shell_profiles(
        cone_outer,
        cone_inner,
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    nose.visual(
        mesh_from_geometry(cone_shell, "cone_shell"),
        origin=Origin(xyz=(CONE_OFFSET_X, 0.0, 0.0)),
        material="shell_white",
        name="cone_shell",
    )
    nose.visual(
        Cylinder(radius=0.04, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="ring_silver",
        name="hinge_barrel",
    )
    for i, sy in enumerate((-1, 1)):
        nose.visual(
            Box((0.16, 0.04, 0.04)),
            origin=Origin(xyz=(0.08, sy * 0.06, 0.0)),
            material="ring_silver",
            name=f"hinge_strap_{i}",
        )

    model.articulation(
        "capsule_to_nose_cone",
        ArticulationType.REVOLUTE,
        parent=capsule,
        child=nose,
        origin=Origin(xyz=HINGE_XYZ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.4, lower=0.0, upper=2.3),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Intentional local embeds: hinge barrel captured between the deck lugs,
    # and soft-capture actuator pins riding inside the docking tunnel wall.
    for i in range(2):
        ctx.allow_overlap(
            "capsule",
            "nose_cone",
            elem_a=f"hinge_lug_{i}",
            elem_b="hinge_barrel",
            reason="nose cone hinge barrel is captured between the deck-edge lugs",
        )
    for i in range(3):
        ctx.allow_overlap(
            "capsule",
            "docking_ring",
            elem_a="docking_tunnel",
            elem_b=f"capture_pin_{i}",
            reason="soft-capture ring actuator pins ride inside the tunnel wall",
        )

    parts = {p.name for p in object_model.parts}
    ctx.check(
        "capsule, nose_cone and docking_ring parts present",
        {"capsule", "nose_cone", "docking_ring"} <= parts,
        f"parts: {sorted(parts)}",
    )

    capsule = object_model.get_part("capsule")
    ring = object_model.get_part("docking_ring")
    n_nozzles = sum(1 for v in capsule.visuals if v.name and v.name.startswith("draco_nozzle_"))
    n_pods = sum(1 for v in capsule.visuals if v.name and v.name.startswith("draco_pod_panel_"))
    ctx.check(
        "4 Draco pods with 2 nozzles each",
        n_pods == 4 and n_nozzles == 8,
        f"pods {n_pods}, nozzles {n_nozzles}",
    )
    n_port = sum(1 for v in capsule.visuals if v.name and v.name.startswith("porthole_rim_"))
    ctx.check("2 round portholes", n_port == 2, f"found {n_port}")
    n_stripes = sum(1 for v in capsule.visuals if v.name and v.name.startswith("flag_stripe_"))
    ctx.check("flag panel carries red stripes", n_stripes == 5, f"found {n_stripes}")
    n_petals = sum(1 for v in ring.visuals if v.name and v.name.startswith("capture_petal_"))
    n_pins = sum(1 for v in ring.visuals if v.name and v.name.startswith("capture_pin_"))
    ctx.check(
        "capture ring has 3 petals and 3 pins",
        n_petals == 3 and n_pins == 3,
        f"petals {n_petals}, pins {n_pins}",
    )

    j_cone = object_model.get_articulation("capsule_to_nose_cone")
    j_ring = object_model.get_articulation("capsule_to_docking_ring")
    ctx.check(
        "nose cone hinge is REVOLUTE about the deck-edge Y axis",
        j_cone.articulation_type == ArticulationType.REVOLUTE and abs(j_cone.axis[1]) > 0.99,
        f"{j_cone.articulation_type}, axis {j_cone.axis}",
    )
    ctx.check(
        "nose cone closes at q=0 and opens past 2 rad",
        j_cone.motion_limits is not None
        and j_cone.motion_limits.lower == 0.0
        and j_cone.motion_limits.upper > 2.0,
        f"limits {j_cone.motion_limits}",
    )
    ctx.check(
        "soft-capture ring extends on a PRISMATIC +Z joint",
        j_ring.articulation_type == ArticulationType.PRISMATIC and abs(j_ring.axis[2]) > 0.99,
        f"{j_ring.articulation_type}, axis {j_ring.axis}",
    )

    # Overall proportions: ~4.1 m across the heat shield, base on z=0.
    aabb = ctx.part_world_aabb("capsule")
    width = aabb[1][0] - aabb[0][0]
    ctx.check("capsule spans ~4.1 m across", 3.9 < width < 4.3, f"width {width:.3f}")
    ctx.check("heat shield sits at the base", aabb[0][2] < 0.01, f"min z {aabb[0][2]:.3f}")

    # Closed nose cone caps the docking assembly.
    cone_aabb0 = ctx.part_world_aabb("nose_cone")
    ctx.check(
        "closed nose cone is centered over the port and tops above 4 m",
        cone_aabb0[0][0] < -1.0 and cone_aabb0[1][0] > 0.9 and cone_aabb0[1][2] > 4.0,
        f"closed aabb {cone_aabb0}",
    )
    ctx.expect_within(
        "docking_ring",
        "nose_cone",
        axes="xy",
        name="closed nose cone covers the soft-capture ring footprint",
    )
    ctx.expect_gap(
        "docking_ring",
        "capsule",
        axis="z",
        positive_elem="capture_ring",
        negative_elem="docking_band",
        min_gap=0.0,
        max_gap=0.03,
        name="retracted capture ring seats just above the maroon seal band",
    )

    # Opening the hinge swings the whole cone off to the -X side of the port.
    with ctx.pose({j_cone: 2.2}):
        cone_open = ctx.part_world_aabb("nose_cone")
        ctx.check(
            "open nose cone clears the docking axis toward -X",
            cone_open[1][0] < -0.9,
            f"open aabb {cone_open}",
        )

    # Extending the prismatic ring raises it while the pins stay inserted.
    ring_aabb0 = ctx.part_world_aabb("docking_ring")
    with ctx.pose({j_ring: RING_TRAVEL}):
        ring_aabb1 = ctx.part_world_aabb("docking_ring")
        ctx.check(
            "soft-capture ring extends upward by its travel",
            ring_aabb1[0][2] > ring_aabb0[0][2] + RING_TRAVEL - 0.01,
            f"rest min z {ring_aabb0[0][2]:.3f}, extended {ring_aabb1[0][2]:.3f}",
        )
        ctx.expect_overlap(
            "docking_ring",
            "capsule",
            axes="z",
            elem_a="capture_pin_0",
            elem_b="docking_tunnel",
            min_overlap=0.05,
            name="extended ring pins retain insertion in the tunnel",
        )

    return ctx.report()


object_model = build_object_model()
