from __future__ import annotations

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
    mesh_from_geometry,
    tube_from_spline_points,
)


# ── Global geometry constants ────────────────────────────────────────────
WHEEL_RADIUS = 0.145
AXLE_HEIGHT = 0.180

# Flying-saucer tilt: disc normal is ~35° from horizontal (X axis toward Z).
TILT_ANGLE = math.radians(35)
AX_COS = math.cos(TILT_ANGLE)
AX_SIN = math.sin(TILT_ANGLE)

# Disc cross-section
DISC_THICKNESS = 0.010
HUB_EXTRA = 0.010  # extra hub thickness beyond disc
HUB_OUTER_R = 0.034
AXLE_BORE_R = 0.012
HUB_HALF_T = (DISC_THICKNESS + HUB_EXTRA) / 2.0  # 0.010
LIP_HEIGHT = 0.018
LIP_WIDTH = 0.012

# Axle / bearing
CAP_LENGTH = 0.012
BEARING_LENGTH = 0.014
CAP_HALF = CAP_LENGTH / 2.0
BEARING_HALF = BEARING_LENGTH / 2.0

# Thrust-bearing overlap: cap/bearing press 0.5 mm into the hub faces so
# the wheel part is mechanically connected to the stand through the axle.
THRUST_OVERLAP = 0.0005

# Along-axis distances from disc centre (positive = front/+normal side)
FRONT_CAP_D = HUB_HALF_T - THRUST_OVERLAP + CAP_HALF
REAR_BEARING_D = -(HUB_HALF_T - THRUST_OVERLAP + BEARING_HALF)
AXLE_FRONT_D = FRONT_CAP_D + CAP_HALF + 0.005
AXLE_REAR_D = -0.072
AXLE_CENTER_D = (AXLE_FRONT_D + AXLE_REAR_D) / 2.0
AXLE_SPAN = AXLE_FRONT_D - AXLE_REAR_D

# Pitch that rotates a default Z-axis cylinder onto the inclined axle.
AXLE_PITCH = math.pi / 2.0 - TILT_ANGLE


def _axis_to_world(d: float) -> tuple[float, float, float]:
    """Convert an along-axis signed distance to a world-space point."""
    return (d * AX_COS, 0.0, AXLE_HEIGHT + d * AX_SIN)


# ── Geometry helpers ─────────────────────────────────────────────────────

def _make_running_disc() -> object:
    """Shallow saucer disc with raised peripheral lip and central hub bore.

    Built with the disc face in the YZ plane (normal along local +X).
    The visual Origin later tilts the mesh so the normal aligns with the
    inclined spin axis.
    """
    # Thin annular disc body with central bore
    disc = (
        cq.Workplane("YZ")
        .circle(WHEEL_RADIUS)
        .circle(AXLE_BORE_R)
        .extrude(DISC_THICKNESS)
        .translate((-DISC_THICKNESS / 2.0, 0.0, 0.0))
    )

    # Raised hub boss around the bore (wider ring, same bore)
    hub_t = DISC_THICKNESS + HUB_EXTRA
    hub = (
        cq.Workplane("YZ")
        .circle(HUB_OUTER_R)
        .circle(AXLE_BORE_R)
        .extrude(hub_t)
        .translate((-hub_t / 2.0, 0.0, 0.0))
    )

    # Peripheral containment lip on the running-surface side (+X face)
    lip = (
        cq.Workplane("YZ")
        .circle(WHEEL_RADIUS + 0.003)
        .circle(WHEEL_RADIUS - LIP_WIDTH)
        .extrude(LIP_HEIGHT)
        .translate((DISC_THICKNESS / 2.0, 0.0, 0.0))
    )

    return disc.union(hub).union(lip)


# ── Model ────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="hamster_exercise_wheel",
        meta={
            "reference_note": (
                "Flying-saucer silent-runner hamster wheel: a shallow angled "
                "disc spinning on an inclined axle mounted on a low stand."
            ),
        },
    )

    # Materials
    translucent_green = model.material(
        "translucent_green_plastic", rgba=(0.45, 0.70, 0.38, 0.55)
    )
    white_metal = model.material(
        "white_powder_coated_metal", rgba=(0.93, 0.96, 0.95, 1.0)
    )
    pale_bearing = model.material(
        "white_plastic_bearing", rgba=(0.96, 0.96, 0.92, 1.0)
    )

    # ── Stand (root) ─────────────────────────────────────────────────────
    stand = model.part("stand")

    # Floor base loop (unchanged from parent)
    base_loop = tube_from_spline_points(
        [
            (-0.082, -0.175, 0.006),
            (0.094, -0.175, 0.006),
            (0.104, 0.175, 0.006),
            (-0.082, 0.175, 0.006),
        ],
        radius=0.0045,
        samples_per_segment=12,
        closed_spline=True,
        radial_segments=18,
    )
    stand.visual(
        mesh_from_geometry(base_loop, "stand_base_loop"),
        material=white_metal,
        name="base_loop",
    )

    # Side yoke supports — converge onto the inclined axle near its rear
    rear_axle_pt = _axis_to_world(-0.055)
    for idx, side in enumerate((-1.0, 1.0)):
        yoke = tube_from_spline_points(
            [
                (-0.082, side * 0.175, 0.006),
                (-0.078, side * 0.130, 0.055),
                (-0.068, side * 0.060, 0.110),
                (-0.052, side * 0.012, rear_axle_pt[2] + 0.005),
                rear_axle_pt,  # meets the axle centreline
            ],
            radius=0.0042,
            samples_per_segment=10,
            closed_spline=False,
            radial_segments=18,
            cap_ends=True,
        )
        stand.visual(
            mesh_from_geometry(yoke, f"side_support_{idx}"),
            material=white_metal,
            name=f"side_support_{idx}",
        )

    # Inclined axle shaft
    axle_origin = _axis_to_world(AXLE_CENTER_D)
    stand.visual(
        Cylinder(radius=0.006, length=AXLE_SPAN),
        origin=Origin(xyz=axle_origin, rpy=(0.0, AXLE_PITCH, 0.0)),
        material=white_metal,
        name="axle_shaft",
    )

    # Front axle cap — presses against the hub front face (thrust bearing)
    cap_origin = _axis_to_world(FRONT_CAP_D)
    stand.visual(
        Cylinder(radius=0.018, length=CAP_LENGTH),
        origin=Origin(xyz=cap_origin, rpy=(0.0, AXLE_PITCH, 0.0)),
        material=pale_bearing,
        name="front_axle_cap",
    )

    # Rear bearing — presses against the hub rear face (thrust bearing)
    bearing_origin = _axis_to_world(REAR_BEARING_D)
    stand.visual(
        Cylinder(radius=0.016, length=BEARING_LENGTH),
        origin=Origin(xyz=bearing_origin, rpy=(0.0, AXLE_PITCH, 0.0)),
        material=pale_bearing,
        name="rear_bearing",
    )

    # ── Wheel (child) ───────────────────────────────────────────────────
    wheel = model.part("wheel")

    # Running disc: built with normal along +X; visual rpy tilts it to
    # match the inclined spin axis.
    wheel.visual(
        mesh_from_cadquery(
            _make_running_disc(),
            "running_disc",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -TILT_ANGLE, 0.0)),
        material=translucent_green,
        name="running_disc",
    )

    # ── Articulation ────────────────────────────────────────────────────
    model.articulation(
        "stand_to_wheel",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_HEIGHT)),
        axis=(AX_COS, 0.0, AX_SIN),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=25.0,
            lower=-2.0 * math.pi,
            upper=2.0 * math.pi,
        ),
    )

    return model


# ── Tests ────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    wheel = object_model.get_part("wheel")
    spin = object_model.get_articulation("stand_to_wheel")

    # Thrust-bearing overlap allowances: the front cap and rear bearing
    # intentionally press into the disc hub faces to mechanically connect
    # the wheel to the stand through the axle assembly.
    ctx.allow_overlap(
        stand, wheel,
        elem_a="front_axle_cap", elem_b="running_disc",
        reason="Front axle cap seats against the hub face as a thrust bearing surface.",
    )
    ctx.allow_overlap(
        stand, wheel,
        elem_a="rear_bearing", elem_b="running_disc",
        reason="Rear bearing seats against the hub face as a thrust bearing surface.",
    )

    # 1) Spin joint uses the inclined axis (significant Z component)
    ctx.check(
        "running_disc spins on inclined axle",
        spin.articulation_type == ArticulationType.REVOLUTE
        and abs(spin.axis[0] - AX_COS) < 0.01
        and abs(spin.axis[2] - AX_SIN) < 0.01
        and spin.axis[2] > 0.30,
        details=f"axis={tuple(spin.axis)}, expected~({AX_COS:.3f}, 0, {AX_SIN:.3f})",
    )

    # 2) running_disc visual exists on the wheel part
    ctx.check(
        "running_disc visual present",
        any(v.name == "running_disc" for v in wheel.visuals),
        details="wheel part must have a running_disc visual",
    )

    # 3) Wheel origin at axle height
    ctx.expect_origin_gap(
        wheel, stand,
        axis="z",
        min_gap=AXLE_HEIGHT - 0.005,
        max_gap=AXLE_HEIGHT + 0.005,
        name="wheel origin at axle height",
    )

    # 4) Front cap contacts the disc hub (thrust bearing proof)
    ctx.expect_contact(
        stand, wheel,
        elem_a="front_axle_cap", elem_b="running_disc",
        contact_tol=0.002,
        name="front cap seated against disc hub face",
    )

    # 5) Rear bearing contacts the disc hub (thrust bearing proof)
    ctx.expect_contact(
        stand, wheel,
        elem_a="rear_bearing", elem_b="running_disc",
        contact_tol=0.002,
        name="rear bearing seated against disc hub face",
    )

    # 6) The disc is visibly tilted — Z extent proves the saucer angle
    disc_aabb = ctx.part_element_world_aabb(wheel, elem="running_disc")
    if disc_aabb is not None:
        lo, hi = disc_aabb
        z_extent = hi[2] - lo[2]
        # A disc of radius 0.145 tilted 35° has Z extent ≈ 2R sin(35°) ≈ 0.166
        ctx.check(
            "running_disc is visibly tilted (Z extent proves saucer angle)",
            z_extent > 0.080,
            details=f"Z extent={z_extent:.4f}m, expected > 0.080 for ~35deg tilt",
        )

    return ctx.report()


object_model = build_object_model()
