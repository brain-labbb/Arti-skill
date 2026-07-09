from __future__ import annotations

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
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_LENGTH = 0.72
BODY_WIDTH = 0.082
BODY_HEIGHT = 0.030
TOP_Z = BODY_HEIGHT


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded hard-plastic block centered at the local origin."""
    shape = cq.Workplane("XY").box(size[0], size[1], size[2])
    if radius > 0.0:
        try:
            shape = shape.edges("|Z").fillet(radius)
        except Exception:
            # Small decorative blocks can become too small for the requested
            # fillet after booleans; retaining the box is preferable to failing.
            pass
    return shape


def _outlet_plate_shape() -> cq.Workplane:
    """NEMA-style receptacle plate with real cut-through slots."""
    sx, sy, sz = 0.044, 0.035, 0.0048
    plate = _rounded_box((sx, sy, sz), 0.004)

    # Two blade slots and a round/arched ground contact cut all the way through
    # the black face plate so the brass backing below is visible.
    cutters = [
        cq.Workplane("XY").box(0.0048, 0.014, sz * 5).translate((0.008, -0.007, 0.0)),
        cq.Workplane("XY").box(0.0048, 0.014, sz * 5).translate((0.008, 0.007, 0.0)),
        cq.Workplane("XY").box(0.0058, 0.010, sz * 5).translate((-0.010, 0.0, 0.0)),
        cq.Workplane("XY").circle(0.0046).extrude(sz * 5).translate((-0.015, 0.0, -sz * 2.5)),
    ]
    for cutter in cutters:
        plate = plate.cut(cutter)
    return plate


def _master_rocker_shape() -> cq.Workplane:
    """Large bevelled master rocker cap spanning the full switch zone."""
    body = _rounded_box((0.052, 0.024, 0.008), 0.003)
    # Raised grip rib on the pressable end for tactile identity.
    rib = _rounded_box((0.038, 0.004, 0.002), 0.001).translate((0.003, 0.0, 0.005))
    # Small detent ridge on the opposite end.
    detent = _rounded_box((0.004, 0.018, 0.0015), 0.0005).translate((-0.022, 0.0, 0.0047))
    return body.union(rib).union(detent)


def _master_bezel_shape() -> cq.Workplane:
    """Black raised bezel well surrounding the single master rocker."""
    sx, sy, sz = 0.064, 0.034, 0.007
    bezel = _rounded_box((sx, sy, sz), 0.004)
    recess = cq.Workplane("XY").box(0.054, 0.026, sz * 3).translate((0.0, 0.0, 0.002))
    return bezel.cut(recess)


def _mount_tab_shape() -> cq.Workplane:
    """Flat end mounting foot with a screw/keyhole opening."""
    sx, sy, sz = 0.056, 0.078, 0.0045
    tab = _rounded_box((sx, sy, sz), 0.004)
    round_hole = cq.Workplane("XY").circle(0.006).extrude(sz * 5).translate((-0.010, 0.0, -sz * 2.5))
    slot_hole = cq.Workplane("XY").box(0.014, 0.006, sz * 5).translate((0.006, 0.0, 0.0))
    return tab.cut(round_hole).cut(slot_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="electrical_wiring_surge_protector_switch",
        meta={
            "class": "Surge protector switch",
            "category": "Electrical_Wiring",
            "description": "Plug-in yellow and black surge-protector power strip with receptacles and a single master rocker switch.",
        },
    )

    yellow = model.material("safety_yellow_molded_plastic", rgba=(1.0, 0.77, 0.03, 1.0))
    black = model.material("matte_black_plastic_rubber", rgba=(0.004, 0.004, 0.005, 1.0))
    dark = model.material("deep_slot_shadow", rgba=(0.0, 0.0, 0.0, 1.0))
    red = model.material("translucent_red_switch_lens", rgba=(0.85, 0.05, 0.035, 0.92))
    green = model.material("green_indicator_lens", rgba=(0.05, 0.95, 0.28, 0.95))
    brass = model.material("brass_copper_contacts", rgba=(0.88, 0.50, 0.16, 1.0))
    steel = model.material("bright_steel_hardware", rgba=(0.72, 0.76, 0.76, 1.0))
    white = model.material("white_printed_labels", rgba=(0.94, 0.94, 0.86, 1.0))

    housing = model.part("housing")

    # One continuous molded yellow extrusion with slightly rounded vertical
    # corners and raised side ribs like the reference.
    housing.visual(
        mesh_from_cadquery(_rounded_box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT), 0.010), "yellow_main_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=yellow,
        name="yellow_main_shell",
    )

    # Long raised protective ribs and the central divider that separates outlets
    # from the switch zone.
    for y in (-0.043, 0.043):
        for z in (0.014, 0.022, 0.030):
            housing.visual(
                Box((BODY_LENGTH * 0.90, 0.004, 0.0026)),
                origin=Origin(xyz=(0.0, y, z)),
                material=yellow,
                name=f"side_protective_rib_{'p' if y > 0 else 'n'}_{int(z*1000)}",
            )
    housing.visual(
        Box((BODY_LENGTH * 0.82, 0.006, 0.009)),
        origin=Origin(xyz=(0.0, -0.006, TOP_Z + 0.002)),
        material=yellow,
        name="raised_center_divider",
    )

    # Black end caps, mounting feet, and yellow bridge handles are fixed to the
    # housing, matching the shop-duty protector in the reference image.
    mount_tab_mesh = mesh_from_cadquery(_mount_tab_shape(), "mounting_keyhole_tab")
    for sign, tag in ((-1.0, "front"), (1.0, "rear")):
        x_cap = sign * (BODY_LENGTH / 2.0 + 0.006)
        housing.visual(
            mesh_from_cadquery(_rounded_box((0.052, 0.095, 0.034), 0.007), f"{tag}_black_end_cap"),
            origin=Origin(xyz=(x_cap, 0.0, 0.017)),
            material=black,
            name=f"{tag}_black_end_cap",
        )
        housing.visual(
            mount_tab_mesh,
            origin=Origin(xyz=(sign * (BODY_LENGTH / 2.0 + 0.045), 0.0, 0.0025)),
            material=black,
            name=f"{tag}_mounting_keyhole",
        )
        housing.visual(
            Box((0.012, 0.104, 0.010)),
            origin=Origin(xyz=(sign * (BODY_LENGTH / 2.0 + 0.055), 0.0, 0.046)),
            material=yellow,
            name=f"{tag}_yellow_guard_bar",
        )
        for y in (-0.043, 0.043):
            housing.visual(
                Box((0.070, 0.009, 0.017)),
                origin=Origin(xyz=(sign * (BODY_LENGTH / 2.0 + 0.020), y, 0.039)),
                material=yellow,
                name=f"{tag}_guard_post_{'p' if y > 0 else 'n'}",
            )
        # Visible dark screw heads in the end cap.
        for y in (-0.032, 0.032):
            housing.visual(
                Cylinder(radius=0.0045, length=0.002),
                origin=Origin(xyz=(sign * (BODY_LENGTH / 2.0 + 0.006), y, 0.0348), rpy=(0.0, 0.0, 0.0)),
                material=steel,
                name=f"{tag}_end_screw_{'p' if y > 0 else 'n'}",
            )

    # Eight receptacles with real cut-through black plates and copper/brass
    # contact backers visible in the slots.
    outlet_mesh = mesh_from_cadquery(_outlet_plate_shape(), "nema_receptacle_plate")
    outlet_xs = [-0.285, -0.205, -0.125, -0.045, 0.035, 0.115, 0.195, 0.275]
    for i, x in enumerate(outlet_xs):
        housing.visual(
            Box((0.031, 0.026, 0.0018)),
            origin=Origin(xyz=(x, 0.018, TOP_Z + 0.001)),
            material=brass,
            name=f"brass_contact_backing_{i}",
        )
        housing.visual(
            outlet_mesh,
            origin=Origin(xyz=(x, 0.018, TOP_Z + 0.0042)),
            material=black,
            name=f"outlet_plate_{i}",
        )
        # Slight shadow pocket around each outlet plate.
        housing.visual(
            Box((0.051, 0.042, 0.0015)),
            origin=Origin(xyz=(x, 0.018, TOP_Z + 0.0004)),
            material=dark,
            name=f"outlet_recess_shadow_{i}",
        )
        # Two molded screw heads per outlet plate.
        for yy in (-0.0145, 0.0145):
            housing.visual(
                Cylinder(radius=0.0022, length=0.0015),
                origin=Origin(xyz=(x + 0.017, 0.018 + yy, TOP_Z + 0.0072)),
                material=dark,
                name=f"outlet_screw_{i}_{'p' if yy > 0 else 'n'}",
            )

    # Single master switch bezel at the cord end (positive X). The bezel is a
    # fixed housing visual; the rocker cap is an articulated child below.
    master_bezel_x = 0.310
    master_bezel_y = -0.027
    master_bezel_mesh = mesh_from_cadquery(_master_bezel_shape(), "master_switch_bezel")
    housing.visual(
        master_bezel_mesh,
        origin=Origin(xyz=(master_bezel_x, master_bezel_y, TOP_Z + 0.0035)),
        material=black,
        name="master_switch_bezel",
    )
    # Printed ON / OFF labels flanking the master rocker, embedded slightly into
    # the housing top face so they read as surface-printed markings.
    housing.visual(
        Box((0.014, 0.0015, 0.0012)),
        origin=Origin(xyz=(master_bezel_x + 0.018, master_bezel_y - 0.012, TOP_Z + 0.0002)),
        material=white,
        name="master_off_label",
    )
    housing.visual(
        Box((0.014, 0.0015, 0.0012)),
        origin=Origin(xyz=(master_bezel_x - 0.018, master_bezel_y + 0.012, TOP_Z + 0.0002)),
        material=white,
        name="master_on_label",
    )

    # Green protection indicator.
    housing.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.240, 0.000, TOP_Z + 0.006), rpy=(0.0, 0.0, 0.0)),
        material=green,
        name="green_protected_indicator_lens",
    )

    # Cable strain relief and power cord, then a round molded plug with visible
    # metal blades. The cord is a swept tube rather than a straight proxy.
    housing.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(xyz=(BODY_LENGTH / 2.0 + 0.038, 0.0, 0.027), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="ribbed_strain_relief_boot",
    )
    for j in range(4):
        housing.visual(
            Cylinder(radius=0.013, length=0.0025),
            origin=Origin(
                xyz=(BODY_LENGTH / 2.0 + 0.018 + j * 0.007, 0.0, 0.027),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black,
            name=f"strain_relief_rib_{j}",
        )
    cable_geom = tube_from_spline_points(
        [
            (BODY_LENGTH / 2.0 + 0.052, 0.0, 0.028),
            (0.505, 0.050, 0.045),
            (0.300, 0.230, 0.040),
            (-0.020, 0.315, 0.033),
            (-0.270, 0.270, 0.020),
        ],
        radius=0.006,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    housing.visual(
        mesh_from_geometry(cable_geom, "curved_black_power_cable"),
        material=black,
        name="curved_black_power_cable",
    )
    housing.visual(
        Cylinder(radius=0.030, length=0.017),
        origin=Origin(xyz=(-0.300, 0.265, 0.010), rpy=(0.0, 0.0, 0.0)),
        material=black,
        name="round_molded_plug_body",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(-0.275, 0.267, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="plug_cable_grommet",
    )
    for i, xx in enumerate((-0.309, -0.291)):
        housing.visual(
            Box((0.0045, 0.0025, 0.034)),
            origin=Origin(xyz=(xx, 0.250, 0.0355)),
            material=steel,
            name=f"plug_flat_blade_{i}",
        )
    housing.visual(
        Cylinder(radius=0.0028, length=0.030),
        origin=Origin(xyz=(-0.300, 0.282, 0.0335), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="plug_ground_pin",
    )

    # Single master rocker switch. The child frame sits at the real pivot line
    # in the center of the master bezel; the cap has detented +/- angular stops.
    master_rocker = model.part("master_rocker")
    rocker_mesh = mesh_from_cadquery(_master_rocker_shape(), "master_rocker_cap")
    master_rocker.visual(
        rocker_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -0.10, 0.0)),
        material=red,
        name="rocker_shell",
    )
    # Pivot pin captured in the bezel trunnion pockets.
    master_rocker.visual(
        Cylinder(radius=0.003, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="pivot_pin",
    )
    # Raised white bars on the cap give testable end markers and read as
    # printed ON/OFF ticks in the molded red actuator. Both markers extend
    # through the rocker body top surface to guarantee mesh connectivity.
    master_rocker.visual(
        Box((0.016, 0.004, 0.004)),
        origin=Origin(xyz=(0.005, 0.0, 0.004)),
        material=white,
        name="on_end_marker",
    )
    master_rocker.visual(
        Box((0.005, 0.016, 0.004)),
        origin=Origin(xyz=(-0.022, 0.0, 0.004)),
        material=white,
        name="off_end_marker",
    )
    model.articulation(
        "housing_to_master_rocker",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=master_rocker,
        origin=Origin(xyz=(master_bezel_x, master_bezel_y, TOP_Z + 0.011)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-0.25, upper=0.25),
        meta={"mechanism": "detented master rocker switch", "lower_label": "OFF", "upper_label": "ON"},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    master_rocker = object_model.get_part("master_rocker")
    master_joint = object_model.get_articulation("housing_to_master_rocker")

    # The pivot pin is intentionally captured in the master bezel's molded
    # trunnion pocket.
    ctx.allow_overlap(
        housing,
        master_rocker,
        elem_a="master_switch_bezel",
        elem_b="pivot_pin",
        reason="The master rocker pivot pin is intentionally captured in the black bezel's molded trunnion pocket.",
    )
    ctx.expect_overlap(
        housing,
        master_rocker,
        axes="xy",
        elem_a="master_switch_bezel",
        elem_b="pivot_pin",
        min_overlap=0.005,
        name="master rocker pivot pin is laterally captured by bezel",
    )
    ctx.expect_gap(
        master_rocker,
        housing,
        axis="z",
        positive_elem="pivot_pin",
        negative_elem="master_switch_bezel",
        max_penetration=0.005,
        max_gap=0.003,
        name="master rocker pivot pin sits in bezel pocket",
    )

    ctx.check(
        "asset remains the Surge protector switch class",
        object_model.name == "electrical_wiring_surge_protector_switch"
        and object_model.meta.get("class") == "Surge protector switch",
        details=f"name={object_model.name!r}, meta={object_model.meta}",
    )

    visual_names = {v.name for v in housing.visuals}
    ctx.check(
        "eight receptacle plates with visible brass contacts",
        sum(name.startswith("outlet_plate_") for name in visual_names) == 8
        and sum(name.startswith("brass_contact_backing_") for name in visual_names) == 8,
        details=f"housing visual count={len(visual_names)}",
    )
    ctx.check(
        "electrical details include cable plug indicator and strain relief",
        {
            "curved_black_power_cable",
            "round_molded_plug_body",
            "plug_flat_blade_0",
            "green_protected_indicator_lens",
            "ribbed_strain_relief_boot",
        }.issubset(visual_names),
        details=f"missing={sorted({'curved_black_power_cable','round_molded_plug_body','plug_flat_blade_0','green_protected_indicator_lens','ribbed_strain_relief_boot'} - visual_names)}",
    )

    # Exactly one revolute joint: the single master rocker.
    ctx.check(
        "surge protector has exactly one master rocker joint",
        len(object_model.articulations) == 1
        and object_model.articulations[0].name == "housing_to_master_rocker"
        and object_model.articulations[0].articulation_type == ArticulationType.REVOLUTE,
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    # Master rocker cap is centered in its bezel.
    ctx.expect_overlap(
        housing,
        master_rocker,
        axes="xy",
        elem_a="master_switch_bezel",
        elem_b="rocker_shell",
        min_overlap=0.015,
        name="master rocker cap is centered in the master bezel",
    )
    ctx.expect_gap(
        master_rocker,
        housing,
        axis="z",
        positive_elem="rocker_shell",
        negative_elem="master_switch_bezel",
        max_penetration=0.004,
        max_gap=0.010,
        name="master rocker rides just proud of the recessed bezel",
    )

    # Prove the rocker actually rotates between detent stops.
    with ctx.pose({master_joint: -0.25}):
        off_aabb = ctx.part_element_world_aabb(master_rocker, elem="on_end_marker")
    with ctx.pose({master_joint: 0.25}):
        on_aabb = ctx.part_element_world_aabb(master_rocker, elem="on_end_marker")

    off_z = None if off_aabb is None else (off_aabb[0][2] + off_aabb[1][2]) / 2.0
    on_z = None if on_aabb is None else (on_aabb[0][2] + on_aabb[1][2]) / 2.0
    ctx.check(
        "master rocker switch rotates between detent stops",
        off_z is not None and on_z is not None and off_z > on_z + 0.002,
        details=f"off detent on-end z={off_z}, on detent on-end z={on_z}",
    )

    # No per-outlet rocker parts should exist.
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no per-outlet rocker parts remain",
        not any(name.startswith("switch_") and name != "master_rocker" for name in part_names),
        details=f"parts={sorted(part_names)}",
    )

    return ctx.report()


object_model = build_object_model()
