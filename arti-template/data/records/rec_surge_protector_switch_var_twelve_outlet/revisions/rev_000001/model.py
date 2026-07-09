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


N_OUTLETS = 12
OUTLET_PITCH = 0.08
OUTLET_SPAN = (N_OUTLETS - 1) * OUTLET_PITCH
BODY_LENGTH = OUTLET_SPAN + 2 * OUTLET_PITCH  # one pitch margin each end
BODY_WIDTH = 0.082
BODY_HEIGHT = 0.030
TOP_Z = BODY_HEIGHT
SWITCH_BEZEL_NAMES = (
    "switch_bezel_0",
    "switch_bezel_1",
    "switch_bezel_2",
    "switch_bezel_3",
    "switch_bezel_4",
    "switch_bezel_5",
    "switch_bezel_6",
    "switch_bezel_7",
    "switch_bezel_8",
    "switch_bezel_9",
    "switch_bezel_10",
    "switch_bezel_11",
)


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


def _rocker_shape() -> cq.Workplane:
    """Small bevelled rectangular rocker cap."""
    body = _rounded_box((0.027, 0.016, 0.006), 0.0022)
    # A shallow raised rib on the cap makes the molded red actuator read as a
    # real pressable rocker rather than a flat color patch.
    rib = _rounded_box((0.019, 0.003, 0.0015), 0.0008).translate((0.000, 0.0, 0.0037))
    return body.union(rib)


def _switch_bezel_shape() -> cq.Workplane:
    """Black raised switch well / bezel surrounding a rocker."""
    sx, sy, sz = 0.036, 0.024, 0.006
    bezel = _rounded_box((sx, sy, sz), 0.0035)
    recess = cq.Workplane("XY").box(0.029, 0.017, sz * 3).translate((0.0, 0.0, 0.0015))
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
            "description": "Plug-in yellow and black surge-protector power strip with receptacles and articulated red rocker switches.",
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
    # from the switch bank.
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

    # Twelve receptacles with real cut-through black plates and copper/brass
    # contact backers visible in the slots.
    outlet_mesh = mesh_from_cadquery(_outlet_plate_shape(), "nema_receptacle_plate")
    outlet_xs = [-OUTLET_SPAN / 2.0 + i * OUTLET_PITCH for i in range(N_OUTLETS)]
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

    # Switch bezels and printed ON/OFF marks stay on the fixed housing. Red
    # rocker caps are separate articulated children below.
    switch_bezel_mesh = mesh_from_cadquery(_switch_bezel_shape(), "individual_switch_bezel")
    for i, x in enumerate(outlet_xs):
        switch_bezel_name = SWITCH_BEZEL_NAMES[i]
        housing.visual(
            switch_bezel_mesh,
            origin=Origin(xyz=(x, -0.027, TOP_Z + 0.003)),
            material=black,
            name=switch_bezel_name,
        )
        housing.visual(
            Box((0.010, 0.0013, 0.001)),
            origin=Origin(xyz=(x + 0.011, -0.0365, TOP_Z + 0.0065)),
            material=white,
            name=f"off_label_bar_{i}",
        )
        housing.visual(
            Box((0.010, 0.0013, 0.001)),
            origin=Origin(xyz=(x - 0.011, -0.0175, TOP_Z + 0.0065)),
            material=white,
            name=f"on_label_bar_{i}",
        )

    # Green protection indicator near the cord end of the strip, seated into
    # the housing top face so the lens reads as a flush-mounted dome.
    housing.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(OUTLET_SPAN / 2.0 + 0.03, 0.000, TOP_Z + 0.001), rpy=(0.0, 0.0, 0.0)),
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
    cable_end_x = BODY_LENGTH / 2.0 + 0.052
    cable_geom = tube_from_spline_points(
        [
            (cable_end_x, 0.0, 0.028),
            (cable_end_x - 0.07, 0.050, 0.045),
            (cable_end_x - 0.28, 0.230, 0.040),
            (cable_end_x - 0.60, 0.315, 0.033),
            (cable_end_x - 0.85, 0.270, 0.020),
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
    plug_x = cable_end_x - 0.85
    housing.visual(
        Cylinder(radius=0.030, length=0.017),
        origin=Origin(xyz=(plug_x, 0.265, 0.010), rpy=(0.0, 0.0, 0.0)),
        material=black,
        name="round_molded_plug_body",
    )
    housing.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(plug_x + 0.025, 0.267, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="plug_cable_grommet",
    )
    for i, dx in enumerate((-0.009, 0.009)):
        housing.visual(
            Box((0.0045, 0.0025, 0.034)),
            origin=Origin(xyz=(plug_x + dx, 0.250, 0.0355)),
            material=steel,
            name=f"plug_flat_blade_{i}",
        )
    housing.visual(
        Cylinder(radius=0.0028, length=0.030),
        origin=Origin(xyz=(plug_x, 0.282, 0.0335), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="plug_ground_pin",
    )

    # Articulated red rocker caps. Each child frame is the real pivot line at
    # the center of its black bezel; the cap has detented +/- angular stops.
    rocker_mesh = mesh_from_cadquery(_rocker_shape(), "red_rocker_cap")
    for i, x in enumerate(outlet_xs):
        rocker = model.part(f"switch_{i}")
        rocker.visual(
            rocker_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -0.10, 0.0)),
            material=red,
            name="rocker_shell",
        )
        rocker.visual(
            Cylinder(radius=0.0025, length=0.029),
            origin=Origin(xyz=(0.0, 0.0, -0.0040), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="pivot_pin",
        )
        # Tiny raised white bars on the cap give testable end markers and read as
        # printed ON/OFF ticks in the molded red actuator.
        rocker.visual(
            Box((0.018, 0.0016, 0.001)),
            origin=Origin(xyz=(0.0045, 0.0, 0.0027), rpy=(0.0, -0.10, 0.0)),
            material=white,
            name="on_end_marker",
        )
        rocker.visual(
            Box((0.008, 0.0016, 0.001)),
            origin=Origin(xyz=(-0.009, 0.0, 0.0027), rpy=(0.0, -0.10, 0.0)),
            material=white,
            name="off_end_marker",
        )
        model.articulation(
            f"housing_to_switch_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=rocker,
            origin=Origin(xyz=(x, -0.027, TOP_Z + 0.010)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.7, velocity=4.0, lower=-0.22, upper=0.22),
            meta={"mechanism": "detented rocker switch", "lower_label": "OFF", "upper_label": "ON"},
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    first_switch = object_model.get_part("switch_0")
    first_joint = object_model.get_articulation("housing_to_switch_0")

    for i, bezel_name in enumerate(SWITCH_BEZEL_NAMES):
        switch = object_model.get_part(f"switch_{i}")
        ctx.allow_overlap(
            housing,
            switch,
            elem_a=bezel_name,
            elem_b="pivot_pin",
            reason="The switch pivot pin is intentionally captured in the black bezel's molded trunnion pocket.",
        )
        ctx.expect_overlap(
            housing,
            switch,
            axes="xy",
            elem_a=bezel_name,
            elem_b="pivot_pin",
            min_overlap=0.004,
            name=f"switch {i} pivot pin is laterally captured by bezel",
        )
        ctx.expect_gap(
            switch,
            housing,
            axis="z",
            positive_elem="pivot_pin",
            negative_elem=bezel_name,
            max_penetration=0.004,
            max_gap=0.002,
            name=f"switch {i} pivot pin sits in bezel pocket",
        )
    ctx.check(
        "asset remains the Surge protector switch class",
        object_model.name == "electrical_wiring_surge_protector_switch"
        and object_model.meta.get("class") == "Surge protector switch",
        details=f"name={object_model.name!r}, meta={object_model.meta}",
    )

    visual_names = {v.name for v in housing.visuals}
    ctx.check(
        "twelve receptacle plates with visible brass contacts",
        sum(name.startswith("outlet_plate_") for name in visual_names) == N_OUTLETS
        and sum(name.startswith("brass_contact_backing_") for name in visual_names) == N_OUTLETS,
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
    ctx.check(
        "surge protector has twelve aligned rocker mechanisms (N=12 fork)",
        len(object_model.articulations) == N_OUTLETS
        and all(j.name == f"housing_to_switch_{i}" for i, j in enumerate(object_model.articulations))
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    # Last rocker and its articulation prove the N=12 extension exists.
    last_switch = object_model.get_part(f"switch_{N_OUTLETS - 1}")
    last_joint = object_model.get_articulation(f"housing_to_switch_{N_OUTLETS - 1}")
    ctx.check(
        f"switch_{N_OUTLETS - 1} exists with revolute joint (N=12 axis)",
        last_switch is not None
        and last_joint is not None
        and last_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"last joint={last_joint.name if last_joint else None}",
    )

    ctx.expect_overlap(
        housing,
        first_switch,
        axes="xy",
        elem_a="switch_bezel_0",
        elem_b="rocker_shell",
        min_overlap=0.010,
        name="rocker cap is centered in black switch bezel",
    )
    ctx.expect_gap(
        first_switch,
        housing,
        axis="z",
        positive_elem="rocker_shell",
        negative_elem="switch_bezel_0",
        max_penetration=0.001,
        max_gap=0.006,
        name="rocker rides just proud of the recessed bezel",
    )

    # Prove the last (12th) rocker is centered in its bezel too, confirming
    # the N=12 multiplicity extension reached the far end of the strip.
    last_bezel = f"switch_bezel_{N_OUTLETS - 1}"
    ctx.expect_overlap(
        housing,
        last_switch,
        axes="xy",
        elem_a=last_bezel,
        elem_b="rocker_shell",
        min_overlap=0.010,
        name=f"switch_{N_OUTLETS - 1} rocker centered in its bezel",
    )

    with ctx.pose({first_joint: -0.22}):
        off_aabb = ctx.part_element_world_aabb(first_switch, elem="on_end_marker")
    with ctx.pose({first_joint: 0.22}):
        on_aabb = ctx.part_element_world_aabb(first_switch, elem="on_end_marker")

    off_z = None if off_aabb is None else (off_aabb[0][2] + off_aabb[1][2]) / 2.0
    on_z = None if on_aabb is None else (on_aabb[0][2] + on_aabb[1][2]) / 2.0
    ctx.check(
        "rocker switch rotates between detent stops",
        off_z is not None and on_z is not None and off_z > on_z + 0.0015,
        details=f"off detent on-end z={off_z}, on detent on-end z={on_z}",
    )

    return ctx.report()


object_model = build_object_model()
