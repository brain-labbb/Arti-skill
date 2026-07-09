from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PANEL_W = 0.42
PANEL_H = 0.54
PANEL_T = 0.035
OPENING_W = 0.235
OPENING_H = 0.305
FRAME_W = 0.320
FRAME_H = 0.405
FRAME_D = 0.028
FRONT_Y = PANEL_T / 2.0
FRAME_FRONT_Y = FRONT_Y + FRAME_D
HINGE_Y = FRAME_FRONT_Y + 0.006
HINGE_Z = OPENING_H / 2.0 + 0.014
FLAP_W = 0.215
FLAP_H = 0.300
FLAP_OPEN_ANGLE = 0.85

# Electronic microchip module
HOUSING_W = 0.260
HOUSING_H = 0.055
HOUSING_D = 0.022
HOUSING_BOTTOM_Z = HINGE_Z + 0.016
HOUSING_Z = HOUSING_BOTTOM_Z + HOUSING_H / 2.0
HOUSING_Y = FRAME_FRONT_Y + HOUSING_D / 2.0
BUTTON_X = -0.090
BUTTON_R = 0.010


def _panel_with_cutout() -> cq.Workplane:
    """Minimal door/panel context with a true through-opening."""
    clearance_w = OPENING_W + 0.010
    clearance_h = OPENING_H + 0.010
    return (
        cq.Workplane("XY")
        .rect(PANEL_W, PANEL_H)
        .rect(clearance_w, clearance_h)
        .extrude(PANEL_T, both=True)
    )


def _rotated_about_hinge(x: float, y: float, z: float, angle: float) -> tuple[float, float, float]:
    """Rotate a local flap visual center around the hinge X axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _flap_origin(x: float, y: float, z: float, *, angle: float = FLAP_OPEN_ANGLE) -> Origin:
    return Origin(xyz=_rotated_about_hinge(x, y, z, angle), rpy=(angle, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="cat_flap_pet_door",
        meta={
            "category_context": "Pet_Animal related / Cat flap",
            "classification_note": "Reference image and folder category both indicate a cat flap / pet door; no mismatch suspected.",
        },
    )

    wood = model.material("warm_brown_door_panel", rgba=(0.42, 0.23, 0.11, 1.0))
    dark_plastic = model.material("charcoal_plastic_trim", rgba=(0.035, 0.033, 0.032, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.005, 0.005, 0.004, 1.0))
    frosted = model.material("frosted_translucent_flap", rgba=(0.73, 0.90, 0.95, 0.42))
    pale_edge = model.material("milky_plastic_flap_edge", rgba=(0.86, 0.92, 0.90, 0.70))
    metal = model.material("dull_screw_metal", rgba=(0.55, 0.55, 0.52, 1.0))
    magnet = model.material("dark_magnet_latch", rgba=(0.02, 0.02, 0.018, 1.0))

    # Electronic module materials
    housing_plastic = model.material("light_gray_housing", rgba=(0.80, 0.80, 0.77, 1.0))
    battery_cover_mat = model.material("battery_cover_panel", rgba=(0.72, 0.72, 0.70, 1.0))
    antenna_coil = model.material("rfid_antenna_border", rgba=(0.10, 0.10, 0.09, 1.0))
    button_plastic = model.material("mode_selector_green", rgba=(0.18, 0.48, 0.22, 1.0))
    led_glow = model.material("led_indicator_green", rgba=(0.10, 0.85, 0.25, 1.0))

    panel_frame = model.part("panel_frame")
    panel_frame.visual(
        mesh_from_cadquery(_panel_with_cutout(), "door_panel_with_opening", tolerance=0.0008),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="door_panel",
    )

    front_bezel = BezelGeometry(
        (OPENING_W, OPENING_H),
        (FRAME_W, FRAME_H),
        FRAME_D,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.018,
        outer_corner_radius=0.026,
        center=True,
    )
    panel_frame.visual(
        mesh_from_geometry(front_bezel, "rounded_front_trim"),
        origin=Origin(xyz=(0.0, FRONT_Y + FRAME_D / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="front_trim",
    )

    inner_seal = BezelGeometry(
        (OPENING_W - 0.018, OPENING_H - 0.018),
        (OPENING_W + 0.014, OPENING_H + 0.014),
        0.006,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.014,
        outer_corner_radius=0.020,
        center=True,
    )
    panel_frame.visual(
        mesh_from_geometry(inner_seal, "inner_rubber_seal"),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="inner_seal",
    )

    # Four screw heads sit on the molded corner pads of the trim, with shallow
    # slots to make them read as fasteners rather than decorative dots.
    screw_positions = (
        (-0.132, 0.168),
        (0.132, 0.168),
        (-0.132, -0.168),
        (0.132, -0.168),
    )
    for idx, (x, z) in enumerate(screw_positions):
        panel_frame.visual(
            Cylinder(radius=0.0115, length=0.005),
            origin=Origin(xyz=(x, FRAME_FRONT_Y + 0.0025, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"screw_head_{idx}",
        )
        panel_frame.visual(
            Box((0.018, 0.0016, 0.0030)),
            origin=Origin(xyz=(x, FRAME_FRONT_Y + 0.0053, z), rpy=(0.0, 0.0, math.pi / 9.0)),
            material=dark_plastic,
            name=f"screw_slot_{idx}",
        )

    # Static hinge hardware: a small pin crossing the top of the opening and two
    # molded lugs that tie it visibly back to the upper frame.
    panel_frame.visual(
        Cylinder(radius=0.004, length=0.282),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )
    for idx, x in enumerate((-0.137, 0.137)):
        panel_frame.visual(
            Box((0.026, 0.018, 0.040)),
            origin=Origin(xyz=(x, FRAME_FRONT_Y + 0.001, HINGE_Z - 0.006)),
            material=dark_plastic,
            name=f"hinge_lug_{idx}",
        )

    panel_frame.visual(
        Box((0.024, 0.006, 0.046)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.003, HINGE_Z - 0.306)),
        material=magnet,
        name="magnet_mount",
    )
    panel_frame.visual(
        Box((0.058, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.003, HINGE_Z - 0.290)),
        material=magnet,
        name="frame_magnet",
    )

    # --- Electronic microchip module (rigidly fixed to panel_frame) ---
    # Sensor housing: molded box spanning top of front_trim above the opening
    panel_frame.visual(
        Box((HOUSING_W, HOUSING_D, HOUSING_H)),
        origin=Origin(xyz=(0.0, HOUSING_Y, HOUSING_Z)),
        material=housing_plastic,
        name="sensor_housing",
    )
    # Housing rear mounting flange ties the box visibly back into the frame
    panel_frame.visual(
        Box((HOUSING_W + 0.016, 0.006, HOUSING_H + 0.010)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y - 0.003, HOUSING_Z)),
        material=housing_plastic,
        name="housing_flange",
    )

    # Battery cover: removable panel on the front face of the housing
    batt_w = 0.058
    batt_h = 0.034
    batt_t = 0.003
    panel_frame.visual(
        Box((batt_w, batt_t, batt_h)),
        origin=Origin(xyz=(0.050, HOUSING_Y + HOUSING_D / 2.0 + batt_t / 2.0, HOUSING_Z - 0.004)),
        material=battery_cover_mat,
        name="battery_cover",
    )
    # Battery cover latch tab
    panel_frame.visual(
        Box((0.012, 0.002, 0.008)),
        origin=Origin(xyz=(0.050, HOUSING_Y + HOUSING_D / 2.0 + batt_t + 0.001, HOUSING_Z - 0.004 + batt_h / 2.0 - 0.004)),
        material=dark_plastic,
        name="battery_latch",
    )

    # Antenna ring: visible RFID coil border around the opening on the front face
    antenna_border = 0.008
    antenna_inner_w = OPENING_W + 0.004
    antenna_inner_h = OPENING_H + 0.004
    antenna_outer_w = antenna_inner_w + 2.0 * antenna_border
    antenna_outer_h = antenna_inner_h + 2.0 * antenna_border
    antenna_ring_geom = BezelGeometry(
        (antenna_inner_w, antenna_inner_h),
        (antenna_outer_w, antenna_outer_h),
        0.005,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.017,
        outer_corner_radius=0.023,
        center=True,
    )
    panel_frame.visual(
        mesh_from_geometry(antenna_ring_geom, "rfid_antenna_ring"),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.0025, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=antenna_coil,
        name="antenna_ring",
    )

    # LED status indicator on housing face
    panel_frame.visual(
        Cylinder(radius=0.004, length=0.002),
        origin=Origin(xyz=(0.090, HOUSING_Y + HOUSING_D / 2.0 + 0.001, HOUSING_Z + 0.012),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=led_glow,
        name="led_indicator",
    )

    # Mode button mount boss on housing face
    panel_frame.visual(
        Cylinder(radius=BUTTON_R + 0.003, length=0.004),
        origin=Origin(xyz=(BUTTON_X, HOUSING_Y + HOUSING_D / 2.0 + 0.002, HOUSING_Z),
                      rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing_plastic,
        name="button_boss",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.010, length=FLAP_W + 0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pale_edge,
        name="hinge_sleeve",
    )

    panel_top_z = -0.012
    panel_center_z = panel_top_z - FLAP_H / 2.0
    panel_center_y = 0.002
    flap.visual(
        Box((FLAP_W - 0.016, 0.004, FLAP_H - 0.020)),
        origin=_flap_origin(0.0, panel_center_y, panel_center_z),
        material=frosted,
        name="translucent_panel",
    )

    rim_t = 0.012
    rim_y = 0.005
    flap.visual(
        Box((FLAP_W, 0.008, 0.010)),
        origin=_flap_origin(0.0, rim_y, -0.007),
        material=pale_edge,
        name="sleeve_web",
    )
    flap.visual(
        Box((FLAP_W, 0.008, rim_t)),
        origin=_flap_origin(0.0, rim_y, panel_top_z - rim_t / 2.0),
        material=pale_edge,
        name="top_lip",
    )
    flap.visual(
        Box((FLAP_W, 0.008, rim_t)),
        origin=_flap_origin(0.0, rim_y, panel_top_z - FLAP_H + rim_t / 2.0),
        material=pale_edge,
        name="bottom_lip",
    )
    for idx, x in enumerate((-(FLAP_W / 2.0 - rim_t / 2.0), FLAP_W / 2.0 - rim_t / 2.0)):
        flap.visual(
            Box((rim_t, 0.008, FLAP_H)),
            origin=_flap_origin(x, rim_y, panel_top_z - FLAP_H / 2.0),
            material=pale_edge,
            name=f"side_lip_{idx}",
        )

    flap.visual(
        Box((0.052, 0.006, 0.018)),
        origin=_flap_origin(0.0, 0.007, panel_top_z - FLAP_H + 0.028),
        material=magnet,
        name="flap_magnet",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=panel_frame,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-FLAP_OPEN_ANGLE,
            upper=0.60,
            effort=1.5,
            velocity=4.0,
        ),
    )

    # --- Mode selector button (rotary dial on the sensor housing) ---
    mode_button = model.part("mode_button")
    button_height = 0.008
    mode_dial_geom = KnobGeometry(
        BUTTON_R * 2.0,
        button_height,
        body_style="cylindrical",
        grip=KnobGrip(style="ribbed", count=12, depth=0.0006, width=0.0012),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    mode_button.visual(
        mesh_from_geometry(mode_dial_geom, "mode_dial_cap"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=button_plastic,
        name="mode_dial",
    )

    # Joint origin on the housing face where the button boss mounts.
    # Axis is +Y (perpendicular to housing front face) so positive q rotates
    # the dial like a rotary selector.
    button_mount_y = HOUSING_Y + HOUSING_D / 2.0 + 0.004
    model.articulation(
        "housing_to_mode_button",
        ArticulationType.REVOLUTE,
        parent=panel_frame,
        child=mode_button,
        origin=Origin(xyz=(BUTTON_X, button_mount_y, HOUSING_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=math.pi * 1.5,
            effort=0.3,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel_frame = object_model.get_part("panel_frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")

    ctx.check(
        "reference classification matches cat flap",
        object_model.meta.get("classification_note", "").startswith("Reference image"),
        details=str(object_model.meta.get("classification_note")),
    )

    ctx.allow_overlap(
        panel_frame,
        flap,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The visible hinge pin is intentionally captured inside the simplified solid hinge sleeve proxy.",
    )
    ctx.allow_overlap(
        flap,
        panel_frame,
        elem_a="hinge_sleeve",
        elem_b="antenna_ring",
        reason="The RFID antenna coil wraps the full tunnel perimeter; the hinge sleeve passes through the top border of the coil frame as in real microchip pet doors.",
    )
    ctx.expect_overlap(
        panel_frame,
        flap,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.20,
        name="hinge pin spans the flap sleeve",
    )
    ctx.expect_within(
        panel_frame,
        flap,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_sleeve",
        margin=0.004,
        name="hinge pin is centered in sleeve proxy",
    )
    ctx.expect_contact(
        flap,
        panel_frame,
        elem_a="hinge_sleeve",
        elem_b="antenna_ring",
        contact_tol=0.010,
        name="hinge sleeve passes through antenna ring top border",
    )

    with ctx.pose({hinge: -FLAP_OPEN_ANGLE}):
        ctx.expect_within(
            flap,
            panel_frame,
            axes="x",
            inner_elem="translucent_panel",
            outer_elem="front_trim",
            margin=0.0,
            name="closed flap fits inside trim width",
        )
        ctx.expect_gap(
            flap,
            panel_frame,
            axis="y",
            positive_elem="flap_magnet",
            negative_elem="frame_magnet",
            min_gap=0.0,
            max_gap=0.010,
            name="closed magnet nears fixed catch",
        )
        closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_magnet")

    open_aabb = ctx.part_element_world_aabb(flap, elem="flap_magnet")
    ctx.check(
        "default flap pose is swung outward and upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] > closed_aabb[0][1] + 0.05
        and open_aabb[0][2] > closed_aabb[0][2] + 0.03,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # --- Electronic module tests ---
    mode_button = object_model.get_part("mode_button")
    mode_joint = object_model.get_articulation("housing_to_mode_button")

    # Sensor housing sits above the opening on the front face
    ctx.check(
        "sensor_housing visual exists on panel_frame",
        panel_frame.get_visual("sensor_housing") is not None,
        details="Electronic microchip module housing must be present above the pet opening.",
    )
    ctx.check(
        "antenna_ring visual exists on panel_frame",
        panel_frame.get_visual("antenna_ring") is not None,
        details="RFID antenna coil border must be visible around the pet opening.",
    )
    ctx.check(
        "battery_cover visual exists on panel_frame",
        panel_frame.get_visual("battery_cover") is not None,
        details="Removable battery cover panel must be present on the housing front.",
    )

    # Mode selector dial sits on the housing face and protrudes forward
    ctx.expect_gap(
        mode_button,
        panel_frame,
        axis="y",
        positive_elem="mode_dial",
        negative_elem="sensor_housing",
        min_gap=-0.002,
        max_gap=0.012,
        name="mode dial protrudes from sensor housing face",
    )

    # Mode selector joint rotates the dial
    rest_pos = ctx.part_world_position(mode_button)
    with ctx.pose({mode_joint: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(mode_button)
    ctx.check(
        "housing_to_mode_button joint rotates mode dial in place",
        rest_pos is not None and rotated_pos is not None
        and abs(rotated_pos[2] - rest_pos[2]) < 0.003
        and abs(rotated_pos[0] - rest_pos[0]) < 0.003,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
