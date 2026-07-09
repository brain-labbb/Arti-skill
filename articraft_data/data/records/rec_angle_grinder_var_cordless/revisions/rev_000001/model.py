from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------------------
# Layout constants (meters). Motor axis runs along X: rear grip at -X, gear
# head at +X. The spindle axis is vertical (Z) with the fiber disc underneath
# the gear head, matching the reference image of a compact DeWalt grinder.
# ---------------------------------------------------------------------------
MOTOR_AXIS_Z = 0.062  # height of the motor centerline
BARREL_RADIUS = 0.035  # ~0.07 m diameter motor housing
BARREL_FRONT_X = -0.035  # front face of the motor barrel (meets the gear head)
HOUSING_REAR_X = -0.245  # rear cap of the grip taper (overall length 0.28 m)
HEAD_REAR_X = -0.050
HEAD_FRONT_X = 0.035
HEAD_CENTER_Z = 0.060
HEAD_TOP_Z = 0.0991  # flat gear-head top directly above the spindle
COLLAR_BOTTOM_Z = 0.014  # bottom plane of the spindle collar boss
SWITCH_X = -0.115  # slide-switch joint frame on top of the barrel
DISC_RADIUS = 0.0575  # 0.115 m diameter abrasive disc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dewalt_angle_grinder")

    yellow = model.material("housing_yellow", rgba=(0.95, 0.72, 0.06, 1.0))
    black = model.material("trim_black", rgba=(0.08, 0.08, 0.08, 1.0))
    steel = model.material("steel_grey", rgba=(0.60, 0.61, 0.63, 1.0))
    fiber = model.material("fiber_disc_brown", rgba=(0.46, 0.38, 0.29, 1.0))

    # ------------------------------------------------------------------
    # body (root): motor housing, rear grip taper, gear head, collar,
    # side-handle boss, vent slots, brand label, and battery rail interface.
    # ------------------------------------------------------------------
    body = model.part("body")

    # Tapered cylindrical motor housing built as a lathe along +Z, then
    # rotated so the lathe axis becomes -X (front at x=BARREL_FRONT_X).
    housing_profile = [
        (0.0, 0.0),
        (0.0345, 0.0),
        (0.035, 0.012),
        (0.035, 0.130),
        (0.032, 0.160),
        (0.0285, 0.185),
        (0.0265, 0.205),
        (0.0258, 0.210),
        (0.0, 0.210),
    ]
    housing_geom = LatheGeometry(housing_profile, segments=48)
    housing_geom.rotate_y(-math.pi / 2.0)  # lathe +Z -> world -X
    housing_geom.translate(BARREL_FRONT_X, 0.0, MOTOR_AXIS_Z)
    body.visual(
        mesh_from_geometry(housing_geom, "motor_housing"),
        material=yellow,
        name="motor_housing",
    )

    # Right-angle gear head: rounded-rectangle loft that narrows toward the
    # front face. Built along +Z then rotated so the loft axis becomes +X.
    head_sections = []
    for vert, lat, rad, zp in (
        (0.080, 0.066, 0.013, 0.000),
        (0.078, 0.064, 0.013, 0.055),
        (0.060, 0.050, 0.010, 0.085),
    ):
        loop = [(px, py, zp) for px, py in rounded_rect_profile(vert, lat, rad)]
        head_sections.append(loop)
    head_geom = LoftGeometry(head_sections, cap=True)
    head_geom.rotate_y(math.pi / 2.0)  # loft +Z -> world +X
    head_geom.translate(HEAD_REAR_X, 0.0, HEAD_CENTER_Z)
    body.visual(
        mesh_from_geometry(head_geom, "gear_head"),
        material=yellow,
        name="gear_head",
    )

    # Spindle collar boss under the gear head (the spindle exits through it).
    body.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=yellow,
        name="spindle_collar",
    )

    # Threaded side-handle boss on the +Y flank of the gear head, with a
    # recessed black thread socket on its outer face.
    body.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(-0.005, 0.038, 0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yellow,
        name="handle_boss",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.003),
        origin=Origin(xyz=(-0.005, 0.0455, 0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="handle_boss_socket",
    )

    # Recessed black ventilation slots around the front of the motor barrel
    # (two axial columns, skipping the top where the slide switch travels).
    slot_index = 0
    for slot_x in (-0.048, -0.078):
        for phi in (-1.15, -0.70, 0.70, 1.15):
            body.visual(
                Box((0.026, 0.0045, 0.006)),
                origin=Origin(
                    xyz=(
                        slot_x,
                        -0.0335 * math.sin(phi),
                        MOTOR_AXIS_Z + 0.0335 * math.cos(phi),
                    ),
                    rpy=(phi, 0.0, 0.0),
                ),
                material=black,
                name=f"vent_slot_{slot_index}",
            )
            slot_index += 1

    # Black vent slots on the gear-head front face.
    for j, yoff in enumerate((-0.011, 0.011)):
        body.visual(
            Box((0.003, 0.005, 0.024)),
            origin=Origin(xyz=(0.0345, yoff, HEAD_CENTER_Z)),
            material=black,
            name=f"head_vent_{j}",
        )

    # Black rectangular brand label panel on the housing flank.
    body.visual(
        Box((0.060, 0.0035, 0.024)),
        origin=Origin(xyz=(-0.115, -0.0342, MOTOR_AXIS_Z)),
        material=black,
        name="brand_label",
    )

    # Battery rail interface on the bottom-rear of the grip. Two parallel
    # yellow rail ridges run along X; the battery pack slides on from behind.
    # Rail base plate sits flush with the grip underside.
    RAIL_BASE_X = -0.210   # center of the rail channel along X
    RAIL_LENGTH = 0.060    # rail travel engagement length
    RAIL_BOTTOM_Z = MOTOR_AXIS_Z - 0.027  # underside of grip where rails mount

    body.visual(
        Box((RAIL_LENGTH, 0.048, 0.004)),
        origin=Origin(xyz=(RAIL_BASE_X, 0.0, RAIL_BOTTOM_Z)),
        material=yellow,
        name="rail_baseplate",
    )
    for i, yoff in enumerate((-0.016, 0.016)):
        body.visual(
            Box((RAIL_LENGTH, 0.006, 0.006)),
            origin=Origin(xyz=(RAIL_BASE_X, yoff, RAIL_BOTTOM_Z - 0.005)),
            material=yellow,
            name=f"rail_guide_{i}",
        )
    # Rear contact wall that the battery seats against.
    body.visual(
        Box((0.004, 0.048, 0.016)),
        origin=Origin(xyz=(RAIL_BASE_X + RAIL_LENGTH / 2.0 + 0.002, 0.0, RAIL_BOTTOM_Z - 0.006)),
        material=black,
        name="rail_rear_stop",
    )

    # ------------------------------------------------------------------
    # spindle_disc: spindle shaft + backing flange + fiber disc + hex clamp
    # nut, spinning continuously about the vertical spindle axis. The joint
    # frame sits at the collar bottom plane (world z = COLLAR_BOTTOM_Z).
    # ------------------------------------------------------------------
    spindle_disc = model.part("spindle_disc")
    spindle_disc.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.0020)),
        material=steel,
        name="spindle_shaft",
    )
    spindle_disc.visual(
        Cylinder(radius=0.013, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, -0.00275)),
        material=steel,
        name="backing_flange",
    )
    spindle_disc.visual(
        Cylinder(radius=DISC_RADIUS, length=0.0028),
        origin=Origin(xyz=(0.0, 0.0, -0.0059)),
        material=fiber,
        name="grinding_disc",
    )
    nut_geom = CylinderGeometry(radius=0.0085, height=0.005, radial_segments=6)
    nut_geom.translate(0.0, 0.0, -0.0098)
    spindle_disc.visual(
        mesh_from_geometry(nut_geom, "clamp_nut"),
        material=steel,
        name="clamp_nut",
    )
    # Off-axis printed label patch on the visible underside of the disc; it
    # also gives the rotation tests a non-axisymmetric feature to track.
    spindle_disc.visual(
        Box((0.016, 0.011, 0.0010)),
        origin=Origin(xyz=(0.030, 0.0, -0.0076)),
        material=black,
        name="disc_label",
    )

    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spindle_disc,
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=120.0),
    )

    # ------------------------------------------------------------------
    # power_switch: black paddle-style slide switch on top of the motor
    # housing. Positive q slides it forward (+X) from OFF (rear) to ON.
    # ------------------------------------------------------------------
    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.034, 0.0165, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=black,
        name="switch_base",
    )
    power_switch.visual(
        Box((0.022, 0.0145, 0.0055)),
        origin=Origin(xyz=(0.004, 0.0, 0.00625)),
        material=black,
        name="switch_paddle",
    )
    for k, rib_x in enumerate((-0.002, 0.004, 0.010)):
        power_switch.visual(
            Box((0.0025, 0.013, 0.0014)),
            origin=Origin(xyz=(rib_x, 0.0, 0.0094)),
            material=black,
            name=f"switch_rib_{k}",
        )

    model.articulation(
        "switch_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_switch,
        origin=Origin(xyz=(SWITCH_X, 0.0, MOTOR_AXIS_Z + BARREL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.012),
    )

    # ------------------------------------------------------------------
    # spindle_lock: small round button on top of the gear head, pressing
    # 4 mm inward (downward) along the spindle axis. Positive q presses in.
    # ------------------------------------------------------------------
    spindle_lock = model.part("spindle_lock")
    spindle_lock.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0030)),
        material=black,
        name="lock_cap",
    )
    spindle_lock.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.0040)),
        material=black,
        name="lock_stem",
    )

    model.articulation(
        "lock_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=spindle_lock,
        origin=Origin(xyz=(0.0, 0.0, HEAD_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.004),
    )

    # ------------------------------------------------------------------
    # battery_pack: slide-on rechargeable battery that seats at the rear
    # of the grip on the prismatic rail joint. Positive q slides the pack
    # backward (-X) to release. At q=0 the pack is fully seated.
    # ------------------------------------------------------------------
    battery_pack = model.part("battery_pack")

    # Main battery enclosure – DeWalt yellow shell with black base.
    # Shell is tall enough to contact the slide plate so the pack reads as
    # one connected assembly.
    battery_pack.visual(
        Box((0.072, 0.052, 0.040)),
        origin=Origin(xyz=(-0.005, 0.0, -0.018)),
        material=yellow,
        name="battery_shell",
    )
    # Black base plate (sliding interface that rides on the rail guides).
    battery_pack.visual(
        Box((0.065, 0.044, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=black,
        name="battery_slide_plate",
    )
    # Rail grooves on top of the battery (engage with body rail guides).
    for i, yoff in enumerate((-0.016, 0.016)):
        battery_pack.visual(
            Box((0.060, 0.007, 0.005)),
            origin=Origin(xyz=(0.0, yoff, 0.0065)),
            material=black,
            name=f"battery_rail_groove_{i}",
        )
    # Release latch on the front face of the battery pack, slightly inset
    # so it connects to the shell body.
    battery_pack.visual(
        Box((0.004, 0.030, 0.014)),
        origin=Origin(xyz=(0.032, 0.0, -0.008)),
        material=black,
        name="battery_latch",
    )
    # Battery indicator LEDs (small colored dots on the rear face).
    for i, yoff in enumerate((-0.010, 0.0, 0.010)):
        battery_pack.visual(
            Box((0.003, 0.006, 0.006)),
            origin=Origin(xyz=(-0.038, yoff, -0.012)),
            material=black,
            name=f"battery_led_{i}",
        )

    BATTERY_RELEASE_TRAVEL = 0.055  # meters of slide to fully disengage

    model.articulation(
        "battery_release",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_pack,
        origin=Origin(xyz=(RAIL_BASE_X, 0.0, RAIL_BOTTOM_Z - 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=BATTERY_RELEASE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spindle_disc = object_model.get_part("spindle_disc")
    power_switch = object_model.get_part("power_switch")
    spindle_lock = object_model.get_part("spindle_lock")
    battery_pack = object_model.get_part("battery_pack")
    spin = object_model.get_articulation("spindle_spin")
    slide = object_model.get_articulation("switch_slide")
    press = object_model.get_articulation("lock_press")
    batt_release = object_model.get_articulation("battery_release")

    # Intentional, scoped mechanical embeddings.
    ctx.allow_overlap(
        spindle_disc,
        body,
        elem_a="spindle_shaft",
        elem_b="spindle_collar",
        reason="The spindle shaft is intentionally captured inside the solid collar boss proxy that stands in for the gear-head bearing bore.",
    )
    ctx.allow_overlap(
        power_switch,
        body,
        elem_a="switch_base",
        elem_b="motor_housing",
        reason="The flat switch base plate is seated slightly into the curved barrel top so the control reads mounted, not floating.",
    )
    ctx.allow_overlap(
        spindle_lock,
        body,
        elem_a="lock_stem",
        elem_b="gear_head",
        reason="The lock-button stem is intentionally captured inside the gear-head boss that stands in for its guide bore.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_rail_groove_0",
        elem_b="rail_guide_0",
        reason="The battery rail grooves intentionally interlock with the body rail guides as a sliding mechanical interface.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_rail_groove_1",
        elem_b="rail_guide_1",
        reason="The battery rail grooves intentionally interlock with the body rail guides as a sliding mechanical interface.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_rail_groove_0",
        elem_b="motor_housing",
        reason="The battery rail grooves nest into the underside of the motor housing where a real T-slot rail channel would be milled; the housing mesh is a simplified lathe proxy without the recess.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_rail_groove_1",
        elem_b="motor_housing",
        reason="The battery rail grooves nest into the underside of the motor housing where a real T-slot rail channel would be milled; the housing mesh is a simplified lathe proxy without the recess.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_shell",
        elem_b="motor_housing",
        reason="The battery shell seats against the curved underside of the motor housing where a real tool would have a flat recess for the pack; the lathe housing proxy does not model that recess.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_shell",
        elem_b="rail_guide_0",
        reason="The battery shell wraps around the rail guide as part of the slide-on battery interface; the guide is captured within the pack body.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_shell",
        elem_b="rail_guide_1",
        reason="The battery shell wraps around the rail guide as part of the slide-on battery interface; the guide is captured within the pack body.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_slide_plate",
        elem_b="motor_housing",
        reason="The battery slide plate contacts the housing underside at the rail mounting interface.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_slide_plate",
        elem_b="rail_baseplate",
        reason="The slide plate rides on top of the rail baseplate as the sliding battery interface.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_slide_plate",
        elem_b="rail_guide_0",
        reason="The slide plate overlaps the rail guide region as part of the T-slot sliding interface.",
    )
    ctx.allow_overlap(
        battery_pack,
        body,
        elem_a="battery_slide_plate",
        elem_b="rail_guide_1",
        reason="The slide plate overlaps the rail guide region as part of the T-slot sliding interface.",
    )

    # --- Hero geometry: disc size, placement, and clamp stack -------------
    disc_aabb = ctx.part_element_world_aabb(spindle_disc, elem="grinding_disc")
    disc_ok = disc_aabb is not None
    if disc_ok:
        dx = disc_aabb[1][0] - disc_aabb[0][0]
        dy = disc_aabb[1][1] - disc_aabb[0][1]
        cx = 0.5 * (disc_aabb[0][0] + disc_aabb[1][0])
        cy = 0.5 * (disc_aabb[0][1] + disc_aabb[1][1])
        ctx.check(
            "disc diameter is ~0.115 m",
            0.112 <= dx <= 0.118 and 0.112 <= dy <= 0.118,
            details=f"dx={dx:.4f}, dy={dy:.4f}",
        )
        ctx.check(
            "disc is centered on the spindle axis",
            abs(cx) < 0.004 and abs(cy) < 0.004,
            details=f"cx={cx:.4f}, cy={cy:.4f}",
        )
    else:
        ctx.fail("disc element resolves", "grinding_disc AABB unavailable")

    ctx.expect_gap(
        body,
        spindle_disc,
        axis="z",
        positive_elem="spindle_collar",
        negative_elem="grinding_disc",
        min_gap=0.001,
        max_gap=0.010,
        name="disc rides just below the gear-head collar",
    )
    ctx.expect_overlap(
        spindle_disc,
        body,
        axes="z",
        elem_a="spindle_shaft",
        elem_b="spindle_collar",
        min_overlap=0.004,
        name="spindle shaft stays inserted in the collar bore",
    )

    nut_aabb = ctx.part_element_world_aabb(spindle_disc, elem="clamp_nut")
    if nut_aabb is not None and disc_aabb is not None:
        ctx.check(
            "hex clamp nut caps the disc from below",
            nut_aabb[1][2] <= disc_aabb[0][2] + 0.001
            and nut_aabb[0][2] < disc_aabb[0][2],
            details=f"nut_z=({nut_aabb[0][2]:.4f},{nut_aabb[1][2]:.4f}), disc_min_z={disc_aabb[0][2]:.4f}",
        )
    else:
        ctx.fail("clamp nut element resolves", "clamp_nut AABB unavailable")

    # --- Overall proportions ----------------------------------------------
    head_aabb = ctx.part_element_world_aabb(body, elem="gear_head")
    barrel_aabb = ctx.part_element_world_aabb(body, elem="motor_housing")
    if head_aabb is not None and barrel_aabb is not None:
        overall = head_aabb[1][0] - barrel_aabb[0][0]
        ctx.check(
            "tool body is ~0.28 m long overall",
            0.270 <= overall <= 0.292,
            details=f"overall={overall:.4f}",
        )
        ctx.check(
            "motor barrel is ~0.07 m in diameter",
            0.066 <= barrel_aabb[1][2] - barrel_aabb[0][2] <= 0.074,
            details=f"barrel_dz={barrel_aabb[1][2] - barrel_aabb[0][2]:.4f}",
        )
    else:
        ctx.fail("body elements resolve", "gear_head/motor_housing AABB unavailable")

    # --- Mounted trim and accents ------------------------------------------
    vents = [v for v in body.visuals if (v.name or "").startswith("vent_slot")]
    ctx.check(
        "recessed black vent slots present on the housing",
        len(vents) >= 8,
        details=f"count={len(vents)}",
    )

    label_aabb = ctx.part_element_world_aabb(body, elem="brand_label")
    ctx.check(
        "brand label panel sits on the housing flank",
        label_aabb is not None and label_aabb[1][1] <= -0.030,
        details=f"label_aabb={label_aabb}",
    )

    boss_aabb = ctx.part_element_world_aabb(body, elem="handle_boss")
    ctx.check(
        "side-handle boss protrudes from the gear-head flank",
        boss_aabb is not None
        and boss_aabb[0][1] < 0.034
        and boss_aabb[1][1] > 0.042,
        details=f"boss_aabb={boss_aabb}",
    )

    # --- Battery rail interface (cordless conversion) ----------------------
    rail_aabb = ctx.part_element_world_aabb(body, elem="rail_baseplate")
    ctx.check(
        "battery rail baseplate present on the grip underside",
        rail_aabb is not None and rail_aabb[1][2] < 0.050,
        details=f"rail_aabb={rail_aabb}",
    )
    # Confirm no cord/boot remnants exist on the body.
    cord_present = any(
        (v.name or "") in ("power_cord", "cord_boot") for v in body.visuals
    )
    ctx.check(
        "power cord and strain-relief boot are removed (cordless)",
        not cord_present,
        details="found cord/boot visuals on body",
    )

    # --- Battery pack seating and rail engagement --------------------------
    # Prove the battery rail grooves interlock with the body rail guides
    # along the non-slide axes (Y centering) and overlap along X (insertion).
    for groove_name, guide_name in (
        ("battery_rail_groove_0", "rail_guide_0"),
        ("battery_rail_groove_1", "rail_guide_1"),
    ):
        ctx.expect_overlap(
            battery_pack,
            body,
            axes="x",
            elem_a=groove_name,
            elem_b=guide_name,
            min_overlap=0.030,
            name=f"{groove_name} engaged with {guide_name} along rail axis",
        )
    ctx.expect_within(
        battery_pack,
        body,
        axes="y",
        elem_a="battery_rail_groove_0",
        elem_b="rail_guide_0",
        margin=0.005,
        name="battery pack rail groove 0 centered on body rail guide 0",
    )
    ctx.expect_within(
        battery_pack,
        body,
        axes="y",
        elem_a="battery_rail_groove_1",
        elem_b="rail_guide_1",
        margin=0.005,
        name="battery pack rail groove 1 centered on body rail guide 1",
    )

    # Battery pack sits below the grip at the rear, centered on the body.
    batt_aabb = ctx.part_world_aabb(battery_pack)
    ctx.check(
        "battery pack is mounted at the rear underside of the grip",
        batt_aabb is not None
        and batt_aabb[0][0] < -0.15
        and batt_aabb[1][2] < 0.050,
        details=f"batt_aabb={batt_aabb}",
    )
    ctx.expect_within(
        battery_pack,
        body,
        axes="y",
        elem_a="battery_shell",
        elem_b="motor_housing",
        margin=0.010,
        name="battery shell stays within housing width",
    )

    # --- Controls seated on the housing -------------------------------------
    ctx.expect_contact(
        power_switch,
        body,
        elem_a="switch_base",
        elem_b="motor_housing",
        name="slide switch is seated on the barrel top",
    )
    ctx.expect_within(
        power_switch,
        body,
        axes="x",
        outer_elem="motor_housing",
        margin=0.001,
        name="slide switch stays on the motor housing",
    )
    switch_aabb = ctx.part_world_aabb(power_switch)
    ctx.check(
        "slide switch rides on top of the housing",
        switch_aabb is not None and switch_aabb[0][2] >= 0.090,
        details=f"switch_aabb={switch_aabb}",
    )
    ctx.expect_contact(
        spindle_lock,
        body,
        elem_a="lock_stem",
        elem_b="gear_head",
        name="spindle-lock button is seated in the gear head",
    )

    # --- Articulation behavior ----------------------------------------------
    ctx.check(
        "spindle joint is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    slide_limits = slide.motion_limits
    ctx.check(
        "switch travel is 12 mm along the motor axis",
        slide_limits is not None
        and slide_limits.lower is not None
        and slide_limits.upper is not None
        and abs((slide_limits.upper - slide_limits.lower) - 0.012) < 1e-9,
        details=f"limits=({slide_limits.lower},{slide_limits.upper})",
    )
    press_limits = press.motion_limits
    ctx.check(
        "lock button press depth is 4 mm",
        press_limits is not None
        and press_limits.lower is not None
        and press_limits.upper is not None
        and abs((press_limits.upper - press_limits.lower) - 0.004) < 1e-9,
        details=f"limits=({press_limits.lower},{press_limits.upper})",
    )

    label0 = ctx.part_element_world_aabb(spindle_disc, elem="disc_label")
    with ctx.pose({spin: math.pi}):
        label1 = ctx.part_element_world_aabb(spindle_disc, elem="disc_label")
    if label0 is not None and label1 is not None:
        cx0 = 0.5 * (label0[0][0] + label0[1][0])
        cx1 = 0.5 * (label1[0][0] + label1[1][0])
        ctx.check(
            "disc assembly spins about the vertical spindle axis",
            cx0 > 0.02 and cx1 < -0.02,
            details=f"cx0={cx0:.4f}, cx1={cx1:.4f}",
        )
    else:
        ctx.fail("disc label element resolves", "disc_label AABB unavailable")

    off_pos = ctx.part_world_position(power_switch)
    with ctx.pose({slide: 0.012}):
        on_pos = ctx.part_world_position(power_switch)
    ctx.check(
        "switch slides forward from OFF to ON",
        off_pos is not None
        and on_pos is not None
        and abs((on_pos[0] - off_pos[0]) - 0.012) < 1e-6,
        details=f"off={off_pos}, on={on_pos}",
    )

    rest_lock = ctx.part_world_position(spindle_lock)
    with ctx.pose({press: 0.004}):
        pressed_lock = ctx.part_world_position(spindle_lock)
    ctx.check(
        "lock button presses inward along the spindle axis",
        rest_lock is not None
        and pressed_lock is not None
        and abs((rest_lock[2] - pressed_lock[2]) - 0.004) < 1e-6,
        details=f"rest={rest_lock}, pressed={pressed_lock}",
    )

    # --- Battery release articulation --------------------------------------
    batt_release_limits = batt_release.motion_limits
    ctx.check(
        "battery_release is a prismatic joint",
        batt_release.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={batt_release.articulation_type}",
    )
    ctx.check(
        "battery release travel is ~55 mm",
        batt_release_limits is not None
        and batt_release_limits.lower is not None
        and batt_release_limits.upper is not None
        and abs((batt_release_limits.upper - batt_release_limits.lower) - 0.055) < 1e-6,
        details=f"limits=({batt_release_limits.lower},{batt_release_limits.upper})",
    )

    seated_pos = ctx.part_world_position(battery_pack)
    with ctx.pose({batt_release: 0.055}):
        released_pos = ctx.part_world_position(battery_pack)
    ctx.check(
        "battery pack slides backward (-X) to release",
        seated_pos is not None
        and released_pos is not None
        and released_pos[0] < seated_pos[0] - 0.040,
        details=f"seated={seated_pos}, released={released_pos}",
    )

    return ctx.report()


object_model = build_object_model()
