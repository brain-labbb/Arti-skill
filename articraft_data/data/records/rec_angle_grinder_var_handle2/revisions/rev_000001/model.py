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
    tube_from_spline_points,
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
HANDLE_BOSS_COUNT = 2  # symmetric pair, one per lateral flank
BOSS_RADIUS = 0.011
BOSS_LENGTH = 0.016
BOSS_SOCKET_RADIUS = 0.0048
BOSS_SOCKET_LENGTH = 0.003
FLANK_Y = 0.032  # gear-head lateral flank surface at the boss X station


def _add_handle_boss_visuals(part, side: int, yellow_mat, black_mat) -> None:
    """Add threaded side-handle boss visuals to a handle_boss_i part.

    The part origin sits at the gear-head flank contact surface.  ``side``
    is +1 for the +Y flank or −1 for the −Y flank; the boss protrudes
    outward along ±Y from the origin.
    """
    y = float(side)
    rot_x = y * math.pi / 2.0  # rotate default Z-axis cylinder toward ±Y

    # Yellow boss cylinder protruding from the flank.
    part.visual(
        Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
        origin=Origin(
            xyz=(0.0, y * BOSS_LENGTH / 2.0, 0.0),
            rpy=(rot_x, 0.0, 0.0),
        ),
        material=yellow_mat,
        name="boss_cylinder",
    )

    # Recessed black thread socket on the outer face.
    socket_offset = BOSS_LENGTH - BOSS_SOCKET_LENGTH / 2.0 + 0.001
    part.visual(
        Cylinder(radius=BOSS_SOCKET_RADIUS, length=BOSS_SOCKET_LENGTH),
        origin=Origin(
            xyz=(0.0, y * socket_offset, 0.0),
            rpy=(rot_x, 0.0, 0.0),
        ),
        material=black_mat,
        name="boss_socket",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dewalt_angle_grinder")

    yellow = model.material("housing_yellow", rgba=(0.95, 0.72, 0.06, 1.0))
    black = model.material("trim_black", rgba=(0.08, 0.08, 0.08, 1.0))
    rubber = model.material("cord_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel_grey", rgba=(0.60, 0.61, 0.63, 1.0))
    fiber = model.material("fiber_disc_brown", rgba=(0.46, 0.38, 0.29, 1.0))

    # ------------------------------------------------------------------
    # body (root): motor housing, rear grip taper, gear head, collar,
    # side-handle boss, vent slots, brand label, cord boot and power cord.
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

    # ------------------------------------------------------------------
    # handle_boss_i: threaded side-handle mounting bosses, one on each
    # lateral flank of the gear head.  Shared geometry helper + FIXED
    # joints anchor each boss at the flank contact surface.
    # ------------------------------------------------------------------
    for i in range(HANDLE_BOSS_COUNT):
        side = 1 if i == 0 else -1
        boss_part = model.part(f"handle_boss_{i}")
        _add_handle_boss_visuals(boss_part, side, yellow, black)

        model.articulation(
            f"body_to_handle_boss_{i}",
            ArticulationType.FIXED,
            parent=body,
            child=boss_part,
            origin=Origin(xyz=(-0.005, side * FLANK_Y, 0.062)),
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

    # Black rubber strain-relief boot and drooping power cord at the rear.
    boot_profile = [
        (0.0, 0.0),
        (0.0095, 0.0),
        (0.0080, 0.018),
        (0.0062, 0.034),
        (0.0, 0.034),
    ]
    boot_geom = LatheGeometry(boot_profile, segments=32)
    boot_geom.rotate_y(-math.pi / 2.0)
    boot_geom.translate(-0.243, 0.0, MOTOR_AXIS_Z)
    body.visual(
        mesh_from_geometry(boot_geom, "cord_boot"),
        material=rubber,
        name="cord_boot",
    )

    cord_pts = [
        (-0.270, 0.000, 0.062),
        (-0.300, 0.002, 0.057),
        (-0.330, 0.006, 0.044),
        (-0.352, 0.010, 0.028),
    ]
    cord_geom = tube_from_spline_points(cord_pts, radius=0.0042)
    body.visual(
        mesh_from_geometry(cord_geom, "power_cord"),
        material=rubber,
        name="power_cord",
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

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spindle_disc = object_model.get_part("spindle_disc")
    power_switch = object_model.get_part("power_switch")
    spindle_lock = object_model.get_part("spindle_lock")
    spin = object_model.get_articulation("spindle_spin")
    slide = object_model.get_articulation("switch_slide")
    press = object_model.get_articulation("lock_press")

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

    # --- Side-handle mounting bosses (symmetric pair) ---------------------
    boss_parts = [
        object_model.get_part(f"handle_boss_{i}")
        for i in range(HANDLE_BOSS_COUNT)
    ]
    ctx.check(
        "two symmetric side-handle bosses exist",
        len(boss_parts) == HANDLE_BOSS_COUNT,
        details=f"count={len(boss_parts)}",
    )

    boss_aabbs = [ctx.part_world_aabb(bp) for bp in boss_parts]
    if all(a is not None for a in boss_aabbs):
        # Boss 0 on +Y flank, boss 1 on −Y flank.
        ctx.check(
            "handle_boss_0 protrudes from the +Y gear-head flank",
            boss_aabbs[0][1][1] > 0.040,
            details=f"max_y={boss_aabbs[0][1][1]:.4f}",
        )
        ctx.check(
            "handle_boss_1 protrudes from the -Y gear-head flank",
            boss_aabbs[1][0][1] < -0.040,
            details=f"min_y={boss_aabbs[1][0][1]:.4f}",
        )
        # Symmetric about Y=0.
        y_span_0 = boss_aabbs[0][1][1] - boss_aabbs[0][0][1]
        y_span_1 = boss_aabbs[1][1][1] - boss_aabbs[1][0][1]
        ctx.check(
            "handle bosses are symmetric in lateral extent",
            abs(y_span_0 - y_span_1) < 0.002,
            details=f"span_0={y_span_0:.4f}, span_1={y_span_1:.4f}",
        )
    else:
        ctx.fail("boss AABBs resolve", "one or more handle_boss_i AABBs unavailable")

    # FIXED joints connect each boss to the body.
    for i in range(HANDLE_BOSS_COUNT):
        joint_name = f"body_to_handle_boss_{i}"
        art = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is a FIXED joint",
            art.articulation_type == ArticulationType.FIXED,
            details=f"type={art.articulation_type}",
        )

    cord_aabb = ctx.part_element_world_aabb(body, elem="power_cord")
    ctx.check(
        "power cord exits the rear of the grip",
        cord_aabb is not None and cord_aabb[0][0] <= -0.30,
        details=f"cord_aabb={cord_aabb}",
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

    return ctx.report()


object_model = build_object_model()
