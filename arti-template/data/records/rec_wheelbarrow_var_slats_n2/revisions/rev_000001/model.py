from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


AXLE_PIVOT = (-0.760, 0.0, 0.250)

# Variant: shallower tray with 2 slat rows per wall (parent had 3).
SIDE_SLAT_ROWS = 2
SLAT_Z_POSITIONS = (0.565, 0.715)[:SIDE_SLAT_ROWS]
TOP_LIP_Z = 0.815
POST_CENTER_Z = 0.615
POST_HEIGHT = 0.40


def _origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _body_origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(
        xyz=(x - AXLE_PIVOT[0], y - AXLE_PIVOT[1], z - AXLE_PIVOT[2]),
        rpy=rpy,
    )


def _body_point(point):
    return tuple(point[i] - AXLE_PIVOT[i] for i in range(3))


def _beam_xz(part, name: str, p0, p1, section_y: float, section_z: float, material) -> None:
    """Add a rectangular beam whose long axis follows a line in the XZ plane."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.sqrt(dx * dx + dz * dz)
    pitch = -math.atan2(dz, dx)
    part.visual(
        Box((length, section_y, section_z)),
        origin=_origin((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5, (0.0, pitch, 0.0)),
        material=material,
        name=name,
    )


def _body_beam_xz(part, name: str, p0, p1, section_y: float, section_z: float, material) -> None:
    _beam_xz(part, name, _body_point(p0), _body_point(p1), section_y, section_z, material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_single_wheelbarrow_002",
        meta={
            "category": "Agricultural",
            "small_class": "Single-Wheelbarrow",
            "visual_source": "picture/Agricultural/Single-Wheelbarrow/002.png",
        },
    )

    sun_worn_wood = Material("sun_worn_wood", rgba=(0.73, 0.48, 0.22, 1.0))
    end_grain = Material("darker_end_grain", rgba=(0.47, 0.28, 0.12, 1.0))
    dark_grain = Material("raised_dark_wood_grain", rgba=(0.18, 0.11, 0.055, 1.0))
    black_steel = Material("blackened_steel", rgba=(0.015, 0.017, 0.018, 1.0))
    dull_bolt = Material("dull_bolt_heads", rgba=(0.06, 0.065, 0.07, 1.0))
    rubber = Material("worn_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    grip_rubber = Material("maroon_handle_grips", rgba=(0.47, 0.08, 0.16, 1.0))
    wheel_wood = Material("wheel_wood", rgba=(0.66, 0.42, 0.18, 1.0))

    axle_pivot = model.part("wheel_axle_pivot")
    body = model.part("barrow_body")

    # Deep open wooden tray: slatted bottom, slatted sides, upright posts, and a
    # proud lip rather than a closed solid box.  All members touch or overlap
    # their neighboring posts/rails to read as one bolted wooden subassembly.
    tray_x = 0.15
    tray_len = 1.08
    for i, y in enumerate((-0.24, -0.12, 0.0, 0.12, 0.24)):
        if i == 2:
            body.visual(
                Box((tray_len, 0.075, 0.035)),
                origin=_body_origin(tray_x, y, 0.425),
                material=sun_worn_wood,
                name="floor_plank_2",
            )
        else:
            body.visual(
                Box((tray_len, 0.075, 0.035)),
                origin=_body_origin(tray_x, y, 0.425),
                material=sun_worn_wood,
                name=f"floor_plank_{i}",
            )
        body.visual(
            Box((0.78, 0.010, 0.004)),
            origin=_body_origin(tray_x + 0.03, y + 0.023, 0.445),
            material=dark_grain,
            name=f"floor_grain_{i}",
        )

    # Under-floor and end cross ties bind the bottom planks into one tray floor.
    body.visual(
        Box((0.070, 0.72, 0.060)),
        origin=_body_origin(-0.42, 0.0, 0.425),
        material=sun_worn_wood,
        name="front_floor_tie",
    )
    for name, x in (("rear_floor_tie", 0.72), ("middle_floor_tie", 0.15)):
        body.visual(Box((0.070, 0.72, 0.060)), origin=_body_origin(x, 0.0, 0.425), material=sun_worn_wood, name=name)

    # Side wall rails and upper lip (variant: 2 slat rows per side).
    for side, y in (("side_0", -0.36), ("side_1", 0.36)):
        for row, z in enumerate(SLAT_Z_POSITIONS):
            body.visual(
                Box((1.12, 0.052, 0.075)),
                origin=_body_origin(0.15, y, z),
                material=sun_worn_wood,
                name=f"{side}_slat_{row}",
            )
            body.visual(
                Box((0.80, 0.006, 0.006)),
                origin=_body_origin(0.11, y + (0.030 if y > 0 else -0.030), z + 0.010),
                material=dark_grain,
                name=f"{side}_grain_{row}",
            )
        if side == "side_0":
            body.visual(
                Box((1.18, 0.072, 0.060)),
                origin=_body_origin(0.15, y, TOP_LIP_Z),
                material=sun_worn_wood,
                name="side_0_top_lip",
            )
        else:
            body.visual(
                Box((1.18, 0.072, 0.060)),
                origin=_body_origin(0.15, y, TOP_LIP_Z),
                material=sun_worn_wood,
                name=f"{side}_top_lip",
            )

    # Front and rear boards close the tray enough to form a deep wheelbarrow tub.
    # Variant: 2 slat rows per end (matching side walls).
    for end_name, x, width in (("front", -0.43, 0.72), ("rear", 0.73, 0.66)):
        for row, z in enumerate(SLAT_Z_POSITIONS):
            body.visual(
                Box((0.070, width, 0.075)),
                origin=_body_origin(x, 0.0, z),
                material=sun_worn_wood,
                name=f"{end_name}_slat_{row}",
            )
        body.visual(
            Box((0.082, width + 0.06, 0.060)),
            origin=_body_origin(x, 0.0, TOP_LIP_Z),
            material=sun_worn_wood,
            name=f"{end_name}_top_lip",
        )

    # Four substantial corner posts, plus intermediate uprights seen in the
    # reference, visibly tie the slatted walls to the bottom frame.
    # Variant: shorter posts for the shallower 2-row tray.
    post_positions = [
        (-0.43, -0.36),
        (-0.43, 0.36),
        (0.73, -0.36),
        (0.73, 0.36),
        (0.15, -0.36),
        (0.15, 0.36),
    ]
    for idx, (x, y) in enumerate(post_positions):
        body.visual(
            Box((0.075, 0.075, POST_HEIGHT)),
            origin=_body_origin(x, y, POST_CENTER_Z),
            material=sun_worn_wood,
            name=f"upright_post_{idx}",
        )

    # Long handles are the same continuous wooden runners that carry the tray
    # and lead down to the front axle yoke.
    for side, y in (("handle_0", -0.285), ("handle_1", 0.285)):
        if side == "handle_0":
            _body_beam_xz(body, "handle_0_runner", (-0.78, y, 0.250), (1.34, y, 0.790), 0.070, 0.070, sun_worn_wood)
        else:
            _body_beam_xz(
                body,
                f"{side}_runner",
                (-0.78, y, 0.250),
                (1.34, y, 0.790),
                0.070,
                0.070,
                sun_worn_wood,
            )
        _body_beam_xz(
            body,
            f"{side}_grip",
            (1.17, y, 0.745),
            (1.38, y, 0.800),
            0.088,
            0.088,
            grip_rubber,
        )
        # Raised rubber wrap bands, like the reddish bands in the reference.
        for band, xb in enumerate((1.19, 1.25, 1.31)):
            body.visual(
                Box((0.026, 0.100, 0.102)),
                origin=_body_origin(xb, y, 0.765 + (xb - 1.17) * 0.25, (0.0, -0.25, 0.0)),
                material=grip_rubber,
                name=f"{side}_grip_band_{band}",
            )

    # Rear support legs and foot crossbar hold the parked wheelbarrow off the
    # ground.  They physically touch the underside runners.
    for side, y in (("leg_0", -0.285), ("leg_1", 0.285)):
        _body_beam_xz(body, f"{side}_support_leg", (0.55, y, 0.420), (0.68, y, 0.060), 0.070, 0.070, sun_worn_wood)
        body.visual(
            Box((0.145, 0.085, 0.045)),
            origin=_body_origin(0.70, y, 0.042),
            material=end_grain,
            name=f"{side}_foot_pad",
        )
        _body_beam_xz(body, f"{side}_rear_brace", (0.48, y, 0.460), (0.93, y, 0.640), 0.045, 0.050, sun_worn_wood)

    body.visual(
        Box((0.080, 0.68, 0.045)),
        origin=_body_origin(0.70, 0.0, 0.060),
        material=sun_worn_wood,
        name="rear_foot_crossbar",
    )

    # Front yoke, axle plates, and metal axle.  The central axle pin intentionally
    # passes through the rotating hub, while the visible outer stubs/caps make the
    # captured support hardware legible.
    for side, y in (("fork_0", -0.075), ("fork_1", 0.075)):
        if side == "fork_0":
            body.visual(
                Box((0.125, 0.020, 0.230)),
                origin=_body_origin(-0.760, y, 0.300),
                material=black_steel,
                name="fork_0_axle_plate",
            )
        else:
            body.visual(
                Box((0.125, 0.020, 0.230)),
                origin=_body_origin(-0.760, y, 0.300),
                material=black_steel,
                name="fork_1_axle_plate",
            )
        _body_beam_xz(body, f"{side}_yoke_brace", (-0.760, y, 0.335), (-0.425, y, 0.535), 0.028, 0.045, black_steel)

    body.visual(
        Cylinder(0.017, 0.180),
        origin=_body_origin(-0.760, 0.0, 0.250, (math.pi / 2.0, 0.0, 0.0)),
        material=black_steel,
        name="axle_pin",
    )

    for side, y in (("axle_stub_0", -0.150), ("axle_stub_1", 0.150)):
        body.visual(
            Cylinder(0.020, 0.180),
            origin=_body_origin(-0.760, y, 0.250, (math.pi / 2.0, 0.0, 0.0)),
            material=black_steel,
            name=side,
        )
        body.visual(
            Cylinder(0.032, 0.010),
            origin=_body_origin(-0.760, y * 1.58, 0.250, (math.pi / 2.0, 0.0, 0.0)),
            material=dull_bolt,
            name=f"{side}_end_cap",
        )

    # Diagonal wooden struts from the runners to the front tray posts.
    for side, y in (("brace_0", -0.285), ("brace_1", 0.285)):
        _body_beam_xz(body, f"{side}_front_strut", (-0.70, y, 0.300), (-0.39, y, 0.535), 0.060, 0.060, sun_worn_wood)
        _body_beam_xz(body, f"{side}_side_strut", (-0.10, y, 0.460), (0.36, y, 0.610), 0.055, 0.055, sun_worn_wood)

    # Bolted construction: small raised dark washers on the side boards, posts,
    # fork plates, and braces.  Variant: 2 bolt rows per wall matching 2 slats.
    bolt_specs = []
    for y, suffix in ((-0.400, "outer_0"), (0.400, "outer_1")):
        for x in (-0.43, 0.15, 0.73):
            for z in (0.57, 0.72):
                bolt_specs.append((x, y, z, suffix))
        for x, z in ((-0.26, 0.57), (0.40, 0.72)):
            bolt_specs.append((x, y * 0.84, z, suffix))
    for idx, (x, y, z, suffix) in enumerate(bolt_specs):
        body.visual(
            Cylinder(0.012, 0.055),
            origin=_body_origin(x, y, z, (math.pi / 2.0, 0.0, 0.0)),
            material=dull_bolt,
            name=f"bolt_{suffix}_{idx}",
        )

    # The single front wheel is a separate rotating part.  Mesh helpers create
    # actual tread, sidewalls, rim, hub, bore, and spokes rather than a plain disc.
    wheel = model.part("front_wheel")
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.235,
            0.092,
            inner_radius=0.170,
            carcass=TireCarcass(belt_width_ratio=0.66, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.010, count=20, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.004),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.010, radius=0.004),
        ),
        "single_wheelbarrow_tire",
    )
    wheel.visual(tire_mesh, material=rubber, name="block_tread_tire")

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.172,
            0.078,
            rim=WheelRim(inner_radius=0.105, flange_height=0.011, flange_thickness=0.005, bead_seat_depth=0.004),
            hub=WheelHub(
                radius=0.043,
                width=0.078,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.060, hole_diameter=0.006),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=8, thickness=0.010, window_radius=0.020),
            bore=WheelBore(style="round", diameter=0.052),
        ),
        "single_wheelbarrow_spoked_wheel",
    )
    wheel.visual(wheel_mesh, material=wheel_wood, name="wood_spoked_rim")
    wheel.visual(
        Cylinder(0.048, 0.118),
        origin=_origin(0.0, 0.0, 0.0, (0.0, math.pi / 2.0, 0.0)),
        material=black_steel,
        name="dark_hub_shell",
    )
    # A small raised marker makes the wheel rotation visually and testably clear.
    wheel.visual(
        Sphere(0.018),
        origin=_origin(0.049, 0.0, 0.235),
        material=grip_rubber,
        name="tread_marker",
    )

    model.articulation(
        "axle_pivot_to_barrow_body",
        ArticulationType.REVOLUTE,
        parent=axle_pivot,
        child=body,
        origin=_origin(*AXLE_PIVOT),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.0, lower=0.0, upper=1.05),
    )

    model.articulation(
        "axle_pivot_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle_pivot,
        child=wheel,
        # Rotate the joint frame so the wheel helper's local X axle becomes the
        # wheelbarrow's world Y axle.
        origin=_origin(-0.760, 0.0, 0.250, (0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("barrow_body")
    wheel = object_model.get_part("front_wheel")
    body_joint = object_model.get_articulation("axle_pivot_to_barrow_body")
    wheel_spin = object_model.get_articulation("axle_pivot_to_front_wheel")

    ctx.check(
        "asset is classified as Single-Wheelbarrow",
        object_model.meta.get("category") == "Agricultural"
        and object_model.meta.get("small_class") == "Single-Wheelbarrow",
        details=f"meta={object_model.meta}",
    )
    ctx.check(
        "axle pivot carries body tip and wheel spin joints",
        len(object_model.articulations) == 2
        and body_joint.parent == "wheel_axle_pivot"
        and body_joint.child == "barrow_body"
        and wheel_spin.parent == "wheel_axle_pivot"
        and wheel_spin.child == "front_wheel",
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )
    ctx.check(
        "front wheel joint is continuous revolute rolling axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "barrow body tips forward around the wheel axle",
        body_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(body_joint.axis) == (0.0, -1.0, 0.0)
        and body_joint.motion_limits.lower == 0.0
        and body_joint.motion_limits.upper >= 1.0,
        details=f"type={body_joint.articulation_type}, axis={body_joint.axis}, limits={body_joint.motion_limits}",
    )

    ctx.allow_overlap(
        body,
        wheel,
        elem_a="axle_pin",
        elem_b="dark_hub_shell",
        reason="The fixed axle pin is intentionally captured inside the rotating wheel hub shell.",
    )

    # The rotating wheel is captured between axle plates with visible clearance.
    ctx.expect_overlap(
        body,
        wheel,
        axes="xz",
        elem_a="axle_pin",
        elem_b="dark_hub_shell",
        min_overlap=0.025,
        name="fixed axle pin passes through rotating hub",
    )
    ctx.expect_gap(
        body,
        wheel,
        axis="y",
        positive_elem="fork_1_axle_plate",
        negative_elem="block_tread_tire",
        min_gap=0.004,
        max_gap=0.050,
        name="right fork plate clears tire sidewall",
    )
    ctx.expect_gap(
        wheel,
        body,
        axis="y",
        positive_elem="block_tread_tire",
        negative_elem="fork_0_axle_plate",
        min_gap=0.004,
        max_gap=0.050,
        name="left fork plate clears tire sidewall",
    )
    ctx.expect_overlap(
        body,
        wheel,
        axes="z",
        elem_a="axle_pin",
        elem_b="dark_hub_shell",
        min_overlap=0.020,
        name="axle pin shares the hub height",
    )
    ctx.expect_gap(
        body,
        wheel,
        axis="x",
        elem_a="front_floor_tie",
        elem_b="block_tread_tire",
        min_gap=0.040,
        max_gap=0.160,
        name="deep tray front tie sits just behind the single wheel",
    )

    # Variant: shallower tray with 2 slat rows per side (parent had 3).
    side_0_slats = [v for v in body.visuals if v.name.startswith("side_0_slat_")]
    side_1_slats = [v for v in body.visuals if v.name.startswith("side_1_slat_")]
    front_slats = [v for v in body.visuals if v.name.startswith("front_slat_")]
    rear_slats = [v for v in body.visuals if v.name.startswith("rear_slat_")]
    ctx.check(
        "variant side walls have exactly 2 slat rows",
        len(side_0_slats) == SIDE_SLAT_ROWS and len(side_1_slats) == SIDE_SLAT_ROWS,
        details=f"side_0={len(side_0_slats)}, side_1={len(side_1_slats)}",
    )
    ctx.check(
        "variant front and rear walls have exactly 2 slat rows",
        len(front_slats) == SIDE_SLAT_ROWS and len(rear_slats) == SIDE_SLAT_ROWS,
        details=f"front={len(front_slats)}, rear={len(rear_slats)}",
    )

    # Prompt-critical parked geometry: rear legs touch the ground plane area
    # below the tray, while the handles extend behind the tray.
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem="side_0_top_lip",
        negative_elem="floor_plank_2",
        min_gap=0.28,
        max_gap=0.42,
        name="shallower tray lip rises above the slatted floor",
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem="floor_plank_2",
        negative_elem="rear_foot_crossbar",
        min_gap=0.25,
        name="tray floor is elevated by rear support legs",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="x",
        elem_a="handle_0_runner",
        elem_b="side_0_top_lip",
        min_overlap=0.25,
        name="handles are continuous runners under the tray",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    at_rest = _aabb_center(ctx.part_element_world_aabb(wheel, elem="tread_marker"))
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        rotated = _aabb_center(ctx.part_element_world_aabb(wheel, elem="tread_marker"))
    ctx.check(
        "wheel rotation moves the raised tire marker around the axle",
        at_rest is not None
        and rotated is not None
        and at_rest[2] > rotated[2] + 0.12
        and abs(at_rest[1] - rotated[1]) < 0.08,
        details=f"rest={at_rest}, rotated={rotated}",
    )

    handle_rest = _aabb_center(ctx.part_element_world_aabb(body, elem="handle_0_grip"))
    with ctx.pose({body_joint: 0.65}):
        handle_tipped = _aabb_center(ctx.part_element_world_aabb(body, elem="handle_0_grip"))
    ctx.check(
        "body tip joint swings the whole barrow body forward around the wheel",
        handle_rest is not None
        and handle_tipped is not None
        and handle_tipped[2] > handle_rest[2] + 0.55
        and handle_tipped[0] < handle_rest[0] - 0.10,
        details=f"rest={handle_rest}, tipped={handle_tipped}",
    )

    return ctx.report()


object_model = build_object_model()
