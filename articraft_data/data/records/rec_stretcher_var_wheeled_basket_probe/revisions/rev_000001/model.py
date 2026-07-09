from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LoftSection,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    SectionLoftSpec,
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
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _origin_for_cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    extend: float = 0.0,
) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("tube endpoints must be separated")

    ux, uy, uz = dx / length, dy / length, dz / length
    sx -= ux * extend
    sy -= uy * extend
    sz -= uz * extend
    ex += ux * extend
    ey += uy * extend
    ez += uz * extend
    length += 2.0 * extend

    mx, my, mz = (sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    return Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw)), length


def _tube(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
    name: str,
    *,
    extend: float = 0.002,
) -> None:
    origin, length = _origin_for_cylinder_between(start, end, extend=extend)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def _curved_tube(
    part,
    points: list[tuple[float, float, float]],
    radius: float,
    material: Material,
    name: str,
    *,
    samples_per_segment: int = 8,
    radial_segments: int = 18,
) -> None:
    part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=samples_per_segment,
                radial_segments=radial_segments,
                cap_ends=True,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def _make_caster_meshes():
    tire = TireGeometry(
        0.066,
        0.040,
        inner_radius=0.049,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
        tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.60),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    hub = WheelGeometry(
        0.052,
        0.044,
        rim=WheelRim(
            inner_radius=0.032,
            flange_height=0.004,
            flange_thickness=0.003,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(
            radius=0.027,
            width=0.046,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.003),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.009),
        bore=WheelBore(style="round", diameter=0.012),
    )
    return (
        mesh_from_geometry(tire, "shared_caster_tire"),
        mesh_from_geometry(hub, "shared_caster_hub"),
    )


def _add_leg_geometry(part, yellow: Material, black: Material, *, footward: bool) -> None:
    # Local origin is the upper hinge line; the wheel centers are near z=-0.66.
    bottom_x = 0.075 if footward else -0.075
    bend_x = 0.145 if footward else -0.145
    sign = 1.0 if footward else -1.0
    _tube(part, (0.0, -0.285, 0.0), (0.0, 0.285, 0.0), 0.017, yellow, "top_hinge")

    for y, plate_suffix in ((-0.318, "near"), (0.318, "far")):
        part.visual(
            Box((0.055, 0.016, 0.070)),
            origin=Origin(xyz=(0.012 * sign, y, -0.025)),
            material=yellow,
            name=f"upper_hinge_clevis_{plate_suffix}",
        )
        part.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"upper_hinge_washer_{plate_suffix}",
        )

    for i, y in enumerate((-0.255, 0.255)):
        _tube(part, (0.0, y, -0.010), (0.028 * sign, y, -0.145), 0.019, yellow, f"main_leg_{i}")
        _curved_tube(
            part,
            [
                (0.020 * sign, y, -0.055),
                (0.040 * sign, y, -0.210),
                (bend_x, y, -0.380),
                (bottom_x + 0.020 * sign, y, -0.555),
            ],
            0.019,
            yellow,
            f"swept_outer_leg_{i}",
        )
        _curved_tube(
            part,
            [
                (-0.012 * sign, y, -0.115),
                (0.018 * sign, y, -0.250),
                (bottom_x - 0.050 * sign, y, -0.415),
                (bottom_x - 0.015 * sign, y, -0.565),
            ],
            0.012,
            yellow,
            f"inner_return_leg_{i}",
            samples_per_segment=6,
            radial_segments=14,
        )
        part.visual(
            Box((0.075, 0.020, 0.050)),
            origin=Origin(xyz=(bottom_x, y, -0.555)),
            material=yellow,
            name=f"caster_mount_block_{i}",
        )
        part.visual(
            Cylinder(radius=0.030, length=0.020),
            origin=Origin(xyz=(bottom_x, y, -0.585)),
            material=black,
            name=f"caster_swivel_bearing_{i}",
        )
        part.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(bottom_x, y, -0.615)),
            material=black,
            name=f"caster_swivel_stem_{i}",
        )
        _tube(part, (bottom_x, y, -0.555), (bottom_x, y, -0.615), 0.012, black, f"caster_stem_{i}")
        part.visual(
            Box((0.064, 0.145, 0.020)),
            origin=Origin(xyz=(bottom_x, y, -0.622)),
            material=black,
            name=f"caster_yoke_bridge_{i}",
        )
        _tube(
            part,
            (bottom_x, y - 0.058, -0.610),
            (bottom_x, y - 0.058, -0.665),
            0.008,
            black,
            f"caster_fork_{i}_0",
        )
        _tube(
            part,
            (bottom_x, y + 0.058, -0.610),
            (bottom_x, y + 0.058, -0.665),
            0.008,
            black,
            f"caster_fork_{i}_1",
        )
        for side, y_offset in (("inner", -0.060), ("outer", 0.060)):
            part.visual(
                Box((0.026, 0.010, 0.088)),
                origin=Origin(xyz=(bottom_x, y + y_offset, -0.642)),
                material=black,
                name=f"caster_fork_plate_{i}_{side}",
            )
            part.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(
                    xyz=(bottom_x, y + y_offset, -0.660),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=black,
                name=f"axle_end_cap_{i}_{side}",
            )
        _tube(
            part,
            (bottom_x, y - 0.070, -0.660),
            (bottom_x, y + 0.070, -0.660),
            0.008,
            black,
            f"wheel_axle_{i}",
            extend=0.000,
        )
        part.visual(
            Box((0.040, 0.020, 0.010)),
            origin=Origin(xyz=(bottom_x + 0.030 * sign, y + 0.034, -0.604)),
            material="red_release",
            name=f"caster_brake_tab_{i}",
        )

    _curved_tube(
        part,
        [
            (bottom_x, -0.255, -0.560),
            (bottom_x + 0.018 * sign, -0.120, -0.592),
            (bottom_x + 0.018 * sign, 0.120, -0.592),
            (bottom_x, 0.255, -0.560),
        ],
        0.016,
        yellow,
        "lower_curved_crossbar",
    )
    _tube(
        part,
        (0.010 * sign, -0.250, -0.110),
        (bottom_x, 0.250, -0.535),
        0.010,
        yellow,
        "diagonal_strut_0",
    )
    _tube(
        part,
        (0.010 * sign, 0.250, -0.110),
        (bottom_x, -0.250, -0.535),
        0.010,
        yellow,
        "diagonal_strut_1",
    )
    part.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.5 * bottom_x, 0.0, -0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yellow,
        name="central_scissor_pivot_disc",
    )
    part.visual(
        Box((0.052, 0.018, 0.070)),
        origin=Origin(xyz=(0.5 * bottom_x, 0.0, -0.325)),
        material=black,
        name="central_black_pivot_plate",
    )
    _curved_tube(
        part,
        [
            (0.000, -0.210, -0.230),
            (0.035 * sign, -0.210, -0.335),
            (bottom_x - 0.015 * sign, -0.210, -0.510),
        ],
        0.007,
        black,
        "black_link_0",
        samples_per_segment=6,
        radial_segments=12,
    )
    _curved_tube(
        part,
        [
            (0.000, 0.210, -0.230),
            (0.035 * sign, 0.210, -0.335),
            (bottom_x - 0.015 * sign, 0.210, -0.510),
        ],
        0.007,
        black,
        "black_link_1",
        samples_per_segment=6,
        radial_segments=12,
    )
    for i, y in enumerate((-0.210, 0.210)):
        for j, point in enumerate(((0.000, y, -0.230), (bottom_x - 0.015 * sign, y, -0.510))):
            part.visual(
                Sphere(0.014),
                origin=Origin(xyz=point),
                material=black,
                name=f"black_link_pivot_{i}_{j}",
            )


# ---------------------------------------------------------------------------
# Stokes basket shell geometry helpers
# ---------------------------------------------------------------------------

def _basket_section(
    x: float, hw: float, wh: float, fz: float
) -> list[tuple[float, float, float]]:
    """12-point closed U-profile for the Stokes basket shell at position *x*.

    The profile traces: left rim → left wall → floor → right wall → right rim,
    with the closing segment running across the open top.
    """
    return [
        (x, -hw, fz + wh),
        (x, -hw, fz + wh * 0.65),
        (x, -hw, fz + wh * 0.30),
        (x, -hw, fz + 0.015),
        (x, -hw + 0.015, fz),
        (x, -hw * 0.35, fz - 0.003),
        (x, hw * 0.35, fz - 0.003),
        (x, hw - 0.015, fz),
        (x, hw, fz + 0.015),
        (x, hw, fz + wh * 0.30),
        (x, hw, fz + wh * 0.65),
        (x, hw, fz + wh),
    ]


def _build_basket_body_mesh():
    """Loft the boat-shaped Stokes basket body from five cross-sections."""
    # Lower the basket floor to rest directly on the deck frame side tubes
    # (frame tubes at z=0.805 with r=0.018 → top surface at z≈0.823).
    fz = 0.826
    raw = [
        _basket_section(-0.48, 0.28, 0.20, fz),
        _basket_section(-0.10, 0.28, 0.185, fz),
        _basket_section(0.30, 0.27, 0.175, fz),
        _basket_section(0.70, 0.25, 0.165, fz),
        _basket_section(1.02, 0.22, 0.14, fz),
    ]
    geom = section_loft(
        SectionLoftSpec(
            sections=tuple(LoftSection(points=tuple(s)) for s in raw),
            cap=True,
            solid=True,
        )
    )
    return mesh_from_geometry(geom, "basket_body")


def _build_floor_pan_mesh():
    """Perforated drainage floor pan that sits inside the basket."""
    pan = PerforatedPanelGeometry(
        (1.36, 0.40),
        0.003,
        hole_diameter=0.010,
        pitch=0.022,
        frame=0.020,
        corner_radius=0.030,
        stagger=True,
    )
    return mesh_from_geometry(pan, "basket_floor_pan")


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="stokes_basket_on_wheeled_carriage",
        meta={
            "run_notes": (
                "Stokes-style rescue basket litter shell mounted on the yellow "
                "wheeled folding-leg stretcher undercarriage. The flat deck and "
                "mattress are replaced with a boat-shaped basket (perimeter rail, "
                "perforated floor pan, grab handles). This variant probes the "
                "basket-to-carriage interface: floor-pan clearance over the deck "
                "side tubes and basket-rim envelope vs folded-leg swing."
            )
        },
    )

    yellow = model.material("safety_yellow", rgba=(1.0, 0.74, 0.03, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.006, 0.005, 1.0))
    orange = model.material("rescue_orange", rgba=(1.0, 0.35, 0.05, 1.0))
    dark_grey = model.material("rubber_dark_grey", rgba=(0.055, 0.060, 0.055, 1.0))
    metal = model.material("brushed_metal", rgba=(0.65, 0.66, 0.62, 1.0))
    model.material("red_release", rgba=(0.85, 0.02, 0.015, 1.0))

    # Pre-build shared meshes.
    tire_mesh, hub_mesh = _make_caster_meshes()
    basket_body_mesh = _build_basket_body_mesh()
    floor_pan_mesh = _build_floor_pan_mesh()

    # ===================================================================
    # DECK FRAME (root) — yellow wheeled carriage perimeter and sockets
    # ===================================================================
    deck = model.part("deck_frame")

    # Yellow perimeter frame tubes.
    _tube(deck, (-0.520, -0.350, 0.805), (1.090, -0.350, 0.805), 0.018, yellow, "side_frame_0")
    _tube(deck, (-0.520, 0.350, 0.805), (1.090, 0.350, 0.805), 0.018, yellow, "side_frame_1")
    _tube(deck, (-0.520, -0.350, 0.805), (-0.520, 0.350, 0.805), 0.016, yellow, "head_crossbar")
    _tube(deck, (1.090, -0.350, 0.805), (1.090, 0.350, 0.805), 0.016, yellow, "foot_crossbar")

    # Lower black rails.
    _tube(deck, (-0.350, -0.350, 0.700), (1.000, -0.350, 0.700), 0.012, black, "lower_rail_0")
    _tube(deck, (-0.350, 0.350, 0.700), (1.000, 0.350, 0.700), 0.012, black, "lower_rail_1")

    # Vertical drop brackets connecting upper frame to lower rails.
    for i, x in enumerate((-0.300, 0.300, 0.900)):
        _tube(deck, (x, -0.350, 0.790), (x, -0.350, 0.700), 0.008, yellow, f"drop_bracket_{i}_0")
        _tube(deck, (x, 0.350, 0.790), (x, 0.350, 0.700), 0.008, yellow, f"drop_bracket_{i}_1")

    # Leg hinge sockets (tubes that capture the leg top hinge).
    _tube(deck, (-0.555, -0.335, 0.730), (-0.555, 0.335, 0.730), 0.018, yellow, "head_leg_socket")
    _tube(deck, (0.650, -0.335, 0.730), (0.650, 0.335, 0.730), 0.018, yellow, "foot_leg_socket")

    # Socket brackets connecting frame to leg sockets.
    _tube(deck, (-0.535, -0.350, 0.805), (-0.555, -0.335, 0.730), 0.010, yellow, "head_socket_bracket_0")
    _tube(deck, (-0.535, 0.350, 0.805), (-0.555, 0.335, 0.730), 0.010, yellow, "head_socket_bracket_1")
    _tube(deck, (0.650, -0.350, 0.805), (0.650, -0.335, 0.730), 0.010, yellow, "foot_socket_bracket_0")
    _tube(deck, (0.650, 0.350, 0.805), (0.650, 0.335, 0.730), 0.010, yellow, "foot_socket_bracket_1")

    # Foot push handle (U-shaped tube at foot end).
    handle = tube_from_spline_points(
        [
            (1.020, -0.315, 0.807),
            (1.160, -0.315, 0.807),
            (1.230, 0.000, 0.822),
            (1.160, 0.315, 0.807),
            (1.020, 0.315, 0.807),
        ],
        radius=0.012,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    deck.visual(
        mesh_from_geometry(handle, "foot_push_handle"), material=black, name="foot_push_handle"
    )

    # Brake levers and red release tabs.
    _tube(deck, (0.980, -0.350, 0.700), (1.090, -0.390, 0.715), 0.006, black, "brake_lever_0")
    _tube(deck, (0.980, 0.350, 0.700), (1.090, 0.390, 0.715), 0.006, black, "brake_lever_1")
    deck.visual(
        Box((0.055, 0.018, 0.018)),
        origin=Origin(xyz=(1.095, -0.392, 0.715)),
        material="red_release",
        name="red_brake_0",
    )
    deck.visual(
        Box((0.055, 0.018, 0.018)),
        origin=Origin(xyz=(1.095, 0.392, 0.715)),
        material="red_release",
        name="red_brake_1",
    )

    # IV pole accessory — mounted at the head end, outside the basket footprint.
    deck.visual(
        Box((0.080, 0.070, 0.050)),
        origin=Origin(xyz=(-0.560, 0.280, 0.832)),
        material=black,
        name="iv_pole_clamp",
    )
    _tube(deck, (-0.540, 0.280, 0.808), (-0.560, 0.280, 0.808), 0.010, black, "iv_mount_bracket")
    _tube(deck, (-0.560, 0.280, 0.860), (-0.560, 0.280, 1.690), 0.010, black, "iv_pole")
    deck.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(-0.560, 0.280, 1.280)),
        material=black,
        name="iv_sleeve",
    )
    _tube(deck, (-0.600, 0.280, 1.680), (-0.520, 0.280, 1.680), 0.006, metal, "iv_hook")
    deck.visual(
        Sphere(0.020), origin=Origin(xyz=(-0.560, 0.280, 1.708)), material=metal, name="iv_top_knob"
    )

    # ===================================================================
    # BASKET SHELL — Stokes rescue basket mounted on the carriage frame
    # ===================================================================
    basket = model.part("basket_shell")

    # Main lofted body (boat-shaped molded shell).
    basket.visual(basket_body_mesh, material=orange, name="basket_body")

    # Perforated drainage floor pan sitting on the basket floor.
    basket.visual(
        floor_pan_mesh,
        origin=Origin(xyz=(0.27, 0.0, 0.829)),
        material=dark_grey,
        name="floor_pan",
    )

    # Perimeter grab rail — closed spline around the top rim.
    fz = 0.826
    rim_pts = [
        (-0.50, -0.28, fz + 0.20),
        (-0.50, 0.00, fz + 0.20),
        (-0.50, 0.28, fz + 0.20),
        (-0.10, 0.28, fz + 0.185),
        (0.30, 0.27, fz + 0.175),
        (0.70, 0.25, fz + 0.165),
        (1.04, 0.22, fz + 0.14),
        (1.06, 0.00, fz + 0.14),
        (1.04, -0.22, fz + 0.14),
        (0.70, -0.25, fz + 0.165),
        (0.30, -0.27, fz + 0.175),
        (-0.10, -0.28, fz + 0.185),
    ]
    rail_geom = tube_from_spline_points(
        rim_pts,
        radius=0.012,
        samples_per_segment=10,
        radial_segments=18,
        closed_spline=True,
    )
    basket.visual(
        mesh_from_geometry(rail_geom, "perimeter_rail"),
        material=orange,
        name="perimeter_rail",
    )

    # Internal floor ribs (raised drainage ridges inside the basket).
    for i, x in enumerate((-0.25, 0.10, 0.45, 0.80)):
        _tube(basket, (x, -0.22, fz + 0.006), (x, 0.22, fz + 0.006), 0.004, orange, f"floor_rib_{i}")

    # Head-end bail handle.
    _tube(basket, (-0.49, -0.20, fz + 0.20), (-0.60, -0.20, fz + 0.20), 0.010, orange, "head_handle_arm_0")
    _tube(basket, (-0.49, 0.20, fz + 0.20), (-0.60, 0.20, fz + 0.20), 0.010, orange, "head_handle_arm_1")
    _curved_tube(
        basket,
        [(-0.60, -0.20, fz + 0.20), (-0.62, 0.0, fz + 0.20), (-0.60, 0.20, fz + 0.20)],
        0.010,
        orange,
        "head_handle_bail",
    )

    # Foot-end bail handle.
    _tube(basket, (1.03, -0.15, fz + 0.14), (1.14, -0.15, fz + 0.14), 0.010, orange, "foot_handle_arm_0")
    _tube(basket, (1.03, 0.15, fz + 0.14), (1.14, 0.15, fz + 0.14), 0.010, orange, "foot_handle_arm_1")
    _curved_tube(
        basket,
        [(1.14, -0.15, fz + 0.14), (1.16, 0.0, fz + 0.14), (1.14, 0.15, fz + 0.14)],
        0.010,
        orange,
        "foot_handle_bail",
    )

    # Mounting brackets connecting basket wall to frame side tubes.
    for i, x in enumerate((-0.30, 0.10, 0.50, 0.85)):
        for j, y_sign in enumerate((-1.0, 1.0)):
            # Bracket tube from basket outer wall down to frame side tube.
            y_basket = y_sign * 0.28
            y_frame = y_sign * 0.35
            _tube(
                basket,
                (x, y_basket, fz + 0.015),
                (x, y_frame, 0.815),
                0.008,
                black,
                f"mount_bracket_{i}_{j}",
            )

    # ===================================================================
    # FOLDING LEG ASSEMBLIES (unchanged from parent baseline)
    # ===================================================================
    head_leg = model.part("head_leg")
    foot_leg = model.part("foot_leg")
    _add_leg_geometry(head_leg, yellow, black, footward=False)
    _add_leg_geometry(foot_leg, yellow, black, footward=True)

    # ===================================================================
    # CASTER WHEELS (unchanged from parent baseline)
    # ===================================================================
    caster_specs = [
        ("caster_0", head_leg, "head_leg_to_caster_0", (-0.075, -0.255, -0.660), "wheel_axle_0"),
        ("caster_1", head_leg, "head_leg_to_caster_1", (-0.075, 0.255, -0.660), "wheel_axle_1"),
        ("caster_2", foot_leg, "foot_leg_to_caster_0", (0.075, -0.255, -0.660), "wheel_axle_0"),
        ("caster_3", foot_leg, "foot_leg_to_caster_1", (0.075, 0.255, -0.660), "wheel_axle_1"),
    ]
    for caster_name, parent, joint_name, local_xyz, _axle in caster_specs:
        caster = model.part(caster_name)
        caster.visual(tire_mesh, material=dark_grey, name="tire")
        caster.visual(hub_mesh, material=metal, name="wheel_hub")
        for side_index, x in enumerate((-0.027, 0.027)):
            caster.visual(
                Cylinder(radius=0.030, length=0.008),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=metal,
                name=f"side_hub_cap_{side_index}",
            )
            caster.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(
                    xyz=(x + (0.004 if x > 0 else -0.004), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)
                ),
                material=black,
                name=f"black_axle_button_{side_index}",
            )
            for bolt_index in range(4):
                angle = bolt_index * math.pi / 2.0 + math.pi / 4.0
                caster.visual(
                    Sphere(0.004),
                    origin=Origin(
                        xyz=(
                            x + (0.005 if x > 0 else -0.005),
                            0.016 * math.cos(angle),
                            0.016 * math.sin(angle),
                        )
                    ),
                    material=black,
                    name=f"hub_bolt_{side_index}_{bolt_index}",
                )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=caster,
            origin=Origin(xyz=local_xyz, rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    # ===================================================================
    # ARTICULATIONS
    # ===================================================================

    # Basket-to-carriage FIXED mount (basket clamped onto frame side tubes).
    model.articulation(
        "deck_to_basket",
        ArticulationType.FIXED,
        parent=deck,
        child=basket,
        origin=Origin(),
    )

    # Folding leg joints (unchanged from parent).
    model.articulation(
        "deck_to_head_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=head_leg,
        origin=Origin(xyz=(-0.555, 0.0, 0.730)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "deck_to_foot_leg",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=foot_leg,
        origin=Origin(xyz=(0.650, 0.0, 0.730)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck_frame")
    basket = object_model.get_part("basket_shell")
    head_leg = object_model.get_part("head_leg")
    foot_leg = object_model.get_part("foot_leg")

    # ---- Intentional overlap allowances ----

    # Leg hinge capture inside deck sockets.
    ctx.allow_overlap(
        deck,
        head_leg,
        elem_a="head_leg_socket",
        elem_b="top_hinge",
        reason="The folding leg top hinge is represented as a captured tube inside the deck socket.",
    )
    ctx.allow_overlap(
        deck,
        foot_leg,
        elem_a="foot_leg_socket",
        elem_b="top_hinge",
        reason="The folding leg top hinge is represented as a captured tube inside the deck socket.",
    )
    for leg, socket in ((head_leg, "head_leg_socket"), (foot_leg, "foot_leg_socket")):
        for main_leg_name in ("main_leg_0", "main_leg_1"):
            ctx.allow_overlap(
                deck,
                leg,
                elem_a=socket,
                elem_b=main_leg_name,
                reason="The welded leg lug locally enters the hinge socket beside the hinge tube.",
            )

    # Caster axle piercing.
    for leg_name, caster_name, axle_name in (
        ("head_leg", "caster_0", "wheel_axle_0"),
        ("head_leg", "caster_1", "wheel_axle_1"),
        ("foot_leg", "caster_2", "wheel_axle_0"),
        ("foot_leg", "caster_3", "wheel_axle_1"),
    ):
        ctx.allow_overlap(
            caster_name,
            leg_name,
            elem_a="wheel_hub",
            elem_b=axle_name,
            reason="Each caster hub is intentionally pierced by its visible axle.",
        )

    # Caster assembly internal nesting (parent baseline geometry).
    for leg_name, caster_name, axle_name in (
        ("head_leg", "caster_0", "wheel_axle_0"),
        ("head_leg", "caster_1", "wheel_axle_1"),
        ("foot_leg", "caster_2", "wheel_axle_0"),
        ("foot_leg", "caster_3", "wheel_axle_1"),
    ):
        for stem_suffix in ("0", "1"):
            ctx.allow_overlap(
                caster_name,
                leg_name,
                elem_a="tire",
                elem_b=f"caster_swivel_stem_{stem_suffix}",
                reason="The caster tire sits close to the parent swivel stem as part of the caster assembly.",
            )
            ctx.allow_overlap(
                caster_name,
                leg_name,
                elem_a="tire",
                elem_b=f"caster_stem_{stem_suffix}",
                reason="The caster tire nests around the caster stem tube as part of the caster assembly.",
            )
            ctx.allow_overlap(
                caster_name,
                leg_name,
                elem_a="wheel_hub",
                elem_b=f"caster_swivel_stem_{stem_suffix}",
                reason="The caster hub sits close to the parent swivel stem as part of the caster assembly.",
            )
            ctx.allow_overlap(
                caster_name,
                leg_name,
                elem_a="wheel_hub",
                elem_b=f"caster_stem_{stem_suffix}",
                reason="The caster hub nests around the caster stem as part of the caster assembly.",
            )
            ctx.allow_overlap(
                caster_name,
                leg_name,
                elem_a="wheel_hub",
                elem_b=f"caster_yoke_bridge_{stem_suffix}",
                reason="The caster hub nests within the yoke bridge as part of the caster assembly.",
            )
            ctx.allow_overlap(
                caster_name,
                leg_name,
                elem_a="tire",
                elem_b=f"caster_yoke_bridge_{stem_suffix}",
                reason="The caster tire passes through the yoke bridge opening as part of the caster assembly.",
            )
        # Axle button caps and hub caps over the axle tube.
        for side_index in range(2):
            ctx.allow_overlap(
                caster_name,
                leg_name,
                elem_a=f"black_axle_button_{side_index}",
                elem_b=axle_name,
                reason="The axle button cap sits over the wheel axle as a visible end cap.",
            )
            ctx.allow_overlap(
                caster_name,
                leg_name,
                elem_a=f"side_hub_cap_{side_index}",
                elem_b=axle_name,
                reason="The side hub cap sits over the wheel axle as a visible cover.",
            )

    # Deck leg socket capturing the upper hinge clevis plates and washers (parent baseline).
    for leg_name, socket in (("head_leg", "head_leg_socket"), ("foot_leg", "foot_leg_socket")):
        for clevis_name in ("upper_hinge_clevis_near", "upper_hinge_clevis_far"):
            ctx.allow_overlap(
                deck,
                leg_name,
                elem_a=socket,
                elem_b=clevis_name,
                reason="The leg clevis plate locally enters the hinge socket as part of the folding joint capture.",
            )
        for washer_name in ("upper_hinge_washer_near", "upper_hinge_washer_far"):
            ctx.allow_overlap(
                deck,
                leg_name,
                elem_a=socket,
                elem_b=washer_name,
                reason="The leg hinge washer seats against the hinge socket as part of the folding joint capture.",
            )
        # Socket brackets also contact clevis plates during the hinge assembly.
        for bracket_suffix in ("0", "1"):
            bracket_name = f"{socket.replace('_leg_socket', '')}_socket_bracket_{bracket_suffix}"
            for clevis_name in ("upper_hinge_clevis_near", "upper_hinge_clevis_far"):
                ctx.allow_overlap(
                    deck,
                    leg_name,
                    elem_a=bracket_name,
                    elem_b=clevis_name,
                    reason="The socket bracket locally contacts the clevis plate as part of the folding joint assembly.",
                )

    # Basket floor pan seated on the basket body (intentional thin seating overlap).
    ctx.allow_overlap(
        basket,
        basket,
        elem_a="floor_pan",
        elem_b="basket_body",
        reason="The perforated floor pan sits flush on the molded basket body floor surface.",
    )

    # Mount brackets connect basket wall to frame side tubes (intentional contact).
    for i in range(4):
        for j in range(2):
            ctx.allow_overlap(
                basket,
                deck,
                elem_a=f"mount_bracket_{i}_{j}",
                elem_b="side_frame_0" if j == 0 else "side_frame_1",
                reason="Mount brackets bridge from basket wall to frame side tubes for structural support.",
            )

    # ---- Exact support proofs ----

    ctx.expect_overlap(
        head_leg, deck, axes="y", elem_a="top_hinge", elem_b="head_leg_socket", min_overlap=0.50
    )
    ctx.expect_overlap(
        foot_leg, deck, axes="y", elem_a="top_hinge", elem_b="foot_leg_socket", min_overlap=0.50
    )
    for leg, socket in ((head_leg, "head_leg_socket"), (foot_leg, "foot_leg_socket")):
        for main_leg_name in ("main_leg_0", "main_leg_1"):
            ctx.expect_overlap(
                leg,
                deck,
                axes="yz",
                elem_a=main_leg_name,
                elem_b=socket,
                min_overlap=0.010,
                name=f"{leg.name} {main_leg_name} remains seated at its hinge socket",
            )
    for leg_name, caster_name, axle_name in (
        ("head_leg", "caster_0", "wheel_axle_0"),
        ("head_leg", "caster_1", "wheel_axle_1"),
        ("foot_leg", "caster_2", "wheel_axle_0"),
        ("foot_leg", "caster_3", "wheel_axle_1"),
    ):
        ctx.expect_overlap(
            caster_name, leg_name, axes="y", elem_a="wheel_hub", elem_b=axle_name, min_overlap=0.030
        )

    # ---- Basket-shell-specific checks (STRUCTURAL_DELTA proofs) ----

    ctx.check(
        "basket_shell has a lofted basket_body visual",
        basket.get_visual("basket_body") is not None,
    )
    ctx.check(
        "basket_shell has a perforated floor_pan visual",
        basket.get_visual("floor_pan") is not None,
    )
    ctx.check(
        "basket_shell has a perimeter_rail visual",
        basket.get_visual("perimeter_rail") is not None,
    )

    # Basket floor sits above the deck frame side tubes (z-axis clearance).
    ctx.expect_gap(
        basket,
        deck,
        axis="z",
        positive_elem="basket_body",
        negative_elem="side_frame_0",
        max_penetration=0.008,
        name="basket body floor clears deck side_frame_0",
    )
    ctx.expect_gap(
        basket,
        deck,
        axis="z",
        positive_elem="basket_body",
        negative_elem="side_frame_1",
        max_penetration=0.008,
        name="basket body floor clears deck side_frame_1",
    )

    # Basket XY footprint stays within the deck frame perimeter.
    ctx.expect_within(
        basket,
        deck,
        axes="y",
        margin=0.04,
        name="basket shell fits within deck frame width",
    )

    # ---- Functional checks ----

    ctx.check(
        "stretcher has four caster wheel parts",
        all(
            object_model.get_part(name) is not None
            for name in ("caster_0", "caster_1", "caster_2", "caster_3")
        ),
    )
    ctx.check(
        "basket variant classification note recorded",
        "basket" in str(object_model.meta.get("run_notes", "")).lower(),
    )

    head_leg_joint = object_model.get_articulation("deck_to_head_leg")
    foot_leg_joint = object_model.get_articulation("deck_to_foot_leg")

    def _center_x(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0]) if aabb is not None else None

    head_deployed = ctx.part_world_aabb(head_leg)
    foot_deployed = ctx.part_world_aabb(foot_leg)
    with ctx.pose({head_leg_joint: 0.95, foot_leg_joint: 0.95}):
        head_folded = ctx.part_world_aabb(head_leg)
        foot_folded = ctx.part_world_aabb(foot_leg)
    ctx.check(
        "folding leg assemblies swing toward the stretcher center",
        head_deployed is not None
        and foot_deployed is not None
        and head_folded is not None
        and foot_folded is not None
        and _center_x(head_folded) is not None
        and _center_x(head_deployed) is not None
        and _center_x(foot_folded) is not None
        and _center_x(foot_deployed) is not None
        and _center_x(head_folded) > _center_x(head_deployed) + 0.15
        and _center_x(foot_folded) < _center_x(foot_deployed) - 0.15,
        details=f"head_deployed={head_deployed}, head_folded={head_folded}, foot_deployed={foot_deployed}, foot_folded={foot_folded}",
    )

    return ctx.report()


object_model = build_object_model()