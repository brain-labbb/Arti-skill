from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
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


AXLE_PIVOT = (0.0, -0.60, 0.25)


def _origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _barrow_origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(
        xyz=(x - AXLE_PIVOT[0], y - AXLE_PIVOT[1], z - AXLE_PIVOT[2]),
        rpy=rpy,
    )


def _barrow_point(point):
    return tuple(point[i] - AXLE_PIVOT[i] for i in range(3))


def _barrow_mesh(geometry: MeshGeometry) -> MeshGeometry:
    return geometry.translate(-AXLE_PIVOT[0], -AXLE_PIVOT[1], -AXLE_PIVOT[2])


def _rounded_rect_loop(width: float, length: float, z: float, segments: int = 64) -> list[tuple[float, float, float]]:
    """Superellipse-like loop in the XY plane, used for the tapered metal tray."""
    pts: list[tuple[float, float, float]] = []
    exponent = 3.6
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        ct = math.cos(t)
        st = math.sin(t)
        x = (width * 0.5) * math.copysign(abs(ct) ** (2.0 / exponent), ct)
        y = (length * 0.5) * math.copysign(abs(st) ** (2.0 / exponent), st)
        pts.append((x, y, z))
    return pts


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _wheelbarrow_tray_geometry() -> MeshGeometry:
    """A hollow, open, deep galvanized tray with a rolled/folded upper lip."""
    geom = MeshGeometry()
    n = 72

    # Four corresponding loops: tapered underside, inner floor, inner mouth, and a
    # larger rolled rim.  The open top stays visibly hollow instead of capped.
    outer_bottom = _rounded_rect_loop(0.42, 0.58, 0.32, n)
    inner_bottom = _rounded_rect_loop(0.32, 0.48, 0.36, n)
    inner_top = _rounded_rect_loop(0.72, 1.04, 0.67, n)
    outer_lip = _rounded_rect_loop(0.86, 1.18, 0.70, n)

    loops = []
    for loop in (outer_bottom, inner_bottom, inner_top, outer_lip):
        idxs = [geom.add_vertex(*p) for p in loop]
        loops.append(idxs)

    ob, ib, it, ol = loops
    center_under = geom.add_vertex(0.0, 0.0, 0.32)
    center_floor = geom.add_vertex(0.0, 0.0, 0.36)

    for i in range(n):
        j = (i + 1) % n
        # Outer sloped steel wall
        _add_quad(geom, ob[i], ob[j], ol[j], ol[i])
        # Inner sloped wall of the usable basin
        _add_quad(geom, it[i], it[j], ib[j], ib[i])
        # Folded/rolled top lip bridges outer shell to inner opening.
        _add_quad(geom, ol[i], ol[j], it[j], it[i])
        # Bottom thickness bridge between outside underside and inside floor.
        _add_quad(geom, ib[i], ib[j], ob[j], ob[i])
        # Underside and interior floor caps.
        geom.add_face(center_under, ob[j], ob[i])
        geom.add_face(center_floor, ib[i], ib[j])

    return geom





def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_single_wheelbarrow",
        meta={
            "category": "Agricultural",
            "small_class": "Single-Wheelbarrow",
            "reference": "picture/Agricultural/Single-Wheelbarrow/001.png",
        },
    )

    galvanized = model.material("weathered_galvanized_tray", rgba=(0.58, 0.62, 0.62, 1.0))
    darker_galv = model.material("darker_scuffed_metal", rgba=(0.42, 0.45, 0.45, 1.0))
    green = model.material("dark_green_painted_steel", rgba=(0.02, 0.18, 0.11, 1.0))
    green_edge = model.material("worn_green_edges", rgba=(0.04, 0.25, 0.16, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.02, 1.0))
    grip_rubber = model.material("grey_textured_grip", rgba=(0.20, 0.20, 0.22, 1.0))
    hardware = model.material("dark_bolt_hardware", rgba=(0.03, 0.03, 0.03, 1.0))

    axle_pivot = model.part("wheel_axle_pivot")
    barrow = model.part("barrow")

    # Deep tapered tray: approximately 0.86 m wide by 1.18 m long at the lip.
    barrow.visual(
        mesh_from_geometry(_barrow_mesh(_wheelbarrow_tray_geometry()), "deep_tray_shell"),
        material=galvanized,
        name="tray_shell",
    )

    # Slightly darker outside reinforcing straps and side panel seam ribs.
    for x, name in [(-0.360, "side_rib_0"), (0.360, "side_rib_1")]:
        barrow.visual(
            Box((0.012, 0.80, 0.028)),
            origin=_barrow_origin(x, 0.05, 0.555),
            material=darker_galv,
            name=name,
        )
    barrow.visual(
        Box((0.58, 0.030, 0.026)),
        origin=_barrow_origin(0.0, -0.535, 0.635),
        material=darker_galv,
        name="front_seam",
    )
    barrow.visual(
        Box((0.52, 0.035, 0.024)),
        origin=_barrow_origin(0.0, 0.535, 0.635),
        material=darker_galv,
        name="rear_seam",
    )

    # ── Welded straight flat-bar / angle-iron frame ─────────────────────

    # Rail geometry: straight Box beams from axle area to handle area.
    rail_front_y, rail_front_z = -0.55, 0.22
    rail_rear_y, rail_rear_z = 1.42, 0.95
    rail_dy = rail_rear_y - rail_front_y
    rail_dz = rail_rear_z - rail_front_z
    rail_length = math.sqrt(rail_dy * rail_dy + rail_dz * rail_dz)
    rail_angle = math.atan2(rail_dz, rail_dy)
    rail_cy = (rail_front_y + rail_rear_y) * 0.5
    rail_cz = (rail_front_z + rail_rear_z) * 0.5

    def _rail_z_at(y: float) -> float:
        return rail_front_z + (rail_dz / rail_dy) * (y - rail_front_y)

    # Main rails: two straight 35 mm × 25 mm rectangular-section beams.
    for x, idx in [(-0.31, 0), (0.31, 1)]:
        barrow.visual(
            Box((0.035, rail_length, 0.025)),
            origin=_barrow_origin(x, rail_cy, rail_cz, (rail_angle, 0.0, 0.0)),
            material=green,
            name=f"handle_rail_{idx}",
        )

    # Front guard: straight flat-bar cross-piece positioned ahead of the wheel.
    guard_y, guard_z = -0.88, 0.210
    barrow.visual(
        Box((0.62, 0.025, 0.040)),
        origin=_barrow_origin(0.0, guard_y, guard_z),
        material=green,
        name="front_guard",
    )
    # Nose bars connecting the rail fronts forward to the front guard.
    for x, idx in [(-0.31, 0), (0.31, 1)]:
        ndy = guard_y - rail_front_y
        ndz = guard_z - rail_front_z
        nlen = math.sqrt(ndy * ndy + ndz * ndz)
        nang = math.atan2(ndz, ndy)
        barrow.visual(
            Box((0.030, nlen, 0.025)),
            origin=_barrow_origin(
                x,
                (rail_front_y + guard_y) * 0.5,
                (rail_front_z + guard_z) * 0.5,
                (nang, 0.0, 0.0),
            ),
            material=green,
            name=f"nose_bar_{idx}",
        )

    # Straight flat-bar cross members welded between the rails.
    for y, nm in [(-0.22, "front_cross_bar"), (0.28, "rear_cross_bar")]:
        z = _rail_z_at(y)
        barrow.visual(
            Box((0.62, 0.025, 0.025)),
            origin=_barrow_origin(0.0, y, z),
            material=green,
            name=nm,
        )

    # A-frame support legs: two straight flat bars per side forming an
    # inverted V from a single apex mount on the rail down to two feet.
    apex_y = 0.25
    for x, idx in [(-0.31, 0), (0.31, 1)]:
        apex_z = _rail_z_at(apex_y)
        # Front leg of A-frame.
        ff_y, ff_z = 0.00, 0.040
        dy_f = ff_y - apex_y
        dz_f = ff_z - apex_z
        len_f = math.sqrt(dy_f * dy_f + dz_f * dz_f)
        ang_f = math.atan2(dz_f, dy_f)
        barrow.visual(
            Box((0.030, len_f, 0.006)),
            origin=_barrow_origin(x, (apex_y + ff_y) * 0.5, (apex_z + ff_z) * 0.5, (ang_f, 0.0, 0.0)),
            material=green_edge,
            name=f"support_leg_{idx}",
        )
        # Rear leg of A-frame.
        rf_y, rf_z = 0.55, 0.040
        dy_r = rf_y - apex_y
        dz_r = rf_z - apex_z
        len_r = math.sqrt(dy_r * dy_r + dz_r * dz_r)
        ang_r = math.atan2(dz_r, dy_r)
        barrow.visual(
            Box((0.030, len_r, 0.006)),
            origin=_barrow_origin(x, (apex_y + rf_y) * 0.5, (apex_z + rf_z) * 0.5, (ang_r, 0.0, 0.0)),
            material=green_edge,
            name=f"aframe_rear_{idx}",
        )
        # Gusset plate at the A-frame apex (welded to the rail).
        barrow.visual(
            Box((0.038, 0.060, 0.006)),
            origin=_barrow_origin(x, apex_y, apex_z - 0.030),
            material=green,
            name=f"aframe_gusset_{idx}",
        )
        # Foot pads at both ground contact points.
        for fy, fz, pad_idx in [(ff_y, ff_z, idx), (rf_y, rf_z, idx + 2)]:
            barrow.visual(
                Box((0.070, 0.055, 0.012)),
                origin=_barrow_origin(x, fy, fz - 0.006),
                material=green_edge,
                name=f"foot_pad_{pad_idx}",
            )

    # Straight diagonal front struts from the tray nose to axle region.
    for x, idx in [(-0.27, 0), (0.27, 1)]:
        s_top_y, s_top_z = -0.42, 0.52
        s_bot_y, s_bot_z = -0.58, 0.26
        sdy = s_bot_y - s_top_y
        sdz = s_bot_z - s_top_z
        slen = math.sqrt(sdy * sdy + sdz * sdz)
        sang = math.atan2(sdz, sdy)
        barrow.visual(
            Box((0.025, slen, 0.006)),
            origin=_barrow_origin(x, (s_top_y + s_bot_y) * 0.5, (s_top_z + s_bot_z) * 0.5, (sang, 0.0, 0.0)),
            material=green,
            name=f"front_strut_{idx}",
        )

    # Straight axle cross-bar (through-axle for the front wheel).
    barrow.visual(
        Cylinder(radius=0.022, length=0.74),
        origin=_barrow_origin(0.0, -0.60, 0.25, (0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="axle",
    )

    # Axle brackets: straight flat-bar plates bolted to the rails, carrying
    # the axle at their lower edge.
    for x, idx in [(-0.090, 0), (0.090, 1)]:
        barrow.visual(
            Box((0.006, 0.150, 0.200)),
            origin=_barrow_origin(x, -0.525, 0.305),
            material=green,
            name=f"axle_bracket_{idx}",
        )
        # Axle nut and two visible mounting bolts on each bracket plate.
        for y, z, rad, length, bolt_idx in [
            (-0.580, 0.250, 0.026, 0.014, 0),
            (-0.555, 0.230, 0.015, 0.010, 1),
            (-0.500, 0.340, 0.015, 0.010, 2),
        ]:
            barrow.visual(
                Cylinder(radius=rad, length=length),
                origin=_barrow_origin(x + (0.005 if x > 0 else -0.005), y, z, (0.0, math.pi / 2.0, 0.0)),
                material=hardware,
                name=f"bracket_bolt_{idx}_{bolt_idx}",
            )

    # Tray mounting pads tie the tray underside to the flat-bar frame.
    for x in (-0.18, 0.18):
        for y in (-0.12, 0.24):
            z = _rail_z_at(y) - 0.035
            suffix = f"{0 if x < 0 else 1}_{0 if y < 0 else 1}"
            barrow.visual(
                Box((0.085, 0.060, 0.055)),
                origin=_barrow_origin(x, y, z),
                material=green,
                name=f"tray_mount_{suffix}",
            )

    # Bolt-on straight handle stubs: short square-section bars extending
    # past the rail ends to carry the rubber grips.
    stub_top_y, stub_top_z = 1.32, _rail_z_at(1.32)
    stub_end_y, stub_end_z = 1.62, 0.970
    sdy_h = stub_end_y - stub_top_y
    sdz_h = stub_end_z - stub_top_z
    slen_h = math.sqrt(sdy_h * sdy_h + sdz_h * sdz_h)
    sang_h = math.atan2(sdz_h, sdy_h)
    for x, idx in [(-0.31, 0), (0.31, 1)]:
        barrow.visual(
            Box((0.028, slen_h, 0.028)),
            origin=_barrow_origin(
                x,
                (stub_top_y + stub_end_y) * 0.5,
                (stub_top_z + stub_end_z) * 0.5,
                (sang_h, 0.0, 0.0),
            ),
            material=green,
            name=f"handle_stub_{idx}",
        )
        # Two visible bolts fixing the stub to the rail end.
        for bolt_dy in (-0.06, 0.04):
            bolt_y = (stub_top_y + stub_end_y) * 0.5 + bolt_dy
            bolt_z = _rail_z_at(bolt_y) + 0.018
            barrow.visual(
                Cylinder(radius=0.008, length=0.012),
                origin=_barrow_origin(x, bolt_y, bolt_z),
                material=hardware,
                name=f"stub_bolt_{idx}_{0 if bolt_dy < 0 else 1}",
            )

    # Rubber handle grips with ring texture, mounted on the handle stubs.
    for x, idx in [(-0.31, 0), (0.31, 1)]:
        barrow.visual(
            Cylinder(radius=0.035, length=0.18),
            origin=_barrow_origin(x, 1.505, 0.965, (-math.pi / 2.0, 0.0, 0.0)),
            material=grip_rubber,
            name=f"grip_{idx}",
        )
        for ring_i, y in enumerate((1.445, 1.485, 1.525, 1.565)):
            barrow.visual(
                Cylinder(radius=0.037, length=0.008),
                origin=_barrow_origin(x, y, 0.965, (-math.pi / 2.0, 0.0, 0.0)),
                material=hardware,
                name=f"grip_ring_{idx}_{ring_i}",
            )

    # Wheel child part.  The helper geometry is centered on the axle and spins
    # about local X, matching the revolute axis below.
    wheel = model.part("wheel")
    tire = TireGeometry(
        0.235,
        0.112,
        inner_radius=0.158,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.08),
        tread=TireTread(style="block", depth=0.010, count=24, land_ratio=0.55),
        grooves=(
            TireGroove(center_offset=0.0, width=0.010, depth=0.004),
            TireGroove(center_offset=-0.030, width=0.006, depth=0.0025),
            TireGroove(center_offset=0.030, width=0.006, depth=0.0025),
        ),
        sidewall=TireSidewall(style="rounded", bulge=0.06),
        shoulder=TireShoulder(width=0.012, radius=0.004),
    )
    wheel.visual(mesh_from_geometry(tire, "rubber_tire"), material=rubber, name="rubber_tire")

    rim = WheelGeometry(
        0.154,
        0.082,
        rim=WheelRim(inner_radius=0.080, flange_height=0.012, flange_thickness=0.006, bead_seat_depth=0.005),
        hub=WheelHub(
            radius=0.048,
            width=0.074,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.062, hole_diameter=0.006),
        ),
        face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.006, window_radius=0.020),
        bore=WheelBore(style="round", diameter=0.044),
    )
    wheel.visual(mesh_from_geometry(rim, "metal_rim"), material=green_edge, name="metal_rim")

    # A tiny valve stem makes wheel rotation visibly pose-dependent.
    wheel.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.063, 0.055, 0.145), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="valve_stem",
    )

    model.articulation(
        "axle_pivot_to_barrow",
        ArticulationType.REVOLUTE,
        parent=axle_pivot,
        child=barrow,
        origin=_origin(*AXLE_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=34.0, velocity=1.0, lower=0.0, upper=1.05),
    )

    model.articulation(
        "axle_pivot_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle_pivot,
        child=wheel,
        origin=_origin(*AXLE_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrow = object_model.get_part("barrow")
    wheel = object_model.get_part("wheel")
    body_tip = object_model.get_articulation("axle_pivot_to_barrow")
    wheel_spin = object_model.get_articulation("axle_pivot_to_wheel")

    ctx.check(
        "small class is Single-Wheelbarrow",
        object_model.meta.get("small_class") == "Single-Wheelbarrow",
        details=f"meta={object_model.meta}",
    )
    ctx.check(
        "agricultural single wheelbarrow has axle pivot, tipping body, and moving wheel",
        len(object_model.parts) == 3
        and body_tip.parent == "wheel_axle_pivot"
        and body_tip.child == "barrow"
        and wheel_spin.parent == "wheel_axle_pivot"
        and wheel_spin.child == "wheel",
        details=f"parts={[p.name for p in object_model.parts]}, articulations={[a.name for a in object_model.articulations]}",
    )
    ctx.check(
        "front wheel joint spins about axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.check(
        "barrow body tips forward around the same wheel axle",
        body_tip.articulation_type == ArticulationType.REVOLUTE
        and tuple(body_tip.axis) == (1.0, 0.0, 0.0)
        and body_tip.motion_limits.lower == 0.0
        and body_tip.motion_limits.upper >= 1.0,
        details=f"type={body_tip.articulation_type}, axis={body_tip.axis}, limits={body_tip.motion_limits}",
    )

    ctx.allow_overlap(
        barrow,
        wheel,
        elem_a="axle",
        elem_b="metal_rim",
        reason=(
            "The fixed steel axle is intentionally captured inside the wheel hub/bushing; "
            "this local hidden fit supports the rotating wheel."
        ),
    )

    tray_aabb = ctx.part_element_world_aabb(barrow, elem="tray_shell")
    ctx.check(
        "deep hollow tray proportions",
        tray_aabb is not None
        and tray_aabb[1][0] - tray_aabb[0][0] > 0.80
        and tray_aabb[1][1] - tray_aabb[0][1] > 1.10
        and tray_aabb[1][2] - tray_aabb[0][2] > 0.35,
        details=f"tray_aabb={tray_aabb}",
    )
    for visual_name in (
        "handle_rail_0",
        "handle_rail_1",
        "grip_0",
        "grip_1",
        "front_guard",
        "support_leg_0",
        "support_leg_1",
        "aframe_rear_0",
        "aframe_rear_1",
        "handle_stub_0",
        "handle_stub_1",
        "front_strut_0",
        "front_strut_1",
        "front_cross_bar",
        "rear_cross_bar",
        "axle",
        "axle_bracket_0",
        "axle_bracket_1",
        "rubber_tire",
        "metal_rim",
    ):
        part = wheel if visual_name in {"rubber_tire", "metal_rim"} else barrow
        ctx.check(
            f"visible {visual_name} present",
            ctx.part_element_world_aabb(part, elem=visual_name) is not None,
            details=visual_name,
        )

    # Variant-specific: flat-bar frame rails are straight Box beams, not tubes.
    # Verify each rail is a long thin beam (length >> width, length >> thickness).
    for rail_name in ("handle_rail_0", "handle_rail_1"):
        rail_aabb = ctx.part_element_world_aabb(barrow, elem=rail_name)
        if rail_aabb is not None:
            dx = rail_aabb[1][0] - rail_aabb[0][0]
            dy = rail_aabb[1][1] - rail_aabb[0][1]
            dz = rail_aabb[1][2] - rail_aabb[0][2]
            longest = max(dx, dy, dz)
            shortest = min(dx, dy, dz)
            ctx.check(
                f"{rail_name} is a straight flat-bar beam (length >> section)",
                longest > 1.5 and shortest < 0.08 and longest / max(shortest, 1e-6) > 20.0,
                details=f"aabb_dims=({dx:.3f}, {dy:.3f}, {dz:.3f})",
            )

    # Variant-specific: A-frame legs have two diverging bars per side,
    # verified by checking both front and rear A-frame bars exist and the
    # rear foot is behind the apex while the front foot is ahead.
    for side_idx in (0, 1):
        front_leg = ctx.part_element_world_aabb(barrow, elem=f"support_leg_{side_idx}")
        rear_leg = ctx.part_element_world_aabb(barrow, elem=f"aframe_rear_{side_idx}")
        if front_leg is not None and rear_leg is not None:
            front_center_y = (front_leg[0][1] + front_leg[1][1]) * 0.5
            rear_center_y = (rear_leg[0][1] + rear_leg[1][1]) * 0.5
            ctx.check(
                f"A-frame side {side_idx} has diverging front and rear legs",
                rear_center_y > front_center_y + 0.10,
                details=f"front_y={front_center_y:.3f}, rear_y={rear_center_y:.3f}",
            )

    # Variant-specific: handle stubs extend rearward past the rail ends.
    for stub_idx in (0, 1):
        stub_aabb = ctx.part_element_world_aabb(barrow, elem=f"handle_stub_{stub_idx}")
        rail_aabb = ctx.part_element_world_aabb(barrow, elem=f"handle_rail_{stub_idx}")
        if stub_aabb is not None and rail_aabb is not None:
            ctx.check(
                f"handle_stub_{stub_idx} extends past handle_rail_{stub_idx} rear end",
                stub_aabb[1][1] > rail_aabb[1][1] - 0.05,
                details=f"stub_max_y={stub_aabb[1][1]:.3f}, rail_max_y={rail_aabb[1][1]:.3f}",
            )

    ctx.expect_within(
        wheel,
        barrow,
        axes="x",
        inner_elem="rubber_tire",
        outer_elem="axle",
        margin=0.0,
        name="single wheel is centered between axle ends",
    )
    ctx.expect_overlap(
        wheel,
        barrow,
        axes="x",
        elem_a="metal_rim",
        elem_b="axle",
        min_overlap=0.07,
        name="rim hub surrounds the axle along its width",
    )

    valve0 = ctx.part_element_world_aabb(wheel, elem="valve_stem")
    with ctx.pose({wheel_spin: 1.25}):
        valve1 = ctx.part_element_world_aabb(wheel, elem="valve_stem")

    def _center_yz(aabb):
        return ((aabb[0][1] + aabb[1][1]) * 0.5, (aabb[0][2] + aabb[1][2]) * 0.5)

    moved = False
    if valve0 is not None and valve1 is not None:
        y0, z0 = _center_yz(valve0)
        y1, z1 = _center_yz(valve1)
        moved = abs(y1 - y0) > 0.035 and abs(z1 - z0) > 0.035
    ctx.check("wheel rotation moves valve stem around axle", moved, details=f"rest={valve0}, posed={valve1}")

    def _center_xyz(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    grip0 = ctx.part_element_world_aabb(barrow, elem="grip_0")
    with ctx.pose({body_tip: 0.65}):
        grip1 = ctx.part_element_world_aabb(barrow, elem="grip_0")
    tipped = False
    if grip0 is not None and grip1 is not None:
        rest_center = _center_xyz(grip0)
        tip_center = _center_xyz(grip1)
        tipped = tip_center[2] > rest_center[2] + 0.75 and tip_center[1] < rest_center[1] - 0.45
    ctx.check("body tip joint swings the whole barrow forward around the wheel", tipped, details=f"rest={grip0}, tipped={grip1}")

    return ctx.report()


object_model = build_object_model()
