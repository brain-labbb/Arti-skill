from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


# ── Round enclosure parameters ──
BOX_RADIUS = 0.070
BOX_H = 0.050
WALL = 0.004
HINGE_Y = BOX_RADIUS + 0.006
HINGE_Z = BOX_H


def _visual(part, geometry, *, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0), material=None, name=None):
    part.visual(geometry, origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_phillips_head(part, x: float, y: float, z: float, *, metal, slot, prefix: str) -> None:
    _visual(part, Cylinder(radius=0.0058, length=0.0024), xyz=(x, y, z), material=metal, name=f"{prefix}_head")
    _visual(part, Box((0.0090, 0.0014, 0.00045)), xyz=(x, y, z + 0.00135), material=slot, name=f"{prefix}_slot_a")
    _visual(part, Box((0.0014, 0.0090, 0.00045)), xyz=(x, y, z + 0.00136), material=slot, name=f"{prefix}_slot_b")


def _add_terminal_screw(part, x: float, y: float, *, white, brass, slot, prefix: str) -> None:
    _visual(part, Cylinder(radius=0.0042, length=0.010), xyz=(x, y, 0.021), material=white, name=f"{prefix}_insulator")
    _visual(part, Cylinder(radius=0.0026, length=0.0009), xyz=(x, y, 0.0264), material=brass, name=f"{prefix}_brass_cup")
    _visual(part, Cylinder(radius=0.0015, length=0.0007), xyz=(x, y, 0.0272), material=slot, name=f"{prefix}_dark_bore")


def _add_radial_gland(part, *, angle: float, gland_z: float, ribbed_mesh, clear, black, dark, name: str) -> None:
    """Add a cable gland entering the cylindrical wall radially at the given angle."""
    R = BOX_RADIUS
    axis_rpy = (0.0, math.pi / 2.0, angle)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    def rpos(r_offset: float):
        return ((R + r_offset) * cos_a, (R + r_offset) * sin_a, gland_z)

    _visual(part, Cylinder(radius=0.0115, length=0.020), xyz=rpos(0.002), rpy=axis_rpy, material=clear, name=f"{name}_clear_thread")
    for i, off in enumerate((-0.007, -0.003, 0.001, 0.005)):
        _visual(part, Cylinder(radius=0.0127, length=0.0012), xyz=rpos(off), rpy=axis_rpy, material=clear, name=f"{name}_thread_ring_{i}")
    _visual(part, ribbed_mesh, xyz=rpos(0.027), rpy=axis_rpy, material=black, name=f"{name}_ribbed_nut")
    _visual(part, Cylinder(radius=0.0155, length=0.0055), xyz=rpos(0.0125), rpy=axis_rpy, material=black, name=f"{name}_collar")
    _visual(part, Cylinder(radius=0.0135, length=0.006), xyz=rpos(0.040), rpy=axis_rpy, material=black, name=f"{name}_domed_end")
    _visual(part, Cylinder(radius=0.0070, length=0.0014), xyz=rpos(0.043), rpy=axis_rpy, material=dark, name=f"{name}_cable_hole")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electrical_junction_box")

    # ── Materials ──
    clear = model.material("clear_polycarbonate", rgba=(0.76, 0.84, 1.00, 0.40))
    clear_edge = model.material("thick_clear_edges", rgba=(0.70, 0.78, 1.00, 0.56))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.008, 1.0))
    dark = model.material("dark_cavity", rgba=(0.0, 0.0, 0.0, 1.0))
    white = model.material("white_nylon_terminal", rgba=(0.92, 0.91, 0.86, 1.0))
    brass = model.material("brass_terminal", rgba=(0.95, 0.68, 0.25, 1.0))
    screw_metal = model.material("polished_screw_steel", rgba=(0.84, 0.86, 0.86, 1.0))
    galvanized = model.material("galvanized_conduit", rgba=(0.50, 0.53, 0.54, 1.0))
    yellow = model.material("yellow_warning_label", rgba=(1.0, 0.84, 0.05, 1.0))
    red = model.material("red_wire_jacket", rgba=(0.86, 0.03, 0.03, 1.0))
    blue = model.material("blue_wire_jacket", rgba=(0.02, 0.14, 0.78, 1.0))
    green = model.material("green_yellow_wire", rgba=(0.08, 0.62, 0.12, 1.0))

    # ── Ribbed gland nut mesh (shared) ──
    gland_mesh = mesh_from_geometry(
        KnobGeometry(
            0.030,
            0.026,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=32, depth=0.0013, width=0.0012),
        ),
        "ribbed_cable_gland_nut",
    )

    # ══════════════════════════════════════════════════════════════
    # ENCLOSURE  (round cylindrical body)
    # ══════════════════════════════════════════════════════════════
    enclosure = model.part("enclosure")

    # ── Cylindrical wall (hollow tube via LatheGeometry) ──
    wall_geom = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(BOX_RADIUS, 0.004), (BOX_RADIUS, BOX_H)],
            [(BOX_RADIUS - WALL, 0.004), (BOX_RADIUS - WALL, BOX_H)],
            segments=48,
        ),
        "cylindrical_wall",
    )
    _visual(enclosure, wall_geom, material=clear, name="cylindrical_wall")

    # ── Circular bottom plate ──
    _visual(enclosure, Cylinder(radius=BOX_RADIUS, length=0.004),
            xyz=(0.0, 0.0, 0.002), material=clear, name="bottom_plate")

    # ── Annular gasket ring on top of wall ──
    gasket_geom = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(BOX_RADIUS - 0.001, BOX_H), (BOX_RADIUS - 0.001, BOX_H + 0.0015)],
            [(BOX_RADIUS - WALL + 0.001, BOX_H), (BOX_RADIUS - WALL + 0.001, BOX_H + 0.0015)],
            segments=48,
        ),
        "gasket_ring",
    )
    _visual(enclosure, gasket_geom, material=rubber, name="gasket_ring")

    # ── Screw bosses (5 arranged in a circle inside the enclosure) ──
    boss_radius = BOX_RADIUS - 0.015
    for i in range(5):
        angle = i * (2.0 * math.pi / 5.0) + math.pi / 10.0
        bx = boss_radius * math.cos(angle)
        by = boss_radius * math.sin(angle)
        _visual(enclosure, Cylinder(radius=0.0066, length=0.036),
                xyz=(bx, by, 0.022), material=clear_edge, name=f"screw_boss_{i}")
        _visual(enclosure, Cylinder(radius=0.0022, length=0.0010),
                xyz=(bx, by, 0.0405), material=dark, name=f"boss_hole_{i}")

    # ── External mounting ears (4 at diagonal positions) ──
    for i in range(4):
        angle = math.pi / 4.0 + i * (math.pi / 2.0)
        ear_r = BOX_RADIUS + 0.010
        ex = ear_r * math.cos(angle)
        ey = ear_r * math.sin(angle)
        ear_rpy = (0.0, 0.0, angle)
        _visual(enclosure, Box((0.020, 0.014, 0.004)),
                xyz=(ex, ey, 0.006), rpy=ear_rpy, material=clear_edge, name=f"mounting_ear_{i}")
        _visual(enclosure, Cylinder(radius=0.0031, length=0.0044),
                xyz=(ex, ey, 0.0084), material=dark, name=f"mounting_hole_{i}")

    # ── Cable glands (2 front + 1 rear) entering radially through curved wall ──
    gland_z = 0.030
    _add_radial_gland(enclosure, angle=-math.pi / 2 - 0.4, gland_z=gland_z,
                      ribbed_mesh=gland_mesh, clear=clear_edge, black=rubber, dark=dark, name="front_gland_0")
    _add_radial_gland(enclosure, angle=-math.pi / 2 + 0.4, gland_z=gland_z,
                      ribbed_mesh=gland_mesh, clear=clear_edge, black=rubber, dark=dark, name="front_gland_1")
    _add_radial_gland(enclosure, angle=math.pi / 2, gland_z=gland_z,
                      ribbed_mesh=gland_mesh, clear=clear_edge, black=rubber, dark=dark, name="rear_gland")

    # ── Side knockouts on curved wall ──
    for i, angle in enumerate((0.0, math.pi)):
        kx = BOX_RADIUS * math.cos(angle)
        ky = BOX_RADIUS * math.sin(angle)
        _visual(enclosure, Cylinder(radius=0.010, length=0.0012),
                xyz=(kx, ky, 0.025), rpy=(0.0, math.pi / 2.0, angle),
                material=clear_edge, name=f"side_knockout_{i}")

    # ── Internal terminal strip, bus bar, terminal screws ──
    _visual(enclosure, Box((0.060, 0.018, 0.012)),
            xyz=(0.0, -0.002, 0.010), material=white, name="terminal_strip")
    _visual(enclosure, Box((0.054, 0.003, 0.0014)),
            xyz=(0.0, -0.002, 0.0167), material=brass, name="brass_bus_bar")

    n = 0
    for x in (-0.020, 0.0, 0.020):
        for y in (-0.008, 0.004):
            _add_terminal_screw(enclosure, x, y, white=white, brass=brass, slot=dark, prefix=f"terminal_{n}")
            n += 1

    # ── Ground lug ──
    _visual(enclosure, Box((0.018, 0.010, 0.003)),
            xyz=(-0.040, 0.024, 0.0065), material=brass, name="ground_lug")
    _visual(enclosure, Cylinder(radius=0.0028, length=0.0012),
            xyz=(-0.040, 0.024, 0.0088), material=screw_metal, name="ground_screw")

    # ── Believable wire ends entering from glands and landing on terminals ──
    a0 = -math.pi / 2 - 0.4
    a1 = -math.pi / 2 + 0.4
    a2 = math.pi / 2
    R = BOX_RADIUS

    wire_specs = [
        ("red_wire", red, [
            (R * math.cos(a0), R * math.sin(a0), gland_z),
            (R * 0.5 * math.cos(a0), R * 0.5 * math.sin(a0), gland_z + 0.005),
            (-0.020, -0.008, 0.027),
        ]),
        ("blue_wire", blue, [
            (R * math.cos(a1), R * math.sin(a1), gland_z),
            (R * 0.5 * math.cos(a1), R * 0.5 * math.sin(a1), gland_z + 0.004),
            (0.020, 0.004, 0.027),
        ]),
        ("green_wire", green, [
            (R * math.cos(a2), R * math.sin(a2), gland_z),
            (R * 0.3 * math.cos(a2), R * 0.3 * math.sin(a2), gland_z + 0.005),
            (-0.040, 0.024, 0.010),
        ]),
        ("black_wire", rubber, [
            (R * math.cos(a0), R * math.sin(a0), gland_z - 0.004),
            (R * 0.4 * math.cos(a0), R * 0.4 * math.sin(a0), 0.026),
            (0.000, -0.008, 0.027),
        ]),
    ]
    for wire_name, mat, pts in wire_specs:
        wire_mesh = mesh_from_geometry(
            tube_from_spline_points(pts, radius=0.00145, samples_per_segment=10,
                                    radial_segments=12, cap_ends=True),
            wire_name,
        )
        _visual(enclosure, wire_mesh, material=mat, name=wire_name)

    # ── Hinge hardware on enclosure (rear leaf + alternating barrels) ──
    hinge_leaf_width = BOX_RADIUS * 1.4
    # Leaf overlaps wall outer surface to ensure connectivity
    _visual(enclosure, Box((hinge_leaf_width, 0.004, 0.006)),
            xyz=(0.0, BOX_RADIUS + 0.001, HINGE_Z - 0.002),
            material=galvanized, name="hinge_leaf")
    for i, (x, length) in enumerate(((-0.042, 0.024), (0.000, 0.018), (0.042, 0.024))):
        _visual(enclosure, Cylinder(radius=0.0037, length=length),
                xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0),
                material=galvanized, name=f"base_hinge_barrel_{i}")

    # ══════════════════════════════════════════════════════════════
    # COVER  (circular hinged lid)
    # ══════════════════════════════════════════════════════════════
    cover = model.part("cover")
    # Child frame = hinge axis.  At q=0 the transparent lid is closed and extends along local -Y.
    lid_center_y = -HINGE_Y
    lid_center_z = 0.005

    # ── Circular lid panel ──
    _visual(cover, Cylinder(radius=BOX_RADIUS + 0.001, length=0.006),
            xyz=(0.0, lid_center_y, lid_center_z), material=clear, name="lid_panel")

    # ── Lid rim (raised ring at the bottom face of the lid) ──
    lid_rim_geom = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(BOX_RADIUS + 0.001, -0.001), (BOX_RADIUS + 0.001, 0.001)],
            [(BOX_RADIUS - 0.003, -0.001), (BOX_RADIUS - 0.003, 0.001)],
            segments=48,
        ),
        "lid_outer_rim",
    )
    _visual(cover, lid_rim_geom,
            xyz=(0.0, lid_center_y, lid_center_z - 0.002), material=clear_edge, name="lid_outer_rim")

    # ── Molded seams on the lid face ──
    _visual(cover, Box((BOX_RADIUS * 1.6, 0.0012, 0.0012)),
            xyz=(0.0, lid_center_y, lid_center_z + 0.0035), material=clear_edge, name="molded_center_seam_x")
    _visual(cover, Box((0.0012, BOX_RADIUS * 1.6, 0.0012)),
            xyz=(0.0, lid_center_y, lid_center_z + 0.0036), material=clear_edge, name="molded_center_seam_y")

    # ── Cover hinge leaf and barrels (alternate with enclosure barrels) ──
    # Hinge leaf bridges from lid panel disk rear edge down to the hinge barrels
    _visual(cover, Box((hinge_leaf_width - 0.020, 0.020, 0.004)),
            xyz=(0.0, -0.006, 0.003), material=galvanized, name="cover_hinge_leaf")
    for i, (x, length) in enumerate(((-0.020, 0.018), (0.022, 0.024))):
        _visual(cover, Cylinder(radius=0.0033, length=length),
                xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0),
                material=galvanized, name=f"cover_hinge_barrel_{i}")

    # ── Retained cover screws (6 evenly around the circular lid) ──
    lid_screw_radius = BOX_RADIUS - 0.012
    for i in range(6):
        alpha = i * (math.pi / 3.0)
        sx = lid_screw_radius * math.cos(alpha)
        sy = lid_center_y + lid_screw_radius * math.sin(alpha)
        sz = lid_center_z + 0.004
        _visual(cover, Cylinder(radius=0.0082, length=0.0020),
                xyz=(sx, sy, sz - 0.0002), material=clear_edge, name=f"lid_screw_pad_{i}")
        _add_phillips_head(cover, sx, sy, sz, metal=screw_metal, slot=dark, prefix=f"cover_screw_{i}")

    # ── Labels and molded warnings on the transparent lid ──
    _visual(cover, Box((0.034, 0.014, 0.0007)),
            xyz=(0.015, lid_center_y, lid_center_z + 0.0034), material=yellow, name="warning_label")
    _visual(cover, Box((0.003, 0.010, 0.0009)),
            xyz=(0.007, lid_center_y, lid_center_z + 0.004), material=dark, name="warning_bolt_0")
    _visual(cover, Box((0.003, 0.010, 0.0009)),
            xyz=(0.013, lid_center_y, lid_center_z + 0.004), rpy=(0.0, 0.0, -0.55),
            material=dark, name="warning_bolt_1")
    _visual(cover, Box((0.026, 0.004, 0.0008)),
            xyz=(-0.020, lid_center_y - 0.015, lid_center_z + 0.0034),
            material=clear_edge, name="raised_ip68_mark")
    _visual(cover, Box((0.020, 0.003, 0.0008)),
            xyz=(-0.022, lid_center_y + 0.012, lid_center_z + 0.0034),
            material=clear_edge, name="raised_rating_mark")

    # ══════════════════════════════════════════════════════════════
    # ARTICULATION
    # ══════════════════════════════════════════════════════════════
    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    cover = object_model.get_part("cover")
    hinge = object_model.get_articulation("cover_hinge")

    # ── Identity and structure ──
    ctx.check("asset is named as a junction box", "junction_box" in object_model.name)
    ctx.check("primary hinged cover joint exists", hinge is not None)
    ctx.check("junction box has exactly one non-fixed cover joint", len(object_model.articulations) == 1)

    # ── Round-enclosure-specific assertion (TARGET verification) ──
    enclosure_visuals = {v.name for v in enclosure.visuals}
    ctx.check(
        "enclosure has a cylindrical wall (round form factor)",
        "cylindrical_wall" in enclosure_visuals and "bottom_plate" in enclosure_visuals,
        details=f"enclosure visuals: {sorted(enclosure_visuals)}",
    )
    ctx.check(
        "enclosure gasket is a continuous ring (round IP68 box)",
        "gasket_ring" in enclosure_visuals,
    )

    # ── Electrical junction details ──
    required_details = {
        "terminal_strip",
        "brass_bus_bar",
        "ground_lug",
        "front_gland_0_ribbed_nut",
        "front_gland_1_ribbed_nut",
        "rear_gland_ribbed_nut",
        "red_wire",
        "blue_wire",
        "green_wire",
        "hinge_leaf",
    }
    ctx.check(
        "electrical junction details are modeled as geometry",
        required_details.issubset(enclosure_visuals),
        details=f"missing={sorted(required_details - enclosure_visuals)}",
    )

    # ── Cover details ──
    cover_visuals = {v.name for v in cover.visuals}
    ctx.check(
        "transparent circular cover has screws, rim, and labels",
        {"lid_panel", "warning_label", "cover_screw_0_head", "lid_outer_rim", "cover_hinge_barrel_0"}.issubset(cover_visuals),
        details=f"missing={sorted({'lid_panel', 'warning_label', 'cover_screw_0_head', 'lid_outer_rim', 'cover_hinge_barrel_0'} - cover_visuals)}",
    )

    # ── Closed pose: lid seats on gasket ──
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            cover,
            enclosure,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="gasket_ring",
            min_gap=0.0,
            max_gap=0.002,
            name="closed lid seats just above gasket without penetrating",
        )
        ctx.expect_overlap(
            cover,
            enclosure,
            axes="xy",
            elem_a="lid_panel",
            elem_b="bottom_plate",
            min_overlap=0.090,
            name="closed cover spans the round enclosure footprint",
        )

    # ── Open pose: lid swings upward ──
    closed_aabb = ctx.part_element_world_aabb(cover, elem="lid_panel")
    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(cover, elem="lid_panel")
        ctx.check(
            "hinge opens the cover upward with a realistic limit",
            closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.045,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
