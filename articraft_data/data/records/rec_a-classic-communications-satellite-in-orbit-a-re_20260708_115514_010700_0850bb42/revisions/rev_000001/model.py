"""Classic box-bus communications satellite with two rotating blue solar wings.

Layout (meters, Z up, bus centered at origin, free-floating root):
- bus: gold-foil rectangular body, top/bottom decks, apogee nozzle underneath,
  solar-array-drive drums on the +/-X faces, dish gimbal mast, feed horn
  cluster, and two rod antennas on the Earth-facing (+Z) deck.
- wing_left / wing_right: silver boom + yoke + central spine rail carrying
  3 bays x 2 columns of blue photovoltaic panels with a light cell-grid,
  each wing spun by a CONTINUOUS solar array drive about the boom (X) axis.
- dish_antenna: parabolic reflector + axial feed on a REVOLUTE elevation
  gimbal captured between two ears at the top of the mast.
"""

from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- dimensions
BUS_X = 0.90
BUS_Y = 0.90
BUS_Z = 1.20

WING_ATTACH_Z = 0.0  # solar array drive height on the bus side faces
BOOM_R = 0.030
BOOM_LEN = 0.34  # spans local x -0.02 .. 0.32 (root embedded in the bus)

PANEL_BAYS = 3  # bays along each wing
PANEL_COLS = 2  # columns each side of the spine rail
PANEL_LX = 0.58
PANEL_LY = 0.44
PANEL_T = 0.025
BAY_PITCH = 0.62
FIRST_BAY_X = 0.70  # center of the first bay (outboard along the wing)

DISH_PIVOT = (0.28, 0.0, 0.98)  # world position of the elevation gimbal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="comsat_box_bus_twin_solar_wings")

    model.material("gold_foil", color=(0.82, 0.64, 0.26))
    model.material("gold_foil_bright", color=(0.90, 0.73, 0.34))
    model.material("gold_foil_dark", color=(0.62, 0.47, 0.18))
    model.material("solar_cell_blue", color=(0.15, 0.27, 0.55))
    model.material("solar_grid_light", color=(0.47, 0.63, 0.86))
    model.material("silver_metal", color=(0.78, 0.79, 0.82))
    model.material("dark_grey", color=(0.24, 0.24, 0.27))
    model.material("dish_white", color=(0.88, 0.88, 0.90))

    # ------------------------------------------------------------------ bus
    bus = model.part("bus")
    bus.visual(
        Box((BUS_X, BUS_Y, BUS_Z)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="gold_foil",
        name="body_core",
    )
    # Slightly proud foil blankets on the +/-Y faces (brighter foil sheets).
    for sy in (-1, 1):
        bus.visual(
            Box((0.84, 0.03, 1.12)),
            origin=Origin(xyz=(0.0, sy * 0.455, 0.0)),
            material="gold_foil_bright",
            name=f"foil_sheet_{'pos' if sy > 0 else 'neg'}_y",
        )
    # Darker foil corner rails along the vertical edges.
    for sx in (-1, 1):
        for sy in (-1, 1):
            bus.visual(
                Box((0.06, 0.06, 1.24)),
                origin=Origin(xyz=(sx * 0.44, sy * 0.44, 0.0)),
                material="gold_foil_dark",
                name=f"corner_rail_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    # Top (Earth-facing) and bottom decks.
    bus.visual(
        Box((0.92, 0.92, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        material="gold_foil_dark",
        name="top_deck",
    )
    bus.visual(
        Box((0.92, 0.92, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.615)),
        material="gold_foil_dark",
        name="bottom_deck",
    )
    # Apogee kick motor nozzle bell under the bottom deck (apex up, embedded).
    bus.visual(
        mesh_from_geometry(ConeGeometry(0.16, 0.28, radial_segments=32), "apogee_nozzle"),
        origin=Origin(xyz=(0.0, 0.0, -0.755)),
        material="dark_grey",
        name="apogee_nozzle",
    )
    # Solar array drive drums on the +/-X faces (wing booms plug into these).
    for sx in (-1, 1):
        bus.visual(
            Cylinder(radius=0.07, length=0.10),
            origin=Origin(xyz=(sx * 0.49, 0.0, WING_ATTACH_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material="silver_metal",
            name=f"sad_drum_{'right' if sx > 0 else 'left'}",
        )
    # Dish gimbal mast + head + ears on the top deck.
    bus.visual(
        Cylinder(radius=0.03, length=0.30),
        origin=Origin(xyz=(DISH_PIVOT[0], 0.0, 0.78)),
        material="silver_metal",
        name="gimbal_mast",
    )
    bus.visual(
        Box((0.08, 0.11, 0.03)),
        origin=Origin(xyz=(DISH_PIVOT[0], 0.0, 0.92)),
        material="silver_metal",
        name="gimbal_head",
    )
    for sy in (-1, 1):
        bus.visual(
            Box((0.06, 0.015, 0.10)),
            origin=Origin(xyz=(DISH_PIVOT[0], sy * 0.0475, 0.955)),
            material="silver_metal",
            name=f"gimbal_ear_{'pos' if sy > 0 else 'neg'}",
        )
    # Feed horn cluster (three flared horns) on the Earth-facing deck.
    horn_mesh = mesh_from_geometry(ConeGeometry(0.055, 0.16, radial_segments=24), "bus_feed_horn")
    for i in range(3):
        y = (i - 1) * 0.18
        bus.visual(
            Cylinder(radius=0.02, length=0.06),
            origin=Origin(xyz=(-0.25, y, 0.665)),
            material="silver_metal",
            name=f"feed_horn_stub_{i}",
        )
        bus.visual(
            horn_mesh,
            origin=Origin(xyz=(-0.25, y, 0.76), rpy=(pi, 0.0, 0.0)),
            material="silver_metal",
            name=f"feed_horn_{i}",
        )
    # Two slim rod antennas with ball tips.
    for sy in (-1, 1):
        tag = "pos" if sy > 0 else "neg"
        bus.visual(
            Cylinder(radius=0.008, length=0.44),
            origin=Origin(xyz=(0.05, sy * 0.30, 0.85)),
            material="silver_metal",
            name=f"rod_antenna_{tag}",
        )
        bus.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(0.05, sy * 0.30, 1.07)),
            material="silver_metal",
            name=f"rod_antenna_tip_{tag}",
        )

    # ---------------------------------------------------------- solar wings
    for sx, side in ((-1, "left"), (1, "right")):
        wing = model.part(f"wing_{side}")
        # Boom from the drive drum out to the yoke (root embedded in the bus).
        wing.visual(
            Cylinder(radius=BOOM_R, length=BOOM_LEN),
            origin=Origin(xyz=(sx * 0.15, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="silver_metal",
            name="wing_boom",
        )
        # Yoke cross bar at the boom tip.
        wing.visual(
            Box((0.05, 0.62, 0.05)),
            origin=Origin(xyz=(sx * 0.335, 0.0, 0.0)),
            material="silver_metal",
            name="yoke_bar",
        )
        # Central spine rail running the wing length.
        wing.visual(
            Box((1.95, 0.06, 0.05)),
            origin=Origin(xyz=(sx * 1.305, 0.0, 0.0)),
            material="silver_metal",
            name="spine_rail",
        )
        # End bar at the wing tip.
        wing.visual(
            Box((0.05, 0.62, 0.05)),
            origin=Origin(xyz=(sx * 2.25, 0.0, 0.0)),
            material="silver_metal",
            name="tip_bar",
        )
        # Photovoltaic panels: PANEL_BAYS bays x PANEL_COLS columns.
        for i in range(PANEL_BAYS):
            x_c = sx * (FIRST_BAY_X + BAY_PITCH * i)
            for sy in (-1, 1):
                col = "p" if sy > 0 else "n"
                y_c = sy * 0.24
                wing.visual(
                    Box((PANEL_LX, PANEL_LY, PANEL_T)),
                    origin=Origin(xyz=(x_c, y_c, 0.0)),
                    material="solar_cell_blue",
                    name=f"panel_{i}_{col}",
                )
                # Light cell-grid lines slightly proud of the sun-facing face.
                for k in range(3):
                    wing.visual(
                        Box((0.56, 0.008, 0.006)),
                        origin=Origin(xyz=(x_c, y_c + (k - 1) * 0.14, 0.0135)),
                        material="solar_grid_light",
                        name=f"grid_long_{i}_{col}_{k}",
                    )
                for m in range(4):
                    wing.visual(
                        Box((0.008, 0.42, 0.006)),
                        origin=Origin(xyz=(x_c + (-0.21 + m * 0.14), y_c, 0.0135)),
                        material="solar_grid_light",
                        name=f"grid_cross_{i}_{col}_{m}",
                    )

        # Solar array drive: each wing spins about its boom (X) axis.
        model.articulation(
            f"bus_to_wing_{side}",
            ArticulationType.CONTINUOUS,
            parent=bus,
            child=wing,
            origin=Origin(xyz=(sx * 0.45, 0.0, WING_ATTACH_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=0.5),
        )

    # -------------------------------------------------------- dish antenna
    dish = model.part("dish_antenna")
    dish.visual(
        Cylinder(radius=0.028, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_grey",
        name="pivot_hub",
    )
    # Pivot pin captured through the bus gimbal ears (intentional embed).
    dish.visual(
        Cylinder(radius=0.010, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="silver_metal",
        name="pivot_pin",
    )
    dish.visual(
        Cylinder(radius=0.022, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material="dark_grey",
        name="dish_stem",
    )
    # Parabolic reflector shell (lathe of a thin curved profile, opens +Z).
    dish_profile = [
        (0.00, 0.000),
        (0.10, 0.010),
        (0.18, 0.033),
        (0.26, 0.070),
        (0.26, 0.082),
        (0.18, 0.045),
        (0.10, 0.022),
        (0.00, 0.012),
    ]
    dish.visual(
        mesh_from_geometry(LatheGeometry(dish_profile, segments=48), "dish_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material="dish_white",
        name="dish_shell",
    )
    # Axial feed boom + feed horn at the focus.
    dish.visual(
        Cylinder(radius=0.008, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material="silver_metal",
        name="feed_boom",
    )
    dish.visual(
        mesh_from_geometry(ConeGeometry(0.030, 0.06, radial_segments=24), "dish_feed_horn"),
        origin=Origin(xyz=(0.0, 0.0, 0.27), rpy=(pi, 0.0, 0.0)),
        material="dark_grey",
        name="dish_feed_horn",
    )

    model.articulation(
        "bus_to_dish",
        ArticulationType.REVOLUTE,
        parent=bus,
        child=dish,
        origin=Origin(xyz=DISH_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.5, upper=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Intentional local embeds: boom roots inside the bus/drive drums, and the
    # dish pivot pin captured through the gimbal ears.
    for side in ("left", "right"):
        ctx.allow_overlap(
            "bus",
            f"wing_{side}",
            elem_a="body_core",
            elem_b="wing_boom",
            reason="solar array boom root plugs into the bus body at the drive",
        )
        ctx.allow_overlap(
            "bus",
            f"wing_{side}",
            elem_a=f"sad_drum_{side}",
            elem_b="wing_boom",
            reason="boom passes through the solar array drive drum (axisymmetric)",
        )
    for tag in ("pos", "neg"):
        ctx.allow_overlap(
            "bus",
            "dish_antenna",
            elem_a=f"gimbal_ear_{tag}",
            elem_b="pivot_pin",
            reason="gimbal pivot pin captured through the mast ears (axisymmetric)",
        )

    parts = {p.name for p in object_model.parts}
    ctx.check(
        "two solar wings present",
        "wing_left" in parts and "wing_right" in parts,
        f"parts: {sorted(parts)}",
    )
    ctx.check("dish antenna part present", "dish_antenna" in parts, f"parts: {sorted(parts)}")

    # Panel count per wing.
    expected_panels = PANEL_BAYS * PANEL_COLS
    for side in ("left", "right"):
        wing = object_model.get_part(f"wing_{side}")
        n_panels = sum(1 for v in wing.visuals if v.name and v.name.startswith("panel_"))
        ctx.check(
            f"wing_{side} has {expected_panels} solar panels",
            n_panels == expected_panels,
            f"found {n_panels}",
        )
        n_grid = sum(1 for v in wing.visuals if v.name and v.name.startswith("grid_"))
        ctx.check(
            f"wing_{side} panels carry cell-grid lines",
            n_grid == expected_panels * 7,
            f"found {n_grid}",
        )

    # Joint types: two continuous solar array drives + one revolute dish gimbal.
    j_left = object_model.get_articulation("bus_to_wing_left")
    j_right = object_model.get_articulation("bus_to_wing_right")
    j_dish = object_model.get_articulation("bus_to_dish")
    ctx.check(
        "wing drives are CONTINUOUS",
        j_left.articulation_type == ArticulationType.CONTINUOUS
        and j_right.articulation_type == ArticulationType.CONTINUOUS,
        f"{j_left.articulation_type}, {j_right.articulation_type}",
    )
    ctx.check(
        "wing drives spin about the boom (X) axis",
        abs(j_left.axis[0]) > 0.99 and abs(j_right.axis[0]) > 0.99,
        f"{j_left.axis}, {j_right.axis}",
    )
    ctx.check(
        "dish gimbal is REVOLUTE",
        j_dish.articulation_type == ArticulationType.REVOLUTE,
        str(j_dish.articulation_type),
    )

    # Wings extend symmetrically along +/-X well past the bus.
    aabb_l = ctx.part_world_aabb("wing_left")
    aabb_r = ctx.part_world_aabb("wing_right")
    ctx.check(
        "left wing extends far along -X",
        aabb_l is not None and aabb_l[0][0] < -2.0,
        f"left aabb: {aabb_l}",
    )
    ctx.check(
        "right wing extends far along +X",
        aabb_r is not None and aabb_r[1][0] > 2.0,
        f"right aabb: {aabb_r}",
    )

    # Flat wing at q=0 is thin in Z; spinning the drive 90 deg sweeps the
    # panel width into Z, so the wing AABB Z extent must grow substantially.
    z0 = aabb_l[1][2] - aabb_l[0][2]
    ctx.check("wing is a thin flat array at rest", z0 < 0.20, f"z extent {z0:.3f}")
    with ctx.pose({j_left: pi / 2.0}):
        aabb90 = ctx.part_world_aabb("wing_left")
        z90 = aabb90[1][2] - aabb90[0][2]
        ctx.check(
            "solar array drive rotates the wing about its boom",
            z90 > 0.45,
            f"z extent at 90deg: {z90:.3f} (rest {z0:.3f})",
        )

    # Dish reflector spans its design diameter and the gimbal actually tilts it.
    dish_aabb0 = ctx.part_world_aabb("dish_antenna")
    dx0 = dish_aabb0[1][0] - dish_aabb0[0][0]
    ctx.check("dish reflector spans ~0.52 m", 0.45 < dx0 < 0.60, f"x extent {dx0:.3f}")
    c0 = tuple((dish_aabb0[0][i] + dish_aabb0[1][i]) / 2.0 for i in range(3))
    with ctx.pose({j_dish: 0.6}):
        dish_aabb1 = ctx.part_world_aabb("dish_antenna")
        c1 = tuple((dish_aabb1[0][i] + dish_aabb1[1][i]) / 2.0 for i in range(3))
        disp = sum((a - b) ** 2 for a, b in zip(c0, c1)) ** 0.5
        ctx.check(
            "dish gimbal tilts the reflector",
            disp > 0.03,
            f"center displacement {disp:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
