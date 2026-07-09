from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Part,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Wall-mounted widespread two-handle faucet, polished chrome.
#
# Layout (meters, Z up, +Y outward from wall):
#   - vertical wall backplate (root) at Y = 0, extending into -Y
#   - center spout at X = 0: round escutcheon on wall, tapered neck, wide
#     flat-topped waterfall spout reaching ~0.18 m outward (+Y), curving down
#   - valve assemblies at X = +/-0.15: round escutcheon, visible stem collar
#     ring, slim stem, and a lever handle that tilts forward-back about a
#     horizontal axis (REVOLUTE, axis along X)
#   - hot/cold cap disks (red/blue) as geometry on each lever handle
#
# Articulation: each lever handle is a revolute joint about its horizontal
# pivot axis (±45 deg forward-back tilt). Spout is fixed.
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve centers at +/-0.150 -> 0.30 m spread

# Wall backplate
WALL_W = 0.44
WALL_H = 0.14
WALL_D = 0.015

# Spout
ESC_SPOUT_R = 0.030
ESC_SPOUT_H = 0.006
SPOUT_NECK_R = 0.018
SPOUT_NECK_LEN = 0.030
SPOUT_WIDTH = 0.048

# Valve escutcheon and stem collar
ESC_VALVE_R = 0.026
ESC_VALVE_H = 0.005
COLLAR_R = 0.014
COLLAR_H = 0.014
STEM_R = 0.007
STEM_LEN = 0.042  # stem extends outward from collar

# Lever handle
LEVER_W = 0.014  # width (along X)
LEVER_T = 0.006  # thickness (along Y)
LEVER_LEN = 0.065  # length from pivot to tip (along Z at rest)
LEVER_CAP_R = 0.008  # base cap under lever
LEVER_CAP_H = 0.006

# Hot/cold indicator cap disks
DISK_R = 0.005
DISK_H = 0.004


def _waterfall_spout_wall() -> cq.Workplane:
    """Wide flat-topped spout extending outward (+Y), curving down.

    Side profile in the YZ plane, extruded across X. Root embedded inside
    the tapered neck (Y ~ 0.028) so the mesh connects with the neck.
    """
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.028, 0.012)
        .lineTo(0.028, -0.008)
        .spline(
            [(0.070, -0.014), (0.120, -0.030), (0.168, -0.058)],
            includeCurrent=True,
        )
        .lineTo(0.155, -0.064)
        .spline(
            [(0.120, -0.040), (0.070, -0.024), (0.028, -0.008)],
            includeCurrent=True,
        )
        .close()
        .extrude(SPOUT_WIDTH)
    )
    return profile.translate((-SPOUT_WIDTH / 2.0, 0.0, 0.0))


def _add_valve_assembly(part: Part, chrome: str, side_sign: float) -> None:
    """Escutcheon plate, visible stem collar, and slim stem for one valve.

    Part frame at the wall surface; escutcheon on wall, collar proud, stem
    extending outward along +Y.
    """
    # Escutcheon disk on wall face
    part.visual(
        Cylinder(radius=ESC_VALVE_R, length=ESC_VALVE_H),
        origin=Origin(xyz=(0.0, ESC_VALVE_H / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="escutcheon",
    )
    # Visible stem collar (decorative ring above escutcheon)
    collar_y = ESC_VALVE_H + COLLAR_H / 2.0
    part.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_H),
        origin=Origin(xyz=(0.0, collar_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="stem_collar",
    )
    # Slim stem extending outward
    stem_y0 = ESC_VALVE_H + COLLAR_H
    stem_yc = stem_y0 + STEM_LEN / 2.0
    part.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(xyz=(0.0, stem_yc, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="valve_stem",
    )


def _add_lever_handle(part: Part, chrome: str, hot_color: str) -> None:
    """Lever handle with base cap and hot/cold indicator disk.

    Part frame origin at the pivot point. The lever arm extends along +Z
    at rest (pointing up). Rotation about X tilts forward-back.
    """
    # Pivot cap (small cylinder at the base, sits on top of the stem)
    part.visual(
        Cylinder(radius=LEVER_CAP_R, length=LEVER_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, LEVER_CAP_H / 2.0)),
        material=chrome,
        name="lever_cap",
    )
    # Lever arm: flat bar extending upward from the cap
    lever_z0 = LEVER_CAP_H
    part.visual(
        Box((LEVER_W, LEVER_T, LEVER_LEN)),
        origin=Origin(xyz=(0.0, 0.0, lever_z0 + LEVER_LEN / 2.0)),
        material=chrome,
        name="lever_arm",
    )
    # Rounded tip sphere at the top of the lever
    part.visual(
        Sphere(radius=LEVER_W / 2.0),
        origin=Origin(xyz=(0.0, 0.0, lever_z0 + LEVER_LEN)),
        material=chrome,
        name="lever_tip",
    )
    # Hot/cold indicator cap disk at the lever tip (colored)
    part.visual(
        Cylinder(radius=DISK_R, height=DISK_H),
        origin=Origin(xyz=(0.0, 0.0, lever_z0 + LEVER_LEN + LEVER_W / 2.0 + DISK_H / 2.0)),
        material=hot_color,
        name="cap_disk",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    wall_mat = model.material("wall_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    hot_red = model.material("hot_red", rgba=(0.80, 0.12, 0.12, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.12, 0.20, 0.75, 1.0))

    # --- Wall backplate (root) ---
    wall = model.part("wall")
    wall.visual(
        Box((WALL_W, WALL_D, WALL_H)),
        origin=Origin(xyz=(0.0, -WALL_D / 2.0, 0.0)),  # front face at Y = 0
        material=wall_mat,
        name="wall_plate",
    )

    # --- Center spout body (fixed to wall) ---
    spout_body = model.part("spout_body")
    # Escutcheon disk
    spout_body.visual(
        Cylinder(radius=ESC_SPOUT_R, length=ESC_SPOUT_H),
        origin=Origin(xyz=(0.0, ESC_SPOUT_H / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome.name,
        name="spout_escutcheon",
    )
    # Tapered neck extending along +Y from the escutcheon
    # Build along Z, then rotate -90° about X to map Z -> +Y
    neck = (
        cq.Workplane("XY")
        .circle(SPOUT_NECK_R)
        .workplane(offset=SPOUT_NECK_LEN)
        .circle(SPOUT_NECK_R * 0.75)
        .loft()
    )
    neck = neck.rotate((0, 0, 0), (1, 0, 0), -90)
    neck_translated = neck.translate((0.0, ESC_SPOUT_H, 0.0))
    spout_body.visual(
        mesh_from_cadquery(neck_translated, "spout_neck"),
        material=chrome.name,
        name="spout_neck",
    )
    # Waterfall spout arm (root embedded inside the neck for connectivity)
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout_wall(), "waterfall_spout"),
        material=chrome.name,
        name="spout",
    )
    model.articulation(
        "wall_to_spout",
        ArticulationType.FIXED,
        parent=wall,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    # --- Valve assemblies and lever handles (left = -X / hot, right = +X / cold) ---
    for side, sx, color_mat, color_name in (
        ("left", -1.0, hot_red, "hot_red"),
        ("right", 1.0, cold_blue, "cold_blue"),
    ):
        valve = model.part(f"{side}_valve")
        _add_valve_assembly(valve, chrome.name, sx)
        model.articulation(
            f"wall_to_{side}_valve",
            ArticulationType.FIXED,
            parent=wall,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        _add_lever_handle(handle, color_name, color_name)
        # The handle pivot sits at the end of the valve stem
        pivot_y = ESC_VALVE_H + COLLAR_H + STEM_LEN
        model.articulation(
            f"{side}_handle_tilt",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, pivot_y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=3.0,
                lower=-math.pi / 4.0,
                upper=math.pi / 4.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall")
    spout_body = object_model.get_part("spout_body")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_tilt")
    j_right = object_model.get_articulation("right_handle_tilt")

    # --- Intentional overlaps: lever cap embeds slightly over stem end ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        ctx.allow_overlap(
            handle,
            valve,
            elem_a="lever_cap",
            elem_b="valve_stem",
            reason="Lever cap intentionally seats over the valve stem end.",
        )

    # --- All three chrome assemblies mounted on the wall, not floating ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            wall,
            axis="y",
            max_gap=0.002,
            max_penetration=0.002,
            name=f"{piece.name} escutcheon contacts wall face",
        )
        ctx.expect_within(
            piece,
            wall,
            axes="x",
            margin=0.005,
            name=f"{piece.name} mounted within wall plate width",
        )

    # --- Three-piece spread of about 0.30 m ---
    ctx.expect_origin_distance(
        left_handle,
        right_handle,
        axes="x",
        min_dist=0.28,
        max_dist=0.32,
        name="handle spread is about 0.30 m",
    )

    # --- Spout is wall-mounted (extends outward along +Y from wall) ---
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout extends outward from wall (positive Y reach ~0.18 m)",
        spout_aabb is not None and spout_aabb[1][1] >= 0.14,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip curves downward below the spout root",
        spout_aabb is not None and spout_aabb[0][2] < 0.01,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Visible stem collars on each valve ---
    for valve in (left_valve, right_valve):
        collar_aabb = ctx.part_element_world_aabb(valve, elem="stem_collar")
        ctx.check(
            f"{valve.name} has visible stem collar",
            collar_aabb is not None
            and (collar_aabb[1][1] - collar_aabb[0][1]) >= 0.010,
            details=f"collar aabb={collar_aabb}",
        )

    # --- Hot and cold cap disks exist as geometry ---
    for handle, expected_name in ((left_handle, "left"), (right_handle, "right")):
        disk_aabb = ctx.part_element_world_aabb(handle, elem="cap_disk")
        ctx.check(
            f"{expected_name} handle has hot/cold cap disk geometry",
            disk_aabb is not None
            and (disk_aabb[1][2] - disk_aabb[0][2]) >= 0.002,
            details=f"disk aabb={disk_aabb}",
        )

    # --- Lever handles: forward-back tilt articulation ---
    for joint, lo, hi in (
        (j_left, -math.pi / 4.0, math.pi / 4.0),
        (j_right, -math.pi / 4.0, math.pi / 4.0),
    ):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is ±45 deg forward-back",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower - lo) < 0.01
            and abs(lim.upper - hi) < 0.01,
        )
        ctx.check(
            f"{joint.name} axis is horizontal (X) for forward-back tilt",
            joint.axis is not None
            and abs(joint.axis[0]) > 0.9
            and abs(joint.axis[1]) < 0.1
            and abs(joint.axis[2]) < 0.1,
        )

    # --- Pose check: lever tips move vertically when tilted ---
    def _lever_tip_z(handle: Part) -> float | None:
        aabb = ctx.part_element_world_aabb(handle, elem="lever_tip")
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    rest_left_z = _lever_tip_z(left_handle)
    with ctx.pose({j_left: math.pi / 4.0}):
        posed_left_z = _lever_tip_z(left_handle)
    ctx.check(
        "left lever tip moves when tilted +45 deg",
        rest_left_z is not None
        and posed_left_z is not None
        and abs(posed_left_z - rest_left_z) > 0.01,
        details=f"rest_z={rest_left_z}, posed_z={posed_left_z}",
    )

    rest_right_z = _lever_tip_z(right_handle)
    with ctx.pose({j_right: -math.pi / 4.0}):
        posed_right_z = _lever_tip_z(right_handle)
    ctx.check(
        "right lever tip moves independently when tilted -45 deg",
        rest_right_z is not None
        and posed_right_z is not None
        and abs(posed_right_z - rest_right_z) > 0.01,
        details=f"rest_z={rest_right_z}, posed_z={posed_right_z}",
    )

    # --- At positive tilt, lever tip goes backward (toward wall, -Y) ---
    rest_left_y = None
    posed_left_y = None
    aabb_rest = ctx.part_element_world_aabb(left_handle, elem="lever_tip")
    if aabb_rest is not None:
        rest_left_y = (aabb_rest[0][1] + aabb_rest[1][1]) / 2.0
    with ctx.pose({j_left: math.pi / 4.0}):
        aabb_posed = ctx.part_element_world_aabb(left_handle, elem="lever_tip")
        if aabb_posed is not None:
            posed_left_y = (aabb_posed[0][1] + aabb_posed[1][1]) / 2.0
    ctx.check(
        "positive tilt moves lever tip toward wall (backward)",
        rest_left_y is not None
        and posed_left_y is not None
        and posed_left_y < rest_left_y - 0.005,
        details=f"rest_y={rest_left_y}, posed_y={posed_left_y}",
    )

    return ctx.report()


object_model = build_object_model()
