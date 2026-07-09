from __future__ import annotations

"""Triple-faucet countertop beverage dispensing tower.

A brushed-stainless drip tray (0.45 x 0.32 x 0.03 m) with a perforated top
plate carries a flared-collar cylindrical column (~0.085 m dia, ~0.40 m tall).
Three chrome faucets fan out near the top of the column at -40/0/+40 degrees
around the front; each faucet carries a glossy black tapered tap handle on a
chrome lever collar. Each handle is an independent revolute joint about a
horizontal axis through its faucet body, perpendicular to that faucet's spout
direction: q=0 is upright/closed, q=+0.70 rad (~40 deg) pulls the handle
forward over the spout to dispense.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- constants
TRAY_X = 0.45  # tray width (left-right)
TRAY_Y = 0.32  # tray depth (front-back); front faces -Y
TRAY_H = 0.030

COLUMN_Y = 0.065  # column axis sits toward the rear of the tray
COLUMN_BASE_Z = TRAY_H - 0.0005  # flare seated 0.5 mm into the perforated plate
COLUMN_R = 0.0425
COLUMN_SHAFT_H = 0.402

FAN_ANGLE = math.radians(40.0)  # faucet yaw fan around the column front
FAUCET_REACH = 0.100  # faucet body / handle pivot distance from column axis
PIVOT_RISE = 0.027  # pivot height above the faucet centerline
HANDLE_LEN = 0.141  # pivot to grip tip
TAP_TRAVEL = 0.70  # rad, ~40 degrees forward

# (side, yaw, faucet centerline height in column-local z)
FAUCETS = (
    ("left", -FAN_ANGLE, 0.334),
    ("center", 0.0, 0.350),
    ("right", FAN_ANGLE, 0.334),
)


def _fan_dir(yaw: float) -> tuple[float, float]:
    """Horizontal unit vector of a faucet: yaw=0 points to the front (-Y)."""
    return (math.sin(yaw), -math.cos(yaw))


# ---------------------------------------------------------------- build
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_faucet_beverage_tower")

    steel = Material("brushed_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    well_steel = Material("tray_well_steel", rgba=(0.36, 0.38, 0.41, 1.0))
    chrome = Material("chrome", rgba=(0.80, 0.82, 0.86, 1.0))
    gloss_black = Material("gloss_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ---------------------------------------------------------- drip tray
    tray = model.part("drip_tray")

    rim = ExtrudeWithHolesGeometry(
        rounded_rect_profile(TRAY_X, TRAY_Y, 0.030),
        [rounded_rect_profile(TRAY_X - 0.024, TRAY_Y - 0.024, 0.022)],
        TRAY_H,
        center=False,
    )
    tray.visual(mesh_from_geometry(rim, "tray_rim_mesh"), material=steel, name="tray_rim")

    floor = ExtrudeGeometry.from_z0(
        rounded_rect_profile(TRAY_X - 0.020, TRAY_Y - 0.020, 0.024), 0.006
    )
    tray.visual(
        mesh_from_geometry(floor, "tray_floor_mesh"), material=well_steel, name="tray_floor"
    )

    plate = PerforatedPanelGeometry(
        (TRAY_X - 0.020, TRAY_Y - 0.020),
        0.004,
        hole_diameter=0.006,
        pitch=(0.015, 0.015),
        frame=0.014,
        corner_radius=0.020,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(plate, "tray_plate_mesh"),
        origin=Origin(xyz=(0.0, 0.0, TRAY_H - 0.002)),
        material=steel,
        name="perforated_plate",
    )

    # ---------------------------------------------------------- tower column
    column = model.part("beverage_column")
    column.visual(
        Cylinder(COLUMN_R, COLUMN_SHAFT_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_SHAFT_H / 2.0)),
        material=steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(COLUMN_R + 0.0015, 0.006),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_SHAFT_H + 0.003)),
        material=steel,
        name="column_cap",
    )
    flare = LatheGeometry(
        [
            (0.0, 0.0),
            (0.075, 0.0),
            (0.068, 0.006),
            (0.055, 0.020),
            (0.047, 0.040),
            (COLUMN_R, 0.060),
            (0.0, 0.060),
        ],
        segments=48,
    )
    column.visual(
        mesh_from_geometry(flare, "flare_collar_mesh"), material=steel, name="flare_collar"
    )

    model.articulation(
        "tray_to_column",
        ArticulationType.FIXED,
        parent=tray,
        child=column,
        origin=Origin(xyz=(0.0, COLUMN_Y, COLUMN_BASE_Z)),
    )

    # ---------------------------------------------------------- faucets + handles
    for side, yaw, z_f in FAUCETS:
        dx, dy = _fan_dir(yaw)

        # shank: horizontal chrome cylinder from the column wall outward
        column.visual(
            Cylinder(0.011, 0.060),
            origin=Origin(xyz=(dx * 0.0675, dy * 0.0675, z_f), rpy=(math.pi / 2.0, 0.0, yaw)),
            material=chrome,
            name=f"{side}_faucet_shank",
        )

        # faucet body: short horizontal chrome capsule
        body = (
            CapsuleGeometry(0.0145, 0.035)
            .rotate_x(math.pi / 2.0)
            .rotate_z(yaw)
            .translate(dx * FAUCET_REACH, dy * FAUCET_REACH, z_f)
        )
        column.visual(
            mesh_from_geometry(body, f"{side}_faucet_body_mesh"),
            material=chrome,
            name=f"{side}_faucet_body",
        )

        # downward-curved spout tube ending in a tapered nozzle
        spout_pts = [
            (dx * s, dy * s, z_f + dz)
            for s, dz in ((0.105, 0.0), (0.126, -0.003), (0.137, -0.016), (0.139, -0.033))
        ]
        tube = tube_from_spline_points(
            spout_pts, radius=0.0085, samples_per_segment=10, radial_segments=18
        )
        column.visual(
            mesh_from_geometry(tube, f"{side}_spout_mesh"),
            material=chrome,
            name=f"{side}_faucet_spout",
        )
        nozzle = LatheGeometry(
            [(0.0, 0.0), (0.0060, 0.0), (0.0092, 0.016), (0.0, 0.016)], segments=24
        ).translate(dx * 0.139, dy * 0.139, z_f - 0.049)
        column.visual(
            mesh_from_geometry(nozzle, f"{side}_nozzle_mesh"),
            material=chrome,
            name=f"{side}_spout_nozzle",
        )

        # bonnet: fixed chrome boss on top of the body that receives the handle
        column.visual(
            Cylinder(0.012, 0.017),
            origin=Origin(xyz=(dx * FAUCET_REACH, dy * FAUCET_REACH, z_f + 0.0185)),
            material=chrome,
            name=f"{side}_faucet_bonnet",
        )

        # ----- moving tap handle (lever collar + glossy black tapered grip)
        handle = model.part(f"{side}_tap_handle")
        handle.visual(
            Cylinder(0.005, 0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=chrome,
            name="pivot_stem",
        )
        collar = LatheGeometry(
            [(0.0, 0.0), (0.0065, 0.0), (0.0105, 0.020), (0.0, 0.020)], segments=24
        ).translate(0.0, 0.0, 0.001)
        handle.visual(
            mesh_from_geometry(collar, f"{side}_handle_collar_mesh"),
            material=chrome,
            name="lever_collar",
        )
        grip = LatheGeometry(
            [
                (0.0, 0.018),
                (0.0090, 0.0205),
                (0.0078, 0.045),
                (0.0108, 0.118),
                (0.0085, 0.136),
                (0.0, HANDLE_LEN),
            ],
            segments=28,
        )
        handle.visual(
            mesh_from_geometry(grip, f"{side}_handle_grip_mesh"),
            material=gloss_black,
            name="tap_grip",
        )

        # Revolute pivot through the faucet bonnet. The joint frame is yawed
        # with the faucet, so local +X is horizontal and perpendicular to the
        # spout; positive q tips the upright handle forward over the spout.
        model.articulation(
            f"{side}_tap_pivot",
            ArticulationType.REVOLUTE,
            parent=column,
            child=handle,
            origin=Origin(
                xyz=(dx * FAUCET_REACH, dy * FAUCET_REACH, z_f + PIVOT_RISE),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=TAP_TRAVEL),
        )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray = object_model.get_part("drip_tray")
    column = object_model.get_part("beverage_column")
    sides = ("left", "center", "right")
    handles = {s: object_model.get_part(f"{s}_tap_handle") for s in sides}
    pivots = {s: object_model.get_articulation(f"{s}_tap_pivot") for s in sides}
    yaws = {"left": -FAN_ANGLE, "center": 0.0, "right": FAN_ANGLE}

    # -------- intentional, scoped overlaps
    ctx.allow_overlap(
        column,
        tray,
        elem_a="flare_collar",
        elem_b="perforated_plate",
        reason="Flared column collar is seated 0.5 mm into the perforated drip-tray plate.",
    )
    for s in sides:
        ctx.allow_overlap(
            handles[s],
            column,
            elem_a="pivot_stem",
            elem_b=f"{s}_faucet_bonnet",
            reason="Captured handle pivot stem is intentionally nested inside the faucet bonnet.",
        )

    # -------- true physical scale and grounding
    tray_aabb = ctx.part_world_aabb(tray)
    ok = tray_aabb is not None
    ctx.check("drip tray AABB resolves", ok, details=f"{tray_aabb}")
    if tray_aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = tray_aabb
        ctx.check(
            "tray is ~0.45 x 0.32 x 0.03 m and grounded at z=0",
            0.44 <= (x1 - x0) <= 0.46
            and 0.31 <= (y1 - y0) <= 0.33
            and abs(z0) <= 0.002
            and 0.026 <= z1 <= 0.034,
            details=f"tray aabb={tray_aabb}",
        )

    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "column top is ~0.43-0.44 m above the floor",
        col_aabb is not None and 0.42 <= col_aabb[1][2] <= 0.45,
        details=f"column aabb={col_aabb}",
    )

    center_aabb = ctx.part_world_aabb(handles["center"])
    ctx.check(
        "upright center handle tops out near 0.55 m",
        center_aabb is not None and 0.52 <= center_aabb[1][2] <= 0.57,
        details=f"center handle aabb={center_aabb}",
    )

    # -------- everything stays over the drip tray footprint
    ctx.expect_within(column, tray, axes="xy", margin=0.001, name="column and faucets over tray")
    for s in sides:
        ctx.expect_within(handles[s], tray, axes="xy", margin=0.001, name=f"{s} handle over tray")

    # -------- mounting: collar seats on the tray, handles ride the bonnets
    ctx.expect_gap(
        column,
        tray,
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        name="column flare seats on the tray plate",
    )
    for s in sides:
        ctx.expect_contact(
            handles[s], column, name=f"{s} handle stem is captured in its faucet bonnet"
        )

    # -------- faucets fan out at distinct angular stations
    aabbs = {s: ctx.part_world_aabb(handles[s]) for s in sides}
    if all(a is not None for a in aabbs.values()):
        cx = {s: 0.5 * (aabbs[s][0][0] + aabbs[s][1][0]) for s in sides}
        ctx.check(
            "left/center/right handles occupy distinct fan stations",
            cx["left"] < -0.04 and abs(cx["center"]) < 0.02 and cx["right"] > 0.04,
            details=f"handle x-centers={cx}",
        )

    # -------- joint plan: three independent revolute taps, 0..~40 deg
    for s in sides:
        joint = pivots[s]
        ctx.check(
            f"{s} tap pivot is revolute",
            str(joint.articulation_type).lower().endswith("revolute"),
            details=f"type={joint.articulation_type}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{s} tap pivot travels 0 to ~40 deg",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower) <= 1e-9
            and 0.60 <= limits.upper <= 0.80,
            details=f"limits={limits}",
        )

    # -------- pulling each tap tips its handle forward over its own spout
    for s in sides:
        dx, dy = _fan_dir(yaws[s])
        rest = ctx.part_world_aabb(handles[s])
        with ctx.pose({pivots[s]: TAP_TRAVEL}):
            pulled = ctx.part_world_aabb(handles[s])
        if rest is None or pulled is None:
            ctx.fail(f"{s} handle pose AABBs resolve", "missing AABB")
            continue
        rest_c = (0.5 * (rest[0][0] + rest[1][0]), 0.5 * (rest[0][1] + rest[1][1]))
        pull_c = (0.5 * (pulled[0][0] + pulled[1][0]), 0.5 * (pulled[0][1] + pulled[1][1]))
        forward = (pull_c[0] - rest_c[0]) * dx + (pull_c[1] - rest_c[1]) * dy
        drop = rest[1][2] - pulled[1][2]
        ctx.check(
            f"pulled {s} handle swings forward along its spout and its tip drops",
            forward > 0.025 and drop > 0.015,
            details=f"forward={forward:.4f}, top drop={drop:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
