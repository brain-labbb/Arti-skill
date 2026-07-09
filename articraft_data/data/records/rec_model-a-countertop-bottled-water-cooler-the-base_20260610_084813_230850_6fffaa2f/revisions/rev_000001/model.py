from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Countertop bottled water cooler.
# Frame: +X = front (dispensing alcove side), +Y = left, +Z = up.
#
# Cabinet: 0.33 m deep (X) x 0.30 m wide (Y) x 0.38 m tall (Z), rounded
# vertical edges, front alcove recess (x 0.06..0.165, y +-0.10, z 0.04..0.21),
# glossy black control panel with red LED readout above the alcove, slotted
# vent grille on one side, and a funnel collar on top.
#
# Articulation: two independent paddle taps (revolute about Y at the paddle
# top, swinging rearward 0..30 deg) and a lift-out drip tray (prismatic Z).
# A translucent blue ribbed 19 L bottle is fixed inverted into the collar.
# ---------------------------------------------------------------------------

CAB_D = 0.33  # X extent
CAB_W = 0.30  # Y extent
CAB_H = 0.38  # Z extent

ALCOVE_BACK_X = 0.06
ALCOVE_FLOOR_Z = 0.04
ALCOVE_CEIL_Z = 0.21
ALCOVE_HALF_W = 0.10

TAP_X = 0.105
TAP_Y = 0.045
PADDLE_PIVOT = (0.093, TAP_Y, 0.181)

TRAY_SEAT = (0.111, 0.0, ALCOVE_FLOOR_Z)

COLLAR_BASE_Z = 0.378
BOTTLE_MOUTH_Z = 0.382
BOTTLE_HEIGHT = 0.35


def _bottle_geometry():
    """Translucent ribbed 5-gallon bottle, built upright then inverted."""
    rib_hi = 0.135
    rib_lo = 0.1265
    outer = [
        (0.020, 0.000),
        (0.105, 0.005),
        (0.128, 0.020),
        (rib_hi, 0.045),
    ]
    # Four ribbed ring grooves along the cylindrical body.
    z = 0.060
    for _ in range(4):
        outer.append((rib_hi, z))
        outer.append((rib_lo, z + 0.0075))
        outer.append((rib_hi, z + 0.015))
        z += 0.045
    outer += [
        (rib_hi, 0.225),
        (0.110, 0.262),
        (0.070, 0.300),
        (0.042, 0.318),
        (0.031, 0.328),
        (0.031, 0.350),
    ]
    inner = [
        (0.000, 0.004),
        (0.102, 0.009),
        (0.1245, 0.0235),
        (0.1315, 0.046),
        (0.1315, 0.224),
        (0.1065, 0.260),
        (0.0665, 0.298),
        (0.0385, 0.316),
        (0.027, 0.327),
        (0.027, 0.350),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64)
    # Invert: mouth (originally at z=0.35) goes to the bottom at z=0.
    geom.rotate_x(math.pi)
    geom.translate(0.0, 0.0, BOTTLE_HEIGHT)
    return geom


def _cabinet_shell():
    shell = (
        cq.Workplane("XY")
        .box(CAB_D, CAB_W, CAB_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )
    pocket = (
        cq.Workplane("XY")
        .box(0.14, 2.0 * ALCOVE_HALF_W, ALCOVE_CEIL_Z - ALCOVE_FLOOR_Z, centered=(True, True, False))
        .translate((ALCOVE_BACK_X + 0.07, 0.0, ALCOVE_FLOOR_Z))
    )
    return shell.cut(pocket)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_bottled_water_cooler")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.95, 1.0))
    body_gray = model.material("body_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    panel_black = model.material("panel_black", rgba=(0.05, 0.05, 0.06, 1.0))
    led_red = model.material("led_red", rgba=(0.95, 0.10, 0.08, 1.0))
    hot_red = model.material("hot_red", rgba=(0.85, 0.12, 0.10, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.15, 0.35, 0.85, 1.0))
    chrome = model.material("chrome", rgba=(0.85, 0.86, 0.88, 1.0))
    bottle_blue = model.material("bottle_blue", rgba=(0.25, 0.55, 0.92, 0.55))

    # ------------------------------------------------------------------ cabinet
    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "cabinet_shell"),
        material=body_white,
        name="cabinet_shell",
    )

    # Gray liner panel on the alcove back wall (embedded 3 mm for connection).
    cabinet.visual(
        Box((0.004, 0.196, 0.168)),
        origin=Origin(xyz=(0.059, 0.0, 0.124)),
        material=body_gray,
        name="alcove_liner",
    )

    # Glossy black control panel above the alcove, with red LED readout and
    # hot/cold indicator dots.
    cabinet.visual(
        Box((0.006, 0.20, 0.08)),
        origin=Origin(xyz=(0.1655, 0.0, 0.265)),
        material=panel_black,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.003, 0.056, 0.024)),
        origin=Origin(xyz=(0.1685, 0.0, 0.272)),
        material=led_red,
        name="led_display",
    )
    cabinet.visual(
        Box((0.003, 0.009, 0.009)),
        origin=Origin(xyz=(0.169, TAP_Y, 0.240)),
        material=hot_red,
        name="hot_indicator",
    )
    cabinet.visual(
        Box((0.003, 0.009, 0.009)),
        origin=Origin(xyz=(0.169, -TAP_Y, 0.240)),
        material=cold_blue,
        name="cold_indicator",
    )

    # Slotted ventilation grille on one side wall.
    grille_geom = SlotPatternPanelGeometry(
        (0.16, 0.13),
        0.004,
        slot_size=(0.12, 0.006),
        pitch=(0.14, 0.013),
        frame=0.010,
        corner_radius=0.004,
    )
    cabinet.visual(
        mesh_from_geometry(grille_geom, "side_vent_grille"),
        origin=Origin(xyz=(-0.01, -0.1505, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_gray,
        name="side_vent_grille",
    )

    # Funnel collar on top that receives the inverted bottle neck.
    collar_geom = LatheGeometry.from_shell_profiles(
        [(0.075, 0.0), (0.075, 0.020), (0.068, 0.026)],
        [(0.030, 0.0), (0.030, 0.014), (0.064, 0.026)],
        segments=64,
    )
    cabinet.visual(
        mesh_from_geometry(collar_geom, "bottle_collar"),
        origin=Origin(xyz=(0.0, 0.0, COLLAR_BASE_Z)),
        material=body_gray,
        name="bottle_collar",
    )

    # Tap valve bodies hanging from the alcove ceiling, with spouts and
    # red/blue marker rings.
    for tag, sign, ring_mat in (("hot", 1.0, hot_red), ("cold", -1.0, cold_blue)):
        y = sign * TAP_Y
        cabinet.visual(
            Cylinder(radius=0.013, length=0.034),
            origin=Origin(xyz=(TAP_X, y, 0.196)),
            material=body_gray,
            name=f"{tag}_tap_body",
        )
        cabinet.visual(
            Cylinder(radius=0.0135, length=0.006),
            origin=Origin(xyz=(TAP_X, y, 0.190)),
            material=ring_mat,
            name=f"{tag}_tap_ring",
        )
        cabinet.visual(
            Cylinder(radius=0.0055, length=0.018),
            origin=Origin(xyz=(TAP_X, y, 0.172)),
            material=chrome,
            name=f"{tag}_tap_spout",
        )

    # ------------------------------------------------------------------ paddles
    for tag, sign, plate_mat in (("hot", 1.0, hot_red), ("cold", -1.0, cold_blue)):
        paddle = model.part(f"{tag}_paddle")
        # Hinge barrel along Y at the paddle top, captured in the tap body.
        paddle.visual(
            Cylinder(radius=0.0045, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_gray,
            name="hinge_barrel",
        )
        paddle.visual(
            Box((0.004, 0.026, 0.050)),
            origin=Origin(xyz=(0.0, 0.0, -0.0275)),
            material=plate_mat,
            name="paddle_plate",
        )
        model.articulation(
            f"{tag}_paddle_hinge",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=paddle,
            origin=Origin(xyz=(PADDLE_PIVOT[0], sign * TAP_Y, PADDLE_PIVOT[2])),
            # Paddle hangs along -Z; positive q about +Y swings the free
            # bottom edge rearward (-X), as a cup pushes it.
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=4.0, lower=0.0, upper=math.radians(30.0)
            ),
        )

    # ----------------------------------------------------------------- drip tray
    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.095, 0.165, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=body_gray,
        name="tray_base",
    )
    drip_tray.visual(
        Box((0.004, 0.165, 0.014)),
        origin=Origin(xyz=(0.0455, 0.0, 0.007)),
        material=body_gray,
        name="tray_wall_front",
    )
    drip_tray.visual(
        Box((0.004, 0.165, 0.014)),
        origin=Origin(xyz=(-0.0455, 0.0, 0.007)),
        material=body_gray,
        name="tray_wall_rear",
    )
    drip_tray.visual(
        Box((0.095, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.0805, 0.007)),
        material=body_gray,
        name="tray_wall_0",
    )
    drip_tray.visual(
        Box((0.095, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.0805, 0.007)),
        material=body_gray,
        name="tray_wall_1",
    )
    tray_grille_geom = PerforatedPanelGeometry(
        (0.090, 0.160),
        0.003,
        hole_diameter=0.0045,
        pitch=0.009,
        frame=0.006,
        stagger=True,
    )
    drip_tray.visual(
        mesh_from_geometry(tray_grille_geom, "tray_grille"),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_gray,
        name="tray_grille",
    )
    model.articulation(
        "drip_tray_lift",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drip_tray,
        origin=Origin(xyz=TRAY_SEAT),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.2, lower=0.0, upper=0.03),
    )

    # ------------------------------------------------------------------- bottle
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_bottle_geometry(), "bottle_shell"),
        material=bottle_blue,
        name="bottle_shell",
    )
    model.articulation(
        "cabinet_to_bottle",
        ArticulationType.FIXED,
        parent=cabinet,
        child=bottle,
        origin=Origin(xyz=(0.0, 0.0, BOTTLE_MOUTH_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    bottle = object_model.get_part("bottle")
    drip_tray = object_model.get_part("drip_tray")
    hot_paddle = object_model.get_part("hot_paddle")
    cold_paddle = object_model.get_part("cold_paddle")
    hot_hinge = object_model.get_articulation("hot_paddle_hinge")
    cold_hinge = object_model.get_articulation("cold_paddle_hinge")
    tray_lift = object_model.get_articulation("drip_tray_lift")

    # Intentional local embeds.
    ctx.allow_overlap(
        hot_paddle,
        cabinet,
        elem_a="hinge_barrel",
        elem_b="hot_tap_body",
        reason="Paddle hinge barrel is intentionally captured inside the tap valve body boss.",
    )
    ctx.allow_overlap(
        cold_paddle,
        cabinet,
        elem_a="hinge_barrel",
        elem_b="cold_tap_body",
        reason="Paddle hinge barrel is intentionally captured inside the tap valve body boss.",
    )
    ctx.allow_overlap(
        bottle,
        cabinet,
        elem_a="bottle_shell",
        elem_b="bottle_collar",
        reason="Inverted bottle neck seats with a slight press fit into the loading collar bore.",
    )

    # ---- cabinet proportions -------------------------------------------------
    cab_aabb = ctx.part_world_aabb(cabinet)
    ctx.check(
        "cabinet footprint ~0.33 x 0.30 m",
        cab_aabb is not None
        and abs((cab_aabb[1][0] - cab_aabb[0][0]) - CAB_D) < 0.012
        and abs((cab_aabb[1][1] - cab_aabb[0][1]) - CAB_W) < 0.012,
        details=f"cabinet aabb={cab_aabb}",
    )

    # ---- bottle mounted inverted into the collar -----------------------------
    ctx.expect_origin_distance(
        bottle, cabinet, axes="xy", max_dist=0.005, name="bottle centered over collar"
    )
    ctx.expect_contact(
        bottle,
        cabinet,
        elem_a="bottle_shell",
        elem_b="bottle_collar",
        name="bottle neck seated in collar",
    )
    collar_aabb = ctx.part_element_world_aabb(cabinet, elem="bottle_collar")
    bottle_aabb = ctx.part_world_aabb(bottle)
    ctx.check(
        "bottle neck inserted below collar rim",
        collar_aabb is not None
        and bottle_aabb is not None
        and bottle_aabb[0][2] < collar_aabb[1][2] - 0.010
        and bottle_aabb[0][2] > CAB_H - 0.01,
        details=f"bottle min z={None if bottle_aabb is None else bottle_aabb[0][2]}, collar={collar_aabb}",
    )
    ctx.check(
        "total assembly height roughly 0.75 m",
        bottle_aabb is not None and 0.70 <= bottle_aabb[1][2] <= 0.78,
        details=f"bottle aabb={bottle_aabb}",
    )
    ctx.check(
        "bottle diameter ~0.27 m and height ~0.35 m",
        bottle_aabb is not None
        and 0.25 <= bottle_aabb[1][0] - bottle_aabb[0][0] <= 0.29
        and 0.33 <= bottle_aabb[1][2] - bottle_aabb[0][2] <= 0.37,
        details=f"bottle aabb={bottle_aabb}",
    )

    # ---- taps and paddles inside the alcove ----------------------------------
    for paddle, tag in ((hot_paddle, "hot"), (cold_paddle, "cold")):
        ctx.expect_contact(
            paddle,
            cabinet,
            elem_a="hinge_barrel",
            elem_b=f"{tag}_tap_body",
            name=f"{tag} paddle hinged to its tap body",
        )
        aabb = ctx.part_world_aabb(paddle)
        ctx.check(
            f"{tag} paddle recessed inside the alcove",
            aabb is not None
            and aabb[0][0] > ALCOVE_BACK_X
            and aabb[1][0] < CAB_D / 2.0
            and aabb[0][2] > ALCOVE_FLOOR_Z
            and aabb[1][2] < ALCOVE_CEIL_Z,
            details=f"{tag} paddle aabb={aabb}",
        )
    ctx.expect_origin_distance(
        hot_paddle, cold_paddle, axes="y", min_dist=0.06, name="hot and cold taps separated"
    )
    ctx.expect_gap(
        cabinet,
        drip_tray,
        axis="z",
        positive_elem="hot_tap_spout",
        min_gap=0.05,
        name="hot spout clears the drip tray for a cup",
    )
    ctx.expect_gap(
        cabinet,
        drip_tray,
        axis="z",
        positive_elem="cold_tap_spout",
        min_gap=0.05,
        name="cold spout clears the drip tray for a cup",
    )

    # ---- paddle swing: rearward about a horizontal Y axis at the paddle top --
    for paddle, hinge, tag in ((hot_paddle, hot_hinge, "hot"), (cold_paddle, cold_hinge, "cold")):
        rest = ctx.part_world_aabb(paddle)
        upper = hinge.motion_limits.upper if hinge.motion_limits is not None else 0.0
        with ctx.pose({hinge: upper}):
            swung = ctx.part_world_aabb(paddle)
            ctx.check(
                f"{tag} paddle swings rearward at full press",
                rest is not None
                and swung is not None
                and swung[0][0] < rest[0][0] - 0.015,
                details=f"rest={rest}, swung={swung}",
            )
            ctx.check(
                f"{tag} paddle stays clear of alcove back wall when pressed",
                swung is not None and swung[0][0] > ALCOVE_BACK_X + 0.001,
                details=f"swung={swung}",
            )

    # ---- lift-out drip tray ---------------------------------------------------
    tray_aabb = ctx.part_world_aabb(drip_tray)
    ctx.check(
        "drip tray rests on the alcove floor",
        tray_aabb is not None and ALCOVE_FLOOR_Z - 0.001 <= tray_aabb[0][2] <= ALCOVE_FLOOR_Z + 0.003,
        details=f"tray aabb={tray_aabb}",
    )
    ctx.check(
        "drip tray fits inside the alcove",
        tray_aabb is not None
        and tray_aabb[0][0] > ALCOVE_BACK_X
        and tray_aabb[1][0] < CAB_D / 2.0
        and tray_aabb[0][1] > -ALCOVE_HALF_W
        and tray_aabb[1][1] < ALCOVE_HALF_W,
        details=f"tray aabb={tray_aabb}",
    )
    tray_upper = tray_lift.motion_limits.upper if tray_lift.motion_limits is not None else 0.0
    with ctx.pose({tray_lift: tray_upper}):
        lifted = ctx.part_world_aabb(drip_tray)
        ctx.check(
            "drip tray lifts out vertically",
            tray_aabb is not None
            and lifted is not None
            and lifted[0][2] > tray_aabb[0][2] + 0.025,
            details=f"seated={tray_aabb}, lifted={lifted}",
        )
    tray_grille = drip_tray.get_visual("tray_grille")
    ctx.check(
        "drip tray has a grille at its floor",
        tray_grille is not None,
        details="missing tray_grille visual",
    )

    # ---- control panel, LED readout, side vent grille ------------------------
    panel_aabb = ctx.part_element_world_aabb(cabinet, elem="control_panel")
    led_aabb = ctx.part_element_world_aabb(cabinet, elem="led_display")
    ctx.check(
        "control panel sits on the front face above the alcove",
        panel_aabb is not None
        and panel_aabb[0][2] >= ALCOVE_CEIL_Z
        and panel_aabb[1][0] > CAB_D / 2.0,
        details=f"panel aabb={panel_aabb}",
    )
    ctx.check(
        "red LED readout is set into the control panel",
        panel_aabb is not None
        and led_aabb is not None
        and led_aabb[1][0] >= panel_aabb[1][0] - 1e-4
        and led_aabb[0][2] > panel_aabb[0][2]
        and led_aabb[1][2] < panel_aabb[1][2],
        details=f"panel={panel_aabb}, led={led_aabb}",
    )
    grille_aabb = ctx.part_element_world_aabb(cabinet, elem="side_vent_grille")
    ctx.check(
        "ventilation grille sits on the side wall",
        grille_aabb is not None and grille_aabb[0][1] <= -(CAB_W / 2.0) + 0.003,
        details=f"grille aabb={grille_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
