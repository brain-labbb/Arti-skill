from __future__ import annotations

# NVIDIA GeForce RTX 4090 Founders Edition graphics card.
#
# Canonical frame: the card lies flat, centered at the origin.
#   X: card length. PCIe bracket end at x=-0.152, tail end at x=+0.152.
#   Y: card height. Gold PCIe edge connector protrudes at -Y, GEFORCE RTX
#      spine rail at +Y.
#   Z: card thickness (3 slots, ~61 mm). Top cooler face at +Z, underside
#      at -Z.
#
# Identity feature: dual-axial FLOW-THROUGH cooler. One fan is set into the
# TOP face near the tail and blows out through +Z; the second fan is set into
# the UNDERSIDE near the bracket and blows out through -Z. The short FE PCB
# ends mid-card, so the tail zone is an open finned duct: the underside panel
# stops before the tail and the dark heatsink fins are visible straight
# through the cutout under the tail fan.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

import cadquery as cq

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

# Materials ------------------------------------------------------------------
GUNMETAL = Material("gunmetal_alloy", (0.37, 0.38, 0.40, 1.0))
DARK_PANEL = Material("dark_shroud_panel", (0.058, 0.058, 0.065, 1.0))
GLOSS_BLACK = Material("gloss_black_plate", (0.035, 0.035, 0.045, 1.0))
FIN_DARK = Material("heatsink_fin_dark", (0.14, 0.14, 0.15, 1.0))
FAN_BLACK = Material("fan_black", (0.045, 0.045, 0.055, 1.0))
LABEL_SILVER = Material("label_silver", (0.86, 0.86, 0.88, 1.0))
GOLD = Material("connector_gold", (0.83, 0.67, 0.22, 1.0))
PCB_BLACK = Material("pcb_black", (0.05, 0.06, 0.05, 1.0))
BRACKET_SILVER = Material("bracket_silver", (0.60, 0.61, 0.63, 1.0))
PORT_DARK = Material("port_dark", (0.03, 0.03, 0.035, 1.0))

# Core dimensions --------------------------------------------------------------
CARD_LEN = 0.304
CARD_HALF_LEN = CARD_LEN / 2.0  # 0.152
BODY_HALF_H = 0.058  # card body height without bracket/connector
BODY_HALF_T = 0.0315  # half thickness including proud trim
RAIL_W = 0.006

FAN_R = 0.0465
FAN_HUB_R = 0.0165
FAN_BLADES = 7
DUCT_INNER_R = 0.0500
DUCT_OUTER_R = 0.0535
BEZEL_OUTER_R = 0.0575

TAIL_FAN_X = 0.092
BRACKET_FAN_X = -0.092
TAIL_FAN_ZC = 0.0185
BRACKET_FAN_ZC = -0.0185

TOP_PANEL_Z0 = 0.0255
TOP_PANEL_Z1 = 0.0285
BOT_PANEL_Z0 = -0.0285
BOT_PANEL_Z1 = -0.0255
BOT_PANEL_X_MAX = 0.045  # underside panel stops here: open flow-through tail

DP_PORT_YS = (-0.042, -0.017, 0.008)
HDMI_PORT_Y = 0.033
PORT_ZC = 0.0065


def _plate_with_hole(width: float, height: float, thickness: float,
                     hole_x: float, hole_y: float, hole_r: float):
    plate = cq.Workplane("XY").rect(width, height).extrude(thickness)
    hole = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .center(hole_x, hole_y)
        .circle(hole_r)
        .extrude(thickness + 0.002)
    )
    return plate.cut(hole)


def _ring(outer_r: float, inner_r: float, height: float):
    outer = cq.Workplane("XY").circle(outer_r).extrude(height)
    inner = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(inner_r)
        .extrude(height + 0.002)
    )
    return outer.cut(inner)


def _polygon_plate(points, thickness: float):
    return cq.Workplane("XY").polyline(list(points)).close().extrude(thickness)


def _bar_between(p0, p1, width: float, z0: float, z1: float):
    """Box + Origin for an angled trim bar between two XY points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    length = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)
    cx = (p0[0] + p1[0]) / 2.0
    cy = (p0[1] + p1[1]) / 2.0
    return (
        Box((length, width, z1 - z0)),
        Origin(xyz=(cx, cy, (z0 + z1) / 2.0), rpy=(0.0, 0.0, angle)),
    )


def _fan_rotor_mesh(name: str):
    rotor = FanRotorGeometry(
        FAN_R,
        FAN_HUB_R,
        FAN_BLADES,
        thickness=0.013,
        blade_pitch_deg=32.0,
        blade_sweep_deg=26.0,
        blade=FanRotorBlade(shape="broad", camber=0.14, tip_pitch_deg=18.0),
        hub=FanRotorHub(style="flat"),
    )
    return mesh_from_geometry(rotor, ASSETS.mesh_path(f"{name}.obj"))


def _add_fan_bay(shroud, fan_x: float, side: int, tag: str) -> None:
    """Static duct ring, stator strut bars and motor boss for one fan bay.

    side=+1 -> fan set into the top face, side=-1 -> underside.
    """
    panel_inner_z = TOP_PANEL_Z0 if side > 0 else BOT_PANEL_Z1  # 0.0255 abs
    duct_z0 = 0.0040 * side
    duct_z1 = 0.0265 * side
    z_lo = min(duct_z0, duct_z1)
    shroud.visual(
        mesh_from_cadquery(
            _ring(DUCT_OUTER_R, DUCT_INNER_R, abs(duct_z1 - duct_z0)),
            f"{tag}_duct",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(fan_x, 0.0, z_lo)),
        material=DARK_PANEL,
        name=f"{tag}_duct",
    )
    strut_z0 = 0.0040 * side
    strut_z1 = 0.0090 * side
    strut_zc = (strut_z0 + strut_z1) / 2.0
    for i, ang in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0)):
        shroud.visual(
            Box((0.106, 0.007, abs(strut_z1 - strut_z0))),
            origin=Origin(xyz=(fan_x, 0.0, strut_zc), rpy=(0.0, 0.0, ang)),
            material=DARK_PANEL,
            name=f"{tag}_strut_bar_{i}",
        )
    boss_z0 = 0.0040 * side
    boss_z1 = 0.0165 * side
    shroud.visual(
        Cylinder(radius=0.0115, length=abs(boss_z1 - boss_z0)),
        origin=Origin(xyz=(fan_x, 0.0, (boss_z0 + boss_z1) / 2.0)),
        material=FAN_BLACK,
        name=f"{tag}_motor_boss",
    )
    # Gunmetal bezel ring proud of the panel face around the fan opening.
    bezel_h = 0.0035
    bezel_z0 = (abs(panel_inner_z) + 0.0025) * side  # 0.0280 abs, 0.5 mm embed
    z_lo = min(bezel_z0, bezel_z0 + bezel_h * side)
    shroud.visual(
        mesh_from_cadquery(
            _ring(BEZEL_OUTER_R, DUCT_INNER_R, bezel_h),
            f"{tag}_bezel",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(fan_x, 0.0, z_lo)),
        material=GUNMETAL,
        name=f"{tag}_bezel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="geforce_rtx_4090_founders_edition", assets=ASSETS)

    # ------------------------------------------------------------- shroud ----
    shroud = model.part("shroud")
    shroud.inertial = Inertial.from_geometry(
        Box((CARD_LEN, 2 * BODY_HALF_H, 2 * BODY_HALF_T)), mass=1.65
    )

    # Gunmetal frame rails and tail end wall.
    shroud.visual(
        Box((CARD_LEN, RAIL_W, 2 * BODY_HALF_T)),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material=GUNMETAL,
        name="side_rail_spine",
    )
    lower_rail = cq.Workplane("XY").rect(CARD_LEN, 2 * BODY_HALF_T).extrude(RAIL_W)
    conn_window = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .center(-0.0905, 0.017)
        .rect(0.101, 0.012)
        .extrude(RAIL_W + 0.002)
    )
    lower_rail = lower_rail.cut(conn_window)
    shroud.visual(
        mesh_from_cadquery(lower_rail, "side_rail_lower", assets=ASSETS),
        # local x -> world x, local y -> world z, local z -> world -y
        origin=Origin(xyz=(0.0, -0.052, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=GUNMETAL,
        name="side_rail_lower",
    )
    shroud.visual(
        Box((0.006, 2 * BODY_HALF_H, 2 * BODY_HALF_T)),
        origin=Origin(xyz=(0.149, 0.0, 0.0)),
        material=GUNMETAL,
        name="tail_end_wall",
    )

    # Dark face panels with circular fan cutouts.
    top_panel_w = 0.299  # x in [-0.152, +0.147]
    top_panel_cx = -0.0025
    shroud.visual(
        mesh_from_cadquery(
            _plate_with_hole(top_panel_w, 0.107, TOP_PANEL_Z1 - TOP_PANEL_Z0,
                             TAIL_FAN_X - top_panel_cx, 0.0, DUCT_INNER_R),
            "top_panel",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(top_panel_cx, 0.0, TOP_PANEL_Z0)),
        material=DARK_PANEL,
        name="top_panel",
    )
    bot_panel_w = BOT_PANEL_X_MAX + 0.152  # 0.197, x in [-0.152, +0.045]
    bot_panel_cx = (BOT_PANEL_X_MAX - 0.152) / 2.0  # -0.0535
    shroud.visual(
        mesh_from_cadquery(
            _plate_with_hole(bot_panel_w, 0.107, BOT_PANEL_Z1 - BOT_PANEL_Z0,
                             BRACKET_FAN_X - bot_panel_cx, 0.0, DUCT_INNER_R),
            "bottom_panel",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(bot_panel_cx, 0.0, BOT_PANEL_Z0)),
        material=DARK_PANEL,
        name="bottom_panel",
    )

    # Fan bays: duct ring + stator struts + motor boss + gunmetal bezel.
    _add_fan_bay(shroud, TAIL_FAN_X, +1, "tail_fan")
    _add_fan_bay(shroud, BRACKET_FAN_X, -1, "bracket_fan")

    # Heatsink fins: flow-through tail block (visible through the underside
    # cutout and through the tail fan) and a second block over the bracket fan.
    for i in range(18):
        x = 0.044 + 0.006 * i
        shroud.visual(
            Box((0.0015, 0.107, 0.0275)),
            origin=Origin(xyz=(x, 0.0, -0.01175)),  # z in [-0.0255, +0.0020]
            material=FIN_DARK,
            name=f"fin_tail_{i}",
        )
    for i in range(18):
        x = -0.146 + 0.006 * i
        shroud.visual(
            Box((0.0015, 0.107, 0.0135)),
            origin=Origin(xyz=(x, 0.0, 0.00775)),  # z in [+0.0010, +0.0145]
            material=FIN_DARK,
            name=f"fin_bracket_{i}",
        )

    # Glossy black center plates with chevron edges pointing at the fan rings.
    shroud.visual(
        mesh_from_cadquery(
            _polygon_plate(
                [(-0.146, -0.046), (0.000, -0.046), (0.028, 0.000),
                 (0.000, 0.046), (-0.146, 0.046)],
                0.0027,
            ),
            "top_center_plate",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0283)),
        material=GLOSS_BLACK,
        name="top_center_plate",
    )
    shroud.visual(
        mesh_from_cadquery(
            _polygon_plate(
                [(0.043, -0.046), (0.043, 0.046), (0.000, 0.046),
                 (-0.028, 0.000), (0.000, -0.046)],
                0.0027,
            ),
            "bottom_center_plate",
            assets=ASSETS,
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0305)),
        material=GLOSS_BLACK,
        name="bottom_center_plate",
    )

    # RTX 4090 labels and GEFORCE RTX side lettering strip.
    shroud.visual(
        Box((0.060, 0.013, 0.0016)),
        origin=Origin(xyz=(-0.058, 0.0, 0.03105)),
        material=LABEL_SILVER,
        name="rtx_4090_label_top",
    )
    shroud.visual(
        Box((0.032, 0.011, 0.0016)),
        origin=Origin(xyz=(0.020, 0.0, -0.03080)),
        material=LABEL_SILVER,
        name="rtx_4090_label_bottom",
    )
    shroud.visual(
        Box((0.072, 0.0016, 0.011)),
        origin=Origin(xyz=(0.072, 0.0585, 0.0)),
        material=LABEL_SILVER,
        name="geforce_rtx_lettering",
    )

    # Gunmetal X-motif trim: chevron arms meeting each fan ring, and angular
    # border bars framing the open flow-through cutout on the underside.
    for i, sy in enumerate((1.0, -1.0)):
        geom, org = _bar_between((-0.004, 0.052 * sy), (0.036, 0.0), 0.008,
                                 0.0283, 0.0320)
        shroud.visual(geom, origin=org, material=GUNMETAL,
                      name=f"top_chevron_trim_{i}")
        geom, org = _bar_between((0.004, 0.052 * sy), (-0.036, 0.0), 0.008,
                                 -0.0320, -0.0283)
        shroud.visual(geom, origin=org, material=GUNMETAL,
                      name=f"bottom_chevron_trim_{i}")
        geom, org = _bar_between((0.042, 0.050 * sy), (0.148, 0.014 * sy), 0.008,
                                 -0.0318, -0.0250)
        shroud.visual(geom, origin=org, material=GUNMETAL,
                      name=f"vent_border_trim_{i}")

    # ---------------------------------------------------------------- fans ---
    tail_fan = model.part("tail_fan")
    tail_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=FAN_R, length=0.016), mass=0.055
    )
    tail_fan.visual(
        _fan_rotor_mesh("tail_fan_rotor"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=FAN_BLACK,
        name="tail_fan_rotor",
    )

    bracket_fan = model.part("bracket_fan")
    bracket_fan.inertial = Inertial.from_geometry(
        Cylinder(radius=FAN_R, length=0.016), mass=0.055
    )
    bracket_fan.visual(
        _fan_rotor_mesh("bracket_fan_rotor"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=FAN_BLACK,
        name="bracket_fan_rotor",
    )

    # ----------------------------------------------------------------- pcb ---
    pcb = model.part("pcb")
    pcb.inertial = Inertial.from_geometry(Box((0.172, 0.108, 0.002)), mass=0.24)
    pcb.visual(
        Box((0.172, 0.108, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=PCB_BLACK,
        name="pcb_board",
    )
    # Gold PCIe x16 edge connector, split at the key notch, protruding below
    # the shroud's lower edge on the bracket half of the card.
    pcb.visual(
        Box((0.0115, 0.0145, 0.002)),
        origin=Origin(xyz=(-0.06525, -0.060, 0.0)),
        material=GOLD,
        name="edge_connector_key",
    )
    pcb.visual(
        Box((0.074, 0.0145, 0.002)),
        origin=Origin(xyz=(-0.0195, -0.060, 0.0)),
        material=GOLD,
        name="edge_connector_main",
    )

    # ------------------------------------------------------------- bracket ---
    bracket = model.part("bracket")
    bracket.inertial = Inertial.from_geometry(Box((0.002, 0.140, 0.061)), mass=0.09)
    plate = (
        cq.Workplane("XY")
        .polyline([(-0.076, -0.0305), (0.064, -0.0305), (0.064, 0.0305),
                   (-0.076, 0.0305)])
        .close()
        .extrude(0.002)
    )
    for y in DP_PORT_YS:
        cutter = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(y, PORT_ZC)
            .rect(0.019, 0.007)
            .extrude(0.004)
        )
        plate = plate.cut(cutter)
    hdmi_cutter = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .center(HDMI_PORT_Y, PORT_ZC)
        .rect(0.016, 0.006)
        .extrude(0.004)
    )
    plate = plate.cut(hdmi_cutter)
    bracket.visual(
        mesh_from_cadquery(plate, "bracket_plate", assets=ASSETS),
        # local x -> world y, local y -> world z, local z -> world x
        origin=Origin(xyz=(-0.0005, 0.0, 0.0), rpy=(math.pi / 2, 0.0, math.pi / 2)),
        material=BRACKET_SILVER,
        name="bracket_plate",
    )
    bracket.visual(
        Box((0.010, 0.003, 0.018)),
        origin=Origin(xyz=(-0.0055, 0.0650, 0.0)),
        material=BRACKET_SILVER,
        name="bracket_tab",
    )
    for i, y in enumerate(DP_PORT_YS):
        bracket.visual(
            Box((0.0115, 0.021, 0.009)),
            origin=Origin(xyz=(0.00675, y, PORT_ZC)),
            material=PORT_DARK,
            name=f"port_dp_{i}",
        )
    bracket.visual(
        Box((0.0115, 0.018, 0.009)),
        origin=Origin(xyz=(0.00675, HDMI_PORT_Y, PORT_ZC)),
        material=PORT_DARK,
        name="port_hdmi",
    )

    # ------------------------------------------------------- articulations ---
    model.articulation(
        "tail_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child=tail_fan,
        origin=Origin(xyz=(TAIL_FAN_X, 0.0, TAIL_FAN_ZC)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=180.0),
    )
    model.articulation(
        "bracket_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child=bracket_fan,
        # Flipped joint frame: this fan faces the underside, so its spin axis
        # points along world -Z, opposite the tail fan.
        origin=Origin(xyz=(BRACKET_FAN_X, 0.0, BRACKET_FAN_ZC),
                      rpy=(math.pi, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=180.0),
    )
    model.articulation(
        "pcb_mount",
        ArticulationType.FIXED,
        parent=shroud,
        child=pcb,
        origin=Origin(xyz=(-0.064, 0.0, 0.017)),
    )
    model.articulation(
        "bracket_mount",
        ArticulationType.FIXED,
        parent=shroud,
        child=bracket,
        origin=Origin(xyz=(-0.1525, 0.0, 0.0)),
    )

    return model


def _world_z_axis_after_rpy(rpy) -> tuple[float, float, float]:
    r, p, y = rpy
    sr, cr = math.sin(r), math.cos(r)
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)
    # Column 3 of Rz(y) @ Ry(p) @ Rx(r)
    return (
        sy * sr + cy * sp * cr,
        -cy * sr + sy * sp * cr,
        cp * cr,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    shroud = object_model.get_part("shroud")
    tail_fan = object_model.get_part("tail_fan")
    bracket_fan = object_model.get_part("bracket_fan")
    pcb = object_model.get_part("pcb")
    bracket = object_model.get_part("bracket")

    # Intentional local embeds ------------------------------------------------
    ctx.allow_overlap(
        "shroud", "tail_fan", elem_a="tail_fan_motor_boss", elem_b="tail_fan_rotor",
        reason="Rotor hub is captured on the static motor boss (shaft proxy).",
    )
    ctx.allow_overlap(
        "shroud", "bracket_fan", elem_a="bracket_fan_motor_boss",
        elem_b="bracket_fan_rotor",
        reason="Rotor hub is captured on the static motor boss (shaft proxy).",
    )
    ctx.allow_overlap(
        "shroud", "pcb", elem_a="side_rail_spine", elem_b="pcb_board",
        reason="PCB edge seats into the spine rail's internal card guide.",
    )
    ctx.allow_overlap(
        "shroud", "pcb", elem_a="side_rail_lower", elem_b="pcb_board",
        reason="PCB edge seats into the lower rail's internal card guide.",
    )
    ctx.allow_overlap(
        "shroud", "bracket", elem_a="side_rail_spine", elem_b="bracket_plate",
        reason="Bracket flange seats flush against the shroud nose.",
    )
    ctx.allow_overlap(
        "shroud", "bracket", elem_a="top_panel", elem_b="bracket_plate",
        reason="Bracket flange seats flush against the shroud nose.",
    )
    ctx.allow_overlap(
        "shroud", "bracket", elem_a="bottom_panel", elem_b="bracket_plate",
        reason="Bracket flange seats flush against the shroud nose.",
    )

    # Overall proportions: ~304 x ~116 x ~63 mm cooler body -------------------
    aabb = ctx.part_world_aabb(shroud)
    ctx.check("shroud_aabb_present", aabb is not None, "no shroud AABB")
    if aabb is not None:
        mins, maxs = aabb
        size = tuple(maxs[i] - mins[i] for i in range(3))
        ctx.check("card_length", 0.290 <= size[0] <= 0.320, f"size={size!r}")
        ctx.check("card_body_height", 0.110 <= size[1] <= 0.128, f"size={size!r}")
        ctx.check("card_thickness_3slot", 0.058 <= size[2] <= 0.070, f"size={size!r}")

    # Flow-through dual-fan layout: opposite faces, opposite ends -------------
    tail_aabb = ctx.part_world_aabb(tail_fan)
    brk_aabb = ctx.part_world_aabb(bracket_fan)
    ctx.check("fan_aabbs_present", tail_aabb is not None and brk_aabb is not None,
              "missing fan AABBs")
    if tail_aabb is not None and brk_aabb is not None:
        tail_c = [(tail_aabb[0][i] + tail_aabb[1][i]) / 2.0 for i in range(3)]
        brk_c = [(brk_aabb[0][i] + brk_aabb[1][i]) / 2.0 for i in range(3)]
        ctx.check("tail_fan_near_tail", tail_c[0] > 0.05, f"tail_c={tail_c!r}")
        ctx.check("bracket_fan_near_bracket", brk_c[0] < -0.05, f"brk_c={brk_c!r}")
        ctx.check("fans_at_opposite_ends", tail_c[0] - brk_c[0] > 0.15,
                  f"tail={tail_c[0]}, bracket={brk_c[0]}")
        ctx.check("tail_fan_in_top_face", 0.005 < tail_c[2] < 0.030,
                  f"tail_c={tail_c!r}")
        ctx.check("bracket_fan_in_underside", -0.030 < brk_c[2] < -0.005,
                  f"brk_c={brk_c!r}")

    # Spin axes point in OPPOSITE world directions ----------------------------
    tail_joint = object_model.get_articulation("tail_fan_spin")
    brk_joint = object_model.get_articulation("bracket_fan_spin")
    tail_axis = _world_z_axis_after_rpy(getattr(tail_joint.origin, "rpy", (0, 0, 0)))
    brk_axis = _world_z_axis_after_rpy(getattr(brk_joint.origin, "rpy", (0, 0, 0)))
    dot = sum(a * b for a, b in zip(tail_axis, brk_axis))
    ctx.check("fan_axes_opposite", dot < -0.99,
              f"tail_axis={tail_axis!r}, bracket_axis={brk_axis!r}, dot={dot}")

    # Rotors stay centered in their static ducts ------------------------------
    ctx.expect_within(tail_fan, shroud, axes="xy", inner_elem="tail_fan_rotor",
                      outer_elem="tail_fan_duct", margin=0.001,
                      name="tail rotor centered in duct")
    ctx.expect_within(bracket_fan, shroud, axes="xy",
                      inner_elem="bracket_fan_rotor",
                      outer_elem="bracket_fan_duct", margin=0.001,
                      name="bracket rotor centered in duct")
    ctx.expect_overlap(shroud, tail_fan, axes="z", elem_a="tail_fan_motor_boss",
                       elem_b="tail_fan_rotor", min_overlap=0.0005,
                       name="tail rotor hub captured on boss")
    ctx.expect_overlap(shroud, bracket_fan, axes="z",
                       elem_a="bracket_fan_motor_boss", elem_b="bracket_fan_rotor",
                       min_overlap=0.0005,
                       name="bracket rotor hub captured on boss")

    # Spinning each fan displaces the blade AABB center (odd blade count) -----
    for joint_name, fan in (("tail_fan_spin", tail_fan),
                            ("bracket_fan_spin", bracket_fan)):
        rest_pos = ctx.part_world_position(fan)
        rest_aabb = ctx.part_world_aabb(fan)
        rest_c = [(rest_aabb[0][i] + rest_aabb[1][i]) / 2.0 for i in range(3)]
        best_disp = 0.0
        z_extent_drift = 0.0
        origin_drift = 0.0
        for q in (math.pi / 7.0, math.pi / 5.0, math.pi / 3.0):
            with ctx.pose({joint_name: q}):
                posed_aabb = ctx.part_world_aabb(fan)
                posed_c = [(posed_aabb[0][i] + posed_aabb[1][i]) / 2.0
                           for i in range(3)]
                disp = math.hypot(posed_c[0] - rest_c[0], posed_c[1] - rest_c[1])
                best_disp = max(best_disp, disp)
                z_extent_drift = max(
                    z_extent_drift,
                    abs((posed_aabb[1][2] - posed_aabb[0][2])
                        - (rest_aabb[1][2] - rest_aabb[0][2])),
                )
                posed_pos = ctx.part_world_position(fan)
                origin_drift = max(origin_drift, math.dist(rest_pos, posed_pos))
        ctx.check(f"{joint_name}_displaces_blade_aabb_center", best_disp > 0.0006,
                  f"best_disp={best_disp}")
        ctx.check(f"{joint_name}_axis_preserves_thickness", z_extent_drift < 1e-5,
                  f"z_extent_drift={z_extent_drift}")
        ctx.check(f"{joint_name}_origin_fixed", origin_drift < 1e-9,
                  f"origin_drift={origin_drift}")

    # Flow-through tail: underside panel stops mid-card, fins fill the tail ---
    bp = ctx.part_element_world_aabb(shroud, elem="bottom_panel")
    ctx.check("bottom_panel_present", bp is not None, "no bottom_panel AABB")
    if bp is not None:
        ctx.check("underside_tail_cutout_open", bp[1][0] < 0.05,
                  f"bottom_panel x_max={bp[1][0]}")
    fin_first = ctx.part_element_world_aabb(shroud, elem="fin_tail_0")
    fin_last = ctx.part_element_world_aabb(shroud, elem="fin_tail_17")
    ctx.check("tail_fin_block_present",
              fin_first is not None and fin_last is not None, "missing tail fins")
    if fin_first is not None and fin_last is not None:
        ctx.check("tail_fins_span_cutout", fin_last[1][0] - fin_first[0][0] > 0.09,
                  f"fin span={fin_last[1][0] - fin_first[0][0]}")
        ctx.check("tail_fins_reach_underside_opening", fin_first[0][2] < -0.024,
                  f"fin z_min={fin_first[0][2]}")
    ctx.expect_overlap(shroud, tail_fan, axes="xy", elem_a="fin_tail_8",
                       elem_b="tail_fan_rotor", min_overlap=0.001,
                       name="fins visible through tail fan disc")

    # Bracket with 3 DisplayPort + 1 HDMI openings ----------------------------
    plate_aabb = ctx.part_element_world_aabb(bracket, elem="bracket_plate")
    ctx.check("bracket_plate_present", plate_aabb is not None, "no bracket plate")
    if plate_aabb is not None:
        ctx.check("bracket_at_front_end", plate_aabb[1][0] < -0.148,
                  f"plate x_max={plate_aabb[1][0]}")
        ctx.check("bracket_spans_three_slots",
                  plate_aabb[1][2] - plate_aabb[0][2] > 0.055,
                  f"plate z span={plate_aabb[1][2] - plate_aabb[0][2]}")
    port_centers = []
    for elem in ("port_dp_0", "port_dp_1", "port_dp_2", "port_hdmi"):
        pa = ctx.part_element_world_aabb(bracket, elem=elem)
        ctx.check(f"{elem}_present", pa is not None, f"missing {elem}")
        if pa is not None:
            port_centers.append((pa[0][1] + pa[1][1]) / 2.0)
    ctx.check("four_display_outputs", len(port_centers) == 4,
              f"count={len(port_centers)}")
    if len(port_centers) == 4:
        ctx.check("ports_in_a_row",
                  all(b - a > 0.01 for a, b in zip(port_centers, port_centers[1:])),
                  f"port y centers={port_centers!r}")

    # Gold PCIe edge connector protrudes below the shroud on the bracket half -
    conn = ctx.part_element_world_aabb(pcb, elem="edge_connector_main")
    ctx.check("edge_connector_present", conn is not None, "no edge connector")
    if conn is not None and aabb is not None:
        ctx.check("edge_connector_below_shroud", conn[0][1] < aabb[0][1] - 0.006,
                  f"conn y_min={conn[0][1]}, shroud y_min={aabb[0][1]}")
        ctx.check("edge_connector_on_bracket_half", conn[1][0] < -0.02,
                  f"conn x_max={conn[1][0]}")

    # Identity labels ----------------------------------------------------------
    label = ctx.part_element_world_aabb(shroud, elem="rtx_4090_label_top")
    ctx.check("rtx_4090_label_present", label is not None, "missing RTX 4090 label")
    lettering = ctx.part_element_world_aabb(shroud, elem="geforce_rtx_lettering")
    ctx.check("geforce_lettering_present", lettering is not None,
              "missing GEFORCE RTX lettering")
    if lettering is not None:
        ctx.check("geforce_lettering_on_spine_rail", lettering[1][1] > 0.058,
                  f"lettering y_max={lettering[1][1]}")

    return ctx.report()


object_model = build_object_model()
