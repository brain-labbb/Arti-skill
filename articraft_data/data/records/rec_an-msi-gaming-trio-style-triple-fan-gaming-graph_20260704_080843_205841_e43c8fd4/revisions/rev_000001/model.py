from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
PCB_BLACK = Material("pcb_black", (0.05, 0.05, 0.06, 1.0))
SHROUD_BLACK = Material("shroud_black", (0.10, 0.10, 0.11, 1.0))
GUNMETAL = Material("gunmetal_gray", (0.38, 0.39, 0.42, 1.0))
FIN_SILVER = Material("fin_silver", (0.72, 0.73, 0.75, 1.0))
NICKEL = Material("nickel_pipe", (0.62, 0.63, 0.65, 1.0))
BLADE_BLACK = Material("blade_black", (0.07, 0.07, 0.08, 1.0))
BADGE_RED = Material("badge_red", (0.72, 0.07, 0.10, 1.0))
BADGE_BLACK = Material("badge_black", (0.03, 0.03, 0.04, 1.0))
EMBLEM_WHITE = Material("emblem_white", (0.95, 0.95, 0.95, 1.0))
DIFFUSER_WHITE = Material("diffuser_white", (0.93, 0.94, 0.96, 0.85))
BRACKET_STEEL = Material("bracket_steel", (0.60, 0.61, 0.64, 1.0))
PORT_DARK = Material("port_dark", (0.14, 0.14, 0.16, 1.0))
GOLD = Material("connector_gold", (0.83, 0.66, 0.20, 1.0))

# ---------------------------------------------------------------------------
# Global layout (meters)
# X = card length (bracket at -X end), Y = card height (edge connector at -Y),
# Z = card thickness (fans face +Z, PCB at the bottom).
# ---------------------------------------------------------------------------
CARD_LEN = 0.320
CARD_H = 0.130
PCB_LEN = 0.300
PCB_H = 0.115
PCB_T = 0.0018

FAN_CENTERS_X = (0.056, 0.160, 0.264)
FAN_CY = 0.065
FAN_R = 0.047
FAN_HUB_R = 0.015
FAN_THICK = 0.012
FAN_CZ = 0.0415
HOLE_R = 0.0505

FIN_Z0 = 0.0035
FIN_Z1 = 0.0345
FIN_Y0 = 0.011
FIN_Y1 = 0.104

PLATE_Z0 = 0.046
PLATE_T = 0.006


def _polygon_plate(points, thickness: float):
    wp = cq.Workplane("XY").polyline(points).close().extrude(thickness)
    return wp


def _build_shroud_plate_mesh():
    outline = [
        (0.012, 0.0),
        (0.308, 0.0),
        (0.320, 0.012),
        (0.320, 0.118),
        (0.308, 0.130),
        (0.012, 0.130),
        (0.0, 0.118),
        (0.0, 0.012),
    ]
    plate = _polygon_plate(outline, PLATE_T)
    for wx0, wx1 in ((0.0985, 0.1195), (0.2015, 0.2225)):
        window = (
            cq.Workplane("XY")
            .workplane(offset=-0.002)
            .center((wx0 + wx1) / 2, 0.065)
            .rect(wx1 - wx0, 0.074)
            .extrude(PLATE_T + 0.004)
        )
        plate = plate.cut(window)
    for cx in FAN_CENTERS_X:
        cutter = (
            cq.Workplane("XY")
            .workplane(offset=-0.002)
            .center(cx, FAN_CY)
            .circle(HOLE_R)
            .extrude(PLATE_T + 0.004)
        )
        plate = plate.cut(cutter)
    return mesh_from_cadquery(plate, ASSETS.mesh_path("shroud_top_plate.obj"))


def _build_bracket_mesh():
    # Authored flat in local XY: local x spans the card-height direction,
    # local y spans the card-thickness direction. Rotated up at attach time.
    outline = [
        (-0.008, 0.004),
        (-0.004, 0.0),
        (0.116, 0.0),
        (0.120, 0.004),
        (0.120, 0.038),
        (0.116, 0.042),
        (-0.004, 0.042),
        (-0.008, 0.038),
    ]
    plate = _polygon_plate(outline, 0.0015)
    openings = [
        (0.022, 0.009, 0.017, 0.0065),  # DisplayPort
        (0.046, 0.009, 0.017, 0.0065),  # DisplayPort
        (0.070, 0.009, 0.017, 0.0065),  # DisplayPort
        (0.093, 0.009, 0.015, 0.0060),  # HDMI
        (0.056, 0.020, 0.080, 0.0035),  # vent slot
        (0.056, 0.027, 0.080, 0.0035),  # vent slot
        (0.056, 0.034, 0.080, 0.0035),  # vent slot
    ]
    for (u, v, w, h) in openings:
        cutter = (
            cq.Workplane("XY")
            .workplane(offset=-0.001)
            .center(u, v)
            .rect(w, h)
            .extrude(0.004)
        )
        plate = plate.cut(cutter)
    return mesh_from_cadquery(plate, ASSETS.mesh_path("io_bracket.obj"))


def _build_fan_rotor_mesh():
    rotor = FanRotorGeometry(
        FAN_R,
        FAN_HUB_R,
        14,
        thickness=FAN_THICK,
        blade_pitch_deg=34.0,
        blade_sweep_deg=36.0,
        blade_root_chord=0.015,
        blade_tip_chord=0.023,
        blade=FanRotorBlade(
            shape="scimitar", camber=0.22, tip_pitch_deg=18.0, tip_clearance=0.0025
        ),
        hub=FanRotorHub(style="flat"),
    )
    return mesh_from_geometry(rotor, ASSETS.mesh_path("fan_rotor.obj"))


def _add_board(model: ArticulatedObject):
    board = model.part("board")
    board.inertial = Inertial.from_geometry(
        Box((CARD_LEN, CARD_H, 0.055)),
        mass=1.1,
        origin=Origin(xyz=(0.16, 0.06, 0.02)),
    )

    # PCB
    board.visual(
        Box((PCB_LEN, PCB_H, PCB_T)),
        origin=Origin(xyz=(PCB_LEN / 2, PCB_H / 2, PCB_T / 2)),
        material=PCB_BLACK,
        name="pcb_board",
    )

    # Gold PCIe x16 edge connector below the PCB bottom edge (short + long key).
    board.visual(
        Box((0.0115, 0.011, 0.0016)),
        origin=Origin(xyz=(0.0608, -0.0045, 0.0009)),
        material=GOLD,
        name="pcie_connector_short_key",
    )
    board.visual(
        Box((0.1565, 0.011, 0.0016)),
        origin=Origin(xyz=(0.1468, -0.0045, 0.0009)),
        material=GOLD,
        name="pcie_connector_long_key",
    )

    # Copper/aluminum cold plate under the fin stack.
    board.visual(
        Box((0.276, FIN_Y1 - FIN_Y0, 0.0025)),
        origin=Origin(xyz=(0.150, (FIN_Y0 + FIN_Y1) / 2, 0.00275)),
        material=NICKEL,
        name="heatsink_baseplate",
    )

    # Silver heatsink fin stack, fins run along the card-height direction.
    fin_pitch = 0.0075
    fin_count = 37
    x0 = 0.015
    for i in range(fin_count):
        x = x0 + i * fin_pitch
        board.visual(
            Box((0.0012, FIN_Y1 - FIN_Y0, FIN_Z1 - FIN_Z0)),
            origin=Origin(xyz=(x, (FIN_Y0 + FIN_Y1) / 2, (FIN_Z0 + FIN_Z1) / 2)),
            material=FIN_SILVER,
            name=f"fin_{i:02d}",
        )

    # Nickel heatpipes peeking out along the lower long edge of the fin stack.
    for k, z in enumerate((0.007, 0.014, 0.021)):
        board.visual(
            Cylinder(radius=0.003, length=0.26),
            origin=Origin(xyz=(0.150, 0.009, z), rpy=(0.0, math.pi / 2, 0.0)),
            material=NICKEL,
            name=f"heatpipe_{k}",
        )

    # Fan motor pods embedded into the fin stack under each rotor hub.
    for k, cx in enumerate(FAN_CENTERS_X):
        board.visual(
            Cylinder(radius=0.016, length=0.009),
            origin=Origin(xyz=(cx, FAN_CY, 0.0325)),
            material=BADGE_BLACK,
            name=f"fan_motor_pod_{k}",
        )

    # 8-pin power connector block on the top edge of the PCB.
    board.visual(
        Box((0.040, 0.011, 0.0105)),
        origin=Origin(xyz=(0.240, 0.1095, 0.00675)),
        material=PORT_DARK,
        name="power_connector_8pin",
    )

    # I/O bracket (silver slot plate with port openings and vent slots).
    bracket_mesh = _build_bracket_mesh()
    board.visual(
        bracket_mesh,
        origin=Origin(xyz=(-0.0035, 0.0, 0.0), rpy=(math.pi / 2, 0.0, math.pi / 2)),
        material=BRACKET_STEEL,
        name="io_bracket_plate",
    )
    # Bracket fastening tabs tie the plate to the PCB.
    board.visual(
        Box((0.0065, 0.006, 0.004)),
        origin=Origin(xyz=(0.00075, 0.007, 0.0035)),
        material=BRACKET_STEEL,
        name="bracket_tab_lower",
    )
    board.visual(
        Box((0.0065, 0.006, 0.004)),
        origin=Origin(xyz=(0.00075, 0.105, 0.0035)),
        material=BRACKET_STEEL,
        name="bracket_tab_upper",
    )

    # Display outputs behind the bracket openings (3x DP + 1x HDMI).
    for k, yc in enumerate((0.022, 0.046, 0.070)):
        board.visual(
            Box((0.0155, 0.016, 0.0105)),
            origin=Origin(xyz=(0.00625, yc, 0.00675)),
            material=PORT_DARK,
            name=f"display_port_{k}",
        )
    board.visual(
        Box((0.0155, 0.014, 0.0105)),
        origin=Origin(xyz=(0.00625, 0.093, 0.00675)),
        material=PORT_DARK,
        name="hdmi_port",
    )
    return board


def _add_shroud(model: ArticulatedObject):
    shroud = model.part("shroud")
    shroud.inertial = Inertial.from_geometry(
        Box((CARD_LEN, CARD_H, 0.035)),
        mass=0.35,
        origin=Origin(xyz=(0.16, 0.065, 0.04)),
    )

    plate_mesh = _build_shroud_plate_mesh()
    shroud.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, PLATE_Z0)),
        material=SHROUD_BLACK,
        name="shroud_top_plate",
    )

    # Perimeter skirt walls hanging below the top plate.
    wall_z = (0.012 + 0.047) / 2
    wall_h = 0.035
    shroud.visual(
        Box((CARD_LEN, 0.005, wall_h)),
        origin=Origin(xyz=(0.160, 0.0025, wall_z)),
        material=SHROUD_BLACK,
        name="skirt_wall_connector_edge",
    )
    shroud.visual(
        Box((CARD_LEN, 0.005, wall_h)),
        origin=Origin(xyz=(0.160, 0.1275, wall_z)),
        material=SHROUD_BLACK,
        name="skirt_wall_top_edge",
    )
    shroud.visual(
        Box((0.005, 0.122, 0.033)),
        origin=Origin(xyz=(0.0025, 0.065, 0.0305)),
        material=SHROUD_BLACK,
        name="skirt_wall_bracket_end",
    )
    shroud.visual(
        Box((0.005, 0.122, 0.033)),
        origin=Origin(xyz=(0.3175, 0.065, 0.0305)),
        material=SHROUD_BLACK,
        name="skirt_wall_tail_end",
    )

    # Angular gunmetal accent panels on the shroud face.
    panels = {
        "accent_strap_mid_front": [
            (0.100, 0.004),
            (0.116, 0.004),
            (0.132, 0.118),
            (0.116, 0.118),
        ],
        "accent_strap_mid_rear": [
            (0.204, 0.004),
            (0.220, 0.004),
            (0.204, 0.118),
            (0.188, 0.118),
        ],
        "accent_wedge_bracket_lower": [
            (0.004, 0.004),
            (0.048, 0.004),
            (0.014, 0.048),
            (0.004, 0.048),
        ],
        "accent_wedge_bracket_upper": [
            (0.004, 0.118),
            (0.048, 0.118),
            (0.014, 0.074),
            (0.004, 0.074),
        ],
        "accent_wedge_tail_lower": [
            (0.316, 0.004),
            (0.272, 0.004),
            (0.306, 0.048),
            (0.316, 0.048),
        ],
        "accent_wedge_tail_upper": [
            (0.316, 0.118),
            (0.272, 0.118),
            (0.306, 0.074),
            (0.316, 0.074),
        ],
    }
    for pname, pts in panels.items():
        mesh = mesh_from_cadquery(
            _polygon_plate(pts, 0.0025), ASSETS.mesh_path(f"{pname}.obj")
        )
        shroud.visual(
            mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0515)),
            material=GUNMETAL,
            name=pname,
        )

    # Standoff posts tie the shroud down to PCB mounting points.
    for k, (px, py) in enumerate(((0.008, 0.007), (0.008, 0.108), (0.295, 0.007), (0.295, 0.108))):
        shroud.visual(
            Cylinder(radius=0.0035, length=0.0457),
            origin=Origin(xyz=(px, py, 0.02435)),
            material=SHROUD_BLACK,
            name=f"shroud_standoff_{k}",
        )

    # White translucent RGB diffuser strip along the top edge of the shroud.
    shroud.visual(
        Box((0.280, 0.010, 0.0035)),
        origin=Origin(xyz=(0.160, 0.124, 0.05325)),
        material=DIFFUSER_WHITE,
        name="rgb_diffuser_strip",
    )
    return shroud


def _add_fan(model: ArticulatedObject, index: int, rotor_mesh):
    fan = model.part(f"fan_{index}")
    fan.inertial = Inertial.from_geometry(
        Cylinder(radius=FAN_R, length=FAN_THICK),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    fan.visual(
        rotor_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=BLADE_BLACK,
        name="rotor_blades",
    )
    # Hub badge: red ring, black center cap, small white dragon emblem chip.
    fan.visual(
        Cylinder(radius=0.01495, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0059)),
        material=BADGE_RED,
        name="hub_ring_red",
    )
    fan.visual(
        Cylinder(radius=0.0108, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.0070)),
        material=BADGE_BLACK,
        name="hub_cap_black",
    )
    fan.visual(
        Box((0.009, 0.005, 0.0012)),
        origin=Origin(xyz=(0.004, 0.0, 0.0080)),
        material=EMBLEM_WHITE,
        name="hub_dragon_emblem",
    )
    return fan


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="msi_triple_fan_graphics_card", assets=ASSETS)

    board = _add_board(model)
    shroud = _add_shroud(model)

    rotor_mesh = _build_fan_rotor_mesh()
    for i in range(3):
        _add_fan(model, i, rotor_mesh)

    model.articulation(
        "shroud_mount",
        ArticulationType.FIXED,
        parent=board,
        child=shroud,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    for i, cx in enumerate(FAN_CENTERS_X):
        model.articulation(
            f"fan_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=board,
            child=f"fan_{i}",
            origin=Origin(xyz=(cx, FAN_CY, FAN_CZ)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.4, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=120, overlap_tol=0.003, overlap_volume_tol=0.0
    )

    board = object_model.get_part("board")
    shroud = object_model.get_part("shroud")

    # Intentional local seats: rotor hubs on motor pods, standoffs into the PCB.
    for i in range(3):
        ctx.allow_overlap(
            "board",
            f"fan_{i}",
            elem_a=f"fan_motor_pod_{i}",
            elem_b="rotor_blades",
            reason="Rotor hub is seated onto the motor pod bearing boss (axisymmetric capture).",
        )
        ctx.expect_contact(
            f"fan_{i}",
            "board",
            elem_a="rotor_blades",
            elem_b=f"fan_motor_pod_{i}",
            name=f"fan_{i}_hub_seats_on_motor_pod",
        )
    for k in range(4):
        ctx.allow_overlap(
            "shroud",
            "board",
            elem_a=f"shroud_standoff_{k}",
            elem_b="pcb_board",
            reason="Shroud standoff post is seated into its PCB mounting point.",
        )
        ctx.expect_contact(
            "shroud",
            "board",
            elem_a=f"shroud_standoff_{k}",
            elem_b="pcb_board",
            name=f"shroud_standoff_{k}_seated_on_pcb",
        )

    # Exactly three fans, each on its own continuous spin joint.
    fans = [p for p in object_model.parts if p.name.startswith("fan_")]
    ctx.check("three_fans_present", len(fans) == 3, f"fan parts={[p.name for p in fans]}")
    for i in range(3):
        joint = object_model.get_articulation(f"fan_{i}_spin")
        ctx.check(
            f"fan_{i}_joint_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"type={joint.articulation_type}",
        )

    # Each fan spins in place: the off-axis dragon emblem AABB-center must
    # displace under rotation while the fan origin stays fixed on its axis.
    for i in range(3):
        fan = object_model.get_part(f"fan_{i}")
        rest_pos = ctx.part_world_position(fan)
        rest_aabb = ctx.part_element_world_aabb(fan, elem="hub_dragon_emblem")
        ctx.check(f"fan_{i}_emblem_aabb", rest_aabb is not None, "missing emblem AABB")
        if rest_aabb is None:
            continue
        rest_c = [(rest_aabb[0][k] + rest_aabb[1][k]) / 2 for k in range(3)]
        with ctx.pose({f"fan_{i}_spin": math.pi}):
            posed_pos = ctx.part_world_position(fan)
            posed_aabb = ctx.part_element_world_aabb(fan, elem="hub_dragon_emblem")
            posed_c = [(posed_aabb[0][k] + posed_aabb[1][k]) / 2 for k in range(3)]
            disp = math.dist(rest_c, posed_c)
            ctx.check(
                f"fan_{i}_spin_displaces_hub_badge",
                disp > 0.005,
                f"emblem AABB-center displacement={disp:.4f}",
            )
            ctx.check(
                f"fan_{i}_spins_in_place",
                math.dist(rest_pos, posed_pos) < 1e-9,
                f"rest={rest_pos} posed={posed_pos}",
            )
        # The rotor stays inside the shroud footprint.
        ctx.expect_within(fan, shroud, axes="xy", margin=0.002)

    # Gold edge connector protrudes below the PCB bottom edge.
    pcb_aabb = ctx.part_element_world_aabb(board, elem="pcb_board")
    conn_aabb = ctx.part_element_world_aabb(board, elem="pcie_connector_long_key")
    ctx.check(
        "connector_below_pcb",
        conn_aabb is not None
        and pcb_aabb is not None
        and conn_aabb[0][1] < pcb_aabb[0][1] - 0.005
        and conn_aabb[1][1] <= pcb_aabb[0][1] + 0.0015,
        f"pcb={pcb_aabb} connector={conn_aabb}",
    )

    # Bracket sits at the -X card end, outside the PCB.
    bracket_aabb = ctx.part_element_world_aabb(board, elem="io_bracket_plate")
    ctx.check(
        "bracket_at_front_end",
        bracket_aabb is not None and bracket_aabb[1][0] <= 0.0005,
        f"bracket={bracket_aabb}",
    )

    # Silver fin stack sits below the fans and behind the shroud plate.
    fin_aabb = ctx.part_element_world_aabb(board, elem="fin_18")
    fan1_aabb = ctx.part_world_aabb(object_model.get_part("fan_1"))
    plate_aabb = ctx.part_element_world_aabb(shroud, elem="shroud_top_plate")
    ctx.check(
        "fins_below_fans",
        fin_aabb is not None and fan1_aabb is not None and fin_aabb[1][2] <= fan1_aabb[0][2],
        f"fin_top={fin_aabb} fan_bottom={fan1_aabb}",
    )
    ctx.check(
        "shroud_plate_above_fins",
        plate_aabb is not None and fin_aabb is not None and plate_aabb[0][2] >= fin_aabb[1][2],
        f"plate={plate_aabb} fin={fin_aabb}",
    )
    fin_count = sum(1 for v in board.visuals if v.name and v.name.startswith("fin_"))
    ctx.check("fin_stack_dense", fin_count >= 30, f"fin_count={fin_count}")

    # RGB diffuser strip runs along the top edge of the shroud face.
    diff_aabb = ctx.part_element_world_aabb(shroud, elem="rgb_diffuser_strip")
    ctx.check(
        "diffuser_along_top_edge",
        diff_aabb is not None
        and diff_aabb[0][1] > 0.110
        and (diff_aabb[1][0] - diff_aabb[0][0]) > 0.20,
        f"diffuser={diff_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
