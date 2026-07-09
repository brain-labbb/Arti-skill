from __future__ import annotations

# ZOTAC Gaming compact single-blower ITX graphics card (blower variant).
# Canonical frame: +x runs from the PCIe bracket end toward the card tail,
# +y is the shroud-face normal (card thickness), +z is card height
# (gold PCIe edge connector hangs below the PCB bottom edge).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- materials
BLOWER_DARK = Material("blower_dark", (0.06, 0.06, 0.065, 1.0))
BLOWER_DARKER = Material("blower_darker", (0.04, 0.04, 0.045, 1.0))
GUNMETAL_DARK = Material("gunmetal_dark", (0.085, 0.09, 0.10, 1.0))
FAN_BLACK = Material("fan_black", (0.045, 0.045, 0.055, 1.0))
HUB_GLOSS = Material("hub_gloss_black", (0.03, 0.03, 0.035, 1.0))
BADGE_WHITE = Material("badge_white", (0.94, 0.94, 0.95, 1.0))
ALU_SILVER = Material("aluminum_fin", (0.62, 0.64, 0.67, 1.0))
PCB_GREEN = Material("pcb_green_black", (0.05, 0.10, 0.08, 1.0))
GOLD = Material("connector_gold", (0.83, 0.66, 0.20, 1.0))
STEEL = Material("bracket_steel", (0.58, 0.60, 0.63, 1.0))
PORT_DARK = Material("port_dark", (0.12, 0.12, 0.13, 1.0))
BRANDING_SILVER = Material("branding_silver", (0.45, 0.46, 0.48, 1.0))

# ---------------------------------------------------------------- key dims (m)
CARD_X0 = -0.0850  # PCB bracket-end edge
CARD_X1 = 0.0850  # card tail-end edge (shroud)
PCB_X1 = 0.0800
PCB_Y0, PCB_Y1 = 0.0, 0.0016
PCB_Z0, PCB_Z1 = 0.010, 0.104

SHROUD_X0, SHROUD_X1 = -0.0840, 0.0850
FACE_Y0, FACE_Y1 = 0.035, 0.040  # shroud face plate
WALL_Y0, WALL_Y1 = 0.022, 0.0355  # shroud perimeter walls
SHROUD_Z0, SHROUD_Z1 = 0.008, 0.104

FAN_CX, FAN_CZ = 0.0, 0.056  # fan axis position
FAN_AXIS_Y = 0.0335  # rotor mid-plane (world y)
HUB_R = 0.015

# Blower wheel (squirrel-cage centrifugal)
CAGE_OUTER_R = 0.030
CAGE_INNER_R = 0.015  # matches HUB_R for cage-to-hub contact
CAGE_WIDTH = 0.008
CAGE_BLADE_COUNT = 28
CAGE_BLADE_THICK = 0.001
CAGE_Y_CENTER = -0.003  # rotor-local Y offset for cage center

# Intake grille (top edge of shroud face)
INTAKE_X0, INTAKE_X1 = -0.040, 0.040
INTAKE_Z0, INTAKE_Z1 = 0.088, 0.102
INTAKE_BAR_COUNT = 5

# Exhaust duct (bracket-end wall)
EXH_Z0, EXH_Z1 = 0.028, 0.082
EXH_LOUVER_COUNT = 7

FIN_Y0, FIN_Y1 = 0.0055, 0.026
BOSS_R, BOSS_Y0, BOSS_Y1 = 0.0125, 0.024, 0.029

BRACKET_XI = -0.0848  # bracket inner face (embeds 0.2 mm into PCB edge)
BRACKET_XO = -0.0878
BRACKET_Z0, BRACKET_Z1 = 0.004, 0.110

PORT_ZS = (0.030, 0.054, 0.078)  # DP, DP, HDMI centers


def _box(part, name, mat, x0, x1, y0, y1, z0, z1):
    part.visual(
        Box((x1 - x0, y1 - y0, z1 - z0)),
        origin=Origin(xyz=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)),
        material=mat,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zotac_compact_gpu")

    # ------------------------------------------------------------- PCB (root)
    pcb = model.part("pcb")
    _box(pcb, "pcb_plate", PCB_GREEN, CARD_X0, PCB_X1, PCB_Y0, PCB_Y1, PCB_Z0, PCB_Z1)
    # gold PCIe edge connector below the PCB bottom edge, split at the key notch
    _box(pcb, "edge_connector_long", GOLD, -0.062, 0.008, 0.0002, 0.0014, 0.002, 0.0105)
    _box(pcb, "edge_connector_short", GOLD, 0.012, 0.030, 0.0002, 0.0014, 0.002, 0.0105)
    # display outputs on the bracket end of the PCB
    _box(pcb, "display_port_0", PORT_DARK, -0.0840, -0.0705, 0.0013, 0.0083, 0.022, 0.038)
    _box(pcb, "display_port_1", PORT_DARK, -0.0840, -0.0705, 0.0013, 0.0083, 0.046, 0.062)
    _box(pcb, "hdmi_port", PORT_DARK, -0.0840, -0.0705, 0.0013, 0.0083, 0.070, 0.086)
    # rear-face SMD packages
    _box(pcb, "smd_chip_0", GUNMETAL_DARK, -0.040, -0.020, -0.002, 0.0002, 0.046, 0.066)
    _box(pcb, "smd_chip_1", GUNMETAL_DARK, 0.018, 0.042, -0.0016, 0.0002, 0.040, 0.072)
    pcb.inertial = Inertial.from_geometry(
        Box((0.165, 0.0016, 0.094)),
        mass=0.15,
        origin=Origin(xyz=(-0.0025, 0.0008, 0.057)),
    )

    # ------------------------------------------------------------- heatsink
    hs_org = (0.0, 0.0016, 0.056)
    heatsink = model.part("heatsink")

    def hbox(name, mat, x0, x1, y0, y1, z0, z1):
        heatsink.visual(
            Box((x1 - x0, y1 - y0, z1 - z0)),
            origin=Origin(
                xyz=(
                    (x0 + x1) / 2 - hs_org[0],
                    (y0 + y1) / 2 - hs_org[1],
                    (z0 + z1) / 2 - hs_org[2],
                )
            ),
            material=mat,
            name=name,
        )

    hbox("heatsink_base", ALU_SILVER, -0.070, 0.070, 0.0013, 0.0055, 0.016, 0.096)
    fin_count = 24
    for i in range(fin_count):
        fx = -0.069 + i * 0.006
        hbox(f"fin_{i:02d}", ALU_SILVER, fx - 0.0008, fx + 0.0008, 0.005, FIN_Y1, 0.017, 0.095)
    heatsink.visual(
        Cylinder(radius=BOSS_R, length=BOSS_Y1 - BOSS_Y0),
        origin=Origin(
            xyz=(FAN_CX - hs_org[0], (BOSS_Y0 + BOSS_Y1) / 2 - hs_org[1], FAN_CZ - hs_org[2]),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        material=GUNMETAL_DARK,
        name="motor_boss",
    )
    heatsink.inertial = Inertial.from_geometry(
        Box((0.140, 0.026, 0.080)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
    )

    # ------------------------------------------------------------- shroud (blower enclosure)
    sh_org = (0.0, 0.022, 0.056)
    shroud = model.part("shroud")

    def sbox(name, mat, x0, x1, y0, y1, z0, z1):
        shroud.visual(
            Box((x1 - x0, y1 - y0, z1 - z0)),
            origin=Origin(
                xyz=(
                    (x0 + x1) / 2 - sh_org[0],
                    (y0 + y1) / 2 - sh_org[1],
                    (z0 + z1) / 2 - sh_org[2],
                )
            ),
            material=mat,
            name=name,
        )

    # Fully enclosed face plate with rectangular intake cutout at top edge
    face_w = SHROUD_X1 - SHROUD_X0  # 0.169
    face_h = SHROUD_Z1 - SHROUD_Z0  # 0.096
    face_t = FACE_Y1 - FACE_Y0  # 0.005
    intake_w = INTAKE_X1 - INTAKE_X0  # 0.080
    intake_h = INTAKE_Z1 - INTAKE_Z0  # 0.014
    # In CadQuery local frame: Y maps to -Z after rpy=(-pi/2,0,0)
    # Intake center Z offset from face center: (INTAKE_Z0+INTAKE_Z1)/2 - (SHROUD_Z0+SHROUD_Z1)/2 = 0.039
    intake_cy = -((INTAKE_Z0 + INTAKE_Z1) / 2 - (SHROUD_Z0 + SHROUD_Z1) / 2)
    face_cq = (
        cq.Workplane("XY")
        .box(face_w, face_h, face_t)
        .faces(">Z")
        .workplane()
        .center(0.0, intake_cy)
        .rect(intake_w, intake_h)
        .cutThruAll()
    )
    shroud.visual(
        mesh_from_cadquery(face_cq, "blower_face_plate"),
        origin=Origin(
            xyz=(
                (SHROUD_X0 + SHROUD_X1) / 2 - sh_org[0],
                (FACE_Y0 + FACE_Y1) / 2 - sh_org[1],
                (SHROUD_Z0 + SHROUD_Z1) / 2 - sh_org[2],
            ),
            rpy=(-math.pi / 2, 0.0, 0.0),
        ),
        material=BLOWER_DARK,
        name="face_plate",
    )

    # Intake grille bars spanning the top-edge rectangular opening
    for i in range(INTAKE_BAR_COUNT):
        bar_z = INTAKE_Z0 + (i + 0.5) * (INTAKE_Z1 - INTAKE_Z0) / INTAKE_BAR_COUNT
        # Extend bars slightly beyond cutout in X to connect to solid face plate
        sbox(
            f"intake_bar_{i}",
            BLOWER_DARKER,
            INTAKE_X0 - 0.002,
            INTAKE_X1 + 0.002,
            FACE_Y0,
            FACE_Y1,
            bar_z - 0.0008,
            bar_z + 0.0008,
        )

    # ZOTAC GAMING branding strip, top-left (bracket side) of the face
    sbox("logo_text_strip", BRANDING_SILVER, -0.082, -0.050, 0.0397, 0.0412, 0.0955, 0.1025)

    # Perimeter walls back toward the PCB
    sbox("top_wall", BLOWER_DARK, SHROUD_X0, SHROUD_X1, WALL_Y0, WALL_Y1, 0.100, SHROUD_Z1)
    sbox("bottom_wall", BLOWER_DARK, SHROUD_X0, SHROUD_X1, WALL_Y0, WALL_Y1, SHROUD_Z0, 0.012)
    sbox("end_wall_tail_end", BLOWER_DARK, 0.081, SHROUD_X1, WALL_Y0, WALL_Y1, SHROUD_Z0, SHROUD_Z1)

    # Bracket-end wall split around exhaust duct opening
    sbox("end_wall_bracket_upper", BLOWER_DARK, SHROUD_X0, -0.080, WALL_Y0, WALL_Y1, EXH_Z1, SHROUD_Z1)
    sbox("end_wall_bracket_lower", BLOWER_DARK, SHROUD_X0, -0.080, WALL_Y0, WALL_Y1, SHROUD_Z0, EXH_Z0)

    # Exhaust duct frame ribs (sides of the opening for structural connectivity)
    exh_x_center = (SHROUD_X0 + (-0.080)) / 2
    exh_x_extent = (-0.080) - SHROUD_X0
    exh_z_center = (EXH_Z0 + EXH_Z1) / 2
    exh_z_span = EXH_Z1 - EXH_Z0 + 0.004  # extends slightly into wall regions

    # Exhaust louver ribs (host-conformal blower-exhaust louver ribs)
    wall_y_extent = WALL_Y1 - WALL_Y0
    for i in range(EXH_LOUVER_COUNT):
        ly = WALL_Y0 + (i + 0.5) * wall_y_extent / EXH_LOUVER_COUNT
        shroud.visual(
            Box((exh_x_extent * 0.85, 0.0012, exh_z_span)),
            origin=Origin(
                xyz=(
                    exh_x_center - sh_org[0],
                    ly - sh_org[1],
                    exh_z_center - sh_org[2],
                ),
                rpy=(0.30, 0.0, 0.0),  # tilted ~17deg to redirect exhaust flow
            ),
            material=BLOWER_DARKER,
            name=f"exhaust_louver_{i}",
        )

    # Mount posts carrying the shroud down to the PCB
    post_specs = (
        (-0.076, 0.0115),
        (-0.076, 0.0995),
        (0.076, 0.0115),
        (0.076, 0.0995),
    )
    for i, (px, pz) in enumerate(post_specs):
        sbox(
            f"mount_post_{i}",
            GUNMETAL_DARK,
            px - 0.003,
            px + 0.003,
            0.0013,
            0.024,
            pz - 0.003,
            pz + 0.003,
        )
    shroud.inertial = Inertial.from_geometry(
        Box((0.169, 0.018, 0.096)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.009, 0.0)),
    )

    # ------------------------------------------------------------- bracket
    br_org = (BRACKET_XI, 0.020, 0.056)
    bracket = model.part("bracket")

    def bbox(name, x0, x1, y0, y1, z0, z1):
        bracket.visual(
            Box((x1 - x0, y1 - y0, z1 - z0)),
            origin=Origin(
                xyz=(
                    (x0 + x1) / 2 - br_org[0],
                    (y0 + y1) / 2 - br_org[1],
                    (z0 + z1) / 2 - br_org[2],
                )
            ),
            material=STEEL,
            name=name,
        )

    # stamped plate decomposed to leave real display-port openings
    bbox("bracket_panel", BRACKET_XO, BRACKET_XI, 0.0098, 0.040, BRACKET_Z0, BRACKET_Z1)
    bbox("bracket_lower_rail", BRACKET_XO, BRACKET_XI, 0.0, 0.002, BRACKET_Z0, BRACKET_Z1)
    seg_zs = (
        (BRACKET_Z0, PORT_ZS[0] - 0.009),
        (PORT_ZS[0] + 0.009, PORT_ZS[1] - 0.009),
        (PORT_ZS[1] + 0.009, PORT_ZS[2] - 0.009),
        (PORT_ZS[2] + 0.009, BRACKET_Z1),
    )
    for i, (z0, z1) in enumerate(seg_zs):
        bbox(f"bracket_band_seg_{i}", BRACKET_XO, BRACKET_XI, 0.0018, 0.0102, z0, z1)
    bbox("bracket_top_tab", -0.0940, BRACKET_XO + 0.0006, 0.0, 0.040, 0.107, BRACKET_Z1)
    bracket.inertial = Inertial.from_geometry(
        Box((0.003, 0.040, 0.106)),
        mass=0.05,
        origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
    )

    # ------------------------------------------------------------- fan rotor (blower wheel)
    rotor = model.part("fan_rotor")
    y0 = FAN_AXIS_Y  # rotor local y=0 plane (world)

    # Squirrel-cage centrifugal blower wheel via shared SDK helper
    blower_wheel = BlowerWheelGeometry(
        CAGE_OUTER_R,
        CAGE_INNER_R,
        CAGE_WIDTH,
        CAGE_BLADE_COUNT,
        blade_thickness=CAGE_BLADE_THICK,
        blade_sweep_deg=0.0,  # straight radial cage blades
        backplate=True,
        shroud=True,
        center=True,
    )
    blower_mesh = mesh_from_geometry(blower_wheel, "blower_cage")
    rotor.visual(
        blower_mesh,
        origin=Origin(
            xyz=(0.0, CAGE_Y_CENTER, 0.0),
            rpy=(-math.pi / 2, 0.0, 0.0),  # align local Z spin axis with card Y
        ),
        material=FAN_BLACK,
        name="cage_wheel",
    )

    # Hub, badge, and shaft (kept from parent, repositioned for enclosed shroud)
    rotor.visual(
        Cylinder(radius=HUB_R, length=0.0095),
        origin=Origin(xyz=(0.0, 0.03425 - y0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=HUB_GLOSS,
        name="fan_hub",
    )
    # Badge positioned just below the enclosed face plate to avoid overlap
    rotor.visual(
        Cylinder(radius=0.0095, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0340 - y0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=BADGE_WHITE,
        name="hub_badge",
    )
    rotor.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.0, 0.0285 - y0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=GUNMETAL_DARK,
        name="rotor_shaft",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=CAGE_OUTER_R, length=CAGE_WIDTH),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
    )

    # ------------------------------------------------------------- joints
    model.articulation(
        "heatsink_mount",
        ArticulationType.FIXED,
        parent=pcb,
        child=heatsink,
        origin=Origin(xyz=hs_org),
    )
    model.articulation(
        "shroud_mount",
        ArticulationType.FIXED,
        parent=pcb,
        child=shroud,
        origin=Origin(xyz=sh_org),
    )
    model.articulation(
        "bracket_mount",
        ArticulationType.FIXED,
        parent=pcb,
        child=bracket,
        origin=Origin(xyz=br_org),
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=shroud,
        child=rotor,
        origin=Origin(xyz=(FAN_CX - sh_org[0], FAN_AXIS_Y - sh_org[1], FAN_CZ - sh_org[2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=180.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pcb = object_model.get_part("pcb")
    heatsink = object_model.get_part("heatsink")
    shroud = object_model.get_part("shroud")
    bracket = object_model.get_part("bracket")
    rotor = object_model.get_part("fan_rotor")
    fan_spin = object_model.get_articulation("fan_spin")

    # ---- intentional, scoped seatings/captures
    ctx.allow_overlap(
        heatsink,
        pcb,
        elem_a="heatsink_base",
        elem_b="pcb_plate",
        reason="Heatsink base plate is seated 0.3 mm into the PCB face as thermal contact.",
    )
    for i in range(4):
        ctx.allow_overlap(
            shroud,
            pcb,
            elem_a=f"mount_post_{i}",
            elem_b="pcb_plate",
            reason="Shroud mount post is screwed/seated 0.3 mm into the PCB.",
        )
    ctx.allow_overlap(
        bracket,
        pcb,
        elem_a="bracket_lower_rail",
        elem_b="pcb_plate",
        reason="The stamped bracket foot laps 0.2 mm onto the PCB edge where it is riveted.",
    )
    ctx.allow_overlap(
        rotor,
        heatsink,
        elem_a="rotor_shaft",
        elem_b="motor_boss",
        reason="Fan shaft is captured inside the motor bushing boss; coaxial for all spin poses.",
    )
    ctx.allow_overlap(
        rotor,
        heatsink,
        elem_a="cage_wheel",
        elem_b="motor_boss",
        reason="Blower wheel backplate surrounds the motor boss; coaxial captured fit inside the housing.",
    )

    # ---- overall card proportions (compact ITX single-blower card)
    aabbs = [ctx.part_world_aabb(p) for p in (pcb, heatsink, shroud, bracket, rotor)]
    ctx.check("all_part_aabbs", all(a is not None for a in aabbs), f"aabbs={aabbs!r}")
    if not all(a is not None for a in aabbs):
        return ctx.report()
    mins = [min(a[0][i] for a in aabbs) for i in range(3)]
    maxs = [max(a[1][i] for a in aabbs) for i in range(3)]
    length = maxs[0] - mins[0]
    thickness = maxs[1] - mins[1]
    height = maxs[2] - mins[2]
    ctx.check("card_is_itx_length", 0.160 <= length <= 0.190, f"length={length:.4f}")
    ctx.check("card_is_dual_slot_thick", 0.035 <= thickness <= 0.047, f"thickness={thickness:.4f}")
    ctx.check("card_height_with_bracket", 0.100 <= height <= 0.120, f"height={height:.4f}")

    # ---- BLOWER-SPECIFIC: enclosed face plate covers the fan center zone
    face_aabb = ctx.part_element_world_aabb(shroud, elem="face_plate")
    ctx.check(
        "face_plate_encloses_fan_center",
        face_aabb is not None
        and face_aabb[0][0] < FAN_CX - 0.025
        and face_aabb[1][0] > FAN_CX + 0.025
        and face_aabb[0][2] < FAN_CZ - 0.025
        and face_aabb[1][2] > FAN_CZ + 0.025,
        f"face_plate aabb={face_aabb!r}",
    )

    # ---- BLOWER-SPECIFIC: intake grille bars at top edge of shroud face
    intake_bar_aabbs = [
        ctx.part_element_world_aabb(shroud, elem=f"intake_bar_{i}")
        for i in range(INTAKE_BAR_COUNT)
    ]
    ctx.check(
        "intake_bars_present",
        all(a is not None for a in intake_bar_aabbs),
        f"missing intake bars",
    )
    if intake_bar_aabbs[0] is not None:
        ctx.check(
            "intake_bars_at_top_edge",
            intake_bar_aabbs[0][0][2] > 0.080,
            f"intake_bar_0 min_z={intake_bar_aabbs[0][0][2]:.4f}",
        )

    # ---- BLOWER-SPECIFIC: exhaust duct louver ribs at bracket end
    exhaust_aabbs = [
        ctx.part_element_world_aabb(shroud, elem=f"exhaust_louver_{i}")
        for i in range(EXH_LOUVER_COUNT)
    ]
    ctx.check(
        "exhaust_louvers_present",
        all(a is not None for a in exhaust_aabbs),
        f"missing exhaust louvers",
    )
    if exhaust_aabbs[0] is not None:
        ctx.check(
            "exhaust_louvers_at_bracket_end",
            exhaust_aabbs[0][1][0] < -0.075,
            f"exhaust_louver_0 max_x={exhaust_aabbs[0][1][0]:.4f}",
        )

    # ---- BLOWER-SPECIFIC: squirrel-cage blower wheel (not open axial rotor)
    cage_aabb = ctx.part_element_world_aabb(rotor, elem="cage_wheel")
    ctx.check("cage_wheel_exists", cage_aabb is not None, "cage_wheel missing")
    if cage_aabb is not None:
        cage_size = tuple(cage_aabb[1][i] - cage_aabb[0][i] for i in range(3))
        ctx.check(
            "cage_wheel_is_blower_sized",
            0.050 <= cage_size[0] <= 0.065 and 0.050 <= cage_size[2] <= 0.065,
            f"cage_size={cage_size!r}",
        )
        # Blower wheel is recessed behind the enclosed face plate
        ctx.check(
            "cage_wheel_behind_face",
            cage_aabb[1][1] <= face_aabb[1][1] - 0.0002,
            f"cage_max_y={cage_aabb[1][1]:.4f} face_max_y={face_aabb[1][1]:.4f}",
        )

    # ---- fan centered in shroud
    ctx.expect_origin_distance(rotor, shroud, axes="xz", max_dist=0.002, name="fan_centered_in_shroud")

    # ---- fan spin articulation: hub stays on axis, cage wheel spins
    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({fan_spin: math.pi / 2.0}):
        posed_pos = ctx.part_world_position(rotor)
        ctx.check(
            "fan_hub_stays_on_axis",
            math.dist(rest_pos, posed_pos) < 1e-9,
            f"rest={rest_pos!r} posed={posed_pos!r}",
        )

    # ---- shaft retained in the motor boss
    ctx.expect_overlap(
        rotor,
        heatsink,
        axes="y",
        elem_a="rotor_shaft",
        elem_b="motor_boss",
        min_overlap=0.0015,
        name="fan_shaft_inserted_in_motor_boss",
    )
    ctx.expect_within(
        rotor,
        heatsink,
        axes="xz",
        inner_elem="rotor_shaft",
        outer_elem="motor_boss",
        margin=0.0005,
        name="fan_shaft_centered_in_motor_boss",
    )

    # ---- heatsink fins behind the blower cage wheel
    fin_aabb = ctx.part_element_world_aabb(heatsink, elem="fin_11")
    if cage_aabb is not None:
        ctx.check(
            "fins_behind_cage_wheel",
            fin_aabb[1][1] <= cage_aabb[0][1] + 0.001,
            f"fin_max_y={fin_aabb[1][1]:.4f} cage_min_y={cage_aabb[0][1]:.4f}",
        )
    ctx.expect_overlap(heatsink, rotor, axes="xz", min_overlap=0.04, name="heatsink_spans_blower_footprint")

    # ---- branding strip proud of the face
    brand_aabb = ctx.part_element_world_aabb(shroud, elem="logo_text_strip")
    ctx.check(
        "logo_strip_proud_of_face",
        brand_aabb is not None and brand_aabb[1][1] >= face_aabb[1][1] + 0.0005,
        f"brand_aabb={brand_aabb!r}",
    )

    # ---- gold edge connector hangs below the PCB bottom edge
    pcb_aabb = ctx.part_element_world_aabb(pcb, elem="pcb_plate")
    conn_aabb = ctx.part_element_world_aabb(pcb, elem="edge_connector_long")
    ctx.check(
        "edge_connector_below_pcb",
        conn_aabb[0][2] < pcb_aabb[0][2] - 0.005,
        f"conn_min_z={conn_aabb[0][2]:.4f} pcb_min_z={pcb_aabb[0][2]:.4f}",
    )
    ctx.expect_overlap(pcb, pcb, elem_a="edge_connector_long", elem_b="pcb_plate", axes="x", min_overlap=0.05, name="connector_under_pcb_footprint")

    # ---- bracket with port openings caps the bracket end
    br_aabb = ctx.part_world_aabb(bracket)
    ctx.check(
        "bracket_at_card_end",
        br_aabb[1][0] < pcb_aabb[0][0] + 0.001,
        f"bracket_max_x={br_aabb[1][0]:.4f} pcb_min_x={pcb_aabb[0][0]:.4f}",
    )
    ctx.check(
        "bracket_taller_than_shroud",
        br_aabb[1][2] >= 0.108,
        f"bracket_max_z={br_aabb[1][2]:.4f}",
    )
    for elem in ("display_port_0", "display_port_1", "hdmi_port"):
        p_aabb = ctx.part_element_world_aabb(pcb, elem=elem)
        ctx.check(
            f"{elem}_faces_bracket",
            p_aabb is not None and p_aabb[0][0] <= br_aabb[1][0] + 0.002,
            f"{elem} aabb={p_aabb!r}",
        )

    return ctx.report()


object_model = build_object_model()
