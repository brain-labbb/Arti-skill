from __future__ import annotations

# ZOTAC Gaming compact single-fan ITX graphics card.
# Canonical frame: +x runs from the PCIe bracket end toward the card tail,
# +y is the shroud-face normal (card thickness), +z is card height
# (gold PCIe edge connector hangs below the PCB bottom edge).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------- materials
GUNMETAL = Material("gunmetal_shroud", (0.135, 0.14, 0.155, 1.0))
GUNMETAL_DARK = Material("gunmetal_dark", (0.085, 0.09, 0.10, 1.0))
COPPER_ACCENT = Material("copper_accent", (0.82, 0.51, 0.20, 1.0))
FAN_BLACK = Material("fan_black", (0.045, 0.045, 0.055, 1.0))
HUB_GLOSS = Material("hub_gloss_black", (0.03, 0.03, 0.035, 1.0))
BADGE_WHITE = Material("badge_white", (0.94, 0.94, 0.95, 1.0))
ALU_SILVER = Material("aluminum_fin", (0.62, 0.64, 0.67, 1.0))
PCB_GREEN = Material("pcb_green_black", (0.05, 0.10, 0.08, 1.0))
GOLD = Material("connector_gold", (0.83, 0.66, 0.20, 1.0))
STEEL = Material("bracket_steel", (0.58, 0.60, 0.63, 1.0))
PORT_DARK = Material("port_dark", (0.12, 0.12, 0.13, 1.0))

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
FAN_HOLE_R = 0.0425  # opening in the shroud face
WELL_RING_RO = 0.047
ROTOR_TIP_R = 0.0405
HUB_R = 0.015

FIN_Y0, FIN_Y1 = 0.0055, 0.026
BOSS_R, BOSS_Y0, BOSS_Y1 = 0.0125, 0.024, 0.029

BRACKET_XI = -0.0848  # bracket inner face (embeds 0.2 mm into PCB edge)
BRACKET_XO = -0.0878
BRACKET_Z0, BRACKET_Z1 = 0.004, 0.110

SLAT_ZONES = {
    "bracket_side": (-0.0810, -0.0520),
    "tail_side": (0.0520, 0.0810),
}
SLAT_Z0, SLAT_Z1 = 0.018, 0.094
SLAT_PITCH_C = 0.0092
SLAT_WIDTH = 0.0045  # in-face width of one diagonal slat bar
SLAT_DEPTH = 0.004  # along y
SLAT_Y_C = 0.036  # recessed slightly behind the face plane

PORT_ZS = (0.030, 0.054, 0.078)  # DP, DP, HDMI centers

SLAT_COUNTS: dict[str, int] = {}


def _box(part, name, mat, x0, x1, y0, y1, z0, z1):
    part.visual(
        Box((x1 - x0, y1 - y0, z1 - z0)),
        origin=Origin(xyz=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)),
        material=mat,
        name=name,
    )


def _blade_section(chord: float, thickness: float, sweep: float, radius: float, twist: float):
    """Curved fan-blade section in the (chord, pitch) plane at a given radius."""
    profile_2d = [
        (-0.48 * chord, -0.16 * thickness),
        (-0.20 * chord, -0.55 * thickness),
        (0.18 * chord, -0.42 * thickness),
        (0.50 * chord, 0.0),
        (0.20 * chord, 0.55 * thickness),
        (-0.40 * chord, 0.24 * thickness),
    ]
    c = math.cos(twist)
    s = math.sin(twist)
    section = []
    for x, y in profile_2d:
        x += sweep
        section.append((c * x - s * y, s * x + c * y, radius))
    return section


def _blade_mesh():
    geom = LoftGeometry(
        [
            _blade_section(chord=0.015, thickness=0.0032, sweep=0.000, radius=0.013, twist=0.55),
            _blade_section(chord=0.020, thickness=0.0026, sweep=0.004, radius=0.026, twist=0.28),
            _blade_section(chord=0.016, thickness=0.0022, sweep=0.008, radius=0.036, twist=0.18),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, "gpu_fan_blade")


def _slat_segments(x0: float, x1: float):
    """45-degree diagonal slat segments (z = x + c) clipped to the zone box."""
    segments = []
    c = (SLAT_Z0 - x1) + 0.004
    c_max = SLAT_Z1 - x0
    while c < c_max:
        sx0 = max(x0, SLAT_Z0 - c)
        sx1 = min(x1, SLAT_Z1 - c)
        if sx1 - sx0 >= 0.0015:
            xm = (sx0 + sx1) / 2
            zm = xm + c
            length = (sx1 - sx0) * math.sqrt(2.0) + 0.005  # overhang into border rails
            segments.append((xm, zm, length))
        c += SLAT_PITCH_C
    return segments


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

    # ------------------------------------------------------------- shroud
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

    # central face plate with the circular fan opening (real through-hole)
    face_plate_cq = (
        cq.Workplane("XY")
        .box(0.098, 0.096, FACE_Y1 - FACE_Y0)
        .faces(">Z")
        .workplane()
        .hole(2.0 * FAN_HOLE_R)
    )
    shroud.visual(
        mesh_from_cadquery(face_plate_cq, "gpu_face_plate"),
        origin=Origin(
            xyz=(0.0, (FACE_Y0 + FACE_Y1) / 2 - sh_org[1], 0.0),
            rpy=(-math.pi / 2, 0.0, 0.0),
        ),
        material=GUNMETAL,
        name="face_plate",
    )
    # recessed fan well ring behind the opening
    well_ring_cq = (
        cq.Workplane("XY").circle(WELL_RING_RO).circle(FAN_HOLE_R).extrude(0.0095)
    )
    shroud.visual(
        mesh_from_cadquery(well_ring_cq, "gpu_fan_well_ring"),
        origin=Origin(xyz=(0.0, 0.0265 - sh_org[1], 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=GUNMETAL_DARK,
        name="fan_well_ring",
    )
    # face borders around the slatted flanks
    sbox("top_border_bracket_side", GUNMETAL, SHROUD_X0, -0.048, FACE_Y0, FACE_Y1, 0.094, SHROUD_Z1)
    sbox("top_border_tail_side", GUNMETAL, 0.048, SHROUD_X1, FACE_Y0, FACE_Y1, 0.094, SHROUD_Z1)
    sbox(
        "bottom_border_bracket_side",
        GUNMETAL,
        SHROUD_X0,
        -0.048,
        FACE_Y0,
        FACE_Y1,
        SHROUD_Z0,
        0.018,
    )
    sbox("bottom_border_tail_side", GUNMETAL, 0.048, SHROUD_X1, FACE_Y0, FACE_Y1, SHROUD_Z0, 0.018)
    sbox("end_cap_bracket_end", GUNMETAL, SHROUD_X0, -0.080, FACE_Y0, FACE_Y1, SHROUD_Z0, SHROUD_Z1)
    sbox("end_cap_tail_end", GUNMETAL, 0.081, SHROUD_X1, FACE_Y0, FACE_Y1, SHROUD_Z0, SHROUD_Z1)
    # copper/amber accent strips separating the fan zone from the slat zones
    sbox("copper_strip_bracket_side", COPPER_ACCENT, -0.052, -0.0485, 0.036, 0.0415, SHROUD_Z0, SHROUD_Z1)
    sbox("copper_strip_tail_side", COPPER_ACCENT, 0.0485, 0.052, 0.036, 0.0415, SHROUD_Z0, SHROUD_Z1)
    # ZOTAC GAMING lettering strip, top-left (bracket side) corner of the face
    sbox("logo_text_strip", COPPER_ACCENT, -0.082, -0.050, 0.0397, 0.0412, 0.0955, 0.1025)

    # diagonal slat vents on both flanks (real bars with through gaps)
    for zone_name, (zx0, zx1) in SLAT_ZONES.items():
        segments = _slat_segments(zx0, zx1)
        SLAT_COUNTS[zone_name] = len(segments)
        for i, (xm, zm, length) in enumerate(segments):
            shroud.visual(
                Box((length, SLAT_DEPTH, SLAT_WIDTH)),
                origin=Origin(
                    xyz=(xm - sh_org[0], SLAT_Y_C - sh_org[1], zm - sh_org[2]),
                    rpy=(0.0, -math.pi / 4, 0.0),
                ),
                material=GUNMETAL_DARK,
                name=f"slat_{zone_name}_{i}",
            )

    # perimeter walls back toward the PCB
    sbox("top_wall", GUNMETAL, SHROUD_X0, SHROUD_X1, WALL_Y0, WALL_Y1, 0.100, SHROUD_Z1)
    sbox("bottom_wall", GUNMETAL, SHROUD_X0, SHROUD_X1, WALL_Y0, WALL_Y1, SHROUD_Z0, 0.012)
    sbox("end_wall_bracket_end", GUNMETAL, SHROUD_X0, -0.080, WALL_Y0, WALL_Y1, SHROUD_Z0, SHROUD_Z1)
    sbox("end_wall_tail_end", GUNMETAL, 0.081, SHROUD_X1, WALL_Y0, WALL_Y1, SHROUD_Z0, SHROUD_Z1)
    # mount posts carrying the shroud down to the PCB
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

    # ------------------------------------------------------------- fan rotor
    rotor = model.part("fan_rotor")
    blade_mesh = _blade_mesh()
    y0 = FAN_AXIS_Y  # rotor local y=0 plane (world)
    rotor.visual(
        Cylinder(radius=HUB_R, length=0.0095),
        origin=Origin(xyz=(0.0, 0.03425 - y0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=HUB_GLOSS,
        name="fan_hub",
    )
    rotor.visual(
        Cylinder(radius=0.0095, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0395 - y0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=BADGE_WHITE,
        name="hub_badge",
    )
    rotor.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.0, 0.0285 - y0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=GUNMETAL_DARK,
        name="rotor_shaft",
    )
    blade_count = 9
    for i in range(blade_count):
        rotor.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, i * 2.0 * math.pi / blade_count, 0.0)),
            material=FAN_BLACK,
            name=f"blade_{i}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=ROTOR_TIP_R, length=0.012),
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

    # ---- overall card proportions (compact ITX single-fan card)
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

    # ---- single large central fan, recessed in the shroud well
    r_aabb = ctx.part_world_aabb(rotor)
    r_size = tuple(r_aabb[1][i] - r_aabb[0][i] for i in range(3))
    ctx.check(
        "fan_is_about_90mm",
        0.076 <= r_size[0] <= 0.084 and 0.076 <= r_size[2] <= 0.084,
        f"rotor size={r_size!r}",
    )
    ctx.expect_origin_distance(rotor, shroud, axes="xz", max_dist=0.002, name="fan_centered_in_shroud")
    face_aabb = ctx.part_element_world_aabb(shroud, elem="face_plate")
    blade_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
    ctx.check(
        "fan_blades_recessed_behind_face",
        blade_aabb[1][1] <= face_aabb[1][1] - 0.0005,
        f"blade_max_y={blade_aabb[1][1]:.4f} face_max_y={face_aabb[1][1]:.4f}",
    )
    badge_aabb = ctx.part_element_world_aabb(rotor, elem="hub_badge")
    ctx.check(
        "white_badge_on_hub_front",
        badge_aabb[1][1] > blade_aabb[1][1],
        f"badge_max_y={badge_aabb[1][1]:.4f}",
    )

    # ---- fan spin articulation: blades orbit, hub stays centered
    rest_center = tuple((blade_aabb[0][i] + blade_aabb[1][i]) / 2 for i in range(3))
    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({fan_spin: math.pi / 2.0}):
        posed_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0")
        posed_center = tuple((posed_aabb[0][i] + posed_aabb[1][i]) / 2 for i in range(3))
        posed_pos = ctx.part_world_position(rotor)
        disp = math.dist(rest_center, posed_center)
        ctx.check("fan_spin_displaces_blade", disp > 0.015, f"blade_0 center disp={disp:.4f}")
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

    # ---- diagonal slat vents on both flanks
    for zone in ("bracket_side", "tail_side"):
        n = int(SLAT_COUNTS.get(zone, 0))
        ctx.check(f"slat_count_{zone}", n >= 8, f"zone={zone} count={n}")
        seen = 0
        for i in range(n):
            if ctx.part_element_world_aabb(shroud, elem=f"slat_{zone}_{i}") is not None:
                seen += 1
        ctx.check(f"slat_elems_present_{zone}", seen == n and seen >= 8, f"seen={seen} n={n}")
    slat_aabb = ctx.part_element_world_aabb(shroud, elem="slat_tail_side_3")
    ctx.check(
        "slats_recessed_behind_face",
        slat_aabb[1][1] <= face_aabb[1][1] - 0.001,
        f"slat_max_y={slat_aabb[1][1]:.4f} face_max_y={face_aabb[1][1]:.4f}",
    )

    # ---- copper accents proud of the gunmetal face
    for elem in ("copper_strip_bracket_side", "copper_strip_tail_side", "logo_text_strip"):
        c_aabb = ctx.part_element_world_aabb(shroud, elem=elem)
        ctx.check(
            f"{elem}_proud_of_face",
            c_aabb is not None and c_aabb[1][1] >= face_aabb[1][1] + 0.0008,
            f"{elem} aabb={c_aabb!r}",
        )

    # ---- finned heatsink visible behind the fan opening
    fin_aabb = ctx.part_element_world_aabb(heatsink, elem="fin_11")
    ctx.check(
        "fins_behind_fan_blades",
        fin_aabb[1][1] <= blade_aabb[0][1] - 0.001,
        f"fin_max_y={fin_aabb[1][1]:.4f} blade_min_y={blade_aabb[0][1]:.4f}",
    )
    ctx.expect_overlap(heatsink, rotor, axes="xz", min_overlap=0.05, name="fins_span_fan_footprint")

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
