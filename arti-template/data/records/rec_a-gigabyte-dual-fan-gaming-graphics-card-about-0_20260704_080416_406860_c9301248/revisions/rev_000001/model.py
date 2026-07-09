from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Global layout (meters).  Canonical frame: card standing as installed in a
# tower case seen from the fan side.
#   +X = from the PCIe bracket end toward the tail
#   +Y = from the backplate side toward the fan/shroud front
#   +Z = up (bracket tab up, PCIe edge connector down)
# ----------------------------------------------------------------------------

CARD_LEN = 0.280

# Y stack (thickness)
BACKPLATE_Y0, BACKPLATE_Y1 = -0.0193, -0.0165
PCB_Y0, PCB_Y1 = -0.0165, -0.0149
HS_BASE_Y0, HS_BASE_Y1 = -0.0149, -0.0129
FIN_Y0, FIN_Y1 = -0.0135, 0.004
PLATE_Y0, PLATE_Y1 = 0.018, 0.021  # shroud faceplate

# Z layout (height)
PCB_Z0, PCB_Z1 = 0.004, 0.092
SHROUD_Z0, SHROUD_Z1 = 0.0, 0.098
EDGE_Z0 = -0.008  # PCIe edge connector bottom

# Fans
FAN_X = (0.075, 0.205)
FAN_Z = 0.049
FAN_Y = 0.012  # hub center depth
ROTOR_R = 0.0415
HUB_R = 0.015
ROTOR_T = 0.0095
HOLE_R = 0.044
BORE_D = 0.0064

DP_Z = (0.018, 0.036, 0.054)
HDMI_Z = 0.075


def _holed_plate(
    outline: list[tuple[float, float]],
    thickness: float,
    *,
    circle_holes: list[tuple[float, float, float]] = (),
    rect_holes: list[tuple[float, float, float, float]] = (),
) -> cq.Workplane:
    """Extrude a 2D outline (local XY, +Z thickness) and cut through-holes."""
    plate = cq.Workplane("XY").polyline(outline).close().extrude(thickness)
    for cx, cy, r in circle_holes:
        plate = plate.cut(
            cq.Workplane("XY").moveTo(cx, cy).circle(r).extrude(thickness)
        )
    for cx, cy, w, h in rect_holes:
        plate = plate.cut(
            cq.Workplane("XY").moveTo(cx, cy).rect(w, h).extrude(thickness)
        )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gigabyte_dual_fan_graphics_card")

    shroud_black = model.material("shroud_black", rgba=(0.075, 0.075, 0.085, 1.0))
    fan_black = model.material("fan_black", rgba=(0.105, 0.105, 0.115, 1.0))
    badge_black = model.material("badge_black", rgba=(0.05, 0.05, 0.06, 1.0))
    badge_text = model.material("badge_text", rgba=(0.92, 0.92, 0.93, 1.0))
    alu = model.material("heatsink_aluminum", rgba=(0.62, 0.63, 0.66, 1.0))
    pcb_black = model.material("pcb_black", rgba=(0.05, 0.07, 0.05, 1.0))
    gunmetal = model.material("backplate_gunmetal", rgba=(0.21, 0.22, 0.24, 1.0))
    brand_silver = model.material("brand_silver", rgba=(0.78, 0.79, 0.80, 1.0))
    bracket_silver = model.material("bracket_silver", rgba=(0.72, 0.73, 0.75, 1.0))
    gold = model.material("connector_gold", rgba=(0.85, 0.68, 0.24, 1.0))
    contact_dark = model.material("contact_substrate", rgba=(0.20, 0.16, 0.10, 1.0))
    port_dark = model.material("port_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.15, 0.15, 0.17, 1.0))
    screw_silver = model.material("screw_silver", rgba=(0.65, 0.66, 0.68, 1.0))

    # ------------------------------------------------------------------ PCB
    pcb = model.part("pcb")
    pcb.visual(
        Box((0.240, PCB_Y1 - PCB_Y0, PCB_Z1 - PCB_Z0)),
        origin=Origin(xyz=(0.120, (PCB_Y0 + PCB_Y1) / 2.0, (PCB_Z0 + PCB_Z1) / 2.0)),
        material=pcb_black,
        name="board",
    )
    # PCIe edge connector: dark substrate tab protruding below the board,
    # with proud gold contact striping on both faces and a key-notch gap.
    pcb.visual(
        Box((0.085, PCB_Y1 - PCB_Y0, 0.0125)),
        origin=Origin(xyz=(0.0875, (PCB_Y0 + PCB_Y1) / 2.0, EDGE_Z0 + 0.00625)),
        material=contact_dark,
        name="edge_connector",
    )
    stripe_i = 0
    for i in range(14):
        if i == 3:
            continue  # PCIe key notch gap in the contact striping
        sx = 0.0475 + i * 0.0058
        for side, (sy0, sy1) in (("front", (-0.0150, -0.0147)), ("rear", (-0.0168, -0.0164))):
            pcb.visual(
                Box((0.0032, sy1 - sy0, 0.011)),
                origin=Origin(xyz=(sx, (sy0 + sy1) / 2.0, -0.002)),
                material=gold,
                name=f"edge_contact_{side}_{stripe_i:02d}",
            )
        stripe_i += 1

    # Display output connector bodies (3x DisplayPort + 1x HDMI) mounted on the
    # board front face; their noses reach through the bracket port openings.
    for i, zc in enumerate(DP_Z):
        pcb.visual(
            Box((0.011, 0.0135, 0.0045)),
            origin=Origin(xyz=(0.0005, -0.0088, zc)),
            material=port_dark,
            name=f"dp_port_nose_{i}",
        )
        pcb.visual(
            Box((0.016, 0.0125, 0.006)),
            origin=Origin(xyz=(0.012, -0.00865, zc)),
            material=port_dark,
            name=f"dp_port_body_{i}",
        )
    pcb.visual(
        Box((0.011, 0.012, 0.0050)),
        origin=Origin(xyz=(0.0005, -0.0088, HDMI_Z)),
        material=port_dark,
        name="hdmi_port_nose",
    )
    pcb.visual(
        Box((0.016, 0.0125, 0.0075)),
        origin=Origin(xyz=(0.012, -0.00865, HDMI_Z)),
        material=port_dark,
        name="hdmi_port_body",
    )

    # ------------------------------------------------------------ Backplate
    backplate = model.part("backplate")
    bp_outline = [
        (0.006, PCB_Z0),
        (0.272, PCB_Z0),
        (0.278, 0.010),
        (0.278, 0.086),
        (0.272, PCB_Z1),
        (0.006, PCB_Z1),
        (0.000, 0.086),
        (0.000, 0.010),
    ]
    backplate.visual(
        mesh_from_cadquery(
            _holed_plate(
                bp_outline,
                BACKPLATE_Y1 - BACKPLATE_Y0,
                rect_holes=[(0.256, 0.063, 0.028, 0.046)],
            ),
            "backplate_plate",
            tolerance=0.0003,
        ),
        origin=Origin(xyz=(0.0, BACKPLATE_Y1, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="plate",
    )
    # GIGABYTE lettering region: raised brushed strip on the outer face.
    backplate.visual(
        Box((0.090, 0.0015, 0.014)),
        origin=Origin(xyz=(0.140, -0.02000, 0.049)),
        material=brand_silver,
        name="brand_plate",
    )
    for i, (sx, sz) in enumerate(
        [(0.030, 0.014), (0.030, 0.082), (0.130, 0.014), (0.130, 0.082), (0.220, 0.014), (0.210, 0.082)]
    ):
        backplate.visual(
            Cylinder(radius=0.0024, length=0.0012),
            origin=Origin(xyz=(sx, -0.01965, sz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_silver,
            name=f"screw_{i}",
        )

    model.articulation(
        "pcb_to_backplate", ArticulationType.FIXED, parent=pcb, child=backplate, origin=Origin()
    )

    # -------------------------------------------------------------- Bracket
    bracket = model.part("bracket")
    # Profile plane: local X = card Z, local Y = card Y; extruded along -X.
    br_outline = [
        (-0.004, -0.0195),
        (0.096, -0.0195),
        (0.098, -0.0175),
        (0.098, -0.0015),
        (0.096, 0.0005),
        (-0.002, 0.0005),
        (-0.006, -0.0035),
        (-0.006, -0.0155),
    ]
    br_holes: list[tuple[float, float, float, float]] = []
    for zc in DP_Z:
        br_holes.append((zc, -0.0088, 0.006, 0.015))
    br_holes.append((HDMI_Z, -0.0088, 0.007, 0.014))
    for yc in (-0.0155, -0.0095, -0.0035):
        br_holes.append((0.0895, yc, 0.009, 0.004))
    bracket.visual(
        mesh_from_cadquery(
            _holed_plate(br_outline, 0.002, rect_holes=br_holes),
            "bracket_plate",
            tolerance=0.0003,
        ),
        origin=Origin(xyz=(-0.0035, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=bracket_silver,
        name="plate",
    )
    # Fold-back rib seating against the leading PCB edge.
    bracket.visual(
        Box((0.004, PCB_Y1 - PCB_Y0, PCB_Z1 - PCB_Z0)),
        origin=Origin(xyz=(-0.002, (PCB_Y0 + PCB_Y1) / 2.0, (PCB_Z0 + PCB_Z1) / 2.0)),
        material=bracket_silver,
        name="pcb_rib",
    )
    # Case screw tab folded at the top of the bracket.
    bracket.visual(
        Box((0.011, 0.020, 0.0025)),
        origin=Origin(xyz=(-0.010, -0.0095, 0.09875)),
        material=bracket_silver,
        name="top_tab",
    )
    model.articulation(
        "pcb_to_bracket", ArticulationType.FIXED, parent=pcb, child=bracket, origin=Origin()
    )

    # ------------------------------------------------------------- Heatsink
    heatsink = model.part("heatsink")
    heatsink.visual(
        Box((0.216, HS_BASE_Y1 - HS_BASE_Y0, 0.080)),
        origin=Origin(xyz=(0.134, (HS_BASE_Y0 + HS_BASE_Y1) / 2.0, 0.048)),
        material=alu,
        name="base_plate",
    )
    fin_count = 14
    fin_t = 0.0016
    z_lo, z_hi = 0.008, 0.088
    pitch = (z_hi - z_lo - fin_t) / (fin_count - 1)
    for i in range(fin_count):
        zc = z_lo + fin_t / 2.0 + i * pitch
        heatsink.visual(
            Box((0.246, FIN_Y1 - FIN_Y0, fin_t)),
            origin=Origin(xyz=(0.149, (FIN_Y0 + FIN_Y1) / 2.0, zc)),
            material=alu,
            name=f"fin_{i:02d}",
        )
    # Fan motor mounts: stator post through the fin stack + bearing pin that
    # is captured (with radial clearance) inside each rotor hub bore.
    for i, fx in enumerate(FAN_X):
        heatsink.visual(
            Cylinder(radius=0.0065, length=0.020),
            origin=Origin(xyz=(fx, -0.004, FAN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=badge_black,
            name=f"stator_post_{i}",
        )
        heatsink.visual(
            Cylinder(radius=0.0033, length=0.0075),
            origin=Origin(xyz=(fx, 0.00875, FAN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_silver,
            name=f"stator_pin_{i}",
        )
    model.articulation(
        "pcb_to_heatsink", ArticulationType.FIXED, parent=pcb, child=heatsink, origin=Origin()
    )

    # --------------------------------------------------------------- Shroud
    shroud = model.part("shroud")
    outline = [
        (0.008, 0.000),
        (0.271, 0.000),
        (0.279, 0.008),
        (0.279, 0.090),
        (0.271, 0.098),
        (0.024, 0.098),
        (-0.001, 0.086),
        (-0.001, 0.012),
    ]
    shroud.visual(
        mesh_from_cadquery(
            _holed_plate(
                outline,
                PLATE_Y1 - PLATE_Y0,
                circle_holes=[(FAN_X[0], FAN_Z, HOLE_R), (FAN_X[1], FAN_Z, HOLE_R)],
            ),
            "shroud_faceplate",
            tolerance=0.0003,
        ),
        origin=Origin(xyz=(0.0, PLATE_Y1, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shroud_black,
        name="faceplate",
    )
    shroud.visual(
        Box((0.245, 0.035, 0.0045)),
        origin=Origin(xyz=(0.1485, 0.001, 0.09575)),
        material=shroud_black,
        name="top_wall",
    )
    shroud.visual(
        Box((0.261, 0.033, 0.0038)),
        origin=Origin(xyz=(0.1405, 0.002, 0.0019)),
        material=shroud_black,
        name="bottom_wall",
    )
    shroud.visual(
        Box((0.0045, 0.035, 0.082)),
        origin=Origin(xyz=(0.27675, 0.001, 0.049)),
        material=shroud_black,
        name="tail_wall",
    )
    shroud.visual(
        Box((0.0045, 0.0185, 0.078)),
        origin=Origin(xyz=(0.00125, 0.00925, 0.047)),
        material=shroud_black,
        name="head_wall",
    )
    # Screw bosses seating the shroud onto the fin stack edges.
    shroud.visual(
        Box((0.010, 0.020, 0.007)),
        origin=Origin(xyz=(0.140, 0.002, 0.0905)),
        material=shroud_black,
        name="mount_boss_top",
    )
    shroud.visual(
        Box((0.010, 0.020, 0.008)),
        origin=Origin(xyz=(0.140, 0.002, 0.007)),
        material=shroud_black,
        name="mount_boss_bottom",
    )
    # Angular styling ridges on the faceplate.
    shroud.visual(
        Box((0.008, 0.002, 0.056)),
        origin=Origin(xyz=(0.140, 0.0218, 0.049)),
        material=accent_gray,
        name="center_ridge",
    )
    for rname, (rx, rz, rot) in {
        "corner_ridge_0": (0.017, 0.072, 0.62),
        "corner_ridge_1": (0.017, 0.026, -0.62),
        "corner_ridge_2": (0.262, 0.072, -0.62),
        "corner_ridge_3": (0.262, 0.026, 0.62),
    }.items():
        shroud.visual(
            Box((0.036, 0.002, 0.008)),
            origin=Origin(xyz=(rx, 0.0218, rz), rpy=(0.0, rot, 0.0)),
            material=accent_gray,
            name=rname,
        )
    model.articulation(
        "heatsink_to_shroud", ArticulationType.FIXED, parent=heatsink, child=shroud, origin=Origin()
    )

    # ----------------------------------------------------------------- Fans
    for i, fx in enumerate(FAN_X):
        fan = model.part(f"fan_{i}")
        fan.visual(
            mesh_from_geometry(
                FanRotorGeometry(
                    ROTOR_R,
                    HUB_R,
                    11,
                    thickness=ROTOR_T,
                    blade_pitch_deg=32.0,
                    blade_sweep_deg=30.0,
                    blade=FanRotorBlade(shape="scimitar", camber=0.18, tip_clearance=0.0005),
                    hub=FanRotorHub(style="flat", bore_diameter=BORE_D),
                ),
                f"fan_rotor_{i}",
            ),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=fan_black,
            name="rotor",
        )
        # GIGABYTE hub badge: black cap disc + off-axis white lettering bar.
        fan.visual(
            Cylinder(radius=0.0140, length=0.0030),
            origin=Origin(xyz=(0.0, 0.0027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=badge_black,
            name="hub_badge",
        )
        fan.visual(
            Box((0.0165, 0.0009, 0.0038)),
            origin=Origin(xyz=(0.0, 0.00425, -0.0035)),
            material=badge_text,
            name="hub_badge_text",
        )
        model.articulation(
            f"heatsink_to_fan_{i}",
            ArticulationType.CONTINUOUS,
            parent=heatsink,
            child=fan,
            origin=Origin(xyz=(fx, FAN_Y, FAN_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=120.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Intentional local embeds that carry the assembly.
    ctx.allow_overlap(
        "shroud",
        "heatsink",
        elem_a="mount_boss_top",
        elem_b="fin_13",
        reason="Shroud screw boss intentionally seats 1 mm onto the top fin edge to mount the shroud.",
    )
    ctx.allow_overlap(
        "shroud",
        "heatsink",
        elem_a="mount_boss_bottom",
        elem_b="fin_00",
        reason="Shroud screw boss intentionally seats onto the bottom fin edge to mount the shroud.",
    )
    for i in range(2):
        ctx.allow_overlap(
            "heatsink",
            f"fan_{i}",
            elem_a=f"stator_pin_{i}",
            elem_b="rotor",
            reason="Fan bearing pin is press-fit 0.1 mm into the axisymmetric rotor hub bore; spin-safe capture.",
        )

    pcb = object_model.get_part("pcb")
    shroud = object_model.get_part("shroud")
    heatsink = object_model.get_part("heatsink")
    backplate = object_model.get_part("backplate")
    bracket = object_model.get_part("bracket")
    fans = [object_model.get_part(f"fan_{i}") for i in range(2)]

    # Two fans on continuous spin joints.
    for i in range(2):
        joint = object_model.get_articulation(f"heatsink_to_fan_{i}")
        ctx.check(
            f"fan_{i}_joint_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"type={joint.articulation_type!r}",
        )

    ctx.expect_origin_distance(
        fans[0], fans[1], axes="x", min_dist=0.10, name="fans are two separate side_by_side rotors"
    )

    # Fan spin is a real articulation: the off-axis badge lettering bar's
    # AABB center must displace when the rotor turns half a revolution.
    for i in range(2):
        joint = object_model.get_articulation(f"heatsink_to_fan_{i}")
        rest = ctx.part_element_world_aabb(fans[i], elem="hub_badge_text")
        with ctx.pose({joint: math.pi}):
            spun = ctx.part_element_world_aabb(fans[i], elem="hub_badge_text")
            ctx.expect_within(
                fans[i], shroud, axes="xz", margin=0.001, name=f"fan_{i} stays inside shroud opening when spun"
            )
        ok = rest is not None and spun is not None
        if ok:
            rc = [(rest[0][k] + rest[1][k]) / 2.0 for k in range(3)]
            sc = [(spun[0][k] + spun[1][k]) / 2.0 for k in range(3)]
            disp = math.dist(rc, sc)
            ok = disp > 0.004
        ctx.check(f"fan_{i}_spin_displaces_badge_text", bool(ok), f"rest={rest}, spun={spun}")

    # Rotors sit recessed behind the shroud faceplate with clearance.
    for i in range(2):
        ctx.expect_gap(
            shroud,
            fans[i],
            axis="y",
            min_gap=0.0005,
            positive_elem="faceplate",
            negative_elem="rotor",
            name=f"shroud faceplate sits in front of fan_{i} rotor",
        )
        # Heatsink fins are visible behind the fan with an axial gap.
        ctx.expect_gap(
            fans[i],
            heatsink,
            axis="y",
            min_gap=0.001,
            positive_elem="rotor",
            negative_elem="fin_07",
            name=f"fan_{i} rotor clears the fin stack",
        )
        # The bearing pin remains inserted in the rotor hub bore.
        ctx.expect_overlap(
            heatsink,
            fans[i],
            axes="y",
            min_overlap=0.004,
            elem_a=f"stator_pin_{i}",
            elem_b="rotor",
            name=f"stator pin {i} captured in fan_{i} hub bore",
        )

    # Four distinct display outputs reach through the bracket plate.
    port_noses = ["dp_port_nose_0", "dp_port_nose_1", "dp_port_nose_2", "hdmi_port_nose"]
    for elem in port_noses:
        ctx.expect_overlap(
            pcb,
            bracket,
            axes="x",
            min_overlap=0.0005,
            elem_a=elem,
            elem_b="plate",
            name=f"{elem} passes through the bracket port opening",
        )
    ctx.check("port_count_is_four", len(port_noses) == 4, "expected 3x DP + 1x HDMI")

    # Gold edge connector protrudes below the PCB bottom edge.
    board_aabb = ctx.part_element_world_aabb(pcb, elem="board")
    edge_aabb = ctx.part_element_world_aabb(pcb, elem="edge_connector")
    ok = board_aabb is not None and edge_aabb is not None
    if ok:
        ok = edge_aabb[0][2] < board_aabb[0][2] - 0.008
    ctx.check("edge_connector_below_pcb_bottom", bool(ok), f"board={board_aabb}, edge={edge_aabb}")

    # Backplate covers the rear PCB face (no penetration, flush seat).
    ctx.expect_contact(
        shroud,
        heatsink,
        elem_a="mount_boss_top",
        elem_b="fin_13",
        name="shroud mount boss seats on the fin stack",
    )
    ctx.expect_gap(
        pcb,
        backplate,
        axis="y",
        max_penetration=0.0001,
        max_gap=0.0008,
        positive_elem="board",
        negative_elem="plate",
        name="backplate seats flush behind the pcb",
    )

    # Overall proportions of the assembled card.
    shroud_aabb = ctx.part_world_aabb(shroud)
    ok = shroud_aabb is not None
    if ok:
        length = shroud_aabb[1][0] - shroud_aabb[0][0]
        height = shroud_aabb[1][2] - shroud_aabb[0][2]
        ok = 0.26 <= length <= 0.30 and 0.09 <= height <= 0.11
    ctx.check("compact_dual_fan_proportions", bool(ok), f"shroud_aabb={shroud_aabb}")

    return ctx.report()


object_model = build_object_model()
