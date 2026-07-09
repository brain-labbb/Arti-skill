from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# ---------------------------------------------------------------------------
# Global layout (world frame, meters)
# ---------------------------------------------------------------------------
# Drip tray: 0.30 m deep (X, +X = front/user side), 0.40 m wide (Y), 0.03 m tall.
TRAY_DEPTH = 0.30
TRAY_WIDTH = 0.40
TRAY_HEIGHT = 0.030
TRAY_FLOOR_T = 0.008
TRAY_WALL_T = 0.010
PLATE_TOP_Z = 0.028  # perforated plate top, recessed 2 mm below the rim
PLATE_T = 0.004

# Tower column near the rear edge of the tray.
COLUMN_X = -0.080
COLUMN_R = 0.0375  # 0.075 m diameter
SKIRT_R = 0.055
SKIRT_H = 0.034
COLUMN_TOP = 0.342  # local to the skirt base; cap brings total to 0.350
CAP_T = 0.008

# Faucet (local frame: origin on the column axis at spout-line height).
FAUCET_Z = 0.315  # above the skirt base -> world z = 0.343, just below the cap
BODY_R = 0.013
BODY_X0 = 0.020  # shank starts inside the column wall (captured pass-through)
BODY_X1 = 0.118  # ~0.08 m forward of the column surface (x = 0.0375)
SPOUT_START_X = 0.115
SPOUT_BEND_R = 0.020
BONNET_X = 0.082
PIVOT_Z = 0.031  # lever pivot height above the faucet axis

# Push-button valve layout (replaces the lever handle in this variant).
BUTTON_R = 0.014  # button cap radius (~28 mm diameter)
BUTTON_CAP_H = 0.012  # visible cap height above the bezel
BUTTON_STEM_R = 0.009  # stem radius (slides inside the bonnet bore)
BUTTON_STEM_H = 0.022  # stem length (retained insertion into the bonnet)
BEZEL_R = 0.018  # chrome bezel ring outer radius
BEZEL_H = 0.005  # bezel ring height
BUTTON_TRAVEL = 0.010  # 10 mm press-in travel


def _tray_basin_geometry() -> MeshGeometry:
    """Hollow rounded-rectangle basin: floor plate plus perimeter wall ring."""
    outer = rounded_rect_profile(TRAY_DEPTH, TRAY_WIDTH, 0.030)
    inner = rounded_rect_profile(
        TRAY_DEPTH - 2.0 * TRAY_WALL_T, TRAY_WIDTH - 2.0 * TRAY_WALL_T, 0.022
    )
    floor = ExtrudeGeometry(outer, TRAY_FLOOR_T, center=True).translate(
        0.0, 0.0, TRAY_FLOOR_T / 2.0
    )
    wall = ExtrudeWithHolesGeometry(
        outer, [inner], TRAY_HEIGHT - 0.006, center=True
    ).translate(0.0, 0.0, 0.006 + (TRAY_HEIGHT - 0.006) / 2.0)
    floor.merge(wall)
    return floor


def _spout_geometry() -> MeshGeometry:
    """Downward-curving tapered spout built as a manual ring-loft tube.

    The bend runs from a horizontal +X tangent to a vertical -Z tangent, then
    a short straight tapered drop. Side quads and fan caps are wound so that
    every face normal points outward.
    """
    n_pts = 24
    n_bend = 10
    cx, cz = SPOUT_START_X, -SPOUT_BEND_R  # bend center (XZ plane)
    specs: list[tuple[float, float, float, float]] = []  # (t, center_x, center_z, r)
    for i in range(n_bend + 1):
        t = (math.pi / 2.0) * i / n_bend
        r = 0.0105 + (0.0080 - 0.0105) * (i / n_bend)
        specs.append((t, cx + SPOUT_BEND_R * math.sin(t), cz + SPOUT_BEND_R * math.cos(t), r))
    # Straight tapered drop below the bend.
    specs.append((math.pi / 2.0, cx + SPOUT_BEND_R, cz - 0.008, 0.0074))
    specs.append((math.pi / 2.0, cx + SPOUT_BEND_R, cz - 0.014, 0.0068))

    geom = MeshGeometry()
    ring_ids: list[list[int]] = []
    for t, rcx, rcz, r in specs:
        # In-plane basis: U = +Y, W = (sin t, 0, cos t); path tangent is
        # T = (cos t, 0, -sin t). Points P = C + r cos(a) U + r sin(a) W.
        wx, wz = math.sin(t), math.cos(t)
        ids: list[int] = []
        for j in range(n_pts):
            a = 2.0 * math.pi * j / n_pts
            px = rcx + r * math.sin(a) * wx
            py = r * math.cos(a)
            pz = rcz + r * math.sin(a) * wz
            ids.append(geom.add_vertex(px, py, pz))
        ring_ids.append(ids)

    # Side walls (outward winding: theta-increasing then path-increasing).
    for i in range(len(ring_ids) - 1):
        ra, rb = ring_ids[i], ring_ids[i + 1]
        for j in range(n_pts):
            j2 = (j + 1) % n_pts
            geom.add_face(ra[j], ra[j2], rb[j2])
            geom.add_face(ra[j], rb[j2], rb[j])

    # End caps (fan around the ring center, outward along -T / +T).
    t0, c0x, c0z, _ = specs[0]
    start_center = geom.add_vertex(c0x, 0.0, c0z)
    for j in range(n_pts):
        j2 = (j + 1) % n_pts
        geom.add_face(start_center, ring_ids[0][j2], ring_ids[0][j])
    tn, cnx, cnz, _ = specs[-1]
    end_center = geom.add_vertex(cnx, 0.0, cnz)
    last = ring_ids[-1]
    for j in range(n_pts):
        j2 = (j + 1) % n_pts
        geom.add_face(end_center, last[j], last[j2])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_faucet_beverage_tower_push_button")

    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.67, 0.69, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.80, 0.81, 0.84, 1.0))
    chrome = model.material("chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark_matte = model.material("dark_matte", rgba=(0.16, 0.16, 0.17, 1.0))
    gloss_black = model.material("gloss_black", rgba=(0.05, 0.05, 0.06, 1.0))

    # ------------------------------------------------------------------ tray
    tray = model.part("drip_tray")
    tray.visual(
        mesh_from_geometry(_tray_basin_geometry(), "tray_basin"),
        material=brushed_steel,
        name="tray_basin",
    )
    plate_geom = PerforatedPanelGeometry(
        (TRAY_DEPTH - 0.016, TRAY_WIDTH - 0.016),
        PLATE_T,
        hole_diameter=0.005,
        pitch=0.015,
        frame=0.012,
        corner_radius=0.018,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(plate_geom, "tray_perforated_plate"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z - PLATE_T / 2.0)),
        material=brushed_steel,
        name="tray_perforated_plate",
    )

    # ---------------------------------------------------------------- column
    tower = model.part("tower_column")
    skirt_geom = LatheGeometry(
        [(SKIRT_R, 0.0), (0.047, 0.010), (0.0405, 0.022), (COLUMN_R, SKIRT_H)],
        segments=48,
    )
    tower.visual(
        mesh_from_geometry(skirt_geom, "base_skirt"),
        material=dark_matte,
        name="base_skirt",
    )
    tower.visual(
        mesh_from_geometry(
            CylinderGeometry(COLUMN_R, COLUMN_TOP - 0.020, radial_segments=48).translate(
                0.0, 0.0, 0.020 + (COLUMN_TOP - 0.020) / 2.0
            ),
            "column_shell",
        ),
        material=dark_matte,
        name="column_shell",
    )
    tower.visual(
        mesh_from_geometry(
            CylinderGeometry(0.0385, 0.012, radial_segments=48).translate(0.0, 0.0, 0.336),
            "cap_band",
        ),
        material=brushed_steel,
        name="cap_band",
    )
    tower.visual(
        mesh_from_geometry(
            CylinderGeometry(0.0390, CAP_T, radial_segments=48).translate(
                0.0, 0.0, COLUMN_TOP + CAP_T / 2.0
            ),
            "top_cap",
        ),
        material=polished_steel,
        name="top_cap",
    )

    model.articulation(
        "tray_to_tower",
        ArticulationType.FIXED,
        parent=tray,
        child=tower,
        origin=Origin(xyz=(COLUMN_X, 0.0, PLATE_TOP_Z)),
    )

    # ---------------------------------------------------------------- faucet
    faucet = model.part("faucet")
    body_geom = (
        CylinderGeometry(BODY_R, BODY_X1 - BODY_X0, radial_segments=32)
        .rotate_y(math.pi / 2.0)
        .translate((BODY_X0 + BODY_X1) / 2.0, 0.0, 0.0)
    )
    flange_geom = (
        CylinderGeometry(0.0165, 0.014, radial_segments=32)
        .rotate_y(math.pi / 2.0)
        .translate(COLUMN_R + 0.007, 0.0, 0.0)
    )
    body_geom.merge(flange_geom)
    faucet.visual(
        mesh_from_geometry(body_geom, "faucet_body"),
        material=chrome,
        name="faucet_body",
    )
    bonnet_geom = CylinderGeometry(0.012, 0.019, radial_segments=32).translate(
        BONNET_X, 0.0, 0.0175
    )
    bonnet_geom.merge(
        CylinderGeometry(0.0145, 0.005, radial_segments=32).translate(BONNET_X, 0.0, 0.0105)
    )
    faucet.visual(
        mesh_from_geometry(bonnet_geom, "faucet_bonnet"),
        material=chrome,
        name="faucet_bonnet",
    )
    faucet.visual(
        mesh_from_geometry(_spout_geometry(), "faucet_spout"),
        material=chrome,
        name="faucet_spout",
    )

    model.articulation(
        "tower_to_faucet",
        ArticulationType.FIXED,
        parent=tower,
        child=faucet,
        origin=Origin(xyz=(0.0, 0.0, FAUCET_Z)),
    )

    # -------------------------------------------- push-button dispense valve
    # Bezel ring sits on top of the bonnet as a fixed chrome trim piece.
    # A short skirt extends into the bonnet top so the bezel mesh connects
    # with the bonnet mesh as one assembly.
    bonnet_top_z = 0.0175 + 0.019 / 2.0  # bonnet cylinder top in faucet-local Z
    bezel_geom = (
        LatheGeometry(
            [
                (BUTTON_STEM_R + 0.001, -0.003),
                (BEZEL_R - 0.001, -0.003),
                (BEZEL_R, -0.002),
                (BEZEL_R, 0.0),
                (BEZEL_R, BEZEL_H),
                (BEZEL_R - 0.002, BEZEL_H + 0.001),
                (BUTTON_STEM_R + 0.001, BEZEL_H + 0.001),
            ],
            segments=40,
        )
        .translate(BONNET_X, 0.0, bonnet_top_z)
    )
    faucet.visual(
        mesh_from_geometry(bezel_geom, "button_bezel"),
        material=chrome,
        name="button_bezel",
    )

    button = model.part("push_button")
    # Stem extends downward from the button cap into the bonnet bore for
    # retained insertion through the full press travel. The top of the stem
    # overlaps slightly with the cap bottom so the two meshes form one
    # connected assembly.
    stem_geom = (
        CylinderGeometry(BUTTON_STEM_R, BUTTON_STEM_H, radial_segments=32)
        .translate(0.0, 0.0, -BUTTON_STEM_H / 2.0 + 0.001)
    )
    button.visual(
        mesh_from_geometry(stem_geom, "button_stem"),
        material=chrome,
        name="button_stem",
    )
    # Domed cap: solid lathe dome for a glossy black push-button face.
    # The solid bottom face overlaps with the stem top so the two visuals
    # form one connected mesh assembly.
    cap_geom = LatheGeometry(
        [
            (0.000, BUTTON_CAP_H + 0.004),
            (BUTTON_R * 0.35, BUTTON_CAP_H + 0.003),
            (BUTTON_R * 0.70, BUTTON_CAP_H),
            (BUTTON_R, BUTTON_CAP_H - 0.003),
            (BUTTON_R + 0.001, BUTTON_CAP_H - 0.005),
            (BUTTON_R + 0.001, 0.0),
            (0.0, 0.0),
        ],
        segments=40,
    )
    button.visual(
        mesh_from_geometry(cap_geom, "button_cap"),
        material=gloss_black,
        name="button_cap",
    )

    # PRISMATIC: positive q presses the button straight down (-Z) into the
    # bonnet to dispense. The joint origin sits at the bezel top surface so
    # the button cap rests just above it at q=0.
    model.articulation(
        "faucet_button",
        ArticulationType.PRISMATIC,
        parent=faucet,
        child=button,
        origin=Origin(xyz=(BONNET_X, 0.0, bonnet_top_z + BEZEL_H + 0.001)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.5, lower=0.0, upper=BUTTON_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray = object_model.get_part("drip_tray")
    tower = object_model.get_part("tower_column")
    faucet = object_model.get_part("faucet")
    button = object_model.get_part("push_button")
    button_joint = object_model.get_articulation("faucet_button")

    ctx.allow_overlap(
        faucet,
        tower,
        elem_a="faucet_body",
        elem_b="column_shell",
        reason="The faucet shank intentionally passes through the column wall, "
        "like a real draft-tower faucet threaded into the tower.",
    )
    ctx.allow_overlap(
        button,
        faucet,
        elem_a="button_stem",
        elem_b="faucet_bonnet",
        reason="The button stem intentionally slides inside the faucet bonnet "
        "bore as a retained prismatic insertion.",
    )

    # Hero: perforated top plate sits recessed inside the tray rim.
    plate_bb = ctx.part_element_world_aabb(tray, elem="tray_perforated_plate")
    basin_bb = ctx.part_element_world_aabb(tray, elem="tray_basin")
    ctx.check(
        "perforated plate sits recessed inside the tray rim",
        plate_bb is not None
        and basin_bb is not None
        and plate_bb[1][2] < basin_bb[1][2] - 0.0005
        and plate_bb[0][0] > basin_bb[0][0]
        and plate_bb[1][0] < basin_bb[1][0]
        and plate_bb[0][1] > basin_bb[0][1]
        and plate_bb[1][1] < basin_bb[1][1],
        details=f"plate={plate_bb}, basin={basin_bb}",
    )

    # Column seated on the tray near its rear edge.
    ctx.expect_contact(
        tower,
        tray,
        elem_a="base_skirt",
        elem_b="tray_perforated_plate",
        contact_tol=5e-4,
        name="column base skirt seats on the tray plate",
    )
    ctx.expect_within(
        tower,
        tray,
        axes="xy",
        inner_elem="base_skirt",
        outer_elem="tray_basin",
        name="column stands inside the tray footprint",
    )
    ctx.expect_origin_gap(
        tray,
        tower,
        axis="x",
        min_gap=0.05,
        name="column is set toward the tray rear edge",
    )

    tower_bb = ctx.part_world_aabb(tower)
    ctx.check(
        "column rises about 0.35 m above the tray",
        tower_bb is not None and 0.36 <= tower_bb[1][2] <= 0.40,
        details=f"tower={tower_bb}",
    )

    # Faucet projects forward just below the cap and the spout curves down
    # over the drip tray.
    spout_bb = ctx.part_element_world_aabb(faucet, elem="faucet_spout")
    body_bb = ctx.part_element_world_aabb(faucet, elem="faucet_body")
    ctx.check(
        "faucet body projects forward of the column",
        body_bb is not None
        and tower_bb is not None
        and body_bb[1][0] > tower_bb[1][0] + 0.05,
        details=f"body={body_bb}, tower={tower_bb}",
    )
    ctx.check(
        "spout curves downward below the faucet body",
        spout_bb is not None and body_bb is not None and spout_bb[0][2] < body_bb[0][2] - 0.005,
        details=f"spout={spout_bb}, body={body_bb}",
    )
    ctx.expect_within(
        faucet,
        tray,
        axes="xy",
        inner_elem="faucet_spout",
        outer_elem="tray_basin",
        name="spout dispenses over the drip tray",
    )
    ctx.expect_gap(
        faucet,
        tray,
        axis="z",
        positive_elem="faucet_spout",
        negative_elem="tray_perforated_plate",
        min_gap=0.20,
        name="spout hangs well above the tray plate",
    )

    # Push-button assembly mounted on the faucet bonnet.
    ctx.expect_contact(
        button,
        faucet,
        elem_a="button_stem",
        elem_b="faucet_bonnet",
        name="button stem seats inside the faucet bonnet",
    )
    cap_bb = ctx.part_element_world_aabb(button, elem="button_cap")
    ctx.check(
        "button cap sits above the faucet body",
        cap_bb is not None and body_bb is not None and cap_bb[0][2] > body_bb[1][2],
        details=f"cap={cap_bb}, body={body_bb}",
    )
    ctx.expect_within(
        button,
        faucet,
        axes="xy",
        inner_elem="button_cap",
        outer_elem="faucet_bonnet",
        margin=0.002,
        name="button cap aligns over the faucet bonnet",
    )
    bezel_bb = ctx.part_element_world_aabb(faucet, elem="button_bezel")
    ctx.check(
        "chrome bezel ring sits between button and bonnet",
        bezel_bb is not None
        and body_bb is not None
        and cap_bb is not None
        and bezel_bb[0][2] > body_bb[1][2] - 0.001
        and bezel_bb[1][2] < cap_bb[1][2] + 0.001,
        details=f"bezel={bezel_bb}, body={body_bb}, cap={cap_bb}",
    )

    # Button joint: PRISMATIC straight down, ~10 mm press-in travel.
    axis_ok = (
        abs(button_joint.axis[0]) < 1e-6
        and abs(button_joint.axis[1]) < 1e-6
        and abs(button_joint.axis[2] + 1.0) < 1e-6
    )
    ctx.check(
        "button presses straight in (-Z axis)",
        axis_ok,
        details=f"axis={button_joint.axis}",
    )
    ctx.check(
        "button joint is prismatic",
        button_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={button_joint.articulation_type}",
    )
    limits = button_joint.motion_limits
    ctx.check(
        "button travel is about 10 mm",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-6
        and 0.006 <= limits.upper <= 0.015,
        details=f"limits={limits}",
    )

    # Pressed pose: button actually moves downward.
    rest_bb = ctx.part_world_aabb(button)
    upper = limits.upper if limits is not None and limits.upper is not None else BUTTON_TRAVEL
    with ctx.pose({button_joint: upper}):
        pressed_bb = ctx.part_world_aabb(button)
        ctx.check(
            "pressed button moves downward",
            rest_bb is not None
            and pressed_bb is not None
            and pressed_bb[0][2] < rest_bb[0][2] - 0.005,
            details=f"rest={rest_bb}, pressed={pressed_bb}",
        )
        ctx.check(
            "pressed button does not move laterally",
            rest_bb is not None
            and pressed_bb is not None
            and abs(pressed_bb[0][0] - rest_bb[0][0]) < 0.001
            and abs(pressed_bb[0][1] - rest_bb[0][1]) < 0.001,
            details=f"rest={rest_bb}, pressed={pressed_bb}",
        )
        # Retained insertion: stem still engaged inside the bonnet at full press.
        ctx.expect_overlap(
            button,
            faucet,
            axes="z",
            elem_a="button_stem",
            elem_b="faucet_bonnet",
            min_overlap=0.005,
            name="stem remains inserted in the bonnet at full press",
        )

    return ctx.report()


object_model = build_object_model()
