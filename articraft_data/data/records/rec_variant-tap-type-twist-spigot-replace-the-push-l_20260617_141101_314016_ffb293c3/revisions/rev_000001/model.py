from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
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

# Twist spigot valve (variant: replaces the push-lever tap).
SPIGOT_X = BONNET_X  # valve stem is coaxial with the bonnet
SPIGOT_KNOB_Z = 0.055  # knob center height above the faucet axis
SPIGOT_TRAVEL = math.radians(90.0)  # quarter-turn valve
N_FAUCETS = 1


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
    model = ArticulatedObject(name="single_faucet_beverage_tower")

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

    # --------------------------------------------------------- spigot knob(s)
    # Twist-spigot valve knob: replaces the push-lever tap with a quarter-turn
    # knob that rotates about the spout axis (X) to open/close the valve.
    spigot_parts: list = []
    spigot_joints: list = []

    def _spigot_stem_geometry() -> MeshGeometry:
        """Valve stem reaching from the bonnet top up toward the knob, with a
        retaining collar at the base."""
        # The stem must span from the bonnet top (world z ≈ 0.370) up to the
        # knob bottom (≈ 0.398 - 0.017 = 0.381) in the spigot part frame
        # whose origin is at world z = 0.398.
        stem_len = 0.030  # total stem reach below part origin
        stem = CylinderGeometry(0.005, stem_len, radial_segments=24).translate(
            0.0, 0.0, -stem_len / 2.0
        )
        # Collar at the bottom, just above the bonnet surface.
        collar = CylinderGeometry(0.008, 0.004, radial_segments=24).translate(
            0.0, 0.0, -stem_len + 0.002
        )
        stem.merge(collar)
        return stem

    def _spigot_knob_geometry() -> MeshGeometry:
        """Tapered fluted twist-spigot knob, axis rotated to local +X."""
        knob = KnobGeometry(
            0.034,
            0.020,
            body_style="tapered",
            top_diameter=0.026,
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised"),
            bore=KnobBore(style="round", diameter=0.006),
            center=True,
        )
        # KnobGeometry builds on local Z; rotate so the twist axis is +X
        # (the spout/flow axis).
        return knob.rotate_y(math.pi / 2.0)

    for i in range(N_FAUCETS):
        spigot = model.part(f"spigot_knob_{i}")
        spigot.visual(
            mesh_from_geometry(_spigot_stem_geometry(), f"spigot_stem_{i}"),
            material=chrome,
            name=f"spigot_stem_{i}",
        )
        spigot.visual(
            mesh_from_geometry(_spigot_knob_geometry(), f"spigot_knob_{i}"),
            material=polished_steel,
            name=f"spigot_knob_{i}",
        )
        joint = model.articulation(
            f"faucet_spigot_{i}",
            ArticulationType.REVOLUTE,
            parent=faucet,
            child=spigot,
            origin=Origin(xyz=(SPIGOT_X, 0.0, SPIGOT_KNOB_Z)),
            # Spout axis (+X): positive q opens the quarter-turn valve.
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0, velocity=4.0, lower=0.0, upper=SPIGOT_TRAVEL
            ),
        )
        spigot_parts.append(spigot)
        spigot_joints.append(joint)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray = object_model.get_part("drip_tray")
    tower = object_model.get_part("tower_column")
    faucet = object_model.get_part("faucet")

    spigot_parts = [object_model.get_part(f"spigot_knob_{i}") for i in range(N_FAUCETS)]
    spigot_joints = [
        object_model.get_articulation(f"faucet_spigot_{i}") for i in range(N_FAUCETS)
    ]

    ctx.allow_overlap(
        faucet,
        tower,
        elem_a="faucet_body",
        elem_b="column_shell",
        reason="The faucet shank intentionally passes through the column wall, "
        "like a real draft-tower faucet threaded into the tower.",
    )
    for i in range(N_FAUCETS):
        ctx.allow_overlap(
            spigot_parts[i],
            faucet,
            elem_a=f"spigot_stem_{i}",
            elem_b="faucet_bonnet",
            reason="The spigot valve stem is captured inside the faucet bonnet boss, "
            "like a real twist-spigot valve shaft seated in its housing.",
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

    # -------------------------------------------------- spigot knob variant
    for i in range(N_FAUCETS):
        spigot = spigot_parts[i]
        joint = spigot_joints[i]

        # Knob sits above the faucet body on the bonnet.
        knob_bb = ctx.part_element_world_aabb(spigot, elem=f"spigot_knob_{i}")
        ctx.check(
            f"spigot_knob_{i} sits above the faucet body",
            knob_bb is not None
            and body_bb is not None
            and knob_bb[0][2] > body_bb[1][2] - 0.005,
            details=f"knob={knob_bb}, body={body_bb}",
        )

        # Stem connects the knob to the bonnet area.
        ctx.expect_contact(
            spigot,
            faucet,
            elem_a=f"spigot_stem_{i}",
            elem_b="faucet_bonnet",
            contact_tol=0.005,
            name=f"spigot stem {i} reaches the faucet bonnet",
        )

        # Twist-spigot: REVOLUTE about the spout axis (+X), quarter-turn travel.
        axis_ok = (
            abs(abs(joint.axis[0]) - 1.0) < 1e-6
            and abs(joint.axis[1]) < 1e-6
            and abs(joint.axis[2]) < 1e-6
        )
        ctx.check(
            f"spigot_knob_{i} axis is along the spout axis (X)",
            axis_ok,
            details=f"axis={joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"spigot_knob_{i} has quarter-turn travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower) < 1e-6
            and abs(limits.upper - math.radians(90.0)) < 0.05,
            details=f"limits={limits}",
        )

        # Pose: turning the knob visibly rotates the knob body.
        rest_bb = ctx.part_world_aabb(spigot)
        upper = (
            limits.upper
            if limits is not None and limits.upper is not None
            else SPIGOT_TRAVEL
        )
        with ctx.pose({joint: upper}):
            open_bb = ctx.part_world_aabb(spigot)
            # At 90° twist about X the Y-extent and Z-extent of the knob swap
            # relative to rest. We check the AABB center stays near the same
            # position but the bounding box dims change.
            rest_dy = rest_bb[1][1] - rest_bb[0][1] if rest_bb else 0.0
            rest_dz = rest_bb[1][2] - rest_bb[0][2] if rest_bb else 0.0
            open_dy = open_bb[1][1] - open_bb[0][1] if open_bb else 0.0
            open_dz = open_bb[1][2] - open_bb[0][2] if open_bb else 0.0
            ctx.check(
                f"spigot_knob_{i} visibly rotates when turned",
                rest_bb is not None
                and open_bb is not None
                and (
                    abs(open_dy - rest_dz) < 0.010
                    or abs(open_dz - rest_dy) < 0.010
                    or abs(open_dy - rest_dy) > 0.004
                ),
                details=(
                    f"rest_dy={rest_dy:.4f} rest_dz={rest_dz:.4f} "
                    f"open_dy={open_dy:.4f} open_dz={open_dz:.4f}"
                ),
            )

    return ctx.report()


object_model = build_object_model()
