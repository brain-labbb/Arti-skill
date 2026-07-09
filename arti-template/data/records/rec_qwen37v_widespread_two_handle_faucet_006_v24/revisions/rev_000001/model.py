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
# Widespread two-handle faucet with cylindrical lever handles, mirror chrome.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered cylinder base (R 0.035 -> 0.023,
#     0.080 tall), cylindrical cap, flat-topped waterfall spout reaching
#     ~0.18 m forward, oval finial on top; SWIVELS on continuous vertical joint
#   - valve columns at x = +/-0.15: tapered cylinder pedestals (R 0.030 ->
#     0.018, 0.070 tall) with cylindrical cap and slim stem carrying a
#     cylindrical lever handle (~0.050 m long bar with ball end)
#   - narrow dark seam rings at all three deck bases
# Articulation: each lever handle revolute about its vertical stem axis
# (-pi..pi); the spout body swivels via a continuous vertical joint.
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Seam rings (narrow dark gaskets between chrome bases and deck)
SEAM_H = 0.002

# Center column
C_BASE_R = 0.035       # radius at deck
C_TOP_R = 0.023        # radius at top
C_COL_H = 0.080        # column height
C_SEAM_R = 0.036       # seam ring radius (slightly proud of base)
CAP_R = 0.025
CAP_H = 0.010
CAP_TOP_Z = SEAM_H + C_COL_H + CAP_H  # 0.092

# Valve columns
V_BASE_R = 0.030
V_TOP_R = 0.018
V_COL_H = 0.070
V_SEAM_R = 0.031
V_CAP_R = 0.020
V_CAP_H = 0.008
V_STEM_R = 0.006
V_STEM_TOP_Z = SEAM_H + V_COL_H + V_CAP_H + 0.018  # 0.098
HANDLE_JOINT_Z = V_STEM_TOP_Z - 0.005               # 0.093, hub captures stem

# Lever handle
HUB_R = 0.010
HUB_H = 0.020
LEVER_R = 0.005
LEVER_LEN = 0.050      # horizontal lever bar length
LEVER_TIP_R = 0.0065   # ball end

# Spout
SPOUT_WIDTH = 0.050

# Finial diverter
FINIAL_RX = 0.018
FINIAL_RY = 0.012
FINIAL_RZ = 0.008
FINIAL_STEM_R = 0.0045
FINIAL_CENTER_Z = 0.014


def _tapered_cylinder(base_r: float, top_r: float, height: float) -> cq.Workplane:
    """Tapered cylinder pedestal, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .circle(base_r)
        .workplane(offset=height)
        .circle(top_r)
        .loft(combine=True)
    )


def _waterfall_spout() -> cq.Workplane:
    """Wide flat-topped spout sweeping forward (+Y) into a waterfall arc.

    Side profile in the YZ plane, extruded across X for slab sides.
    Root (y ~ 0.010) is buried inside the column body so the spout reads
    as emerging from it.
    """
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.010, 0.062)
        .lineTo(0.010, 0.078)
        .spline(
            [(0.060, 0.077), (0.110, 0.070), (0.150, 0.054), (0.174, 0.030)],
            includeCurrent=True,
        )
        .lineTo(0.160, 0.024)
        .spline(
            [(0.140, 0.040), (0.105, 0.054), (0.060, 0.061), (0.010, 0.062)],
            includeCurrent=True,
        )
        .close()
        .extrude(SPOUT_WIDTH)
    )
    return profile.translate((-SPOUT_WIDTH / 2.0, 0.0, 0.0))


def _oval_finial() -> cq.Shape:
    """Small oval (elliptical-plan) finial button via nonuniform sphere scale."""
    unit = cq.Workplane("XY").sphere(1.0).val()
    mat = cq.Matrix(
        [
            [FINIAL_RX, 0.0, 0.0, 0.0],
            [0.0, FINIAL_RY, 0.0, 0.0],
            [0.0, 0.0, FINIAL_RZ, 0.0],
        ]
    )
    return unit.transformGeometry(mat)


def _add_lever_handle(part: Part, chrome: str) -> None:
    """Cylindrical lever handle rotating about local +Z.

    Local frame origin is the handle joint frame: hub bottom at z=0.
    Hub is a short vertical cylinder, lever bar extends horizontally along +X.
    """
    # Hub (vertical cylinder)
    part.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0)),
        material=chrome,
        name="hub",
    )
    # Dome cap on hub top
    part.visual(
        Sphere(radius=HUB_R),
        origin=Origin(xyz=(0.0, 0.0, HUB_H)),
        material=chrome,
        name="hub_dome",
    )
    # Lever bar (horizontal cylinder extending along +X from hub)
    part.visual(
        Cylinder(radius=LEVER_R, length=LEVER_LEN),
        origin=Origin(
            xyz=(LEVER_LEN / 2.0, 0.0, HUB_H / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="lever_bar",
    )
    # Ball end on lever tip
    part.visual(
        Sphere(radius=LEVER_TIP_R),
        origin=Origin(xyz=(LEVER_LEN, 0.0, HUB_H / 2.0)),
        material=chrome,
        name="lever_tip",
    )


def _add_valve_column(part: Part, chrome: str, seam_mat: str) -> None:
    """Tapered cylindrical pedestal with cap, slim stem, and base seam."""
    # Narrow dark seam ring at deck interface
    part.visual(
        Cylinder(radius=V_SEAM_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2.0)),
        material=seam_mat,
        name="base_seam",
    )
    # Tapered cylinder pedestal
    part.visual(
        mesh_from_cadquery(
            _tapered_cylinder(V_BASE_R, V_TOP_R, V_COL_H),
            f"{part.name}_pedestal",
        ),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H)),
        material=chrome,
        name="valve_pedestal",
    )
    # Cylindrical cap
    cap_z = SEAM_H + V_COL_H
    part.visual(
        Cylinder(radius=V_CAP_R, length=V_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, cap_z + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    # Slim bonnet stem
    stem_z0 = cap_z
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))
    seam_mat = model.material("seam_dark", rgba=(0.04, 0.04, 0.05, 1.0))

    # --- Dark deck plate (root) ---
    deck = model.part("deck")
    deck.visual(
        Box((0.42, 0.20, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),  # top face at z = 0
        material=deck_mat,
        name="deck_plate",
    )

    # --- Center spout column (swivels on continuous vertical joint) ---
    spout_body = model.part("spout_body")
    # Base seam
    spout_body.visual(
        Cylinder(radius=C_SEAM_R, length=SEAM_H),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H / 2.0)),
        material=seam_mat,
        name="base_seam",
    )
    # Tapered cylinder column
    spout_body.visual(
        mesh_from_cadquery(
            _tapered_cylinder(C_BASE_R, C_TOP_R, C_COL_H),
            "center_pedestal",
        ),
        origin=Origin(xyz=(0.0, 0.0, SEAM_H)),
        material=chrome.name,
        name="spout_pedestal",
    )
    # Cylindrical cap
    cap_z = SEAM_H + C_COL_H
    spout_body.visual(
        Cylinder(radius=CAP_R, length=CAP_H),
        origin=Origin(xyz=(0.0, 0.0, cap_z + CAP_H / 2.0)),
        material=chrome.name,
        name="spout_cap",
    )
    # Waterfall spout
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout(), "waterfall_spout"),
        material=chrome.name,
        name="spout",
    )
    # Continuous swivel joint: spout body rotates freely about vertical axis
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
    )

    # --- Oval finial diverter button on the cap ---
    finial = model.part("diverter_finial")
    finial.visual(
        Cylinder(radius=FINIAL_STEM_R, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),  # embeds 2 mm into the cap
        material=chrome.name,
        name="finial_stem",
    )
    finial.visual(
        mesh_from_cadquery(_oval_finial(), "finial_oval"),
        origin=Origin(xyz=(0.0, 0.0, FINIAL_CENTER_Z)),
        material=chrome.name,
        name="finial_oval",
    )
    model.articulation(
        "diverter_spin",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=finial,
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=-math.pi / 2.0, upper=math.pi / 2.0
        ),
    )

    # --- Valve columns and lever handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name, seam_mat)
        model.articulation(
            f"deck_to_{side}_valve",
            ArticulationType.FIXED,
            parent=deck,
            child=valve,
            origin=Origin(xyz=(sx * HANDLE_SPREAD_X, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        _add_lever_handle(handle, chrome.name)
        model.articulation(
            f"{side}_handle_spin",
            ArticulationType.REVOLUTE,
            parent=valve,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout_body = object_model.get_part("spout_body")
    finial = object_model.get_part("diverter_finial")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_spout = object_model.get_articulation("deck_to_spout_body")
    j_div = object_model.get_articulation("diverter_spin")

    # --- Intentional captured fits ---
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Lever handle hub intentionally captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Lever handle hub intentionally captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        finial,
        spout_body,
        elem_a="finial_stem",
        elem_b="spout_cap",
        reason="Finial stem is intentionally seated 2 mm into the spout cap.",
    )

    # --- All three chrome pieces seated on the dark deck ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.003,
            max_penetration=0.001,
            name=f"{piece.name} base seated on deck top",
        )
        ctx.expect_within(
            piece,
            deck,
            axes="x",
            margin=0.001,
            name=f"{piece.name} stands within the deck plate",
        )

    # --- Narrow seams present at all three deck bases ---
    for piece_name in ("spout_body", "left_valve", "right_valve"):
        piece = object_model.get_part(piece_name)
        seam_aabb = ctx.part_element_world_aabb(piece, elem="base_seam")
        ctx.check(
            f"{piece_name} has a visible base seam ring",
            seam_aabb is not None and (seam_aabb[1][2] - seam_aabb[0][2]) < 0.004,
            details=f"seam aabb={seam_aabb}",
        )

    # --- Three-piece spread of about 0.30 m ---
    ctx.expect_origin_distance(
        left_handle,
        right_handle,
        axes="x",
        min_dist=0.29,
        max_dist=0.31,
        name="handle spread is about 0.30 m",
    )

    # --- Spout body is a continuous joint (swivel) ---
    ctx.check(
        "spout swivel is a continuous joint",
        j_spout.articulation_type == ArticulationType.CONTINUOUS,
        details=f"spout joint type={j_spout.articulation_type}",
    )

    # --- Lever handles: check lever bar exists and is about 0.050 m long ---
    for handle in (left_handle, right_handle):
        lever_aabb = ctx.part_element_world_aabb(handle, elem="lever_bar")
        ctx.check(
            f"{handle.name} has a cylindrical lever bar ~0.050 m",
            lever_aabb is not None
            and 0.045 <= (lever_aabb[1][0] - lever_aabb[0][0]) <= 0.055,
            details=f"lever aabb={lever_aabb}",
        )
        ctx.expect_gap(
            handle,
            left_valve if handle == left_handle else right_valve,
            axis="z",
            max_gap=0.001,
            max_penetration=0.006,
            name=f"{handle.name} hub seats over the valve stem",
        )

    # --- Valve pedestals are cylindrical (circular XY footprint) ---
    for valve in (left_valve, right_valve):
        ped_aabb = ctx.part_element_world_aabb(valve, elem="valve_pedestal")
        ctx.check(
            f"{valve.name} pedestal has roughly circular cross-section",
            ped_aabb is not None
            and abs(
                (ped_aabb[1][0] - ped_aabb[0][0]) - (ped_aabb[1][1] - ped_aabb[0][1])
            )
            < 0.004,
            details=f"pedestal aabb={ped_aabb}",
        )

    # --- Waterfall spout: forward reach ~0.18 m ---
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches about 0.18 m forward",
        spout_aabb is not None and 0.16 <= spout_aabb[1][1] <= 0.20,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip arcs down but stays above the deck",
        spout_aabb is not None and 0.01 <= spout_aabb[0][2] <= 0.050,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Handle revolute joint limits ---
    for joint in (j_left, j_right):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is -180..+180 deg",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower + math.pi) < 0.01
            and abs(lim.upper - math.pi) < 0.01,
        )

    # --- Decisive pose: spout swivels about vertical axis ---
    spout_tip_rest = ctx.part_element_world_aabb(spout_body, elem="spout")
    rest_y = None
    if spout_tip_rest is not None:
        rest_y = spout_tip_rest[1][1]  # max Y of spout (forward reach)
    with ctx.pose({j_spout: math.pi / 2.0}):
        spout_tip_posed = ctx.part_element_world_aabb(spout_body, elem="spout")
    posed_min_x = None
    if spout_tip_posed is not None:
        # +90° about +Z rotates forward (+Y) to -X, so min X should be very negative
        posed_min_x = spout_tip_posed[0][0]
    ctx.check(
        "spout swivels 90 deg about vertical axis",
        rest_y is not None
        and posed_min_x is not None
        and posed_min_x < -0.10,
        details=f"rest max_y={rest_y}, posed min_x={posed_min_x}",
    )

    # --- Decisive pose: lever handles rotate independently ---
    lever_rest = ctx.part_element_world_aabb(left_handle, elem="lever_tip")
    rest_tip_x = None
    if lever_rest is not None:
        rest_tip_x = lever_rest[1][0]
    with ctx.pose({j_left: math.pi / 2.0}):
        lever_posed = ctx.part_element_world_aabb(left_handle, elem="lever_tip")
    posed_tip_y = None
    if lever_posed is not None:
        posed_tip_y = lever_posed[1][1]
    ctx.check(
        "left lever handle rotates 90 deg about vertical axis",
        rest_tip_x is not None
        and posed_tip_y is not None
        and abs(posed_tip_y) > 0.02,
        details=f"rest tip_x={rest_tip_x}, posed tip_y={posed_tip_y}",
    )

    return ctx.report()


object_model = build_object_model()
