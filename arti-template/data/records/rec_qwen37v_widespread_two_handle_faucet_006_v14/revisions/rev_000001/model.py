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
# Widespread two-handle faucet variant (cylindrical lever handles, tapered
# pedestals, hollow outlet, pivoting aerator, deck-base seams).
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered cylindrical pedestal (r=0.035 at
#     deck -> r=0.023 at z = 0.08), stepped cylindrical cap, flat-topped
#     waterfall spout reaching ~0.18 forward, hollow outlet tube visible at top
#   - valve columns at x = +/-0.15: tapered cylindrical pedestals (r=0.030,
#     0.07 tall) with slim stem carrying a cylindrical lever handle
#   - narrow dark seam rings at each of the three deck bases
#   - small aerator at spout tip, pivots downward on a horizontal hinge
# Articulation: each lever handle revolute about its vertical stem axis
# (-pi..pi); aerator revolute about horizontal axis (0..pi/4 downward).
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Center column
C_PED_R_BASE = 0.035
C_PED_R_TOP = 0.023
C_PED_H = 0.080
CAP_R1 = 0.028
CAP_H1 = 0.010
CAP_R2 = 0.023
CAP_H2 = 0.008
CAP_TOP_Z = C_PED_H + CAP_H1 + CAP_H2  # 0.098

# Hollow outlet tube (visible through cap center)
OUTER_TUBE_R = 0.010
INNER_TUBE_R = 0.007
OUTER_TUBE_H = 0.012  # protrudes above cap

# Valve columns
V_PED_R_BASE = 0.030
V_PED_R_TOP = 0.017
V_PED_H = 0.070
V_CAP_R = 0.020
V_CAP_H = 0.008
V_STEM_R = 0.006
V_STEM_TOP_Z = 0.094
HANDLE_JOINT_Z = 0.091  # handle captures stem top by ~3 mm

# Lever handle
LEVER_R = 0.006
LEVER_LEN = 0.060  # total length of lever
LEVER_CAP_R = 0.007

# Spout
SPOUT_WIDTH = 0.044

# Seam rings at deck bases
SEAM_WIDTH = 0.003
SEAM_THICKNESS = 0.002

# Aerator
AERATOR_R = 0.012
AERATOR_H = 0.010
AERATOR_HINGE_OFFSET_Y = 0.168  # near spout tip
AERATOR_HINGE_Z = 0.028  # at spout outlet level


def _tapered_cylinder(r_base: float, r_top: float, height: float) -> cq.Workplane:
    """Tapered cylindrical pedestal, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .circle(r_base)
        .workplane(offset=height)
        .circle(r_top)
        .loft(combine=True)
    )


def _hollow_outlet_tube() -> cq.Workplane:
    """Hollow cylindrical outlet tube protruding above the cap."""
    outer = (
        cq.Workplane("XY")
        .circle(OUTER_TUBE_R)
        .extrude(OUTER_TUBE_H)
    )
    inner = (
        cq.Workplane("XY")
        .circle(INNER_TUBE_R)
        .extrude(OUTER_TUBE_H + 0.001)
    )
    return outer.cut(inner)


def _waterfall_spout() -> cq.Workplane:
    """Wide flat-topped spout sweeping forward (+Y) into a waterfall arc.

    Side profile drawn in the YZ plane, extruded across X for slab sides.
    The root (y ~ 0.010) is buried inside the column so the spout reads as
    emerging from the body.
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


def _seam_ring(radius: float) -> cq.Workplane:
    """Thin dark seam ring sitting at the deck base of a column."""
    outer = (
        cq.Workplane("XY")
        .circle(radius + SEAM_WIDTH)
        .extrude(SEAM_THICKNESS)
    )
    inner = (
        cq.Workplane("XY")
        .circle(radius - 0.001)
        .extrude(SEAM_THICKNESS + 0.001)
    )
    return outer.cut(inner)


def _add_lever_handle(part: Part, chrome: str) -> None:
    """Cylindrical lever handle rotating about local +Z.

    Local frame origin is the handle joint frame: hub bottom at z=0.
    The lever extends along +X from a small cylindrical hub.
    """
    # Hub (captures the stem)
    part.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=chrome,
        name="hub",
    )
    # Dome top
    part.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=chrome,
        name="hub_dome",
    )
    # Lever arm extends along +X from the hub mid-height
    part.visual(
        Cylinder(radius=LEVER_R, length=LEVER_LEN),
        origin=Origin(xyz=(LEVER_LEN / 2.0, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_arm",
    )
    # Rounded end cap
    part.visual(
        Sphere(radius=LEVER_CAP_R),
        origin=Origin(xyz=(LEVER_LEN, 0.0, 0.016)),
        material=chrome,
        name="lever_tip",
    )


def _add_valve_column(part: Part, chrome: str) -> None:
    """Tapered cylindrical valve pedestal with cap and slim bonnet stem."""
    part.visual(
        mesh_from_cadquery(
            _tapered_cylinder(V_PED_R_BASE, V_PED_R_TOP, V_PED_H),
            f"{part.name}_pedestal",
        ),
        material=chrome,
        name="valve_pedestal",
    )
    part.visual(
        Cylinder(radius=V_CAP_R, length=V_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, V_PED_H + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    stem_z0 = V_PED_H + V_CAP_H / 2.0
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

    # --- Center spout column ---
    spout_body = model.part("spout_body")
    spout_body.visual(
        mesh_from_cadquery(
            _tapered_cylinder(C_PED_R_BASE, C_PED_R_TOP, C_PED_H),
            "center_pedestal",
        ),
        material=chrome.name,
        name="spout_pedestal",
    )
    spout_body.visual(
        Cylinder(radius=CAP_R1, length=CAP_H1),
        origin=Origin(xyz=(0.0, 0.0, C_PED_H + CAP_H1 / 2.0)),
        material=chrome.name,
        name="cap_step_lower",
    )
    spout_body.visual(
        Cylinder(radius=CAP_R2, length=CAP_H2),
        origin=Origin(xyz=(0.0, 0.0, C_PED_H + CAP_H1 + CAP_H2 / 2.0)),
        material=chrome.name,
        name="cap_step_upper",
    )
    # Waterfall spout
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout(), "waterfall_spout"),
        material=chrome.name,
        name="spout",
    )
    # Hollow central outlet tube protruding above cap
    spout_body.visual(
        mesh_from_cadquery(_hollow_outlet_tube(), "hollow_outlet"),
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z)),
        material=chrome.name,
        name="outlet_tube",
    )
    # Seam ring at center base
    spout_body.visual(
        mesh_from_cadquery(_seam_ring(C_PED_R_BASE), "center_seam"),
        origin=Origin(xyz=(0.0, 0.0, -SEAM_THICKNESS / 2.0)),
        material=seam_mat,
        name="center_seam",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Aerator at spout tip, pivots downward on horizontal hinge ---
    aerator = model.part("aerator")
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_H),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_H / 2.0)),
        material=chrome.name,
        name="aerator_body",
    )
    # Small mesh/screen face at bottom
    aerator.visual(
        Cylinder(radius=AERATOR_R - 0.002, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -AERATOR_H - 0.001)),
        material=seam_mat,
        name="aerator_screen",
    )
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=spout_body,
        child=aerator,
        origin=Origin(xyz=(0.0, AERATOR_HINGE_OFFSET_Y, AERATOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),  # rotates about X so +q tips downward (toward -Z)
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=math.pi / 4.0
        ),
    )

    # --- Valve columns and lever handles (left = -X, right = +X) ---
    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        _add_valve_column(valve, chrome.name)
        # Seam ring at valve base
        valve.visual(
            mesh_from_cadquery(_seam_ring(V_PED_R_BASE), f"{side}_seam"),
            origin=Origin(xyz=(0.0, 0.0, -SEAM_THICKNESS / 2.0)),
            material=seam_mat,
            name=f"{side}_seam",
        )
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
    aerator = object_model.get_part("aerator")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    j_left = object_model.get_articulation("left_handle_spin")
    j_right = object_model.get_articulation("right_handle_spin")
    j_aerator = object_model.get_articulation("aerator_hinge")

    # Intentional captured fits: handle hubs over valve stems.
    ctx.allow_overlap(
        left_handle,
        left_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Lever-handle hub intentionally captures the valve bonnet stem.",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        elem_a="hub",
        elem_b="valve_stem",
        reason="Lever-handle hub intentionally captures the valve bonnet stem.",
    )

    # --- All three chrome pieces seated on the dark deck ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.002,
            max_penetration=0.002,
            name=f"{piece.name} base seated on deck top",
        )
        ctx.expect_within(
            piece,
            deck,
            axes="x",
            margin=0.001,
            name=f"{piece.name} stands within the deck plate",
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
    ctx.expect_origin_gap(
        right_valve,
        spout_body,
        axis="x",
        min_gap=0.10,
        max_gap=0.16,
        name="right valve flanks the spout column",
    )
    ctx.expect_origin_gap(
        spout_body,
        left_valve,
        axis="x",
        min_gap=0.10,
        max_gap=0.16,
        name="left valve flanks the spout column",
    )

    # --- Hollow outlet tube exists above cap ---
    outlet_aabb = ctx.part_element_world_aabb(spout_body, elem="outlet_tube")
    ctx.check(
        "hollow outlet tube protrudes above the cap",
        outlet_aabb is not None and outlet_aabb[1][2] > CAP_TOP_Z + 0.005,
        details=f"outlet_tube aabb={outlet_aabb}",
    )

    # --- Seam rings present at all three deck bases ---
    center_seam_aabb = ctx.part_element_world_aabb(spout_body, elem="center_seam")
    ctx.check(
        "center base seam ring exists at deck level",
        center_seam_aabb is not None and abs(center_seam_aabb[0][2]) < 0.004,
        details=f"center_seam aabb={center_seam_aabb}",
    )
    for side in ("left", "right"):
        valve = object_model.get_part(f"{side}_valve")
        seam_aabb = ctx.part_element_world_aabb(valve, elem=f"{side}_seam")
        ctx.check(
            f"{side} valve base seam ring exists at deck level",
            seam_aabb is not None and abs(seam_aabb[0][2]) < 0.004,
            details=f"{side}_seam aabb={seam_aabb}",
        )

    # --- Waterfall spout reaches forward ---
    spout_aabb = ctx.part_element_world_aabb(spout_body, elem="spout")
    ctx.check(
        "spout reaches about 0.18 m forward",
        spout_aabb is not None and 0.16 <= spout_aabb[1][1] <= 0.20,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "spout tip arcs down well below the cap but stays above the deck",
        spout_aabb is not None and 0.01 <= spout_aabb[0][2] <= 0.045,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Cylindrical pedestal proportions ---
    ped_aabb = ctx.part_element_world_aabb(spout_body, elem="spout_pedestal")
    ctx.check(
        "center pedestal base diameter is about 0.07 m",
        ped_aabb is not None
        and 0.064 <= (ped_aabb[1][0] - ped_aabb[0][0]) <= 0.076,
        details=f"pedestal aabb={ped_aabb}",
    )

    # --- Lever handles: elongated shape, seated over stems ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        h_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            f"{handle.name} lever extends at least 0.05 m from hub",
            h_aabb is not None and (h_aabb[1][0] - h_aabb[0][0]) >= 0.050,
            details=f"{handle.name} aabb={h_aabb}",
        )
        ctx.expect_gap(
            handle,
            valve,
            axis="z",
            max_gap=0.001,
            max_penetration=0.004,
            name=f"{handle.name} hub seats over the valve stem",
        )

    # --- Aerator mounted near spout tip ---
    ctx.expect_overlap(
        aerator,
        spout_body,
        axes="y",
        min_overlap=0.0,
        name="aerator longitudinally near spout tip",
    )

    # --- Joint limits ---
    for joint, lo, hi in ((j_left, -math.pi, math.pi), (j_right, -math.pi, math.pi)):
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name} range is -180..+180 deg",
            lim is not None
            and lim.lower is not None
            and lim.upper is not None
            and abs(lim.lower - lo) < 0.01
            and abs(lim.upper - hi) < 0.01,
        )
    aer_lim = j_aerator.motion_limits
    ctx.check(
        "aerator hinge range is 0..45 deg downward",
        aer_lim is not None
        and aer_lim.lower is not None
        and aer_lim.upper is not None
        and abs(aer_lim.lower) < 0.01
        and abs(aer_lim.upper - math.pi / 4.0) < 0.01,
    )

    # --- Decisive pose: lever handle rotates visibly ---
    lever_rest = ctx.part_element_world_aabb(left_handle, elem="lever_tip")
    with ctx.pose({j_left: math.pi / 3.0}):
        lever_posed = ctx.part_element_world_aabb(left_handle, elem="lever_tip")
    ctx.check(
        "left lever handle rotates about its vertical stem axis",
        lever_rest is not None
        and lever_posed is not None
        and math.hypot(
            lever_posed[0][0] - lever_rest[0][0],
            lever_posed[0][1] - lever_rest[0][1],
        )
        > 0.01,
        details=f"rest={lever_rest}, posed={lever_posed}",
    )

    # --- Decisive pose: aerator pivots downward ---
    aerator_rest_z = ctx.part_element_world_aabb(aerator, elem="aerator_screen")
    with ctx.pose({j_aerator: math.pi / 5.0}):
        aerator_posed_z = ctx.part_element_world_aabb(aerator, elem="aerator_screen")
    ctx.check(
        "aerator pivots downward when hinge opens",
        aerator_rest_z is not None
        and aerator_posed_z is not None
        and aerator_posed_z[0][2] < aerator_rest_z[0][2] - 0.002,
        details=f"rest={aerator_rest_z}, posed={aerator_posed_z}",
    )

    return ctx.report()


object_model = build_object_model()
