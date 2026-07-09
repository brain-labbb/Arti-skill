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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Widespread two-handle deck-mounted faucet, polished chrome on dark deck.
# Variant 04: cylindrical lever handles with tapered pedestals, narrow seams
# at all three deck bases, decorative ring ridges on handle pedestals.
#
# Layout (meters, Z up, spout sweeps forward along +Y):
#   - dark deck plate (root) with three chrome pieces mounted on top (z = 0)
#   - center spout column at x = 0: tapered cylindrical base, stepped cap,
#     flat-topped waterfall spout reaching ~0.18 forward, oval finial diverter
#   - valve columns at x = +/-0.15: tapered cylindrical pedestals with
#     decorative ring ridges, slim stem carrying a cylindrical lever handle
#   - narrow seam rings at all three deck contact points
#
# Articulation: each lever handle revolute about its vertical stem axis
# (-pi..pi); the oval finial is a revolute diverter (-pi/2..pi/2).
# ---------------------------------------------------------------------------

HANDLE_SPREAD_X = 0.150  # valve column centers at +/-0.150 -> 0.30 m spread

# Center column (cylindrical tapered base)
C_BASE_R = 0.036  # radius at deck
C_TOP_R = 0.024   # radius at top
C_PYR_H = 0.080
CAP1_R = 0.029
CAP1_H = 0.010
CAP2_R = 0.024
CAP2_H = 0.008
CAP_TOP_Z = C_PYR_H + CAP1_H + CAP2_H  # 0.098

# Valve columns (cylindrical tapered pedestals)
V_BASE_R = 0.030   # radius at deck
V_TOP_R = 0.018    # radius at top
V_PYR_H = 0.070
V_CAP_R = 0.021
V_CAP_H = 0.008
V_STEM_R = 0.0065
V_STEM_TOP_Z = 0.096
HANDLE_JOINT_Z = 0.093  # hub captures the stem top by 3 mm

# Decorative ring ridges on valve pedestals
RING_COUNT = 3
RING_TUBE_R = 0.0018  # tube radius of decorative torus

# Seam rings at deck bases
SEAM_TUBE_R = 0.0012  # thin torus tube
SEAM_EXTRA = 0.003    # seam ring extends beyond base radius

# Lever handle
HUB_R = 0.010
HUB_H = 0.026
LEVER_R = 0.005       # lever arm radius (cylinder)
LEVER_LEN = 0.048     # lever arm length
LEVER_TIP_R = 0.007   # ball end at lever tip

# Spout
SPOUT_WIDTH = 0.050

# Finial diverter
FINIAL_RX = 0.018
FINIAL_RY = 0.012
FINIAL_RZ = 0.008
FINIAL_STEM_R = 0.0045
FINIAL_CENTER_Z = 0.014


def _tapered_cylinder(base_r: float, top_r: float, height: float) -> cq.Workplane:
    """Tapered cylindrical column, base on z=0, narrowing upward."""
    return (
        cq.Workplane("XY")
        .circle(base_r)
        .workplane(offset=height)
        .circle(top_r)
        .loft(combine=True)
    )


def _waterfall_spout() -> cq.Workplane:
    """Wide flat-topped spout sweeping forward (+Y) into a waterfall arc.

    Side profile drawn in the YZ plane, extruded across X for flat slab sides.
    The root (y ~ 0.010) is buried inside the column body so the spout reads
    as emerging from the body.
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
    The lever arm extends along +X from the hub mid-height.
    """
    # Hub cylinder
    part.visual(
        Cylinder(radius=HUB_R, length=HUB_H),
        origin=Origin(xyz=(0.0, 0.0, HUB_H / 2.0)),
        material=chrome,
        name="hub",
    )
    # Dome cap on top of hub
    part.visual(
        Sphere(radius=HUB_R),
        origin=Origin(xyz=(0.0, 0.0, HUB_H)),
        material=chrome,
        name="hub_dome",
    )
    # Lever arm extending along +X, rotated from Z onto X
    lever_center_x = HUB_R + LEVER_LEN / 2.0
    part.visual(
        Cylinder(radius=LEVER_R, length=LEVER_LEN),
        origin=Origin(xyz=(lever_center_x, 0.0, HUB_H * 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lever_arm",
    )
    # Ball end at lever tip
    tip_x = HUB_R + LEVER_LEN
    part.visual(
        Sphere(radius=LEVER_TIP_R),
        origin=Origin(xyz=(tip_x, 0.0, HUB_H * 0.55)),
        material=chrome,
        name="lever_tip",
    )


def _add_valve_pedestal(part: Part, chrome: str, side: str) -> None:
    """Tapered cylindrical valve pedestal with decorative ring ridges and cap."""
    part.visual(
        mesh_from_cadquery(
            _tapered_cylinder(V_BASE_R, V_TOP_R, V_PYR_H),
            f"{side}_pedestal",
        ),
        material=chrome,
        name="valve_pedestal",
    )
    # Round cap on top
    part.visual(
        Cylinder(radius=V_CAP_R, length=V_CAP_H),
        origin=Origin(xyz=(0.0, 0.0, V_PYR_H + V_CAP_H / 2.0)),
        material=chrome,
        name="valve_cap",
    )
    # Slim stem rising from cap
    stem_z0 = V_PYR_H + V_CAP_H / 2.0
    part.visual(
        Cylinder(radius=V_STEM_R, length=V_STEM_TOP_Z - stem_z0),
        origin=Origin(xyz=(0.0, 0.0, (stem_z0 + V_STEM_TOP_Z) / 2.0)),
        material=chrome,
        name="valve_stem",
    )
    # Decorative ring ridges: evenly spaced torus rings on the pedestal
    for i in range(RING_COUNT):
        frac = (i + 1) / (RING_COUNT + 1)
        z_ring = V_PYR_H * frac
        # Interpolate radius at this height
        r_ring = V_BASE_R + (V_TOP_R - V_BASE_R) * frac
        torus = TorusGeometry(r_ring, RING_TUBE_R, radial_segments=12, tubular_segments=32)
        part.visual(
            mesh_from_geometry(torus, f"{side}_ring_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z_ring)),
            material=chrome,
            name=f"deco_ring_{i}",
        )


def _add_seam_ring(part: Part, chrome: str, base_radius: float, name_prefix: str) -> None:
    """Thin seam ring at the deck-contact base of a column.

    The torus is placed so its tube partially embeds into the pedestal base
    wall, ensuring mesh connectivity with the parent body.
    """
    # Center the torus tube on the pedestal outer surface, slightly above z=0
    seam_r = base_radius - SEAM_TUBE_R * 0.4
    torus = TorusGeometry(seam_r, SEAM_TUBE_R, radial_segments=10, tubular_segments=32)
    part.visual(
        mesh_from_geometry(torus, f"{name_prefix}_seam"),
        origin=Origin(xyz=(0.0, 0.0, SEAM_TUBE_R * 0.5)),
        material=chrome,
        name="base_seam",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    chrome = model.material("chrome", rgba=(0.88, 0.89, 0.92, 1.0))
    deck_mat = model.material("deck_charcoal", rgba=(0.09, 0.09, 0.10, 1.0))

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
            _tapered_cylinder(C_BASE_R, C_TOP_R, C_PYR_H), "center_pedestal"
        ),
        material=chrome.name,
        name="spout_pedestal",
    )
    # Seam ring at center base
    _add_seam_ring(spout_body, chrome.name, C_BASE_R, "center")
    # Stepped round caps
    spout_body.visual(
        Cylinder(radius=CAP1_R, length=CAP1_H),
        origin=Origin(xyz=(0.0, 0.0, C_PYR_H + CAP1_H / 2.0)),
        material=chrome.name,
        name="cap_step_lower",
    )
    spout_body.visual(
        Cylinder(radius=CAP2_R, length=CAP2_H),
        origin=Origin(xyz=(0.0, 0.0, C_PYR_H + CAP1_H + CAP2_H / 2.0)),
        material=chrome.name,
        name="cap_step_upper",
    )
    spout_body.visual(
        mesh_from_cadquery(_waterfall_spout(), "waterfall_spout"),
        material=chrome.name,
        name="spout",
    )
    model.articulation(
        "deck_to_spout_body",
        ArticulationType.FIXED,
        parent=deck,
        child=spout_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
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
        _add_valve_pedestal(valve, chrome.name, side)
        # Seam ring at valve base
        _add_seam_ring(valve, chrome.name, V_BASE_R, side)
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
    j_div = object_model.get_articulation("diverter_spin")

    # Intentional captured fits: handle hubs over valve stems, finial stem
    # seated into the stepped cap.
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
    ctx.allow_overlap(
        finial,
        spout_body,
        elem_a="finial_stem",
        elem_b="cap_step_upper",
        reason="Finial stem is intentionally seated 2 mm into the stepped cap.",
    )

    # --- All three chrome pieces seated on the dark deck, not floating ---
    for piece in (spout_body, left_valve, right_valve):
        ctx.expect_gap(
            piece,
            deck,
            axis="z",
            max_gap=0.001,
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

    # --- Three-piece spread of about 0.30 m, spout centered ---
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
        min_gap=0.14,
        max_gap=0.16,
        name="right valve flanks the spout column",
    )
    ctx.expect_origin_gap(
        spout_body,
        left_valve,
        axis="x",
        min_gap=0.14,
        max_gap=0.16,
        name="left valve flanks the spout column",
    )

    # --- Waterfall spout: forward reach ~0.18 m ---
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

    # --- Cylindrical tapered pedestals (not square pyramids) ---
    for valve, side_name in ((left_valve, "left"), (right_valve, "right")):
        ped_aabb = ctx.part_element_world_aabb(valve, elem="valve_pedestal")
        ctx.check(
            f"{side_name} pedestal is cylindrical tapered (roughly round footprint)",
            ped_aabb is not None,
            details=f"pedestal aabb={ped_aabb}",
        )
        if ped_aabb is not None:
            dx = ped_aabb[1][0] - ped_aabb[0][0]
            dy = ped_aabb[1][1] - ped_aabb[0][1]
            ctx.check(
                f"{side_name} pedestal is roughly circular (dx~dy)",
                abs(dx - dy) < 0.005,
                details=f"dx={dx:.4f}, dy={dy:.4f}",
            )
            ctx.check(
                f"{side_name} pedestal base diameter ~0.06 m",
                0.055 <= dx <= 0.065,
                details=f"dx={dx:.4f}",
            )

    # --- Decorative ring ridges exist on valve pedestals ---
    for valve, side_name in ((left_valve, "left"), (right_valve, "right")):
        for i in range(RING_COUNT):
            ring_aabb = ctx.part_element_world_aabb(valve, elem=f"deco_ring_{i}")
            ctx.check(
                f"{side_name} pedestal has decorative ring {i}",
                ring_aabb is not None,
                details=f"ring_{i} aabb={ring_aabb}",
            )

    # --- Narrow seams at all three deck bases ---
    for piece, piece_name in ((spout_body, "center"), (left_valve, "left"), (right_valve, "right")):
        seam_aabb = ctx.part_element_world_aabb(piece, elem="base_seam")
        ctx.check(
            f"{piece_name} piece has a narrow seam ring at deck base",
            seam_aabb is not None,
            details=f"seam aabb={seam_aabb}",
        )
        if seam_aabb is not None:
            # Seam should be thin in Z
            seam_thickness = seam_aabb[1][2] - seam_aabb[0][2]
            ctx.check(
                f"{piece_name} seam ring is thin (decorative)",
                seam_thickness < 0.005,
                details=f"thickness={seam_thickness:.4f}",
            )

    # --- Lever handles: arm extends outward, not a cross shape ---
    for handle, side_name in ((left_handle, "left"), (right_handle, "right")):
        lever_aabb = ctx.part_element_world_aabb(handle, elem="lever_arm")
        ctx.check(
            f"{side_name} handle has a cylindrical lever arm",
            lever_aabb is not None,
            details=f"lever aabb={lever_aabb}",
        )
        if lever_aabb is not None:
            # Lever arm should extend significantly more in one direction (X) than Y
            lx = lever_aabb[1][0] - lever_aabb[0][0]
            ly = lever_aabb[1][1] - lever_aabb[0][1]
            ctx.check(
                f"{side_name} lever arm is elongated (not a cross)",
                lx > ly + 0.02,
                details=f"lx={lx:.4f}, ly={ly:.4f}",
            )

    # --- Lever handles seated over stems ---
    for handle, valve in ((left_handle, left_valve), (right_handle, right_valve)):
        ctx.expect_gap(
            handle,
            valve,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.004,
            name=f"{handle.name} hub seats over the valve stem",
        )

    # --- Finial button seated on the cap top ---
    ctx.expect_gap(
        finial,
        spout_body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.003,
        name="finial stem seats into the cap top",
    )
    ctx.expect_within(
        finial,
        spout_body,
        axes="xy",
        inner_elem="finial_oval",
        outer_elem="cap_step_upper",
        margin=0.001,
        name="oval finial centered on the cap",
    )

    # --- Joint limits match the prompt ---
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
    div_lim = j_div.motion_limits
    ctx.check(
        "diverter range is -90..+90 deg",
        div_lim is not None
        and div_lim.lower is not None
        and div_lim.upper is not None
        and abs(div_lim.lower + math.pi / 2.0) < 0.01
        and abs(div_lim.upper - math.pi / 2.0) < 0.01,
    )

    # --- Decisive pose checks: lever tip moves when handle rotates ---
    def _lever_tip(handle: Part) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(handle, elem="lever_tip")
        if aabb is None:
            return None
        return (
            (aabb[0][0] + aabb[1][0]) / 2.0,
            (aabb[0][1] + aabb[1][1]) / 2.0,
        )

    rest_left = _lever_tip(left_handle)
    with ctx.pose({j_left: math.pi / 4.0}):
        posed_left = _lever_tip(left_handle)
    ctx.check(
        "left lever handle rotates about its vertical stem axis",
        rest_left is not None
        and posed_left is not None
        and math.hypot(posed_left[0] - rest_left[0], posed_left[1] - rest_left[1])
        > 0.02,
        details=f"rest={rest_left}, posed={posed_left}",
    )

    rest_right = _lever_tip(right_handle)
    with ctx.pose({j_right: -math.pi / 4.0}):
        posed_right = _lever_tip(right_handle)
    ctx.check(
        "right lever handle rotates independently about its stem axis",
        rest_right is not None
        and posed_right is not None
        and math.hypot(posed_right[0] - rest_right[0], posed_right[1] - rest_right[1])
        > 0.02,
        details=f"rest={rest_right}, posed={posed_right}",
    )

    # Oval finial: long axis along X at rest; rotated 90 deg it swaps to Y.
    oval_rest = ctx.part_element_world_aabb(finial, elem="finial_oval")
    ctx.check(
        "finial oval long axis along X at rest",
        oval_rest is not None
        and (oval_rest[1][0] - oval_rest[0][0]) > (oval_rest[1][1] - oval_rest[0][1]) + 0.008,
        details=f"oval rest aabb={oval_rest}",
    )
    with ctx.pose({j_div: math.pi / 2.0}):
        oval_posed = ctx.part_element_world_aabb(finial, elem="finial_oval")
    ctx.check(
        "diverter finial rotates 90 deg about the vertical axis",
        oval_posed is not None
        and (oval_posed[1][1] - oval_posed[0][1]) > (oval_posed[1][0] - oval_posed[0][0]) + 0.008,
        details=f"oval posed aabb={oval_posed}",
    )

    return ctx.report()


object_model = build_object_model()
