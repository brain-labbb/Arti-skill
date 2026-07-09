from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Roof pitch (radians). Ridge is at u=0 and high; the slope falls toward the eave at +u.
PITCH = 0.42
_COS_P = math.cos(PITCH)
_SIN_P = math.sin(PITCH)
RIDGE_HEIGHT = 2.18

HALF_W = 0.72                    # half width across the slope (world Y)
EAVE_U = 2.60                    # down-slope length to the eave

# --- Ridge vent flap geometry (variant: long flap along the ridge) ---
FLAP_HALF = 0.62                 # flap half-width across the ridge (Y)
FLAP_U0 = 0.06                   # flap start along the slope (just past ridge)
FLAP_U1 = 1.16                   # flap end along the slope
FLAP_LEN = FLAP_U1 - FLAP_U0    # 1.10 m down-slope


def _plane_xyz(u: float, v: float, w: float) -> tuple[float, float, float]:
    """Map roof-plane coords (u=down-slope, v=across=world Y, w=out-of-plane) to world xyz."""
    x = u * _COS_P + w * _SIN_P
    y = v
    z = RIDGE_HEIGHT - u * _SIN_P + w * _COS_P
    return (x, y, z)


def _plane_box(part, name, size, u, v, w, material):
    """Add a box lying in the pitched roof plane. size = (along-slope, across, out-of-plane)."""
    part.visual(
        Box(size),
        origin=Origin(xyz=_plane_xyz(u, v, w), rpy=(0.0, PITCH, 0.0)),
        material=material,
        name=name,
    )


def _plane_cyl_y(part, name, radius, length, u, v, w, material):
    """Add a Y-axis cylinder (runs across-slope, along the ridge direction) at a plane point."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_plane_xyz(u, v, w), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _vert_twine(part, name, top_xyz, length, radius, material):
    """A thin cord hanging straight down in world Z from a roof point."""
    cx, cy, cz = top_xyz
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(cx, cy, cz - length / 2.0)),
        material=material,
        name=name,
    )


def _add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_cyl_y(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_greenhouse_vent_roof",
        meta={
            "class": "Agricultural/Greenhouse vent roof",
            "description": (
                "A pitched greenhouse roof bay: glazed pane grid in weathered aluminum and "
                "aged-steel glazing bars, a long ridge vent flap hinged along the ridge rail "
                "lifting upward for ventilation, a folding scissor stay arm, a latch handle, "
                "hanging tomato twine and a tensioned support wire."
            ),
        },
    )

    aluminum = model.material("weathered_aluminum", rgba=(0.74, 0.75, 0.70, 1.0))
    aged_steel = model.material("aged_glazing_bar", rgba=(0.30, 0.26, 0.20, 1.0))
    galvanized = model.material("galvanized_hardware", rgba=(0.55, 0.57, 0.52, 1.0))
    glass = model.material("clear_sky_glass", rgba=(0.60, 0.80, 0.95, 0.26))
    rubber = model.material("black_epdm_seal", rgba=(0.02, 0.02, 0.02, 1.0))
    black_steel = model.material("black_painted_steel", rgba=(0.04, 0.04, 0.045, 1.0))
    bolt_dark = model.material("dark_fasteners", rgba=(0.03, 0.03, 0.03, 1.0))
    jute = model.material("jute_twine", rgba=(0.42, 0.34, 0.22, 1.0))
    wire = model.material("tension_wire", rgba=(0.62, 0.60, 0.55, 1.0))

    # ----------------------------------------------------------------- roof_frame
    roof = model.part("roof_frame")

    # Perimeter and structural rails of the pitched roof bay.
    _plane_box(roof, "ridge_rail", (0.085, 2 * HALF_W + 0.06, 0.060), 0.0, 0.0, 0.028, aluminum)
    _plane_box(roof, "eave_rail", (0.060, 2 * HALF_W + 0.06, 0.050), EAVE_U, 0.0, 0.022, aluminum)
    _plane_box(roof, "rake_rail_0", (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, -HALF_W, 0.022, aluminum)
    _plane_box(roof, "rake_rail_1", (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, HALF_W, 0.022, aged_steel)

    # Transoms (across-slope glazing bars) and mullions (down-slope bars) -> pane grid.
    _plane_box(roof, "transom_mid", (0.050, 2 * HALF_W, 0.044), 1.42, 0.0, 0.024, aged_steel)
    _plane_box(roof, "transom_low", (0.050, 2 * HALF_W, 0.044), 2.02, 0.0, 0.024, aluminum)
    for i, v in enumerate((-0.24, 0.24)):
        _plane_box(roof, f"mullion_lower_{i}", (1.22, 0.042, 0.040), 2.01, v, 0.026, aluminum)
    # Side mullions flanking the ridge flap opening.
    for i, v in enumerate((-0.66, 0.66)):
        _plane_box(roof, f"mullion_side_{i}", (1.40, 0.042, 0.040), 0.71, v, 0.026, aged_steel)

    # Curb framing the operable ridge flap (sill at the bottom edge of the flap opening).
    _plane_box(roof, "flap_curb_sill", (0.065, 2 * FLAP_HALF + 0.12, 0.048), FLAP_U1, 0.0, 0.024, aluminum)
    for i, v in enumerate((-FLAP_HALF - 0.04, FLAP_HALF + 0.04)):
        _plane_box(roof, f"flap_curb_jamb_{i}", (FLAP_U1 - FLAP_U0, 0.045, 0.046), (FLAP_U0 + FLAP_U1) / 2.0, v, 0.024, aluminum)

    # EPDM weather seals the closed flap beds onto (on the ridge rail and the curb sill).
    _plane_box(roof, "ridge_weather_seal", (0.024, 2 * FLAP_HALF, 0.014), 0.045, 0.0, 0.044, rubber)
    _plane_box(roof, "sill_weather_seal", (0.024, 2 * FLAP_HALF, 0.014), FLAP_U1 - 0.02, 0.0, 0.046, rubber)

    # Fixed glazing panes in the lower field (below the flap). Thin clear sheets.
    for ci, v in enumerate((-0.46, 0.0, 0.46)):
        for ri, u in enumerate((1.72, 2.31)):
            _plane_box(roof, f"glass_lower_{ci}_{ri}", (0.55, 0.44, 0.006), u, v, 0.045, glass)

    # Ridge hinge line: fixed leaves + alternating knuckles the flap interleaves with.
    for i, v in enumerate((-0.42, -0.14, 0.14, 0.42)):
        _plane_box(roof, f"fixed_hinge_leaf_{i}", (0.060, 0.130, 0.006), -0.05, v, 0.046, galvanized)
        _plane_cyl_y(roof, f"fixed_hinge_knuckle_{i}", 0.012, 0.130, 0.020, v, 0.064, galvanized)
    _plane_cyl_y(roof, "hinge_pin_left", 0.007, 0.040, 0.020, -0.56, 0.064, bolt_dark)
    _plane_cyl_y(roof, "hinge_pin_right", 0.007, 0.040, 0.020, 0.56, 0.064, bolt_dark)

    # Glazing-clip screws dotted along the lower rails.
    sx = 0
    for u, v in ((1.42, -0.40), (1.42, 0.40), (2.02, -0.40), (2.02, 0.40), (2.01, 0.0)):
        roof.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=_plane_xyz(u, v, 0.046), rpy=(0.0, PITCH, 0.0)),
            material=bolt_dark,
            name=f"roof_screw_{sx}",
        )
        sx += 1

    # Tensioned support wire spanning the bay in the roof plane.
    _plane_box(roof, "support_wire", (0.012, 2 * HALF_W + 0.10, 0.012), 1.95, 0.0, 0.0, wire)
    # Hanging tomato twine dropping through the wire.
    for i, v in enumerate((-0.38, -0.05, 0.22, 0.50)):
        top = _plane_xyz(1.95, v, 0.012)
        length = 0.34 + 0.05 * i
        _vert_twine(roof, f"twine_{i}", top, length, 0.004, jute)
        _add_box(roof, f"twine_knot_{i}", (0.020, 0.014, 0.024), (top[0], top[1], top[2] - length), jute)

    # --------------------------------------------------------------- ridge_vent_flap
    # Long ridge flap: hinged along the ridge_rail top edge, lifts upward for ventilation.
    # Authored flat in a hinge-line local frame: +X runs down-slope from the ridge hinge.
    flap = model.part("ridge_vent_flap")
    midx = FLAP_LEN / 2.0

    _add_box(flap, "flap_glass", (FLAP_LEN - 0.10, 2 * FLAP_HALF - 0.08, 0.007), (midx, 0.0, -0.002), glass)
    _add_box(flap, "flap_top_rail", (0.060, 2 * FLAP_HALF, 0.038), (0.030, 0.0, 0.0), aluminum)
    _add_box(flap, "flap_bottom_rail", (0.080, 2 * FLAP_HALF, 0.038), (FLAP_LEN - 0.030, 0.0, 0.0), aluminum)
    _add_box(flap, "flap_stile_0", (FLAP_LEN, 0.052, 0.038), (midx, -FLAP_HALF + 0.026, 0.0), aluminum)
    _add_box(flap, "flap_stile_1", (FLAP_LEN, 0.052, 0.038), (midx, FLAP_HALF - 0.026, 0.0), aged_steel)
    _add_box(flap, "flap_glazing_bar", (0.038, 2 * FLAP_HALF - 0.08, 0.028), (midx, 0.0, 0.010), aluminum)
    _add_box(flap, "flap_drip_lip", (0.030, 2 * FLAP_HALF - 0.06, 0.022), (FLAP_LEN - 0.060, 0.0, -0.024), aluminum)

    # EPDM gaskets around the flap perimeter.
    _add_box(flap, "flap_gasket_0", (FLAP_LEN - 0.10, 0.016, 0.012), (midx, -FLAP_HALF + 0.012, -0.018), rubber)
    _add_box(flap, "flap_gasket_1", (FLAP_LEN - 0.10, 0.016, 0.012), (midx, FLAP_HALF - 0.012, -0.018), rubber)
    _add_box(flap, "flap_gasket_bottom", (0.018, 2 * FLAP_HALF - 0.06, 0.012), (FLAP_LEN - 0.020, 0.0, -0.010), rubber)

    # Moving hinge leaves + knuckles interleaving with fixed ridge knuckles.
    for i, v in enumerate((-0.28, 0.0, 0.28)):
        _add_box(flap, f"flap_hinge_leaf_{i}", (0.060, 0.125, 0.006), (0.020, v, 0.012), galvanized)
        _add_cyl_y(flap, f"flap_hinge_knuckle_{i}", 0.011, 0.125, (0.004, v, 0.006), galvanized)

    # Stay-arm mount lug projecting below the flap bottom rail.
    _add_box(flap, "stay_mount_tab", (0.060, 0.080, 0.110), (FLAP_LEN - 0.01, -0.30, -0.045), galvanized)
    # Corner gusset plates at the four stile/rail junctions (seated on the stile face).
    stile_y = FLAP_HALF - 0.026  # stile Y center
    for i, (x, sy) in enumerate(((0.04, -stile_y), (0.04, stile_y), (FLAP_LEN - 0.04, -stile_y), (FLAP_LEN - 0.04, stile_y))):
        _add_box(flap, f"flap_corner_plate_{i}", (0.075, 0.052, 0.006), (x, sy, 0.020), galvanized)
    # Fastening screws through the stiles into the rails (on the stile center Y).
    for i, x in enumerate((0.04, 0.55, FLAP_LEN - 0.04)):
        for j, sy in enumerate((-stile_y, stile_y)):
            flap.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(xyz=(x, sy, 0.020)),
                material=bolt_dark,
                name=f"flap_screw_{i}_{j}",
            )

    # --------------------------------------------------------------- latch_handle
    latch = model.part("latch_handle")
    _add_cyl_y(latch, "latch_pivot_pin", 0.015, 0.075, (0.0, 0.0, 0.0), bolt_dark)
    _add_box(latch, "latch_back_plate", (0.065, 0.085, 0.010), (0.0, 0.0, -0.006), galvanized)
    _add_box(latch, "latch_hook_tongue", (0.115, 0.024, 0.012), (0.090, 0.0, -0.078), black_steel)
    _add_box(latch, "latch_hook_drop", (0.018, 0.024, 0.072), (0.034, 0.0, -0.044), black_steel)
    _add_box(latch, "pull_handle_stem", (0.020, 0.028, 0.140), (-0.012, 0.0, -0.080), black_steel)
    _add_box(latch, "rubber_grip", (0.038, 0.110, 0.024), (-0.012, 0.0, -0.150), rubber)

    # --------------------------------------------------------------- stay_arm
    stay = model.part("stay_arm")
    _add_cyl_y(stay, "stay_top_pin", 0.010, 0.040, (0.0, 0.0, 0.0), bolt_dark)
    _add_box(stay, "stay_pivot_plate", (0.050, 0.050, 0.010), (0.0, 0.0, -0.006), galvanized)
    _add_box(stay, "stay_upper_arm", (0.020, 0.020, 0.240), (0.0, 0.0, -0.125), black_steel)
    _add_cyl_y(stay, "stay_knuckle", 0.014, 0.026, (0.0, 0.0, -0.245), bolt_dark)
    _add_box(stay, "stay_elbow_jog", (0.140, 0.020, 0.020), (0.060, 0.0, -0.245), black_steel)
    _add_box(stay, "stay_lower_arm", (0.020, 0.020, 0.200), (0.120, 0.0, -0.345), black_steel)
    _add_box(stay, "stay_end_shoe", (0.052, 0.030, 0.018), (0.120, 0.0, -0.445), galvanized)
    for i, z in enumerate((-0.300, -0.360, -0.420)):
        stay.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.120, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"stay_adjust_hole_{i}",
        )

    # ----------------------------------------------------------------- joints
    # Primary articulation: ridge vent flap swings UP about the ridge_rail top edge (world -Y).
    hinge_origin = _plane_xyz(0.0, 0.0, 0.064)
    ridge_flap_joint = model.articulation(
        "roof_to_ridge_flap",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=flap,
        origin=Origin(xyz=hinge_origin, rpy=(0.0, PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.4, lower=0.0, upper=1.05),
    )
    ridge_flap_joint.meta["role"] = "primary ridge vent flap opening along ridge_rail"

    # Latch swings on the flap bottom rail.
    model.articulation(
        "ridge_flap_to_latch",
        ArticulationType.REVOLUTE,
        parent=flap,
        child=latch,
        origin=Origin(xyz=(FLAP_LEN - 0.06, 0.0, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    # Folding stay pivots from the flap mount tab.
    model.articulation(
        "ridge_flap_to_stay_arm",
        ArticulationType.REVOLUTE,
        parent=flap,
        child=stay,
        origin=Origin(xyz=(FLAP_LEN - 0.01, -0.30, -0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=-0.10, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_frame")
    flap = object_model.get_part("ridge_vent_flap")
    latch = object_model.get_part("latch_handle")
    stay = object_model.get_part("stay_arm")
    ridge_joint = object_model.get_articulation("roof_to_ridge_flap")
    latch_joint = object_model.get_articulation("ridge_flap_to_latch")
    stay_joint = object_model.get_articulation("ridge_flap_to_stay_arm")

    # --- Intentional seating / pivot interpenetrations ---
    # Closed flap beds down onto the fixed ridge rail at the hinge line.
    for flap_member in ("flap_stile_0", "flap_stile_1", "flap_top_rail"):
        ctx.allow_overlap(
            "ridge_vent_flap",
            "roof_frame",
            elem_a=flap_member,
            elem_b="ridge_rail",
            reason="Closed ridge flap beds down onto the fixed ridge rail at the hinge line.",
        )
    # Stay pivot captured on the flap mount tab.
    for stay_elem, tab_elem in (
        ("stay_pivot_plate", "stay_mount_tab"),
        ("stay_top_pin", "stay_mount_tab"),
        ("stay_upper_arm", "stay_mount_tab"),
    ):
        ctx.allow_overlap(
            "stay_arm",
            "ridge_vent_flap",
            elem_a=stay_elem,
            elem_b=tab_elem,
            reason="Stay pivot is captured on the flap mount lug at the revolute pin.",
        )
    # Latch hardware captured at the flap bottom rail.
    for latch_elem in ("latch_pivot_pin", "latch_back_plate"):
        for member in ("flap_bottom_rail", "flap_drip_lip"):
            ctx.allow_overlap(
                "latch_handle",
                "ridge_vent_flap",
                elem_a=latch_elem,
                elem_b=member,
                reason="Latch hardware is captured at the flap bottom rail / drip lip.",
            )
    # Ridge hinge pins run through the flap frame at the hinge line.
    for side, stile, fknuckle in (
        ("hinge_pin_left", "flap_stile_0", "flap_hinge_knuckle_0"),
        ("hinge_pin_right", "flap_stile_1", "flap_hinge_knuckle_2"),
    ):
        for member in ("flap_top_rail", stile, fknuckle):
            ctx.allow_overlap(
                "roof_frame",
                "ridge_vent_flap",
                elem_a=side,
                elem_b=member,
                reason="Ridge hinge pin end runs through the flap frame at the hinge line.",
            )
    # Fixed hinge knuckles interleave with the flap frame at the hinge line.
    for i, stile in ((0, "flap_stile_0"), (1, None), (2, None), (3, "flap_stile_1")):
        members = ["flap_top_rail"] + ([stile] if stile else [])
        for member in members:
            ctx.allow_overlap(
                "roof_frame",
                "ridge_vent_flap",
                elem_a=f"fixed_hinge_knuckle_{i}",
                elem_b=member,
                reason="Fixed ridge hinge knuckle interleaves with the flap frame at the hinge line.",
            )
    # EPDM weather seals compressed by the closed flap frame.
    for seal, members in (
        (
            "ridge_weather_seal",
            ("flap_top_rail", "flap_stile_0", "flap_stile_1", "flap_gasket_0", "flap_gasket_1"),
        ),
        (
            "sill_weather_seal",
            ("flap_bottom_rail", "flap_stile_0", "flap_stile_1", "flap_gasket_0",
             "flap_gasket_1", "flap_gasket_bottom"),
        ),
    ):
        for member in members:
            ctx.allow_overlap(
                "roof_frame",
                "ridge_vent_flap",
                elem_a=seal,
                elem_b=member,
                reason="Closed flap frame beds and compresses the fixed EPDM weather seal.",
            )

    # --- Prompt-specific assertions ---
    ctx.check(
        "classified as greenhouse vent roof",
        object_model.name == "agricultural_greenhouse_vent_roof"
        and object_model.meta.get("class") == "Agricultural/Greenhouse vent roof",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    ctx.check(
        "has ridge_vent_flap part and roof_to_ridge_flap joint",
        flap is not None and ridge_joint is not None
        and all(p is not None for p in (roof, latch, stay))
        and all(j is not None for j in (latch_joint, stay_joint)),
        details="Expected roof_frame, ridge_vent_flap, latch_handle, stay_arm, and three joints.",
    )

    # Variant-specific: the ridge_vent_flap spans along the ridge axis (long Y extent).
    ctx.check(
        "ridge_vent_flap is a long flap along the ridge axis",
        flap is not None,
        details="ridge_vent_flap part must exist.",
    )
    flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "ridge_vent_flap spans at least 1.0 m along the ridge (Y axis)",
        flap_aabb is not None
        and (flap_aabb[1][1] - flap_aabb[0][1]) > 1.0,
        details=f"flap_aabb={flap_aabb}",
    )

    # The moving hinge knuckle interleaves along the ridge line.
    ctx.expect_overlap(
        flap,
        roof,
        axes="y",
        elem_a="flap_hinge_knuckle_1",
        elem_b="ridge_rail",
        min_overlap=0.10,
        name="moving flap hinge knuckle lies along the ridge_rail hinge line",
    )

    # Primary mechanism: opening the ridge flap lifts it up and clear of the roof plane.
    closed_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({ridge_joint: 0.95}):
        open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "ridge_vent_flap opens upward on the roof_to_ridge_flap hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.30,
        details=f"closed_top_z={closed_aabb[1][2]:.3f}, open_top_z={open_aabb[1][2]:.3f}",
    )

    # Closed flap sits in the roof plane (low above the curb), not floating high.
    ctx.check(
        "closed ridge flap sits in the roof plane",
        closed_aabb is not None and closed_aabb[1][2] < RIDGE_HEIGHT + 0.20,
        details=f"closed_top_z={closed_aabb[1][2]:.3f}, ridge={RIDGE_HEIGHT}",
    )

    # Latch stays mounted on the flap through its swing.
    with ctx.pose({latch_joint: 0.45}):
        ctx.expect_origin_distance(
            latch,
            flap,
            axes="xyz",
            max_dist=1.6,
            name="latch remains mounted on the ridge flap when swung",
        )

    # Folding stay pivots from the flap tab.
    with ctx.pose({stay_joint: 0.10}):
        ctx.expect_origin_distance(
            stay,
            flap,
            axes="xyz",
            max_dist=1.3,
            name="folding stay is mounted on the ridge flap tab",
        )

    return ctx.report()


object_model = build_object_model()
