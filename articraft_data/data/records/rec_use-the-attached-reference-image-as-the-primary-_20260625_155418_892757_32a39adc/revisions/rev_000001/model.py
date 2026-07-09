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
VENT_U0, VENT_U1 = 0.07, 1.28   # vent opening band along the slope
VENT_HALF = 0.46                # vent half width across


def _plane_xyz(u: float, v: float, w: float) -> tuple[float, float, float]:
    """Map roof-plane coords (u=down-slope, v=across=world Y, w=out-of-plane) to world xyz.

    The plane is the roof pitched about the world Y axis by PITCH.
    """
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
                "aged-steel glazing bars, a top-hinged vent sash propped open on a folding "
                "scissor stay, a ridge hinge line, a latch handle, hanging tomato twine and a "
                "tensioned support wire."
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
    # Side mullions flanking the vent opening (span from ridge down past the curb sill).
    for i, v in enumerate((-0.60, 0.60)):
        _plane_box(roof, f"mullion_side_{i}", (1.40, 0.042, 0.040), 0.71, v, 0.026, aged_steel)

    # Curb framing the operable vent (curb sill at the bottom, jambs down the sides).
    _plane_box(roof, "vent_curb_sill", (0.065, 2 * VENT_HALF + 0.12, 0.048), VENT_U1, 0.0, 0.024, aluminum)
    for i, v in enumerate((-VENT_HALF - 0.04, VENT_HALF + 0.04)):
        _plane_box(roof, f"vent_curb_jamb_{i}", (VENT_U1 - VENT_U0, 0.045, 0.046), (VENT_U0 + VENT_U1) / 2.0, v, 0.024, aluminum)

    # EPDM weather seals the closed sash beds onto (on the ridge rail and the curb sill).
    _plane_box(roof, "ridge_weather_seal", (0.024, 2 * VENT_HALF, 0.014), 0.045, 0.0, 0.044, rubber)
    _plane_box(roof, "sill_weather_seal", (0.024, 2 * VENT_HALF, 0.014), VENT_U1 - 0.02, 0.0, 0.046, rubber)

    # Fixed glazing panes around the vent (lower field + side strips). Thin clear sheets.
    for ci, v in enumerate((-0.46, 0.0, 0.46)):
        for ri, u in enumerate((1.72, 2.31)):
            _plane_box(roof, f"glass_lower_{ci}_{ri}", (0.55, 0.44, 0.006), u, v, 0.045, glass)
    for i, v in enumerate((-0.61, 0.61)):
        _plane_box(roof, f"glass_side_{i}", (1.18, 0.18, 0.006), 0.71, v, 0.045, glass)

    # Ridge hinge line: fixed leaves + alternating knuckles the sash interleaves with.
    for i, v in enumerate((-0.34, 0.0, 0.34)):
        _plane_box(roof, f"fixed_hinge_leaf_{i}", (0.060, 0.150, 0.006), -0.05, v, 0.046, galvanized)
        _plane_cyl_y(roof, f"fixed_hinge_knuckle_{i}", 0.012, 0.150, 0.020, v, 0.064, galvanized)
    _plane_cyl_y(roof, "hinge_pin_left", 0.007, 0.040, 0.020, -0.42, 0.064, bolt_dark)
    _plane_cyl_y(roof, "hinge_pin_right", 0.007, 0.040, 0.020, 0.42, 0.064, bolt_dark)

    # Glazing-clip screws dotted along the lower rails (seated on the structural bars).
    sx = 0
    for u, v in ((1.42, -0.40), (1.42, 0.40), (2.02, -0.40), (2.02, 0.40), (2.01, 0.0)):
        roof.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=_plane_xyz(u, v, 0.046), rpy=(0.0, PITCH, 0.0)),
            material=bolt_dark,
            name=f"roof_screw_{sx}",
        )
        sx += 1

    # Tensioned support wire spanning the bay in the roof plane (ends bear on the rake rails).
    _plane_box(roof, "support_wire", (0.012, 2 * HALF_W + 0.10, 0.012), 1.95, 0.0, 0.0, wire)
    # Hanging tomato twine dropping through the wire, each with a tied loop knot at the bottom.
    for i, v in enumerate((-0.38, -0.05, 0.22, 0.50)):
        top = _plane_xyz(1.95, v, 0.012)
        length = 0.34 + 0.05 * i
        _vert_twine(roof, f"twine_{i}", top, length, 0.004, jute)
        _add_box(roof, f"twine_knot_{i}", (0.020, 0.014, 0.024), (top[0], top[1], top[2] - length), jute)

    # --------------------------------------------------------------- vent_sash
    # Authored flat in a hinge-line local frame: +X runs down-slope from the ridge hinge.
    vent = model.part("vent_sash")
    SASH_LEN = VENT_U1 - VENT_U0           # 1.21
    SASH_HALF = VENT_HALF                  # 0.46
    midx = SASH_LEN / 2.0

    _add_box(vent, "vent_glass", (SASH_LEN - 0.10, 2 * SASH_HALF - 0.08, 0.007), (midx, 0.0, -0.002), glass)
    _add_box(vent, "sash_top_rail", (0.060, 2 * SASH_HALF, 0.038), (0.030, 0.0, 0.0), aluminum)
    _add_box(vent, "sash_bottom_rail", (0.080, 2 * SASH_HALF, 0.038), (SASH_LEN - 0.030, 0.0, 0.0), aluminum)
    _add_box(vent, "sash_stile_0", (SASH_LEN, 0.052, 0.038), (midx, -SASH_HALF + 0.026, 0.0), aluminum)
    _add_box(vent, "sash_stile_1", (SASH_LEN, 0.052, 0.038), (midx, SASH_HALF - 0.026, 0.0), aged_steel)
    _add_box(vent, "sash_glazing_bar", (0.038, 2 * SASH_HALF - 0.08, 0.028), (midx, 0.0, 0.010), aluminum)
    _add_box(vent, "sash_drip_lip", (0.030, 2 * SASH_HALF - 0.06, 0.022), (SASH_LEN - 0.060, 0.0, -0.024), aluminum)

    # EPDM gaskets around the sash perimeter.
    _add_box(vent, "sash_gasket_0", (SASH_LEN - 0.10, 0.016, 0.012), (midx, -SASH_HALF + 0.012, -0.018), rubber)
    _add_box(vent, "sash_gasket_1", (SASH_LEN - 0.10, 0.016, 0.012), (midx, SASH_HALF - 0.012, -0.018), rubber)
    _add_box(vent, "sash_gasket_bottom", (0.018, 2 * SASH_HALF - 0.06, 0.012), (SASH_LEN - 0.020, 0.0, -0.010), rubber)

    # Moving hinge leaves + knuckles that interleave with the fixed ridge knuckles (local x~0).
    for i, v in enumerate((-0.17, 0.17, 0.50)):
        _add_box(vent, f"sash_hinge_leaf_{i}", (0.060, 0.145, 0.006), (0.020, v, 0.012), galvanized)
        _add_cyl_y(vent, f"sash_hinge_knuckle_{i}", 0.011, 0.145, (0.004, v, 0.006), galvanized)

    # Stay-arm mount lug projecting below the sash bottom rail (the folding stay hangs from it).
    _add_box(vent, "stay_mount_tab", (0.060, 0.080, 0.110), (SASH_LEN - 0.01, -0.30, -0.045), galvanized)
    for i, (x, v) in enumerate(((0.10, -0.40), (0.10, 0.40), (SASH_LEN - 0.06, -0.40), (SASH_LEN - 0.06, 0.40))):
        _add_box(vent, f"sash_corner_plate_{i}", (0.075, 0.045, 0.006), (x, v, 0.016), galvanized)
    for i, x in enumerate((0.22, 0.60, 0.98)):
        for j, v in enumerate((-0.41, 0.41)):
            vent.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(xyz=(x, v, 0.006)),
                material=bolt_dark,
                name=f"sash_screw_{i}_{j}",
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
    # Folding stay built as a connected zigzag strut in a local frame; revolute at the sash tab.
    # Origin = top pin on the bottom-rail mount tab; the strut reaches down-slope then folds
    # straight down past the opening (the visible black folding scissor of the reference).
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
    # Primary articulation: vent sash swings UP about the ridge hinge line (world -Y).
    hinge_origin = _plane_xyz(0.0, 0.0, 0.064)
    vent_hinge = model.articulation(
        "roof_to_vent_sash",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=vent,
        origin=Origin(xyz=hinge_origin, rpy=(0.0, PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.4, lower=0.0, upper=1.05),
    )
    vent_hinge.meta["role"] = "primary top-hinged roof vent opening"

    # Latch swings on the sash bottom rail.
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent=vent,
        child=latch,
        origin=Origin(xyz=(SASH_LEN - 0.06, 0.0, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    # Folding stay pivots from the sash mount tab.
    model.articulation(
        "sash_to_stay_arm",
        ArticulationType.REVOLUTE,
        parent=vent,
        child=stay,
        origin=Origin(xyz=(SASH_LEN - 0.01, -0.30, -0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=-0.10, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_frame")
    vent = object_model.get_part("vent_sash")
    latch = object_model.get_part("latch_handle")
    stay = object_model.get_part("stay_arm")
    hinge = object_model.get_articulation("roof_to_vent_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")
    stay_joint = object_model.get_articulation("sash_to_stay_arm")

    # Intentional seating / pivot interpenetrations (small, local, mechanically explanatory).
    ctx.allow_overlap(
        "vent_sash",
        "roof_frame",
        elem_a="sash_stile_0",
        elem_b="ridge_rail",
        reason="Closed sash beds down onto the fixed ridge rail at the hinge line.",
    )
    ctx.allow_overlap(
        "vent_sash",
        "roof_frame",
        elem_a="sash_stile_1",
        elem_b="ridge_rail",
        reason="Closed sash beds down onto the fixed ridge rail at the hinge line.",
    )
    ctx.allow_overlap(
        "vent_sash",
        "roof_frame",
        elem_a="sash_top_rail",
        elem_b="ridge_rail",
        reason="Closed sash top rail beds down onto the fixed ridge rail at the hinge line.",
    )
    ctx.allow_overlap(
        "stay_arm",
        "vent_sash",
        elem_a="stay_pivot_plate",
        elem_b="stay_mount_tab",
        reason="Stay pivot plate is captured on the sash mount tab at the revolute pin.",
    )
    ctx.allow_overlap(
        "stay_arm",
        "vent_sash",
        elem_a="stay_top_pin",
        elem_b="stay_mount_tab",
        reason="Stay top pin is captured through the sash mount lug at the revolute pin.",
    )
    ctx.allow_overlap(
        "stay_arm",
        "vent_sash",
        elem_a="stay_upper_arm",
        elem_b="stay_mount_tab",
        reason="The stay upper arm is captured inside the sash mount lug it pivots from.",
    )
    for latch_elem in ("latch_pivot_pin", "latch_back_plate"):
        for member in ("sash_bottom_rail", "sash_drip_lip"):
            ctx.allow_overlap(
                "latch_handle",
                "vent_sash",
                elem_a=latch_elem,
                elem_b=member,
                reason="Latch hardware is captured at the sash bottom rail / drip lip it mounts to.",
            )
    for side, stile, sknuckle in (
        ("hinge_pin_left", "sash_stile_0", "sash_hinge_knuckle_0"),
        ("hinge_pin_right", "sash_stile_1", "sash_hinge_knuckle_2"),
    ):
        for member in ("sash_top_rail", stile, sknuckle):
            ctx.allow_overlap(
                "roof_frame",
                "vent_sash",
                elem_a=side,
                elem_b=member,
                reason="Ridge hinge pin end runs through the sash frame at the hinge line.",
            )
    for i, stile in ((0, "sash_stile_0"), (1, None), (2, "sash_stile_1")):
        members = ["sash_top_rail"] + ([stile] if stile else [])
        for member in members:
            ctx.allow_overlap(
                "roof_frame",
                "vent_sash",
                elem_a=f"fixed_hinge_knuckle_{i}",
                elem_b=member,
                reason="Fixed ridge hinge knuckle interleaves with the sash frame at the hinge line.",
            )
    # Compliant EPDM weather seals are bedded (compressed) by the closed sash frame.
    for seal, members in (
        (
            "ridge_weather_seal",
            ("sash_top_rail", "sash_stile_0", "sash_stile_1", "sash_gasket_0", "sash_gasket_1"),
        ),
        (
            "sill_weather_seal",
            ("sash_bottom_rail", "sash_stile_0", "sash_stile_1", "sash_gasket_0",
             "sash_gasket_1", "sash_gasket_bottom"),
        ),
    ):
        for member in members:
            ctx.allow_overlap(
                "roof_frame",
                "vent_sash",
                elem_a=seal,
                elem_b=member,
                reason="Closed sash frame beds and compresses the fixed EPDM curb weather seal.",
            )

    ctx.check(
        "classified as greenhouse vent roof",
        object_model.name == "agricultural_greenhouse_vent_roof"
        and object_model.meta.get("class") == "Agricultural/Greenhouse vent roof",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    ctx.check(
        "has the vent / stay / latch subassemblies and joints",
        all(p is not None for p in (roof, vent, latch, stay))
        and all(j is not None for j in (hinge, latch_joint, stay_joint)),
        details="Expected roof frame, vent sash, latch handle, folding stay arm, and three joints.",
    )

    # The moving hinge knuckle interleaves along the ridge line (tilt-independent Y overlap).
    ctx.expect_overlap(
        vent,
        roof,
        axes="y",
        elem_a="sash_hinge_knuckle_1",
        elem_b="ridge_rail",
        min_overlap=0.10,
        name="moving hinge knuckle lies along the ridge hinge line",
    )

    # Primary mechanism: opening the vent lifts the sash up and clear of the roof plane.
    closed_aabb = ctx.part_world_aabb(vent)
    with ctx.pose({hinge: 0.95}):
        open_aabb = ctx.part_world_aabb(vent)
    ctx.check(
        "vent opens upward on the ridge hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.30,
        details=f"closed_top_z={closed_aabb[1][2]:.3f}, open_top_z={open_aabb[1][2]:.3f}",
    )

    # Closed sash beds down near the roof plane (low above the curb), not floating high.
    ctx.check(
        "closed sash sits in the roof plane",
        closed_aabb is not None and closed_aabb[1][2] < RIDGE_HEIGHT + 0.20,
        details=f"closed_top_z={closed_aabb[1][2]:.3f}, ridge={RIDGE_HEIGHT}",
    )

    # Latch stays mounted on the sash through its swing.
    with ctx.pose({latch_joint: 0.45}):
        ctx.expect_origin_distance(
            latch,
            vent,
            axes="xyz",
            max_dist=1.6,
            name="latch remains mounted on the sash when swung",
        )

    # Folding stay pivots from the sash tab (foot stays within reach of the curb bracket).
    with ctx.pose({stay_joint: 0.10}):
        ctx.expect_origin_distance(
            stay,
            vent,
            axes="xyz",
            max_dist=1.3,
            name="folding stay is mounted on the sash tab",
        )

    return ctx.report()


object_model = build_object_model()
