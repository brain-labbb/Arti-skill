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

# ── COMPANION_VARIATIONS: pitch angle per slope ──────────────────────────
# Five variant pitches (radians): 0.26 (~15°), 0.35 (~20°), 0.42 (~24°),
# 0.52 (~30°), 0.61 (~35°).  Changing PITCH re-derives every slope element.
PITCH = 0.42
_COS_P = math.cos(PITCH)
_SIN_P = math.sin(PITCH)
RIDGE_HEIGHT = 2.18

HALF_W = 0.72                     # half width along the ridge (world Y)
EAVE_U = 2.60                     # rafter run from ridge to eave (per slope)
VENT_U0, VENT_U1 = 0.07, 1.28    # vent opening band along the right slope
VENT_HALF = 0.46                  # vent half width across


# ── slope-plane coordinate mappers ───────────────────────────────────────

def _right_plane_xyz(u: float, v: float, w: float) -> tuple[float, float, float]:
    """Right slope: u = down-slope from ridge, v = across (Y), w = above surface."""
    x = u * _COS_P + w * _SIN_P
    y = v
    z = RIDGE_HEIGHT - u * _SIN_P + w * _COS_P
    return (x, y, z)


def _left_plane_xyz(u: float, v: float, w: float) -> tuple[float, float, float]:
    """Left slope (mirror): u = down-slope from ridge, v = across (Y), w = above surface."""
    x = -(u * _COS_P) - w * _SIN_P
    y = v
    z = RIDGE_HEIGHT - u * _SIN_P + w * _COS_P
    return (x, y, z)


def _right_plane_box(part, name, size, u, v, w, material):
    """Box lying on the right slope.  size = (along-slope, across, thickness)."""
    part.visual(
        Box(size),
        origin=Origin(xyz=_right_plane_xyz(u, v, w), rpy=(0.0, PITCH, 0.0)),
        material=material,
        name=name,
    )


def _left_plane_box(part, name, size, u, v, w, material):
    """Box lying on the left slope (mirror)."""
    part.visual(
        Box(size),
        origin=Origin(xyz=_left_plane_xyz(u, v, w), rpy=(0.0, -PITCH, 0.0)),
        material=material,
        name=name,
    )


def _right_plane_cyl_y(part, name, radius, length, u, v, w, material):
    """Y-axis cylinder on the right slope (runs along the ridge direction)."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_right_plane_xyz(u, v, w), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _left_plane_cyl_y(part, name, radius, length, u, v, w, material):
    """Y-axis cylinder on the left slope."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_left_plane_xyz(u, v, w), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _vert_twine(part, name, top_xyz, length, radius, material):
    """Thin cord hanging straight down in world Z from a roof point."""
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


# ── build ────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_greenhouse_vent_roof",
        meta={
            "class": "Agricultural/Greenhouse vent roof",
            "description": (
                "An even-span greenhouse roof bay with two symmetric pitched slopes: "
                "glazed pane grid in weathered aluminium and aged-steel glazing bars, "
                "a top-hinged vent sash on the right slope propped open on a folding "
                "scissor stay, a ridge hinge line, a latch handle, hanging tomato "
                "twine and a tensioned support wire."
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

    # ─────────────────────────── roof_frame ───────────────────────────────
    roof = model.part("roof_frame")

    # Ridge cap rail – horizontal at the peak, bridging both slopes.
    _add_box(roof, "ridge_rail",
             (0.085, 2 * HALF_W + 0.06, 0.060),
             (0.0, 0.0, RIDGE_HEIGHT + 0.030), aluminum)

    # ────────── RIGHT SLOPE (vent side, +X) ──────────
    _right_plane_box(roof, "eave_rail_right",
                     (0.060, 2 * HALF_W + 0.06, 0.050), EAVE_U, 0.0, 0.022, aluminum)
    _right_plane_box(roof, "rake_rail_right_0",
                     (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, -HALF_W, 0.022, aluminum)
    _right_plane_box(roof, "rake_rail_right_1",
                     (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, HALF_W, 0.022, aged_steel)

    # Right-slope transoms (across-slope glazing bars).
    _right_plane_box(roof, "transom_right_mid",
                     (0.050, 2 * HALF_W, 0.044), 1.42, 0.0, 0.024, aged_steel)
    _right_plane_box(roof, "transom_right_low",
                     (0.050, 2 * HALF_W, 0.044), 2.02, 0.0, 0.024, aluminum)

    # Right-slope mullions (down-slope bars).
    for i, v in enumerate((-0.24, 0.24)):
        _right_plane_box(roof, f"mullion_right_lower_{i}",
                         (1.22, 0.042, 0.040), 2.01, v, 0.026, aluminum)
    for i, v in enumerate((-0.60, 0.60)):
        _right_plane_box(roof, f"mullion_right_side_{i}",
                         (1.40, 0.042, 0.040), 0.71, v, 0.026, aged_steel)

    # Vent curb framing the operable opening on the right slope.
    _right_plane_box(roof, "vent_curb_sill",
                     (0.065, 2 * VENT_HALF + 0.12, 0.048), VENT_U1, 0.0, 0.024, aluminum)
    for i, v in enumerate((-VENT_HALF - 0.04, VENT_HALF + 0.04)):
        _right_plane_box(roof, f"vent_curb_jamb_{i}",
                         (VENT_U1 - VENT_U0, 0.045, 0.046),
                         (VENT_U0 + VENT_U1) / 2.0, v, 0.024, aluminum)

    # EPDM weather seals (bed surfaces the closed sash compresses onto).
    # Ridge seal sits right at the hinge line, contacting the ridge cap rail.
    _right_plane_box(roof, "ridge_weather_seal",
                     (0.024, 2 * VENT_HALF, 0.014), 0.020, 0.0, 0.044, rubber)
    _right_plane_box(roof, "sill_weather_seal",
                     (0.024, 2 * VENT_HALF, 0.014), VENT_U1 - 0.02, 0.0, 0.046, rubber)

    # Fixed glazing panes on the right slope (below vent + side strips).
    for ci, v in enumerate((-0.46, 0.0, 0.46)):
        for ri, u in enumerate((1.72, 2.31)):
            _right_plane_box(roof, f"glass_lower_{ci}_{ri}",
                             (0.55, 0.44, 0.006), u, v, 0.045, glass)
    for i, v in enumerate((-0.61, 0.61)):
        _right_plane_box(roof, f"glass_side_{i}",
                         (1.18, 0.18, 0.006), 0.71, v, 0.045, glass)

    # Ridge hinge line: fixed leaves + alternating knuckles the sash interleaves with.
    for i, v in enumerate((-0.34, 0.0, 0.34)):
        _right_plane_box(roof, f"fixed_hinge_leaf_{i}",
                         (0.060, 0.150, 0.006), -0.01, v, 0.046, galvanized)
        _right_plane_cyl_y(roof, f"fixed_hinge_knuckle_{i}",
                           0.012, 0.150, 0.020, v, 0.064, galvanized)
    _right_plane_cyl_y(roof, "hinge_pin_left",
                       0.007, 0.040, 0.020, -0.42, 0.064, bolt_dark)
    _right_plane_cyl_y(roof, "hinge_pin_right",
                       0.007, 0.040, 0.020, 0.42, 0.064, bolt_dark)

    # Glazing-clip screws seated on the right-slope structural bars.
    sx = 0
    for u, v in ((1.42, -0.40), (1.42, 0.40), (2.02, -0.40), (2.02, 0.40), (2.01, 0.0)):
        roof.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=_right_plane_xyz(u, v, 0.046), rpy=(0.0, PITCH, 0.0)),
            material=bolt_dark,
            name=f"roof_screw_{sx}",
        )
        sx += 1

    # Tensioned support wire spanning the right slope + hanging tomato twine.
    _right_plane_box(roof, "support_wire",
                     (0.012, 2 * HALF_W + 0.10, 0.012), 1.95, 0.0, 0.0, wire)
    for i, v in enumerate((-0.38, -0.05, 0.22, 0.50)):
        top = _right_plane_xyz(1.95, v, 0.012)
        length = 0.34 + 0.05 * i
        _vert_twine(roof, f"twine_{i}", top, length, 0.004, jute)
        _add_box(roof, f"twine_knot_{i}",
                 (0.020, 0.014, 0.024), (top[0], top[1], top[2] - length), jute)

    # ────────── LEFT SLOPE (mirror, −X, no vent) ──────────
    _left_plane_box(roof, "eave_rail_left",
                    (0.060, 2 * HALF_W + 0.06, 0.050), EAVE_U, 0.0, 0.022, aluminum)
    _left_plane_box(roof, "rake_rail_left_0",
                    (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, -HALF_W, 0.022, aluminum)
    _left_plane_box(roof, "rake_rail_left_1",
                    (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, HALF_W, 0.022, aged_steel)

    # Left-slope transoms (including one at u=0.70 where the right slope has the vent).
    for i, u in enumerate((0.70, 1.42, 2.02)):
        _left_plane_box(roof, f"transom_left_{i}",
                        (0.050, 2 * HALF_W, 0.044), u, 0.0, 0.024,
                        aged_steel if i < 2 else aluminum)

    # Left-slope mullions (full-length intermediate rafters).
    for i, v in enumerate((-0.24, 0.24)):
        _left_plane_box(roof, f"mullion_left_{i}",
                        (EAVE_U - 0.12, 0.042, 0.040), EAVE_U / 2.0, v, 0.026, aluminum)

    # Left-slope fixed glazing (full field, no vent opening).
    for ci, v in enumerate((-0.46, 0.0, 0.46)):
        for ri, u in enumerate((0.36, 1.06, 1.72, 2.31)):
            _left_plane_box(roof, f"glass_left_{ci}_{ri}",
                            (0.50, 0.44, 0.006), u, v, 0.045, glass)

    # Left-slope glazing screws.
    for i, (u, v) in enumerate(((1.42, -0.40), (1.42, 0.40),
                                (2.02, -0.40), (2.02, 0.40))):
        roof.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=_left_plane_xyz(u, v, 0.046), rpy=(0.0, -PITCH, 0.0)),
            material=bolt_dark,
            name=f"left_roof_screw_{i}",
        )

    # ─────────────────────────── vent_sash ────────────────────────────────
    # Local frame: +X runs down-slope from the ridge hinge; authored flat.
    vent = model.part("vent_sash")
    SASH_LEN = VENT_U1 - VENT_U0           # 1.21
    SASH_HALF = VENT_HALF                  # 0.46
    midx = SASH_LEN / 2.0

    _add_box(vent, "vent_glass",
             (SASH_LEN - 0.10, 2 * SASH_HALF - 0.08, 0.007), (midx, 0.0, -0.002), glass)
    _add_box(vent, "sash_top_rail",
             (0.060, 2 * SASH_HALF, 0.038), (0.030, 0.0, 0.0), aluminum)
    _add_box(vent, "sash_bottom_rail",
             (0.080, 2 * SASH_HALF, 0.038), (SASH_LEN - 0.030, 0.0, 0.0), aluminum)
    _add_box(vent, "sash_stile_0",
             (SASH_LEN, 0.052, 0.038), (midx, -SASH_HALF + 0.026, 0.0), aluminum)
    _add_box(vent, "sash_stile_1",
             (SASH_LEN, 0.052, 0.038), (midx, SASH_HALF - 0.026, 0.0), aged_steel)
    _add_box(vent, "sash_glazing_bar",
             (0.038, 2 * SASH_HALF - 0.08, 0.028), (midx, 0.0, 0.010), aluminum)
    _add_box(vent, "sash_drip_lip",
             (0.030, 2 * SASH_HALF - 0.06, 0.022), (SASH_LEN - 0.060, 0.0, -0.024), aluminum)

    # EPDM gaskets around the sash perimeter.
    _add_box(vent, "sash_gasket_0",
             (SASH_LEN - 0.10, 0.016, 0.012), (midx, -SASH_HALF + 0.012, -0.018), rubber)
    _add_box(vent, "sash_gasket_1",
             (SASH_LEN - 0.10, 0.016, 0.012), (midx, SASH_HALF - 0.012, -0.018), rubber)
    _add_box(vent, "sash_gasket_bottom",
             (0.018, 2 * SASH_HALF - 0.06, 0.012), (SASH_LEN - 0.020, 0.0, -0.010), rubber)

    # Moving hinge leaves + knuckles interleaving with the fixed ridge knuckles.
    for i, v in enumerate((-0.17, 0.17, 0.50)):
        _add_box(vent, f"sash_hinge_leaf_{i}",
                 (0.060, 0.145, 0.006), (0.020, v, 0.012), galvanized)
        _add_cyl_y(vent, f"sash_hinge_knuckle_{i}",
                   0.011, 0.145, (0.004, v, 0.006), galvanized)

    # Stay-arm mount lug below the sash bottom rail.
    _add_box(vent, "stay_mount_tab",
             (0.060, 0.080, 0.110), (SASH_LEN - 0.01, -0.30, -0.045), galvanized)
    for i, (x, v) in enumerate(((0.10, -0.40), (0.10, 0.40),
                                (SASH_LEN - 0.06, -0.40), (SASH_LEN - 0.06, 0.40))):
        _add_box(vent, f"sash_corner_plate_{i}",
                 (0.075, 0.045, 0.006), (x, v, 0.016), galvanized)
    for i, x in enumerate((0.22, 0.60, 0.98)):
        for j, v in enumerate((-0.41, 0.41)):
            vent.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(xyz=(x, v, 0.006)),
                material=bolt_dark,
                name=f"sash_screw_{i}_{j}",
            )

    # ─────────────────────────── latch_handle ─────────────────────────────
    latch = model.part("latch_handle")
    _add_cyl_y(latch, "latch_pivot_pin", 0.015, 0.075, (0.0, 0.0, 0.0), bolt_dark)
    _add_box(latch, "latch_back_plate",
             (0.065, 0.085, 0.010), (0.0, 0.0, -0.006), galvanized)
    _add_box(latch, "latch_hook_tongue",
             (0.115, 0.024, 0.012), (0.090, 0.0, -0.078), black_steel)
    _add_box(latch, "latch_hook_drop",
             (0.018, 0.024, 0.072), (0.034, 0.0, -0.044), black_steel)
    _add_box(latch, "pull_handle_stem",
             (0.020, 0.028, 0.140), (-0.012, 0.0, -0.080), black_steel)
    _add_box(latch, "rubber_grip",
             (0.038, 0.110, 0.024), (-0.012, 0.0, -0.150), rubber)

    # ─────────────────────────── stay_arm ─────────────────────────────────
    stay = model.part("stay_arm")
    _add_cyl_y(stay, "stay_top_pin", 0.010, 0.040, (0.0, 0.0, 0.0), bolt_dark)
    _add_box(stay, "stay_pivot_plate",
             (0.050, 0.050, 0.010), (0.0, 0.0, -0.006), galvanized)
    _add_box(stay, "stay_upper_arm",
             (0.020, 0.020, 0.240), (0.0, 0.0, -0.125), black_steel)
    _add_cyl_y(stay, "stay_knuckle", 0.014, 0.026, (0.0, 0.0, -0.245), bolt_dark)
    _add_box(stay, "stay_elbow_jog",
             (0.140, 0.020, 0.020), (0.060, 0.0, -0.245), black_steel)
    _add_box(stay, "stay_lower_arm",
             (0.020, 0.020, 0.200), (0.120, 0.0, -0.345), black_steel)
    _add_box(stay, "stay_end_shoe",
             (0.052, 0.030, 0.018), (0.120, 0.0, -0.445), galvanized)
    for i, z in enumerate((-0.300, -0.360, -0.420)):
        stay.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.120, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"stay_adjust_hole_{i}",
        )

    # ─────────────────────────── joints ───────────────────────────────────
    # Primary articulation: vent sash swings UP about the ridge hinge line on
    # the right slope.  Positive q lifts the free (eave) edge skyward.
    hinge_origin = _right_plane_xyz(0.0, 0.0, 0.064)
    vent_hinge = model.articulation(
        "roof_to_vent_sash",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=vent,
        origin=Origin(xyz=hinge_origin, rpy=(0.0, PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.4, lower=0.0, upper=1.05),
    )
    vent_hinge.meta["role"] = "primary top-hinged roof vent opening (right slope)"

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


# ── tests ────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_frame")
    vent = object_model.get_part("vent_sash")
    latch = object_model.get_part("latch_handle")
    stay = object_model.get_part("stay_arm")
    hinge = object_model.get_articulation("roof_to_vent_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")
    stay_joint = object_model.get_articulation("sash_to_stay_arm")

    # ── intentional seating / pivot interpenetrations ────────────────────
    ctx.allow_overlap(
        "vent_sash", "roof_frame",
        elem_a="sash_stile_0", elem_b="ridge_rail",
        reason="Closed sash beds down onto the ridge cap rail at the hinge line.",
    )
    ctx.allow_overlap(
        "vent_sash", "roof_frame",
        elem_a="sash_stile_1", elem_b="ridge_rail",
        reason="Closed sash beds down onto the ridge cap rail at the hinge line.",
    )
    ctx.allow_overlap(
        "vent_sash", "roof_frame",
        elem_a="sash_top_rail", elem_b="ridge_rail",
        reason="Closed sash top rail beds down onto the ridge cap rail at the hinge line.",
    )
    ctx.allow_overlap(
        "stay_arm", "vent_sash",
        elem_a="stay_pivot_plate", elem_b="stay_mount_tab",
        reason="Stay pivot plate is captured on the sash mount tab at the revolute pin.",
    )
    ctx.allow_overlap(
        "stay_arm", "vent_sash",
        elem_a="stay_top_pin", elem_b="stay_mount_tab",
        reason="Stay top pin is captured through the sash mount lug at the revolute pin.",
    )
    ctx.allow_overlap(
        "stay_arm", "vent_sash",
        elem_a="stay_upper_arm", elem_b="stay_mount_tab",
        reason="The stay upper arm is captured inside the sash mount lug it pivots from.",
    )
    for latch_elem in ("latch_pivot_pin", "latch_back_plate"):
        for member in ("sash_bottom_rail", "sash_drip_lip"):
            ctx.allow_overlap(
                "latch_handle", "vent_sash",
                elem_a=latch_elem, elem_b=member,
                reason="Latch hardware is captured at the sash bottom rail / drip lip it mounts to.",
            )
    for side, stile, sknuckle in (
        ("hinge_pin_left", "sash_stile_0", "sash_hinge_knuckle_0"),
        ("hinge_pin_right", "sash_stile_1", "sash_hinge_knuckle_2"),
    ):
        for member in ("sash_top_rail", stile, sknuckle):
            ctx.allow_overlap(
                "roof_frame", "vent_sash",
                elem_a=side, elem_b=member,
                reason="Ridge hinge pin end runs through the sash frame at the hinge line.",
            )
    for i, stile in ((0, "sash_stile_0"), (1, None), (2, "sash_stile_1")):
        members = ["sash_top_rail"] + ([stile] if stile else [])
        for member in members:
            ctx.allow_overlap(
                "roof_frame", "vent_sash",
                elem_a=f"fixed_hinge_knuckle_{i}", elem_b=member,
                reason="Fixed ridge hinge knuckle interleaves with the sash frame at the hinge line.",
            )
    # Fixed hinge leaves interleave with sash hinge leaves at the hinge line.
    for i, stile in ((0, "sash_stile_0"), (1, None), (2, "sash_stile_1")):
        members = ["sash_top_rail"] + ([stile] if stile else [])
        for member in members:
            ctx.allow_overlap(
                "roof_frame", "vent_sash",
                elem_a=f"fixed_hinge_leaf_{i}", elem_b=member,
                reason="Fixed ridge hinge leaf interleaves with the sash frame at the hinge line.",
            )
    # Sash hinge knuckles sit at the hinge line under the ridge cap rail.
    for i in range(3):
        ctx.allow_overlap(
            "roof_frame", "vent_sash",
            elem_a="ridge_rail", elem_b=f"sash_hinge_knuckle_{i}",
            reason="Sash hinge knuckle sits at the hinge line captured under the ridge cap rail.",
        )
    # Sash hinge leaves bed onto the ridge cap rail at the hinge line.
    for i in range(3):
        ctx.allow_overlap(
            "roof_frame", "vent_sash",
            elem_a="ridge_rail", elem_b=f"sash_hinge_leaf_{i}",
            reason="Sash hinge leaf beds onto the ridge cap rail at the hinge line.",
        )
    # Compliant EPDM weather seals bedded (compressed) by the closed sash.
    for seal, members in (
        ("ridge_weather_seal",
         ("sash_top_rail", "sash_stile_0", "sash_stile_1",
          "sash_gasket_0", "sash_gasket_1")),
        ("sill_weather_seal",
         ("sash_bottom_rail", "sash_stile_0", "sash_stile_1",
          "sash_gasket_0", "sash_gasket_1", "sash_gasket_bottom")),
    ):
        for member in members:
            ctx.allow_overlap(
                "roof_frame", "vent_sash",
                elem_a=seal, elem_b=member,
                reason="Closed sash frame beds and compresses the fixed EPDM curb weather seal.",
            )

    # ── classification & subassembly presence ────────────────────────────
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

    # ── even-span roof structure (TARGET axis) ───────────────────────────
    roof_aabb = ctx.part_world_aabb(roof)
    ctx.check(
        "even-span roof: eave_rail_left and eave_rail_right on mirrored slopes",
        roof_aabb is not None
        and roof_aabb[0][0] < -1.5   # left slope extends well into −X
        and roof_aabb[1][0] > 1.5,   # right slope extends well into +X
        details=(
            f"roof_aabb_min_x={roof_aabb[0][0]:.3f}, "
            f"roof_aabb_max_x={roof_aabb[1][0]:.3f}"
            if roof_aabb else "no aabb"
        ),
    )

    # The ridge rail sits at the peak above both slopes (horizontal cap).
    vent_aabb_closed = ctx.part_world_aabb(vent)
    ctx.check(
        "ridge_rail sits above the vent sash closed position at the peak",
        roof_aabb is not None and vent_aabb_closed is not None
        and roof_aabb[1][2] >= vent_aabb_closed[1][2] - 0.05,
        details=(
            f"roof_top_z={roof_aabb[1][2]:.3f}, "
            f"vent_top_z={vent_aabb_closed[1][2]:.3f}"
            if roof_aabb and vent_aabb_closed else "no aabb"
        ),
    )

    # ── hinge knuckle along the ridge ────────────────────────────────────
    ctx.expect_overlap(
        vent, roof,
        axes="y",
        elem_a="sash_hinge_knuckle_1", elem_b="ridge_rail",
        min_overlap=0.10,
        name="moving hinge knuckle lies along the ridge hinge line",
    )

    # ── primary mechanism: vent opens upward ─────────────────────────────
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

    # Closed sash sits in the right slope plane (not floating high).
    ctx.check(
        "closed sash sits in the right slope plane",
        closed_aabb is not None and closed_aabb[1][2] < RIDGE_HEIGHT + 0.20,
        details=f"closed_top_z={closed_aabb[1][2]:.3f}, ridge={RIDGE_HEIGHT}",
    )

    # Vent sash is on the right (+X) slope of the even-span roof.
    ctx.check(
        "vent sash is on the right slope (positive X side)",
        closed_aabb is not None and closed_aabb[0][0] > -0.20,
        details=f"vent_aabb_min_x={closed_aabb[0][0]:.3f}" if closed_aabb else "no aabb",
    )

    # ── latch stays mounted through swing ────────────────────────────────
    with ctx.pose({latch_joint: 0.45}):
        ctx.expect_origin_distance(
            latch, vent,
            axes="xyz", max_dist=1.6,
            name="latch remains mounted on the sash when swung",
        )

    # ── folding stay mounted on sash tab ─────────────────────────────────
    with ctx.pose({stay_joint: 0.10}):
        ctx.expect_origin_distance(
            stay, vent,
            axes="xyz", max_dist=1.3,
            name="folding stay is mounted on the sash tab",
        )

    return ctx.report()


object_model = build_object_model()
