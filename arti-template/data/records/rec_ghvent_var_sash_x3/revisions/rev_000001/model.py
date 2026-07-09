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

# --- Layout constants ---
N_BAYS = 3
HALF_W = 1.15                     # half width across the slope (world Y)
EAVE_U = 2.60                     # down-slope length to the eave
VENT_U0, VENT_U1 = 0.07, 1.28    # vent opening band along the slope
VENT_HALF = 0.28                  # vent half width per bay (across Y)
BAY_SPACING = 0.66                # center-to-center bay spacing along ridge
BAY_CENTERS = [(i - (N_BAYS - 1) / 2.0) * BAY_SPACING for i in range(N_BAYS)]
# = [-0.66, 0.0, 0.66]

SASH_LEN = VENT_U1 - VENT_U0     # 1.21


def _plane_xyz(u: float, v: float, w: float) -> tuple[float, float, float]:
    """Map roof-plane coords (u=down-slope, v=across=world Y, w=out-of-plane) to world xyz."""
    x = u * _COS_P + w * _SIN_P
    y = v
    z = RIDGE_HEIGHT - u * _SIN_P + w * _COS_P
    return (x, y, z)


def _plane_box(part, name, size, u, v, w, material):
    """Add a box lying in the pitched roof plane."""
    part.visual(
        Box(size),
        origin=Origin(xyz=_plane_xyz(u, v, w), rpy=(0.0, PITCH, 0.0)),
        material=material,
        name=name,
    )


def _plane_cyl_y(part, name, radius, length, u, v, w, material):
    """Add a Y-axis cylinder at a plane point."""
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


def _build_vent_sash(model, bay_idx, mats):
    """Build one vent sash part in its local hinge-line frame (+X runs down-slope from hinge)."""
    aluminum, aged_steel, galvanized, glass, rubber, black_steel, bolt_dark = mats
    vent = model.part(f"vent_sash_{bay_idx}")
    SH = VENT_HALF
    midx = SASH_LEN / 2.0

    # Glazing pane
    _add_box(vent, "vent_glass", (SASH_LEN - 0.10, 2 * SH - 0.08, 0.007),
             (midx, 0.0, -0.002), glass)

    # Frame rails (top = hinge edge, bottom = latch edge)
    _add_box(vent, "sash_top_rail", (0.060, 2 * SH, 0.038),
             (0.030, 0.0, 0.0), aluminum)
    _add_box(vent, "sash_bottom_rail", (0.080, 2 * SH, 0.038),
             (SASH_LEN - 0.030, 0.0, 0.0), aluminum)

    # Stiles (side members)
    _add_box(vent, "sash_stile_0", (SASH_LEN, 0.052, 0.038),
             (midx, -SH + 0.026, 0.0), aluminum)
    _add_box(vent, "sash_stile_1", (SASH_LEN, 0.052, 0.038),
             (midx, SH - 0.026, 0.0), aged_steel)

    # Central glazing bar
    _add_box(vent, "sash_glazing_bar", (0.038, 2 * SH - 0.08, 0.028),
             (midx, 0.0, 0.010), aluminum)

    # Drip lip at bottom edge
    _add_box(vent, "sash_drip_lip", (0.030, 2 * SH - 0.06, 0.022),
             (SASH_LEN - 0.060, 0.0, -0.024), aluminum)

    # EPDM gaskets around sash perimeter
    _add_box(vent, "sash_gasket_0", (SASH_LEN - 0.10, 0.016, 0.012),
             (midx, -SH + 0.012, -0.018), rubber)
    _add_box(vent, "sash_gasket_1", (SASH_LEN - 0.10, 0.016, 0.012),
             (midx, SH - 0.012, -0.018), rubber)
    _add_box(vent, "sash_gasket_bottom", (0.018, 2 * SH - 0.06, 0.012),
             (SASH_LEN - 0.020, 0.0, -0.010), rubber)

    # Moving hinge knuckles (interleave with fixed ridge knuckles).
    # Fixed knuckles at dv=(-0.18, 0.0, 0.18) len=0.080; leaves fit in the gaps.
    for i, v in enumerate((-0.09, 0.09)):
        _add_box(vent, f"sash_hinge_leaf_{i}", (0.060, 0.060, 0.006),
                 (0.020, v, 0.012), galvanized)
        _add_cyl_y(vent, f"sash_hinge_knuckle_{i}", 0.011, 0.060,
                   (0.004, v, 0.006), galvanized)

    # Stay-arm mount lug below the sash bottom rail
    _add_box(vent, "stay_mount_tab", (0.060, 0.080, 0.110),
             (SASH_LEN - 0.01, -0.16, -0.045), galvanized)

    # Corner reinforcement plates
    stile_v = SH - 0.04
    for i, (x, v) in enumerate(((0.10, -stile_v), (0.10, stile_v),
                                 (SASH_LEN - 0.06, -stile_v), (SASH_LEN - 0.06, stile_v))):
        _add_box(vent, f"sash_corner_plate_{i}", (0.075, 0.045, 0.006),
                 (x, v, 0.016), galvanized)

    # Glazing-clip screws on the sash frame
    for i, x in enumerate((0.22, 0.60, 0.98)):
        for j, v in enumerate((-SH + 0.03, SH - 0.03)):
            vent.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(xyz=(x, v, 0.006)),
                material=bolt_dark,
                name=f"sash_screw_{i}_{j}",
            )

    return vent


def _build_stay_arm(model, bay_idx, mats):
    """Build one folding stay arm part (scissor stay for propping the vent open)."""
    aluminum, aged_steel, galvanized, glass, rubber, black_steel, bolt_dark = mats
    stay = model.part(f"stay_arm_{bay_idx}")

    _add_cyl_y(stay, "stay_top_pin", 0.010, 0.040, (0.0, 0.0, 0.0), bolt_dark)
    _add_box(stay, "stay_pivot_plate", (0.050, 0.050, 0.010),
             (0.0, 0.0, -0.006), galvanized)
    _add_box(stay, "stay_upper_arm", (0.020, 0.020, 0.240),
             (0.0, 0.0, -0.125), black_steel)
    _add_cyl_y(stay, "stay_knuckle", 0.014, 0.026,
               (0.0, 0.0, -0.245), bolt_dark)
    _add_box(stay, "stay_elbow_jog", (0.140, 0.020, 0.020),
             (0.060, 0.0, -0.245), black_steel)
    _add_box(stay, "stay_lower_arm", (0.020, 0.020, 0.200),
             (0.120, 0.0, -0.345), black_steel)
    _add_box(stay, "stay_end_shoe", (0.052, 0.030, 0.018),
             (0.120, 0.0, -0.445), galvanized)

    for i, z in enumerate((-0.300, -0.360, -0.420)):
        stay.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.120, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"stay_adjust_hole_{i}",
        )

    return stay


def _build_latch(model, bay_idx, mats):
    """Build one latch handle part (hooks onto the curb sill to hold the vent closed)."""
    aluminum, aged_steel, galvanized, glass, rubber, black_steel, bolt_dark = mats
    latch = model.part(f"latch_{bay_idx}")

    _add_cyl_y(latch, "latch_pivot_pin", 0.015, 0.075, (0.0, 0.0, 0.0), bolt_dark)
    _add_box(latch, "latch_back_plate", (0.065, 0.085, 0.010),
             (0.0, 0.0, -0.006), galvanized)
    _add_box(latch, "latch_hook_tongue", (0.115, 0.024, 0.012),
             (0.090, 0.0, -0.078), black_steel)
    _add_box(latch, "latch_hook_drop", (0.018, 0.024, 0.072),
             (0.034, 0.0, -0.044), black_steel)
    _add_box(latch, "pull_handle_stem", (0.020, 0.028, 0.140),
             (-0.012, 0.0, -0.080), black_steel)
    _add_box(latch, "rubber_grip", (0.038, 0.110, 0.024),
             (-0.012, 0.0, -0.150), rubber)

    return latch


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_greenhouse_vent_roof",
        meta={
            "class": "Agricultural/Greenhouse vent roof",
            "description": (
                "A pitched greenhouse roof bay with three ridge-hinged vent sashes "
                "along the ridge, glazed pane grid in weathered aluminum and aged-steel "
                "glazing bars, folding scissor stays, latch handles, EPDM weather seals, "
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
    wire_mat = model.material("tension_wire", rgba=(0.62, 0.60, 0.55, 1.0))

    mats = (aluminum, aged_steel, galvanized, glass, rubber, black_steel, bolt_dark)

    # ================================================================= roof_frame
    roof = model.part("roof_frame")

    # Perimeter and structural rails of the pitched roof bay
    _plane_box(roof, "ridge_rail", (0.085, 2 * HALF_W + 0.06, 0.060), 0.0, 0.0, 0.028, aluminum)
    _plane_box(roof, "eave_rail", (0.060, 2 * HALF_W + 0.06, 0.050), EAVE_U, 0.0, 0.022, aluminum)
    _plane_box(roof, "rake_rail_0", (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, -HALF_W, 0.022, aluminum)
    _plane_box(roof, "rake_rail_1", (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, HALF_W, 0.022, aged_steel)

    # Transoms (across-slope glazing bars)
    _plane_box(roof, "transom_mid", (0.050, 2 * HALF_W, 0.044), 1.42, 0.0, 0.024, aged_steel)
    _plane_box(roof, "transom_low", (0.050, 2 * HALF_W, 0.044), 2.02, 0.0, 0.024, aluminum)

    # Mullions (down-slope glazing bars) in the lower fixed glazing field
    for i, v in enumerate((-0.60, -0.20, 0.20, 0.60)):
        _plane_box(roof, f"mullion_lower_{i}", (1.22, 0.042, 0.040), 2.01, v, 0.026, aluminum)

    # Side mullions flanking the vent zone (outer edges)
    side_mullion_v = BAY_CENTERS[-1] + VENT_HALF + 0.06
    for i, v in enumerate((-side_mullion_v, side_mullion_v)):
        _plane_box(roof, f"mullion_side_{i}", (1.40, 0.042, 0.040), 0.71, v, 0.026, aged_steel)

    # Mullion dividers between adjacent bays
    for i in range(N_BAYS - 1):
        v = (BAY_CENTERS[i] + BAY_CENTERS[i + 1]) / 2.0
        _plane_box(roof, f"mullion_divider_{i}", (1.40, 0.042, 0.040), 0.71, v, 0.026, aged_steel)

    # Per-bay roof frame elements (curb, seals, hinge hardware)
    for bay_i in range(N_BAYS):
        yc = BAY_CENTERS[bay_i]

        # Curb framing the operable vent (sill at bottom, jambs on sides)
        _plane_box(roof, f"vent_curb_sill_{bay_i}", (0.065, 2 * VENT_HALF + 0.12, 0.048),
                   VENT_U1, yc, 0.024, aluminum)
        for j, dv in enumerate((-VENT_HALF - 0.04, VENT_HALF + 0.04)):
            _plane_box(roof, f"vent_curb_jamb_{bay_i}_{j}",
                       (VENT_U1 - VENT_U0, 0.045, 0.046),
                       (VENT_U0 + VENT_U1) / 2.0, yc + dv, 0.024, aluminum)

        # EPDM weather seals (ridge edge + sill edge)
        _plane_box(roof, f"ridge_weather_seal_{bay_i}", (0.024, 2 * VENT_HALF, 0.014),
                   0.045, yc, 0.044, rubber)
        _plane_box(roof, f"sill_weather_seal_{bay_i}", (0.024, 2 * VENT_HALF, 0.014),
                   VENT_U1 - 0.02, yc, 0.046, rubber)

        # Fixed hinge leaves + knuckles along the ridge for this bay
        for k, dv in enumerate((-0.18, 0.0, 0.18)):
            _plane_box(roof, f"fixed_hinge_leaf_{bay_i}_{k}", (0.060, 0.080, 0.006),
                       -0.05, yc + dv, 0.046, galvanized)
            _plane_cyl_y(roof, f"fixed_hinge_knuckle_{bay_i}_{k}", 0.012, 0.080,
                         0.020, yc + dv, 0.064, galvanized)

        # Hinge pin ends at bay edges
        _plane_cyl_y(roof, f"hinge_pin_{bay_i}_a", 0.007, 0.040,
                     0.020, yc - VENT_HALF + 0.02, 0.064, bolt_dark)
        _plane_cyl_y(roof, f"hinge_pin_{bay_i}_b", 0.007, 0.040,
                     0.020, yc + VENT_HALF - 0.02, 0.064, bolt_dark)

    # Fixed glazing panes (lower field, 5 columns across the wider bay)
    for ci, v in enumerate((-0.80, -0.40, 0.0, 0.40, 0.80)):
        for ri, u in enumerate((1.72, 2.31)):
            _plane_box(roof, f"glass_lower_{ci}_{ri}", (0.55, 0.36, 0.006),
                       u, v, 0.045, glass)

    # Side glass strips (between vent zone edge and rake rail)
    for i, v in enumerate((-side_mullion_v - 0.08, side_mullion_v + 0.08)):
        _plane_box(roof, f"glass_side_{i}", (1.18, 0.14, 0.006), 0.71, v, 0.045, glass)

    # Glazing-clip screws on structural bars
    sx = 0
    for u, v in ((1.42, -0.70), (1.42, 0.70), (2.02, -0.70), (2.02, 0.70), (2.01, 0.0)):
        roof.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=_plane_xyz(u, v, 0.046), rpy=(0.0, PITCH, 0.0)),
            material=bolt_dark,
            name=f"roof_screw_{sx}",
        )
        sx += 1

    # Tensioned support wire + hanging tomato twine
    _plane_box(roof, "support_wire", (0.012, 2 * HALF_W + 0.10, 0.012), 1.95, 0.0, 0.0, wire_mat)
    for i, v in enumerate((-0.80, -0.48, -0.16, 0.16, 0.48, 0.80)):
        top = _plane_xyz(1.95, v, 0.012)
        length = 0.34 + 0.04 * i
        _vert_twine(roof, f"twine_{i}", top, length, 0.004, jute)
        _add_box(roof, f"twine_knot_{i}", (0.020, 0.014, 0.024),
                 (top[0], top[1], top[2] - length), jute)

    # ================================================================= vent bays (loop)
    vent_parts = []
    stay_parts = []
    latch_parts = []
    hinge_joints = []
    latch_joints = []
    stay_joints = []

    for bay_i in range(N_BAYS):
        yc = BAY_CENTERS[bay_i]

        vent = _build_vent_sash(model, bay_i, mats)
        stay = _build_stay_arm(model, bay_i, mats)
        latch = _build_latch(model, bay_i, mats)

        # Primary hinge: vent sash swings UP about the ridge hinge line (world -Y)
        hinge_origin = _plane_xyz(0.0, yc, 0.064)
        vh = model.articulation(
            f"roof_to_vent_sash_{bay_i}",
            ArticulationType.REVOLUTE,
            parent=roof,
            child=vent,
            origin=Origin(xyz=hinge_origin, rpy=(0.0, PITCH, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=0.4, lower=0.0, upper=1.05),
        )
        vh.meta["role"] = f"bay {bay_i} top-hinged roof vent opening"

        # Latch swings on the sash bottom rail
        lj = model.articulation(
            f"sash_to_latch_{bay_i}",
            ArticulationType.REVOLUTE,
            parent=vent,
            child=latch,
            origin=Origin(xyz=(SASH_LEN - 0.06, 0.0, -0.030)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.55, upper=0.55),
        )

        # Folding stay pivots from the sash mount tab
        sj = model.articulation(
            f"sash_to_stay_arm_{bay_i}",
            ArticulationType.REVOLUTE,
            parent=vent,
            child=stay,
            origin=Origin(xyz=(SASH_LEN - 0.01, -0.16, -0.090)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=-0.10, upper=1.20),
        )

        vent_parts.append(vent)
        stay_parts.append(stay)
        latch_parts.append(latch)
        hinge_joints.append(vh)
        latch_joints.append(lj)
        stay_joints.append(sj)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_frame")

    vent_parts = [object_model.get_part(f"vent_sash_{i}") for i in range(N_BAYS)]
    stay_parts = [object_model.get_part(f"stay_arm_{i}") for i in range(N_BAYS)]
    latch_parts = [object_model.get_part(f"latch_{i}") for i in range(N_BAYS)]
    hinge_joints = [object_model.get_articulation(f"roof_to_vent_sash_{i}") for i in range(N_BAYS)]
    latch_joints = [object_model.get_articulation(f"sash_to_latch_{i}") for i in range(N_BAYS)]
    stay_joints = [object_model.get_articulation(f"sash_to_stay_arm_{i}") for i in range(N_BAYS)]

    # --- Allowances for intentional seating / pivot interpenetrations ---
    for bay_i in range(N_BAYS):
        vent_name = f"vent_sash_{bay_i}"
        stay_name = f"stay_arm_{bay_i}"
        latch_name = f"latch_{bay_i}"

        # Closed sash beds down onto the fixed ridge rail at the hinge line
        for stile in ("sash_stile_0", "sash_stile_1", "sash_top_rail"):
            ctx.allow_overlap(
                vent_name, "roof_frame",
                elem_a=stile, elem_b="ridge_rail",
                reason=f"Bay {bay_i} closed sash beds onto the ridge rail at the hinge line.",
            )

        # Stay pivot captured on sash mount tab
        for stay_elem, sash_elem in (
            ("stay_pivot_plate", "stay_mount_tab"),
            ("stay_top_pin", "stay_mount_tab"),
            ("stay_upper_arm", "stay_mount_tab"),
        ):
            ctx.allow_overlap(
                stay_name, vent_name,
                elem_a=stay_elem, elem_b=sash_elem,
                reason=f"Bay {bay_i} stay hardware is captured at the sash mount lug.",
            )

        # Latch hardware captured on sash bottom rail
        for latch_elem in ("latch_pivot_pin", "latch_back_plate"):
            for member in ("sash_bottom_rail", "sash_drip_lip"):
                ctx.allow_overlap(
                    latch_name, vent_name,
                    elem_a=latch_elem, elem_b=member,
                    reason=f"Bay {bay_i} latch hardware is captured at the sash bottom rail.",
                )

        # Fixed hinge knuckles interleave with sash frame at hinge line
        for k in range(3):
            members = ["sash_top_rail"]
            if k == 0:
                members.append("sash_stile_0")
            elif k == 2:
                members.append("sash_stile_1")
            # Sash hinge leaves sit between fixed knuckles (interleaved)
            for leaf_j in range(2):
                members.append(f"sash_hinge_leaf_{leaf_j}")
            for member in members:
                ctx.allow_overlap(
                    "roof_frame", vent_name,
                    elem_a=f"fixed_hinge_knuckle_{bay_i}_{k}",
                    elem_b=member,
                    reason=f"Bay {bay_i} fixed ridge knuckle interleaves with sash frame at the hinge line.",
                )

        # Hinge pin ends run through sash frame at hinge line
        for pin_suffix, stile_name in (("a", "sash_stile_0"), ("b", "sash_stile_1")):
            for member in ("sash_top_rail", stile_name):
                ctx.allow_overlap(
                    "roof_frame", vent_name,
                    elem_a=f"hinge_pin_{bay_i}_{pin_suffix}",
                    elem_b=member,
                    reason=f"Bay {bay_i} hinge pin end runs through sash frame at the hinge line.",
                )

        # EPDM weather seal compression by closed sash
        for seal_name, members in (
            (f"ridge_weather_seal_{bay_i}",
             ("sash_top_rail", "sash_stile_0", "sash_stile_1", "sash_gasket_0", "sash_gasket_1")),
            (f"sill_weather_seal_{bay_i}",
             ("sash_bottom_rail", "sash_stile_0", "sash_stile_1",
              "sash_gasket_0", "sash_gasket_1", "sash_gasket_bottom")),
        ):
            for member in members:
                ctx.allow_overlap(
                    "roof_frame", vent_name,
                    elem_a=seal_name, elem_b=member,
                    reason=f"Bay {bay_i} closed sash compresses the EPDM curb weather seal.",
                )

    # --- Classification and structure checks ---
    ctx.check(
        "classified as greenhouse vent roof",
        object_model.name == "agricultural_greenhouse_vent_roof"
        and object_model.meta.get("class") == "Agricultural/Greenhouse vent roof",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    ctx.check(
        "has 3 vent sash bays with stays and latches",
        all(p is not None for p in vent_parts + stay_parts + latch_parts)
        and all(j is not None for j in hinge_joints + latch_joints + stay_joints),
        details="Expected 3 vent sashes, 3 stay arms, 3 latches, and 9 joints total.",
    )

    # Each bay has a non-fixed revolute primary joint
    for bay_i in range(N_BAYS):
        hj = hinge_joints[bay_i]
        ctx.check(
            f"bay {bay_i} hinge is revolute",
            hj is not None and hj.articulation_type == ArticulationType.REVOLUTE,
            details=f"bay {bay_i} primary vent hinge must be revolute",
        )

    # Primary mechanism: opening each vent lifts the sash up and clear of the roof plane
    for bay_i in range(N_BAYS):
        vent = vent_parts[bay_i]
        hinge = hinge_joints[bay_i]
        closed_aabb = ctx.part_world_aabb(vent)
        with ctx.pose({hinge: 0.95}):
            open_aabb = ctx.part_world_aabb(vent)
        ctx.check(
            f"bay {bay_i} vent opens upward on the ridge hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.30,
            details=f"closed_top_z={closed_aabb[1][2]:.3f}, open_top_z={open_aabb[1][2]:.3f}",
        )

    # Closed sashes sit in the roof plane (not floating high above)
    for bay_i in range(N_BAYS):
        vent = vent_parts[bay_i]
        closed_aabb = ctx.part_world_aabb(vent)
        ctx.check(
            f"bay {bay_i} closed sash sits in the roof plane",
            closed_aabb is not None and closed_aabb[1][2] < RIDGE_HEIGHT + 0.20,
            details=f"closed_top_z={closed_aabb[1][2]:.3f}, ridge={RIDGE_HEIGHT}",
        )

    # Moving hinge knuckle overlaps ridge line (bay 1 = center bay, representative)
    ctx.expect_overlap(
        vent_parts[1], roof,
        axes="y",
        elem_a="sash_hinge_knuckle_0",
        elem_b="ridge_rail",
        min_overlap=0.05,
        name="bay 1 moving hinge knuckle lies along the ridge hinge line",
    )

    # Latch stays mounted on sash through swing (bay 0 representative)
    with ctx.pose({latch_joints[0]: 0.45}):
        ctx.expect_origin_distance(
            latch_parts[0], vent_parts[0],
            axes="xyz", max_dist=1.6,
            name="bay 0 latch remains mounted on the sash when swung",
        )

    # Folding stay pivots from the sash tab (bay 0 representative)
    with ctx.pose({stay_joints[0]: 0.10}):
        ctx.expect_origin_distance(
            stay_parts[0], vent_parts[0],
            axes="xyz", max_dist=1.3,
            name="bay 0 folding stay is mounted on the sash tab",
        )

    return ctx.report()


object_model = build_object_model()
