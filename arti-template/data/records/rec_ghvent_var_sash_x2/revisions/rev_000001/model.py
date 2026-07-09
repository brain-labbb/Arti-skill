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
VENT_HALF = 0.46                 # vent half width across (total opening = 0.92)
N_BAYS = 2
BAY_HALF = VENT_HALF / N_BAYS   # 0.23 per bay half-width
SASH_LEN = VENT_U1 - VENT_U0    # 1.21 along-slope sash length


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
    """Add a Y-axis cylinder (runs across-slope) at a plane point."""
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


def _bay_v_center(bay_idx: int) -> float:
    """Return the across-slope center of the given bay."""
    return (2 * bay_idx - (N_BAYS - 1)) * BAY_HALF


def _build_vent_sash(model, bay_idx, *, aluminum, aged_steel, galvanized, glass,
                     rubber, black_steel, bolt_dark):
    """Build one vent bay: sash + latch + stay_arm + articulations.

    The sash local frame has +X running down-slope from the ridge hinge line
    and Y spanning +-BAY_HALF across the bay.
    """
    v_center = _bay_v_center(bay_idx)
    prefix = f"vent_sash_{bay_idx}"

    # --- vent sash ---
    vent = model.part(prefix)
    midx = SASH_LEN / 2.0

    _add_box(vent, "vent_glass", (SASH_LEN - 0.10, 2 * BAY_HALF - 0.08, 0.007),
             (midx, 0.0, -0.002), glass)
    _add_box(vent, "sash_top_rail", (0.060, 2 * BAY_HALF, 0.038),
             (0.030, 0.0, 0.0), aluminum)
    _add_box(vent, "sash_bottom_rail", (0.080, 2 * BAY_HALF, 0.038),
             (SASH_LEN - 0.030, 0.0, 0.0), aluminum)
    _add_box(vent, "sash_stile_0", (SASH_LEN, 0.052, 0.038),
             (midx, -BAY_HALF + 0.026, 0.0), aluminum)
    _add_box(vent, "sash_stile_1", (SASH_LEN, 0.052, 0.038),
             (midx, BAY_HALF - 0.026, 0.0), aged_steel)
    _add_box(vent, "sash_glazing_bar", (0.038, 2 * BAY_HALF - 0.08, 0.028),
             (midx, 0.0, 0.010), aluminum)
    _add_box(vent, "sash_drip_lip", (0.030, 2 * BAY_HALF - 0.06, 0.022),
             (SASH_LEN - 0.060, 0.0, -0.024), aluminum)

    # EPDM gaskets around the sash perimeter.
    _add_box(vent, "sash_gasket_0", (SASH_LEN - 0.10, 0.016, 0.012),
             (midx, -BAY_HALF + 0.012, -0.018), rubber)
    _add_box(vent, "sash_gasket_1", (SASH_LEN - 0.10, 0.016, 0.012),
             (midx, BAY_HALF - 0.012, -0.018), rubber)
    _add_box(vent, "sash_gasket_bottom", (0.018, 2 * BAY_HALF - 0.06, 0.012),
             (SASH_LEN - 0.020, 0.0, -0.010), rubber)

    # Moving hinge leaves + knuckles (2 per bay, interleaving with fixed ridge knuckles).
    for j, dv in enumerate((-0.08, 0.08)):
        _add_box(vent, f"sash_hinge_leaf_{j}", (0.060, 0.100, 0.006),
                 (0.020, dv, 0.012), galvanized)
        _add_cyl_y(vent, f"sash_hinge_knuckle_{j}", 0.011, 0.100,
                   (0.004, dv, 0.006), galvanized)

    # Stay-arm mount lug projecting below the sash bottom rail.
    stay_v = -BAY_HALF + 0.08
    _add_box(vent, "stay_mount_tab", (0.060, 0.080, 0.110),
             (SASH_LEN - 0.01, stay_v, -0.045), galvanized)

    # Corner gusset plates.
    for j, (x, dv) in enumerate(((0.10, -BAY_HALF + 0.06), (0.10, BAY_HALF - 0.06),
                                  (SASH_LEN - 0.06, -BAY_HALF + 0.06),
                                  (SASH_LEN - 0.06, BAY_HALF - 0.06))):
        _add_box(vent, f"sash_corner_plate_{j}", (0.075, 0.045, 0.006),
                 (x, dv, 0.016), galvanized)

    # Fastener screws along sash rails.
    for j, x in enumerate((0.22, 0.60, 0.98)):
        for k, dv in enumerate((-BAY_HALF + 0.05, BAY_HALF - 0.05)):
            vent.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(xyz=(x, dv, 0.006)),
                material=bolt_dark,
                name=f"sash_screw_{j}_{k}",
            )

    # --- latch ---
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

    # --- stay arm ---
    stay = model.part(f"stay_arm_{bay_idx}")
    _add_cyl_y(stay, "stay_top_pin", 0.010, 0.040, (0.0, 0.0, 0.0), bolt_dark)
    _add_box(stay, "stay_pivot_plate", (0.050, 0.050, 0.010),
             (0.0, 0.0, -0.006), galvanized)
    _add_box(stay, "stay_upper_arm", (0.020, 0.020, 0.240),
             (0.0, 0.0, -0.125), black_steel)
    _add_cyl_y(stay, "stay_knuckle", 0.014, 0.026, (0.0, 0.0, -0.245), bolt_dark)
    _add_box(stay, "stay_elbow_jog", (0.140, 0.020, 0.020),
             (0.060, 0.0, -0.245), black_steel)
    _add_box(stay, "stay_lower_arm", (0.020, 0.020, 0.200),
             (0.120, 0.0, -0.345), black_steel)
    _add_box(stay, "stay_end_shoe", (0.052, 0.030, 0.018),
             (0.120, 0.0, -0.445), galvanized)
    for j, z in enumerate((-0.300, -0.360, -0.420)):
        stay.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.120, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"stay_adjust_hole_{j}",
        )

    # --- articulations ---
    roof = model.get_part("roof_frame")

    # Primary: vent sash swings UP about the ridge hinge line at bay center.
    hinge_origin = _plane_xyz(0.0, v_center, 0.064)
    vent_hinge = model.articulation(
        f"roof_to_vent_sash_{bay_idx}",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=vent,
        origin=Origin(xyz=hinge_origin, rpy=(0.0, PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.4, lower=0.0, upper=1.05),
    )
    vent_hinge.meta["role"] = f"bay {bay_idx} top-hinged roof vent opening"

    # Latch swings on the sash bottom rail.
    model.articulation(
        f"sash_{bay_idx}_to_latch_{bay_idx}",
        ArticulationType.REVOLUTE,
        parent=vent,
        child=latch,
        origin=Origin(xyz=(SASH_LEN - 0.06, 0.0, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.55, upper=0.55),
    )

    # Folding stay pivots from the sash mount tab.
    model.articulation(
        f"sash_{bay_idx}_to_stay_arm_{bay_idx}",
        ArticulationType.REVOLUTE,
        parent=vent,
        child=stay,
        origin=Origin(xyz=(SASH_LEN - 0.01, stay_v, -0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=-0.10, upper=1.20),
    )

    return vent, latch, stay, vent_hinge


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_greenhouse_vent_roof",
        meta={
            "class": "Agricultural/Greenhouse vent roof",
            "description": (
                "A pitched greenhouse roof bay with two side-by-side vent sashes: "
                "glazed pane grid in weathered aluminum and aged-steel glazing bars, "
                "two top-hinged vent sashes each propped open on a folding scissor stay, "
                "per-bay ridge hinge lines, latch handles, hanging tomato twine and a "
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
    # Side mullions flanking the vent opening.
    for i, v in enumerate((-0.60, 0.60)):
        _plane_box(roof, f"mullion_side_{i}", (1.40, 0.042, 0.040), 0.71, v, 0.026, aged_steel)

    # Center divider mullion between the two vent bays.
    _plane_box(roof, "vent_center_mullion", (VENT_U1 - VENT_U0, 0.045, 0.046),
               (VENT_U0 + VENT_U1) / 2.0, 0.0, 0.024, aluminum)

    # Curb sill spanning both bays.
    _plane_box(roof, "vent_curb_sill", (0.065, 2 * VENT_HALF + 0.12, 0.048),
               VENT_U1, 0.0, 0.024, aluminum)

    # Per-bay curb jambs, fixed hinges, hinge pins, and weather seals.
    for i in range(N_BAYS):
        v_center = _bay_v_center(i)
        low_edge = v_center - BAY_HALF    # bay opening low-Y edge
        high_edge = v_center + BAY_HALF   # bay opening high-Y edge

        # Curb jamb at the outer edge only; the center mullion separates the bays.
        outer_jv = low_edge - 0.02 if i == 0 else high_edge + 0.02
        _plane_box(roof, f"vent_curb_jamb_{i}",
                   (VENT_U1 - VENT_U0, 0.045, 0.046),
                   (VENT_U0 + VENT_U1) / 2.0, outer_jv, 0.024, aluminum)

        # Fixed hinge leaves + knuckles per bay (2 per bay, flanking bay center).
        for j, dv in enumerate((-0.12, 0.12)):
            hv = v_center + dv
            _plane_box(roof, f"fixed_hinge_leaf_{i}_{j}",
                       (0.060, 0.100, 0.006), -0.05, hv, 0.046, galvanized)
            _plane_cyl_y(roof, f"fixed_hinge_knuckle_{i}_{j}",
                         0.012, 0.100, 0.020, hv, 0.064, galvanized)

        # Hinge pin at outer edge of bay.
        pin_v = low_edge - 0.02 if i == 0 else high_edge + 0.02
        _plane_cyl_y(roof, f"hinge_pin_{i}", 0.007, 0.040,
                     0.020, pin_v, 0.064, bolt_dark)

        # Per-bay EPDM weather seals.
        _plane_box(roof, f"ridge_weather_seal_{i}",
                   (0.024, 2 * BAY_HALF, 0.014), 0.045, v_center, 0.044, rubber)
        _plane_box(roof, f"sill_weather_seal_{i}",
                   (0.024, 2 * BAY_HALF, 0.014), VENT_U1 - 0.02, v_center, 0.046, rubber)

    # Fixed glazing panes around the vent (lower field + side strips).
    for ci, v in enumerate((-0.46, 0.0, 0.46)):
        for ri, u in enumerate((1.72, 2.31)):
            _plane_box(roof, f"glass_lower_{ci}_{ri}",
                       (0.55, 0.44, 0.006), u, v, 0.045, glass)
    for i, v in enumerate((-0.61, 0.61)):
        _plane_box(roof, f"glass_side_{i}", (1.18, 0.18, 0.006), 0.71, v, 0.045, glass)

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

    # Tensioned support wire spanning the bay + hanging tomato twine.
    _plane_box(roof, "support_wire", (0.012, 2 * HALF_W + 0.10, 0.012), 1.95, 0.0, 0.0, wire)
    for i, v in enumerate((-0.38, -0.05, 0.22, 0.50)):
        top = _plane_xyz(1.95, v, 0.012)
        length = 0.34 + 0.05 * i
        _vert_twine(roof, f"twine_{i}", top, length, 0.004, jute)
        _add_box(roof, f"twine_knot_{i}", (0.020, 0.014, 0.024),
                 (top[0], top[1], top[2] - length), jute)

    # ----------------------------------------------------------------- vent bays
    for i in range(N_BAYS):
        _build_vent_sash(
            model, i,
            aluminum=aluminum, aged_steel=aged_steel, galvanized=galvanized,
            glass=glass, rubber=rubber, black_steel=black_steel, bolt_dark=bolt_dark,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_frame")

    # Per-bay parts and joints.
    vents = [object_model.get_part(f"vent_sash_{i}") for i in range(N_BAYS)]
    latches = [object_model.get_part(f"latch_{i}") for i in range(N_BAYS)]
    stays = [object_model.get_part(f"stay_arm_{i}") for i in range(N_BAYS)]
    hinges = [object_model.get_articulation(f"roof_to_vent_sash_{i}") for i in range(N_BAYS)]
    latch_joints = [object_model.get_articulation(f"sash_{i}_to_latch_{i}") for i in range(N_BAYS)]
    stay_joints = [object_model.get_articulation(f"sash_{i}_to_stay_arm_{i}") for i in range(N_BAYS)]

    # ---- Intentional seating / pivot interpenetrations per bay ----
    for i in range(N_BAYS):
        vname = f"vent_sash_{i}"
        lname = f"latch_{i}"
        sname = f"stay_arm_{i}"

        # Closed sash beds down onto the fixed ridge rail at the hinge line.
        for stile in ("sash_stile_0", "sash_stile_1"):
            ctx.allow_overlap(
                vname, "roof_frame", elem_a=stile, elem_b="ridge_rail",
                reason="Closed sash beds down onto the fixed ridge rail at the hinge line.",
            )
        # Inner-edge sash elements seat against the center mullion when closed.
        inner_stile = "sash_stile_1" if i == 0 else "sash_stile_0"
        inner_gasket = "sash_gasket_1" if i == 0 else "sash_gasket_0"
        for elem in (inner_stile, "sash_bottom_rail", "sash_top_rail", inner_gasket):
            ctx.allow_overlap(
                vname, "roof_frame", elem_a=elem, elem_b="vent_center_mullion",
                reason=f"Bay {i} inner-edge {elem} seats against the center divider mullion when closed.",
            )
        ctx.allow_overlap(
            vname, "roof_frame", elem_a="sash_top_rail", elem_b="ridge_rail",
            reason="Closed sash top rail beds down onto the fixed ridge rail at the hinge line.",
        )

        # Stay pivot plate captured on the sash mount tab.
        for stay_elem, sash_elem in (
            ("stay_pivot_plate", "stay_mount_tab"),
            ("stay_top_pin", "stay_mount_tab"),
            ("stay_upper_arm", "stay_mount_tab"),
        ):
            ctx.allow_overlap(
                sname, vname, elem_a=stay_elem, elem_b=sash_elem,
                reason="Stay pivot assembly is captured on the sash mount lug at the revolute pin.",
            )

        # Latch hardware captured at the sash bottom rail.
        for latch_elem in ("latch_pivot_pin", "latch_back_plate"):
            for member in ("sash_bottom_rail", "sash_drip_lip"):
                ctx.allow_overlap(
                    lname, vname, elem_a=latch_elem, elem_b=member,
                    reason="Latch hardware is captured at the sash bottom rail / drip lip it mounts to.",
                )

        # Ridge hinge pin end runs through the sash frame at the hinge line.
        for stile in ("sash_stile_0", "sash_stile_1"):
            ctx.allow_overlap(
                "roof_frame", vname,
                elem_a=f"hinge_pin_{i}", elem_b=stile,
                reason="Ridge hinge pin end runs through the sash frame at the hinge line.",
            )
        ctx.allow_overlap(
            "roof_frame", vname,
            elem_a=f"hinge_pin_{i}", elem_b="sash_top_rail",
            reason="Ridge hinge pin end runs through the sash top rail at the hinge line.",
        )

        # Fixed hinge knuckles interleave with the sash frame and hinge leaves.
        for j in range(2):
            for member in ("sash_top_rail",):
                ctx.allow_overlap(
                    "roof_frame", vname,
                    elem_a=f"fixed_hinge_knuckle_{i}_{j}", elem_b=member,
                    reason="Fixed ridge hinge knuckle interleaves with the sash frame at the hinge line.",
                )
        # All fixed hinge knuckles interleave with all sash hinge leaves and knuckles.
        for jk in range(2):
            for jl in range(2):
                ctx.allow_overlap(
                    "roof_frame", vname,
                    elem_a=f"fixed_hinge_knuckle_{i}_{jk}",
                    elem_b=f"sash_hinge_leaf_{jl}",
                    reason="Fixed ridge hinge knuckle interleaves with the moving sash hinge leaf at the hinge line.",
                )
                ctx.allow_overlap(
                    "roof_frame", vname,
                    elem_a=f"fixed_hinge_knuckle_{i}_{jk}",
                    elem_b=f"sash_hinge_knuckle_{jl}",
                    reason="Fixed and moving hinge knuckles share the same hinge pin axis at the ridge line.",
                )

        # EPDM weather seals bedded by closed sash.
        for seal_base in ("ridge_weather_seal", "sill_weather_seal"):
            seal = f"{seal_base}_{i}"
            members = ["sash_top_rail", "sash_bottom_rail", "sash_stile_0", "sash_stile_1",
                       "sash_gasket_0", "sash_gasket_1"]
            if seal_base == "sill_weather_seal":
                members.append("sash_gasket_bottom")
            for member in members:
                ctx.allow_overlap(
                    "roof_frame", vname, elem_a=seal, elem_b=member,
                    reason="Closed sash frame beds and compresses the fixed EPDM curb weather seal.",
                )

    # ---- Classification and structural checks ----
    ctx.check(
        "classified as greenhouse vent roof",
        object_model.name == "agricultural_greenhouse_vent_roof"
        and object_model.meta.get("class") == "Agricultural/Greenhouse vent roof",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )
    ctx.check(
        "has 2 vent bays with sash, stay, latch subassemblies and joints",
        all(v is not None for v in vents)
        and all(l is not None for l in latches)
        and all(s is not None for s in stays)
        and all(h is not None for h in hinges)
        and all(j is not None for j in latch_joints)
        and all(j is not None for j in stay_joints),
        details="Expected 2 vent sashes, 2 latches, 2 stay arms, and 6 joints total.",
    )

    # ---- Per-bay mechanism checks ----
    for i in range(N_BAYS):
        vent = vents[i]
        latch = latches[i]
        stay = stays[i]
        hinge = hinges[i]
        latch_jt = latch_joints[i]
        stay_jt = stay_joints[i]

        # Moving hinge knuckle lies along the ridge hinge line (Y overlap).
        ctx.expect_overlap(
            vent, roof, axes="y",
            elem_a="sash_hinge_knuckle_0", elem_b="ridge_rail",
            min_overlap=0.06,
            name=f"bay {i} moving hinge knuckle lies along the ridge hinge line",
        )

        # Primary mechanism: opening the vent lifts the sash up and clear of the roof plane.
        closed_aabb = ctx.part_world_aabb(vent)
        with ctx.pose({hinge: 0.95}):
            open_aabb = ctx.part_world_aabb(vent)
        ctx.check(
            f"bay {i} vent opens upward on the ridge hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.30,
            details=f"closed_top_z={closed_aabb[1][2]:.3f}, open_top_z={open_aabb[1][2]:.3f}",
        )

        # Closed sash sits in the roof plane.
        ctx.check(
            f"bay {i} closed sash sits in the roof plane",
            closed_aabb is not None and closed_aabb[1][2] < RIDGE_HEIGHT + 0.20,
            details=f"closed_top_z={closed_aabb[1][2]:.3f}, ridge={RIDGE_HEIGHT}",
        )

        # Latch stays mounted on the sash through its swing.
        with ctx.pose({latch_jt: 0.45}):
            ctx.expect_origin_distance(
                latch, vent, axes="xyz", max_dist=1.6,
                name=f"bay {i} latch remains mounted on the sash when swung",
            )

        # Folding stay pivots from the sash tab.
        with ctx.pose({stay_jt: 0.10}):
            ctx.expect_origin_distance(
                stay, vent, axes="xyz", max_dist=1.3,
                name=f"bay {i} folding stay is mounted on the sash tab",
            )

    # ---- Cross-bay tiling: two sashes occupy distinct Y bands ----
    ctx.check(
        "two vent sashes are tiled across-slope into distinct bays",
        vents[0] is not None and vents[1] is not None,
        details="Both vent_sash_0 and vent_sash_1 must exist.",
    )
    aabb0 = ctx.part_world_aabb(vents[0])
    aabb1 = ctx.part_world_aabb(vents[1])
    if aabb0 is not None and aabb1 is not None:
        # Bay 0 should be on the negative-Y side, bay 1 on the positive-Y side.
        ctx.check(
            "bay 0 is on the negative-Y side and bay 1 is on the positive-Y side",
            aabb0[0][1] < aabb1[1][1] and aabb1[0][1] > aabb0[0][1],
            details=f"bay0_y=[{aabb0[0][1]:.3f},{aabb0[1][1]:.3f}], "
                    f"bay1_y=[{aabb1[0][1]:.3f},{aabb1[1][1]:.3f}]",
        )

    return ctx.report()


object_model = build_object_model()
