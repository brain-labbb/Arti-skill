from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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
VENT_HALF = 0.46                 # vent half width across

# Louvre blade parameters
N_BLADES = 6
BLADE_SPAN = VENT_U1 - VENT_U0  # 1.21
BLADE_PITCH_U = BLADE_SPAN / N_BLADES
BLADE_WIDTH = 2.0 * VENT_HALF - 0.08   # across-slope width with jamb clearance
BLADE_LENGTH = BLADE_PITCH_U - 0.005   # slight gap between adjacent blades
BLADE_THICK = 0.006
BLADE_PIVOT_W = 0.058           # out-of-plane height for the pivot axis


# --------------------------------------------------------------------------- helpers

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


# --------------------------------------------------------------------------- blade builder

def _build_louvre_blade(model, roof, i, aluminum, galvanized, rubber, bolt_dark):
    """Build one louvre blade part with visuals and its revolute joint to the roof frame.

    Each blade is a thin flat panel pivoting on a galvanized axle that runs
    across the slope.  The blade part frame sits at the pivot axis in the
    pitched roof plane; local +X runs down-slope from the pivot.
    """
    blade = model.part(f"louvre_blade_{i}")

    # Thin flat panel extending down-slope from the pivot axis
    _add_box(blade, "panel",
             (BLADE_LENGTH, BLADE_WIDTH, BLADE_THICK),
             (BLADE_LENGTH / 2.0, 0.0, 0.0), aluminum)

    # Pivot axle rod running across the slope at the upper edge
    axle_len = 2.0 * (VENT_HALF + 0.02)
    _add_cyl_y(blade, "axle", 0.006, axle_len, (0.0, 0.0, 0.0), galvanized)

    # End brackets connecting panel to axle at each Y-end
    for s, side in enumerate((-1, 1)):
        _add_box(blade, f"bracket_{s}",
                 (0.035, 0.028, 0.018),
                 (0.018, side * (BLADE_WIDTH / 2.0 - 0.014), 0.0), galvanized)

    # Trailing-edge EPDM seal strip (slightly proud of the panel top surface)
    _add_box(blade, "edge_seal",
             (0.010, BLADE_WIDTH - 0.04, 0.005),
             (BLADE_LENGTH - 0.005, 0.0, BLADE_THICK / 2.0 + 0.001), rubber)

    # Small fastener on each bracket (along Z in the blade local frame)
    for s, side in enumerate((-1, 1)):
        blade.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(0.018, side * (BLADE_WIDTH / 2.0 - 0.014), 0.010)),
            material=bolt_dark,
            name=f"bolt_{s}",
        )

    # Revolute joint: across-slope axis at the blade pivot point
    u_pivot = VENT_U0 + (i + 0.5) * BLADE_PITCH_U
    pivot_xyz = _plane_xyz(u_pivot, 0.0, BLADE_PIVOT_W)
    joint = model.articulation(
        f"frame_to_louvre_blade_{i}",
        ArticulationType.REVOLUTE,
        parent=roof,
        child=blade,
        origin=Origin(xyz=pivot_xyz, rpy=(0.0, PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=0.70),
    )

    return blade, joint


# --------------------------------------------------------------------------- model

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="agricultural_greenhouse_vent_roof",
        meta={
            "class": "Agricultural/Greenhouse vent roof",
            "description": (
                "A pitched greenhouse roof bay: glazed pane grid in weathered aluminum "
                "and aged-steel glazing bars, a six-blade louvre/jalousie bank in the "
                "vent opening with individually pivoting aluminum blades on galvanized "
                "axles in curb-mounted bearings, EPDM weather seals, linkage bar, "
                "hanging tomato twine and a tensioned support wire."
            ),
        },
    )

    aluminum = model.material("weathered_aluminum", rgba=(0.74, 0.75, 0.70, 1.0))
    aged_steel = model.material("aged_glazing_bar", rgba=(0.30, 0.26, 0.20, 1.0))
    galvanized = model.material("galvanized_hardware", rgba=(0.55, 0.57, 0.52, 1.0))
    glass = model.material("clear_sky_glass", rgba=(0.60, 0.80, 0.95, 0.26))
    rubber = model.material("black_epdm_seal", rgba=(0.02, 0.02, 0.02, 1.0))
    bolt_dark = model.material("dark_fasteners", rgba=(0.03, 0.03, 0.03, 1.0))
    jute = model.material("jute_twine", rgba=(0.42, 0.34, 0.22, 1.0))
    wire = model.material("tension_wire", rgba=(0.62, 0.60, 0.55, 1.0))

    # ================================================================= roof_frame
    roof = model.part("roof_frame")

    # --- Perimeter and structural rails ---
    _plane_box(roof, "ridge_rail", (0.085, 2 * HALF_W + 0.06, 0.060), 0.0, 0.0, 0.028, aluminum)
    _plane_box(roof, "eave_rail", (0.060, 2 * HALF_W + 0.06, 0.050), EAVE_U, 0.0, 0.022, aluminum)
    _plane_box(roof, "rake_rail_0", (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, -HALF_W, 0.022, aluminum)
    _plane_box(roof, "rake_rail_1", (EAVE_U + 0.06, 0.055, 0.050), EAVE_U / 2.0, HALF_W, 0.022, aged_steel)

    # --- Transoms and mullions (glazing bar grid) ---
    _plane_box(roof, "transom_mid", (0.050, 2 * HALF_W, 0.044), 1.42, 0.0, 0.024, aged_steel)
    _plane_box(roof, "transom_low", (0.050, 2 * HALF_W, 0.044), 2.02, 0.0, 0.024, aluminum)
    for i, v in enumerate((-0.24, 0.24)):
        _plane_box(roof, f"mullion_lower_{i}", (1.22, 0.042, 0.040), 2.01, v, 0.026, aluminum)
    for i, v in enumerate((-0.60, 0.60)):
        _plane_box(roof, f"mullion_side_{i}", (1.40, 0.042, 0.040), 0.71, v, 0.026, aged_steel)

    # --- Curb framing the louvre opening ---
    _plane_box(roof, "vent_curb_sill", (0.065, 2 * VENT_HALF + 0.12, 0.048), VENT_U1, 0.0, 0.024, aluminum)
    for i, v in enumerate((-VENT_HALF - 0.04, VENT_HALF + 0.04)):
        _plane_box(roof, f"vent_curb_jamb_{i}",
                   (VENT_U1 - VENT_U0, 0.045, 0.046),
                   (VENT_U0 + VENT_U1) / 2.0, v, 0.024, aluminum)

    # Head rail at the top of the louvre opening
    _plane_box(roof, "louvre_head_rail", (0.055, 2 * VENT_HALF + 0.10, 0.040),
               VENT_U0 + 0.025, 0.0, 0.042, aluminum)

    # EPDM weather seal on the sill (blades bed onto this when closed)
    _plane_box(roof, "sill_weather_seal", (0.024, 2 * VENT_HALF, 0.014),
               VENT_U1 - 0.02, 0.0, 0.046, rubber)

    # --- Fixed glazing panes ---
    for ci, v in enumerate((-0.46, 0.0, 0.46)):
        for ri, u in enumerate((1.72, 2.31)):
            _plane_box(roof, f"glass_lower_{ci}_{ri}", (0.55, 0.44, 0.006), u, v, 0.045, glass)
    for i, v in enumerate((-0.61, 0.61)):
        _plane_box(roof, f"glass_side_{i}", (1.18, 0.18, 0.006), 0.71, v, 0.045, glass)

    # --- Pivot bearing blocks on the jambs for each louvre blade ---
    for i in range(N_BLADES):
        u_pivot = VENT_U0 + (i + 0.5) * BLADE_PITCH_U
        for s, v in enumerate((-VENT_HALF - 0.02, VENT_HALF + 0.02)):
            _plane_box(roof, f"blade_bearing_{i}_{s}",
                       (0.032, 0.040, 0.030), u_pivot, v, BLADE_PIVOT_W, galvanized)

    # --- Linkage bar and tabs on one side ---
    _plane_box(roof, "louvre_linkage_bar",
               (BLADE_SPAN + 0.06, 0.018, 0.014),
               (VENT_U0 + VENT_U1) / 2.0, -VENT_HALF - 0.06, BLADE_PIVOT_W - 0.014, galvanized)
    for i in range(N_BLADES):
        u_pivot = VENT_U0 + (i + 0.5) * BLADE_PITCH_U
        _plane_box(roof, f"linkage_tab_{i}",
                   (0.020, 0.024, 0.020), u_pivot, -VENT_HALF - 0.04, BLADE_PIVOT_W, galvanized)

    # --- Glazing-clip screws on the structural bars ---
    sx = 0
    for u, v in ((1.42, -0.40), (1.42, 0.40), (2.02, -0.40), (2.02, 0.40), (2.01, 0.0)):
        roof.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=_plane_xyz(u, v, 0.046), rpy=(0.0, PITCH, 0.0)),
            material=bolt_dark,
            name=f"roof_screw_{sx}",
        )
        sx += 1

    # --- Support wire and hanging tomato twine ---
    _plane_box(roof, "support_wire", (0.012, 2 * HALF_W + 0.10, 0.012), 1.95, 0.0, 0.0, wire)
    for i, v in enumerate((-0.38, -0.05, 0.22, 0.50)):
        top = _plane_xyz(1.95, v, 0.012)
        length = 0.34 + 0.05 * i
        _vert_twine(roof, f"twine_{i}", top, length, 0.004, jute)
        _add_box(roof, f"twine_knot_{i}", (0.020, 0.014, 0.024),
                 (top[0], top[1], top[2] - length), jute)

    # ================================================================= louvre blades
    for i in range(N_BLADES):
        _build_louvre_blade(model, roof, i, aluminum, galvanized, rubber, bolt_dark)

    return model


# --------------------------------------------------------------------------- tests

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_frame")

    # ---- classification ----
    ctx.check(
        "classified as greenhouse vent roof",
        object_model.name == "agricultural_greenhouse_vent_roof"
        and object_model.meta.get("class") == "Agricultural/Greenhouse vent roof",
        details=f"name={object_model.name}, meta={object_model.meta}",
    )

    # ---- louvre blade parts and revolute joints (TARGET assertion) ----
    blade_parts = [object_model.get_part(f"louvre_blade_{i}") for i in range(N_BLADES)]
    blade_joints = [object_model.get_articulation(f"frame_to_louvre_blade_{i}") for i in range(N_BLADES)]

    ctx.check(
        "all 6 louvre_blade_{i} parts and frame_to_louvre_blade_{i} revolute joints exist",
        all(p is not None for p in blade_parts)
        and all(j is not None for j in blade_joints),
        details=(
            f"blades_ok={[p is not None for p in blade_parts]}, "
            f"joints_ok={[j is not None for j in blade_joints]}"
        ),
    )

    # ---- roof_frame preserved elements (KEEP assertion) ----
    kept_names = (
        "ridge_rail", "eave_rail", "rake_rail_0", "rake_rail_1",
        "vent_curb_sill", "vent_curb_jamb_0", "vent_curb_jamb_1",
        "glass_lower_0_0", "glass_side_0",
    )
    ctx.check(
        "roof_frame preserves perimeter rails, curb, and glass grid",
        roof is not None
        and all(
            any(v.name == n for v in roof.visuals)
            for n in kept_names
        ),
        details=f"checked visual names: {kept_names}",
    )

    # ---- dropped parts must not exist ----
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "vent_sash, latch_handle, and stay_arm are removed",
        "vent_sash" not in part_names
        and "latch_handle" not in part_names
        and "stay_arm" not in part_names,
        details="The single-sash mechanism parts are replaced by the louvre blade bank.",
    )

    # ---- overlap allowances: blade axles sit in jamb bearing blocks ----
    for i in range(N_BLADES):
        for s in (0, 1):
            ctx.allow_overlap(
                f"louvre_blade_{i}",
                "roof_frame",
                elem_a="axle",
                elem_b=f"blade_bearing_{i}_{s}",
                reason="Blade pivot axle is captured in the jamb bearing block it rotates in.",
            )

    # ---- linkage tabs near blade axles (visual coupling) ----
    for i in range(N_BLADES):
        ctx.allow_overlap(
            f"louvre_blade_{i}",
            "roof_frame",
            elem_a="axle",
            elem_b=f"linkage_tab_{i}",
            reason="Linkage tab visually couples to the blade axle end for simultaneous operation.",
        )

    # ---- primary mechanism: opening blades tilts them outward ----
    # Test blade 0 (representative): positive q lifts the trailing edge up.
    blade0 = blade_parts[0]
    joint0 = blade_joints[0]

    closed_z = ctx.part_world_aabb(blade0)
    with ctx.pose({joint0: 0.65}):
        open_z = ctx.part_world_aabb(blade0)

    ctx.check(
        "louvre_blade_0 opens by tilting outward from the roof plane",
        closed_z is not None
        and open_z is not None
        and open_z[1][2] > closed_z[1][2] + 0.03,
        details=f"closed_top_z={closed_z[1][2]:.4f}, open_top_z={open_z[1][2]:.4f}",
    )

    # All blades remain mounted within the roof frame opening (origin-distance
    # from roof_frame origin at the ridge to blade pivots down the slope).
    for i, bp in enumerate(blade_parts):
        ctx.expect_origin_distance(
            bp, roof,
            axes="xyz",
            max_dist=3.0,
            name=f"louvre_blade_{i} stays mounted within the roof frame",
        )

    # Blade 3 (mid-bank) also opens correctly
    joint3 = blade_joints[3]
    blade3 = blade_parts[3]
    closed3 = ctx.part_world_aabb(blade3)
    with ctx.pose({joint3: 0.65}):
        open3 = ctx.part_world_aabb(blade3)
    ctx.check(
        "louvre_blade_3 opens by tilting outward (mid-bank verification)",
        closed3 is not None
        and open3 is not None
        and open3[1][2] > closed3[1][2] + 0.03,
        details=f"closed_top_z={closed3[1][2]:.4f}, open_top_z={open3[1][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
