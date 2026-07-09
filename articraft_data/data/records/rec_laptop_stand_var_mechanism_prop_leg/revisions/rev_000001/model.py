from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
SILVER_RGBA = (0.80, 0.81, 0.79, 1.0)
HARDWARE_RGBA = (0.32, 0.33, 0.34, 1.0)
RUBBER_RGBA = (0.03, 0.03, 0.03, 1.0)

# ---------------------------------------------------------------------------
# Dimensions (metres)
# ---------------------------------------------------------------------------
TRAY_LEN = 0.300          # front-to-back
TRAY_WID = 0.230          # side-to-side
TRAY_THK = 0.005          # plate stock thickness
TILT = 0.45               # rad (~26°) tray incline, rear high / front low

BAR_WID = 0.030           # ventilation slot-bar width
BAR_Y = (-0.080, -0.040, 0.000, 0.040, 0.080)  # 5 parallel bars

LUG_Y = 0.105             # hinge lug Y offset from centre
LUG_DX, LUG_DY, LUG_DZ = 0.016, 0.016, 0.020  # hinge lug box dims

LEG_LEN = 0.132           # prop-leg side-bar length (hinge to foot)
LEG_SPAN = LUG_Y * 2.0    # 0.210 – distance between side-bar centres
LEG_BAR_W = 0.024
LEG_BAR_T = 0.005

# Derived
H = TRAY_LEN * math.sin(TILT)  # hinge height above ground plane


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _tilt(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Tray-local (origin at rear hinge, −X = front) → world coords."""
    ca, sa = math.cos(TILT), math.sin(TILT)
    return (x * ca - z * sa, y, x * sa + z * ca + H)


def _tilt_rpy() -> tuple[float, float, float]:
    """RPY that tilts a visual to align with the inclined tray surface."""
    return (0.0, -TILT, 0.0)


def _lightening_slot_bar(
    *,
    length: float,
    width: float,
    thickness: float,
    cuts: tuple[tuple[float, float, float, bool], ...],
    name: str,
) -> object:
    """Flat rounded-end bar stock with a row of through cuts."""
    body = cq.Workplane("XY").slot2D(length - width, width).extrude(thickness)
    body = body.translate((0.0, 0.0, -thickness / 2.0))
    for center_x, cut_len, cut_wid, rounded in cuts:
        wp = body.faces(">Z").workplane(centerOption="CenterOfMass").center(center_x, 0.0)
        if rounded:
            body = wp.slot2D(max(cut_len - cut_wid, 0.001), cut_wid).cutThruAll()
        else:
            body = wp.rect(cut_len, cut_wid).cutThruAll()
    try:
        body = body.edges("|Z").chamfer(0.0010).edges(">Z").chamfer(0.0006)
    except Exception:
        pass
    return mesh_from_cadquery(body, name, tolerance=0.0006, angular_tolerance=0.08)


def _rubber_pad_mesh(length: float, width: float, thickness: float, name: str) -> object:
    pad = cq.Workplane("XY").slot2D(max(length - width, 0.001), width).extrude(thickness)
    pad = pad.translate((0.0, 0.0, -thickness / 2.0))
    return mesh_from_cadquery(pad, name, tolerance=0.0005, angular_tolerance=0.08)


def _hook_bracket_mesh(width: float, name: str) -> object:
    """Upturned front hook: L-shaped lip at the tray's front edge."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.030, 0.0)
        .lineTo(-0.036, 0.0)
        .lineTo(-0.036, 0.030)
        .lineTo(-0.030, 0.030)
        .lineTo(-0.030, 0.006)
        .lineTo(0.030, 0.006)
        .close()
        .extrude(width)
    )
    profile = profile.translate((0.0, width / 2.0, 0.0))
    try:
        profile = profile.edges("|Y").fillet(0.0022)
    except Exception:
        pass
    return mesh_from_cadquery(profile, name, tolerance=0.0005, angular_tolerance=0.08)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kickstand_laptop_stand")

    silver = model.material("brushed_silver_aluminum", rgba=SILVER_RGBA)
    hardware = model.material("satin_pivot_hardware", rgba=HARDWARE_RGBA)
    rubber = model.material("matte_black_rubber", rgba=RUBBER_RGBA)

    # =======================================================================
    # SUPPORT TRAY  (root)
    # =======================================================================
    tray = model.part("support_tray")

    # -- Five parallel slotted bars forming the ventilated tray surface ------
    bar_cuts: tuple[tuple[float, float, float, bool], ...] = (
        (-0.100, 0.034, 0.013, True),
        (-0.033, 0.034, 0.013, True),
        (0.033, 0.034, 0.013, True),
        (0.100, 0.034, 0.013, True),
    )
    bar_mesh = _lightening_slot_bar(
        length=TRAY_LEN, width=BAR_WID, thickness=TRAY_THK,
        cuts=bar_cuts, name="tray_slot_bar",
    )
    for i, y_pos in enumerate(BAR_Y):
        tray.visual(
            bar_mesh,
            origin=Origin(xyz=_tilt(-TRAY_LEN / 2.0, y_pos, 0.0), rpy=_tilt_rpy()),
            material=silver,
            name=f"slot_bar_{i}",
        )

    # -- Cross-ties on the underside binding bars into one tray --------------
    _tie_span = abs(BAR_Y[-1] - BAR_Y[0]) + BAR_WID
    _tie_thk = TRAY_THK + 0.006  # overlaps into bar stock for connectivity
    for i, x_frac in enumerate((0.30, 0.70)):
        x_local = -TRAY_LEN * x_frac
        tray.visual(
            Box((0.014, _tie_span, _tie_thk)),
            origin=Origin(
                xyz=_tilt(x_local, 0.0, -_tie_thk / 2.0),
                rpy=_tilt_rpy(),
            ),
            material=silver,
            name=f"cross_tie_{i}",
        )

    # -- Hinge lugs at rear underside (carry the prop-leg pivot) -------------
    _lug_dx = 0.028  # long in X to embed into the bar end
    _lug_dz = 0.022  # taller so lug top penetrates into bar stock
    _lug_dy = 0.044  # wide enough to bridge into the outer slot bars
    for i, y_sign in enumerate((-1, 1)):
        tray.visual(
            Box((_lug_dx, _lug_dy, _lug_dz)),
            origin=Origin(
                xyz=_tilt(-0.010, y_sign * LUG_Y, -_lug_dz / 2.0 + 0.002),
                rpy=_tilt_rpy(),
            ),
            material=hardware,
            name=f"hinge_lug_{i}",
        )

    # -- Rubber feet at front underside (ground contact when front rests) ----
    for i, y_pos in enumerate((BAR_Y[0], BAR_Y[-1])):
        tray.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(
                xyz=_tilt(-TRAY_LEN + 0.025, y_pos, -TRAY_THK / 2.0 + 0.001),
                rpy=_tilt_rpy(),
            ),
            material=rubber,
            name=f"rail_foot_{i}",
        )

    # -- Front laptop-retaining hook (hook_bracket + rubber_saddle) ----------
    tray.visual(
        _hook_bracket_mesh(0.034, "hook_bracket_mesh"),
        origin=Origin(
            xyz=_tilt(-TRAY_LEN, 0.0, TRAY_THK / 2.0),
            rpy=_tilt_rpy(),
        ),
        material=silver,
        name="hook_bracket",
    )
    tray.visual(
        Box((0.042, 0.026, 0.008)),
        origin=Origin(
            xyz=_tilt(-TRAY_LEN - 0.012, 0.0, TRAY_THK / 2.0 + 0.007),
            rpy=_tilt_rpy(),
        ),
        material=rubber,
        name="rubber_saddle",
    )

    tray.inertial = Inertial.from_geometry(
        Box((TRAY_LEN, TRAY_WID, TRAY_THK + 0.010)),
        mass=0.28,
        origin=Origin(xyz=_tilt(-TRAY_LEN / 2.0, 0.0, 0.0)),
    )

    # =======================================================================
    # PROP LEG  (child – one folding rear kickstand)
    # =======================================================================
    leg = model.part("prop_leg")

    # -- Side bars with lightening slots ------------------------------------
    leg_bar_cuts: tuple[tuple[float, float, float, bool], ...] = (
        (0.0, LEG_LEN * 0.55, 0.008, True),
    )
    for i, y_sign in enumerate((-1, 1)):
        bar_m = _lightening_slot_bar(
            length=LEG_LEN, width=LEG_BAR_W, thickness=LEG_BAR_T,
            cuts=leg_bar_cuts, name=f"leg_bar_slots_{i}",
        )
        # Mesh long axis = X → rotate to leg-frame −Z via Ry(−π/2)
        leg.visual(
            bar_m,
            origin=Origin(
                xyz=(0.0, y_sign * LEG_SPAN / 2.0, -LEG_LEN / 2.0),
                rpy=(0.0, -math.pi / 2.0, 0.0),
            ),
            material=silver,
            name=f"leg_bar_{i}",
        )

    # -- Cross bar at foot of U-frame ---------------------------------------
    leg.visual(
        Box((LEG_BAR_T + 0.006, LEG_SPAN, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -LEG_LEN + 0.015)),
        material=silver,
        name="cross_bar",
    )

    # -- Hinge bosses at top of side bars (captured-knuckle convention) -----
    for i, y_sign in enumerate((-1, 1)):
        leg.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(
                xyz=(0.0, y_sign * LEG_SPAN / 2.0, -0.006),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"hinge_boss_{i}",
        )

    # -- Captured hinge pin spanning both lugs -----------------------------
    leg.visual(
        Cylinder(radius=0.003, length=LEG_SPAN + 0.030),
        origin=Origin(
            xyz=(0.0, 0.0, -0.006),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="hinge_pin",
    )

    # -- Rubber ground pads on cross bar -----------------------------------
    for i, y_pos in enumerate((-0.055, 0.055)):
        leg.visual(
            Box((0.030, 0.020, 0.004)),
            origin=Origin(xyz=(0.0, y_pos, -LEG_LEN - 0.001)),
            material=rubber,
            name=f"ground_pad_{i}",
        )

    leg.inertial = Inertial.from_geometry(
        Box((LEG_BAR_T + 0.010, LEG_SPAN, LEG_LEN)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, -LEG_LEN / 2.0)),
    )

    # =======================================================================
    # ARTICULATION: single revolute kickstand hinge
    # =======================================================================
    # Joint origin at the pivot-axis centre (middle of the two hinge lugs).
    pivot_z_local = -_lug_dz / 2.0 + 0.002  # centre of the lug box
    model.articulation(
        "tray_to_leg",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=leg,
        origin=Origin(
            xyz=_tilt(-0.010, 0.0, pivot_z_local),
            rpy=(0.0, -0.15, 0.0),  # slight rearward lean at q=0 (deployed)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=1.5, lower=0.0, upper=1.40,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray = object_model.get_part("support_tray")
    leg = object_model.get_part("prop_leg")
    leg_joint = object_model.get_articulation("tray_to_leg")

    def vnames(p) -> set[str]:
        return {v.name for v in p.visuals}

    # -- Flat ventilated support tray built from _lightening_slot_bar rails --
    tray_vis = vnames(tray)
    ctx.check(
        "flat ventilated support tray has five slot_bar rails",
        all(f"slot_bar_{i}" in tray_vis for i in range(5)),
        details=f"tray visuals: {sorted(tray_vis)}",
    )
    ctx.check(
        "slot bars use lightening-slot mesh generation",
        any(
            hasattr(v.geometry, "filename") and "tray_slot_bar" in v.geometry.filename
            for v in tray.visuals
        ),
        details="tray_slot_bar is the managed mesh name from _lightening_slot_bar",
    )

    # -- Front hook preserved (hook_bracket + rubber_saddle) -----------------
    ctx.check(
        "front hook bracket and rubber saddle exist on tray",
        "hook_bracket" in tray_vis and "rubber_saddle" in tray_vis,
    )

    # -- Rubber feet preserved (rail_foot_*) ---------------------------------
    ctx.check(
        "rubber feet at front underside",
        "rail_foot_0" in tray_vis and "rail_foot_1" in tray_vis,
    )

    # -- Prop leg structure --------------------------------------------------
    leg_vis = vnames(leg)
    ctx.check(
        "prop_leg has side bars, cross bar, hinge bosses, and hinge pin",
        "leg_bar_0" in leg_vis
        and "leg_bar_1" in leg_vis
        and "cross_bar" in leg_vis
        and "hinge_boss_0" in leg_vis
        and "hinge_boss_1" in leg_vis
        and "hinge_pin" in leg_vis,
        details=f"leg visuals: {sorted(leg_vis)}",
    )

    # -- Captured-knuckle hinge overlaps ------------------------------------
    for i in range(2):
        lug_name = f"hinge_lug_{i}"
        boss_name = f"hinge_boss_{i}"
        ctx.allow_overlap(
            tray, leg, elem_a=lug_name, elem_b=boss_name,
            reason=f"Prop-leg hinge boss {i} nests inside tray hinge lug (captured knuckle).",
        )
        ctx.allow_overlap(
            tray, leg, elem_a=lug_name, elem_b="hinge_pin",
            reason=f"Hinge pin passes through tray hinge lug {i}.",
        )
        ctx.allow_overlap(
            tray, leg, elem_a=lug_name, elem_b=f"leg_bar_{i}",
            reason=f"Prop-leg side bar {i} seats into tray hinge lug at the pivot.",
        )
        ctx.expect_overlap(
            leg, tray, axes="y",
            elem_a=boss_name, elem_b=lug_name,
            min_overlap=0.004,
            name=f"prop-leg boss {i} engages tray lug {i}",
        )

    # -- Exactly one non-fixed joint (the prop-leg revolute) ----------------
    non_fixed = [
        j for j in object_model.articulations
        if j.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "exactly one non-fixed joint: tray_to_leg prop-leg revolute",
        len(non_fixed) == 1 and non_fixed[0].name == "tray_to_leg",
        details=f"non_fixed={[j.name for j in non_fixed]}",
    )

    # -- Prop-leg deployed pose: foot extends below tray ---------------------
    deployed_aabb = ctx.part_world_aabb(leg)
    tray_aabb = ctx.part_world_aabb(tray)
    ctx.check(
        "prop leg extends below the tray when deployed (q=0)",
        deployed_aabb is not None
        and tray_aabb is not None
        and deployed_aabb[0][2] < tray_aabb[0][2] + 0.01,
        details=f"leg_min_z={deployed_aabb[0][2] if deployed_aabb else None}, "
                f"tray_min_z={tray_aabb[0][2] if tray_aabb else None}",
    )

    # -- Prop-leg folds toward the tray underside at max q ------------------
    with ctx.pose({leg_joint: 1.30}):
        folded_aabb = ctx.part_world_aabb(leg)
    ctx.check(
        "prop leg folds upward toward the tray at q=1.30",
        deployed_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > deployed_aabb[0][2] + 0.04,
        details=f"deployed_min_z={deployed_aabb[0][2]}, folded_min_z={folded_aabb[0][2]}",
    )

    return ctx.report()


object_model = build_object_model()
