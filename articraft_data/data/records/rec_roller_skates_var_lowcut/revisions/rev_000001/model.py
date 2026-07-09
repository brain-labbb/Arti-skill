from __future__ import annotations

"""A pair of inline roller skates with low-cut speed boots, short ankle cuffs,
laces, and four inline wheels per frame (reference: picture/Sports/Roller
scates/001.png, low-cut fork variant).

Layout (per-skate local frame): +X toe, +Y lateral, +Z up, ground at z=0.
Both skates share one construction routine; the right skate is placed with
mirrored lateral coordinates (side multiplier), never negative mesh scaling.

Articulation: 8 continuous wheel-spin joints about lateral axles and 2 revolute
ankle-cuff flex joints with realistic forward/back limits.

Fork change vs parent: the tall hard-shell boot shaft is replaced by a low-cut
molded shell that ends near the ankle bone, with a short flexing ankle cuff
band instead of a tall cuff collar.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    resample_side_sections,
    rounded_rect_profile,
    superellipse_profile,
    superellipse_side_loft,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters)
# ---------------------------------------------------------------------------

WHEEL_RADIUS = 0.040
WHEEL_WIDTH = 0.024
AXLE_Z = WHEEL_RADIUS  # wheels rest on the ground plane z=0
AXLE_XS = (0.123, 0.041, -0.041, -0.123)  # wheel_0 = front
AXLE_LENGTH = 0.036
AXLE_RADIUS = 0.0045

RAIL_HALF_SPAN = 0.140
RAIL_THICKNESS = 0.005
RAIL_CENTER_Y = 0.016  # inner faces +-0.0135, outer +-0.0185
RAIL_TOP_Z = 0.0815
DECK_PLATE_Z0 = 0.081
DECK_PLATE_Z1 = 0.0873  # 0.3 mm intentional seat into the sole above

SOLE_BOTTOM_Z = 0.087
SOLE_TOP_Z = 0.097
SHELL_BASE_Z = 0.095

# Low-cut shell: the molded upper ends near the ankle bone, no tall shaft.
ANKLE_CX = -0.048  # ankle center X (heel/ankle zone)
SHELL_TOP_Z = 0.148  # highest point of the shell at the ankle

# Thin padded liner collar visible at the shell's ankle opening.
LINER_HEIGHT = 0.014
LINER_CENTER_Z = SHELL_TOP_Z - 0.002  # peeks just below the shell rim

# Short ankle cuff band (replaces the tall cuff collar of the parent).
CUFF_PIVOT = (-0.048, 0.0, 0.135)  # pivot at the ankle bone
CUFF_Z0 = 0.123  # cuff band bottom
CUFF_Z1 = 0.155  # cuff band top (~32 mm tall)
CUFF_LOWER = -0.18  # rearward ankle flex (rad)
CUFF_UPPER = 0.50  # forward ankle flex (rad)

RIGHT_SKATE_OFFSET = (-0.018, -0.135, 0.0)
RIGHT_SKATE_YAW = 0.07  # slight toe-in toward the left skate

TONGUE_TILT = -0.45  # lean back toward -X (radians about +Y)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

MAT_SHELL = Material(name="boot_shell_graphite", rgba=(0.28, 0.28, 0.30, 1.0))
MAT_LINER = Material(name="liner_dark", rgba=(0.18, 0.18, 0.20, 1.0))
MAT_SOLE = Material(name="sole_dark", rgba=(0.20, 0.20, 0.22, 1.0))
MAT_TONGUE = Material(name="tongue_charcoal", rgba=(0.32, 0.32, 0.35, 1.0))
MAT_RED = Material(name="accent_red", rgba=(0.70, 0.13, 0.16, 1.0))
MAT_LACE = Material(name="lace_white", rgba=(0.90, 0.89, 0.87, 1.0))
MAT_CUFF = Material(name="cuff_dark", rgba=(0.24, 0.24, 0.27, 1.0))
MAT_FRAME = Material(name="frame_silver", rgba=(0.82, 0.83, 0.85, 1.0))
MAT_TIRE = Material(name="urethane_translucent", rgba=(0.93, 0.90, 0.85, 0.55))
MAT_HUB = Material(name="hub_white", rgba=(0.96, 0.96, 0.97, 1.0))
MAT_STEEL = Material(name="hardware_steel", rgba=(0.52, 0.53, 0.56, 1.0))
MAT_TOE_CAP = Material(name="toe_cap_dark", rgba=(0.22, 0.22, 0.24, 1.0))


# ---------------------------------------------------------------------------
# Shared mesh construction (built once, reused by both skates)
# ---------------------------------------------------------------------------


def _build_shell_mesh():
    """Low-cut molded boot shell: superellipse side loft ending at ankle height.

    The loft peaks at the heel/ankle zone (~SHELL_TOP_Z) and slopes smoothly
    down to the toe, reading as a single continuous molded form with no tall
    shaft or separate ankle column.
    """
    sections = [
        # (station, z_min, z_max, width) in the loft frame (axis +Y -> boot +X)
        (-0.138, SHELL_BASE_Z, 0.140, 0.050),  # heel back
        (-0.125, SHELL_BASE_Z, 0.146, 0.072),  # heel sides rise
        (-0.108, SHELL_BASE_Z, 0.148, 0.086),  # ankle peak
        (-0.088, SHELL_BASE_Z, 0.144, 0.090),  # behind ankle
        (-0.055, SHELL_BASE_Z, 0.135, 0.092),  # midfoot arch
        (-0.020, SHELL_BASE_Z, 0.128, 0.094),  # instep
        (0.015, SHELL_BASE_Z, 0.120, 0.095),   # midfoot front
        (0.050, SHELL_BASE_Z, 0.112, 0.094),   # forefoot
        (0.085, SHELL_BASE_Z, 0.106, 0.091),   # ball
        (0.115, SHELL_BASE_Z, 0.100, 0.083),   # toe
        (0.138, SHELL_BASE_Z, 0.095, 0.068),   # toe tip
        (0.150, SHELL_BASE_Z, 0.091, 0.046),   # toe end
    ]
    smooth = resample_side_sections(sections, samples_per_span=3, smooth_passes=1)
    geom = superellipse_side_loft(smooth, exponents=2.5, segments=64)
    geom.rotate_z(-math.pi / 2.0)  # loft axis +Y -> boot +X
    return mesh_from_geometry(geom, "boot_shell")


def _build_heel_panel_mesh():
    """Red heel/achilles accent panel on the low-cut shell, slightly proud."""
    sections = [
        (-0.142, 0.097, 0.138, 0.052),
        (-0.128, 0.097, 0.146, 0.074),
        (-0.112, 0.097, 0.148, 0.084),
        (-0.094, 0.097, 0.144, 0.088),
    ]
    smooth = resample_side_sections(sections, samples_per_span=3, smooth_passes=1)
    geom = superellipse_side_loft(smooth, exponents=2.5, segments=48)
    geom.rotate_z(-math.pi / 2.0)
    return mesh_from_geometry(geom, "heel_panel")


def _build_toe_bumper_mesh():
    """Dark toe bumper cap, slightly proud of the shell."""
    sections = [
        (0.118, 0.0915, 0.103, 0.087),
        (0.140, 0.0915, 0.097, 0.072),
        (0.154, 0.0915, 0.092, 0.044),
    ]
    smooth = resample_side_sections(sections, samples_per_span=3, smooth_passes=1)
    geom = superellipse_side_loft(smooth, exponents=2.5, segments=48)
    geom.rotate_z(-math.pi / 2.0)
    return mesh_from_geometry(geom, "toe_bumper")


def _build_sole_mesh():
    geom = ExtrudeGeometry(
        superellipse_profile(0.285, 0.100, 2.3),
        SOLE_TOP_Z - SOLE_BOTTOM_Z,
        center=True,
    )
    geom.translate(0.005, 0.0, (SOLE_TOP_Z + SOLE_BOTTOM_Z) / 2.0)
    return mesh_from_geometry(geom, "boot_sole")


def _build_liner_mesh():
    """Thin padded liner collar visible at the low-cut shell's ankle opening."""
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.092, 0.078, 2.4),
        [superellipse_profile(0.080, 0.066, 2.4)],
        LINER_HEIGHT,
        center=True,
    )
    geom.translate(ANKLE_CX, 0.0, LINER_CENTER_Z)
    return mesh_from_geometry(geom, "liner_collar")


def _build_tongue_mesh():
    """Shorter tongue matching the low-cut boot height."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(0.013, 0.048, 0.005),
        0.058,
        center=True,
    )
    geom.rotate_y(TONGUE_TILT)
    geom.translate(0.022, 0.0, 0.130)
    return mesh_from_geometry(geom, "boot_tongue")


def _build_rail_mesh():
    """One frame rail: extruded side profile with axle bosses and arches."""
    boss_bottom = 0.026
    arch_bottom = 0.060
    boss_half = 0.013
    blend = 0.013

    def bottom_z(x: float) -> float:
        d = min(abs(x - ax) for ax in AXLE_XS)
        if d <= boss_half:
            return boss_bottom
        if d >= boss_half + blend:
            return arch_bottom
        t = (d - boss_half) / blend
        s = 0.5 - 0.5 * math.cos(math.pi * t)
        return boss_bottom + (arch_bottom - boss_bottom) * s

    xs = [-RAIL_HALF_SPAN + i * 0.004 for i in range(int(2 * RAIL_HALF_SPAN / 0.004) + 1)]
    profile = [(x, bottom_z(x)) for x in xs]
    profile.append((RAIL_HALF_SPAN, RAIL_TOP_Z))
    profile.append((-RAIL_HALF_SPAN, RAIL_TOP_Z))
    geom = ExtrudeGeometry(profile, RAIL_THICKNESS, center=True)
    geom.rotate_x(math.pi / 2.0)  # profile height -> +Z, thickness -> lateral
    return mesh_from_geometry(geom, "frame_rail")


def _build_cuff_band_mesh():
    """Short ankle cuff band: hollow ring ~32mm tall, pivoting at ankle bone."""
    height = CUFF_Z1 - CUFF_Z0
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.118, 0.100, 2.4),
        [superellipse_profile(0.104, 0.086, 2.4)],
        height,
        center=True,
    )
    # Cuff visuals are authored relative to the ankle pivot frame.
    geom.translate(0.0, 0.0, (CUFF_Z0 + CUFF_Z1) / 2.0 - CUFF_PIVOT[2])
    return mesh_from_geometry(geom, "cuff_band")


def _build_cuff_lip_mesh():
    """Thin reinforced lip at the top of the cuff band."""
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.112, 0.096, 2.4),
        [superellipse_profile(0.100, 0.084, 2.4)],
        0.008,
        center=True,
    )
    geom.translate(-0.004, 0.0, CUFF_Z1 + 0.004 - CUFF_PIVOT[2])
    return mesh_from_geometry(geom, "cuff_lip")


def _build_tire_mesh():
    geom = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.026,
        carcass=TireCarcass(sidewall_bulge=0.10),
        sidewall=TireSidewall(style="rounded", bulge=0.10),
        shoulder=TireShoulder(style="soft", radius=0.003),
    )
    geom.rotate_z(math.pi / 2.0)  # spin axis local +X -> lateral +Y
    return mesh_from_geometry(geom, "wheel_tire")


def _build_hub_mesh():
    geom = WheelGeometry(
        0.027,
        0.020,
        rim=WheelRim(inner_radius=0.020),
        hub=WheelHub(radius=0.008, width=0.021, cap_style="flat"),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.0045),
        bore=WheelBore(style="round", diameter=0.0055),
    )
    geom.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, "wheel_hub")


def _shared_meshes() -> dict:
    return {
        "shell": _build_shell_mesh(),
        "heel_panel": _build_heel_panel_mesh(),
        "toe_bumper": _build_toe_bumper_mesh(),
        "sole": _build_sole_mesh(),
        "liner": _build_liner_mesh(),
        "tongue": _build_tongue_mesh(),
        "rail": _build_rail_mesh(),
        "cuff_band": _build_cuff_band_mesh(),
        "cuff_lip": _build_cuff_lip_mesh(),
        "tire": _build_tire_mesh(),
        "hub": _build_hub_mesh(),
    }


# ---------------------------------------------------------------------------
# Skate assembly (shared logic; `side` mirrors lateral coordinates)
# ---------------------------------------------------------------------------


def _lace_centers():
    """Lace rung centers riding proud on the tongue's upper face."""
    d = (math.sin(TONGUE_TILT), 0.0, math.cos(TONGUE_TILT))
    n = (math.cos(TONGUE_TILT), 0.0, -math.sin(TONGUE_TILT))
    centers = []
    for t in (0.18, 0.38, 0.58, 0.78):
        a = 0.058 * (t - 0.5)
        off = 0.0083
        centers.append(
            (
                0.022 + d[0] * a + n[0] * off,
                0.0,
                0.130 + d[2] * a + n[2] * off,
            )
        )
    return centers


def _add_skate(
    model: ArticulatedObject,
    *,
    prefix: str,
    side: float,
    meshes: dict,
    parent_boot=None,
    mount_origin: Origin | None = None,
):
    """Build one skate. `side=+1` for the left skate, `side=-1` for the right.

    All lateral (Y) coordinates are multiplied by `side`, so the two skates are
    true mirrored constructions of the same logic (no negative mesh scaling).
    Returns the boot part (subtree root of this skate).
    """
    boot = model.part(f"{prefix}_boot")
    if parent_boot is not None:
        model.articulation(
            f"{prefix}_boot_mount",
            ArticulationType.FIXED,
            parent=parent_boot,
            child=boot,
            origin=mount_origin or Origin(),
        )

    # --- boot visuals (low-cut shell) -----------------------------------
    boot.visual(meshes["shell"], material=MAT_SHELL, name="shell")
    boot.visual(meshes["sole"], material=MAT_SOLE, name="sole")
    boot.visual(meshes["liner"], material=MAT_LINER, name="liner_collar")
    boot.visual(meshes["heel_panel"], material=MAT_RED, name="heel_panel")
    boot.visual(meshes["toe_bumper"], material=MAT_TOE_CAP, name="toe_bumper")
    boot.visual(meshes["tongue"], material=MAT_TONGUE, name="tongue")

    # Red eyelet stays flanking the tongue (shorter for low-cut, positioned
    # below the cuff strap to avoid overlap).
    for label, sy in (("lateral", side), ("medial", -side)):
        boot.visual(
            Box((0.011, 0.013, 0.044)),
            origin=Origin(xyz=(0.026, sy * 0.0285, 0.118), rpy=(0.0, TONGUE_TILT, 0.0)),
            material=MAT_RED,
            name=f"eyelet_stay_{label}",
        )

    # Lace rungs across the tongue.
    for i, (cx, _cy, cz) in enumerate(_lace_centers()):
        boot.visual(
            Cylinder(radius=0.0022, length=0.046),
            origin=Origin(xyz=(cx, 0.0, cz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=MAT_LACE,
            name=f"lace_{i}",
        )

    # --- frame ------------------------------------------------------------
    frame = model.part(f"{prefix}_frame")
    model.articulation(
        f"{prefix}_frame_mount",
        ArticulationType.FIXED,
        parent=boot,
        child=frame,
        origin=Origin(),
    )
    frame.visual(
        Box((2 * RAIL_HALF_SPAN, 0.040, DECK_PLATE_Z1 - DECK_PLATE_Z0)),
        origin=Origin(xyz=(0.0, 0.0, (DECK_PLATE_Z0 + DECK_PLATE_Z1) / 2.0)),
        material=MAT_FRAME,
        name="deck_plate",
    )
    for label, sy in (("lateral", side), ("medial", -side)):
        frame.visual(
            meshes["rail"],
            origin=Origin(xyz=(0.0, sy * RAIL_CENTER_Y, 0.0)),
            material=MAT_FRAME,
            name=f"rail_{label}",
        )

    # --- wheels (front to rear: 0..3) --------------------------------------
    wheels = []
    for i, ax in enumerate(AXLE_XS):
        wheel = model.part(f"{prefix}_wheel_{i}")
        wheel.visual(meshes["tire"], material=MAT_TIRE, name="tire")
        wheel.visual(meshes["hub"], material=MAT_HUB, name="hub")
        wheel.visual(
            Cylinder(radius=AXLE_RADIUS, length=AXLE_LENGTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=MAT_STEEL,
            name="axle",
        )
        model.articulation(
            f"{prefix}_wheel_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(ax, 0.0, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=60.0),
        )
        wheels.append(wheel)

    # --- short ankle cuff ---------------------------------------------------
    cuff = model.part(f"{prefix}_cuff")
    cuff.visual(meshes["cuff_band"], material=MAT_CUFF, name="cuff_band")
    cuff.visual(meshes["cuff_lip"], material=MAT_CUFF, name="cuff_lip")
    # Velcro-style strap across the front of the cuff band.
    cuff.visual(
        Box((0.012, 0.068, 0.014)),
        origin=Origin(xyz=(0.058, 0.0, 0.008)),
        material=MAT_RED,
        name="strap",
    )
    cuff.visual(
        Box((0.010, 0.016, 0.018)),
        origin=Origin(xyz=(0.055, side * 0.028, 0.008)),
        material=MAT_STEEL,
        name="strap_buckle",
    )
    # Pivot rivets on lateral and medial sides at the ankle bone.
    for label, sy in (("lateral", side), ("medial", -side)):
        cuff.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=Origin(xyz=(0.0, sy * 0.044, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=MAT_STEEL,
            name=f"pivot_rivet_{label}",
        )
    model.articulation(
        f"{prefix}_cuff_flex",
        ArticulationType.REVOLUTE,
        parent=boot,
        child=cuff,
        origin=Origin(xyz=CUFF_PIVOT),
        axis=(0.0, 1.0, 0.0),  # positive q flexes the cuff forward (toward +X)
        motion_limits=MotionLimits(effort=30.0, velocity=4.0, lower=CUFF_LOWER, upper=CUFF_UPPER),
    )

    return boot


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inline_roller_skates_pair")
    meshes = _shared_meshes()

    left_boot = _add_skate(model, prefix="left", side=1.0, meshes=meshes)
    _add_skate(
        model,
        prefix="right",
        side=-1.0,
        meshes=meshes,
        parent_boot=left_boot,
        mount_origin=Origin(xyz=RIGHT_SKATE_OFFSET, rpy=(0.0, 0.0, RIGHT_SKATE_YAW)),
    )
    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sides = ("left", "right")
    boots = {s: object_model.get_part(f"{s}_boot") for s in sides}
    frames = {s: object_model.get_part(f"{s}_frame") for s in sides}
    cuffs = {s: object_model.get_part(f"{s}_cuff") for s in sides}
    wheels = {s: [object_model.get_part(f"{s}_wheel_{i}") for i in range(4)] for s in sides}
    spin_joints = {
        s: [object_model.get_articulation(f"{s}_wheel_{i}_spin") for i in range(4)] for s in sides
    }
    cuff_joints = {s: object_model.get_articulation(f"{s}_cuff_flex") for s in sides}

    # ----- intentional, scoped allowances ----------------------------------
    # Right skate stands beside the left skate without touching it.
    for part in (
        [boots["right"], frames["right"], cuffs["right"]] + wheels["right"]
    ):
        ctx.allow_isolated_part(
            part.name,
            reason="Right skate of the pair intentionally stands separately beside the left skate.",
        )
    for s in sides:
        for wheel in wheels[s]:
            for rail in ("rail_lateral", "rail_medial"):
                ctx.allow_overlap(
                    frames[s],
                    wheel,
                    elem_a=rail,
                    elem_b="axle",
                    reason="Wheel axle is intentionally captured through both frame rails.",
                )
        # The short cuff band wraps concentrically around the low-cut shell
        # ankle area with real radial clearance; conservative overlap QC flags
        # the concave band as a solid.
        ctx.allow_overlap(
            boots[s],
            cuffs[s],
            elem_a="shell",
            elem_b="cuff_band",
            reason="Short ankle cuff band nests concentrically around the low-cut shell ankle zone.",
        )
        ctx.allow_overlap(
            boots[s],
            cuffs[s],
            elem_a="heel_panel",
            elem_b="cuff_band",
            reason="Heel accent panel is proud of the shell at the ankle and is wrapped by the cuff band.",
        )
        for label in ("lateral", "medial"):
            ctx.allow_overlap(
                boots[s],
                cuffs[s],
                elem_a="shell",
                elem_b=f"pivot_rivet_{label}",
                reason="Cuff pivot rivet is intentionally seated into the low-cut shell wall.",
            )
        ctx.allow_overlap(
            boots[s],
            cuffs[s],
            elem_a="tongue",
            elem_b="strap",
            reason="The cuff closure strap intentionally crosses over the boot tongue, as on a real low-cut speed skate.",
        )
        ctx.allow_overlap(
            boots[s],
            cuffs[s],
            elem_a="tongue",
            elem_b="cuff_band",
            reason="The boot tongue extends upward through the cuff band opening, as on a real skate where the tongue is visible above the cuff.",
        )
        # The cuff band wraps around the ankle zone of the shell; prove they
        # overlap laterally (the band encloses the shell at the ankle).
        ctx.expect_overlap(
            boots[s],
            cuffs[s],
            axes="xy",
            elem_a="shell",
            elem_b="cuff_band",
            min_overlap=0.020,
            name=f"{s} cuff band wraps around the shell ankle zone",
        )
        ctx.expect_contact(
            boots[s],
            cuffs[s],
            elem_a="tongue",
            elem_b="strap",
            contact_tol=0.015,
            name=f"{s} cuff strap crosses over the tongue",
        )

    # ----- wheel count, joint types, and lateral spin axes ------------------
    for s in sides:
        for i, joint in enumerate(spin_joints[s]):
            ctx.check(
                f"{s}_wheel_{i}_spin is a continuous lateral-axle joint",
                str(joint.articulation_type).lower().endswith("continuous")
                and abs(joint.axis[1]) > 0.99
                and abs(joint.axis[0]) < 1e-6
                and abs(joint.axis[2]) < 1e-6,
                details=f"type={joint.articulation_type}, axis={joint.axis}",
            )
    ctx.check(
        "skate pair has exactly 8 wheels",
        sum(1 for p in object_model.parts if "_wheel_" in p.name) == 8,
        details=f"wheel parts: {[p.name for p in object_model.parts if '_wheel_' in p.name]}",
    )

    # ----- all 8 wheels coplanar on the ground plane ------------------------
    min_zs = []
    for s in sides:
        for wheel in wheels[s]:
            aabb = ctx.part_world_aabb(wheel)
            if aabb is not None:
                min_zs.append(aabb[0][2])
    spread = max(min_zs) - min(min_zs) if min_zs else 1.0
    ctx.check(
        "all 8 wheels rest coplanar on the ground",
        len(min_zs) == 8 and spread <= 0.0015 and all(abs(z) <= 0.004 for z in min_zs),
        details=f"wheel min-z values: {[round(z, 5) for z in min_zs]}",
    )

    # ----- cuff joints: revolute with realistic ankle-flex limits -----------
    for s in sides:
        joint = cuff_joints[s]
        limits = joint.motion_limits
        ctx.check(
            f"{s}_cuff_flex is a limited revolute ankle joint",
            str(joint.articulation_type).lower().endswith("revolute")
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and -0.6 <= limits.lower < 0.0 < limits.upper <= 0.8,
            details=f"type={joint.articulation_type}, limits=({limits.lower}, {limits.upper})",
        )

    # ----- low-cut boot geometry claims ------------------------------------
    for s in sides:
        shell_aabb = ctx.part_element_world_aabb(boots[s], elem="shell")
        if shell_aabb is not None:
            shell_top = shell_aabb[1][2]
            ctx.check(
                f"{s} shell is low-cut (top below 0.160 m, no tall shaft)",
                shell_top < 0.160,
                details=f"shell top z={shell_top:.4f} m",
            )
        # Verify no tall shaft visual exists (the fork removed it).
        has_shaft = False
        try:
            boots[s].get_visual("shaft")
            has_shaft = True
        except Exception:
            pass
        ctx.check(
            f"{s} boot has no tall shaft (low-cut fork)",
            not has_shaft,
            details="shaft visual should not exist on the low-cut variant",
        )
        # Cuff band is short (under 40 mm tall).
        cuff_band_aabb = ctx.part_element_world_aabb(cuffs[s], elem="cuff_band")
        if cuff_band_aabb is not None:
            band_height = cuff_band_aabb[1][2] - cuff_band_aabb[0][2]
            ctx.check(
                f"{s} cuff band is short (under 40 mm)",
                band_height < 0.040,
                details=f"cuff band height={band_height:.4f} m",
            )

    # ----- mounting and clearances ------------------------------------------
    for s in sides:
        ctx.expect_contact(
            boots[s],
            frames[s],
            name=f"{s} frame deck seats against the boot sole",
        )
        ctx.expect_contact(
            cuffs[s],
            boots[s],
            name=f"{s} cuff band is riveted to the low-cut shell",
        )
        ctx.expect_gap(
            boots[s],
            wheels[s][0],
            axis="z",
            min_gap=0.0,
            name=f"{s} boot sole clears the wheel tops",
        )
    ctx.expect_within(
        wheels["left"][1],
        frames["left"],
        axes="y",
        margin=0.001,
        name="wheels run between the frame rails",
    )
    ctx.expect_origin_distance(
        boots["left"],
        boots["right"],
        axes="y",
        min_dist=0.10,
        max_dist=0.20,
        name="the two skates stand side by side",
    )

    # ----- decisive pose checks ----------------------------------------------
    rest_cuff = ctx.part_world_aabb(cuffs["left"])
    with ctx.pose({cuff_joints["left"]: 0.4}):
        flexed_cuff = ctx.part_world_aabb(cuffs["left"])
    # A forward flex of the short cuff band tilts it, so the Z span increases
    # (back rises, front dips) while the center stays near the pivot.
    ok_flex = False
    if rest_cuff is not None and flexed_cuff is not None:
        rest_span = rest_cuff[1][2] - rest_cuff[0][2]
        flex_span = flexed_cuff[1][2] - flexed_cuff[0][2]
        ok_flex = flex_span > rest_span + 0.005
    ctx.check(
        "positive cuff flex tilts the short ankle band forward",
        ok_flex,
        details=f"rest={rest_cuff}, flexed={flexed_cuff}",
    )

    rest_wheel = ctx.part_world_aabb(wheels["left"][0])
    with ctx.pose({spin_joints["left"][0]: 1.3}):
        spun_wheel = ctx.part_world_aabb(wheels["left"][0])

    def _center(aabb):
        return tuple((aabb[0][k] + aabb[1][k]) / 2.0 for k in range(3))

    ok_spin = False
    if rest_wheel is not None and spun_wheel is not None:
        rc, sc = _center(rest_wheel), _center(spun_wheel)
        ok_spin = all(abs(sc[k] - rc[k]) <= 0.002 for k in range(3))
    ctx.check(
        "wheel spins in place about its lateral axle",
        ok_spin,
        details=f"rest={rest_wheel}, spun={spun_wheel}",
    )

    # ----- hero features present ---------------------------------------------
    for s in sides:
        try:
            boots[s].get_visual("tongue")
            boots[s].get_visual("lace_0")
            boots[s].get_visual("lace_3")
            cuffs[s].get_visual("cuff_band")
            ok_feats = True
        except Exception as exc:  # noqa: BLE001
            ok_feats = False
            ctx.fail(f"{s} skate hero features", f"missing visual: {exc}")
        if ok_feats:
            ctx.check(f"{s} skate has tongue, laces, and short cuff band", True)

    return ctx.report()


object_model = build_object_model()
