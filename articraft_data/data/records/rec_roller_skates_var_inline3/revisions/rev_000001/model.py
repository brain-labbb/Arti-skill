from __future__ import annotations

"""A pair of inline roller skates with hard boots, ankle cuffs, laces, and three
larger inline wheels per frame (reference: picture/Sports/Roller scates/001.png).

Layout (per-skate local frame): +X toe, +Y lateral, +Z up, ground at z=0.
Both skates share one construction routine; the right skate is placed with
mirrored lateral coordinates (side multiplier), never negative mesh scaling.

Articulation: 6 continuous wheel-spin joints about lateral axles and 2 revolute
ankle-cuff flex joints with realistic forward/back limits.
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

WHEELS_PER_SKATE = 3
WHEEL_RADIUS = 0.044  # bumped from parent 0.040 for 3-wheel fill
WHEEL_WIDTH = 0.024
AXLE_Z = WHEEL_RADIUS  # wheels rest on the ground plane z=0

# Evenly spaced axle stations over the same wheelbase as the parent (0.246 m).
_AXLE_SPAN = 0.246
AXLE_XS = tuple(
    _AXLE_SPAN / 2.0 - i * _AXLE_SPAN / max(WHEELS_PER_SKATE - 1, 1)
    for i in range(WHEELS_PER_SKATE)
)  # wheel_0 = front

AXLE_LENGTH = 0.036
AXLE_RADIUS = 0.0045

# Vertical lift to clear the larger wheel diameter (parent was 0.080 top).
_Z = 2.0 * (WHEEL_RADIUS - 0.040)  # 0.008

RAIL_HALF_SPAN = 0.148  # lengthened from parent 0.140
RAIL_THICKNESS = 0.005
RAIL_CENTER_Y = 0.016  # inner faces +-0.0135, outer +-0.0185
RAIL_TOP_Z = 0.0815 + _Z
DECK_PLATE_Z0 = 0.081 + _Z
DECK_PLATE_Z1 = 0.0873 + _Z  # 0.3 mm intentional seat into the sole above

SOLE_BOTTOM_Z = 0.087 + _Z
SOLE_TOP_Z = 0.097 + _Z
SHELL_BASE_Z = 0.095 + _Z

SHAFT_CX = -0.048
SHAFT_Z0 = 0.150 + _Z
SHAFT_Z1 = 0.225 + _Z

CUFF_PIVOT = (-0.048, 0.0, 0.215 + _Z)
CUFF_Z0 = 0.205 + _Z
CUFF_Z1 = 0.275 + _Z
CUFF_LOWER = -0.15  # rearward ankle flex (rad)
CUFF_UPPER = 0.45  # forward ankle flex (rad)

RIGHT_SKATE_OFFSET = (-0.018, -0.135, 0.0)
RIGHT_SKATE_YAW = 0.07  # slight toe-in toward the left skate

TONGUE_TILT = -0.50  # lean back toward -X (radians about +Y)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

MAT_SHELL = Material(name="boot_shell_gray", rgba=(0.74, 0.74, 0.76, 1.0))
MAT_SHAFT = Material(name="boot_upper_gray", rgba=(0.60, 0.61, 0.64, 1.0))
MAT_LINER = Material(name="liner_dark", rgba=(0.22, 0.22, 0.24, 1.0))
MAT_SOLE = Material(name="sole_dark", rgba=(0.20, 0.20, 0.22, 1.0))
MAT_TONGUE = Material(name="tongue_gray", rgba=(0.45, 0.45, 0.48, 1.0))
MAT_RED = Material(name="accent_red", rgba=(0.70, 0.13, 0.16, 1.0))
MAT_LACE = Material(name="lace_light_gray", rgba=(0.82, 0.81, 0.79, 1.0))
MAT_CUFF = Material(name="cuff_gray", rgba=(0.55, 0.56, 0.59, 1.0))
MAT_FRAME = Material(name="frame_white", rgba=(0.92, 0.93, 0.95, 1.0))
MAT_TIRE = Material(name="urethane_translucent", rgba=(0.93, 0.90, 0.85, 0.55))
MAT_HUB = Material(name="hub_white", rgba=(0.96, 0.96, 0.97, 1.0))
MAT_STEEL = Material(name="hardware_steel", rgba=(0.52, 0.53, 0.56, 1.0))


# ---------------------------------------------------------------------------
# Shared mesh construction (built once, reused by both skates)
# ---------------------------------------------------------------------------


def _build_shell_mesh():
    """Hard-boot foot shell: superellipse side loft along the boot length."""
    sections = [
        # (station, z_min, z_max, width) in the loft frame (axis +Y -> boot +X)
        (-0.135, SHELL_BASE_Z, 0.156, 0.052),
        (-0.118, SHELL_BASE_Z, 0.171, 0.076),
        (-0.090, SHELL_BASE_Z, 0.178, 0.088),
        (-0.055, SHELL_BASE_Z, 0.178, 0.092),
        (-0.020, SHELL_BASE_Z, 0.170, 0.094),
        (0.015, SHELL_BASE_Z, 0.158, 0.095),
        (0.050, SHELL_BASE_Z, 0.143, 0.094),
        (0.085, SHELL_BASE_Z, 0.132, 0.091),
        (0.115, SHELL_BASE_Z, 0.124, 0.083),
        (0.138, SHELL_BASE_Z, 0.117, 0.068),
        (0.148, SHELL_BASE_Z, 0.112, 0.046),
    ]
    smooth = resample_side_sections(sections, samples_per_span=3, smooth_passes=1)
    geom = superellipse_side_loft(smooth, exponents=2.5, segments=64)
    geom.rotate_z(-math.pi / 2.0)  # loft axis +Y -> boot +X
    return mesh_from_geometry(geom, "boot_shell")


def _build_heel_panel_mesh():
    """Red heel/achilles riser panel, slightly proud of the shell, rising to
    just under the cuff collar like the reference skate's rear accent."""
    sections = [
        (-0.140, 0.105, 0.164, 0.054),
        (-0.126, 0.105, 0.184, 0.074),
        (-0.112, 0.105, 0.198, 0.082),
        (-0.096, 0.105, 0.204, 0.086),
    ]
    smooth = resample_side_sections(sections, samples_per_span=3, smooth_passes=1)
    geom = superellipse_side_loft(smooth, exponents=2.5, segments=48)
    geom.rotate_z(-math.pi / 2.0)
    return mesh_from_geometry(geom, "heel_panel")


def _build_toe_bumper_mesh():
    """Dark toe bumper cap, slightly proud of the shell."""
    sections = [
        (0.118, 0.0995, 0.122, 0.087),
        (0.140, 0.0995, 0.116, 0.072),
        (0.154, 0.0995, 0.110, 0.044),
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


def _build_shaft_mesh():
    """Hollow ankle shaft of the hard boot (open at the top)."""
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.105, 0.082, 2.4),
        [superellipse_profile(0.093, 0.070, 2.4)],
        SHAFT_Z1 - SHAFT_Z0,
        center=True,
    )
    geom.translate(SHAFT_CX, 0.0, (SHAFT_Z0 + SHAFT_Z1) / 2.0)
    return mesh_from_geometry(geom, "boot_shaft")


def _build_liner_mesh():
    """Dark padded liner collar nested inside the shaft opening."""
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.092, 0.069, 2.4),
        [superellipse_profile(0.082, 0.059, 2.4)],
        0.053,
        center=True,
    )
    geom.translate(SHAFT_CX, 0.0, SHAFT_Z0 + 0.0265)  # top stays below the cuff collar
    return mesh_from_geometry(geom, "liner_collar")


def _build_tongue_mesh():
    geom = ExtrudeGeometry(
        rounded_rect_profile(0.013, 0.050, 0.005),
        0.080,
        center=True,
    )
    geom.rotate_y(TONGUE_TILT)
    geom.translate(0.016, 0.0, 0.168)
    return mesh_from_geometry(geom, "boot_tongue")


def _build_rail_mesh():
    """One white frame rail: extruded side profile with axle bosses and arches."""
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


def _build_cuff_ring_mesh():
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.130, 0.110, 2.4),
        [superellipse_profile(0.116, 0.096, 2.4)],
        CUFF_Z1 - CUFF_Z0,
        center=True,
    )
    # Cuff visuals are authored relative to the ankle pivot frame.
    geom.translate(0.0, 0.0, (CUFF_Z0 + CUFF_Z1) / 2.0 - CUFF_PIVOT[2])
    return mesh_from_geometry(geom, "cuff_ring")


def _build_cuff_upper_mesh():
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(0.122, 0.104, 2.4),
        [superellipse_profile(0.108, 0.090, 2.4)],
        0.024,
        center=True,
    )
    geom.translate(-0.008, 0.0, 0.295 - CUFF_PIVOT[2])
    return mesh_from_geometry(geom, "cuff_upper_collar")


def _build_tire_mesh():
    geom = TireGeometry(
        WHEEL_RADIUS,
        WHEEL_WIDTH,
        inner_radius=0.029,
        carcass=TireCarcass(sidewall_bulge=0.10),
        sidewall=TireSidewall(style="rounded", bulge=0.10),
        shoulder=TireShoulder(style="soft", radius=0.003),
    )
    geom.rotate_z(math.pi / 2.0)  # spin axis local +X -> lateral +Y
    return mesh_from_geometry(geom, "wheel_tire")


def _build_hub_mesh():
    geom = WheelGeometry(
        0.030,
        0.022,
        rim=WheelRim(inner_radius=0.022),
        hub=WheelHub(radius=0.009, width=0.023, cap_style="flat"),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.005),
        bore=WheelBore(style="round", diameter=0.006),
    )
    geom.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, "wheel_hub")


def _shared_meshes() -> dict:
    return {
        "shell": _build_shell_mesh(),
        "heel_panel": _build_heel_panel_mesh(),
        "toe_bumper": _build_toe_bumper_mesh(),
        "sole": _build_sole_mesh(),
        "shaft": _build_shaft_mesh(),
        "liner": _build_liner_mesh(),
        "tongue": _build_tongue_mesh(),
        "rail": _build_rail_mesh(),
        "cuff_ring": _build_cuff_ring_mesh(),
        "cuff_upper": _build_cuff_upper_mesh(),
        "tire": _build_tire_mesh(),
        "hub": _build_hub_mesh(),
    }


# ---------------------------------------------------------------------------
# Skate assembly (shared logic; `side` mirrors lateral coordinates)
# ---------------------------------------------------------------------------


def _lace_centers():
    """Lace rung centers riding proud on the tongue's upper face."""
    # direction along the tongue length (top leans toward -X) and outward normal
    d = (math.sin(TONGUE_TILT), 0.0, math.cos(TONGUE_TILT))
    n = (math.cos(TONGUE_TILT), 0.0, -math.sin(TONGUE_TILT))
    centers = []
    for t in (0.15, 0.32, 0.49, 0.66, 0.83):
        a = 0.080 * (t - 0.5)
        off = 0.0083
        centers.append(
            (
                0.016 + d[0] * a + n[0] * off,
                0.0,
                0.168 + d[2] * a + n[2] * off,
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

    # --- boot visuals -----------------------------------------------------
    boot.visual(meshes["shell"], material=MAT_SHELL, name="shell")
    boot.visual(meshes["sole"], material=MAT_SOLE, name="sole")
    boot.visual(meshes["shaft"], material=MAT_SHAFT, name="shaft")
    boot.visual(meshes["liner"], material=MAT_LINER, name="liner_collar")
    boot.visual(meshes["heel_panel"], material=MAT_RED, name="heel_panel")
    boot.visual(meshes["toe_bumper"], material=MAT_SOLE, name="toe_bumper")
    boot.visual(meshes["tongue"], material=MAT_TONGUE, name="tongue")

    # Red eyelet stays flanking the tongue (mirrored pair).
    for label, sy in (("lateral", side), ("medial", -side)):
        boot.visual(
            Box((0.011, 0.013, 0.085)),
            origin=Origin(xyz=(0.018, sy * 0.0295, 0.166), rpy=(0.0, TONGUE_TILT, 0.0)),
            material=MAT_RED,
            name=f"eyelet_stay_{label}",
        )

    # Lace rungs across the tongue.
    for i, (cx, _cy, cz) in enumerate(_lace_centers()):
        boot.visual(
            Cylinder(radius=0.0022, length=0.048),
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

    # --- wheels (front to rear: 0..WHEELS_PER_SKATE-1) --------------------
    for i in range(WHEELS_PER_SKATE):
        ax = AXLE_XS[i]
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

    # --- ankle cuff ---------------------------------------------------------
    cuff = model.part(f"{prefix}_cuff")
    cuff.visual(meshes["cuff_ring"], material=MAT_CUFF, name="collar")
    cuff.visual(meshes["cuff_upper"], material=MAT_CUFF, name="upper_collar")
    cuff.visual(
        Box((0.012, 0.072, 0.016)),
        origin=Origin(xyz=(0.0665, 0.0, 0.047)),
        material=MAT_RED,
        name="strap",
    )
    cuff.visual(
        Box((0.010, 0.018, 0.020)),
        origin=Origin(xyz=(0.063, side * 0.030, 0.047)),
        material=MAT_STEEL,
        name="strap_buckle",
    )
    for label, sy in (("lateral", side), ("medial", -side)):
        cuff.visual(
            Cylinder(radius=0.0085, length=0.016),
            origin=Origin(xyz=(0.0, sy * 0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
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
    wheels = {
        s: [object_model.get_part(f"{s}_wheel_{i}") for i in range(WHEELS_PER_SKATE)]
        for s in sides
    }
    spin_joints = {
        s: [
            object_model.get_articulation(f"{s}_wheel_{i}_spin")
            for i in range(WHEELS_PER_SKATE)
        ]
        for s in sides
    }
    cuff_joints = {s: object_model.get_articulation(f"{s}_cuff_flex") for s in sides}

    # ----- intentional, scoped allowances ----------------------------------
    # The right skate of the pair stands beside the left skate without touching
    # it; the pair is one prompt-mandated object with two separate stands.
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
        for label in ("lateral", "medial"):
            ctx.allow_overlap(
                boots[s],
                cuffs[s],
                elem_a="shaft",
                elem_b=f"pivot_rivet_{label}",
                reason="Cuff pivot rivet is intentionally seated into the boot shaft wall.",
            )
        # The hard cuff is a hollow collar wrapped concentrically around the
        # boot shaft with real radial clearance; conservative overlap QC flags
        # the concave ring as a solid.
        ctx.allow_overlap(
            boots[s],
            cuffs[s],
            elem_a="shaft",
            elem_b="collar",
            reason="Hollow ankle cuff collar nests concentrically around the boot shaft with radial clearance.",
        )
        ctx.expect_within(
            boots[s],
            cuffs[s],
            axes="xy",
            inner_elem="shaft",
            outer_elem="collar",
            margin=0.003,
            name=f"{s} boot shaft stays centered inside the cuff collar",
        )

    # ----- wheel count: 3 per skate, 6 total -------------------------------
    total_wheels = sum(1 for p in object_model.parts if "_wheel_" in p.name)
    ctx.check(
        f"skate pair has exactly {2 * WHEELS_PER_SKATE} wheels",
        total_wheels == 2 * WHEELS_PER_SKATE,
        details=f"wheel parts: {[p.name for p in object_model.parts if '_wheel_' in p.name]}",
    )
    for s in sides:
        side_wheels = [p for p in object_model.parts if p.name.startswith(f"{s}_wheel_")]
        ctx.check(
            f"{s} skate has exactly {WHEELS_PER_SKATE} wheels",
            len(side_wheels) == WHEELS_PER_SKATE,
            details=f"found: {[p.name for p in side_wheels]}",
        )

    # ----- wheel spin joints: continuous lateral-axle -----------------------
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

    # ----- 3 wheels evenly spaced along each frame -------------------------
    for s in sides:
        x_positions = sorted(
            (spin_joints[s][i].origin.xyz[0] for i in range(WHEELS_PER_SKATE)),
            reverse=True,
        )
        spacings = [
            x_positions[j] - x_positions[j + 1] for j in range(len(x_positions) - 1)
        ]
        ctx.check(
            f"{s} wheels are evenly spaced along the frame",
            len(spacings) >= 1 and all(abs(sp - spacings[0]) < 0.002 for sp in spacings),
            details=f"x_positions={[round(x, 4) for x in x_positions]}, spacings={[round(sp, 4) for sp in spacings]}",
        )

    # ----- larger wheel diameter than the parent 4-wheel variant -----------
    for s in sides:
        wheel_aabb = ctx.part_world_aabb(wheels[s][0])
        if wheel_aabb is not None:
            wheel_diameter = wheel_aabb[1][2] - wheel_aabb[0][2]
            ctx.check(
                f"{s} wheel diameter exceeds parent 80 mm",
                wheel_diameter > 0.080,
                details=f"measured diameter={round(wheel_diameter, 4)} m",
            )

    # ----- all 6 wheels coplanar on the ground plane ------------------------
    min_zs = []
    for s in sides:
        for wheel in wheels[s]:
            aabb = ctx.part_world_aabb(wheel)
            if aabb is not None:
                min_zs.append(aabb[0][2])
    spread = max(min_zs) - min(min_zs) if min_zs else 1.0
    expected_total = 2 * WHEELS_PER_SKATE
    ctx.check(
        f"all {expected_total} wheels rest coplanar on the ground",
        len(min_zs) == expected_total
        and spread <= 0.0015
        and all(abs(z) <= 0.004 for z in min_zs),
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
            name=f"{s} cuff is riveted to the boot shaft",
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
    ok_flex = (
        rest_cuff is not None
        and flexed_cuff is not None
        and ((flexed_cuff[0][0] + flexed_cuff[1][0]) - (rest_cuff[0][0] + rest_cuff[1][0])) / 2.0
        > 0.004
    )
    ctx.check(
        "positive cuff flex leans the cuff forward over the toes",
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
            boots[s].get_visual("lace_4")
            cuffs[s].get_visual("collar")
            ok_feats = True
        except Exception as exc:  # noqa: BLE001
            ok_feats = False
            ctx.fail(f"{s} skate hero features", f"missing visual: {exc}")
        if ok_feats:
            ctx.check(f"{s} skate has tongue, laces, and cuff collar", True)

    return ctx.report()


object_model = build_object_model()
