from __future__ import annotations

"""Futuristic military knife with a fixed full-tang tanto blade and a sliding protective sheath collar.

Layout (world frame, knife lying flat on the ground):
- +X is the blade direction (front of the handle, where the blade extends).
- Handle spans x [0, 0.14], y +-0.015, z [0, 0.02]; ground plane at z = 0.
- The blade is rigidly fixed to the handle, extending from x=0.14 to x~0.266.
- A tubular sheath collar slides along the blade axis on a prismatic joint:
  q=0 (retracted): sheath near the handle, blade exposed for use.
  q=SHEATH_TRAVEL (forward): sheath pushed toward the tip, blade fully covered.
"""

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
HANDLE_LEN = 0.14
HANDLE_W = 0.03
HANDLE_T = 0.02
HANDLE_CHAMFER = 0.004

GRIP_X0 = 0.0
GRIP_X1 = 0.044
SHELL_X0 = 0.042  # 2 mm overlap seam with the grip block (same part)
SHELL_X1 = HANDLE_LEN

# Blade (fixed, full-tang, extending forward from handle front face).
BLADE_LEN = 0.126
BLADE_X_START = HANDLE_LEN
BLADE_X_TIP = HANDLE_LEN + BLADE_LEN
BLADE_Y_HALF = 0.0125
BLADE_T = 0.003
BLADE_Z_BOT = 0.0085
BLADE_CENTER_Z = BLADE_Z_BOT + BLADE_T / 2.0  # 0.01

# Sheath collar (rectangular tube sliding over the blade).
SHEATH_LEN = 0.080
SHEATH_BORE_Y_HALF = 0.014
SHEATH_BORE_Z_HALF = 0.0025
SHEATH_WALL = 0.0025
SHEATH_OUTER_Y_HALF = SHEATH_BORE_Y_HALF + SHEATH_WALL  # 0.0165
SHEATH_OUTER_Z_HALF = SHEATH_BORE_Z_HALF + SHEATH_WALL  # 0.005
SHEATH_TRAVEL = 0.050  # enough to push the sheath front past the blade tip

# Front guard stop plate (prevents sheath from sliding back past the handle).
GUARD_THICK = 0.004
GUARD_Y_HALF = HANDLE_W / 2.0 + 0.004  # 0.019, wider than handle
GUARD_Z_HALF = HANDLE_T / 2.0  # same height as handle (sits on ground plane)


# ---------------------------------------------------------------- cq helpers
def _chamfered_bar(length: float, cx: float, chamfer: float) -> cq.Workplane:
    """Handle bar segment with angular chamfered long facets, centered on the
    handle centerline (y=0, z=HANDLE_T/2) at x-center cx."""
    bar = cq.Workplane("XY").box(length, HANDLE_W, HANDLE_T)
    bar = bar.edges("|X").chamfer(chamfer)
    return bar.translate((cx, 0.0, HANDLE_T / 2.0))


def _build_shell_shape() -> cq.Workplane:
    """Front metal body: chamfered bar (solid, no blade channel needed)."""
    return _chamfered_bar(
        SHELL_X1 - SHELL_X0, (SHELL_X0 + SHELL_X1) / 2.0, HANDLE_CHAMFER
    )


def _build_grip_shape() -> cq.Workplane:
    """Rear grip block (brown/tan), solid."""
    return _chamfered_bar(
        GRIP_X1 - GRIP_X0, (GRIP_X0 + GRIP_X1) / 2.0, HANDLE_CHAMFER
    )


def _build_blade_shape() -> cq.Workplane:
    """Tanto blade with angled clipped tip and a notched/serrated spine.

    Built in world coordinates: extends from x=HANDLE_LEN to x=HANDLE_LEN+BLADE_LEN.
    Spine at +y (serrated), cutting edge at -y.
    """
    x0 = BLADE_X_START
    x1 = BLADE_X_TIP
    pts = [
        (x0, -BLADE_Y_HALF),
        (x1 - 0.030, -BLADE_Y_HALF),
        (x1, 0.0090),
        (x1 - 0.016, BLADE_Y_HALF),
        (x0, BLADE_Y_HALF),
    ]
    blade = cq.Workplane("XY").polyline(pts).close().extrude(BLADE_T)
    blade = blade.translate((0.0, 0.0, BLADE_Z_BOT))
    # Serrated / notched spine: rectangular notches cut into the +y spine.
    for i in range(6):
        cx = x0 + 0.010 + i * 0.012
        notch = (
            cq.Workplane("XY")
            .box(0.005, 0.006, BLADE_T * 4.0)
            .translate((cx, BLADE_Y_HALF, BLADE_Z_BOT + BLADE_T / 2.0))
        )
        blade = blade.cut(notch)
    return blade


def _build_guard_shape() -> cq.Workplane:
    """Front guard stop plate at the handle-blade junction.

    Slightly wider and taller than the handle cross-section; the sheath cannot
    pass it because the guard is wider than the sheath outer profile.
    """
    guard_x = HANDLE_LEN + GUARD_THICK / 2.0
    return (
        cq.Workplane("XY")
        .box(GUARD_THICK, 2.0 * GUARD_Y_HALF, 2.0 * GUARD_Z_HALF)
        .translate((guard_x, 0.0, HANDLE_T / 2.0))
    )


def _build_sheath_shape() -> cq.Workplane:
    """Rectangular tubular sheath collar that slides over the blade.

    Local frame: origin at the rear face center (bore axis on y=0, z=0).
    Extends +X for SHEATH_LEN. Bore through the full length.
    """
    # Outer shell
    outer = (
        cq.Workplane("XY")
        .box(SHEATH_LEN, 2.0 * SHEATH_OUTER_Y_HALF, 2.0 * SHEATH_OUTER_Z_HALF)
        .translate((SHEATH_LEN / 2.0, 0.0, 0.0))
    )
    outer = outer.edges("|X").chamfer(0.001)

    # Through-bore (slightly oversized for blade clearance)
    bore = (
        cq.Workplane("XY")
        .box(SHEATH_LEN + 0.002, 2.0 * SHEATH_BORE_Y_HALF, 2.0 * SHEATH_BORE_Z_HALF)
        .translate((SHEATH_LEN / 2.0, 0.0, 0.0))
    )
    sheath = outer.cut(bore)

    # Grip tab at the rear top for thumb pushing
    tab = (
        cq.Workplane("XY")
        .box(0.008, 2.0 * (SHEATH_OUTER_Y_HALF + 0.002), 0.003)
        .translate((0.004, 0.0, SHEATH_OUTER_Z_HALF + 0.0015))
    )
    sheath = sheath.union(tab)

    # Two small side ridges for grip texture
    for i in range(2):
        sx = 0.015 + i * 0.025
        for sgn in (1.0, -1.0):
            ridge = (
                cq.Workplane("XY")
                .box(0.012, 0.001, 2.0 * SHEATH_OUTER_Z_HALF - 0.002)
                .translate((sx, sgn * (SHEATH_OUTER_Y_HALF + 0.0005), 0.0))
            )
            sheath = sheath.union(ridge)

    return sheath


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="military_knife_sheath")

    model.material("gunmetal", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("gunmetal_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("blade_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("accent_orange", rgba=(0.95, 0.45, 0.10, 1.0))
    model.material("grip_tan", rgba=(0.56, 0.40, 0.24, 1.0))
    model.material("grip_brown_dark", rgba=(0.36, 0.25, 0.15, 1.0))
    model.material("sheath_olive", rgba=(0.20, 0.22, 0.16, 1.0))

    # ----- handle (root): metal body + grip + accents + fixed blade + guard
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_shell_shape(), "handle_shell"),
        material="gunmetal",
        name="metal_body",
    )
    handle.visual(
        mesh_from_cadquery(_build_grip_shape(), "handle_grip"),
        material="grip_tan",
        name="grip_block",
    )
    # Fixed full-tang blade (rigidly attached, extends from handle front)
    handle.visual(
        mesh_from_cadquery(_build_blade_shape(), "fixed_blade"),
        material="blade_steel",
        name="tanto_blade",
    )
    # Front guard stop plate (wider than sheath to act as travel stop)
    handle.visual(
        mesh_from_cadquery(_build_guard_shape(), "guard_plate"),
        material="gunmetal_dark",
        name="guard",
    )
    # Textured grip ridges on the rear block (slightly proud of the top facet).
    for i in range(4):
        cx = 0.010 + i * 0.008
        handle.visual(
            Box((0.0025, 0.020, 0.0011)),
            origin=Origin(xyz=(cx, 0.0, HANDLE_T - 0.0003 + 0.00055)),
            material="grip_brown_dark",
            name=f"grip_ridge_{i}",
        )
    # Thin orange accent strips near the front (top + both side facets).
    for j, sx in enumerate((0.117, 0.127)):
        handle.visual(
            Box((0.005, 0.020, 0.0011)),
            origin=Origin(xyz=(sx, 0.0, HANDLE_T - 0.0003 + 0.00055)),
            material="accent_orange",
            name=f"accent_top_{j}",
        )
        for side, sgn in (("left", 1.0), ("right", -1.0)):
            handle.visual(
                Box((0.005, 0.0011, 0.010)),
                origin=Origin(
                    xyz=(sx, sgn * (HANDLE_W / 2.0 - 0.0003 + 0.00055), HANDLE_T / 2.0)
                ),
                material="accent_orange",
                name=f"accent_{side}_{j}",
            )

    # ----- sheath: tubular collar sliding over the blade on a prismatic joint
    sheath = model.part("sheath")
    sheath.visual(
        mesh_from_cadquery(_build_sheath_shape(), "sheath_collar"),
        material="sheath_olive",
        name="sheath_tube",
    )

    # Sheath slide: prismatic along +X.
    # Joint frame at the guard front face, blade center height.
    # The sheath child frame (rear face center) sits at the joint origin at q=0.
    # q=0: sheath retracted (blade exposed), butted against the guard stop.
    # q=SHEATH_TRAVEL: sheath pushed forward (blade tip covered).
    guard_front_x = HANDLE_LEN + GUARD_THICK
    model.articulation(
        "handle_to_sheath",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=sheath,
        origin=Origin(xyz=(guard_front_x, 0.0, BLADE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.5, lower=0.0, upper=SHEATH_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    sheath = object_model.get_part("sheath")
    sheath_joint = object_model.get_articulation("handle_to_sheath")

    # --- handle body scale: ~0.14 x 0.03 x 0.02 m, grounded at z=0
    shell_bb = ctx.part_element_world_aabb(handle, elem="metal_body")
    ctx.check(
        "handle metal body ~0.10 x 0.03 x 0.02 m",
        shell_bb is not None
        and abs((shell_bb[1][0] - shell_bb[0][0]) - (SHELL_X1 - SHELL_X0)) < 0.005
        and abs((shell_bb[1][1] - shell_bb[0][1]) - HANDLE_W) < 0.004
        and HANDLE_T - 0.002 < (shell_bb[1][2] - shell_bb[0][2]) < HANDLE_T + 0.006,
        details=f"shell aabb={shell_bb}",
    )
    hbb = ctx.part_world_aabb(handle)
    ctx.check(
        "knife rests on the ground plane (zmin ~ 0)",
        hbb is not None and abs(hbb[0][2]) < 0.0015,
        details=f"handle aabb={hbb}",
    )
    ctx.check(
        "overall knife length ~0.26-0.27 m (handle + blade)",
        hbb is not None
        and 0.255 < (hbb[1][0] - hbb[0][0]) < 0.280,
        details=f"handle aabb={hbb}",
    )

    # --- blade is a fixed visual on the handle (not a separate part)
    blade_visual = handle.get_visual("tanto_blade")
    ctx.check(
        "tanto blade is a fixed visual on the handle (full-tang)",
        blade_visual is not None,
        details="tanto_blade visual present on handle",
    )

    # --- blade form: ~0.12 m long, thin (~0.003 m), tanto profile
    # Measure the blade visual directly
    bbb = ctx.part_element_world_aabb(handle, elem="tanto_blade")
    ctx.check(
        "blade is ~0.12 m long and ~0.003 m thin",
        bbb is not None
        and abs((bbb[1][0] - bbb[0][0]) - BLADE_LEN) < 0.005
        and (bbb[1][2] - bbb[0][2]) < 0.004,
        details=f"blade aabb={bbb}",
    )
    ctx.check(
        "blade extends forward from the handle front face",
        bbb is not None and bbb[0][0] >= HANDLE_LEN - 0.002,
        details=f"blade xmin={bbb[0][0] if bbb else None}, handle front={HANDLE_LEN}",
    )

    # --- sheath joint: prismatic along +X, travel 0..SHEATH_TRAVEL
    sl = sheath_joint.motion_limits
    ctx.check(
        "sheath joint is prismatic along +X with correct travel",
        sheath_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(sheath_joint.axis) == (1.0, 0.0, 0.0)
        and sl is not None
        and abs(sl.lower - 0.0) < 1e-9
        and abs(sl.upper - SHEATH_TRAVEL) < 1e-9,
        details=f"axis={sheath_joint.axis}, limits=({sl.lower}, {sl.upper})",
    )

    # --- no blade or slider parts exist (mechanism changed)
    all_part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no separate blade or thumb_slider parts (fixed blade design)",
        "blade" not in all_part_names and "thumb_slider" not in all_part_names,
        details=f"parts={sorted(all_part_names)}",
    )

    # --- sheath at rest (q=0): retracted near handle, blade tip exposed
    sbb_rest = ctx.part_world_aabb(sheath)
    ctx.check(
        "sheath at rest is retracted (rear near handle front)",
        sbb_rest is not None and sbb_rest[0][0] < HANDLE_LEN + GUARD_THICK + 0.005,
        details=f"sheath rest aabb={sbb_rest}",
    )
    ctx.check(
        "sheath at rest does not cover the blade tip",
        sbb_rest is not None and bbb is not None and sbb_rest[1][0] < bbb[1][0] - 0.01,
        details=f"sheath front={sbb_rest[1][0] if sbb_rest else None}, blade tip={bbb[1][0] if bbb else None}",
    )

    # --- sheath at max travel: pushed forward, covers the blade tip
    rest_sheath_pos = ctx.part_world_position(sheath)
    with ctx.pose({sheath_joint: SHEATH_TRAVEL}):
        sbb_ext = ctx.part_world_aabb(sheath)
        ctx.check(
            "sheath at max travel extends past the blade tip (blade covered)",
            sbb_ext is not None
            and bbb is not None
            and sbb_ext[1][0] >= bbb[1][0] - 0.002,
            details=f"sheath front at max={sbb_ext[1][0] if sbb_ext else None}, blade tip={bbb[1][0] if bbb else None}",
        )
        ext_sheath_pos = ctx.part_world_position(sheath)

    ctx.check(
        "positive sheath travel moves the sheath forward (+X) to cover the blade",
        rest_sheath_pos is not None
        and ext_sheath_pos is not None
        and ext_sheath_pos[0] - rest_sheath_pos[0] > SHEATH_TRAVEL - 1e-6,
        details=f"rest={rest_sheath_pos}, extended={ext_sheath_pos}",
    )

    # --- sheath bore fits around the blade (sheath contains blade on y,z)
    ctx.expect_within(
        handle,
        sheath,
        axes="y",
        margin=0.005,
        inner_elem="tanto_blade",
        outer_elem="sheath_tube",
        name="blade fits within the sheath bore width at rest",
    )

    # --- guard plate present at handle-blade junction
    guard_visual = handle.get_visual("guard")
    ctx.check(
        "guard stop plate present at handle-blade junction",
        guard_visual is not None,
        details="guard visual present",
    )

    # --- hero finishes: gunmetal / orange / tan / steel / olive materials
    mat_names = {m.name for m in object_model.materials}
    ctx.check(
        "gunmetal / orange / tan / steel / olive material set present",
        {"gunmetal", "accent_orange", "grip_tan", "blade_steel", "sheath_olive"} <= mat_names,
        details=f"materials={sorted(mat_names)}",
    )
    accents = [v for v in handle.visuals if v.name and v.name.startswith("accent_")]
    ctx.check(
        "orange accent strips near the handle front",
        len(accents) >= 4
        and all(v.origin.xyz[0] > 0.10 for v in accents),
        details=f"accent count={len(accents)}",
    )
    grip = handle.get_visual("grip_block")
    ctx.check(
        "tan grip block sits at the rear of the handle",
        grip is not None,
        details="grip_block visual present",
    )

    return ctx.report()


object_model = build_object_model()
