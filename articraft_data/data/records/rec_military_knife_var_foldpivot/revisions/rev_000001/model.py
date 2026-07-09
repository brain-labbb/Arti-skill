from __future__ import annotations

"""Futuristic military folding pocket knife with a side-pivoting tanto blade.

Layout (world frame, knife lying flat on the ground):
- +X is the deploy direction (front of the handle, where the blade exits).
- Handle spans x [0, 0.14], y +-0.015, z [0, 0.02]; ground plane at z = 0.
- The blade pivots on a revolute joint (Z axis) near the front of the handle,
  swinging ~180 degrees from closed (folded inside the handle slot, q=0) to
  fully deployed (inline with the handle, q=pi).
"""

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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

# Internal blade slot (hollow channel), open through the front face.
CHANNEL_Y_HALF = 0.0135
CHANNEL_Z0 = 0.0085
CHANNEL_Z1 = 0.0130
CHANNEL_X0 = 0.008

# Pivot location near the front of the handle, centered in thickness.
PIVOT_X = 0.133
PIVOT_Z = HANDLE_T / 2.0

# Blade (local frame: pivot at origin, blade extends along -X when closed).
BLADE_Y_HALF = 0.0125
BLADE_T = 0.003
BLADE_LEN = 0.125  # visible cutting length from pivot to tip

# Pivot pin visual.
PIN_RADIUS = 0.003
PIN_EXTRA = 0.002  # how far the pin head protrudes past each handle face


# ---------------------------------------------------------------- cq shapes
def _chamfered_bar(length: float, cx: float, chamfer: float) -> cq.Workplane:
    """Handle bar segment with angular chamfered long facets, centered on the
    handle centerline (y=0, z=HANDLE_T/2) at x-center cx."""
    bar = cq.Workplane("XY").box(length, HANDLE_W, HANDLE_T)
    bar = bar.edges("|X").chamfer(chamfer)
    return bar.translate((cx, 0.0, HANDLE_T / 2.0))


def _channel_cut() -> cq.Workplane:
    """Blade slot void, open through the front face of the handle."""
    length = (SHELL_X1 + 0.01) - CHANNEL_X0
    return (
        cq.Workplane("XY")
        .box(length, 2.0 * CHANNEL_Y_HALF, CHANNEL_Z1 - CHANNEL_Z0)
        .translate(
            (
                CHANNEL_X0 + length / 2.0,
                0.0,
                (CHANNEL_Z0 + CHANNEL_Z1) / 2.0,
            )
        )
    )


def _build_shell_shape() -> cq.Workplane:
    """Front metal body: chamfered bar with hollow blade slot and a recessed
    top groove (jimping / decorative track from the original design)."""
    shell = _chamfered_bar(
        SHELL_X1 - SHELL_X0, (SHELL_X0 + SHELL_X1) / 2.0, HANDLE_CHAMFER
    )
    shell = shell.cut(_channel_cut())
    # Recessed groove on top face (reinterpreted as jimping / texture area).
    groove = (
        cq.Workplane("XY")
        .box(0.062, 0.010, 0.008)
        .translate((0.079, 0.0, 0.017 + 0.004))
    )
    return shell.cut(groove)


def _build_grip_shape() -> cq.Workplane:
    """Rear grip block (brown/tan), hollowed where the stowed blade reaches."""
    grip = _chamfered_bar(
        GRIP_X1 - GRIP_X0, (GRIP_X0 + GRIP_X1) / 2.0, HANDLE_CHAMFER
    )
    return grip.cut(_channel_cut())


def _build_blade_shape() -> cq.Workplane:
    """Tanto folding blade.

    Local frame: pivot at origin (0, 0, 0).
    - At q=0 (closed) the blade body extends along -X into the handle slot.
    - At q=pi (open) it extends along +X forward from the pivot.
    - Spine at +Y (serrated), cutting edge at -Y.
    - Centered at z=0 (extruded +-BLADE_T/2).
    """
    pts = [
        (0.003, -0.004),  # narrow heel near pivot (cutting-edge side)
        (-0.010, -BLADE_Y_HALF),  # transition to full width
        (-0.095, -BLADE_Y_HALF),  # straight cutting edge
        (-BLADE_LEN, 0.009),  # tanto angled secondary edge to the tip
        (-BLADE_LEN + 0.012, BLADE_Y_HALF),  # clipped tip back to spine
        (-0.010, BLADE_Y_HALF),  # spine near root
        (0.003, 0.004),  # narrow heel near pivot (spine side)
    ]
    blade = cq.Workplane("XY").polyline(pts).close().extrude(BLADE_T)
    blade = blade.translate((0.0, 0.0, -BLADE_T / 2.0))
    # Serrated / notched spine: rectangular notches cut into the +y spine.
    for i in range(6):
        cx = -0.090 + i * 0.012
        notch = (
            cq.Workplane("XY")
            .box(0.005, 0.006, BLADE_T * 4.0)
            .translate((cx, BLADE_Y_HALF, 0.0))
        )
        blade = blade.cut(notch)
    return blade


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_military_knife")

    model.material("gunmetal", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("gunmetal_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("blade_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("accent_orange", rgba=(0.95, 0.45, 0.10, 1.0))
    model.material("grip_tan", rgba=(0.56, 0.40, 0.24, 1.0))
    model.material("grip_brown_dark", rgba=(0.36, 0.25, 0.15, 1.0))

    # ----- handle (root): metal front body + tan grip block + accents + pivot
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
    # Pivot pin / bolster: visible pin through the handle at the blade pivot.
    # Shifted up by half the extra so the pin is flush with the handle bottom
    # and protrudes slightly above the top face only.
    pin_len = HANDLE_T + PIN_EXTRA
    handle.visual(
        Cylinder(radius=PIN_RADIUS, length=pin_len),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z + PIN_EXTRA / 2.0)),
        material="gunmetal_dark",
        name="pivot_pin",
    )

    # ----- blade: tanto folding blade pivoting on Z axis
    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_build_blade_shape(), "blade"),
        material="blade_steel",
        name="tanto_blade",
    )
    # Thumb stud for one-handed opening (small protrusion on blade spine near pivot).
    blade.visual(
        Cylinder(radius=0.0025, length=0.004),
        origin=Origin(
            xyz=(-0.012, BLADE_Y_HALF + 0.002, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="gunmetal_dark",
        name="thumb_stud",
    )

    # Blade pivot: revolute joint on Z axis, 0 (closed/folded) to pi (open/inline).
    model.articulation(
        "handle_to_blade",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=blade,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=2.0, lower=0.0, upper=math.pi
        ),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    blade = object_model.get_part("blade")
    blade_joint = object_model.get_articulation("handle_to_blade")

    # --- handle scale: ~0.14 x 0.03 x 0.02 m, grounded at z=0
    hbb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle dimensions ~0.14 x 0.03 x 0.02 m",
        hbb is not None
        and abs((hbb[1][0] - hbb[0][0]) - HANDLE_LEN) < 0.005
        and abs((hbb[1][1] - hbb[0][1]) - HANDLE_W) < 0.004
        and HANDLE_T - 0.002 < (hbb[1][2] - hbb[0][2]) < HANDLE_T + 0.006,
        details=f"handle aabb={hbb}",
    )
    ctx.check(
        "knife rests on the ground plane (zmin ~ 0)",
        hbb is not None and abs(hbb[0][2]) < 0.0015,
        details=f"handle aabb={hbb}",
    )

    # --- joint plan: blade revolute on Z axis, 0..pi
    bl = blade_joint.motion_limits
    ctx.check(
        "blade joint is revolute on Z axis with 0..pi range (~180 deg fold)",
        blade_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(blade_joint.axis) == (0.0, 0.0, 1.0)
        and bl is not None
        and abs(bl.lower - 0.0) < 1e-9
        and abs(bl.upper - math.pi) < 1e-6,
        details=(
            f"type={blade_joint.articulation_type}, "
            f"axis={blade_joint.axis}, limits=({bl.lower}, {bl.upper})"
        ),
    )

    # --- only one non-fixed joint (the blade revolute)
    all_joints = [
        a
        for a in object_model.articulations
        if a.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "exactly one non-fixed joint (blade revolute pivot)",
        len(all_joints) == 1 and all_joints[0].name == "handle_to_blade",
        details=f"non-fixed joints: {[j.name for j in all_joints]}",
    )

    # --- blade form: thin (~0.003 m)
    blade_bb = ctx.part_element_world_aabb(blade, elem="tanto_blade")
    ctx.check(
        "tanto blade is thin (~0.003 m in thickness)",
        blade_bb is not None and (blade_bb[1][2] - blade_bb[0][2]) < 0.004,
        details=f"tanto_blade aabb={blade_bb}",
    )

    # --- closed pose (q=0): blade stowed inside the handle slot
    ctx.expect_within(
        blade,
        handle,
        axes="xy",
        margin=0.002,
        name="closed blade stows inside the handle XY footprint",
    )
    ctx.expect_within(
        blade,
        handle,
        axes="z",
        margin=0.002,
        name="closed blade fits within the handle thickness",
    )

    # --- deployed pose (q=pi): blade inline with handle, total ~0.25-0.26 m
    rest_bb = ctx.part_world_aabb(blade)
    with ctx.pose({blade_joint: math.pi}):
        ext_bb = ctx.part_world_aabb(blade)
        ctx.check(
            "fully deployed knife tip reaches ~0.25-0.27 m from handle rear",
            ext_bb is not None and 0.245 < ext_bb[1][0] < 0.275,
            details=f"extended blade aabb={ext_bb}",
        )
        ctx.expect_overlap(
            blade,
            handle,
            axes="x",
            min_overlap=0.005,
            name="deployed blade retains insertion overlap at the pivot",
        )

    ctx.check(
        "positive q swings the blade forward (tip moves to +X)",
        rest_bb is not None
        and ext_bb is not None
        and ext_bb[1][0] > rest_bb[1][0] + 0.05,
        details=f"rest_max_x={rest_bb[1][0]:.4f}, open_max_x={ext_bb[1][0]:.4f}",
    )

    # --- half-open pose (q=pi/2): blade perpendicular to handle
    with ctx.pose({blade_joint: math.pi / 2.0}):
        half_bb = ctx.part_world_aabb(blade)
        ctx.check(
            "half-open blade extends to the side (negative Y)",
            half_bb is not None and half_bb[0][1] < -0.04,
            details=f"half-open blade aabb={half_bb}",
        )

    # --- pivot pin visible at the blade pivot location
    pivot_pin = handle.get_visual("pivot_pin")
    ctx.check(
        "pivot pin visible at the blade pivot point",
        pivot_pin is not None,
        details="pivot_pin visual present on handle",
    )

    # --- hero finishes: gunmetal body, orange accents, tan grip, steel blade
    mat_names = {m.name for m in object_model.materials}
    ctx.check(
        "gunmetal / orange / tan / steel material set present",
        {"gunmetal", "accent_orange", "grip_tan", "blade_steel"} <= mat_names,
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
