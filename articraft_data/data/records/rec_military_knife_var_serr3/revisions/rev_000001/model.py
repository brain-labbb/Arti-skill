from __future__ import annotations

"""Futuristic military OTF (out-the-front) knife with a retractable sliding blade.

Layout (world frame, knife lying flat on the ground):
- +X is the deploy direction (front of the handle, where the blade exits).
- Handle spans x [0, 0.14], y +-0.015, z [0, 0.02]; ground plane at z = 0.
- The blade slides out the front slot along +X (prismatic, 0 -> 0.115 m).
- A raised thumb slider rides in a recessed top track (prismatic, 0 -> 0.04 m),
  mimic-coupled to the blade at reduced ratio.
"""

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
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

# Internal blade channel (hollow front), open through the front face.
CHANNEL_Y_HALF = 0.0135
CHANNEL_Z0 = 0.0085
CHANNEL_Z1 = 0.0130
CHANNEL_X0 = 0.008

# Blade (local frame: joint frame sits at the front slot, channel floor).
BLADE_LEN = 0.126  # ~0.12 m per the brief
BLADE_X_REAR = -0.127
BLADE_X_TIP = -0.001
BLADE_Y_HALF = 0.0125
BLADE_T = 0.003
BLADE_TRAVEL = 0.115

# Thumb slider track (recessed groove in the handle top face).
TRACK_X0 = 0.048
TRACK_X1 = 0.110
TRACK_Y_HALF = 0.005
TRACK_FLOOR_Z = 0.017
SLIDER_TRAVEL = 0.040
SLIDER_LEN = 0.018
SLIDER_Y_HALF = 0.0045


# ---------------------------------------------------------------- cq shapes
def _chamfered_bar(length: float, cx: float, chamfer: float) -> cq.Workplane:
    """Handle bar segment with angular chamfered long facets, centered on the
    handle centerline (y=0, z=HANDLE_T/2) at x-center cx."""
    bar = cq.Workplane("XY").box(length, HANDLE_W, HANDLE_T)
    bar = bar.edges("|X").chamfer(chamfer)
    return bar.translate((cx, 0.0, HANDLE_T / 2.0))


def _channel_cut() -> cq.Workplane:
    """Blade channel void, open through the front face of the handle."""
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
    """Front metal body: chamfered bar, hollow blade channel, recessed top track."""
    shell = _chamfered_bar(SHELL_X1 - SHELL_X0, (SHELL_X0 + SHELL_X1) / 2.0, HANDLE_CHAMFER)
    shell = shell.cut(_channel_cut())
    groove = (
        cq.Workplane("XY")
        .box(TRACK_X1 - TRACK_X0, 2.0 * TRACK_Y_HALF, 0.008)
        .translate(((TRACK_X0 + TRACK_X1) / 2.0, 0.0, TRACK_FLOOR_Z + 0.004))
    )
    return shell.cut(groove)


def _build_grip_shape() -> cq.Workplane:
    """Rear grip block (brown/tan), hollowed where the stowed blade reaches."""
    grip = _chamfered_bar(GRIP_X1 - GRIP_X0, (GRIP_X0 + GRIP_X1) / 2.0, HANDLE_CHAMFER)
    return grip.cut(_channel_cut())


def _build_blade_shape() -> cq.Workplane:
    """Tanto blade with angled clipped tip and a notched/serrated spine.

    Local frame: x=0 at the front slot plane (joint frame), blade body extends
    rearward to x=-0.127; z=0 at the channel floor, thickness +0.003.
    Spine at +y (serrated), cutting edge at -y.
    """
    pts = [
        (BLADE_X_REAR, -BLADE_Y_HALF),
        (-0.030, -BLADE_Y_HALF),  # straight main edge
        (BLADE_X_TIP, 0.0090),  # angled tanto secondary edge up to the tip
        (-0.016, BLADE_Y_HALF),  # clipped tip back to the spine
        (BLADE_X_REAR, BLADE_Y_HALF),
    ]
    blade = cq.Workplane("XY").polyline(pts).close().extrude(BLADE_T)
    # Serrated / notched spine: three larger evenly spaced notches cut into the +y spine.
    for i in range(3):
        cx = -0.100 + i * 0.030
        notch = (
            cq.Workplane("XY")
            .box(0.008, 0.008, BLADE_T * 4.0)
            .translate((cx, BLADE_Y_HALF, BLADE_T / 2.0))
        )
        blade = blade.cut(notch)
    return blade


def _build_slider_shape() -> cq.Workplane:
    """Raised thumb slider button: base plug in the track + ridged cap.

    Local frame: x=0 at the rear face of the button, z=0 on the track floor.
    """
    base = (
        cq.Workplane("XY")
        .box(SLIDER_LEN, 2.0 * SLIDER_Y_HALF, 0.004)
        .translate((SLIDER_LEN / 2.0, 0.0, 0.002))
    )
    cap = (
        cq.Workplane("XY")
        .box(0.015, 0.008, 0.0025)
        .translate((SLIDER_LEN / 2.0, 0.0, 0.004 + 0.00125))
    )
    button = base.union(cap)
    for i in range(3):
        cx = SLIDER_LEN / 2.0 + (i - 1) * 0.004
        rib = (
            cq.Workplane("XY")
            .box(0.0015, 0.008, 0.001)
            .translate((cx, 0.0, 0.0065 + 0.0005))
        )
        button = button.union(rib)
    return button


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="otf_military_knife")

    model.material("gunmetal", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("gunmetal_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("blade_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("accent_orange", rgba=(0.95, 0.45, 0.10, 1.0))
    model.material("grip_tan", rgba=(0.56, 0.40, 0.24, 1.0))
    model.material("grip_brown_dark", rgba=(0.36, 0.25, 0.15, 1.0))

    # ----- handle (root): metal front body + tan grip block + accents
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

    # ----- blade: tanto OTF blade riding on the channel floor
    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_build_blade_shape(), "blade"),
        material="blade_steel",
        name="tanto_blade",
    )

    # ----- thumb slider button in the recessed top track
    slider = model.part("thumb_slider")
    slider.visual(
        mesh_from_cadquery(_build_slider_shape(), "thumb_slider"),
        material="gunmetal_dark",
        name="slider_button",
    )

    # Blade slide: joint frame at the front slot, on the channel floor.
    model.articulation(
        "handle_to_blade",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade,
        origin=Origin(xyz=(SHELL_X1, 0.0, CHANNEL_Z0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=BLADE_TRAVEL),
    )

    # Thumb slider: same axis, reduced travel, mimics blade deployment.
    model.articulation(
        "handle_to_thumb_slider",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=slider,
        origin=Origin(xyz=(0.050, 0.0, TRACK_FLOOR_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.5, lower=0.0, upper=SLIDER_TRAVEL),
        mimic=Mimic(joint="handle_to_blade", multiplier=SLIDER_TRAVEL / BLADE_TRAVEL),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    blade = object_model.get_part("blade")
    slider = object_model.get_part("thumb_slider")
    blade_joint = object_model.get_articulation("handle_to_blade")
    slider_joint = object_model.get_articulation("handle_to_thumb_slider")

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

    # --- joint plan: blade prismatic 0..0.115 along +X
    bl = blade_joint.motion_limits
    ctx.check(
        "blade joint is prismatic along +X with 0..0.115 m travel",
        blade_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(blade_joint.axis) == (1.0, 0.0, 0.0)
        and bl is not None
        and abs(bl.lower - 0.0) < 1e-9
        and abs(bl.upper - BLADE_TRAVEL) < 1e-9,
        details=f"axis={blade_joint.axis}, limits=({bl.lower}, {bl.upper})",
    )

    # --- joint plan: thumb slider prismatic 0..0.04, mimic of the blade slide
    sl = slider_joint.motion_limits
    ctx.check(
        "thumb slider joint is prismatic along +X with 0..0.04 m travel",
        slider_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(slider_joint.axis) == (1.0, 0.0, 0.0)
        and sl is not None
        and abs(sl.lower - 0.0) < 1e-9
        and abs(sl.upper - SLIDER_TRAVEL) < 1e-9,
        details=f"axis={slider_joint.axis}, limits=({sl.lower}, {sl.upper})",
    )
    mim = slider_joint.mimic
    ctx.check(
        "thumb slider mimics the blade slide at reduced ratio",
        mim is not None
        and mim.joint == "handle_to_blade"
        and abs(mim.multiplier - SLIDER_TRAVEL / BLADE_TRAVEL) < 1e-6,
        details=f"mimic={mim}",
    )

    # --- blade form: ~0.12 m long, thin (~0.003 m), asymmetric tanto tip
    bbb = ctx.part_world_aabb(blade)
    ctx.check(
        "blade is ~0.12 m long and ~0.003 m thin",
        bbb is not None
        and abs((bbb[1][0] - bbb[0][0]) - BLADE_LEN) < 0.004
        and (bbb[1][2] - bbb[0][2]) < 0.0036,
        details=f"blade aabb={bbb}",
    )

    # --- retracted pose: blade fully hidden inside the hollow handle
    ctx.expect_within(
        blade,
        handle,
        axes="xyz",
        margin=0.0005,
        name="retracted blade stows fully inside the handle",
    )
    # The blade rides on the channel floor (supported, not floating).
    ctx.expect_contact(
        blade,
        handle,
        contact_tol=1e-5,
        name="blade rests on the internal channel floor",
    )
    # The hollow channel really clears the blade sides (no interpenetration).
    ctx.expect_within(
        blade,
        handle,
        axes="y",
        margin=0.0,
        name="blade clears the channel side walls",
    )

    # --- thumb slider seated in the recessed track, raised above the top face
    ctx.expect_within(
        slider,
        handle,
        axes="xy",
        margin=0.0005,
        name="thumb slider stays inside the handle footprint",
    )
    ctx.expect_contact(
        slider,
        handle,
        contact_tol=1e-5,
        name="thumb slider seats on the track floor",
    )
    sbb = ctx.part_world_aabb(slider)
    ctx.check(
        "thumb slider button is raised above the handle top face",
        sbb is not None and hbb is not None and sbb[1][2] > HANDLE_T + 0.002,
        details=f"slider aabb={sbb}",
    )

    # --- deployed pose: blade extends out the front, total length ~0.26 m
    rest_blade = ctx.part_world_position(blade)
    rest_slider = ctx.part_world_position(slider)
    with ctx.pose({blade_joint: BLADE_TRAVEL}):
        ext_bb = ctx.part_world_aabb(blade)
        ctx.check(
            "fully deployed knife is ~0.25-0.26 m overall",
            ext_bb is not None and 0.245 < ext_bb[1][0] < 0.265,
            details=f"extended blade aabb={ext_bb}",
        )
        ctx.expect_overlap(
            blade,
            handle,
            axes="x",
            min_overlap=0.008,
            name="deployed blade keeps retained insertion in the handle",
        )
        ext_blade = ctx.part_world_position(blade)
        ext_slider = ctx.part_world_position(slider)

    ctx.check(
        "positive blade travel deploys the blade out the front (+X)",
        rest_blade is not None
        and ext_blade is not None
        and ext_blade[0] - rest_blade[0] > BLADE_TRAVEL - 1e-6,
        details=f"rest={rest_blade}, extended={ext_blade}",
    )
    ctx.check(
        "mimic drives the thumb slider forward ~0.04 m at full deploy",
        rest_slider is not None
        and ext_slider is not None
        and abs((ext_slider[0] - rest_slider[0]) - SLIDER_TRAVEL) < 1e-4,
        details=f"rest={rest_slider}, extended={ext_slider}",
    )

    # --- blade spine serration: exactly 3 larger evenly spaced notches
    blade_cq = _build_blade_shape().val()
    # Notch box (y-size 0.008 centered at BLADE_Y_HALF) removes spine material
    # from y ≈ 0.0085 to 0.0125 at notch x-centers; spine remains intact between.
    probe_y = BLADE_Y_HALF - 0.002  # 0.0105 — inside the notch cut zone
    probe_z = BLADE_T / 2.0
    notch_centers = [-0.100, -0.070, -0.040]
    between_centers = [-0.085, -0.055, -0.025]
    notches_cut = all(
        not blade_cq.isInside(cq.Vector(nx, probe_y, probe_z))
        for nx in notch_centers
    )
    spine_intact = all(
        blade_cq.isInside(cq.Vector(bx, probe_y, probe_z))
        for bx in between_centers
    )
    ctx.check(
        "blade spine has exactly 3 evenly spaced serration notches",
        notches_cut and spine_intact,
        details=f"notches_cut={notches_cut}, spine_intact={spine_intact}",
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
