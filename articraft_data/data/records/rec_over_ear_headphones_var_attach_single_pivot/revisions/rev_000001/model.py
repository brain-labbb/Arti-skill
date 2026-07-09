from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ── dimensions ──────────────────────────────────────────────────────────────
BAND_C_Z    = 0.020
BAND_R      = 0.082
BAND_APEX_Z = BAND_C_Z + BAND_R    # 0.102
CROWN_HALF  = math.radians(75)

# Band-arc endpoints (world positions)
LEG_Y = BAND_R * math.sin(CROWN_HALF)               # ≈ 0.079
LEG_Z = BAND_C_Z + BAND_R * math.cos(CROWN_HALF)   # ≈ 0.041

# Housing tube (part of band, surrounds slider top so the gap never shows)
HOUSING_H = 0.032   # housing extends LEG_Z → LEG_Z − HOUSING_H in Z
HOUSING_W = 0.015   # housing outer X width  (slightly wider than slider)
HOUSING_D = 0.010   # housing outer Y depth  (slightly deeper than slider)

# Slider arm
SLIDER_TRAVEL  = 0.022   # PRISMATIC range (0 → 22 mm)
SLIDER_W       = 0.010   # bar X width
SLIDER_D       = 0.006   # bar Y depth
# Slider bar in slider-local frame: top at SLIDER_TOP_Z, bottom at SLIDER_BOT_Z
SLIDER_TOP_Z   = -0.004               # 4 mm below joint origin → safely inside housing
SLIDER_VISIBLE = 0.034                # length that hangs below the housing at rest
SLIDER_BOT_Z   = SLIDER_TOP_Z - SLIDER_VISIBLE   # = −0.038

# Hanger arm (single-pivot, replaces clevis yoke)
ARM_W       = 0.008    # slim arm width (X)
ARM_D       = 0.005    # arm depth (Y)
ARM_H       = 0.022    # arm height from slider bottom to pivot centre
PIVOT_R     = 0.003    # pivot post radius
PIVOT_LEN   = 0.014    # pivot post total length along X

# Cup: pivot at local z = 0 (cup top), drum hangs below
CUP_R     = 0.040
CUP_DEPTH = 0.024
CUP_HD    = CUP_DEPTH / 2

PAD_MAJOR = 0.032
PAD_TUBE  = 0.012


# ── helpers ─────────────────────────────────────────────────────────────────

def _band_arc_path(n: int = 44):
    """Headband centreline arc in the YZ plane, +Y to −Y."""
    pts = []
    for i in range(n):
        phi = CROWN_HALF - 2 * CROWN_HALF * (i / (n - 1))
        pts.append((0.0, BAND_R * math.sin(phi), BAND_C_Z + BAND_R * math.cos(phi)))
    return pts


def _band_shell_mesh():
    return sweep_profile_along_spline(
        _band_arc_path(),
        profile=rounded_rect_profile(0.022, 0.010, radius=0.004),
        samples_per_segment=3,
        cap_profile=True,
        up_hint=(1.0, 0.0, 0.0),
    )


def _crown_pad_mesh():
    span = math.radians(52)
    n = 20
    r = BAND_R - 0.009
    pts = [
        (0.0,
         r * math.sin(span - 2 * span * (i / n)),
         BAND_C_Z + r * math.cos(span - 2 * span * (i / n)))
        for i in range(n + 1)
    ]
    return tube_from_spline_points(pts, radius=0.008, samples_per_segment=5, radial_segments=16)


def _housing_mesh(side: int):
    """Rectangular housing tube on the band leg that permanently hides the slider top."""
    geo = BoxGeometry((HOUSING_W, HOUSING_D, HOUSING_H))
    geo.translate(0.0, side * LEG_Y, LEG_Z - HOUSING_H / 2)
    return geo


def _slider_mesh():
    """
    Slider bar in slider-local frame.
    Top at SLIDER_TOP_Z (below joint origin → hidden inside housing at all times).
    Bottom at SLIDER_BOT_Z (where the hanger arm attaches).
    """
    bar = BoxGeometry((SLIDER_W, SLIDER_D, SLIDER_VISIBLE))
    bar.translate(0.0, 0.0, (SLIDER_TOP_Z + SLIDER_BOT_Z) / 2)
    return bar


def _hanger_arm_mesh():
    """
    Single slim hanger arm with horizontal pivot post at bottom.
    Built in slider-local coordinates:
      - Arm bar from z = SLIDER_BOT_Z down to z = SLIDER_BOT_Z − ARM_H
      - Pivot post (cylinder along X) centred at z = SLIDER_BOT_Z − ARM_H
    """
    geo = MeshGeometry()

    # Slim vertical arm bar
    arm = BoxGeometry((ARM_W, ARM_D, ARM_H))
    arm.translate(0.0, 0.0, SLIDER_BOT_Z - ARM_H / 2)
    geo.merge(arm)

    # Pivot post — cylinder along X at arm bottom
    pivot = CylinderGeometry(PIVOT_R, PIVOT_LEN, radial_segments=16)
    pivot.rotate_y(math.pi / 2)  # align cylinder axis with X
    pivot.translate(0.0, 0.0, SLIDER_BOT_Z - ARM_H)
    geo.merge(pivot)

    return geo


def _cup_shell_mesh(side: int):
    """Cup drum + shallow outer dome. Pivot is at local z = 0 (cup top); drum hangs below."""
    geo = MeshGeometry()

    # Drum: cylinder axis along Y, centre at z = −CUP_R
    drum = CylinderGeometry(CUP_R, CUP_DEPTH, radial_segments=52)
    drum.rotate_x(math.pi / 2)
    drum.translate(0.0, 0.0, -CUP_R)
    geo.merge(drum)

    # Outer dome on the ±Y face, centred on cup drum centre
    dome = SphereGeometry(CUP_R, width_segments=40, height_segments=22)
    dome.scale(1.0, 0.28, 1.0)
    dome.translate(0.0, side * (CUP_HD + 0.006), -CUP_R)
    geo.merge(dome)

    return geo


def _cup_inner_cap_mesh(side: int):
    cap = CylinderGeometry(CUP_R - 0.003, 0.004, radial_segments=52)
    cap.rotate_x(math.pi / 2)
    cap.translate(0.0, -side * (CUP_HD + 0.001), -CUP_R)
    return cap


def _ear_pad_mesh(side: int):
    pad = TorusGeometry(PAD_MAJOR, PAD_TUBE, radial_segments=20, tubular_segments=56)
    pad.rotate_x(math.pi / 2)
    pad.translate(0.0, -side * (CUP_HD + 0.002), -CUP_R)
    return pad


# ── build ───────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="over_ear_headphones")

    gunmetal = model.material("gunmetal", rgba=(0.24, 0.24, 0.26, 1.0))
    leather  = model.material("leather",  rgba=(0.08, 0.07, 0.07, 1.0))
    metal_dk = model.material("metal_dk", rgba=(0.19, 0.19, 0.21, 1.0))
    shell_lt = model.material("shell_lt", rgba=(0.73, 0.74, 0.75, 1.0))

    # ── band (root) ─────────────────────────────────────────────────────────
    band = model.part("band")
    band.visual(mesh_from_geometry(_band_shell_mesh(), "band_shell"),
                material=gunmetal, name="band_shell")
    band.visual(mesh_from_geometry(_crown_pad_mesh(), "band_pad"),
                material=leather, name="band_pad")
    # Housing tubes — part of band, hide the sliding slider tops permanently
    for i in range(2):
        side = 1 - 2 * i  # +1 for i=0, −1 for i=1
        hn = f"housing_{i}"
        band.visual(mesh_from_geometry(_housing_mesh(side), hn),
                    material=gunmetal, name=hn)
    band.inertial = Inertial.from_geometry(
        Box((0.026, 2 * LEG_Y, BAND_APEX_Z)), mass=0.120,
        origin=Origin(xyz=(0.0, 0.0, BAND_APEX_Z * 0.45)),
    )

    # ── per-side assembly ───────────────────────────────────────────────────
    for i in range(2):
        side = 1 - 2 * i  # +1 for i=0 (left), −1 for i=1 (right)
        sl_nm = f"slider_{i}"
        cp_nm = f"cup_{i}"

        # slider ─────────────────────────────────────────────────────────────
        slider = model.part(sl_nm)
        slider.visual(mesh_from_geometry(_slider_mesh(), f"{sl_nm}_bar"),
                      material=metal_dk, name=f"{sl_nm}_bar")
        # Hanger arm is fixed to slider bottom — inlined as slider visual
        slider.visual(mesh_from_geometry(_hanger_arm_mesh(), f"{sl_nm}_arm"),
                      material=metal_dk, name=f"{sl_nm}_arm")
        slider.inertial = Inertial.from_geometry(
            Box((SLIDER_W, SLIDER_D, SLIDER_VISIBLE + ARM_H)), mass=0.015,
            origin=Origin(xyz=(0.0, 0.0, (SLIDER_TOP_Z + SLIDER_BOT_Z - ARM_H) / 2)),
        )
        # PRISMATIC joint origin at band-leg arc endpoint (= housing top)
        model.articulation(
            f"band_to_{sl_nm}",
            ArticulationType.PRISMATIC,
            parent=band,
            child=slider,
            origin=Origin(xyz=(0.0, side * LEG_Y, LEG_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=12.0, velocity=0.04, lower=0.0, upper=SLIDER_TRAVEL
            ),
        )

        # cup (pivots on hanger arm single pivot post) ───────────────────────
        cup = model.part(cp_nm)
        cup.visual(mesh_from_geometry(_cup_shell_mesh(side), f"{cp_nm}_shell"),
                   material=shell_lt, name=f"{cp_nm}_shell")
        cup.visual(mesh_from_geometry(_cup_inner_cap_mesh(side), f"{cp_nm}_cap"),
                   material=metal_dk, name=f"{cp_nm}_cap")
        cup.visual(mesh_from_geometry(_ear_pad_mesh(side), f"{cp_nm}_pad"),
                   material=leather, name=f"{cp_nm}_pad")
        cup.inertial = Inertial.from_geometry(
            Box((2 * CUP_R, CUP_DEPTH + 0.025, 2 * CUP_R)), mass=0.055,
            origin=Origin(xyz=(0.0, 0.0, -CUP_R)),
        )
        # Cup pivot at arm bottom: slider-local z = SLIDER_BOT_Z − ARM_H
        # Axis along X → cup swings forward/backward (tilts to fit ear)
        model.articulation(
            f"{sl_nm}_to_{cp_nm}",
            ArticulationType.REVOLUTE,
            parent=slider,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, SLIDER_BOT_Z - ARM_H)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=-0.44, upper=0.44
            ),
        )

    return model


def run_tests() -> "TestReport":
    from sdk import TestContext, TestReport

    ctx = TestContext(object_model)

    band        = object_model.get_part("band")
    slider_0    = object_model.get_part("slider_0")
    slider_1    = object_model.get_part("slider_1")
    cup_0       = object_model.get_part("cup_0")
    cup_1       = object_model.get_part("cup_1")

    slide_0 = object_model.get_articulation("band_to_slider_0")
    tilt_0  = object_model.get_articulation("slider_0_to_cup_0")

    # ── Structural: no yoke/fork parts (single-pivot hanger only) ──────────
    all_part_names = [p.name for p in object_model.parts]
    ctx.check("no yoke parts (single-pivot hanger design)",
              not any("yoke" in n for n in all_part_names),
              details=f"parts={all_part_names}")

    # ── Cups on correct sides ──────────────────────────────────────────────
    lp = ctx.part_world_position(cup_0)
    rp = ctx.part_world_position(cup_1)
    ctx.check("cup_0 on +Y side",  lp is not None and lp[1] > 0.04, details=f"{lp}")
    ctx.check("cup_1 on −Y side", rp is not None and rp[1] < -0.04, details=f"{rp}")
    ctx.check("cups symmetric",
              lp is not None and rp is not None and abs(lp[1] + rp[1]) < 0.006,
              details=f"lp={lp}, rp={rp}")

    # ── Cups below crown ───────────────────────────────────────────────────
    band_top = ctx.part_world_aabb(band)[1][2]
    ctx.check("cups below band crown",
              lp[2] < band_top - 0.04 and rp[2] < band_top - 0.04,
              details=f"band_top={band_top:.4f}, lc={lp[2]:.4f}")

    # ── Slider top inside housing ──────────────────────────────────────────
    sl = ctx.part_world_aabb(slider_0)
    ctx.check("slider top inside housing (no exposed gap to band)",
              sl[1][2] <= LEG_Z + 0.002,
              details=f"slider_top={sl[1][2]:.4f}, LEG_Z={LEG_Z:.4f}")

    # ── Hanger arm bottom reaches cup top (pivot at contact surface) ───────
    cp = ctx.part_world_aabb(cup_0)
    # At rest (q=0), slider origin is at LEG_Z in world.
    # Arm bottom (pivot centre) in world: LEG_Z + SLIDER_BOT_Z − ARM_H
    arm_bottom_world = LEG_Z + SLIDER_BOT_Z - ARM_H
    ctx.check("pivot at cup top edge",
              abs(arm_bottom_world - cp[1][2]) < 0.005,
              details=f"pivot_z={arm_bottom_world:.4f}, cup_top={cp[1][2]:.4f}")

    # ── Slider extends without revealing band-gap at max extension ─────────
    with ctx.pose({slide_0: SLIDER_TRAVEL}):
        sl_ext = ctx.part_world_aabb(slider_0)
        ctx.check("slider top inside housing at max extension",
                  sl_ext[1][2] >= LEG_Z - HOUSING_H - 0.002,
                  details=f"sl_top_ext={sl_ext[1][2]:.4f}, housing_bot={LEG_Z - HOUSING_H:.4f}")

    # ── Cup tilts on single pivot (REVOLUTE joint works) ───────────────────
    cp0 = ctx.part_world_aabb(cup_0)
    with ctx.pose({tilt_0: 0.38}):
        cp1 = ctx.part_world_aabb(cup_0)
    ctx.check("cup tilts on single pivot",
              abs(cp1[0][2] - cp0[0][2]) > 0.002,
              details=f"rest={cp0[0][2]:.4f}, tilted={cp1[0][2]:.4f}")

    # ── Verify cup swing changes X-extent (forward/backward tilt) ──────────
    ctx.check("cup tilt changes Z extent (not just XY)",
              abs(cp1[1][2] - cp0[1][2]) > 0.001 or abs(cp1[0][2] - cp0[0][2]) > 0.001,
              details=f"rest_min_z={cp0[0][2]:.4f}, tilt_min_z={cp1[0][2]:.4f}")

    # ── Intentional overlaps ───────────────────────────────────────────────
    ctx.allow_overlap(slider_0, band,
                      reason="slider bar slides inside housing tube")
    ctx.allow_overlap(slider_1, band,
                      reason="slider bar slides inside housing tube")
    ctx.allow_overlap(cup_0, slider_0,
                      elem_b="slider_0_arm",
                      reason="cup top wraps around hanger arm pivot post at single side pivot")
    ctx.allow_overlap(cup_1, slider_1,
                      elem_b="slider_1_arm",
                      reason="cup top wraps around hanger arm pivot post at single side pivot")

    # ── Contact proofs ─────────────────────────────────────────────────────
    ctx.expect_contact(slider_0, band,  name="slider_0 in band housing")
    ctx.expect_contact(slider_1, band,  name="slider_1 in band housing")
    ctx.expect_contact(cup_0, slider_0,
                       elem_b="slider_0_arm",
                       name="cup_0 seated on hanger arm pivot post")
    ctx.expect_contact(cup_1, slider_1,
                       elem_b="slider_1_arm",
                       name="cup_1 seated on hanger arm pivot post")

    return ctx.report()


object_model = build_object_model()
