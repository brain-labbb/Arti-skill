from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ClevisBracketGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

# ── dimensions ────────────────────────────────────────────────────────────────
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
# SLIDER_TOP_Z < 0 → the bar starts BELOW the joint origin so it is always
# inside the housing (housing extends HOUSING_H downward from joint origin).
# Constraint: SLIDER_TOP_Z − SLIDER_TRAVEL ≥ −HOUSING_H
# (at max extension the bar top is still inside the housing)
SLIDER_TOP_Z = -0.004               # 4 mm below joint origin → safely inside housing
SLIDER_VISIBLE = 0.034              # length that hangs below the housing at rest
SLIDER_BOT_Z   = SLIDER_TOP_Z - SLIDER_VISIBLE   # = −0.038 → FIXED joint here

# Clevis yoke (center=False then rotate_x(π) → base at top, tines hang down)
YOKE_W      = 0.030
YOKE_D      = 0.016
YOKE_H      = 0.026
YOKE_GAP    = 0.013
YOKE_BASE   = 0.007
YOKE_BORE_D = 0.006
YOKE_BORE_Z = 0.017   # from original bottom → bore at z = −0.017 in yoke frame after flip

# Cup: pivot at local z = 0 (cup top), drum hangs below
CUP_R     = 0.040
CUP_DEPTH = 0.024
CUP_HD    = CUP_DEPTH / 2

PAD_MAJOR = 0.032
PAD_TUBE  = 0.012

# Cup-back grille: N evenly spaced vent slots (clearly ≠ 12)
VENTS_PER_CUP = 8


# ── helpers ───────────────────────────────────────────────────────────────────

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
    # Centre the housing: top face at LEG_Z, bottom face at LEG_Z − HOUSING_H
    geo.translate(0.0, side * LEG_Y, LEG_Z - HOUSING_H / 2)
    return geo


def _slider_mesh():
    """
    Slider bar in slider-local frame.
    Top at SLIDER_TOP_Z (below joint origin → hidden inside housing at all times).
    Bottom at SLIDER_BOT_Z (where the yoke attaches).
    """
    bar = BoxGeometry((SLIDER_W, SLIDER_D, SLIDER_VISIBLE))
    bar.translate(0.0, 0.0, (SLIDER_TOP_Z + SLIDER_BOT_Z) / 2)
    return bar


def _yoke_mesh(side: int):
    """
    ClevisBracketGeometry (center=False) then rotate_x(π):
      - base face moves to z = 0 in yoke frame (flush with slider bottom)
      - tines hang down to z = −YOKE_H
      - bore centre at z = −YOKE_BORE_Z
    """
    geo = ClevisBracketGeometry(
        (YOKE_W, YOKE_D, YOKE_H),
        gap_width=YOKE_GAP,
        bore_diameter=YOKE_BORE_D,
        bore_center_z=YOKE_BORE_Z,
        base_thickness=YOKE_BASE,
        corner_radius=0.003,
        center=False,
    )
    geo.rotate_x(math.pi)
    if side < 0:
        geo.scale(1.0, -1.0, 1.0)
    return geo


def _cup_shell_mesh():
    """Cup drum only (open back for grille). Pivot at local z = 0; drum hangs below."""
    drum = CylinderGeometry(CUP_R, CUP_DEPTH, radial_segments=32)
    drum.rotate_x(math.pi / 2)
    drum.translate(0.0, 0.0, -CUP_R)
    return drum


def _grille_ring_mesh(side: int):
    """Annular torus ring at the cup back face — grille perimeter frame."""
    ring = TorusGeometry(CUP_R - 0.006, 0.003, radial_segments=8, tubular_segments=28)
    ring.rotate_x(math.pi / 2)
    ring.translate(0.0, side * CUP_HD, -CUP_R)
    return ring


def _grille_hub_mesh(side: int):
    """Small centre hub disk at the cup back — anchors the inner rib ends."""
    hub = CylinderGeometry(0.010, 0.005, radial_segments=16)
    hub.rotate_x(math.pi / 2)
    hub.translate(0.0, side * CUP_HD, -CUP_R)
    return hub


def _vent_slot_mesh():
    """One radial vent-slot rib for the cup back grille.

    Built along +Z from *inner_r* to *outer_r* at the origin in XY.
    Caller rotates around Y then translates to the cup-back centre.
    """
    inner_r = 0.011
    outer_r = CUP_R - 0.005
    length = outer_r - inner_r
    rib = BoxGeometry((0.004, 0.005, length))
    rib.translate(0.0, 0.0, inner_r + length / 2)
    return rib


def _cup_inner_cap_mesh(side: int):
    cap = CylinderGeometry(CUP_R - 0.003, 0.004, radial_segments=32)
    cap.rotate_x(math.pi / 2)
    cap.translate(0.0, -side * (CUP_HD + 0.001), -CUP_R)
    return cap


def _ear_pad_mesh(side: int):
    pad = TorusGeometry(PAD_MAJOR, PAD_TUBE, radial_segments=12, tubular_segments=36)
    pad.rotate_x(math.pi / 2)
    pad.translate(0.0, -side * (CUP_HD + 0.002), -CUP_R)
    return pad


# ── build ─────────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="over_ear_headphones")

    gunmetal = model.material("gunmetal", rgba=(0.24, 0.24, 0.26, 1.0))
    leather  = model.material("leather",  rgba=(0.08, 0.07, 0.07, 1.0))
    metal_dk = model.material("metal_dk", rgba=(0.19, 0.19, 0.21, 1.0))
    shell_lt = model.material("shell_lt", rgba=(0.73, 0.74, 0.75, 1.0))

    # ── band (root) ───────────────────────────────────────────────────────────
    band = model.part("band")
    band.visual(mesh_from_geometry(_band_shell_mesh(), "band_shell"),
                material=gunmetal, name="band_shell")
    band.visual(mesh_from_geometry(_crown_pad_mesh(), "band_pad"),
                material=leather, name="band_pad")
    # Housing tubes — part of band, hide the sliding slider tops permanently
    for hs, hn in ((+1, "housing_l"), (-1, "housing_r")):
        band.visual(mesh_from_geometry(_housing_mesh(hs), hn),
                    material=gunmetal, name=hn)
    band.inertial = Inertial.from_geometry(
        Box((0.026, 2 * LEG_Y, BAND_APEX_Z)), mass=0.120,
        origin=Origin(xyz=(0.0, 0.0, BAND_APEX_Z * 0.45)),
    )

    # ── per-side assembly ─────────────────────────────────────────────────────
    for side, sl_nm, yk_nm, cp_nm in (
        ( 1, "left_slider",  "left_yoke",  "left_cup"),
        (-1, "right_slider", "right_yoke", "right_cup"),
    ):
        # slider ──────────────────────────────────────────────────────────────
        slider = model.part(sl_nm)
        slider.visual(mesh_from_geometry(_slider_mesh(), f"{sl_nm}_bar"),
                      material=metal_dk, name=f"{sl_nm}_bar")
        slider.inertial = Inertial.from_geometry(
            Box((SLIDER_W, SLIDER_D, SLIDER_VISIBLE)), mass=0.012,
            origin=Origin(xyz=(0.0, 0.0, (SLIDER_TOP_Z + SLIDER_BOT_Z) / 2)),
        )
        # PRISMATIC joint origin is at the band-leg arc endpoint (= housing top)
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

        # yoke (fixed to slider bottom) ───────────────────────────────────────
        yoke = model.part(yk_nm)
        yoke.visual(mesh_from_geometry(_yoke_mesh(side), f"{yk_nm}_body"),
                    material=metal_dk, name=f"{yk_nm}_body")
        yoke.inertial = Inertial.from_geometry(
            Box((YOKE_W, YOKE_D, YOKE_H)), mass=0.007,
        )
        model.articulation(
            f"{sl_nm}_to_{yk_nm}",
            ArticulationType.FIXED,
            parent=slider,
            child=yoke,
            origin=Origin(xyz=(0.0, 0.0, SLIDER_BOT_Z)),
        )

        # cup (pivots on yoke bore) ────────────────────────────────────────────
        cup = model.part(cp_nm)
        cup.visual(mesh_from_geometry(_cup_shell_mesh(), f"{cp_nm}_shell"),
                   material=shell_lt, name=f"{cp_nm}_shell")
        # Open-back grille: ring + hub + N vent-slot ribs
        cup.visual(mesh_from_geometry(_grille_ring_mesh(side), f"{cp_nm}_grille_ring"),
                   material=metal_dk, name=f"{cp_nm}_grille_ring")
        cup.visual(mesh_from_geometry(_grille_hub_mesh(side), f"{cp_nm}_grille_hub"),
                   material=metal_dk, name=f"{cp_nm}_grille_hub")
        for i in range(VENTS_PER_CUP):
            angle = 2.0 * math.pi * i / VENTS_PER_CUP
            rib = _vent_slot_mesh()
            rib.rotate_y(angle)
            rib.translate(0.0, side * CUP_HD, -CUP_R)
            cup.visual(mesh_from_geometry(rib, f"{cp_nm}_vent_{i}"),
                       material=metal_dk, name=f"{cp_nm}_vent_{i}")
        cup.visual(mesh_from_geometry(_cup_inner_cap_mesh(side), f"{cp_nm}_cap"),
                   material=metal_dk, name=f"{cp_nm}_cap")
        cup.visual(mesh_from_geometry(_ear_pad_mesh(side), f"{cp_nm}_pad"),
                   material=leather, name=f"{cp_nm}_pad")
        cup.inertial = Inertial.from_geometry(
            Box((2 * CUP_R, CUP_DEPTH + 0.025, 2 * CUP_R)), mass=0.055,
            origin=Origin(xyz=(0.0, 0.0, -CUP_R)),
        )
        # Bore is at z = −YOKE_BORE_Z in yoke frame (after the rotate_x flip).
        # Cup frame origin coincides with this bore → pivot at cup TOP.
        model.articulation(
            f"{yk_nm}_to_{cp_nm}",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, -YOKE_BORE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=2.0, lower=-0.44, upper=0.44
            ),
        )

    return model


def run_tests() -> "TestReport":
    from sdk import TestContext, TestReport

    ctx = TestContext(object_model)

    band         = object_model.get_part("band")
    left_slider  = object_model.get_part("left_slider")
    right_slider = object_model.get_part("right_slider")
    left_yoke    = object_model.get_part("left_yoke")
    right_yoke   = object_model.get_part("right_yoke")
    left_cup     = object_model.get_part("left_cup")
    right_cup    = object_model.get_part("right_cup")

    l_slide = object_model.get_articulation("band_to_left_slider")
    l_tilt  = object_model.get_articulation("left_yoke_to_left_cup")

    # Cups on correct sides
    lp = ctx.part_world_position(left_cup)
    rp = ctx.part_world_position(right_cup)
    ctx.check("left cup on +Y side",  lp is not None and lp[1] > 0.04, details=f"{lp}")
    ctx.check("right cup on −Y side", rp is not None and rp[1] < -0.04, details=f"{rp}")
    ctx.check("cups symmetric",
              lp is not None and rp is not None and abs(lp[1] + rp[1]) < 0.006,
              details=f"lp={lp}, rp={rp}")

    # Cups below crown
    band_top = ctx.part_world_aabb(band)[1][2]
    ctx.check("cups below band crown",
              lp[2] < band_top - 0.04 and rp[2] < band_top - 0.04,
              details=f"band_top={band_top:.4f}, lc={lp[2]:.4f}")

    # Key no-gap checks (static)
    sl = ctx.part_world_aabb(left_slider)
    yk = ctx.part_world_aabb(left_yoke)
    cp = ctx.part_world_aabb(left_cup)

    ctx.check("slider top inside housing (no exposed gap to band)",
              sl[1][2] <= LEG_Z + 0.002,
              details=f"slider_top={sl[1][2]:.4f}, LEG_Z={LEG_Z:.4f}")
    ctx.check("slider bottom flush with yoke top",
              abs(sl[0][2] - yk[1][2]) < 0.004,
              details=f"sl_bot={sl[0][2]:.4f}, yk_top={yk[1][2]:.4f}")
    ctx.check("yoke bottom reaches cup top",
              yk[0][2] <= cp[1][2] + 0.005,
              details=f"yk_bot={yk[0][2]:.4f}, cp_top={cp[1][2]:.4f}")

    # Slider extends without revealing band-gap at max extension
    with ctx.pose({l_slide: SLIDER_TRAVEL}):
        sl_ext = ctx.part_world_aabb(left_slider)
        # slider top must still be inside the housing (≥ housing bottom = LEG_Z − HOUSING_H)
        ctx.check("slider top inside housing at max extension",
                  sl_ext[1][2] >= LEG_Z - HOUSING_H - 0.002,
                  details=f"sl_top_ext={sl_ext[1][2]:.4f}, housing_bot={LEG_Z - HOUSING_H:.4f}")

    # Cup tilts on pivot
    cp0 = ctx.part_world_aabb(left_cup)
    with ctx.pose({l_tilt: 0.38}):
        cp1 = ctx.part_world_aabb(left_cup)
    ctx.check("cup tilts on yoke pivot",
              abs(cp1[0][2] - cp0[0][2]) > 0.002,
              details=f"rest={cp0[0][2]:.4f}, tilted={cp1[0][2]:.4f}")

    # ── grille vent count ────────────────────────────────────────────────
    for cp_nm, cup_part in (("left_cup", left_cup), ("right_cup", right_cup)):
        vent_names = [v for v in (vis.name for vis in cup_part.visuals)
                      if v.startswith(f"{cp_nm}_vent_")]
        ctx.check(
            f"{cp_nm} has {VENTS_PER_CUP} vent slots",
            len(vent_names) == VENTS_PER_CUP,
            details=f"found {len(vent_names)} vents: {vent_names}",
        )
        ring_name = f"{cp_nm}_grille_ring"
        hub_name = f"{cp_nm}_grille_hub"
        all_vis = {vis.name for vis in cup_part.visuals}
        ctx.check(f"{cp_nm} grille ring present", ring_name in all_vis, details=f"{all_vis}")
        ctx.check(f"{cp_nm} grille hub present", hub_name in all_vis, details=f"{all_vis}")

    # Vent count is explicitly not 12
    ctx.check("vent count differs from 12",
              VENTS_PER_CUP != 12,
              details=f"VENTS_PER_CUP={VENTS_PER_CUP}")

    # Allow intentional overlaps
    ctx.allow_overlap(left_slider,  band,         reason="slider bar slides inside housing tube")
    ctx.allow_overlap(right_slider, band,         reason="slider bar slides inside housing tube")
    ctx.allow_overlap(left_yoke,    left_slider,  reason="yoke base flush against slider bottom")
    ctx.allow_overlap(right_yoke,   right_slider, reason="yoke base flush against slider bottom")
    ctx.allow_overlap(left_cup,     left_yoke,    reason="cup top inserts into yoke clevis gap")
    ctx.allow_overlap(right_cup,    right_yoke,   reason="cup top inserts into yoke clevis gap")

    ctx.expect_contact(left_slider,  band,        name="left slider in band housing")
    ctx.expect_contact(right_slider, band,        name="right slider in band housing")
    ctx.expect_contact(left_cup,     left_yoke,   name="left cup on yoke")
    ctx.expect_contact(right_cup,    right_yoke,  name="right cup on yoke")

    return ctx.report()


object_model = build_object_model()
