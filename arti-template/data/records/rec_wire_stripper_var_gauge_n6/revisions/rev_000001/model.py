"""Yellow-and-black multifunction electrician wire-stripping pliers.

Construction notes
------------------
- The tool lies in the XY plane: +X = nose direction, Y = jaw opening plane,
  Z = blade thickness. The pivot is at the origin.
- Both arms are authored as 2D profiles in a "closed" configuration (jaw
  cutting edges along y=0) and then counter-rotated by +/-DELTA via their
  visual origins, so the rest pose (q=0) is the OPEN pliers of the reference
  photo. The squeeze articulation rotates the moving arm by up to
  2*DELTA - 0.02 rad, stopping a hair short of edge-to-edge contact.
- The pivot is a half-lap joint: the fixed arm keeps the lower half of the
  boss disc, the moving arm keeps the upper half (an annulus around the pin
  bore), separated by a small z gap. Full-thickness geometry of each arm
  stays >= 0.5 mm outside the boss circle, so the arms never interpenetrate
  at any pose.
- The stripping notches are real boolean semicircular cuts paired across the
  two blades; the nose serrations are cut into the blade profile polyline.
- The coil spring is welded to the fixed arm only; its free end stops short
  of the moving handle (URDF tree, no closed loop).
"""

from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

MM = 0.001

# ---------------------------------------------------------------------------
# Layout constants (millimetres unless noted).
# ---------------------------------------------------------------------------
DELTA = 0.19                      # half opening angle at rest (rad)
Q_CLOSE = 2.0 * DELTA - 0.02      # squeeze travel: stops just shy of contact

T_STEEL = 6.0                     # blade / arm plate thickness
Z_TOP = T_STEEL / 2.0
Z_BOT = -T_STEEL / 2.0
LAP_GAP = 0.30                    # z gap between the two lap halves

BOSS_R = 12.0                     # pivot boss disc radius
PIN_BORE_R = 2.9                  # bore in the moving lap plate
# The pin shaft is an intentional interference (bushing) fit in the moving
# lap bore: it is the physical support path that carries the moving arm.
# The fit is axisymmetric about the joint axis, so it is pose-invariant.
PIN_SHAFT_R = PIN_BORE_R + 0.15
PLATE_HOLE_R = 3.3                # pivot plate clears the pin shaft

# Graduated wire-stripping notches (x centre, cut radius) – 6 ascending gauges
NOTCH_SPECS = [
    (26.0, 0.70),
    (29.5, 0.90),
    (33.5, 1.15),
    (38.0, 1.50),
    (43.0, 1.90),
    (48.5, 2.30),
]
# Crimping / gripping scallops near the pivot
CRIMP_SPECS = [(16.5, 2.6), (22.0, 2.3)]

SHANK_SLOPE = 8.5 / 35.4          # handle shank edge slope

MESH_TOL = 0.00015
MESH_ANG_TOL = 0.25

_BUILD_INFO: dict[str, float] = {}


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------
def _rot2(p: tuple[float, float], a: float) -> tuple[float, float]:
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def _mirror(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(x, -y) for (x, y) in reversed(pts)]


def _poly_solid(pts_mm, z0_mm: float, z1_mm: float) -> cq.Workplane:
    pts = [(x * MM, y * MM) for (x, y) in pts_mm]
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, z0_mm * MM))
        .polyline(pts)
        .close()
        .extrude((z1_mm - z0_mm) * MM)
    )


def _circle_solid(cx_mm, cy_mm, r_mm, z0_mm, z1_mm) -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(cx_mm * MM, cy_mm * MM, z0_mm * MM))
        .circle(r_mm * MM)
        .extrude((z1_mm - z0_mm) * MM)
    )


def _cut_edge_circles(solid: cq.Workplane, specs) -> cq.Workplane:
    """Cut through-thickness cylinders centred on the cutting edge (y=0)."""
    for (x, r) in specs:
        solid = solid.cut(_circle_solid(x, 0.0, r, -20.0, 20.0))
    return solid


def _shank_top_y(x: float) -> float:
    return -1.5 + SHANK_SLOPE * (x + 12.6)


def _shank_bot_y(x: float) -> float:
    return -9.5 + SHANK_SLOPE * (x + 12.6)


def _teeth_pts(x0=54.0, n=8, pitch=2.75, depth=1.0) -> list[tuple[float, float]]:
    """Serration zigzag along the nose cutting edge (cuts up into the blade)."""
    pts: list[tuple[float, float]] = []
    for i in range(n):
        xa = x0 + i * pitch
        pts.append((xa + pitch * 0.5, depth))
        pts.append((xa + pitch, 0.0))
    return pts


# ---------------------------------------------------------------------------
# 2D profiles, authored in the CLOSED configuration for the +Y (fixed) arm.
# ---------------------------------------------------------------------------
def _jaw_profile() -> list[tuple[float, float]]:
    pts = [(13.0, 0.0), (54.0, 0.0)]
    pts += _teeth_pts()                       # serrated nose edge -> (76, 0)
    pts += [
        (79.3, 0.35), (80.2, 1.1),            # pointed tip
        (76.5, 3.0), (66.0, 6.2), (56.0, 9.8), (50.0, 12.2),   # nose back
        (46.0, 14.6), (40.0, 15.2), (24.0, 15.2), (16.0, 15.0),
        (13.0, 12.0),
    ]
    return pts


def _blade_face_profile() -> list[tuple[float, float]]:
    return [
        (13.6, 0.0), (48.0, 0.0), (46.0, 13.9), (40.0, 14.5),
        (24.0, 14.5), (16.4, 14.3), (13.6, 11.6),
    ]


def _nose_face_profile() -> list[tuple[float, float]]:
    pts = [(50.5, 0.0), (54.0, 0.0)]
    pts += _teeth_pts()
    pts += [(79.0, 0.35), (79.6, 0.95), (76.2, 2.7), (66.0, 5.9),
            (56.0, 9.4), (50.5, 11.6)]
    return pts


def _shank_profile() -> list[tuple[float, float]]:
    return [(-12.6, -1.5), (-48.0, -10.0), (-48.0, -18.0), (-12.6, -9.5)]


def _grip_profile() -> list[tuple[float, float]]:
    bottom_raw = [
        (-120.0, -20.0), (-122.5, -24.0), (-122.0, -29.0), (-118.0, -33.0),
        (-110.0, -33.8), (-90.0, -30.5), (-70.0, -26.5), (-52.0, -23.0),
        (-46.0, -22.0),
    ]
    bottom = sample_catmull_rom_spline_2d(bottom_raw, samples_per_segment=5)
    # Slightly bowed inner edge for an ergonomic molded-grip read.
    return [(-46.0, -8.0), (-80.0, -12.3)] + [(x, y) for (x, y) in bottom]


# Yellow/black split line for the grip overmold (gentle, no tight radii).
_SPLIT_LINE = [(-43.0, -19.3), (-70.0, -23.0), (-95.0, -27.5),
               (-114.0, -30.0), (-126.0, -25.0)]


def _region_below(line_pts) -> list[tuple[float, float]]:
    return list(line_pts) + [(-126.0, -70.0), (-43.0, -70.0)]


def _region_above(line_pts) -> list[tuple[float, float]]:
    return list(line_pts) + [(-126.0, 40.0), (-43.0, 40.0)]


def _collar_profile() -> list[tuple[float, float]]:
    return [
        (-15.0, _shank_top_y(-15.0) + 1.0),
        (-21.0, _shank_top_y(-21.0) + 1.0),
        (-21.0, _shank_bot_y(-21.0) - 1.0),
        (-15.0, _shank_bot_y(-15.0) - 1.0),
    ]


# ---------------------------------------------------------------------------
# Solid builders
# ---------------------------------------------------------------------------
def _build_jaw_plate() -> tuple[cq.Workplane, float, float]:
    solid = _poly_solid(_jaw_profile(), Z_BOT, Z_TOP)
    vol0 = solid.val().Volume()
    solid = _cut_edge_circles(solid, CRIMP_SPECS + NOTCH_SPECS)
    vol1 = solid.val().Volume()
    return solid, vol0, vol1


def _build_blade_face() -> cq.Workplane:
    solid = _poly_solid(_blade_face_profile(), Z_TOP - 0.2, Z_TOP + 0.5)
    return _cut_edge_circles(solid, CRIMP_SPECS + NOTCH_SPECS)


def _build_nose_face() -> cq.Workplane:
    return _poly_solid(_nose_face_profile(), Z_TOP - 0.2, Z_TOP + 0.5)


def _build_fixed_lap() -> cq.Workplane:
    z0, z1 = Z_BOT, -LAP_GAP / 2.0
    disc = _circle_solid(0.0, 0.0, BOSS_R, z0, z1)
    jaw_tab = _poly_solid(
        [(9.0, 0.8), (16.0, 0.8), (16.0, 13.5), (9.0, 13.5)], z0, z1)
    shank_tab = _poly_solid(
        [(-16.0, -9.5), (-9.0, -9.5), (-9.0, -0.8), (-16.0, -0.8)], z0, z1)
    return disc.union(jaw_tab).union(shank_tab)


def _build_moving_lap() -> cq.Workplane:
    z0, z1 = LAP_GAP / 2.0, Z_TOP
    disc = _circle_solid(0.0, 0.0, BOSS_R, z0, z1)
    disc = disc.cut(_circle_solid(0.0, 0.0, PIN_BORE_R, -20.0, 20.0))
    jaw_tab = _poly_solid(
        [(9.0, -13.5), (16.0, -13.5), (16.0, -0.8), (9.0, -0.8)], z0, z1)
    shank_tab = _poly_solid(
        [(-16.0, 0.8), (-9.0, 0.8), (-9.0, 9.5), (-16.0, 9.5)], z0, z1)
    return disc.union(jaw_tab).union(shank_tab)


def _build_grip_pair(mirrored: bool):
    """Yellow grip body + black overmold band (outer edge and rear cap)."""
    grip_pts = _grip_profile()
    split = list(_SPLIT_LINE)
    split_low = [(x, y - 1.0) for (x, y) in split]   # 1 mm seam overlap
    if mirrored:
        grip_pts = _mirror(grip_pts)
        below = _poly_solid(_mirror(_region_below(split_low)), -9.0, 9.0)
        above = _poly_solid(_mirror(_region_above(split)), -9.0, 9.0)
    else:
        below = _poly_solid(_region_below(split_low), -9.0, 9.0)
        above = _poly_solid(_region_above(split), -9.0, 9.0)
    yellow = _poly_solid(grip_pts, -7.0, 7.0).cut(below)
    black = _poly_solid(grip_pts, -7.6, 7.6).cut(above)

    def _soften(solid: cq.Workplane, r_mm: float) -> cq.Workplane:
        for sel in (">Z", "<Z"):
            try:
                solid = solid.edges(sel).fillet(r_mm * MM)
            except Exception:
                try:
                    solid = solid.edges(sel).chamfer(0.6 * r_mm * MM)
                except Exception:
                    pass
        return solid

    return _soften(yellow, 1.8), _soften(black, 1.8)


def _build_pivot_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY", origin=(0.0, 0.0, 2.9 * MM))
        .ellipse(11.0 * MM, 8.0 * MM)
        .extrude(0.5 * MM)
    )
    plate = plate.cut(_circle_solid(0.0, 0.0, PLATE_HOLE_R, -20.0, 20.0))
    # Stadium-shaped inspection cutout, as in the reference photo.
    slot = _poly_solid([(4.3, -1.2), (8.3, -1.2), (8.3, 1.2), (4.3, 1.2)],
                       -20.0, 20.0)
    slot = slot.union(_circle_solid(4.3, 0.0, 1.2, -20.0, 20.0))
    slot = slot.union(_circle_solid(8.3, 0.0, 1.2, -20.0, 20.0))
    return plate.cut(slot)


def _build_pivot_pin():
    flange = CylinderGeometry(5.0 * MM, 1.4 * MM, radial_segments=40)
    flange.translate(0.0, 0.0, -3.5 * MM)
    shaft = CylinderGeometry(PIN_SHAFT_R * MM, 7.62 * MM, radial_segments=40)
    shaft.translate(0.0, 0.0, -0.19 * MM)
    head = CylinderGeometry(4.6 * MM, 1.4 * MM, radial_segments=40)
    head.translate(0.0, 0.0, 4.2 * MM)
    return flange.merge(shaft).merge(head)


def _spring_anchor() -> tuple[float, float]:
    """World-frame (rest pose) anchor point on the fixed handle inner edge."""
    return _rot2((-40.0, _shank_top_y(-40.0)), DELTA)


def _build_spring():
    """Coil spring welded to the fixed handle; free end near the moving one."""
    bx, by = _spring_anchor()
    # By symmetry the open-pose gap direction is exactly +Y.
    seat = CylinderGeometry(5.5 * MM, 4.5 * MM, radial_segments=32)
    seat.rotate_x(math.pi / 2.0)
    seat.translate(bx * MM, (by - 0.75) * MM, 0.0)

    coil_r, wire_r, turns = 4.2, 1.05, 6.5
    s0, s1 = 1.0, 26.5
    n = int(turns * 16)
    pts = []
    for i in range(n + 1):
        t = i / n
        s = s0 + t * (s1 - s0)
        th = 2.0 * math.pi * turns * t
        pts.append((
            (bx + coil_r * math.cos(th)) * MM,
            (by + s) * MM,
            (coil_r * math.sin(th)) * MM,
        ))
    coil = tube_from_spline_points(
        pts, radius=wire_r * MM, samples_per_segment=2,
        radial_segments=12, cap_ends=True,
    )
    return seat.merge(coil)


def _build_latch() -> cq.Workplane:
    """Thumb lock latch, authored relative to its joint frame."""
    pad = cq.Workplane("XY").box(9.0 * MM, 6.7 * MM, 3.5 * MM)
    try:
        pad = pad.edges("|Z").fillet(1.5 * MM)
    except Exception:
        pass
    pad = pad.translate((4.0 * MM, -0.35 * MM, 1.75 * MM))
    hook = (
        cq.Workplane("XY")
        .box(4.0 * MM, 4.5 * MM, 4.5 * MM)
        .translate((3.5 * MM, -5.25 * MM, 0.25 * MM))
    )
    latch = pad.union(hook)
    for cx in (1.8, 4.0, 6.2):
        rib = (
            cq.Workplane("XY")
            .box(0.9 * MM, 6.3 * MM, 0.9 * MM)
            .translate((cx * MM, -0.35 * MM, 3.65 * MM))
        )
        latch = latch.union(rib)
    return latch


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wire_stripper_pliers")

    steel_dark = model.material("steel_dark_textured", rgba=(0.14, 0.145, 0.15, 1.0))
    steel_bright = model.material("steel_brushed_bright", rgba=(0.80, 0.82, 0.84, 1.0))
    grip_yellow = model.material("grip_yellow", rgba=(0.95, 0.76, 0.05, 1.0))
    overmold_black = model.material("overmold_black", rgba=(0.055, 0.055, 0.06, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.24, 0.24, 0.26, 1.0))
    latch_dark = model.material("latch_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    open_fixed = Origin(rpy=(0.0, 0.0, DELTA))
    open_moving = Origin(rpy=(0.0, 0.0, -DELTA))

    # ---- fixed arm (root): +Y jaw, -Y handle -------------------------------
    fixed = model.part("fixed_arm")

    jaw, vol0, vol1 = _build_jaw_plate()
    _BUILD_INFO["jaw_uncut_vol"] = vol0
    _BUILD_INFO["jaw_cut_vol"] = vol1
    fixed.visual(mesh_from_cadquery(jaw, "fixed_jaw_plate",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 origin=open_fixed, material=steel_dark, name="jaw_plate")
    fixed.visual(mesh_from_cadquery(_build_blade_face(), "fixed_blade_face",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 origin=open_fixed, material=steel_bright, name="blade_face")
    fixed.visual(mesh_from_cadquery(_build_nose_face(), "fixed_nose_face",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 origin=open_fixed, material=steel_bright, name="nose_face")
    fixed.visual(mesh_from_cadquery(_build_fixed_lap(), "fixed_lap_plate",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 origin=open_fixed, material=steel_dark, name="lap_plate")
    fixed.visual(mesh_from_cadquery(_poly_solid(_shank_profile(), Z_BOT, Z_TOP),
                                    "fixed_arm_shank",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 origin=open_fixed, material=steel_dark, name="arm_shank")

    grip_y, grip_b = _build_grip_pair(mirrored=False)
    fixed.visual(mesh_from_cadquery(grip_y, "fixed_grip_body",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 origin=open_fixed, material=grip_yellow, name="grip_body")
    fixed.visual(mesh_from_cadquery(grip_b, "fixed_grip_overmold",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 origin=open_fixed, material=overmold_black, name="grip_overmold")
    fixed.visual(mesh_from_cadquery(_poly_solid(_collar_profile(), -4.0, 4.0),
                                    "fixed_grip_collar",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 origin=open_fixed, material=grip_yellow, name="grip_collar")

    fixed.visual(mesh_from_geometry(_build_pivot_pin(), "pivot_pin"),
                 origin=open_fixed, material=latch_dark, name="pivot_pin")
    # Spring is authored directly in the rest (open) frame: no rotation.
    fixed.visual(mesh_from_geometry(_build_spring(), "handle_spring"),
                 material=spring_steel, name="handle_spring")

    # ---- moving arm: -Y jaw, +Y handle (mirror of the fixed arm) -----------
    moving = model.part("moving_arm")

    jaw_m = _poly_solid(_mirror(_jaw_profile()), Z_BOT, Z_TOP)
    jaw_m = _cut_edge_circles(jaw_m, CRIMP_SPECS + NOTCH_SPECS)
    moving.visual(mesh_from_cadquery(jaw_m, "moving_jaw_plate",
                                     tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                  origin=open_moving, material=steel_dark, name="jaw_plate")

    blade_m = _poly_solid(_mirror(_blade_face_profile()), Z_TOP - 0.2, Z_TOP + 0.5)
    blade_m = _cut_edge_circles(blade_m, CRIMP_SPECS + NOTCH_SPECS)
    moving.visual(mesh_from_cadquery(blade_m, "moving_blade_face",
                                     tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                  origin=open_moving, material=steel_bright, name="blade_face")
    moving.visual(mesh_from_cadquery(
        _poly_solid(_mirror(_nose_face_profile()), Z_TOP - 0.2, Z_TOP + 0.5),
        "moving_nose_face", tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
        origin=open_moving, material=steel_bright, name="nose_face")
    moving.visual(mesh_from_cadquery(_build_moving_lap(), "moving_lap_plate",
                                     tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                  origin=open_moving, material=steel_dark, name="lap_plate")
    moving.visual(mesh_from_cadquery(
        _poly_solid(_mirror(_shank_profile()), Z_BOT, Z_TOP), "moving_arm_shank",
        tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
        origin=open_moving, material=steel_dark, name="arm_shank")

    grip_ym, grip_bm = _build_grip_pair(mirrored=True)
    moving.visual(mesh_from_cadquery(grip_ym, "moving_grip_body",
                                     tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                  origin=open_moving, material=grip_yellow, name="grip_body")
    moving.visual(mesh_from_cadquery(grip_bm, "moving_grip_overmold",
                                     tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                  origin=open_moving, material=overmold_black, name="grip_overmold")
    moving.visual(mesh_from_cadquery(
        _poly_solid(_mirror(_collar_profile()), -4.0, 4.0), "moving_grip_collar",
        tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
        origin=open_moving, material=grip_yellow, name="grip_collar")

    moving.visual(mesh_from_cadquery(_build_pivot_plate(), "pivot_plate",
                                     tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                  origin=open_moving, material=steel_bright, name="pivot_plate")

    # ---- thumb lock latch on the moving handle -----------------------------
    latch = model.part("lock_latch")
    latch.visual(mesh_from_cadquery(_build_latch(), "latch_body",
                                    tolerance=MESH_TOL, angular_tolerance=MESH_ANG_TOL),
                 material=latch_dark, name="latch_body")

    # ---- articulations ------------------------------------------------------
    model.articulation(
        "pivot_squeeze",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=moving,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        # +q (right-hand rule about +Z) swings the -Y jaw upward: closes.
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=4.0,
                                   lower=0.0, upper=Q_CLOSE),
    )

    latch_xy = _rot2((-31.0, 5.5), -DELTA)
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=moving,
        child=latch,
        origin=Origin(xyz=(latch_xy[0] * MM, latch_xy[1] * MM, Z_TOP * MM),
                      rpy=(0.0, 0.0, -DELTA)),
        # -Z so +q swings the hook toward the opposite handle (engage).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0,
                                   lower=0.0, upper=0.45),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def _aabb_center(aabb):
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _dist3(a, b):
    return math.dist(a, b)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_arm")
    moving = object_model.get_part("moving_arm")
    latch = object_model.get_part("lock_latch")
    squeeze = object_model.get_articulation("pivot_squeeze")
    latch_joint = object_model.get_articulation("latch_pivot")

    # Intentional overlaps: the rigid coil-spring proxy is compressed by the
    # moving handle when the pliers are squeezed shut, and the latch hook
    # reaches the opposite handle when engaged. Rest pose is overlap-free.
    ctx.allow_overlap(fixed, moving, elem_a="handle_spring", elem_b="arm_shank",
                      reason="Compression spring is rigid; the moving handle "
                             "compresses it when the handles are squeezed.")
    ctx.allow_overlap(fixed, moving, elem_a="handle_spring", elem_b="grip_body",
                      reason="Spring free end meets the moving grip when the "
                             "handles are squeezed shut.")
    ctx.allow_overlap(latch, fixed, elem_a="latch_body", elem_b="arm_shank",
                      reason="Lock latch hook engages the opposite handle "
                             "when flipped while the pliers are closed.")
    ctx.allow_overlap(fixed, moving, elem_a="pivot_pin", elem_b="lap_plate",
                      reason="Captured pivot pin: the pin shaft is an "
                             "intentional axisymmetric bushing fit inside the "
                             "moving lap bore and is the support path that "
                             "carries the moving arm at every pose.")

    # --- overall scale -------------------------------------------------------
    aabbs = [ctx.part_world_aabb(p) for p in (fixed, moving, latch)]
    lo = [min(a[0][i] for a in aabbs) for i in range(3)]
    hi = [max(a[1][i] for a in aabbs) for i in range(3)]
    length, width, thick = hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]
    ctx.check("overall length ~200 mm", 0.185 <= length <= 0.212,
              details=f"length={length:.4f}")
    ctx.check("open-tool width plausible", 0.085 <= width <= 0.125,
              details=f"width={width:.4f}")
    ctx.check("tool thickness plausible", 0.013 <= thick <= 0.019,
              details=f"thick={thick:.4f}")

    # --- pivot assembly ------------------------------------------------------
    ctx.expect_contact(fixed, moving, contact_tol=6e-4,
                       name="arms mate at the lap joint pivot")
    ctx.expect_overlap(fixed, moving, axes="xy",
                       elem_a="pivot_pin", elem_b="lap_plate",
                       min_overlap=0.004,
                       name="pivot pin passes through the moving lap plate")
    ctx.expect_contact(fixed, moving, elem_a="pivot_pin", elem_b="lap_plate",
                       contact_tol=1e-6,
                       name="pivot pin bushing carries the moving arm")

    lim = squeeze.motion_limits
    ctx.check("squeeze joint range",
              lim is not None and abs(lim.lower) < 1e-9
              and abs(lim.upper - Q_CLOSE) < 1e-6,
              details=f"lower={lim.lower}, upper={lim.upper}")

    # --- jaws open at rest, close under squeeze ------------------------------
    ctx.expect_gap(fixed, moving, axis="y",
                   positive_elem="nose_face", negative_elem="nose_face",
                   min_gap=0.006, max_gap=0.05,
                   name="nose jaws are open at rest")

    rest_center = _aabb_center(ctx.part_world_aabb(moving))
    with ctx.pose({squeeze: Q_CLOSE}):
        ctx.expect_contact(fixed, moving,
                           elem_a="nose_face", elem_b="nose_face",
                           contact_tol=0.0035,
                           name="nose jaws nearly touch when squeezed")
        closed_center = _aabb_center(ctx.part_world_aabb(moving))
    disp = _dist3(rest_center, closed_center)
    ctx.check("squeeze visibly swings the moving arm", disp >= 0.005,
              details=f"aabb-center displacement={disp:.4f}")

    # --- stripping notches are real paired cuts ------------------------------
    ctx.check("six graduated stripping notches",
              len(NOTCH_SPECS) == 6
              and all(NOTCH_SPECS[i][1] < NOTCH_SPECS[i + 1][1]
                      for i in range(5)),
              details=f"radii={[r for (_, r) in NOTCH_SPECS]}")
    removed = _BUILD_INFO["jaw_uncut_vol"] - _BUILD_INFO["jaw_cut_vol"]
    expected = sum(0.5 * math.pi * (r * MM) ** 2 * (T_STEEL * MM)
                   for (_, r) in (NOTCH_SPECS + CRIMP_SPECS))
    ctx.check("notch/crimp cuts removed real material",
              0.55 * expected <= removed <= 1.45 * expected,
              details=f"removed={removed:.3e}, expected~{expected:.3e}")

    # --- spring sits between the handles, welded to one side only ------------
    ctx.expect_contact(fixed, moving, elem_a="handle_spring",
                       contact_tol=0.006,
                       name="spring free end reaches near the moving handle")

    # --- lock latch seated and articulated ------------------------------------
    ctx.expect_contact(latch, moving, contact_tol=5e-4,
                       name="latch seats on the moving handle face")
    latch_rest = _aabb_center(ctx.part_world_aabb(latch))
    with ctx.pose({latch_joint: 0.45}):
        latch_flip = _aabb_center(ctx.part_world_aabb(latch))
    ldisp = _dist3(latch_rest, latch_flip)
    ctx.check("lock latch flips on its pivot", ldisp >= 0.0012,
              details=f"latch aabb-center displacement={ldisp:.5f}")

    # --- materials read as the reference tool --------------------------------
    def _mat_name(part, elem):
        mat = part.get_visual(elem).material
        return getattr(mat, "name", mat) or ""

    ctx.check("yellow soft grips",
              "yellow" in _mat_name(fixed, "grip_body")
              and "yellow" in _mat_name(moving, "grip_body"))
    ctx.check("black overmold on grips",
              "black" in _mat_name(fixed, "grip_overmold")
              and "black" in _mat_name(moving, "grip_overmold"))
    ctx.check("bright steel blade faces",
              "bright" in _mat_name(fixed, "blade_face")
              and "bright" in _mat_name(moving, "nose_face"))

    return ctx.report()


object_model = build_object_model()
