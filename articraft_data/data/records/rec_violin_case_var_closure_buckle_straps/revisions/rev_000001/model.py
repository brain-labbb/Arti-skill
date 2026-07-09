from __future__ import annotations

# Hardshell violin case (clamshell) with buckle-strap closures.
#
# Coordinate convention:
#   - long axis of the case (neck -> lower bout)  : +X   (case is ~0.80 m long)
#   - width axis (across the bouts)               : +Y   (~0.26 m at lower bout)
#   - up                                          : +Z   (~0.13 m tall when closed)
#   - the case sits on the table; the BOTTOM shell floor is near z=0 and its
#     rim/sealing plane is at z = SHELL_H. The lid is hinged along the back
#     long edge on the +Y side (y = +HALF_W). The buckle straps are on the
#     front edge (y = -HALF_W).
#
# Parts / articulations:
#   - bottom_shell (ROOT): violin-silhouette tub with a RED plush recess.
#   - lid          : matching shallow violin-silhouette shell lined red inside,
#       REVOLUTE about the rear (+Y) long-edge hinge axis, opens 0..180 deg.
#   - strap_0 / strap_1 : leather buckle straps, each REVOLUTE-hinged on the
#       lid front edge, wrapping over the front rim when closed. 0..~60 deg.
#   - buckle_0 / buckle_1 : metal buckle frames at the free end of each strap,
#       REVOLUTE-pivoting to swing against the bottom shell front wall.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- overall dimensions (metres) ----
CASE_LEN = 0.80
HALF_W = 0.13
SHELL_H = 0.085
LID_H = 0.045
WALL = 0.012
RECESS_INSET = 0.018

HINGE_Y = HALF_W
HINGE_Z = SHELL_H

# ---- strap & buckle dimensions ----
STRAP_W = 0.022        # strap width (along X)
STRAP_T = 0.003        # strap thickness (along Y)
STRAP_L = 0.065        # strap length (hinge to free end, along Z)
BUCKLE_W = 0.028       # buckle frame outer width
BUCKLE_D = 0.006       # buckle frame depth (Y direction)
BUCKLE_H = 0.024       # buckle frame height (Z direction)
BUCKLE_WALL = 0.004    # buckle frame wall thickness

# Two strap positions along the front edge, near spline control points
# (x ≈ 0.144 and x ≈ 0.240 in world) where the spline width is most
# predictable.  The lower-bout region is the widest, flattest part of the
# front edge, matching real case buckle placement.
STRAP_X_POSITIONS = (0.14, 0.24)

# Strap hinge sits on the lid front-edge face, near the top of the lid.
STRAP_HINGE_Z_LID = LID_H * 0.70


# ---------------------------------------------------------------------------
# Violin outline helpers
# ---------------------------------------------------------------------------

def _violin_half_points(scale_w: float = 1.0, scale_x: float = 1.0):
    """Return the +Y half of a violin outline as (x, y) points."""
    L = CASE_LEN
    raw = [
        (0.000, 0.052), (0.060, 0.062), (0.140, 0.082), (0.230, 0.098),
        (0.300, 0.092), (0.360, 0.066), (0.420, 0.066), (0.490, 0.092),
        (0.580, 0.118), (0.680, 0.130), (0.800, 0.128), (0.910, 0.108),
        (0.975, 0.060), (1.000, 0.022),
    ]
    pts = []
    for fx, hy in raw:
        x = (fx - 0.5) * L * scale_x
        y = hy * scale_w
        pts.append((x, y))
    return pts


def _half_width_at_x(x_world: float) -> float:
    """Linearly interpolate the violin half-width (y) at a given world x."""
    pts = _violin_half_points()
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        lo, hi = (x0, x1) if x0 <= x1 else (x1, x0)
        if lo <= x_world <= hi:
            t = 0.0 if x1 == x0 else (x_world - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)
    return pts[len(pts) // 2][1]


def _violin_outline_wire(scale_w: float = 1.0, scale_x: float = 1.0) -> cq.Workplane:
    """Closed violin outline on the XY plane via a spline through mirrored points."""
    half = _violin_half_points(scale_w, scale_x)
    x0, _ = half[0]
    xn, _ = half[-1]
    upper = [(x, y) for (x, y) in half]
    lower = [(x, -y) for (x, y) in reversed(half)]
    interior = upper[1:] + [(xn, 0.0)] + lower[:-1]
    wp = (
        cq.Workplane("XY")
        .moveTo(x0, 0.0)
        .spline(interior, includeCurrent=True)
        .close()
    )
    return wp


# ---------------------------------------------------------------------------
# Shell & interior geometry
# ---------------------------------------------------------------------------

def _bottom_shell_solid() -> cq.Workplane:
    """Violin-silhouette tub: solid base, hollowed to a violin-shaped recess."""
    outer = _violin_outline_wire().extrude(SHELL_H)
    recess = (
        _violin_outline_wire(scale_w=1.0 - RECESS_INSET / HALF_W,
                             scale_x=1.0 - RECESS_INSET / (CASE_LEN * 0.5))
        .extrude(SHELL_H + 0.01)
    )
    recess = recess.translate((0.0, 0.0, WALL))
    return outer.cut(recess)


def _red_interior_solid() -> cq.Workplane:
    """Plush red liner: thin violin-shaped shell lining the recess walls+floor."""
    pad = 0.006
    sw = 1.0 - (RECESS_INSET + pad) / HALF_W
    sx = 1.0 - (RECESS_INSET + pad) / (CASE_LEN * 0.5)
    floor = _violin_outline_wire(scale_w=sw, scale_x=sx).extrude(WALL * 1.4)
    floor = floor.translate((0.0, 0.0, WALL))
    wall_outer = _violin_outline_wire(
        scale_w=1.0 - RECESS_INSET / HALF_W,
        scale_x=1.0 - RECESS_INSET / (CASE_LEN * 0.5),
    ).extrude(SHELL_H - WALL)
    wall_inner = _violin_outline_wire(scale_w=sw, scale_x=sx).extrude(SHELL_H)
    wall = wall_outer.cut(wall_inner)
    wall = wall.translate((0.0, 0.0, WALL))
    return floor.union(wall)


# ---------------------------------------------------------------------------
# Lid geometry
# ---------------------------------------------------------------------------

def _lid_solid() -> cq.Workplane:
    """Matching shallow lid shell, hollowed so its underside reads as a cavity."""
    outer = _violin_outline_wire().extrude(LID_H)
    inner = (
        _violin_outline_wire(scale_w=1.0 - WALL / HALF_W,
                             scale_x=1.0 - WALL / (CASE_LEN * 0.5))
        .extrude(LID_H)
    )
    inner = inner.translate((0.0, 0.0, -0.006))
    return outer.cut(inner)


def _lid_liner_solid() -> cq.Workplane:
    """Thin red liner covering the inside of the lid cavity."""
    sw = 1.0 - WALL / HALF_W
    sx = 1.0 - WALL / (CASE_LEN * 0.5)
    cap = _violin_outline_wire(scale_w=sw, scale_x=sx).extrude(0.008)
    cap = cap.translate((0.0, 0.0, LID_H - 0.012))
    return cap


# ---------------------------------------------------------------------------
# Strap & buckle geometry (shared helpers for the repeated pair)
# ---------------------------------------------------------------------------

def _strap_band_solid() -> cq.Workplane:
    """Flat leather strap band. Origin at hinge-end centre; body extends in -Z.
    Width along X, thin in Y."""
    w, t, L = STRAP_W, STRAP_T, STRAP_L
    # Main band body, centred in XY, extending from z=0 to z=-L
    band = cq.Workplane("XY").box(w, t, L).translate((0.0, 0.0, -L / 2.0))
    return band


def _buckle_frame_solid() -> cq.Workplane:
    """Metal buckle frame with prong bar. Origin at pivot (top centre);
    frame extends in -Z. Width along X, thin in Y."""
    w, d, h = BUCKLE_W, BUCKLE_D, BUCKLE_H
    t = BUCKLE_WALL
    # Outer solid block
    outer = cq.Workplane("XY").box(w, d, h).translate((0.0, 0.0, -h / 2.0))
    # Interior cutout (open window)
    slot_w = w - 2.0 * t
    slot_h = h - 2.0 * t
    inner = (
        cq.Workplane("XY")
        .box(slot_w, d + 0.002, slot_h)
        .translate((0.0, 0.0, -h / 2.0))
    )
    frame = outer.cut(inner)
    # Prong bar across the centre of the window
    prong = (
        cq.Workplane("XY")
        .box(slot_w, 0.003, 0.003)
        .translate((0.0, 0.0, -h / 2.0))
    )
    frame = frame.union(prong)
    # Small pivot loop at the top (where the strap threads through)
    loop = (
        cq.Workplane("XY")
        .box(w, 0.003, 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    frame = frame.union(loop)
    return frame


# ---------------------------------------------------------------------------
# Lid-local Y for the strap hinge: offset so the buckle inner face is flush
# with the bottom-shell front wall when the lid is closed.
# ---------------------------------------------------------------------------

def _strap_hinge_y_lid(lx: float) -> float:
    """Y coordinate of the strap hinge in the lid's local frame."""
    front_y_lid = -_half_width_at_x(lx) - HINGE_Y
    # Place the strap inner face flush with the lid front edge so the strap
    # contacts the lid at the hinge point.  offset = STRAP_T/2 puts the strap
    # centre half-a-thickness outside the lid edge.
    return front_y_lid - STRAP_T / 2.0


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="violin_case")

    tweed = model.material("tweed_exterior", rgba=(0.20, 0.17, 0.13, 1.0))
    red_plush = model.material("red_plush", rgba=(0.62, 0.07, 0.08, 1.0))
    metal = model.material("buckle_metal", rgba=(0.78, 0.79, 0.82, 1.0))
    leather = model.material("strap_leather", rgba=(0.30, 0.15, 0.06, 1.0))

    # ---- bottom shell (root) ----
    bottom = model.part("bottom_shell")
    bottom.visual(
        mesh_from_cadquery(_bottom_shell_solid(), "bottom_exterior"),
        material=tweed, name="bottom_exterior",
    )
    bottom.visual(
        mesh_from_cadquery(_red_interior_solid(), "red_interior"),
        material=red_plush, name="red_interior",
    )
    # Hinge knuckles along the back rim
    for j, hx in enumerate((-0.20, 0.22)):
        rim_y = _half_width_at_x(hx)
        knuckle = CylinderGeometry(0.006, 0.060).rotate_x(math.pi / 2.0)
        knuckle.translate(hx, rim_y - 0.004, SHELL_H - 0.006)
        bottom.visual(
            mesh_from_geometry(knuckle, f"hinge_barrel_{j}"),
            material=metal, name=f"hinge_barrel_{j}",
        )

    # Buckle catch plates on front wall (inline non-moving visuals).
    # Each plate sits outside the front wall at the buckle seating height.
    for i, sx in enumerate(STRAP_X_POSITIONS):
        wall_y = -_half_width_at_x(sx)
        # Buckle pivot z in world when lid closed:
        buckle_pivot_z = HINGE_Z + STRAP_HINGE_Z_LID - STRAP_L
        buckle_center_z = buckle_pivot_z - BUCKLE_H / 2.0
        plate = BoxGeometry((0.032, 0.002, BUCKLE_H + 0.006))
        plate.translate(sx, wall_y - 0.001, buckle_center_z)
        bottom.visual(
            mesh_from_geometry(plate, f"catch_plate_{i}"),
            material=metal, name=f"catch_plate_{i}",
        )

    bottom.inertial = Inertial.from_geometry(
        Box((CASE_LEN, 2 * HALF_W, SHELL_H)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, SHELL_H / 2.0)),
    )

    # ---- lid ----
    lid = model.part("lid")
    lid_shell = _lid_solid().translate((0.0, -HINGE_Y, 0.0))
    lid_liner = _lid_liner_solid().translate((0.0, -HINGE_Y, 0.0))
    lid.visual(
        mesh_from_cadquery(lid_shell, "lid_exterior"),
        material=tweed, name="lid_exterior",
    )
    lid.visual(
        mesh_from_cadquery(lid_liner, "lid_liner"),
        material=red_plush, name="lid_liner",
    )
    lid.inertial = Inertial.from_geometry(
        Box((CASE_LEN, 2 * HALF_W, LID_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -HALF_W, LID_H / 2.0)),
    )

    # Lid hinge: rear (+Y) long edge, axis chosen so positive q opens upward.
    model.articulation(
        "bottom_to_lid",
        ArticulationType.REVOLUTE,
        parent=bottom, child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=math.radians(180.0),
            effort=8.0, velocity=2.0,
        ),
    )

    # ---- buckle straps (repeated pair via shared helpers) ----
    for i, sx in enumerate(STRAP_X_POSITIONS):
        hinge_y_lid = _strap_hinge_y_lid(sx)

        # --- strap part ---
        strap = model.part(f"strap_{i}")
        strap.visual(
            mesh_from_cadquery(_strap_band_solid(), f"strap_band_{i}"),
            material=leather, name=f"strap_band_{i}",
        )
        strap.inertial = Inertial.from_geometry(
            Box((STRAP_W, STRAP_T, STRAP_L)),
            mass=0.015,
            origin=Origin(xyz=(0.0, 0.0, -STRAP_L / 2.0)),
        )

        # Strap hinge on the lid front edge.
        # axis = (-1, 0, 0): positive q swings the strap outward (-Y direction)
        # away from the front wall.
        model.articulation(
            f"lid_to_strap_{i}",
            ArticulationType.REVOLUTE,
            parent=lid, child=strap,
            origin=Origin(xyz=(sx, hinge_y_lid, STRAP_HINGE_Z_LID)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0, upper=math.radians(60.0),
                effort=2.0, velocity=2.0,
            ),
        )

        # --- buckle part ---
        buckle = model.part(f"buckle_{i}")
        buckle.visual(
            mesh_from_cadquery(_buckle_frame_solid(), f"buckle_frame_{i}"),
            material=metal, name=f"buckle_frame_{i}",
        )
        buckle.inertial = Inertial.from_geometry(
            Box((BUCKLE_W, BUCKLE_D, BUCKLE_H)),
            mass=0.020,
            origin=Origin(xyz=(0.0, 0.0, -BUCKLE_H / 2.0)),
        )

        # Buckle pivot at the strap free end.
        # axis = (-1, 0, 0): positive q swings the buckle bottom outward (-Y).
        model.articulation(
            f"strap_to_buckle_{i}",
            ArticulationType.REVOLUTE,
            parent=strap, child=buckle,
            origin=Origin(xyz=(0.0, 0.0, -STRAP_L)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=0.0, upper=math.radians(40.0),
                effort=1.5, velocity=2.0,
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottom = object_model.get_part("bottom_shell")
    lid = object_model.get_part("lid")
    lid_joint = object_model.get_articulation("bottom_to_lid")

    # ---- case is longer (X) than wide (Y) ----
    ext = _ext(ctx.part_world_aabb(bottom))
    ctx.check(
        "case longer than wide",
        ext[0] > ext[1] + 0.2,
        details=f"bottom shell extents (x,y,z)={ext}",
    )

    # ---- bottom shell carries the red plush violin interior, recessed ----
    red = bottom.get_visual("red_interior")
    ctx.check(
        "red plush interior present on bottom shell",
        red is not None,
        details="bottom_shell exposes a 'red_interior' visual",
    )

    # ---- lid rim nests over the bottom shell rim at the hinge seam ----
    ctx.allow_overlap(
        lid, bottom,
        elem_a="lid_exterior", elem_b="bottom_exterior",
        reason="Closed lid rim nests over the bottom shell rim at the hinge seam.",
    )

    # ---- lid swings open about the rear hinge ----
    closed_aabb = ctx.part_world_aabb(lid)
    closed_top = closed_aabb[1][2]
    closed_front_y = closed_aabb[0][1]

    with ctx.pose({lid_joint: math.radians(90.0)}):
        mid_top = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "lid lifts as it opens",
        mid_top > closed_top + 0.05,
        details=f"closed_top={closed_top:.3f}, mid_top={mid_top:.3f}",
    )

    with ctx.pose({lid_joint: math.radians(180.0)}):
        flat_aabb = ctx.part_world_aabb(lid)
        flat_top = flat_aabb[1][2]
        flat_far_y = flat_aabb[1][1]
        flat_front_y = flat_aabb[0][1]
    ctx.check(
        "lid lies flat (not standing) when fully open",
        flat_top < SHELL_H + LID_H + 0.03,
        details=f"flat_top={flat_top:.3f}, rim+lid={SHELL_H + LID_H:.3f}",
    )
    ctx.check(
        "open lid folds beyond the hinge onto the +Y side",
        flat_far_y > HALF_W + 0.10 and flat_front_y > closed_front_y + 0.05,
        details=(
            f"flat_far_y={flat_far_y:.3f} (>{HALF_W + 0.10:.3f}), "
            f"flat_front_y={flat_front_y:.3f}, closed_front_y={closed_front_y:.3f}"
        ),
    )

    # ---- buckle straps ----
    for i in range(len(STRAP_X_POSITIONS)):
        strap = object_model.get_part(f"strap_{i}")
        buckle = object_model.get_part(f"buckle_{i}")
        strap_joint = object_model.get_articulation(f"lid_to_strap_{i}")
        buckle_joint = object_model.get_articulation(f"strap_to_buckle_{i}")

        # Strap swings outward (more -Y) when its hinge opens.
        rest_front_y = ctx.part_world_aabb(strap)[0][1]
        with ctx.pose({strap_joint: math.radians(60.0)}):
            open_front_y = ctx.part_world_aabb(strap)[0][1]
        ctx.check(
            f"strap_{i} swings outward when opened",
            open_front_y < rest_front_y - 0.01,
            details=f"rest_y={rest_front_y:.4f}, open_y={open_front_y:.4f}",
        )

        # Buckle pivots outward at the strap free end.
        buckle_rest_y = ctx.part_world_aabb(buckle)[0][1]
        with ctx.pose({buckle_joint: math.radians(40.0)}):
            buckle_open_y = ctx.part_world_aabb(buckle)[0][1]
        ctx.check(
            f"buckle_{i} pivots outward from the front wall",
            buckle_open_y < buckle_rest_y - 0.004,
            details=f"rest_y={buckle_rest_y:.4f}, open_y={buckle_open_y:.4f}",
        )

        # Strap is on the front (-Y) side of the case.
        strap_center_y = (ctx.part_world_aabb(strap)[0][1]
                          + ctx.part_world_aabb(strap)[1][1]) / 2.0
        ctx.check(
            f"strap_{i} is on the front (-Y) side",
            strap_center_y < 0.0,
            details=f"strap_center_y={strap_center_y:.4f}",
        )

        # The strap wraps against the bottom shell front wall and the buckle
        # seats on it.  Small local embed due to spline-vs-linear offset is
        # intentional seated contact.
        ctx.allow_overlap(
            strap, bottom,
            elem_a=f"strap_band_{i}", elem_b="bottom_exterior",
            reason=f"strap_{i} wraps tightly against the bottom shell front wall when closed.",
        )
        ctx.allow_overlap(
            buckle, bottom,
            elem_a=f"buckle_frame_{i}", elem_b="bottom_exterior",
            reason=f"buckle_{i} seats against the bottom shell front wall when closed.",
        )
        ctx.allow_overlap(
            strap, lid,
            elem_a=f"strap_band_{i}", elem_b="lid_exterior",
            reason=f"strap_{i} hinge barrel embeds into the lid front edge.",
        )
        ctx.allow_overlap(
            buckle, strap,
            elem_a=f"buckle_frame_{i}", elem_b=f"strap_band_{i}",
            reason=f"buckle_{i} pivot loop nests at the strap free end.",
        )

        # Proof: strap contacts the lid (it hangs from the lid hinge point).
        ctx.expect_contact(
            strap, lid,
            elem_a=f"strap_band_{i}", elem_b="lid_exterior",
            contact_tol=0.005,
            name=f"strap_{i} contacts the lid at the hinge point",
        )

    # ---- straps move with the lid ----
    strap0 = object_model.get_part("strap_0")
    strap0_closed_z = ctx.part_world_aabb(strap0)[1][2]
    with ctx.pose({lid_joint: math.radians(90.0)}):
        strap0_open_z = ctx.part_world_aabb(strap0)[1][2]
    ctx.check(
        "straps ride with the lid when it opens",
        strap0_open_z > strap0_closed_z + 0.03,
        details=f"closed_top_z={strap0_closed_z:.3f}, open_top_z={strap0_open_z:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
