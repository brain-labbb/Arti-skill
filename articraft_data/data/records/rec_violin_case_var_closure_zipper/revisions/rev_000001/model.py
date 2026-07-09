from __future__ import annotations

# Violin case with soft-style zipper-perimeter closure.
#
# Fork of the hardshell clamshell case: same violin-contoured outline, same
# red plush recessed interior, same overall proportions.
#
# Changed closure mechanism:
#   - zipper track around 3 sides of the violin-shaped rim (front long edge
#     + two short neck/tail ends; rear +Y long edge is the natural fold)
#   - zipper pull slider translates along the front seam (PRISMATIC)
#   - lid is a softer padded flap hinged along the rear fold
#   - no metal clasp latches
#
# Coordinates:
#   +X = long axis (neck -> tail), ~0.80 m
#   +Y = width, ~0.26 m at lower bout
#   +Z = up
#   Rim at z = SHELL_H.  Rear hinge at y = +HALF_W.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- dimensions (metres, same as parent) ----
CASE_LEN = 0.80
HALF_W = 0.13
SHELL_H = 0.085
LID_H = 0.045
WALL = 0.012
RECESS_INSET = 0.018

HINGE_Y = HALF_W
HINGE_Z = SHELL_H

# Zipper parameters
TRACK_RADIUS = 0.003          # tube radius for zipper track cord
TRACK_Z = SHELL_H - 0.001     # track centre slightly below rim for shell embedding
TEETH_COUNT = 8               # zipper teeth along the front edge
TOOTH_SIZE = (0.003, 0.005, 0.005)  # wide+tall enough to overlap track and shell
STOPPER_SIZE = (0.006, 0.006, 0.005)
PULL_TRAVEL_HALF = 0.18       # half-travel in metres from centre


# ---- violin outline helpers (identical to parent) ----

def _violin_half_points(scale_w: float = 1.0, scale_x: float = 1.0):
    """Return the +Y half of a violin outline as (x, y) points."""
    L = CASE_LEN
    raw = [
        (0.000, 0.052), (0.060, 0.062), (0.140, 0.082),
        (0.230, 0.098), (0.300, 0.092), (0.360, 0.066),
        (0.420, 0.066), (0.490, 0.092), (0.580, 0.118),
        (0.680, 0.130), (0.800, 0.128), (0.910, 0.108),
        (0.975, 0.060), (1.000, 0.022),
    ]
    return [((fx - 0.5) * L * scale_x, hy * scale_w) for fx, hy in raw]


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
    upper = [(x, y) for x, y in half]
    lower = [(x, -y) for x, y in reversed(half)]
    interior = upper[1:] + [(xn, 0.0)] + lower[:-1]
    return (
        cq.Workplane("XY")
        .moveTo(x0, 0.0)
        .spline(interior, includeCurrent=True)
        .close()
    )


# ---- shell geometry (identical to parent) ----

def _bottom_shell_solid() -> cq.Workplane:
    """Violin-silhouette tub: solid base, hollowed to a violin-shaped recess."""
    outer = _violin_outline_wire().extrude(SHELL_H)
    recess = (
        _violin_outline_wire(
            scale_w=1.0 - RECESS_INSET / HALF_W,
            scale_x=1.0 - RECESS_INSET / (CASE_LEN * 0.5),
        )
        .extrude(SHELL_H + 0.01)
    )
    recess = recess.translate((0.0, 0.0, WALL))
    return outer.cut(recess)


def _red_interior_solid() -> cq.Workplane:
    """Plush red liner inside the bottom shell cavity."""
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


def _lid_solid() -> cq.Workplane:
    """Shallow lid shell, hollowed so its underside reads as a cavity."""
    outer = _violin_outline_wire().extrude(LID_H)
    inner = (
        _violin_outline_wire(
            scale_w=1.0 - WALL / HALF_W,
            scale_x=1.0 - WALL / (CASE_LEN * 0.5),
        )
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


# ---- NEW: padded lid exterior ----

def _lid_padding_solid() -> cq.Workplane:
    """Thin padded layer on the lid exterior for a soft/puffy look."""
    pad = _violin_outline_wire(scale_w=0.88, scale_x=0.88).extrude(0.008)
    pad = pad.translate((0.0, 0.0, LID_H - 0.001))
    return pad


# ---- NEW: zipper track path ----

def _zipper_track_path_3d():
    """3D path for the zipper track around 3 sides of the violin rim.

    Starts on the +Y side near the neck, curves around the neck end, follows
    the -Y front edge to the tail, curves around the tail end, and ends on the
    +Y side near the tail.  The rear +Y long edge (hinge fold) is skipped.
    """
    half = _violin_half_points()
    z = TRACK_Z

    neck_fringe = [(x, y, z) for x, y in half[:2]]
    tail_fringe = [(x, y, z) for x, y in half[-2:]]
    front_edge = [(x, -y, z) for x, y in half]

    x_neck = half[0][0]
    x_tail = half[-1][0]

    return (
        list(reversed(neck_fringe))
        + [(x_neck, 0.0, z)]
        + front_edge
        + [(x_tail, 0.0, z)]
        + list(reversed(tail_fringe))
    )


# ---- NEW: front-edge interpolation ----

def _front_edge_point(t: float):
    """Interpolate a point on the front (-Y) edge.  t=0 neck, t=1 tail."""
    pts = _violin_half_points()
    idx_f = t * (len(pts) - 1)
    idx = min(int(idx_f), len(pts) - 2)
    frac = idx_f - idx
    x0, y0 = pts[idx]
    x1, y1 = pts[idx + 1]
    x = x0 + frac * (x1 - x0)
    y = -(y0 + frac * (y1 - y0))
    return (x, y)


# ---- NEW: shared sub-part geometry helpers ----

def _zipper_tooth_geometry():
    """Shared geometry for one zipper tooth."""
    return BoxGeometry(TOOTH_SIZE)


def _zipper_stopper_geometry():
    """Shared geometry for one zipper end stopper."""
    return BoxGeometry(STOPPER_SIZE)


def _zipper_pull_body():
    """Zipper pull slider body with grip tab hanging down the front face."""
    body = BoxGeometry((0.014, 0.010, 0.006))
    # Pull tab hanging down the front (-Y) face of the case
    tab = BoxGeometry((0.010, 0.002, 0.012)).translate(0.0, -0.006, -0.004)
    body.merge(tab)
    return body


# ---- build ----

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="violin_case_zipper")

    # Materials
    tweed = model.material("tweed_exterior", rgba=(0.20, 0.17, 0.13, 1.0))
    red_plush = model.material("red_plush", rgba=(0.62, 0.07, 0.08, 1.0))
    metal = model.material("zipper_metal", rgba=(0.72, 0.73, 0.76, 1.0))
    padded_nylon = model.material("padded_nylon", rgba=(0.14, 0.14, 0.17, 1.0))
    track_fabric = model.material("track_fabric", rgba=(0.10, 0.10, 0.12, 1.0))

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

    # Zipper track: thin tube around 3 sides of the rim (inline decoration)
    track_mesh = tube_from_spline_points(
        _zipper_track_path_3d(),
        radius=TRACK_RADIUS,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    bottom.visual(
        mesh_from_geometry(track_mesh, "zipper_track"),
        material=track_fabric, name="zipper_track",
    )

    # Zipper teeth along the front edge (repeated inline visuals via loop)
    tooth_z = SHELL_H - 0.002  # embedded deeper for shell connectivity
    for i in range(TEETH_COUNT):
        t = (i + 0.5) / TEETH_COUNT
        tx, ty = _front_edge_point(t)
        tooth = _zipper_tooth_geometry().translate(tx, ty, tooth_z)
        bottom.visual(
            mesh_from_geometry(tooth, f"zipper_tooth_{i}"),
            material=metal, name=f"zipper_tooth_{i}",
        )

    # Zipper end stoppers (repeated inline visuals via loop)
    for i in range(2):
        t = 0.03 if i == 0 else 0.97
        sx, sy = _front_edge_point(t)
        stopper = _zipper_stopper_geometry().translate(sx, sy, SHELL_H - 0.002)
        bottom.visual(
            mesh_from_geometry(stopper, f"zipper_stopper_{i}"),
            material=metal, name=f"zipper_stopper_{i}",
        )

    bottom.inertial = Inertial.from_geometry(
        Box((CASE_LEN, 2 * HALF_W, SHELL_H)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, SHELL_H / 2.0)),
    )

    # ---- lid: softer padded flap ----
    lid = model.part("lid")
    lid_shell = _lid_solid().translate((0.0, -HINGE_Y, 0.0))
    lid_liner = _lid_liner_solid().translate((0.0, -HINGE_Y, 0.0))
    lid_pad = _lid_padding_solid().translate((0.0, -HINGE_Y, 0.0))

    lid.visual(mesh_from_cadquery(lid_shell, "lid_exterior"),
               material=padded_nylon, name="lid_exterior")
    lid.visual(mesh_from_cadquery(lid_liner, "lid_liner"),
               material=red_plush, name="lid_liner")
    lid.visual(mesh_from_cadquery(lid_pad, "lid_padding"),
               material=padded_nylon, name="lid_padding")

    lid.inertial = Inertial.from_geometry(
        Box((CASE_LEN, 2 * HALF_W, LID_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -HALF_W, LID_H / 2.0)),
    )

    # Rear hinge: natural fold along the +Y long edge, opens 0..180 deg
    model.articulation(
        "bottom_to_lid",
        ArticulationType.REVOLUTE,
        parent=bottom, child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=math.radians(180.0),
            effort=6.0, velocity=2.0,
        ),
    )

    # ---- zipper pull: prismatic along front seam ----
    zipper_pull = model.part("zipper_pull")
    zipper_pull.visual(
        mesh_from_geometry(_zipper_pull_body(), "pull_body"),
        material=metal, name="pull_body",
    )
    zipper_pull.inertial = Inertial.from_geometry(
        Box((0.020, 0.014, 0.024)), mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    # Pull sits on the track at the centre of the front edge
    pull_y = -_half_width_at_x(0.0)
    pull_z = SHELL_H + 0.004  # above the track crown
    model.articulation(
        "bottom_to_zipper_pull",
        ArticulationType.PRISMATIC,
        parent=bottom, child=zipper_pull,
        origin=Origin(xyz=(0.0, pull_y, pull_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-PULL_TRAVEL_HALF, upper=PULL_TRAVEL_HALF,
            effort=2.0, velocity=0.5,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottom = object_model.get_part("bottom_shell")
    lid = object_model.get_part("lid")
    zipper_pull = object_model.get_part("zipper_pull")
    lid_joint = object_model.get_articulation("bottom_to_lid")
    pull_joint = object_model.get_articulation("bottom_to_zipper_pull")

    # ---- case proportions (same as parent) ----
    ext = _ext(ctx.part_world_aabb(bottom))
    ctx.check(
        "case longer than wide",
        ext[0] > ext[1] + 0.2,
        details=f"bottom shell extents (x,y,z)={ext}",
    )

    # ---- red plush interior present ----
    ctx.check(
        "red plush interior present on bottom shell",
        bottom.get_visual("red_interior") is not None,
        details="bottom_shell exposes a 'red_interior' visual",
    )

    # ---- zipper track on the rim ----
    ctx.check(
        "zipper track present on rim",
        bottom.get_visual("zipper_track") is not None,
        details="bottom_shell exposes a 'zipper_track' visual",
    )

    # ---- zipper teeth present ----
    ctx.check(
        "zipper teeth present along front edge",
        bottom.get_visual("zipper_tooth_0") is not None,
    )

    # ---- zipper stoppers present ----
    ctx.check(
        "zipper end stoppers present",
        bottom.get_visual("zipper_stopper_0") is not None
        and bottom.get_visual("zipper_stopper_1") is not None,
    )

    # ---- no latch parts (closure mechanism changed) ----
    for name in ("latch_0", "latch_1"):
        try:
            object_model.get_part(name)
            exists = True
        except Exception:
            exists = False
        ctx.check(
            f"no {name} part (zipper replaces latches)",
            not exists,
            details=f"part '{name}' should not exist in zipper variant",
        )

    # ---- zipper pull translates along the front seam ----
    rest_pos = ctx.part_world_position(zipper_pull)
    with ctx.pose({pull_joint: PULL_TRAVEL_HALF}):
        moved_pos = ctx.part_world_position(zipper_pull)
    ctx.check(
        "zipper pull translates along front seam (+X)",
        rest_pos is not None and moved_pos is not None
        and moved_pos[0] > rest_pos[0] + PULL_TRAVEL_HALF * 0.9,
        details=f"rest_x={rest_pos}, moved_x={moved_pos}",
    )

    # ---- zipper pull on front (-Y) edge ----
    pull_aabb = ctx.part_world_aabb(zipper_pull)
    ctx.check(
        "zipper pull on front (-Y) edge",
        pull_aabb[0][1] < 0.0,
        details=f"pull AABB min_y={pull_aabb[0][1]:.3f}",
    )

    # ---- lid opens via rear hinge and lies flat ----
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
        "lid lies flat when fully open",
        flat_top < SHELL_H + LID_H + 0.03,
        details=f"flat_top={flat_top:.3f}, rim+lid={SHELL_H + LID_H:.3f}",
    )
    ctx.check(
        "open lid folds beyond hinge onto +Y side",
        flat_far_y > HALF_W + 0.10 and flat_front_y > closed_front_y + 0.05,
        details=(
            f"flat_far_y={flat_far_y:.3f} (>{HALF_W + 0.10:.3f}), "
            f"flat_front_y={flat_front_y:.3f}, closed_front_y={closed_front_y:.3f}"
        ),
    )

    # ---- padded lid exterior ----
    ctx.check(
        "lid has padded exterior layer",
        lid.get_visual("lid_exterior") is not None
        and lid.get_visual("lid_padding") is not None,
        details="lid exposes 'lid_exterior' and 'lid_padding' visuals",
    )

    # ---- overlap allowances ----
    ctx.allow_overlap(
        lid, bottom,
        elem_a="lid_exterior", elem_b="bottom_exterior",
        reason="Closed lid rim nests over the bottom shell rim at the rear fold seam.",
    )
    ctx.allow_overlap(
        lid, zipper_pull,
        elem_a="lid_exterior", elem_b="pull_body",
        reason="Closed lid covers the zipper pull sitting on the front rim track.",
    )
    ctx.allow_overlap(
        zipper_pull, bottom,
        elem_a="pull_body", elem_b="zipper_track",
        reason="Zipper pull slider seats onto the track with local contact embedding.",
    )
    ctx.allow_overlap(
        zipper_pull, bottom,
        elem_a="pull_body", elem_b="bottom_exterior",
        reason="Zipper pull tab hangs below the track into the shell rim region.",
    )

    # Prove the pull stays on the track in XY
    ctx.expect_overlap(
        zipper_pull, bottom,
        axes="xy",
        elem_a="pull_body", elem_b="zipper_track",
        min_overlap=0.002,
        name="zipper pull overlaps the track in XY",
    )

    # Prove the closed lid covers the pull (pull is under the lid)
    ctx.expect_overlap(
        lid, zipper_pull,
        axes="xy",
        elem_a="lid_exterior", elem_b="pull_body",
        min_overlap=0.001,
        name="closed lid covers the zipper pull in XY",
    )

    return ctx.report()


object_model = build_object_model()
