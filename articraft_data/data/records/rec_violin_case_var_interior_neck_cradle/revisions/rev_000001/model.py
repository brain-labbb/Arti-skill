from __future__ import annotations

# Hardshell violin case (clamshell), lying open like the reference photo.
#
# Coordinate convention:
#   - long axis of the case (neck -> lower bout)  : +X   (case is ~0.80 m long)
#   - width axis (across the bouts)               : +Y   (~0.26 m at lower bout)
#   - up                                          : +Z   (~0.12 m tall when closed)
#   - the case sits on the table; the BOTTOM shell floor is near z=0 and its
#     rim/sealing plane is at z = SHELL_H. The lid is hinged along the back
#     long edge on the +Y side (y = +HALF_W). The latches are on the front
#     edge (y = -HALF_W).
#
# Parts / articulations:
#   - bottom_shell (ROOT): violin-silhouette tub with a RED plush recess molded
#       to a slightly-smaller violin outline. Static.
#   - lid          : matching shallow violin-silhouette shell lined red inside,
#       REVOLUTE about the rear (+Y) long-edge hinge axis, opens 0..180 deg so
#       it folds all the way back and lies FLAT beside the base like the
#       reference photo (open clamshell, both red interiors facing up).
#   - latch_0 / latch_1 : two metal clasp latches on the front (-Y) edge,
#       each a REVOLUTE flip joint, 0..~80 deg.

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
CASE_LEN = 0.80          # +X extent
HALF_W = 0.13            # half width at the lower bout (full width 0.26)
SHELL_H = 0.085          # height of the bottom shell tub (rim plane z)
LID_H = 0.045            # depth of the lid shell
WALL = 0.012             # shell wall thickness
RECESS_INSET = 0.018     # how far the red recess outline is inset from the rim

HINGE_Y = HALF_W           # hinge line along the back rim
HINGE_Z = SHELL_H          # hinge at the rim plane


def _violin_half_points(scale_w: float = 1.0, scale_x: float = 1.0):
    """Return the +Y half of a violin outline as (x, y) points, neck at small x,
    lower bout at large x. y >= 0. The outline is later mirrored to -Y.

    The profile sweeps: scroll/neck end -> upper bout -> waist (C-bouts) ->
    lower bout -> tail. Widths are fractions of HALF_W.
    """
    L = CASE_LEN
    # (x fraction along length, y half-width in metres)
    raw = [
        (0.000, 0.052),   # neck/scroll end (rounded)
        (0.060, 0.062),
        (0.140, 0.082),   # shoulder approaching upper bout
        (0.230, 0.098),   # upper bout widest
        (0.300, 0.092),
        (0.360, 0.066),   # waist (C-bout) pinch
        (0.420, 0.066),
        (0.490, 0.092),
        (0.580, 0.118),
        (0.680, 0.130),   # lower bout widest
        (0.800, 0.128),
        (0.910, 0.108),
        (0.975, 0.060),
        (1.000, 0.022),   # tail end (rounded)
    ]
    pts = []
    for fx, hy in raw:
        x = (fx - 0.5) * L * scale_x   # centre the case on x=0
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
    # upper edge: +Y points from neck to tail
    upper = [(x, y) for (x, y) in half]
    # lower edge: -Y points from tail back to neck
    lower = [(x, -y) for (x, y) in reversed(half)]
    interior = upper[1:] + [(xn, 0.0)] + lower[:-1]  # skip duplicate endpoints
    wp = (
        cq.Workplane("XY")
        .moveTo(x0, 0.0)
        .spline(interior, includeCurrent=True)
        .close()
    )
    return wp


def _bottom_shell_solid() -> cq.Workplane:
    """Violin-silhouette tub: solid base, hollowed to a violin-shaped recess."""
    outer = _violin_outline_wire().extrude(SHELL_H)
    # Inner recess: inset violin outline, leaving a floor of thickness WALL.
    recess = (
        _violin_outline_wire(scale_w=1.0 - RECESS_INSET / HALF_W,
                             scale_x=1.0 - RECESS_INSET / (CASE_LEN * 0.5))
        .extrude(SHELL_H + 0.01)
    )
    recess = recess.translate((0.0, 0.0, WALL))
    return outer.cut(recess)


def _red_interior_solid() -> cq.Workplane:
    """Plush red liner: a thin violin-shaped shell lining the recess walls+floor
    of the bottom shell, sitting inside the cavity."""
    pad = 0.006
    sw = 1.0 - (RECESS_INSET + pad) / HALF_W
    sx = 1.0 - (RECESS_INSET + pad) / (CASE_LEN * 0.5)
    floor = _violin_outline_wire(scale_w=sw, scale_x=sx).extrude(WALL * 1.4)
    floor = floor.translate((0.0, 0.0, WALL))
    # a low raised lip around the cavity wall to read as molded plush padding
    wall_outer = _violin_outline_wire(
        scale_w=1.0 - RECESS_INSET / HALF_W,
        scale_x=1.0 - RECESS_INSET / (CASE_LEN * 0.5),
    ).extrude(SHELL_H - WALL)
    wall_inner = _violin_outline_wire(scale_w=sw, scale_x=sx).extrude(SHELL_H)
    wall = wall_outer.cut(wall_inner)
    wall = wall.translate((0.0, 0.0, WALL))
    return floor.union(wall)


def _neck_cradle_solid() -> cq.Workplane:
    """Raised contoured padded block with a saddle notch for the violin neck.

    Positioned at the neck end of the cavity (negative X). The block rises from
    the cavity floor and blends into the recess walls via tapered sides and
    filleted edges. A U-shaped groove across the top cradles the violin neck.
    """
    # --- placement: neck region of the violin cavity ---
    cx = -0.30  # center X (neck/upper-bout junction area)
    block_len = 0.070  # extent along X
    half_w_at_x = _half_width_at_x(cx)
    # the cradle sits inside the recess, narrower than the cavity
    recess_scale_w = 1.0 - RECESS_INSET / HALF_W
    cavity_half_w = half_w_at_x * recess_scale_w - 0.004
    block_half_w = cavity_half_w * 0.82  # slightly narrower than cavity

    block_h = 0.032  # total height above cavity floor
    floor_z = WALL   # cavity floor z

    # --- contoured block: wider base tapering to narrower top ---
    # Build via loft from base rect to smaller top rect for the molded-plush look
    base_half_len = block_len / 2.0
    top_half_len = block_len / 2.0 - 0.008
    base_hw = block_half_w
    top_hw = block_half_w - 0.006

    block = (
        cq.Workplane("XY")
        .rect(2 * base_half_len, 2 * base_hw)
        .workplane(offset=block_h)
        .rect(2 * top_half_len, 2 * top_hw)
        .loft()
    )

    # --- saddle notch: semicircular groove along Y across the top ---
    neck_radius = 0.016  # violin neck ~30-32mm diameter
    notch_depth = 0.016
    # Cylinder along Y at the top of the block, cutting downward
    notch_cutter = (
        cq.Workplane("XZ")
        .center(0.0, block_h - notch_depth + neck_radius)
        .circle(neck_radius)
        .extrude(2 * block_half_w + 0.02, both=True)
    )
    result = block.cut(notch_cutter)

    # Fillet the top longitudinal edges for the soft plush look
    # (select edges near the top of the block)
    try:
        result = result.edges("|Y and >Z").fillet(0.004)
    except Exception:
        pass  # fillet may fail on complex topology; block is still valid

    # Position in world frame
    result = result.translate((cx, 0.0, floor_z))
    return result


def _lid_solid() -> cq.Workplane:
    """Matching shallow lid shell, hollowed so its underside reads as a red-lined
    cavity. Built in a local frame where the rim plane is z=0 and the lid rises
    in +Z; it is later mounted/rotated by the articulation."""
    outer = _violin_outline_wire().extrude(LID_H)
    inner = (
        _violin_outline_wire(scale_w=1.0 - WALL / HALF_W,
                             scale_x=1.0 - WALL / (CASE_LEN * 0.5))
        .extrude(LID_H)
    )
    inner = inner.translate((0.0, 0.0, -0.006))  # open the bottom face
    return outer.cut(inner)


def _lid_liner_solid() -> cq.Workplane:
    """Thin red liner covering the inside (underside) of the lid cavity. It fills
    the full cavity footprint (inset by WALL, matching the cavity wall) so it
    contacts the lid shell, and is thin in Z."""
    sw = 1.0 - WALL / HALF_W
    sx = 1.0 - WALL / (CASE_LEN * 0.5)
    cap = _violin_outline_wire(scale_w=sw, scale_x=sx).extrude(0.008)
    cap = cap.translate((0.0, 0.0, LID_H - 0.012))
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="violin_case")

    tweed = model.material("tweed_exterior", rgba=(0.20, 0.17, 0.13, 1.0))
    red_plush = model.material("red_plush", rgba=(0.62, 0.07, 0.08, 1.0))
    metal = model.material("latch_metal", rgba=(0.78, 0.79, 0.82, 1.0))

    # ---- bottom shell (root) ----
    bottom = model.part("bottom_shell")
    bottom.visual(
        mesh_from_cadquery(_bottom_shell_solid(), "bottom_exterior"),
        material=tweed,
        name="bottom_exterior",
    )
    bottom.visual(
        mesh_from_cadquery(_red_interior_solid(), "red_interior"),
        material=red_plush,
        name="red_interior",
    )
    # Suspension neck cradle: raised contoured padded block with saddle notch
    bottom.visual(
        mesh_from_cadquery(_neck_cradle_solid(), "neck_cradle"),
        material=red_plush,
        name="neck_cradle",
    )
    # Hinge knuckles along the back rim (two barrels seated on the rim, embedded
    # into the shell wall so they connect to the shell body).
    for j, hx in enumerate((-0.20, 0.22)):
        rim_y = _half_width_at_x(hx)
        knuckle = CylinderGeometry(0.006, 0.060).rotate_x(math.pi / 2.0)
        # sit the barrel just inside the rim so it merges with the shell wall
        knuckle.translate(hx, rim_y - 0.004, SHELL_H - 0.006)
        bottom.visual(mesh_from_geometry(knuckle, f"hinge_barrel_{j}"),
                      material=metal, name=f"hinge_barrel_{j}")
    bottom.inertial = Inertial.from_geometry(
        Box((CASE_LEN, 2 * HALF_W, SHELL_H)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, SHELL_H / 2.0)),
    )

    # ---- lid: revolute about the rear (+Y) long-edge hinge ----
    # Lid geometry is authored in the lid's local frame so that, at q=0 (closed),
    # it sits directly above the bottom rim. The hinge origin is the rear rim
    # edge; the lid body extends from the hinge toward -Y and upward.
    lid = model.part("lid")
    # Translate the lid solids so the hinge edge (y=+HALF_W, z=rim) is the local
    # origin: shift by (-0, -HINGE_Y, 0). The lid then occupies y<=0-ish and z>=0.
    lid_shell = _lid_solid().translate((0.0, -HINGE_Y, 0.0))
    lid_liner = _lid_liner_solid().translate((0.0, -HINGE_Y, 0.0))
    lid.visual(mesh_from_cadquery(lid_shell, "lid_exterior"),
               material=tweed, name="lid_exterior")
    lid.visual(mesh_from_cadquery(lid_liner, "lid_liner"),
               material=red_plush, name="lid_liner")
    lid.inertial = Inertial.from_geometry(
        Box((CASE_LEN, 2 * HALF_W, LID_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -HALF_W, LID_H / 2.0)),
    )
    # Axis along the back long edge (-X). With the lid body extending toward -Y
    # and +Z from the hinge, positive q lifts the free (-Y) edge up and swings it
    # back over the hinge. At q=180 deg the lid has folded fully over and lies
    # flat on the far (+Y) side of the hinge, red liner up, coplanar with the
    # base -- the open-book pose in the reference photo.
    model.articulation(
        "bottom_to_lid",
        ArticulationType.REVOLUTE,
        parent=bottom,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.radians(180.0),
                                   effort=8.0, velocity=2.0),
    )

    # ---- two clasp latches on the front (-Y) edge ----
    # Each latch is a small metal flip lever hinged on the bottom shell's front
    # rim. Authored in a local frame with the pivot at the local origin; the body
    # extends up in +Z (closed against the rim) and flips outward (-Y) when open.
    # Two latches near the front edge of the lower bout (the widest, flat-front
    # region) where real cases mount their clasps.
    latch_positions = (0.10, 0.30)
    for i, lx in enumerate(latch_positions):
        # actual front-wall y for this x (negative side); seat the pivot on it.
        wall_y = -_half_width_at_x(lx)
        latch = model.part(f"latch_{i}")
        # lever plate standing against the front wall (closed state), sitting just
        # outside the wall (-Y side) so it does not invade the plush cavity.
        lever = BoxGeometry((0.030, 0.007, 0.032)).translate(0.0, -0.0055, 0.016)
        # hook lip at the top that reaches over the lid edge (clasp grip)
        hook = BoxGeometry((0.030, 0.013, 0.006)).translate(0.0, -0.0035, 0.031)
        lever.merge(hook)
        # small pivot pin across the lever base, embedded into the shell wall
        pin = CylinderGeometry(0.004, 0.040).rotate_y(math.pi / 2.0)
        lever.merge(pin)
        latch.visual(mesh_from_geometry(lever, f"latch_body_{i}"),
                     material=metal, name=f"latch_body_{i}")
        latch.inertial = Inertial.from_geometry(
            Box((0.040, 0.016, 0.040)), mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, 0.017)),
        )
        # pivot sits on the front rim of the bottom shell, embedded a few mm in.
        model.articulation(
            f"bottom_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=bottom,
            child=latch,
            origin=Origin(xyz=(lx, wall_y + 0.002, SHELL_H - 0.020)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=math.radians(80.0),
                                       effort=2.0, velocity=2.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottom = object_model.get_part("bottom_shell")
    lid = object_model.get_part("lid")
    latch0 = object_model.get_part("latch_0")
    latch1 = object_model.get_part("latch_1")
    lid_joint = object_model.get_articulation("bottom_to_lid")
    latch0_joint = object_model.get_articulation("bottom_to_latch_0")
    latch1_joint = object_model.get_articulation("bottom_to_latch_1")

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

    # ---- suspension neck cradle: raised padded block with saddle notch ----
    cradle = bottom.get_visual("neck_cradle")
    ctx.check(
        "neck cradle present in bottom shell cavity",
        cradle is not None,
        details="bottom_shell exposes a 'neck_cradle' visual",
    )
    # Cradle must be at the neck end (negative X) of the case
    cradle_aabb = ctx.part_element_world_aabb(bottom, elem="neck_cradle")
    if cradle_aabb is not None:
        cradle_cx = (cradle_aabb[0][0] + cradle_aabb[1][0]) / 2.0
        cradle_min_z = cradle_aabb[0][2]
        cradle_max_z = cradle_aabb[1][2]
        ctx.check(
            "neck cradle positioned at neck end (negative X)",
            cradle_cx < -0.10,
            details=f"cradle center x={cradle_cx:.3f} (should be < -0.10)",
        )
        ctx.check(
            "neck cradle rises above cavity floor",
            cradle_max_z > WALL + 0.020,
            details=f"cradle top z={cradle_max_z:.4f}, floor+0.02={WALL + 0.020:.4f}",
        )
        ctx.check(
            "neck cradle seated on cavity floor",
            cradle_min_z < WALL + 0.005,
            details=f"cradle bottom z={cradle_min_z:.4f}, floor={WALL:.4f}",
        )
    # Cradle must be contained within the bottom shell footprint
    ctx.expect_within(
        bottom, bottom,
        axes="xy",
        inner_elem="neck_cradle",
        outer_elem="bottom_exterior",
        margin=0.005,
        name="neck cradle contained within bottom shell footprint",
    )

    # ---- lid is seated above the bottom rim and overlaps it (hinge seam) ----
    ctx.allow_overlap(
        lid, bottom,
        elem_a="lid_exterior", elem_b="bottom_exterior",
        reason="Closed lid rim nests over the bottom shell rim at the hinge seam.",
    )

    # ---- lid swings open about the rear hinge ----
    closed_aabb = ctx.part_world_aabb(lid)
    closed_top = closed_aabb[1][2]
    closed_front_y = closed_aabb[0][1]  # most -Y extent when closed
    # mid-travel: the free (-Y) edge lifts up in +Z as the lid rises.
    with ctx.pose({lid_joint: math.radians(90.0)}):
        mid_top = ctx.part_world_aabb(lid)[1][2]
    ctx.check(
        "lid lifts as it opens",
        mid_top > closed_top + 0.05,
        details=f"closed_top={closed_top:.3f}, mid_top={mid_top:.3f}",
    )
    # fully open (180 deg): the lid folds all the way over and lies FLAT on the
    # far (+Y) side of the hinge, level with the base -- the reference open pose.
    with ctx.pose({lid_joint: math.radians(180.0)}):
        flat_aabb = ctx.part_world_aabb(lid)
        flat_top = flat_aabb[1][2]
        flat_far_y = flat_aabb[1][1]   # most +Y extent when folded back
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

    # ---- each clasp latch flips about its front-edge hinge ----
    for latch, joint, name in (
        (latch0, latch0_joint, "latch_0"),
        (latch1, latch1_joint, "latch_1"),
    ):
        rest_y = ctx.part_world_aabb(latch)[0][1]   # -Y extent at rest
        with ctx.pose({joint: math.radians(80.0)}):
            flip_y = ctx.part_world_aabb(latch)[0][1]
        ctx.check(
            f"{name} flips outward about its hinge",
            flip_y < rest_y - 0.01,
            details=f"rest_y={rest_y:.4f}, flip_y={flip_y:.4f}",
        )

    # latches sit on the bottom shell front rim (pivot pin embedded in the wall),
    # and their hook lips reach over the lid edge to clamp it shut.
    for latch in (latch0, latch1):
        ctx.allow_overlap(
            latch, bottom,
            reason="Latch pivot pin and lever base seat into the bottom shell front wall.",
        )
        ctx.allow_overlap(
            latch, lid,
            reason="Closed clasp hook lip reaches over the lid front edge to clamp it.",
        )

    return ctx.report()


object_model = build_object_model()
