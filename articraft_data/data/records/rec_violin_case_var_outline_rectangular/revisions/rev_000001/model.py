from __future__ import annotations

# Hardshell rectangular violin case (clamshell), lying open.
#
# Coordinate convention:
#   - long axis of the case                          : +X   (case is ~0.80 m long)
#   - width axis                                     : +Y   (~0.26 m wide)
#   - up                                             : +Z   (~0.13 m tall when closed)
#   - the case sits on the table; the BOTTOM shell floor is near z=0 and its
#     rim/sealing plane is at z = SHELL_H. The lid is hinged along the back
#     long edge on the +Y side (y = +HALF_W). The latches are on the front
#     edge (y = -HALF_W).
#
# Parts / articulations:
#   - bottom_shell (ROOT): rounded-rectangle tub with a RED plush recess molded
#       to a slightly-smaller rectangular cavity. Static.
#   - lid          : matching shallow rounded-rectangle shell lined red inside,
#       REVOLUTE about the rear (+Y) long-edge hinge axis, opens 0..180 deg so
#       it folds all the way back and lies FLAT beside the base (open clamshell,
#       both red interiors facing up).
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
HALF_W = 0.13            # half width (full width 0.26)
SHELL_H = 0.085          # height of the bottom shell tub (rim plane z)
LID_H = 0.045            # depth of the lid shell
WALL = 0.012             # shell wall thickness
RECESS_INSET = 0.018     # how far the red recess outline is inset from the rim
CORNER_R = 0.025         # corner fillet radius for the rounded rectangle

HINGE_Y = HALF_W           # hinge line along the back rim
HINGE_Z = SHELL_H          # hinge at the rim plane


def _rounded_rect_solid(length: float, width: float, height: float,
                        corner_r: float) -> cq.Workplane:
    """Create a rounded-rectangle solid centered at origin, base at z=0.

    The rectangle is centered on XY with the given length (+X) and width (+Y).
    Vertical edges are filleted to produce rounded corners.
    """
    r = min(corner_r, length / 2.0 - 0.001, width / 2.0 - 0.001)
    r = max(r, 0.001)
    return (
        cq.Workplane("XY")
        .rect(length, width)
        .extrude(height)
        .edges("|Z")
        .fillet(r)
    )


def _bottom_shell_solid() -> cq.Workplane:
    """Rounded-rectangle tub: solid base, hollowed to a rectangular recess."""
    outer = _rounded_rect_solid(CASE_LEN, 2 * HALF_W, SHELL_H, CORNER_R)
    # Inner recess: inset rectangle, leaving a floor of thickness WALL.
    inner_len = CASE_LEN - 2 * RECESS_INSET
    inner_w = 2 * HALF_W - 2 * RECESS_INSET
    inner_r = max(CORNER_R - RECESS_INSET, 0.005)
    recess = _rounded_rect_solid(inner_len, inner_w, SHELL_H + 0.01, inner_r)
    recess = recess.translate((0.0, 0.0, WALL))
    return outer.cut(recess)


def _red_interior_solid() -> cq.Workplane:
    """Plush red liner: a thin rectangular shell lining the recess walls+floor
    of the bottom shell, sitting inside the cavity."""
    pad = 0.006
    inner_len = CASE_LEN - 2 * (RECESS_INSET + pad)
    inner_w = 2 * HALF_W - 2 * (RECESS_INSET + pad)
    inner_r = max(CORNER_R - RECESS_INSET - pad, 0.004)
    # Floor pad
    floor = _rounded_rect_solid(inner_len, inner_w, WALL * 1.4, inner_r)
    floor = floor.translate((0.0, 0.0, WALL))
    # Raised lip around the cavity wall to read as molded plush padding
    outer_recess_len = CASE_LEN - 2 * RECESS_INSET
    outer_recess_w = 2 * HALF_W - 2 * RECESS_INSET
    outer_r = max(CORNER_R - RECESS_INSET, 0.005)
    wall_outer = _rounded_rect_solid(outer_recess_len, outer_recess_w,
                                     SHELL_H - WALL, outer_r)
    wall_inner = _rounded_rect_solid(inner_len, inner_w, SHELL_H, inner_r)
    wall = wall_outer.cut(wall_inner)
    wall = wall.translate((0.0, 0.0, WALL))
    return floor.union(wall)


def _lid_solid() -> cq.Workplane:
    """Matching shallow lid shell, hollowed so its underside reads as a red-lined
    cavity. Built in a local frame where the rim plane is z=0 and the lid rises
    in +Z; it is later mounted/rotated by the articulation."""
    outer = _rounded_rect_solid(CASE_LEN, 2 * HALF_W, LID_H, CORNER_R)
    inner_len = CASE_LEN - 2 * WALL
    inner_w = 2 * HALF_W - 2 * WALL
    inner_r = max(CORNER_R - WALL, 0.005)
    inner = _rounded_rect_solid(inner_len, inner_w, LID_H, inner_r)
    inner = inner.translate((0.0, 0.0, -0.006))  # open the bottom face
    return outer.cut(inner)


def _lid_liner_solid() -> cq.Workplane:
    """Thin red liner covering the inside (underside) of the lid cavity. It fills
    the full cavity footprint (inset by WALL, matching the cavity wall) so it
    contacts the lid shell, and is thin in Z."""
    inner_len = CASE_LEN - 2 * WALL
    inner_w = 2 * HALF_W - 2 * WALL
    inner_r = max(CORNER_R - WALL, 0.005)
    cap = _rounded_rect_solid(inner_len, inner_w, 0.008, inner_r)
    cap = cap.translate((0.0, 0.0, LID_H - 0.012))
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="violin_case_rect")

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
    # Hinge knuckles along the back rim (two barrels seated on the rim, embedded
    # into the shell wall so they connect to the shell body).
    for i, hx in enumerate((-0.20, 0.22)):
        knuckle = CylinderGeometry(0.006, 0.060).rotate_x(math.pi / 2.0)
        # sit the barrel just inside the rim so it merges with the shell wall
        knuckle.translate(hx, HALF_W - 0.004, SHELL_H - 0.006)
        bottom.visual(mesh_from_geometry(knuckle, f"hinge_barrel_{i}"),
                      material=metal, name=f"hinge_barrel_{i}")
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
    # origin: shift by (0, -HINGE_Y, 0). The lid then occupies y<=0-ish and z>=0.
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
    # base -- the open-book pose.
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
    latch_positions = (-0.15, 0.15)
    for i, lx in enumerate(latch_positions):
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
        wall_y = -HALF_W
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

    # ---- case is longer (X) than wide (Y) and has rectangular footprint ----
    ext = _ext(ctx.part_world_aabb(bottom))
    ctx.check(
        "case longer than wide",
        ext[0] > ext[1] + 0.2,
        details=f"bottom shell extents (x,y,z)={ext}",
    )
    # Rectangular footprint: width ratio should be close to the design ratio
    expected_ratio = CASE_LEN / (2 * HALF_W)
    actual_ratio = ext[0] / ext[1]
    ctx.check(
        "rectangular oblong footprint proportions",
        abs(actual_ratio - expected_ratio) < 0.3,
        details=f"expected~{expected_ratio:.2f}, actual={actual_ratio:.2f}",
    )

    # ---- bottom shell carries the red plush rectangular interior, recessed ----
    red = bottom.get_visual("red_interior")
    ctx.check(
        "red plush interior present on bottom shell",
        red is not None,
        details="bottom_shell exposes a 'red_interior' visual",
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
