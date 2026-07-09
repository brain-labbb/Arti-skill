from __future__ import annotations

"""Traditional cast-iron cooking cauldron with a side-hinged flip lid.

Variant (hinged_flip): the lid pivots open on a REVOLUTE hinge fixed to the
pot rim instead of lifting straight up.  The swing bail handle remains REVOLUTE.

Structure:
- pot_body: hollow round-bellied vessel (single revolved wall profile) that
  bulges to ~0.45 m at the belly, necks in to a 0.28 m open mouth with visible
  ~12 mm wall thickness at the rim, and stands on an integral cylindrical
  pedestal foot (0.20 m dia × ~0.04 m tall).  A cast hinge lug rises from the
  back rim carrying a horizontal barrel.
- lid: shallow domed disc, 0.30 m dia, with a stepped rim that overhangs the
  mouth and a hollow cast underside.  A hinge strap wraps around the pot barrel
  at the back.  Two small cast bosses at the apex carry the handle pivot.
- loop_handle: arched semicircular loop whose pin ends are captured in the two
  lid bosses.

Articulations:
- lid_hinge: REVOLUTE about +X at the back rim; 0 rad = closed, positive q
  flips the front edge upward to open (~2.0 rad).
- handle_pivot: REVOLUTE about the horizontal boss axis (+X); 0 rad lies flat
  against the lid dome, +1.57 rad stands the loop upright.
"""

from functools import lru_cache
import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
POT_FOOT_R = 0.100  # pedestal foot radius (0.20 m diameter)
POT_FOOT_H = 0.042  # pedestal foot height
POT_BELLY_R = 0.225  # widest belly radius (0.45 m diameter)
POT_RIM_Z = 0.310  # pot mouth rim height
POT_RIM_OUTER_R = 0.140  # mouth outer radius (0.28 m mouth diameter)
POT_RIM_INNER_R = 0.128  # mouth inner radius -> 12 mm visible wall at the rim

LID_R = 0.150  # lid outer radius (0.30 m diameter, overhangs the mouth)
LID_APEX_Z = 0.060  # dome apex above the seat plane

BOSS_X = 0.033  # boss centers flank the apex along +-X
BOSS_SIZE = (0.016, 0.018, 0.020)
BOSS_Z = 0.064  # boss center height (dome local); base embeds into the dome
PIVOT_Z = 0.068  # handle pivot axis height (dome local)

LOOP_R = 0.030  # loop arch radius
LOOP_ROD_R = 0.006  # loop rod radius
PIN_R = 0.005  # pivot pin radius
PIN_X0 = 0.026  # pin start (just inside the loop end)
PIN_X1 = 0.040  # pin end (buried in the boss)

# ---- hinge geometry ---------------------------------------------------------
HINGE_BARREL_R = 0.007  # barrel outer radius
HINGE_BARREL_LEN = 0.042  # barrel length along X
# Place the hinge well outside the dome edge (LID_R = 0.150) so the hinge lug
# does not overlap the dome when the lid is closed.
HINGE_Y = -(LID_R + 0.015)  # barrel centre 15 mm outside the dome edge
HINGE_Z_OFFSET = 0.010  # barrel centre 10 mm above the rim top
HINGE_Z = POT_RIM_Z + HINGE_Z_OFFSET

# In the lid part frame (which coincides with the hinge frame at q=0) the dome
# centre is offset by:
LID_OFFSET_Y = -HINGE_Y  # forward from hinge to dome centre
LID_OFFSET_Z = POT_RIM_Z - HINGE_Z  # = -HINGE_Z_OFFSET

# Hinge lug on pot_body — offset outward so the inner face stays clear of the
# dome.  The lug extends deep enough to embed into the pot outer wall.
LUG_W = 0.042  # lug width along X
LUG_D = 0.016  # lug depth along Y (narrow to stay outside dome)
LUG_CY = HINGE_Y - 0.002  # offset 2 mm outward from barrel centre
LUG_BOTTOM_Z = 0.280  # embed into the pot belly wall for connectivity
LUG_TOP_Z = HINGE_Z + HINGE_BARREL_R + 0.003

# Hinge strap on lid
STRAP_W = 0.030  # strap width along X
STRAP_T = 0.005  # plate thickness
STRAP_WRAP_IR = HINGE_BARREL_R - 0.001  # 1 mm interference on barrel
STRAP_WRAP_OR = HINGE_BARREL_R + STRAP_T + 0.001
STRAP_PLATE_LEN = 0.055  # plate extends from wrap toward dome

HANDLE_UP = 1.5708  # handle flat -> upright
LID_OPEN = 2.0  # lid opens ~115 degrees

CAST_IRON = Material(name="cast_iron_black", rgba=(0.10, 0.10, 0.11, 1.0))
CAST_IRON_LID = Material(name="cast_iron_lid", rgba=(0.13, 0.13, 0.14, 1.0))
CAST_IRON_DARK = Material(name="cast_iron_dark", rgba=(0.08, 0.08, 0.09, 1.0))
CAST_IRON_HINGE = Material(name="cast_iron_hinge", rgba=(0.09, 0.09, 0.10, 1.0))


# ---------------------------------------------------------------- geometry
@lru_cache(maxsize=1)
def _pot_solid() -> cq.Workplane:
    """Hollow bulbous pot wall + integral pedestal foot, one revolved profile."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(POT_FOOT_R, 0.0)
        .lineTo(POT_FOOT_R, POT_FOOT_H)
        .lineTo(0.085, 0.046)
        .spline(
            [
                (0.150, 0.075),
                (0.210, 0.135),
                (POT_BELLY_R, 0.185),
                (0.205, 0.250),
                (0.155, 0.295),
                (POT_RIM_OUTER_R, POT_RIM_Z),
            ],
            includeCurrent=True,
        )
        .lineTo(POT_RIM_INNER_R, POT_RIM_Z)
        .spline(
            [
                (0.142, 0.293),
                (0.193, 0.250),
                (0.213, 0.185),
                (0.198, 0.135),
                (0.138, 0.078),
                (0.070, 0.052),
                (0.0, 0.044),
            ],
            includeCurrent=True,
        )
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


@lru_cache(maxsize=1)
def _pot_filled_solid() -> cq.Workplane:
    """Same outer silhouette but solid: only used to prove hollowness in tests."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(POT_FOOT_R, 0.0)
        .lineTo(POT_FOOT_R, POT_FOOT_H)
        .lineTo(0.085, 0.046)
        .spline(
            [
                (0.150, 0.075),
                (0.210, 0.135),
                (POT_BELLY_R, 0.185),
                (0.205, 0.250),
                (0.155, 0.295),
                (POT_RIM_OUTER_R, POT_RIM_Z),
            ],
            includeCurrent=True,
        )
        .lineTo(0.0, POT_RIM_Z)
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


@lru_cache(maxsize=1)
def _hinge_lug_solid() -> cq.Workplane:
    """Cast hinge lug on the pot rim: vertical plate + barrel cylinder.

    The plate embeds slightly into the rim for geometric connectivity and
    rises to carry the barrel at HINGE_Z.
    """
    lug_h = LUG_TOP_Z - LUG_BOTTOM_Z
    lug_cz = (LUG_TOP_Z + LUG_BOTTOM_Z) / 2.0
    # Vertical plate (offset outward so inner face clears the dome edge)
    plate = (
        cq.Workplane("XY")
        .workplane(offset=lug_cz)
        .center(0.0, LUG_CY)
        .box(LUG_W, LUG_D, lug_h)
    )
    # Barrel cylinder along X at the hinge point
    barrel = (
        cq.Workplane("YZ")
        .workplane(offset=-HINGE_BARREL_LEN / 2.0)
        .center(HINGE_Y, HINGE_Z)
        .circle(HINGE_BARREL_R)
        .extrude(HINGE_BARREL_LEN)
    )
    return plate.union(barrel)


@lru_cache(maxsize=1)
def _lid_solid() -> cq.Workplane:
    """Domed lid disc: stepped overhanging rim, hollow cast underside.

    Built at the origin in its own local frame; the visual origin offsets
    it into the lid part frame.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(LID_R, 0.0)
        .lineTo(LID_R, 0.014)
        .lineTo(0.126, 0.020)
        .spline([(0.105, 0.032), (0.060, 0.052), (0.0, LID_APEX_Z)], includeCurrent=True)
        .lineTo(0.0, 0.034)
        .spline([(0.060, 0.027), (0.100, 0.014), (0.120, 0.0)], includeCurrent=True)
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


@lru_cache(maxsize=1)
def _hinge_strap_solid() -> cq.Workplane:
    """Hinge strap on the lid: cylindrical wrap + flat connecting plate.

    Built in the lid part frame.  The wrap ring sits at the hinge origin
    (0, 0, 0); the plate extends in +Y toward the dome surface so the two
    lid visuals form one connected assembly.
    """
    # Cylindrical ring (wrap) at the hinge origin, along X
    wrap = (
        cq.Workplane("YZ")
        .workplane(offset=-STRAP_W / 2.0)
        .center(0.0, 0.0)
        .circle(STRAP_WRAP_OR)
        .circle(STRAP_WRAP_IR)
        .extrude(STRAP_W)
    )
    # Flat plate from wrap toward dome centre (+Y in lid frame).
    # Plate centred at z = LID_OFFSET_Z + STRAP_T/2 so the underside is
    # flush with the dome seat plane, guaranteeing overlap with the dome.
    plate_cz = LID_OFFSET_Z + STRAP_T / 2.0
    plate_cy = STRAP_PLATE_LEN / 2.0
    plate = (
        cq.Workplane("XY")
        .workplane(offset=plate_cz)
        .center(0.0, plate_cy)
        .box(STRAP_W, STRAP_PLATE_LEN, STRAP_T)
    )
    return wrap.union(plate)


@lru_cache(maxsize=1)
def _handle_solid() -> cq.Workplane:
    """Semicircular loop (half torus) with pin ends along the X pivot axis."""
    loop = (
        cq.Workplane("XZ")
        .moveTo(LOOP_R, 0.0)
        .circle(LOOP_ROD_R)
        .revolve(180.0, (0, -1, 0), (0, 1, 0))
    )
    pin_pos = cq.Workplane("YZ").workplane(offset=PIN_X0).circle(PIN_R).extrude(PIN_X1 - PIN_X0)
    pin_neg = cq.Workplane("YZ").workplane(offset=-PIN_X1).circle(PIN_R).extrude(PIN_X1 - PIN_X0)
    return loop.union(pin_pos).union(pin_neg)


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_cauldron")

    # ---- pot body -----------------------------------------------------------
    pot = model.part("pot_body")
    pot.visual(
        mesh_from_cadquery(_pot_solid(), "pot_shell"),
        material=CAST_IRON,
        name="pot_shell",
    )
    pot.visual(
        mesh_from_cadquery(_hinge_lug_solid(), "hinge_lug"),
        material=CAST_IRON_HINGE,
        name="hinge_lug",
    )

    # ---- lid ----------------------------------------------------------------
    lid = model.part("lid")
    # Dome: built at origin, visual origin places it at the correct position
    # in the lid part frame (dome centre at (0, LID_OFFSET_Y, LID_OFFSET_Z)).
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_dome"),
        material=CAST_IRON_LID,
        origin=Origin(xyz=(0.0, LID_OFFSET_Y, LID_OFFSET_Z)),
        name="lid_dome",
    )
    # Two cast pivot bosses at the apex; positioned relative to dome centre.
    for i, sign in enumerate((+1, -1)):
        lid.visual(
            Box(BOSS_SIZE),
            origin=Origin(
                xyz=(sign * BOSS_X, LID_OFFSET_Y, BOSS_Z + LID_OFFSET_Z)
            ),
            material=CAST_IRON_LID,
            name=f"handle_boss_{i}",
        )
    # Hinge strap at the lid hinge end (lid part frame origin).
    lid.visual(
        mesh_from_cadquery(_hinge_strap_solid(), "hinge_strap"),
        material=CAST_IRON_HINGE,
        name="hinge_strap",
    )

    # ---- handle -------------------------------------------------------------
    handle = model.part("loop_handle")
    handle.visual(
        mesh_from_cadquery(_handle_solid(), "handle_loop"),
        material=CAST_IRON_DARK,
        name="handle_loop",
    )

    # ---- articulations ------------------------------------------------------
    # Lid hinge: REVOLUTE at the back rim.  At q=0 the lid is closed; positive
    # q rotates about +X, lifting the front edge (+Y side) upward toward +Z.
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=pot,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=1.0, lower=0.0, upper=LID_OPEN
        ),
    )

    # Handle: authored flat (loop sweeps below the pivot in its own frame);
    # rotating +q about +X lifts the free arch from flat (0) to upright (+1.57).
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(
            xyz=(0.0, LID_OFFSET_Y, PIVOT_Z + LID_OFFSET_Z)
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=HANDLE_UP
        ),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pot = object_model.get_part("pot_body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("loop_handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")
    lid_dome = lid.get_visual("lid_dome")
    hinge_strap = lid.get_visual("hinge_strap")
    boss_0 = lid.get_visual("handle_boss_0")
    boss_1 = lid.get_visual("handle_boss_1")
    handle_loop = handle.get_visual("handle_loop")
    pot_shell = pot.get_visual("pot_shell")
    hinge_lug = pot.get_visual("hinge_lug")

    # ---- intentional-overlap allowances ------------------------------------
    # Hinge strap wrap intentionally nests around the pot hinge barrel.
    ctx.allow_overlap(
        lid,
        pot,
        elem_a=hinge_strap,
        elem_b=hinge_lug,
        reason="Hinge strap cylindrical wrap fits around the hinge barrel; "
        "intentional captured-pivot embedding (~1 mm radial interference).",
    )

    # Handle pin ends are intentionally captured inside the two lid bosses.
    ctx.allow_overlap(
        handle,
        lid,
        elem_a=handle_loop,
        elem_b=boss_0,
        reason="Handle pivot pin is intentionally captured inside the cast lid boss.",
    )
    ctx.allow_overlap(
        handle,
        lid,
        elem_a=handle_loop,
        elem_b=boss_1,
        reason="Handle pivot pin is intentionally captured inside the cast lid boss.",
    )

    # ---- overall proportions / hero silhouette ------------------------------
    pot_aabb = ctx.part_world_aabb(pot)
    ctx.check(
        "pot belly spans ~0.45 m and foot sits on the ground",
        pot_aabb is not None
        and 0.44 <= (pot_aabb[1][0] - pot_aabb[0][0]) <= 0.46
        and abs(pot_aabb[0][2]) <= 0.002
        and 0.30 <= pot_aabb[1][2] <= 0.36,
        details=f"pot_aabb={pot_aabb}",
    )
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid is a ~0.30 m disc overhanging the 0.28 m mouth",
        lid_aabb is not None and 0.295 <= (lid_aabb[1][0] - lid_aabb[0][0]) <= 0.305,
        details=f"lid_aabb={lid_aabb}",
    )

    # ---- pot interior is genuinely hollow ----------------------------------
    hollow_vol = float(_pot_solid().val().Volume())
    filled_vol = float(_pot_filled_solid().val().Volume())
    ctx.check(
        "pot body is hollow with an open mouth (thin revolved wall)",
        0.0 < hollow_vol < 0.40 * filled_vol,
        details=f"wall_volume={hollow_vol:.5f}, filled_volume={filled_vol:.5f}",
    )

    # ---- lid seats centered on the mouth at rest ----------------------------
    # The lid part origin is at the hinge (offset from the pot centre); check
    # the dome visual overlaps the pot mouth in XY instead.
    ctx.expect_overlap(
        lid,
        pot,
        axes="xy",
        elem_a=lid_dome,
        elem_b=pot_shell,
        min_overlap=0.20,
        name="dome footprint overlaps the pot mouth in XY",
    )
    ctx.expect_contact(
        lid,
        pot,
        elem_a=lid_dome,
        elem_b=pot_shell,
        contact_tol=0.005,
        name="lid dome seats on the pot rim (contact, not floating)",
    )

    # ---- hinge lug is near the pot rim -------------------------------------
    lug_aabb = ctx.part_element_world_aabb(pot, elem=hinge_lug)
    ctx.check(
        "hinge lug sits near the pot rim",
        lug_aabb is not None
        and lug_aabb[0][2] >= 0.270
        and lug_aabb[1][2] <= HINGE_Z + HINGE_BARREL_R + 0.010,
        details=f"lug_aabb={lug_aabb}",
    )

    # ---- lid hinge: positive q opens the lid upward -------------------------
    rest_dome_aabb = ctx.part_element_world_aabb(lid, elem=lid_dome)
    with ctx.pose({lid_hinge: LID_OPEN}):
        open_dome_aabb = ctx.part_element_world_aabb(lid, elem=lid_dome)
        # Dome top should rise substantially when the lid flips open.
        ctx.check(
            "lid hinge opens: dome top rises by at least 0.08 m",
            rest_dome_aabb is not None
            and open_dome_aabb is not None
            and open_dome_aabb[1][2] > rest_dome_aabb[1][2] + 0.08,
            details=f"rest_dome={rest_dome_aabb}, open_dome={open_dome_aabb}",
        )

    # At a mid-open pose the dome should not deeply interpenetrate the pot shell.
    with ctx.pose({lid_hinge: 1.0}):
        ctx.expect_gap(
            lid,
            pot,
            axis="z",
            positive_elem=lid_dome,
            negative_elem=pot_shell,
            min_gap=-0.020,
            name="half-open dome does not deeply penetrate the pot shell",
        )

    # ---- handle mounting: pins captured by both bosses ----------------------
    ctx.expect_overlap(
        handle,
        lid,
        axes="x",
        elem_a=handle_loop,
        elem_b=boss_0,
        min_overlap=0.008,
        name="handle pin engages the +X boss",
    )
    ctx.expect_overlap(
        handle,
        lid,
        axes="x",
        elem_a=handle_loop,
        elem_b=boss_1,
        min_overlap=0.008,
        name="handle pin engages the -X boss",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a=handle_loop,
        elem_b=boss_0,
        contact_tol=0.002,
        name="handle is connected to the +X boss",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a=handle_loop,
        elem_b=boss_1,
        contact_tol=0.002,
        name="handle is connected to the -X boss",
    )

    # ---- handle pose: flat vs upright --------------------------------------
    flat_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_pivot: HANDLE_UP}):
        up_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "upright handle arch rises above the flat position",
            up_aabb is not None
            and flat_aabb is not None
            and up_aabb[1][2] > flat_aabb[1][2] + 0.02,
            details=f"flat={flat_aabb}, up={up_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
