from __future__ import annotations

"""Four-legged standing cast-iron cooking cauldron with a removable domed lid.

Variant (leg_count=4) of the traditional cast-iron cauldron family.

Structure:
- pot_body: hollow round-bellied vessel (single revolved wall profile) that
  bulges to ~0.45 m at the belly, necks in to a 0.28 m open mouth with visible
  ~12 mm wall thickness at the rim, and has a smoothly rounded bottom (no
  pedestal foot). Four cast-iron legs are cast inline into the pot bottom at
  90-degree spacing, extending down to the ground plane.
- lid: shallow domed disc, 0.30 m dia, with a stepped rim that overhangs the
  mouth, a hollow cast underside, and a short locating lip that drops into the
  mouth opening. Two small cast bosses at the apex carry the handle pivot.
- loop_handle: arched semicircular loop whose pin ends are captured in the two
  lid bosses.

Articulations:
- lid_lift: PRISMATIC +Z, lid slides 0.12 m straight up off the mouth.
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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
POT_BELLY_R = 0.225  # widest belly radius (0.45 m diameter)
POT_RIM_Z = 0.310  # pot mouth rim height
POT_RIM_OUTER_R = 0.140  # mouth outer radius (0.28 m mouth diameter)
POT_RIM_INNER_R = 0.128  # mouth inner radius -> 12 mm visible wall at the rim
POT_BOTTOM_Z = 0.045  # center of rounded pot bottom (outer surface)

LID_Z = POT_RIM_Z  # lid frame: seat plane rests directly on the pot rim
LID_R = 0.150  # lid outer radius (0.30 m diameter, overhangs the mouth)
LID_APEX_Z = 0.060  # dome apex above the seat plane
LID_LIP_R = 0.124  # locating lip outer radius (4 mm clearance in the mouth)
LID_LIP_DEPTH = 0.008

BOSS_X = 0.033  # boss centers flank the apex along +-X
BOSS_SIZE = (0.016, 0.018, 0.020)
BOSS_Z = 0.064  # boss center height (lid local); base embeds into the dome
PIVOT_Z = 0.068  # handle pivot axis height (lid local)

LOOP_R = 0.030  # loop arch radius
LOOP_ROD_R = 0.006  # loop rod radius
PIN_R = 0.005  # pivot pin radius
PIN_X0 = 0.026  # pin start (just inside the loop end)
PIN_X1 = 0.040  # pin end (buried in the boss)

LID_LIFT = 0.12  # prismatic travel
HANDLE_UP = 1.5708  # revolute travel (flat -> upright)

# Leg dimensions
LEG_COUNT = 4
LEG_HEIGHT = 0.058  # leg height (extends from ground up into pot bottom)
LEG_TOP_R = 0.016  # radius at top (wider where cast into pot)
LEG_FOOT_R = 0.013  # radius at foot
LEG_ATTACH_R = 0.082  # radial distance from center for leg placement

CAST_IRON = Material(name="cast_iron_black", rgba=(0.10, 0.10, 0.11, 1.0))
CAST_IRON_LID = Material(name="cast_iron_lid", rgba=(0.13, 0.13, 0.14, 1.0))
CAST_IRON_DARK = Material(name="cast_iron_dark", rgba=(0.08, 0.08, 0.09, 1.0))
CAST_IRON_LEG = Material(name="cast_iron_leg", rgba=(0.09, 0.09, 0.10, 1.0))


# ---------------------------------------------------------------- geometry
@lru_cache(maxsize=1)
def _pot_solid() -> cq.Workplane:
    """Hollow bulbous pot wall with rounded bottom (no pedestal foot)."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, POT_BOTTOM_Z)
        .spline(
            [
                (0.04, 0.047),
                (0.08, 0.055),
                (0.12, 0.070),
                (0.16, 0.100),
                (0.20, 0.145),
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
                (0.198, 0.140),
                (0.148, 0.095),
                (0.108, 0.065),
                (0.065, 0.054),
                (0.0, 0.052),
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
        .moveTo(0.0, POT_BOTTOM_Z)
        .spline(
            [
                (0.04, 0.047),
                (0.08, 0.055),
                (0.12, 0.070),
                (0.16, 0.100),
                (0.20, 0.145),
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
def _leg_solid() -> cq.Workplane:
    """Tapered cast-iron cauldron leg with a slight foot flare.

    The leg is a revolved profile: wider at the top where it is cast into the
    pot bottom, tapering to a slightly narrower shaft, then flaring at the foot
    for a stable ground contact.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.017, 0.0)       # foot flare outer edge
        .lineTo(LEG_FOOT_R, 0.005)  # foot neck
        .spline(
            [
                (0.012, 0.018),
                (0.013, 0.035),
                (LEG_TOP_R, LEG_HEIGHT - 0.008),
                (LEG_TOP_R, LEG_HEIGHT),
            ],
            includeCurrent=True,
        )
        .lineTo(0.0, LEG_HEIGHT)
        .lineTo(0.0, 0.0)
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


@lru_cache(maxsize=1)
def _lid_solid() -> cq.Workplane:
    """Domed lid disc: stepped overhanging rim, hollow cast underside."""
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
def _lid_lip_solid() -> cq.Workplane:
    """Locating lip ring under the lid seat: drops into the mouth opening.

    The ring rises 4 mm above the seat plane so it merges into the lid
    underside casting (same part), and hangs LID_LIP_DEPTH below the seat.
    """
    return (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(LID_LIP_R)
        .circle(0.118)
        .extrude(-(LID_LIP_DEPTH + 0.004))
    )


@lru_cache(maxsize=1)
def _handle_solid() -> cq.Workplane:
    """Semicircular loop (half torus through +Y) with pin ends along the X pivot axis."""
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
    model = ArticulatedObject(name="cast_iron_cauldron_4leg")

    pot = model.part("pot_body")
    pot.visual(
        mesh_from_cadquery(_pot_solid(), "pot_shell"),
        material=CAST_IRON,
        name="pot_shell",
    )

    # Four cast-iron legs at equal 90-degree spacing around the pot bottom.
    # Each leg is an inline visual on pot_body (fixed, no separate part).
    leg_mesh = mesh_from_cadquery(_leg_solid(), "cauldron_leg")
    for i in range(LEG_COUNT):
        angle = i * (2.0 * math.pi / LEG_COUNT)
        x = LEG_ATTACH_R * math.cos(angle)
        y = LEG_ATTACH_R * math.sin(angle)
        pot.visual(
            leg_mesh,
            origin=Origin(xyz=(x, y, 0.0)),
            material=CAST_IRON_LEG,
            name=f"leg_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_dome"),
        material=CAST_IRON_LID,
        name="lid_dome",
    )
    lid.visual(
        mesh_from_cadquery(_lid_lip_solid(), "lid_lip"),
        material=CAST_IRON_LID,
        name="lid_lip",
    )
    # Two cast pivot bosses at the apex; bases embed into the dome shell.
    lid.visual(
        Box(BOSS_SIZE),
        origin=Origin(xyz=(BOSS_X, 0.0, BOSS_Z)),
        material=CAST_IRON_LID,
        name="handle_boss_0",
    )
    lid.visual(
        Box(BOSS_SIZE),
        origin=Origin(xyz=(-BOSS_X, 0.0, BOSS_Z)),
        material=CAST_IRON_LID,
        name="handle_boss_1",
    )

    handle = model.part("loop_handle")
    handle.visual(
        mesh_from_cadquery(_handle_solid(), "handle_loop"),
        material=CAST_IRON_DARK,
        name="handle_loop",
    )

    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=pot,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=LID_LIFT),
    )

    # Handle authored flat (loop sweeps +Y in its own frame); rotating +q about
    # +X lifts the free arch from flat (0) to upright (+1.57).
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=HANDLE_UP),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pot = object_model.get_part("pot_body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("loop_handle")
    lid_lift = object_model.get_articulation("lid_lift")
    handle_pivot = object_model.get_articulation("handle_pivot")
    lid_dome = lid.get_visual("lid_dome")
    lid_lip = lid.get_visual("lid_lip")
    boss_0 = lid.get_visual("handle_boss_0")
    boss_1 = lid.get_visual("handle_boss_1")
    handle_loop = handle.get_visual("handle_loop")
    pot_shell = pot.get_visual("pot_shell")

    # The locating lip intentionally nests inside the open mouth; the hull-based
    # overlap proxy treats the hollow mouth as solid, so scope an allowance to
    # exactly this lip/shell interface (proven separately below).
    ctx.allow_overlap(
        lid,
        pot,
        elem_a=lid_lip,
        elem_b=pot_shell,
        reason="Lid locating lip nests inside the open pot mouth; the pot is hollow "
        "and the lip keeps 4 mm radial clearance to the inner rim wall.",
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

    # --- four legs: existence, placement, and ground contact ---------------
    legs = [pot.get_visual(f"leg_{i}") for i in range(LEG_COUNT)]
    ctx.check(
        "four cast-iron legs exist on the pot body",
        all(leg is not None for leg in legs),
        details=f"leg_count={sum(1 for leg in legs if leg is not None)}",
    )

    # Check 90-degree angular spacing between consecutive legs
    leg_centers = []
    for i, leg in enumerate(legs):
        if leg is not None:
            c = ctx.part_element_world_aabb(pot, elem=f"leg_{i}")
            if c is not None:
                cx = (c[0][0] + c[1][0]) / 2.0
                cy = (c[0][1] + c[1][1]) / 2.0
                leg_centers.append((cx, cy))

    if len(leg_centers) == LEG_COUNT:
        angles = sorted(math.atan2(cy, cx) for cx, cy in leg_centers)
        gaps = []
        for j in range(LEG_COUNT):
            gap = (angles[(j + 1) % LEG_COUNT] - angles[j]) % (2.0 * math.pi)
            gaps.append(gap)
        expected_gap = math.pi / 2.0  # 90 degrees
        ctx.check(
            "legs are at equal 90-degree spacing around the pot",
            all(abs(g - expected_gap) < 0.15 for g in gaps),
            details=f"angular_gaps_deg={[round(math.degrees(g), 1) for g in gaps]}",
        )

        # All legs should be at approximately the same radial distance
        radii = [math.sqrt(cx * cx + cy * cy) for cx, cy in leg_centers]
        ctx.check(
            "legs are at uniform radial distance from pot center",
            max(radii) - min(radii) < 0.010,
            details=f"radii={[round(r, 4) for r in radii]}",
        )

    # Each leg should contact the ground plane (z=0)
    for i in range(LEG_COUNT):
        leg_aabb = ctx.part_element_world_aabb(pot, elem=f"leg_{i}")
        ctx.check(
            f"leg_{i} contacts the ground plane",
            leg_aabb is not None and abs(leg_aabb[0][2]) < 0.003,
            details=f"leg_{i}_min_z={leg_aabb[0][2] if leg_aabb else None}",
        )

    # --- overall proportions / hero silhouette ---------------------------
    pot_aabb = ctx.part_world_aabb(pot)
    ctx.check(
        "pot belly spans ~0.45 m",
        pot_aabb is not None
        and 0.44 <= (pot_aabb[1][0] - pot_aabb[0][0]) <= 0.46,
        details=f"pot_aabb={pot_aabb}",
    )
    ctx.check(
        "pot assembly sits on the ground (legs extend to z=0)",
        pot_aabb is not None and abs(pot_aabb[0][2]) <= 0.003,
        details=f"pot_min_z={pot_aabb[0][2] if pot_aabb else None}",
    )
    ctx.check(
        "pot rim height is ~0.31 m above ground",
        pot_aabb is not None and 0.30 <= pot_aabb[1][2] <= 0.32,
        details=f"pot_max_z={pot_aabb[1][2] if pot_aabb else None}",
    )

    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid is a ~0.30 m disc overhanging the 0.28 m mouth",
        lid_aabb is not None and 0.295 <= (lid_aabb[1][0] - lid_aabb[0][0]) <= 0.305,
        details=f"lid_aabb={lid_aabb}",
    )

    # --- pot interior is genuinely hollow --------------------------------
    hollow_vol = float(_pot_solid().val().Volume())
    filled_vol = float(_pot_filled_solid().val().Volume())
    ctx.check(
        "pot body is hollow with an open mouth (thin revolved wall)",
        0.0 < hollow_vol < 0.40 * filled_vol,
        details=f"wall_volume={hollow_vol:.5f}, filled_volume={filled_vol:.5f}",
    )

    # --- pot has no pedestal foot (rounded bottom) -----------------------
    # The pot shell alone (without legs) should not have a flat bottom at z=0.
    # At the center axis, the pot bottom outer surface is at POT_BOTTOM_Z > 0.
    pot_shell_aabb = ctx.part_element_world_aabb(pot, elem="pot_shell")
    ctx.check(
        "pot shell has a rounded bottom (no flat pedestal foot at z=0)",
        pot_shell_aabb is not None and pot_shell_aabb[0][2] > 0.030,
        details=f"pot_shell_min_z={pot_shell_aabb[0][2] if pot_shell_aabb else None}",
    )

    # --- lid seats centered on the mouth at rest -------------------------
    ctx.expect_origin_distance(
        lid, pot, axes="xy", max_dist=0.002, name="lid stays centered over the mouth"
    )
    ctx.expect_within(
        lid, pot, axes="xy", margin=0.001, name="lid footprint stays within the belly footprint"
    )
    ctx.expect_contact(
        lid,
        pot,
        contact_tol=0.002,
        name="lid seat ring rests on the pot rim (seated, not floating)",
    )
    ctx.expect_within(
        lid,
        pot,
        axes="xy",
        inner_elem=lid_lip,
        outer_elem=pot_shell,
        margin=0.001,
        name="locating lip stays centered inside the mouth opening",
    )

    # --- prismatic lid lift: straight up, fully clear of the mouth -------
    rest_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_lift: LID_LIFT}):
        lifted_pos = ctx.part_world_position(lid)
        ctx.expect_gap(
            lid,
            pot,
            axis="z",
            min_gap=0.10,
            name="lifted lid clears the pot mouth by ~0.11 m",
        )
        ctx.expect_origin_distance(
            lid, pot, axes="xy", max_dist=0.002, name="lid lifts straight up (no lateral drift)"
        )
    ctx.check(
        "lid_lift slides the lid upward by the commanded travel",
        rest_pos is not None
        and lifted_pos is not None
        and abs((lifted_pos[2] - rest_pos[2]) - LID_LIFT) <= 0.001,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    # --- handle mounting: pins captured by both bosses --------------------
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
        contact_tol=0.0005,
        name="handle is connected to the +X boss",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a=handle_loop,
        elem_b=boss_1,
        contact_tol=0.0005,
        name="handle is connected to the -X boss",
    )

    # --- handle pose 0: lies flat just above the dome ---------------------
    flat_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle lies flat against the lid at q=0 (thin z extent)",
        flat_aabb is not None and (flat_aabb[1][2] - flat_aabb[0][2]) <= 0.020,
        details=f"flat_aabb={flat_aabb}",
    )
    ctx.expect_gap(
        handle,
        lid,
        axis="z",
        negative_elem=lid_dome,
        max_penetration=0.0,
        max_gap=0.010,
        name="flat handle hovers just above the dome without sinking in",
    )

    # --- handle pose upright: arch stands up ------------------------------
    with ctx.pose({handle_pivot: HANDLE_UP}):
        up_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "upright handle arch rises above the lid apex",
            up_aabb is not None
            and flat_aabb is not None
            and up_aabb[1][2] >= 0.405
            and up_aabb[1][2] > flat_aabb[1][2] + 0.02,
            details=f"up_aabb={up_aabb}",
        )
        ctx.check(
            "upright handle folds out of the flat plane (y extent shrinks)",
            up_aabb is not None and up_aabb[1][1] <= 0.015,
            details=f"up_aabb={up_aabb}",
        )

    return ctx.report()


object_model = build_object_model()