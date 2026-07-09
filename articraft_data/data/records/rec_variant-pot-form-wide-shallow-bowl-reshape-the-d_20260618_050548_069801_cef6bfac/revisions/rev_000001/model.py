from __future__ import annotations

"""Wide shallow cast-iron cauldron (cazo) with removable domed lid.

Variant: pot_form = wide_shallow_bowl
- Wide shallow bowl body (cazo / wide wok form), much wider than tall.
- Built with a lathe (revolved) profile for the bowl wall and bottom.
- 3 cast-iron legs placed at 120° around the pot bottom, emitted via a
  for-loop as inline visuals on pot_body (no FIXED-joint decoration parts).
- Lift-off lid (PRISMATIC +Z) with a shallow dome and stepped rim.
- Swing bail handle (REVOLUTE) folding from flat to upright.

Reference: picture/Other/cauldron/001.png
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
# --- legs ---
LEG_COUNT = 3
LEG_HEIGHT = 0.050  # leg height from ground (m)
LEG_R_FROM_CENTER = 0.085  # radial placement of each leg from the Z axis

# --- wide shallow bowl ---
BOWL_BOTTOM_Z = LEG_HEIGHT  # bowl bottom sits on top of legs
BOWL_BOTTOM_R = 0.120  # outer radius at the bowl bottom (0.24 m dia)
BOWL_RIM_Z = 0.175  # rim height from ground
BOWL_RIM_R = 0.250  # outer radius at rim (0.50 m dia)
WALL_THICK = 0.010  # wall thickness throughout
BOWL_INNER_BOTTOM_R = BOWL_BOTTOM_R - WALL_THICK
BOWL_INNER_BOTTOM_Z = BOWL_BOTTOM_Z + WALL_THICK
BOWL_INNER_RIM_R = BOWL_RIM_R - WALL_THICK

# --- lid ---
LID_Z = BOWL_RIM_Z  # lid seat plane at the pot rim
LID_R = 0.260  # lid outer radius (0.52 m dia, overhangs rim)
LID_APEX_Z = 0.035  # dome apex above seat plane
LID_LIP_R = BOWL_INNER_RIM_R - 0.004  # locating lip with ~4 mm radial clearance
LID_LIP_DEPTH = 0.008

# --- bosses and handle ---
BOSS_X = 0.035  # boss centers flank the apex along +-X
BOSS_SIZE = (0.016, 0.018, 0.020)
BOSS_Z = LID_APEX_Z + 0.004  # boss center height (lid local)
PIVOT_Z = LID_APEX_Z + 0.008  # handle pivot axis height (lid local)

LOOP_R = 0.030  # loop arch radius
LOOP_ROD_R = 0.006  # loop rod radius
PIN_R = 0.005  # pivot pin radius
PIN_X0 = 0.026  # pin start (just inside the loop end)
PIN_X1 = 0.042  # pin end (buried in the boss)

# --- articulation limits ---
LID_LIFT = 0.12  # prismatic travel (m)
HANDLE_UP = 1.5708  # revolute travel: flat -> upright (rad)

# --- materials ---
CAST_IRON = Material(name="cast_iron_black", rgba=(0.10, 0.10, 0.11, 1.0))
CAST_IRON_LID = Material(name="cast_iron_lid", rgba=(0.12, 0.12, 0.13, 1.0))
CAST_IRON_DARK = Material(name="cast_iron_dark", rgba=(0.08, 0.08, 0.09, 1.0))
CAST_IRON_LEG = Material(name="cast_iron_leg", rgba=(0.09, 0.09, 0.10, 1.0))


# ---------------------------------------------------------------- geometry helpers
@lru_cache(maxsize=1)
def _bowl_solid() -> cq.Workplane:
    """Hollow wide shallow bowl wall + flat bottom, one revolved lathe profile.

    The profile traces the outer wall from the bottom edge outward and up to
    the rim, steps inward across the rim thickness, follows the inner wall back
    down to the inner bottom, then closes across the solid bottom disc.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(BOWL_BOTTOM_R, BOWL_BOTTOM_Z)
        .spline(
            [
                (0.155, BOWL_BOTTOM_Z + 0.018),
                (0.195, BOWL_BOTTOM_Z + 0.048),
                (0.228, BOWL_RIM_Z - 0.018),
                (BOWL_RIM_R, BOWL_RIM_Z),
            ],
            includeCurrent=True,
        )
        .lineTo(BOWL_INNER_RIM_R, BOWL_RIM_Z)
        .spline(
            [
                (0.218, BOWL_RIM_Z - 0.018),
                (0.185, BOWL_BOTTOM_Z + 0.048),
                (0.145, BOWL_BOTTOM_Z + 0.018),
                (BOWL_INNER_BOTTOM_R, BOWL_INNER_BOTTOM_Z),
            ],
            includeCurrent=True,
        )
        .lineTo(0.0, BOWL_INNER_BOTTOM_Z)
        .lineTo(0.0, BOWL_BOTTOM_Z)
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


@lru_cache(maxsize=1)
def _bowl_filled_solid() -> cq.Workplane:
    """Same outer silhouette but solid — only used to prove hollowness in tests."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(BOWL_BOTTOM_R, BOWL_BOTTOM_Z)
        .spline(
            [
                (0.155, BOWL_BOTTOM_Z + 0.018),
                (0.195, BOWL_BOTTOM_Z + 0.048),
                (0.228, BOWL_RIM_Z - 0.018),
                (BOWL_RIM_R, BOWL_RIM_Z),
            ],
            includeCurrent=True,
        )
        .lineTo(0.0, BOWL_RIM_Z)
        .lineTo(0.0, BOWL_BOTTOM_Z)
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


@lru_cache(maxsize=1)
def _lid_solid() -> cq.Workplane:
    """Shallow domed lid disc: stepped overhanging rim, hollow cast underside."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(LID_R, 0.0)
        .lineTo(LID_R, 0.012)
        .lineTo(BOWL_INNER_RIM_R + 0.006, 0.016)
        .spline(
            [(0.150, 0.024), (0.080, 0.031), (0.0, LID_APEX_Z)],
            includeCurrent=True,
        )
        .lineTo(0.0, 0.022)
        .spline(
            [(0.080, 0.017), (0.150, 0.010), (BOWL_INNER_RIM_R, 0.0)],
            includeCurrent=True,
        )
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


@lru_cache(maxsize=1)
def _lid_lip_solid() -> cq.Workplane:
    """Locating lip ring under the lid seat: drops into the mouth opening."""
    return (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(LID_LIP_R)
        .circle(LID_LIP_R - 0.008)
        .extrude(-(LID_LIP_DEPTH + 0.004))
    )


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


def _leg_solid() -> cq.Workplane:
    """One cast-iron leg: tapered support from ground to bowl bottom.

    Shared geometry helper; the caller places N copies around the pot bottom.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.026, 0.0)  # foot outer radius at ground
        .lineTo(0.030, 0.003)  # slight foot flare
        .lineTo(0.020, 0.010)  # taper in above foot
        .lineTo(0.016, LEG_HEIGHT - 0.008)  # narrow shaft
        .lineTo(0.024, LEG_HEIGHT)  # widen at top to meet bowl
        .lineTo(0.0, LEG_HEIGHT)
        .lineTo(0.0, 0.0)
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_cazo")

    # ---- pot body (wide shallow bowl + integral legs) ----
    pot = model.part("pot_body")
    pot.visual(
        mesh_from_cadquery(_bowl_solid(), "bowl_shell"),
        material=CAST_IRON,
        name="bowl_shell",
    )

    # Legs: repeated sub-parts emitted via for-loop, inline as parent visuals.
    # Each leg is a revolved solid (rotationally symmetric), so only translation
    # is needed — no rpy rotation required.
    leg_mesh = mesh_from_cadquery(_leg_solid(), "leg")
    for i in range(LEG_COUNT):
        angle_rad = i * (2.0 * math.pi / LEG_COUNT)
        x = LEG_R_FROM_CENTER * math.cos(angle_rad)
        y = LEG_R_FROM_CENTER * math.sin(angle_rad)
        pot.visual(
            leg_mesh,
            origin=Origin(xyz=(x, y, 0.0)),
            material=CAST_IRON_LEG,
            name=f"leg_{i}",
        )

    # ---- lid ----
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

    # ---- loop handle ----
    handle = model.part("loop_handle")
    handle.visual(
        mesh_from_cadquery(_handle_solid(), "handle_loop"),
        material=CAST_IRON_DARK,
        name="handle_loop",
    )

    # ---- articulations ----
    # Lid: prismatic +Z, slides straight up off the rim.
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=pot,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=LID_LIFT),
    )

    # Handle: revolute about the horizontal boss axis (+X); 0 rad lies flat
    # against the lid dome, +1.57 rad stands the loop upright.
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
    bowl_shell = pot.get_visual("bowl_shell")

    # --- overlap allowances ---
    # The locating lip nests inside the open mouth; the pot is hollow.
    ctx.allow_overlap(
        lid,
        pot,
        elem_a=lid_lip,
        elem_b=bowl_shell,
        reason="Lid locating lip nests inside the open pot mouth; the bowl is hollow "
        "and the lip keeps ~4 mm radial clearance to the inner rim wall.",
    )

    # Handle pin ends are captured inside the two lid bosses.
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

    # --- wide shallow bowl proportions ---
    pot_aabb = ctx.part_world_aabb(pot)
    ctx.check(
        "pot body is wide and shallow: rim diameter > 2x rim height from ground",
        pot_aabb is not None
        and (pot_aabb[1][0] - pot_aabb[0][0]) >= 0.48
        and abs(pot_aabb[0][2]) <= 0.002
        and 0.165 <= pot_aabb[1][2] <= 0.185,
        details=f"pot_aabb={pot_aabb}",
    )

    # Width/height ratio proves "much wider than tall"
    if pot_aabb is not None:
        width_x = pot_aabb[1][0] - pot_aabb[0][0]
        height_z = pot_aabb[1][2] - pot_aabb[0][2]
        ctx.check(
            "bowl width/height ratio > 2.5 (cazo form)",
            height_z > 0.001 and width_x / height_z > 2.5,
            details=f"width={width_x:.3f}, height={height_z:.3f}",
        )

    # --- 3 legs visible at ground level ---
    for i in range(LEG_COUNT):
        leg_vis = pot.get_visual(f"leg_{i}")
        leg_aabb = ctx.part_element_world_aabb(pot, elem=f"leg_{i}")
        ctx.check(
            f"leg_{i} exists and touches the ground plane",
            leg_vis is not None
            and leg_aabb is not None
            and abs(leg_aabb[0][2]) <= 0.002,
            details=f"leg_aabb={leg_aabb}",
        )

    # --- bowl interior is genuinely hollow ---
    hollow_vol = float(_bowl_solid().val().Volume())
    filled_vol = float(_bowl_filled_solid().val().Volume())
    ctx.check(
        "bowl body is hollow with an open mouth (thin revolved wall)",
        0.0 < hollow_vol < 0.40 * filled_vol,
        details=f"wall_volume={hollow_vol:.5f}, filled_volume={filled_vol:.5f}",
    )

    # --- lid proportions match the wider bowl ---
    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid is a ~0.52 m disc overhanging the ~0.50 m mouth",
        lid_aabb is not None and 0.51 <= (lid_aabb[1][0] - lid_aabb[0][0]) <= 0.53,
        details=f"lid_aabb={lid_aabb}",
    )

    # --- lid seats centered on the mouth at rest ---
    ctx.expect_origin_distance(
        lid, pot, axes="xy", max_dist=0.002, name="lid stays centered over the mouth"
    )
    ctx.expect_within(
        lid, pot, axes="xy", margin=0.010, name="lid footprint stays within the bowl footprint"
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
        outer_elem=bowl_shell,
        margin=0.005,
        name="locating lip stays centered inside the mouth opening",
    )

    # --- prismatic lid lift: straight up, fully clear of the mouth ---
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

    # --- handle mounting: pins captured by both bosses ---
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

    # --- handle pose 0: lies flat just above the dome ---
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
        max_gap=0.012,
        name="flat handle hovers just above the dome without sinking in",
    )

    # --- handle pose upright: arch stands up ---
    with ctx.pose({handle_pivot: HANDLE_UP}):
        up_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "upright handle arch rises above the lid apex",
            up_aabb is not None
            and flat_aabb is not None
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
