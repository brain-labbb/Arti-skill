from __future__ import annotations

"""Three-footed cast-iron cooking cauldron (gypsy cauldron) with a domed lid.

Variant of the traditional cast-iron cauldron: three short cast-iron legs under
the pot at 120° spacing, fixed to the pot bottom.

Structure (reference: picture/Other/cauldron/001.png):
- pot_body: hollow round-bellied vessel (single revolved wall profile) that
  bulges to ~0.45 m at the belly, necks in to a 0.28 m open mouth with visible
  ~12 mm wall thickness at the rim, and stands on an integral cylindrical
  pedestal foot (0.20 m dia x ~0.04 m tall).
- lid: shallow domed disc, 0.30 m dia, with a stepped rim that overhangs the
  mouth, a hollow cast underside, and a short locating lip that drops into the
  mouth opening. Two small cast bosses at the apex carry the handle pivot.
- loop_handle: arched semicircular loop (swing bail) whose pin ends are captured
  in the two lid bosses.
- leg_0, leg_1, leg_2: three tapered cast-iron legs at equal 120° spacing under
  the pot, each ~0.08 m tall, fixed to the pot pedestal bottom.

Articulations:
- lid_lift: PRISMATIC +Z, lid slides 0.12 m straight up off the mouth.
- handle_pivot: REVOLUTE about the horizontal boss axis (+X); 0 rad lies flat
  against the lid dome, +1.57 rad stands the loop upright.
- pot_to_leg_{0,1,2}: FIXED joints attaching each leg to the pot body.
"""

import math
from functools import lru_cache

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
POT_FOOT_R = 0.100  # pedestal foot radius (0.20 m diameter)
POT_FOOT_H = 0.042  # pedestal foot height
POT_BELLY_R = 0.225  # widest belly radius (0.45 m diameter)
POT_RIM_Z = 0.310  # pot mouth rim height
POT_RIM_OUTER_R = 0.140  # mouth outer radius (0.28 m mouth diameter)
POT_RIM_INNER_R = 0.128  # mouth inner radius -> 12 mm visible wall at the rim

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

# ---- three-legged variant (gypsy cauldron) ----------------------------
LEG_COUNT = 3
LEG_ATTACH_R = 0.070  # radial distance of each leg center from pot axis
LEG_H = 0.080  # total leg height (ground to attachment plane)
LEG_EMBED = 0.012  # how far the leg top embeds into the pedestal
LEG_R_TOP = 0.018  # leg radius at the top (wider, cast into the pot)
LEG_R_BOT = 0.014  # leg radius at the foot (narrower)

CAST_IRON = Material(name="cast_iron_black", rgba=(0.10, 0.10, 0.11, 1.0))
CAST_IRON_LID = Material(name="cast_iron_lid", rgba=(0.13, 0.13, 0.14, 1.0))
CAST_IRON_DARK = Material(name="cast_iron_dark", rgba=(0.08, 0.08, 0.09, 1.0))
CAST_IRON_LEG = Material(name="cast_iron_leg", rgba=(0.09, 0.09, 0.10, 1.0))


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


@lru_cache(maxsize=1)
def _leg_solid() -> cq.Workplane:
    """Single cast-iron cauldron leg: tapered shaft with a rounded foot.

    The part-local origin sits at the attachment plane (pot pedestal bottom).
    The leg shaft extends downward (-Z) to the ground and embeds slightly
    upward (+Z) into the pedestal for a cast-in-place joint.
    """
    shaft_h = LEG_H  # length below the attachment plane
    # Tapered shaft: wider at top (cast into pot), narrower at foot.
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=LEG_EMBED)
        .circle(LEG_R_TOP)
        .workplane(offset=-(shaft_h + LEG_EMBED))
        .circle(LEG_R_BOT)
        .loft()
    )
    # Rounded foot pad at the ground contact.
    foot = (
        cq.Workplane("XY")
        .workplane(offset=-shaft_h)
        .circle(LEG_R_BOT * 1.20)
        .extrude(-0.004)
    )
    return shaft.union(foot)


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_cauldron")

    pot = model.part("pot_body")
    pot.visual(
        mesh_from_cadquery(_pot_solid(), "pot_shell"),
        material=CAST_IRON,
        name="pot_shell",
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

    # Three cast-iron legs at equal 120° spacing under the pot pedestal.
    # Each leg is a separate part (FIXED to pot_body) following the repeated
    # sub-part pattern: shared geometry helper, for-loop, leg_{i} naming.
    for i in range(LEG_COUNT):
        angle = math.radians(i * 360.0 / LEG_COUNT)
        x = LEG_ATTACH_R * math.cos(angle)
        y = LEG_ATTACH_R * math.sin(angle)

        leg = model.part(f"leg_{i}")
        leg.visual(
            mesh_from_cadquery(_leg_solid(), f"leg_shaft_{i}"),
            material=CAST_IRON_LEG,
            name=f"leg_shaft_{i}",
        )

        model.articulation(
            f"pot_to_leg_{i}",
            ArticulationType.FIXED,
            parent=pot,
            child=leg,
            origin=Origin(xyz=(x, y, 0.0)),
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

    # Each leg top intentionally embeds ~12 mm into the pot pedestal bottom
    # for a cast-in-place joint; scoped per-leg to the shaft/shell interface.
    legs = [object_model.get_part(f"leg_{i}") for i in range(LEG_COUNT)]
    for i in range(LEG_COUNT):
        leg_i = legs[i]
        shaft_i = leg_i.get_visual(f"leg_shaft_{i}")
        ctx.allow_overlap(
            leg_i,
            pot,
            elem_a=shaft_i,
            elem_b=pot_shell,
            reason=f"Leg {i} top embeds into the pedestal bottom for a cast-in-place joint.",
        )

    # --- overall proportions / hero silhouette ---------------------------
    pot_aabb = ctx.part_world_aabb(pot)
    ctx.check(
        "pot belly spans ~0.45 m at the widest",
        pot_aabb is not None
        and 0.44 <= (pot_aabb[1][0] - pot_aabb[0][0]) <= 0.46,
        details=f"pot_aabb={pot_aabb}",
    )
    ctx.check(
        "pot rim height is ~0.31 m above the pot origin",
        pot_aabb is not None
        and 0.30 <= pot_aabb[1][2] <= 0.32,
        details=f"pot_aabb={pot_aabb}",
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

    # --- three legs: count, 120° spacing, fixed to pot -------------------
    leg_positions = [ctx.part_world_position(leg_i) for leg_i in legs]
    ctx.check(
        f"cauldron has {LEG_COUNT} cast-iron legs",
        len(legs) == LEG_COUNT and all(p is not None for p in leg_positions),
        details=f"positions={leg_positions}",
    )

    # Each leg extends below the pot (ground clearance for a fire).
    for i in range(LEG_COUNT):
        leg_aabb = ctx.part_world_aabb(legs[i])
        ctx.check(
            f"leg_{i} extends below the pot bottom",
            leg_aabb is not None
            and pot_aabb is not None
            and leg_aabb[0][2] < pot_aabb[0][2] - 0.03,
            details=f"leg_aabb={leg_aabb}, pot_bottom_z={pot_aabb[0][2] if pot_aabb else None}",
        )

    # 120° equal spacing around the pot axis.
    if all(p is not None for p in leg_positions):
        angles = [math.atan2(p[1], p[0]) for p in leg_positions]
        radii = [math.hypot(p[0], p[1]) for p in leg_positions]
        # All legs at approximately the same radial distance.
        ctx.check(
            "all legs at the same radial distance from the pot axis",
            all(abs(r - LEG_ATTACH_R) < 0.005 for r in radii),
            details=f"radii={radii}",
        )
        # Angular separation between consecutive legs is ~120°.
        angle_diffs = sorted(
            [
                ((angles[(j + 1) % LEG_COUNT] - angles[j]) + 2 * math.pi) % (2 * math.pi)
                for j in range(LEG_COUNT)
            ]
        )
        ctx.check(
            "legs are equally spaced at ~120° intervals",
            all(abs(d - 2 * math.pi / LEG_COUNT) < 0.05 for d in angle_diffs),
            details=f"angle_diffs_rad={[f'{d:.3f}' for d in angle_diffs]}",
        )

    # Each leg is connected to the pot via a FIXED articulation.
    for i in range(LEG_COUNT):
        joint_name = f"pot_to_leg_{i}"
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"leg_{i} is FIXED to pot_body",
            joint is not None
            and joint.articulation_type == ArticulationType.FIXED
            and joint.parent == "pot_body"
            and joint.child == f"leg_{i}",
            details=f"joint={joint.name if joint else None}",
        )
        # Leg remains attached (contact) to the pot at all times.
        ctx.expect_contact(
            legs[i],
            pot,
            contact_tol=0.001,
            name=f"leg_{i} maintains contact with the pot body",
        )

    return ctx.report()


object_model = build_object_model()
