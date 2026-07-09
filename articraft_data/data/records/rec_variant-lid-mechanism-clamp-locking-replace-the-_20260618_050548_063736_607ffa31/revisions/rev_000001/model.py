from __future__ import annotations

"""Traditional cast-iron cooking cauldron with a clamped locking lid.

Variant: lid_mechanism = clamp_locking
(reference: picture/Other/cauldron/001.png)

Structure:
- pot_body: hollow round-bellied vessel (single revolved wall profile) that
  bulges to ~0.45 m at the belly, necks in to a 0.28 m open mouth with visible
  ~12 mm wall thickness at the rim, and stands on an integral cylindrical
  pedestal foot (0.20 m dia x ~0.04 m tall).
- lid: shallow domed disc, 0.30 m dia, with a stepped rim that overhangs the
  mouth, a hollow cast underside, and a short locating lip that drops into the
  mouth opening. Two small cast bosses at the apex carry the handle pivot.
  Two cast mounting lugs at the rim carry the toggle clamp pivots.
- loop_handle: arched semicircular loop whose pin ends are captured in the two
  lid bosses.
- clamp_{0,1}: mirrored toggle clamp latch arms at the lid rim. Each is a flat
  lever with a pivot boss at one end and an inward-facing hook lip at the other,
  swinging from closed (hook engages pot rim) to open (swung outward/upward).

Articulations:
- lid_lift: PRISMATIC +Z, lid slides 0.12 m straight up off the mouth.
- handle_pivot: REVOLUTE about the horizontal boss axis (+X); 0 rad lies flat
  against the lid dome, +1.57 rad stands the loop upright.
- clamp_pivot_{0,1}: REVOLUTE about tangent axis at each rim lug; 0 rad closed
  (hook down engaging pot rim), positive swings the arm outward and upward
  to clear the pot for lid removal.
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

# --- toggle clamp mechanism ---------------------------------------------------
CLAMP_COUNT = 2
CLAMP_R = 0.154  # pivot radial position (just outside the 0.150 lid rim)
CLAMP_LUG_CY = 0.148  # lug visual center (straddles the lid rim edge)
CLAMP_LUG_BASE_Z = 0.012  # lug base on the lid rim step surface
CLAMP_LUG_H = 0.022  # lug height (tapered cast protrusion)
CLAMP_PIVOT_Z = CLAMP_LUG_BASE_Z + CLAMP_LUG_H * 0.55  # pivot pin height ≈0.024
CLAMP_ARM_W = 0.014  # arm width (along pivot axis)
CLAMP_ARM_T = 0.005  # arm thickness (radial)
CLAMP_ARM_L = 0.032  # arm length (pivot to hook bottom)
CLAMP_HOOK_D = 0.016  # hook lip depth (extends inward toward pot center)
CLAMP_HOOK_H = 0.006  # hook lip height
CLAMP_PIVOT_R = 0.006  # pivot boss radius
CLAMP_OPEN = 1.80  # revolute travel: 0 closed, 1.8 rad (~103°) fully open

CAST_IRON = Material(name="cast_iron_black", rgba=(0.10, 0.10, 0.11, 1.0))
CAST_IRON_LID = Material(name="cast_iron_lid", rgba=(0.13, 0.13, 0.14, 1.0))
CAST_IRON_DARK = Material(name="cast_iron_dark", rgba=(0.08, 0.08, 0.09, 1.0))


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
def _clamp_lug_solid() -> cq.Workplane:
    """Cast mounting lug for the toggle clamp pivot.

    Tapered lofted block: wider base on the lid rim, narrower top at the pivot.
    Built from z=0 (base) to z=CLAMP_LUG_H (top), centered on XY origin.
    """
    return (
        cq.Workplane("XY")
        .rect(0.024, 0.018)
        .workplane(offset=CLAMP_LUG_H)
        .rect(0.018, 0.013)
        .loft()
    )


@lru_cache(maxsize=1)
def _clamp_arm_solid() -> cq.Workplane:
    """Toggle clamp latch arm: pivot boss at origin, bar hangs -Z, hook lip at bottom.

    Local frame: pivot at origin, arm extends along -Z, hook extends along -Y
    (toward pot center when mounted on the +Y side; the -Y side uses rpy=(0,0,π)
    so the same geometry mirrors correctly).
    """
    # Main lever bar: centered on XY, extends from z=0 to z=-CLAMP_ARM_L
    bar = (
        cq.Workplane("XY")
        .rect(CLAMP_ARM_W, CLAMP_ARM_T)
        .extrude(-CLAMP_ARM_L)
    )

    # Pivot boss: cylinder along X at the origin (wider than arm to show the pin)
    boss_len = CLAMP_ARM_W + 0.006
    boss = (
        cq.Workplane("YZ")
        .circle(CLAMP_PIVOT_R)
        .extrude(boss_len)
        .translate((-boss_len / 2.0, 0.0, 0.0))
    )

    # Hook lip: tab at the arm bottom extending -Y (toward pot center)
    # Built on the XZ workplane offset to the -Y face of the arm
    hook = (
        cq.Workplane("XZ")
        .workplane(offset=-(CLAMP_ARM_T / 2.0))
        .center(0.0, -(CLAMP_ARM_L - CLAMP_HOOK_H / 2.0))
        .rect(CLAMP_ARM_W, CLAMP_HOOK_H)
        .extrude(-CLAMP_HOOK_D)
    )

    return bar.union(boss).union(hook)


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

    # --- toggle clamp mechanism (mirrored pair at the lid rim) ---------------
    clamp_arm_mesh = mesh_from_cadquery(_clamp_arm_solid(), "clamp_arm")
    clamp_lug_mesh = mesh_from_cadquery(_clamp_lug_solid(), "clamp_lug")

    for i in range(CLAMP_COUNT):
        sign = 1 - 2 * i  # +1 for i=0 (+Y side), -1 for i=1 (-Y side)
        cy_lug = sign * CLAMP_LUG_CY
        cy_pivot = sign * CLAMP_R

        # Cast mounting lug on the lid rim (part of the lid casting)
        lid.visual(
            clamp_lug_mesh,
            origin=Origin(xyz=(0.0, cy_lug, CLAMP_LUG_BASE_Z)),
            material=CAST_IRON_LID,
            name=f"clamp_lug_{i}",
        )

        # Toggle clamp latch arm
        clamp = model.part(f"clamp_{i}")
        clamp.visual(
            clamp_arm_mesh,
            material=CAST_IRON_DARK,
            name=f"clamp_arm_{i}",
        )

        # Revolute pivot: at q=0 the arm hangs -Z (closed, hook engages pot rim);
        # positive q swings the arm outward and upward. For the -Y side, rpy
        # rotates the frame 180° about Z so the same geometry and axis mirror
        # correctly (local -Y becomes world +Y, toward pot center).
        model.articulation(
            f"clamp_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=clamp,
            origin=Origin(
                xyz=(0.0, cy_pivot, CLAMP_PIVOT_Z),
                rpy=(0.0, 0.0, i * math.pi),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0, lower=0.0, upper=CLAMP_OPEN
            ),
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

    # --- overall proportions / hero silhouette ---------------------------
    pot_aabb = ctx.part_world_aabb(pot)
    ctx.check(
        "pot belly spans ~0.45 m and foot sits on the ground",
        pot_aabb is not None
        and 0.44 <= (pot_aabb[1][0] - pot_aabb[0][0]) <= 0.46
        and abs(pot_aabb[0][2]) <= 0.002
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

    # --- toggle clamp mechanism -----------------------------------------------
    for i in range(CLAMP_COUNT):
        clamp = object_model.get_part(f"clamp_{i}")
        clamp_pivot = object_model.get_articulation(f"clamp_pivot_{i}")
        clamp_arm = clamp.get_visual(f"clamp_arm_{i}")
        lug = lid.get_visual(f"clamp_lug_{i}")

        # Pivot boss intentionally captured inside the cast lug
        ctx.allow_overlap(
            clamp,
            lid,
            elem_a=clamp_arm,
            elem_b=lug,
            reason=f"Clamp {i} pivot boss is captured inside the lid mounting lug bore.",
        )

        # Hook lip intentionally engages the pot rim flange
        ctx.allow_overlap(
            clamp,
            pot,
            elem_a=clamp_arm,
            elem_b=pot_shell,
            reason=f"Clamp {i} hook lip engages the pot rim flange; small local contact "
            "representing the latch catching under the rim.",
        )

        # Lug is cast onto the lid rim; base embeds into the dome shell
        ctx.allow_overlap(
            lid,
            lid,
            elem_a=lug,
            elem_b=lid_dome,
            reason=f"Clamp {i} mounting lug is cast onto the lid rim, merging into the dome shell.",
        )

        # At q=0 (closed): clamp arm hangs down, hook near/below the pot rim
        rest_aabb = ctx.part_world_aabb(clamp)
        ctx.check(
            f"clamp_{i} closed position extends below the lid seat plane",
            rest_aabb is not None and rest_aabb[0][2] < LID_Z + 0.002,
            details=f"rest_aabb={rest_aabb}",
        )

        # At q=CLAMP_OPEN: clamp arm swings outward and upward
        with ctx.pose({clamp_pivot: CLAMP_OPEN}):
            open_aabb = ctx.part_world_aabb(clamp)
            ctx.check(
                f"clamp_{i} open pose rises above the closed position",
                open_aabb is not None
                and rest_aabb is not None
                and open_aabb[1][2] > rest_aabb[1][2] + 0.005,
                details=f"rest={rest_aabb}, open={open_aabb}",
            )
            # Arm swings outward: for +Y clamp, y_max increases; for -Y clamp, y_min decreases
            if i == 0:
                ctx.check(
                    f"clamp_{i} swings outward (+Y) when opened",
                    open_aabb is not None
                    and rest_aabb is not None
                    and open_aabb[1][1] > rest_aabb[1][1] + 0.008,
                    details=f"rest_y_max={rest_aabb[1][1]}, open_y_max={open_aabb[1][1]}",
                )
            else:
                ctx.check(
                    f"clamp_{i} swings outward (-Y) when opened",
                    open_aabb is not None
                    and rest_aabb is not None
                    and open_aabb[0][1] < rest_aabb[0][1] - 0.008,
                    details=f"rest_y_min={rest_aabb[0][1]}, open_y_min={open_aabb[0][1]}",
                )

        # Clamp pivot articulation is revolute with correct limits
        pivot_limits = clamp_pivot.motion_limits
        ctx.check(
            f"clamp_pivot_{i} has revolute limits from 0 to ~{CLAMP_OPEN:.1f} rad",
            pivot_limits is not None
            and pivot_limits.lower is not None
            and pivot_limits.upper is not None
            and abs(pivot_limits.lower) < 0.01
            and abs(pivot_limits.upper - CLAMP_OPEN) < 0.01,
            details=f"limits=({pivot_limits.lower}, {pivot_limits.upper})",
        )

    # Verify clamp lugs are visible on the lid
    for i in range(CLAMP_COUNT):
        lug = lid.get_visual(f"clamp_lug_{i}")
        lug_aabb = ctx.part_element_world_aabb(lid, elem=f"clamp_lug_{i}")
        ctx.check(
            f"clamp_lug_{i} is a visible cast feature on the lid rim",
            lug_aabb is not None
            and lug_aabb[1][2] > LID_Z + 0.005
            and lug_aabb[0][2] < LID_Z + 0.040,
            details=f"lug_aabb={lug_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
