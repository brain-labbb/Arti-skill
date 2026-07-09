from __future__ import annotations

"""Traditional cast-iron cooking cauldron with a removable domed lid.

Structure (reference: picture/Other/cauldron/001.png):
- pot_body: hollow round-bellied vessel (single revolved wall profile) that
  bulges to ~0.45 m at the belly, necks in to a 0.28 m open mouth with visible
  ~12 mm wall thickness at the rim, and stands on an integral cylindrical
  pedestal foot (0.20 m dia x ~0.04 m tall).
- lid: shallow domed disc, 0.30 m dia, with a stepped rim that overhangs the
  mouth, a hollow cast underside, and a short locating lip that drops into the
  mouth opening. Two close cast bosses at the apex carry a chain pin.
- chain_link_{i}: a short pull-chain made of interlocked oval links, using the
  same alternating link construction as the gutter downchain link-chain sample.

Articulations:
- lid_lift: PRISMATIC +Z, lid slides 0.12 m straight up off the mouth.
- chain_swing_{i}: REVOLUTE chain-link joints. The first link is attached to the
  lid bosses, and every following link swings from the previous link.
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
    mesh_from_geometry,
    tube_from_spline_points,
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

BOSS_X = 0.010  # close boss centers flank the chain pin along +-X
BOSS_SIZE = (0.016, 0.018, 0.020)
BOSS_Z = 0.064  # boss center height (lid local); base embeds into the dome
PIVOT_Z = 0.068  # chain anchor pivot height (lid local)

CHAIN_HALF_LEN = 0.011  # short pull-chain link long-axis half length
CHAIN_HALF_WID = 0.0055  # short-axis half width
CHAIN_WIRE_R = 0.0015  # link wire/tube radius
CHAIN_PITCH = 2.0 * CHAIN_HALF_LEN - 2.0 * CHAIN_WIRE_R
CHAIN_TOP_PIVOT_Z = CHAIN_PITCH
CHAIN_LINKS = 4

LID_LIFT = 0.12  # prismatic travel
CHAIN_SWING = math.radians(35.0)

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
def _chain_pin_solid() -> cq.Workplane:
    """Small cross pin captured by the lid bosses; first link hangs from it."""
    return cq.Workplane("YZ").workplane(offset=-0.017).circle(0.003).extrude(0.034)


def _oval_chain_link_mesh(in_yz_plane: bool, name: str):
    """Oval chain link, adapted from the gutter downchain link-chain sample.

    The local origin is the lower pivot point, and the visible oval rises upward
    from the lid so it works as a small linked pull handle.
    """
    pts = []
    for j in range(36):
        t = 2.0 * math.pi * j / 36
        short = CHAIN_HALF_WID * math.cos(t)
        long = CHAIN_HALF_LEN * math.sin(t)
        if in_yz_plane:
            pts.append((0.0, short, CHAIN_HALF_LEN + long))
        else:
            pts.append((short, 0.0, CHAIN_HALF_LEN + long))
    geom = tube_from_spline_points(
        pts,
        radius=CHAIN_WIRE_R,
        samples_per_segment=10,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
    )
    return mesh_from_geometry(geom, name)


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
        name="chain_boss_0",
    )
    lid.visual(
        Box(BOSS_SIZE),
        origin=Origin(xyz=(-BOSS_X, 0.0, BOSS_Z)),
        material=CAST_IRON_LID,
        name="chain_boss_1",
    )
    lid.visual(
        mesh_from_cadquery(_chain_pin_solid(), "chain_anchor_pin"),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        material=CAST_IRON_DARK,
        name="chain_anchor_pin",
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

    prev_part = lid
    parent_pivot_z = PIVOT_Z
    for i in range(CHAIN_LINKS):
        link = model.part(f"chain_link_{i}")
        in_yz = i % 2 == 1
        link.visual(
            _oval_chain_link_mesh(in_yz, f"chain_link_{i}_oval"),
            material=CAST_IRON_DARK,
            name="oval_body",
        )
        model.articulation(
            f"chain_swing_{i}",
            ArticulationType.REVOLUTE,
            parent=prev_part,
            child=link,
            origin=Origin(xyz=(0.0, 0.0, parent_pivot_z)),
            axis=(1.0, 0.0, 0.0) if in_yz else (0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.0,
                lower=-CHAIN_SWING,
                upper=CHAIN_SWING,
            ),
        )
        prev_part = link
        parent_pivot_z = CHAIN_TOP_PIVOT_Z

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pot = object_model.get_part("pot_body")
    lid = object_model.get_part("lid")
    chain_links = [object_model.get_part(f"chain_link_{i}") for i in range(CHAIN_LINKS)]
    lid_lift = object_model.get_articulation("lid_lift")
    chain_joints = [object_model.get_articulation(f"chain_swing_{i}") for i in range(CHAIN_LINKS)]
    lid_dome = lid.get_visual("lid_dome")
    lid_lip = lid.get_visual("lid_lip")
    boss_0 = lid.get_visual("chain_boss_0")
    boss_1 = lid.get_visual("chain_boss_1")
    anchor_pin = lid.get_visual("chain_anchor_pin")
    first_link_body = chain_links[0].get_visual("oval_body")
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

    # Chain anchor pin and the first link are intentionally captured by the
    # two lid bosses.
    ctx.allow_overlap(
        lid,
        lid,
        elem_a=anchor_pin,
        elem_b=boss_0,
        reason="Chain anchor pin is intentionally cast through the +X lid boss.",
    )
    ctx.allow_overlap(
        lid,
        lid,
        elem_a=anchor_pin,
        elem_b=boss_1,
        reason="Chain anchor pin is intentionally cast through the -X lid boss.",
    )
    ctx.allow_overlap(
        chain_links[0],
        lid,
        elem_a=first_link_body,
        elem_b=anchor_pin,
        reason="First chain link is interlocked through the lid anchor pin.",
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

    # --- chain mounting: pin captured by both bosses ----------------------
    ctx.expect_overlap(
        lid,
        lid,
        axes="x",
        elem_a=anchor_pin,
        elem_b=boss_0,
        min_overlap=0.008,
        name="chain anchor pin engages the +X boss",
    )
    ctx.expect_overlap(
        lid,
        lid,
        axes="x",
        elem_a=anchor_pin,
        elem_b=boss_1,
        min_overlap=0.008,
        name="chain anchor pin engages the -X boss",
    )
    ctx.expect_contact(
        chain_links[0],
        lid,
        elem_a=first_link_body,
        elem_b=anchor_pin,
        contact_tol=0.002,
        name="first chain link is connected to the lid anchor pin",
    )

    # --- chain structure: alternating revolute links, attached to lid -----
    ctx.check(
        "chain replaces the former wire loop with four articulated links",
        len(chain_links) == 4 and len(chain_joints) == 4,
        details=f"links={len(chain_links)}, joints={len(chain_joints)}",
    )
    for i, joint in enumerate(chain_joints):
        expected_axis = (1.0, 0.0, 0.0) if i % 2 == 1 else (0.0, 1.0, 0.0)
        ctx.check(
            f"chain link {i} keeps a revolute swing joint with alternating axis",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == expected_axis
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower + CHAIN_SWING) < 1e-6
            and abs(joint.motion_limits.upper - CHAIN_SWING) < 1e-6,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
        )

    chain_aabb = ctx.part_world_aabb(chain_links[-1])
    ctx.check(
        "linked chain rises above the lid and reads as a pull chain, not a bent wire",
        chain_aabb is not None and chain_aabb[1][2] > POT_RIM_Z + PIVOT_Z + 0.08,
        details=f"last_link_aabb={chain_aabb}",
    )

    with ctx.pose({chain_joints[0]: CHAIN_SWING, chain_joints[1]: -CHAIN_SWING}):
        swung_aabb = ctx.part_world_aabb(chain_links[-1])
        ctx.check(
            "chain links retain visible swing motion while remaining attached to the lid",
            swung_aabb is not None
            and chain_aabb is not None
            and abs(swung_aabb[0][0] - chain_aabb[0][0]) > 0.003,
            details=f"rest={chain_aabb}, swung={swung_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
