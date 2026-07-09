from __future__ import annotations

"""Traditional cast-iron cooking cauldron — side_loop_ears variant.

Variant of the parent cauldron: the swing bail handle is replaced by two fixed
cast-iron side loop ear handles (ear_0, ear_1) mirrored on the ±Y sides of the
rim.  Three tapered cast-iron legs (leg_0, leg_1, leg_2) support the pot at
120° intervals around the pedestal foot.  The lift-off domed lid remains as a
PRISMATIC lift — the only non-fixed joint.

Structure:
- pot_body: hollow round-bellied vessel with pedestal foot (root)
- lid: shallow domed disc with stepped rim and locating lip
- ear_0, ear_1: fixed side loop ear handles on ±Y sides
- leg_0, leg_1, leg_2: three tapered legs under the pot

Articulations:
- lid_lift: PRISMATIC +Z, 0.12 m travel
- pot_to_ear_0, pot_to_ear_1: FIXED
- pot_to_leg_0, pot_to_leg_1, pot_to_leg_2: FIXED
"""

import math
from functools import lru_cache

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------- dimensions
POT_FOOT_R = 0.100       # pedestal foot radius (0.20 m diameter)
POT_FOOT_H = 0.042       # pedestal foot height
POT_BELLY_R = 0.225      # widest belly radius (0.45 m diameter)
POT_RIM_Z = 0.310        # pot mouth rim height
POT_RIM_OUTER_R = 0.140  # mouth outer radius (0.28 m mouth diameter)
POT_RIM_INNER_R = 0.128  # mouth inner radius -> 12 mm visible wall at the rim

LID_Z = POT_RIM_Z        # lid frame: seat plane rests directly on the pot rim
LID_R = 0.150            # lid outer radius (0.30 m diameter, overhangs mouth)
LID_APEX_Z = 0.060       # dome apex above the seat plane
LID_LIP_R = 0.124        # locating lip outer radius (4 mm clearance in mouth)
LID_LIP_DEPTH = 0.008

LID_LIFT = 0.12          # prismatic travel

# Ear dimensions
EAR_ARCH_R = 0.035       # arch center radius (foot-to-foot span = 2R = 70 mm)
EAR_ROD_R = 0.007        # rod cross-section radius (14 mm thick bar)
EAR_PAD_W = 0.018        # mounting pad width (along X)
EAR_PAD_H = 0.024        # mounting pad height (along Z)
EAR_PAD_D = 0.006        # mounting pad depth (along Y, embeds into pot wall)
EAR_MOUNT_Z = 0.260      # ear center height on the pot
EAR_MOUNT_R = 0.196      # ear origin Y: pot surface + half pad depth

# Leg dimensions
LEG_TOP_R = 0.018        # leg top radius (wider, at pot foot)
LEG_BOT_R = 0.014        # leg bottom radius (narrower, at ground)
LEG_H = 0.045            # leg height
LEG_COUNT = 3            # number of legs
LEG_PLACEMENT_R = 0.082  # placement radius from pot center (near foot edge)

# ---------------------------------------------------------------- materials
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
    """Locating lip ring under the lid seat: drops into the mouth opening."""
    return (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(LID_LIP_R)
        .circle(0.118)
        .extrude(-(LID_LIP_DEPTH + 0.004))
    )


@lru_cache(maxsize=1)
def _ear_solid() -> cq.Workplane:
    """Half-torus side-loop ear with mounting pads.

    The arch lies in the YZ plane and protrudes along +Y.  Two rectangular
    mounting pads at the feet (z = ±EAR_ARCH_R) extend from y = -EAR_PAD_D
    to y = 0 so they embed slightly into the pot wall when placed.
    """
    # Half-torus: revolve a small circle 180° around the Z axis (XZ workplane
    # convention matches the pot).  This creates an arch in the XY plane:
    #   (R,0,0) -> (0,R,0) -> (-R,0,0)
    profile = (
        cq.Workplane("XZ")
        .moveTo(EAR_ARCH_R, 0.0)
        .circle(EAR_ROD_R)
    )
    half_torus = profile.revolve(180.0, (0, -1, 0), (0, 1, 0))

    # Rotate 90° around Y so the arch stands in the YZ plane:
    #   (R,0,0) -> (0,0,-R)   lower foot
    #   (0,R,0) -> (0,R,0)    outermost point (+Y)
    #   (-R,0,0) -> (0,0,R)   upper foot
    rotated = half_torus.rotate((0, 0, 0), (0, 1, 0), 90.0)

    # Mounting pads at the two feet (z = ±R).
    # Each pad is a small box from y = -EAR_PAD_D to y = 0, centered on the
    # foot in XZ.
    pad_upper = (
        cq.Workplane("XZ")
        .workplane(offset=-EAR_PAD_D)
        .center(0.0, EAR_ARCH_R)
        .rect(EAR_PAD_W, EAR_PAD_H)
        .extrude(EAR_PAD_D)
    )
    pad_lower = (
        cq.Workplane("XZ")
        .workplane(offset=-EAR_PAD_D)
        .center(0.0, -EAR_ARCH_R)
        .rect(EAR_PAD_W, EAR_PAD_H)
        .extrude(EAR_PAD_D)
    )

    return rotated.union(pad_upper).union(pad_lower)


def _make_leg_solid() -> cq.Workplane:
    """Short tapered cast-iron leg extending downward from the origin."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(LEG_TOP_R, 0.0)
        .lineTo(LEG_BOT_R, -LEG_H)
        .lineTo(0.0, -LEG_H)
        .lineTo(0.0, 0.0)
        .close()
    )
    return profile.revolve(360.0, (0, -1, 0), (0, 1, 0))


# The leg geometry is identical for all three legs; cache one instance.
_leg_cached: cq.Workplane | None = None


def _leg_solid() -> cq.Workplane:
    global _leg_cached
    if _leg_cached is None:
        _leg_cached = _make_leg_solid()
    return _leg_cached


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_cauldron")

    # ---- root: pot body ---------------------------------------------------
    pot = model.part("pot_body")
    pot.visual(
        mesh_from_cadquery(_pot_solid(), "pot_shell"),
        material=CAST_IRON,
        name="pot_shell",
    )

    # ---- legs (3 at 120° intervals, FIXED to pot) -------------------------
    for i in range(LEG_COUNT):
        angle_deg = i * (360.0 / LEG_COUNT)
        angle_rad = math.radians(angle_deg)
        leg = model.part(f"leg_{i}")
        leg.visual(
            mesh_from_cadquery(_leg_solid(), f"leg_body_{i}"),
            material=CAST_IRON_DARK,
            name=f"leg_body_{i}",
        )
        x_pos = LEG_PLACEMENT_R * math.cos(angle_rad)
        y_pos = LEG_PLACEMENT_R * math.sin(angle_rad)
        model.articulation(
            f"pot_to_leg_{i}",
            ArticulationType.FIXED,
            parent=pot,
            child=leg,
            origin=Origin(xyz=(x_pos, y_pos, 0.0)),
        )

    # ---- lid (no bosses — handle removed in this variant) -----------------
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

    # ---- side loop ears (2, mirrored on ±Y, FIXED to pot) -----------------
    for i in range(2):
        ear = model.part(f"ear_{i}")
        ear.visual(
            mesh_from_cadquery(_ear_solid(), f"ear_body_{i}"),
            material=CAST_IRON,
            name=f"ear_body_{i}",
        )
        sign = 1 if i == 0 else -1
        spin = 0.0 if i == 0 else math.pi
        model.articulation(
            f"pot_to_ear_{i}",
            ArticulationType.FIXED,
            parent=pot,
            child=ear,
            origin=Origin(
                xyz=(0.0, sign * EAR_MOUNT_R, EAR_MOUNT_Z),
                rpy=(0.0, 0.0, spin),
            ),
        )

    # ---- lid lift (prismatic, same as parent) -----------------------------
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=pot,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=0.0, upper=LID_LIFT),
    )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pot = object_model.get_part("pot_body")
    lid = object_model.get_part("lid")
    lid_lift = object_model.get_articulation("lid_lift")
    lid_lip = lid.get_visual("lid_lip")
    pot_shell = pot.get_visual("pot_shell")

    # The locating lip intentionally nests inside the open mouth.
    ctx.allow_overlap(
        lid,
        pot,
        elem_a=lid_lip,
        elem_b=pot_shell,
        reason="Lid locating lip nests inside the open pot mouth; the pot is hollow "
        "and the lip keeps 4 mm radial clearance to the inner rim wall.",
    )

    # Ear mounting pads embed slightly into the pot wall (cast-on feature).
    for i in range(2):
        ear_part = object_model.get_part(f"ear_{i}")
        ear_elem = ear_part.get_visual(f"ear_body_{i}")
        ctx.allow_overlap(
            ear_part,
            pot,
            elem_a=ear_elem,
            elem_b=pot_shell,
            reason="Ear mounting pads are intentionally embedded into the pot wall "
            "as cast-on features typical of cast-iron cookware.",
        )
        # Prove the ear is actually in contact with the pot (not floating).
        ctx.expect_contact(
            ear_part,
            pot,
            contact_tol=0.012,
            name=f"ear_{i} contacts the pot wall at the mounting surface",
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
        "pot foot bottom sits at z=0",
        pot_aabb is not None and abs(pot_aabb[0][2]) <= 0.002,
        details=f"pot_aabb={pot_aabb}",
    )
    ctx.check(
        "pot rim height ~0.31 m",
        pot_aabb is not None and 0.30 <= pot_aabb[1][2] <= 0.32,
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
        lid, pot, axes="xy", max_dist=0.002,
        name="lid stays centered over the mouth",
    )
    ctx.expect_within(
        lid, pot, axes="xy", margin=0.001,
        name="lid footprint stays within the belly footprint",
    )
    ctx.expect_contact(
        lid, pot, contact_tol=0.002,
        name="lid seat ring rests on the pot rim (seated, not floating)",
    )
    ctx.expect_within(
        lid, pot, axes="xy",
        inner_elem=lid_lip, outer_elem=pot_shell, margin=0.001,
        name="locating lip stays centered inside the mouth opening",
    )

    # --- prismatic lid lift: straight up, fully clear of the mouth -------
    rest_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_lift: LID_LIFT}):
        lifted_pos = ctx.part_world_position(lid)
        ctx.expect_gap(
            lid, pot, axis="z", min_gap=0.10,
            name="lifted lid clears the pot mouth by ~0.11 m",
        )
        ctx.expect_origin_distance(
            lid, pot, axes="xy", max_dist=0.002,
            name="lid lifts straight up (no lateral drift)",
        )
    ctx.check(
        "lid_lift slides the lid upward by the commanded travel",
        rest_pos is not None
        and lifted_pos is not None
        and abs((lifted_pos[2] - rest_pos[2]) - LID_LIFT) <= 0.001,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    # --- side loop ears: mounted on pot sides, protruding outward --------
    for i in range(2):
        ear_part = object_model.get_part(f"ear_{i}")
        ear_aabb = ctx.part_world_aabb(ear_part)

        # Ear protrudes beyond the pot belly on its side
        if i == 0:
            ctx.check(
                "ear_0 protrudes beyond pot belly on +Y side",
                ear_aabb is not None and pot_aabb is not None
                and ear_aabb[1][1] > pot_aabb[1][1],
                details=f"ear_max_y={ear_aabb[1][1] if ear_aabb else None}, "
                f"pot_max_y={pot_aabb[1][1] if pot_aabb else None}",
            )
        else:
            ctx.check(
                "ear_1 protrudes beyond pot belly on -Y side",
                ear_aabb is not None and pot_aabb is not None
                and ear_aabb[0][1] < pot_aabb[0][1],
                details=f"ear_min_y={ear_aabb[0][1] if ear_aabb else None}, "
                f"pot_min_y={pot_aabb[0][1] if pot_aabb else None}",
            )

        # Ear is mounted near the rim height
        ctx.check(
            f"ear_{i} is mounted near the pot rim (z center between 0.22 and 0.35)",
            ear_aabb is not None
            and ear_aabb[1][2] > 0.22
            and ear_aabb[0][2] < 0.35,
            details=f"ear_aabb={ear_aabb}",
        )

    # --- legs: extend below the pot foot, at 120° intervals --------------
    for i in range(LEG_COUNT):
        leg_part = object_model.get_part(f"leg_{i}")
        leg_aabb = ctx.part_world_aabb(leg_part)

        # Leg extends below the pot foot bottom (z < 0)
        ctx.check(
            f"leg_{i} extends below the pot foot",
            leg_aabb is not None and leg_aabb[0][2] < -0.01,
            details=f"leg_aabb={leg_aabb}",
        )

        # Leg contacts the pot (foot bottom)
        ctx.expect_contact(
            leg_part, pot, contact_tol=0.002,
            name=f"leg_{i} contacts the pot foot bottom",
        )

    # Legs are at approximately equal angular spacing (120° apart)
    leg_positions = []
    for i in range(LEG_COUNT):
        pos = ctx.part_world_position(object_model.get_part(f"leg_{i}"))
        if pos is not None:
            leg_positions.append(pos)
    if len(leg_positions) == LEG_COUNT:
        angles = [math.atan2(p[1], p[0]) for p in leg_positions]
        # Check that the angular spacing is roughly 120° (2π/3 rad)
        sorted_angles = sorted(angles)
        gaps = [
            (sorted_angles[(j + 1) % LEG_COUNT] - sorted_angles[j]) % (2 * math.pi)
            for j in range(LEG_COUNT)
        ]
        ctx.check(
            "legs are at approximately equal angular spacing (~120° apart)",
            all(abs(g - 2 * math.pi / LEG_COUNT) < 0.2 for g in gaps),
            details=f"angle_gaps_rad={gaps}",
        )

    # --- no bail handle in this variant -----------------------------------
    has_handle = False
    try:
        object_model.get_part("loop_handle")
        has_handle = True
    except Exception:
        pass
    ctx.check(
        "no bail handle in side_loop_ears variant",
        not has_handle,
        details="loop_handle part should not exist in this variant",
    )

    return ctx.report()


object_model = build_object_model()
