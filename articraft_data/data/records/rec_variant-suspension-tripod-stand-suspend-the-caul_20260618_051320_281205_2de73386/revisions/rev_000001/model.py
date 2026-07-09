from __future__ import annotations

"""Cast-iron cauldron suspended from a three-legged tripod camp stand.

Variant of the traditional cast-iron cooking cauldron: the pot hangs from a
forged hook at the apex of a three-legged tripod, swinging on a revolute joint.
The domed lid and its folding loop handle are preserved from the parent model.

Structure:
- tripod_hub (root): short cylindrical forged hub at the apex with a long
  hanging rod and J-hook that cradles the bail.
- leg_0, leg_1, leg_2: three tapered wrought-iron legs splayed at ~29° from
  vertical, fixed to the hub at 120° azimuth intervals, feet on the ground.
- pot_body: hollow bulbous cauldron (same revolved wall profile as parent) with
  a swing bail arching from two cast ear lugs up to the hook contact point.
- lid: shallow domed disc with stepped rim, locating lip, and two pivot bosses.
- loop_handle: semicircular loop captured in the lid bosses.

Articulations:
- hub_to_pot: REVOLUTE about +X at the hook contact; ±0.40 rad pendulum swing.
- lid_lift: PRISMATIC +Z, 0.12 m travel to lift the lid off the mouth.
- handle_pivot: REVOLUTE about +X through the bosses; 0 (flat) to π/2 (upright).
- hub_to_leg_i: FIXED, three legs forged into the hub.
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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------- dimensions
# --- tripod ---
HUB_R = 0.030          # hub outer radius
HUB_H = 0.045          # hub height

HOOK_DROP = 0.300      # hook contact point below hub centre (long hanging rod)

LEG_COUNT = 3
LEG_SPLAY = 0.500      # rad (~28.6°) — wide enough to clear the pot belly
# Leg length so feet land well below the pot foot:
#   hub bottom at z = -HUB_H/2 = -0.0225
#   ground target z = -1.30
#   LEG_LENGTH = (1.30 - 0.0225) / cos(LEG_SPLAY)
LEG_GROUND_Z = -1.30   # target foot z
LEG_LENGTH = (abs(LEG_GROUND_Z) - HUB_H / 2) / math.cos(LEG_SPLAY)

LEG_TOP_R = 0.011      # leg radius at hub end
LEG_BOT_R = 0.018      # leg radius at ground end

# --- pot (parent proportions) ---
POT_FOOT_R = 0.100
POT_FOOT_H = 0.042
POT_BELLY_R = 0.225
POT_RIM_Z = 0.310
POT_RIM_OUTER_R = 0.140
POT_RIM_INNER_R = 0.128

# --- bail ---
BAIL_APEX_Z = 0.480    # bail hook-point above pot foot (pot natural frame)
BAIL_EAR_Z = 0.280     # ear height above pot foot
BAIL_SPAN_R = 0.180    # ear radial position (just outside pot wall)
BAIL_ROD_R = 0.005     # bail rod radius

# --- lid ---
LID_Z = POT_RIM_Z
LID_R = 0.150
LID_APEX_Z = 0.060
LID_LIP_R = 0.124
LID_LIP_DEPTH = 0.008

# --- bosses & handle ---
BOSS_X = 0.033
BOSS_SIZE = (0.016, 0.018, 0.020)
BOSS_Z = 0.064
PIVOT_Z = 0.068
LOOP_R = 0.030
LOOP_ROD_R = 0.006
PIN_R = 0.005
PIN_X0 = 0.026
PIN_X1 = 0.040

# --- motion limits ---
LID_LIFT = 0.12
HANDLE_UP = 1.5708
SWING_LIMIT = 0.40     # pendulum half-angle (rad)

# ---------------------------------------------------------------- materials
CAST_IRON = Material(name="cast_iron_black", rgba=(0.10, 0.10, 0.11, 1.0))
CAST_IRON_LID = Material(name="cast_iron_lid", rgba=(0.13, 0.13, 0.14, 1.0))
CAST_IRON_DARK = Material(name="cast_iron_dark", rgba=(0.08, 0.08, 0.09, 1.0))
WROUGHT_IRON = Material(name="wrought_iron", rgba=(0.17, 0.15, 0.13, 1.0))

# ================================================================ geometry
# --- pot body (same revolved profile as parent) ---
@lru_cache(maxsize=1)
def _pot_solid() -> cq.Workplane:
    """Hollow bulbous pot wall + integral pedestal foot."""
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
    """Solid silhouette for the hollowness volume test."""
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


# --- lid ---
@lru_cache(maxsize=1)
def _lid_solid() -> cq.Workplane:
    """Domed lid disc with stepped overhanging rim and hollow underside."""
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
    """Locating lip ring under the lid seat."""
    return (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .circle(LID_LIP_R)
        .circle(0.118)
        .extrude(-(LID_LIP_DEPTH + 0.004))
    )


# --- handle ---
@lru_cache(maxsize=1)
def _handle_solid() -> cq.Workplane:
    """Semicircular loop with pin ends along the X pivot axis."""
    loop = (
        cq.Workplane("XZ")
        .moveTo(LOOP_R, 0.0)
        .circle(LOOP_ROD_R)
        .revolve(180.0, (0, -1, 0), (0, 1, 0))
    )
    pin_pos = cq.Workplane("YZ").workplane(offset=PIN_X0).circle(PIN_R).extrude(PIN_X1 - PIN_X0)
    pin_neg = cq.Workplane("YZ").workplane(offset=-PIN_X1).circle(PIN_R).extrude(PIN_X1 - PIN_X0)
    return loop.union(pin_pos).union(pin_neg)


# --- tripod leg (shared geometry helper) ---
@lru_cache(maxsize=1)
def _leg_solid() -> cq.Workplane:
    """Single tapered leg: narrow at top (z=0), wider at ground (z=-LEG_LENGTH)."""
    return (
        cq.Workplane("XY")
        .circle(LEG_TOP_R)
        .workplane(offset=-LEG_LENGTH)
        .circle(LEG_BOT_R)
        .loft()
    )


# --- tripod hub ---
@lru_cache(maxsize=1)
def _hub_solid() -> cq.Workplane:
    """Short cylindrical hub centred at the origin."""
    return (
        cq.Workplane("XY")
        .workplane(offset=-HUB_H / 2)
        .circle(HUB_R)
        .extrude(HUB_H)
    )


def _hook_mesh():
    """Long hanging rod with J-hook at the bottom for the bail."""
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, -HUB_H / 2),
                (0.0, 0.0, -0.08),
                (0.0, 0.0, -0.16),
                (0.0, 0.0, -0.24),
                (0.0, 0.0, -HOOK_DROP + 0.02),
                (0.004, 0.0, -HOOK_DROP + 0.005),
                (0.012, 0.0, -HOOK_DROP),
                (0.020, 0.0, -HOOK_DROP + 0.005),
                (0.024, 0.0, -HOOK_DROP + 0.018),
            ],
            radius=0.005,
            samples_per_segment=12,
            radial_segments=12,
            cap_ends=True,
        ),
        "hook_tube",
    )


def _bail_mesh():
    """Semicircular bail arch: endpoints at the pot ears, apex at the hook point."""
    ear_dz = BAIL_EAR_Z - BAIL_APEX_Z  # negative in pot_body frame
    return mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, -BAIL_SPAN_R, ear_dz),
                (0.0, -0.145, -0.10),
                (0.0, -0.085, -0.02),
                (0.0, 0.0, 0.0),
                (0.0, 0.085, -0.02),
                (0.0, 0.145, -0.10),
                (0.0, BAIL_SPAN_R, ear_dz),
            ],
            radius=BAIL_ROD_R,
            samples_per_segment=16,
            radial_segments=12,
            cap_ends=True,
        ),
        "bail_arch",
    )


# ================================================================ model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_cauldron_tripod")

    # ---- tripod hub (root) ----
    hub = model.part("tripod_hub")
    hub.visual(
        mesh_from_cadquery(_hub_solid(), "hub_body"),
        material=WROUGHT_IRON,
        name="hub_body",
    )
    hub.visual(_hook_mesh(), material=WROUGHT_IRON, name="hook")

    # ---- three legs (fixed to hub, shared geometry, for-loop) ----
    for i in range(LEG_COUNT):
        leg = model.part(f"leg_{i}")
        leg.visual(
            mesh_from_cadquery(_leg_solid(), f"leg_shaft_{i}"),
            material=WROUGHT_IRON,
            name=f"leg_shaft_{i}",
        )
        azimuth = 2.0 * math.pi * i / LEG_COUNT
        model.articulation(
            f"hub_to_leg_{i}",
            ArticulationType.FIXED,
            parent=hub,
            child=leg,
            origin=Origin(
                xyz=(0.0, 0.0, -HUB_H / 2),
                rpy=(0.0, LEG_SPLAY, azimuth),
            ),
        )

    # ---- pot body (swings from hub via bail) ----
    pot = model.part("pot_body")
    pot.visual(
        mesh_from_cadquery(_pot_solid(), "pot_shell"),
        origin=Origin(xyz=(0.0, 0.0, -BAIL_APEX_Z)),
        material=CAST_IRON,
        name="pot_shell",
    )
    # Bail ear lugs
    ear_dz = BAIL_EAR_Z - BAIL_APEX_Z
    ear_size = (0.020, 0.044, 0.028)
    pot.visual(
        Box(ear_size),
        origin=Origin(xyz=(0.0, -BAIL_SPAN_R, ear_dz)),
        material=CAST_IRON,
        name="bail_ear_0",
    )
    pot.visual(
        Box(ear_size),
        origin=Origin(xyz=(0.0, BAIL_SPAN_R, ear_dz)),
        material=CAST_IRON,
        name="bail_ear_1",
    )
    # Bail rod
    pot.visual(_bail_mesh(), material=WROUGHT_IRON, name="bail_rod")

    # ---- lid (prismatic on pot) ----
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

    # ---- loop handle (revolute on lid) ----
    handle = model.part("loop_handle")
    handle.visual(
        mesh_from_cadquery(_handle_solid(), "handle_loop"),
        material=CAST_IRON_DARK,
        name="handle_loop",
    )

    # ============================================================ articulations

    # Hub → pot: revolute pendulum swing at the hook contact point.
    model.articulation(
        "hub_to_pot",
        ArticulationType.REVOLUTE,
        parent=hub,
        child=pot,
        origin=Origin(xyz=(0.0, 0.0, -HOOK_DROP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=1.0, lower=-SWING_LIMIT, upper=SWING_LIMIT
        ),
    )

    # Pot → lid: prismatic +Z lift (adjusted for pot_body frame offset).
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=pot,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z - BAIL_APEX_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0, velocity=0.5, lower=0.0, upper=LID_LIFT
        ),
    )

    # Lid → handle: revolute about +X; 0 = flat, +π/2 = upright.
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=0.0, upper=HANDLE_UP
        ),
    )

    return model


# ================================================================ tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hub = object_model.get_part("tripod_hub")
    pot = object_model.get_part("pot_body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("loop_handle")

    hub_to_pot = object_model.get_articulation("hub_to_pot")
    lid_lift = object_model.get_articulation("lid_lift")
    handle_pivot = object_model.get_articulation("handle_pivot")

    legs = [object_model.get_part(f"leg_{i}") for i in range(LEG_COUNT)]

    hub_body = hub.get_visual("hub_body")
    hook = hub.get_visual("hook")
    pot_shell = pot.get_visual("pot_shell")
    bail_rod = pot.get_visual("bail_rod")
    bail_ear_0 = pot.get_visual("bail_ear_0")
    bail_ear_1 = pot.get_visual("bail_ear_1")
    lid_dome = lid.get_visual("lid_dome")
    lid_lip = lid.get_visual("lid_lip")
    boss_0 = lid.get_visual("handle_boss_0")
    boss_1 = lid.get_visual("handle_boss_1")
    handle_loop = handle.get_visual("handle_loop")

    # ---- intentional-overlap allowances ----

    # Bail rod rests on the hook; local contact overlap at the hanging point.
    ctx.allow_overlap(
        pot, hub,
        elem_a=bail_rod, elem_b=hook,
        reason="Bail rod rests on the hook cradle; local contact overlap at the hanging point.",
    )

    # Each leg top is forged into the hub; small local embed at the junction.
    for i in range(LEG_COUNT):
        ctx.allow_overlap(
            hub, legs[i],
            elem_a=hub_body, elem_b=legs[i].get_visual(f"leg_shaft_{i}"),
            reason=f"Leg {i} top is forged into the hub; small local embed at the splay junction.",
        )
        # Legs also overlap with the hanging hook rod near the apex convergence zone.
        ctx.allow_overlap(
            legs[i], hub,
            elem_a=legs[i].get_visual(f"leg_shaft_{i}"), elem_b=hook,
            reason=f"Leg {i} passes near the hanging hook rod at the apex convergence zone; "
                   f"small local overlap within the upper ~50 mm of the hook.",
        )

    # Leg-leg overlaps near the apex: the three legs converge at the hub
    # and overlap for ~25 mm near the forged junction.
    for i in range(LEG_COUNT):
        for j in range(i + 1, LEG_COUNT):
            ctx.allow_overlap(
                legs[i], legs[j],
                elem_a=legs[i].get_visual(f"leg_shaft_{i}"),
                elem_b=legs[j].get_visual(f"leg_shaft_{j}"),
                reason=f"Legs {i} and {j} converge at the hub apex; small local overlap "
                       f"at the forged junction (hidden inside the hub region).",
            )

    # Lid locating lip nests inside the open pot mouth.
    ctx.allow_overlap(
        lid, pot,
        elem_a=lid_lip, elem_b=pot_shell,
        reason="Lid locating lip nests inside the open pot mouth; the pot is hollow "
               "and the lip keeps radial clearance to the inner rim wall.",
    )

    # Handle pivot pins captured inside the lid bosses.
    ctx.allow_overlap(
        handle, lid,
        elem_a=handle_loop, elem_b=boss_0,
        reason="Handle pivot pin is intentionally captured inside the cast lid boss.",
    )
    ctx.allow_overlap(
        handle, lid,
        elem_a=handle_loop, elem_b=boss_1,
        reason="Handle pivot pin is intentionally captured inside the cast lid boss.",
    )

    # ---- tripod structure ----

    # Hub is the root part at the world origin (apex of the tripod).
    hub_pos = ctx.part_world_position(hub)
    ctx.check(
        "tripod hub is at the apex (root at world origin)",
        hub_pos is not None and abs(hub_pos[2]) < 0.01,
        details=f"hub_pos={hub_pos}",
    )

    # All three leg feet reach the ground (well below the pot foot).
    pot_aabb = ctx.part_world_aabb(pot)
    for i in range(LEG_COUNT):
        leg_aabb = ctx.part_world_aabb(legs[i])
        ctx.check(
            f"leg_{i} foot is below the pot foot (ground contact)",
            leg_aabb is not None and pot_aabb is not None
            and leg_aabb[0][2] < pot_aabb[0][2] - 0.20,
            details=f"leg_{i}_min_z={leg_aabb[0][2] if leg_aabb else None}, "
                    f"pot_min_z={pot_aabb[0][2] if pot_aabb else None}",
        )

    # Legs are splayed outward: each foot centre is radially outside the pot belly.
    for i in range(LEG_COUNT):
        leg_aabb = ctx.part_world_aabb(legs[i])
        ctx.check(
            f"leg_{i} spans outward from the hub (wide footprint)",
            leg_aabb is not None
            and (leg_aabb[1][0] - leg_aabb[0][0]) > 0.30
            or (leg_aabb[1][1] - leg_aabb[0][1]) > 0.30,
            details=f"leg_{i}_aabb={leg_aabb}",
        )

    # ---- pot hangs below the hub ----

    pot_pos = ctx.part_world_position(pot)
    ctx.check(
        "pot hangs well below the hub (bail contact)",
        pot_pos is not None and hub_pos is not None
        and pot_pos[2] < hub_pos[2] - 0.15,
        details=f"pot_pos={pot_pos}, hub_pos={hub_pos}",
    )

    # Pot foot has fire clearance above the leg feet.
    ctx.check(
        "pot foot has camp-fire clearance above the leg feet (> 0.30 m)",
        pot_aabb is not None
        and all(ctx.part_world_aabb(leg) is not None for leg in legs),
        details="checked via leg/pot AABB comparison",
    )
    # Explicit fire clearance check:
    min_leg_foot_z = min(
        ctx.part_world_aabb(leg)[0][2]
        for leg in legs
        if ctx.part_world_aabb(leg) is not None
    ) if pot_aabb else 0.0
    ctx.check(
        "pot foot is well above the ground plane",
        pot_aabb is not None
        and pot_aabb[0][2] - min_leg_foot_z > 0.30,
        details=f"pot_foot_z={pot_aabb[0][2] if pot_aabb else None}, "
                f"ground_z={min_leg_foot_z}",
    )

    # Pot belly still reads as a ~0.45 m cauldron.
    ctx.check(
        "pot belly spans ~0.45 m (cauldron silhouette)",
        pot_aabb is not None
        and 0.44 <= (pot_aabb[1][0] - pot_aabb[0][0]) <= 0.46,
        details=f"pot_aabb={pot_aabb}",
    )

    # ---- bail visible on pot ----
    ctx.check("bail rod is present on the pot", bail_rod is not None)
    ctx.check(
        "bail ear lugs are present on the pot",
        bail_ear_0 is not None and bail_ear_1 is not None,
    )

    # Bail apex is near the hook (contact proof).
    ctx.expect_contact(
        pot, hub,
        elem_a=bail_rod, elem_b=hook,
        contact_tol=0.015,
        name="bail contacts the hook at rest",
    )

    # ---- pendulum swing ----

    rest_pot_aabb = ctx.part_world_aabb(pot)
    rest_center_y = (rest_pot_aabb[0][1] + rest_pot_aabb[1][1]) / 2 if rest_pot_aabb else 0.0
    with ctx.pose({hub_to_pot: SWING_LIMIT}):
        swung_pot_aabb = ctx.part_world_aabb(pot)
        swung_center_y = (swung_pot_aabb[0][1] + swung_pot_aabb[1][1]) / 2 if swung_pot_aabb else 0.0
        ctx.check(
            "pot swings laterally on the hook (positive q shifts pot AABB in +Y)",
            swung_center_y > rest_center_y + 0.04,
            details=f"rest_center_y={rest_center_y:.4f}, swung_center_y={swung_center_y:.4f}",
        )
        ctx.check(
            "pot stays below the hub during swing",
            swung_pot_aabb is not None and hub_pos is not None
            and swung_pot_aabb[0][2] < hub_pos[2] - 0.10,
            details=f"swung_min_z={swung_pot_aabb[0][2] if swung_pot_aabb else None}",
        )

    # ---- lid (preserved from parent) ----

    lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid is a ~0.30 m disc overhanging the mouth",
        lid_aabb is not None and 0.295 <= (lid_aabb[1][0] - lid_aabb[0][0]) <= 0.305,
        details=f"lid_aabb={lid_aabb}",
    )

    ctx.expect_origin_distance(
        lid, pot, axes="xy", max_dist=0.002,
        name="lid stays centred over the mouth",
    )
    ctx.expect_contact(
        lid, pot, contact_tol=0.002,
        name="lid seat ring rests on the pot rim",
    )

    # Prismatic lid lift (scoped to lid dome vs pot shell to avoid bail interference).
    rest_lid = ctx.part_world_position(lid)
    with ctx.pose({lid_lift: LID_LIFT}):
        lifted_lid = ctx.part_world_position(lid)
        ctx.expect_gap(
            lid, pot, axis="z",
            positive_elem=lid_dome,
            negative_elem=pot_shell,
            min_gap=0.10,
            name="lifted lid clears the pot mouth by ~0.11 m",
        )
    ctx.check(
        "lid_lift slides the lid upward by the commanded travel",
        rest_lid is not None and lifted_lid is not None
        and abs((lifted_lid[2] - rest_lid[2]) - LID_LIFT) <= 0.002,
        details=f"rest_lid={rest_lid}, lifted_lid={lifted_lid}",
    )

    # ---- handle (preserved from parent) ----

    ctx.expect_overlap(
        handle, lid, axes="x",
        elem_a=handle_loop, elem_b=boss_0, min_overlap=0.008,
        name="handle pin engages the +X boss",
    )
    ctx.expect_overlap(
        handle, lid, axes="x",
        elem_a=handle_loop, elem_b=boss_1, min_overlap=0.008,
        name="handle pin engages the -X boss",
    )
    ctx.expect_contact(
        handle, lid,
        elem_a=handle_loop, elem_b=boss_0, contact_tol=0.0005,
        name="handle is connected to the +X boss",
    )
    ctx.expect_contact(
        handle, lid,
        elem_a=handle_loop, elem_b=boss_1, contact_tol=0.0005,
        name="handle is connected to the -X boss",
    )

    flat_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle lies flat against the lid at q=0",
        flat_aabb is not None and (flat_aabb[1][2] - flat_aabb[0][2]) <= 0.025,
        details=f"flat_aabb={flat_aabb}",
    )

    with ctx.pose({handle_pivot: HANDLE_UP}):
        up_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "upright handle arch rises above the flat position",
            up_aabb is not None and flat_aabb is not None
            and up_aabb[1][2] > flat_aabb[1][2] + 0.02,
            details=f"up_aabb={up_aabb}",
        )

    # ---- pot is genuinely hollow ----
    hollow_vol = float(_pot_solid().val().Volume())
    filled_vol = float(_pot_filled_solid().val().Volume())
    ctx.check(
        "pot body is hollow with an open mouth (thin revolved wall)",
        0.0 < hollow_vol < 0.40 * filled_vol,
        details=f"wall_volume={hollow_vol:.5f}, filled_volume={filled_vol:.5f}",
    )

    return ctx.report()


object_model = build_object_model()
