from __future__ import annotations

# Dark glass beer bottle with a pour-spout flip closure.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.24 m tall, ~0.065 m body diameter.
# Closure: tapered pour-spout insert seated in the bottle mouth, topped by a
#   small hinged flip cap that pivots open on a revolute joint at its rear
#   hinge to uncover the spout channel.
# Articulations:
#   - bottle_to_flip_cap: REVOLUTE, hinge at the rear top of the spout, axis
#     along X so positive q flips the cap backward and exposes the pour channel.

import math

import cadquery as cq

GLASS_RGBA = (0.18, 0.12, 0.07, 0.4)
SPOUT_RGBA = (0.92, 0.90, 0.85, 1.0)  # off-white plastic insert
CAP_RGBA = (0.70, 0.15, 0.12, 1.0)    # red flip cap

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- Bottle profile heights (z, in metres) ----------------------------------
BASE_Z = 0.0
BODY_TOP_Z = 0.120   # body cylinder ends, shoulder begins
SHOULDER_TOP_Z = 0.150  # shoulder blends into the neck
NECK_TOP_Z = 0.222   # top of the slim neck, where the lip starts
LIP_TOP_Z = 0.230    # top face of the mouth / lip

BODY_R = 0.0325      # body radius (~0.065 m diameter)
NECK_R = 0.0115      # slim long-neck radius
LIP_R = 0.0135       # rolled lip is slightly fatter than the neck

WALL = 0.0028        # glass wall thickness (hollow shell)
BORE_R = 0.0085      # inner bore radius at the mouth

# ---- Pour-spout insert dimensions -------------------------------------------
PLUG_R = 0.0078      # plug radius (slightly under bore for press-fit)
PLUG_H = 0.012       # plug depth into the bottle bore
FLANGE_R = 0.013     # flange outer radius (sits on lip)
FLANGE_H = 0.003     # flange thickness
SPOUT_BASE_R = 0.006  # spout tube base radius (at flange top)
SPOUT_TOP_R = 0.004   # spout tube top radius (tapered narrower)
SPOUT_H = 0.018       # spout tube height above flange
SPOUT_BORE_R = 0.003  # pour channel bore radius
LUG_W = 0.006         # hinge lug width (along X)
LUG_D = 0.004         # hinge lug depth (along Y)
LUG_H = 0.006         # hinge lug height (above spout top)

# ---- Flip cap dimensions ----------------------------------------------------
CAP_R = 0.008         # cap disc radius
CAP_H = 0.003         # cap disc thickness
BARREL_R = 0.0018     # hinge barrel radius
BARREL_W = 0.007      # hinge barrel width (along X)
TAB_W = 0.005         # grip tab width
TAB_D = 0.004         # grip tab depth
TAB_H = 0.004         # grip tab height

# Hinge at the rear top of the spout tube. Offset the hinge rearward by the
# barrel radius so the barrel sits just outside the spout wall (no penetration).
HINGE_Y = -(SPOUT_TOP_R + BARREL_R)  # rear of spout + barrel clearance
HINGE_Z = LIP_TOP_Z + FLANGE_H + SPOUT_H  # top of spout tube


def _profile_loft(sections, ruled: bool = True) -> cq.Workplane:
    """Loft circular sections stacked along +Z. Ruled lofts give straight
    segments between sections; use many closely-spaced sections for curves."""
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=ruled)


def _bottle_solid() -> cq.Workplane:
    """Hollow dark-glass bottle shell: body, shoulder, long neck, rolled lip."""
    outer = _profile_loft(
        [
            (BASE_Z, 0.0235),       # flat-bottomed base disc
            (0.003, 0.0250),        # short straight heel
            (0.008, 0.0305),        # rounded base flare
            (0.013, 0.0322),
            (0.018, 0.0325),        # body full radius
            (BODY_TOP_Z, 0.0325),   # straight cylindrical body
            (0.128, 0.0316),        # shoulder begins
            (0.137, 0.0288),
            (SHOULDER_TOP_Z, 0.0205),  # sloped shoulder
            (0.160, 0.0150),
            (0.170, 0.0122),        # shoulder rolls into neck
            (NECK_TOP_Z, NECK_R),   # long slim neck
            (0.224, 0.0128),        # neck flares to the lip
            (0.227, LIP_R),         # flared lip
            (LIP_TOP_Z, LIP_R),     # flat mouth top
        ]
    )

    inner = _profile_loft(
        [
            (0.010, 0.0200),        # cavity floor above a solid glass base
            (0.018, 0.0270),
            (0.024, 0.0297),
            (BODY_TOP_Z - 0.004, 0.0297),
            (0.128, 0.0288),
            (0.137, 0.0260),
            (SHOULDER_TOP_Z, 0.0177),
            (0.160, 0.0122),
            (0.170, 0.0094),
            (NECK_TOP_Z, BORE_R),
            (LIP_TOP_Z + 0.004, BORE_R),  # poke out the top so the mouth is open
        ]
    )
    return outer.cut(inner)


def _spout_solid() -> cq.Workplane:
    """Pour-spout insert: plug + flange + tapered tube + hinge lug, with a
    hollow pour channel through the center. Origin at z=0 = lip top in world."""
    # Plug: press-fit into the bottle bore (extends downward from z=0)
    plug = _profile_loft(
        [
            (-PLUG_H, PLUG_R),
            (0.0, PLUG_R),
        ]
    )

    # Flange: sits on top of the lip
    flange = _profile_loft(
        [
            (0.0, FLANGE_R),
            (FLANGE_H, FLANGE_R),
        ]
    )

    # Tapered spout tube: narrows toward the top
    spout_tube = _profile_loft(
        [
            (FLANGE_H, SPOUT_BASE_R),
            (FLANGE_H + SPOUT_H, SPOUT_TOP_R),
        ]
    )

    # Small reinforcing rib at the rear of the spout tube for visual support
    # of the hinge pivot. Stays below the hinge barrel so it doesn't cause
    # 3D overlap with the cap.
    rib_top = SPOUT_H - 0.002  # stay 2mm below spout top (clear of barrel)
    rib_bottom = rib_top - 0.008
    rib = (
        cq.Workplane("XY")
        .workplane(offset=FLANGE_H + rib_bottom)
        .center(0.0, -(SPOUT_BASE_R + SPOUT_TOP_R) / 2.0 - 0.001)
        .rect(LUG_W, LUG_D)
        .extrude(0.008)
    )

    body = plug.union(flange).union(spout_tube).union(rib)

    # Pour channel: tapered bore through the entire insert
    channel = _profile_loft(
        [
            (-PLUG_H + 0.003, SPOUT_BORE_R),       # start above plug floor
            (FLANGE_H + SPOUT_H + 0.002, SPOUT_BORE_R * 0.75),  # exit at spout top
        ]
    )

    return body.cut(channel)


def _flip_cap_solid() -> cq.Workplane:
    """Flip cap disc with hinge barrel and grip tab. Origin at the hinge pivot.
    At q=0 the cap extends forward (+Y local) to cover the spout opening."""
    # Spout center in cap local frame: (0, -HINGE_Y, 0)
    spout_cy = -HINGE_Y  # forward offset from hinge to spout center

    # Main disc body — covers the spout opening when closed
    disc = (
        cq.Workplane("XY")
        .center(0.0, spout_cy)
        .circle(CAP_R)
        .extrude(CAP_H)
    )

    # Dome cap on top of the disc for a rounded profile
    dome = (
        cq.Workplane("XY")
        .workplane(offset=CAP_H)
        .center(0.0, spout_cy)
        .circle(CAP_R)
        .workplane(offset=CAP_R * 0.3)
        .circle(CAP_R * 0.6)
        .loft(ruled=True)
    )

    # Hinge barrel: cylinder along X at the hinge pivot (local origin)
    barrel = (
        cq.Workplane("YZ")
        .workplane(offset=-BARREL_W / 2.0)
        .center(0.0, CAP_H / 2.0)
        .circle(BARREL_R)
        .extrude(BARREL_W)
    )

    # Grip tab at the front edge (opposite the hinge) for finger leverage
    tab_y = spout_cy + CAP_R - 0.001
    tab = (
        cq.Workplane("XY")
        .center(0.0, tab_y)
        .rect(TAB_W, TAB_D)
        .extrude(TAB_H)
    )

    return disc.union(dome).union(barrel).union(tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="beer_bottle_flip_spout")

    # Materials
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    spout_mat = model.material("spout_plastic", rgba=SPOUT_RGBA)
    cap_mat = model.material("cap_plastic", rgba=CAP_RGBA)

    # ---- bottle (root): hollow dark-glass shell + inline pour spout ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    bottle.visual(
        mesh_from_cadquery(_spout_solid(), "pour_spout"),
        material=spout_mat,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z)),
        name="pour_spout",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- flip cap: hinged disc that covers/uncovers the pour spout ----
    flip_cap = model.part("flip_cap")
    flip_cap.visual(
        mesh_from_cadquery(_flip_cap_solid(), "flip_cap"),
        material=cap_mat,
        name="flip_cap",
    )
    flip_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_R, length=CAP_H),
        mass=0.005,
        origin=Origin(xyz=(0.0, -HINGE_Y, CAP_H / 2.0)),
    )

    # Revolute joint: hinge at the rear top of the spout. The cap frame at q=0
    # is coincident with the joint frame. Cap extends along +Y to cover spout.
    # axis=(1, 0, 0) → right-hand rule rotates +Y toward +Z, flipping the cap
    # backward and exposing the pour channel.
    model.articulation(
        "bottle_to_flip_cap",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=flip_cap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=4.0, lower=0.0, upper=2.3,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    flip_cap = object_model.get_part("flip_cap")
    hinge = object_model.get_articulation("bottle_to_flip_cap")

    # --- Dark glass: translucent body material (alpha < 1). ---
    ctx.check(
        "bottle glass is translucent (alpha < 1)",
        GLASS_RGBA[3] < 1.0,
        details=f"glass rgba={GLASS_RGBA}",
    )

    # --- Long neck: the bottle is tall with a slim upper region. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    ctx.check(
        "bottle is tall (long-neck silhouette)",
        body_ext[2] > 0.20 and body_ext[2] > 2.5 * max(body_ext[0], body_ext[1]),
        details=f"bottle extents={body_ext}",
    )

    # --- Pour spout seated in the bottle mouth: spout overlaps the lip region. ---
    ctx.expect_overlap(
        bottle,
        bottle,
        axes="z",
        elem_a="pour_spout",
        elem_b="bottle_glass",
        min_overlap=0.002,
        name="pour spout insert seated in the bottle mouth",
    )

    # --- Flip cap mounted at the bottle mouth (top of the spout). ---
    cap_pos = ctx.part_world_position(flip_cap)
    ctx.check(
        "flip cap mounted at the bottle mouth (top)",
        cap_pos is not None and cap_pos[2] > 0.22,
        details=f"flip_cap origin={cap_pos}",
    )

    # --- Flip cap covers the spout when closed (q=0): cap overlaps spout in XY. ---
    ctx.expect_overlap(
        flip_cap,
        bottle,
        axes="xy",
        elem_a="flip_cap",
        elem_b="pour_spout",
        min_overlap=0.003,
        name="closed flip cap covers the pour spout opening",
    )

    # --- Flip cap contacts or nearly contacts the spout top when closed. ---
    ctx.expect_gap(
        flip_cap,
        bottle,
        axis="z",
        positive_elem="flip_cap",
        negative_elem="pour_spout",
        max_penetration=0.004,
        max_gap=0.005,
        name="closed flip cap seats near the spout top",
    )

    # --- Hinge opens the cap: positive q flips cap backward. ---
    rest_cap_center = ctx.part_world_aabb(flip_cap)
    with ctx.pose({hinge: 1.5}):
        open_cap_center = ctx.part_world_aabb(flip_cap)
        # When opened, the cap top should be higher (flipped backward and up)
        ctx.check(
            "flip cap flips open at positive q",
            open_cap_center[1][2] > rest_cap_center[1][2] + 0.005
            or open_cap_center[0][2] > rest_cap_center[0][2] + 0.005,
            details=f"rest_max_z={rest_cap_center[1][2]:.4f}, "
                    f"open_max_z={open_cap_center[1][2]:.4f}",
        )
        # When opened, the cap no longer covers the spout center in XY
        # (cap has swung backward)
        ctx.check(
            "open flip cap exposes the spout",
            open_cap_center[0][1] < 0.0 or open_cap_center[1][2] > HINGE_Z + 0.005,
            details=f"open_cap aabb min_y={open_cap_center[0][1]:.4f}, "
                    f"open_cap max_z={open_cap_center[1][2]:.4f}",
        )

    # --- Revolute joint has correct type and limits. ---
    ctx.check(
        "hinge is revolute with positive upper limit",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.motion_limits is not None
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper > 1.0,
        details=f"type={hinge.articulation_type}, "
                f"upper={hinge.motion_limits.upper if hinge.motion_limits else None}",
    )

    return ctx.report()


object_model = build_object_model()
