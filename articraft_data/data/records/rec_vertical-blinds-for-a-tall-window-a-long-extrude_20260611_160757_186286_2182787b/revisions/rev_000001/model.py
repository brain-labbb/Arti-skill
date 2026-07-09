from __future__ import annotations

# Vertical blinds for a tall window.
# Reference image: picture/Curtain/blind/001.png
#
# Layout (world frame):
#   X = along the headrail (left end at -X), Y = depth (front at -Y), Z = up.
#   The blind hangs in space from its extruded-aluminum headrail (root part).
#
# Mechanism:
#   headrail (root)
#     -> vane_set_traverse (prismatic, X): carrier_train, one rigid train of
#        20 carrier trucks tied together by a spacer rail, sliding under the rail
#     -> vane_NN_tilt (revolute, vertical Z axis) per vane; vane_01_tilt is the
#        driver and vane_02..vane_20 mimic it with multiplier 1.0 so all vanes
#        tilt in unison
#     -> control_chain_swing (revolute, X): control chain + black tassel
#        hanging from a boss at the left end of the headrail

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

# ----------------------------------------------------------------------------
# Shared layout constants
# ----------------------------------------------------------------------------
VANE_COUNT = 20
VANE_WIDTH = 0.089          # standard 89 mm vane
VANE_THICKNESS = 0.0025     # stiffened fabric strip
VANE_DROP = 2.0             # 2 m drop for a tall window
VANE_PITCH = 0.075          # carrier spacing -> 14 mm side overlap
VANE_STAGGER = 0.0017       # alternating front/back offset so closed vanes
                            # overlap in X without interpenetrating

RAIL_LENGTH = 1.62
RAIL_DEPTH = 0.045
RAIL_HEIGHT = 0.045
RAIL_BOTTOM_Z = 2.080       # underside of the extrusion
RAIL_TOP_Z = RAIL_BOTTOM_Z + RAIL_HEIGHT

STEM_BOTTOM_Z = 2.056       # bottom of each carrier hook stem = vane pivot
TILT_LIMIT = 1.5708         # +/- 90 deg vane tilt
TRAVERSE_LIMIT = 0.045      # +/- 45 mm rigid-train traverse along the rail

CHAIN_X = -0.78             # control chain hangs at the left rail end
CHAIN_Y = -0.035            # just in front of the rail fascia
CHAIN_TOP_Z = 2.066
CHAIN_LENGTH = 1.16

# Leftmost carrier; carriers are evenly pitched along the rail.
FIRST_VANE_X = -0.5 * (VANE_COUNT - 1) * VANE_PITCH  # -0.7125


def _vane_x(index: int) -> float:
    """World X of carrier/vane `index` (1-based) at zero traverse."""
    return FIRST_VANE_X + (index - 1) * VANE_PITCH


def _vane_y_offset(index: int) -> float:
    """Alternating front/back stagger so overlapped closed vanes do not collide."""
    return VANE_STAGGER if index % 2 == 1 else -VANE_STAGGER


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_blinds")

    aluminum = model.material("extruded_aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    fabric = model.material("gray_vane_fabric", rgba=(0.45, 0.45, 0.46, 1.0))
    carrier_plastic = model.material("carrier_white_plastic", rgba=(0.90, 0.90, 0.90, 1.0))
    chain_metal = model.material("chain_silver", rgba=(0.72, 0.73, 0.74, 1.0))
    tassel_black = model.material("tassel_black", rgba=(0.05, 0.05, 0.055, 1.0))

    # ------------------------------------------------------------------
    # Headrail: long extruded-aluminum channel with a top lip, end caps,
    # and a chain boss at the left end. Root part of the assembly.
    # ------------------------------------------------------------------
    headrail = model.part("headrail")
    headrail.visual(
        Box((RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BOTTOM_Z + 0.5 * RAIL_HEIGHT)),
        material=aluminum,
        name="rail_body",
    )
    headrail.visual(
        Box((RAIL_LENGTH, RAIL_DEPTH + 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z + 0.004)),
        material=aluminum,
        name="top_lip",
    )
    for side, sign in (("end_cap_0", -1.0), ("end_cap_1", 1.0)):
        headrail.visual(
            Box((0.012, RAIL_DEPTH + 0.007, RAIL_HEIGHT + 0.010)),
            origin=Origin(xyz=(sign * (0.5 * RAIL_LENGTH - 0.004), 0.0, RAIL_BOTTOM_Z + 0.5 * RAIL_HEIGHT)),
            material=aluminum,
            name=side,
        )
    # Small boss under the left rail end that carries the control chain.
    headrail.visual(
        Box((0.012, 0.028, 0.014)),
        origin=Origin(xyz=(CHAIN_X, -0.030, 2.073)),
        material=aluminum,
        name="chain_boss",
    )

    # ------------------------------------------------------------------
    # Carrier train: 20 carrier trucks riding the underside of the rail,
    # tied together by one spacer rail so they traverse as a set. The
    # train frame origin sits at the rail underside center.
    # ------------------------------------------------------------------
    carrier_train = model.part("carrier_train")
    carrier_train.visual(
        Box((1.45, 0.006, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, -0.0105)),
        material=carrier_plastic,
        name="spacer_rail",
    )
    for i in range(1, VANE_COUNT + 1):
        x = _vane_x(i)
        carrier_train.visual(
            Box((0.016, 0.028, 0.016)),
            origin=Origin(xyz=(x, 0.0, -0.008)),
            material=carrier_plastic,
            name=f"carrier_{i:02d}",
        )
        carrier_train.visual(
            Cylinder(radius=0.0035, length=0.008),
            origin=Origin(xyz=(x, 0.0, -0.020)),
            material=carrier_plastic,
            name=f"carrier_stem_{i:02d}",
        )

    model.articulation(
        "vane_set_traverse",
        ArticulationType.PRISMATIC,
        parent=headrail,
        child=carrier_train,
        origin=Origin(xyz=(0.0, 0.0, RAIL_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.3, lower=-TRAVERSE_LIMIT, upper=TRAVERSE_LIMIT),
    )

    # ------------------------------------------------------------------
    # Vanes: each one hangs from its carrier stem on a hanger clip and
    # tilts about its own vertical axis. vane_01 drives, the rest mimic.
    # ------------------------------------------------------------------
    driver = None
    for i in range(1, VANE_COUNT + 1):
        x = _vane_x(i)
        y_off = _vane_y_offset(i)
        vane = model.part(f"vane_{i:02d}")
        vane.visual(
            Box((0.020, 0.007, 0.018)),
            origin=Origin(xyz=(0.0, y_off, -0.009)),
            material=carrier_plastic,
            name="hanger_clip",
        )
        vane.visual(
            Box((VANE_WIDTH, VANE_THICKNESS, VANE_DROP)),
            origin=Origin(xyz=(0.0, y_off, -0.018 - 0.5 * VANE_DROP)),
            material=fabric,
            name="vane_strip",
        )
        joint = model.articulation(
            f"vane_{i:02d}_tilt",
            ArticulationType.REVOLUTE,
            parent=carrier_train,
            child=vane,
            origin=Origin(xyz=(x, 0.0, STEM_BOTTOM_Z - RAIL_BOTTOM_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-TILT_LIMIT, upper=TILT_LIMIT),
            mimic=None if i == 1 else Mimic(joint="vane_01_tilt", multiplier=1.0, offset=0.0),
        )
        if i == 1:
            driver = joint
    assert driver is not None

    # ------------------------------------------------------------------
    # Control chain: thin cord with a black tassel weight, hanging from
    # the chain boss at the left rail end. It can swing front/back.
    # ------------------------------------------------------------------
    chain = model.part("control_chain")
    chain.visual(
        Cylinder(radius=0.002, length=CHAIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * CHAIN_LENGTH)),
        material=chain_metal,
        name="chain_cord",
    )
    chain.visual(
        Cylinder(radius=0.0085, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -CHAIN_LENGTH - 0.0275)),
        material=tassel_black,
        name="tassel_body",
    )
    chain.visual(
        Sphere(radius=0.0085),
        origin=Origin(xyz=(0.0, 0.0, -CHAIN_LENGTH - 0.055)),
        material=tassel_black,
        name="tassel_tip",
    )
    model.articulation(
        "control_chain_swing",
        ArticulationType.REVOLUTE,
        parent=headrail,
        child=chain,
        origin=Origin(xyz=(CHAIN_X, CHAIN_Y, CHAIN_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.25, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headrail = object_model.get_part("headrail")
    carrier_train = object_model.get_part("carrier_train")
    chain = object_model.get_part("control_chain")
    vanes = [object_model.get_part(f"vane_{i:02d}") for i in range(1, VANE_COUNT + 1)]
    driver = object_model.get_articulation("vane_01_tilt")
    traverse = object_model.get_articulation("vane_set_traverse")
    swing = object_model.get_articulation("control_chain_swing")

    # --- vane count and joint contract -------------------------------
    vane_parts = [p for p in object_model.parts if p.name.startswith("vane_")]
    ctx.check(
        "blind has exactly 20 vanes",
        len(vane_parts) == VANE_COUNT,
        details=f"found {len(vane_parts)} vane parts",
    )

    limits = driver.motion_limits
    ctx.check(
        "driver vane tilts +/- 90 degrees",
        limits is not None
        and math.isclose(limits.lower, -TILT_LIMIT, abs_tol=1e-6)
        and math.isclose(limits.upper, TILT_LIMIT, abs_tol=1e-6),
        details=f"limits={None if limits is None else (limits.lower, limits.upper)}",
    )

    bad_mimics: list[str] = []
    for i in range(2, VANE_COUNT + 1):
        joint = object_model.get_articulation(f"vane_{i:02d}_tilt")
        mim = joint.mimic
        if (
            mim is None
            or mim.joint != "vane_01_tilt"
            or not math.isclose(mim.multiplier, 1.0, abs_tol=1e-9)
            or not math.isclose(mim.offset, 0.0, abs_tol=1e-9)
        ):
            bad_mimics.append(joint.name)
    ctx.check(
        "19 follower vanes mimic vane_01_tilt with multiplier 1.0",
        not bad_mimics,
        details=f"non-conforming joints: {bad_mimics}",
    )

    # --- closed-panel geometry ----------------------------------------
    for i in range(VANE_COUNT - 1):
        ctx.expect_overlap(
            vanes[i],
            vanes[i + 1],
            axes="x",
            elem_a="vane_strip",
            elem_b="vane_strip",
            min_overlap=0.010,
            name=f"closed vanes {i + 1:02d}/{i + 2:02d} overlap sideways",
        )
    ctx.expect_within(vanes[0], headrail, axes="x", margin=0.005, name="leftmost vane stays under the headrail")
    ctx.expect_within(vanes[-1], headrail, axes="x", margin=0.005, name="rightmost vane stays under the headrail")

    # --- mounting chain: vane -> carrier -> headrail, chain -> headrail
    ctx.expect_contact(vanes[0], carrier_train, contact_tol=1e-4, name="vane_01 hangs on its carrier stem")
    ctx.expect_contact(vanes[-1], carrier_train, contact_tol=1e-4, name="vane_20 hangs on its carrier stem")
    ctx.expect_contact(carrier_train, headrail, contact_tol=1e-4, name="carrier train rides the rail underside")
    ctx.expect_contact(chain, headrail, contact_tol=1e-4, name="control chain hangs from the rail boss")
    ctx.expect_gap(headrail, vanes[9], axis="z", min_gap=0.005, name="vane panel hangs below the headrail")

    chain_aabb = ctx.part_world_aabb(chain)
    vane_aabb = ctx.part_world_aabb(vanes[9])
    ctx.check(
        "chain tassel reaches below blind mid-height",
        chain_aabb is not None
        and vane_aabb is not None
        and chain_aabb[0][2] < 0.5 * (vane_aabb[0][2] + vane_aabb[1][2]),
        details=f"chain_min_z={None if chain_aabb is None else chain_aabb[0][2]}",
    )
    chain_pos = ctx.part_world_position(chain)
    ctx.check(
        "control chain hangs at the left rail end",
        chain_pos is not None and chain_pos[0] < -0.70,
        details=f"chain position={chain_pos}",
    )
    ctx.check(
        "vanes hang clear of the floor",
        vane_aabb is not None and vane_aabb[0][2] > 0.0,
        details=f"vane_10 min_z={None if vane_aabb is None else vane_aabb[0][2]}",
    )

    # --- tilt mechanism: driver + mimic followers rotate in unison ----
    with ctx.pose({driver: TILT_LIMIT}):
        for probe in (vanes[4], vanes[15]):
            aabb = ctx.part_world_aabb(probe)
            depth = None if aabb is None else aabb[1][1] - aabb[0][1]
            ctx.check(
                f"{probe.name} turns edge-on at +90 degrees",
                depth is not None and depth > 0.085,
                details=f"depth={depth}",
            )
        ctx.expect_gap(
            vanes[5],
            vanes[4],
            axis="x",
            min_gap=0.030,
            name="fully tilted vanes clear each other",
        )
    with ctx.pose({driver: -TILT_LIMIT}):
        aabb = ctx.part_world_aabb(vanes[9])
        depth = None if aabb is None else aabb[1][1] - aabb[0][1]
        ctx.check(
            "vane_10 turns edge-on at -90 degrees",
            depth is not None and depth > 0.085,
            details=f"depth={depth}",
        )

    # --- traverse: rigid vane set slides along the rail ----------------
    rest_train = ctx.part_world_position(carrier_train)
    with ctx.pose({traverse: TRAVERSE_LIMIT}):
        ctx.expect_within(vanes[-1], headrail, axes="x", margin=0.001, name="traversed-right vane set stays under the rail")
        moved_train = ctx.part_world_position(carrier_train)
    with ctx.pose({traverse: -TRAVERSE_LIMIT}):
        ctx.expect_within(vanes[0], headrail, axes="x", margin=0.001, name="traversed-left vane set stays under the rail")
    ctx.check(
        "traverse slides the carrier train along the rail",
        rest_train is not None
        and moved_train is not None
        and moved_train[0] > rest_train[0] + 0.5 * TRAVERSE_LIMIT,
        details=f"rest={rest_train}, moved={moved_train}",
    )

    # --- chain swing ----------------------------------------------------
    rest_chain = ctx.part_world_aabb(chain)
    with ctx.pose({swing: 0.25}):
        swung_chain = ctx.part_world_aabb(chain)
    ctx.check(
        "control chain swings about its boss",
        rest_chain is not None
        and swung_chain is not None
        and swung_chain[1][1] > rest_chain[1][1] + 0.15,
        details=f"rest_max_y={None if rest_chain is None else rest_chain[1][1]}, "
        f"swung_max_y={None if swung_chain is None else swung_chain[1][1]}",
    )

    return ctx.report()


object_model = build_object_model()
