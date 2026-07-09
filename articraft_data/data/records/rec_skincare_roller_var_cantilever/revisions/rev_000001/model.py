from __future__ import annotations

# Jade facial massage roller with single cantilever arm mounts.
# Frame: handle axis along +Z (vertical), object centered on x=0, y=0.
#   - Static body = marbled stone handle + two polished metal collars +
#     two single cantilever arms (one at each end, both curving to -X).
#   - Top arm (+Z) holds a SMALL polished oval stone roller on a stub axle.
#   - Bottom arm (-Z) holds a LARGER polished oval stone roller on a stub axle.
# Each arm is a single bent tube rising from the collar and reaching to one
# side; the roller spins on a cantilever stub axle that seats into the arm tip.
# Articulations (both CONTINUOUS spin about the roller axle, axle along X):
#   - roller_spin_0 : continuous rotation on the top cantilever arm
#   - roller_spin_1 : continuous rotation on the bottom cantilever arm

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- key geometry constants (meters) ----
HANDLE_R = 0.0095          # handle radius (slim stone shaft)
HANDLE_HALF = 0.052        # handle half-length along Z (handle spans ~0.104)
COLLAR_R = 0.0115          # metal collar/ferrule radius
COLLAR_H = 0.012           # metal collar height

ARM_ROD_R = 0.002          # cantilever arm tube radius (sturdy single arm)
TOP_AXLE_Z = 0.094         # z of the small roller axle (top)
BOT_AXLE_Z = -0.094        # z of the large roller axle (bottom)

SMALL_HALF_X = 0.0175      # small roller half-length along axle (X) -> ~0.035 long
SMALL_RY = 0.0125          # small roller cross radius
LARGE_HALF_X = 0.025       # large roller half-length along axle (X) -> ~0.050 long
LARGE_RY = 0.018           # large roller cross radius


def _cantilever_arm(z_root: float, z_axle: float, reach: float):
    """Shared bent-tube helper: single cantilever arm from collar to one-sided tip.

    The arm starts embedded inside the collar (for mesh connectivity), rises
    through the collar surface, then curves outward to the -X side and
    terminates at (-reach, 0, z_axle) — the arm tip face where the axle stub
    seats.
    """
    dz = z_axle - z_root  # signed vertical travel
    direction = 1.0 if dz > 0.0 else -1.0
    # Embed root deep inside the collar so the tube pierces the collar face.
    z_embed = z_root - direction * COLLAR_H * 0.5
    pts = [
        (0.0, 0.0, z_embed),                       # embedded inside collar
        (0.0, 0.0, z_root),                         # at collar surface exit
        (0.0, 0.0, z_root + dz * 0.40),             # rising/falling along Z
        (-reach * 0.30, 0.0, z_root + dz * 0.70),   # beginning to bend outward
        (-reach * 0.68, 0.0, z_root + dz * 0.90),   # nearing tip height
        (-reach, 0.0, z_axle),                       # arm tip face
    ]
    return tube_from_spline_points(
        pts,
        radius=ARM_ROD_R,
        samples_per_segment=18,
        radial_segments=14,
        cap_ends=True,
    )


def _oval_roller_geom(half_x: float, ry: float):
    """Oblate ellipsoid stone geometry centered at origin with off-axis marker."""
    body = SphereGeometry(1.0, width_segments=36, height_segments=24)
    body.scale(half_x, ry, ry)
    # Small marker nub on +Y rim for spin detectability.
    marker = SphereGeometry(0.0028, width_segments=12, height_segments=8)
    marker.translate(0.0, ry * 0.96, 0.0)
    body.merge(marker)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jade_facial_roller")

    jade = model.material("marbled_jade", rgba=(0.94, 0.91, 0.80, 1.0))
    metal = model.material("satin_metal", rgba=(0.78, 0.77, 0.74, 1.0))

    # ================= static body =================
    body = model.part("body")

    # Marbled stone handle: slim cylinder with a slight barrel mid-bulge.
    handle = CylinderGeometry(HANDLE_R, 2.0 * HANDLE_HALF, radial_segments=40)
    bulge = SphereGeometry(1.0, width_segments=36, height_segments=20)
    bulge.scale(HANDLE_R * 1.07, HANDLE_R * 1.07, HANDLE_HALF * 0.62)
    handle.merge(bulge)
    body.visual(mesh_from_geometry(handle, "handle"), material=jade, name="handle")

    # Two polished metal collars/ferrules at each end of the handle.
    top_collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=40)
    top_collar.translate(0.0, 0.0, HANDLE_HALF - COLLAR_H * 0.25)
    body.visual(mesh_from_geometry(top_collar, "top_collar"), material=metal, name="top_collar")

    bot_collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=40)
    bot_collar.translate(0.0, 0.0, -(HANDLE_HALF - COLLAR_H * 0.25))
    body.visual(mesh_from_geometry(bot_collar, "bottom_collar"), material=metal, name="bottom_collar")

    body.inertial = Inertial.from_geometry(
        Box((0.06, 0.025, 0.20)), mass=0.18, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ================= cantilever arms + rollers (for-loop) =================
    # Each end gets: one cantilever arm (visual on body) + one roller part
    # with stone + stub axle. They differ only in arm reach and axle stub size.
    arm_roller_configs = [
        {
            "z_root": HANDLE_HALF + COLLAR_H * 0.4,   # top arm root
            "z_axle": TOP_AXLE_Z,
            "half_x": SMALL_HALF_X,
            "ry": SMALL_RY,
            "mass": 0.02,
            "effort": 0.2,
        },
        {
            "z_root": -(HANDLE_HALF + COLLAR_H * 0.4), # bottom arm root
            "z_axle": BOT_AXLE_Z,
            "half_x": LARGE_HALF_X,
            "ry": LARGE_RY,
            "mass": 0.045,
            "effort": 0.3,
        },
    ]

    for i in range(2):
        cfg = arm_roller_configs[i]
        reach = cfg["half_x"] + 0.003           # arm tip distance from center axis
        stub_len = reach + cfg["half_x"] * 0.65  # axle stub extends past roller center

        # -- cantilever arm (static visual on body) --
        arm_geom = _cantilever_arm(cfg["z_root"], cfg["z_axle"], reach)
        body.visual(
            mesh_from_geometry(arm_geom, f"arm_{i}"),
            material=metal,
            name=f"arm_{i}",
        )

        # -- roller part --
        roller = model.part(f"roller_{i}")

        # Oval stone: centered at world x=0, offset from arm tip by +reach in local X.
        stone_geom = _oval_roller_geom(cfg["half_x"], cfg["ry"])
        stone_geom.translate(reach, 0.0, 0.0)
        roller.visual(
            mesh_from_geometry(stone_geom, f"stone_{i}"),
            material=jade,
            name=f"stone_{i}",
        )

        # Cantilever stub axle: short pin from arm tip face extending along +X.
        stub_geom = CylinderGeometry(ARM_ROD_R * 0.8, stub_len, radial_segments=16)
        stub_geom.rotate_y(math.pi / 2.0)          # align along X
        stub_geom.translate(stub_len / 2.0, 0.0, 0.0)  # from origin outward
        roller.visual(
            mesh_from_geometry(stub_geom, f"axle_{i}"),
            material=metal,
            name=f"axle_{i}",
        )

        roller.inertial = Inertial.from_geometry(
            Box((2.0 * cfg["half_x"], 2.0 * cfg["ry"], 2.0 * cfg["ry"])),
            mass=cfg["mass"],
        )

        # Continuous spin: origin at arm tip face, axis along X.
        model.articulation(
            f"roller_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=roller,
            origin=Origin(xyz=(-reach, 0.0, cfg["z_axle"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=cfg["effort"], velocity=8.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    for i in range(2):
        roller = object_model.get_part(f"roller_{i}")
        spin = object_model.get_articulation(f"roller_spin_{i}")

        # Axle stub seats into arm tip (intentional local overlap at insertion).
        ctx.allow_overlap(
            roller, body,
            elem_a=f"axle_{i}", elem_b=f"arm_{i}",
            reason=f"Roller {i} cantilever stub axle is seated into arm_{i} tip face.",
        )

        # Axle stub contacts arm tip at the seating face.
        ctx.expect_contact(
            roller, body,
            elem_a=f"axle_{i}", elem_b=f"arm_{i}",
            name=f"roller_{i} axle contacts arm_{i} at tip face",
        )

        # Spin changes the YZ silhouette (marker + oval profile).
        a0 = _ext(ctx.part_world_aabb(roller))
        with ctx.pose({spin: math.pi / 2.0}):
            a90 = _ext(ctx.part_world_aabb(roller))
        ctx.check(
            f"roller_{i} spin changes YZ silhouette",
            abs(a90[1] - a0[1]) > 0.001 or abs(a90[2] - a0[2]) > 0.001,
            details=f"rest={a0}, quarter={a90}",
        )

        # Cantilever support: arm reaches to -X side of roller stone center.
        arm_aabb = ctx.part_element_world_aabb(body, elem=f"arm_{i}")
        stone_aabb = ctx.part_element_world_aabb(roller, elem=f"stone_{i}")
        arm_min_x = arm_aabb[0][0]
        stone_center_x = (stone_aabb[0][0] + stone_aabb[1][0]) / 2.0
        ctx.check(
            f"arm_{i} supports roller_{i} from -X side only (cantilever)",
            arm_min_x < stone_center_x - 0.005,
            details=f"arm_min_x={arm_min_x:.4f}, stone_center_x={stone_center_x:.4f}",
        )

        # Single arm (not a U-fork): arm max X should be near the Z axis,
        # NOT reaching to the +X side of the roller.
        arm_max_x = arm_aabb[1][0]
        stone_max_x = stone_aabb[1][0]
        ctx.check(
            f"arm_{i} does not reach to +X side of roller (single cantilever)",
            arm_max_x < stone_max_x - 0.005,
            details=f"arm_max_x={arm_max_x:.4f}, stone_max_x={stone_max_x:.4f}",
        )

    # Top roller above handle, bottom roller below handle.
    r0_pos = ctx.part_world_position(object_model.get_part("roller_0"))
    r1_pos = ctx.part_world_position(object_model.get_part("roller_1"))
    bp = ctx.part_world_position(body)
    ctx.check(
        "roller_0 above body, roller_1 below body",
        r0_pos is not None and r1_pos is not None and r0_pos[2] > bp[2] > r1_pos[2],
        details=f"r0_z={r0_pos}, body_z={bp}, r1_z={r1_pos}",
    )

    # Bottom roller is larger than top roller.
    s0_ext = _ext(ctx.part_element_world_aabb(object_model.get_part("roller_0"), elem="stone_0"))
    s1_ext = _ext(ctx.part_element_world_aabb(object_model.get_part("roller_1"), elem="stone_1"))
    ctx.check(
        "bottom roller longer than top along axle",
        s1_ext[0] > s0_ext[0] + 0.005,
        details=f"top_len={s0_ext[0]:.4f}, bot_len={s1_ext[0]:.4f}",
    )
    ctx.check(
        "bottom roller wider in cross-section than top",
        s1_ext[2] > s0_ext[2] + 0.003,
        details=f"top_cross={s0_ext[2]:.4f}, bot_cross={s1_ext[2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
