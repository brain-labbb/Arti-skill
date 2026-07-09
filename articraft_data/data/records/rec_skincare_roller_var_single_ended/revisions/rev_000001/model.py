from __future__ import annotations

# Single-ended jade facial massage roller.
# Frame: handle axis along +Z (vertical), object centered on x=0, y=0.
#   - Static body = marbled stone handle + two polished metal collars +
#     one U-shaped metal fork at the bottom end + a rounded stone tip at top.
#   - Bottom fork (-Z) holds a LARGE polished oval stone roller.
#   - Top end (+Z) terminates in a plain rounded stone cap with its collar.
# Articulation (CONTINUOUS spin about the roller axle, axle along X):
#   - roller_0 : continuous rotation in the bottom fork
# The roller is an oblate ellipsoid (oval) and carries a tiny off-axis marker
# so its spin is unambiguously detectable by AABB spin tests.

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
HANDLE_HALF = 0.052        # handle half-length along Z  (handle spans ~0.104)
COLLAR_R = 0.0115          # metal collar/ferrule radius
COLLAR_H = 0.012           # metal collar height

# Fork geometry
FORK_ROD_R = 0.0018        # thin metal fork rod radius
BOT_AXLE_Z = -0.094        # z of the large roller axle (bottom)

LARGE_HALF_X = 0.025       # large roller half-length along axle (X)  -> ~0.050 long
LARGE_RY = 0.018           # large roller cross radius
LARGE_FORK_SPAN = LARGE_HALF_X + 0.004   # half-spacing of the bottom fork tips

# Roller count: single-ended tool has exactly one roller-fork unit
ROLLER_COUNT = 1

# Roller configurations (loop-driven)
ROLLER_CONFIGS = [
    {
        "name": "roller_0",
        "half_x": LARGE_HALF_X,
        "ry": LARGE_RY,
        "fork_span": LARGE_FORK_SPAN,
        "z_root": -(HANDLE_HALF + COLLAR_H * 0.4),
        "z_axle": BOT_AXLE_Z,
        "mass": 0.045,
        "effort": 0.3,
    },
]


def _u_fork(z_root: float, z_axle: float, span: float) -> "MeshGeometry":  # noqa: F821
    # A U-shaped metal fork: rises out of the collar end of the handle on the
    # axis, then splits into two arms that reach out to +/-X axle tips.
    # Built as one continuous bent tube down one arm, across, and up the other.
    direction = 1.0 if z_axle > z_root else -1.0
    z_tip = z_axle  # arms terminate at the axle height, holding the roller axle
    # Path: tip(+X) -> up toward root -> across the root yoke -> down -> tip(-X)
    pts = [
        (span, 0.0, z_tip),
        (span, 0.0, z_tip - direction * (z_tip - z_root) * 0.45),
        (span * 0.55, 0.0, z_root + direction * 0.010),
        (0.0, 0.0, z_root),
        (-span * 0.55, 0.0, z_root + direction * 0.010),
        (-span, 0.0, z_tip - direction * (z_tip - z_root) * 0.45),
        (-span, 0.0, z_tip),
    ]
    return tube_from_spline_points(
        pts,
        radius=FORK_ROD_R,
        samples_per_segment=16,
        radial_segments=14,
        cap_ends=True,
    )


def _oval_roller(half_x: float, ry: float, name: str):
    # Oblate ellipsoid (oval gemstone): unit sphere scaled long along the axle
    # (X) and round in cross section (Y/Z). Add a small off-axis marker bump so
    # the spin is detectable and to break any residual symmetry.
    body = SphereGeometry(1.0, width_segments=36, height_segments=24)
    body.scale(half_x, ry, ry)
    # Small marker nub on the +Y rim, off the spin axis.
    marker = SphereGeometry(0.0028, width_segments=12, height_segments=8)
    marker.translate(0.0, ry * 0.96, 0.0)
    body.merge(marker)
    return mesh_from_geometry(body, name)


def _stone_tip() -> "MeshGeometry":  # noqa: F821
    # Rounded stone cap at the top end of the handle: a smooth dome sitting
    # above the top collar. Hemisphere slightly squashed for a natural pebble
    # finish that matches the handle stone.
    cap_r = HANDLE_R * 1.05
    dome = SphereGeometry(cap_r, width_segments=28, height_segments=18)
    # Flatten the bottom half so it sits flush on the collar top face.
    dome.scale(1.0, 1.0, 0.72)
    # Position: center above the top collar, sitting on its top face.
    z_collar_top = HANDLE_HALF - COLLAR_H * 0.25 + COLLAR_H * 0.5
    dome.translate(0.0, 0.0, z_collar_top + cap_r * 0.72 * 0.35)
    return mesh_from_geometry(dome, "stone_tip")


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

    # Polished metal collar at the top end (plain termination).
    top_collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=40)
    top_collar.translate(0.0, 0.0, HANDLE_HALF - COLLAR_H * 0.25)
    body.visual(mesh_from_geometry(top_collar, "top_collar"), material=metal, name="top_collar")

    # Rounded stone tip capping the top end above the collar.
    body.visual(_stone_tip(), material=jade, name="stone_tip")

    # Polished metal collar at the bottom end (fork end).
    bot_collar = CylinderGeometry(COLLAR_R, COLLAR_H, radial_segments=40)
    bot_collar.translate(0.0, 0.0, -(HANDLE_HALF - COLLAR_H * 0.25))
    body.visual(mesh_from_geometry(bot_collar, "bottom_collar"), material=metal, name="bottom_collar")

    body.inertial = Inertial.from_geometry(
        Box((0.06, 0.025, 0.20)), mass=0.18, origin=Origin(xyz=(0.0, 0.0, 0.0))
    )

    # ================= roller-fork units (loop-driven) =================
    for i in range(ROLLER_COUNT):
        cfg = ROLLER_CONFIGS[i]

        # U-fork on the body, rooted at the bottom collar.
        fork = _u_fork(
            z_root=cfg["z_root"],
            z_axle=cfg["z_axle"],
            span=cfg["fork_span"],
        )
        body.visual(
            mesh_from_geometry(fork, f"fork_{i}"),
            material=metal,
            name=f"fork_{i}",
        )

        # Roller part: oval polished stone + thin metal axle.
        roller = model.part(cfg["name"])
        roller.visual(
            _oval_roller(cfg["half_x"], cfg["ry"], f"roller_stone_{i}"),
            material=jade,
            name=f"roller_stone_{i}",
        )
        axle = CylinderGeometry(
            FORK_ROD_R * 0.85,
            2.0 * cfg["fork_span"] + 0.004,
            radial_segments=16,
        ).rotate_y(math.pi / 2.0)
        roller.visual(
            mesh_from_geometry(axle, f"axle_{i}"),
            material=metal,
            name=f"axle_{i}",
        )
        roller.inertial = Inertial.from_geometry(
            Box((2.0 * cfg["half_x"], 2.0 * cfg["ry"], 2.0 * cfg["ry"])),
            mass=cfg["mass"],
        )

        # Continuous spin about the roller axle (X axis), anchored at the
        # fork-tip contact face where the axle seats.
        model.articulation(
            f"roller_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=roller,
            origin=Origin(xyz=(0.0, 0.0, cfg["z_axle"])),
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
    roller_0 = object_model.get_part("roller_0")
    spin_0 = object_model.get_articulation("roller_spin_0")

    # --- single-ended: only one roller exists ---
    all_parts = [p for p in object_model.parts if p.name != "body"]
    ctx.check(
        "exactly one roller part (single-ended tool)",
        len(all_parts) == 1 and all_parts[0].name == "roller_0",
        details=f"non-body parts: {[p.name for p in all_parts]}",
    )

    # --- roller axle seated in fork tips ---
    ctx.allow_overlap(
        roller_0, body,
        elem_a="axle_0", elem_b="fork_0",
        reason="The roller axle ends are seated through the fork tips.",
    )
    ctx.expect_contact(roller_0, body, name="roller captured by fork")

    # --- roller is at the bottom of the handle ---
    rp = ctx.part_world_position(roller_0)
    bp = ctx.part_world_position(body)
    ctx.check(
        "roller is below the handle body center",
        rp is not None and bp is not None and rp[2] < bp[2],
        details=f"roller_z={rp}, body_z={bp}",
    )

    # --- stone tip is at the top (above handle center) ---
    tip_aabb = ctx.part_element_world_aabb(body, elem="stone_tip")
    ctx.check(
        "stone tip sits above handle center",
        tip_aabb is not None and tip_aabb[0][2] > bp[2],
        details=f"tip_min_z={tip_aabb[0][2]}, body_z={bp[2]}",
    )

    # --- top end has no fork (only collar + stone tip) ---
    body_visuals = [v.name for v in body.visuals]
    ctx.check(
        "no fork at the top end (single-ended design)",
        "top_fork" not in body_visuals and "fork_0" in body_visuals,
        details=f"body visuals: {body_visuals}",
    )

    # --- roller spin changes YZ silhouette (proves continuous rotation works) ---
    r0 = _ext(ctx.part_world_aabb(roller_0))
    with ctx.pose({spin_0: math.pi / 2.0}):
        r90 = _ext(ctx.part_world_aabb(roller_0))
    ctx.check(
        "roller spin changes its YZ silhouette",
        abs(r90[1] - r0[1]) > 0.001 or abs(r90[2] - r0[2]) > 0.001,
        details=f"rest={r0}, quarter={r90}",
    )

    # --- roller is the large oval form (not the removed small one) ---
    stone_ext = _ext(ctx.part_element_world_aabb(roller_0, elem="roller_stone_0"))
    stone_len = stone_ext[0]  # along axle (X)
    ctx.check(
        "roller is the large oval form (>=0.045m along axle)",
        stone_len >= 0.045,
        details=f"roller_axle_length={stone_len}",
    )

    return ctx.report()


object_model = build_object_model()
