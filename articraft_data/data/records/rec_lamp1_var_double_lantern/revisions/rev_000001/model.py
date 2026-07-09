from __future__ import annotations

# Double-head wall lantern (galvanized / weathered-zinc finish).
# Fork variant of the single-head parent: two smaller lantern heads hang from
# a shared forked bracket arm. Each lantern swings independently on its own
# pendulum pivot (REVOLUTE about X at its hook eye).
#
# Convention (real meters, Z-up):
#   wall plane is X-Z at y = 0; bracket extends into +Y away from wall.
#   X -> horizontal across the wall (lateral)
#   +Y -> outward from the wall
#   +Z -> up

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Dimensions (meters) -- base (unscaled) lantern dimensions
# ---------------------------------------------------------------------------

PLATE_H = 0.230
PLATE_W = 0.120
PLATE_T = 0.014

ARM_R = 0.012

HOOK_Y = 0.235
HOOK_Z = 0.060
HOOK_R = 0.012
HOOK_RING_R = 0.026

LINK_R = 0.034
LINK_TUBE = 0.006

FINIAL_R = 0.018
FINIAL_H = 0.030

ROOF_TOP_R = 0.040
ROOF_BOT_R = 0.140
ROOF_H = 0.110

GLASS_R = 0.082
GLASS_H = 0.150

STRAP_N = 6
STRAP_W = 0.012
STRAP_T = 0.006
BAND_N = 3
BAND_T = 0.010
BAND_H = 0.014

BOTTOM_RING_H = 0.018
DRIP_R = 0.014
DRIP_H = 0.022

TOL = 0.0012
ATOL = 0.2

# ---------------------------------------------------------------------------
# Fork variant parameters
# ---------------------------------------------------------------------------

N_HEADS = 2
FORK_X = 0.130            # lateral offset of each hook from centerline
LANTERN_SCALE = 0.72      # each head is 72% of parent lantern size

# Hook positions: each hook is at (±FORK_X, HOOK_Y, HOOK_Z).
HOOK_XS = [-(FORK_X), +(FORK_X)]

# Tessellation
_TOL = TOL
_ATOL = ATOL


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _lathe_z(prof_rz: list[tuple[float, float]]) -> cq.Workplane:
    """Solid of revolution about the world Z axis."""
    pts = [(float(r), float(z)) for (r, z) in prof_rz]
    solid = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .revolve(360.0, (0, 0, 0), (0, 1, 0))
    )
    return solid.rotate((0, 0, 0), (1, 0, 0), 90.0)


# ---------------------------------------------------------------------------
# Geometry builders -- BRACKET (fixed root)
# ---------------------------------------------------------------------------


def _wall_plate_shape() -> cq.Workplane:
    """Decorative leaf / fleur-de-lis wall mounting plate."""
    hw = PLATE_W / 2.0
    hh = PLATE_H / 2.0
    pts = [
        (0.0, hh),
        (0.32 * hw, 0.55 * hh),
        (0.95 * hw, 0.62 * hh),
        (0.62 * hw, 0.28 * hh),
        (0.85 * hw, 0.0),
        (0.62 * hw, -0.28 * hh),
        (0.95 * hw, -0.62 * hh),
        (0.32 * hw, -0.55 * hh),
        (0.0, -hh),
        (-0.32 * hw, -0.55 * hh),
        (-0.95 * hw, -0.62 * hh),
        (-0.62 * hw, -0.28 * hh),
        (-0.85 * hw, 0.0),
        (-0.62 * hw, 0.28 * hh),
        (-0.95 * hw, 0.62 * hh),
        (-0.32 * hw, 0.55 * hh),
    ]
    plate = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(PLATE_T)
    )
    plate = plate.mirror("XZ")
    try:
        plate = plate.edges("|Y").fillet(0.004)
    except Exception:
        pass
    for z in (0.55 * hh, -0.55 * hh):
        hole = (
            cq.Workplane("XZ")
            .center(0.0, z)
            .circle(0.006)
            .extrude(-(PLATE_T + 0.02))
            .translate((0.0, -0.01, 0.0))
        )
        plate = plate.cut(hole)
    return plate


def _fork_trunk_mesh():
    """Main trunk of the scroll arm rising from the plate to the fork split."""
    path = [
        (0.0, 0.020, 0.080),
        (0.0, 0.060, 0.130),
        (0.0, 0.090, 0.150),
    ]
    return tube_from_spline_points(
        path, radius=ARM_R, samples_per_segment=16,
        radial_segments=16, cap_ends=True, up_hint=(1.0, 0.0, 0.0),
    )


def _fork_branch_mesh(x_target: float):
    """One branch of the fork: sweeps from the trunk split out to a hook eye."""
    path = [
        (0.0, 0.090, 0.150),
        (x_target * 0.45, 0.130, 0.148),
        (x_target * 0.80, 0.180, 0.130),
        (x_target, HOOK_Y, HOOK_Z + 0.025),
    ]
    return tube_from_spline_points(
        path, radius=ARM_R * 0.9, samples_per_segment=16,
        radial_segments=14, cap_ends=True, up_hint=(1.0, 0.0, 0.0),
    )


def _hook_mesh(cx: float):
    """Downward hook curl at one fork tip, in the Y-Z plane at x = cx."""
    cy, cz = HOOK_Y, HOOK_Z
    pts = []
    a0 = math.radians(70.0)
    a1 = math.radians(70.0 + 300.0)
    n = 28
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        y = cy + HOOK_RING_R * math.cos(a)
        z = cz + HOOK_RING_R * math.sin(a)
        pts.append((cx, y, z))
    return tube_from_spline_points(
        pts, radius=HOOK_R, samples_per_segment=8,
        radial_segments=14, cap_ends=True, up_hint=(1.0, 0.0, 0.0),
    )


# ---------------------------------------------------------------------------
# Geometry builders -- LANTERN (parameterized by scale factor s)
# ---------------------------------------------------------------------------
# Each lantern is authored in its own local frame: origin at the pendulum
# pivot (hook eye). The lantern hangs straight down from there.


def _lantern_z_layout(s: float) -> dict[str, float]:
    """Vertical layout of lantern sub-parts in local frame, scaled by s."""
    link_top_z = -0.018 * s
    link_cz = link_top_z - LINK_R * s
    link_bot_z = link_cz - LINK_R * s
    finial_top_z = link_bot_z + 0.014 * s
    finial_base_z = finial_top_z - FINIAL_H * s
    roof_top_z = finial_base_z + 0.006 * s
    roof_bot_z = roof_top_z - ROOF_H * s
    glass_top_z = roof_top_z - 0.020 * s
    glass_bot_z = glass_top_z - GLASS_H * s
    bottom_ring_top_z = glass_bot_z + 0.004 * s
    bottom_ring_bot_z = bottom_ring_top_z - BOTTOM_RING_H * s
    return {
        "link_cz": link_cz,
        "finial_base_z": finial_base_z,
        "roof_bot_z": roof_bot_z,
        "roof_top_z": roof_top_z,
        "glass_bot_z": glass_bot_z,
        "glass_top_z": glass_top_z,
        "bottom_ring_bot_z": bottom_ring_bot_z,
        "bottom_ring_top_z": bottom_ring_top_z,
    }


def _chain_link_shape(s: float, z_layout: dict) -> cq.Workplane:
    r = LINK_R * s
    tube = LINK_TUBE * s
    link = cq.Workplane("XY").add(cq.Solid.makeTorus(r, tube))
    link = link.rotate((0, 0, 0), (1, 0, 0), 90.0)
    return link.translate((0.0, 0.0, z_layout["link_cz"]))


def _finial_shape(s: float, z_layout: dict) -> cq.Workplane:
    base_r = ROOF_TOP_R * s * 0.6 + 0.008 * s
    finial_r = FINIAL_R * s
    finial_h = FINIAL_H * s
    prof = [
        (0.004 * s, 0.0),
        (base_r, 0.002 * s),
        (base_r, 0.010 * s),
        (finial_r, finial_h * 0.45),
        (finial_r * 0.45, finial_h * 0.80),
        (0.004 * s, finial_h),
        (0.0, finial_h),
        (0.0, 0.0),
    ]
    finial = _lathe_z(prof)
    return finial.translate((0.0, 0.0, z_layout["finial_base_z"]))


def _roof_shape(s: float, z_layout: dict) -> cq.Workplane:
    roof_bot_r = ROOF_BOT_R * s
    roof_top_r = ROOF_TOP_R * s
    roof_h = ROOF_H * s
    out = [
        (roof_bot_r, 0.0),
        (roof_top_r, roof_h),
        (roof_top_r * 0.6, roof_h + 0.012 * s),
    ]
    inn = [
        (roof_top_r * 0.6 - 0.004 * s, roof_h + 0.012 * s),
        (roof_top_r - 0.004 * s, roof_h),
        (roof_bot_r - 0.010 * s, 0.0),
    ]
    roof = _lathe_z(out + inn)
    return roof.translate((0.0, 0.0, z_layout["roof_bot_z"]))


def _glass_shape(s: float, z_layout: dict) -> cq.Workplane:
    glass_r = GLASS_R * s
    glass_h = GLASS_H * s
    glass = cq.Workplane("XY").circle(glass_r).extrude(glass_h)
    return glass.translate((0.0, 0.0, z_layout["glass_bot_z"]))


def _cage_shape(s: float, z_layout: dict) -> cq.Workplane:
    glass_r = GLASS_R * s
    glass_h = GLASS_H * s
    glass_bot_z = z_layout["glass_bot_z"]
    strap_w = STRAP_W * s
    strap_t = STRAP_T * s
    band_t = BAND_T * s
    band_h = BAND_H * s
    cage = None
    strap_h = glass_h + 0.010 * s
    rr = glass_r + strap_t / 2.0 - 0.001 * s
    for i in range(STRAP_N):
        a = 2.0 * math.pi * i / STRAP_N
        strap = (
            cq.Workplane("XY")
            .box(strap_t, strap_w, strap_h, centered=(True, True, True))
            .translate((rr, 0.0, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
            .translate((0.0, 0.0, glass_bot_z + glass_h / 2.0))
        )
        cage = strap if cage is None else cage.union(strap)
    band_zs = [
        glass_bot_z + glass_h - 0.012 * s,
        glass_bot_z + glass_h / 2.0,
        glass_bot_z + 0.012 * s,
    ]
    for z in band_zs[:BAND_N]:
        ring = (
            cq.Workplane("XY")
            .circle(glass_r + band_t)
            .circle(glass_r - 0.001 * s)
            .extrude(band_h)
            .translate((0.0, 0.0, z - band_h / 2.0))
        )
        cage = ring if cage is None else cage.union(ring)
    return cage


def _bottom_ring_shape(s: float, z_layout: dict) -> cq.Workplane:
    glass_r = GLASS_R * s
    bottom_ring_r = glass_r + 0.006 * s
    bottom_ring_h = BOTTOM_RING_H * s
    drip_r = DRIP_R * s
    drip_h = DRIP_H * s
    bot_z = z_layout["bottom_ring_bot_z"]
    ring = (
        cq.Workplane("XY")
        .circle(bottom_ring_r)
        .circle(glass_r - 0.004 * s)
        .extrude(bottom_ring_h)
        .translate((0.0, 0.0, bot_z))
    )
    floor = (
        cq.Workplane("XY")
        .circle(bottom_ring_r)
        .extrude(0.008 * s)
        .translate((0.0, 0.0, bot_z - 0.008 * s))
    )
    body = ring.union(floor)
    drip_prof = [
        (drip_r, 0.0),
        (drip_r * 0.5, -drip_h * 0.6),
        (0.002 * s, -drip_h),
        (0.0, -drip_h),
        (0.0, 0.0),
    ]
    drip = _lathe_z(drip_prof)
    drip = drip.translate((0.0, 0.0, bot_z - 0.008 * s))
    return body.union(drip)


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_head_wall_lantern")

    model.material("zinc", rgba=(0.66, 0.68, 0.69, 1.0))
    model.material("zinc_dark", rgba=(0.55, 0.57, 0.59, 1.0))
    model.material("iron_rust", rgba=(0.55, 0.40, 0.34, 1.0))
    model.material("glass", rgba=(0.62, 0.74, 0.74, 0.45))

    # --- BRACKET (fixed root): plate + forked arm + two hooks ---
    bracket = model.part("wall_bracket")
    bracket.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate",
                           tolerance=_TOL, angular_tolerance=_ATOL),
        material="zinc",
        name="wall_plate",
    )
    bracket.visual(
        mesh_from_geometry(_fork_trunk_mesh(), "fork_trunk"),
        material="zinc",
        name="fork_trunk",
    )
    # Fork branches and hooks -- emitted per head index
    for i in range(N_HEADS):
        hx = HOOK_XS[i]
        bracket.visual(
            mesh_from_geometry(_fork_branch_mesh(hx), f"fork_branch_{i}"),
            material="zinc",
            name=f"fork_branch_{i}",
        )
        bracket.visual(
            mesh_from_geometry(_hook_mesh(hx), f"bracket_hook_{i}"),
            material="zinc",
            name=f"bracket_hook_{i}",
        )

    # --- LANTERN HEADS (swinging children) ---
    s = LANTERN_SCALE
    z_layout = _lantern_z_layout(s)

    for i in range(N_HEADS):
        hx = HOOK_XS[i]
        lantern = model.part(f"lantern_{i}")

        lantern.visual(
            mesh_from_cadquery(_chain_link_shape(s, z_layout), f"chain_link_{i}",
                               tolerance=_TOL, angular_tolerance=_ATOL),
            material="zinc_dark",
            name="chain_link",
        )
        lantern.visual(
            mesh_from_cadquery(_finial_shape(s, z_layout), f"finial_{i}",
                               tolerance=_TOL, angular_tolerance=_ATOL),
            material="zinc",
            name="finial",
        )
        lantern.visual(
            mesh_from_cadquery(_roof_shape(s, z_layout), f"roof_{i}",
                               tolerance=_TOL, angular_tolerance=_ATOL),
            material="zinc",
            name="roof",
        )
        lantern.visual(
            mesh_from_cadquery(_glass_shape(s, z_layout), f"glass_{i}",
                               tolerance=_TOL, angular_tolerance=_ATOL),
            material="glass",
            name="glass",
        )
        lantern.visual(
            mesh_from_cadquery(_cage_shape(s, z_layout), f"cage_{i}",
                               tolerance=_TOL, angular_tolerance=_ATOL),
            material="iron_rust",
            name="cage",
        )
        lantern.visual(
            mesh_from_cadquery(_bottom_ring_shape(s, z_layout), f"bottom_ring_{i}",
                               tolerance=_TOL, angular_tolerance=_ATOL),
            material="iron_rust",
            name="bottom_ring",
        )

        # Pendulum swing articulation for this lantern head.
        # Origin at the hook eye for this head; axis = X so positive q
        # swings the lantern outward/inward in the Y-Z plane.
        model.articulation(
            f"lantern_swing_{i}",
            ArticulationType.REVOLUTE,
            parent=bracket,
            child=lantern,
            origin=Origin(xyz=(hx, HOOK_Y, HOOK_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0, velocity=2.0, lower=-0.45, upper=0.45
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bracket = object_model.get_part("wall_bracket")

    # --- Per-lantern checks ---
    for i in range(N_HEADS):
        hx = HOOK_XS[i]
        lantern = object_model.get_part(f"lantern_{i}")
        swing = object_model.get_articulation(f"lantern_swing_{i}")

        # Intentional overlaps: chain link interlinks hook curl, finial captured
        ctx.allow_overlap(
            lantern, bracket,
            elem_a="chain_link", elem_b=f"bracket_hook_{i}",
            reason=f"Chain link {i} interlinked through hook curl {i} "
                   f"(real chain-link capture).",
        )
        ctx.allow_overlap(
            lantern, lantern,
            elem_a="chain_link", elem_b="finial",
            reason=f"Finial {i} neck threads through chain link {i} "
                   f"(suspension capture).",
        )

        # Prove chain capture
        ctx.expect_contact(
            lantern, bracket, elem_a="chain_link", elem_b=f"bracket_hook_{i}",
            contact_tol=0.001, name=f"chain link {i} interlinks hook {i}",
        )
        ctx.expect_contact(
            lantern, lantern, elem_a="chain_link", elem_b="finial",
            contact_tol=0.001, name=f"chain link {i} captures finial {i}",
        )

        # Joint identity
        ctx.check(
            f"swing_{i} is revolute",
            str(swing.articulation_type).lower().endswith("revolute"),
            details=f"type={swing.articulation_type}",
        )
        ax = tuple(round(c, 6) for c in swing.axis)
        ctx.check(
            f"swing_{i} axis is X",
            abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
            details=f"axis={ax}",
        )
        jo = swing.origin.xyz
        ctx.check(
            f"swing_{i} pivot at hook eye {i}",
            abs(jo[0] - hx) < 1e-4 and abs(jo[1] - HOOK_Y) < 1e-4
            and abs(jo[2] - HOOK_Z) < 1e-4,
            details=f"origin={jo}, expected=({hx}, {HOOK_Y}, {HOOK_Z})",
        )

        # Geometry relationships
        roof_aabb = ctx.part_element_world_aabb(lantern, elem="roof")
        glass_aabb = ctx.part_element_world_aabb(lantern, elem="glass")
        ring_aabb = ctx.part_element_world_aabb(lantern, elem="bottom_ring")
        ctx.check(
            f"roof_{i} wider than glass_{i} (flared shade)",
            roof_aabb is not None and glass_aabb is not None
            and (roof_aabb[1][0] - roof_aabb[0][0])
            > (glass_aabb[1][0] - glass_aabb[0][0]) + 0.03,
            details=f"roof_w={None if roof_aabb is None else roof_aabb[1][0]-roof_aabb[0][0]}, "
                    f"glass_w={None if glass_aabb is None else glass_aabb[1][0]-glass_aabb[0][0]}",
        )
        ctx.check(
            f"roof_{i} above glass_{i}",
            roof_aabb is not None and glass_aabb is not None
            and roof_aabb[1][2] > glass_aabb[1][2],
        )
        ctx.check(
            f"bottom_ring_{i} below glass_{i}",
            ring_aabb is not None and glass_aabb is not None
            and ring_aabb[0][2] < glass_aabb[0][2] + 0.002,
        )

        # Cage wraps glass
        ctx.expect_overlap(
            lantern, lantern, axes="xy",
            elem_a="cage", elem_b="glass",
            min_overlap=GLASS_R * LANTERN_SCALE * 0.8,
            name=f"cage_{i} wraps glass_{i}",
        )

        # Glass under roof eave
        ctx.expect_within(
            lantern, lantern, axes="xy",
            inner_elem="glass", outer_elem="roof",
            margin=0.001,
            name=f"glass_{i} under roof_{i} eave",
        )

        # Swing mechanism: positive q moves lantern outward in Y
        rest_ring = ctx.part_element_world_aabb(lantern, elem="bottom_ring")
        with ctx.pose({swing: 0.45}):
            swung_ring = ctx.part_element_world_aabb(lantern, elem="bottom_ring")
        rest_yc = None if rest_ring is None else (rest_ring[0][1] + rest_ring[1][1]) / 2.0
        swung_yc = None if swung_ring is None else (swung_ring[0][1] + swung_ring[1][1]) / 2.0
        rest_zc = None if rest_ring is None else (rest_ring[0][2] + rest_ring[1][2]) / 2.0
        swung_zc = None if swung_ring is None else (swung_ring[0][2] + swung_ring[1][2]) / 2.0
        ctx.check(
            f"swing_{i} moves lantern_{i} outward in Y",
            rest_yc is not None and swung_yc is not None
            and abs(swung_yc - rest_yc) > 0.03,
            details=f"rest_yc={rest_yc}, swung_yc={swung_yc}",
        )
        ctx.check(
            f"swing_{i} raises lantern_{i} (pendulum arc)",
            rest_zc is not None and swung_zc is not None
            and swung_zc > rest_zc + 0.005,
            details=f"rest_zc={rest_zc}, swung_zc={swung_zc}",
        )

    # --- Two distinct lantern heads exist at different X positions ---
    l0 = object_model.get_part("lantern_0")
    l1 = object_model.get_part("lantern_1")
    l0_pos = ctx.part_world_position(l0)
    l1_pos = ctx.part_world_position(l1)
    ctx.check(
        "two lantern heads at different X positions",
        l0_pos is not None and l1_pos is not None
        and abs(l1_pos[0] - l0_pos[0]) > 0.15,
        details=f"l0_x={None if l0_pos is None else l0_pos[0]}, "
                f"l1_x={None if l1_pos is None else l1_pos[0]}",
    )

    # Wall plate at wall plane
    plate_aabb = ctx.part_element_world_aabb(bracket, elem="wall_plate")
    ctx.check(
        "wall plate mounts at wall plane",
        plate_aabb is not None and plate_aabb[0][1] <= 0.002
        and plate_aabb[1][1] <= PLATE_T + 0.01,
        details=f"plate_aabb={plate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
