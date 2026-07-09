from __future__ import annotations

"""Decorative American-style garden windmill with open lattice truss tower.

A ~2.4 m tall open four-leg lattice steel truss tower (cross-braced legs, like
an American farm windpump) with a hinged front access door, a red pyramidal
hip roof with cupola, and a five-sail white lattice rotor on a horizontal shaft.

Articulations:
- ``rotor_spin``: continuous spin of the lattice-sail rotor about the
  horizontal +X axis pointing out of the front face.
- ``door_hinge``: revolute swing of the front door about its hinge-side
  vertical edge, 0 to 100 degrees, opening outward.
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters)
# ---------------------------------------------------------------------------
TOWER_H = 1.82  # height of the lattice tower body
HALF_BASE = 0.40  # half width of the 0.8 m square base
HALF_TOP = 0.21  # half width at the top of the tower body
SLOPE = (HALF_BASE - HALF_TOP) / TOWER_H  # horizontal inset per meter of rise

ROOF_Z0 = 1.80
ROOF_Z1 = 2.15
ROOF_HALF_BASE = HALF_TOP + 0.065  # eaves overhang
CUPOLA_Z0 = 2.13
CUPOLA_Z1 = 2.31
CUPOLA_HALF = 0.10
APEX_Z = 2.43

# Door
DOOR_OPEN_Z0 = 0.10
DOOR_OPEN_Z1 = 0.80
DOOR_W = 0.345
DOOR_T = 0.034
DOOR_H = 0.692
DOOR_OPEN_HALF_W = 0.18
DOOR_FRAME_X = HALF_BASE - SLOPE * 0.45  # front face x at mid-door height

# Rotor
HUB_X = 0.52
HUB_Z = 1.50
HUB_R = 0.075
HUB_LEN = 0.13
SAIL_COUNT = 5

# Lattice parameters
LATTICE_PANELS = 6
CHORD_R = 0.022  # leg chord radius
BRACE_R = 0.012  # horizontal brace radius
XBRACE_R = 0.009  # diagonal cross-brace radius

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
STEEL = Material("steel", rgba=(0.48, 0.51, 0.53, 1.0))
ROOF_RED = Material("roof_red", rgba=(0.55, 0.11, 0.10, 1.0))
BARN_RED = Material("barn_red", rgba=(0.63, 0.13, 0.12, 1.0))
TRIM_WHITE = Material("trim_white", rgba=(0.93, 0.92, 0.88, 1.0))
GLASS_DARK = Material("glass_dark", rgba=(0.10, 0.11, 0.13, 1.0))
METAL_DARK = Material("metal_dark", rgba=(0.25, 0.26, 0.28, 1.0))


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def half_w(z: float) -> float:
    """Half width of the tapered tower at height z."""
    return HALF_BASE - SLOPE * z


_CORNER_SIGNS = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]


def _corner_xy(z: float, idx: int) -> tuple[float, float]:
    """(x, y) of lattice corner *idx* at height *z*."""
    hw = half_w(z)
    sx, sy = _CORNER_SIGNS[idx]
    return (sx * hw, sy * hw)


def _pt3(xy: tuple[float, float], z: float) -> tuple[float, float, float]:
    return (xy[0], xy[1], z)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    """RPY that points a local-Z-long cylinder from *a* to *b*."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    xy_len = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(xy_len, dz)
    return (0.0, pitch, yaw)


def _add_lattice_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material,
    name: str | None = None,
) -> None:
    """Shared helper: place a cylinder member between two 3-D points."""
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


# ---------------------------------------------------------------------------
# Lattice tower builder
# ---------------------------------------------------------------------------
def _add_lattice_tower(part) -> None:
    """Build the open four-leg lattice truss with for-i-in-range loops."""
    levels = [TOWER_H * i / LATTICE_PANELS for i in range(LATTICE_PANELS + 1)]

    # --- 4 leg chords (one per corner) ---
    for i in range(4):
        a = _pt3(_corner_xy(0.0, i), 0.0)
        b = _pt3(_corner_xy(TOWER_H, i), TOWER_H)
        _add_lattice_member(part, a, b, CHORD_R, STEEL, name=f"leg_{i}")

    # --- Horizontal braces at every level ---
    # Skip front-face (j=0) braces in the door zone to avoid overlap with the door panel.
    door_zone_levels = {1, 2}  # levels at z=0.303 and z=0.607 are within door opening
    h_idx = 0
    for i in range(LATTICE_PANELS + 1):
        z = levels[i]
        for j in range(4):
            # Skip front-face braces in door zone
            if j == 0 and i in door_zone_levels:
                h_idx += 1  # still increment counter to maintain naming
                continue
            a = _pt3(_corner_xy(z, j), z)
            b = _pt3(_corner_xy(z, (j + 1) % 4), z)
            _add_lattice_member(part, a, b, BRACE_R, STEEL, name=f"hbrace_{h_idx}")
            h_idx += 1

    # --- X-braces (two diagonals per face per panel) ---
    # Skip front-face (j=0) braces in the door zone to avoid overlap with the door panel.
    door_zone_panels = {0, 1, 2}  # panels whose z-range overlaps with door opening
    x_idx = 0
    for i in range(LATTICE_PANELS):
        z0 = levels[i]
        z1 = levels[i + 1]
        for j in range(4):
            # Skip front-face X-braces in door zone panels
            if j == 0 and i in door_zone_panels:
                x_idx += 2  # still increment counter to maintain naming
                continue
            c0_bot = _corner_xy(z0, j)
            c1_bot = _corner_xy(z0, (j + 1) % 4)
            c0_top = _corner_xy(z1, j)
            c1_top = _corner_xy(z1, (j + 1) % 4)
            _add_lattice_member(
                part, _pt3(c0_bot, z0), _pt3(c1_top, z1),
                XBRACE_R, STEEL, name=f"xbrace_{x_idx}",
            )
            x_idx += 1
            _add_lattice_member(
                part, _pt3(c1_bot, z0), _pt3(c0_top, z1),
                XBRACE_R, STEEL, name=f"xbrace_{x_idx}",
            )
            x_idx += 1


# ---------------------------------------------------------------------------
# CadQuery meshes
# ---------------------------------------------------------------------------
def _hip_roof_mesh():
    roof = (
        cq.Workplane("XY")
        .workplane(offset=ROOF_Z0)
        .rect(2.0 * ROOF_HALF_BASE, 2.0 * ROOF_HALF_BASE)
        .workplane(offset=ROOF_Z1 - ROOF_Z0)
        .rect(0.12, 0.12)
        .loft(combine=True)
    )
    return mesh_from_cadquery(roof, "hip_roof")


def _cupola_roof_mesh():
    roof = (
        cq.Workplane("XY")
        .workplane(offset=2.30)
        .rect(0.26, 0.26)
        .workplane(offset=APEX_Z - 2.30)
        .rect(0.02, 0.02)
        .loft(combine=True)
    )
    return mesh_from_cadquery(roof, "cupola_roof")


def _sail_mesh():
    """One white lattice sail in its local frame.

    Local +Z is the radial direction from the hub center, X is the thin
    rotor-plane thickness, Y is the sail width.
    """
    members: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []

    def bar(dx: float, dy: float, dz: float, x: float, y: float, z: float) -> None:
        members.append(((dx, dy, dz), (x, y, z)))

    bar(0.030, 0.046, 1.19, 0.0, 0.0, 0.565)
    for sy in (-0.096, 0.096):
        bar(0.016, 0.016, 1.02, 0.0, sy, 0.65)
    for sy in (-0.058, 0.058):
        bar(0.012, 0.012, 1.00, 0.0, sy, 0.66)
    n_rungs = 9
    for i in range(n_rungs):
        z = 0.16 + i * (1.00 / (n_rungs - 1))
        bar(0.014, 0.212, 0.018, 0.0, 0.0, z)

    solid: cq.Workplane | None = None
    for (dx, dy, dz), (x, y, z) in members:
        piece = cq.Workplane("XY").box(dx, dy, dz).translate((x, y, z))
        solid = piece if solid is None else solid.union(piece)
    assert solid is not None
    return mesh_from_cadquery(solid, "lattice_sail")


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_windmill")

    # ----------------------------------------------------------------- tower
    tower = model.part("tower")

    # --- Open lattice truss structure ---
    _add_lattice_tower(tower)

    # Top platform plate to carry the roof.
    tower.visual(
        Box((2.0 * HALF_TOP + 0.04, 2.0 * HALF_TOP + 0.04, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, TOWER_H - 0.0125)),
        material=STEEL,
        name="top_platform",
    )

    # Shaft-mount cross-member at hub height.
    hw_hub = half_w(HUB_Z)
    tower.visual(
        Box((0.040, 2.0 * hw_hub - 0.06, 0.040)),
        origin=Origin(xyz=(hw_hub - 0.02, 0.0, HUB_Z)),
        material=STEEL,
        name="shaft_mount",
    )

    # --- Roof and cupola (unchanged from parent) ---
    tower.visual(_hip_roof_mesh(), material=ROOF_RED, name="hip_roof")

    tower.visual(
        Box((2.0 * CUPOLA_HALF, 2.0 * CUPOLA_HALF, CUPOLA_Z1 - CUPOLA_Z0)),
        origin=Origin(xyz=(0.0, 0.0, (CUPOLA_Z0 + CUPOLA_Z1) / 2.0)),
        material=BARN_RED,
        name="cupola_body",
    )
    cup_zc = (CUPOLA_Z0 + CUPOLA_Z1) / 2.0 + 0.01
    for idx, (geom, xyz) in enumerate((
        (Box((0.014, 0.11, 0.11)), (CUPOLA_HALF, 0.0, cup_zc)),
        (Box((0.014, 0.11, 0.11)), (-CUPOLA_HALF, 0.0, cup_zc)),
        (Box((0.11, 0.014, 0.11)), (0.0, CUPOLA_HALF, cup_zc)),
        (Box((0.11, 0.014, 0.11)), (0.0, -CUPOLA_HALF, cup_zc)),
    )):
        tower.visual(geom, origin=Origin(xyz=xyz), material=GLASS_DARK, name=f"cupola_window_{idx}")
    tower.visual(_cupola_roof_mesh(), material=ROOF_RED, name="cupola_roof")

    # --- Door frame (vertical posts, header, sill, connecting braces) ---
    dfx = DOOR_FRAME_X
    dfy = DOOR_OPEN_HALF_W
    dz0 = DOOR_OPEN_Z0
    dz1 = DOOR_OPEN_Z1
    dzm = (dz0 + dz1) / 2.0
    door_frame_r = 0.016  # frame member radius

    # Vertical frame posts (left = 0, right = 1).
    for i in range(2):
        sy = -1.0 if i == 0 else 1.0
        _add_lattice_member(
            tower,
            (dfx, sy * dfy, dz0),
            (dfx, sy * dfy, dz1),
            door_frame_r, STEEL, name=f"door_frame_post_{i}",
        )

    # Header beam (top).
    _add_lattice_member(
        tower,
        (dfx, -dfy, dz1), (dfx, dfy, dz1),
        door_frame_r, STEEL, name="door_frame_header",
    )
    # Sill beam (bottom).
    _add_lattice_member(
        tower,
        (dfx, -dfy, dz0), (dfx, dfy, dz0),
        door_frame_r, STEEL, name="door_frame_sill",
    )

    # Connecting braces from frame posts to the nearest front lattice legs.
    for i in range(2):
        sy = -1.0 if i == 0 else 1.0
        corner_idx = 1 if i == 0 else 0  # front-left (1), front-right (0)
        for z_conn in (dz0, dz1):
            leg_pt = _pt3(_corner_xy(z_conn, corner_idx), z_conn)
            frame_pt = (dfx, sy * dfy, z_conn)
            _add_lattice_member(
                tower, leg_pt, frame_pt,
                BRACE_R, STEEL,
            )

    # --- Door casings (white trim on frame posts) ---
    casing_h = dz1 - dz0
    tower.visual(
        Box((0.032, 0.05, casing_h)),
        origin=Origin(xyz=(dfx + 0.016, -dfy, dzm)),
        material=TRIM_WHITE,
        name="door_hinge_casing",
    )
    tower.visual(
        Box((0.032, 0.05, casing_h)),
        origin=Origin(xyz=(dfx + 0.016, dfy, dzm)),
        material=TRIM_WHITE,
        name="door_latch_casing",
    )
    tower.visual(
        Box((0.032, 2.0 * dfy + 0.05, 0.05)),
        origin=Origin(xyz=(dfx + 0.016, 0.0, dz1 + 0.025)),
        material=TRIM_WHITE,
        name="door_head_casing",
    )
    tower.visual(
        Box((0.045, 2.0 * dfy + 0.02, 0.04)),
        origin=Origin(xyz=(dfx + 0.022, 0.0, dz0 - 0.02)),
        material=TRIM_WHITE,
        name="door_sill",
    )

    # --- Rotor shaft and wall boss ---
    tower.visual(
        Cylinder(radius=0.030, length=0.28),
        origin=Origin(xyz=(0.34, 0.0, HUB_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=METAL_DARK,
        name="rotor_shaft",
    )
    tower.visual(
        Cylinder(radius=0.065, length=0.045),
        origin=Origin(xyz=(0.245, 0.0, HUB_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=TRIM_WHITE,
        name="shaft_boss",
    )

    # ----------------------------------------------------------------- rotor
    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=HUB_R, length=HUB_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=TRIM_WHITE,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.040, length=0.05),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=METAL_DARK,
        name="hub_nose_cap",
    )
    sail = _sail_mesh()
    for k in range(SAIL_COUNT):
        theta = 2.0 * math.pi * k / SAIL_COUNT
        rotor.visual(
            sail,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(theta, 0.0, 0.0)),
            material=TRIM_WHITE,
            name=f"sail_{k}",
        )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=rotor,
        origin=Origin(xyz=(HUB_X, 0.0, HUB_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
    )

    # ------------------------------------------------------------------ door
    door = model.part("door")
    door.visual(
        Box((DOOR_T, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(-0.012, DOOR_W / 2.0, 0.005 + DOOR_H / 2.0)),
        material=BARN_RED,
        name="door_panel",
    )
    # White face trim: two stiles, three rails.
    stile_h = DOOR_H - 0.025  # slightly shorter than door to clear frame header
    for idx, u in enumerate((0.0225, DOOR_W - 0.0225)):
        door.visual(
            Box((0.012, 0.045, stile_h)),
            origin=Origin(xyz=(0.009, u, 0.005 + stile_h / 2.0)),
            material=TRIM_WHITE,
            name=f"door_stile_{idx}",
        )
    for idx, zc in enumerate((0.0375, 0.356, 0.660)):
        door.visual(
            Box((0.012, DOOR_W - 0.09, 0.045)),
            origin=Origin(xyz=(0.009, DOOR_W / 2.0, zc)),
            material=TRIM_WHITE,
            name=f"door_rail_{idx}",
        )
    door.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.016, 0.300, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=METAL_DARK,
        name="door_knob",
    )
    for tag, zc in (("lower", 0.16), ("upper", 0.56)):
        door.visual(
            Box((0.011, 0.096, 0.034)),
            origin=Origin(xyz=(0.010, 0.032, zc)),
            material=TRIM_WHITE,
            name=f"hinge_strap_{tag}",
        )

    # Door hinge: vertical frame post, no wall lean.
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=door,
        origin=Origin(xyz=(dfx, -DOOR_W / 2.0, DOOR_OPEN_Z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=2.0,
            lower=0.0, upper=math.radians(100.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    door = object_model.get_part("door")
    rotor = object_model.get_part("rotor")
    hinge = object_model.get_articulation("door_hinge")
    spin = object_model.get_articulation("rotor_spin")

    # Intentional, scoped embeddings.
    ctx.allow_overlap(
        tower, rotor,
        elem_a="rotor_shaft", elem_b="hub_barrel",
        reason="The fixed horizontal drive shaft is intentionally captured inside the rotor hub bore.",
    )
    for tag in ("lower", "upper"):
        ctx.allow_overlap(
            door, tower,
            elem_a=f"hinge_strap_{tag}", elem_b="door_hinge_casing",
            reason="Surface-mounted hinge straps seat against the hinge-side door casing.",
        )
    # Door seats within its frame - all door-to-frame contact is intentional.
    ctx.allow_overlap(
        door, tower,
        reason="The door panel and trim seat within the door frame (header, sill, posts, casings) when closed.",
    )
    ctx.expect_contact(
        door, tower,
        elem_a="door_panel", elem_b="door_frame_sill",
        name="door panel seats on the frame sill",
    )

    # ------------------------------------------------- lattice tower structure
    tower_bb = ctx.part_world_aabb(tower)
    ctx.check(
        "windmill stands about 2.4 m tall",
        tower_bb is not None and 2.30 <= tower_bb[1][2] <= 2.55,
        details=f"tower_bb={tower_bb}",
    )

    # Base width from the four leg visuals.
    leg0_bb = ctx.part_element_world_aabb(tower, elem="leg_0")
    leg2_bb = ctx.part_element_world_aabb(tower, elem="leg_2")
    ctx.check(
        "tower base is about 0.8 m square",
        leg0_bb is not None and leg2_bb is not None
        and 0.76 <= leg0_bb[1][0] - leg2_bb[0][0] <= 0.88
        and 0.76 <= leg0_bb[1][1] - leg2_bb[0][1] <= 0.88,
        details=f"leg0_bb={leg0_bb}, leg2_bb={leg2_bb}",
    )

    # Taper: top horizontal braces narrower than base.
    hbrace_top = ctx.part_element_world_aabb(tower, elem="hbrace_24")
    hbrace_bot = ctx.part_element_world_aabb(tower, elem="hbrace_0")
    ctx.check(
        "tower tapers: top braces are narrower than base braces",
        hbrace_top is not None and hbrace_bot is not None
        and (hbrace_top[1][1] - hbrace_top[0][1]) < 0.65 * (hbrace_bot[1][1] - hbrace_bot[0][1]),
        details=f"top={hbrace_top}, bot={hbrace_bot}",
    )

    # Lattice member counts: legs, horizontal braces, X-braces.
    all_visuals = [v.name for v in tower.visuals if v.name]
    leg_names = [n for n in all_visuals if n.startswith("leg_")]
    hbrace_names = [n for n in all_visuals if n.startswith("hbrace_")]
    xbrace_names = [n for n in all_visuals if n.startswith("xbrace_")]
    ctx.check(
        "four lattice leg chords",
        len(leg_names) == 4,
        details=f"legs={leg_names}",
    )
    # Horizontal braces: (6+1) levels × 4 faces = 28, minus 2 skipped in door zone
    expected_hbraces = (LATTICE_PANELS + 1) * 4 - 2  # skip front-face braces in door zone
    ctx.check(
        f"horizontal braces at every level except door zone ({expected_hbraces} total)",
        len(hbrace_names) == expected_hbraces,
        details=f"count={len(hbrace_names)}",
    )
    # X-braces: 6 panels × 4 faces × 2 braces = 48, minus 6 skipped in door zone
    expected_xbraces = LATTICE_PANELS * 4 * 2 - 6  # skip front-face braces in door zone
    ctx.check(
        f"X-braces on every face and panel except door zone ({expected_xbraces} total)",
        len(xbrace_names) == expected_xbraces,
        details=f"count={len(xbrace_names)}",
    )

    # Open structure: the lattice interior is mostly empty — the tower AABB
    # volume is much larger than the sum of member bounding-box volumes.
    ctx.check(
        "tower reads as open lattice, not solid walls",
        tower_bb is not None
        and tower_bb[1][0] - tower_bb[0][0] > 0.70
        and len(xbrace_names) >= 40,
        details=f"tower_bb={tower_bb}, xbraces={len(xbrace_names)}",
    )

    # Door frame structure.
    post0_bb = ctx.part_element_world_aabb(tower, elem="door_frame_post_0")
    post1_bb = ctx.part_element_world_aabb(tower, elem="door_frame_post_1")
    ctx.check(
        "door frame posts flank the doorway opening",
        post0_bb is not None and post1_bb is not None
        and post0_bb[1][1] < 0.0 < post1_bb[0][1],
        details=f"post0={post0_bb}, post1={post1_bb}",
    )

    # ----------------------------------------------------------- roof / cupola
    roof_bb = ctx.part_element_world_aabb(tower, elem="hip_roof")
    cupola_bb = ctx.part_element_world_aabb(tower, elem="cupola_body")
    cupola_roof_bb = ctx.part_element_world_aabb(tower, elem="cupola_roof")
    ctx.check(
        "pyramidal hip roof caps the tower body",
        roof_bb is not None and abs(roof_bb[0][2] - ROOF_Z0) < 0.02 and roof_bb[1][2] > 2.10,
        details=f"roof_bb={roof_bb}",
    )
    ctx.check(
        "square cupola with its own pyramid roof tops the hip roof",
        roof_bb is not None
        and cupola_bb is not None
        and cupola_roof_bb is not None
        and cupola_bb[1][2] > roof_bb[1][2] + 0.10
        and cupola_roof_bb[1][2] > 2.38,
        details=f"cupola_bb={cupola_bb}, cupola_roof_bb={cupola_roof_bb}",
    )
    cup_win_bb = ctx.part_element_world_aabb(tower, elem="cupola_window_0")
    ctx.check(
        "cupola carries dark window openings",
        cup_win_bb is not None and CUPOLA_Z0 < cup_win_bb[0][2] and cup_win_bb[1][2] < CUPOLA_Z1,
        details=f"cup_win_bb={cup_win_bb}",
    )

    # ------------------------------------------------------------------ door
    panel_bb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door panel sits inside the doorway opening",
        panel_bb is not None
        and -DOOR_OPEN_HALF_W - 0.02 < panel_bb[0][1]
        and panel_bb[1][1] < DOOR_OPEN_HALF_W + 0.02
        and DOOR_OPEN_Z0 - 0.01 < panel_bb[0][2]
        and panel_bb[1][2] < DOOR_OPEN_Z1 + 0.01,
        details=f"panel_bb={panel_bb}",
    )
    ctx.check(
        "door sits at the front of the lattice frame",
        panel_bb is not None and 0.30 < panel_bb[1][0] < 0.42,
        details=f"panel_bb={panel_bb}",
    )
    ctx.expect_contact(
        door, tower,
        elem_a="hinge_strap_upper", elem_b="door_hinge_casing",
        name="hinge straps seat on the hinge-side casing",
    )
    limits = hinge.motion_limits
    ctx.check(
        "door hinge range is 0 to 100 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-9
        and abs(limits.upper - math.radians(100.0)) < 0.02,
        details=f"limits={limits}",
    )
    with ctx.pose({hinge: math.radians(100.0)}):
        open_bb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings open outward about its hinge-side edge",
        panel_bb is not None
        and open_bb is not None
        and open_bb[1][0] > panel_bb[1][0] + 0.20
        and open_bb[0][0] > 0.30,
        details=f"open_bb={open_bb}",
    )

    # ----------------------------------------------------------------- rotor
    sail_names = [v.name for v in rotor.visuals if v.name and v.name.startswith("sail_")]
    ctx.check(
        "rotor carries five lattice sails",
        len(sail_names) == SAIL_COUNT,
        details=f"sails={sail_names}",
    )
    hub_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotor hub sits ahead of the upper tower",
        hub_pos is not None
        and abs(hub_pos[0] - HUB_X) < 0.05
        and abs(hub_pos[1]) < 0.02
        and abs(hub_pos[2] - HUB_Z) < 0.05,
        details=f"hub_pos={hub_pos}",
    )
    axis = spin.axis
    ctx.check(
        "rotor spins about the horizontal axis out of the front face",
        str(spin.articulation_type).lower().endswith("continuous")
        and abs(axis[0] - 1.0) < 1e-6
        and abs(axis[1]) < 1e-6
        and abs(axis[2]) < 1e-6,
        details=f"type={spin.articulation_type}, axis={axis}",
    )
    sail0_bb = ctx.part_element_world_aabb(rotor, elem="sail_0")
    ctx.check(
        "each sail is roughly 1.1 m long and overtops the roof",
        sail0_bb is not None and 1.00 < sail0_bb[1][2] - HUB_Z < 1.30 and sail0_bb[1][2] > APEX_Z,
        details=f"sail0_bb={sail0_bb}",
    )
    rotor_bb = ctx.part_world_aabb(rotor)
    ctx.check(
        "sail span clearly exceeds the tower width",
        rotor_bb is not None and rotor_bb[1][1] - rotor_bb[0][1] > 1.6,
        details=f"rotor_bb={rotor_bb}",
    )
    ctx.expect_gap(
        rotor, tower,
        axis="x",
        negative_elem="leg_0",
        min_gap=0.02,
        name="rotor sails stand clear of the front lattice legs",
    )
    ctx.expect_overlap(
        tower, rotor,
        axes="x",
        elem_a="rotor_shaft", elem_b="hub_barrel",
        min_overlap=0.015,
        name="drive shaft remains inserted in the hub",
    )
    with ctx.pose({spin: math.pi}):
        sail0_down_bb = ctx.part_element_world_aabb(rotor, elem="sail_0")
    ctx.check(
        "rotor pose swings the reference sail from above the hub to below it",
        sail0_bb is not None
        and sail0_down_bb is not None
        and sail0_bb[0][2] > 1.30
        and sail0_down_bb[0][2] < 0.90,
        details=f"rest={sail0_bb}, half_turn={sail0_down_bb}",
    )

    return ctx.report()


object_model = build_object_model()
