from __future__ import annotations

"""Decorative American-style garden windmill — tail-vane yaw variant.

A ~2.4 m tall tapered four-sided tower with red lapboard siding and white
trim, a hinged front door, multi-pane windows, a pyramidal hip roof with a
small cupola, and a five-sail white lattice rotor on a horizontal shaft.

Variant: a directional tail-vane fin on a horizontal boom yaws the rotor
head left–right about the vertical Z axis at the tower top.

Articulations:
- ``rotor_yaw``: revolute yaw of the rotor head (shaft + rotor + boom + vane)
  about the vertical Z axis at the tower top, ±60°.
- ``rotor_spin``: continuous spin of the lattice-sail rotor about the
  horizontal +X axis pointing out of the front face.
- ``door_hinge``: revolute swing of the front door about its hinge-side
  vertical edge (tilted with the tapered wall), 0 to 100 degrees, opening
  outward.
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
TOWER_H = 1.82  # height of the wooden tower body
HALF_BASE = 0.40  # half width of the 0.8 m square base
HALF_TOP = 0.21  # half width at the top of the tower body
SLOPE = (HALF_BASE - HALF_TOP) / TOWER_H  # horizontal inset per meter of rise
LEAN = math.atan(SLOPE)  # wall lean angle from vertical (rad)
WALL_T = 0.031  # wall thickness of the hollow shell

ROOF_Z0 = 1.80
ROOF_Z1 = 2.15
ROOF_HALF_BASE = HALF_TOP + 0.065  # eaves overhang
CUPOLA_Z0 = 2.13
CUPOLA_Z1 = 2.31
CUPOLA_HALF = 0.10
APEX_Z = 2.43  # top of the cupola's tiny pyramid roof

# Door opening (front face)
DOOR_OPEN_HALF_W = 0.18
DOOR_OPEN_Z0 = 0.10
DOOR_OPEN_Z1 = 0.80
DOOR_W = 0.345
DOOR_T = 0.034
DOOR_H = 0.692

# Front window above the door
FWIN_W = 0.26
FWIN_H = 0.32
FWIN_ZC = 1.17

# Small side windows
SWIN_W = 0.18
SWIN_H = 0.22
SWIN_ZC = 0.975

# Rotor
HUB_X = 0.52
HUB_Z = 1.50
HUB_R = 0.075
HUB_LEN = 0.13
SAIL_TIP_R = 1.16  # ~1.1 m lattice sail measured from the hub
SAIL_COUNT = 5

# Yaw / tail-vane assembly
YAW_Z = 1.82  # yaw bearing height at the tower top
BOOM_LEN = 0.55
BOOM_S = 0.028  # boom cross-section
VANE_DEPTH = 0.26  # vane extent along the boom direction (X)
VANE_HEIGHT = 0.34  # vane vertical extent (Z)
VANE_T = 0.012  # vane plate thickness (Y)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
BARN_RED = Material("barn_red", rgba=(0.63, 0.13, 0.12, 1.0))
SIDING_RED = Material("siding_red", rgba=(0.52, 0.10, 0.09, 1.0))
ROOF_RED = Material("roof_red", rgba=(0.55, 0.11, 0.10, 1.0))
TRIM_WHITE = Material("trim_white", rgba=(0.93, 0.92, 0.88, 1.0))
GLASS_DARK = Material("glass_dark", rgba=(0.10, 0.11, 0.13, 1.0))
METAL_DARK = Material("metal_dark", rgba=(0.25, 0.26, 0.28, 1.0))
VANE_CREAM = Material("vane_cream", rgba=(0.92, 0.90, 0.82, 1.0))


def half_w(z: float) -> float:
    """Half width of the tapered tower body at height z."""
    return HALF_BASE - SLOPE * z


def face_origin(yaw: float, lateral: float, z: float, *, proud: float = 0.0) -> Origin:
    """Origin on a tapered tower face.

    The returned frame has local +X along the face's outward normal,
    local +Y along the in-face horizontal, and local +Z up the sloped wall.
    ``proud`` offsets radially beyond the outer wall surface.
    """
    r = half_w(z) + proud
    x = r * math.cos(yaw) - lateral * math.sin(yaw)
    y = r * math.sin(yaw) + lateral * math.cos(yaw)
    return Origin(xyz=(x, y, z), rpy=(0.0, -LEAN, yaw))


def _head_face_origin(yaw: float, lateral: float, z: float, *, proud: float = 0.0) -> Origin:
    """Like face_origin but expressed in the rotor_head local frame."""
    o = face_origin(yaw, lateral, z, proud=proud)
    return Origin(xyz=(o.xyz[0], o.xyz[1], o.xyz[2] - YAW_Z), rpy=o.rpy)


# ---------------------------------------------------------------------------
# CadQuery meshes
# ---------------------------------------------------------------------------
def _cut_box(cx: float, cy: float, cz: float, dx: float, dy: float, dz: float) -> cq.Workplane:
    return cq.Workplane("XY").box(dx, dy, dz).translate((cx, cy, cz))


def _tower_shell_mesh():
    outer = (
        cq.Workplane("XY")
        .rect(2.0 * HALF_BASE, 2.0 * HALF_BASE)
        .workplane(offset=TOWER_H)
        .rect(2.0 * HALF_TOP, 2.0 * HALF_TOP)
        .loft(combine=True)
    )
    iw0 = 2.0 * (half_w(0.05) - WALL_T)
    iw1 = 2.0 * (half_w(1.90) - WALL_T)
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.05)
        .rect(iw0, iw0)
        .workplane(offset=1.85)
        .rect(iw1, iw1)
        .loft(combine=True)
    )
    shell = outer.cut(cavity)
    # Door opening on the lower front face.
    shell = shell.cut(
        _cut_box(0.33, 0.0, (DOOR_OPEN_Z0 + DOOR_OPEN_Z1) / 2.0, 0.30, 2.0 * DOOR_OPEN_HALF_W, DOOR_OPEN_Z1 - DOOR_OPEN_Z0)
    )
    # Multi-pane window opening above the door.
    shell = shell.cut(_cut_box(0.30, 0.0, FWIN_ZC, 0.28, FWIN_W, FWIN_H))
    # Small window openings on the two side faces.
    shell = shell.cut(_cut_box(0.0, 0.30, SWIN_ZC, SWIN_W, 0.30, SWIN_H))
    shell = shell.cut(_cut_box(0.0, -0.30, SWIN_ZC, SWIN_W, 0.30, SWIN_H))
    return mesh_from_cadquery(shell, "tower_shell")


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

    # Main spar from inside the hub out to the tip.
    bar(0.030, 0.046, 1.19, 0.0, 0.0, 0.565)
    # Outer edge stringers.
    for sy in (-0.096, 0.096):
        bar(0.016, 0.016, 1.02, 0.0, sy, 0.65)
    # Intermediate longerons forming the grid columns.
    for sy in (-0.058, 0.058):
        bar(0.012, 0.012, 1.00, 0.0, sy, 0.66)
    # Cross rungs forming the open rectangular grid cells.
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
# Tower detailing helpers
# ---------------------------------------------------------------------------
def _add_siding(part) -> None:
    """Horizontal lapboard shadow strips on all four tapered faces."""
    faces = {
        0.0: [
            (-0.19, 0.19, 0.06, 0.88),  # door + casing zone
            (-0.185, 0.185, 0.94, 1.40),  # front window zone
            (-0.08, 0.08, 1.40, 1.85),  # front bracket + shaft boss zone (yaw mechanism)
        ],
        math.pi / 2.0: [(-0.14, 0.14, 0.80, 1.16)],
        -math.pi / 2.0: [(-0.14, 0.14, 0.80, 1.16)],
        math.pi: [],
    }
    for yaw, openings in faces.items():
        z = 0.16
        while z < 1.66:
            avail = half_w(z) - 0.040
            spans = [(-avail, avail)]
            for (u0, u1, z0, z1) in openings:
                if z0 <= z <= z1:
                    trimmed: list[tuple[float, float]] = []
                    for a, b in spans:
                        if u1 <= a or u0 >= b:
                            trimmed.append((a, b))
                            continue
                        if u0 - a > 0.07:
                            trimmed.append((a, u0))
                        if b - u1 > 0.07:
                            trimmed.append((u1, b))
                    spans = trimmed
            for a, b in spans:
                if b - a < 0.07:
                    continue
                part.visual(
                    Box((0.014, b - a, 0.026)),
                    origin=face_origin(yaw, (a + b) / 2.0, z, proud=0.0035),
                    material=SIDING_RED,
                )
            z += 0.12


def _add_window(part, yaw: float, z_c: float, w: float, h: float, prefix: str, cols: int, rows: int) -> None:
    """White-framed multi-pane window with a recessed dark pane."""
    bar = 0.042
    part.visual(
        Box((0.030, w + 2.0 * bar, bar)),
        origin=face_origin(yaw, 0.0, z_c + h / 2.0 + bar / 2.0, proud=0.004),
        material=TRIM_WHITE,
        name=f"{prefix}_frame_head",
    )
    part.visual(
        Box((0.030, w + 2.0 * bar, bar)),
        origin=face_origin(yaw, 0.0, z_c - h / 2.0 - bar / 2.0, proud=0.004),
        material=TRIM_WHITE,
        name=f"{prefix}_frame_sill",
    )
    for idx, u in enumerate((-(w / 2.0 + bar / 2.0), w / 2.0 + bar / 2.0)):
        part.visual(
            Box((0.030, bar, h)),
            origin=face_origin(yaw, u, z_c, proud=0.004),
            material=TRIM_WHITE,
            name=f"{prefix}_jamb_{idx}",
        )
    # Recessed dark glazing, oversized so it seats into the wall reveal.
    part.visual(
        Box((0.012, w + 0.04, h + 0.04)),
        origin=face_origin(yaw, 0.0, z_c, proud=-0.022),
        material=GLASS_DARK,
        name=f"{prefix}_pane",
    )
    # White muntin bars dividing the panes.
    for i in range(1, cols):
        u = -w / 2.0 + w * i / cols
        part.visual(
            Box((0.012, 0.018, h)),
            origin=face_origin(yaw, u, z_c, proud=-0.013),
            material=TRIM_WHITE,
        )
    for j in range(1, rows):
        zz = z_c - h / 2.0 + h * j / rows
        part.visual(
            Box((0.012, w, 0.018)),
            origin=face_origin(yaw, 0.0, zz, proud=-0.013),
            material=TRIM_WHITE,
        )


def _edge_rpy(d: tuple[float, float, float]) -> tuple[float, float, float]:
    """rpy that points a local-Z-long box along direction d."""
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(math.hypot(d[0], d[1]), d[2])
    return (0.0, pitch, yaw)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_windmill_tail_vane")

    # ----------------------------------------------------------------- tower
    tower = model.part("tower")
    tower.visual(_tower_shell_mesh(), material=BARN_RED, name="tower_shell")
    tower.visual(_hip_roof_mesh(), material=ROOF_RED, name="hip_roof")

    # Cupola with dark window openings and its own tiny pyramid roof.
    tower.visual(
        Box((2.0 * CUPOLA_HALF, 2.0 * CUPOLA_HALF, CUPOLA_Z1 - CUPOLA_Z0)),
        origin=Origin(xyz=(0.0, 0.0, (CUPOLA_Z0 + CUPOLA_Z1) / 2.0)),
        material=BARN_RED,
        name="cupola_body",
    )
    cup_zc = (CUPOLA_Z0 + CUPOLA_Z1) / 2.0 + 0.01
    cupola_windows = (
        (Box((0.014, 0.11, 0.11)), (CUPOLA_HALF, 0.0, cup_zc)),
        (Box((0.014, 0.11, 0.11)), (-CUPOLA_HALF, 0.0, cup_zc)),
        (Box((0.11, 0.014, 0.11)), (0.0, CUPOLA_HALF, cup_zc)),
        (Box((0.11, 0.014, 0.11)), (0.0, -CUPOLA_HALF, cup_zc)),
    )
    for idx, (geom, xyz) in enumerate(cupola_windows):
        tower.visual(geom, origin=Origin(xyz=xyz), material=GLASS_DARK, name=f"cupola_window_{idx}")
    tower.visual(_cupola_roof_mesh(), material=ROOF_RED, name="cupola_roof")

    # White corner boards along the four tapered edges.
    edge_len = math.hypot(math.hypot(HALF_BASE - HALF_TOP, HALF_BASE - HALF_TOP), TOWER_H) + 0.02
    for sx in (1.0, -1.0):
        for sy in (1.0, -1.0):
            d = (sx * (HALF_TOP - HALF_BASE), sy * (HALF_TOP - HALF_BASE), TOWER_H)
            mid = (sx * (HALF_BASE + HALF_TOP) / 2.0, sy * (HALF_BASE + HALF_TOP) / 2.0, TOWER_H / 2.0)
            tower.visual(
                Box((0.055, 0.055, edge_len)),
                origin=Origin(xyz=mid, rpy=_edge_rpy(d)),
                material=TRIM_WHITE,
            )

    # White base skirt boards.
    tower.visual(Box((0.03, 0.86, 0.08)), origin=Origin(xyz=(0.405, 0.0, 0.04)), material=TRIM_WHITE)
    tower.visual(Box((0.03, 0.86, 0.08)), origin=Origin(xyz=(-0.405, 0.0, 0.04)), material=TRIM_WHITE)
    tower.visual(Box((0.86, 0.03, 0.08)), origin=Origin(xyz=(0.0, 0.405, 0.04)), material=TRIM_WHITE)
    tower.visual(Box((0.86, 0.03, 0.08)), origin=Origin(xyz=(0.0, -0.405, 0.04)), material=TRIM_WHITE)

    # White frieze band under the eaves.
    frieze_w = 2.0 * half_w(1.75) + 0.06
    for idx, yaw in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        tower.visual(
            Box((0.024, frieze_w, 0.10)),
            origin=face_origin(yaw, 0.0, 1.75, proud=0.005),
            material=TRIM_WHITE,
            name=f"frieze_band_{idx}",
        )

    # Lapboard siding shadow strips.
    _add_siding(tower)

    # Windows: multi-pane front window plus two small side windows.
    _add_window(tower, 0.0, FWIN_ZC, FWIN_W, FWIN_H, "front_window", cols=2, rows=3)
    _add_window(tower, math.pi / 2.0, SWIN_ZC, SWIN_W, SWIN_H, "side_window_0", cols=2, rows=2)
    _add_window(tower, -math.pi / 2.0, SWIN_ZC, SWIN_W, SWIN_H, "side_window_1", cols=2, rows=2)

    # White door casing around the front opening.
    tower.visual(
        Box((0.032, 0.05, 0.70)),
        origin=face_origin(0.0, -(DOOR_OPEN_HALF_W + 0.025), 0.45, proud=0.004),
        material=TRIM_WHITE,
        name="door_hinge_casing",
    )
    tower.visual(
        Box((0.032, 0.05, 0.70)),
        origin=face_origin(0.0, DOOR_OPEN_HALF_W + 0.025, 0.45, proud=0.004),
        material=TRIM_WHITE,
        name="door_latch_casing",
    )
    tower.visual(
        Box((0.032, 0.50, 0.05)),
        origin=face_origin(0.0, 0.0, 0.825, proud=0.004),
        material=TRIM_WHITE,
        name="door_head_casing",
    )
    tower.visual(
        Box((0.045, 0.46, 0.05)),
        origin=face_origin(0.0, 0.0, 0.075, proud=0.012),
        material=TRIM_WHITE,
        name="door_sill",
    )

    # --------------------------------------------------------- rotor head
    # The rotor head carries the horizontal shaft, the rotor, the tail boom,
    # and the directional vane.  It yaws about the vertical Z axis at the
    # tower top so the tail vane can align the rotor into the wind.
    rotor_head = model.part("rotor_head")

    # Yaw plate — a flat horizontal steel plate at the tower top that
    # connects the front bracket to the rear strut, extending backward
    # past the roof eaves. This is the structural base for the yaw bearing.
    yaw_plate_front_x = 0.24
    yaw_plate_rear_x = -0.30  # extends behind the roof eave
    yaw_plate_len = yaw_plate_front_x - yaw_plate_rear_x
    yaw_plate_center_x = (yaw_plate_front_x + yaw_plate_rear_x) / 2.0
    rotor_head.visual(
        Box((yaw_plate_len, 0.08, 0.022)),
        origin=Origin(xyz=(yaw_plate_center_x, 0.0, 0.011)),
        material=METAL_DARK,
        name="yaw_plate",
    )

    # Front bracket — a dark metal plate on the front face, connecting the yaw
    # plate down to the shaft position. Placed outside the siding strips to
    # avoid overlap.
    rotor_head.visual(
        Box((0.028, 0.10, 0.34)),
        origin=_head_face_origin(0.0, 0.0, 1.66, proud=0.015),
        material=METAL_DARK,
        name="front_bracket",
    )

    # Horizontal drive shaft (carried by the head, not the tower).
    shaft_z_local = HUB_Z - YAW_Z  # = -0.32
    rotor_head.visual(
        Cylinder(radius=0.030, length=0.28),
        origin=Origin(xyz=(0.34, 0.0, shaft_z_local), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=METAL_DARK,
        name="rotor_shaft",
    )
    rotor_head.visual(
        Cylinder(radius=0.065, length=0.045),
        origin=Origin(xyz=(0.245, 0.0, shaft_z_local), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=TRIM_WHITE,
        name="shaft_boss",
    )

    # Tail boom — a horizontal bar extending backward from the rear strut.
    boom_z_local = -0.12  # world z ≈ 1.70, just below the frieze band

    # Rear strut — a vertical bar hanging from the back edge of the yaw plate
    # down to the boom level, positioned behind the tower so it clears the
    # roof and frieze band.
    strut_x = yaw_plate_rear_x + 0.015  # just inside the yaw plate rear edge
    strut_top_z = 0.0  # world z = YAW_Z
    strut_bottom_z = boom_z_local  # world z = YAW_Z + boom_z_local
    strut_len = strut_top_z - strut_bottom_z
    strut_center_z = (strut_top_z + strut_bottom_z) / 2.0
    rotor_head.visual(
        Box((0.028, 0.028, strut_len)),
        origin=Origin(xyz=(strut_x, 0.0, strut_center_z)),
        material=METAL_DARK,
        name="rear_strut",
    )
    boom_rear_x = -(half_w(1.70) + 0.030)  # just outside the rear wall at boom height
    boom_center_x = boom_rear_x - BOOM_LEN / 2.0
    rotor_head.visual(
        Box((BOOM_LEN, BOOM_S, BOOM_S)),
        origin=Origin(xyz=(boom_center_x, 0.0, boom_z_local)),
        material=METAL_DARK,
        name="tail_boom",
    )

    # Tail vane — a flat directional fin at the boom end, surface normal
    # along Y so cross-winds push the boom sideways and yaw the head.
    vane_center_x = boom_rear_x - BOOM_LEN - VANE_DEPTH / 2.0
    rotor_head.visual(
        Box((VANE_DEPTH, VANE_T, VANE_HEIGHT)),
        origin=Origin(xyz=(vane_center_x, 0.0, boom_z_local)),
        material=VANE_CREAM,
        name="tail_vane",
    )
    # Vane frame: white border bars around the fin edges.
    vane_frame_bars = (
        # top rail
        ((VANE_DEPTH + 0.02, 0.024, 0.024), (vane_center_x, 0.0, boom_z_local + VANE_HEIGHT / 2.0)),
        # bottom rail
        ((VANE_DEPTH + 0.02, 0.024, 0.024), (vane_center_x, 0.0, boom_z_local - VANE_HEIGHT / 2.0)),
        # leading edge upright
        ((0.024, 0.024, VANE_HEIGHT), (vane_center_x + VANE_DEPTH / 2.0, 0.0, boom_z_local)),
        # trailing edge upright
        ((0.024, 0.024, VANE_HEIGHT), (vane_center_x - VANE_DEPTH / 2.0, 0.0, boom_z_local)),
    )
    for idx, (size, xyz) in enumerate(vane_frame_bars):
        rotor_head.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=TRIM_WHITE,
            name=f"vane_frame_{idx}",
        )

    # Yaw articulation: rotor head rotates about the vertical Z axis at the
    # tower top.  Positive q yaws the boom toward +Y (left when facing front).
    model.articulation(
        "rotor_yaw",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=rotor_head,
        origin=Origin(xyz=(0.0, 0.0, YAW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.0,
            lower=-math.radians(60.0),
            upper=math.radians(60.0),
        ),
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

    # Rotor spin: continuous rotation about the horizontal shaft axis.
    # Parent is now the rotor_head so the rotor yaws with the head.
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=rotor_head,
        child=rotor,
        origin=Origin(xyz=(HUB_X, 0.0, HUB_Z - YAW_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
    )

    # ------------------------------------------------------------------ door
    # The door lies in the sloped front-wall plane; its hinge runs along the
    # hinge-side edge of the doorway. In the joint/child frame local +X is the
    # outward wall normal, +Y runs across the door, +Z runs up the wall.
    door = model.part("door")
    door.visual(
        Box((DOOR_T, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(-0.012, DOOR_W / 2.0, 0.005 + DOOR_H / 2.0)),
        material=BARN_RED,
        name="door_panel",
    )
    # White face trim: two stiles, three rails.
    for u in (0.0225, DOOR_W - 0.0225):
        door.visual(
            Box((0.012, 0.045, DOOR_H)),
            origin=Origin(xyz=(0.009, u, 0.005 + DOOR_H / 2.0)),
            material=TRIM_WHITE,
        )
    for zc in (0.0375, 0.356, 0.6745):
        door.visual(
            Box((0.012, DOOR_W - 0.09, 0.045)),
            origin=Origin(xyz=(0.009, DOOR_W / 2.0, zc)),
            material=TRIM_WHITE,
        )
    # Dark knob on the latch side.
    door.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.016, 0.300, 0.360), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=METAL_DARK,
        name="door_knob",
    )
    # White hinge straps crossing onto the hinge-side casing.
    for tag, zc in (("lower", 0.16), ("upper", 0.56)):
        door.visual(
            Box((0.011, 0.096, 0.034)),
            origin=Origin(xyz=(0.010, 0.032, zc)),
            material=TRIM_WHITE,
            name=f"hinge_strap_{tag}",
        )

    hinge_r = half_w(DOOR_OPEN_Z0) - 0.004
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=door,
        origin=Origin(xyz=(hinge_r, -0.1725, DOOR_OPEN_Z0), rpy=(0.0, -LEAN, 0.0)),
        # Hinge axis runs along the door's hinge edge in the wall plane;
        # -Z in the tilted joint frame makes positive q swing the free edge
        # outward (+X), away from the tower.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    door = object_model.get_part("door")
    rotor = object_model.get_part("rotor")
    rotor_head = object_model.get_part("rotor_head")
    hinge = object_model.get_articulation("door_hinge")
    spin = object_model.get_articulation("rotor_spin")
    yaw = object_model.get_articulation("rotor_yaw")

    # Intentional, scoped embeddings.
    ctx.allow_overlap(
        rotor_head,
        rotor,
        elem_a="rotor_shaft",
        elem_b="hub_barrel",
        reason="The horizontal drive shaft is intentionally captured inside the rotor hub bore.",
    )
    ctx.allow_overlap(
        rotor_head,
        tower,
        elem_a="rotor_shaft",
        elem_b="tower_shell",
        reason="The horizontal drive shaft passes through the front tower wall to carry the rotor; small local penetration at the wall bore.",
    )
    ctx.allow_overlap(
        rotor_head,
        tower,
        elem_a="shaft_boss",
        elem_b="tower_shell",
        reason="The shaft bearing boss seats into the front tower wall as a bearing housing; small local embed at the bore surround.",
    )
    # The front bracket is a structural member that passes through the front
    # tower face. It needs explicit element-level allowances for every tower
    # visual it overlaps with.
    for _fb_elem_b in ("tower_shell", "hip_roof", "frieze_band_0",
                       "frieze_band_1", "frieze_band_2", "frieze_band_3"):
        ctx.allow_overlap(
            rotor_head,
            tower,
            elem_a="front_bracket",
            elem_b=_fb_elem_b,
            reason=f"Front bracket passes through the front face zone; small local embed into {_fb_elem_b}.",
        )
    ctx.allow_overlap(
        rotor_head,
        tower,
        elem_a="yaw_plate",
        elem_b="hip_roof",
        reason="The yaw plate sits at the tower top inside the hip-roof envelope; the structural plate is embedded in the roof volume as part of the yaw mechanism.",
    )
    ctx.allow_overlap(
        rotor_head,
        tower,
        elem_a="rear_strut",
        reason="The rear strut is a structural member passing through the rear tower face and hip roof; it penetrates the shell, siding strips, and roof eave as part of the yaw mechanism support.",
    )
    ctx.allow_overlap(
        rotor_head,
        tower,
        elem_a="yaw_plate",
        elem_b="tower_shell",
        reason="The yaw plate sits directly on top of the tower shell at the yaw bearing height; small local embed at the seating surface.",
    )
    for tag in ("lower", "upper"):
        ctx.allow_overlap(
            door,
            tower,
            elem_a=f"hinge_strap_{tag}",
            elem_b="door_hinge_casing",
            reason="Surface-mounted hinge straps seat against the hinge-side door casing.",
        )

    # ----------------------------------------------------- overall structure
    shell_bb = ctx.part_element_world_aabb(tower, elem="tower_shell")
    tower_bb = ctx.part_world_aabb(tower)
    ctx.check(
        "tower base is about 0.8 m square",
        shell_bb is not None
        and 0.78 <= shell_bb[1][0] - shell_bb[0][0] <= 0.84
        and 0.78 <= shell_bb[1][1] - shell_bb[0][1] <= 0.84,
        details=f"shell_bb={shell_bb}",
    )
    ctx.check(
        "windmill stands about 2.4 m tall",
        tower_bb is not None and 2.30 <= tower_bb[1][2] <= 2.55,
        details=f"tower_bb={tower_bb}",
    )
    frieze_bb = ctx.part_element_world_aabb(tower, elem="frieze_band_0")
    ctx.check(
        "tower tapers outward from top to base",
        shell_bb is not None
        and frieze_bb is not None
        and (frieze_bb[1][1] - frieze_bb[0][1]) < 0.65 * (shell_bb[1][1] - shell_bb[0][1]),
        details=f"frieze_bb={frieze_bb}",
    )
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

    # -------------------------------------------------------------- windows
    pane_bb = ctx.part_element_world_aabb(tower, elem="front_window_pane")
    jamb_bb = ctx.part_element_world_aabb(tower, elem="front_window_jamb_0")
    ctx.check(
        "front window pane is recessed behind its white frame",
        pane_bb is not None and jamb_bb is not None and pane_bb[1][0] + 0.008 < jamb_bb[1][0],
        details=f"pane_bb={pane_bb}, jamb_bb={jamb_bb}",
    )
    side0_bb = ctx.part_element_world_aabb(tower, elem="side_window_0_pane")
    side1_bb = ctx.part_element_world_aabb(tower, elem="side_window_1_pane")
    ctx.check(
        "small windows sit on both side faces",
        side0_bb is not None
        and side1_bb is not None
        and side0_bb[0][1] > 0.20
        and side1_bb[1][1] < -0.20,
        details=f"side0_bb={side0_bb}, side1_bb={side1_bb}",
    )

    # ----------------------------------------------------------------- door
    panel_bb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door panel sits inside the doorway opening",
        panel_bb is not None
        and -DOOR_OPEN_HALF_W < panel_bb[0][1]
        and panel_bb[1][1] < DOOR_OPEN_HALF_W
        and DOOR_OPEN_Z0 - 0.01 < panel_bb[0][2]
        and panel_bb[1][2] < DOOR_OPEN_Z1,
        details=f"panel_bb={panel_bb}",
    )
    ctx.check(
        "door sits flush in the lower front wall",
        panel_bb is not None and 0.35 < panel_bb[1][0] < 0.41,
        details=f"panel_bb={panel_bb}",
    )
    ctx.check(
        "front window sits above the door",
        panel_bb is not None and pane_bb is not None and pane_bb[0][2] > panel_bb[1][2],
        details=f"panel_top={panel_bb[1][2] if panel_bb else None}",
    )
    ctx.expect_contact(
        door,
        tower,
        elem_a="hinge_strap_upper",
        elem_b="door_hinge_casing",
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

    # ---------------------------------------------------------------- rotor
    sail_names = [v.name for v in rotor.visuals if v.name and v.name.startswith("sail_")]
    ctx.check(
        "rotor carries five lattice sails",
        len(sail_names) == SAIL_COUNT,
        details=f"sails={sail_names}",
    )
    hub_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotor hub sits ahead of the upper front face",
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
        rotor,
        tower,
        axis="x",
        negative_elem="tower_shell",
        min_gap=0.02,
        name="rotor assembly stands clear of the front wall",
    )
    ctx.expect_overlap(
        rotor_head,
        rotor,
        axes="x",
        elem_a="rotor_shaft",
        elem_b="hub_barrel",
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

    # -------------------------------------------------------- rotor yaw + tail vane
    yaw_axis = yaw.axis
    ctx.check(
        "rotor yaw is revolute about the vertical Z axis at the tower top",
        str(yaw.articulation_type).lower().endswith("revolute")
        and abs(yaw_axis[0]) < 1e-6
        and abs(yaw_axis[1]) < 1e-6
        and abs(yaw_axis[2] - 1.0) < 1e-6,
        details=f"type={yaw.articulation_type}, axis={yaw_axis}",
    )
    yaw_limits = yaw.motion_limits
    ctx.check(
        "rotor yaw range is ±60 degrees",
        yaw_limits is not None
        and yaw_limits.lower is not None
        and yaw_limits.upper is not None
        and abs(yaw_limits.lower + math.radians(60.0)) < 0.02
        and abs(yaw_limits.upper - math.radians(60.0)) < 0.02,
        details=f"limits={yaw_limits}",
    )
    yaw_origin = yaw.origin
    ctx.check(
        "yaw joint origin sits at the tower top height",
        yaw_origin is not None
        and abs(yaw_origin.xyz[0]) < 0.01
        and abs(yaw_origin.xyz[1]) < 0.01
        and abs(yaw_origin.xyz[2] - YAW_Z) < 0.02,
        details=f"origin={yaw_origin}",
    )

    # Tail vane exists behind the tower.
    vane_bb = ctx.part_element_world_aabb(rotor_head, elem="tail_vane")
    ctx.check(
        "tail vane fin sits behind the tower",
        vane_bb is not None and vane_bb[1][0] < -0.40,
        details=f"vane_bb={vane_bb}",
    )
    boom_bb = ctx.part_element_world_aabb(rotor_head, elem="tail_boom")
    ctx.check(
        "tail boom extends behind the tower carrying the vane",
        boom_bb is not None
        and boom_bb[0][0] < -0.30
        and boom_bb[1][0] > -1.20,
        details=f"boom_bb={boom_bb}",
    )

    # Yaw pose: positive q swings the boom toward +Y (left looking from front).
    vane_rest_center_y = (vane_bb[0][1] + vane_bb[1][1]) / 2.0 if vane_bb else 0.0
    with ctx.pose({yaw: math.radians(45.0)}):
        vane_yawed_bb = ctx.part_element_world_aabb(rotor_head, elem="tail_vane")
    ctx.check(
        "yaw pose swings the tail vane laterally",
        vane_bb is not None
        and vane_yawed_bb is not None
        and abs(vane_yawed_bb[0][1] - vane_bb[0][1]) > 0.20,
        details=f"rest={vane_bb}, yawed={vane_yawed_bb}",
    )

    # Rotor moves with the head when yawed.
    hub_rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: math.radians(30.0)}):
        hub_yawed_pos = ctx.part_world_position(rotor)
    ctx.check(
        "rotor hub swings laterally when the head yaws",
        hub_rest_pos is not None
        and hub_yawed_pos is not None
        and abs(hub_yawed_pos[1] - hub_rest_pos[1]) > 0.10,
        details=f"rest={hub_rest_pos}, yawed={hub_yawed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
