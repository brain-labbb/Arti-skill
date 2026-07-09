from __future__ import annotations

# Public outdoor "refill" drinking-water fountain, modeled from the reference
# image. A tall teal-blue painted-steel pylon with a curved (swooshed) side
# profile carries, at its top, an open stainless-steel rectangular catch basin
# fed by a short gooseneck spout. The front face is a brushed-steel plate with
# an engraved water-bottle pictogram. A chrome PUSH BUTTON valve actuator on the
# upper front dispenses water, and a perforated stainless bottle-rest grille
# shelf cantilevers from the lower front so a bottle can be set under the spout.
#
# Coordinate convention: +Z up, fountain stands on the ground at z=0. The user
# faces the +Y side (front). The pylon is tall in Z, slim in X, and curved in
# the Y-Z plane (the "swoosh" side profile visible from the side).
#
# Primary articulation: the chrome push button is a PRISMATIC plunger. Pressing
# it travels a few millimeters inward (-Y, into the faceplate) to open the valve.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------
# Pylon (curved blue standard). Slim in X, tall in Z, swooshed in Y over height.
PYLON_H = 1.020  # overall height of the pylon
PYLON_X = 0.150  # width (left-right)
PYLON_T = 0.090  # nominal front-back thickness of the body
WALL_T = 0.006  # sheet steel wall thickness (hollow body)

# Side-profile control points (front face Y over height). The body leans so the
# top front edge sits forward of the base, giving the curved "swoosh" look.
BASE_FRONT_Y = 0.060
BASE_BACK_Y = -0.030
TOP_FRONT_Y = 0.085
TOP_BACK_Y = -0.005

# Catch basin (open-top stainless box) sitting on top of the pylon.
BASIN_X = 0.180
BASIN_Y = 0.150
BASIN_H = 0.075
BASIN_WALL = 0.006
BASIN_Z = PYLON_H  # basin floor sits at the pylon top
# The basin overhangs the front of the pylon (cantilevered forward).
BASIN_CY = 0.085  # basin center Y (forward of pylon center)

# Front faceplate (brushed steel strip with engraved bottle pictogram).
FACE_X = 0.120
FACE_H = 0.430  # vertical extent of the steel face strip
FACE_T = 0.010  # plate thickness (in Y)
FACE_TOP_Z = BASIN_Z - 0.010  # top of the face strip, just under the basin
FACE_BOT_Z = FACE_TOP_Z - FACE_H
FACE_CENTER_Z = 0.5 * (FACE_TOP_Z + FACE_BOT_Z)  # world z of the plate center
FACE_Y = TOP_FRONT_Y - 0.006  # plate back seats against / slightly into pylon

# Push-button valve actuator (chrome plunger) on the upper front face.
BTN_R = 0.013  # button cap radius
BTN_LEN = 0.022  # button cap length (protrusion along Y)
BTN_Z = FACE_TOP_Z - 0.060  # button height on the face
BTN_BOSS_R = 0.018  # mounting boss radius on the faceplate
BTN_BOSS_LEN = 0.014  # boss protrusion
BTN_TRAVEL = 0.008  # plunger press travel

# Secondary fixed knob/fitting next to the button (a hose/valve fitting).
KNOB_R = 0.010
KNOB_LEN = 0.020
KNOB_Z = BTN_Z
KNOB_DX = 0.040  # offset in +X from the button

# Gooseneck spout rising from the basin back wall and arching over the basin.
SPOUT_R = 0.008
SPOUT_RISE = 0.085

# Upper bottle-filler arch: a taller gooseneck that rises well above the basin
# so a tall bottle can be filled underneath. Same back-wall mounting as the
# drinking spout but much taller and with a wider forward reach.
FILLER_R = 0.010  # slightly larger tube than the drinking spout
FILLER_RISE = 0.260  # peak height above the basin rim
FILLER_REACH = 0.70  # how far forward (as fraction of basin Y) the nozzle reaches

# Bottle-rest grille shelf (perforated stainless D-shaped tray) on lower front.
GRILLE_R = 0.060  # half-round shelf radius
GRILLE_T = 0.010  # shelf thickness
GRILLE_Z = FACE_BOT_Z + 0.090  # height of the shelf
GRILLE_HOLE_R = 0.006
GRILLE_PITCH = 0.018

# Materials
BLUE = (0.06, 0.45, 0.62, 1.0)  # teal blue painted steel
STEEL = (0.74, 0.76, 0.78, 1.0)  # brushed stainless
CHROME = (0.86, 0.88, 0.90, 1.0)  # polished chrome
DARK = (0.20, 0.22, 0.24, 1.0)  # engraved pictogram


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _unit(v):
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if length < 1e-9:
        return (0.0, 0.0, 1.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _combine_mesh_geometries(*geometries):
    vertices = []
    faces = []
    for geom in geometries:
        offset = len(vertices)
        vertices.extend(geom.vertices)
        faces.extend((a + offset, b + offset, c + offset) for a, b, c in geom.faces)
    return MeshGeometry(vertices=vertices, faces=faces)


def _hollow_tube_mesh_from_path(points, outer_radius, inner_radius=None, segments=24):
    """Thin-wall open tube mesh following a 3D path, with visible hollow ends."""
    inner_radius = inner_radius or outer_radius * 0.55
    vertices = []
    faces = []
    rings = []
    n = len(points)
    for i, p in enumerate(points):
        if i == 0:
            tangent = (points[1][0] - p[0], points[1][1] - p[1], points[1][2] - p[2])
        elif i == n - 1:
            tangent = (p[0] - points[i - 1][0], p[1] - points[i - 1][1], p[2] - points[i - 1][2])
        else:
            tangent = (
                points[i + 1][0] - points[i - 1][0],
                points[i + 1][1] - points[i - 1][1],
                points[i + 1][2] - points[i - 1][2],
            )
        tangent = _unit(tangent)
        u = (1.0, 0.0, 0.0)
        if abs(tangent[0]) > 0.92:
            u = (0.0, 1.0, 0.0)
        v = _unit(_cross(tangent, u))
        u = _unit(_cross(v, tangent))

        outer = []
        inner = []
        for k in range(segments):
            a = 2.0 * math.pi * k / segments
            ca = math.cos(a)
            sa = math.sin(a)
            outer.append(len(vertices))
            vertices.append((
                p[0] + outer_radius * (u[0] * ca + v[0] * sa),
                p[1] + outer_radius * (u[1] * ca + v[1] * sa),
                p[2] + outer_radius * (u[2] * ca + v[2] * sa),
            ))
            inner.append(len(vertices))
            vertices.append((
                p[0] + inner_radius * (u[0] * ca + v[0] * sa),
                p[1] + inner_radius * (u[1] * ca + v[1] * sa),
                p[2] + inner_radius * (u[2] * ca + v[2] * sa),
            ))
        rings.append((outer, inner))

    for i in range(n - 1):
        outer0, inner0 = rings[i]
        outer1, inner1 = rings[i + 1]
        for k in range(segments):
            j = (k + 1) % segments
            faces.append((outer0[k], outer1[k], outer1[j]))
            faces.append((outer0[k], outer1[j], outer0[j]))
            faces.append((inner0[k], inner1[j], inner1[k]))
            faces.append((inner0[k], inner0[j], inner1[j]))

    for ring_index in (0, n - 1):
        outer, inner = rings[ring_index]
        for k in range(segments):
            j = (k + 1) % segments
            if ring_index == 0:
                faces.append((outer[k], inner[j], inner[k]))
                faces.append((outer[k], outer[j], inner[j]))
            else:
                faces.append((outer[k], inner[k], inner[j]))
                faces.append((outer[k], inner[j], outer[j]))
    return MeshGeometry(vertices=vertices, faces=faces)


def _annular_cylinder_mesh(center, outer_radius, inner_radius, height, segments=32):
    cx, cy, z0 = center
    z1 = z0 + height
    vertices = []
    for z in (z0, z1):
        for r in (outer_radius, inner_radius):
            for k in range(segments):
                a = 2.0 * math.pi * k / segments
                vertices.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
    faces = []
    bo = 0
    bi = segments
    to = 2 * segments
    ti = 3 * segments
    for k in range(segments):
        j = (k + 1) % segments
        faces.append((bo + k, bo + j, to + j))
        faces.append((bo + k, to + j, to + k))
        faces.append((bi + k, ti + j, bi + j))
        faces.append((bi + k, ti + k, ti + j))
        faces.append((to + k, to + j, ti + j))
        faces.append((to + k, ti + j, ti + k))
        faces.append((bo + k, bi + j, bo + j))
        faces.append((bo + k, bi + k, bi + j))
    return MeshGeometry(vertices=vertices, faces=faces)

def _loft_levels(levels, width):
    """Loft rectangular sections (centered at per-level Y) up the height.

    levels: ordered list of (z, back_y, front_y). At each level we place a
    rectangle of the given X width and Y depth, centered at that level's Y, then
    loft through all of them. Uses one chained Workplane so the section wires
    accumulate as pending wires (the reliable CadQuery loft pattern).
    """
    z0 = levels[0][0]
    wp = cq.Workplane("XY").workplane(offset=z0)
    first = True
    for z, by, fy in levels:
        cy = 0.5 * (by + fy)
        depth = max(fy - by, 0.004)
        if first:
            wp = wp.center(0.0, cy).rect(width, depth)
            prev_cy = cy
            first = False
        else:
            # Move to the next section plane and re-center to this level's Y.
            wp = (
                wp.workplane(offset=(z - z0))
                .center(0.0, cy - prev_cy)
                .rect(width, depth)
            )
            prev_cy = cy
        z0 = z
    return wp.loft(combine=True)


def _side_profile(front_pts, back_pts):
    """Build the curved side-profile of the pylon as a lofted hollow shell.

    front_pts / back_pts are ordered (y, z) control points up the height. We
    loft rectangular cross-sections (full width in X) between successive
    levels so the body curves in the Y-Z plane while staying slim in X.
    """
    levels = []
    for (fy, fz), (by, _bz) in zip(front_pts, back_pts):
        levels.append((fz, by, fy))  # (z, back_y, front_y)

    solid = _loft_levels(levels, PYLON_X)

    # Hollow it: subtract an inner loft inset by the wall thickness. Keep the
    # bottom and top closed enough to read as a sheet-steel standard.
    inner_levels = []
    for z, by, fy in levels:
        cy = 0.5 * (by + fy)
        depth = max(fy - by - 2.0 * WALL_T, 0.004)
        inner_levels.append((z, cy - depth / 2.0, cy + depth / 2.0))
    inner_solid = _loft_levels(inner_levels, PYLON_X - 2.0 * WALL_T)
    body = solid.cut(inner_solid)
    return body


def _build_pylon():
    """Curved teal pylon (hollow sheet-steel standard)."""
    n = 11
    front_pts = []
    back_pts = []
    for i in range(n):
        t = i / (n - 1)
        z = t * PYLON_H
        # Smooth swoosh: ease the front edge forward toward the top.
        e = 0.5 - 0.5 * math.cos(math.pi * t)
        fy = BASE_FRONT_Y + (TOP_FRONT_Y - BASE_FRONT_Y) * e
        by = BASE_BACK_Y + (TOP_BACK_Y - BASE_BACK_Y) * t
        front_pts.append((fy, z))
        back_pts.append((by, z))
    body = _side_profile(front_pts, back_pts)

    # Solid foot pad at the bottom so it reads as a planted standard and so the
    # base is closed (not an open tube on the ground).
    foot = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .center(0.0, 0.5 * (BASE_BACK_Y + BASE_FRONT_Y))
        .rect(PYLON_X, BASE_FRONT_Y - BASE_BACK_Y)
        .extrude(0.018)
    )
    body = body.union(foot)
    return body


def _build_basin():
    """Open-top stainless catch basin: hollow rectangular tray with a thin rim
    and a small drain hole in the floor. Authored centered at origin in XY with
    its floor at z=0; placed via the part visual origin."""
    outer = (
        cq.Workplane("XY")
        .box(BASIN_X, BASIN_Y, BASIN_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
    )
    # Inner cavity (open top): inset walls, floor thickness at the bottom.
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BASIN_WALL)
        .box(
            BASIN_X - 2.0 * BASIN_WALL,
            BASIN_Y - 2.0 * BASIN_WALL,
            BASIN_H,  # taller than the box so the top stays open
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.006)
    )
    basin = outer.cut(inner)
    # Drain hole in the floor.
    drain = (
        cq.Workplane("XY")
        .workplane(offset=-0.005)
        .circle(0.012)
        .extrude(BASIN_WALL + 0.01)
    )
    basin = basin.cut(drain)
    return basin


def _build_spout():
    """Short gooseneck spout: vertical riser from the basin back wall arching
    forward and down over the basin. Authored in basin-local frame (origin at
    basin floor center). Built as a chain of short oriented cylinders so the
    gooseneck is robust without a swept profile."""
    back_y = -BASIN_Y / 2.0 + BASIN_WALL + 0.006
    riser_base_z = BASIN_WALL  # start at the basin floor so it meets the flange
    riser_top_z = BASIN_H  # vertical riser height before the arch begins
    pts = []
    # Vertical riser from the basin floor up the back wall.
    n_riser = 6
    for i in range(n_riser + 1):
        t = i / n_riser
        pts.append((0.0, back_y, riser_base_z + (riser_top_z - riser_base_z) * t))
    # Arch forward over the basin, tipping gently down at the end.
    n = 22
    for i in range(1, n + 1):
        t = i / n
        z = riser_top_z + SPOUT_RISE * math.sin(math.pi * 0.5 * min(t * 1.25, 1.0))
        y = back_y + (0.55 * BASIN_Y) * (0.5 - 0.5 * math.cos(math.pi * t))
        pts.append((0.0, y, z))


    return _combine_mesh_geometries(
        _hollow_tube_mesh_from_path(pts, SPOUT_R),
        _annular_cylinder_mesh(
            (0.0, back_y, BASIN_WALL - 0.002),
            SPOUT_R + 0.006,
            max(SPOUT_R * 0.55, 0.0025),
            0.014,
        ),
    )


def _build_bottle_filler_arch():
    """Upper bottle-filler spout arch: a tall gooseneck that rises well above
    the basin rim so a tall bottle can be filled underneath. Authored in the
    same basin-local frame as the drinking spout (origin at basin floor center).
    Uses the same chain-of-cylinders pattern as the drinking gooseneck."""
    back_y = -BASIN_Y / 2.0 + BASIN_WALL + 0.006
    riser_base_z = BASIN_WALL
    # The vertical riser goes up above the basin rim before the arch begins.
    riser_top_z = BASIN_H + 0.040  # riser extends 40 mm above the basin rim

    pts = []
    # Vertical riser from basin floor up past the basin rim.
    n_riser = 8
    for i in range(n_riser + 1):
        t = i / n_riser
        pts.append((0.0, back_y, riser_base_z + (riser_top_z - riser_base_z) * t))

    # Arch forward and upward over the basin, then gently down at the nozzle.
    n_arch = 28
    peak_z = BASIN_H + FILLER_RISE  # peak height in basin-local z
    for i in range(1, n_arch + 1):
        t = i / n_arch
        # Smooth sine rise to peak, then gentle drop at the end.
        rise = FILLER_RISE * math.sin(math.pi * 0.5 * min(t * 1.15, 1.0))
        # Small downward tip at the nozzle end.
        tip_drop = 0.025 * max(t - 0.85, 0.0) / 0.15 if t > 0.85 else 0.0
        z = riser_top_z + rise - tip_drop
        # Forward reach: from back wall to near the front of the basin.
        y = back_y + (FILLER_REACH * BASIN_Y) * (0.5 - 0.5 * math.cos(math.pi * t))
        pts.append((0.0, y, z))


    tip_pt = pts[-1]
    nozzle_len = 0.018
    return _combine_mesh_geometries(
        _hollow_tube_mesh_from_path(pts, FILLER_R),
        _hollow_tube_mesh_from_path(
            [tip_pt, (tip_pt[0], tip_pt[1], tip_pt[2] - nozzle_len)],
            FILLER_R - 0.002,
            max((FILLER_R - 0.002) * 0.55, 0.0025),
            segments=24,
        ),
        _annular_cylinder_mesh(
            (0.0, back_y, BASIN_WALL - 0.002),
            FILLER_R + 0.008,
            max(FILLER_R * 0.55, 0.003),
            0.016,
        ),
    )


def _build_faceplate():
    """Brushed-steel front face strip with a recessed engraved bottle
    pictogram. Authored in faceplate-local frame: the plate lies in the X-Z
    plane, thickness along Y. The plate spans local y in [0, FACE_T] (back face
    at y=0, front face at y=FACE_T = +Y), and is centered in X and Z. It is
    placed via the part visual origin.

    Note on CadQuery: `Workplane("XZ")` extrudes toward -Y, so to build features
    that protrude toward +Y (the front), we offset by a negative amount and
    extrude with a negative length.
    """
    plate = (
        cq.Workplane("XY")
        .box(FACE_X, FACE_T, FACE_H, centered=(True, False, True))
        .edges("|Y")
        .fillet(0.006)
    )
    # Local z of a feature = world height - plate center height.
    btn_local_z = BTN_Z - FACE_CENTER_Z
    knob_local_z = KNOB_Z - FACE_CENTER_Z

    def _front_cyl(cx, cz, r, length, start=FACE_T):
        # Cylinder protruding toward +Y from the plate front face (y=start).
        return (
            cq.Workplane("XZ")
            .workplane(offset=-start)
            .center(cx, cz)
            .circle(r)
            .extrude(-length)
        )

    # Button mounting boss on the front face (+Y), raised cylinder.
    boss = _front_cyl(0.0, btn_local_z, BTN_BOSS_R, BTN_BOSS_LEN)
    plate = plate.union(boss)
    # Secondary fitting boss next to the button.
    knob_boss = _front_cyl(KNOB_DX, knob_local_z, KNOB_R + 0.004, 0.008)
    plate = plate.union(knob_boss)

    # Engraved water-bottle pictogram recessed into the lower-middle of the
    # front face. Build a simple bottle silhouette (body + neck + cap) sitting
    # just inside the front plate face, then cut a shallow recess and return the
    # silhouette as a separate dark-filled visual that fills that recess.
    bz = -0.5 * FACE_H + 0.130  # pictogram center height (plate-local z)
    recess = 0.0025  # engraving depth
    body_w, body_h = 0.034, 0.060

    def _picto_part(cz, w, h):
        # A thin slab spanning local y in [FACE_T - recess, FACE_T + 0.0005]
        # (i.e. just inside the front face, poking out a hair so the cut is
        # clean). Built front-facing via the negative-extrude convention.
        return (
            cq.Workplane("XZ")
            .workplane(offset=-(FACE_T - recess))
            .center(0.0, cz)
            .rect(w, h)
            .extrude(-(recess + 0.0005))
        )

    bottle = _picto_part(bz, body_w, body_h)
    neck = _picto_part(bz + body_h / 2.0 + 0.010, 0.014, 0.022)
    cap = _picto_part(bz + body_h / 2.0 + 0.026, 0.018, 0.008)
    picto = bottle.union(neck).union(cap)
    # Recess it into the plate (cut a shallow pocket on the front face).
    plate = plate.cut(picto)
    return plate, picto


def _build_button():
    """Chrome push-button plunger cap. Authored in button-local frame with its
    origin (y=0) at the boss outer face. The cap protrudes toward +Y (front);
    a thin stem extends back toward -Y so it stays captured inside the boss.
    (XZ workplane extrudes -Y, so the cap uses a negative extrude to face +Y.)"""
    cap = (
        cq.Workplane("XZ")
        .circle(BTN_R)
        .extrude(-BTN_LEN)  # protrude toward +Y (front)
        .edges(">Y")
        .fillet(0.003)
    )
    # A short stem behind the cap (toward -Y) that stays captured in the boss.
    # Start it a little inside the cap (+Y of y=0) so the two solids fuse.
    stem_len = BTN_BOSS_LEN + BTN_TRAVEL + 0.010
    stem = (
        cq.Workplane("XZ")
        .workplane(offset=-0.004)  # start at y=+0.004, inside the cap
        .circle(BTN_R - 0.005)
        .extrude(stem_len)  # extends toward -Y (into the boss)
    )
    return cap.union(stem)


def _build_knob():
    """Fixed secondary valve/hose fitting next to the button (chrome cylinder
    with a small cap). Authored in knob-local frame with its base at y=0 and the
    body protruding toward +Y (front)."""
    fitting = (
        cq.Workplane("XZ")
        .circle(KNOB_R)
        .extrude(-KNOB_LEN)  # toward +Y (front)
    )
    cap = (
        cq.Workplane("XZ")
        .workplane(offset=-KNOB_LEN)
        .circle(KNOB_R + 0.003)
        .extrude(-0.006)  # cap on the front tip
    )
    return fitting.union(cap)


def _build_grille():
    """Perforated stainless bottle-rest shelf: a half-round (D-shaped) tray with
    a raised lip and a grid of round drain holes. Authored centered in XY with
    its bottom at z=0; placed via the part visual origin. The flat (chord) side
    faces back toward the faceplate; the round side faces forward (+Y)."""
    # D-shaped plate: a half disc (front half) -> chord along X at y=0.
    shelf = (
        cq.Workplane("XY")
        .moveTo(-GRILLE_R, 0.0)
        .threePointArc((0.0, GRILLE_R), (GRILLE_R, 0.0))
        .lineTo(-GRILLE_R, 0.0)
        .close()
        .extrude(GRILLE_T)
    )
    # Raised perimeter lip on the round side.
    lip = (
        cq.Workplane("XY")
        .workplane(offset=0.0)
        .moveTo(-GRILLE_R, 0.0)
        .threePointArc((0.0, GRILLE_R), (GRILLE_R, 0.0))
        .lineTo(-GRILLE_R, 0.0)
        .close()
        .extrude(GRILLE_T + 0.010)
    )
    lip_inner = (
        cq.Workplane("XY")
        .moveTo(-(GRILLE_R - 0.006), 0.0)
        .threePointArc((0.0, GRILLE_R - 0.006), (GRILLE_R - 0.006, 0.0))
        .lineTo(-(GRILLE_R - 0.006), 0.0)
        .close()
        .extrude(GRILLE_T + 0.012)
    )
    lip = lip.cut(lip_inner)
    shelf = shelf.union(lip)

    # Drain holes in a grid over the half-disc.
    holes = None
    steps = int((2 * GRILLE_R) / GRILLE_PITCH) + 1
    start = -GRILLE_R + 0.012
    for ix in range(steps):
        x = start + ix * GRILLE_PITCH
        for iy in range(steps):
            y = 0.012 + iy * GRILLE_PITCH
            if x * x + y * y < (GRILLE_R - 0.010) ** 2 and y < GRILLE_R - 0.006:
                hole = (
                    cq.Workplane("XY")
                    .workplane(offset=-0.005)
                    .center(x, y)
                    .circle(GRILLE_HOLE_R)
                    .extrude(GRILLE_T + 0.01)
                )
                holes = hole if holes is None else holes.union(hole)
    if holes is not None:
        shelf = shelf.cut(holes)

    # A short bracket arm on the chord side that mounts to the faceplate.
    bracket = (
        cq.Workplane("XY")
        .center(0.0, -0.012)
        .box(0.060, 0.024, GRILLE_T, centered=(True, True, False))
    )
    shelf = shelf.union(bracket)
    return shelf


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refill_water_fountain")

    blue = model.material("pylon_blue", rgba=BLUE)
    steel = model.material("stainless_steel", rgba=STEEL)
    chrome = model.material("chrome", rgba=CHROME)
    dark = model.material("engraving_dark", rgba=DARK)

    # --- Root: the curved blue pylon standard. ------------------------------
    pylon = model.part("pylon_body")
    pylon.visual(mesh_from_cadquery(_build_pylon(), "pylon_body"), material=blue)
    pylon.inertial = Inertial.from_geometry(
        Box((PYLON_X, PYLON_T + 0.06, PYLON_H)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.03, PYLON_H / 2.0)),
    )

    # --- Catch basin (open-top stainless tray) on top of the pylon. ---------
    basin = model.part("catch_basin")
    basin.visual(mesh_from_cadquery(_build_basin(), "catch_basin"), material=steel)
    # Spout rises from the basin back wall (fixed to the basin).
    spout = _build_spout()
    basin.visual(mesh_from_geometry(spout, "spout"), material=steel)
    basin.inertial = Inertial.from_geometry(
        Box((BASIN_X, BASIN_Y, BASIN_H)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, BASIN_H / 2.0)),
    )
    model.articulation(
        "pylon_to_basin",
        ArticulationType.FIXED,
        parent=pylon,
        child=basin,
        origin=Origin(xyz=(0.0, BASIN_CY, BASIN_Z)),
    )

    # --- Upper bottle-filler arch (tall gooseneck above basin). ------------
    filler = model.part("bottle_filler_arch")
    filler.visual(
        mesh_from_geometry(_build_bottle_filler_arch(), "bottle_filler_arch"),
        material=steel,
    )
    filler.inertial = Inertial.from_geometry(
        Cylinder(radius=FILLER_R, length=FILLER_RISE + BASIN_H),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, (FILLER_RISE + BASIN_H) / 2.0)),
    )
    # Fixed to the basin at the same back-wall mounting point as the drinking spout.
    model.articulation(
        "basin_to_filler",
        ArticulationType.FIXED,
        parent=basin,
        child=filler,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # same frame as basin (basin-local)
    )

    # --- Front faceplate (brushed steel strip + engraved bottle). -----------
    plate_geom, picto_geom = _build_faceplate()
    faceplate = model.part("front_faceplate")
    faceplate.visual(
        mesh_from_cadquery(plate_geom, "front_faceplate"),
        material=steel,
        name="plate_face",
    )
    # The engraved bottle pictogram as a separate dark-filled visual sitting in
    # the recess, so it reads as a contrasting engraving.
    faceplate.visual(
        mesh_from_cadquery(picto_geom, "bottle_pictogram"),
        material=dark,
        name="bottle_pictogram",
    )
    faceplate.inertial = Inertial.from_geometry(
        Box((FACE_X, FACE_T, FACE_H)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Faceplate frame: origin at the front face plane, centered vertically on the
    # face strip. Plate-local z=0 is the face strip center.
    face_center_z = 0.5 * (FACE_TOP_Z + FACE_BOT_Z)
    model.articulation(
        "pylon_to_faceplate",
        ArticulationType.FIXED,
        parent=pylon,
        child=faceplate,
        origin=Origin(xyz=(0.0, FACE_Y, face_center_z)),
    )

    # --- Secondary fixed fitting/knob next to the button. -------------------
    knob = model.part("valve_fitting")
    knob.visual(mesh_from_cadquery(_build_knob(), "valve_fitting"), material=chrome)
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_R, length=KNOB_LEN),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Mounted on the faceplate front, at the knob boss.
    knob_y = FACE_Y + FACE_T  # front face of the plate in world Y
    model.articulation(
        "faceplate_to_fitting",
        ArticulationType.FIXED,
        parent=faceplate,
        child=knob,
        origin=Origin(xyz=(KNOB_DX, FACE_T, KNOB_Z - face_center_z)),
    )

    # --- Bottle-rest grille shelf (perforated D-tray) on the lower front. ---
    grille = model.part("bottle_grille")
    grille.visual(mesh_from_cadquery(_build_grille(), "bottle_grille"), material=steel)
    grille.inertial = Inertial.from_geometry(
        Box((2 * GRILLE_R, GRILLE_R, GRILLE_T)),
        mass=0.4,
        origin=Origin(xyz=(0.0, GRILLE_R / 2.0, 0.0)),
    )
    # Mounted to the faceplate front, cantilevered forward at shelf height.
    model.articulation(
        "faceplate_to_grille",
        ArticulationType.FIXED,
        parent=faceplate,
        child=grille,
        origin=Origin(xyz=(0.0, FACE_T - 0.004, GRILLE_Z - face_center_z)),
    )

    # --- Push button plunger (PRISMATIC: press inward to dispense). ---------
    button = model.part("push_button")
    button.visual(mesh_from_cadquery(_build_button(), "push_button"), material=chrome)
    button.inertial = Inertial.from_geometry(
        Cylinder(radius=BTN_R, length=BTN_LEN),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Button frame at the outer end of the mounting boss on the faceplate front.
    # The button cap protrudes along +Y; pressing travels along -Y. q=0 is the
    # rest (out) pose; positive q presses the button inward.
    btn_front_y = FACE_Y + FACE_T + BTN_BOSS_LEN  # boss outer face in world Y
    model.articulation(
        "faceplate_to_button",
        ArticulationType.PRISMATIC,
        parent=faceplate,
        child=button,
        origin=Origin(xyz=(0.0, FACE_T + BTN_BOSS_LEN, BTN_Z - face_center_z)),
        axis=(0.0, -1.0, 0.0),  # +q presses the button inward (-Y)
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.1, lower=0.0, upper=BTN_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pylon = object_model.get_part("pylon_body")
    basin = object_model.get_part("catch_basin")
    faceplate = object_model.get_part("front_faceplate")
    grille = object_model.get_part("bottle_grille")
    button = object_model.get_part("push_button")
    fitting = object_model.get_part("valve_fitting")
    btn_joint = object_model.get_articulation("faceplate_to_button")

    # --- Overall proportions: tall, slim pylon standing on the ground. ------
    p_lo, p_hi = ctx.part_world_aabb(pylon)
    p_h = p_hi[2] - p_lo[2]
    p_w = p_hi[0] - p_lo[0]
    ctx.check(
        "pylon stands on the ground at z~0",
        abs(p_lo[2]) < 0.02,
        details=f"min_z={p_lo[2]:.4f}",
    )
    ctx.check(
        "pylon is tall (about 1 m)",
        0.95 < p_h < 1.10,
        details=f"height={p_h:.3f}",
    )
    ctx.check(
        "pylon is slim (much taller than wide)",
        p_h > 5.0 * p_w,
        details=f"h={p_h:.3f}, w={p_w:.3f}",
    )

    # --- Basin sits on top of the pylon and is open/hollow. -----------------
    b_lo, b_hi = ctx.part_world_aabb(basin)
    ctx.check(
        "basin sits at the top of the pylon",
        b_lo[2] > p_hi[2] - 0.06,
        details=f"basin_min_z={b_lo[2]:.3f}, pylon_top={p_hi[2]:.3f}",
    )
    # Open-top tray: floor near the bottom, top rim well above (a hollow tray).
    ctx.check(
        "basin has real height (catch tray)",
        (b_hi[2] - b_lo[2]) > 0.05,
        details=f"basin_h={(b_hi[2] - b_lo[2]):.3f}",
    )
    # The basin overhangs forward of the pylon centerline (cantilevered front).
    basin_cy = 0.5 * (b_lo[1] + b_hi[1])
    ctx.check(
        "basin overhangs toward the front (+Y)",
        basin_cy > 0.03,
        details=f"basin_center_y={basin_cy:.3f}",
    )

    # --- Faceplate is a vertical front strip just proud of the pylon. -------
    f_lo, f_hi = ctx.part_world_aabb(faceplate)
    ctx.check(
        "faceplate is a tall front strip",
        (f_hi[2] - f_lo[2]) > 0.30,
        details=f"face_h={(f_hi[2] - f_lo[2]):.3f}",
    )
    ctx.check(
        "faceplate sits on the front of the pylon (+Y)",
        0.5 * (f_lo[1] + f_hi[1]) > 0.04,
        details=f"face_center_y={0.5 * (f_lo[1] + f_hi[1]):.3f}",
    )

    # --- Bottle-rest grille shelf cantilevers from the lower front. ---------
    g_lo, g_hi = ctx.part_world_aabb(grille)
    ctx.check(
        "grille shelf is low on the front",
        0.5 * (g_lo[2] + g_hi[2]) < 0.5 * (f_lo[2] + f_hi[2]),
        details=f"grille_z={0.5 * (g_lo[2] + g_hi[2]):.3f}",
    )
    ctx.check(
        "grille extends forward of the faceplate (+Y)",
        g_hi[1] > f_hi[1] + 0.02,
        details=f"grille_front_y={g_hi[1]:.3f}, face_front_y={f_hi[1]:.3f}",
    )

    # --- Bottle-filler arch rises above the basin. -------------------------
    filler = object_model.get_part("bottle_filler_arch")
    filler_joint = object_model.get_articulation("basin_to_filler")
    fl_lo, fl_hi = ctx.part_world_aabb(filler)
    ctx.check(
        "bottle filler arch exists above the basin",
        fl_hi[2] > b_hi[2] + 0.15,
        details=f"filler_top_z={fl_hi[2]:.3f}, basin_top_z={b_hi[2]:.3f}",
    )
    ctx.check(
        "bottle filler arch peaks well above the drinking spout",
        fl_hi[2] > b_hi[2] + FILLER_RISE * 0.7,
        details=f"filler_top_z={fl_hi[2]:.3f}, basin_top_z={b_hi[2]:.3f}",
    )
    ctx.check(
        "bottle filler arch extends forward over the basin (+Y)",
        fl_hi[1] > b_lo[1] + 0.04,
        details=f"filler_front_y={fl_hi[1]:.3f}, basin_back_y={b_lo[1]:.3f}",
    )
    ctx.check(
        "bottle filler arch joint is FIXED to basin",
        str(filler_joint.joint_type).lower().endswith("fixed"),
        details=f"type={filler_joint.joint_type}",
    )
    # The filler base/flange seats into the basin floor (intentional embed).
    ctx.allow_overlap(
        basin,
        filler,
        reason="The bottle-filler arch mounting flange is intentionally seated into the basin floor/back wall.",
    )
    ctx.expect_contact(
        filler,
        basin,
        name="bottle filler arch mounted on basin",
    )

    # --- Button is a chrome plunger on the upper front face, near the fitting.
    bt_lo, bt_hi = ctx.part_world_aabb(button)
    btn_cz = 0.5 * (bt_lo[2] + bt_hi[2])
    ctx.check(
        "button is high on the front face (near the spout)",
        btn_cz > 0.5 * (f_lo[2] + f_hi[2]),
        details=f"button_z={btn_cz:.3f}",
    )
    fit_pos = ctx.part_world_position(fitting)
    btn_pos = ctx.part_world_position(button)
    ctx.check(
        "valve fitting sits beside the button",
        fit_pos is not None
        and btn_pos is not None
        and 0.02 < abs(fit_pos[0] - btn_pos[0]) < 0.08,
        details=f"btn_x={btn_pos[0]:.3f}, fit_x={fit_pos[0]:.3f}",
    )

    # --- Joint TYPE/AXIS: the button is a PRISMATIC inward plunger. ---------
    ctx.check(
        "button joint is prismatic",
        str(btn_joint.joint_type).lower().endswith("prismatic"),
        details=f"type={btn_joint.joint_type}",
    )
    ctx.check(
        "button press axis is along Y (into the face)",
        abs(btn_joint.axis[1]) > 0.99 and abs(btn_joint.axis[0]) < 0.01,
        details=f"axis={btn_joint.axis}",
    )

    # --- Pressing the button moves it inward (-Y), bounded travel. ----------
    rest = ctx.part_world_position(button)
    with ctx.pose({btn_joint: BTN_TRAVEL}):
        pressed = ctx.part_world_position(button)
    ctx.check(
        "pressing the button moves it inward (-Y)",
        rest is not None
        and pressed is not None
        and pressed[1] < rest[1] - 0.004,
        details=f"rest_y={rest[1]:.4f}, pressed_y={pressed[1]:.4f}",
    )
    # Travel is short/realistic (a few mm, not a long slide).
    lim = btn_joint.motion_limits
    ctx.check(
        "button travel is short and realistic",
        lim is not None
        and lim.lower == 0.0
        and lim.upper is not None
        and 0.003 < lim.upper < 0.020,
        details=f"travel={None if lim is None else lim.upper}",
    )

    # --- The button stem stays captured in the faceplate boss. --------------
    # The plunger stem nests into the mounting boss; allow that local overlap.
    ctx.allow_overlap(
        faceplate,
        button,
        reason="The push-button stem is intentionally captured inside the faceplate mounting boss.",
    )
    # Retained engagement: button overlaps the faceplate footprint in X/Z so it
    # is supported by the boss, not floating.
    ctx.expect_overlap(
        button,
        faceplate,
        axes="xz",
        min_overlap=0.010,
        name="button captured by the faceplate boss",
    )
    # The plunger stem runs back through the boss and the faceplate into the
    # pylon front (the valve body sits inside the pylon). Intentional capture.
    ctx.allow_overlap(
        button,
        pylon,
        reason="The push-button plunger stem runs back through the faceplate into the pylon, where the valve body sits.",
    )

    # --- Mounting seats (intentional flush-mount / seated embeds). -----------
    # The faceplate is a sheet-steel panel flush-mounted onto the pylon front;
    # its back edge seats a few mm into the pylon face.
    ctx.allow_overlap(
        faceplate,
        pylon,
        reason="The steel faceplate is flush-mounted onto the pylon front; its back face seats slightly into the pylon.",
    )
    ctx.expect_contact(
        faceplate,
        pylon,
        name="faceplate seated against the pylon front",
    )
    # The bottle-rest grille bracket tabs into the faceplate/pylon front to
    # cantilever the shelf forward.
    ctx.allow_overlap(
        grille,
        faceplate,
        reason="The grille shelf bracket is intentionally tabbed into the faceplate front for a cantilever mount.",
    )
    ctx.allow_overlap(
        grille,
        pylon,
        reason="The grille shelf bracket seats into the pylon front behind the faceplate.",
    )
    ctx.expect_contact(
        grille,
        faceplate,
        name="grille bracket seated on the faceplate",
    )
    # The secondary valve fitting seats into its boss on the faceplate front.
    ctx.allow_overlap(
        faceplate,
        fitting,
        reason="The secondary valve fitting base is intentionally seated into its faceplate mounting boss.",
    )
    ctx.allow_overlap(
        pylon,
        fitting,
        reason="The valve fitting base seats through the faceplate boss into the pylon front behind it.",
    )
    ctx.expect_contact(
        fitting,
        faceplate,
        name="valve fitting seated on the faceplate",
    )

    return ctx.report()


object_model = build_object_model()
