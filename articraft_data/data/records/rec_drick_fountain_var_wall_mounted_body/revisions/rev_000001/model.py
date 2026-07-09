from __future__ import annotations

# Wall-mounted "Drick" drinking-water fountain variant. A compact teal-blue
# painted-steel housing with filleted edges carries an open stainless-steel
# catch basin on top, fed by a short gooseneck spout. The front face is a
# brushed-steel plate with an engraved water-bottle pictogram. A chrome PUSH
# BUTTON valve actuator on the upper front dispenses water, and a perforated
# stainless bottle-rest grille shelf cantilevers from the lower front.
#
# A flat stainless mounting plate bolts to the wall at the rear; the compact
# body protrudes forward from the plate. The fountain hangs at typical ADA
# drinking-fountain height (~0.9 m to the basin rim).
#
# Coordinate convention: +Z up, fountain user faces +Y (front). The wall is at
# the -Y side; the mounting plate back face sits at y=0 (wall surface).
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
# Flat rear mounting plate (wall interface).
PLATE_X = 0.360  # width
PLATE_H = 0.440  # height
PLATE_T = 0.006  # thickness
PLATE_Z_CENTER = 0.900  # center height above floor
PLATE_HOLE_R = 0.005  # screw-hole radius
PLATE_HOLE_DX = 0.150  # hole x-offset from center
PLATE_HOLE_DZ = 0.190  # hole z-offset from center

WALL_T = 0.006  # body shell wall thickness

# Compact body housing (hollow painted-steel shell).
BODY_X = 0.280  # width
BODY_Y = 0.170  # depth (front-back, from plate front)
BODY_H = 0.320  # height
BODY_FILLET = 0.012  # edge fillet radius
BODY_BOT_Z = PLATE_Z_CENTER - PLATE_H / 2.0 + 0.050  # body bottom
BODY_TOP_Z = BODY_BOT_Z + BODY_H
BODY_FRONT_Y = PLATE_T + BODY_Y  # front face Y
BODY_CENTER_Y = PLATE_T + BODY_Y / 2.0  # body center Y
BODY_CENTER_Z = BODY_BOT_Z + BODY_H / 2.0

# Catch basin (open-top stainless box) sitting on top of the body.
BASIN_X = 0.220
BASIN_Y = 0.150
BASIN_H = 0.068
BASIN_WALL = 0.006
BASIN_Z = BODY_TOP_Z  # basin floor sits at the body top
BASIN_CY = PLATE_T + BODY_Y * 0.55  # basin center, slightly forward

# Front faceplate (brushed steel panel with engraved bottle pictogram).
FACE_X = 0.200
FACE_H = 0.240  # vertical extent of the steel face
FACE_T = 0.010  # plate thickness (in Y)
FACE_TOP_Z = BODY_TOP_Z - 0.020  # top of face strip, near body top
FACE_BOT_Z = FACE_TOP_Z - FACE_H
FACE_CENTER_Z = 0.5 * (FACE_TOP_Z + FACE_BOT_Z)
FACE_Y = BODY_FRONT_Y - 0.004  # plate back seats slightly into body front

# Push-button valve actuator (chrome plunger) on the upper front face.
BTN_R = 0.013
BTN_LEN = 0.022
BTN_Z = FACE_TOP_Z - 0.050
BTN_BOSS_R = 0.018
BTN_BOSS_LEN = 0.014
BTN_TRAVEL = 0.008

# Secondary fixed fitting next to the button.
KNOB_R = 0.010
KNOB_LEN = 0.020
KNOB_Z = BTN_Z
KNOB_DX = 0.040

# Gooseneck spout rising from the basin back wall and arching over the basin.
SPOUT_R = 0.008
SPOUT_RISE = 0.080

# Bottle-rest grille shelf on lower front.
GRILLE_R = 0.055
GRILLE_T = 0.010
GRILLE_Z = FACE_BOT_Z + 0.070
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

def _build_mounting_plate():
    """Flat stainless wall-mounting plate with four counterbored screw holes.
    Authored with back face at y=0, front face at y=PLATE_T, centered in X and
    at PLATE_Z_CENTER in Z."""
    plate = (
        cq.Workplane("XY")
        .box(PLATE_X, PLATE_T, PLATE_H, centered=(True, False, True))
    )
    # Translate to center at PLATE_Z_CENTER in Z (plate is centered at z=0).
    plate = plate.translate((0.0, 0.0, PLATE_Z_CENTER))

    # Four mounting screw holes near corners.
    for sx in (-1, 1):
        for sz in (-1, 1):
            hx = sx * PLATE_HOLE_DX
            hz = PLATE_Z_CENTER + sz * PLATE_HOLE_DZ
            hole = (
                cq.Workplane("XY")
                .workplane(offset=-0.005)
                .center(hx, hz)
                .circle(PLATE_HOLE_R)
                .extrude(PLATE_T + 0.01)
            )
            plate = plate.cut(hole)
    return plate


def _build_body():
    """Compact hollow housing: filleted rectangular shell with a closed bottom
    and open top (basin seats on it). Authored with back face at y=0, centered
    in X, bottom at z=0. Placed via the part visual origin."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, BODY_H, centered=(True, False, False))
        .edges("|Z")
        .fillet(BODY_FILLET)
    )
    # Hollow interior: inset on X and Y, floor at WALL_T, open at the top.
    # The inner box is made taller than the outer so the top cuts through
    # cleanly, leaving an open-top tray.
    inner = (
        cq.Workplane("XY")
        .workplane(offset=WALL_T)
        .box(
            BODY_X - 2.0 * WALL_T,
            BODY_Y - 2.0 * WALL_T,
            BODY_H + 0.01,  # overshoot to cleanly open the top
            centered=(True, True, False),
        )
    )
    body = outer.cut(inner)
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
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BASIN_WALL)
        .box(
            BASIN_X - 2.0 * BASIN_WALL,
            BASIN_Y - 2.0 * BASIN_WALL,
            BASIN_H,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.006)
    )
    basin = outer.cut(inner)
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
    forward and down over the basin. Authored in basin-local frame."""
    back_y = -BASIN_Y / 2.0 + BASIN_WALL + 0.006
    riser_base_z = BASIN_WALL
    riser_top_z = BASIN_H
    pts = []
    n_riser = 6
    for i in range(n_riser + 1):
        t = i / n_riser
        pts.append((0.0, back_y, riser_base_z + (riser_top_z - riser_base_z) * t))
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


def _build_faceplate():
    """Brushed-steel front face panel with a recessed engraved bottle pictogram.
    Authored in faceplate-local frame: back face at y=0, front face at y=FACE_T,
    centered in X and Z."""
    plate = (
        cq.Workplane("XY")
        .box(FACE_X, FACE_T, FACE_H, centered=(True, False, True))
        .edges("|Y")
        .fillet(0.006)
    )
    btn_local_z = BTN_Z - FACE_CENTER_Z
    knob_local_z = KNOB_Z - FACE_CENTER_Z

    def _front_cyl(cx, cz, r, length, start=FACE_T):
        return (
            cq.Workplane("XZ")
            .workplane(offset=-start)
            .center(cx, cz)
            .circle(r)
            .extrude(-length)
        )

    boss = _front_cyl(0.0, btn_local_z, BTN_BOSS_R, BTN_BOSS_LEN)
    plate = plate.union(boss)
    knob_boss = _front_cyl(KNOB_DX, knob_local_z, KNOB_R + 0.004, 0.008)
    plate = plate.union(knob_boss)

    # Engraved water-bottle pictogram.
    bz = -0.5 * FACE_H + 0.080
    recess = 0.0025
    body_w, body_h = 0.034, 0.055

    def _picto_part(cz, w, h):
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
    plate = plate.cut(picto)
    return plate, picto


def _build_button():
    """Chrome push-button plunger cap. Authored in button-local frame with its
    origin (y=0) at the boss outer face. The cap protrudes toward +Y (front);
    a thin stem extends back toward -Y so it stays captured inside the boss."""
    cap = (
        cq.Workplane("XZ")
        .circle(BTN_R)
        .extrude(-BTN_LEN)
        .edges(">Y")
        .fillet(0.003)
    )
    stem_len = BTN_BOSS_LEN + BTN_TRAVEL + 0.010
    stem = (
        cq.Workplane("XZ")
        .workplane(offset=-0.004)
        .circle(BTN_R - 0.005)
        .extrude(stem_len)
    )
    return cap.union(stem)


def _build_knob():
    """Fixed secondary valve/hose fitting next to the button."""
    fitting = (
        cq.Workplane("XZ")
        .circle(KNOB_R)
        .extrude(-KNOB_LEN)
    )
    cap = (
        cq.Workplane("XZ")
        .workplane(offset=-KNOB_LEN)
        .circle(KNOB_R + 0.003)
        .extrude(-0.006)
    )
    return fitting.union(cap)


def _build_grille():
    """Perforated stainless bottle-rest shelf: a half-round (D-shaped) tray with
    a raised lip and a grid of round drain holes. Authored centered in XY with
    its bottom at z=0."""
    shelf = (
        cq.Workplane("XY")
        .moveTo(-GRILLE_R, 0.0)
        .threePointArc((0.0, GRILLE_R), (GRILLE_R, 0.0))
        .lineTo(-GRILLE_R, 0.0)
        .close()
        .extrude(GRILLE_T)
    )
    lip = (
        cq.Workplane("XY")
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
    model = ArticulatedObject(name="drick_wall_fountain")

    blue = model.material("body_blue", rgba=BLUE)
    steel = model.material("stainless_steel", rgba=STEEL)
    chrome = model.material("chrome", rgba=CHROME)
    dark = model.material("engraving_dark", rgba=DARK)

    # --- Root: the flat wall mounting plate. --------------------------------
    plate = model.part("mounting_plate")
    plate.visual(mesh_from_cadquery(_build_mounting_plate(), "mounting_plate"), material=steel)
    plate.inertial = Inertial.from_geometry(
        Box((PLATE_X, PLATE_T, PLATE_H)),
        mass=3.0,
        origin=Origin(xyz=(0.0, PLATE_T / 2.0, PLATE_Z_CENTER)),
    )

    # --- Compact body housing (hollow painted shell, FIXED to plate). -------
    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body(), "body"), material=blue)
    body.inertial = Inertial.from_geometry(
        Box((BODY_X, BODY_Y, BODY_H)),
        mass=8.0,
        origin=Origin(xyz=(0.0, BODY_Y / 2.0, BODY_H / 2.0)),
    )
    # Body back face at plate front (y=PLATE_T), bottom at BODY_BOT_Z.
    model.articulation(
        "plate_to_body",
        ArticulationType.FIXED,
        parent=plate,
        child=body,
        origin=Origin(xyz=(0.0, PLATE_T, BODY_BOT_Z)),
    )

    # --- Catch basin (open-top stainless tray) on top of the body. ---------
    basin = model.part("catch_basin")
    basin.visual(mesh_from_cadquery(_build_basin(), "catch_basin"), material=steel)
    spout = _build_spout()
    basin.visual(mesh_from_geometry(spout, "spout"), material=steel)
    basin.inertial = Inertial.from_geometry(
        Box((BASIN_X, BASIN_Y, BASIN_H)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, BASIN_H / 2.0)),
    )
    model.articulation(
        "body_to_basin",
        ArticulationType.FIXED,
        parent=body,
        child=basin,
        origin=Origin(xyz=(0.0, BASIN_CY - PLATE_T, BODY_H)),
    )

    # --- Front faceplate (brushed steel panel + engraved bottle). -----------
    plate_geom, picto_geom = _build_faceplate()
    faceplate = model.part("front_faceplate")
    faceplate.visual(
        mesh_from_cadquery(plate_geom, "front_faceplate"),
        material=steel,
        name="plate_face",
    )
    faceplate.visual(
        mesh_from_cadquery(picto_geom, "bottle_pictogram"),
        material=dark,
        name="bottle_pictogram",
    )
    faceplate.inertial = Inertial.from_geometry(
        Box((FACE_X, FACE_T, FACE_H)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Faceplate back at body front face, centered vertically on the face strip.
    # In body-local frame: y = BODY_Y (front face), z = FACE_CENTER_Z - BODY_BOT_Z.
    model.articulation(
        "body_to_faceplate",
        ArticulationType.FIXED,
        parent=body,
        child=faceplate,
        origin=Origin(xyz=(0.0, BODY_Y, FACE_CENTER_Z - BODY_BOT_Z)),
    )

    # --- Secondary fixed fitting/knob next to the button. -------------------
    knob = model.part("valve_fitting")
    knob.visual(mesh_from_cadquery(_build_knob(), "valve_fitting"), material=chrome)
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_R, length=KNOB_LEN),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "faceplate_to_fitting",
        ArticulationType.FIXED,
        parent=faceplate,
        child=knob,
        origin=Origin(xyz=(KNOB_DX, FACE_T, KNOB_Z - FACE_CENTER_Z)),
    )

    # --- Bottle-rest grille shelf on the lower front. ----------------------
    grille = model.part("bottle_grille")
    grille.visual(mesh_from_cadquery(_build_grille(), "bottle_grille"), material=steel)
    grille.inertial = Inertial.from_geometry(
        Box((2 * GRILLE_R, GRILLE_R, GRILLE_T)),
        mass=0.4,
        origin=Origin(xyz=(0.0, GRILLE_R / 2.0, 0.0)),
    )
    model.articulation(
        "faceplate_to_grille",
        ArticulationType.FIXED,
        parent=faceplate,
        child=grille,
        origin=Origin(xyz=(0.0, FACE_T - 0.004, GRILLE_Z - FACE_CENTER_Z)),
    )

    # --- Push button plunger (PRISMATIC: press inward to dispense). ---------
    button = model.part("push_button")
    button.visual(mesh_from_cadquery(_build_button(), "push_button"), material=chrome)
    button.inertial = Inertial.from_geometry(
        Cylinder(radius=BTN_R, length=BTN_LEN),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "faceplate_to_button",
        ArticulationType.PRISMATIC,
        parent=faceplate,
        child=button,
        origin=Origin(xyz=(0.0, FACE_T + BTN_BOSS_LEN, BTN_Z - FACE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
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

    plate = object_model.get_part("mounting_plate")
    body = object_model.get_part("body")
    basin = object_model.get_part("catch_basin")
    faceplate = object_model.get_part("front_faceplate")
    grille = object_model.get_part("bottle_grille")
    button = object_model.get_part("push_button")
    fitting = object_model.get_part("valve_fitting")
    btn_joint = object_model.get_articulation("faceplate_to_button")

    # --- Mounting plate is flat and at the rear (-Y). ----------------------
    pl_lo, pl_hi = ctx.part_world_aabb(plate)
    pl_depth = pl_hi[1] - pl_lo[1]
    pl_width = pl_hi[0] - pl_lo[0]
    pl_height = pl_hi[2] - pl_lo[2]
    ctx.check(
        "mounting plate is thin (flat wall plate)",
        pl_depth < 0.020,
        details=f"depth={pl_depth:.4f}",
    )
    ctx.check(
        "mounting plate is wider than deep",
        pl_width > 10.0 * pl_depth,
        details=f"width={pl_width:.3f}, depth={pl_depth:.4f}",
    )
    ctx.check(
        "mounting plate is at wall height (~0.9 m)",
        0.6 < 0.5 * (pl_lo[2] + pl_hi[2]) < 1.2,
        details=f"center_z={0.5 * (pl_lo[2] + pl_hi[2]):.3f}",
    )
    ctx.check(
        "mounting plate back face at the wall (y~0)",
        abs(pl_lo[1]) < 0.010,
        details=f"back_y={pl_lo[1]:.4f}",
    )

    # --- Body is compact (not tall like a pylon), protrudes from wall. -----
    b_lo, b_hi = ctx.part_world_aabb(body)
    b_h = b_hi[2] - b_lo[2]
    b_w = b_hi[0] - b_lo[0]
    b_d = b_hi[1] - b_lo[1]
    ctx.check(
        "body is compact (about 0.3 m tall, not a tall pylon)",
        0.20 < b_h < 0.50,
        details=f"height={b_h:.3f}",
    )
    ctx.check(
        "body protrudes forward from the wall (+Y)",
        b_hi[1] > pl_hi[1] + 0.05,
        details=f"body_front={b_hi[1]:.3f}, plate_front={pl_hi[1]:.3f}",
    )
    ctx.check(
        "body is wider than deep (wall-mount proportions)",
        b_w > b_d,
        details=f"width={b_w:.3f}, depth={b_d:.3f}",
    )

    # --- Basin sits on top of the body. ------------------------------------
    bs_lo, bs_hi = ctx.part_world_aabb(basin)
    ctx.check(
        "basin sits at the top of the body",
        bs_lo[2] > b_hi[2] - 0.06,
        details=f"basin_min_z={bs_lo[2]:.3f}, body_top={b_hi[2]:.3f}",
    )
    ctx.check(
        "basin has real height (catch tray)",
        (bs_hi[2] - bs_lo[2]) > 0.04,
        details=f"basin_h={(bs_hi[2] - bs_lo[2]):.3f}",
    )

    # --- Faceplate is on the front of the body. ----------------------------
    f_lo, f_hi = ctx.part_world_aabb(faceplate)
    ctx.check(
        "faceplate is on the front of the body (+Y)",
        f_lo[1] > b_lo[1] + 0.05,
        details=f"face_back_y={f_lo[1]:.3f}, body_back_y={b_lo[1]:.3f}",
    )
    ctx.check(
        "faceplate is a vertical front panel",
        (f_hi[2] - f_lo[2]) > 0.15,
        details=f"face_h={(f_hi[2] - f_lo[2]):.3f}",
    )

    # --- Grille shelf cantilevers from the lower front. --------------------
    g_lo, g_hi = ctx.part_world_aabb(grille)
    ctx.check(
        "grille shelf is low on the front",
        0.5 * (g_lo[2] + g_hi[2]) < 0.5 * (f_lo[2] + f_hi[2]),
        details=f"grille_z={0.5 * (g_lo[2] + g_hi[2]):.3f}",
    )
    ctx.check(
        "grille extends forward of the faceplate (+Y)",
        g_hi[1] > f_hi[1] + 0.01,
        details=f"grille_front_y={g_hi[1]:.3f}, face_front_y={f_hi[1]:.3f}",
    )

    # --- Button is a chrome plunger on the upper front face. ---------------
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

    # --- Joint TYPE/AXIS: the button is a PRISMATIC inward plunger. --------
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

    # --- Pressing the button moves it inward (-Y), bounded travel. ---------
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
    lim = btn_joint.motion_limits
    ctx.check(
        "button travel is short and realistic",
        lim is not None
        and lim.lower == 0.0
        and lim.upper is not None
        and 0.003 < lim.upper < 0.020,
        details=f"travel={None if lim is None else lim.upper}",
    )

    # --- The button stem stays captured in the faceplate boss. -------------
    ctx.allow_overlap(
        faceplate,
        button,
        reason="The push-button stem is intentionally captured inside the faceplate mounting boss.",
    )
    ctx.expect_overlap(
        button,
        faceplate,
        axes="xz",
        min_overlap=0.010,
        name="button captured by the faceplate boss",
    )
    ctx.allow_overlap(
        button,
        body,
        reason="The push-button plunger stem runs back through the faceplate into the body housing.",
    )

    # --- Mounting seats. ---------------------------------------------------
    ctx.allow_overlap(
        faceplate,
        body,
        reason="The steel faceplate is flush-mounted onto the body front; its back face seats slightly into the body.",
    )
    ctx.expect_contact(
        faceplate,
        body,
        name="faceplate seated against the body front",
    )
    ctx.allow_overlap(
        grille,
        faceplate,
        reason="The grille shelf bracket is intentionally tabbed into the faceplate front for a cantilever mount.",
    )
    ctx.allow_overlap(
        grille,
        body,
        reason="The grille shelf bracket seats into the body front behind the faceplate.",
    )
    ctx.expect_contact(
        grille,
        faceplate,
        name="grille bracket seated on the faceplate",
    )
    ctx.allow_overlap(
        faceplate,
        fitting,
        reason="The secondary valve fitting base is intentionally seated into its faceplate mounting boss.",
    )
    ctx.allow_overlap(
        body,
        fitting,
        reason="The valve fitting base seats through the faceplate boss into the body front behind it.",
    )
    ctx.expect_contact(
        fitting,
        faceplate,
        name="valve fitting seated on the faceplate",
    )

    return ctx.report()


object_model = build_object_model()
