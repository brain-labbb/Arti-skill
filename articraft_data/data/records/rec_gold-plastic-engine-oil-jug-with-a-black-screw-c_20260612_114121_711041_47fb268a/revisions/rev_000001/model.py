from __future__ import annotations

# Gold/tan plastic 4L engine oil jug ("DAYTONA 5W-40").
# Frame: Z is up (jug taller than wide), body footprint in XY.
#   - x: width  (~0.135 m). Cap/neck sit on the -X (top-left) corner.
#   - y: depth  (~0.100 m). Purple label faces +Y (front).
#   - z: height (~0.270 m). Bottle base at z=0, top deck near z=0.255.
# The grip is an INTEGRATED D-handle: the upper-+X shoulder is a solid slab
# flush within the jug silhouette, with an oval finger hole BOOLEAN-CUT through
# the full Y thickness so daylight is visible through it. The grip is part of
# the fixed body, NOT a separate protruding loop.
# Articulations (both share vertical +Z at the neck, decoupled via a massless
# carrier):
#   - cap_rotate: CONTINUOUS  (black screw cap spins about +Z)
#   - cap_slide:  PRISMATIC   (cap slides straight up off the neck)

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tessellate_cadquery,
)

# ---- key dimensions -------------------------------------------------------
BODY_W = 0.135          # full width  (x)
BODY_D = 0.100          # full depth  (y)
BODY_H = 0.255          # body height (deck)
NECK_X = -0.040         # neck offset toward -X (top-left)
DECK_Z = 0.255          # z of the flat top deck of the body
NECK_TOP_Z = 0.270      # z of the neck top / where the cap seats
NECK_R = 0.0150         # neck outer radius
CAP_R = 0.0185          # cap outer radius (sits over the neck)
CAP_H = 0.024           # cap height

# Grip finger hole (cut through the upper-+X shoulder, full Y depth).
GRIP_CX = 0.040         # hole center x (toward +X / right)
GRIP_CZ = 0.190         # hole center z
GRIP_RX = 0.017         # hole half-width  (x)
GRIP_RZ = 0.040         # hole half-height (z)


def _rrect_pts(w: float, d: float, r: float, seg: int = 6):
    # Counter-clockwise rounded-rectangle point loop in local XY.
    hw, hd = w / 2.0 - r, d / 2.0 - r
    pts = []
    corners = [
        (hw, hd, 0.0),
        (-hw, hd, math.pi / 2.0),
        (-hw, -hd, math.pi),
        (hw, -hd, 1.5 * math.pi),
    ]
    for cx, cy, a0 in corners:
        for k in range(seg + 1):
            a = a0 + (math.pi / 2.0) * (k / seg)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _loft_rrects(sections) -> cq.Workplane:
    # sections: list of (z, width, depth, radius); lofts rounded rectangles
    # stacked along +Z.
    wp = cq.Workplane("XY")
    prev_z = 0.0
    for i, (z, w, d, r) in enumerate(sections):
        off = z if i == 0 else z - prev_z
        wp = wp.workplane(offset=off)
        wp = wp.polyline(_rrect_pts(w, d, r)).close()
        prev_z = z
    return wp.loft(ruled=False)


def _bottle_body() -> cq.Workplane:
    # Squarish HDPE bottle. The top stays full-width (no draw-in deck) so the
    # +X shoulder forms a solid grip slab and the -X corner carries the neck.
    body = _loft_rrects(
        [
            (0.000, 0.122, 0.090, 0.014),
            (0.012, 0.135, 0.100, 0.018),
            (0.180, 0.135, 0.100, 0.018),
            (0.225, 0.133, 0.098, 0.020),
            (DECK_Z, 0.122, 0.090, 0.024),   # gentle draw-in at the top deck
        ]
    )

    # Recessed base (typical of blow-moulded jugs): a shallow dish cut into the
    # underside so the jug stands on a ring foot.
    base_recess = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .polyline(_rrect_pts(0.104, 0.072, 0.016)).close()
        .extrude(0.010)
    )
    body = body.cut(base_recess)

    # Open pour mouth: bore the neck axis deep into the body so the mouth reads
    # as a real through opening, not a shallow dimple under the cap.
    mouth = (
        cq.Workplane("XY")
        .workplane(offset=0.165)
        .center(NECK_X, 0.0)
        .circle(0.009)
        .extrude(NECK_TOP_Z + 0.002 - 0.165)
    )
    body = body.cut(mouth)

    # ---- INTEGRATED D-GRIP: oval finger hole cut through the +X shoulder ----
    # Cut along Y through the full body depth so daylight is visible through the
    # grip. The remaining material on +X of the hole is the grip bar; the
    # material on -X of the hole fuses into the bottle wall.
    grip_hole = (
        cq.Workplane("XZ")
        .workplane(offset=-(BODY_D / 2.0 + 0.02))
        .center(GRIP_CX, GRIP_CZ)
        .ellipse(GRIP_RX, GRIP_RZ)
        .extrude(BODY_D + 0.04)
    )
    body = body.cut(grip_hole)
    return body


def _neck() -> cq.Workplane:
    # Threaded neck stub rising from the deck, bored down the middle so the cap
    # caps a real open mouth.
    base_z = DECK_Z - 0.008
    neck = (
        cq.Workplane("XY")
        .workplane(offset=base_z)
        .center(NECK_X, 0.0)
        .circle(NECK_R)
        .extrude(NECK_TOP_Z - base_z)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=NECK_TOP_Z - 0.030)
        .center(NECK_X, 0.0)
        .circle(NECK_R - 0.004)
        .extrude(0.034)
    )
    return neck.cut(bore)


def _cap_mesh():
    # Black ribbed screw cap: a short cylinder with a flat top and a ring of
    # vertical grip ribs around the skirt, plus a moulded top rim.
    cap = CylinderGeometry(CAP_R, CAP_H, radial_segments=40)
    cap.translate(0.0, 0.0, CAP_H / 2.0)
    n = 20
    for i in range(n):
        a = 2.0 * math.pi * i / n
        rib = CylinderGeometry(0.0016, CAP_H * 0.85, radial_segments=8)
        rib.translate(CAP_R * 0.99, 0.0, CAP_H / 2.0)
        rib.rotate_z(a)
        cap.merge(rib)
    rim = TorusGeometry(CAP_R - 0.002, 0.0022, radial_segments=10, tubular_segments=40)
    rim.translate(0.0, 0.0, CAP_H - 0.002)
    cap.merge(rim)
    return mesh_from_geometry(cap, "cap_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="engine_oil_jug")

    gold = model.material("gold_hdpe", rgba=(0.62, 0.52, 0.18, 1.0))
    black = model.material("cap_black", rgba=(0.09, 0.09, 0.10, 1.0))
    purple = model.material("label_purple", rgba=(0.45, 0.12, 0.42, 1.0))
    accent = model.material("label_accent", rgba=(0.90, 0.88, 0.90, 1.0))
    marker_mat = model.material("marker_white", rgba=(0.92, 0.92, 0.92, 1.0))

    # ---- body (root): bottle + neck + integrated D-grip + label ----
    body = model.part("body")

    shell = _bottle_body().union(_neck())
    body.visual(mesh_from_cadquery(shell, "jug_shell"), material=gold, name="jug_shell")

    # Purple "DAYTONA 5W-40" label panel proud of the front (+Y) face, lower-left.
    body.visual(
        Box((0.090, 0.005, 0.115)),
        origin=Origin(xyz=(-0.018, BODY_D / 2.0 + 0.0010, 0.105)),
        material=purple,
        name="label_panel",
    )
    # Light "5W-40" text accent bar across the label.
    body.visual(
        Box((0.072, 0.004, 0.014)),
        origin=Origin(xyz=(-0.018, BODY_D / 2.0 + 0.0035, 0.120)),
        material=accent,
        name="label_text",
    )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)), mass=0.65, origin=Origin(xyz=(0.0, 0.0, 0.125))
    )

    # ---- massless carrier: decouples cap rotation from cap slide ----
    carrier = model.part("cap_carrier")
    carrier.inertial = Inertial.from_geometry(Box((0.01, 0.01, 0.01)), mass=1e-4)

    # ---- black ribbed screw cap ----
    cap = model.part("cap")
    cap.visual(_cap_mesh(), material=black, name="cap_shell")
    # Off-axis marker so a spin is observable in tests.
    cap.visual(
        Box((0.006, 0.006, 0.006)),
        origin=Origin(xyz=(CAP_R + 0.002, 0.0, CAP_H * 0.5)),
        material=marker_mat,
        name="cap_marker",
    )
    cap.inertial = Inertial.from_geometry(
        Box((2 * CAP_R, 2 * CAP_R, CAP_H)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, CAP_H / 2.0)),
    )

    # cap_rotate: body -> carrier, CONTINUOUS about +Z at the neck.
    model.articulation(
        "cap_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=carrier,
        origin=Origin(xyz=(NECK_X, 0.0, NECK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )
    # cap_slide: carrier -> cap, PRISMATIC straight up +Z off the neck.
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=CAP_H, effort=1.0, velocity=1.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _ray_solid_intervals_along_y(verts, faces, px, pz, y0, y1):
    # Return sorted entry/exit y-values where the +Y ray at (px, *, pz) crosses
    # the triangle mesh. Used to prove the grip is a real through-hole: there
    # must be an OPEN gap (no solid) spanning the hole center (y ~ 0).
    crossings = []
    for f in faces:
        a = verts[f[0]]
        b = verts[f[1]]
        c = verts[f[2]]
        # Triangle in XZ plane test for ray parallel to Y at fixed (px, pz).
        # Solve barycentric for the point (px, pz) inside triangle's XZ
        # projection; if inside, the crossing y is the interpolated y.
        ax, az = a[0], a[2]
        bx, bz = b[0], b[2]
        cx, cz = c[0], c[2]
        d = (bz - cz) * (ax - cx) + (cx - bx) * (az - cz)
        if abs(d) < 1e-12:
            continue
        l1 = ((bz - cz) * (px - cx) + (cx - bx) * (pz - cz)) / d
        l2 = ((cz - az) * (px - cx) + (ax - cx) * (pz - cz)) / d
        l3 = 1.0 - l1 - l2
        if l1 < -1e-9 or l2 < -1e-9 or l3 < -1e-9:
            continue
        y = l1 * a[1] + l2 * b[1] + l3 * c[1]
        if y0 - 1e-6 <= y <= y1 + 1e-6:
            crossings.append(y)
    crossings.sort()
    return crossings


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    rotate = object_model.get_articulation("cap_rotate")
    slide = object_model.get_articulation("cap_slide")

    # Cap seats on the neck at the top: small intentional embed of the cap
    # skirt over the threaded neck stub.
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_shell",
        elem_b="jug_shell",
        reason="Black screw cap skirt is intentionally seated over the threaded neck stub.",
    )

    # 1. Jug taller than wide (and taller than deep).
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "jug taller than wide and deep",
        bext[2] > bext[0] + 0.08 and bext[2] > bext[1] + 0.08,
        details=f"body extents={bext}",
    )

    # 2. Neck/cap are at the top of the jug.
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap is at the top of the jug",
        cap_pos is not None and cap_pos[2] > 0.25,
        details=f"cap origin={cap_pos}",
    )

    # 3. Cap is offset toward -X (top-left neck corner), not centered.
    ctx.check(
        "cap is on the offset (top-left) neck",
        cap_pos is not None and cap_pos[0] < -0.020,
        details=f"cap origin={cap_pos}",
    )

    # 4. The integrated grip is part of the body, NOT a separate link/part.
    part_names = {p.name for p in object_model.parts}
    ctx.check(
        "no separate handle part (grip is integral to body)",
        part_names == {"body", "cap_carrier", "cap"},
        details=f"parts={sorted(part_names)}",
    )

    # 5. The grip slab is flush within the body silhouette: the body's +X edge
    # is at ~half width (the grip bar is part of the outer wall, not a loop
    # sticking out past it).
    shell_aabb = ctx.part_element_world_aabb(body, elem="jug_shell")
    body_max_x = shell_aabb[1][0]
    ctx.check(
        "grip slab is flush within the body silhouette (+X edge ~ half width)",
        body_max_x > GRIP_CX + GRIP_RX and body_max_x < BODY_W / 2.0 + 0.012,
        details=f"jug_shell max_x={body_max_x}, BODY_W/2={BODY_W / 2.0}",
    )

    # 6. The grip finger hole is a REAL THROUGH-HOLE: rebuild the body shell,
    # tessellate it, and ray-cast along +Y through the hole center. A solid slab
    # would give one continuous solid interval; a through-hole gives an OPEN gap
    # spanning y=0 (daylight visible front-to-back).
    shell = _bottle_body().union(_neck())
    verts, faces = tessellate_cadquery(shell, tolerance=0.0008)
    crossings = _ray_solid_intervals_along_y(
        verts, faces, GRIP_CX, GRIP_CZ, -0.5, 0.5
    )
    # Inside/outside parity: solid where an odd number of crossings lie ahead.
    def _solid_at(y: float) -> bool:
        ahead = sum(1 for c in crossings if c > y + 1e-9)
        return (ahead % 2) == 1

    # A Y-ray through a Y-axis through-hole passes through open space at the
    # hole center: it strikes no front/back wall there, so the center is NOT
    # solid (daylight visible). Contrast with the grip bar below, which is solid.
    hole_open = not _solid_at(0.0)
    ctx.check(
        "grip finger hole is a real through-hole (daylight in Y at hole center)",
        hole_open,
        details=f"y-crossings at hole center={crossings}",
    )

    # 7. A solid grip bar exists OUTBOARD (+X) of the finger hole: a ray just
    # outside the hole edge must hit continuous solid (no open gap there).
    bar_cross = _ray_solid_intervals_along_y(
        verts, faces, GRIP_CX + GRIP_RX + 0.008, GRIP_CZ, -0.5, 0.5
    )
    def _solid_at_bar(y: float) -> bool:
        ahead = sum(1 for c in bar_cross if c > y + 1e-9)
        return (ahead % 2) == 1

    bar_solid = (len(bar_cross) >= 2) and _solid_at_bar(0.0)
    ctx.check(
        "solid grip bar exists outboard of the finger hole",
        bar_solid,
        details=f"y-crossings at grip bar={bar_cross}",
    )

    # 8. Cap spins about +Z: the off-axis marker swings around.
    m0 = ctx.part_element_world_aabb(cap, elem="cap_marker")
    mc0 = ((m0[0][0] + m0[1][0]) / 2.0, (m0[0][1] + m0[1][1]) / 2.0)
    with ctx.pose({rotate: math.pi / 2.0}):
        m1 = ctx.part_element_world_aabb(cap, elem="cap_marker")
        mc1 = ((m1[0][0] + m1[1][0]) / 2.0, (m1[0][1] + m1[1][1]) / 2.0)
    moved = math.hypot(mc1[0] - mc0[0], mc1[1] - mc0[1])
    ctx.check(
        "cap marker swings around when cap spins",
        moved > 0.01,
        details=f"marker rest={mc0}, spun={mc1}, moved={moved}",
    )

    # 9. Cap slides straight up off the neck.
    rest_z = ctx.part_world_position(cap)[2]
    with ctx.pose({slide: CAP_H}):
        up_z = ctx.part_world_position(cap)[2]
    ctx.check(
        "cap slides up off the neck",
        up_z > rest_z + CAP_H * 0.8,
        details=f"rest_z={rest_z}, up_z={up_z}",
    )

    return ctx.report()


object_model = build_object_model()
