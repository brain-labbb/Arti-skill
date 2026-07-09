from __future__ import annotations

# Blue plastic open-top storage drum (~30 L) with a carry bail handle.
# Fork of the parent drum: the front swinging buckle clasp is replaced by a
# single arching U-shaped wire bail hung from two molded lugs near the rim on
# opposite sides (±X). The bail swings up over the lid and down to the side
# about a horizontal REVOLUTE pivot along X.
#
# Frame: vertical drum axis along +Z, base at z=0, centerline at x=y=0.
# Lugs are on the ±X sides near the rim.
#
# Articulations:
#   - lid_lift: PRISMATIC lift-off disc (identical to parent)
#   - bail_swing: REVOLUTE U-shaped wire bail. At q=0 the bail hangs straight
#       down against the barrel; positive q swings the grip up and over the lid.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tessellate_cadquery,
)

# ---- key dimensions (meters) --------------------------------------------------
BARREL_H = 0.35        # overall barrel height
BARREL_R = 0.125       # outer radius at the belly
TOP_R = 0.115          # radius at the top rim
WALL_T = 0.010         # nominal wall thickness
RIM_Z = BARREL_H       # top rim plane
LID_Z = RIM_Z - 0.002  # lid underside seats on the rolled rim

# Bail handle dimensions
WIRE_R = 0.004         # bail wire radius (8 mm diameter wire)
HALF_SPAN = 0.134      # pivot-hole centre to drum centreline
LEG_H = 0.130          # leg length from pivot to grip junction
ARCH_DEPTH = 0.022     # how far the grip arches outward below the legs
GRIP_REACH = 0.160     # how far the grip extends outward in +Y from centre
LUG_Z = RIM_Z - 0.044  # lug pivot height (below the lid skirt reach)
LUG_W = 0.038          # lug width along X (protrusion)
LUG_D = 0.026          # lug depth along Y
LUG_HGT = 0.034        # lug height along Z


# ---- shared CadQuery helpers --------------------------------------------------

def _cq_geom(solid: cq.Workplane) -> MeshGeometry:
    verts, faces = tessellate_cadquery(solid)
    geom = MeshGeometry()
    for vx, vy, vz in verts:
        geom.add_vertex(vx, vy, vz)
    for a, b, c in faces:
        geom.add_face(a, b, c)
    return geom


def _profile_loft(sections) -> cq.Workplane:
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=False)


# ---- barrel body (open-top ribbed shell + rolled rim) -------------------------

def _barrel_solid() -> cq.Workplane:
    outer = _profile_loft(
        [
            (0.000, 0.116),
            (0.012, 0.122),
            (0.060, 0.125),
            (0.175, 0.124),
            (0.300, 0.119),
            (0.330, 0.117),
            (RIM_Z, TOP_R),
        ]
    )
    shell = outer.faces(">Z").shell(-0.012)

    for z in (0.090, 0.150, 0.210, 0.270):
        ring = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .circle(0.127)
            .circle(0.100)
            .extrude(0.006)
        )
        shell = shell.union(ring)

    lip = (
        cq.Workplane("XY")
        .workplane(offset=RIM_Z - 0.014)
        .circle(0.122)
        .circle(0.104)
        .extrude(0.016)
    )
    shell = shell.union(lip)
    return shell


# ---- molded bail lugs (repeated sub-parts, shared helper) ---------------------

def _lug_solid(sign: int) -> cq.Workplane:
    """One molded lug boss at the given barrel side (sign ±1 for ±X).

    The lug is a small rounded box protruding outward from the barrel wall with
    a horizontal hole through it for the bail wire pivot.
    """
    cx = sign * HALF_SPAN  # lug centre X

    # Solid boss
    body = (
        cq.Workplane("XY")
        .box(LUG_W, LUG_D, LUG_HGT, centered=(True, True, True))
        .translate((cx, 0, LUG_Z))
    )
    # Fillet the outer vertical edges for a molded look
    body = body.edges("|Z").fillet(0.004)

    # Horizontal bore along X for the bail wire
    bore_len = LUG_W + 0.006
    bore = (
        cq.Workplane("YZ")
        .circle(WIRE_R + 0.001)
        .extrude(bore_len)
        .translate((cx - bore_len / 2, 0, LUG_Z))
    )
    return body.cut(bore)


# ---- lift-off lid -------------------------------------------------------------

def _lid_mesh():
    disc = (
        cq.Workplane("XY")
        .circle(0.122)
        .extrude(0.012)
        .faces(">Z")
        .workplane()
        .circle(0.122)
        .workplane(offset=0.006)
        .circle(0.088)
        .loft(ruled=False)
    )
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=-0.022)
        .circle(0.127)
        .circle(0.121)
        .extrude(0.030)
    )
    return mesh_from_cadquery(disc.union(skirt), "lid_shell")


# ---- U-shaped wire bail handle -----------------------------------------------

def _bail_mesh():
    """U-shaped wire bail. Part frame origin is the pivot centre (midpoint
    between the two lug holes). At q=0 the legs hang in -Z and the grip
    extends outward in -Y, arching below the legs."""

    bail = None

    # Two vertical legs (cylinders along Z from pivot down to grip junction)
    for i in range(2):
        sx = -1 if i == 0 else 1
        leg = (
            cq.Workplane("XY")
            .circle(WIRE_R)
            .extrude(LEG_H + WIRE_R)
            .translate((sx * HALF_SPAN, 0, -LEG_H))
        )
        bail = leg if bail is None else bail.union(leg)

    # Arching grip bar: curved wire from left leg bottom to right leg bottom,
    # following an elliptical arc that stays outside the barrel shell at every
    # point.  x(t) = -HALF_SPAN*cos(pi*t) keeps the wire centreline at
    # distance >= HALF_SPAN from the drum centreline, clearing the barrel wall.
    # y(t) = GRIP_REACH*sin(pi*t) bows the grip outward in +Y.
    n_seg = 10
    pts = []
    for j in range(n_seg + 1):
        t = j / n_seg
        x = -HALF_SPAN * math.cos(math.pi * t)
        y = GRIP_REACH * math.sin(math.pi * t)
        z = -LEG_H - ARCH_DEPTH * math.sin(math.pi * t)
        pts.append((x, y, z))

    for j in range(n_seg):
        p0 = pts[j]
        p1 = pts[j + 1]
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dz = p1[2] - p0[2]
        seg_len = math.sqrt(dx * dx + dy * dy + dz * dz)
        if seg_len < 1e-9:
            continue

        # Build a cylinder along local +Z of length seg_len
        seg = cq.Workplane("XY").circle(WIRE_R).extrude(seg_len)

        # Rotate local +Z to align with segment direction (dx, dy, dz)
        # Using two rotations: first about Y to tilt into XZ, then about Z for Y
        # Actually, use a single rotation via axis-angle.
        # Target direction: (dx, dy, dz) / seg_len
        # Source direction: (0, 0, 1)
        # Rotation axis: cross((0,0,1), (dx,dy,dz)/L) = (-dy/L, dx/L, 0)
        # Rotation angle: acos(dz/L)
        ux, uy, uz = -dy / seg_len, dx / seg_len, 0.0
        u_len = math.sqrt(ux * ux + uy * uy)
        if u_len > 1e-9:
            ux /= u_len
            uy /= u_len
            angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dz / seg_len))))
            seg = seg.rotate((0, 0, 0), (ux, uy, 0), angle_deg)

        # Translate so the cylinder base is at p0
        seg = seg.translate(p0)
        bail = bail.union(seg)

    return mesh_from_cadquery(bail, "bail_wire")


# ---- model assembly -----------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="storage_drum")

    blue = model.material("drum_blue", rgba=(0.16, 0.36, 0.72, 1.0))
    black = model.material("lid_black", rgba=(0.09, 0.09, 0.10, 1.0))
    zinc = model.material("bail_zinc", rgba=(0.55, 0.55, 0.52, 1.0))

    # ---- barrel body (root) ----
    body = model.part("barrel_body")
    body.visual(
        mesh_from_cadquery(_barrel_solid(), "barrel_shell"),
        material=blue,
        name="barrel_shell",
    )
    # Molded bail lugs: two identical bosses on ±X sides (inline parent visuals)
    for i in range(2):
        sign = -1 if i == 0 else 1
        body.visual(
            mesh_from_cadquery(_lug_solid(sign), f"lug_{i}"),
            material=blue,
            name=f"lug_{i}",
        )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BARREL_R, length=BARREL_H),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, BARREL_H / 2.0)),
    )

    # ---- lid: lifts straight up off the rim (prismatic, same as parent) ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=black, name="lid_shell")
    boss = CylinderGeometry(0.024, 0.012).translate(0.0, 0.0, 0.024)
    lid.visual(mesh_from_geometry(boss, "lid_boss"), material=black, name="lid_boss")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.119, length=0.04),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "lid_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.2, lower=0.0, upper=0.05),
    )

    # ---- bail handle: U-shaped wire, REVOLUTE about horizontal X axis ----
    bail = model.part("bail_handle")
    bail.visual(_bail_mesh(), material=zinc, name="bail_wire")
    bail.inertial = Inertial.from_geometry(
        Box((2 * HALF_SPAN, GRIP_REACH, LEG_H + ARCH_DEPTH)),
        mass=0.12,
        origin=Origin(xyz=(0.0, GRIP_REACH / 2, -(LEG_H + ARCH_DEPTH) / 2)),
    )
    model.articulation(
        "bail_swing",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(0.0, 0.0, LUG_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=math.pi,
        ),
    )

    return model


# ---- helpers for tests --------------------------------------------------------

def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")
    lid = object_model.get_part("lid")
    bail = object_model.get_part("bail_handle")
    lid_lift = object_model.get_articulation("lid_lift")
    bail_swing = object_model.get_articulation("bail_swing")

    # --- barrel is taller than it is wide (lugs add ~20 mm each side in X) ---
    bext = _ext(ctx.part_world_aabb(body))
    ctx.check(
        "barrel taller than wide",
        bext[2] > bext[0] * 1.10 and bext[2] > bext[1] + 0.05,
        details=f"barrel extents={bext}",
    )

    # --- lid sits on top of the barrel rim ---
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits on top of the barrel",
        lid_pos is not None and lid_pos[2] > 0.30,
        details=f"lid origin={lid_pos}",
    )
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_shell", elem_b="barrel_shell",
        reason="Lift-off lid skirt is intentionally seated down over the rolled rim lip.",
    )

    # --- lid lifts straight up off the rim (prismatic, +Z only) ---
    lz0 = ctx.part_world_position(lid)[2]
    with ctx.pose({lid_lift: 0.05}):
        lifted = ctx.part_world_position(lid)
    lz1 = lifted[2]
    ctx.check(
        "lid lifts straight up off the rim",
        lz1 > lz0 + 0.045
        and abs(lifted[0] - lid_pos[0]) < 1e-4
        and abs(lifted[1] - lid_pos[1]) < 1e-4,
        details=f"rest_z={lz0}, lifted={lifted}",
    )

    # --- bail handle is mounted at the lugs near the rim ---
    bail_pos = ctx.part_world_position(bail)
    ctx.check(
        "bail pivot mounted near the rim",
        bail_pos is not None and bail_pos[2] > 0.28,
        details=f"bail origin={bail_pos}",
    )

    # --- bail wire legs are seated inside the lug pivot holes ---
    for i in range(2):
        ctx.allow_overlap(
            bail, body,
            elem_a="bail_wire", elem_b=f"lug_{i}",
            reason=f"Bail wire leg is intentionally seated inside lug_{i} pivot bore.",
        )

    # --- bail swings up about the horizontal pivot ---
    bail_rest_bottom = ctx.part_world_aabb(bail)[0][2]
    with ctx.pose({bail_swing: math.pi / 2}):
        bail_mid_aabb = ctx.part_world_aabb(bail)
    with ctx.pose({bail_swing: math.pi}):
        bail_full_aabb = ctx.part_world_aabb(bail)

    ctx.check(
        "bail swings up from rest",
        bail_mid_aabb[1][2] > bail_rest_bottom + 0.05,
        details=f"rest_bottom_z={bail_rest_bottom}, mid_top_z={bail_mid_aabb[1][2]}",
    )
    ctx.check(
        "bail arches over the lid at full swing",
        bail_full_aabb[1][2] > RIM_Z + 0.05,
        details=f"full_top_z={bail_full_aabb[1][2]}, rim_z={RIM_Z}",
    )

    # --- bail grip extends outward from the barrel at rest ---
    bail_rest_aabb = ctx.part_world_aabb(bail)
    ctx.check(
        "bail grip hangs outside the barrel",
        bail_rest_aabb[1][1] > BARREL_R,
        details=f"bail max_y={bail_rest_aabb[1][1]}, barrel_r={BARREL_R}",
    )

    return ctx.report()


object_model = build_object_model()
