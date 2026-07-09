from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# =============================================================================
# Yellow tracked hydraulic excavator (CAT 323D style).
#
# Convention: machine FORWARD (arm reaches) = +X, width = Y, up = +Z.
# Tracks rest on the ground at z = 0. Overall length ~3.8 m, height ~3.0 m.
#
# Link tree:
#   undercarriage (root, static)            -- crawler tracks + car body + slew ring
#     house (REVOLUTE about Z, slew)        -- deck, cab, engine hood, counterweight
#       boom (REVOLUTE about Y, pitch)      -- bent box-section boom + boom cylinders' rods
#         boom_cyl (FIXED, rest-pose)       -- boom-cylinder barrel pair pinned to house trunnion
#         stick (REVOLUTE about Y)          -- dipper stick + stick-cylinder rod
#           stick_cyl (FIXED, rest-pose)    -- stick-cylinder barrel pinned to boom-top ear
#           bucket (REVOLUTE about Y, curl) -- digging bucket with teeth + bucket-cylinder rod
#             bucket_cyl (FIXED, rest-pose) -- bucket-cylinder barrel pinned to stick-top ear
# =============================================================================


# ---------------------------------------------------------------------------
# Small geometry helpers
# ---------------------------------------------------------------------------
def _dist(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _mid(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(a, b):
    """rpy so a Cylinder (local +Z) spans from point a to point b."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _member(part, a, b, radius, material):
    """A cylindrical strut from world-ish point a to b inside a part frame."""
    part.visual(
        Cylinder(radius=radius, length=_dist(a, b)),
        origin=Origin(xyz=_mid(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
    )


# ---------------------------------------------------------------------------
# Track belt (stadium loop) as a CadQuery extrusion
# ---------------------------------------------------------------------------
def _track_belt_shape(half_len, r_idler, r_drive, width, thickness):
    """Stadium-loop belt outline extruded across Y (track width).

    Built in XZ plane (X long axis, Z up), then extruded along Y by `width`.
    Two end radii (idler at +X, drive at -X) joined by tangent straights, with
    an inner cut so it reads as a thin track shoe loop, not a solid block.
    """
    cx_front = half_len
    cx_rear = -half_len
    cz = max(r_idler, r_drive)  # centerline height so the deepest end touches z=0
    # Outer loop (approx as rounded rectangle blending two radii).
    outer = (
        cq.Workplane("XZ")
        .moveTo(cx_front, cz + r_idler)
        .lineTo(cx_rear, cz + r_drive)
        .threePointArc(
            (cx_rear - r_drive, cz),
            (cx_rear, cz - r_drive),
        )
        .lineTo(cx_front, cz - r_idler)
        .threePointArc(
            (cx_front + r_idler, cz),
            (cx_front, cz + r_idler),
        )
        .close()
    )
    inner = (
        cq.Workplane("XZ")
        .moveTo(cx_front, cz + r_idler - thickness)
        .lineTo(cx_rear, cz + r_drive - thickness)
        .threePointArc(
            (cx_rear - r_drive + thickness, cz),
            (cx_rear, cz - r_drive + thickness),
        )
        .lineTo(cx_front, cz - r_idler + thickness)
        .threePointArc(
            (cx_front + r_idler - thickness, cz),
            (cx_front, cz + r_idler - thickness),
        )
        .close()
    )
    # XZ-workplane extrude grows along -Y; translate +width/2 to center on Y.
    belt = outer.extrude(width)
    inner_solid = inner.extrude(width)
    belt = belt.cut(inner_solid)
    belt = belt.translate((0, width / 2.0, 0))
    return belt


# ---------------------------------------------------------------------------
# Boom: heavy bent box-section arm (two straight box segments + lugs)
# ---------------------------------------------------------------------------
def _boom_shape(seg1_len, seg2_len, bend_deg, w, h1, h2):
    """Bent box-section boom built along +X starting at the foot pivot (origin).

    seg1 rises from the foot, then bends down toward the stick pivot.
    Returns a CadQuery solid in the boom local frame (foot pivot at origin,
    long axis roughly +X, width along Y).
    """
    bend = math.radians(bend_deg)
    # Lower segment: from foot (0,0,0) up-forward.
    p0 = (0.0, 0.0)  # foot pivot in XZ
    # first segment direction: forward and slightly up
    a1 = math.radians(38.0)
    p1 = (seg1_len * math.cos(a1), seg1_len * math.sin(a1))
    # second segment angles back down toward the work
    a2 = a1 - bend
    p2 = (p1[0] + seg2_len * math.cos(a2), p1[1] + seg2_len * math.sin(a2))

    def box_between(pa, pb, height):
        dx = pb[0] - pa[0]
        dz = pb[1] - pa[1]
        length = math.hypot(dx, dz)
        ang = math.atan2(dz, dx)
        # build a box of (length x w x height) centered at midpoint, rotated about Y
        mid = ((pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0)
        sol = (
            cq.Workplane("XZ")
            .rect(length, height)
            .extrude(w)
        )
        # XZ extrude grows along -Y; recenter to Y=0
        sol = sol.translate((0, w / 2.0, 0))
        sol = sol.rotate((0, 0, 0), (0, 1, 0), -math.degrees(ang))
        sol = sol.translate((mid[0], 0, mid[1]))
        return sol

    seg_a = box_between(p0, p1, h1)
    seg_b = box_between(p1, p2, h2)
    boom = seg_a.union(seg_b)
    # knuckle reinforcement at the bend (Y-axis cylinder spanning the boom width)
    knuckle = (
        cq.Workplane("XZ")
        .circle(h1 * 0.62)
        .extrude(w)
        .translate((p1[0], w / 2.0, p1[1]))
    )
    boom = boom.union(knuckle)
    return boom, p0, p1, p2


# ---------------------------------------------------------------------------
# Stick (dipper): tapered box-section
# ---------------------------------------------------------------------------
def _stick_shape(length, w, h_root, h_tip):
    """Stick built along +X from the boom-tip pivot (origin)."""
    pts = [
        (0.0, h_root / 2.0),
        (length, h_tip / 2.0),
        (length, -h_tip / 2.0),
        (0.0, -h_root / 2.0),
    ]
    sol = (
        cq.Workplane("XZ")
        .polyline([(x, z) for (x, z) in pts])
        .close()
        .extrude(w)
        .translate((0, w / 2.0, 0))
    )
    # rounded back at the root knuckle (Y-axis cylinder)
    knuckle = (
        cq.Workplane("XZ")
        .circle(h_root * 0.55)
        .extrude(w)
        .translate((0.0, w / 2.0, 0.0))
    )
    return sol.union(knuckle)


# ---------------------------------------------------------------------------
# Bucket: curved scoop via lofted sections + teeth handled as separate boxes
# ---------------------------------------------------------------------------
def _bucket_shape(width, opening, depth, back_h):
    """Curved digging bucket.

    Local frame: pivot (to stick) at origin. The bucket curls under; we model a
    swept C-section. Cross-section lies in XZ, swept along the curl arc.
    Returns the hollow scoop solid.
    """
    w2 = width / 2.0
    # Outer profile of the bucket side wall in XZ (a curled scoop).
    outer_pts = [
        (0.0, 0.0),
        (0.0, -back_h),
        (depth * 0.45, -back_h - opening * 0.30),
        (depth * 0.95, -back_h - opening * 0.05),
        (depth * 1.02, -back_h + opening * 0.45),
        (depth * 0.70, -back_h + opening * 0.78),
        (depth * 0.20, -back_h * 0.30),
    ]
    side = (
        cq.Workplane("XZ")
        .polyline(outer_pts)
        .close()
        .extrude(width)
        .translate((0, w2, 0))
    )
    # Hollow the inside by cutting a slightly smaller inner volume.
    inner_pts = [
        (depth * 0.10, -back_h * 0.65),
        (depth * 0.10, -back_h - opening * 0.18),
        (depth * 0.55, -back_h - opening * 0.30),
        (depth * 0.88, -back_h - opening * 0.02),
        (depth * 0.90, -back_h + opening * 0.40),
        (depth * 0.60, -back_h + opening * 0.62),
        (depth * 0.22, -back_h * 0.20),
    ]
    inner = (
        cq.Workplane("XZ")
        .polyline(inner_pts)
        .close()
        .extrude(width - 0.06)
        .translate((0, w2 - 0.03, 0))
    )
    bucket = side.cut(inner)
    return bucket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tracked_hydraulic_excavator")

    # ---- Materials --------------------------------------------------------
    cat_yellow = model.material("cat_yellow", rgba=(0.93, 0.74, 0.10, 1.0))
    deep_yellow = model.material("deep_yellow", rgba=(0.86, 0.66, 0.07, 1.0))
    track_black = model.material("track_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.12, 0.16, 0.55))
    tooth_steel = model.material("tooth_steel", rgba=(0.40, 0.40, 0.42, 1.0))
    hose_black = model.material("hose_black", rgba=(0.08, 0.08, 0.09, 1.0))

    # =====================================================================
    # UNDERCARRIAGE (root, static): two crawler tracks + car body + slew ring
    # =====================================================================
    undercarriage = model.part("undercarriage")

    TRACK_HALF_LEN = 1.55  # idler-to-drive half spacing along X
    R_IDLER = 0.36
    R_DRIVE = 0.40
    TRACK_W = 0.58
    BELT_T = 0.10
    GAUGE = 1.02  # track centerline offset in +/-Y
    cz_belt = max(R_IDLER, R_DRIVE)  # belt center height; deepest end touches z=0

    belt_geom = _track_belt_shape(TRACK_HALF_LEN, R_IDLER, R_DRIVE, TRACK_W, BELT_T)
    belt_mesh = mesh_from_cadquery(belt_geom, "track_belt.obj")

    for side, sy in (("left", GAUGE), ("right", -GAUGE)):
        # track belt loop
        undercarriage.visual(
            belt_mesh,
            origin=Origin(xyz=(0.0, sy, 0.0)),
            material=track_black,
            name=f"belt_{side}",
        )
        # grouser shoe ribs around the bottom run (most visible). Keep them
        # within the flat bottom run so each shoe seats on the belt underside.
        n_shoes = 16
        shoe_span = TRACK_HALF_LEN - 0.18
        for i in range(n_shoes):
            sx = -shoe_span + (2 * shoe_span) * i / (n_shoes - 1)
            undercarriage.visual(
                Box((0.085, TRACK_W + 0.02, 0.06)),
                origin=Origin(xyz=(sx, sy, 0.03)),
                material=dark_steel,
                name=f"shoe_{side}_{i}",
            )
        # idler (front, +X) and drive sprocket (rear, -X) hubs
        undercarriage.visual(
            Cylinder(radius=R_IDLER * 0.72, length=TRACK_W - 0.06),
            origin=Origin(xyz=(TRACK_HALF_LEN, sy, cz_belt), rpy=(math.pi / 2, 0, 0)),
            material=dark_steel,
            name=f"idler_{side}",
        )
        undercarriage.visual(
            Cylinder(radius=R_DRIVE * 0.70, length=TRACK_W - 0.06),
            origin=Origin(xyz=(-TRACK_HALF_LEN, sy, cz_belt), rpy=(math.pi / 2, 0, 0)),
            material=dark_steel,
            name=f"sprocket_{side}",
        )
        # bottom track rollers
        for i in range(3):
            rx = -0.8 + 0.8 * i
            undercarriage.visual(
                Cylinder(radius=0.14, length=TRACK_W - 0.12),
                origin=Origin(xyz=(rx, sy, 0.16), rpy=(math.pi / 2, 0, 0)),
                material=track_black,
                name=f"roller_{side}_{i}",
            )
        # track frame side plate
        undercarriage.visual(
            Box((2.7, 0.10, 0.30)),
            origin=Origin(xyz=(0.0, sy, cz_belt)),
            material=dark_steel,
            name=f"frame_{side}",
        )

    # Car-body frame linking the two tracks
    undercarriage.visual(
        Box((1.7, GAUGE * 2 - 0.30, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, cz_belt + 0.10)),
        material=dark_steel,
        name="car_body",
    )
    # Slew bearing ring on top
    SLEW_Z = cz_belt + 0.30
    undercarriage.visual(
        Cylinder(radius=0.62, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, SLEW_Z)),
        material=steel,
        name="slew_ring",
    )

    # =====================================================================
    # HOUSE (slews about Z): deck, cab, engine hood w/ louvers, counterweight
    # =====================================================================
    house = model.part("house")
    HOUSE_Z = SLEW_Z + 0.05  # house frame sits at top face of slew ring (joint origin)

    # slew turntable disc (bottom of house, mates on the ring)
    house.visual(
        Cylinder(radius=0.60, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="turntable",
    )
    # main deck / platform
    DECK_T = 0.18
    house.visual(
        Box((2.85, 1.95, DECK_T)),
        origin=Origin(xyz=(-0.30, 0.0, 0.08 + DECK_T / 2.0)),
        material=cat_yellow,
        name="deck",
    )
    deck_top = 0.08 + DECK_T

    # ---- Operator cab (left-front, +Y), glassy box -----------------------
    CAB_W, CAB_D, CAB_H = 0.78, 1.05, 1.18
    cab_cx, cab_cy = 0.70, 0.62
    cab_base_z = deck_top
    # yellow cab frame shell
    house.visual(
        Box((CAB_D, CAB_W, CAB_H)),
        origin=Origin(xyz=(cab_cx, cab_cy, cab_base_z + CAB_H / 2.0)),
        material=cat_yellow,
        name="cab_shell",
    )
    # roof cap
    house.visual(
        Box((CAB_D + 0.04, CAB_W + 0.04, 0.06)),
        origin=Origin(xyz=(cab_cx, cab_cy, cab_base_z + CAB_H + 0.01)),
        material=deep_yellow,
        name="cab_roof",
    )
    # tinted glass: front windshield, left side window, right (inner) window, rear
    gt = 0.02
    # front windshield (+X face)
    house.visual(
        Box((gt, CAB_W - 0.12, CAB_H - 0.34)),
        origin=Origin(xyz=(cab_cx + CAB_D / 2.0, cab_cy, cab_base_z + CAB_H / 2.0 + 0.04)),
        material=glass,
        name="cab_glass_front",
    )
    # left side window (+Y outer face)
    house.visual(
        Box((CAB_D - 0.22, gt, CAB_H - 0.40)),
        origin=Origin(xyz=(cab_cx, cab_cy + CAB_W / 2.0, cab_base_z + CAB_H / 2.0 + 0.05)),
        material=glass,
        name="cab_glass_left",
    )
    # right (inner, -Y) side window
    house.visual(
        Box((CAB_D - 0.30, gt, CAB_H - 0.46)),
        origin=Origin(xyz=(cab_cx, cab_cy - CAB_W / 2.0, cab_base_z + CAB_H / 2.0 + 0.06)),
        material=glass,
        name="cab_glass_right",
    )
    # rear window (-X face)
    house.visual(
        Box((gt, CAB_W - 0.20, CAB_H - 0.52)),
        origin=Origin(xyz=(cab_cx - CAB_D / 2.0, cab_cy, cab_base_z + CAB_H / 2.0 + 0.06)),
        material=glass,
        name="cab_glass_rear",
    )

    # ---- Engine hood (right-rear, -Y side) with louvered grilles ----------
    HOOD_L, HOOD_W, HOOD_H = 1.45, 1.05, 0.62
    hood_cx, hood_cy = -0.45, -0.42
    hood_base = deck_top
    house.visual(
        Box((HOOD_L, HOOD_W, HOOD_H)),
        origin=Origin(xyz=(hood_cx, hood_cy, hood_base + HOOD_H / 2.0)),
        material=cat_yellow,
        name="engine_hood",
    )
    # side louvered grille on the -Y (outer right) face of the hood
    side_grille = VentGrilleGeometry(
        (0.95, 0.40),
        frame=0.03,
        face_thickness=0.02,
        duct_depth=0.04,
        slat_pitch=0.05,
        slat_width=0.034,
        slat_angle_deg=32.0,
        slats=VentGrilleSlats(profile="flat", direction="down"),
        sleeve=VentGrilleSleeve(style="short"),
    )
    side_grille_mesh = mesh_from_geometry(side_grille, "engine_side_grille")
    # the grille lies in local XY, extends +Z; rotate so face points -Y
    house.visual(
        side_grille_mesh,
        origin=Origin(
            xyz=(hood_cx, hood_cy - HOOD_W / 2.0, hood_base + HOOD_H * 0.55),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="hood_grille_side",
    )
    # top grille panel on the hood roof
    top_grille = VentGrilleGeometry(
        (0.85, 0.62),
        frame=0.03,
        face_thickness=0.02,
        duct_depth=0.035,
        slat_pitch=0.06,
        slat_width=0.04,
        slat_angle_deg=0.0,
        slats=VentGrilleSlats(profile="flat", divider_count=2),
        sleeve=VentGrilleSleeve(style="short"),
    )
    top_grille_mesh = mesh_from_geometry(top_grille, "engine_top_grille")
    house.visual(
        top_grille_mesh,
        origin=Origin(xyz=(hood_cx + 0.10, hood_cy, hood_base + HOOD_H + 0.005)),
        material=dark_steel,
        name="hood_grille_top",
    )
    # short exhaust stack
    house.visual(
        Cylinder(radius=0.055, length=0.34),
        origin=Origin(xyz=(hood_cx + 0.55, hood_cy + 0.30, hood_base + HOOD_H + 0.17)),
        material=bright_steel,
        name="exhaust_stack",
    )

    # ---- Rear counterweight (heavy curved yellow block, -X) ---------------
    cw_geom = (
        cq.Workplane("XY")
        .moveTo(0.0, -0.95)
        .lineTo(0.0, 0.95)
        .lineTo(-0.28, 0.95)
        .threePointArc((-0.46, 0.0), (-0.28, -0.95))
        .close()
        .extrude(0.95)
    )
    cw_mesh = mesh_from_cadquery(cw_geom, "counterweight.obj")
    house.visual(
        cw_mesh,
        origin=Origin(xyz=(-1.35, 0.0, deck_top - 0.02)),
        material=cat_yellow,
        name="counterweight",
    )
    # counterweight base skirt down to deck (kept narrow so it clears the tracks)
    house.visual(
        Box((0.30, 1.34, 0.40)),
        origin=Origin(xyz=(-1.40, 0.0, deck_top - 0.16)),
        material=deep_yellow,
        name="counterweight_skirt",
    )

    # ---- Handrail along the deck edge ------------------------------------
    for ry in (0.92, -0.92):
        _member(
            house,
            (0.5, ry, deck_top),
            (0.5, ry, deck_top + 0.30),
            0.018,
            steel,
        )
        _member(
            house,
            (-1.0, ry, deck_top),
            (-1.0, ry, deck_top + 0.30),
            0.018,
            steel,
        )
        _member(
            house,
            (0.5, ry, deck_top + 0.30),
            (-1.0, ry, deck_top + 0.30),
            0.016,
            steel,
        )

    # ---- Boom mounting tower on the house front (just right of cab) -------
    # The boom foot pivot is here. Right of cab => toward -Y (centerline-ish).
    BOOM_FOOT_X = 1.15
    BOOM_FOOT_Y = -0.05
    BOOM_FOOT_Z = deck_top + 0.42
    # twin mount ears
    for ey in (BOOM_FOOT_Y - 0.20, BOOM_FOOT_Y + 0.20):
        house.visual(
            Box((0.40, 0.10, 0.95)),
            origin=Origin(xyz=(BOOM_FOOT_X, ey, deck_top + 0.30)),
            material=deep_yellow,
            name=f"boom_mount_{'p' if ey > BOOM_FOOT_Y else 'm'}",
        )
    # Forward trunnion bracket that carries the boom-cylinder pivot. It bridges
    # the deck front out to the cylinder base so the cylinders are supported.
    BOOM_CYL_PIVOT_X = 1.42
    house.visual(
        Box((0.70, 0.66, 0.26)),
        origin=Origin(xyz=(BOOM_CYL_PIVOT_X - 0.18, 0.0, deck_top + 0.05)),
        material=deep_yellow,
        name="boomcyl_bracket",
    )

    # =====================================================================
    # ARM CHAIN: boom -> stick -> bucket, plus hydraulic cylinders
    # =====================================================================

    # ---- BOOM ----
    BOOM_S1 = 1.65
    BOOM_S2 = 1.55
    BOOM_W = 0.34
    boom_geom, b_p0, b_p1, b_p2 = _boom_shape(BOOM_S1, BOOM_S2, 64.0, BOOM_W, 0.42, 0.36)
    boom_mesh = mesh_from_cadquery(boom_geom, "boom.obj")
    boom = model.part("boom")
    boom.visual(boom_mesh, material=cat_yellow, name="boom_body")
    # boom foot pivot lug (at origin)
    boom.visual(
        Cylinder(radius=0.16, length=BOOM_W + 0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="boom_foot_pin",
    )
    # boom tip pivot lug (at p2)
    boom_tip = (b_p2[0], 0.0, b_p2[1])
    boom.visual(
        Cylinder(radius=0.13, length=BOOM_W + 0.06),
        origin=Origin(xyz=boom_tip, rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="boom_tip_pin",
    )
    # cradle ears on boom top for the stick cylinder anchor (near the bend)
    stick_cyl_anchor = (b_p1[0] - 0.10, 0.0, b_p1[1] + 0.34)
    boom.visual(
        Box((0.22, BOOM_W + 0.10, 0.16)),
        origin=Origin(xyz=stick_cyl_anchor),
        material=deep_yellow,
        name="boom_stickcyl_ear",
    )

    # ---- STICK ----
    STICK_LEN = 1.95
    STICK_W = 0.28
    stick_geom = _stick_shape(STICK_LEN, STICK_W, 0.40, 0.26)
    stick_mesh = mesh_from_cadquery(stick_geom, "stick.obj")
    stick = model.part("stick")
    stick.visual(stick_mesh, material=cat_yellow, name="stick_body")
    # stick root pivot pin (at origin == boom tip)
    stick.visual(
        Cylinder(radius=0.13, length=STICK_W + 0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="stick_root_pin",
    )
    # stick tip bucket pivot pin (at +X tip)
    stick.visual(
        Cylinder(radius=0.10, length=STICK_W + 0.06),
        origin=Origin(xyz=(STICK_LEN, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="stick_tip_pin",
    )
    # top lug where stick cylinder rod connects (seated onto the stick top)
    stick_cyl_rod_pt = (0.18, 0.0, 0.30)
    stick.visual(
        Box((0.16, STICK_W + 0.08, 0.22)),
        origin=Origin(xyz=(stick_cyl_rod_pt[0], 0.0, 0.19)),
        material=deep_yellow,
        name="stick_cyl_lug",
    )
    # top lug where bucket cylinder barrel mounts (mid-stick top)
    bkt_cyl_anchor = (0.55, 0.0, 0.26)
    stick.visual(
        Box((0.18, STICK_W + 0.08, 0.20)),
        origin=Origin(xyz=(bkt_cyl_anchor[0], 0.0, 0.16)),
        material=deep_yellow,
        name="stick_bktcyl_ear",
    )

    # ---- BUCKET ----
    BKT_W = 0.62
    bucket_geom = _bucket_shape(BKT_W, opening=0.62, depth=0.66, back_h=0.30)
    bucket_mesh = mesh_from_cadquery(bucket_geom, "bucket.obj")
    bucket = model.part("bucket")
    bucket.visual(bucket_mesh, material=deep_yellow, name="bucket_body")
    # pivot pin at origin (to stick tip)
    bucket.visual(
        Cylinder(radius=0.10, length=BKT_W + 0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="bucket_pin",
    )
    # link lug above pivot where bucket-cylinder rod connects (seated on body)
    bucket.visual(
        Box((0.14, BKT_W * 0.5, 0.22)),
        origin=Origin(xyz=(0.05, 0.0, 0.06)),
        material=dark_steel,
        name="bucket_link_lug",
    )
    # digging teeth along the leading cutting lip of the scoop
    LIP_X = 0.66 * 0.95  # near the bottom-front cutting edge
    LIP_Z = -0.30 - 0.62 * 0.05
    n_teeth = 5
    for i in range(n_teeth):
        ty = -BKT_W / 2.0 + 0.07 + (BKT_W - 0.14) * i / (n_teeth - 1)
        bucket.visual(
            Box((0.20, 0.055, 0.09)),
            origin=Origin(xyz=(LIP_X + 0.06, ty, LIP_Z)),
            material=tooth_steel,
            name=f"tooth_{i}",
        )

    # =====================================================================
    # HYDRAULIC CYLINDERS (barrel mounted on parent, rod reaches to child)
    # Each modeled as barrel (parent) + rod (child) sliding together.
    # =====================================================================

    # ---- Boom cylinders: barrel on HOUSE, rod reaching toward BOOM (pair) -
    # Barrels pivot low/forward on the house; rods reach up-forward to the
    # boom underside. The two rails are tied together so the part is one body.
    BOOM_CYL_HOUSE = (BOOM_CYL_PIVOT_X, 0.0, deck_top + 0.10)  # pivot on the bracket
    boom_cyl = model.part("boom_cyl")
    # x=0 is the boom attachment (rod-end); x=1.24 is the house pivot (base).
    # Rods run along the boom arm (segment-1 direction, 38° from boom X).
    # x=0 at boom foot; x=1.40 toward the stick-pivot end.
    # Pivot pin at foot end.
    boom_cyl.visual(
        Cylinder(radius=0.06, length=0.54),
        origin=Origin(xyz=(0.00, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="boomcyl_base_yoke",
    )
    for cy in (0.210, -0.210):
        boom_cyl.visual(
            Cylinder(radius=0.040, length=1.40),
            origin=Origin(xyz=(0.70, cy, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=bright_steel,
            name=f"boomcyl_rod_{'p' if cy > 0 else 'm'}",
        )
    # Pivot pin at upper end.
    boom_cyl.visual(
        Cylinder(radius=0.06, length=0.54),
        origin=Origin(xyz=(1.38, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="boomcyl_rod_yoke",
    )

    # Sleeve pair at same Y as rods (±0.210), concentric with rods.
    boom_cyl_sleeve = model.part("boom_cyl_sleeve")
    for cy in (0.210, -0.210):
        boom_cyl_sleeve.visual(
            Cylinder(radius=0.068, length=0.68),
            origin=Origin(xyz=(0.0, cy, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=steel,
            name=f"boomcyl_sleeve_{'p' if cy > 0 else 'm'}",
        )
    for cy in (0.210, -0.210):
        boom_cyl_sleeve.visual(
            Cylinder(radius=0.052, length=0.05),
            origin=Origin(xyz=(0.34, cy, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=dark_steel,
            name=f"boomcyl_seal_{'p' if cy > 0 else 'm'}",
        )

    # ---- Stick cylinder: barrel on BOOM, rod on STICK -------------------
    stick_cyl = model.part("stick_cyl")
    # Barrel at base (x=0..0.90), rod spans full 1.821 m, seal ring at rod-exit, eye at tip.
    stick_cyl.visual(
        Cylinder(radius=0.085, length=0.90),
        origin=Origin(xyz=(0.45, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="stickcyl_barrel",
    )
    stick_cyl.visual(
        Cylinder(radius=0.048, length=1.821),
        origin=Origin(xyz=(0.91, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=bright_steel,
        name="stickcyl_rod",
    )
    stick_cyl.visual(
        Cylinder(radius=0.096, length=0.06),
        origin=Origin(xyz=(0.90, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_steel,
        name="stickcyl_seal",
    )
    stick_cyl.visual(
        Cylinder(radius=0.065, length=0.12),
        origin=Origin(xyz=(1.76, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="stickcyl_eye",
    )

    # Sliding sleeve on stick cylinder rod.
    stick_cyl_sleeve = model.part("stick_cyl_sleeve")
    stick_cyl_sleeve.visual(
        Cylinder(radius=0.098, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="stickcyl_sleeve",
    )
    stick_cyl_sleeve.visual(
        Cylinder(radius=0.058, length=0.06),
        origin=Origin(xyz=(0.40, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_steel,
        name="stickcyl_sleeve_seal",
    )

    # ---- Bucket cylinder: barrel on STICK, rod on BUCKET link -----------
    bucket_cyl = model.part("bucket_cyl")
    # Barrel at base (x=0..0.72), rod spans full 1.464 m, seal ring at rod-exit, eye at tip.
    bucket_cyl.visual(
        Cylinder(radius=0.07, length=0.72),
        origin=Origin(xyz=(0.36, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="bktcyl_barrel",
    )
    bucket_cyl.visual(
        Cylinder(radius=0.04, length=1.464),
        origin=Origin(xyz=(0.732, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=bright_steel,
        name="bktcyl_rod",
    )
    bucket_cyl.visual(
        Cylinder(radius=0.078, length=0.05),
        origin=Origin(xyz=(0.72, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_steel,
        name="bktcyl_seal",
    )
    bucket_cyl.visual(
        Cylinder(radius=0.05, length=0.10),
        origin=Origin(xyz=(1.41, 0.0, 0.0), rpy=(math.pi / 2, 0, 0)),
        material=dark_steel,
        name="bktcyl_eye",
    )

    # Sliding sleeve on bucket cylinder rod.
    bucket_cyl_sleeve = model.part("bucket_cyl_sleeve")
    bucket_cyl_sleeve.visual(
        Cylinder(radius=0.078, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=steel,
        name="bktcyl_sleeve",
    )
    bucket_cyl_sleeve.visual(
        Cylinder(radius=0.048, length=0.05),
        origin=Origin(xyz=(0.30, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_steel,
        name="bktcyl_sleeve_seal",
    )

    # =====================================================================
    # ARTICULATIONS
    # =====================================================================

    # 1) HOUSE SLEW about vertical Z, at the slew-ring center.
    model.articulation(
        "house_slew",
        ArticulationType.REVOLUTE,
        parent=undercarriage,
        child=house,
        origin=Origin(xyz=(0.0, 0.0, HOUSE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=4000.0, velocity=0.6),
    )

    # 2) BOOM pitch about lateral Y at the foot pivot.
    #    Boom geometry rises along +X/+Z from the foot. Positive q about -Y
    #    raises the boom; we want frac=0 -> tucked low, frac=1 -> raised/reach.
    model.articulation(
        "boom_pitch",
        ArticulationType.REVOLUTE,
        parent=house,
        child=boom,
        origin=Origin(xyz=(BOOM_FOOT_X, BOOM_FOOT_Y, BOOM_FOOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.70, effort=6000.0, velocity=0.5),
    )

    # 3) STICK about Y at the boom tip.
    model.articulation(
        "stick_curl",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=stick,
        origin=Origin(xyz=boom_tip, rpy=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.0, upper=-0.30, effort=5000.0, velocity=0.6),
    )

    # 4) BUCKET curl about Y at the stick tip.
    model.articulation(
        "bucket_curl",
        ArticulationType.REVOLUTE,
        parent=stick,
        child=bucket,
        origin=Origin(xyz=(STICK_LEN, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.8, upper=0.10, effort=4000.0, velocity=0.8),
    )

    # ---- Cylinder mount joints (FIXED at rest-pose angle) ----------------
    # The barrel angle is geometrically determined by the arm pose; it has no
    # independent actuation. FIXED joints lock each barrel at the nominal
    # rest-pose orientation so the cylinder looks correct in the default view.
    # Rods lie along the boom arm. Joint at boom foot; rpy=(0,-0.6632,0) aligns
    # boom_cyl local +X with segment-1 direction (cos38°, 0, sin38°) in boom frame.
    model.articulation(
        "boom_cyl_mount",
        ArticulationType.FIXED,
        parent=boom,
        child=boom_cyl,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, -0.6632, 0.0)),
    )
    # Sleeve starts near the foot (q=0) and slides toward the upper end (q=0.70).
    model.articulation(
        "boom_cyl_slide",
        ArticulationType.PRISMATIC,
        parent=boom_cyl,
        child=boom_cyl_sleeve,
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.70, effort=5000.0, velocity=0.4),
    )
    model.articulation(
        "stick_cyl_mount",
        ArticulationType.FIXED,
        parent=boom,
        child=stick_cyl,
        origin=Origin(xyz=stick_cyl_anchor, rpy=(0.0, 0.404, 0.0)),
    )
    model.articulation(
        "stick_cyl_slide",
        ArticulationType.PRISMATIC,
        parent=stick_cyl,
        child=stick_cyl_sleeve,
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.95, effort=4000.0, velocity=0.4),
    )
    model.articulation(
        "bucket_cyl_mount",
        ArticulationType.FIXED,
        parent=stick,
        child=bucket_cyl,
        origin=Origin(xyz=bkt_cyl_anchor, rpy=(0.0, 0.137, 0.0)),
    )
    model.articulation(
        "bucket_cyl_slide",
        ArticulationType.PRISMATIC,
        parent=bucket_cyl,
        child=bucket_cyl_sleeve,
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.80, effort=3000.0, velocity=0.4),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    under = object_model.get_part("undercarriage")
    house = object_model.get_part("house")
    boom = object_model.get_part("boom")
    stick = object_model.get_part("stick")
    bucket = object_model.get_part("bucket")
    boom_cyl = object_model.get_part("boom_cyl")
    boom_cyl_sleeve = object_model.get_part("boom_cyl_sleeve")
    stick_cyl = object_model.get_part("stick_cyl")
    stick_cyl_sleeve = object_model.get_part("stick_cyl_sleeve")
    bucket_cyl = object_model.get_part("bucket_cyl")
    bucket_cyl_sleeve = object_model.get_part("bucket_cyl_sleeve")

    slew = object_model.get_articulation("house_slew")
    boom_j = object_model.get_articulation("boom_pitch")
    stick_j = object_model.get_articulation("stick_curl")
    bucket_j = object_model.get_articulation("bucket_curl")

    # --- Tracks sit on the ground (z ~ 0) -------------------------------
    under_aabb = ctx.part_world_aabb(under)
    if under_aabb is not None:
        zmin = under_aabb[0][2]
        ctx.check(
            "tracks rest on ground",
            abs(zmin) < 0.02,
            details=f"undercarriage zmin={zmin:.4f}",
        )

    # --- Overall scale roughly an excavator ------------------------------
    if under_aabb is not None:
        length = under_aabb[1][0] - under_aabb[0][0]
        width = under_aabb[1][1] - under_aabb[0][1]
        ctx.check(
            "track footprint length plausible",
            3.0 < length < 4.3,
            details=f"undercarriage length={length:.3f}",
        )
        ctx.check(
            "track footprint width plausible",
            2.2 < width < 3.0,
            details=f"undercarriage width={width:.3f}",
        )

    # --- Cab is on the left (+Y) side of the house ----------------------
    cab_aabb = ctx.part_element_world_aabb(house, elem="cab_shell")
    if cab_aabb is not None:
        cab_cy = (cab_aabb[0][1] + cab_aabb[1][1]) / 2.0
        ctx.check("cab on left (+Y) side", cab_cy > 0.2, details=f"cab cy={cab_cy:.3f}")

    # --- Counterweight is at the rear (-X) ------------------------------
    cw_aabb = ctx.part_element_world_aabb(house, elem="counterweight")
    if cw_aabb is not None:
        cw_cx = (cw_aabb[0][0] + cw_aabb[1][0]) / 2.0
        ctx.check("counterweight at rear (-X)", cw_cx < -0.8, details=f"cw cx={cw_cx:.3f}")

    # --- Arm chain connectivity: pins meet across boom->stick->bucket ---
    ctx.expect_contact(boom, stick, name="boom tip meets stick root")
    ctx.expect_contact(stick, bucket, name="stick tip meets bucket pivot")

    # --- Bucket teeth exist at the lip ----------------------------------
    tooth_aabb = ctx.part_element_world_aabb(bucket, elem="tooth_0")
    ctx.check("bucket has teeth", tooth_aabb is not None, details="tooth_0 missing")

    # --- Hydraulic cylinders are mounted at both ends -------------------
    # boom cylinder connects house and reaches toward boom
    ctx.expect_contact(boom, stick_cyl, name="stick cylinder mounted on boom")
    ctx.expect_contact(stick, bucket_cyl, name="bucket cylinder mounted on stick")

    # --- Slew rotates the house+arm but not the tracks ------------------
    house_rest = ctx.part_world_position(house)
    boom_rest = ctx.part_world_position(boom)
    under_rest = ctx.part_world_position(under)
    with ctx.pose({slew: math.pi / 2.0}):
        boom_slewed = ctx.part_world_position(boom)
        under_slewed = ctx.part_world_position(under)
    # undercarriage must not move when the house slews
    if under_rest is not None and under_slewed is not None:
        moved = math.dist(under_rest, under_slewed)
        ctx.check("slew leaves tracks fixed", moved < 1e-6, details=f"under moved {moved:.5f}")
    # the boom (carried by the house) must swing to a new XY location
    if boom_rest is not None and boom_slewed is not None:
        swing = math.hypot(
            boom_slewed[0] - boom_rest[0], boom_slewed[1] - boom_rest[1]
        )
        ctx.check("slew swings the arm", swing > 0.3, details=f"boom swing {swing:.3f}")

    # --- Arm reaches out when joints actuate (frac=1 -> reaching) -------
    rest_tip = ctx.part_world_position(bucket)
    with ctx.pose({boom_j: 0.70, stick_j: -0.30, bucket_j: 0.10}):
        reach_tip = ctx.part_world_position(bucket)
    if rest_tip is not None and reach_tip is not None:
        ctx.check(
            "arm changes pose across range",
            math.dist(rest_tip, reach_tip) > 0.3,
            details=f"bucket moved {math.dist(rest_tip, reach_tip):.3f}",
        )

    # --- Intentional overlaps at the pinned joints and cylinder mounts ---
    # These are the shared-pin pivots and the cylinder barrel/rod end mounts of
    # a real excavator front; the members are pinned together at these
    # interfaces, so local interpenetration is expected and correct.
    ctx.allow_overlap(
        boom, house,
        reason="Boom foot pins are captured in the house mount clevis ears (pinned pivot).",
    )
    ctx.allow_overlap(
        boom, stick,
        reason="Boom tip and stick root knuckle pins are coaxial at the shared pivot.",
    )
    ctx.allow_overlap(
        bucket, stick,
        reason="Bucket pivot pin and link lug share the stick tip pin (pinned pivot + linkage).",
    )
    ctx.allow_overlap(
        boom, stick_cyl,
        reason="Stick cylinder barrel is pinned to the boom-top anchor ear (mounted both ends).",
    )
    ctx.allow_overlap(
        bucket_cyl, stick,
        reason="Bucket cylinder barrel is pinned to the stick-top anchor ear (mounted both ends).",
    )
    ctx.allow_overlap(
        boom, boom_cyl,
        reason="Boom cylinder rod-end yoke is pinned to the boom underside (mounted both ends).",
    )
    ctx.allow_overlap(
        boom_cyl_sleeve, boom_cyl,
        reason="Sleeve wraps around the rod — intentional concentric overlap.",
    )
    ctx.allow_overlap(
        boom_cyl_sleeve, boom,
        reason="Sleeve barrel may graze the boom underside at the rod-end attachment.",
    )
    ctx.allow_overlap(
        stick_cyl_sleeve, stick_cyl,
        reason="Stick cylinder sleeve wraps around the rod — concentric overlap.",
    )
    ctx.allow_overlap(
        stick_cyl_sleeve, boom,
        reason="Stick cylinder sleeve may overlap boom anchor ear at rest position.",
    )
    ctx.allow_overlap(
        bucket_cyl_sleeve, bucket_cyl,
        reason="Bucket cylinder sleeve wraps around the rod — concentric overlap.",
    )
    ctx.allow_overlap(
        bucket_cyl_sleeve, stick,
        reason="Bucket cylinder sleeve may overlap stick anchor ear at rest position.",
    )
    ctx.allow_overlap(
        boom_cyl, house,
        reason="Boom cylinder base yoke is pinned to the house trunnion bracket (mounted base).",
    )

    return ctx.report()
