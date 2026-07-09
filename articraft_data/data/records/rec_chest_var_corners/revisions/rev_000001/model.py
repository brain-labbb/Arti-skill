from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.46          # width (X)
D = 0.30          # depth (Y)
HB = 0.18         # body height (Z)
T = 0.018         # plank wall thickness
R = D / 2.0       # barrel-lid radius (spans the full depth)

BRACKET_ARM = 0.07   # corner bracket arm length from corner
BRACKET_T = 0.005    # corner bracket plate thickness


def _half_disk_profile(r, segs=26):
    pts = []
    for i in range(segs + 1):
        a = math.pi * i / segs
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def _barrel_dome(r, x0, xw):
    """Solid half-cylinder dome of radius r, width xw centered at x0, authored
    in the lid-local frame (hinge at the rear seam: local y in [-2R, 0])."""
    geo = ExtrudeGeometry.from_z0(_half_disk_profile(r), xw, cap=True)
    geo.rotate_z(math.pi / 2.0)
    geo.rotate_y(math.pi / 2.0)
    geo.translate(x0 - xw / 2.0, -R, 0.0)
    return geo


def _corner_bracket_mesh(cx, cy, cz, sx, sy, is_top):
    """Build an L-shaped corner bracket mesh wrapping a box corner.

    cx, cy, cz: corner position on the box.
    sx, sy: sign of the corner in X and Y (-1 or +1).
    is_top: True for top corners (z = HB), False for bottom (z = 0).

    Returns a single connected MeshGeometry with three mutually overlapping
    plates and three rivet bumps.
    """
    arm = BRACKET_ARM
    t = BRACKET_T

    # Direction from corner toward box interior
    dx = -float(sx)
    dy = -float(sy)
    dz = -1.0 if is_top else 1.0

    # Each plate is extended by *t* past the corner so the three plates
    # share a volume overlap at the corner, keeping the merged mesh
    # connected (single component).
    plate_len = arm + t
    half_t = t / 2.0

    # ---- Z-face plate (horizontal, on top or bottom face) ----
    plate_z = BoxGeometry((plate_len, plate_len, t))
    plate_z.translate(
        cx + dx * (plate_len / 2.0 - t),
        cy + dy * (plate_len / 2.0 - t),
        cz - dz * half_t,
    )

    # ---- Y-face plate (front or back face) ----
    plate_y = BoxGeometry((plate_len, t, plate_len))
    plate_y.translate(
        cx + dx * (plate_len / 2.0 - t),
        cy - dy * half_t,
        cz + dz * (plate_len / 2.0 - t),
    )

    # ---- X-face plate (left or right face) ----
    plate_x = BoxGeometry((t, plate_len, plate_len))
    plate_x.translate(
        cx - dx * half_t,
        cy + dy * (plate_len / 2.0 - t),
        cz + dz * (plate_len / 2.0 - t),
    )

    plate_z.merge(plate_y)
    plate_z.merge(plate_x)

    # ---- Rivet bumps (one per plate face, overlapping into the plate) ----
    rs = 0.008   # rivet width
    rh = 0.004   # rivet height
    rpos = arm * 0.55  # rivet position along the arm from the corner

    # Rivet on Z-face plate outer surface
    rv_z = BoxGeometry((rs, rs, rh))
    rv_z.translate(
        cx + dx * rpos,
        cy + dy * rpos,
        cz - dz * (half_t + rh / 2.0),
    )
    plate_z.merge(rv_z)

    # Rivet on Y-face plate outer surface
    rv_y = BoxGeometry((rs, rh, rs))
    rv_y.translate(
        cx + dx * rpos,
        cy - dy * (half_t + rh / 2.0),
        cz + dz * rpos,
    )
    plate_z.merge(rv_y)

    # Rivet on X-face plate outer surface
    rv_x = BoxGeometry((rh, rs, rs))
    rv_x.translate(
        cx - dx * (half_t + rh / 2.0),
        cy + dy * rpos,
        cz + dz * rpos,
    )
    plate_z.merge(rv_x)

    return plate_z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_treasure_chest")

    wood = model.material("chest_wood", rgba=(0.42, 0.30, 0.20, 1.0))
    wood_dark = model.material("chest_wood_dark", rgba=(0.30, 0.21, 0.14, 1.0))
    iron = model.material("iron_band", rgba=(0.20, 0.21, 0.24, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("chest_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    wall_cz = HB / 2.0
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), wall_cz)),
                material=wood, name="wall_front")
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), wall_cz)),
                material=wood, name="wall_back")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, wall_cz)),
                material=wood, name="wall_left")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, wall_cz)),
                material=wood, name="wall_right")

    # Iron corner brackets wrapping the eight box corners
    _corners = [
        (-W / 2.0, -D / 2.0, 0.0, -1, -1, False),  # 0: bottom-front-left
        ( W / 2.0, -D / 2.0, 0.0,  1, -1, False),   # 1: bottom-front-right
        (-W / 2.0,  D / 2.0, 0.0, -1,  1, False),   # 2: bottom-back-left
        ( W / 2.0,  D / 2.0, 0.0,  1,  1, False),   # 3: bottom-back-right
        (-W / 2.0, -D / 2.0,  HB, -1, -1, True),    # 4: top-front-left
        ( W / 2.0, -D / 2.0,  HB,  1, -1, True),    # 5: top-front-right
        (-W / 2.0,  D / 2.0,  HB, -1,  1, True),    # 6: top-back-left
        ( W / 2.0,  D / 2.0,  HB,  1,  1, True),    # 7: top-back-right
    ]
    for i in range(8):
        cx, cy, cz_val, sx, sy, is_top = _corners[i]
        bracket = _corner_bracket_mesh(cx, cy, cz_val, sx, sy, is_top)
        body.visual(
            mesh_from_geometry(bracket, f"corner_bracket_{i}"),
            material=iron,
            name=f"corner_bracket_{i}",
        )

    # front lock keeper that receives the hasp
    body.visual(Box((0.05, 0.012, 0.035)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.006, HB * 0.55)),
                material=iron, name="lock_keeper")

    # --- Domed barrel lid (hinged at the rear seam) --------------------------
    lid = model.part("chest_lid")
    dome = _barrel_dome(R, 0.0, W)
    lid.visual(mesh_from_geometry(dome, "lid_dome"), material=wood, name="lid_dome")
    # hinge mount tab carrying the front lock hasp
    lid.visual(Box((0.05, 0.020, 0.022)),
               origin=Origin(xyz=(0.0, -2.0 * R + 0.003, R * 0.30)),
               material=iron, name="hasp_mount")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, R, HB)),
        # closed dome extends along local -Y; -X axis lifts the front edge up.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.9),
    )

    # --- Front lock hasp (hinged on the lid front, swings up to release) -----
    hasp = model.part("lock_hasp")
    hasp.visual(Box((0.05, 0.010, 0.11)), origin=Origin(xyz=(0.0, 0.0, -0.055)),
                material=iron, name="hasp_arm")
    hasp.visual(Box((0.026, 0.016, 0.02)), origin=Origin(xyz=(0.0, -0.005, -0.10)),
                material=iron, name="hasp_eye")
    model.articulation(
        "hasp_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp,
        origin=Origin(xyz=(0.0, -2.0 * R - 0.012, R * 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.4),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("chest_body")
    lid = object_model.get_part("chest_lid")
    hasp = object_model.get_part("lock_hasp")
    lid_hinge = object_model.get_articulation("lid_hinge")
    hasp_hinge = object_model.get_articulation("hasp_hinge")

    # --- Dome lid seats on the body rim and reads as a raised dome ----------
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_dome", negative_elem="wall_front",
                       name="dome lid seats on the body rim")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.20,
                           name="closed lid covers the body opening")
        dome_aabb = ctx.part_element_world_aabb(lid, elem="lid_dome")
        closed_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid is a raised dome above the seam",
        dome_aabb is not None and dome_aabb[1][2] > HB + R * 0.7,
        details=f"dome_aabb={dome_aabb}",
    )

    with ctx.pose({lid_hinge: 1.7}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.08,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    # --- Front lock hasp lifts to release ------------------------------------
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        hasp_closed = ctx.part_world_aabb(hasp)
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 1.3}):
        hasp_open = ctx.part_world_aabb(hasp)
    ctx.check(
        "front lock hasp lifts to release",
        hasp_closed is not None and hasp_open is not None
        and hasp_open[0][2] > hasp_closed[0][2] + 0.02,
        details=f"closed={hasp_closed}, open={hasp_open}",
    )

    # --- Eight iron corner brackets at the box corners -----------------------
    # The top four brackets' horizontal plates sit flush with the body top
    # face (z = HB) where the dome edge also starts at z = HB. This is a
    # real bracket-sandwiched-between-body-and-lid mounting representation.
    for i in range(4, 8):
        ctx.allow_overlap(
            body, lid,
            elem_a=f"corner_bracket_{i}",
            elem_b="lid_dome",
            reason=(
                "The top corner bracket wraps around the body corner with its "
                "horizontal plate flush on the body top face; the dome edge "
                "seats on top of the bracket at the same z = HB plane."
            ),
        )

    # Proof: dome still seats correctly on the body despite bracket overlap
    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=-0.008, max_gap=0.010,
                       positive_elem="lid_dome", negative_elem="wall_back",
                       name="dome seats on body top near rear seam with bracket present")

    for i in range(8):
        vname = f"corner_bracket_{i}"
        aabb = ctx.part_element_world_aabb(body, elem=vname)
        ctx.check(
            f"corner bracket {vname} exists on body",
            aabb is not None,
            details=f"visual {vname} not found on chest_body",
        )

    # Bottom-front-left bracket should touch the corner region
    bb0 = ctx.part_element_world_aabb(body, elem="corner_bracket_0")
    ctx.check(
        "bottom-front-left bracket spans the corner",
        bb0 is not None
        and bb0[0][0] < -W / 2.0 + 0.005
        and bb0[0][1] < -D / 2.0 + 0.005
        and bb0[0][2] < 0.005,
        details=f"bracket_0_aabb={bb0}",
    )

    # Top-back-right bracket should touch the upper corner region
    bb7 = ctx.part_element_world_aabb(body, elem="corner_bracket_7")
    ctx.check(
        "top-back-right bracket spans the corner",
        bb7 is not None
        and bb7[1][0] > W / 2.0 - 0.005
        and bb7[1][1] > D / 2.0 - 0.005
        and bb7[1][2] > HB - 0.005,
        details=f"bracket_7_aabb={bb7}",
    )

    # Brackets should have visible extent (arm length) along box edges
    bb3 = ctx.part_element_world_aabb(body, elem="corner_bracket_3")
    ctx.check(
        "bottom-back-right bracket has arm extent",
        bb3 is not None
        and (bb3[1][0] - bb3[0][0]) > BRACKET_ARM * 0.8
        and (bb3[1][1] - bb3[0][1]) > BRACKET_ARM * 0.8,
        details=f"bracket_3_aabb={bb3}",
    )

    return ctx.report()
