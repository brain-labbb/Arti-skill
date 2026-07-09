from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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


def _half_disk_profile(r, segs=26):
    pts = []
    for i in range(segs + 1):
        a = math.pi * i / segs
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def _barrel_band(r, x0, xw):
    """Solid half-cylinder band of radius r, width xw centered at x0, authored
    in the lid-local frame (hinge at the rear seam: local y in [-2R, 0])."""
    geo = ExtrudeGeometry.from_z0(_half_disk_profile(r), xw, cap=True)
    geo.rotate_z(math.pi / 2.0)
    geo.rotate_y(math.pi / 2.0)
    geo.translate(x0 - xw / 2.0, -R, 0.0)
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_treasure_chest")

    wood = model.material("chest_wood", rgba=(0.42, 0.30, 0.20, 1.0))
    wood_dark = model.material("chest_wood_dark", rgba=(0.30, 0.21, 0.14, 1.0))
    iron = model.material("iron_band", rgba=(0.20, 0.21, 0.24, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("chest_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    cz = HB / 2.0
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_front")
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_back")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_left")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_right")

    # iron corner posts on the four vertical edges
    bk = 0.034
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((bk, bk, HB)),
                origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.003),
                                   sy * (D / 2.0 - bk / 2.0 + 0.003), HB / 2.0)),
                material=iron, name=f"corner_post_{idx}")
            idx += 1

    # vertical iron bands on the long faces (with rivet caps)
    for sx in (-0.12, 0.12):
        body.visual(Box((0.03, D + 0.004, 0.006)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.5)),
                    material=iron, name=f"body_band_{'l' if sx < 0 else 'r'}")

    # front lock keeper that receives the hasp
    body.visual(Box((0.05, 0.012, 0.035)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.006, HB * 0.55)),
                material=iron, name="lock_keeper")

    # --- Domed barrel lid (hinged at the rear seam) --------------------------
    lid = model.part("chest_lid")
    dome = _barrel_band(R, 0.0, W)
    lid.visual(mesh_from_geometry(dome, "lid_dome"), material=wood, name="lid_dome")
    # iron straps wrapping over the dome (slightly larger radius)
    for i, x0 in enumerate((-W / 2.0 + 0.016, -0.13, 0.0, 0.13, W / 2.0 - 0.016)):
        strap = _barrel_band(R + 0.004, x0, 0.028)
        lid.visual(mesh_from_geometry(strap, f"lid_strap_{i}"), material=iron,
                   name=f"lid_strap_{i}")
    # hinge mount tab carrying the front lock hasp (reaches out to contact it)
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

    # Closed dome lid seats on the body rim and reads as a raised dome.
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

    return ctx.report()
