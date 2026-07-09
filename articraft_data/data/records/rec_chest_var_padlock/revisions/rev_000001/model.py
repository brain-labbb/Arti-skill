from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
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


def _u_bar(w, h, t, depth):
    """U-shaped bar profile opening downward (gap at z=0 after rotation).
    Profile is built in XY, extruded along Z, then rotated so the profile
    lies in XZ and the extrusion goes along -Y."""
    profile = [
        (-w / 2.0, 0.0),
        (-w / 2.0, h),
        (w / 2.0, h),
        (w / 2.0, 0.0),
        (w / 2.0 - t, 0.0),
        (w / 2.0 - t, h - t),
        (-(w / 2.0 - t), h - t),
        (-(w / 2.0 - t), 0.0),
    ]
    geo = ExtrudeGeometry.from_z0(profile, depth, cap=True)
    # Rotate so profile is in XZ and extrusion goes along -Y
    geo.rotate_x(math.pi / 2.0)
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_treasure_chest")

    wood = model.material("chest_wood", rgba=(0.42, 0.30, 0.20, 1.0))
    wood_dark = model.material("chest_wood_dark", rgba=(0.30, 0.21, 0.14, 1.0))
    iron = model.material("iron_band", rgba=(0.20, 0.21, 0.24, 1.0))
    iron_dark = model.material("iron_dark", rgba=(0.16, 0.17, 0.20, 1.0))
    brass = model.material("brass_lock", rgba=(0.60, 0.50, 0.22, 1.0))

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

    # vertical iron bands on the long faces
    for sx in (-0.12, 0.12):
        body.visual(Box((0.03, D + 0.004, 0.006)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.5)),
                    material=iron, name=f"body_band_{'l' if sx < 0 else 'r'}")

    # --- Staple loop (fixed on body front) -----------------------------------
    staple_w = 0.044
    staple_h = 0.038
    staple_t = 0.008
    staple_d = 0.020
    staple_z_top = HB * 0.62

    staple_geo = _u_bar(staple_w, staple_h, staple_t, staple_d)
    # Position: back face at front wall, crossbar at staple_z_top
    staple_geo.translate(0.0, -D / 2.0, staple_z_top - staple_h)
    body.visual(mesh_from_geometry(staple_geo, "staple_loop"),
                material=iron_dark, name="staple_loop")

    # small mounting plate behind the staple on the front wall
    body.visual(Box((staple_w + 0.012, 0.006, staple_h + 0.010)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - 0.003), staple_z_top - staple_h / 2.0)),
                material=iron, name="staple_plate")

    # --- Domed barrel lid (hinged at the rear seam) --------------------------
    lid = model.part("chest_lid")
    dome = _barrel_band(R, 0.0, W)
    lid.visual(mesh_from_geometry(dome, "lid_dome"), material=wood, name="lid_dome")
    # iron straps wrapping over the dome (slightly larger radius)
    for i, x0 in enumerate((-W / 2.0 + 0.016, -0.13, 0.0, 0.13, W / 2.0 - 0.016)):
        strap = _barrel_band(R + 0.004, x0, 0.028)
        lid.visual(mesh_from_geometry(strap, f"lid_strap_{i}"), material=iron,
                   name=f"lid_strap_{i}")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, R, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.9),
    )

    # --- Padlock body (hangs from the staple) --------------------------------
    padlock_w = 0.042
    padlock_d = 0.020
    padlock_h = 0.050

    # Padlock body position: hangs below the staple, centered on staple
    padlock_body_top_z = staple_z_top - staple_h - 0.003  # small gap below staple legs
    padlock_body_cz = padlock_body_top_z - padlock_h / 2.0
    padlock_body_cy = -D / 2.0 - staple_d / 2.0

    padlock = model.part("padlock_body")

    # CadQuery rounded box for the padlock body
    padlock_shape = (
        cq.Workplane("XY")
        .box(padlock_w, padlock_d, padlock_h)
        .edges("|Z").fillet(0.005)
        .edges("#Z").fillet(0.003)
    )
    padlock.visual(
        mesh_from_cadquery(padlock_shape, "padlock_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass, name="padlock_case",
    )

    # keyhole plate on front face
    padlock.visual(
        Box((0.012, 0.003, 0.016)),
        origin=Origin(xyz=(0.0, -padlock_d / 2.0 - 0.001, -0.006)),
        material=iron_dark, name="keyhole_plate",
    )

    # Fix padlock body to the chest body (hangs from the staple)
    model.articulation(
        "body_to_padlock",
        ArticulationType.FIXED,
        parent=body,
        child=padlock,
        origin=Origin(xyz=(0.0, padlock_body_cy, padlock_body_cz)),
    )

    # --- Shackle (U-bar through staple, pivots on padlock body) --------------
    shackle_w = 0.030
    shackle_h = 0.042
    shackle_t = 0.006
    shackle_d = 0.014

    shackle = model.part("padlock_shackle")

    # The shackle pivot is at the left leg center, at the top of the padlock body.
    # Pivot position in world:
    pivot_x = -shackle_w / 2.0 + shackle_t / 2.0
    pivot_y = padlock_body_cy
    pivot_z = padlock_body_top_z

    # Build shackle U-shape in the shackle part's local frame.
    # The part origin is at the pivot point (left leg center at padlock body top).
    # The U opens downward; the crossbar is at z=shackle_h above the pivot.
    shackle_geo = _u_bar(shackle_w, shackle_h, shackle_t, shackle_d)

    # Shift so the pivot (left leg center at U bottom) is at the part origin:
    shackle_geo.translate(
        shackle_w / 2.0 - shackle_t / 2.0,
        shackle_d / 2.0,
        0.0,
    )

    shackle.visual(
        mesh_from_geometry(shackle_geo, "shackle_bow"),
        material=iron_dark, name="shackle_bow",
    )

    # Leg extensions below the pivot (going into the padlock body)
    leg_insert = 0.014  # how far legs go into the body below pivot
    # Fixed leg (at pivot, x=0)
    shackle.visual(
        Cylinder(shackle_t / 2.0, leg_insert),
        origin=Origin(xyz=(0.0, 0.0, -leg_insert / 2.0)),
        material=iron_dark, name="shackle_leg_fixed",
    )
    # Free leg (at x = shackle_w - shackle_t from pivot)
    shackle.visual(
        Cylinder(shackle_t / 2.0, leg_insert),
        origin=Origin(xyz=(shackle_w - shackle_t, 0.0, -leg_insert / 2.0)),
        material=iron_dark, name="shackle_leg_free",
    )

    # Articulation: shackle pivots around Z at the fixed leg
    # Origin is in the padlock part frame: pivot is at top of padlock body,
    # offset to the left leg position.
    model.articulation(
        "shackle_hinge",
        ArticulationType.REVOLUTE,
        parent=padlock,
        child=shackle,
        origin=Origin(xyz=(pivot_x, 0.0, padlock_h / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=2.4),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("chest_body")
    lid = object_model.get_part("chest_lid")
    padlock = object_model.get_part("padlock_body")
    shackle = object_model.get_part("padlock_shackle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    shackle_hinge = object_model.get_articulation("shackle_hinge")

    # --- Dome lid checks ---
    with ctx.pose({lid_hinge: 0.0}):
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

    # --- Padlock body checks ---
    ctx.expect_overlap(padlock, body, axes="xy", min_overlap=0.01,
                       name="padlock body overlaps with body front region")
    padlock_aabb = ctx.part_world_aabb(padlock)
    staple_aabb = ctx.part_element_world_aabb(body, elem="staple_loop")
    ctx.check(
        "padlock body hangs below the staple",
        padlock_aabb is not None and staple_aabb is not None
        and padlock_aabb[0][2] < staple_aabb[0][2],
        details=f"padlock_aabb={padlock_aabb}, staple_aabb={staple_aabb}",
    )

    # --- Shackle checks ---
    # Shackle closed: overlaps with staple (passes through the loop)
    with ctx.pose({shackle_hinge: 0.0}):
        shackle_closed = ctx.part_world_aabb(shackle)
        ctx.expect_overlap(shackle, body, axes="xy", min_overlap=0.005,
                           elem_a="shackle_bow", elem_b="staple_loop",
                           name="closed shackle passes through staple loop")

    # Shackle open: free end swings away
    with ctx.pose({shackle_hinge: 2.0}):
        shackle_open = ctx.part_world_aabb(shackle)

    ctx.check(
        "shackle opens by swinging the free end outward",
        shackle_closed is not None and shackle_open is not None
        and (shackle_open[1][1] - shackle_open[0][1]) > (shackle_closed[1][1] - shackle_closed[0][1]) + 0.01,
        details=f"closed={shackle_closed}, open={shackle_open}",
    )

    # Shackle has a real revolute joint
    ctx.check(
        "shackle hinge is revolute with positive range",
        shackle_hinge.articulation_type == ArticulationType.REVOLUTE
        and shackle_hinge.motion_limits.upper > 0.5,
        details=f"type={shackle_hinge.articulation_type}, upper={shackle_hinge.motion_limits.upper}",
    )

    # --- Intentional overlap allowances ---
    # The shackle passes through the staple loop (intentional nesting)
    ctx.allow_overlap(
        body, shackle,
        elem_a="staple_loop", elem_b="shackle_bow",
        reason="The shackle is intentionally represented as passing through the staple loop.",
    )
    ctx.allow_overlap(
        padlock, shackle,
        elem_a="padlock_case", elem_b="shackle_leg_fixed",
        reason="The fixed shackle leg is intentionally inserted into the padlock body.",
    )
    ctx.allow_overlap(
        padlock, shackle,
        elem_a="padlock_case", elem_b="shackle_leg_free",
        reason="The free shackle leg is intentionally inserted into the padlock body when closed.",
    )

    # Prove the shackle is retained in the padlock body
    ctx.expect_within(
        shackle, padlock,
        axes="xy",
        inner_elem="shackle_leg_fixed", outer_elem="padlock_case",
        margin=0.005,
        name="fixed shackle leg stays within padlock body footprint",
    )

    return ctx.report()
