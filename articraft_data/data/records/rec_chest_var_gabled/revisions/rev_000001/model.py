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
R = D / 2.0       # half-depth (used for lid frame offsets)
GABLE_PEAK = 0.10 # peak rise above the seam (Z)


def _gabled_profile(depth, peak_h):
    """Triangular gabled cross-section in the extrusion profile plane.

    After the standard rotate_z(pi/2) + rotate_y(pi/2) + translate transform,
    these profile points map to lid-local coordinates where:
      - (R, 0)       → rear/hinge edge at Y=0, Z=0
      - (-R, 0)      → front edge at Y=-D, Z=0
      - (0, peak_h)  → ridge peak at Y=-D/2, Z=peak_h
    """
    half = depth / 2.0
    return [
        (half, 0.0),
        (-half, 0.0),
        (0.0, peak_h),
    ]


def _gabled_panel(depth, peak_h, width, x_center=0.0):
    """Solid gabled triangular prism, authored in the lid-local frame
    (hinge at rear seam: local Y in [-D, 0])."""
    profile = _gabled_profile(depth, peak_h)
    geo = ExtrudeGeometry.from_z0(profile, width, cap=True)
    geo.rotate_z(math.pi / 2.0)
    geo.rotate_y(math.pi / 2.0)
    geo.translate(x_center - width / 2.0, -depth / 2.0, 0.0)
    return geo


def _gabled_strap(depth, peak_h, width, x_center=0.0, proud=0.004):
    """Iron strap wrapping over the gabled surface (slightly proud)."""
    half = depth / 2.0
    profile = [
        (half + proud, -proud),
        (-(half + proud), -proud),
        (0.0, peak_h + proud),
    ]
    geo = ExtrudeGeometry.from_z0(profile, width, cap=True)
    geo.rotate_z(math.pi / 2.0)
    geo.rotate_y(math.pi / 2.0)
    geo.translate(x_center - width / 2.0, -depth / 2.0, 0.0)
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

    # horizontal iron bands on the long faces
    for sx in (-0.12, 0.12):
        body.visual(Box((0.03, D + 0.004, 0.006)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.5)),
                    material=iron, name=f"body_band_{'l' if sx < 0 else 'r'}")

    # front lock keeper that receives the hasp
    body.visual(Box((0.05, 0.012, 0.035)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.006, HB * 0.55)),
                material=iron, name="lock_keeper")

    # --- Gabled peaked lid (hinged at the rear seam) -------------------------
    lid = model.part("chest_lid")

    # Main gabled wooden panel (triangular prism spanning full width)
    lid_panel = _gabled_panel(D, GABLE_PEAK, W)
    lid.visual(mesh_from_geometry(lid_panel, "lid_panel"), material=wood, name="lid_panel")

    # Iron straps wrapping over the gabled slopes
    strap_positions = (-W / 2.0 + 0.016, -0.13, 0.0, 0.13, W / 2.0 - 0.016)
    for i, x0 in enumerate(strap_positions):
        strap = _gabled_strap(D, GABLE_PEAK, 0.028, x_center=x0, proud=0.004)
        lid.visual(mesh_from_geometry(strap, f"lid_strap_{i}"), material=iron,
                   name=f"lid_strap_{i}")

    # Ridge cap - iron strip running along the peak
    lid.visual(Box((W - 0.02, 0.032, 0.006)),
               origin=Origin(xyz=(0.0, -D / 2.0, GABLE_PEAK + 0.003)),
               material=iron, name="ridge_cap")

    # Front face skirt (vertical board at the gable front edge, proud of body)
    lid.visual(Box((W - 0.02, 0.008, 0.04)),
               origin=Origin(xyz=(0.0, -D - 0.005, -0.02)),
               material=wood_dark, name="lid_front_skirt")

    # Hasp mount tab on the front skirt (proud of body wall)
    lid.visual(Box((0.05, 0.012, 0.025)),
               origin=Origin(xyz=(0.0, -D - 0.008, -0.015)),
               material=iron, name="hasp_mount")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, R, HB)),
        # closed lid extends along local -Y; -X axis lifts the front edge up.
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
        origin=Origin(xyz=(0.0, -D - 0.010, -0.015)),
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

    # --- Gabled profile verification ---
    # The ridge peak must be higher than the side edges (proves peaked shape)
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ridge_cap_aabb = ctx.part_element_world_aabb(lid, elem="ridge_cap")
        closed_lid = ctx.part_world_aabb(lid)

    ctx.check(
        "lid_panel exists and has valid bounds",
        lid_aabb is not None and lid_aabb[1][2] > HB,
        details=f"lid_panel_aabb={lid_aabb}",
    )

    # Ridge cap sits at the peak - proves the gabled peaked shape
    ctx.check(
        "ridge peak is above seam by expected gable height",
        ridge_cap_aabb is not None
        and ridge_cap_aabb[1][2] > HB + GABLE_PEAK * 0.8,
        details=f"ridge_cap_aabb={ridge_cap_aabb}",
    )

    # The ridge peak must be higher than the front skirt (proves two slopes)
    front_skirt_aabb = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    ctx.check(
        "gabled peak rises above front edge (two-slope profile)",
        ridge_cap_aabb is not None and front_skirt_aabb is not None
        and ridge_cap_aabb[1][2] > front_skirt_aabb[1][2] + 0.04,
        details=f"ridge_cap_top={ridge_cap_aabb[1][2] if ridge_cap_aabb else None}, "
                f"skirt_top={front_skirt_aabb[1][2] if front_skirt_aabb else None}",
    )

    # Closed lid seats on body rim
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=-0.005, max_gap=0.006,
                       positive_elem="lid_panel", negative_elem="wall_front",
                       name="gabled lid seats on the body rim")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.20,
                           name="closed lid covers the body opening")

    # Lid opens upward
    with ctx.pose({lid_hinge: 1.7}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on hinge",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.08,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    # Front lock hasp lifts to release
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

    # Iron straps present on the lid slopes
    strap_names = [f"lid_strap_{i}" for i in range(5)]
    for sn in strap_names:
        sa = ctx.part_element_world_aabb(lid, elem=sn)
        ctx.check(
            f"iron strap {sn} present on gabled lid",
            sa is not None and sa[1][2] > HB + GABLE_PEAK * 0.5,
            details=f"strap_aabb={sa}",
        )

    # --- Intentional overlap allowances ---
    # Hasp arm rests against the lock keeper when closed (engagement)
    ctx.allow_overlap(
        body, hasp,
        elem_a="lock_keeper", elem_b="hasp_arm",
        reason="The hasp arm reaches down to engage the lock keeper; small contact overlap at the latch interface.",
    )
    ctx.expect_contact(
        hasp, body,
        elem_a="hasp_arm", elem_b="lock_keeper",
        contact_tol=0.012,
        name="hasp arm contacts lock keeper when closed",
    )

    # Hasp arm pivots from the hasp mount (mounting overlap at hinge point)
    ctx.allow_overlap(
        lid, hasp,
        elem_a="hasp_mount", elem_b="hasp_arm",
        reason="The hasp arm pivots from the hasp mount tab; small overlap at the hinge connection.",
    )
    ctx.expect_contact(
        hasp, lid,
        elem_a="hasp_arm", elem_b="hasp_mount",
        contact_tol=0.010,
        name="hasp arm seated on hasp mount",
    )

    return ctx.report()
