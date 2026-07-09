from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.42          # width (X)
D = 0.40          # depth (Y)
HB = 0.30         # body height (Z)
T = 0.018         # plank wall / rail thickness
POST = 0.030      # corner post size
LID_T = 0.026     # flat lid thickness
SEAM_GAP = 0.0    # lid rests on the body rim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apple_crate_storage_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")

    post_z = T + (HB - T) / 2.0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((POST, POST, HB - T)),
                origin=Origin(
                    xyz=(
                        sx * (W / 2.0 - POST / 2.0),
                        sy * (D / 2.0 - POST / 2.0),
                        post_z,
                    )
                ),
                material=wood_dark,
                name=f"corner_post_{int((sx + 1) * 0.5)}_{int((sy + 1) * 0.5)}",
            )

    rail_h = 0.045
    mid_h = 0.032
    slot_h = 0.062
    bottom_rail_z = T + rail_h / 2.0
    middle_rail_z = T + rail_h + slot_h + mid_h / 2.0
    top_rail_z = HB - rail_h / 2.0

    front_back_span = W - 2.0 * POST
    side_span = D - 2.0 * POST
    wall_y = D / 2.0 - T / 2.0
    wall_x = W / 2.0 - T / 2.0

    for side, sy in (("front", -1.0), ("back", 1.0)):
        y = sy * wall_y
        for z, h, tag in (
            (bottom_rail_z, rail_h, "bottom"),
            (middle_rail_z, mid_h, "middle"),
            (top_rail_z, rail_h, "top"),
        ):
            body.visual(
                Box((front_back_span, T, h)),
                origin=Origin(xyz=(0.0, y, z)),
                material=wood,
                name=f"{side}_rail_{tag}",
            )
        body.visual(
            Box((POST, T, top_rail_z - bottom_rail_z + rail_h)),
            origin=Origin(xyz=(0.0, y, (top_rail_z + bottom_rail_z) / 2.0)),
            material=wood_dark,
            name=f"{side}_center_stile",
        )

    pocket_w = 0.18
    pocket_h = 0.11
    pocket_bottom = T + 0.085
    pocket_top = pocket_bottom + pocket_h
    upper_slot_top = HB - rail_h
    pocket_center_z = (pocket_bottom + pocket_top) / 2.0

    for side, sx in (("left", -1.0), ("right", 1.0)):
        x = sx * wall_x
        body.visual(
            Box((T, side_span, rail_h)),
            origin=Origin(xyz=(x, 0.0, bottom_rail_z)),
            material=wood,
            name=f"{side}_rail_bottom",
        )
        body.visual(
            Box((T, side_span, rail_h)),
            origin=Origin(xyz=(x, 0.0, top_rail_z)),
            material=wood,
            name=f"{side}_rail_top",
        )
        stile_depth = (side_span - pocket_w) / 2.0
        for sy, tag in ((-1.0, "front"), (1.0, "rear")):
            body.visual(
                Box((T, stile_depth, top_rail_z - bottom_rail_z + rail_h)),
                origin=Origin(
                    xyz=(x, sy * (pocket_w / 2.0 + stile_depth / 2.0), (top_rail_z + bottom_rail_z) / 2.0)
                ),
                material=wood_dark,
                name=f"{side}_stile_{tag}",
            )
        body.visual(
            Box((T, pocket_w, pocket_bottom - T)),
            origin=Origin(xyz=(x, 0.0, T + (pocket_bottom - T) / 2.0)),
            material=wood,
            name=f"{side}_pocket_sill",
        )
        body.visual(
            Box((T, pocket_w, upper_slot_top - pocket_top)),
            origin=Origin(xyz=(x, 0.0, pocket_top + (upper_slot_top - pocket_top) / 2.0)),
            material=wood,
            name=f"{side}_pocket_lintel",
        )
        body.visual(
            Box((0.028, pocket_w + 0.018, pocket_h + 0.016)),
            origin=Origin(xyz=(sx * (W / 2.0 - 0.014), 0.0, pocket_center_z)),
            material=iron,
            name=f"{side}_handle_pocket",
        )

    # iron corner brackets at the four vertical edges
    bk = 0.032
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((bk, bk, HB - 0.03)),
                origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.004),
                                   sy * (D / 2.0 - bk / 2.0 + 0.004), HB / 2.0)),
                material=iron, name=f"corner_bracket_{idx}",
            )
            idx += 1

    # front and rear iron straps across the plank slats
    for side, sy in (("front", -1.0), ("back", 1.0)):
        for x in (-0.11, 0.11):
            body.visual(
                Box((0.026, 0.006, HB - 0.05)),
                origin=Origin(xyz=(x, sy * (D / 2.0 + 0.003), HB / 2.0)),
                material=iron,
                name=f"{side}_strap_{'l' if x < 0.0 else 'r'}",
            )

    # --- Lid (flat, hinged at the back top edge) -----------------------------
    lid = model.part("box_lid")
    lid.visual(Box((W + 0.016, D + 0.016, LID_T)),
               origin=Origin(xyz=(0.0, -D / 2.0 - 0.008, SEAM_GAP + LID_T / 2.0)),
               material=wood, name="lid_panel")
    lid.visual(Box((W - 0.030, D - 0.030, 0.010)),
               origin=Origin(xyz=(0.0, -D / 2.0 - 0.008, SEAM_GAP + 0.005)),
               material=wood_dark, name="lid_batten")
    # iron straps across the lid
    for sx in (-0.13, 0.13):
        lid.visual(Box((0.028, D + 0.02, 0.006)),
                   origin=Origin(xyz=(sx, -D / 2.0 - 0.008, SEAM_GAP + LID_T + 0.003)),
                   material=iron, name=f"lid_strap_{'l' if sx < 0 else 'r'}")
    for x in (-0.11, 0.11):
        lid.visual(
            Box((0.028, 0.035, 0.010)),
            origin=Origin(xyz=(x, -0.015, SEAM_GAP + 0.005)),
            material=wood_dark,
            name=f"lid_runner_{'l' if x < 0.0 else 'r'}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Recessed folding side handles ---------------------------------------
    handle_w = 0.12
    handle_drop = 0.085
    handle_bar = 0.012
    for side, sx, axis_y in (("left", -1.0, 1.0), ("right", 1.0, -1.0)):
        handle = model.part(f"{side}_handle")
        x_off = sx * 0.016
        handle.visual(
            Box((handle_bar, handle_w, handle_bar)),
            origin=Origin(xyz=(x_off, 0.0, 0.0)),
            material=iron,
            name="top_bar",
        )
        for sy, tag in ((-1.0, "front"), (1.0, "rear")):
            handle.visual(
                Box((handle_bar, handle_bar, handle_drop)),
                origin=Origin(xyz=(x_off, sy * (handle_w / 2.0 - handle_bar / 2.0), -handle_drop / 2.0)),
                material=iron,
                name=f"side_bar_{tag}",
            )
        handle.visual(
            Box((handle_bar, handle_w - handle_bar * 2.0, handle_bar)),
            origin=Origin(xyz=(x_off, 0.0, -handle_drop)),
            material=iron,
            name="bottom_bar",
        )
        handle.visual(
            Box((0.020, handle_w + 0.030, 0.020)),
            origin=Origin(xyz=(sx * 0.020, 0.0, 0.0)),
            material=iron,
            name="pivot_plate",
        )
        model.articulation(
            f"{side}_handle_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=handle,
            origin=Origin(xyz=(sx * (W / 2.0 - 0.010), 0.0, pocket_top - 0.020)),
            axis=(0.0, axis_y, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=pi / 2.0),
        )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("box_lid")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    left_handle_hinge = object_model.get_articulation("left_handle_hinge")
    right_handle_hinge = object_model.get_articulation("right_handle_hinge")

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, left_handle_hinge: 0.0, right_handle_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.008,
                       positive_elem="lid_panel", negative_elem="front_rail_top",
                       name="lid seats on the body rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.32,
                           name="closed lid covers the body opening")
        closed_lid = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.9}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    # Folding side handles exist and swing outward from their recessed pockets.
    ctx.check(
        "folding side handles present on both sides",
        left_handle is not None and right_handle is not None,
        details="expected left_handle and right_handle",
    )
    with ctx.pose({left_handle_hinge: 0.0}):
        left_closed = ctx.part_world_aabb(left_handle)
    with ctx.pose({left_handle_hinge: 1.2}):
        left_open = ctx.part_world_aabb(left_handle)
    ctx.check(
        "left handle folds outward",
        left_closed is not None and left_open is not None
        and left_open[0][0] < left_closed[0][0] - 0.02,
        details=f"closed={left_closed}, open={left_open}",
    )
    with ctx.pose({right_handle_hinge: 0.0}):
        right_closed = ctx.part_world_aabb(right_handle)
    with ctx.pose({right_handle_hinge: 1.2}):
        right_open = ctx.part_world_aabb(right_handle)
    ctx.check(
        "right handle folds outward",
        right_closed is not None and right_open is not None
        and right_open[1][0] > right_closed[1][0] + 0.02,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()
