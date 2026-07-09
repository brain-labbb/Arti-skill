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
    mesh_from_cadquery,
)


# ── Overall envelope (unchanged from parent) ──────────────────────────────
BASE_W = 0.130
BASE_D = 0.100
BASE_BOTTOM_H = 0.004
WALL_H = 0.010
WALL_TOP_Z = BASE_BOTTOM_H + WALL_H
LID_T = 0.003
INNER_LID_BOTTOM_Z = WALL_TOP_Z
OUTER_LID_BOTTOM_Z = 0.023

# ── 7-column × 2-row (AM / PM) grid ──────────────────────────────────────
NUM_COLS = 7
NUM_ROWS = 2
NUM_COMPARTMENTS = NUM_COLS * NUM_ROWS  # 14


def rounded_box_mesh(width: float, depth: float, height: float, radius: float, name: str):
    """Return a mesh-backed rounded rectangular solid centered on its local origin."""
    solid = cq.Workplane("XY").box(width, depth, height)
    if radius > 0:
        solid = solid.edges("|Z").fillet(min(radius, width * 0.45, depth * 0.45, height * 0.45))
    return mesh_from_cadquery(solid, name, tolerance=0.00035, angular_tolerance=0.12)


def add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def add_cylinder_x(part, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def add_digit(part, digit: int, lid_depth: float, material) -> None:
    """Raised white seven-segment number decal bonded into the lid top."""
    patterns = {
        1: "bc",
        2: "abged",
        3: "abgcd",
        4: "fgbc",
        5: "afgcd",
        6: "afgecd",
        7: "abc",
    }
    dw = 0.0070
    dh = 0.0100
    s = 0.00090
    z = LID_T + 0.00012
    cy = -lid_depth * 0.48
    segs = {
        "a": ((dw, s, 0.00035), (0.0, cy + dh * 0.50, z)),
        "b": ((s, dh * 0.50, 0.00035), (dw * 0.50, cy + dh * 0.25, z)),
        "c": ((s, dh * 0.50, 0.00035), (dw * 0.50, cy - dh * 0.25, z)),
        "d": ((dw, s, 0.00035), (0.0, cy - dh * 0.50, z)),
        "e": ((s, dh * 0.50, 0.00035), (-dw * 0.50, cy - dh * 0.25, z)),
        "f": ((s, dh * 0.50, 0.00035), (-dw * 0.50, cy + dh * 0.25, z)),
        "g": ((dw, s, 0.00035), (0.0, cy, z)),
    }
    for seg in patterns[digit]:
        size, xyz = segs[seg]
        add_box(part, size, xyz, material, f"number_{digit}_{seg}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_14_compartment_pill_organizer")

    cream = model.material("warm_white_plastic", rgba=(0.94, 0.88, 0.68, 1.0))
    tray_blue = model.material("soft_blue_tray", rgba=(0.58, 0.74, 0.88, 1.0))
    dark_shadow = model.material("dark_recess_shadow", rgba=(0.08, 0.10, 0.12, 1.0))
    white_print = model.material("white_print", rgba=(1.0, 1.0, 0.96, 1.0))
    clear = model.material("smoky_clear_plastic", rgba=(0.72, 0.78, 0.82, 0.30))
    latch_mat = model.material("white_latch_plastic", rgba=(0.96, 0.96, 0.90, 1.0))
    lid_mats = [
        model.material("rose_translucent", rgba=(1.00, 0.47, 0.42, 0.58)),
        model.material("blue_translucent", rgba=(0.42, 0.63, 0.86, 0.55)),
        model.material("amber_translucent", rgba=(0.88, 0.66, 0.48, 0.55)),
        model.material("lavender_translucent", rgba=(0.78, 0.72, 0.90, 0.55)),
        model.material("smoke_translucent", rgba=(0.55, 0.56, 0.62, 0.55)),
        model.material("aqua_translucent", rgba=(0.42, 0.72, 0.86, 0.55)),
        model.material("butter_translucent", rgba=(0.97, 0.90, 0.52, 0.55)),
    ]

    # ── Base tray (same wall/proportion style as parent) ──────────────────
    base = model.part("base_tray")
    base.visual(
        rounded_box_mesh(BASE_W, BASE_D, BASE_BOTTOM_H, 0.009, "rounded_base_bottom"),
        origin=Origin(xyz=(0.0, 0.0, BASE_BOTTOM_H / 2.0)),
        material=cream,
        name="rounded_base_bottom",
    )

    rim_t = 0.005
    add_box(base, (BASE_W, rim_t, WALL_H), (0.0, BASE_D / 2.0 - rim_t / 2.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, "rear_rim")
    add_box(base, (BASE_W, rim_t, WALL_H), (0.0, -BASE_D / 2.0 + rim_t / 2.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, "front_rim")
    add_box(base, (rim_t, BASE_D - 2.0 * rim_t, WALL_H), (-BASE_W / 2.0 + rim_t / 2.0, 0.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, "side_rim_0")
    add_box(base, (rim_t, BASE_D - 2.0 * rim_t, WALL_H), (BASE_W / 2.0 - rim_t / 2.0, 0.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, "side_rim_1")

    # ── Compute 7×2 uniform grid inside the rim ──────────────────────────
    wall_t = 0.0022
    inner_w = BASE_W - 2.0 * rim_t
    inner_d = BASE_D - 2.0 * rim_t
    cell_w = (inner_w - (NUM_COLS - 1) * wall_t) / NUM_COLS
    cell_d = (inner_d - (NUM_ROWS - 1) * wall_t) / NUM_ROWS

    lid_specs: list[dict] = []
    for n in range(NUM_COMPARTMENTS):
        col = n % NUM_COLS
        row = n // NUM_COLS
        x = (-inner_w / 2.0 + wall_t + cell_w / 2.0) + col * (cell_w + wall_t)
        y = (inner_d / 2.0 - wall_t - cell_d / 2.0) - row * (cell_d + wall_t)
        lid_specs.append({"n": n, "col": col, "row": row, "x": x, "y": y, "w": cell_w, "d": cell_d})

    # ── Per-well floors, divider walls, and gap reveals ───────────────────
    floor_h = 0.0008
    for spec in lid_specs:
        n = spec["n"]
        x = spec["x"]
        y = spec["y"]
        w = spec["w"]
        d = spec["d"]
        add_box(base, (w - 2.0 * wall_t, d - 2.0 * wall_t, floor_h),
                (x, y, BASE_BOTTOM_H + floor_h / 2.0), tray_blue, f"well_{n}_floor")
        add_box(base, (w, wall_t, WALL_H),
                (x, y + d / 2.0 - wall_t / 2.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, f"well_{n}_rear_wall")
        add_box(base, (w, wall_t, WALL_H),
                (x, y - d / 2.0 + wall_t / 2.0, BASE_BOTTOM_H + WALL_H / 2.0), cream, f"well_{n}_front_wall")
        add_box(base, (wall_t, d, WALL_H),
                (x - w / 2.0 + wall_t / 2.0, y, BASE_BOTTOM_H + WALL_H / 2.0), cream, f"well_{n}_side_wall_0")
        add_box(base, (wall_t, d, WALL_H),
                (x + w / 2.0 - wall_t / 2.0, y, BASE_BOTTOM_H + WALL_H / 2.0), cream, f"well_{n}_side_wall_1")
        add_box(base, (w * 0.70, 0.0010, 0.0014),
                (x, y - d / 2.0 + 0.0001, WALL_TOP_Z + 0.0003), dark_shadow, f"lid_gap_{n}")

    # ── Outer-lid hinge bosses and front latch catch (unchanged) ──────────
    add_box(base, (0.020, 0.004, 0.006), (-0.040, BASE_D / 2.0 + 0.001, WALL_TOP_Z + 0.003), cream, "outer_hinge_boss_0")
    add_box(base, (0.020, 0.004, 0.006), (0.040, BASE_D / 2.0 + 0.001, WALL_TOP_Z + 0.003), cream, "outer_hinge_boss_1")
    add_box(base, (0.030, 0.005, 0.005), (0.0, -BASE_D / 2.0 - 0.0022, WALL_TOP_Z - 0.001), latch_mat, "front_latch_catch")

    # ── 14 compartment lids emitted with a loop ───────────────────────────
    for spec in lid_specs:
        n = spec["n"]
        col = spec["col"]
        x = spec["x"]
        y = spec["y"]
        w = spec["w"]
        d = spec["d"]
        mat = lid_mats[col % len(lid_mats)]

        lid = model.part(f"compartment_lid_{n}")
        lid.visual(
            rounded_box_mesh(w - 0.0025, d - 0.0025, LID_T, 0.0055, f"compartment_lid_{n}_panel"),
            origin=Origin(xyz=(0.0, -d / 2.0, LID_T / 2.0)),
            material=mat,
            name="lid_panel",
        )
        add_box(lid, (w * 0.60, 0.0030, 0.0011), (0.0, -d + 0.0012, LID_T + 0.00055), mat, "front_fingernail")
        add_cylinder_x(lid, 0.0013, w * 0.55, (0.0, 0.0012, 0.0022), mat, "hinge_barrel")
        add_box(lid, (w * 0.50, 0.0030, 0.0010), (0.0, -0.0010, 0.0010), mat, "hinge_leaf")
        add_digit(lid, col + 1, d, white_print)

        hinge_y = y + d / 2.0
        model.articulation(
            f"base_to_lid_{n}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=lid,
            origin=Origin(xyz=(x, hinge_y, INNER_LID_BOTTOM_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=0.0, upper=1.75),
        )

    # ── Outer clear lid (unchanged from parent baseline) ──────────────────
    outer_lid = model.part("outer_lid")
    outer_lid.visual(
        rounded_box_mesh(BASE_W - 0.006, BASE_D - 0.006, 0.0025, 0.011, "outer_clear_panel"),
        origin=Origin(xyz=(0.0, -(BASE_D - 0.006) / 2.0, 0.00125)),
        material=clear,
        name="outer_panel",
    )
    add_box(outer_lid, (BASE_W - 0.026, 0.0020, 0.0010), (0.0, -0.020, 0.00285), clear, "outer_rib_0")
    add_box(outer_lid, (BASE_W - 0.026, 0.0020, 0.0010), (0.0, -0.062, 0.00285), clear, "outer_rib_1")
    add_box(outer_lid, (0.0020, BASE_D - 0.026, 0.0010), (-0.045, -0.048, 0.00285), clear, "outer_rib_2")
    add_box(outer_lid, (0.0020, BASE_D - 0.026, 0.0010), (0.045, -0.048, 0.00285), clear, "outer_rib_3")
    add_cylinder_x(outer_lid, 0.0020, 0.075, (0.0, 0.0015, 0.0020), clear, "outer_hinge_barrel")
    add_box(outer_lid, (0.038, 0.006, 0.0020), (0.0, -(BASE_D - 0.006) + 0.001, 0.0010), clear, "latch_hinge_leaf")

    outer_joint = model.articulation(
        "base_to_outer_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_lid,
        origin=Origin(xyz=(0.0, BASE_D / 2.0, OUTER_LID_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.65),
    )

    # ── Front latch (unchanged from parent baseline) ──────────────────────
    latch = model.part("front_latch")
    add_box(latch, (0.030, 0.0045, 0.010), (0.0, -0.0024, -0.0050), latch_mat, "latch_tab")
    add_box(latch, (0.025, 0.0010, 0.0010), (0.0, -0.0050, -0.0018), cream, "grip_ridge_0")
    add_box(latch, (0.025, 0.0010, 0.0010), (0.0, -0.0050, -0.0040), cream, "grip_ridge_1")
    add_box(latch, (0.025, 0.0010, 0.0010), (0.0, -0.0050, -0.0062), cream, "grip_ridge_2")
    model.articulation(
        "outer_lid_to_latch",
        ArticulationType.REVOLUTE,
        parent=outer_lid,
        child=latch,
        origin=Origin(xyz=(0.0, -(BASE_D - 0.006) + 0.003, 0.0005)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=0.0, upper=0.85),
    )

    model.meta["outer_lid_joint"] = outer_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_tray")
    outer_lid = object_model.get_part("outer_lid")
    latch = object_model.get_part("front_latch")
    outer_joint = object_model.get_articulation("base_to_outer_lid")
    latch_joint = object_model.get_articulation("outer_lid_to_latch")

    # ── Variant-specific: 14 compartment lids and hinges ──────────────────
    ctx.check(
        "14_compartment_lid_parts",
        all(object_model.get_part(f"compartment_lid_{n}") for n in range(14)),
        details="expected compartment_lid_0..13",
    )
    ctx.check(
        "14_compartment_lid_hinges",
        all(object_model.get_articulation(f"base_to_lid_{n}") for n in range(14)),
        details="expected base_to_lid_0..13",
    )

    # Per-lid coverage and flip-open motion
    for n in range(14):
        lid = object_model.get_part(f"compartment_lid_{n}")
        hinge = object_model.get_articulation(f"base_to_lid_{n}")
        ctx.expect_overlap(
            lid, base,
            axes="xy",
            min_overlap=0.008,
            elem_a="lid_panel",
            elem_b=f"well_{n}_floor",
            name=f"lid_{n}_covers_its_well",
        )
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        with ctx.pose({hinge: 1.35}):
            open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            f"lid_{n}_flips_up",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.015,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    # Outer lid spans every well
    for n in range(14):
        ctx.expect_within(
            base, outer_lid,
            axes="xy",
            margin=0.005,
            inner_elem=f"well_{n}_floor",
            outer_elem="outer_panel",
            name=f"outer_lid_spans_well_{n}",
        )

    closed_outer = ctx.part_element_world_aabb(outer_lid, elem="outer_panel")
    with ctx.pose({outer_joint: 1.25}):
        open_outer = ctx.part_element_world_aabb(outer_lid, elem="outer_panel")
    ctx.check(
        "outer_lid_opens_above_case",
        closed_outer is not None
        and open_outer is not None
        and open_outer[1][2] > closed_outer[1][2] + 0.035,
        details=f"closed={closed_outer}, open={open_outer}",
    )

    latched = ctx.part_world_aabb(latch)
    with ctx.pose({latch_joint: 0.65}):
        released = ctx.part_world_aabb(latch)
    ctx.check(
        "front_latch_flips_out",
        latched is not None
        and released is not None
        and released[0][1] < latched[0][1] - 0.002,
        details=f"latched={latched}, released={released}",
    )

    # ── Variant axis proof: AM row (n=0..6) sits behind PM row (n=7..13) ─
    lid_am = object_model.get_part("compartment_lid_0")
    lid_pm = object_model.get_part("compartment_lid_7")
    am_pos = ctx.part_world_position(lid_am)
    pm_pos = ctx.part_world_position(lid_pm)
    ctx.check(
        "am_row_rear_of_pm_row",
        am_pos is not None and pm_pos is not None and am_pos[1] > pm_pos[1] + 0.010,
        details=f"am_y={am_pos[1] if am_pos else None}, pm_y={pm_pos[1] if pm_pos else None}",
    )

    return ctx.report()


object_model = build_object_model()
