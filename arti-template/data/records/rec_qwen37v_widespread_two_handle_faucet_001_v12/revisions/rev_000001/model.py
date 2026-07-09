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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Widespread two-handle deck-mounted bathroom faucet in polished gold brass.
#
# Frame conventions:
#   - The deck (countertop) top face at z = 0.
#   - Z is up; the faucet projects upward from the deck.
#   - Three mounting centers along X:
#     left at x = -PITCH, spout bridge at x = 0, right at x = +PITCH.
# ---------------------------------------------------------------------------

PITCH = 0.10
DECK_W = 0.38
DECK_D = 0.18
DECK_T = 0.012

# Bridge arch tube
TUBE_R = 0.014
BORE_R = 0.009
ARCH_RISE = 0.040  # rise above the arch base

# Escutcheon
ESC_R1 = 0.030
ESC_T1 = 0.008
ESC_R2 = 0.022
ESC_T2 = 0.006

# Valve body
VALVE_R = 0.013
VALVE_H = 0.028

# Handle riser (short stub above valve to clear the arch tube)
RISER_R = 0.009
RISER_H = 0.035  # height above valve top to handle joint

# Cross handle (compact to clear the arch)
HROD_R = 0.004
HROD_LEN = 0.090
HUB_R = 0.011
HUB_LEN = 0.018
KNURL_R = 0.012
STEM_R = 0.005
STEM_LEN = 0.014

# Diverter
DIV_R = 0.011
DIV_H = 0.016
DIV_STEM_R = 0.005
DIV_STEM_H = 0.022
DIV_TRAVEL = 0.020

# Seam
SEAM_W = 0.001
SEAM_R = ESC_R1 + 0.001

# Derived heights
VALVE_TOP_Z = ESC_T1 + ESC_T2 + VALVE_H  # 0.042
ARCH_BASE_Z = VALVE_TOP_Z - TUBE_R  # arch tube center sits at valve top minus tube radius
HANDLE_JOINT_Z = VALVE_TOP_Z + RISER_H  # handle rotates above the arch


def _arch_points(n=13):
    """Parabolic arch points in (x, z) for bridge tube centerline."""
    apex_z = ARCH_BASE_Z + ARCH_RISE
    pts = []
    for i in range(n):
        t = i / (n - 1)
        x = -PITCH + t * 2.0 * PITCH
        z = apex_z - (apex_z - ARCH_BASE_Z) * (2.0 * t - 1.0) ** 2
        pts.append((x, z))
    return pts


def _build_tube_from_points(pts, radius):
    """Build a tube as union of spheres and connecting half-cylinders."""
    x0, z0 = pts[0]
    result = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .center(x0, 0.0)
        .sphere(radius)
    )
    for i in range(1, len(pts)):
        x1, z1 = pts[i]
        sph = (
            cq.Workplane("XY")
            .workplane(offset=z1)
            .center(x1, 0.0)
            .sphere(radius)
        )
        result = result.union(sph)
        dx, dz = x1 - x0, z1 - z0
        seg = math.sqrt(dx * dx + dz * dz)
        if seg < 1e-9:
            x0, z0 = x1, z1
            continue
        ang = math.degrees(math.atan2(dz, dx))
        mx, mz = (x0 + x1) / 2.0, (z0 + z1) / 2.0
        c1 = (
            cq.Workplane("XY")
            .workplane(offset=mz)
            .center(mx, 0.0)
            .transformed(rotate=(0.0, -ang, 0.0))
            .circle(radius)
            .extrude(seg / 2.0)
        )
        c2 = (
            cq.Workplane("XY")
            .workplane(offset=mz)
            .center(mx, 0.0)
            .transformed(rotate=(0.0, -ang + 180.0, 0.0))
            .circle(radius)
            .extrude(seg / 2.0)
        )
        result = result.union(c1).union(c2)
        x0, z0 = x1, z1
    return result


def _build_bridge_solid():
    """Low bridge arch tube with hollow bore. Endpoints at valve centers."""
    pts = _arch_points()
    outer = _build_tube_from_points(pts, TUBE_R)
    bore = _build_tube_from_points(pts, BORE_R)
    return outer.cut(bore)


def _build_hub_solid():
    """Cross-handle hub: axis +Z, base at z=0."""
    hub = cq.Workplane("XY").circle(HUB_R).extrude(HUB_LEN)
    knurl = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .polygon(16, 2.0 * KNURL_R)
        .extrude(0.008)
    )
    cap = cq.Workplane("XY").workplane(offset=HUB_LEN).sphere(HUB_R * 0.85)
    return hub.union(knurl).union(cap)


def _build_diverter_solid():
    """Diverter knob: cylindrical body with grip band, axis +Z."""
    body = cq.Workplane("XY").circle(DIV_R).extrude(DIV_H)
    dome = cq.Workplane("XY").workplane(offset=DIV_H).sphere(DIV_R * 0.85)
    grip = (
        cq.Workplane("XY")
        .workplane(offset=0.003)
        .polygon(12, 2.0 * DIV_R * 1.05)
        .extrude(0.007)
    )
    return body.union(dome).union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_two_handle_faucet")

    gold = model.material("polished_gold_brass", rgba=(0.85, 0.66, 0.20, 1.0))
    dark = model.material("dark_seam", rgba=(0.15, 0.12, 0.05, 1.0))
    deck_mat = model.material("countertop", rgba=(0.88, 0.86, 0.83, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((DECK_W, DECK_D, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_T / 2.0)),
        material=deck_mat,
        name="slab",
    )

    # --- central bridge arch spout ---
    spout = model.part("spout_bridge")
    bridge_mesh = mesh_from_cadquery(_build_bridge_solid(), "bridge_arch")
    spout.visual(bridge_mesh, material=gold, name="arch_tube")
    # Spout escutcheon at center
    spout.visual(
        Cylinder(radius=ESC_R1, length=ESC_T1),
        origin=Origin(xyz=(0.0, 0.0, ESC_T1 / 2.0)),
        material=gold, name="esc_base",
    )
    spout.visual(
        Cylinder(radius=ESC_R2, length=ESC_T2),
        origin=Origin(xyz=(0.0, 0.0, ESC_T1 + ESC_T2 / 2.0)),
        material=gold, name="esc_step",
    )
    # Center support column from escutcheon up to the arch tube
    col_base_z = ESC_T1 + ESC_T2
    col_top_z = ARCH_BASE_Z + ARCH_RISE  # arch apex centerline z
    col_h = col_top_z - col_base_z
    col_r = TUBE_R * 0.7  # fits within arch tube radius
    spout.visual(
        Cylinder(radius=col_r, length=col_h),
        origin=Origin(xyz=(0.0, 0.0, col_base_z + col_h / 2.0)),
        material=gold, name="center_column",
    )
    spout.visual(
        Cylinder(radius=SEAM_R, length=SEAM_W),
        origin=Origin(xyz=(0.0, 0.0, SEAM_W / 2.0)),
        material=dark, name="seam",
    )
    model.articulation(
        "deck_to_spout", ArticulationType.FIXED,
        parent=deck, child=spout, origin=Origin(),
    )

    # --- valve assemblies and handles ---
    hub_mesh = mesh_from_cadquery(_build_hub_solid(), "handle_hub")

    for side, sx in (("left", -1.0), ("right", 1.0)):
        valve = model.part(f"{side}_valve")
        valve.visual(
            Cylinder(radius=ESC_R1, length=ESC_T1),
            origin=Origin(xyz=(0.0, 0.0, ESC_T1 / 2.0)),
            material=gold, name="esc_base",
        )
        valve.visual(
            Cylinder(radius=ESC_R2, length=ESC_T2),
            origin=Origin(xyz=(0.0, 0.0, ESC_T1 + ESC_T2 / 2.0)),
            material=gold, name="esc_step",
        )
        valve.visual(
            Cylinder(radius=VALVE_R, length=VALVE_H),
            origin=Origin(xyz=(0.0, 0.0, ESC_T1 + ESC_T2 + VALVE_H / 2.0)),
            material=gold, name="valve_body",
        )
        # Riser above valve body to carry the handle above the arch tube
        valve.visual(
            Cylinder(radius=RISER_R, length=RISER_H),
            origin=Origin(xyz=(0.0, 0.0, VALVE_TOP_Z + RISER_H / 2.0)),
            material=gold, name="riser",
        )
        valve.visual(
            Cylinder(radius=SEAM_R, length=SEAM_W),
            origin=Origin(xyz=(0.0, 0.0, SEAM_W / 2.0)),
            material=dark, name="seam",
        )
        model.articulation(
            f"deck_to_{side}_valve", ArticulationType.FIXED,
            parent=deck, child=valve,
            origin=Origin(xyz=(sx * PITCH, 0.0, 0.0)),
        )

        handle = model.part(f"{side}_handle")
        handle.visual(
            Cylinder(radius=STEM_R, length=STEM_LEN),
            origin=Origin(xyz=(0.0, 0.0, -STEM_LEN / 2.0)),
            material=gold, name="stem",
        )
        handle.visual(hub_mesh, material=gold, name="hub")
        spoke_z = HUB_LEN / 2.0
        handle.visual(
            Cylinder(radius=HROD_R, length=HROD_LEN),
            origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gold, name="spoke_x",
        )
        handle.visual(
            Cylinder(radius=HROD_R, length=HROD_LEN),
            origin=Origin(xyz=(0.0, 0.0, spoke_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=gold, name="spoke_y",
        )
        half = HROD_LEN / 2.0
        for nm, (dx, dy) in (
            ("tip_px", (half, 0.0)), ("tip_nx", (-half, 0.0)),
            ("tip_py", (0.0, half)), ("tip_ny", (0.0, -half)),
        ):
            handle.visual(
                Sphere(radius=HROD_R),
                origin=Origin(xyz=(dx, dy, spoke_z)),
                material=gold, name=nm,
            )
        model.articulation(
            f"{side}_handle_spindle", ArticulationType.REVOLUTE,
            parent=valve, child=handle,
            origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0, velocity=3.0, lower=-math.pi, upper=math.pi,
            ),
        )

    # --- diverter knob (prismatic along Z) ---
    diverter = model.part("diverter_knob")
    div_mesh = mesh_from_cadquery(_build_diverter_solid(), "diverter_knob")
    diverter.visual(
        Cylinder(radius=DIV_STEM_R, length=DIV_STEM_H),
        origin=Origin(xyz=(0.0, 0.0, -DIV_STEM_H / 2.0)),
        material=gold, name="stem",
    )
    diverter.visual(div_mesh, material=gold, name="knob")

    div_mount_z = ARCH_BASE_Z + ARCH_RISE * 0.5
    model.articulation(
        "spout_to_diverter", ArticulationType.PRISMATIC,
        parent=spout, child=diverter,
        origin=Origin(xyz=(0.0, 0.025, div_mount_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=0.5, lower=0.0, upper=DIV_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    spout = object_model.get_part("spout_bridge")
    left_valve = object_model.get_part("left_valve")
    right_valve = object_model.get_part("right_valve")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    diverter = object_model.get_part("diverter_knob")

    left_joint = object_model.get_articulation("left_handle_spindle")
    right_joint = object_model.get_articulation("right_handle_spindle")
    div_joint = object_model.get_articulation("spout_to_diverter")

    # Handle joints are revolute about Z
    for joint in (left_joint, right_joint):
        ctx.check(
            f"{joint.name}_revolute",
            str(joint.joint_type).lower().endswith("revolute"),
            f"type={joint.joint_type}",
        )
        ax = joint.axis
        ctx.check(
            f"{joint.name}_axis_vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(ax[2] - 1.0) < 1e-9,
            f"axis={ax}",
        )
        lim = joint.motion_limits
        ctx.check(
            f"{joint.name}_full_turn_range",
            lim is not None
            and abs(lim.lower + math.pi) < 1e-6
            and abs(lim.upper - math.pi) < 1e-6,
            f"limits=({lim.lower}, {lim.upper})",
        )

    # Diverter is prismatic along Z
    ctx.check(
        "diverter_is_prismatic",
        str(div_joint.joint_type).lower().endswith("prismatic"),
        f"type={div_joint.joint_type}",
    )
    dax = div_joint.axis
    ctx.check(
        "diverter_axis_vertical",
        abs(dax[0]) < 1e-9 and abs(dax[1]) < 1e-9 and abs(dax[2] - 1.0) < 1e-9,
        f"axis={dax}",
    )
    dlim = div_joint.motion_limits
    ctx.check(
        "diverter_limited_travel",
        dlim is not None and abs(dlim.lower) < 1e-6 and abs(dlim.upper - DIV_TRAVEL) < 1e-6,
        f"limits=({dlim.lower}, {dlim.upper})",
    )

    # Stem overlaps: handle stems seat into valve riser
    ctx.allow_overlap(
        left_handle, left_valve,
        elem_a=left_handle.get_visual("stem"),
        elem_b=left_valve.get_visual("riser"),
        reason="handle stem seated inside valve riser bore",
    )
    ctx.allow_overlap(
        right_handle, right_valve,
        elem_a=right_handle.get_visual("stem"),
        elem_b=right_valve.get_visual("riser"),
        reason="handle stem seated inside valve riser bore",
    )
    # Diverter stem captured inside bridge tube bore
    ctx.allow_overlap(
        diverter, spout,
        elem_a=diverter.get_visual("stem"),
        elem_b=spout.get_visual("arch_tube"),
        reason="diverter stem captured inside bridge tube bore",
    )
    # Bridge arch tube passes through valve bodies at endpoints
    ctx.allow_overlap(
        left_valve, spout,
        elem_a=left_valve.get_visual("valve_body"),
        elem_b=spout.get_visual("arch_tube"),
        reason="bridge arch tube passes through valve body at arch endpoint",
    )
    ctx.allow_overlap(
        right_valve, spout,
        elem_a=right_valve.get_visual("valve_body"),
        elem_b=spout.get_visual("arch_tube"),
        reason="bridge arch tube passes through valve body at arch endpoint",
    )
    # Riser passes through arch tube at endpoint
    ctx.allow_overlap(
        left_valve, spout,
        elem_a=left_valve.get_visual("riser"),
        elem_b=spout.get_visual("arch_tube"),
        reason="valve riser passes through bridge arch tube at endpoint",
    )
    ctx.allow_overlap(
        right_valve, spout,
        elem_a=right_valve.get_visual("riser"),
        elem_b=spout.get_visual("arch_tube"),
        reason="valve riser passes through bridge arch tube at endpoint",
    )

    # Proof checks for allowed overlaps
    ctx.expect_overlap(
        left_valve, spout, axes="z", min_overlap=0.005,
        elem_a="valve_body", elem_b="arch_tube",
        name="left valve body overlaps arch tube in Z",
    )
    ctx.expect_overlap(
        right_valve, spout, axes="z", min_overlap=0.005,
        elem_a="valve_body", elem_b="arch_tube",
        name="right valve body overlaps arch tube in Z",
    )

    # Bridge arch spans between handles and rises above deck
    spout_aabb = ctx.part_world_aabb(spout)
    assert spout_aabb is not None
    (sx0, sy0, sz0), (sx1, sy1, sz1) = spout_aabb
    ctx.check(
        "bridge_arch_rises_above_deck",
        sz1 > 0.04,
        f"spout z_max={sz1:.3f}",
    )
    ctx.check(
        "bridge_spans_between_handles",
        (sx1 - sx0) >= 0.10,
        f"spout x_span={sx1 - sx0:.3f}",
    )

    # Three-piece layout
    lv = ctx.part_world_position(left_valve)
    rv = ctx.part_world_position(right_valve)
    assert lv is not None and rv is not None
    ctx.check(
        "valves_flank_spout_at_pitch",
        abs(lv[0] + PITCH) < 1e-6 and abs(rv[0] - PITCH) < 1e-6,
        f"left_x={lv[0]:.4f}, right_x={rv[0]:.4f}",
    )

    # Overall width
    lh_aabb = ctx.part_world_aabb(left_handle)
    rh_aabb = ctx.part_world_aabb(right_handle)
    assert lh_aabb is not None and rh_aabb is not None
    total_w = rh_aabb[1][0] - lh_aabb[0][0]
    ctx.check(
        "overall_width_about_0p30",
        0.24 <= total_w <= 0.36,
        f"width={total_w:.3f}",
    )

    # Seams at all three deck bases
    for pn in ("spout_bridge", "left_valve", "right_valve"):
        p = object_model.get_part(pn)
        sv = p.get_visual("seam")
        ctx.check(f"{pn}_has_seam", sv is not None, f"missing seam on {pn}")

    # Seam is narrow
    seam_aabb = ctx.part_element_world_aabb(spout, elem="seam")
    assert seam_aabb is not None
    ctx.check(
        "seam_is_narrow",
        (seam_aabb[1][2] - seam_aabb[0][2]) < 0.003,
        f"seam z extent={seam_aabb[1][2] - seam_aabb[0][2]:.4f}",
    )

    # Handle rotation proof
    with ctx.pose({left_joint: math.pi / 4.0}):
        rot_aabb = ctx.part_world_aabb(left_handle)
        assert rot_aabb is not None
        rot_x = rot_aabb[1][0] - rot_aabb[0][0]
        ctx.check(
            "left_handle_rotates_off_axis",
            rot_x < 0.085,
            f"x extent at 45deg={rot_x:.3f}",
        )

    # Diverter slides up
    rest_pos = ctx.part_world_position(diverter)
    assert rest_pos is not None
    with ctx.pose({div_joint: DIV_TRAVEL}):
        ext_pos = ctx.part_world_position(diverter)
        assert ext_pos is not None
        ctx.check(
            "diverter_slides_upward",
            ext_pos[2] > rest_pos[2] + DIV_TRAVEL * 0.9,
            f"rest_z={rest_pos[2]:.4f}, ext_z={ext_pos[2]:.4f}",
        )

    # Deck grounded
    deck_aabb = ctx.part_world_aabb(deck)
    assert deck_aabb is not None
    ctx.check(
        "deck_grounded",
        abs(deck_aabb[0][2] + DECK_T) < 1e-6,
        f"deck zmin={deck_aabb[0][2]:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
